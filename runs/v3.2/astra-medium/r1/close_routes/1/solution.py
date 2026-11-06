import json
import math
from pathlib import Path

import gtsam
import numpy as np


def _read_spec(public_spec):
    """The supplied public specification is authoritative."""
    if public_spec is None:
        public_spec = "spec.json"
    if isinstance(public_spec, (str, Path)):
        with open(public_spec, "r", encoding="utf-8") as stream:
            return json.load(stream)
    return public_spec


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _length_scale(unit):
    return {"m": 1.0, "mm": 0.001, "cm": 0.01}[unit]


def _angle_scale(unit):
    return {"rad": 1.0, "deg": math.pi / 180.0}[unit]


class Estimator:
    def __init__(self, public_spec):
        self.spec = _read_spec(public_spec)
        noise = self.spec["sensor_noise_std"]
        units = self.spec["input_units"]

        self.odom_scale = _length_scale(units["odom"])
        self.bearing_scale = _angle_scale(units["bearing"])
        self.odom_angle_scale = _angle_scale(
            units.get("odom_angle", units.get("angle", units["bearing"]))
        )
        self.range_scale = _length_scale(units.get("range", "m"))
        self.gps_scale = _length_scale(units.get("gps", "m"))

        # Noise parameters are already in SI, independently of input units.
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(noise["odom"], dtype=float)
        )
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(
            2, float(noise["gps"])
        )
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([noise["bearing"], noise["range"]], dtype=float)
        )
        self.landmarks = {
            str(identifier): np.asarray(point, dtype=float)
            for identifier, point in self.spec["landmarks"].items()
        }

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-5)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)

        self.last_step = -1
        self.last_pose = None
        self.last_output = None
        # Keep Python callbacks alive for the lifetime of the graph.
        self.callbacks = []

    def _custom_factor(self, graph, key, noise, callback):
        self.callbacks.append(callback)
        graph.add(gtsam.CustomFactor(noise, [key], callback))

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k == self.last_step:
            return (
                self.last_output[0].copy(),
                [row.copy() for row in self.last_output[1]],
            )
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive in consecutive step order.")

        key = gtsam.symbol("x", k)
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        if k == 0:
            mean = np.asarray(self.spec["initial_pose"]["mean"], dtype=float)
            variances = np.asarray(
                self.spec["initial_pose"]["cov_diag_variances"], dtype=float
            )
            guess = gtsam.Pose2(*mean)
            prior_noise = gtsam.noiseModel.Diagonal.Variances(variances)

            # The sampled initial prior is Gaussian in world x, y, theta.
            def prior_error(factor, values, jacobians, key=key, mean=mean):
                pose = values.atPose2(key)
                if jacobians is not None:
                    jac = np.zeros((3, 3), order="F")
                    jac[:2, :2] = pose.rotation().matrix()
                    jac[2, 2] = 1.0
                    jacobians[0] = jac
                return np.array([
                    pose.x() - mean[0],
                    pose.y() - mean[1],
                    _wrap(pose.theta() - mean[2]),
                ])

            self._custom_factor(graph, key, prior_noise, prior_error)
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("Each noninitial step needs relative odometry.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.odom_scale,
                float(odom["dy"]) * self.odom_scale,
                float(odom["dtheta"]) * self.odom_angle_scale,
            )
            previous_key = gtsam.symbol("x", k - 1)
            graph.add(gtsam.BetweenFactorPose2(
                previous_key, key, relative, self.odom_noise
            ))
            guess = self.last_pose.compose(relative)

        initial.insert(key, guess)

        gps = new_measurements.get("gps")
        if gps is not None:
            measured = self.gps_scale * np.array(
                [gps["x"], gps["y"]], dtype=float
            )

            def gps_error(factor, values, jacobians, key=key, measured=measured):
                pose = values.atPose2(key)
                if jacobians is not None:
                    jac = np.zeros((2, 3), order="F")
                    jac[:, :2] = pose.rotation().matrix()
                    jacobians[0] = jac
                return np.array([pose.x(), pose.y()]) - measured

            self._custom_factor(graph, key, self.gps_noise, gps_error)

        for observation in new_measurements.get("landmarks", []):
            point = self.landmarks[str(observation["id"])]
            bearing = float(observation["bearing"]) * self.bearing_scale
            distance = float(observation["range"]) * self.range_scale

            def landmark_error(
                factor, values, jacobians,
                key=key, point=point, bearing=bearing, distance=distance,
            ):
                pose = values.atPose2(key)
                q = pose.rotation().matrix().T @ (
                    point - np.array([pose.x(), pose.y()])
                )
                qx, qy = float(q[0]), float(q[1])
                r2 = max(qx * qx + qy * qy, 1e-20)
                r = math.sqrt(r2)
                if jacobians is not None:
                    jacobians[0] = np.array([
                        [qy / r2, -qx / r2, -1.0],
                        [-qx / r, -qy / r, 0.0],
                    ], order="F")
                return np.array([
                    _wrap(math.atan2(qy, qx) - bearing),
                    r - distance,
                ])

            # Landmarks are known constants, not uncertain estimated variables.
            self._custom_factor(
                graph, key, self.landmark_noise, landmark_error
            )

        self.isam.update(graph, initial)
        # Additional incremental iterations settle nonlinear measurement factors.
        for _ in range(3):
            self.isam.update()

        pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(self.isam.marginalCovariance(key), dtype=float)

        # iSAM's Gaussian marginal uses its linearization-point tangent frame.
        # Express its translational coordinates at the returned pose instead.
        linear_pose = self.isam.getLinearizationPoint().atPose2(key)
        transport = np.eye(3)
        transport[:2, :2] = (
            pose.rotation().matrix().T @ linear_pose.rotation().matrix()
        )
        covariance = transport @ covariance @ transport.T
        covariance = 0.5 * (covariance + covariance.T)

        output = (
            [float(pose.x()), float(pose.y()), float(pose.theta())],
            covariance.tolist(),
        )
        self.last_step = k
        self.last_pose = pose
        self.last_output = output
        return output[0].copy(), [row.copy() for row in output[1]]


def make_estimator(public_spec):
    return Estimator(public_spec)


def choose_route(public_spec):
    """
    Compare routes with a nominal covariance rollout and a conservative
    accumulated wall-contact risk. This is a planning approximation, not an
    empirically calibrated guarantee.
    """
    spec = _read_spec(public_spec)
    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    spacing = speed * dt
    radius = float(spec["robot_radius"])
    lookahead = float(controller["lookahead"])
    noise = spec["sensor_noise_std"]
    actuator = spec["actuator_noise_std"]
    walls = [np.asarray(w, dtype=float).reshape(2, 2) for w in spec["walls"]]
    landmarks = [
        np.asarray(p, dtype=float) for p in spec["landmarks"].values()
    ]
    visibility = float(spec["landmark_visibility"]["max_range"])
    schedule = spec["gps_schedule"]
    dropout = schedule["dropout_steps"]

    def gps_available(k):
        if k % int(schedule["rate_steps"]) != 0:
            return False
        # The published pair specifies the inclusive outage interval.
        if len(dropout) == 2:
            return not (int(dropout[0]) <= k <= int(dropout[1]))
        return k not in dropout

    def nearest_on_wall(point, wall):
        a, b = wall
        direction = b - a
        denominator = float(direction @ direction)
        if denominator == 0.0:
            return a
        fraction = np.clip(float((point - a) @ direction) / denominator, 0, 1)
        return a + fraction * direction

    def update_covariance(covariance, h, variances):
        innovation = h @ covariance @ h.T + np.diag(variances)
        gain = np.linalg.solve(innovation, h @ covariance).T
        identity = np.eye(3) - gain @ h
        result = (
            identity @ covariance @ identity.T
            + gain @ np.diag(variances) @ gain.T
        )
        return 0.5 * (result + result.T)

    results = []
    for name, raw_points in spec["routes"].items():
        points = np.asarray(raw_points, dtype=float)
        lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cumulative = np.r_[0.0, np.cumsum(lengths)]
        length = float(cumulative[-1])
        number = max(1, int(math.ceil(length / spacing)))
        covariance = np.diag(
            np.asarray(spec["initial_pose"]["cov_diag_variances"], dtype=float)
        )
        previous = points[0]
        contact_risk = 0.0

        # Small closed-loop tracking errors caused by unobserved actuation
        # during a sampling interval are additional to localization error.
        tracking_variance = (
            (float(actuator["v"]) * dt) ** 2
            + (lookahead * float(actuator["omega"]) * dt) ** 2
        )

        for k in range(number + 1):
            along = min(k * spacing, length)
            segment = min(
                int(np.searchsorted(cumulative, along, side="right") - 1),
                len(lengths) - 1,
            )
            segment = max(segment, 0)
            fraction = (
                (along - cumulative[segment]) / lengths[segment]
                if lengths[segment] > 0 else 0.0
            )
            point = points[segment] + fraction * (
                points[segment + 1] - points[segment]
            )
            direction = points[segment + 1] - points[segment]
            heading = math.atan2(direction[1], direction[0])

            if k:
                delta = point - previous
                f = np.eye(3)
                f[0, 2], f[1, 2] = -delta[1], delta[0]
                c, s = math.cos(heading), math.sin(heading)
                rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                q = rotation @ np.diag(
                    np.square(np.asarray(noise["odom"], dtype=float))
                ) @ rotation.T
                covariance = f @ covariance @ f.T + q

            if gps_available(k):
                covariance = update_covariance(
                    covariance,
                    np.array([[1., 0., 0.], [0., 1., 0.]]),
                    [float(noise["gps"]) ** 2] * 2,
                )

            for landmark in landmarks:
                dx, dy = landmark - point
                r2 = float(dx * dx + dy * dy)
                if 1e-12 < r2 <= visibility ** 2:
                    r = math.sqrt(r2)
                    h = np.array([
                        [dy / r2, -dx / r2, -1.],
                        [-dx / r, -dy / r, 0.],
                    ])
                    covariance = update_covariance(
                        covariance, h,
                        [float(noise["bearing"]) ** 2,
                         float(noise["range"]) ** 2],
                    )

            for wall in walls:
                separation = point - nearest_on_wall(point, wall)
                distance = float(np.linalg.norm(separation))
                if distance <= radius:
                    contact_risk += 1.0
                    continue
                normal = separation / distance
                variance = float(
                    normal @ covariance[:2, :2] @ normal
                ) + tracking_variance
                z = (distance - radius) / math.sqrt(max(variance, 1e-20))
                contact_risk += 0.5 * math.erfc(z / math.sqrt(2.0))
            previous = point

        # Allow turning time in the finite-horizon feasibility check.
        headings = np.arctan2(
            np.diff(points, axis=0)[:, 1],
            np.diff(points, axis=0)[:, 0],
        )
        turn = abs(_wrap(headings[0] - spec["initial_pose"]["mean"][2]))
        turn += sum(abs(_wrap(b - a)) for a, b in zip(headings, headings[1:]))
        nominal_time = length / speed + turn / float(controller["omega_max"])
        budget = float(spec["max_steps"]) * dt
        time_std = max(
            float(actuator["v"]) * dt * math.sqrt(number) / speed, 1e-12
        )
        time_probability = 0.5 * math.erfc(
            (nominal_time - budget) / (math.sqrt(2.0) * time_std)
        )
        probability = max(0.0, 1.0 - min(contact_risk, 1.0)) * time_probability
        results.append((name, probability, length, contact_risk))

    best_probability = max(item[1] for item in results)
    near_best = [item for item in results if item[1] >= best_probability - 1e-3]
    chosen = min(near_best, key=lambda item: (item[2], item[0]))
    name, probability, length, risk = chosen
    return {
        "route": name,
        "success_prob": float(np.clip(probability, 0.0, 1.0)),
        "reasoning": (
            f"Route {name} balances wall clearance and localization uncertainty "
            f"over its {length:.2f} m length. The nominal covariance rollout uses "
            "the estimator's odometry, GPS outages, and visible landmark "
            "information, with an additional actuator tracking allowance. "
            f"Accumulated modeled contact risk is {min(risk, 1.0):.4g}; "
            "the time budget is also checked. The reported probability is an "
            "approximate planning score, not a Monte Carlo guarantee."
        ),
    }
