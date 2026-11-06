"""GTSAM pose estimation and geometry/uncertainty-based route selection."""

import math
import numpy as np
import gtsam


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _length_scale(unit):
    scales = {
        "m": 1.0, "meter": 1.0, "meters": 1.0,
        "cm": 0.01, "centimeter": 0.01, "centimeters": 0.01,
        "mm": 0.001, "millimeter": 0.001, "millimeters": 0.001,
    }
    if unit not in scales:
        raise ValueError("Unsupported distance unit: " + str(unit))
    return scales[unit]


def _angle_scale(unit):
    if unit in ("rad", "radian", "radians"):
        return 1.0
    if unit in ("deg", "degree", "degrees"):
        return math.pi / 180.0
    raise ValueError("Unsupported angle unit: " + str(unit))


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        initial = public_spec["initial_pose"]
        self.initial_pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )

        noise = public_spec["sensor_noise_std"]
        # Noise entries are already in SI units, not measurement input units.
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
            str(key): np.asarray(point, dtype=float)
            for key, point in public_spec["landmarks"].items()
        }
        units = public_spec["input_units"]
        self.odom_scale = _length_scale(units["odom"])
        self.bearing_scale = _angle_scale(units["bearing"])

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-4)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.last_step = -1
        self.pose = self.initial_pose

        # Explicitly retain Python callbacks for the lifetime of the graph.
        self.callbacks = []

    def _gps_factor(self, key, measurement):
        observed = np.array(
            [measurement["x"], measurement["y"]], dtype=float
        )

        def error(factor, values, jacobians):
            pose = values.atPose2(key)
            if jacobians is not None:
                c, s = math.cos(pose.theta()), math.sin(pose.theta())
                # World position differentiated with respect to a local
                # Pose2 perturbation.
                jacobians[0] = np.array(
                    [[c, -s, 0.0], [s, c, 0.0]], order="F"
                )
            return np.array([pose.x(), pose.y()]) - observed

        self.callbacks.append(error)
        return gtsam.CustomFactor(self.gps_noise, [key], error)

    def _landmark_factor(self, key, measurement):
        point = self.landmarks[str(measurement["id"])].copy()
        observed_bearing = (
            float(measurement["bearing"]) * self.bearing_scale
        )
        observed_range = float(measurement["range"])

        def error(factor, values, jacobians):
            pose = values.atPose2(key)
            q = np.asarray(pose.transformTo(point), dtype=float)
            qx, qy = float(q[0]), float(q[1])
            radius_squared = max(qx * qx + qy * qy, 1e-24)
            radius = math.sqrt(radius_squared)
            if jacobians is not None:
                jacobians[0] = np.array(
                    [
                        [qy / radius_squared, -qx / radius_squared, -1.0],
                        [-qx / radius, -qy / radius, 0.0],
                    ],
                    order="F",
                )
            return np.array([
                _wrap(math.atan2(qy, qx) - observed_bearing),
                radius - observed_range,
            ])

        self.callbacks.append(error)
        return gtsam.CustomFactor(self.landmark_noise, [key], error)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive sequentially from step 0.")

        key = gtsam.symbol("x", k)
        graph = gtsam.NonlinearFactorGraph()
        initial_values = gtsam.Values()

        if k == 0:
            guess = self.initial_pose
            graph.add(gtsam.PriorFactorPose2(
                key, self.initial_pose, self.prior_noise
            ))
        else:
            odom = new_measurements["odom"]
            # input_units.odom describes translation; dtheta is radians.
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.odom_scale,
                float(odom["dy"]) * self.odom_scale,
                float(odom["dtheta"]),
            )
            guess = self.pose.compose(relative)
            graph.add(gtsam.BetweenFactorPose2(
                gtsam.symbol("x", k - 1), key, relative, self.odom_noise
            ))

        initial_values.insert(key, guess)
        if new_measurements.get("gps") is not None:
            graph.add(self._gps_factor(key, new_measurements["gps"]))
        for measurement in new_measurements.get("landmarks", []):
            graph.add(self._landmark_factor(key, measurement))

        self.isam.update(graph, initial_values)
        # Additional incremental nonlinear iterations, without adding data.
        self.isam.update()
        self.isam.update()

        self.pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(self.isam.marginalCovariance(key), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k

        return (
            [float(self.pose.x()), float(self.pose.y()),
             float(self.pose.theta())],
            covariance.tolist(),
        )


def make_estimator(public_spec):
    return Estimator(public_spec)


def choose_route(public_spec):
    """
    Predict route risk using the estimator's linearized sensor model.

    This is a deterministic engineering approximation, not an empirically
    calibrated Monte Carlo probability. It includes scheduled GPS outages,
    landmark availability, body-frame odometry uncertainty, clearance,
    actuator tracking uncertainty, and the available travel time.
    """
    spec = public_spec
    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    omega_max = float(controller["omega_max"])
    lookahead = float(controller["lookahead"])
    budget = (int(spec["max_steps"]) - 1) * dt
    radius = float(spec["robot_radius"])
    tolerance = float(spec["arrival_tolerance"])
    noise = spec["sensor_noise_std"]
    actuator = spec["actuator_noise_std"]
    landmark_points = [
        np.asarray(p, dtype=float) for p in spec["landmarks"].values()
    ]
    walls = [
        (np.asarray(w[:2], dtype=float), np.asarray(w[2:], dtype=float))
        for w in spec["walls"]
    ]
    max_range = float(spec["landmark_visibility"]["max_range"])
    schedule = spec["gps_schedule"]
    dropout = schedule["dropout_steps"]
    rate = int(schedule["rate_steps"])

    def gps_available(k):
        # A pair denotes the inclusive dropout interval.
        if len(dropout) == 2:
            absent = int(dropout[0]) <= k <= int(dropout[1])
        else:
            absent = k in dropout
        return rate > 0 and k % rate == 0 and not absent

    def update_covariance(P, H, R):
        S = H @ P @ H.T + R
        K = np.linalg.solve(S, H @ P).T
        A = np.eye(3) - K @ H
        return A @ P @ A.T + K @ R @ K.T

    def normal_cdf(x):
        return 0.5 * math.erfc(-x / math.sqrt(2.0))

    def evaluate(points):
        points = np.asarray(points, dtype=float)
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        usable = lengths > 1e-12
        vectors = vectors[usable]
        starts = points[:-1][usable]
        lengths = lengths[usable]
        if len(lengths) == 0:
            return 1.0, 0.0, float("inf")

        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        length = float(cumulative[-1])
        headings = np.arctan2(vectors[:, 1], vectors[:, 0])

        theta0 = float(spec["initial_pose"]["mean"][2])
        c, s = math.cos(theta0), math.sin(theta0)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        P = rotation @ np.diag(
            spec["initial_pose"]["cov_diag_variances"]
        ) @ rotation.T
        Q = np.diag(np.square(np.asarray(noise["odom"], dtype=float)))
        stride = speed * dt
        if stride <= 0 or omega_max <= 0:
            return 0.0, length, 0.0

        count = max(1, int(math.ceil(max(0.0, length - tolerance) / stride)))
        previous_position = points[0].copy()
        log_survival = 0.0
        minimum_clearance = float("inf")
        # Consecutive errors are correlated over roughly a lookahead distance.
        correlation_steps = max(1.0, lookahead / stride)

        for k in range(count + 1):
            distance = min(k * stride, max(0.0, length - tolerance))
            segment = min(
                int(np.searchsorted(cumulative, distance, side="right") - 1),
                len(lengths) - 1,
            )
            fraction = (distance - cumulative[segment]) / lengths[segment]
            position = starts[segment] + fraction * vectors[segment]
            theta = float(headings[segment])

            if k:
                displacement = position - previous_position
                F = np.eye(3)
                F[0, 2] = -displacement[1]
                F[1, 2] = displacement[0]
                c, s = math.cos(theta), math.sin(theta)
                G = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
                P = F @ P @ F.T + G @ Q @ G.T

            if gps_available(k):
                H = np.array([[1., 0., 0.], [0., 1., 0.]])
                P = update_covariance(
                    P, H, np.eye(2) * float(noise["gps"]) ** 2
                )

            for landmark in landmark_points:
                delta = landmark - position
                r2 = float(delta @ delta)
                if 1e-12 < r2 <= max_range ** 2:
                    r = math.sqrt(r2)
                    H = np.array([
                        [delta[1] / r2, -delta[0] / r2, -1.0],
                        [-delta[0] / r, -delta[1] / r, 0.0],
                    ])
                    R = np.diag([
                        float(noise["bearing"]) ** 2,
                        float(noise["range"]) ** 2,
                    ])
                    P = update_covariance(P, H, R)

            tracking_variance = (
                (float(actuator["v"]) * dt) ** 2
                + (speed * dt ** 2 * float(actuator["omega"])) ** 2
            )
            collision_tail = 0.0
            for a, b in walls:
                ab = b - a
                denominator = float(ab @ ab)
                u = (
                    np.clip(float((position - a) @ ab) / denominator, 0, 1)
                    if denominator > 0 else 0.0
                )
                delta = position - (a + u * ab)
                separation = float(np.linalg.norm(delta))
                clearance = separation - radius
                minimum_clearance = min(minimum_clearance, clearance)
                if separation < 1e-12:
                    collision_tail = 1.0
                    break
                direction = delta / separation
                variance = float(direction @ P[:2, :2] @ direction)
                sigma = math.sqrt(max(variance + tracking_variance, 1e-24))
                collision_tail += normal_cdf(-clearance / sigma)

            collision_tail = min(collision_tail, 1.0 - 1e-15)
            log_survival += math.log1p(-collision_tail) / correlation_steps
            previous_position = position

        turns = abs(_wrap(float(headings[0]) - theta0))
        turns += sum(
            abs(_wrap(float(b - a)))
            for a, b in zip(headings[:-1], headings[1:])
        )
        # Conservative allowance: turning time is charged in addition to
        # translational travel, though pure pursuit does both simultaneously.
        required_time = max(0.0, length - tolerance) / speed + turns / omega_max
        time_sigma = max(
            float(actuator["v"]) * dt * math.sqrt(count) / speed, 1e-12
        )
        arrival_probability = normal_cdf((budget - required_time) / time_sigma)
        probability = math.exp(log_survival) * arrival_probability
        return float(np.clip(probability, 0.0, 1.0)), length, minimum_clearance

    candidates = []
    for name, points in spec["routes"].items():
        probability, length, clearance = evaluate(points)
        candidates.append((name, probability, length, clearance))

    # Round only for the close-tie decision; report the unrounded prediction.
    selected = max(
        candidates,
        key=lambda item: (round(item[1], 3), -item[2], item[0]),
    )
    name, probability, length, clearance = selected
    comparisons = "; ".join(
        f"{n}: approximate success {p:.4f}, length {d:.2f} m, "
        f"minimum nominal clearance {c:.2f} m"
        for n, p, d, c in candidates
    )
    return {
        "route": name,
        "success_prob": probability,
        "reasoning": (
            "Selected using a deterministic linearized closed-loop risk "
            "approximation with this estimator's odometry, GPS and landmark "
            "information, actuator tracking uncertainty, wall clearance and "
            "the time budget. Probabilities are approximate, not calibrated "
            "simulation frequencies. " + comparisons
        ),
    }
