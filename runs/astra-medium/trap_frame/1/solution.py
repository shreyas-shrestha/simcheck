"""Online Pose2 estimation with GTSAM and a geometry-aware route selector."""

import math

import gtsam
import numpy as np


def _length_scale(unit):
    scales = {
        "m": 1.0, "meter": 1.0, "metre": 1.0,
        "cm": 0.01, "mm": 0.001, "km": 1000.0,
    }
    return scales[str(unit).lower()]


def _angle_scale(unit):
    scales = {
        "rad": 1.0, "radian": 1.0, "radians": 1.0,
        "deg": math.pi / 180.0, "degree": math.pi / 180.0,
        "degrees": math.pi / 180.0,
    }
    return scales[str(unit).lower()]


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        initial = public_spec["initial_pose"]
        self.initial_pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.initial_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )

        noise = public_spec["sensor_noise_std"]
        # Noise specifications are already SI, independent of input_units.
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
            str(identifier): np.asarray(position, dtype=float)
            for identifier, position in public_spec["landmarks"].items()
        }

        units = public_spec["input_units"]
        self.odom_scale = _length_scale(units["odom"])
        self.bearing_scale = _angle_scale(units["bearing"])
        # The odom unit specifies translation; dtheta is in radians unless an
        # independent odom-angle unit is explicitly provided.
        self.odom_angle_scale = _angle_scale(units.get("odom_angle", "rad"))
        self.odom_frame = public_spec["odom_frame"]

        parameters = gtsam.ISAM2Params()
        parameters.setRelinearizeThreshold(1e-4)
        parameters.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(parameters)

        self.last_step = -1
        self.last_pose = self.initial_pose
        self.reported_heading = 0.0
        # Keep Python callback objects alive as long as their C++ factors.
        self.callbacks = []

    def _gps_factor(self, key, measurement):
        observed = np.array(
            [measurement["x"], measurement["y"]], dtype=float
        )

        def error(factor, values, jacobians):
            pose = values.atPose2(key)
            if jacobians is not None:
                c, s = math.cos(pose.theta()), math.sin(pose.theta())
                jacobians[0] = np.array(
                    [[c, -s, 0.0], [s, c, 0.0]], order="F"
                )
            return np.array([pose.x(), pose.y()]) - observed

        self.callbacks.append(error)
        return gtsam.CustomFactor(self.gps_noise, [key], error)

    def _landmark_factor(self, key, measurement):
        point = self.landmarks[str(measurement["id"])]
        measured_bearing = float(measurement["bearing"]) * self.bearing_scale
        measured_range = float(measurement["range"])

        def error(factor, values, jacobians):
            pose = values.atPose2(key)
            c, s = math.cos(pose.theta()), math.sin(pose.theta())
            dx, dy = point[0] - pose.x(), point[1] - pose.y()
            qx, qy = c * dx + s * dy, -s * dx + c * dy
            squared_range = max(qx * qx + qy * qy, 1e-24)
            distance = math.sqrt(squared_range)

            if jacobians is not None:
                # Derivatives with respect to Pose2's LOCAL tangent vector.
                jacobians[0] = np.array(
                    [
                        [qy / squared_range, -qx / squared_range, -1.0],
                        [-qx / distance, -qy / distance, 0.0],
                    ],
                    order="F",
                )
            return np.array([
                _wrap(math.atan2(qy, qx) - measured_bearing),
                distance - measured_range,
            ])

        self.callbacks.append(error)
        return gtsam.CustomFactor(self.landmark_noise, [key], error)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must be supplied sequentially from step 0.")

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        key = gtsam.symbol("x", k)

        if k == 0:
            guess = self.initial_pose
            graph.add(gtsam.PriorFactorPose2(
                key, self.initial_pose, self.initial_noise
            ))
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("A transition odometry measurement is required.")

            dx = float(odom["dx"]) * self.odom_scale
            dy = float(odom["dy"]) * self.odom_scale
            dtheta = float(odom["dtheta"]) * self.odom_angle_scale

            if self.odom_frame == "odom":
                # Undo the encoder's running orientation BEFORE incorporating
                # this transition's dtheta. Never use the corrected estimate
                # here: the reporting frame is defined by reported odometry.
                c = math.cos(self.reported_heading)
                s = math.sin(self.reported_heading)
                dx, dy = c * dx + s * dy, -s * dx + c * dy
            elif self.odom_frame not in ("body", "robot", "local"):
                raise ValueError("Unsupported odometry frame: " + self.odom_frame)

            relative_pose = gtsam.Pose2(dx, dy, dtheta)
            graph.add(gtsam.BetweenFactorPose2(
                gtsam.symbol("x", k - 1), key,
                relative_pose, self.odom_noise
            ))
            guess = self.last_pose.compose(relative_pose)
            self.reported_heading = _wrap(self.reported_heading + dtheta)

        values.insert(key, guess)

        gps = new_measurements.get("gps")
        if gps is not None:
            graph.add(self._gps_factor(key, gps))

        for observation in new_measurements.get("landmarks", []):
            graph.add(self._landmark_factor(key, observation))

        self.isam.update(graph, values)
        # Extra nonlinear iterations are particularly useful when landmarks
        # first become visible or GPS returns after a dropout.
        self.isam.update()
        self.isam.update()

        pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(self.isam.marginalCovariance(key), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)

        self.last_pose = pose
        self.last_step = k
        return (
            [float(pose.x()), float(pose.y()), _wrap(pose.theta())],
            covariance.tolist(),
        )


def make_estimator(public_spec):
    return Estimator(public_spec)


def choose_route(public_spec):
    """
    Compare nominal routes using a linearized approximation to this estimator,
    wall-clearance Gaussian tails, and the travel-time budget.

    The returned probability is an analytical planning approximation, not an
    assertion that closed-loop Monte Carlo trials have been performed.
    """
    spec = public_spec
    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    spacing = speed * dt
    if spacing <= 0:
        raise ValueError("Controller speed and dt must be positive.")

    noise = spec["sensor_noise_std"]
    odom_variances = np.square(np.asarray(noise["odom"], dtype=float))
    gps_variance = float(noise["gps"]) ** 2
    bearing_variance = float(noise["bearing"]) ** 2
    range_variance = float(noise["range"]) ** 2
    actuator = spec["actuator_noise_std"]
    radius = float(spec["robot_radius"])
    visibility = float(spec["landmark_visibility"]["max_range"])
    landmarks = [
        np.asarray(p, dtype=float) for p in spec["landmarks"].values()
    ]
    walls = [np.asarray(w, dtype=float) for w in spec["walls"]]
    schedule = spec["gps_schedule"]
    gps_rate = int(schedule["rate_steps"])
    dropout = schedule["dropout_steps"]
    tolerance = float(spec["arrival_tolerance"])
    max_steps = int(spec["max_steps"])

    def gps_available(k):
        if gps_rate <= 0 or k % gps_rate:
            return False
        # A pair specifies the inclusive dropout interval in this scenario.
        if len(dropout) == 2 and all(np.isscalar(x) for x in dropout):
            return not (int(dropout[0]) <= k <= int(dropout[1]))
        return k not in dropout

    def update_covariance(covariance, h, measurement_covariance):
        innovation = h @ covariance @ h.T + measurement_covariance
        gain = np.linalg.solve(innovation, h @ covariance).T
        a = np.eye(3) - gain @ h
        result = (
            a @ covariance @ a.T
            + gain @ measurement_covariance @ gain.T
        )
        return 0.5 * (result + result.T)

    def normal_cdf(x):
        return 0.5 * math.erfc(-x / math.sqrt(2.0))

    def evaluate(route):
        vertices = np.asarray(route, dtype=float)
        differences = np.diff(vertices, axis=0)
        lengths = np.linalg.norm(differences, axis=1)
        valid = lengths > 0
        differences = differences[valid]
        starts = vertices[:-1][valid]
        lengths = lengths[valid]
        total_length = float(lengths.sum())
        cumulative = np.cumsum(lengths)
        count = max(1, int(math.ceil(total_length / spacing)))

        mean = spec["initial_pose"]["mean"]
        theta = float(mean[2])
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        covariance = (
            rotation
            @ np.diag(spec["initial_pose"]["cov_diag_variances"])
            @ rotation.T
        )
        previous = vertices[0].copy()
        collision_bound = 0.0
        minimum_clearance = float("inf")

        for k in range(count + 1):
            travelled = min(k * spacing, total_length)
            if len(lengths):
                segment = min(
                    int(np.searchsorted(cumulative, travelled, side="right")),
                    len(lengths) - 1,
                )
                before = 0.0 if segment == 0 else cumulative[segment - 1]
                fraction = (travelled - before) / lengths[segment]
                position = starts[segment] + fraction * differences[segment]
                theta = math.atan2(
                    differences[segment, 1], differences[segment, 0]
                )
            else:
                position = vertices[0]

            if k:
                delta = position - previous
                f = np.eye(3)
                f[0, 2], f[1, 2] = -delta[1], delta[0]
                c, s = math.cos(theta), math.sin(theta)
                rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
                covariance = (
                    f @ covariance @ f.T
                    + rotation @ np.diag(odom_variances) @ rotation.T
                )

            if gps_available(k):
                covariance = update_covariance(
                    covariance, np.array([[1., 0, 0], [0, 1., 0]]),
                    gps_variance * np.eye(2),
                )

            for landmark in landmarks:
                dx, dy = landmark - position
                r2 = float(dx * dx + dy * dy)
                if 1e-20 < r2 <= visibility ** 2:
                    r = math.sqrt(r2)
                    h = np.array([
                        [dy / r2, -dx / r2, -1.0],
                        [-dx / r, -dy / r, 0.0],
                    ])
                    covariance = update_covariance(
                        covariance, h,
                        np.diag([bearing_variance, range_variance]),
                    )

            # Include one control interval of actuation uncertainty in the
            # collision calculation, but not in the odometry factor model.
            motion_variance = (
                (float(actuator["v"]) * dt) ** 2
                + (speed * float(actuator["omega"]) * dt * dt) ** 2
            )
            position_covariance = covariance[:2, :2] + motion_variance * np.eye(2)

            for wall in walls:
                a, b = wall[:2], wall[2:]
                direction = b - a
                denominator = float(direction @ direction)
                fraction = (
                    np.clip((position - a) @ direction / denominator, 0., 1.)
                    if denominator > 0 else 0.0
                )
                separation = position - (a + fraction * direction)
                distance = float(np.linalg.norm(separation))
                clearance = distance - radius
                minimum_clearance = min(minimum_clearance, clearance)
                if distance > 1e-12:
                    normal = separation / distance
                    variance = float(normal @ position_covariance @ normal)
                else:
                    variance = float(np.linalg.eigvalsh(position_covariance)[-1])
                sigma = math.sqrt(max(variance, 1e-24))
                collision_bound += normal_cdf(-clearance / sigma)

            previous = position.copy()

        # Available travel distance, with integrated speed-actuation noise.
        distance_budget = max_steps * spacing
        budget_sigma = max(
            float(actuator["v"]) * dt * math.sqrt(max_steps), 1e-12
        )
        timely = normal_cdf(
            (distance_budget - max(total_length - tolerance, 0.0)) / budget_sigma
        )
        terminal_variance = max(
            float(np.linalg.eigvalsh(covariance[:2, :2])[-1]), 1e-24
        )
        localized = -math.expm1(-tolerance ** 2 / (2 * terminal_variance))
        probability = timely * localized * max(0.0, 1.0 - collision_bound)
        return float(np.clip(probability, 0., 1.)), total_length, minimum_clearance

    results = {
        name: evaluate(route) for name, route in spec["routes"].items()
    }
    # Prefer shorter distance when probabilities agree to three decimals.
    selected = max(
        results,
        key=lambda name: (round(results[name][0], 3), -results[name][1]),
    )
    probability, length, clearance = results[selected]
    return {
        "route": selected,
        "success_prob": probability,
        "reasoning": (
            f"Route {selected} has nominal length {length:.2f} m and minimum "
            f"wall clearance after robot radius {clearance:.2f} m. "
            "Selection balances Gaussian wall-risk estimates, landmark/GPS "
            "localization (including GPS dropout), and the travel-time budget. "
            "The probability is a nominal linearized planning approximation; "
            "it does not include a full pure-pursuit Monte Carlo simulation."
        ),
    }
