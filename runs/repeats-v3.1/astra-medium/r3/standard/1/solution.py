"""Pose2 localization with known landmarks and incremental GTSAM smoothing."""

import copy
import math

import gtsam
import numpy as np


def _length_scale(unit):
    unit = str(unit).strip().lower()
    scales = {
        "m": 1.0, "meter": 1.0, "meters": 1.0,
        "metre": 1.0, "metres": 1.0,
        "cm": 0.01, "centimeter": 0.01, "centimeters": 0.01,
        "mm": 0.001, "millimeter": 0.001, "millimeters": 0.001,
    }
    if unit not in scales:
        raise ValueError("Unsupported position unit: " + unit)
    return scales[unit]


def _angle_scale(unit):
    unit = str(unit).strip().lower()
    if unit in ("rad", "radian", "radians"):
        return 1.0
    if unit in ("deg", "degree", "degrees"):
        return math.pi / 180.0
    raise ValueError("Unsupported angle unit: " + unit)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _gps_factor(key, measurement, noise):
    measurement = np.asarray(measurement, dtype=float).copy()

    def error(this, values, jacobians):
        pose = values.atPose2(key)
        if jacobians is not None:
            c, s = math.cos(pose.theta()), math.sin(pose.theta())
            jacobians[0] = np.array(
                [[c, -s, 0.0], [s, c, 0.0]], order="F"
            )
        return np.array([pose.x(), pose.y()]) - measurement

    return gtsam.CustomFactor(noise, [key], error)


def _landmark_factor(key, point, bearing, distance, noise):
    # Landmarks are constants, not uncertain graph variables.
    point = np.asarray(point, dtype=float).copy()

    def error(this, values, jacobians):
        pose = values.atPose2(key)
        q = np.asarray(pose.transformTo(point), dtype=float)
        qx, qy = float(q[0]), float(q[1])
        squared_range = qx * qx + qy * qy
        predicted_range = math.sqrt(squared_range)

        if jacobians is not None:
            # Right/local Pose2 perturbations, as used by GTSAM.
            safe_squared_range = max(squared_range, 1e-24)
            safe_range = max(predicted_range, 1e-12)
            jacobians[0] = np.array(
                [
                    [qy / safe_squared_range,
                     -qx / safe_squared_range, -1.0],
                    [-qx / safe_range, -qy / safe_range, 0.0],
                ],
                order="F",
            )

        return np.array([
            _wrap(math.atan2(qy, qx) - bearing),
            predicted_range - distance,
        ])

    return gtsam.CustomFactor(noise, [key], error)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        spec = self.spec
        sensor = spec["sensor_noise_std"]

        self.initial_pose = gtsam.Pose2(
            *map(float, spec["initial_pose"]["mean"])
        )
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(
                spec["initial_pose"]["cov_diag_variances"], dtype=float
            )
        )
        # These are already SI standard deviations: never rescale them.
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(sensor["odom"], dtype=float)
        )
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(
            2, float(sensor["gps"])
        )
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([sensor["bearing"], sensor["range"]], dtype=float)
        )
        self.landmarks = {
            str(identifier): np.asarray(point, dtype=float)
            for identifier, point in spec["landmarks"].items()
        }

        units = spec["input_units"]
        odom_units = units["odom"]
        if isinstance(odom_units, dict):
            self.odom_scale = _length_scale(odom_units["position"])
            self.odom_angle_scale = _angle_scale(odom_units["angle"])
        else:
            self.odom_scale = _length_scale(odom_units)
            # A position unit on odometry does not change dtheta.
            self.odom_angle_scale = 1.0
        self.bearing_scale = _angle_scale(units["bearing"])

        if spec["odom_frame"] != "body":
            raise ValueError("This estimator requires body-frame odometry.")

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-5)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.last_step = -1
        self.last_pose = None
        # Keep Python callbacks alive for as long as their factors are stored.
        self.unary_factors = []

    def step(self, new_measurements):
        measurement = new_measurements
        k = int(measurement["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in consecutive order.")

        key = gtsam.symbol("x", k)
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        if k == 0:
            guess = self.initial_pose
            graph.add(gtsam.PriorFactorPose2(
                key, self.initial_pose, self.prior_noise
            ))
        else:
            odom = measurement["odom"]
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.odom_scale,
                float(odom["dy"]) * self.odom_scale,
                float(odom["dtheta"]) * self.odom_angle_scale,
            )
            guess = self.last_pose.compose(relative)
            graph.add(gtsam.BetweenFactorPose2(
                gtsam.symbol("x", k - 1), key, relative, self.odom_noise
            ))

        initial.insert(key, guess)

        gps = measurement.get("gps")
        if gps is not None:
            factor = _gps_factor(
                key, [float(gps["x"]), float(gps["y"])], self.gps_noise
            )
            self.unary_factors.append(factor)
            graph.add(factor)

        for observation in measurement.get("landmarks", []):
            identifier = str(observation["id"])
            factor = _landmark_factor(
                key,
                self.landmarks[identifier],
                float(observation["bearing"]) * self.bearing_scale,
                float(observation["range"]),
                self.landmark_noise,
            )
            self.unary_factors.append(factor)
            graph.add(factor)

        self.isam.update(graph, initial)
        # Additional nonlinear iterations also refresh the marginal's
        # linearization point after incorporating the new observations.
        for _ in range(3):
            self.isam.update()

        pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(
            self.isam.marginalCovariance(key), dtype=float
        )
        covariance = 0.5 * (covariance + covariance.T)

        self.last_pose = pose
        self.last_step = k
        return (
            [float(pose.x()), float(pose.y()), float(pose.theta())],
            covariance.tolist(),
        )


def make_estimator(public_spec):
    return Estimator(public_spec)


def _gps_available(k, schedule):
    if k % int(schedule["rate_steps"]) != 0:
        return False
    dropout = schedule["dropout_steps"]
    # In this specification, [first, last] is an inclusive outage interval.
    if len(dropout) == 2 and all(np.isscalar(x) for x in dropout):
        return not (int(dropout[0]) <= k <= int(dropout[1]))
    for interval in dropout:
        if isinstance(interval, (list, tuple)):
            if int(interval[0]) <= k <= int(interval[1]):
                return False
        elif k == int(interval):
            return False
    return True


def _covariance_update(covariance, H, variances):
    R = np.diag(np.asarray(variances, dtype=float))
    innovation_covariance = H @ covariance @ H.T + R
    gain = np.linalg.solve(innovation_covariance, H @ covariance).T
    residual_map = np.eye(3) - gain @ H
    result = (
        residual_map @ covariance @ residual_map.T + gain @ R @ gain.T
    )
    return 0.5 * (result + result.T)


def _route_assessment(spec, route):
    """Conservative nominal-path risk approximation, not a Monte Carlo claim.

    The covariance recursion approximates the current-pose marginal of the
    estimator. Wall risk uses a Gaussian clearance tail and a union bound.
    An additional tracking allowance accounts for unobserved motion between
    control updates.
    """
    points = np.asarray(route, dtype=float)
    differences = np.diff(points, axis=0)
    lengths = np.linalg.norm(differences, axis=1)
    keep = lengths > 1e-12
    starts = points[:-1][keep]
    differences = differences[keep]
    lengths = lengths[keep]
    total_length = float(np.sum(lengths))

    if not len(lengths):
        return 1.0, total_length, 0

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    omega_max = float(controller["omega_max"])
    lookahead = float(controller["lookahead"])
    tolerance = float(spec["arrival_tolerance"])
    max_transitions = int(spec["max_steps"]) - 1
    travel_distance = max(0.0, total_length - tolerance)

    if speed <= 0.0:
        return 0.0, total_length, max_transitions + 1

    headings = np.arctan2(differences[:, 1], differences[:, 0])
    initial_heading = float(spec["initial_pose"]["mean"][2])
    turning = abs(_wrap(float(headings[0]) - initial_heading))
    turning += sum(
        abs(_wrap(float(b - a))) for a, b in zip(headings[:-1], headings[1:])
    )
    # Conservative time allowance: assume turns consume additional time,
    # although pure pursuit normally turns while translating.
    turn_time = turning / omega_max if omega_max > 0 else (
        0.0 if turning == 0.0 else math.inf
    )
    nominal_time = travel_distance / speed + turn_time
    required_steps = (
        int(math.ceil(nominal_time / dt))
        if math.isfinite(nominal_time) else max_transitions + 1
    )

    sensor = spec["sensor_noise_std"]
    odom_variances = np.square(np.asarray(sensor["odom"], dtype=float))
    covariance = np.diag(
        np.asarray(spec["initial_pose"]["cov_diag_variances"], dtype=float)
    )
    c, s = math.cos(initial_heading), math.sin(initial_heading)
    transform = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    covariance = transform @ covariance @ transform.T

    landmarks = [
        np.asarray(point, dtype=float)
        for point in spec["landmarks"].values()
    ]
    visibility = float(spec["landmark_visibility"]["max_range"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)
    radius = float(spec["robot_radius"])
    actuator = spec["actuator_noise_std"]
    tracking_variance = (
        (float(actuator["v"]) * dt) ** 2
        + (lookahead * float(actuator["omega"]) * dt) ** 2
    )

    motion_steps = max(1, int(math.ceil(travel_distance / (speed * dt))))
    collision_bound = 0.0
    previous_position = starts[0].copy()
    H_gps = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    for k in range(motion_steps + 1):
        distance = min(k * speed * dt, travel_distance)
        segment = min(
            int(np.searchsorted(cumulative, distance, side="right") - 1),
            len(lengths) - 1,
        )
        fraction = (distance - cumulative[segment]) / lengths[segment]
        position = starts[segment] + fraction * differences[segment]
        heading = float(headings[segment])

        if k:
            displacement = position - previous_position
            F = np.eye(3)
            F[0, 2] = -displacement[1]
            F[1, 2] = displacement[0]
            c, s = math.cos(heading), math.sin(heading)
            G = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            covariance = F @ covariance @ F.T + (
                G @ np.diag(odom_variances) @ G.T
            )

        if _gps_available(k, spec["gps_schedule"]):
            covariance = _covariance_update(
                covariance, H_gps, [float(sensor["gps"]) ** 2] * 2
            )

        for landmark in landmarks:
            delta = landmark - position
            r2 = float(delta @ delta)
            if 1e-20 < r2 <= visibility * visibility:
                r = math.sqrt(r2)
                H = np.array([
                    [delta[1] / r2, -delta[0] / r2, -1.0],
                    [-delta[0] / r, -delta[1] / r, 0.0],
                ])
                covariance = _covariance_update(
                    covariance, H,
                    [float(sensor["bearing"]) ** 2,
                     float(sensor["range"]) ** 2],
                )

        for wall in walls:
            a, b = wall[:2], wall[2:]
            direction = b - a
            squared_length = float(direction @ direction)
            projection = (
                np.clip(float((position - a) @ direction) / squared_length,
                        0.0, 1.0)
                if squared_length > 0.0 else 0.0
            )
            separation = position - (a + projection * direction)
            clearance_distance = float(np.linalg.norm(separation))
            if clearance_distance <= radius:
                collision_bound = 1.0
                continue
            normal = separation / clearance_distance
            variance = float(normal @ covariance[:2, :2] @ normal)
            sigma = math.sqrt(max(variance + tracking_variance, 1e-24))
            z = (clearance_distance - radius) / sigma
            collision_bound += 0.5 * math.erfc(z / math.sqrt(2.0))

        previous_position = position

    # Approximate deadline risk from accumulated longitudinal actuator noise.
    available_time = max_transitions * dt
    time_sigma = (
        float(actuator["v"]) * dt
        * math.sqrt(max(required_steps, 1)) / speed
    )
    if math.isfinite(nominal_time):
        if time_sigma > 0.0:
            deadline_failure = 0.5 * math.erfc(
                (available_time - nominal_time)
                / (math.sqrt(2.0) * time_sigma)
            )
        else:
            deadline_failure = float(nominal_time > available_time)
    else:
        deadline_failure = 1.0

    success = max(0.0, min(1.0, 1.0 - collision_bound - deadline_failure))
    return success, total_length, required_steps


def choose_route(public_spec):
    assessments = {
        name: _route_assessment(public_spec, route)
        for name, route in public_spec["routes"].items()
    }
    best_probability = max(value[0] for value in assessments.values())
    # Treat probabilities within one percentage point as a close tie.
    candidates = [
        name for name, value in assessments.items()
        if value[0] >= best_probability - 0.01
    ]
    chosen = min(candidates, key=lambda name: (assessments[name][1], name))
    probability, length, steps = assessments[chosen]

    summary = "; ".join(
        f"{name}: approximate success={value[0]:.4f}, "
        f"length={value[1]:.2f} m"
        for name, value in sorted(assessments.items())
    )
    return {
        "route": chosen,
        "success_prob": float(probability),
        "reasoning": (
            f"Choose {chosen}. {summary}. "
            "Risk is estimated from nominal-route sensor-fusion covariance, "
            "robot-radius-adjusted wall clearance, inter-update actuator "
            "tracking uncertainty, and the travel deadline. This is a "
            "Gaussian/union-bound approximation, not an empirical closed-loop "
            f"success rate. The chosen route is {length:.2f} m with a "
            f"conservative time allowance of {steps} transitions. "
            "Distance breaks probability ties within one percentage point."
        ),
    }
