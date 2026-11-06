"""GTSAM Pose2 localization and specification-driven route selection."""

import copy
import math

import gtsam
import numpy as np
from scipy.special import ndtr


def _length_scale(unit):
    scales = {
        "m": 1.0, "meter": 1.0, "metre": 1.0,
        "cm": 0.01, "mm": 0.001, "km": 1000.0,
    }
    if unit not in scales:
        raise ValueError("Unsupported position unit: %r" % unit)
    return scales[unit]


def _angle_scale(unit):
    if unit in ("rad", "radian", "radians"):
        return 1.0
    if unit in ("deg", "degree", "degrees"):
        return math.pi / 180.0
    raise ValueError("Unsupported angle unit: %r" % unit)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _rotation(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _gps_factor(key, measurement, noise):
    z = np.asarray(measurement, dtype=float).copy()

    def error(factor, values, jacobians):
        pose = values.atPose2(key)
        if jacobians is not None:
            H = np.zeros((2, 3), order="F")
            H[:, :2] = _rotation(pose.theta())
            jacobians[0] = H
        return np.array([pose.x(), pose.y()]) - z

    return gtsam.CustomFactor(noise, [key], error)


def _landmark_factor(key, point, bearing, distance, noise):
    landmark = np.asarray(point, dtype=float).copy()

    def error(factor, values, jacobians):
        pose = values.atPose2(key)
        q = _rotation(pose.theta()).T @ (
            landmark - np.array([pose.x(), pose.y()])
        )
        r2 = max(float(q @ q), np.finfo(float).tiny)
        r = math.sqrt(r2)
        if jacobians is not None:
            # Derivatives with respect to a right/local Pose2 perturbation.
            jacobians[0] = np.array([
                [q[1] / r2, -q[0] / r2, -1.0],
                [-q[0] / r, -q[1] / r, 0.0],
            ], order="F")
        return np.array([
            _wrap(math.atan2(q[1], q[0]) - bearing),
            r - distance,
        ])

    return gtsam.CustomFactor(noise, [key], error)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        spec = self.spec
        units = spec["input_units"]
        self.odom_scale = _length_scale(units["odom"])
        self.bearing_scale = _angle_scale(units["bearing"])
        # Odometry's length unit does not change its angular component.
        self.odom_angle_scale = _angle_scale(units.get("odom_angle", "rad"))
        self.range_scale = _length_scale(units.get("range", "m"))
        self.gps_scale = _length_scale(units.get("gps", "m"))

        initial = spec["initial_pose"]
        self.initial_pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )

        noise = spec["sensor_noise_std"]
        # All noise parameters already have SI units.
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
            str(k): np.asarray(v, dtype=float)
            for k, v in spec["landmarks"].items()
        }

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-4)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.last_step = -1
        self.last_pose = self.initial_pose

    def step(self, new_measurements):
        m = new_measurements
        k = int(m["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in step order from zero.")

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        key = gtsam.symbol("x", k)

        if k == 0:
            guess = self.initial_pose
            graph.add(gtsam.PriorFactorPose2(
                key, self.initial_pose, self.prior_noise
            ))
        else:
            odom = m["odom"]
            if odom is None:
                raise ValueError("A noninitial step requires relative odometry.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.odom_scale,
                float(odom["dy"]) * self.odom_scale,
                float(odom["dtheta"]) * self.odom_angle_scale,
            )
            guess = self.last_pose.compose(relative)
            graph.add(gtsam.BetweenFactorPose2(
                gtsam.symbol("x", k - 1), key, relative, self.odom_noise
            ))

        values.insert(key, guess)

        gps = m.get("gps")
        if gps is not None:
            graph.add(_gps_factor(
                key,
                [float(gps["x"]) * self.gps_scale,
                 float(gps["y"]) * self.gps_scale],
                self.gps_noise,
            ))

        for observation in m.get("landmarks", []):
            point = self.landmarks[str(observation["id"])]
            graph.add(_landmark_factor(
                key, point,
                float(observation["bearing"]) * self.bearing_scale,
                float(observation["range"]) * self.range_scale,
                self.landmark_noise,
            ))

        self.isam.update(graph, values)
        # Additional incremental iterations resolve nonlinear bearing updates
        # and bring the covariance linearization close to the returned pose.
        for _ in range(3):
            self.isam.update()

        pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(self.isam.marginalCovariance(key), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)

        self.last_step = k
        self.last_pose = pose
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
    # The supplied specification encodes an inclusive outage interval.
    if len(dropout) == 2 and all(np.isscalar(x) for x in dropout):
        return not (int(dropout[0]) <= k <= int(dropout[1]))
    if dropout and isinstance(dropout[0], (list, tuple)):
        return not any(int(a) <= k <= int(b) for a, b in dropout)
    return k not in dropout


def _route_assessment(spec, waypoints):
    """Nominal Gaussian risk approximation, not an empirical success claim."""
    points = np.asarray(waypoints, dtype=float)
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = lengths > 0.0
    segments, lengths = segments[valid], lengths[valid]
    starts = points[:-1][valid]
    if not len(lengths):
        return 1.0, 0.0, math.inf

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    total = float(cumulative[-1])
    headings = np.arctan2(segments[:, 1], segments[:, 0])
    dt = float(spec["dt"])
    speed = float(spec["controller"]["v"])
    stride = speed * dt
    if stride <= 0:
        return 0.0, total, -math.inf

    radius = float(spec["robot_radius"])
    noise = spec["sensor_noise_std"]
    Q = np.diag(np.square(np.asarray(noise["odom"], dtype=float)))
    P = np.diag(np.asarray(
        spec["initial_pose"]["cov_diag_variances"], dtype=float
    ))
    prior_pose = gtsam.Pose2(*map(float, spec["initial_pose"]["mean"]))
    landmarks = [np.asarray(p, dtype=float)
                 for p in spec["landmarks"].values()]
    max_range = float(spec["landmark_visibility"]["max_range"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)

    risk_sum = 0.0
    minimum_clearance = math.inf
    travel = max(0.0, total - float(spec["arrival_tolerance"]))
    steps = int(math.ceil(travel / stride))

    for k in range(steps + 1):
        distance = min(k * stride, travel)
        j = min(int(np.searchsorted(cumulative, distance, side="right") - 1),
                len(lengths) - 1)
        position = starts[j] + segments[j] * (
            (distance - cumulative[j]) / lengths[j]
        )
        theta = float(headings[j])
        pose = gtsam.Pose2(float(position[0]), float(position[1]), theta)
        R = _rotation(theta)

        if k:
            relative = prior_pose.between(pose)
            F = relative.inverse().AdjointMap()
            P = F @ P @ F.T + Q
        else:
            # Express the initial uncertainty at the nominal route heading.
            T = np.eye(3)
            T[:2, :2] = R.T @ _rotation(prior_pose.theta())
            P = T @ P @ T.T

        information = np.linalg.inv(P)
        if _gps_available(k, spec["gps_schedule"]):
            H = np.zeros((2, 3))
            H[:, :2] = R
            information += H.T @ H / float(noise["gps"]) ** 2

        for landmark in landmarks:
            q = R.T @ (landmark - position)
            r2 = float(q @ q)
            if 0.0 < r2 <= max_range ** 2:
                r = math.sqrt(r2)
                H = np.array([
                    [q[1] / r2, -q[0] / r2, -1.0],
                    [-q[0] / r, -q[1] / r, 0.0],
                ])
                information += H.T @ np.diag([
                    1.0 / float(noise["bearing"]) ** 2,
                    1.0 / float(noise["range"]) ** 2,
                ]) @ H
        P = np.linalg.inv(information)

        # Approximate tracking uncertainty over a controller lookahead.
        lookahead = float(spec["controller"]["lookahead"])
        tracking = np.array([[1.0, 0.0, 0.0],
                             [0.0, 1.0, lookahead]])
        local_cov = tracking @ P @ tracking.T
        actuator = spec["actuator_noise_std"]
        local_cov += np.diag([
            (float(actuator["v"]) * dt) ** 2,
            (speed * float(actuator["omega"]) * dt ** 2 / 2.0) ** 2,
        ])
        world_cov = R @ local_cov @ R.T

        for wall in walls:
            a, b = wall[:2], wall[2:]
            ab = b - a
            denom = float(ab @ ab)
            fraction = (float((position - a) @ ab) / denom
                        if denom > 0.0 else 0.0)
            closest = a + np.clip(fraction, 0.0, 1.0) * ab
            delta = position - closest
            d = float(np.linalg.norm(delta))
            clearance = d - radius
            minimum_clearance = min(minimum_clearance, clearance)
            normal = delta / d if d > 0.0 else np.array([1.0, 0.0])
            variance = max(float(normal @ world_cov @ normal),
                           np.finfo(float).tiny)
            risk_sum += float(ndtr(-clearance / math.sqrt(variance)))
        prior_pose = pose

    # Conservative turn-time allowance and longitudinal actuator uncertainty.
    turns = abs(_wrap(float(headings[0]) -
                      float(spec["initial_pose"]["mean"][2])))
    turns += sum(abs(_wrap(float(b - a)))
                 for a, b in zip(headings[:-1], headings[1:]))
    omega_max = float(spec["controller"]["omega_max"])
    turning_time = turns / omega_max if omega_max > 0 else math.inf
    budget = (int(spec["max_steps"]) - 1) * dt
    slack = speed * (budget - turning_time) - travel
    distance_std = (float(spec["actuator_noise_std"]["v"]) * dt
                    * math.sqrt(max(1, int(spec["max_steps"]) - 1)))
    arrival_prob = (float(ndtr(slack / distance_std))
                    if distance_std > 0 else float(slack >= 0))
    probability = arrival_prob * max(0.0, 1.0 - risk_sum)
    return float(np.clip(probability, 0.0, 1.0)), total, minimum_clearance


def choose_route(public_spec):
    assessments = {
        name: _route_assessment(public_spec, route)
        for name, route in public_spec["routes"].items()
    }
    # Round only for tie-breaking; retain the unrounded reported probability.
    selected = min(
        assessments,
        key=lambda name: (
            -round(assessments[name][0], 3),
            assessments[name][1],
            name,
        ),
    )
    probability, length, clearance = assessments[selected]
    return {
        "route": selected,
        "success_prob": probability,
        "reasoning": (
            "Selected using nominal propagation of this estimator's local "
            "Pose2 uncertainty, scheduled GPS and visible landmark updates, "
            "controller lookahead, actuator noise, wall clearance and time "
            "budget. Route %s has length %.2f m and minimum nominal "
            "robot-edge clearance %.2f m. The probability is a Gaussian "
            "planning approximation, not a calibrated closed-loop Monte "
            "Carlo result; distance breaks close probability ties."
            % (selected, length, clearance)
        ),
    }
