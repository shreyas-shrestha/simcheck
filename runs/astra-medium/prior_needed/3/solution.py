"""GTSAM odometry estimation and deterministic, uncertainty-aware route selection."""

import copy
import math

import gtsam
import numpy as np


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))

        if self.spec["odom_frame"] != "body":
            raise ValueError("Expected body-frame relative-pose odometry.")

        unit = self.spec["input_units"]["odom"].lower()
        self.translation_scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[unit]

        # Covariance entries are variances; sensor noise entries are SI sigmas.
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        )

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPose2(0, self.pose, prior_noise))
        values.insert(0, self.pose)
        self.isam.update(graph, values)
        self.last_step = -1

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in step order.")
        if new_measurements.get("gps") is not None:
            raise ValueError("This estimator is for the specified no-GPS scenario.")
        if new_measurements.get("landmarks"):
            raise ValueError("This estimator is for the specified no-landmark scenario.")

        if k == 0:
            if new_measurements.get("odom") is not None:
                raise ValueError("Step zero must not contain odometry.")
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("Missing odometry transition.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]),
            )
            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(k - 1, k, relative, self.odom_noise)
            )
            values.insert(k, self.pose.compose(relative))
            self.isam.update(graph, values)

        self.pose = self.isam.calculateEstimatePose2(k)
        covariance = np.asarray(self.isam.marginalCovariance(k), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k
        return (
            [self.pose.x(), self.pose.y(), self.pose.theta()],
            covariance.tolist(),
        )


def _wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def _compose(pose, relative):
    c, s = np.cos(pose[:, 2]), np.sin(pose[:, 2])
    result = np.empty_like(pose)
    result[:, 0] = pose[:, 0] + c * relative[:, 0] - s * relative[:, 1]
    result[:, 1] = pose[:, 1] + s * relative[:, 0] + c * relative[:, 1]
    result[:, 2] = _wrap(pose[:, 2] + relative[:, 2])
    return result


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _point_segment_distance_squared(point, start, end):
    delta = end - start
    denominator = np.sum(delta * delta, axis=-1)
    numerator = np.sum((point - start) * delta, axis=-1)
    fraction = np.divide(
        numerator,
        denominator,
        out=np.zeros_like(numerator, dtype=float),
        where=denominator > 0.0,
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    difference = point - (start + fraction[..., None] * delta)
    return np.sum(difference * difference, axis=-1)


def _collisions(previous, current, walls, radius):
    collided = np.zeros(len(current), dtype=bool)
    motion = current - previous
    for wall in walls:
        a, b = wall[:2], wall[2:]
        edge = b - a
        d2 = np.minimum(
            _point_segment_distance_squared(previous, a, b),
            _point_segment_distance_squared(current, a, b),
        )
        d2 = np.minimum(d2, _point_segment_distance_squared(a, previous, current))
        d2 = np.minimum(d2, _point_segment_distance_squared(b, previous, current))

        denominator = _cross(motion, edge)
        offset = a - previous
        nonparallel = np.abs(denominator) > 1e-14
        t = np.divide(
            _cross(offset, edge),
            denominator,
            out=np.zeros_like(denominator),
            where=nonparallel,
        )
        u = np.divide(
            _cross(offset, motion),
            denominator,
            out=np.zeros_like(denominator),
            where=nonparallel,
        )
        crossing = nonparallel & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
        collided |= crossing | (d2 <= radius * radius)
    return collided


def _prepare_path(waypoints):
    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("A route must contain two-dimensional waypoints.")
    if len(points) > 1:
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 0)
        )
        points = points[keep]
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    return points, delta, lengths, cumulative


def _pursuit_target(position, progress, path, lookahead):
    points, delta, lengths, cumulative = path
    if len(lengths) == 0:
        return np.broadcast_to(points[-1], position.shape), progress

    difference = position[:, None, :] - points[None, :-1, :]
    fraction = np.sum(difference * delta[None, :, :], axis=2)
    fraction = np.clip(fraction / lengths[None, :] ** 2, 0.0, 1.0)
    projection = points[None, :-1, :] + fraction[:, :, None] * delta
    distance2 = np.sum((position[:, None, :] - projection) ** 2, axis=2)
    indices = np.argmin(distance2, axis=1)
    rows = np.arange(len(position))
    projected = cumulative[indices] + fraction[rows, indices] * lengths[indices]
    progress = np.maximum(progress, projected)

    target_progress = np.minimum(progress + lookahead, cumulative[-1])
    segment = np.searchsorted(cumulative, target_progress, side="right") - 1
    segment = np.clip(segment, 0, len(lengths) - 1)
    alpha = (target_progress - cumulative[segment]) / lengths[segment]
    return points[segment] + alpha[:, None] * delta[segment], progress


def _assess_route(spec, waypoints, samples=4096):
    """Assess both terminal-control interpretations of public pure pursuit.

    The public specification does not fully define terminal controller behavior.
    Continuing pursuit after estimated arrival can generate repeated passes
    through the goal and substantially overstate success. We also evaluate
    termination at estimated arrival, using exactly the same sample paths.
    The reported estimate averages these two models, rather than treating
    repeated attempts as certain to be available.
    """
    rng = np.random.default_rng(47129)
    dt = float(spec["dt"])
    steps = int(spec["max_steps"])
    controller = spec["controller"]
    speed = float(controller["v"])
    omega_limit = float(controller["omega_max"])
    lookahead = float(controller["lookahead"])
    radius = float(spec["robot_radius"])
    tolerance = float(spec["arrival_tolerance"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)

    initial = spec["initial_pose"]
    mean = np.asarray(initial["mean"], dtype=float)
    initial_std = np.sqrt(np.asarray(initial["cov_diag_variances"], dtype=float))
    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    actuator_v = float(spec["actuator_noise_std"]["v"])
    actuator_omega = float(spec["actuator_noise_std"]["omega"])

    path = _prepare_path(waypoints)
    goal = path[0][-1]
    length = float(path[3][-1])
    if dt <= 0 or steps < 1:
        return {
            "probability": 0.0, "length": length, "error": 0.0,
            "terminal": 0.0, "continued": 0.0,
        }

    estimate = np.tile(mean, (samples, 1))
    truth = mean + rng.normal(size=(samples, 3)) * initial_std
    truth[:, 2] = _wrap(truth[:, 2])
    progress = np.zeros(samples)

    safe = ~_collisions(truth[:, :2], truth[:, :2], walls, radius)
    reached = np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    continued_success = safe & reached
    terminal_success = continued_success.copy()
    alive = safe & ~reached

    # Eligibility for success under the estimated-arrival stopping model.
    terminal_active = alive & (
        np.linalg.norm(estimate[:, :2] - goal, axis=1) > tolerance
    )

    for _ in range(1, steps):
        if not np.any(alive):
            break

        target, progress = _pursuit_target(
            estimate[:, :2], progress, path, lookahead
        )
        displacement = target - estimate[:, :2]
        distance = np.linalg.norm(displacement, axis=1)
        alpha = _wrap(
            np.arctan2(displacement[:, 1], displacement[:, 0]) - estimate[:, 2]
        )
        omega = np.clip(
            2 * speed * np.sin(alpha) / np.maximum(distance, 1e-12),
            -omega_limit,
            omega_limit,
        )

        v_true = speed + actuator_v * rng.normal(size=samples)
        omega_true = omega + actuator_omega * rng.normal(size=samples)
        turn = omega_true * dt
        travel = v_true * dt * np.sinc(turn / (2 * np.pi))
        relative = np.column_stack(
            (travel * np.cos(turn / 2), travel * np.sin(turn / 2), turn)
        )
        next_truth = _compose(truth, relative)

        # GTSAM's odometry-only MAP mean is exactly pose composition.
        # Pose2 local translational coordinates rotate with the relative pose.
        noise = rng.normal(size=(samples, 3)) * odom_std
        measured_relative = _compose(relative, noise)
        estimate = _compose(estimate, measured_relative)

        hit = _collisions(truth[:, :2], next_truth[:, :2], walls, radius)
        alive &= ~hit
        terminal_active &= ~hit
        truth = next_truth

        reached = np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
        continued_success |= alive & reached
        terminal_success |= terminal_active & reached
        alive &= ~reached
        terminal_active &= ~reached

        estimated_arrival = (
            np.linalg.norm(estimate[:, :2] - goal, axis=1) <= tolerance
        )
        terminal_active &= ~estimated_arrival

    # Paired model averaging: include sample correlation in the MC error.
    scores = 0.5 * (
        continued_success.astype(float) + terminal_success.astype(float)
    )
    return {
        "probability": float(np.mean(scores)),
        "length": length,
        "error": float(np.std(scores, ddof=1) / math.sqrt(samples)),
        "terminal": float(np.mean(terminal_success)),
        "continued": float(np.mean(continued_success)),
    }


def choose_route(public_spec):
    if public_spec["gps_schedule"] is not None or public_spec["landmarks"]:
        raise ValueError("Route assessment is for the odometry-only scenario.")

    assessments = {
        name: _assess_route(public_spec, waypoints)
        for name, waypoints in public_spec["routes"].items()
    }
    best = max(assessments, key=lambda name: assessments[name]["probability"])
    best_result = assessments[best]

    candidates = [
        name
        for name, result in assessments.items()
        if best_result["probability"] - result["probability"]
        <= 2 * math.hypot(best_result["error"], result["error"])
    ]
    selected = min(
        candidates,
        key=lambda name: (
            assessments[name]["length"],
            -assessments[name]["probability"],
            name,
        ),
    )
    result = assessments[selected]
    comparison = ", ".join(
        f"{name}: {assessment['probability']:.3f}"
        for name, assessment in sorted(assessments.items())
    )
    return {
        "route": selected,
        "success_prob": result["probability"],
        "reasoning": (
            f"Choose {selected}; route length {result['length']:.2f} m. "
            f"Model-averaged collision-free arrival estimates: {comparison}. "
            "Deterministic rollouts use the same odometry-composition mean "
            "as the GTSAM estimator, with initial, actuator and odometry "
            "uncertainty, finite wall segments, robot radius and step budget. "
            "Without absolute observations, endpoint drift remains important "
            "even when the route avoids the narrow passage. "
            "Terminal pure-pursuit behavior is not completely specified: "
            f"estimated-arrival stopping gives {result['terminal']:.3f}, "
            f"whereas continued pursuit gives {result['continued']:.3f}. "
            "The reported probability averages these interpretations instead "
            "of assuming repeated goal approaches are always available. "
            f"Sampling error is about {result['error']:.3f}; terminal-control "
            "and simulator-model uncertainty are larger."
        ),
    }
