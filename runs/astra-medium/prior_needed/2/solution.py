"""Odometry-only GTSAM estimation and deterministic closed-loop route assessment."""

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

        variances = np.asarray(initial["cov_diag_variances"], dtype=float)
        sigmas = np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)

        if self.spec["odom_frame"] != "body":
            raise ValueError("Expected body-frame relative-pose odometry.")
        unit = self.spec["input_units"]["odom"].lower()
        scales = {"m": 1.0, "cm": 0.01, "mm": 0.001}
        if unit not in scales:
            raise ValueError("Unsupported odometry translation unit: " + unit)
        self.translation_scale = scales[unit]

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(
            gtsam.PriorFactorPose2(
                0, self.pose, gtsam.noiseModel.Diagonal.Variances(variances)
            )
        )
        values.insert(0, self.pose)
        self.isam.update(graph, values)
        self.last_step = -1

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in step order.")
        if new_measurements.get("gps") is not None:
            raise ValueError("This scenario has no GPS.")
        if new_measurements.get("landmarks"):
            raise ValueError("This scenario has no landmarks.")

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
                gtsam.BetweenFactorPose2(
                    k - 1, k, relative, self.odom_noise
                )
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


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _compose(pose, relative):
    """Vectorized Pose2 composition, with one pose per row."""
    c = np.cos(pose[:, 2])
    s = np.sin(pose[:, 2])
    result = np.empty_like(pose)
    result[:, 0] = pose[:, 0] + c * relative[:, 0] - s * relative[:, 1]
    result[:, 1] = pose[:, 1] + s * relative[:, 0] + c * relative[:, 1]
    result[:, 2] = _wrap(pose[:, 2] + relative[:, 2])
    return result


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _point_segment_distance_squared(point, start, end):
    """Supports broadcasting a fixed point or segment over a batch."""
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
    """Swept-disc collision test against finite wall segments."""
    collided = np.zeros(len(current), dtype=bool)
    motion = current - previous

    for wall in walls:
        a, b = wall[:2], wall[2:]
        edge = b - a
        d2 = np.minimum(
            _point_segment_distance_squared(previous, a, b),
            _point_segment_distance_squared(current, a, b),
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(a, previous, current)
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(b, previous, current)
        )

        # Distances to endpoints alone do not detect crossing segments.
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
        crossing = (
            nonparallel
            & (t >= 0.0)
            & (t <= 1.0)
            & (u >= 0.0)
            & (u <= 1.0)
        )
        collided |= crossing | (d2 <= radius * radius)

    return collided


def _prepare_path(waypoints):
    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("A route must contain two-dimensional waypoints.")
    if len(points) > 1:
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 0.0)
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

    # Project the estimated position onto the polyline, then advance by the
    # lookahead distance. Progress is monotone along the selected route.
    differences = position[:, None, :] - points[None, :-1, :]
    fraction = np.sum(differences * delta[None, :, :], axis=2)
    fraction /= lengths[None, :] ** 2
    fraction = np.clip(fraction, 0.0, 1.0)
    projection = points[None, :-1, :] + fraction[:, :, None] * delta
    distance2 = np.sum(
        (position[:, None, :] - projection) ** 2, axis=2
    )
    indices = np.argmin(distance2, axis=1)
    rows = np.arange(len(position))
    projected_progress = (
        cumulative[indices] + fraction[rows, indices] * lengths[indices]
    )
    progress = np.maximum(progress, projected_progress)
    target_progress = np.minimum(progress + lookahead, cumulative[-1])
    segment = np.searchsorted(cumulative, target_progress, side="right") - 1
    segment = np.clip(segment, 0, len(lengths) - 1)
    alpha = (target_progress - cumulative[segment]) / lengths[segment]
    target = points[segment] + alpha[:, None] * delta[segment]
    return target, progress


def _assess_route(spec, waypoints, samples=4096):
    """Approximate collision-free TRUE arrival using closed-loop rollouts.

    In this observation-free scenario, the GTSAM factor-chain solution is
    exactly the prior mean composed with all measured relative poses. The
    vectorized rollouts use that same estimator mean, not perfect localization.

    Numerical integration and pure-pursuit implementation details can differ
    from the verifier; this is a model-based probability estimate.
    """
    # Local RNG: repeated calls are deterministic, with no persistent state.
    # The same seed for each route supplies common random numbers.
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
    initial_std = np.sqrt(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    actuator_v = float(spec["actuator_noise_std"]["v"])
    actuator_omega = float(spec["actuator_noise_std"]["omega"])

    path = _prepare_path(waypoints)
    goal = path[0][-1]
    length = float(path[3][-1])
    if dt <= 0.0 or steps < 1:
        return 0.0, length, 0.0

    estimate = np.tile(mean, (samples, 1))
    # initial_truth_rule specifies a Gaussian in the supplied pose coordinates.
    truth = mean + rng.normal(size=(samples, 3)) * initial_std
    truth[:, 2] = _wrap(truth[:, 2])
    progress = np.zeros(samples)

    alive = ~_collisions(truth[:, :2], truth[:, :2], walls, radius)
    arrived = alive & (
        np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    )
    alive &= ~arrived

    for _ in range(1, steps):
        if not np.any(alive):
            break

        target, progress = _pursuit_target(
            estimate[:, :2], progress, path, lookahead
        )
        displacement = target - estimate[:, :2]
        distance = np.linalg.norm(displacement, axis=1)
        alpha = _wrap(
            np.arctan2(displacement[:, 1], displacement[:, 0])
            - estimate[:, 2]
        )
        # Geometric pure pursuit: curvature = 2*sin(alpha)/target distance.
        omega = 2.0 * speed * np.sin(alpha) / np.maximum(distance, 1e-12)
        omega = np.clip(omega, -omega_limit, omega_limit)

        v_true = speed + actuator_v * rng.normal(size=samples)
        omega_true = omega + actuator_omega * rng.normal(size=samples)
        turn = omega_true * dt

        # Exact constant-twist integration, including the zero-turn limit.
        travel = v_true * dt * np.sinc(turn / (2.0 * np.pi))
        relative = np.column_stack(
            (
                travel * np.cos(0.5 * turn),
                travel * np.sin(0.5 * turn),
                turn,
            )
        )
        next_truth = _compose(truth, relative)

        # Right-local Pose2 perturbation of the measured relative pose.
        # Noise standard deviations are already SI, independent of input_units.
        noise = rng.normal(size=(samples, 3)) * odom_std
        measured_relative = _compose(relative, noise)
        estimate = _compose(estimate, measured_relative)

        hit = _collisions(
            truth[:, :2], next_truth[:, :2], walls, radius
        )
        alive &= ~hit
        truth = next_truth

        reached = alive & (
            np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
        )
        arrived |= reached
        alive &= ~reached

    probability = float(np.mean(arrived))
    sampling_error = math.sqrt(
        probability * (1.0 - probability) / samples
    )
    return probability, length, sampling_error


def choose_route(public_spec):
    if public_spec["gps_schedule"] is not None or public_spec["landmarks"]:
        raise ValueError("Route assessment is for the odometry-only scenario.")

    assessments = {
        name: _assess_route(public_spec, waypoints)
        for name, waypoints in public_spec["routes"].items()
    }
    best = max(assessments, key=lambda name: assessments[name][0])

    # Treat differences below Monte Carlo resolution as close ties, then use
    # the specified distance tie-breaker.
    candidates = []
    for name, (probability, length, error) in assessments.items():
        best_probability, _, best_error = assessments[best]
        resolution = 2.0 * math.hypot(error, best_error)
        if best_probability - probability <= resolution:
            candidates.append(name)
    selected = min(
        candidates,
        key=lambda name: (assessments[name][1], -assessments[name][0], name),
    )
    probability, length, error = assessments[selected]
    comparison = ", ".join(
        f"{name}: {result[0]:.3f}" for name, result in sorted(assessments.items())
    )

    return {
        "route": selected,
        "success_prob": probability,
        "reasoning": (
            f"Choose {selected} (polyline length {length:.2f} m). "
            f"Deterministic closed-loop rollout estimates are {comparison}. "
            "Rollouts drive pure pursuit using the same odometry-composition "
            "mean as the GTSAM estimator, sample initial, actuator, and "
            "relative-pose measurement uncertainty, and test swept-disc "
            "wall collisions and true arrival within the step budget. "
            "In particular, avoiding walls is not counted as success unless "
            "the true robot reaches the goal: without absolute observations, "
            "endpoint drift remains significant even on the detour. "
            f"Monte Carlo standard error is approximately {error:.3f}; "
            "controller and simulator implementation differences introduce "
            "additional model uncertainty."
        ),
    }
