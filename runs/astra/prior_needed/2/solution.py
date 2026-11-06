"""GTSAM odometry estimator and deterministic closed-loop route evaluation."""

import copy
import math

import gtsam
import numpy as np


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    """Prior plus Pose2 odometry factors; covariances use local coordinates."""

    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        )
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPose2(0, self.pose, prior_noise))
        values.insert(0, self.pose)
        self.isam.update(graph, values)

        self.last_step = -1
        self.last_result = None
        units = self.spec["input_units"]
        self.translation_scale = {
            "m": 1.0, "cm": 0.01, "mm": 0.001
        }[units["odom"]]
        self.angle_scale = {
            "rad": 1.0, "deg": math.pi / 180.0
        }[units["bearing"]]

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k == self.last_step:
            return tuple(a.copy() for a in self.last_result)
        if k != self.last_step + 1:
            raise ValueError("Expected consecutive measurements starting at step 0.")
        if new_measurements.get("gps") is not None:
            raise ValueError("This scenario specifies no GPS.")
        if new_measurements.get("landmarks"):
            raise ValueError("This scenario specifies no landmarks.")

        odom = new_measurements.get("odom")
        if k == 0:
            if odom is not None:
                raise ValueError("Step 0 must have null odometry.")
        else:
            if odom is None:
                raise ValueError("Missing odometry transition.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]) * self.angle_scale,
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
        covariance = (covariance + covariance.T) * 0.5
        pose = np.array(
            [self.pose.x(), self.pose.y(), self.pose.theta()], dtype=float
        )
        self.last_step = k
        self.last_result = (pose.copy(), covariance.copy())
        return pose, covariance


def _point_segment_distance_squared(point, start, end):
    delta = end - start
    denominator = np.sum(delta * delta, axis=-1)
    numerator = np.sum((point - start) * delta, axis=-1)
    fraction = np.clip(
        numerator / np.maximum(denominator, np.finfo(float).tiny), 0.0, 1.0
    )
    difference = point - (start + fraction[..., None] * delta)
    return np.sum(difference * difference, axis=-1)


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _swept_collision(start, end, walls, radius):
    collision = np.zeros(len(start), dtype=bool)
    for wall in walls:
        a, b = wall[:2], wall[2:]
        motion, barrier = end - start, b - a
        c1 = _cross(motion, a - start)
        c2 = _cross(motion, b - start)
        c3 = _cross(barrier, start - a)
        c4 = _cross(barrier, end - a)
        box_overlap = np.all(
            np.maximum(np.minimum(start, end), np.minimum(a, b))
            <= np.minimum(np.maximum(start, end), np.maximum(a, b)),
            axis=1,
        )
        intersects = (c1 * c2 <= 0.0) & (c3 * c4 <= 0.0) & box_overlap
        distance_squared = np.minimum.reduce([
            _point_segment_distance_squared(start, a, b),
            _point_segment_distance_squared(end, a, b),
            _point_segment_distance_squared(a, start, end),
            _point_segment_distance_squared(b, start, end),
        ])
        collision |= intersects | (distance_squared <= radius * radius)
    return collision


def _route_probability(spec, waypoints, trials):
    """Simulate feedback using the exact mean of the odometry-only graph.

    Arrival is tested on truth. An estimated arrival is neither a true arrival
    nor an episode-termination condition in the supplied success definition.
    In particular, it must not discard trials that can arrive later.
    """
    rng = np.random.default_rng(1729)
    route = np.asarray(waypoints, dtype=float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) == 0:
        raise ValueError("Expected a nonempty list of 2D waypoints.")
    keep = np.r_[True, np.linalg.norm(np.diff(route, axis=0), axis=1) > 0.0]
    route = route[keep]
    segments = np.diff(route, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    total_length = float(cumulative[-1])

    initial = spec["initial_pose"]
    mean = np.asarray(initial["mean"], dtype=float)
    initial_std = np.sqrt(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    truth = mean + rng.standard_normal((trials, 3)) * initial_std
    truth[:, 2] = _wrap(truth[:, 2])
    estimate = np.tile(mean, (trials, 1))

    dt = float(spec["dt"])
    controller = spec["controller"]
    if controller["type"] != "pure_pursuit":
        raise ValueError("Expected pure_pursuit.")
    speed = float(controller["v"])
    lookahead = float(controller["lookahead"])
    omega_max = float(controller["omega_max"])
    actuator = spec["actuator_noise_std"]
    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    tolerance = float(spec["arrival_tolerance"])
    radius = float(spec["robot_radius"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)
    goal = route[-1]

    collided = _swept_collision(truth[:, :2], truth[:, :2], walls, radius)
    success = (
        np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    ) & ~collided
    active = ~collided & ~success
    progress = np.zeros(trials)
    rows = np.arange(trials)

    for _ in range(max(0, int(spec["max_steps"]) - 1)):
        if not np.any(active):
            break

        if len(lengths):
            displacement = estimate[:, None, :2] - route[None, :-1, :]
            fraction = np.clip(
                np.sum(displacement * segments[None, :, :], axis=2)
                / lengths[None, :] ** 2,
                0.0,
                1.0,
            )
            projection = (
                route[None, :-1, :]
                + fraction[:, :, None] * segments[None, :, :]
            )
            distance_squared = np.sum(
                (estimate[:, None, :2] - projection) ** 2, axis=2
            )
            closest = np.argmin(distance_squared, axis=1)
            projected_progress = (
                cumulative[closest]
                + fraction[rows, closest] * lengths[closest]
            )
            progress = np.maximum(progress, projected_progress)
            target_progress = np.minimum(progress + lookahead, total_length)
            target_segment = np.clip(
                np.searchsorted(cumulative, target_progress, side="right") - 1,
                0,
                len(lengths) - 1,
            )
            target_fraction = (
                target_progress - cumulative[target_segment]
            ) / lengths[target_segment]
            target = (
                route[target_segment]
                + target_fraction[:, None] * segments[target_segment]
            )
        else:
            target = np.broadcast_to(goal, (trials, 2))

        toward = target - estimate[:, :2]
        squared_distance = np.sum(toward * toward, axis=1)
        lateral = (
            -np.sin(estimate[:, 2]) * toward[:, 0]
            + np.cos(estimate[:, 2]) * toward[:, 1]
        )
        commanded_omega = np.clip(
            2.0 * speed * lateral / np.maximum(squared_distance, 1e-12),
            -omega_max,
            omega_max,
        )
        realized_v = speed + float(actuator["v"]) * rng.standard_normal(trials)
        realized_omega = (
            commanded_omega
            + float(actuator["omega"]) * rng.standard_normal(trials)
        )
        turn = realized_omega * dt

        # Constant-twist SE(2) integration of the realized actuator motion.
        forward = realized_v * dt * np.sinc(turn / np.pi)
        sideways = (
            realized_v * dt * (0.5 * turn)
            * np.sinc(turn / (2.0 * np.pi)) ** 2
        )
        ct, st = np.cos(truth[:, 2]), np.sin(truth[:, 2])
        next_truth = truth.copy()
        next_truth[:, 0] += ct * forward - st * sideways
        next_truth[:, 1] += st * forward + ct * sideways
        next_truth[:, 2] = _wrap(truth[:, 2] + turn)
        next_truth[~active] = truth[~active]

        # Pose2 right-local measurement perturbation. Noise is already SI.
        noise = rng.standard_normal((trials, 3)) * odom_std
        cr, sr = np.cos(turn), np.sin(turn)
        measured_x = forward + cr * noise[:, 0] - sr * noise[:, 1]
        measured_y = sideways + sr * noise[:, 0] + cr * noise[:, 1]
        measured_theta = turn + noise[:, 2]

        # This composition equals the GTSAM chain's zero-residual MAP estimate.
        ce, se = np.cos(estimate[:, 2]), np.sin(estimate[:, 2])
        next_estimate = estimate.copy()
        next_estimate[:, 0] += ce * measured_x - se * measured_y
        next_estimate[:, 1] += se * measured_x + ce * measured_y
        next_estimate[:, 2] = _wrap(estimate[:, 2] + measured_theta)
        next_estimate[~active] = estimate[~active]

        collided = _swept_collision(
            truth[:, :2], next_truth[:, :2], walls, radius
        ) & active
        reached = (
            np.linalg.norm(next_truth[:, :2] - goal, axis=1) <= tolerance
        ) & active & ~collided
        success |= reached
        active &= ~collided & ~reached
        truth, estimate = next_truth, next_estimate

    return float(np.mean(success)), total_length


def choose_route(public_spec):
    spec = copy.deepcopy(public_spec)
    trials = 4096
    results = {
        name: _route_probability(spec, points, trials)
        for name, points in spec["routes"].items()
    }
    best_probability = max(p for p, _ in results.values())
    standard_error = math.sqrt(
        best_probability * (1.0 - best_probability) / trials
    )
    candidates = [
        name for name, (probability, _) in results.items()
        if best_probability - probability <= 2.0 * standard_error
    ]
    chosen = min(candidates, key=lambda name: (results[name][1], name))
    summary = "; ".join(
        f"{name}: success approximately {p:.3f}, length {length:.2f} m"
        for name, (p, length) in sorted(results.items())
    )
    return {
        "route": chosen,
        "success_prob": results[chosen][0],
        "reasoning": (
            "Deterministic closed-loop Monte Carlo using the odometry-only "
            "GTSAM estimator's mean, configured pure pursuit, and initial, "
            "actuator, and odometry uncertainties. Collision checks include "
            "robot radius and swept motion. Arrival is checked on true position "
            "throughout the time budget: estimated proximity alone does not "
            "terminate a trial. Without absolute observations, heading drift "
            "makes the narrow corridor risky. Distance breaks statistically "
            "close ties. " + summary + ". Probabilities are approximate because "
            "the detailed pure-pursuit implementation is not specified."
        ),
    }
