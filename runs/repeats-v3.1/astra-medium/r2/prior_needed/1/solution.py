"""GTSAM odometry estimator and deterministic, closed-loop route assessment."""

import copy
import math

import gtsam
import numpy as np


def _unit_scales(spec):
    units = spec["input_units"]
    position_scale = {"m": 1.0, "cm": 0.01, "mm": 0.001}[units["odom"]]
    angle_unit = units.get("angle", units["bearing"])
    angle_scale = {
        "rad": 1.0,
        "radian": 1.0,
        "radians": 1.0,
        "deg": math.pi / 180.0,
        "degree": math.pi / 180.0,
        "degrees": math.pi / 180.0,
    }[angle_unit]
    return position_scale, angle_scale


class Estimator:
    """An anchored Pose2 chain, maintained incrementally using iSAM2.

    No motion-command factors are added: actuator noise is already reflected
    in the measured relative motion, and is not additional odometry noise.
    """

    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        self.position_scale, self.angle_scale = _unit_scales(self.spec)

        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))
        prior_variances = np.asarray(
            initial["cov_diag_variances"], dtype=float
        )
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        )

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(
            gtsam.PriorFactorPose2(
                0,
                self.pose,
                gtsam.noiseModel.Diagonal.Variances(prior_variances),
            )
        )
        values.insert(0, self.pose)
        self.isam.update(graph, values)

        self.last_step = -1
        self.key = 0

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive consecutively, starting at step 0.")

        # This scenario has neither GPS nor landmark observations.
        if new_measurements.get("gps") is not None:
            raise ValueError("GPS is not part of this scenario.")
        if new_measurements.get("landmarks"):
            raise ValueError("Landmarks are not part of this scenario.")

        if k == 0:
            if new_measurements.get("odom") is not None:
                raise ValueError("Step 0 must not contain an odometry transition.")
        else:
            odom = new_measurements["odom"]
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.position_scale,
                float(odom["dy"]) * self.position_scale,
                float(odom["dtheta"]) * self.angle_scale,
            )
            predicted = self.pose.compose(relative)

            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(
                    self.key, k, relative, self.odom_noise
                )
            )
            values.insert(k, predicted)
            self.isam.update(graph, values)
            self.key = k

        self.pose = self.isam.calculateEstimatePose2(self.key)
        covariance = np.asarray(
            self.isam.marginalCovariance(self.key), dtype=float
        )
        covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k

        return (
            [self.pose.x(), self.pose.y(), self.pose.theta()],
            covariance.tolist(),
        )


def make_estimator(public_spec):
    return Estimator(public_spec)


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _point_segment_distance_squared(p, a, b):
    d = b - a
    denominator = np.sum(d * d, axis=-1)
    numerator = np.sum((p - a) * d, axis=-1)
    fraction = np.divide(
        numerator,
        denominator,
        out=np.zeros_like(numerator, dtype=float),
        where=denominator > 0.0,
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    residual = p - (a + fraction[..., None] * d)
    return np.sum(residual * residual, axis=-1)


def _swept_collisions(old_xy, new_xy, walls, radius):
    """Collision of each swept robot disc with any finite wall segment."""
    collided = np.zeros(len(old_xy), dtype=bool)
    motion = new_xy - old_xy

    for wall in walls:
        a = wall[:2]
        b = wall[2:]
        w = b - a

        d2 = np.minimum(
            _point_segment_distance_squared(old_xy, a, b),
            _point_segment_distance_squared(new_xy, a, b),
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(a, old_xy, new_xy)
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(b, old_xy, new_xy)
        )

        s1 = _cross(motion, a - old_xy)
        s2 = _cross(motion, b - old_xy)
        s3 = _cross(w, old_xy - a)
        s4 = _cross(w, new_xy - a)
        boxes_overlap = np.all(
            (np.maximum(old_xy, new_xy) >= np.minimum(a, b))
            & (np.minimum(old_xy, new_xy) <= np.maximum(a, b)),
            axis=1,
        )
        intersects = (
            (s1 * s2 <= 0.0)
            & (s3 * s4 <= 0.0)
            & boxes_overlap
        )
        collided |= intersects | (d2 <= radius * radius)

    return collided


def _route_trials(spec, waypoints, trial_count, seed):
    """Approximate the public controller in a deterministic Monte Carlo model.

    With only a prior and odometry, the GTSAM MAP pose is exactly the product
    of measured Pose2 increments. The vectorized estimate below therefore
    implements the same estimator mean without constructing thousands of
    identical factor-graph topologies.
    """
    rng = np.random.default_rng(seed)
    points = np.asarray(waypoints, dtype=float)
    if len(points) < 2:
        raise ValueError("A route must contain at least two distinct points.")

    # Remove zero-length segments.
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 0.0]
    points = points[keep]
    if len(points) < 2:
        raise ValueError("A route must contain at least two distinct points.")

    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]

    initial = spec["initial_pose"]
    mean = np.asarray(initial["mean"], dtype=float)
    initial_std = np.sqrt(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    truth = mean + rng.normal(size=(trial_count, 3)) * initial_std
    estimate = np.tile(mean, (trial_count, 1))

    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    actuator = spec["actuator_noise_std"]
    controller = spec["controller"]
    dt = float(spec["dt"])
    speed = float(controller["v"])
    lookahead = float(controller["lookahead"])
    omega_max = float(controller["omega_max"])
    radius = float(spec["robot_radius"])
    tolerance = float(spec["arrival_tolerance"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)
    goal = points[-1]

    collided = _swept_collisions(
        truth[:, :2], truth[:, :2], walls, radius
    )
    successful = (
        np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    ) & ~collided
    active = ~(successful | collided)
    progress = np.zeros(trial_count)

    # max_steps is the maximum number of poses, including x_0.
    for _ in range(1, int(spec["max_steps"])):
        if not np.any(active):
            break

        # Project the estimator pose onto the polyline and select a target
        # lookahead metres farther along it.
        offset = estimate[:, None, :2] - points[None, :-1, :]
        fractions = np.clip(
            np.sum(offset * segments[None, :, :], axis=2)
            / (lengths[None, :] ** 2),
            0.0,
            1.0,
        )
        projections = (
            points[None, :-1, :]
            + fractions[:, :, None] * segments[None, :, :]
        )
        distance2 = np.sum(
            (estimate[:, None, :2] - projections) ** 2, axis=2
        )
        nearest = np.argmin(distance2, axis=1)
        row = np.arange(trial_count)
        projected_progress = (
            cumulative[nearest] + fractions[row, nearest] * lengths[nearest]
        )
        progress = np.maximum(progress, projected_progress)
        target_progress = np.minimum(progress + lookahead, cumulative[-1])
        target_segment = np.minimum(
            np.searchsorted(cumulative, target_progress, side="right") - 1,
            len(lengths) - 1,
        )
        target_fraction = (
            target_progress - cumulative[target_segment]
        ) / lengths[target_segment]
        target = (
            points[target_segment]
            + target_fraction[:, None] * segments[target_segment]
        )

        delta = target - estimate[:, :2]
        target_distance2 = np.sum(delta * delta, axis=1)
        heading = estimate[:, 2]
        lateral = (
            -np.sin(heading) * delta[:, 0]
            + np.cos(heading) * delta[:, 1]
        )
        command_omega = np.clip(
            2.0 * speed * lateral / np.maximum(target_distance2, 1e-12),
            -omega_max,
            omega_max,
        )

        velocity = speed + rng.normal(
            0.0, float(actuator["v"]), trial_count
        )
        omega = command_omega + rng.normal(
            0.0, float(actuator["omega"]), trial_count
        )
        angle = omega * dt

        # Exact constant-twist integration in the previous body frame.
        distance = velocity * dt
        dx = distance * np.sinc(angle / np.pi)
        dy = distance * (
            0.5 * angle * np.sinc(angle / (2.0 * np.pi)) ** 2
        )

        old_xy = truth[:, :2].copy()
        c = np.cos(truth[:, 2])
        s = np.sin(truth[:, 2])
        truth[:, 0] += c * dx - s * dy
        truth[:, 1] += s * dx + c * dy
        truth[:, 2] += angle

        hit = _swept_collisions(old_xy, truth[:, :2], walls, radius)
        collided |= active & hit
        arrived = (
            np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
        )
        successful |= active & ~hit & arrived
        active &= ~(hit | arrived)

        # Right-local Pose2 measurement noise, in SI units. Compose the
        # noisy relative pose into the estimated world pose.
        noise = rng.normal(size=(trial_count, 3)) * odom_std
        ca = np.cos(angle)
        sa = np.sin(angle)
        measured_dx = dx + ca * noise[:, 0] - sa * noise[:, 1]
        measured_dy = dy + sa * noise[:, 0] + ca * noise[:, 1]

        c = np.cos(estimate[:, 2])
        s = np.sin(estimate[:, 2])
        estimate[:, 0] += c * measured_dx - s * measured_dy
        estimate[:, 1] += s * measured_dx + c * measured_dy
        estimate[:, 2] += angle + noise[:, 2]

        truth[:, 2] = np.arctan2(
            np.sin(truth[:, 2]), np.cos(truth[:, 2])
        )
        estimate[:, 2] = np.arctan2(
            np.sin(estimate[:, 2]), np.cos(estimate[:, 2])
        )

    return float(np.mean(successful)), float(cumulative[-1])


def choose_route(public_spec):
    # Local fixed seeds ensure reproducibility without shared episode state.
    trial_count = 2048
    assessments = {}
    for name, waypoints in public_spec["routes"].items():
        assessments[name] = _route_trials(
            public_spec, waypoints, trial_count, seed=0
        )

    best = max(assessments, key=lambda name: assessments[name][0])
    best_probability = assessments[best][0]

    # Resolve differences smaller than Monte Carlo sampling resolution by
    # route length, as requested by the public objective.
    close = []
    for name, (probability, length) in assessments.items():
        standard_error = math.sqrt(
            (
                probability * (1.0 - probability)
                + best_probability * (1.0 - best_probability)
            )
            / trial_count
        )
        if best_probability - probability <= standard_error:
            close.append(name)

    selected = min(
        close,
        key=lambda name: (assessments[name][1], -assessments[name][0], name),
    )
    summary = "; ".join(
        f"{name}: estimated success {probability:.3f}, length {length:.2f} m"
        for name, (probability, length) in sorted(assessments.items())
    )

    return {
        "route": selected,
        "success_prob": assessments[selected][0],
        "reasoning": (
            f"{summary}. Selected {selected} using deterministic closed-loop "
            "Monte Carlo with initial uncertainty, actuator noise, tangent-space "
            "odometry noise, finite-radius wall collisions, and the time limit. "
            "The simulated estimator mean equals the odometry-only GTSAM MAP "
            "estimate. Probabilities are approximate because the public "
            "pure-pursuit description does not specify every controller detail; "
            "distance breaks statistically close ties."
        ),
    }
