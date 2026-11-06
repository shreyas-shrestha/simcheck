"""GTSAM dead-reckoning estimator and uncertainty-aware route selection."""

import copy
import math

import gtsam
import numpy as np
from scipy.special import ndtr


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)

        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))
        variances = np.asarray(initial["cov_diag_variances"], dtype=float)
        odom_sigmas = np.asarray(
            self.spec["sensor_noise_std"]["odom"], dtype=float
        )

        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        self.isam = gtsam.ISAM2()

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(
            gtsam.PriorFactorPose2(
                0,
                self.pose,
                gtsam.noiseModel.Diagonal.Variances(variances),
            )
        )
        values.insert(0, self.pose)
        self.isam.update(graph, values)

        self.last_step = -1
        self.covariance = np.diag(variances)

        # Only measured translation is converted. Noise is already in SI.
        unit = self.spec["input_units"]["odom"].lower()
        if unit == "m":
            self.translation_scale = 1.0
        elif unit == "cm":
            self.translation_scale = 0.01
        elif unit == "mm":
            self.translation_scale = 0.001
        else:
            raise ValueError("Unsupported odometry translation unit: " + unit)

        if self.spec["odom_frame"] != "body":
            raise ValueError("This estimator requires body-frame odometry.")

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must be supplied once, in step order.")

        # This scenario has no absolute observations. Do not silently discard
        # observations if a different scenario is accidentally supplied.
        if new_measurements.get("gps") is not None:
            raise ValueError("GPS is not part of this scenario.")
        if new_measurements.get("landmarks"):
            raise ValueError("Landmarks are not part of this scenario.")

        if k == 0:
            if new_measurements.get("odom") is not None:
                raise ValueError("Step zero must not contain odometry.")
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("An odometry transition is required.")

            relative_pose = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]),
            )
            prediction = self.pose.compose(relative_pose)

            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(
                    k - 1, k, relative_pose, self.odom_noise
                )
            )
            values.insert(k, prediction)
            self.isam.update(graph, values)

        self.pose = self.isam.calculateEstimatePose2(k)
        covariance = np.asarray(self.isam.marginalCovariance(k), dtype=float)
        self.covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k

        # marginalCovariance is already in this Pose2's local tangent frame.
        return (
            [float(self.pose.x()), float(self.pose.y()), float(self.pose.theta())],
            self.covariance.tolist(),
        )


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _nearest_on_segment(point, a, b):
    direction = b - a
    squared_length = float(direction @ direction)
    if squared_length == 0.0:
        return a
    fraction = float((point - a) @ direction) / squared_length
    return a + np.clip(fraction, 0.0, 1.0) * direction


def _route_assessment(spec, waypoints):
    """Approximate route risk, not a calibrated closed-loop success model.

    Propagate the same local-tangent uncertainty as the odometry factor chain.
    Use Gaussian separating-plane bounds at nominal travel samples. Summing
    the bounds is deliberately conservative about temporal correlation.
    """
    points = np.asarray(waypoints, dtype=float)
    dt = float(spec["dt"])
    speed = float(spec["controller"]["v"])
    omega_max = float(spec["controller"]["omega_max"])
    available_time = (int(spec["max_steps"]) - 1) * dt
    radius = float(spec["robot_radius"])

    initial = spec["initial_pose"]
    pose = gtsam.Pose2(*map(float, initial["mean"]))
    covariance = np.diag(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    noise_covariance = np.diag(
        np.square(np.asarray(spec["sensor_noise_std"]["odom"], dtype=float))
    )
    walls = [
        (np.asarray(w[:2], dtype=float), np.asarray(w[2:], dtype=float))
        for w in spec["walls"]
    ]

    if speed <= 0.0 or dt <= 0.0:
        return 0.0, float("inf"), float("inf")

    length = 0.0
    turn_time = 0.0
    risk_sum = 0.0

    def collision_bound(current_pose, local_covariance):
        position = np.array([current_pose.x(), current_pose.y()])
        theta = current_pose.theta()
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        world_covariance = (
            rotation @ local_covariance[:2, :2] @ rotation.T
        )
        total = 0.0
        for a, b in walls:
            closest = _nearest_on_segment(position, a, b)
            displacement = position - closest
            distance = float(np.linalg.norm(displacement))
            clearance = distance - radius

            if clearance <= 0.0:
                total += 1.0
                continue

            normal = displacement / distance
            variance = max(float(normal @ world_covariance @ normal), 0.0)
            if variance > 0.0:
                total += float(ndtr(-clearance / math.sqrt(variance)))
        return total

    risk_sum += collision_bound(pose, covariance)

    # Travel from the specified initial mean through the supplied waypoints.
    for target in points:
        start = np.array([pose.x(), pose.y()])
        displacement = target - start
        distance = float(np.linalg.norm(displacement))
        if distance <= np.finfo(float).eps:
            continue

        heading = math.atan2(displacement[1], displacement[0])
        turn = abs(_wrap(heading - pose.theta()))
        if turn > 0.0:
            if omega_max <= 0.0:
                return 0.0, float("inf"), float("inf")
            duration = turn / omega_max
            turn_time += duration
        else:
            duration = 0.0

        # A nominal heading change transports the existing local covariance.
        rotated_pose = gtsam.Pose2(pose.x(), pose.y(), heading)
        relative = pose.between(rotated_pose)
        adjoint = np.asarray(relative.inverse().AdjointMap())
        covariance = (
            adjoint @ covariance @ adjoint.T
            + (duration / dt) * noise_covariance
        )
        pose = rotated_pose

        count = max(1, int(math.ceil(distance / (speed * dt))))
        increment = distance / count
        transition = gtsam.Pose2(increment, 0.0, 0.0)
        adjoint = np.asarray(transition.inverse().AdjointMap())
        noise_fraction = increment / (speed * dt)

        for _ in range(count):
            covariance = (
                adjoint @ covariance @ adjoint.T
                + noise_fraction * noise_covariance
            )
            pose = pose.compose(transition)
            risk_sum += collision_bound(pose, covariance)

        length += distance

    # Turning in place overestimates nominal time for a moving pure-pursuit
    # controller, providing a useful feasibility screen.
    travel_time = length / speed + turn_time
    probability = float(np.clip(1.0 - risk_sum, 0.0, 1.0))
    if travel_time > available_time:
        probability = 0.0
    return probability, length, travel_time


def choose_route(public_spec):
    assessments = {
        name: _route_assessment(public_spec, waypoints)
        for name, waypoints in public_spec["routes"].items()
    }
    # Favor estimated safety; use distance to resolve equal safety scores.
    selected = max(
        assessments,
        key=lambda name: (
            assessments[name][0],
            -assessments[name][1],
            name,
        ),
    )
    probability, length, travel_time = assessments[selected]

    return {
        "route": selected,
        "success_prob": float(probability),
        "reasoning": (
            f"Choose {selected}: nominal length {length:.2f} m and "
            f"conservative travel-time estimate {travel_time:.1f} s. "
            "Without GPS or landmarks, this estimator integrates odometry "
            "and cannot correct heading or lateral drift. The detour avoids "
            "the narrow wall corridor, whose clearance is small compared "
            "with accumulated positional uncertainty. The reported probability "
            "is a nominal-path Gaussian clearance approximation using the "
            "estimator's tangent covariance, not a calibrated closed-loop "
            "Monte Carlo probability; pure-pursuit corner cutting, between-"
            "sample collisions, and true endpoint error are not modeled."
        ),
    }
