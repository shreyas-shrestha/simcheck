"""GTSAM localization and specification-driven route selection.

public_spec is the decoded contents of spec.json. All physical parameters are
taken from it; no episode data or estimator state is shared between instances.
"""

import math

import gtsam
import numpy as np
from scipy.special import ndtr


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        noise = public_spec["sensor_noise_std"]
        initial = public_spec["initial_pose"]

        self.initial_pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(noise["odom"], dtype=float)
        )
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(2, float(noise["gps"]))
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([noise["bearing"], noise["range"]], dtype=float)
        )
        self.landmarks = {
            str(key): np.asarray(point, dtype=float)
            for key, point in public_spec["landmarks"].items()
        }

        units = public_spec["input_units"]
        self.translation_scale = {"m": 1.0, "mm": 0.001}[units["odom"]]
        self.bearing_scale = {"rad": 1.0, "deg": math.pi / 180.0}[
            units["bearing"]
        ]
        if public_spec["odom_frame"] != "body":
            raise ValueError("This estimator requires body-frame odometry.")

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-5)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)

        self.last_step = -1
        self.last_pose = None
        # Explicitly retain Python callbacks for the lifetime of their factors.
        self.callbacks = []

    def _gps_factor(self, key, observation):
        target = np.array(
            [float(observation["x"]), float(observation["y"])], dtype=float
        )

        def error_function(factor, values, jacobians):
            pose = values.atPose2(key)
            if jacobians is not None:
                c, s = math.cos(pose.theta()), math.sin(pose.theta())
                # Pose2 translation increments are in the local tangent frame.
                jacobians[0] = np.array(
                    [[c, -s, 0.0], [s, c, 0.0]], order="F"
                )
            return np.array([pose.x(), pose.y()]) - target

        self.callbacks.append(error_function)
        return gtsam.CustomFactor(self.gps_noise, [key], error_function)

    def _landmark_factor(self, key, observation):
        point = self.landmarks[str(observation["id"])]
        bearing = float(observation["bearing"]) * self.bearing_scale
        measured_range = float(observation["range"])

        def error_function(factor, values, jacobians):
            pose = values.atPose2(key)
            c, s = math.cos(pose.theta()), math.sin(pose.theta())
            dx, dy = point[0] - pose.x(), point[1] - pose.y()
            qx, qy = c * dx + s * dy, -s * dx + c * dy
            squared_range = max(qx * qx + qy * qy, 1e-24)
            distance = math.sqrt(squared_range)
            angle_error = math.atan2(qy, qx) - bearing
            angle_error = math.atan2(
                math.sin(angle_error), math.cos(angle_error)
            )

            if jacobians is not None:
                jacobians[0] = np.array(
                    [
                        [qy / squared_range, -qx / squared_range, -1.0],
                        [-qx / distance, -qy / distance, 0.0],
                    ],
                    dtype=float,
                    order="F",
                )
            return np.array([angle_error, distance - measured_range])

        self.callbacks.append(error_function)
        return gtsam.CustomFactor(self.landmark_noise, [key], error_function)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in step order.")

        key = gtsam.symbol("x", k)
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        if k == 0:
            prediction = self.initial_pose
            graph.add(
                gtsam.PriorFactorPose2(key, self.initial_pose, self.prior_noise)
            )
        else:
            observation = new_measurements["odom"]
            if observation is None:
                raise ValueError("A transition requires an odometry measurement.")
            # Only dx/dy use the odometry distance unit. dtheta is radians;
            # the bearing unit applies only to landmark bearings.
            relative_pose = gtsam.Pose2(
                float(observation["dx"]) * self.translation_scale,
                float(observation["dy"]) * self.translation_scale,
                float(observation["dtheta"]),
            )
            prediction = self.last_pose.compose(relative_pose)
            graph.add(
                gtsam.BetweenFactorPose2(
                    gtsam.symbol("x", k - 1),
                    key,
                    relative_pose,
                    self.odom_noise,
                )
            )

        values.insert(key, prediction)

        if new_measurements.get("gps") is not None:
            graph.add(self._gps_factor(key, new_measurements["gps"]))

        for observation in new_measurements.get("landmarks", []):
            graph.add(self._landmark_factor(key, observation))

        self.isam.update(graph, values)
        # Additional nonlinear relinearization passes, without adding data.
        self.isam.update()
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


def _route_assessment(spec, waypoints):
    """Approximate closed-loop risk, not a calibrated Monte Carlo probability.

    Propagate nominal localization covariance along the route, incorporating
    the stated GPS schedule and landmark visibility. Add a pure-pursuit
    tracking-error allowance, then use a union approximation for wall risk.
    """
    points = np.asarray(waypoints, dtype=float)
    vectors = np.diff(points, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    keep = lengths > 1e-12
    vectors, lengths = vectors[keep], lengths[keep]
    starts = points[:-1][keep]
    if len(lengths) == 0:
        return 1.0, 0.0, math.inf

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    length = float(cumulative[-1])
    headings = np.arctan2(vectors[:, 1], vectors[:, 0])

    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    lookahead = float(controller["lookahead"])
    omega_max = float(controller["omega_max"])
    max_steps = int(spec["max_steps"])
    tolerance = float(spec["arrival_tolerance"])
    radius = float(spec["robot_radius"])

    sensor = spec["sensor_noise_std"]
    actuator = spec["actuator_noise_std"]
    odom_variances = np.square(np.asarray(sensor["odom"], dtype=float))
    gps_variance = float(sensor["gps"]) ** 2
    bearing_variance = float(sensor["bearing"]) ** 2
    range_variance = float(sensor["range"]) ** 2

    landmarks = [
        np.asarray(point, dtype=float) for point in spec["landmarks"].values()
    ]
    visibility = float(spec["landmark_visibility"]["max_range"])
    walls = [np.asarray(wall, dtype=float).reshape(2, 2) for wall in spec["walls"]]
    schedule = spec["gps_schedule"]
    gps_rate = int(schedule["rate_steps"])
    dropout = schedule["dropout_steps"]

    def gps_available(k):
        if gps_rate <= 0 or k % gps_rate != 0:
            return False
        # The two endpoints describe the inclusive blackout interval.
        if len(dropout) == 2:
            return not (int(dropout[0]) <= k <= int(dropout[1]))
        return k not in dropout

    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    turn_angle = abs(
        wrap(float(headings[0]) - float(spec["initial_pose"]["mean"][2]))
    )
    turn_angle += sum(
        abs(wrap(float(b - a))) for a, b in zip(headings[:-1], headings[1:])
    )
    # Conservative time allowance for heading acquisition and corners.
    turn_time = turn_angle / max(omega_max, 1e-12)
    available_time = max_steps * dt - turn_time
    needed_distance = max(0.0, length - tolerance)
    distance_std = float(actuator["v"]) * dt * math.sqrt(max_steps)
    time_probability = float(
        ndtr(
            (speed * available_time - needed_distance)
            / max(distance_std, 1e-12)
        )
    )

    theta0 = float(spec["initial_pose"]["mean"][2])
    c, s = math.cos(theta0), math.sin(theta0)
    rotation = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    covariance = rotation @ np.diag(
        spec["initial_pose"]["cov_diag_variances"]
    ) @ rotation.T

    increment = max(speed * dt, 1e-12)
    count = min(max_steps, int(math.ceil(needed_distance / increment)))
    previous_position = starts[0].copy()
    collision_union = 0.0
    minimum_clearance = math.inf

    for k in range(count + 1):
        distance = min(k * increment, needed_distance)
        segment = min(
            len(lengths) - 1,
            int(np.searchsorted(cumulative[1:], distance, side="right")),
        )
        direction = vectors[segment] / lengths[segment]
        position = starts[segment] + (
            distance - cumulative[segment]
        ) * direction

        if k > 0:
            displacement = position - previous_position
            transition = np.eye(3)
            transition[0, 2] = -displacement[1]
            transition[1, 2] = displacement[0]
            c, s = direction
            rotation = np.array(
                [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
            )
            covariance = (
                transition @ covariance @ transition.T
                + rotation @ np.diag(odom_variances) @ rotation.T
            )
        previous_position = position

        information = np.linalg.inv(covariance)
        if gps_available(k):
            information[0, 0] += 1.0 / gps_variance
            information[1, 1] += 1.0 / gps_variance

        for landmark in landmarks:
            delta = landmark - position
            r2 = float(delta @ delta)
            if 1e-12 < r2 <= visibility * visibility:
                r = math.sqrt(r2)
                bearing_jacobian = np.array(
                    [delta[1] / r2, -delta[0] / r2, -1.0]
                )
                range_jacobian = np.array(
                    [-delta[0] / r, -delta[1] / r, 0.0]
                )
                information += (
                    np.outer(bearing_jacobian, bearing_jacobian)
                    / bearing_variance
                    + np.outer(range_jacobian, range_jacobian) / range_variance
                )
        covariance = np.linalg.inv(information)

        # Pure pursuit corrects cross-track errors over roughly L/v seconds.
        response_time = lookahead / max(speed, 1e-12)
        tracking_variance = (
            float(actuator["v"]) ** 2
            + lookahead**2 * float(actuator["omega"]) ** 2
        ) * dt * response_time / 2.0
        tracking_variance += lookahead**2 * covariance[2, 2]

        for wall in walls:
            edge = wall[1] - wall[0]
            denominator = float(edge @ edge)
            fraction = (
                np.clip(float((position - wall[0]) @ edge) / denominator, 0, 1)
                if denominator > 0
                else 0.0
            )
            delta = position - (wall[0] + fraction * edge)
            separation = float(np.linalg.norm(delta))
            clearance = separation - radius
            minimum_clearance = min(minimum_clearance, clearance)
            if separation > 1e-12:
                normal = delta / separation
                variance = float(normal @ covariance[:2, :2] @ normal)
            else:
                variance = float(np.linalg.eigvalsh(covariance[:2, :2])[-1])
            sigma = math.sqrt(max(variance + tracking_variance, 1e-24))
            collision_union += float(ndtr(-clearance / sigma))

    probability = time_probability * max(0.0, 1.0 - collision_union)
    return float(np.clip(probability, 0.0, 1.0)), length, minimum_clearance


def choose_route(public_spec):
    assessments = {
        name: _route_assessment(public_spec, route)
        for name, route in public_spec["routes"].items()
    }
    best_probability = max(result[0] for result in assessments.values())
    # Treat probabilities within one percentage point as a close tie.
    candidates = [
        name
        for name, result in assessments.items()
        if result[0] >= best_probability - 0.01
    ]
    chosen = min(candidates, key=lambda name: (assessments[name][1], name))
    probability, length, clearance = assessments[chosen]

    clearance_text = (
        f"{clearance:.2f} m" if math.isfinite(clearance) else "unbounded"
    )
    return {
        "route": chosen,
        "success_prob": probability,
        "reasoning": (
            f"Route {chosen} has length {length:.2f} m and nominal "
            f"robot-boundary wall clearance {clearance_text}. Selection uses "
            "the estimator's odometry/GPS/landmark covariance model, scheduled "
            "GPS blackout, landmark visibility, a pure-pursuit tracking-error "
            "allowance, actuator noise, and the arrival time budget. "
            "The probability is an approximate Gaussian risk estimate, not a "
            "calibrated closed-loop simulation result; shorter routes break "
            "close probability ties."
        ),
    }
