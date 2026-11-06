"""GTSAM Pose2 localization and uncertainty-aware route selection."""

import math
import numpy as np
import gtsam


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        noise = public_spec["sensor_noise_std"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(noise["odom"], dtype=float)
        )
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(2, float(noise["gps"]))
        # Residual ordering is bearing, range.
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([noise["bearing"], noise["range"]], dtype=float)
        )
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(public_spec["initial_pose"]["cov_diag_variances"], dtype=float)
        )
        self.initial = gtsam.Pose2(*public_spec["initial_pose"]["mean"])
        self.landmarks = {
            str(key): np.asarray(point, dtype=float)
            for key, point in public_spec["landmarks"].items()
        }

        units = public_spec.get("input_units", {})
        self.translation_scale = {
            "m": 1.0, "cm": 0.01, "mm": 0.001
        }[units.get("odom", "m")]
        self.bearing_scale = {
            "rad": 1.0, "deg": math.pi / 180.0
        }[units.get("bearing", "rad")]

        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(1e-4)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.last_step = -1
        self.pose = self.initial
        # Explicitly retain Python callbacks for the lifetime of their factors.
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

    def _landmark_factor(self, key, observation):
        point = self.landmarks[str(observation["id"])]
        bearing = float(observation["bearing"]) * self.bearing_scale
        distance = float(observation["range"])

        def error(factor, values, jacobians):
            pose = values.atPose2(key)
            q = pose.transformTo(point)
            x, y = float(q[0]), float(q[1])
            squared_range = max(x * x + y * y, 1e-24)
            predicted_range = math.sqrt(squared_range)
            angle_error = math.atan2(y, x) - bearing
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            if jacobians is not None:
                # Derivatives with respect to a right/local Pose2 perturbation.
                jacobians[0] = np.array(
                    [
                        [y / squared_range, -x / squared_range, -1.0],
                        [-x / predicted_range, -y / predicted_range, 0.0],
                    ],
                    order="F",
                )
            return np.array([angle_error, predicted_range - distance])

        self.callbacks.append(error)
        return gtsam.CustomFactor(self.landmark_noise, [key], error)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must be supplied once, in consecutive order.")

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        key = gtsam.symbol("x", k)

        if k == 0:
            guess = self.initial
            graph.add(gtsam.PriorFactorPose2(key, guess, self.prior_noise))
        else:
            odom = new_measurements["odom"]
            # Only dx/dy use the odometry length unit; dtheta remains radians.
            delta = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]),
            )
            guess = self.pose.compose(delta)
            graph.add(
                gtsam.BetweenFactorPose2(
                    gtsam.symbol("x", k - 1), key, delta, self.odom_noise
                )
            )

        values.insert(key, guess)
        if new_measurements.get("gps") is not None:
            graph.add(self._gps_factor(key, new_measurements["gps"]))
        for observation in new_measurements.get("landmarks", []):
            graph.add(self._landmark_factor(key, observation))

        self.isam.update(graph, values)
        # Additional nonlinear refinement without counting any measurement twice.
        self.isam.update()
        self.isam.update()
        self.pose = self.isam.calculateEstimatePose2(key)
        covariance = np.asarray(self.isam.marginalCovariance(key), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k

        return (
            [float(self.pose.x()), float(self.pose.y()), float(self.pose.theta())],
            covariance.tolist(),
        )


def choose_route(public_spec):
    """
    Score routes using nominal-path covariance propagation and Gaussian clearance
    risk. This is a deterministic planning approximation, not an empirically
    calibrated closed-loop Monte Carlo probability.
    """
    spec = public_spec
    dt = float(spec["dt"])
    controller = spec["controller"]
    speed = float(controller["v"])
    lookahead = float(controller["lookahead"])
    omega_max = float(controller["omega_max"])
    radius = float(spec["robot_radius"])
    noise = spec["sensor_noise_std"]
    actuator = spec["actuator_noise_std"]
    landmarks = [np.asarray(p, dtype=float) for p in spec["landmarks"].values()]
    walls = [np.asarray(w, dtype=float).reshape(2, 2) for w in spec["walls"]]
    visibility = float(spec["landmark_visibility"]["max_range"])
    schedule = spec["gps_schedule"]
    gps_rate = int(schedule["rate_steps"])
    dropout = schedule["dropout_steps"]
    initial_mean = np.asarray(spec["initial_pose"]["mean"], dtype=float)
    initial_cov = np.diag(
        np.asarray(spec["initial_pose"]["cov_diag_variances"], dtype=float)
    )
    odom_cov = np.diag(np.asarray(noise["odom"], dtype=float) ** 2)

    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def rotation(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s], [s, c]])

    def normal_cdf(x):
        return 0.5 * math.erfc(-x / math.sqrt(2.0))

    def gps_available(k):
        # The supplied two-element schedule denotes the inclusive outage interval.
        if len(dropout) == 2:
            absent = int(dropout[0]) <= k <= int(dropout[1])
        else:
            absent = k in dropout
        return gps_rate > 0 and k % gps_rate == 0 and not absent

    def update_covariance(p, h, r):
        innovation = h @ p @ h.T + r
        gain = np.linalg.solve(innovation, h @ p).T
        a = np.eye(3) - gain @ h
        result = a @ p @ a.T + gain @ r @ gain.T
        return 0.5 * (result + result.T)

    def nearest_on_wall(point, wall):
        a, b = wall
        d = b - a
        denominator = float(d @ d)
        u = 0.0 if denominator == 0 else float((point - a) @ d) / denominator
        return a + np.clip(u, 0.0, 1.0) * d

    results = []
    for name, route in spec["routes"].items():
        points = np.asarray(route, dtype=float)
        vectors = np.diff(points, axis=0)
        lengths = np.linalg.norm(vectors, axis=1)
        valid = lengths > 1e-12
        vectors = vectors[valid]
        lengths = lengths[valid]
        starts = points[:-1][valid]
        total_length = float(np.sum(lengths))
        if not len(lengths):
            results.append((name, 1.0, total_length, float("inf")))
            continue

        headings = np.arctan2(vectors[:, 1], vectors[:, 0])
        cumulative = np.cumsum(lengths)
        travel_step = max(speed * dt, 1e-12)
        count = max(1, int(math.ceil(total_length / travel_step)))

        covariance = initial_cov.copy()
        previous_heading = float(initial_mean[2])
        previous_position = points[0].copy()
        hazard = 0.0
        minimum_clearance = float("inf")

        # Pure pursuit corrects disturbances over approximately lookahead / v.
        response_time = lookahead / max(speed, 1e-12)
        correction_steps = max(1.0, response_time / dt)
        tracking_variance = (
            (float(actuator["v"]) * dt) ** 2 * correction_steps
            + (
                speed * float(actuator["omega"]) * dt * response_time
            ) ** 2 * correction_steps / 3.0
        )

        for k in range(count + 1):
            along = min(k * travel_step, total_length)
            j = min(int(np.searchsorted(cumulative, along, side="right")),
                    len(lengths) - 1)
            before = 0.0 if j == 0 else cumulative[j - 1]
            fraction = np.clip((along - before) / lengths[j], 0.0, 1.0)
            position = starts[j] + fraction * vectors[j]
            heading = float(headings[j])
            rot = rotation(heading)

            if k > 0:
                displacement = position - previous_position
                # World-coordinate prediction, with body/tangent odometry noise.
                f = np.eye(3)
                f[0, 2] = -displacement[1]
                f[1, 2] = displacement[0]
                g = np.eye(3)
                g[:2, :2] = rot
                covariance = f @ covariance @ f.T + g @ odom_cov @ g.T
            elif previous_heading != 0.0:
                g = np.eye(3)
                g[:2, :2] = rotation(previous_heading)
                covariance = g @ covariance @ g.T

            if gps_available(k):
                h = np.array([[1., 0., 0.], [0., 1., 0.]])
                covariance = update_covariance(
                    covariance, h, np.eye(2) * float(noise["gps"]) ** 2
                )

            for landmark in landmarks:
                delta = landmark - position
                r2 = float(delta @ delta)
                if 1e-12 < r2 <= visibility ** 2:
                    r = math.sqrt(r2)
                    h = np.array([
                        [delta[1] / r2, -delta[0] / r2, -1.0],
                        [-delta[0] / r, -delta[1] / r, 0.0],
                    ])
                    covariance = update_covariance(
                        covariance, h,
                        np.diag([float(noise["bearing"]) ** 2,
                                 float(noise["range"]) ** 2]),
                    )

            for wall in walls:
                separation = position - nearest_on_wall(position, wall)
                distance = float(np.linalg.norm(separation))
                clearance = distance - radius
                minimum_clearance = min(minimum_clearance, clearance)
                if distance > 1e-12:
                    normal = separation / distance
                    variance = float(normal @ covariance[:2, :2] @ normal)
                else:
                    variance = float(np.max(np.linalg.eigvalsh(covariance[:2, :2])))
                sigma = math.sqrt(max(variance + tracking_variance, 1e-16))
                collision_probability = normal_cdf(-clearance / sigma)
                if clearance <= 0.0:
                    collision_probability = 1.0
                # Nearby samples are correlated over the controller response time.
                weight = min(1.0, travel_step / max(lookahead, travel_step))
                hazard -= weight * math.log(max(1.0 - collision_probability, 1e-15))

            previous_position = position
            previous_heading = heading

        turn_angle = abs(wrap(float(headings[0]) - float(initial_mean[2])))
        turn_angle += sum(
            abs(wrap(float(b - a))) for a, b in zip(headings[:-1], headings[1:])
        )
        nominal_time = (
            max(0.0, total_length - float(spec["arrival_tolerance"]))
            / max(speed, 1e-12)
            + turn_angle / max(omega_max, 1e-12)
        )
        time_sigma = max(
            dt,
            float(actuator["v"]) * dt * math.sqrt(count) / max(speed, 1e-12),
        )
        arrival_probability = normal_cdf(
            (float(spec["max_steps"]) * dt - nominal_time) / time_sigma
        )
        probability = float(np.clip(math.exp(-hazard) * arrival_probability, 0., 1.))
        results.append((name, probability, total_length, minimum_clearance))

    best_probability = max(result[1] for result in results)
    # Distance breaks close probability ties.
    contenders = [r for r in results if r[1] >= best_probability - 0.01]
    chosen = min(contenders, key=lambda r: (r[2], str(r[0])))
    summary = "; ".join(
        f"{name}: estimated success {prob:.4f}, length {length:.2f} m, "
        f"minimum nominal clearance {clearance:.2f} m"
        for name, prob, length, clearance in results
    )
    return {
        "route": chosen[0],
        "success_prob": chosen[1],
        "reasoning": (
            "Nominal-path Gaussian risk approximation using localization covariance, "
            "GPS outages, visible landmarks, robot radius, actuator disturbances, "
            "pure-pursuit correction time, and the arrival deadline. "
            "This is not a calibrated closed-loop simulation probability. " + summary
        ),
    }
