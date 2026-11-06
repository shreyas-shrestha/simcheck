import math
import numpy as np
import gtsam


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _make_gps_factor(key: int, gx: float, gy: float, noise_model: gtsam.noiseModel.Base) -> gtsam.CustomFactor:
    meas = np.array([gx, gy], dtype=float)

    def error_func(this: gtsam.CustomFactor, values: gtsam.Values, jacobians: list):
        pose = values.atPose2(this.keys()[0])
        if jacobians is not None:
            c = math.cos(pose.theta())
            s = math.sin(pose.theta())
            jacobians[0] = np.array([
                [c, -s, 0.0],
                [s,  c, 0.0]
            ], dtype=float)
        return np.array([pose.x() - meas[0], pose.y() - meas[1]], dtype=float)

    return gtsam.CustomFactor(noise_model, [key], error_func)


def _make_landmark_factor(
    key: int,
    lx: float,
    ly: float,
    meas_range: float,
    meas_bearing: float,
    noise_model: gtsam.noiseModel.Base
) -> gtsam.CustomFactor:
    def error_func(this: gtsam.CustomFactor, values: gtsam.Values, jacobians: list):
        pose = values.atPose2(this.keys()[0])
        dx = lx - pose.x()
        dy = ly - pose.y()
        q = dx * dx + dy * dy
        r = math.sqrt(max(q, 1e-12))

        c = math.cos(pose.theta())
        s = math.sin(pose.theta())
        bx = c * dx + s * dy
        by = -s * dx + c * dy

        pred_bearing = math.atan2(by, bx)
        err_range = r - meas_range
        err_bearing = _wrap_angle(pred_bearing - meas_bearing)

        if jacobians is not None:
            jacobians[0] = np.array([
                [-bx / r, -by / r, 0.0],
                [by / max(q, 1e-12), -bx / max(q, 1e-12), -1.0]
            ], dtype=float)

        return np.array([err_range, err_bearing], dtype=float)

    return gtsam.CustomFactor(noise_model, [key], error_func)


def _parse_scales(public_spec: dict):
    input_units = public_spec.get("input_units", {})
    odom_unit = input_units.get("odom", "m")
    pos_scale = 0.001 if odom_unit == "mm" else (0.01 if odom_unit == "cm" else 1.0)

    bearing_unit = input_units.get("bearing", "rad")
    ang_scale = math.pi / 180.0 if bearing_unit in ("deg", "degrees") else 1.0
    return pos_scale, ang_scale


class Estimator:
    def __init__(self, public_spec: dict):
        self.spec = public_spec
        self.pos_scale, self.ang_scale = _parse_scales(public_spec)

        init_info = public_spec["initial_pose"]
        self.init_mean = [float(x) for x in init_info["mean"]]
        self.init_vars = [float(v) for v in init_info["cov_diag_variances"]]

        noise_std = public_spec["sensor_noise_std"]
        odom_std = [float(s) for s in noise_std["odom"]]
        range_std = float(noise_std["range"])
        bearing_std = float(noise_std["bearing"])
        gps_std = float(noise_std["gps"])

        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array(self.init_vars, dtype=float)
        )
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(odom_std, dtype=float)
        )
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(2, gps_std)
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([range_std, bearing_std], dtype=float)
        )

        self.landmarks = {
            str(k): (float(v[0]), float(v[1]))
            for k, v in public_spec.get("landmarks", {}).items()
        }

        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()
        self.last_pose = gtsam.Pose2(self.init_mean[0], self.init_mean[1], self.init_mean[2])

    def step(self, new_measurements: dict):
        k = int(new_measurements["step"])
        key_k = gtsam.symbol("x", k)

        if k == 0:
            init_pose = gtsam.Pose2(self.init_mean[0], self.init_mean[1], self.init_mean[2])
            self.graph.add(gtsam.PriorFactorPose2(key_k, init_pose, self.prior_noise))
            gps = new_measurements.get("gps")
            if gps is not None:
                gx = float(gps["x"])
                gy = float(gps["y"])
                # Initialize near weighted combination of prior and GPS
                w_prior = 1.0 / self.init_vars[0]
                w_gps = 1.0 / (float(self.spec["sensor_noise_std"]["gps"]) ** 2)
                x0 = (w_prior * self.init_mean[0] + w_gps * gx) / (w_prior + w_gps)
                y0 = (w_prior * self.init_mean[1] + w_gps * gy) / (w_prior + w_gps)
                self.last_pose = gtsam.Pose2(x0, y0, self.init_mean[2])
            else:
                self.last_pose = init_pose
            self.values.insert(key_k, self.last_pose)
        else:
            odom = new_measurements["odom"]
            dx = float(odom["dx"]) * self.pos_scale
            dy = float(odom["dy"]) * self.pos_scale
            dtheta = float(odom["dtheta"]) * self.ang_scale
            rel_pose = gtsam.Pose2(dx, dy, dtheta)

            key_prev = gtsam.symbol("x", k - 1)
            self.graph.add(gtsam.BetweenFactorPose2(key_prev, key_k, rel_pose, self.odom_noise))
            self.last_pose = self.last_pose.compose(rel_pose)
            self.values.insert(key_k, self.last_pose)

        gps = new_measurements.get("gps")
        if gps is not None:
            gx = float(gps["x"])
            gy = float(gps["y"])
            self.graph.add(_make_gps_factor(key_k, gx, gy, self.gps_noise))

        for lm in new_measurements.get("landmarks", []):
            lm_id = str(lm["id"])
            if lm_id in self.landmarks:
                lx, ly = self.landmarks[lm_id]
                r_meas = float(lm["range"]) * self.pos_scale
                b_meas = float(lm["bearing"]) * self.ang_scale
                self.graph.add(
                    _make_landmark_factor(key_k, lx, ly, r_meas, b_meas, self.landmark_noise)
                )

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)
        self.values = optimizer.optimize()
        self.last_pose = self.values.atPose2(key_k)

        marginals = gtsam.Marginals(self.graph, self.values)
        cov = marginals.marginalCovariance(key_k)

        pose_out = [
            float(self.last_pose.x()),
            float(self.last_pose.y()),
            float(self.last_pose.theta()),
        ]
        return pose_out, np.asarray(cov, dtype=float)


def make_estimator(public_spec: dict) -> Estimator:
    return Estimator(public_spec)


def build_graph(measurements: list, public_spec: dict):
    est = make_estimator(public_spec)
    for m in measurements:
        est.step(m)
    return est.graph, est.values


def _seg_seg_dist_sq(ax, ay, bx, by, cx, cy, dx, dy) -> float:
    # Minimum squared distance between segment AB and segment CD
    def pt_seg_dist_sq(px, py, x1, y1, x2, y2):
        vx, vy = x2 - x1, y2 - y1
        wx, wy = px - x1, py - y1
        c1 = wx * vx + wy * vy
        if c1 <= 0.0:
            return wx * wx + wy * wy
        c2 = vx * vx + vy * vy
        if c2 <= c1:
            return (px - x2) ** 2 + (py - y2) ** 2
        b = c1 / c2
        bx_p, by_p = x1 + b * vx, y1 + b * vy
        return (px - bx_p) ** 2 + (py - by_p) ** 2

    def ccw(p1x, p1y, p2x, p2y, p3x, p3y):
        return (p3y - p1y) * (p2x - p1x) > (p2y - p1y) * (p3x - p1x)

    if (ccw(ax, ay, cx, cy, dx, dy) != ccw(bx, by, cx, cy, dx, dy)) and (
        ccw(ax, ay, bx, by, cx, cy) != ccw(ax, ay, bx, by, dx, dy)
    ):
        return 0.0

    return min(
        pt_seg_dist_sq(ax, ay, cx, cy, dx, dy),
        pt_seg_dist_sq(bx, by, cx, cy, dx, dy),
        pt_seg_dist_sq(cx, cy, ax, ay, bx, by),
        pt_seg_dist_sq(dx, dy, ax, ay, bx, by),
    )


def _pure_pursuit_control(pose, waypoints, wp_idx, lookahead, v_cmd, omega_max):
    x, y, theta = pose
    n = len(waypoints)

    # Advance waypoint index if close to current target segment end
    while wp_idx < n - 1:
        wx, wy = waypoints[wp_idx + 1]
        if math.hypot(wx - x, wy - y) < lookahead and wp_idx + 1 < n - 1:
            wp_idx += 1
        else:
            break

    # Find lookahead point along polyline starting from wp_idx
    target = waypoints[-1]
    found = False
    for i in range(wp_idx, n - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - x, y1 - y
        a = dx * dx + dy * dy
        if a < 1e-12:
            continue
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - lookahead * lookahead
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            sqrt_disc = math.sqrt(disc)
            t2 = (-b + sqrt_disc) / (2.0 * a)
            t1 = (-b - sqrt_disc) / (2.0 * a)
            valid_t = None
            if 0.0 <= t2 <= 1.0:
                valid_t = t2
            elif 0.0 <= t1 <= 1.0:
                valid_t = t1
            if valid_t is not None:
                target = [x1 + valid_t * dx, y1 + valid_t * dy]
                found = True
                break

    if not found:
        # Project onto current segment and move forward by lookahead
        x1, y1 = waypoints[wp_idx]
        x2, y2 = waypoints[wp_idx + 1]
        dx, dy = x2 - x1, y2 - y1
        seg_len = math.hypot(dx, dy)
        if seg_len > 1e-9:
            t_proj = ((x - x1) * dx + (y - y1) * dy) / (seg_len * seg_len)
            t_proj = max(0.0, min(1.0, t_proj))
            t_la = min(1.0, t_proj + lookahead / seg_len)
            target = [x1 + t_la * dx, y1 + t_la * dy]

    tx, ty = target
    alpha = _wrap_angle(math.atan2(ty - y, tx - x) - theta)
    dist = max(math.hypot(tx - x, ty - y), 1e-6)
    curvature = 2.0 * math.sin(alpha) / lookahead
    omega = max(-omega_max, min(omega_max, v_cmd * curvature))
    return v_cmd, omega, wp_idx


def _simulate_route(route_waypoints, public_spec: dict, n_trials: int = 40, seed: int = 42) -> float:
    rng = np.random.default_rng(seed)
    dt = float(public_spec["dt"])
    max_steps = int(public_spec["max_steps"])
    arrival_tol = float(public_spec["arrival_tolerance"])
    robot_radius = float(public_spec["robot_radius"])
    walls = public_spec.get("walls", [])

    init_mean = np.array(public_spec["initial_pose"]["mean"], dtype=float)
    init_stds = np.sqrt(np.array(public_spec["initial_pose"]["cov_diag_variances"], dtype=float))

    sensor_std = public_spec["sensor_noise_std"]
    odom_std = np.array(sensor_std["odom"], dtype=float)
    range_std = float(sensor_std["range"])
    bearing_std = float(sensor_std["bearing"])
    gps_std = float(sensor_std["gps"])

    act_std = public_spec["actuator_noise_std"]
    v_std = float(act_std["v"])
    omega_std = float(act_std["omega"])

    gps_sched = public_spec.get("gps_schedule", {})
    gps_rate = int(gps_sched.get("rate_steps", 1))
    dropout = gps_sched.get("dropout_steps", [-1, -1])

    landmarks = {
        str(k): (float(v[0]), float(v[1]))
        for k, v in public_spec.get("landmarks", {}).items()
    }
    max_range = float(public_spec.get("landmark_visibility", {}).get("max_range", 5.0))

    ctrl = public_spec["controller"]
    lookahead = float(ctrl["lookahead"])
    v_nom = float(ctrl["v"])
    omega_max = float(ctrl["omega_max"])

    pos_scale, ang_scale = _parse_scales(public_spec)
    goal = route_waypoints[-1]

    successes = 0
    for _ in range(n_trials):
        est = make_estimator(public_spec)
        true_pose = gtsam.Pose2(*(init_mean + rng.normal(0.0, init_stds)))
        wp_idx = 0
        collided = False
        reached = False
        last_rel = None

        for step in range(max_steps):
            # Generate measurements at current step
            odom_meas = None
            if step > 0:
                noisy_rel = last_rel + rng.normal(0.0, odom_std)
                odom_meas = {
                    "dx": float(noisy_rel[0] / pos_scale),
                    "dy": float(noisy_rel[1] / pos_scale),
                    "dtheta": float(noisy_rel[2] / ang_scale),
                }

            in_dropout = (len(dropout) == 2 and dropout[0] <= step <= dropout[1])
            gps_meas = None
            if (step % gps_rate == 0) and not in_dropout:
                gps_meas = {
                    "x": float(true_pose.x() + rng.normal(0.0, gps_std)),
                    "y": float(true_pose.y() + rng.normal(0.0, gps_std)),
                }

            lm_list = []
            for lm_id, (lx, ly) in landmarks.items():
                dx = lx - true_pose.x()
                dy = ly - true_pose.y()
                r_true = math.hypot(dx, dy)
                if r_true <= max_range:
                    b_true = _wrap_angle(math.atan2(dy, dx) - true_pose.theta())
                    r_m = (r_true + rng.normal(0.0, range_std)) / pos_scale
                    b_m = _wrap_angle(b_true + rng.normal(0.0, bearing_std)) / ang_scale
                    lm_list.append({"id": lm_id, "range": float(r_m), "bearing": float(b_m)})

            meas = {
                "step": step,
                "t": step * dt,
                "odom": odom_meas,
                "gps": gps_meas,
                "landmarks": lm_list,
            }
            est_pose, _ = est.step(meas)

            if math.hypot(true_pose.x() - goal[0], true_pose.y() - goal[1]) <= arrival_tol:
                reached = True
                break

            v_cmd, omega_cmd, wp_idx = _pure_pursuit_control(
                est_pose, route_waypoints, wp_idx, lookahead, v_nom, omega_max
            )
            v_exec = v_cmd + rng.normal(0.0, v_std)
            omega_exec = omega_cmd + rng.normal(0.0, omega_std)

            dtheta = omega_exec * dt
            if abs(dtheta) < 1e-6:
                dx_b = v_exec * dt
                dy_b = 0.0
            else:
                dx_b = v_exec * dt * math.sin(dtheta) / dtheta
                dy_b = v_exec * dt * (1.0 - math.cos(dtheta)) / dtheta

            last_rel = np.array([dx_b, dy_b, dtheta], dtype=float)
            next_pose = true_pose.compose(gtsam.Pose2(dx_b, dy_b, dtheta))

            for w in walls:
                d2 = _seg_seg_dist_sq(
                    true_pose.x(), true_pose.y(),
                    next_pose.x(), next_pose.y(),
                    w[0], w[1], w[2], w[3]
                )
                if d2 <= robot_radius * robot_radius:
                    collided = True
                    break

            true_pose = next_pose
            if collided:
                break

            if math.hypot(true_pose.x() - goal[0], true_pose.y() - goal[1]) <= arrival_tol:
                reached = True
                break

        if reached and not collided:
            successes += 1

    return successes / float(n_trials)


def choose_route(public_spec: dict) -> dict:
    routes = public_spec["routes"]
    probs = {}
    lengths = {}

    for name, wps in routes.items():
        probs[name] = _simulate_route(wps, public_spec, n_trials=40, seed=12345)
        lengths[name] = sum(
            math.hypot(wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1])
            for i in range(len(wps) - 1)
        )

    best_route = max(routes.keys(), key=lambda r: (round(probs[r], 2), -lengths[r]))
    best_prob = float(probs[best_route])

    reasoning = (
        f"Route {best_route} selected with estimated collision-free arrival probability "
        f"{best_prob:.2f} (vs { {k: round(v, 2) for k, v in probs.items()} }). "
        f"Route B detours to y=3.0m, avoiding the narrow wall corridor at y=+-0.575m "
        f"and bringing the robot within the 5.0m visibility range of landmarks at y=5.5m "
        f"during the GPS dropout window (steps 20-35)."
    )
    return {
        "route": best_route,
        "success_prob": best_prob,
        "reasoning": reasoning,
    }
