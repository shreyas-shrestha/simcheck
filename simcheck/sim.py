"""Trusted simulator: unicycle dynamics, sensors, pure-pursuit controller, collision check.

Identical for every agent. Noise is injected exactly in the residual model the reference uses:
  initial pose  x0   = mean.retract(n),        n ~ N(0, diag(var))
  odometry      z    = between(x_{k-1}, x_k).retract(n)      (Pose2 tangent noise)
  gps           z    = t_k + n                                (world frame, metres)
  landmark      z    = [bearing(x_k, l) + n_b, range(x_k, l) + n_r]
Every noise stream has its own RNG spawned from the trial seed, and every stream draws the same
number of samples per step whatever happens, so trial i on route A and route B share noise
(common random numbers).
"""
import math

import gtsam
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------- geometry

def point_seg_dist(p, a, b):
    p, a, b = map(np.asarray, (p, a, b))
    ab = b - a
    denom = float(ab @ ab)
    u = 0.0 if denom == 0 else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + u * ab)))


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def seg_seg_dist(p1, p2, q1, q2):
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    if (d1 * d2 < 0) and (d3 * d4 < 0):
        return 0.0
    return min(point_seg_dist(p1, q1, q2), point_seg_dist(p2, q1, q2),
               point_seg_dist(q1, p1, p2), point_seg_dist(q2, p1, p2))


def collides(p_old, p_new, walls, radius):
    """True robot disk swept from p_old to p_new touches any wall segment."""
    return any(seg_seg_dist(p_old, p_new, w[:2], w[2:]) < radius for w in walls)


# ---------------------------------------------------------------- controller

class PurePursuit:
    """Tracks a polyline using whatever pose it is given (the estimator's, in the loop)."""

    def __init__(self, route, lookahead, v, omega_max, arrival_tolerance):
        self.pts = np.asarray(route, float)
        seg = np.diff(self.pts, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.total = float(self.cum[-1])
        self.L, self.v, self.wmax = lookahead, v, omega_max
        self.stop_radius = 0.5 * arrival_tolerance
        self.s = 0.0  # monotone progress along the path

    def point_at(self, s):
        s = float(np.clip(s, 0.0, self.total))
        i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.seg_len) - 1)
        u = (s - self.cum[i]) / self.seg_len[i]
        return self.pts[i] + u * (self.pts[i + 1] - self.pts[i])

    def _project(self, p):
        best = (math.inf, self.s)
        for i in range(len(self.seg_len)):
            a, b = self.pts[i], self.pts[i + 1]
            u = float(np.clip((p - a) @ (b - a) / self.seg_len[i] ** 2, 0.0, 1.0))
            s = self.cum[i] + u * self.seg_len[i]
            if not (self.s - 0.5 <= s <= self.s + 1.5):  # local search window, no skipping ahead
                continue
            d = float(np.linalg.norm(p - (a + u * (b - a))))
            if d < best[0]:
                best = (d, s)
        return best[1]

    def command(self, pose):
        x, y, th = pose
        p = np.array([x, y])
        if np.linalg.norm(p - self.pts[-1]) < self.stop_radius:
            return 0.0, 0.0
        self.s = max(self.s, self._project(p))
        tgt = self.point_at(self.s + self.L)
        ld = max(float(np.linalg.norm(tgt - p)), 1e-6)
        alpha = wrap(math.atan2(tgt[1] - y, tgt[0] - x) - th)
        omega = float(np.clip(self.v * 2.0 * math.sin(alpha) / ld, -self.wmax, self.wmax))
        return self.v, omega


# ---------------------------------------------------------------- sensors and encoding

class Sensors:
    def __init__(self, spec, rngs):
        self.spec = spec
        n = spec["sensor_noise_std"]
        self.s_odom = np.asarray(n["odom"], float)
        self.s_range, self.s_bearing, self.s_gps = n["range"], n["bearing"], n["gps"]
        self.lm_ids = sorted(spec["landmarks"], key=int)
        self.lm = np.array([spec["landmarks"][i] for i in self.lm_ids], float).reshape(-1, 2)
        self.max_range = spec["landmark_visibility"]["max_range"]
        gs = spec.get("gps_schedule")
        self.gps_on = gs is not None
        self.gps_rate = gs["rate_steps"] if gs else 1
        self.dropout = gs.get("dropout_steps") if gs else None
        self.r_odom, self.r_gps, self.r_lm = rngs
        self.odom_units_mm = spec["input_units"]["odom"] == "mm"
        self.bearing_deg = spec["input_units"]["bearing"] == "deg"
        self.odom_world = spec["odom_frame"] == "odom"
        self.odom_heading = 0.0  # dead-reckoned heading of the odom frame (trap_frame only)

    def gps_available(self, k):
        if not self.gps_on or k % self.gps_rate:
            return False
        return not (self.dropout and self.dropout[0] <= k <= self.dropout[1])

    def observe(self, k, dt, x, x_prev):
        """Truth -> encoded measurement dict exactly as the estimator receives it."""
        n_odom = self.r_odom.normal(size=3) * self.s_odom
        n_gps = self.r_gps.normal(size=2) * self.s_gps
        n_lm = self.r_lm.normal(size=(len(self.lm), 2)) * [self.s_bearing, self.s_range]

        odom = None
        if x_prev is not None:
            z = x_prev.between(x).retract(n_odom)
            d = np.array([z.x(), z.y()])
            if self.odom_world:
                c, s = math.cos(self.odom_heading), math.sin(self.odom_heading)
                d = np.array([[c, -s], [s, c]]) @ d
                self.odom_heading = wrap(self.odom_heading + z.theta())
            if self.odom_units_mm:
                d = d * 1000.0
            odom = {"dx": float(d[0]), "dy": float(d[1]), "dtheta": float(z.theta())}

        gps = None
        if self.gps_available(k):
            g = np.array([x.x(), x.y()]) + n_gps
            gps = {"x": float(g[0]), "y": float(g[1])}

        lms = []
        R = x.rotation().matrix()
        t = np.array([x.x(), x.y()])
        for j, lid in enumerate(self.lm_ids):
            pb = R.T @ (self.lm[j] - t)
            r = float(np.hypot(*pb))
            if r > self.max_range:
                continue
            b = wrap(math.atan2(pb[1], pb[0]) + n_lm[j, 0])
            if self.bearing_deg:
                b = math.degrees(b)
            lms.append({"id": lid, "range": r + float(n_lm[j, 1]), "bearing": b})

        return {"step": k, "t": k * dt, "odom": odom, "gps": gps, "landmarks": lms}


# ---------------------------------------------------------------- episode

def trial_rngs(seed):
    ss = np.random.SeedSequence(seed).spawn(5)
    return [np.random.default_rng(s) for s in ss]  # init, actuator, odom, gps, landmarks


def sample_x0(spec, rng):
    ip = spec["initial_pose"]
    n = rng.normal(size=3) * np.sqrt(ip["cov_diag_variances"])
    return gtsam.Pose2(*ip["mean"]).retract(n)


def run_episode(spec, route, seed, estimator=None, oracle=False, max_steps=None, stop_on_arrival=True):
    """Drive the true robot along spec['routes'][route].

    oracle=True: the controller sees the TRUE pose (used by the generator to make datasets).
    Otherwise the controller sees estimator.step(...)'s pose, and only observations up to the
    current step are ever passed to the estimator.
    """
    r_init, r_act, r_odom, r_gps, r_lm = trial_rngs(seed)
    sensors = Sensors(spec, (r_odom, r_gps, r_lm))
    c = spec["controller"]
    ctrl = PurePursuit(spec["routes"][route], c["lookahead"], c["v"], c["omega_max"], spec["arrival_tolerance"])
    dt, tol, rad, walls = spec["dt"], spec["arrival_tolerance"], spec["robot_radius"], spec["walls"]
    s_v, s_w = spec["actuator_noise_std"]["v"], spec["actuator_noise_std"]["omega"]
    goal = np.asarray(spec["routes"][route][-1], float)
    max_steps = max_steps or spec["max_steps"]

    x = sample_x0(spec, r_init)
    truth, meas, est, covs = [x], [], [], []

    def feed(m):
        meas.append(m)
        if estimator is not None:
            pose, P = estimator.step(m)
            est.append(np.asarray(pose, float))
            covs.append(np.asarray(P, float))

    feed(sensors.observe(0, dt, x, None))
    outcome = {"arrived": False, "collision": False, "steps": 0}
    for k in range(1, max_steps + 1):
        a_noise = r_act.normal(size=2) * [s_v, s_w]
        ctl_pose = (x.x(), x.y(), x.theta()) if oracle else tuple(est[-1])
        v, w = ctrl.command(ctl_pose)
        if v != 0.0 or w != 0.0:  # a stopped robot stays put
            v, w = v + a_noise[0], w + a_noise[1]
        x_new = x.compose(gtsam.Pose2.Expmap(np.array([v * dt, 0.0, w * dt])))
        hit = collides((x.x(), x.y()), (x_new.x(), x_new.y()), walls, rad)
        x_prev, x = x, x_new
        truth.append(x)
        feed(sensors.observe(k, dt, x, x_prev))
        outcome["steps"] = k
        if hit:
            outcome["collision"] = True
            break
        if np.linalg.norm([x.x() - goal[0], x.y() - goal[1]]) <= tol:
            outcome["arrived"] = True
            if stop_on_arrival:
                break
    outcome["success"] = outcome["arrived"] and not outcome["collision"]
    return {
        "truth": np.array([[p.x(), p.y(), p.theta()] for p in truth]),
        "measurements": meas,
        "est": np.array(est) if est else None,
        "cov": np.array(covs) if covs else None,
        "outcome": outcome,
    }
