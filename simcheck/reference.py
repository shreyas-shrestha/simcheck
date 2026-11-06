"""Reference estimator: iSAM2 over Pose2 with prior, odometry, GPS and known-landmark factors.

Its factor residuals are exactly the noise model sim.py injects, so it is the yardstick for
accuracy (Layer 2) and NLL (Layer 3). Same contract as an agent's solution.py.
"""
import math

import gtsam
import numpy as np

X = gtsam.symbol_shorthand.X


def _gps_factor(z, sigma, key):
    def err(this, values, H):
        p = values.atPose2(this.keys()[0])
        if H is not None:
            H[0] = np.hstack([p.rotation().matrix(), np.zeros((2, 1))])
        return np.array([p.x() - z[0], p.y() - z[1]])
    return gtsam.CustomFactor(gtsam.noiseModel.Isotropic.Sigma(2, sigma), [key], err)


def _landmark_factor(lm, bearing, rng, s_b, s_r, key):
    lm = np.asarray(lm, float)

    def err(this, values, H):
        p = values.atPose2(this.keys()[0])
        pb = p.rotation().matrix().T @ (lm - np.array([p.x(), p.y()]))
        r = float(np.hypot(*pb))
        if H is not None:
            # perturbation x' = x * Exp([dx, dy, dth]); pb' ~= pb - [dx, dy] + dth * [pb_y, -pb_x]
            H[0] = np.array([[pb[1] / r**2, -pb[0] / r**2, -1.0],
                             [-pb[0] / r, -pb[1] / r, 0.0]])
        b_err = (math.atan2(pb[1], pb[0]) - bearing + math.pi) % (2 * math.pi) - math.pi
        return np.array([b_err, r - rng])
    return gtsam.CustomFactor(gtsam.noiseModel.Diagonal.Sigmas(np.array([s_b, s_r])), [key], err)


class Estimator:
    def __init__(self, spec):
        n = spec["sensor_noise_std"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.asarray(n["odom"], float))
        self.s_gps, self.s_b, self.s_r = n["gps"], n["bearing"], n["range"]
        ip = spec["initial_pose"]
        self.prior_mean = gtsam.Pose2(*ip["mean"])
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(np.asarray(ip["cov_diag_variances"], float))
        self.landmarks = spec["landmarks"]
        self.odom_scale = 1e-3 if spec["input_units"]["odom"] == "mm" else 1.0
        self.bearing_deg = spec["input_units"]["bearing"] == "deg"
        self.odom_frame = spec["odom_frame"]
        self.odom_heading = 0.0
        self.use_prior = True
        p = gtsam.ISAM2Params()
        p.setRelinearizeThreshold(0.01)
        p.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(p)
        self.last = None

    def _odom_body(self, o):
        d = np.array([o["dx"], o["dy"]]) * self.odom_scale
        if self.odom_frame == "odom":
            c, s = math.cos(self.odom_heading), math.sin(self.odom_heading)
            d = np.array([[c, s], [-s, c]]) @ d
            self.odom_heading += o["dtheta"]
        return gtsam.Pose2(d[0], d[1], o["dtheta"])

    def step(self, m):
        k = m["step"]
        g, v = gtsam.NonlinearFactorGraph(), gtsam.Values()
        if k == 0:
            if self.use_prior:
                g.add(gtsam.PriorFactorPose2(X(0), self.prior_mean, self.prior_noise))
            v.insert(X(0), self.prior_mean)
        else:
            odom = self._odom_body(m["odom"])
            g.add(gtsam.BetweenFactorPose2(X(k - 1), X(k), odom, self.odom_noise))
            v.insert(X(k), self.last.compose(odom))
        if m.get("gps"):
            g.add(_gps_factor((m["gps"]["x"], m["gps"]["y"]), self.s_gps, X(k)))
        for obs in m.get("landmarks") or []:
            b = math.radians(obs["bearing"]) if self.bearing_deg else obs["bearing"]
            g.add(_landmark_factor(self.landmarks[str(obs["id"])], b, obs["range"], self.s_b, self.s_r, X(k)))
        self.isam.update(g, v)
        self.isam.update()  # one extra relinearisation pass for the nonlinear factors
        self.last = self.isam.calculateEstimatePose2(X(k))
        P = self.isam.marginalCovariance(X(k))
        return [self.last.x(), self.last.y(), self.last.theta()], P


class ScaledNoiseEstimator(Estimator):
    """unknown_noise: odometry std = nominal x unknown scale k. Batch LM each step; k^2 from the odometry
    chi-square per degree of freedom (variance-component estimate), shrunk toward the middle of the stated
    range and clipped to it. Covariances follow because the odometry factors are re-weighted by k."""

    PRIOR_DOF = 6.0  # pseudo-observations at the geometric middle of the range

    def __init__(self, spec):
        super().__init__(spec)
        self.sig = np.asarray(spec["sensor_noise_std"]["odom"], float)
        self.lo, self.hi = spec["odom_noise_scale_range"]
        self.k = math.sqrt(self.lo * self.hi)
        self.odoms, self.unary, self.n_unary_dims = [], [], 0
        self.values = gtsam.Values()

    def _graph(self, k):
        g = gtsam.NonlinearFactorGraph()
        g.add(gtsam.PriorFactorPose2(X(0), self.prior_mean, self.prior_noise))
        noise = gtsam.noiseModel.Diagonal.Sigmas(self.sig * k)
        for i, z in self.odoms:
            g.add(gtsam.BetweenFactorPose2(X(i - 1), X(i), z, noise))
        for f in self.unary:
            g.add(f)
        return g

    def _solve(self, k):
        g = self._graph(k)
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(20)
        return g, gtsam.LevenbergMarquardtOptimizer(g, self.values, params).optimize()

    def step(self, m):
        k_step = m["step"]
        if k_step == 0:
            self.values.insert(X(0), self.prior_mean)
        else:
            odom = self._odom_body(m["odom"])
            self.odoms.append((k_step, odom))
            self.values.insert(X(k_step), self.values.atPose2(X(k_step - 1)).compose(odom))
        if m.get("gps"):
            self.unary.append(_gps_factor((m["gps"]["x"], m["gps"]["y"]), self.s_gps, X(k_step)))
            self.n_unary_dims += 2
        for obs in m.get("landmarks") or []:
            b = math.radians(obs["bearing"]) if self.bearing_deg else obs["bearing"]
            self.unary.append(_landmark_factor(self.landmarks[str(obs["id"])], b, obs["range"], self.s_b, self.s_r,
                                               X(k_step)))
            self.n_unary_dims += 2
        n_odom = 3 * len(self.odoms)
        k0 = math.sqrt(self.lo * self.hi)
        for _ in range(2 if k_step % 5 == 0 else 0):  # re-estimate the scale every 5th step (speed)
            g, est = self._solve(self.k)
            if not n_odom:
                break
            # Variance-component estimate: whitened odometry chi-square over its redundancy numbers
            # (1 - leverage, from the hat-matrix diagonal of the whitened Jacobian). Rows follow factor
            # order: prior (3), odometry (3 each), then GPS/landmark rows.
            A, r = g.linearize(est).jacobian()
            Q, _ = np.linalg.qr(A)
            lev = np.sum(Q ** 2, axis=1)[3:3 + n_odom]
            chi = float(np.sum(r[3:3 + n_odom] ** 2))
            dof = float(np.sum(1.0 - lev))
            k2 = (self.PRIOR_DOF * k0 ** 2 + self.k ** 2 * chi) / (self.PRIOR_DOF + dof)
            self.k = float(np.clip(math.sqrt(k2), self.lo, self.hi))
        g, est = self._solve(self.k)
        self.values = est
        P = gtsam.Marginals(g, est).marginalCovariance(X(k_step))
        p = est.atPose2(X(k_step))
        return [p.x(), p.y(), p.theta()], P


def make_estimator(public_spec):
    if public_spec.get("odom_noise_scale_range"):
        return ScaledNoiseEstimator(public_spec)
    return Estimator(public_spec)


def choose_route(public_spec, n_trials=40, seed_base=10_000_000):
    """Monte Carlo with this estimator in the loop on private seeds (not eval/fresh seeds)."""
    from simcheck import sim
    rates, length = {}, {}
    for r, pts in public_spec["routes"].items():
        # unknown_noise: simulate with scales drawn from the stated range (private seeds, nothing hidden)
        from simcheck.generator import true_spec
        ok = sum(sim.run_episode(true_spec(public_spec, seed_base + i), r, seed_base + i,
                                 estimator=make_estimator(public_spec))["outcome"]["success"]
                 for i in range(n_trials))
        rates[r] = ok / n_trials
        length[r] = float(np.sum(np.linalg.norm(np.diff(np.asarray(pts), axis=0), axis=1)))
    best = max(rates, key=lambda r: (rates[r], -length[r]))
    close = [r for r in rates if rates[best] - rates[r] <= 0.02]
    best = min(close, key=lambda r: length[r])  # distance breaks close ties
    return {"route": best, "success_prob": rates[best],
            "reasoning": f"MC with reference estimator, {n_trials} trials/route: {rates}"}
