import math
import numpy as np
import gtsam
from gtsam import symbol_shorthand

X = symbol_shorthand.X
L = symbol_shorthand.L


def _make_gps_factor(key, gx, gy, noise_model):
    def error_func(this, values, jacobians):
        pose = values.atPose2(key)
        if jacobians is not None:
            theta = pose.theta()
            c = math.cos(theta)
            s = math.sin(theta)
            jacobians[0] = np.array(
                [[c, -s, 0.0], [s, c, 0.0]], dtype=np.float64
            )
        return np.array([pose.x() - gx, pose.y() - gy], dtype=np.float64)

    return gtsam.CustomFactor(noise_model, [key], error_func)


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()

        # Parse unit conversions
        input_units = public_spec.get("input_units", {})
        self.odom_pos_scale = 0.001 if input_units.get("odom") == "mm" else 1.0
        self.bearing_scale = (
            math.pi / 180.0 if input_units.get("bearing") == "deg" else 1.0
        )

        # Noise models (all in SI units: metres and radians)
        init_cov = public_spec["initial_pose"]["cov_diag_variances"]
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array(init_cov, dtype=np.float64)
        )

        s_noise = public_spec["sensor_noise_std"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(s_noise["odom"], dtype=np.float64)
        )
        self.br_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([s_noise["bearing"], s_noise["range"]], dtype=np.float64)
        )
        gps_std = float(s_noise["gps"])
        self.gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([gps_std, gps_std], dtype=np.float64)
        )

        # Known landmarks with tight finite prior so Cholesky marginals remain well-conditioned
        self.landmarks = {}
        self.active_landmarks = set()
        self.lm_prior_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1e-6)
        for lm_id_str, pos in public_spec.get("landmarks", {}).items():
            lm_idx = int(lm_id_str)
            pt = np.array([float(pos[0]), float(pos[1])], dtype=np.float64)
            self.landmarks[lm_id_str] = (lm_idx, pt)

        self.current_pose = None

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        key_k = X(k)

        if k == 0:
            mean = self.spec["initial_pose"]["mean"]
            init_pose = gtsam.Pose2(
                float(mean[0]), float(mean[1]), float(mean[2])
            )
            self.graph.add(
                gtsam.PriorFactorPose2(key_k, init_pose, self.prior_noise)
            )
            self.initial_estimates.insert(key_k, init_pose)
            self.current_pose = init_pose
        else:
            odom = new_measurements["odom"]
            dx = float(odom["dx"]) * self.odom_pos_scale
            dy = float(odom["dy"]) * self.odom_pos_scale
            dtheta = float(odom["dtheta"])
            odom_pose = gtsam.Pose2(dx, dy, dtheta)

            key_prev = X(k - 1)
            self.graph.add(
                gtsam.BetweenFactorPose2(
                    key_prev, key_k, odom_pose, self.odom_noise
                )
            )

            pred_pose = self.current_pose.compose(odom_pose)
            self.initial_estimates.insert(key_k, pred_pose)
            self.current_pose = pred_pose

        # World-frame 2D GPS measurement
        gps = new_measurements.get("gps")
        if gps is not None:
            gx = float(gps["x"])
            gy = float(gps["y"])
            self.graph.add(_make_gps_factor(key_k, gx, gy, self.gps_noise))

        # Landmark range-bearing measurements
        landmarks = new_measurements.get("landmarks", [])
        for lm in landmarks:
            lm_id_str = str(lm["id"])
            if lm_id_str in self.landmarks:
                lm_idx, pt = self.landmarks[lm_id_str]
                lm_key = L(lm_idx)
                if lm_idx not in self.active_landmarks:
                    self.active_landmarks.add(lm_idx)
                    self.initial_estimates.insert(lm_key, pt)
                    self.graph.add(
                        gtsam.PriorFactorPoint2(
                            lm_key, pt, self.lm_prior_noise
                        )
                    )
                bearing = float(lm["bearing"]) * self.bearing_scale
                rng = float(lm["range"])
                self.graph.add(
                    gtsam.BearingRangeFactor2D(
                        key_k,
                        lm_key,
                        gtsam.Rot2(bearing),
                        rng,
                        self.br_noise,
                    )
                )

        # Batch optimize
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, self.initial_estimates, params
        )
        result = optimizer.optimize()

        self.initial_estimates = result
        self.current_pose = result.atPose2(key_k)

        marginals = gtsam.Marginals(self.graph, result)
        cov = marginals.marginalCovariance(key_k)

        pose_out = [
            float(self.current_pose.x()),
            float(self.current_pose.y()),
            float(self.current_pose.theta()),
        ]
        return pose_out, np.array(cov, dtype=np.float64)


def make_estimator(public_spec):
    return Estimator(public_spec)


def build_graph(measurements, public_spec):
    est = make_estimator(public_spec)
    for m in measurements:
        est.step(m)
    return est.graph, est.initial_estimates


def choose_route(public_spec):
    return {
        "route": "B",
        "success_prob": 0.98,
        "reasoning": (
            "Route A passes along y=0 through a narrow wall corridor at x in [5.4, 9.0], y=+/-0.45 "
            "(only 0.25m clearance for robot_radius=0.2m). During steps 20-35, GPS drops out and "
            "all landmarks at y=5.5 are out of range (5.5m > max_range=5.0m), leaving Route A with "
            "pure dead-reckoning inside the corridor and a high collision risk. Route B detours to "
            "y=3.0, completely avoiding the walls and staying within ~2.5m of the landmarks at y=5.5 "
            "for continuous range-bearing localization throughout the GPS dropout."
        ),
    }
