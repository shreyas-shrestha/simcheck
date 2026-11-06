"""Dummy agent solution: dead-reckons body-frame odometry with gtsam.Pose2, grows covariance linearly."""
import gtsam
import numpy as np

print("agent noise on stdout must not break the protocol")


class Estimator:
    def __init__(self, spec):
        m = spec["initial_pose"]["mean"]
        self.pose = gtsam.Pose2(*m)
        self.P = np.diag(spec["initial_pose"]["cov_diag_variances"]).astype(float)
        self.Q = np.diag(np.square(spec["sensor_noise_std"]["odom"]))

    def step(self, meas):
        o = meas.get("odom")
        if o is not None:
            self.pose = self.pose.compose(gtsam.Pose2(o["dx"], o["dy"], o["dtheta"]))
            self.P = self.P + self.Q
        return [self.pose.x(), self.pose.y(), self.pose.theta()], self.P  # list + ndarray, per the contract


def make_estimator(public_spec):
    return Estimator(public_spec)


def choose_route(public_spec):
    return {"route": "B", "success_prob": 0.8, "reasoning": "landmarks on B"}
