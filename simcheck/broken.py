"""Deliberately broken variants of the reference. Each must be caught by the layer named.

Each variant is a module-like object with make_estimator / choose_route, usable with InProcessRunner.
"""
import math
from types import SimpleNamespace

import gtsam
import numpy as np

from simcheck import reference


class _ScaledCov(reference.Estimator):
    scale = 1.0

    def step(self, m):
        pose, P = super().step(m)
        return pose, np.asarray(P) * self.scale  # trajectory unchanged


class _Overconfident(_ScaledCov):
    scale = 0.1


class _Underconfident(_ScaledCov):
    scale = 10.0


class _WrongOdomFrame(reference.Estimator):
    """Treats body-frame odometry as world-frame displacement."""

    def _odom_body(self, o):
        d = np.array([o["dx"], o["dy"]]) * self.odom_scale
        th = self.last.theta()
        c, s = math.cos(th), math.sin(th)
        d = np.array([[c, s], [-s, c]]) @ d  # world -> body using the current heading, wrongly
        return gtsam.Pose2(d[0], d[1], o["dtheta"])


class _MissingPrior(reference.Estimator):
    """No informative prior on x_0: sigma 10 everywhere (a truly absent prior makes iSAM2 indeterminate)."""

    def __init__(self, spec):
        super().__init__(spec)
        self.prior_noise = gtsam.noiseModel.Isotropic.Sigma(3, 10.0)


def _variant(cls, route=None):
    def choose(spec):
        if route is not None:
            return {"route": route, "success_prob": 0.95, "reasoning": f"always {route}"}
        return reference.choose_route(spec)
    return SimpleNamespace(make_estimator=cls, choose_route=choose, __name__=cls.__name__)


VARIANTS = {
    "cov_x0.1": _variant(_Overconfident),
    "cov_x10": _variant(_Underconfident),
    "wrong_odom_frame": _variant(_WrongOdomFrame),
    "missing_prior": _variant(_MissingPrior),
    "always_A": _variant(reference.Estimator, route="A"),
    # v3.2, unknown_noise only: ignores odom_noise_scale_range and uses the nominal std
    "nominal_noise": _variant(reference.Estimator),
}
