"""localCoordinates convention against a hand-computed Pose2 example, plus factor Jacobians."""
import math

import gtsam
import numpy as np

from simcheck import reference


def test_local_coordinates_hand_example():
    # a = (1, 2, 90deg), b = (1, 3, 90deg + 0.1). between = a^-1 b = (R(-90) [0, 1], 0.1) = (1, 0, 0.1)
    a = gtsam.Pose2(1, 2, math.pi / 2)
    b = gtsam.Pose2(1, 3, math.pi / 2 + 0.1)
    rel = a.between(b)
    assert np.allclose([rel.x(), rel.y(), rel.theta()], [1, 0, 0.1], atol=1e-12)
    # Logmap of (1, 0, w=0.1): V^-1 t with V = [[sin w/w, -(1-cos w)/w], [(1-cos w)/w, sin w/w]]
    w = 0.1
    V = np.array([[math.sin(w) / w, -(1 - math.cos(w)) / w], [(1 - math.cos(w)) / w, math.sin(w) / w]])
    expected = np.r_[np.linalg.solve(V, [1, 0]), w]
    assert np.allclose(a.localCoordinates(b), expected, atol=1e-12)
    assert np.allclose(a.retract(expected).matrix(), b.matrix(), atol=1e-12)


def _numeric_jac(f, p, eps=1e-6):
    J = []
    for i in range(3):
        d = np.zeros(3); d[i] = eps
        J.append((f(p.retract(d)) - f(p.retract(-d))) / (2 * eps))
    return np.array(J).T


def _check(factor, p):
    vals = gtsam.Values(); vals.insert(1, p)
    lin = factor.linearize(vals)
    A, _ = lin.jacobian()
    sig = factor.noiseModel().sigmas()
    def f(q):
        v = gtsam.Values(); v.insert(1, q)
        return factor.unwhitenedError(v)
    assert np.allclose(A * sig[:, None], _numeric_jac(f, p), atol=1e-5)


def test_factor_jacobians():
    p = gtsam.Pose2(0.3, -0.2, 0.7)
    _check(reference._gps_factor((0.5, 0.1), 0.2, 1), p)
    _check(reference._landmark_factor((3.0, 2.0), 0.4, 3.0, 0.05, 0.1, 1), p)


# ---- convention details the verifier relies on (added with the sandbox/harness integration)

def test_second_hand_example_large_rotation():
    # est = (1, 2, pi/2), true = (1, 3, pi): between = (1, 0, pi/2).
    # V(pi/2) = (2/pi) [[1, -1], [1, 1]], V^-1 = (pi/4) [[1, 1], [-1, 1]] -> e = [pi/4, -pi/4, pi/2]
    e = gtsam.Pose2(1, 2, math.pi / 2).localCoordinates(gtsam.Pose2(1, 3, math.pi))
    assert np.allclose(e, [math.pi / 4, -math.pi / 4, math.pi / 2], atol=1e-12)


def test_error_is_in_estimate_body_frame_not_world():
    # est faces +y; true is 1 m further along world +y. World difference (0, 1, 0); body frame (1, 0, 0).
    e = gtsam.Pose2(0, 0, math.pi / 2).localCoordinates(gtsam.Pose2(0, 1, math.pi / 2))
    assert np.allclose(e, [1, 0, 0], atol=1e-12)


def test_argument_order_matters():
    est, true = gtsam.Pose2(0, 0, math.pi / 2), gtsam.Pose2(0, 1, math.pi / 2)
    assert np.allclose(true.localCoordinates(est), [-1, 0, 0], atol=1e-12)


def test_angle_wraps_through_pi():
    # theta 3.0 -> -3.0 is a +0.2832 rad rotation, not -6.0
    e = gtsam.Pose2(0, 0, 3.0).localCoordinates(gtsam.Pose2(0, 0, -3.0))
    assert np.allclose(e, [0, 0, 2 * math.pi - 6.0], atol=1e-12)


def test_verifier_pose_error_uses_this_convention():
    from simcheck import verifier
    e = verifier.pose_error([1, 2, math.pi / 2], [1, 3, math.pi])
    assert np.allclose(e, [math.pi / 4, -math.pi / 4, math.pi / 2], atol=1e-12)
