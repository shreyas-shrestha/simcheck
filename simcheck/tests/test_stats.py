"""Hand-computed checks of the verifier's statistics. Expected values come from closed forms, exact
binomial sums (math.comb) or printed chi-square tables, never from re-running scipy."""
import math

import numpy as np
import pytest

from simcheck import verifier as V

TH = V.CONFIG["thresholds"]
I3 = np.eye(3)
N = 50                          # trajectory length for synthetic datasets
SPEC = {"gps_schedule": {"rate_steps": 1, "dropout_steps": [20, 35]}}


# ---------------------------------------------------------------- synthetic datasets

def _dataset(nees_drop, nees_end, P=I3):
    """True poses all at the origin; the estimate at the two evaluation steps is offset along x so that
    e = est.localCoordinates(true) = (-d, 0, 0) and NEES = d^2 under P = I."""
    est = np.zeros((N, 3))
    est[35, 0] = math.sqrt(nees_drop)
    est[N - 1, 0] = math.sqrt(nees_end)
    return (est, np.repeat(P[None], N, axis=0))


def _calib(nees_values, ref_nees=3.0):
    est = [_dataset(v, v) for v in nees_values]
    ref = [_dataset(ref_nees, ref_nees) for _ in nees_values]
    scen = [(f"s{i}", SPEC, None, np.zeros((N, 3))) for i in range(len(nees_values))]
    return V.layer3_calibration(est, ref, scen, TH)


def _exact_binom_cdf(c, K, p=0.95):
    return sum(math.comb(K, i) * p ** i * (1 - p) ** (K - i) for i in range(c + 1))


# ---------------------------------------------------------------- Wilson

@pytest.mark.parametrize("s,n,lo,hi", [(5, 10, 0.23659, 0.76341), (0, 10, 0.0, 0.27753),
                                       (10, 10, 0.72247, 1.0), (30, 30, 0.88648, 1.0)])
def test_wilson_hand(s, n, lo, hi):
    assert list(V.wilson(s, n)) == pytest.approx([lo, hi], abs=2e-4)   # z = 1.96 vs 1.95996: < 1e-4


# ---------------------------------------------------------------- pose error / NEES / NLL

def test_pose_error_and_nees_hand():
    # est = (2, 0, 0), true = origin -> e = (-2, 0, 0); P = diag(4, 1, .25), det P = 1
    nees, nll = V.nees_nll([2.0, 0.0, 0.0], np.diag([4.0, 1.0, 0.25]), [0.0, 0.0, 0.0])
    assert nees == pytest.approx(1.0)
    assert nll == pytest.approx(0.5 * (1.0 + 3 * math.log(2 * math.pi)))


def test_nll_scaling_hand():
    _, l1 = V.nees_nll([0, 0, 0], I3, [0, 0, 0])
    _, l2 = V.nees_nll([0, 0, 0], 2 * I3, [0, 0, 0])
    assert l1 == pytest.approx(1.5 * math.log(2 * math.pi))
    assert l2 - l1 == pytest.approx(1.5 * math.log(2))


# ---------------------------------------------------------------- calibration rule (a): overconfidence

def test_perfect_coverage_does_not_fail():
    c = _calib([3.0] * 30)
    for t in c["times"].values():
        assert t["covered"] == 30 and t["overconfidence_ok"] and t["coverage_p_value"] == pytest.approx(1.0)
    assert c["pass"] and c["wording"] == "no calibration inconsistency detected (K=30)"


@pytest.mark.parametrize("covered,ok", [(25, True), (24, False)])
def test_binomial_threshold_hand(covered, ok):
    # exact sums: P(X<=25 | 30, .95) = 0.015636 (keep), P(X<=24) = 0.0032825 (reject at alpha .01)
    t = _calib([3.0] * covered + [20.0] * (30 - covered))["times"]["end_of_trajectory"]
    assert t["covered"] == covered and t["overconfidence_ok"] is ok
    assert t["coverage_p_value"] == pytest.approx(_exact_binom_cdf(covered, 30), rel=1e-9)
    assert t["coverage_p_value"] == pytest.approx({25: 0.01563551, 24: 0.0032824856}[covered], rel=1e-6)


def test_nees_gate_is_7_81():
    t = _calib([7.80] * 30)["times"]["end_of_trajectory"]
    assert t["covered"] == 30
    t = _calib([7.82] * 30)["times"]["end_of_trajectory"]
    assert t["covered"] == 0


# ---------------------------------------------------------------- calibration rule (b): underconfidence

def test_nees_lower_bound_table():
    # chi-square table: chi2_{0.005}(90) = 59.196 -> 59.196 / 30 = 1.9732
    t = _calib([3.0] * 30)["times"]["end_of_trajectory"]
    assert t["nees_lower_bound"] == pytest.approx(59.196 / 30, abs=1e-3)


def test_underconfident_fails_only_rule_b():
    t = _calib([0.3] * 30, ref_nees=0.3)["times"]["end_of_trajectory"]   # same NLL as ref -> only (b) fails
    assert t["overconfidence_ok"] and not t["underconfidence_ok"] and t["nll_ok"] and not t["pass"]


# ---------------------------------------------------------------- calibration rule (c): NLL margin

def test_nll_margin_boundary():
    # NLL = 0.5 (NEES + 3 log 2pi) under P = I. NEES 3 vs reference NEES 1: difference exactly 1.0 nat = margin
    t = _calib([3.0] * 30, ref_nees=1.0)["times"]["end_of_trajectory"]
    assert t["mean_nll"] - t["reference_mean_nll"] == pytest.approx(1.0)
    assert t["nll_ok"]
    t = _calib([3.1] * 30, ref_nees=1.0)["times"]["end_of_trajectory"]   # 1.05 nat > margin
    assert not t["nll_ok"] and not t["pass"]


def test_layer3_requires_both_times():
    est = [_dataset(3.0, 0.3) for _ in range(30)]                # fine at dropout end, inflated at the end
    ref = [_dataset(3.0, 0.3) for _ in range(30)]
    scen = [(f"s{i}", SPEC, None, np.zeros((N, 3))) for i in range(30)]
    c = V.layer3_calibration(est, ref, scen, TH)
    assert c["times"]["end_of_dropout"]["pass"] and not c["times"]["end_of_trajectory"]["pass"]
    assert not c["pass"] and c["wording"] == "calibration inconsistency detected (K=30)"


# ---------------------------------------------------------------- schema

@pytest.mark.parametrize("pose,cov,msg", [
    ([0, 0], I3, "shape"),
    ([0, 0, 0], np.eye(2), "shape"),
    ([0, np.nan, 0], I3, "non-finite"),
    ([0, 0, 0], np.full((3, 3), np.inf), "non-finite"),
    ([0, 0, 0], [[1, 0.1, 0], [0, 1, 0], [0, 0, 1]], "symmetric"),
    ([0, 0, 0], np.diag([1, -1, 1]), "positive definite"),
    ([0, 0, 0], np.zeros((3, 3)), "positive definite"),   # ||P|| = 0 passes symmetry, must fail Cholesky
])
def test_check_matrix_rejects(pose, cov, msg):
    assert msg in V.check_matrix(pose, cov)


def test_check_matrix_symmetry_tolerance():
    P = I3.copy()
    P[0, 1] = 1e-12                       # ||P - P^T|| = 1.41e-12 <= 1e-9 * ||P|| = 1.73e-9
    assert V.check_matrix([0, 0, 0], P) is None


# ---------------------------------------------------------------- accuracy

def test_accuracy_bound_hand():
    truth = np.array([[0, 0, 0], [1, 2, 0]], float)
    scen = [("s", {"gps_schedule": None}, None, truth)]
    est = [(np.array([[0, 0, 0], [1, 0, 0]], float), None)]          # errors 0 and 2 -> ATE sqrt(2) = 1.414
    good_ref = [(np.array([[0, 0, 0], [1, 1.9, 0]], float), None)]   # errors 0, 0.1 -> ATE sqrt(0.005)
    a = V.layer2_accuracy(est, good_ref, scen, TH)
    assert a["ate_m"] == pytest.approx(math.sqrt(2))
    assert a["reference_ate_m"] == pytest.approx(math.sqrt(0.005))
    assert a["bound_m"] == pytest.approx(0.25 + 1.5 * math.sqrt(0.005))   # 0.356 < 1.414 -> fail
    assert not a["pass"]
    poor_ref = [(np.array([[0, 0, 0], [1, 0.2, 0]], float), None)]   # errors 0, 1.8 -> ATE 1.273
    a = V.layer2_accuracy(est, poor_ref, scen, TH)
    assert a["bound_m"] == pytest.approx(0.25 + 1.5 * math.sqrt(1.62)) and a["pass"]   # 2.159 >= 1.414


# ---------------------------------------------------------------- route (layer 4)

def _route(A, B, chosen, stated):
    succ = {"A": list(map(bool, A)), "B": list(map(bool, B))}
    coll = {"A": [False] * len(A), "B": [False] * len(B)}
    return V.layer4_route({"route": chosen, "success_prob": stated, "reasoning": ""}, succ, coll, {}, TH, len(A))


def test_route_clear_pass_and_fail():
    r = _route([0] * 40, [1] * 40, "B", 1.0)
    assert r["regret_outcome"] == "pass" and r["regret_ci95"] == [0.0, 0.0] and r["pass"]
    assert r["diff_ci95"] == [1.0, 1.0]
    r = _route([0] * 40, [1] * 40, "A", 0.0)
    assert r["regret_outcome"] == "fail" and r["regret"] == 1.0 and not r["pass"]


def test_bootstrap_is_paired():
    # Identical outcomes on both routes: every resampled difference is exactly 0, which only holds if the
    # same trial indices are resampled for A and B.
    A = [1, 0, 1, 1, 0, 1, 0, 1, 1, 1]
    bs = V.paired_bootstrap({"A": A, "B": A}, "A")
    assert bs["diff_ci95"] == [0.0, 0.0] and bs["regret_ci95"] == [0.0, 0.0]


def test_route_inconclusive_close_rates():
    rng = np.random.default_rng(0)
    A, B = (rng.random(30) < 0.7).astype(int), (rng.random(30) < 0.75).astype(int)
    r = _route(A, B, "A", float(np.mean(A)))
    lo, hi = r["regret_ci95"]
    assert r["regret_outcome"] == "inconclusive" and lo <= TH["regret_delta"] < hi and not r["pass"]


def test_stated_probability_gates_layer4():
    # regret passes, but stated 0.80 vs measured 1.00 differs by 0.20 > 0.15 -> layer 4 fails (repo rule)
    r = _route([0] * 40, [1] * 40, "B", 0.80)
    assert r["regret_outcome"] == "pass" and not r["stated_check"]["pass"] and not r["pass"]
    assert r["stated_check"]["abs_diff"] == pytest.approx(0.20)
    assert _route([0] * 40, [1] * 40, "B", 0.86)["pass"]          # 0.14 <= 0.15


# ---------------------------------------------------------------- reward

@pytest.mark.parametrize("e,a,c,r,want", [
    (True, True, False, False, 0.30),    # SPEC example
    (True, True, True, False, 0.70),     # inconclusive route counts as not passed
    (True, True, True, True, 1.00),
    (False, False, False, False, 0.0),
    (True, False, True, False, 0.50),
])
def test_reward_weights(e, a, c, r, want):
    rep = {"layers": {"execution": {"pass": e}, "accuracy": {"pass": a}, "calibration": {"pass": c},
                      "route": {"pass": r}}}
    assert V.reward(rep) == pytest.approx(want)
