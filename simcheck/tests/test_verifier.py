"""Tests that must pass before trusting model results (spec v3.1 'Tests')."""
import json
from pathlib import Path

import numpy as np
import pytest

from simcheck import broken, generator, reference, sim, verifier as V
from simcheck.sandbox import InProcessRunner, RemoteEstimator, SubprocessRunner

TH = V.CONFIG["thresholds"]
K = TH["calibration_K"]
ROOT = Path(__file__).resolve().parents[2]


def layers23(mod, task, seed_set="dev"):
    scen = V.load_scenarios(task, seed_set, K)
    est = V.run_datasets(InProcessRunner(mod), scen)
    ref = V.reference_datasets(task, seed_set, K)
    return V.layer2_accuracy(est, ref, scen, TH), V.layer3_calibration(est, ref, scen, TH)


# ---- reference meets the predefined criteria on dev seeds

@pytest.mark.parametrize("task", list(V.TASKS))
def test_reference_passes_dev(task):
    a, c = layers23(reference, task)
    assert a["pass"] and c["pass"], (a, c)


def test_reference_accepted_with_route_dev():
    rep = V.verify(InProcessRunner(reference), "standard", "dev", progress=lambda *_: None)
    assert rep["accepted"] and rep["reward"] == 1.0, rep["feedback"]


# ---- broken variants

def test_overconfident_fails_calibration():
    a, c = layers23(broken.VARIANTS["cov_x0.1"], "standard")
    assert a["pass"] and not c["pass"]
    assert not any(t["overconfidence_ok"] for t in c["times"].values())


def test_underconfident_fails_calibration():
    a, c = layers23(broken.VARIANTS["cov_x10"], "standard")
    assert a["pass"] and not c["pass"]
    assert not any(t["underconfidence_ok"] for t in c["times"].values())


def test_wrong_odom_frame_fails_accuracy():
    a, _ = layers23(broken.VARIANTS["wrong_odom_frame"], "standard")
    assert not a["pass"]


def test_missing_prior_fails_on_prior_needed():
    a, c = layers23(broken.VARIANTS["missing_prior"], "prior_needed")
    assert not (a["pass"] and c["pass"])


def test_always_A_fails_route():
    rep = V.verify(InProcessRunner(broken.VARIANTS["always_A"]), "standard", "dev", progress=lambda *_: None)
    r = rep["layers"]["route"]
    assert r["regret_outcome"] == "fail" and not rep["accepted"]
    assert r["routes"]["B"]["success_rate"] - r["routes"]["A"]["success_rate"] > 0.2  # B clearly better


# ---- indexing: step k never receives step k+1 data

class _Spy:
    def __init__(self, spec, truth_ref):
        self.inner, self.seen, self.truth_ref = reference.Estimator(spec), [], truth_ref

    def step(self, m):
        self.seen.append(m)
        return self.inner.step(m)


def test_step_k_never_sees_future():
    cfg = generator.ScenarioConfig(odom_noise=(1e-9,) * 3, actuator_noise=(1e-9, 1e-9), range_noise=1e-9,
                                   bearing_noise=1e-9, gps_noise=1e-9, gps_dropout=None)
    spec = generator.build_spec(cfg)
    spy = _Spy(spec, None)
    ep = sim.run_episode(spec, "B", 7, estimator=spy)
    truth = ep["truth"]
    assert [m["step"] for m in spy.seen] == list(range(len(truth)))
    for k, m in enumerate(spy.seen):
        assert m["t"] == pytest.approx(k * spec["dt"])
        assert (m["odom"] is None) == (k == 0)
        assert np.allclose([m["gps"]["x"], m["gps"]["y"]], truth[k, :2], atol=1e-6)  # step k data is truth k
        if k + 1 < len(truth):
            assert not np.allclose(truth[k, :2], truth[k + 1, :2], atol=1e-3)  # and the robot did move on


# ---- fresh estimator per episode

LEAKY = '''
_COUNT = [0]
def make_estimator(spec):
    _COUNT[0] += 1
    off = 0.0 if _COUNT[0] == 1 else 1e-3   # state from earlier episodes leaks in
    class E:
        def step(self, m):
            import numpy as np
            return [off, 0.0, 0.0], (np.eye(3) * 0.1).tolist()
    return E()
def choose_route(spec):
    return {"route": "B", "success_prob": 0.9, "reasoning": "x"}
'''


def _solution_from_reference(tmp_path, extra=""):
    src = (ROOT / "simcheck" / "reference.py").read_text()
    src += '\n\ndef choose_route(public_spec):\n    return {"route": "B", "success_prob": 0.97, "reasoning": "fixed"}\n' + extra
    p = tmp_path / "solution.py"
    p.write_text(src)
    return p


def test_leaked_state_fails_execution(tmp_path):
    p = tmp_path / "solution.py"
    p.write_text(LEAKY)
    r = SubprocessRunner(p)
    try:
        scen = V.load_scenarios("standard", "dev", 3)
        first = V.run_datasets(r, scen[:1])[0]
        V.run_datasets(r, scen[1:])
        again = V.run_datasets(r, scen[:1])[0]
        assert not np.allclose(first[0], again[0], atol=1e-6, rtol=0)  # what the L0 check compares
    finally:
        r.close()


def test_fresh_estimator_each_trial_in_sim():
    """Running trial i first must not change trial i+1's outcome."""
    spec = V.task_spec("standard")
    solo = sim.run_episode(spec, "A", 42, estimator=reference.make_estimator(spec))
    sim.run_episode(spec, "B", 41, estimator=reference.make_estimator(spec))
    after = sim.run_episode(spec, "A", 42, estimator=reference.make_estimator(spec))
    assert np.array_equal(solo["est"], after["est"]) and solo["outcome"] == after["outcome"]


# ---- sandbox protocol end to end (subprocess fallback; Docker path is identical over stdin/stdout)

def test_subprocess_runner_matches_in_process(tmp_path):
    p = _solution_from_reference(tmp_path, extra='\nprint("agent noise on stdout must not break the protocol")\n')
    r = SubprocessRunner(p)
    try:
        scen = V.load_scenarios("standard", "dev", 2)
        sub = V.run_datasets(r, scen)
        inp = V.run_datasets(InProcessRunner(reference), scen)
        for (e1, c1), (e2, c2) in zip(sub, inp):
            assert np.allclose(e1, e2, atol=1e-9) and np.allclose(c1, c2, rtol=1e-9)
        assert r.call({"cmd": "choose_route", "spec": scen[0][1]})["result"]["route"] == "B"
    finally:
        r.close()


def test_crashing_solution_fails_execution(tmp_path):
    p = tmp_path / "solution.py"
    p.write_text("def make_estimator(spec):\n    raise ValueError('boom')\ndef choose_route(spec):\n    return {}\n")
    rep = V.verify(SubprocessRunner(p), "standard", "dev", progress=lambda *_: None)
    assert not rep["layers"]["execution"]["pass"] and rep["reward"] == 0.0
    assert "boom" in rep["feedback"][0]


def test_schema_checks():
    good = np.diag([0.1, 0.2, 0.01])
    assert V.check_matrix([0, 0, 0], good) is None
    assert "symmetric" in V.check_matrix([0, 0, 0], good + np.triu(np.ones((3, 3)), 1) * 1e-3)
    assert "positive definite" in V.check_matrix([0, 0, 0], np.diag([0.1, -0.2, 0.01]))
    assert "non-finite" in V.check_matrix([0, np.nan, 0], good)
    assert "shape" in V.check_matrix([0, 0], good)


def test_stats_helpers():
    lo, hi = V.wilson(30, 30)
    assert hi == 1.0 and 0.88 < lo < 0.89  # perfect coverage is not a failure by itself
    from scipy import stats
    assert stats.binom.cdf(30, 30, 0.95) >= 0.01
    assert stats.binom.cdf(24, 30, 0.95) < 0.01
    bs = V.paired_bootstrap({"A": [1] * 50 + [0] * 50, "B": [1] * 100}, "B")
    assert bs["regret_ci95"] == [0.0, 0.0] and bs["diff_B_minus_A"] == 0.5


def test_reward_example():
    rep = {"layers": {"execution": {"pass": True}, "accuracy": {"pass": True},
                      "calibration": {"pass": False}, "route": {"pass": False}}}
    assert V.reward(rep) == pytest.approx(0.30)


def test_feedback_has_no_host_paths(tmp_path):
    p = tmp_path / "solution.py"
    p.write_text("def make_estimator(spec):\n    raise ValueError('boom')\ndef choose_route(spec):\n    return {}\n")
    rep = V.verify(SubprocessRunner(p), "standard", "dev", progress=lambda *_: None)
    text = "\n".join(rep["feedback"])
    assert str(tmp_path) not in text and "/Users/" not in text and "/var/folders" not in text
    assert 'File "solution.py"' in text


def test_total_time_budget():
    import time
    from simcheck.sandbox import Budgeted, RunnerError

    class Slow:
        isolation = "test"
        def call(self, req, timeout=None):
            time.sleep(0.2)
            return {"ok": True}
        def close(self):
            pass
    b = Budgeted(Slow(), 0.3)
    b.call({"cmd": "make"}); b.call({"cmd": "make"})
    with pytest.raises(RunnerError, match="total time limit"):
        b.call({"cmd": "make"})
