"""Spec v3.2 additions: close_routes, unknown_noise, and route feedback without measured rates."""
import re

import numpy as np
import pytest

from simcheck import broken, generator, reference, sim, verifier as V
from simcheck.sandbox import InProcessRunner
from simcheck.tests.test_verifier import layers23


def _quiet(*_):
    pass


@pytest.mark.parametrize("task", ["close_routes", "unknown_noise"])
def test_reference_accepted_dev(task):
    rep = V.verify(InProcessRunner(reference), task, "dev", progress=_quiet)
    assert rep["accepted"] and rep["reward"] == 1.0, rep["feedback"]


def test_close_routes_are_close_dev():
    rr = V.reference_route_trials("close_routes", "dev", V.CONFIG["route_trials"]["T"])
    rates = {r: float(np.mean(v)) for r, v in rr["success"].items()}
    assert 0.05 <= abs(rates["B"] - rates["A"]) <= 0.15, rates


def test_nominal_noise_fails_calibration_on_unknown_noise():
    a, c = layers23(broken.VARIANTS["nominal_noise"], "unknown_noise")
    assert not c["pass"], c


def test_unknown_noise_scale_is_hidden_and_varies():
    spec = V.task_spec("unknown_noise")
    assert spec["odom_noise_scale_range"] == [0.5, 3.0]
    assert spec["sensor_noise_std"]["odom"] == list(generator.ScenarioConfig().odom_noise)  # nominal only
    ks = [generator.odom_noise_scale(spec, s) for s in range(200)]
    assert 0.5 <= min(ks) < 0.7 and 2.8 < max(ks) <= 3.0
    assert generator.true_spec(V.task_spec("standard"), 1) is not None
    assert generator.odom_noise_scale(V.task_spec("standard"), 1) == 1.0
    assert "between 0.5x and 3x the nominal value" in generator.task_md(spec)


def test_reference_scale_estimate_tracks_truth_dev():
    errs = []
    for sid, spec, meas, _ in V.load_scenarios("unknown_noise", "dev", 6):
        est = reference.make_estimator(spec)
        for m in meas:
            est.step(m)
        k_true = generator.odom_noise_scale(spec, V.seed_base("dev") + int(sid[-3:]))
        errs.append(abs(np.log(est.k / k_true)))
    assert np.median(errs) < np.log(1.5), errs


def test_route_feedback_has_no_measured_rates():
    rep = {"layers": {"execution": {"pass": True}, "accuracy": {"pass": True}, "calibration": {"K": 30, "times": {}},
                      "route": {"regret_outcome": "fail", "chosen": "A", "T": 200, "delta": 0.05, "regret": 0.31,
                                "regret_ci95": [0.25, 0.37], "routes": {"A": {"success_rate": 0.685},
                                                                        "B": {"success_rate": 0.995}},
                                "stated_check": {"pass": False, "stated": 0.95, "estimated": 0.685,
                                                 "estimated_wilson95": [0.62, 0.74], "tolerance": 0.15}}}}
    fb = " ".join(V.feedback(rep))
    assert "too high" in fb
    for n in ("0.685", "0.995", "0.31", "0.62", "0.74", "0.25", "0.37"):
        assert n not in fb, fb
    assert not re.search(r"0\.\d{3}", fb), fb


@pytest.mark.parametrize("name", ["cov_x0.1", "cov_x10", "wrong_odom_frame", "missing_prior", "always_A"])
def test_v31_variants_still_registered(name):
    assert name in broken.VARIANTS


def test_true_spec_used_in_route_trials():
    spec = V.task_spec("unknown_noise")
    a = sim.run_episode(generator.true_spec(spec, 5), "B", 5, oracle=True)["measurements"]
    b = sim.run_episode(spec, "B", 5, oracle=True)["measurements"]
    k = generator.odom_noise_scale(spec, 5)
    assert k != 1.0 and a[3]["odom"] != b[3]["odom"]
