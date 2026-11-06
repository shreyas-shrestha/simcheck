"""Isolation and protocol tests for the runners in sandbox.py, run against the real Docker container.

Docker tests skip when the daemon or image is missing, unless SIMCHECK_REQUIRE_DOCKER=1, in which case
collection fails loudly. Build the image first:  docker build -t simcheck-agent:latest .
"""
import ast
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from simcheck import reference, verifier as V
from simcheck.sandbox import (AgentError, DockerRunner, InProcessRunner, RunnerError, SubprocessRunner,
                              check_public_dir, make_runner)

HERE = Path(__file__).resolve().parent
FIX = HERE / "fixtures"
BAD = FIX / "bad"
DUMMY = FIX / "dummy_solution.py"
PUBLIC = FIX / "public"
ROOT = HERE.parents[1]

HAVE_DOCKER = DockerRunner.available()
if os.environ.get("SIMCHECK_REQUIRE_DOCKER") == "1" and not HAVE_DOCKER:
    raise RuntimeError("SIMCHECK_REQUIRE_DOCKER=1 but the Docker daemon or simcheck-agent:latest image is unavailable")
docker = pytest.mark.skipif(not HAVE_DOCKER, reason="Docker daemon or simcheck-agent:latest image unavailable")

SPEC = {"initial_pose": {"mean": [0, 0, 0], "cov_diag_variances": [0.01, 0.01, 0.001]},
        "sensor_noise_std": {"odom": [0.05, 0.05, 0.01]}}


def m(k, odom=None):
    return {"step": k, "t": 0.5 * k, "odom": odom, "gps": None, "landmarks": []}


def running(name):
    return name in subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True).stdout


def _agent_error(path, **kw):
    r = DockerRunner(path, **kw)
    try:
        r.call({"cmd": "make", "spec": SPEC})
        with pytest.raises(AgentError) as ei:
            r.call({"cmd": "step", "m": m(0)})
        return str(ei.value)
    finally:
        r.close()


# ---------------------------------------------------------------- protocol over Docker

@docker
def test_dummy_roundtrip_and_stdout_noise():
    r = DockerRunner(DUMMY, PUBLIC)
    try:
        r.call({"cmd": "make", "spec": SPEC})
        r.call({"cmd": "step", "m": m(0)})
        r.call({"cmd": "step", "m": m(1, {"dx": 1.0, "dy": 0.0, "dtheta": math.pi / 2})})
        resp = r.call({"cmd": "step", "m": m(2, {"dx": 1.0, "dy": 0.0, "dtheta": 0.0})})
        assert resp["pose"] == pytest.approx([1.0, 1.0, math.pi / 2])   # body-frame compose
        assert resp["cov"][0][0] == pytest.approx(0.01 + 2 * 0.05 ** 2)
        assert r.call({"cmd": "choose_route", "spec": SPEC})["result"] == \
            {"route": "B", "success_prob": 0.8, "reasoning": "landmarks on B"}
        assert "agent noise" in r.stderr_tail()   # import-time print went to stderr, not the protocol
    finally:
        r.close()


@docker
def test_docker_matches_in_process_reference(tmp_path):
    src = (ROOT / "simcheck" / "reference.py").read_text()
    src += '\n\ndef choose_route(public_spec):\n    return {"route": "B", "success_prob": 0.97, "reasoning": "fixed"}\n'
    p = tmp_path / "solution.py"
    p.write_text(src)
    scen = V.load_scenarios("standard", "dev", 2)
    r = DockerRunner(p, ROOT / "scenarios" / scen[0][0] / "public")
    try:
        dock = V.run_datasets(r, scen)
    finally:
        r.close()
    inp = V.run_datasets(InProcessRunner(reference), scen)
    for (e1, c1), (e2, c2) in zip(dock, inp):
        assert np.allclose(e1, e2, atol=1e-9) and np.allclose(c1, c2, rtol=1e-9)


@docker
def test_leaked_global_state_is_visible_to_the_determinism_check():
    r = DockerRunner(BAD / "global_state.py")
    try:
        r.call({"cmd": "make", "spec": SPEC})
        first = r.call({"cmd": "step", "m": m(0)})["pose"]
        r.call({"cmd": "make", "spec": SPEC})
        again = r.call({"cmd": "step", "m": m(0)})["pose"]
        assert first != again   # module globals persist in one worker; verify() re-runs dataset 0 to catch this
    finally:
        r.close()


@docker
def test_agent_exception_keeps_worker_alive():
    r = DockerRunner(DUMMY)
    try:
        with pytest.raises(AgentError) as ei:
            r.call({"cmd": "step", "m": m(0)})   # no estimator made yet
        assert "AttributeError" in str(ei.value)
        r.call({"cmd": "make", "spec": SPEC})
        assert r.call({"cmd": "step", "m": m(0)})["pose"] == [0.0, 0.0, 0.0]
    finally:
        r.close()


@docker
def test_nan_reaches_schema_check():
    r = DockerRunner(BAD / "nan.py")
    try:
        r.call({"cmd": "make", "spec": SPEC})
        resp = r.call({"cmd": "step", "m": m(0)})
    finally:
        r.close()
    assert math.isnan(resp["pose"][0])
    assert "non-finite" in V.check_matrix(resp["pose"], resp["cov"])


@docker
def test_import_error_is_runner_error():
    r = DockerRunner(BAD / "crash_import.py")
    try:
        with pytest.raises(RunnerError) as ei:
            r.call({"cmd": "make", "spec": SPEC})
        assert "boom at import" in str(ei.value)
    finally:
        r.close()


@docker
def test_hard_exit_is_runner_error():
    r = DockerRunner(BAD / "hard_exit.py")
    try:
        with pytest.raises(RunnerError):
            r.call({"cmd": "make", "spec": SPEC})
    finally:
        r.close()


@docker
def test_pose2_return_is_execution_failure():
    # agent_worker serializes outside its try/except, so a contract-violating Pose2 return kills the worker.
    # The verifier still reports it as an execution failure with the TypeError in the message.
    r = DockerRunner(BAD / "returns_pose2.py")
    try:
        r.call({"cmd": "make", "spec": SPEC})
        with pytest.raises(RunnerError) as ei:
            r.call({"cmd": "step", "m": m(0)})
        assert "not JSON serializable" in str(ei.value)
    finally:
        r.close()


# ---------------------------------------------------------------- timeouts and cleanup

@docker
def test_timeout_kills_the_container():
    r = DockerRunner(BAD / "hang.py", call_timeout=3)
    r.call({"cmd": "make", "spec": SPEC})
    assert running(r.name)
    with pytest.raises(RunnerError, match="timeout"):
        r.call({"cmd": "step", "m": m(0)})
    time.sleep(1)
    assert not running(r.name)
    r.close()   # idempotent


@docker
def test_close_kills_running_container():
    r = DockerRunner(DUMMY)
    r.call({"cmd": "make", "spec": SPEC})
    assert running(r.name)
    r.close()
    time.sleep(1)
    assert not running(r.name)


# ---------------------------------------------------------------- isolation properties

@docker
def test_no_network():
    msg = _agent_error(BAD / "network.py")
    assert "OSError" in msg or "gaierror" in msg or "unreachable" in msg


@docker
def test_env_files_and_permissions():
    msg = _agent_error(BAD / "snoop.py", public_dir=PUBLIC)
    info = ast.literal_eval(msg.split("RuntimeError: ", 1)[1].split("\n", 1)[0])
    # Only base-image vars (GPG_KEY is python.org's public release-signing fingerprint, not a secret).
    assert set(info["env"]) <= {"PATH", "HOSTNAME", "LANG", "GPG_KEY", "PYTHON_VERSION", "PYTHON_SHA256", "HOME"}
    assert not [k for k in info["env"] if k != "GPG_KEY" and any(s in k for s in ("KEY", "TOKEN", "SECRET", "OPENAI", "GEMINI"))]
    assert info["files"] == ["/agent/", "/agent/solution.py", "/public/", "/public/spec.json"]   # worker is baked into the image
    assert info["uid"] == 65534 and not info["can_write_agent"] and not info["can_write_public"]


@docker
def test_no_capabilities_and_no_new_privs():
    info = ast.literal_eval(_agent_error(BAD / "privs.py").split("RuntimeError: ", 1)[1].split("\n", 1)[0])
    assert int(info["CapEff"], 16) == 0 and int(info["CapBnd"], 16) == 0
    assert info["NoNewPrivs"] == "1"


@docker
def test_tmp_is_size_limited():
    msg = _agent_error(BAD / "tmpfill.py")
    assert "No space left on device" in msg


# ---------------------------------------------------------------- no Docker needed

def test_refuses_public_dir_with_truth_or_symlinks(tmp_path):
    for bad in ("truth.json", "hidden"):
        pub = tmp_path / bad.replace(".", "_") / "public"
        pub.mkdir(parents=True)
        (pub / "spec.json").write_text("{}")
        (pub / bad).mkdir() if bad == "hidden" else (pub / bad).write_text("{}")
        with pytest.raises(RunnerError, match="refusing"):
            check_public_dir(pub)
    pub = tmp_path / "link" / "public"
    pub.mkdir(parents=True)
    (tmp_path / "secret.json").write_text("{}")
    (pub / "spec.json").symlink_to(tmp_path / "secret.json")
    with pytest.raises(RunnerError, match="refusing"):
        check_public_dir(pub)
    check_public_dir(PUBLIC)   # the clean fixture is fine


def test_require_docker_refuses_fallback(monkeypatch):
    monkeypatch.setenv("SIMCHECK_REQUIRE_DOCKER", "1")
    with pytest.raises(RunnerError, match="SIMCHECK_REQUIRE_DOCKER"):
        make_runner(DUMMY, prefer_docker=False)
    monkeypatch.delenv("SIMCHECK_REQUIRE_DOCKER")
    r = make_runner(DUMMY, prefer_docker=False)
    try:
        assert isinstance(r, SubprocessRunner) and r.isolation.startswith("NONE")
    finally:
        r.close()


# ---------------------------------------------------------------- Budgeted keeps the per-call limit (DECISIONS #25)

def _hung_step_seconds(runner_cls):
    from simcheck.sandbox import Budgeted, RemoteEstimator
    b = Budgeted(runner_cls(BAD / "hang.py", call_timeout=3), 60)   # as verify() wraps it, with a large total budget
    try:
        est = RemoteEstimator(b, SPEC)                               # make: no explicit timeout, like verify/sim
        t0 = time.perf_counter()
        with pytest.raises(RunnerError, match="timeout"):
            est.step(m(0))                                           # step: no explicit timeout either
        return time.perf_counter() - t0
    finally:
        b.close()


def test_budgeted_hung_step_fails_at_per_call_limit_subprocess():
    assert 2.5 < _hung_step_seconds(SubprocessRunner) < 10           # ~3 s, not the 60 s budget


@docker
def test_budgeted_hung_step_fails_at_per_call_limit_docker():
    assert 2.5 < _hung_step_seconds(DockerRunner) < 10


def test_budgeted_total_budget_still_caps_explicit_timeouts():
    from simcheck.sandbox import Budgeted
    b = Budgeted(SubprocessRunner(BAD / "hang.py", call_timeout=3), 2)
    try:
        b.call({"cmd": "make", "spec": SPEC})
        t0 = time.perf_counter()
        with pytest.raises(RunnerError):
            b.call({"cmd": "step", "m": m(0)}, timeout=900)          # like choose_route's 900 s: budget wins
        assert time.perf_counter() - t0 < 3
    finally:
        b.close()
