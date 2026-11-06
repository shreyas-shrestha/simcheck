"""Docker sandbox guarantees. Skipped without Docker; set SIMCHECK_REQUIRE_DOCKER=1 to make that a failure."""
import os

import numpy as np
import pytest

from simcheck import reference, verifier as V
from simcheck.sandbox import AgentError, DockerRunner, InProcessRunner, RunnerError

if not DockerRunner.available():
    if os.environ.get("SIMCHECK_REQUIRE_DOCKER") == "1":
        pytest.fail("SIMCHECK_REQUIRE_DOCKER=1 but Docker or the simcheck-agent image is unavailable",
                    pytrace=False)
    pytest.skip("Docker or the simcheck-agent image is unavailable", allow_module_level=True)

PROBE = '''
import os, socket, json
def make_estimator(spec):
    class E:
        def step(self, m):
            return [0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    return E()
def choose_route(spec):
    out = {}
    try:
        socket.create_connection(("1.1.1.1", 53), timeout=3); out["net"] = "open"
    except OSError as e:
        out["net"] = "blocked"
    out["env"] = sorted(k for k in os.environ if k not in ("PATH", "HOSTNAME", "HOME", "LANG", "GPG_KEY",
                                                           "PYTHON_VERSION", "PYTHON_SHA256"))
    try:
        open("/opt/simcheck/agent_worker.py", "a").write("#")
        out["worker"] = "writable"
    except OSError:
        out["worker"] = "read-only"
    out["hidden"] = [p for p in ("/hidden", "/scenarios") if os.path.exists(p)]
    return {"route": "B", "success_prob": 0.5, "reasoning": json.dumps(out)}
'''

HANG = '''
def make_estimator(spec):
    while True:
        pass
def choose_route(spec):
    return {"route": "B", "success_prob": 0.5, "reasoning": ""}
'''


def _sol(tmp_path, src):
    p = tmp_path / "solution.py"
    p.write_text(src)
    return p


def test_isolation_properties(tmp_path, monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "not-a-real-key")
    r = DockerRunner(_sol(tmp_path, PROBE))
    try:
        import json
        out = json.loads(r.call({"cmd": "choose_route", "spec": {}}, timeout=60)["result"]["reasoning"])
    finally:
        r.close()
    assert out["net"] == "blocked"
    assert out["env"] == []
    assert out["worker"] == "read-only"
    assert out["hidden"] == []


def test_hang_is_killed(tmp_path):
    r = DockerRunner(_sol(tmp_path, HANG), call_timeout=5)
    try:
        with pytest.raises(RunnerError, match="timeout"):
            r.call({"cmd": "make", "spec": {}})
    finally:
        r.close()


def test_matches_in_process(tmp_path):
    src = (V.ROOT / "simcheck" / "reference.py").read_text()
    src += '\n\ndef choose_route(public_spec):\n    return {"route": "B", "success_prob": 0.97, "reasoning": "x"}\n'
    r = DockerRunner(_sol(tmp_path, src), call_timeout=60)
    try:
        scen = V.load_scenarios("standard", "dev", 2)
        dock = V.run_datasets(r, scen)
    finally:
        r.close()
    inp = V.run_datasets(InProcessRunner(reference), scen)
    for (e1, c1), (e2, c2) in zip(dock, inp):
        assert np.allclose(e1, e2, atol=1e-9) and np.allclose(c1, c2, rtol=1e-9)


def test_agent_exception_is_reported(tmp_path):
    r = DockerRunner(_sol(tmp_path, "def make_estimator(s):\n    raise ValueError('boom')\n"))
    try:
        with pytest.raises(AgentError, match="boom"):
            r.call({"cmd": "make", "spec": {}}, timeout=60)
    finally:
        r.close()
