"""Runs agent code behind a process boundary and speaks the agent_worker.py JSON-lines protocol.

DockerRunner      real isolation: --network none, read-only, only solution.py and a copy of public/
                  mounted read-only, no env vars. Hidden truth never leaves the parent process.
SubprocessRunner  fallback when Docker is unavailable. A separate `python -I` process with an empty
                  environment and a temp cwd. This is NOT isolation and every report says so.
InProcessRunner   trusted code only (reference, broken variants). Same JSON round-trip, no process.
"""
import json
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
from collections import deque
from pathlib import Path

WORKER = Path(__file__).resolve().parent / "agent_worker.py"
IMAGE = "simcheck-agent:latest"


class RunnerError(Exception):
    """The sandbox died, timed out, or spoke garbage."""


class AgentError(Exception):
    """The agent's code raised; the worker is still alive."""


def _roundtrip(x):
    return json.loads(json.dumps(x))


class _ProcRunner:
    isolation = "?"

    def __init__(self, argv, env, cwd, call_timeout=30.0):
        self.call_timeout = call_timeout
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                  env=env, cwd=cwd, text=True, bufsize=1)
        self.out, self.err = queue.Queue(), deque(maxlen=200)
        threading.Thread(target=self._pump_out, daemon=True).start()
        threading.Thread(target=self._pump_err, daemon=True).start()

    def _pump_out(self):
        for line in self.p.stdout:
            self.out.put(line)
        self.out.put(None)

    def _pump_err(self):
        for line in self.p.stderr:
            self.err.append(line)

    def stderr_tail(self, n=30):
        return "".join(list(self.err)[-n:])

    def call(self, req, timeout=None):
        try:
            self.p.stdin.write(json.dumps(req) + "\n")
            self.p.stdin.flush()
        except (BrokenPipeError, OSError):
            raise RunnerError(f"sandbox exited (rc={self.p.poll()}): {self.stderr_tail()}")
        try:
            line = self.out.get(timeout=timeout or self.call_timeout)
        except queue.Empty:
            self.close()
            raise RunnerError(f"timeout after {timeout or self.call_timeout}s on '{req['cmd']}'")
        if line is None:
            raise RunnerError(f"sandbox exited (rc={self.p.poll()}): {self.stderr_tail()}")
        try:
            resp = json.loads(line)
        except json.JSONDecodeError:
            raise RunnerError(f"malformed protocol line: {line[:200]!r}")
        if not resp.get("ok"):
            raise AgentError(resp.get("error", "unknown error") + "\n" + resp.get("trace", ""))
        return resp

    def close(self):
        if self.p.poll() is None:
            self.p.kill()
            self.p.wait()


class SubprocessRunner(_ProcRunner):
    isolation = "NONE: subprocess fallback (Docker unavailable); not a security boundary"

    def __init__(self, solution_path, call_timeout=30.0):
        self.tmp = tempfile.mkdtemp(prefix="simcheck_sub_")
        sol = Path(self.tmp) / "solution.py"
        shutil.copy(solution_path, sol)
        env = {"PATH": "/usr/bin:/bin", "HOME": self.tmp}
        super().__init__([sys.executable, "-I", str(WORKER), str(sol)], env, self.tmp, call_timeout)

    def close(self):
        super().close()
        shutil.rmtree(self.tmp, ignore_errors=True)


class DockerRunner(_ProcRunner):
    isolation = "docker --network none, read-only rootfs, public/ copy mounted ro, no env vars"

    def __init__(self, solution_path, public_dir=None, image=IMAGE, call_timeout=30.0):
        self.tmp = tempfile.mkdtemp(prefix="simcheck_dock_")
        agent = Path(self.tmp) / "agent"
        agent.mkdir()
        shutil.copy(solution_path, agent / "solution.py")
        shutil.copy(WORKER, agent / "agent_worker.py")
        argv = ["docker", "run", "--rm", "-i", "--network", "none", "--read-only", "--tmpfs", "/tmp",
                "--memory", "2g", "--cpus", "2", "--pids-limit", "256", "--user", "65534:65534",
                "-v", f"{agent}:/agent:ro"]
        if public_dir is not None:
            pub = Path(self.tmp) / "public"
            shutil.copytree(public_dir, pub)
            argv += ["-v", f"{pub}:/public:ro"]
        argv += [image, "python", "-I", "/agent/agent_worker.py", "/agent/solution.py"]
        super().__init__(argv, {"PATH": "/usr/local/bin:/usr/bin:/bin"}, self.tmp, call_timeout)

    @staticmethod
    def available(image=IMAGE):
        try:
            ok = subprocess.run(["docker", "image", "inspect", image], capture_output=True, timeout=10).returncode == 0
            return ok
        except (OSError, subprocess.TimeoutExpired):
            return False

    def close(self):
        super().close()
        shutil.rmtree(self.tmp, ignore_errors=True)


class InProcessRunner:
    """For trusted modules (reference.py, broken.py variants). Mirrors the worker's JSON round-trip."""
    isolation = "in-process (trusted code only)"

    def __init__(self, module):
        self.mod, self.est = module, None

    def call(self, req, timeout=None):
        req = _roundtrip(req)
        try:
            if req["cmd"] == "make":
                self.est = self.mod.make_estimator(req["spec"])
                return {"ok": True}
            if req["cmd"] == "step":
                pose, cov = self.est.step(req["m"])
                import numpy as np
                return _roundtrip({"ok": True, "pose": np.asarray(pose, float).tolist(),
                                   "cov": np.asarray(cov, float).tolist()})
            if req["cmd"] == "choose_route":
                r = self.mod.choose_route(req["spec"])
                return _roundtrip({"ok": True, "result": {"route": r["route"], "success_prob": float(r["success_prob"]),
                                                          "reasoning": str(r.get("reasoning", ""))}})
            if req["cmd"] == "build_graph":
                return {"ok": True, "summary": None}
        except Exception as e:
            raise AgentError(f"{type(e).__name__}: {e}")
        raise AgentError(f"unknown cmd {req['cmd']}")

    def close(self):
        pass


def make_runner(solution_path, public_dir=None, prefer_docker=True, call_timeout=30.0):
    if prefer_docker and DockerRunner.available():
        return DockerRunner(solution_path, public_dir, call_timeout=call_timeout)
    return SubprocessRunner(solution_path, call_timeout=call_timeout)


class RemoteEstimator:
    """Adapter so sim.run_episode can drive any runner. Creating one sends 'make' (fresh state)."""

    def __init__(self, runner, spec):
        self.r = runner
        self.r.call({"cmd": "make", "spec": spec})

    def step(self, m):
        resp = self.r.call({"cmd": "step", "m": m})
        return resp["pose"], resp["cov"]
