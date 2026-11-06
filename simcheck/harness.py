"""Model call -> code extraction -> isolated run -> verifier -> feedback, closed loop.

    python -m simcheck.harness --model astra --tasks standard trap_units --concurrency 2 --cost-cap 25

Layout: runs/<model>/<task>/<attempt>/{prompt.md, response.txt, call.json, model_id, settings.json,
        solution.py, report.json, log.txt}, runs/<model>/<task>/final_fresh/..., runs/<model>/<task>/summary.json.
Every step is cached: a rerun skips any attempt whose report.json exists and never re-calls the model
for an attempt whose response.txt exists. API keys stay in this process; the sandbox gets none.
"""
import argparse
import ast
import json
import os
import re
import ssl
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import certifi

from simcheck import generator, verifier as V
from simcheck.sandbox import make_runner

ROOT = Path(__file__).resolve().parent.parent


def _load_dotenv(path=ROOT / ".env"):
    """KEY=value lines from a gitignored .env; real environment variables win."""
    if path.exists():
        for line in path.read_text().splitlines():
            k, sep, v = line.strip().partition("=")
            if sep and not k.startswith("#"):
                os.environ.setdefault(k.strip().removeprefix("export "), v.strip().strip("\"'"))


_load_dotenv()
MODELS = json.loads((ROOT / "models.json").read_text())
N_ATTEMPTS = 3  # initial + 2 revisions
EXAMPLE_SEED_SET = "dev"  # the agent sees one dev-seed scenario; grading uses eval, then fresh

CONTRACT = '''\
Implement solution.py with exactly this interface:

def make_estimator(public_spec):
    """Return a NEW estimator object. One object per episode; the verifier creates a fresh one for every
    trial, so no state may persist across episodes (no module-level caches or globals)."""

class Estimator:
    def step(self, new_measurements):
        """new_measurements is a dict, always containing "step" (int) and "t" (seconds):
          step 0:  {"step": 0, "t": 0.0, "odom": None, "gps": {"x","y"} or None, "landmarks": [...]}
          step k:  {"step": k, "t": k*dt, "odom": {"dx","dy","dtheta"} for transition k-1 -> k,
                    "gps": {"x","y"} or None, "landmarks": [{"id","range","bearing"}, ...] (possibly empty)}
        Returns (pose [x, y, theta], covariance 3x3 in local tangent frame of pose)."""

def choose_route(public_spec):
    """Returns {"route": "A" or "B", "success_prob": float, "reasoning": str}."""

Incremental (e.g. iSAM2) or batch inside step() are both allowed. Optional:
build_graph(measurements, public_spec) -> (gtsam.NonlinearFactorGraph, gtsam.Values) for diagnostics.
Same measurements fed to two fresh estimators must give identical outputs (deterministic).
The robot is driven by a pure-pursuit controller that uses YOUR estimator's pose, so choose_route should
reflect the collision-free arrival probability with your own estimator in the loop.
Available in the sandbox: Python 3.12, numpy, scipy, gtsam. No network, no files except what you write.
Answer with ONE ```python fenced block containing the complete solution.py.'''

SYSTEM = "You are an expert in robot state estimation and GTSAM. You write complete, correct, runnable Python."


# ---------------------------------------------------------------- cost cap

class CostCapReached(Exception):
    pass


class CostTracker:
    def __init__(self, cap_usd, ledger):
        self.cap, self.ledger, self.lock = cap_usd, Path(ledger), threading.Lock()
        self.spent = sum(json.loads(l)["usd"] for l in self.ledger.read_text().splitlines()) if self.ledger.exists() else 0.0
        self.reserved = 0.0  # worst-case cost of calls currently in flight

    def check(self):
        with self.lock:
            if self.spent >= self.cap:
                raise CostCapReached(f"cost cap ${self.cap:.2f} reached (spent ${self.spent:.2f})")

    def reserve(self, usd):
        """Hold a call's worst-case cost before making it, so concurrent calls can't jointly overshoot the cap."""
        with self.lock:
            if self.spent + self.reserved + usd > self.cap:
                raise CostCapReached(f"cost cap ${self.cap:.2f}: spent ${self.spent:.2f}, in flight "
                                     f"${self.reserved:.2f}, this call up to ${usd:.2f}")
            self.reserved += usd
        return usd

    def release(self, usd):
        with self.lock:
            self.reserved -= usd

    def add(self, usd, tag):
        with self.lock:
            self.spent += usd
            self.ledger.parent.mkdir(parents=True, exist_ok=True)
            with self.ledger.open("a") as f:
                f.write(json.dumps({"usd": usd, "tag": tag, "time": time.time()}) + "\n")


def price(model_cfg, usage):
    p = model_cfg["price_per_mtok"]
    if p["input"] is None or p["output"] is None:
        raise RuntimeError(f"set price_per_mtok for {model_cfg['model']} in models.json before real runs (cost cap)")
    return (usage["input_tokens"] * p["input"] + usage["output_tokens"] * p["output"]) / 1e6


def worst_case_usd(model_cfg, system, user):
    """Upper bound for one call: ~3 chars per input token (conservative) plus the full output token limit."""
    s = model_cfg["settings"]
    max_out = s.get("max_completion_tokens") or s.get("maxOutputTokens") or 32000
    return price(model_cfg, {"input_tokens": (len(system) + len(user)) // 3 + 100, "output_tokens": max_out})


# ---------------------------------------------------------------- providers

_SSL = ssl.create_default_context(cafile=certifi.where())  # python.org builds ship without CA certs


def _retry_wait(e, body, attempt):
    """Seconds to wait before retrying: the server's hint if it gives one, else exponential backoff."""
    hint = e.headers.get("retry-after") if e.headers else None
    m = re.search(r"try again in ([\d.]+)(ms|s)", body)
    if m:
        hint = float(m.group(1)) / (1000 if m.group(2) == "ms" else 1)
    try:
        return min(120.0, float(hint) + 2.0)
    except (TypeError, ValueError):
        return min(120.0, 2 ** attempt * 5.0)


def _post(url, body, headers, retries=10):
    data = json.dumps(body).encode()
    for i in range(retries):
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json", **headers})
        try:
            with urllib.request.urlopen(req, timeout=900, context=_SSL) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            msg = e.read().decode(errors="replace")
            if e.code in (429, 500, 502, 503, 504) and i < retries - 1 and "insufficient_quota" not in msg:
                time.sleep(_retry_wait(e, msg, i))
                continue
            raise RuntimeError(f"HTTP {e.code}: {msg[:500]}")
        except urllib.error.URLError:
            if i < retries - 1:
                time.sleep(min(60, 2 ** i * 5))
                continue
            raise


def call_openai(cfg, system, user):
    body = {"model": cfg["model"], "messages": [{"role": "system", "content": system},
                                                {"role": "user", "content": user}], **cfg["settings"]}
    r = _post("https://api.openai.com/v1/chat/completions", body,
              {"Authorization": f"Bearer {os.environ['OPENAI_API_KEY']}"})
    u = r.get("usage", {})
    return {"text": r["choices"][0]["message"]["content"], "model_id": r.get("model", cfg["model"]),
            "usage": {"input_tokens": u.get("prompt_tokens", 0), "output_tokens": u.get("completion_tokens", 0)},
            "raw_meta": {k: r.get(k) for k in ("id", "model", "system_fingerprint", "usage")}}


def call_gemini(cfg, system, user):
    body = {"systemInstruction": {"parts": [{"text": system}]},
            "contents": [{"role": "user", "parts": [{"text": user}]}], "generationConfig": cfg["settings"]}
    r = _post(f"https://generativelanguage.googleapis.com/v1beta/models/{cfg['model']}:generateContent", body,
              {"x-goog-api-key": os.environ["GEMINI_API_KEY"]})
    parts = r["candidates"][0]["content"]["parts"]
    u = r.get("usageMetadata", {})
    return {"text": "".join(p.get("text", "") for p in parts if not p.get("thought")),
            "model_id": r.get("modelVersion", cfg["model"]),
            "usage": {"input_tokens": u.get("promptTokenCount", 0),
                      "output_tokens": u.get("candidatesTokenCount", 0) + u.get("thoughtsTokenCount", 0)},
            "raw_meta": {k: r.get(k) for k in ("modelVersion", "responseId", "usageMetadata")}}


def fake_source(variant):
    """Self-contained solution.py text built from the reference, for offline closed-loop tests."""
    src = (ROOT / "simcheck" / "reference.py").read_text()
    route = "A" if variant == "always_A" else "B"
    src += (f'\n\ndef choose_route(public_spec):\n'
            f'    return {{"route": "{route}", "success_prob": 0.97, "reasoning": "fake model ({variant})"}}\n')
    if variant == "cov_x10":
        src += ("\n_Base = Estimator\n\nclass Estimator(_Base):\n    def step(self, m):\n"
                "        pose, P = super().step(m)\n        return pose, P * 10.0\n")
    return src


def call_fake(cfg, system, user, attempt):
    v = cfg["script"][min(attempt, len(cfg["script"])) - 1]
    return {"text": f"Here is my solution ({v}).\n```python\n{fake_source(v)}```\n", "model_id": cfg["model"],
            "usage": {"input_tokens": 0, "output_tokens": 0}, "raw_meta": {"variant": v}}


def call_model(cfg, system, user, attempt):
    if cfg["provider"] == "openai":
        return call_openai(cfg, system, user)
    if cfg["provider"] == "gemini":
        return call_gemini(cfg, system, user)
    if cfg["provider"] == "fake":
        return call_fake(cfg, system, user, attempt)
    raise ValueError(cfg["provider"])


# ---------------------------------------------------------------- prompts and extraction

def public_dir(task):
    sid = V.load_scenarios(task, EXAMPLE_SEED_SET, 1)[0][0]
    return ROOT / "scenarios" / sid / "public"


def build_prompt(task, prev_code=None, feedback=None, attempt=1):
    pub = public_dir(task)
    # Render task.md from spec.json with config.json "task_md_version" (default "v3.1") instead of reading the
    # cached scenarios/<id>/public/task.md, so the version setting always takes effect.
    task_text = generator.task_md(json.loads((pub / "spec.json").read_text()))
    parts = [task_text, "## spec.json\n```json\n" + (pub / "spec.json").read_text() + "\n```",
             "## measurements.json (one example episode; your estimator is graded on many others)\n```json\n"
             + (pub / "measurements.json").read_text() + "\n```", "## Contract\n" + CONTRACT]
    if prev_code is not None:
        parts += [f"## Revision {attempt - 1} of {N_ATTEMPTS - 1}: your previous solution.py\n```python\n{prev_code}\n```",
                  "## Verifier feedback on that solution\n" + "\n".join(f"- {f}" for f in feedback)
                  + "\n\nFix the failed checks. Return the complete revised solution.py."]
    return "\n\n".join(parts)


def _top_level_defs(code):
    try:
        return {n.name for n in ast.parse(code).body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
    except SyntaxError:
        return set()


def extract_code(text):
    """Prefer the last block that parses and defines BOTH make_estimator and choose_route (a later
    usage snippet or partial patch must not win). Otherwise: last parseable block defining make_estimator."""
    blocks = re.findall(r"```(?:python|py)?[ \t]*\n(.*?)```", text, flags=re.S)
    syntax_err = None
    for b in reversed(blocks):
        if {"make_estimator", "choose_route"} <= _top_level_defs(b):
            return b
    for b in reversed(blocks):
        if "def make_estimator" in b:
            try:
                ast.parse(b)
                return b
            except SyntaxError as e:
                syntax_err = syntax_err or e
    if syntax_err is not None:  # tell the model what to fix, not just that it failed
        raise ValueError(f"solution.py does not parse: {syntax_err.msg} at line {syntax_err.lineno}: "
                         f"{(syntax_err.text or '').strip()!r}")
    raise ValueError("no ```python block defining make_estimator found in the response")


def feedback_echo(code, prev_code, feedback):
    """Numbers quoted in the verifier feedback that first appear as literals in the revised code.

    The feedback reports measured rates (spec v3.1), so a revision can pass the stated-probability check
    by pasting them in. This flags it; it does not change the grade.
    """
    quoted = {m for f in feedback or [] for m in re.findall(r"(?<![\w.])\d+\.\d{2,}", f)}
    lit = lambda src: set(re.findall(r"(?<![\w.])\d+\.\d+", src or ""))
    new, old = lit(code), lit(prev_code)
    known = {float(v) for v in V.CONFIG["thresholds"].values() if isinstance(v, (int, float)) and not isinstance(v, bool)}
    return sorted(q for q in quoted if q in new and q not in old and float(q) not in known)


# ---------------------------------------------------------------- one (model, task) job

def _write(p, s):
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(s if isinstance(s, str) else json.dumps(s, indent=2, default=float))


def _verify(sol_path, task, seed_set, d, decision_only, prefer_docker):
    d.mkdir(parents=True, exist_ok=True)
    log = (d / "log.txt").open("a")
    runner = make_runner(sol_path, public_dir(task), prefer_docker=prefer_docker,
                         call_timeout=V.CONFIG["execution"]["call_timeout_s"])
    try:
        return V.verify(runner, task, seed_set, decision_only=decision_only,
                        progress=lambda s: (log.write(s + "\n"), log.flush()), out_path=d / "report.json")
    finally:
        runner.close()
        log.close()


def _say(msg):
    print(msg, flush=True)  # logs are redirected to files; flush so progress is visible live


def run_job(model_key, task, cost, decision_only=False, prefer_docker=True, echo=_say):
    cfg = MODELS[model_key]
    base = ROOT / "runs" / model_key / task
    rounds, code, rep = [], None, None
    for attempt in range(1, N_ATTEMPTS + 1):
        d = base / str(attempt)
        prev_code, prev_feedback = code, (rep or {}).get("feedback")
        if (d / "report.json").exists():
            rep = json.loads((d / "report.json").read_text())
            code = (d / "solution.py").read_text() if (d / "solution.py").exists() else code
        else:
            prompt = build_prompt(task, code, rep["feedback"] if rep else None, attempt) if attempt > 1 \
                else build_prompt(task)
            _write(d / "prompt.md", prompt)
            if (d / "response.txt").exists():
                text = (d / "response.txt").read_text()
            else:
                cost.check()
                held = cost.reserve(worst_case_usd(cfg, SYSTEM, prompt))
                t0 = time.perf_counter()
                try:
                    resp = call_model(cfg, SYSTEM, prompt, attempt)
                finally:
                    cost.release(held)
                usd = price(cfg, resp["usage"])
                cost.add(usd, f"{model_key}/{task}/{attempt}")
                text = resp["text"]
                _write(d / "response.txt", text)
                _write(d / "model_id", resp["model_id"] + "\n")
                _write(d / "settings.json", {"provider": cfg["provider"], "requested_model": cfg["model"],
                                             "settings": cfg["settings"], "system": SYSTEM})
                _write(d / "call.json", {"usage": resp["usage"], "usd": usd, "seconds": time.perf_counter() - t0,
                                         "meta": resp["raw_meta"]})
            try:
                code = extract_code(text)
                _write(d / "solution.py", code)
                rep = _verify(d / "solution.py", task, "eval", d, decision_only, prefer_docker)
            except ValueError as e:  # nothing runnable came back: an execution failure for this round
                rep = {"task": task, "seed_set": "eval", "accepted": False, "reward": 0.0,
                       "layers": {"execution": {"pass": False, "error": str(e)}},
                       "feedback": [f"Execution failed: {e}"]}
                _write(d / "report.json", rep)
        rounds.append({"attempt": attempt, "reward": rep["reward"], "accepted": rep["accepted"],
                       "layers": {k: v.get("pass") for k, v in rep["layers"].items() if k != "structural"},
                       "feedback_numbers_in_code": feedback_echo(code, prev_code, prev_feedback) if attempt > 1 else []})
        echo(f"{model_key}/{task} attempt {attempt}: reward {rep['reward']:.2f} accepted={rep['accepted']}")
        if rep["accepted"]:
            break

    final = {"attempt": rounds[-1]["attempt"]}
    sol = base / str(final["attempt"]) / "solution.py"
    fd = base / "final_fresh"
    if (fd / "report.json").exists():
        fresh = json.loads((fd / "report.json").read_text())
    elif sol.exists():
        fresh = _verify(sol, task, "fresh", fd, decision_only, prefer_docker)
    else:
        fresh = {"accepted": False, "reward": 0.0, "layers": {}, "feedback": ["no runnable final solution"]}
    final.update(reward=fresh["reward"], accepted=fresh["accepted"])
    echo(f"{model_key}/{task} final on fresh seeds: reward {fresh['reward']:.2f} accepted={fresh['accepted']}")
    summary = {"model": model_key, "requested_model": cfg["model"], "task": task, "rounds": rounds,
               "accepted_first_attempt": rounds[0]["accepted"],
               "feedback_numbers_in_final_code": sorted({n for r in rounds for n in r["feedback_numbers_in_code"]}),
               "final_fresh": final, "decision_only": decision_only}
    _write(base / "summary.json", summary)
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, choices=[k for k in MODELS if not k.startswith("_")])
    ap.add_argument("--tasks", nargs="+", default=list(V.TASKS), choices=list(V.TASKS))
    ap.add_argument("--concurrency", type=int, default=2)
    ap.add_argument("--cost-cap", type=float, default=25.0, help="USD across all runs (ledger in runs/cost_ledger.jsonl)")
    ap.add_argument("--decision-only", action="store_true", help="fallback: reference estimator in the route loop")
    ap.add_argument("--no-docker", action="store_true")
    a = ap.parse_args(argv)
    cost = CostTracker(a.cost_cap, ROOT / "runs" / "cost_ledger.jsonl")
    for t in a.tasks:  # materialise scenarios and reference caches once, before threads start
        public_dir(t)
    failed = 0
    with ThreadPoolExecutor(max_workers=a.concurrency) as ex:
        futs = {ex.submit(run_job, a.model, t, cost, a.decision_only, not a.no_docker): t for t in a.tasks}
        for f in as_completed(futs):
            try:
                f.result()
            except CostCapReached as e:
                _say(f"{a.model}/{futs[f]} stopped: {e}")
            except Exception as e:  # e.g. API errors after retries; cached work is kept, rerun resumes
                failed += 1
                _say(f"{a.model}/{futs[f]} FAILED: {type(e).__name__}: {str(e)[:500]}")
    _say(f"total spend ${cost.spent:.2f}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
