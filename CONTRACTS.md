# SimCheck contracts

These are the interfaces between the sandbox (`simcheck/sandbox.py`, `simcheck/agent_worker.py`), the verifier
(`simcheck/verifier.py`) and the harness (`simcheck/harness.py`). They describe the code as it is, so if you
change a field name, update this file and the tests that pin it.

---

## 1. Container protocol (verifier ↔ agent worker)

### Isolation (`DockerRunner`)
```
docker run --rm -i --name simcheck-<12 hex>
  --network none --read-only --tmpfs /tmp:rw,noexec,nosuid,size=64m
  --cap-drop ALL --security-opt no-new-privileges
  --memory 2g --cpus 2 --pids-limit 256 --user 65534:65534
  -v <tmp>/agent:/agent:ro          # solution.py only (copy)
  -v <tmp>/public:/public:ro        # copy of scenarios/<id>/public, only if public_dir is given
  simcheck-agent:latest python -I /opt/simcheck/agent_worker.py /agent/solution.py
```
- No `-e`: the container gets no host env vars and no API keys. The docker CLI itself runs with only `PATH`.
- Before mounting `public/`, `check_public_dir` refuses it if it contains `truth.json`, a `hidden/` folder or any
  symlink. `hidden/` never leaves the parent process.
- `close()` (and a call timeout) runs `docker kill <name>`, and an `atexit` hook kills any container whose
  runner was never closed.
- Image: `docker build -t simcheck-agent:latest .` (Python 3.12, gtsam 4.3.0, numpy 2.5.4, scipy 1.18.1). The worker
  is baked in at `/opt/simcheck/agent_worker.py` (root-owned, read-only), so **rebuild after changing it**.
  `.dockerignore` limits the build context to that one file.
- Fallback: `make_runner` uses `SubprocessRunner` (a `python -I` process with an empty env; **not isolation**) when
  the image or daemon is unavailable. With `SIMCHECK_REQUIRE_DOCKER=1` it raises `RunnerError` instead. Every
  report records the runner in `isolation`.
- `InProcessRunner` is for trusted code only (reference, broken variants). It mirrors the JSON round trip.

### Framing
- One JSON object per line on stdin and stdout. Requests carry no ids: each request gets exactly one reply.
- `agent_worker.py` sets `sys.stdout = sys.stderr` before importing the agent, so `print()` cannot corrupt the
  stream. (Checked: gtsam's C++ `print()` also goes to stderr.)
- `solution.py` is imported at worker start. An import error kills the worker, and the parent sees a `RunnerError`
  carrying the stderr tail.

### Requests and replies

| request | reply on success |
|---|---|
| `{"cmd": "make", "spec": {...}}` | `{"ok": true}`. Replaces the worker's single estimator; there are no handles. |
| `{"cmd": "step", "m": {...}}` | `{"ok": true, "pose": [x, y, θ], "cov": [[3×3]]}` |
| `{"cmd": "choose_route", "spec": {...}}` | `{"ok": true, "result": {"route", "success_prob": float, "reasoning": str}}` |
| `{"cmd": "build_graph", "measurements": [...], "spec": {...}}` | `{"ok": true, "summary": {...} \| null}`, where the summary has `n_factors`, `factor_types`, `n_variables`, `n_values`, `connected_components`, `unary_factors`, `first_key` |

- **Agent error:** `{"ok": false, "error": "Type: message", "trace": "..."}`. The worker keeps serving, and the
  parent raises `AgentError`.
- **Runner failure:** the worker died, a call timed out, or a line was malformed. The parent raises `RunnerError`.
  A non-JSON-serializable return value (for example a `gtsam.Pose2` instead of a list) also kills the worker,
  because serialization happens outside its try/except.
- **Values are not validated in the worker.** `verifier.check_matrix` checks shape, finiteness, symmetry
  (`‖P−Pᵀ‖ ≤ 1e-9‖P‖`) and Cholesky. NaN and Inf pass through the protocol so that check can reject them.
- **Timeouts:** set in `config.json` `execution`: `call_timeout_s` 30 (per request), `choose_route_timeout_s` 900,
  and `total_budget_s` 3600. `verify()` wraps the runner in `Budgeted`, and every call shares the total deadline;
  exceeding any limit is an execution failure. `build_graph` gets 120 s. (Known issue: under `Budgeted`, a call
  that passes no timeout gets the remaining budget instead of `call_timeout_s`. See DECISIONS.md #25.)
- **Feedback:** `sandbox.clean()` strips host paths from tracebacks, leaving only file names, before they reach the model.
- **Fresh state:** each dataset gets a new `make`. `verify()` re-runs dataset 0 after the other K−1 datasets, and
  it must match (poses `atol=1e-6`, covariances element-wise `rtol=1e-6`). This catches both nondeterminism and
  state leaked through module globals.

### Measurement dict (`m`), from `sim.Sensors.observe`
```
{"step": k, "t": k*dt,
 "odom": null (k = 0) | {"dx", "dy", "dtheta"},     # dx/dy in input_units.odom ("m" | "mm"); frame per odom_frame
 "gps": null | {"x", "y"},                          # world frame, metres; null during dropout_steps [a, b] inclusive
 "landmarks": [{"id": str, "range": m, "bearing"}]}  # bearing relative to heading, wrapped, in input_units.bearing
```
`odom_frame`: `"body"`, or `"odom"` (trap). In the odom frame, dx/dy are rotated by the running sum of the
reported dtheta. Noise standard deviations in `spec.json` are always SI.

---

## 2. report.json (verifier → harness / demo)

`verifier.verify(runner, task, seed_set, decision_only=False, progress=print, out_path=None) -> dict`

```jsonc
{
  "task": "standard" | "trap_units" | "trap_frame" | "prior_needed",
  "seed_set": "dev" | "eval" | "fresh",
  "isolation": "docker --network none, ..." | "NONE: subprocess fallback ..." | "in-process (trusted code only)",
  "thresholds_frozen": true,
  "accepted": false,           // execution & accuracy & calibration & route all pass
  "reward": 0.3,               // 0.1 exec + 0.2 accuracy + 0.4 calibration + 0.3 route (config.json reward_weights)
  "feedback": ["..."],         // list of plain-English strings; the harness sends them to the model as bullets
  "seconds": 0.0,
  "layers": {
    "execution": {"pass": true, "determinism_max_pose_diff": 0.0, "determinism_max_rel_cov_diff": 0.0},
                 // on failure: {"pass": false, "error": "Type: message"}, and the other layers are ABSENT
    "structural": {"provided": true, "summary": {...} | null, "notes": ["..."]},      // diagnostic only
    "accuracy": {"pass": true, "ate_m": 0.0, "reference_ate_m": 0.0, "bound_m": 0.0,  // bound = 0.25 + 1.5 * ref
                 "dropout_ate_m": 0.0, "reference_dropout_ate_m": 0.0, "per_scenario_ate_m": [...]},
    "calibration": {
      "pass": true, "K": 30, "wording": "no calibration inconsistency detected (K=30)",
      "times": {
        "end_of_dropout" | "end_of_trajectory": {
          "pass": true,
          "covered": 27, "coverage_p_value": 0.188, "overconfidence_ok": true,          // (a) binom.cdf(c, K, .95) >= .01
          "mean_nees": 3.01, "nees_lower_bound": 1.97, "underconfidence_ok": true,     // (b) chi2.ppf(.005, 3K)/K
          "mean_nll": -5.54, "reference_mean_nll": -5.54, "nll_bound": -4.54, "nll_ok": true,  // (c) ref + 1.0 nat
          "mean_position_cov_trace_m2": 0.0, "per_trajectory_coverage_diagnostic": 0.0
        }
      }
    },
    "route": {
      "pass": true,                // regret_outcome == "pass" AND stated_check.pass
      "regret_outcome": "pass" | "fail" | "inconclusive",
      "mode": "submitted estimator in the loop" | "decision-only grading (reference estimator in the loop)",
      "T": 200, "chosen": "B", "stated_success_prob": 0.97, "reasoning": "...",
      "routes": {"A": {"success_rate": 0.64, "wilson95": [lo, hi], "collision_rate": 0.0}, "B": {...}},
      "diff_B_minus_A": 0.36, "diff_ci95": [lo, hi],      // paired percentile bootstrap, 10k resamples, seed 12345
      "regret": 0.0, "regret_ci95": [lo, hi], "delta": 0.05,
      "stated_check": {"pass": true, "stated": 0.97, "estimated": 1.0, "estimated_wilson95": [lo, hi],
                       "abs_diff": 0.03, "tolerance": 0.15},
      "reference_estimator_rates": {"A": 0.64, "B": 1.0}
      // when the runner fails mid-trials: "error" is added and the numbers are NaN
    }
  }
}
```
When there is no runnable code, the harness writes `{"task", "seed_set", "accepted": false, "reward": 0.0,
"layers": {"execution": {"pass": false, "error"}}, "feedback": [...]}`.

---

## 3. Harness files and the cost cap

```
runs/<model>/<task>/<attempt 1..3>/  prompt.md  response.txt  model_id  settings.json  call.json  solution.py  report.json  log.txt
runs/<model>/<task>/final_fresh/     report.json  log.txt          # final code re-graded on fresh seeds
runs/<model>/<task>/summary.json     {model, requested_model, task, rounds[{attempt, reward, accepted, layers}], final_fresh, decision_only}
runs/cost_ledger.jsonl               one {"usd", "tag", "time"} per paid call; spend survives restarts
runs/_reference/                     cached reference datasets and route trials
```
- `model_id` is the model string the API reported back. `call.json` holds token usage, USD, seconds and response metadata.
- **Resume:** an attempt is skipped if its `report.json` exists, and the model is never re-called if `response.txt` exists.
- **Cost cap:** before each call, the harness reserves the call's worst case (prompt ≈ chars/3 tokens plus the full
  output token limit) and refuses the call if spent + in-flight + this call would exceed the cap. A model without
  prices in `models.json` is refused before any call.
- **Prompts:** the prompt is task.md + spec.json + one dev-seed measurements.json + the contract. Revisions add the
  previous solution.py and the feedback bullets (single turn, no chat history). Code extraction takes the last
  parseable block defining both `make_estimator` and `choose_route`, falling back to the last one defining
  `make_estimator`.
- **task.md version:** `config.json` `"task_md_version"` is `"v3.1"` (default; the original text, and
  `test_v31_prompt_is_byte_identical_to_original_run` pins it to the saved v3.1 prompt) or `"v3.2"` (adds the x₀
  prior, SI noise units, covariance frame and a grading summary). `build_prompt` renders task.md from `spec.json`
  with that version at prompt time, so the cached `scenarios/` folders need no regeneration. The
  `public/task.md` file in the scenario folder (also mounted into the container) is whatever version was
  current when it was generated.
