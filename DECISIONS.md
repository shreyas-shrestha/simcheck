# SimCheck: decision log (harness / sandbox / stats / task.md side)

Status: **open** · **deferred** (waiting on something) · **decided** (with the date and where it is recorded)
Last updated 2026-11-06, after integrating the sandbox and harness work.

## A. Model calls

| # | Decision | Status | Notes |
|---|---|---|---|
| 1 | Exact API model IDs | **deferred**: confirm when both keys are in | `models.json`: astra = `gpt-6-astra` (the Astra smoke run's API reported back `gpt-6-astra`, so it may be a moving alias; check for a dated snapshot). argon = `gemini-argon` is a placeholder (teammate's note: probably `gemini-4-argon`, unverified; no Google key yet). |
| 2 | OpenAI endpoint | decided (implementation) | Chat Completions in `harness.call_openai`. Revisit only if Astra needs the Responses API. |
| 3 | Generation settings | **open** | Only output limits are set (32k each). No temperature or reasoning budget is set for either model, so both use provider defaults. Decide whether to pin them and make them equivalent. |
| 4 | Prices | **open** | Astra $10 / $50 per M tokens from third-party listings (confirm on OpenAI's pricing page). Argon is unset, and the harness refuses to call it until set. |
| 5 | Cost cap and concurrency | **open** | Defaults are `--cost-cap 25` and `--concurrency 2`. Worst case per Astra call ≈ $1.60 of output plus input. |
| 6 | Prompt contents | decided (teammate) | task.md + spec.json + one dev-seed measurements.json + the contract. |
| 7 | Revision prompt | decided (teammate) | Previous solution.py + feedback bullets, single turn. |

## B. Scenario format (settled by `sim.py` / `generator.py`)

| # | Decision | Status | Notes |
|---|---|---|---|
| 8 | Measurement field names | decided | `gps {"x","y"}`, `landmarks [{"id","range","bearing"}]` (see CONTRACTS.md §1) |
| 9 | Bearing convention | decided | Relative to heading, wrapped |
| 10 | `dropout_steps` | decided | Inclusive `[a, b]` |
| 11 | Noise std units | decided | Always SI, and now stated in task.md |
| 12 | Trap values | decided | Units: mm and degrees. Frame: `"odom"` (rotated by the running sum of dtheta) |
| 13 | Config location | decided | `config.json` (frozen thresholds with their dev evidence) |
| 14 | Spec interpretations | decided (teammate, README "Interpretation notes") | ATE = mean over K of per-dataset RMS. The stated-probability check gates Layer 4. Determinism = re-running dataset 0 after the others. |
| 21 | task.md conventions | decided 2026-11-06 | Added: x₀ sampled from the prior, SI noise std, covariance in the pose's local tangent frame (not world), and a short grading summary with no thresholds. |

## C. Runtime limits

| # | Decision | Status | Notes |
|---|---|---|---|
| 15 | Sandbox timeouts and limits | **open** | 30 s per call (900 s for `choose_route`, 120 s for `build_graph`), 2 GB, 2 CPUs, 256 pids, 64 MB /tmp. There is no total budget per evaluation. A full reference verify through Docker took 20 s, but a slow agent (0.77 s per paired trial in `bench_slow.py`) would need about 3 min for T=200, with no upper bound. |
| 16 | Fresh state per trial | decided (teammate) | One worker per evaluation, with `make` per episode. Leaks are caught by re-running dataset 0. |
| 17 | Frozen thresholds and T | decided (teammate) | `config.json`: T=200, K=30, α=0.01, margin 1.0 nat, abs_tol 0.25 m, δ=0.05, stated tolerance 0.15. |

## D. Housekeeping and follow-ups for the teammate

| # | Item | Status | Notes |
|---|---|---|---|
| 18 | Move the work into the repo | done | Branch `integrate-sandbox-harness` |
| 19 | Where the run layer lives | done | `simcheck/sandbox.py` (theirs, with the fixes merged in) |
| 20 | README isolation note | **open** (teammate) | The README still says Docker never ran. The Docker path is now tested: 16 sandbox tests, plus a full reference verify accepted under Docker. The Astra smoke run in `runs/astra/standard/` used the subprocess fallback and the old task.md, so re-run it under Docker. |
| 22 | `agent_worker.py` serialization | **open** (teammate) | A non-JSON-serializable return (e.g. a `gtsam.Pose2`) kills the worker instead of returning `ok: false`. It is graded as an execution failure, but the message is a raw traceback. Pinned by `test_pose2_return_is_execution_failure`. |
| 23 | Tracked run outputs | **open** (teammate) | `runs/_trusted/...` and `runs/astra/...` are committed even though `.gitignore` has `runs/*`. Note that `scripts/verify_trusted.py` overwrites `runs/_trusted/.../report.json`. |
| 24 | Stale `scenarios/` caches | note | Delete `scenarios/` on any machine that generated it before the task.md change. |
