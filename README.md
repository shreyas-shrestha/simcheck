# SimCheck (spec v3.1, frozen)

Verifies agent-written GTSAM state estimators and route choices against simulated truth.

## Isolation status

Agent code runs through `simcheck/sandbox.py`:

- `DockerRunner` is the real boundary: `--network none`, read-only rootfs, only `solution.py` (and a copy
  of `public/`) mounted read-only, no env vars, stdin/stdout protocol. Build the image with
  `docker build -t simcheck-agent:latest .`
- **Limitation:** on the build machine the Docker daemon was not running, so every result so far used
  `SubprocessRunner`, a separate `python -I` process with an empty environment. That is **not** a
  security boundary. Each `report.json` records which one was used in its `isolation` field.

## Layout

| File | Role |
|---|---|
| `simcheck/generator.py` | ScenarioConfig + seed -> `scenarios/<id>/{public,hidden}` |
| `simcheck/sim.py` | dynamics, sensors, pure-pursuit controller, collision check |
| `simcheck/reference.py` | correct iSAM2 estimator and `choose_route` |
| `simcheck/broken.py` | deliberately broken variants |
| `simcheck/verifier.py` | layers 0-4, reward, feedback -> `report.json` |
| `simcheck/sandbox.py`, `simcheck/agent_worker.py` | runners and the in-sandbox protocol server |
| `simcheck/harness.py` | model call, code extraction, sandboxed verify, 3-attempt closed loop, fresh-seed re-grade |
| `models.json` | model registry (one-line swap via `--model`), settings, prices for the cost cap |
| `config.json` | frozen T and thresholds, with the dev-seed evidence for each |

## Interpretation notes (no design changes, just how ambiguous spec text was implemented)

- Calibration datasets drive route B with a truth-following controller, so turns expose frame/unit bugs.
- `prior_needed=True`: no GPS and no landmarks at all, so the prior is the only absolute anchor.
- Frame trap: `odom_frame = "odom"`, dx/dy rotated by the running sum of reported dtheta (stated in task.md).
- `*_noise_std` are always SI; traps only change measurement encoding.
- Layer 2 compares the mean ATE over the K datasets against `abs_tol + 1.5 x reference mean ATE`.
- Layer 4 passes only if the regret outcome is "pass" and the stated-probability check passes.
- Determinism and fresh-state: dataset 0 is re-run after the other K-1 datasets and must match.

## Setup

```bash
uv venv -p 3.12 .venv && uv pip install -p .venv/bin/python -r requirements.txt
```

## Commands

```bash
.venv/bin/python -m pytest -q simcheck/tests
.venv/bin/python scripts/milestone1.py 0
.venv/bin/python scripts/quick_layers.py dev
.venv/bin/python scripts/verify_trusted.py reference standard dev
```

## Running models

1. In `models.json`, confirm the API model strings for `astra` and `argon` and fill in `price_per_mtok`.
   The harness refuses to call a model whose prices are unset, because the cost cap depends on them.
2. Export `OPENAI_API_KEY` / `GEMINI_API_KEY`. Keys stay in the harness process and never reach the sandbox.
3. Run:

```bash
.venv/bin/python -m simcheck.harness --model astra --concurrency 2 --cost-cap 25
```

Outputs go to `runs/<model>/<task>/<attempt>/` (prompt, raw response, reported model ID, settings, usage
and cost, solution.py, report.json, log) plus `final_fresh/` and `summary.json` (reward per round).
Reruns resume: finished attempts are skipped, and saved responses are never re-requested. Spend is
tracked across runs in `runs/cost_ledger.jsonl`. `--decision-only` puts the reference estimator in the
route loop; those reports are labelled "decision-only grading". `fake-good` and `fake-learner` are
offline models for testing the loop.
