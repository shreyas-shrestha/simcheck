# SimCheck results (2026-11-06)

Start with `summary/summary.md` and `summary/with_vs_without.png`.

## Headline: Gemini 4 Argon (medium thinking), paired runs

| | First try (no SimCheck feedback) | After SimCheck feedback, regraded on fresh seeds |
|---|---|---|
| v3.1 tasks (20 runs) | 10/20 | 17/20 |
| v3.2 tasks (5 runs) | 3/5 | 4/5 |
| **All (25 runs)** | **13/25 (52%)** | **21/25 (84%)** |

Feedback fixed 8 runs and broke none (exact McNemar p = 0.008). No revision copied a number from the feedback.
An accuracy-only grader (code runs, trajectory close) would accept 17/25 first attempts; 4 of those fail SimCheck
(wrong stated route probability or overconfident covariance).

Before and after are the same 25 runs. Excluded: one v3.2 `unknown_noise` run whose second attempt never returned
from the Gemini API, and all GPT-6 Astra runs (stopped at 8 first attempts by the OpenAI 50 requests/day limit, so
no closed loop). The full data for those is on the `results-v3.1` branch.

## Layout

| Path | Contents |
|---|---|
| `summary/` | `summary.md`, `summary.json`, `with_vs_without.png`, `first_try.png` (regenerate: `.venv/bin/python scripts/summarize.py`) |
| `repeats-v3.1/argon-medium/r1..r5/<task>/` | v3.1: 5 repeats x 4 tasks, first attempt then closed loop |
| `v3.2/argon-medium/r1..r3/<task>/` | v3.2: `close_routes`, `unknown_noise` |
| `argon-medium/<task>/` | initial one-run-per-task closed-loop runs |
| `_trusted/`, `_trusted_v3.2/` | reference and broken-variant validation on dev seeds |
| `docker_regrade.md` | saved solutions regraded in the Docker sandbox (identical rewards) |
| `cost_ledger.jsonl` | per-call API spend |
| `scenarios/` | generated scenarios (public inputs and hidden truth); regenerable from seeds |
| `benchmark/`, `milestone1/` | T benchmark and first plotted reference episode |

Each attempt folder `<task>/<n>/` has `prompt.md`, `response.txt`, `solution.py`, `report.json` (layers, feedback),
`call.json` (tokens, cost), `run_meta.json` (git commit, task_md_version, reported model ID, settings) and
`artifacts.npz` (per-step estimates and covariances, truth, per-trial route outcomes). `final_fresh/` is the
fresh-seed regrade. Two files from the earliest Argon run had a local path redacted from a traceback.
