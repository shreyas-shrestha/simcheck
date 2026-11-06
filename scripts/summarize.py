"""Collect model runs into one summary.

    .venv/bin/python scripts/summarize.py [--v31 runs/repeats-v3.1] [--v32 ../simcheck-v32/runs/v3.2]
                                          [--loops runs] [--out runs/summary]

Headline: first-attempt pass rate per model per task with Wilson 95% intervals, from repeat folders laid out
as <root>/<model>/r<i>/<task>/1/report.json.
Secondary: closed-loop runs (<loops>/<model>/<task>/<attempt>/): reward per attempt, attempts used, and
numbers from feedback that first appear in revised code.
v3.2: coverage95 and mean NEES on unknown_noise; stated vs measured success probability on close_routes.
Cost: spend per model from call.json files under all roots.
Writes <out>/summary.json, <out>/summary.md and <out>/first_try.png.
"""
import argparse
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck.harness import feedback_echo  # noqa: E402

SKIP = {"_reference", "_trusted", "benchmark", "milestone1", "fake-good", "fake-learner", "summary"}


def wilson(k, n, z=1.96):
    if n == 0:
        return [0.0, 1.0]
    p, d = k / n, 1 + z * z / n
    c, h = (p + z * z / (2 * n)) / d, z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [max(0.0, c - h), min(1.0, c + h)]


def _json(p):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, json.JSONDecodeError):
        return None


def first_try(root, version):
    """{(model, task): [report of attempt 1 per repeat]} for a repeats root."""
    out = defaultdict(list)
    root = Path(root)
    if not root.exists():
        return out
    for mdir in sorted(p for p in root.iterdir() if p.is_dir()):
        for rdir in sorted(p for p in mdir.iterdir() if p.is_dir() and p.name.startswith("r")):
            for tdir in sorted(p for p in rdir.iterdir() if p.is_dir()):
                rep = _json(tdir / "1" / "report.json")
                if rep is not None:
                    out[(mdir.name, tdir.name)].append(dict(rep, _dir=str(tdir), _version=version,
                                                            _meta=_json(tdir / "1" / "run_meta.json")))
    return out


def headline(groups):
    rows = []
    for (model, task), reps in sorted(groups.items()):
        k, n = sum(bool(r.get("accepted")) for r in reps), len(reps)
        fail_layers = defaultdict(int)
        for r in reps:
            for name, L in (r.get("layers") or {}).items():
                if name != "structural" and not L.get("pass", True):
                    fail_layers[name] += 1
        rows.append({"version": reps[0]["_version"], "model": model, "task": task, "n": n, "passed": k,
                     "rate": k / n if n else None, "wilson95": wilson(k, n),
                     "mean_reward": sum(r.get("reward", 0.0) for r in reps) / n,
                     "failed_layers": dict(fail_layers),
                     "model_ids": sorted({(r["_meta"] or {}).get("model_id_reported", "?") for r in reps}),
                     "commits": sorted({(r["_meta"] or {}).get("git_commit", "?") for r in reps}),
                     "task_md_versions": sorted({(r["_meta"] or {}).get("task_md_version", "?") for r in reps})})
    return rows


def loops(root):
    """Closed-loop runs: <root>/<model>/<task>/<attempt>/ with summary.json."""
    rows, root = [], Path(root)
    if not root.exists():
        return rows
    for mdir in sorted(p for p in root.iterdir() if p.is_dir() and p.name not in SKIP and not p.name.startswith("repeats")):
        for tdir in sorted(p for p in mdir.iterdir() if p.is_dir() and (p / "summary.json").exists()):
            attempts = sorted(int(p.name) for p in tdir.iterdir() if p.name.isdigit())
            echo = set()
            for a in attempts[1:]:
                code, prev = tdir / str(a) / "solution.py", tdir / str(a - 1) / "solution.py"
                fb = (_json(tdir / str(a - 1) / "report.json") or {}).get("feedback")
                if code.exists():
                    echo |= set(feedback_echo(code.read_text(), prev.read_text() if prev.exists() else None, fb))
            fresh = _json(tdir / "final_fresh" / "report.json") or {}
            rows.append({"model": mdir.name, "task": tdir.name, "attempts_used": len(attempts),
                         "rewards": [(_json(tdir / str(a) / "report.json") or {}).get("reward") for a in attempts],
                         "fresh_reward": fresh.get("reward"), "fresh_accepted": fresh.get("accepted"),
                         "copied_feedback_numbers": sorted(echo)})
    return rows


def v32_details(groups):
    out = {"unknown_noise": [], "close_routes": []}
    for (model, task), reps in sorted(groups.items()):
        if task == "unknown_noise":
            for t in ("end_of_dropout", "end_of_trajectory"):
                cal = [r["layers"]["calibration"] for r in reps
                       if (r.get("layers") or {}).get("calibration", {}).get("times")]
                cs = [c["times"][t] for c in cal]
                if cs:
                    K = cal[0]["K"]
                    out["unknown_noise"].append({
                        "model": model, "time": t, "n_runs": len(cs),
                        "coverage95": sum(c["covered"] for c in cs) / (K * len(cs)),
                        "mean_nees": sum(c["mean_nees"] for c in cs) / len(cs),
                        "per_run_coverage95": [c["covered"] / K for c in cs],
                        "per_run_mean_nees": [c["mean_nees"] for c in cs]})
        if task == "close_routes":
            for r in reps:
                rt = (r.get("layers") or {}).get("route") or {}
                if rt.get("routes"):
                    s = rt["stated_check"]
                    out["close_routes"].append({"model": model, "chosen": rt["chosen"], "stated": s["stated"],
                                                "measured": s["estimated"], "measured_wilson95": s["estimated_wilson95"],
                                                "pass": s["pass"],
                                                "rates": {k: v["success_rate"] for k, v in rt["routes"].items()}})
    return out


def cost(roots):
    spend, seen = defaultdict(float), set()
    for root in roots:
        root = Path(root)
        if not root.exists():
            continue
        for c in root.rglob("call.json"):
            if c.resolve() in seen:
                continue
            seen.add(c.resolve())
            rel = c.relative_to(root).parts
            model = next(p for p in rel if not p.startswith(("repeats", "v3.")))
            if model not in SKIP and not model.startswith("_"):
                spend[model] += (_json(c) or {}).get("usd", 0.0)
    return {k: round(v, 3) for k, v in sorted(spend.items())}


def mcnemar(b, c):
    """Exact two-sided McNemar test on discordant pairs (fixed vs lost)."""
    n = b + c
    if n == 0:
        return 1.0
    tail = sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n
    return min(1.0, 2 * tail)


def with_vs_without(groups):
    """Per model and version: first try (no verifier feedback) vs after the closed loop, and what a
    'runs and is accurate' grader would have accepted on the first try."""
    out = []
    by = defaultdict(list)
    for (model, task), reps in groups.items():
        by[(reps[0]["_version"], model)] += reps
    for (version, model), reps in sorted(by.items()):
        first = sum(bool(r.get("accepted")) for r in reps)
        naive = sum(bool(r["layers"]["execution"]["pass"] and r["layers"].get("accuracy", {}).get("pass")) for r in reps)
        looped, finals, echo, fixed, broke = 0, 0, set(), 0, 0
        for r in reps:
            sm = _json(Path(r["_dir"]) / "summary.json") or {}
            if sm.get("max_attempts", 1) > 1:  # finished the closed loop (and fresh re-grade)
                looped += 1
                last = sm.get("rounds", [{}])[-1]
                fresh = (sm.get("final_fresh") or {}).get("accepted")
                ok = bool(last.get("accepted") and fresh is not False)
                finals += ok
                fixed += ok and not r.get("accepted")
                broke += (not ok) and bool(r.get("accepted"))
                echo |= set(sm.get("feedback_numbers_in_final_code", []))
        n = len(reps)
        out.append({"version": version, "model": model, "n": n,
                    "first_try": first, "first_try_wilson95": wilson(first, n),
                    "after_loop_n": looped, "after_loop": finals, "after_loop_wilson95": wilson(finals, looped),
                    "accuracy_only_grader_accepts": naive, "wrongly_accepted_by_accuracy_only": naive - first,
                    "copied_feedback_numbers": sorted(echo),
                    "fixed_by_feedback": fixed, "first_try_pass_lost": broke,
                    "mcnemar_exact_p": mcnemar(fixed, broke)})
    return out


def chart(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tasks = list(dict.fromkeys(r["task"] for r in rows))
    models = list(dict.fromkeys(r["model"] for r in rows))
    colors = ["#2a6fdb", "#e07b39", "#3a9d5d", "#9b59b6"]
    fig, ax = plt.subplots(figsize=(max(6, 1.6 * len(tasks)), 3.6), dpi=150)
    w = 0.8 / max(1, len(models))
    for j, m in enumerate(models):
        for i, t in enumerate(tasks):
            r = next((r for r in rows if r["model"] == m and r["task"] == t), None)
            if r is None:
                continue
            x = i - 0.4 + w * (j + 0.5)
            lo, hi = r["wilson95"]
            ax.bar(x, r["rate"], w * 0.9, color=colors[j % len(colors)], label=m if i == 0 else None)
            ax.errorbar(x, r["rate"], yerr=[[r["rate"] - lo], [hi - r["rate"]]], color="#333", capsize=3, lw=1)
            ax.text(x, 0.02, f"{r['passed']}/{r['n']}", ha="center", va="bottom", fontsize=7,
                    color="white" if r["rate"] > 0.08 else "#333")
    ax.set_xticks(range(len(tasks)), [f"{t}\n({next(r['version'] for r in rows if r['task'] == t)})" for t in tasks],
                  fontsize=8)
    ax.set_ylim(0, 1.05)
    ax.set_ylabel("first-attempt pass rate")
    ax.set_title("First-attempt pass rate, Wilson 95% intervals", fontsize=10)
    ax.legend(fontsize=8, frameon=False, loc="upper right", bbox_to_anchor=(1, 1.18), ncol=len(models))
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path)


def chart_delta(rows, path):
    """First try vs after SimCheck feedback, per model and spec version (only rows that ran the loop)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [r for r in rows if r["after_loop_n"]]
    if not rows:
        return
    fig, ax = plt.subplots(figsize=(max(6.5, 2.2 * len(rows)), 3.8), dpi=150)
    for i, r in enumerate(rows):
        for j, (k, n, ci, c, lab) in enumerate((
                (r["first_try"], r["n"], r["first_try_wilson95"], "#9aa5b1", "first try (no feedback)"),
                (r["after_loop"], r["after_loop_n"], r["after_loop_wilson95"], "#2a6fdb",
                 "after SimCheck feedback (fresh seeds)"))):
            x, v = i + (j - 0.5) * 0.38, k / n
            ax.bar(x, v, 0.34, color=c, label=lab if i == 0 else None)
            ax.errorbar(x, v, yerr=[[v - ci[0]], [ci[1] - v]], color="#333", capsize=3, lw=1)
            ax.text(x, 0.03, f"{k}/{n}", ha="center", va="bottom", fontsize=9, color="white", fontweight="bold")
    ax.set_xticks(range(len(rows)), [f"{r['model']}\n{r['version']}" for r in rows], fontsize=8)
    ax.set_ylim(0, 1.08)
    ax.set_ylabel("accepted (all layers)")
    ax.set_title("Pass rate without vs with SimCheck feedback (Wilson 95%)", fontsize=10)
    ax.legend(fontsize=8, frameon=False, loc="upper left", bbox_to_anchor=(0, 1.2), ncol=2)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path)


def to_md(s):
    f = lambda x: "-" if x is None else f"{x:.2f}"
    L = ["## With vs without SimCheck", "",
         "| Version | Model | First try (no feedback) | After SimCheck feedback (fresh seeds) | "
         "Fixed / lost by feedback | McNemar exact p | Accuracy-only grader accepts | ...of which actually failing | "
         "Copied feedback numbers |", "|" + "---|" * 9]
    for r in s["with_vs_without"]:
        L.append(f"| {r['version']} | {r['model']} | {r['first_try']}/{r['n']} "
                 f"[{r['first_try_wilson95'][0]:.2f}, {r['first_try_wilson95'][1]:.2f}] | "
                 f"{r['after_loop']}/{r['after_loop_n']} [{r['after_loop_wilson95'][0]:.2f}, "
                 f"{r['after_loop_wilson95'][1]:.2f}] | {r['fixed_by_feedback']} / {r['first_try_pass_lost']} | "
                 f"{r['mcnemar_exact_p']:.3g} | {r['accuracy_only_grader_accepts']}/{r['n']} | "
                 f"{r['wrongly_accepted_by_accuracy_only']} | {', '.join(r['copied_feedback_numbers']) or '-'} |")
    L += ["", "## First-attempt pass rate per task", "",
         "| Version | Model | Task | Passed | Rate | Wilson 95% | Mean reward | Failed layers |", "|" + "---|" * 8]
    for r in s["headline"]:
        L.append(f"| {r['version']} | {r['model']} | {r['task']} | {r['passed']}/{r['n']} | {f(r['rate'])} | "
                 f"[{r['wilson95'][0]:.2f}, {r['wilson95'][1]:.2f}] | {f(r['mean_reward'])} | "
                 f"{', '.join(f'{k} {v}' for k, v in r['failed_layers'].items()) or '-'} |")
    L += ["", "## Closed loop (secondary)", "",
          "| Model | Task | Attempts | Reward per attempt | Fresh reward | Copied feedback numbers |", "|" + "---|" * 6]
    for r in s["closed_loop"]:
        L.append(f"| {r['model']} | {r['task']} | {r['attempts_used']} | "
                 f"{' → '.join(f(x) for x in r['rewards'])} | {f(r['fresh_reward'])} | "
                 f"{', '.join(r['copied_feedback_numbers']) or '-'} |")
    if s["v3_2"]["unknown_noise"]:
        L += ["", "## v3.2 unknown_noise calibration", "", "| Model | Time | Runs | coverage95 | Mean NEES |",
              "|---|---|---|---|---|"]
        for r in s["v3_2"]["unknown_noise"]:
            L.append(f"| {r['model']} | {r['time']} | {r['n_runs']} | {r['coverage95']:.3f} | {r['mean_nees']:.2f} |")
    if s["v3_2"]["close_routes"]:
        L += ["", "## v3.2 close_routes stated vs measured", "", "| Model | Chosen | Stated | Measured [95%] | Pass |",
              "|---|---|---|---|---|"]
        for r in s["v3_2"]["close_routes"]:
            lo, hi = r["measured_wilson95"]
            L.append(f"| {r['model']} | {r['chosen']} | {r['stated']:.2f} | {r['measured']:.3f} [{lo:.3f}, {hi:.3f}] | "
                     f"{'yes' if r['pass'] else 'no'} |")
    L += ["", "## Spend", "", "| Model | USD |", "|---|---|"] + [f"| {k} | {v:.2f} |" for k, v in s["cost_usd"].items()]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v31", default=str(ROOT / "runs" / "repeats-v3.1"))
    ap.add_argument("--v32", default=str(ROOT.parent / "simcheck-v32" / "runs" / "v3.2"))
    ap.add_argument("--loops", nargs="*", default=[str(ROOT / "runs")])
    ap.add_argument("--out", default=str(ROOT / "runs" / "summary"))
    a = ap.parse_args()
    g31, g32 = first_try(a.v31, "v3.1"), first_try(a.v32, "v3.2")
    rows = headline(g31) + headline(g32)
    s = {"with_vs_without": with_vs_without(g31) + with_vs_without(g32),
         "headline": rows,
         "closed_loop": [r for root in a.loops for r in loops(root)],
         "v3_2": v32_details(g32),
         "cost_usd": cost([a.v31, a.v32] + a.loops)}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(s, indent=2))
    (out / "summary.md").write_text(to_md(s) + "\n")
    if rows:
        chart(rows, out / "first_try.png")
        chart_delta(s["with_vs_without"], out / "with_vs_without.png")
    print(to_md(s))


if __name__ == "__main__":
    main()
