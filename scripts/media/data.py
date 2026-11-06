"""Every number the deck and clips show, with the file it came from."""
import json
from pathlib import Path

import numpy as np
import summarize as S

from media import common as C

TASK_ORDER = ["prior_needed", "standard", "trap_frame", "trap_units", "close_routes", "unknown_noise"]


class Numbers:
    """Collects (asset, label, value, source) for deck_numbers.json / manifest.json."""

    def __init__(self):
        self.rows = []

    def add(self, asset, label, value, source):
        self.rows.append({"asset": asset, "label": label, "value": value, "source": source})
        return value


def headline_totals():
    """13/25 -> 21/25 etc. from summary.json (with_vs_without, version 'all')."""
    row = next(r for r in C.summary()["with_vs_without"] if r["version"] == "all" and r["model"] == "argon-medium")
    return row, C.rel(C.SUMMARY) + ' -> with_vs_without[version="all"]'


def per_task():
    """First try (summary.json headline) vs after feedback on fresh seeds (run folders, paired exactly as
    scripts/summarize.py pairs them). Totals are checked against summary.json."""
    head = {r["task"]: r for r in C.summary()["headline"] if r["model"] == "argon-medium"}
    groups = {**S.first_try(C.RESULTS / "repeats-v3.1", "v3.1"), **S.first_try(C.RESULTS / "v3.2", "v3.2")}
    out = []
    for task in TASK_ORDER:
        reps = [r for r in groups[("argon-medium", task)]
                if (S._json(Path(r["_dir"]) / "summary.json") or {}).get("max_attempts", 1) > 1]
        after, dirs = 0, []
        for r in reps:
            sm = S._json(Path(r["_dir"]) / "summary.json") or {}
            last = sm.get("rounds", [{}])[-1]
            fresh = (sm.get("final_fresh") or {}).get("accepted")
            after += bool(last.get("accepted") and fresh is not False)
            dirs.append(C.rel(Path(r["_dir"]) / "summary.json"))
        h = head[task]
        assert h["n"] == len(reps), (task, h["n"], len(reps))
        out.append({"task": task, "version": h["version"], "n": len(reps),
                    "first_try": h["passed"], "first_try_wilson95": h["wilson95"],
                    "first_try_source": C.rel(C.SUMMARY) + f' -> headline[task="{task}"]',
                    "after": after, "after_wilson95": list(S.wilson(after, len(reps))),
                    "after_source": dirs})
    tot, _ = headline_totals()
    assert sum(r["first_try"] for r in out) == tot["first_try"] and sum(r["n"] for r in out) == tot["n"]
    assert sum(r["after"] for r in out) == tot["after_loop"], "per-task after-feedback does not add up to summary.json"
    return out


def unknown_noise_coverage():
    """coverage95 per attempt (eval seeds) for both v3.2 unknown_noise runs, at both evaluation times."""
    runs = []
    for rdir in sorted((C.RESULTS / "v3.2" / "argon-medium").glob("r*/unknown_noise")):
        att = []
        for a in sorted((p for p in rdir.iterdir() if p.name.isdigit()), key=lambda p: int(p.name)):
            rep = json.loads((a / "report.json").read_text())
            cal = rep["layers"].get("calibration")
            row = {"attempt": int(a.name), "source": C.rel(a / "report.json"), "accepted": rep["accepted"]}
            if cal and cal.get("times"):
                K = cal["K"]
                row.update({t: {"covered": v["covered"], "K": K, "coverage95": v["covered"] / K}
                            for t, v in cal["times"].items()})
            else:
                row["did_not_run"] = rep["layers"]["execution"].get("error", "").splitlines()[0][:120]
            att.append(row)
        runs.append({"run": rdir.parent.name, "attempts": att})
    # cross-check the attempt-1 value that summary.json reports
    s = {r["time"]: r for r in C.summary()["v3_2"]["unknown_noise"]}
    feat = next(r for r in runs if r["run"] == "r2")["attempts"][0]
    for t in ("end_of_dropout", "end_of_trajectory"):
        assert abs(s[t]["per_run_coverage95"][0] - feat[t]["coverage95"]) < 1e-12
    return runs


def featured_feedback():
    rep = C.featured_report("attempt1")
    return rep["feedback"], C.rel(C.FEATURED / "1" / "report.json")


def v31_feedback_with_rate():
    p = C.RESULTS / "repeats-v3.1" / "argon-medium" / "r1" / "prior_needed" / "1" / "report.json"
    fb = [f for f in json.loads(p.read_text())["feedback"] if "measured rate" in f]
    return fb, C.rel(p)


def checklist(name):
    """Runs / Accurate / Honest / Safe = execution / accuracy / calibration / route layers."""
    rep = C.featured_report(name)
    L = rep["layers"]
    return ([("Runs", L["execution"]["pass"]), ("Accurate", L["accuracy"]["pass"]),
             ("Honest", L["calibration"]["pass"]), ("Safe", L["route"]["pass"])],
            C.rel(C.FEATURED / C.ATTEMPTS[name] / "report.json"))


def choose_route_output(name="attempt1"):
    rt = C.featured_report(name)["layers"]["route"]
    return ({"route": rt["chosen"], "success_prob": rt["stated_success_prob"], "reasoning": rt["reasoning"]},
            C.rel(C.FEATURED / C.ATTEMPTS[name] / "report.json") + " -> layers.route")


def unknown_noise_task_line():
    task_md = (C.FEATURED / "1" / "prompt.md").read_text()
    line = next(l for l in task_md.splitlines() if l.startswith("- ODOMETRY NOISE:"))
    return line[2:], C.rel(C.FEATURED / "1" / "prompt.md")


def calibration(name, time="end_of_trajectory"):
    rep = C.featured_report(name)
    cal = rep["layers"]["calibration"]
    t = cal["times"][time]
    return {"covered": t["covered"], "K": cal["K"], "coverage95": t["covered"] / cal["K"],
            "mean_nees": t["mean_nees"]}, C.rel(C.FEATURED / C.ATTEMPTS.get(name, name) / "report.json")


def per_seed_nees(name, time="end_of_trajectory"):
    """Per eval seed NEES at `time`, recomputed from artifacts.npz with the verifier's own nees_nll and
    eval_times; the covered count is checked against report.json."""
    d = C.FEATURED / C.ATTEMPTS[name]
    z = np.load(d / "artifacts.npz", allow_pickle=True)
    cal, spec = C.featured_report(name)["layers"]["calibration"], C.task_spec()
    out = []
    for i in range(cal["K"]):
        truth = z[f"truth{i}"]
        t = C.V.eval_times(spec, len(truth))[time]
        out.append(C.V.nees_nll(z[f"est{i}"][t], z[f"cov{i}"][t], truth[t])[0])
    assert sum(n <= C.V.NEES_GATE_3 for n in out) == cal["times"][time]["covered"]
    return out, C.rel(d / "artifacts.npz")


def fresh_accepted():
    rep = json.loads((C.FEATURED / "final_fresh" / "report.json").read_text())
    return rep["accepted"], C.rel(C.FEATURED / "final_fresh" / "report.json")


def wilson(k, n):
    return list(S.wilson(k, n))


def fmt_p(p):
    return f"{p:.3f}".rstrip("0") if p >= 0.001 else f"{p:.1g}"


__all__ = ["Numbers", "headline_totals", "per_task", "unknown_noise_coverage", "np"]
