"""Collect runs/<model>/<task>/ into one results table: runs/results.md and runs/results.json.

    .venv/bin/python scripts/summarize.py [model ...]
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck.harness import feedback_echo  # noqa: E402

RUNS = ROOT / "runs"
SKIP = {"_reference", "_trusted", "benchmark", "milestone1", "fake-good", "fake-learner"}


def _json(p):
    return json.loads(p.read_text()) if p.exists() else None


def rows(models=None):
    out = []
    for mdir in sorted(p for p in RUNS.iterdir() if p.is_dir() and p.name not in SKIP):
        if models and mdir.name not in models:
            continue
        for tdir in sorted(p for p in mdir.iterdir() if p.is_dir()):
            s = _json(tdir / "summary.json")
            attempts = sorted(int(p.name) for p in tdir.iterdir() if p.name.isdigit())
            usd = sum((_json(tdir / str(a) / "call.json") or {}).get("usd", 0.0) for a in attempts)
            out_tok = sum(((_json(tdir / str(a) / "call.json") or {}).get("usage") or {}).get("output_tokens", 0)
                          for a in attempts)
            last = _json(tdir / str(attempts[-1]) / "report.json") if attempts else None
            fresh = _json(tdir / "final_fresh" / "report.json")
            settings = _json(tdir / str(attempts[0]) / "settings.json") if attempts else None
            model_id = (tdir / str(attempts[0]) / "model_id").read_text().strip() \
                if attempts and (tdir / str(attempts[0]) / "model_id").exists() else "?"
            r = (fresh or last or {}).get("layers", {}).get("route", {})
            echo = set()
            for a in attempts[1:]:  # recomputed from saved files, so older runs are checked too
                code, prev = (tdir / str(a) / "solution.py"), (tdir / str(a - 1) / "solution.py")
                fb = (_json(tdir / str(a - 1) / "report.json") or {}).get("feedback")
                if code.exists():
                    echo |= set(feedback_echo(code.read_text(), prev.read_text() if prev.exists() else None, fb))
            out.append({
                "model": mdir.name, "model_id": model_id, "task": tdir.name,
                "settings": (settings or {}).get("settings"),
                "status": "done" if s else ("in progress" if attempts else "not started"),
                "attempts": len(attempts),
                "rewards": [(_json(tdir / str(a) / "report.json") or {}).get("reward") for a in attempts],
                "accepted_first_attempt": (s or {}).get("accepted_first_attempt",
                                                        (s or {}).get("rounds", [{}])[0].get("accepted") if s else None),
                "accepted_eval": (last or {}).get("accepted"),
                "accepted_fresh": (fresh or {}).get("accepted"),
                "fresh_reward": (fresh or {}).get("reward"),
                "feedback_numbers_in_code": sorted(echo),
                "route": r.get("chosen"), "stated": r.get("stated_success_prob"),
                "measured": {k: v.get("success_rate") for k, v in (r.get("routes") or {}).items()},
                "usd": round(usd, 3), "output_tokens": out_tok,
                "isolation": (fresh or last or {}).get("isolation", "?"),
            })
    return out


def to_markdown(rs):
    yn = lambda v: "-" if v is None else ("yes" if v else "no")
    lines = ["| Model | Task | Status | Rewards per attempt | 1st try | Eval | Fresh | Route (stated / measured) | "
             "Copied feedback numbers | Cost |", "|" + "---|" * 10]
    for r in rs:
        meas = ", ".join(f"{k} {v:.2f}" for k, v in r["measured"].items()) if r["measured"] else "-"
        stated = "-" if r["stated"] is None else f"{r['stated']:.2f}"
        lines.append(f"| {r['model']} | {r['task']} | {r['status']} | "
                     f"{' → '.join('-' if x is None else f'{x:.2f}' for x in r['rewards']) or '-'} | "
                     f"{yn(r['accepted_first_attempt'])} | {yn(r['accepted_eval'])} | {yn(r['accepted_fresh'])} | "
                     f"{r['route'] or '-'} ({stated} / {meas}) | {', '.join(r['feedback_numbers_in_code']) or '-'} | "
                     f"${r['usd']:.2f} |")
    iso = sorted({r["isolation"] for r in rs})
    lines += ["", "Isolation: " + "; ".join(iso)]
    return "\n".join(lines)


if __name__ == "__main__":
    rs = rows(set(sys.argv[1:]) or None)
    md = to_markdown(rs)
    (RUNS / "results.md").write_text(md + "\n")
    (RUNS / "results.json").write_text(json.dumps(rs, indent=2))
    print(md)
