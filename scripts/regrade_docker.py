"""Re-grade each saved final solution inside the Docker sandbox, on eval and fresh seeds. No API calls.

    .venv/bin/python scripts/regrade_docker.py [--jobs 3] [model ...]

Writes runs/<model>/<task>/docker/{eval,fresh}/{report.json,log.txt} and prints a comparison with the
original (subprocess) grades. Refuses to run if Docker or the simcheck-agent image is unavailable.
"""
import argparse
import json
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck.harness import _verify  # noqa: E402
from simcheck.sandbox import DockerRunner  # noqa: E402

RUNS = ROOT / "runs"
SETS = {"eval": None, "fresh": "final_fresh"}


def final_solution(tdir):
    attempts = sorted(int(p.name) for p in tdir.iterdir() if p.name.isdigit() and (p / "solution.py").exists())
    return tdir / str(attempts[-1]) / "solution.py" if attempts else None


def original(tdir, seed_set):
    if seed_set == "fresh":
        p = tdir / "final_fresh" / "report.json"
    else:
        attempts = sorted(int(p.name) for p in tdir.iterdir() if p.name.isdigit())
        p = tdir / str(attempts[-1]) / "report.json"
    return json.loads(p.read_text()) if p.exists() else {}


def job(model, task, seed_set):
    tdir = RUNS / model / task
    d = tdir / "docker" / seed_set
    if (d / "report.json").exists():
        rep = json.loads((d / "report.json").read_text())
    else:
        rep = _verify(final_solution(tdir), task, seed_set, d, decision_only=False, prefer_docker=True)
    print(f"{model}/{task}/{seed_set}: reward {rep.get('reward')}", flush=True)
    return model, task, seed_set, rep, original(tdir, seed_set)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("models", nargs="*", default=["astra-medium", "argon-medium"])
    ap.add_argument("--jobs", type=int, default=3)
    a = ap.parse_args()
    if not DockerRunner.available():
        sys.exit("Docker or the simcheck-agent image is unavailable; run: docker build -t simcheck-agent:latest .")
    todo = [(m, t.name, s) for m in a.models for t in sorted((RUNS / m).iterdir())
            if t.is_dir() and (t / "summary.json").exists() for s in SETS]
    with ThreadPoolExecutor(a.jobs) as ex:
        res = list(ex.map(lambda x: job(*x), todo))
    lines = ["| Model | Task | Seeds | Reward (subprocess) | Reward (Docker) | Accepted (Docker) | Isolation |",
             "|---|---|---|---|---|---|---|"]
    for m, t, s, rep, orig in res:
        lines.append(f"| {m} | {t} | {s} | {orig.get('reward')} | {rep.get('reward')} | {rep.get('accepted')} | "
                     f"{rep.get('isolation')} |")
    md = "\n".join(lines)
    (RUNS / "docker_regrade.md").write_text(md + "\n")
    print(md)
    sys.exit(0 if all(r[3].get("reward") == r[4].get("reward") for r in res) else 1)


if __name__ == "__main__":
    main()
