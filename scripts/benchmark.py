"""Runtime benchmark: N paired trials per route, reference estimator in the loop."""
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck import generator, reference, sim  # noqa: E402


def main(n=int(sys.argv[1]) if len(sys.argv) > 1 else 20, seed_base=1_000_000):
    spec = generator.build_spec(generator.ScenarioConfig(seed=0))
    sim.run_episode(spec, "B", 999, estimator=reference.make_estimator(spec))  # warm-up (imports, first iSAM2)
    res = {r: {"success": [], "collision": [], "sec": [], "steps": [], "sec_per_step": []} for r in "AB"}
    for i in range(n):
        for r in "AB":
            t0 = time.perf_counter()
            ep = sim.run_episode(spec, r, seed_base + i, estimator=reference.make_estimator(spec))
            dt = time.perf_counter() - t0
            o = ep["outcome"]
            res[r]["success"].append(o["success"]); res[r]["collision"].append(o["collision"])
            res[r]["sec"].append(dt); res[r]["steps"].append(o["steps"] + 1)
            res[r]["sec_per_step"].append(dt / (o["steps"] + 1))
    out = {}
    for r, d in res.items():
        out[r] = {"success_rate": float(np.mean(d["success"])), "collision_rate": float(np.mean(d["collision"])),
                  "mean_sec_per_episode": float(np.mean(d["sec"])), "max_sec_per_episode": float(np.max(d["sec"])),
                  "mean_steps": float(np.mean(d["steps"])), "mean_ms_per_step": 1e3 * float(np.mean(d["sec_per_step"]))}
    out["paired_both_succeed"] = int(np.sum(np.array(res["A"]["success"]) & np.array(res["B"]["success"])))
    out["n_trials"] = n
    out["sec_per_paired_trial"] = out["A"]["mean_sec_per_episode"] + out["B"]["mean_sec_per_episode"]
    p = ROOT / "runs" / "benchmark" / f"bench_{n}.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
