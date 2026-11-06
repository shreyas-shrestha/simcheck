"""Full verifier run for trusted modules (reference / broken variants), in-process."""
import json, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck import broken, reference, verifier as V  # noqa: E402
from simcheck.sandbox import InProcessRunner  # noqa: E402

name, task, seed_set = sys.argv[1], sys.argv[2], sys.argv[3]
mod = reference if name == "reference" else broken.VARIANTS[name]
rep = V.verify(InProcessRunner(mod), task, seed_set, out_path=ROOT / "runs" / "_trusted" / name / task / seed_set / "report.json")
L = rep["layers"]
print(json.dumps({"accepted": rep["accepted"], "reward": rep["reward"],
                  "layers": {k: v.get("pass") for k, v in L.items() if k != "structural"},
                  "route": {k: L["route"][k] for k in ("chosen", "stated_success_prob", "regret", "regret_ci95", "regret_outcome", "diff_ci95")}
                  | {"rates": {r: (v["success_rate"], [round(x, 3) for x in v["wilson95"]]) for r, v in L["route"]["routes"].items()}},
                  "feedback": rep["feedback"], "seconds": round(rep["seconds"], 1)}, indent=1))
