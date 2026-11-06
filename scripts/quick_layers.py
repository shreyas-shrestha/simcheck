"""Fast diagnostic: layers 2-3 only (no route trials) for reference and broken variants on a seed set."""
import json, sys, time
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck import broken, reference, verifier as V  # noqa: E402
from simcheck.sandbox import InProcessRunner  # noqa: E402

seed_set = sys.argv[1] if len(sys.argv) > 1 else "dev"
th, K = V.CONFIG["thresholds"], V.CONFIG["thresholds"]["calibration_K"]
cases = [("reference", reference, t) for t in V.TASKS] + [
    ("cov_x0.1", broken.VARIANTS["cov_x0.1"], "standard"), ("cov_x10", broken.VARIANTS["cov_x10"], "standard"),
    ("wrong_odom_frame", broken.VARIANTS["wrong_odom_frame"], "standard"),
    ("missing_prior", broken.VARIANTS["missing_prior"], "prior_needed")]
for name, mod, task in cases:
    t0 = time.perf_counter()
    scen = V.load_scenarios(task, seed_set, K)
    est = V.run_datasets(InProcessRunner(mod), scen)
    ref = V.reference_datasets(task, seed_set, K)
    a = V.layer2_accuracy(est, ref, scen, th); c = V.layer3_calibration(est, ref, scen, th)
    ts = {k: f"cov {v['covered']}/{K} p={v['coverage_p_value']:.3f} NEES {v['mean_nees']:.2f}(lb {v['nees_lower_bound']:.2f}) "
             f"NLL {v['mean_nll']:.2f} vs ref {v['reference_mean_nll']:.2f} -> {'PASS' if v['pass'] else 'FAIL'}"
          for k, v in c["times"].items()}
    print(f"{name:17s} {task:13s} ATE {a['ate_m']:.3f} (ref {a['reference_ate_m']:.3f}, bound {a['bound_m']:.3f}) "
          f"{'PASS' if a['pass'] else 'FAIL'} | {time.perf_counter()-t0:.1f}s")
    for k, v in ts.items():
        print(f"    {k:18s} {v}")
