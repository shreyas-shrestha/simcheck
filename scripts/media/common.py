"""Shared data, style and rollout replay for the deck stills and demo clips.

Everything is offline: numbers come from results/summary/summary.json or the saved run folders, and
rollouts replay the saved solution.py files through sim.py (Docker if available, else the subprocess
fallback; the runner used is recorded).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from simcheck import generator, sim, verifier as V  # noqa: E402
from simcheck.sandbox import RemoteEstimator, make_runner  # noqa: E402

RESULTS = ROOT / "results"
SUMMARY = RESULTS / "summary" / "summary.json"
FEATURED = RESULTS / "v3.2" / "argon-medium" / "r2" / "unknown_noise"
FEATURED_TASK = "unknown_noise"
ATTEMPTS = {"attempt1": "1", "final": "2"}  # final = last attempt; accepted on eval and on fresh seeds
ROLLOUT_ROUTE = "A"  # both attempts chose B, where all 200 trials succeeded for both; A is where they differ
SEED_SET = "eval"
SELECT_RNG_SEED = 20261106
CACHE = ROOT / "build" / "media_cache"
DECK = ROOT / "demo" / "deck_assets"
CLIPS = ROOT / "demo" / "demo_clips"
FONTS = ROOT / "assets" / "fonts"
CHI2_2D_95 = 5.991464547107979  # 95% cutoff, 2 dof (position ellipse)

# ---------------------------------------------------------------- style

W, H, DPI = 1920, 1080, 100
BG = "#121316"
FG = "#ECEAE4"
AMBER = "#F2C14E"
GRAY = "#A3A39C"
RED = "#FF5A5A"
ORANGE = "#F28C38"  # route A
GREEN = "#5CC97B"  # route B, arrival
TRUTH = "#FFFFFF"
GRID = "#2A2C31"
SANS, MONO = "IBM Plex Sans", "IBM Plex Mono"
MIN_PT = 24 * 72 / DPI  # 24 px minimum text size, in points at this dpi


def px(n):
    """Pixels -> points at the canvas dpi (font sizes are given in px)."""
    return max(n, 24) * 72 / DPI


def setup_matplotlib():
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager, rcParams
    missing = []
    for f in ("IBMPlexSans-Regular", "IBMPlexSans-Medium", "IBMPlexSans-SemiBold", "IBMPlexSans-Bold",
              "IBMPlexMono-Regular", "IBMPlexMono-Medium", "IBMPlexMono-SemiBold"):
        p = FONTS / f"{f}.ttf"
        if not p.exists():
            missing.append(p.name)
            continue
        font_manager.fontManager.addfont(str(p))
    if missing:
        raise SystemExit(f"IBM Plex fonts missing in {FONTS}: {missing} (see scripts/media/README.md)")
    rcParams.update({
        "font.family": SANS, "font.size": px(28), "text.color": FG, "axes.labelcolor": FG,
        "axes.edgecolor": GRAY, "xtick.color": GRAY, "ytick.color": GRAY, "axes.facecolor": BG,
        "figure.facecolor": BG, "savefig.facecolor": BG, "axes.titlesize": px(32), "axes.labelsize": px(26),
        "xtick.labelsize": px(24), "ytick.labelsize": px(24), "legend.fontsize": px(24),
        "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False,
        "lines.antialiased": True, "patch.antialiased": True,
    })


# ---------------------------------------------------------------- data

def jload(p):
    return json.loads(Path(p).read_text())


def summary():
    return jload(SUMMARY)


def rel(p):
    return str(Path(p).resolve().relative_to(ROOT))


def featured_report(name):
    return jload(FEATURED / ATTEMPTS[name] / "report.json") if name in ATTEMPTS else jload(FEATURED / name / "report.json")


def task_spec():
    return V.task_spec(FEATURED_TASK)


def route_seed(i):
    return V.seed_base(SEED_SET) + V.ROUTE_SEED_OFFSET + i


def recorded_outcome(name, route, i):
    """The verifier's recorded per-trial outcome for attempt `name` (from artifacts.npz)."""
    z = np.load(FEATURED / ATTEMPTS[name] / "artifacts.npz", allow_pickle=True)
    return {"success": bool(z[f"route_{route}_success"][i]), "collision": bool(z[f"route_{route}_collision"][i])}


def world_pos_cov(pose, P):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return R @ np.asarray(P)[:2, :2] @ R.T


def truth_inside(est, cov, truth):
    """Per step: is the true position inside the 95% position ellipse (2D block, rotated to world)?"""
    out = []
    for e, P, t in zip(est, cov, truth):
        C = world_pos_cov(e, P)
        d = np.asarray(t[:2]) - np.asarray(e[:2])
        out.append(float(d @ np.linalg.solve(C, d)) <= CHI2_2D_95)
    return np.array(out, bool)


# ---------------------------------------------------------------- rollouts

def _runner(name):
    sol = FEATURED / ATTEMPTS[name] / "solution.py"
    return make_runner(sol, prefer_docker=True)


def rollout_one(runner, name, route, i):
    spec = task_spec()
    seed = route_seed(i)
    ep = sim.run_episode(generator.true_spec(spec, seed), route, seed, estimator=RemoteEstimator(runner, spec))
    inside = truth_inside(ep["est"], ep["cov"], ep["truth"])
    return {"truth": ep["truth"], "est": ep["est"], "cov": ep["cov"], "inside": inside,
            "outcome": ep["outcome"], "seed": seed, "index": i, "route": route, "attempt": name,
            "odom_noise_scale": generator.odom_noise_scale(spec, seed)}


def rollout(name, route, i, runner=None):
    """One closed-loop rollout of a saved solution, cached on disk."""
    f = CACHE / f"{name}_{route}_{i:03d}.npz"
    if f.exists():
        z = np.load(f, allow_pickle=True)
        return {k: (z[k].item() if z[k].ndim == 0 else z[k]) for k in z.files}
    own = runner is None
    runner = runner or _runner(name)
    try:
        r = rollout_one(runner, name, route, i)
        r["isolation"] = runner.isolation
    finally:
        if own:
            runner.close()
    CACHE.mkdir(parents=True, exist_ok=True)
    np.savez(f, **{k: (np.array(v, dtype=object) if isinstance(v, (dict, str)) else v) for k, v in r.items()})
    return r


def all_rollouts(route=ROLLOUT_ROUTE, T=None, progress=print):
    T = T or V.CONFIG["route_trials"]["T"]
    out = {}
    for name in ATTEMPTS:
        need = [i for i in range(T) if not (CACHE / f"{name}_{route}_{i:03d}.npz").exists()]
        runner = _runner(name) if need else None
        try:
            for i in range(T):
                out[(name, i)] = rollout(name, route, i, runner)
                if progress and (i + 1) % 50 == 0:
                    progress(f"{name} route {route}: {i + 1}/{T}")
        finally:
            if runner:
                runner.close()
    return out


def select_seed(route=ROLLOUT_ROUTE):
    """One eval route-trial seed, chosen at random (fixed RNG) among seeds where the outcome or the
    truth-inside-ellipse rate differs between attempt 1 and the final attempt."""
    R = all_rollouts(route)
    T = len(R) // 2
    outcome_diff, rate_diff = [], []
    for i in range(T):
        a, b = R[("attempt1", i)], R[("final", i)]
        if a["outcome"]["success"] != b["outcome"]["success"] or a["outcome"]["collision"] != b["outcome"]["collision"]:
            outcome_diff.append(i)
        if not math.isclose(float(a["inside"].mean()), float(b["inside"].mean()), abs_tol=1e-12):
            rate_diff.append(i)
    rule_cands = sorted(set(outcome_diff) | set(rate_diff))
    original_pick = int(np.random.default_rng(SELECT_RNG_SEED).choice(rule_cands))

    def mid_miss(i):
        ins = R[("attempt1", i)]["inside"]
        n = len(ins)
        return any(not ins[k] for k in range(n) if 0.25 * n <= k <= 0.75 * n)
    # Refinement agreed with the user: the original pick only missed at step 3, so there was no
    # mid-route red ellipse to show. Same RNG, candidates also need an attempt-1 miss mid-route.
    cands = [i for i in rule_cands if mid_miss(i)]
    pick = int(np.random.default_rng(SELECT_RNG_SEED).choice(cands))
    a, b = R[("attempt1", pick)], R[("final", pick)]
    return {
        "seed": route_seed(pick), "trial_index": pick, "route": route, "seed_set": SEED_SET,
        "rule": "uniform random choice (numpy default_rng(%d).choice) over eval route-trial indices where the "
                "rollout outcome OR the per-step truth-inside-95%%-ellipse rate differs between attempt 1 and "
                "the final attempt" % SELECT_RNG_SEED,
        "route_note": "both attempts chose route B, where all 200 trials succeeded for both; route A (the "
                      "featureless hallway) is where outcomes differ, so rollouts use route A",
        "refinement": "candidates further restricted to seeds where attempt 1's ellipse misses the truth "
                      "at least once in the middle half of the route (0.25n <= step <= 0.75n); requested by the "
                      "user after the unrefined rule picked seed %d, whose only miss was at step 3"
                      % route_seed(original_pick),
        "unrefined_pick": {"seed": route_seed(original_pick), "trial_index": original_pick,
                           "n_candidates": len(rule_cands)},
        "n_trials": T, "n_candidates": len(cands), "n_outcome_differs": len(outcome_diff),
        "n_inside_rate_differs": len(rate_diff), "outcome_differs_indices": outcome_diff,
        "attempt1": {"outcome": a["outcome"], "inside_rate": float(a["inside"].mean()),
                     "recorded": recorded_outcome("attempt1", route, pick)},
        "final": {"outcome": b["outcome"], "inside_rate": float(b["inside"].mean()),
                  "recorded": recorded_outcome("final", route, pick)},
        "odom_noise_scale": a["odom_noise_scale"], "isolation": str(a["isolation"]),
    }


def featured_seed():
    """Cached selection (recomputed if the cache is missing)."""
    f = CACHE / "selection.json"
    if f.exists():
        return jload(f)
    s = select_seed()
    CACHE.mkdir(parents=True, exist_ok=True)
    f.write_text(json.dumps(s, indent=2))
    return s


if __name__ == "__main__":
    print(json.dumps(featured_seed(), indent=2))
