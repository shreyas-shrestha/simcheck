"""Milestone 1: one reference-controlled episode per route (same trial seed), plotted."""
import json
import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from simcheck import generator, reference, sim  # noqa: E402

CHI2_2D_95 = 5.99


def world_pos_cov(pose, P):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return R @ P[:2, :2] @ R.T


def plot_episode(ax, spec, ep, route, title):
    for w in spec["walls"]:
        ax.plot([w[0], w[2]], [w[1], w[3]], "k-", lw=3)
    for r, pts in spec["routes"].items():
        pts = np.array(pts)
        ax.plot(pts[:, 0], pts[:, 1], ":", color="0.6" if r != route else "0.3", lw=1)
        ax.annotate(f"route {r}", pts[len(pts) // 2 - (len(pts) == 2)] + [0.2, 0.15], color="0.4", fontsize=8)
    lm = np.array(list(spec["landmarks"].values()))
    ax.scatter(lm[:, 0], lm[:, 1], marker="^", c="tab:green", s=40, label="landmarks")
    tr, est = ep["truth"], ep["est"]
    ax.plot(tr[:, 0], tr[:, 1], "-", c="tab:blue", lw=1.8, label="true")
    ax.plot(est[:, 0], est[:, 1], "--", c="tab:orange", lw=1.4, label="estimate")
    d0, d1 = spec["gps_schedule"]["dropout_steps"]
    d1 = min(d1, len(tr) - 1)
    if d0 < len(tr):
        ax.plot(tr[d0:d1 + 1, 0], tr[d0:d1 + 1, 1], "-", c="tab:red", lw=3, alpha=0.35, label="GPS dropout")
    for k in range(0, len(est), 3):
        C = world_pos_cov(est[k], ep["cov"][k])
        vals, vecs = np.linalg.eigh(C)
        ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
        w, h = 2 * np.sqrt(CHI2_2D_95 * vals[::-1])
        ax.add_patch(Ellipse(est[k, :2], w, h, angle=ang, fill=False, ec="tab:orange", lw=0.7, alpha=0.8))
    o = ep["outcome"]
    end = tr[-1]
    if o["collision"]:
        ax.plot(*end[:2], "rX", ms=14, label="collision")
    elif o["arrived"]:
        ax.plot(*end[:2], "g*", ms=16, label="arrived")
    status = "COLLISION" if o["collision"] else ("ARRIVED" if o["arrived"] else "TIMEOUT")
    ax.set_title(f"{title}: {status} at step {o['steps']}")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")


def main(trial_seed=int(sys.argv[1]) if len(sys.argv) > 1 else 0):
    spec = generator.build_spec(generator.ScenarioConfig(seed=0))
    out = ROOT / "runs" / "milestone1"
    out.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 1, figsize=(11, 9))
    summary = {}
    for ax, route in zip(axes, "AB"):
        t0 = time.perf_counter()
        ep = sim.run_episode(spec, route, trial_seed, estimator=reference.make_estimator(spec))
        dtime = time.perf_counter() - t0
        err = np.linalg.norm(ep["est"][:, :2] - ep["truth"][:, :2], axis=1)
        summary[route] = {**ep["outcome"], "seconds": round(dtime, 3),
                          "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))), "max_pos_err_m": float(err.max())}
        plot_episode(ax, spec, ep, route, f"Route {route}, reference estimator in the loop, trial seed {trial_seed}")
    axes[0].legend(loc="center left", bbox_to_anchor=(0.0, 0.42), fontsize=8, ncol=3)
    fig.tight_layout()
    fig.savefig(out / f"episode_seed{trial_seed}.png", dpi=130)
    (out / f"episode_seed{trial_seed}.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
