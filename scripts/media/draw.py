"""Drawing primitives shared by the deck stills and the demo clips (flat style, no gradients or glows)."""
import math

import numpy as np

from media import common as C

MAP_XLIM = (-0.9, 12.9)
MAP_YLIM = (-1.3, 6.4)


def new_fig(w=C.W, h=C.H, transparent=False):
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(w / C.DPI, h / C.DPI), dpi=C.DPI)
    if transparent:
        fig.patch.set_alpha(0.0)
    return fig


def map_axes(fig, rect=(0.03, 0.06, 0.94, 0.88), xlim=MAP_XLIM, ylim=MAP_YLIM):
    ax = fig.add_axes(rect)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    return ax


def hallway_rect(spec):
    xs = [w[0] for w in spec["walls"]] + [w[2] for w in spec["walls"]]
    ys = [w[1] for w in spec["walls"]]
    return min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys)


def frame_rgb(fig):
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()


def draw_hallway(ax, spec, alpha=1.0, label=True):
    from matplotlib.patches import Rectangle
    x, y, w, h = hallway_rect(spec)
    ax.add_patch(Rectangle((x, y), w, h, facecolor="#24262B", edgecolor="none", alpha=alpha, zorder=1))
    if label:
        ax.text(x + w / 2, y - 0.42, "featureless hallway", ha="center", va="top", family=C.MONO,
                fontsize=C.px(24), color=C.GRAY, alpha=alpha, zorder=6)


def draw_walls(ax, spec, alpha=1.0):
    for w in spec["walls"]:
        ax.plot([w[0], w[2]], [w[1], w[3]], color=C.FG, lw=5, solid_capstyle="butt", alpha=alpha, zorder=5)


def draw_landmarks(ax, spec, alpha=1.0, label=True):
    lm = np.array(list(spec["landmarks"].values()), float).reshape(-1, 2)
    if len(lm):
        ax.scatter(lm[:, 0], lm[:, 1], marker="^", s=260, color=C.GRAY, edgecolor="none", alpha=alpha, zorder=6)
        if label:
            ax.text(lm[:, 0].mean(), lm[0, 1] + 0.38, "landmarks", ha="center", va="bottom", family=C.MONO,
                    fontsize=C.px(24), color=C.GRAY, alpha=alpha, zorder=6)


def _partial(pts, frac):
    pts = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    s = frac * cum[-1]
    i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(seg) - 1)
    u = 0 if seg[i] == 0 else (s - cum[i]) / seg[i]
    return np.vstack([pts[: i + 1], pts[i] + u * (pts[i + 1] - pts[i])])


def draw_route(ax, spec, name, frac=1.0, label=True, alpha=1.0, lw=4):
    color = C.ORANGE if name == "A" else C.GREEN
    pts = _partial(spec["routes"][name], frac)
    ax.plot(pts[:, 0], pts[:, 1], color=color, lw=lw, alpha=alpha, solid_capstyle="round", zorder=3,
            dashes=(4, 2.5))
    if label and frac >= 1.0:
        if name == "A":
            ax.text(1.0, -0.62, "A  short, no landmarks", ha="left", va="top", fontsize=C.px(28),
                    color=color, alpha=alpha, zorder=7, weight="medium")
        else:
            p = np.asarray(spec["routes"][name], float)
            ax.text(p[1:3, 0].mean(), p[1, 1] + 0.32, "B  longer, past landmarks", ha="center", va="bottom",
                    fontsize=C.px(28), color=color, alpha=alpha, zorder=7, weight="medium")


def draw_robot(ax, pose, spec, color=C.FG, alpha=1.0, zorder=9):
    from matplotlib.patches import Circle
    r = spec["robot_radius"]
    ax.add_patch(Circle(pose[:2], r, facecolor=C.BG, edgecolor=color, lw=3, alpha=alpha, zorder=zorder))
    ax.plot([pose[0], pose[0] + r * math.cos(pose[2])], [pose[1], pose[1] + r * math.sin(pose[2])],
            color=color, lw=3, alpha=alpha, zorder=zorder + 1)


def draw_start(ax, spec, alpha=1.0):
    draw_robot(ax, np.asarray(spec["initial_pose"]["mean"], float), spec, color=C.FG, alpha=alpha)
    ax.text(-0.05, -0.42, "start", ha="center", va="top", family=C.MONO, fontsize=C.px(24),
            color=C.GRAY, alpha=alpha)


def draw_world(ax, spec, routes=("A", "B"), labels=True):
    draw_hallway(ax, spec, label=labels)
    draw_walls(ax, spec)
    draw_landmarks(ax, spec, label=labels)
    for r in routes:
        draw_route(ax, spec, r, label=labels, alpha=0.55 if not labels else 1.0)


def ellipse_patch(est, P, inside):
    from matplotlib.patches import Ellipse
    Cw = C.world_pos_cov(est, P)
    vals, vecs = np.linalg.eigh(Cw)
    ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
    w, h = 2 * np.sqrt(C.CHI2_2D_95 * np.maximum(vals[::-1], 1e-12))
    color = C.AMBER if inside else C.RED
    return Ellipse(est[:2], w, h, angle=ang, fill=False, edgecolor=color, lw=4, zorder=8)


def dropout_steps(spec):
    gs = spec.get("gps_schedule") or {}
    return gs.get("dropout_steps")


def draw_rollout(ax, spec, ro, k, route_ghost=True, show_dropout=True, robot=True):
    """State of rollout `ro` at (possibly fractional) step k: truth white, estimate amber, ellipse amber or red."""
    tr, est, cov, inside = ro["truth"], ro["est"], ro["cov"], ro["inside"]
    n = len(tr)
    k = float(np.clip(k, 0, n - 1))
    k0 = int(math.floor(k))
    u = k - k0
    k1 = min(k0 + 1, n - 1)

    def interp(a):
        return a[k0] + u * (a[k1] - a[k0])

    if route_ghost:
        draw_route(ax, spec, ro["route"], label=False, alpha=0.35, lw=3)
    d = dropout_steps(spec)
    if show_dropout and d and d[0] <= k:
        a, b = d[0], min(d[1], k0)
        seg = tr[a: b + 1, :2]
        if len(seg) >= 1:
            seg = np.vstack([seg, interp(tr)[:2]]) if k0 < d[1] else seg
            ax.plot(seg[:, 0], seg[:, 1], color=C.GRAY, lw=26, alpha=0.28, solid_capstyle="butt", zorder=2)
    T = np.vstack([tr[: k0 + 1, :2], interp(tr)[:2]])
    E = np.vstack([est[: k0 + 1, :2], interp(est)[:2]])
    ax.plot(T[:, 0], T[:, 1], color=C.TRUTH, lw=3.5, zorder=6)
    ax.plot(E[:, 0], E[:, 1], color=C.AMBER, lw=3.5, zorder=7)
    ki = int(round(k))
    ax.add_patch(ellipse_patch(est[ki], cov[ki], bool(inside[ki])))
    ax.scatter([est[ki, 0]], [est[ki, 1]], s=70, color=C.AMBER, zorder=9)
    if robot:
        draw_robot(ax, interp(tr), spec, color=C.TRUTH)
    return ki


def end_marker(ax, ro):
    o = ro["outcome"]
    p = ro["truth"][-1]
    if o["collision"]:
        ax.scatter([p[0]], [p[1]], marker="X", s=900, color=C.RED, edgecolor=C.BG, lw=2, zorder=12)
    elif o["arrived"]:
        ax.scatter([p[0]], [p[1]], marker="o", s=700, facecolor="none", edgecolor=C.GREEN, lw=5, zorder=12)


def outcome_text(o):
    if o["collision"]:
        return "COLLISION", C.RED
    if o["arrived"]:
        return "ARRIVED", C.GREEN
    return "DID NOT ARRIVE", C.GRAY
