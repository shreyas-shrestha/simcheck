"""Demo clips -> demo_clips/ (MP4, H.264 -preset veryfast, 30 fps) and demo_clips/manifest.json.

    .venv/bin/python scripts/media/clips.py            # all clips
    .venv/bin/python scripts/media/clips.py demo04     # only clips whose name starts with demo04
"""
import json
import sys
import textwrap
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from media import common as C  # noqa: E402

C.setup_matplotlib()
import imageio_ffmpeg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from media import data as Dt, draw as Dr  # noqa: E402
from media.deck import bars, featured_rollouts, miss_step  # noqa: E402

FPS = 30
MANIFEST = {}


def ease(u):
    u = float(np.clip(u, 0, 1))
    return u * u * (3 - 2 * u)


def ramp(t, t0, t1):
    """0 before t0, eased 0..1 between t0 and t1, 1 after."""
    return ease((t - t0) / (t1 - t0)) if t1 > t0 else float(t >= t0)


def write_clip(name, seconds, draw, sources, numbers, notes=None):
    """Render draw(fig, t) for every frame and encode with libx264 -preset veryfast."""
    C.CLIPS.mkdir(parents=True, exist_ok=True)
    path = C.CLIPS / f"{name}.mp4"
    n = int(round(seconds * FPS))
    w = imageio_ffmpeg.write_frames(str(path), (C.W, C.H), fps=FPS, codec="libx264", pix_fmt_out="yuv420p",
                                    output_params=["-preset", "veryfast", "-crf", "18"], macro_block_size=8)
    w.send(None)
    fig = Dr.new_fig()
    try:
        for f in range(n):
            fig.clf()
            draw(fig, f / FPS)
            w.send(Dr.frame_rgb(fig).tobytes())
    finally:
        w.close()
        plt.close(fig)
    MANIFEST[name] = {"file": f"{name}.mp4", "duration_s": n / FPS, "frames": n, "fps": FPS,
                      "codec": "H.264 (libx264 -preset veryfast -crf 18, yuv420p)", "size": [C.W, C.H],
                      "sources": sources, "numbers_shown": numbers, **({"notes": notes} if notes else {})}
    print(f"wrote {path.name} ({n / FPS:.1f}s)")


def text(fig, x, y, s, size=28, color=C.FG, alpha=1.0, mono=False, **kw):
    if alpha <= 0:
        return None
    return fig.text(x, y, s, fontsize=C.px(size), color=color, alpha=alpha, family=C.MONO if mono else C.SANS, **kw)


def typed(s, u):
    """First fraction u of string s (typewriter reveal)."""
    return s[: int(round(len(s) * float(np.clip(u, 0, 1))))]


def check_rows(fig, rows, t0, t, x=0.1, y0=0.78, dy=0.1, step=0.35):
    for j, (label, ok) in enumerate(rows):
        a = ramp(t, t0 + j * step, t0 + j * step + 0.3)
        text(fig, x, y0 - j * dy, label, 40, alpha=a, weight="medium")
        text(fig, x + 0.2, y0 - j * dy, "PASS" if ok else "FAIL", 40, C.GREEN if ok else C.RED, a, mono=True,
             weight="semibold")


# ---------------------------------------------------------------- clips

def demo01_title():
    def draw(fig, t):
        text(fig, 0.5, 0.53, "SimCheck", 120, alpha=ramp(t, 0.2, 1.2), ha="center", weight="semibold")
        text(fig, 0.5, 0.42, "Does the agent know what it doesn't know?", 44, C.AMBER, ramp(t, 1.2, 2.2),
             ha="center")
    write_clip("demo01_title", 5, draw, [], {})


def demo02_world_map():
    spec = C.task_spec()

    def draw(fig, t):
        ax = Dr.map_axes(fig)
        a_h, a_w, a_l = ramp(t, 0.2, 1.4), ramp(t, 1.2, 2.4), ramp(t, 2.4, 3.6)
        if a_h > 0:
            Dr.draw_hallway(ax, spec, alpha=a_h)
        if a_w > 0:
            Dr.draw_walls(ax, spec, alpha=a_w)
        if a_l > 0:
            Dr.draw_landmarks(ax, spec, alpha=a_l)
        for r, (t0, t1) in (("A", (3.6, 5.0)), ("B", (5.0, 6.6))):
            fr = ramp(t, t0, t1)
            if fr > 0:
                Dr.draw_route(ax, spec, r, frac=max(fr, 1e-3), label=fr >= 1.0)
        a_s = ramp(t, 6.4, 7.2)
        if a_s > 0:
            Dr.draw_start(ax, spec, alpha=a_s)
    write_clip("demo02_world_map", 8, draw, ["simcheck.verifier.task_spec('unknown_noise') (config_v3.2.json)"],
               {"landmarks": len(spec["landmarks"]), "routes": ["A", "B"]})


def demo03_agent_plan_attempt1():
    out, src = Dt.choose_route_output("attempt1")
    noise, nsrc = Dt.unknown_noise_task_line()
    call = f'choose_route() -> route "{out["route"]}", success_prob {out["success_prob"]}'
    reason = textwrap.fill(out["reasoning"], 88)

    def draw(fig, t):
        text(fig, 0.07, 0.88, "Attempt 1: the agent's plan", 44, weight="semibold")
        text(fig, 0.07, 0.75, typed(call, ramp(t, 0.3, 1.6)), 36, C.AMBER, mono=True)
        text(fig, 0.07, 0.66, reason, 26, C.FG, ramp(t, 1.6, 2.4), mono=True, va="top", linespacing=1.5)
        a = ramp(t, 3.2, 4.0)
        text(fig, 0.07, 0.25, "task statement", 24, C.GRAY, a, mono=True)
        text(fig, 0.07, 0.2, textwrap.fill(noise, 96), 26, C.FG, a, mono=True, va="top", linespacing=1.5)
    write_clip("demo03_agent_plan_attempt1", 6, draw, [src, nsrc],
               {"route": out["route"], "success_prob": out["success_prob"], "reasoning": out["reasoning"],
                "task_line": noise})


def _rollout_header(fig, title, sel):
    text(fig, 0.04, 0.9, title, 44, weight="semibold")
    text(fig, 0.04, 0.84, "white: true path    amber: estimate + 95% ellipse    gray band: GPS dropout", 24,
         C.GRAY, mono=True)
    text(fig, 0.04, 0.05, f"Example trial, seed {sel['seed']}.", 26, C.GRAY, mono=True)


def _step_schedule(t, k_miss, n, t_run1, t_hold, t_run2):
    """Step index over time: 0 -> k_miss, hold on the miss, then k_miss -> n-1."""
    if t < t_run1:
        return k_miss * t / t_run1
    if t < t_run1 + t_hold:
        return float(k_miss)
    return min(k_miss + (n - 1 - k_miss) * (t - t_run1 - t_hold) / t_run2, n - 1)


ROLL_XLIM, ROLL_YLIM = (-0.9, 12.9), (-2.4, 2.4)


def _dropout_label(ax, spec, ro, k):
    d = Dr.dropout_steps(spec)
    if d and k >= d[0]:
        p = ro["truth"][d[0]]
        ax.text(p[0], -1.35, "GPS dropout", ha="left", va="top", family=C.MONO, fontsize=C.px(24), color=C.GRAY)


def demo04_rollout_attempt1_overconfident(sel, a):
    spec = C.task_spec()
    k_miss, m = miss_step(a)
    n = len(a["truth"])
    T1, HOLD, T2 = 7.0, 1.5, 3.0
    t_end = T1 + HOLD + T2
    label, color = Dr.outcome_text(a["outcome"])

    def draw(fig, t):
        _rollout_header(fig, "Attempt 1 on route A", sel)
        ax = Dr.map_axes(fig, (0.02, 0.14, 0.96, 0.66), xlim=ROLL_XLIM, ylim=ROLL_YLIM)
        Dr.draw_hallway(ax, spec, label=False)
        Dr.draw_walls(ax, spec)
        k = _step_schedule(t, k_miss, n, T1, HOLD, T2)
        ki = Dr.draw_rollout(ax, spec, a, k)
        _dropout_label(ax, spec, a, k)
        if not a["inside"][ki]:
            text(fig, 0.96, 0.9, "truth outside 95% ellipse", 30, C.RED, mono=True, ha="right")
        text(fig, 0.96, 0.84, f"step {ki}", 24, C.GRAY, mono=True, ha="right")
        if t >= t_end:
            Dr.end_marker(ax, a)
            text(fig, 0.5, 0.14, label, 56, color, ramp(t, t_end, t_end + 0.4), ha="center", weight="semibold")
    rec = C.recorded_outcome("attempt1", sel["route"], sel["trial_index"])
    write_clip("demo04_rollout_attempt1_overconfident", 14, draw,
               ["rollout replay of " + C.rel(C.FEATURED / "1" / "solution.py") + " through simcheck/sim.py",
                "build/media_cache/selection.json", C.rel(C.FEATURED / "1" / "artifacts.npz")],
               {"seed": sel["seed"], "route": sel["route"], "outcome": label, "steps": n,
                "miss_steps": [int(i) for i in np.flatnonzero(~a["inside"])], "held_on_step": int(k_miss),
                "recorded_outcome": rec},
               notes=f"isolation: {a['isolation']}. Attempt 1 chose route B for its graded run; route A is shown "
                     "because that is where attempt 1 and the final attempt differ.")


def demo05_calibration_tally_attempt1():
    nees, src = Dt.per_seed_nees("attempt1")
    cal, csrc = Dt.calibration("attempt1")
    gate = C.V.NEES_GATE_3
    K = len(nees)

    def draw(fig, t):
        text(fig, 0.07, 0.88, "Attempt 1: is the truth inside the 95% region?", 44, weight="semibold")
        text(fig, 0.07, 0.82, f"{K} eval seeds, end of trajectory (NEES <= 7.81)", 26, C.GRAY, mono=True)
        shown = int(np.clip((t - 0.5) / 4.5 * K, 0, K))
        ax = fig.add_axes((0.07, 0.42, 0.86, 0.3))
        ax.set_xlim(0, 15); ax.set_ylim(0, 2); ax.axis("off"); ax.set_aspect("equal")
        from matplotlib.patches import Rectangle
        for i in range(K):
            x, y = i % 15, 1 - i // 15
            if i < shown:
                inside = nees[i] <= gate
                ax.add_patch(Rectangle((x + 0.08, y + 0.08), 0.84, 0.84, facecolor=C.AMBER if inside else C.RED,
                                       edgecolor="none"))
            else:
                ax.add_patch(Rectangle((x + 0.08, y + 0.08), 0.84, 0.84, facecolor="none", edgecolor=C.GRID, lw=2))
        covered = sum(v <= gate for v in nees[:shown])
        bx = fig.add_axes((0.07, 0.2, 0.86, 0.09))
        bx.set_xlim(0, 1); bx.set_ylim(0, 1); bx.axis("off")
        bx.barh(0.5, 1, 0.7, color=C.GRID)
        bx.barh(0.5, covered / K, 0.7, color=C.AMBER)
        bx.axvline(0.95, color=C.FG, lw=3, ls=(0, (5, 3)))
        text(fig, 0.07 + 0.86 * 0.95, 0.31, "0.95 target", 24, ha="center", mono=True)
        text(fig, 0.07, 0.11, f"{covered}/{K} inside = {covered / K:.2f}", 36,
             C.RED if shown == K and covered / K < 0.95 else C.FG, mono=True, weight="semibold")
    write_clip("demo05_calibration_tally_attempt1", 8, draw, [src, csrc],
               {"covered": cal["covered"], "K": cal["K"], "coverage95": cal["coverage95"], "target": 0.95,
                "gate": "NEES <= 7.81 (chi2 3 dof, 95%)",
                "per_seed_nees": [round(v, 3) for v in nees]})


def demo06_verifier_feedback():
    rows, src = Dt.checklist("attempt1")
    fb, fsrc = Dt.featured_feedback()
    body = "\n".join(textwrap.fill(l, 70) for l in fb)

    def draw(fig, t):
        text(fig, 0.1, 0.9, "SimCheck verdict, attempt 1", 44, weight="semibold")
        check_rows(fig, rows, 0.2, t, y0=0.74, dy=0.09)
        a = ramp(t, 2.2, 2.6)
        text(fig, 0.1, 0.33, "feedback to the agent", 24, C.GRAY, a, mono=True)
        text(fig, 0.1, 0.28, typed(body, ramp(t, 2.4, 4.6)), 30, C.AMBER, mono=True, va="top", linespacing=1.5)
    write_clip("demo06_verifier_feedback", 6, draw, [src, fsrc],
               {"checklist": {k: v for k, v in rows}, "feedback": fb})


def demo07_before_after_same_seed(sel, a, b):
    spec = C.task_spec()
    k_miss, _ = miss_step(a)
    n = max(len(a["truth"]), len(b["truth"]))
    T1, HOLD, T2 = 5.5, 1.5, 3.0
    t_end = T1 + HOLD + T2
    rows, src = Dt.checklist("final")
    fresh, fsrc = Dt.fresh_accepted()
    assert fresh and all(ok for _, ok in rows)

    def draw(fig, t):
        if t < t_end + 1.5:
            k = _step_schedule(t, k_miss, n, T1, HOLD, T2)
            for i, (ro, title) in enumerate(((a, "Attempt 1"), (b, "After feedback"))):
                ax = Dr.map_axes(fig, (0.01 + i * 0.5, 0.14, 0.48, 0.62), xlim=(1.0, 10.0), ylim=(-2.2, 2.2))
                Dr.draw_hallway(ax, spec, label=False)
                Dr.draw_walls(ax, spec)
                ki = Dr.draw_rollout(ax, spec, ro, k)
                text(fig, 0.03 + i * 0.5, 0.88, title, 40, weight="semibold")
                ins = bool(ro["inside"][ki])
                text(fig, 0.03 + i * 0.5, 0.82, "truth inside 95% ellipse" if ins else "truth outside 95% ellipse",
                     26, C.AMBER if ins else C.RED, mono=True)
                if k >= len(ro["truth"]) - 1:
                    Dr.end_marker(ax, ro)
                    lab, col = Dr.outcome_text(ro["outcome"])
                    text(fig, 0.25 + i * 0.5, 0.2, lab, 40, col, ha="center", weight="semibold")
            fig.add_artist(plt.Line2D([0.5, 0.5], [0.14, 0.92], color=C.GRID, lw=2))
            text(fig, 0.5, 0.05, f"Example trial, seed {sel['seed']}, route A, same seed for both.", 26, C.GRAY,
                 mono=True, ha="center")
        else:
            t2 = t - (t_end + 1.5)
            text(fig, 0.1, 0.9, "SimCheck verdict, after feedback", 44, weight="semibold")
            check_rows(fig, rows, 0.1, t2, y0=0.74, dy=0.09)
            text(fig, 0.1, 0.3, "Fresh seeds: pass.", 44, C.GREEN, ramp(t2, 1.6, 2.0), weight="semibold")
            text(fig, 0.1, 0.12, "Checklist grades the whole run (the agent's chosen route B);\n"
                 "the trial above is an example on route A.", 24, C.GRAY, ramp(t2, 1.6, 2.0), mono=True,
                 linespacing=1.5)
    recs = {nm: C.recorded_outcome(nm, sel["route"], sel["trial_index"]) for nm in ("attempt1", "final")}
    write_clip("demo07_before_after_same_seed", 15, draw,
               ["rollout replay of " + C.rel(C.FEATURED / "1" / "solution.py") + " and "
                + C.rel(C.FEATURED / "2" / "solution.py"), "build/media_cache/selection.json", src, fsrc],
               {"seed": sel["seed"], "route": sel["route"],
                "attempt1": {"outcome": Dr.outcome_text(a["outcome"])[0], "steps": len(a["truth"]),
                             "inside_rate": float(a["inside"].mean()), "recorded_outcome": recs["attempt1"]},
                "final": {"outcome": Dr.outcome_text(b["outcome"])[0], "steps": len(b["truth"]),
                          "inside_rate": float(b["inside"].mean()), "recorded_outcome": recs["final"]},
                "held_on_step": int(k_miss), "final_checklist": {k: v for k, v in rows}, "fresh_seeds_accepted": fresh},
               notes="Both attempts collide on this route-A trial (as the verifier recorded); the final checklist "
                     "is for the graded run, where both attempts chose route B.")


def demo08_results_52_to_84():
    rows = Dt.per_task()
    tot, tsrc = Dt.headline_totals()
    p = Dt.fmt_p(tot["mcnemar_exact_p"])
    end = (f"{tot['first_try']}/{tot['n']} → {tot['after_loop']}/{tot['after_loop_n']} · "
           f"{tot['fixed_by_feedback']} fixed, {tot['first_try_pass_lost']} broken · p = {p}")

    def draw(fig, t):
        g1, g2 = ramp(t, 0.3, 2.5), ramp(t, 2.8, 5.0)
        scaled = [{**r, "first_try": r["first_try"] * g1, "after": r["after"] * g2,
                   "first_try_wilson95": [r["first_try"] * g1 / r["n"]] * 2,
                   "after_wilson95": [r["after"] * g2 / r["n"]] * 2} for r in rows]
        ax = fig.add_axes((0.07, 0.2, 0.9, 0.58))
        final = t >= 5.2
        bars(ax, rows if final else scaled, "first_try", "after", ("first try", "after feedback, fresh seeds"),
             annotate=final)
        leg = ax.legend(loc="upper left", ncol=2, bbox_to_anchor=(0.0, 1.1), handlelength=1.2, columnspacing=2)
        for tx in leg.get_texts():
            tx.set_color(C.FG)
        text(fig, 0.07, 0.9, "Pass rate per task, Gemini 4 Argon", 44, weight="semibold")
        text(fig, 0.5, 0.05, end, 40, C.AMBER, ramp(t, 6.5, 7.3), ha="center", weight="semibold")
    write_clip("demo08_results_52_to_84", 10, draw, [tsrc] + [r["first_try_source"] for r in rows],
               {"per_task": [{k: r[k] for k in ("task", "n", "first_try", "after", "first_try_wilson95",
                                                "after_wilson95")} for r in rows],
                "end_line": end, "mcnemar_exact_p": tot["mcnemar_exact_p"]})


def demo09_accuracy_only_misses():
    tot, src = Dt.headline_totals()
    acc, wrong, n = tot["accuracy_only_grader_accepts"], tot["wrongly_accepted_by_accuracy_only"], tot["n"]
    l1 = f"An accuracy-only check accepts {acc}/{n} runs, {wrong} of them actually failing."

    def draw(fig, t):
        from matplotlib.patches import Rectangle
        ax = fig.add_axes((0.15, 0.42, 0.7, 0.28))
        ax.set_xlim(0, 13); ax.set_ylim(0, 2); ax.axis("off"); ax.set_aspect("equal")
        lit = int(np.clip((t - 0.3) / 1.5 * acc, 0, acc))
        catch = ramp(t, 3.0, 3.6)
        for i in range(n):
            x, y = i % 13, 1 - i // 13
            fc, ec = "none", C.GRID
            if i < lit:
                fc = C.AMBER
                if i >= acc - wrong and catch > 0:
                    fc = C.RED
            ax.add_patch(Rectangle((x + 0.1, y + 0.1), 0.8, 0.8, facecolor=fc, edgecolor=ec if fc == "none" else "none",
                                   lw=2))
        text(fig, 0.5, 0.82, l1, 36, ha="center", alpha=ramp(t, 0.2, 0.8))
        text(fig, 0.5, 0.25, "SimCheck catches them.", 48, C.RED, catch, ha="center", weight="semibold")
    write_clip("demo09_accuracy_only_misses", 6, draw, [src],
               {"accuracy_only_grader_accepts": acc, "wrongly_accepted_by_accuracy_only": wrong, "n": n, "text": l1},
               notes="cells are a count display only; their order does not identify specific runs")


def demo10_close():
    def draw(fig, t):
        text(fig, 0.5, 0.62, "The agent says it's confident.", 52, alpha=ramp(t, 0.2, 0.9), ha="center",
             weight="semibold")
        text(fig, 0.5, 0.52, "SimCheck shows whether it should be.", 52, C.AMBER, ramp(t, 1.0, 1.7), ha="center",
             weight="semibold")
        text(fig, 0.5, 0.33, "github.com/shreyas-shrestha/simcheck", 32, alpha=ramp(t, 2.2, 2.8), mono=True,
             ha="center")
        text(fig, 0.5, 0.25, "Aadi, Shreyas, Eshan · Georgia Tech", 30, C.GRAY, ramp(t, 2.6, 3.2), ha="center")
    write_clip("demo10_close", 6, draw, [], {})


def main(only=None):
    sel, a, b = featured_rollouts()
    jobs = [("demo01", demo01_title), ("demo02", demo02_world_map), ("demo03", demo03_agent_plan_attempt1),
            ("demo04", lambda: demo04_rollout_attempt1_overconfident(sel, a)),
            ("demo05", demo05_calibration_tally_attempt1), ("demo06", demo06_verifier_feedback),
            ("demo07", lambda: demo07_before_after_same_seed(sel, a, b)), ("demo08", demo08_results_52_to_84),
            ("demo09", demo09_accuracy_only_misses), ("demo10", demo10_close)]
    mf = C.CLIPS / "manifest.json"
    if mf.exists():
        MANIFEST.update(json.loads(mf.read_text()).get("clips", {}))
    for key, fn in jobs:
        if only is None or key.startswith(only):
            fn()
    C.CLIPS.mkdir(parents=True, exist_ok=True)
    mf.write_text(json.dumps({"featured_run": C.rel(C.FEATURED), "model": "argon-medium", "seed_selection": sel,
                              "rollout_runner": str(a["isolation"]), "clips": dict(sorted(MANIFEST.items()))},
                             indent=2, default=float))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
