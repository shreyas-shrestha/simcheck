"""Slide stills -> demo/deck_assets/ (PNG; charts also as *_transparent.png) and demo/deck_assets/deck_numbers.json.

    .venv/bin/python scripts/media/deck.py
"""
import json
import sys
import textwrap
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from media import common as C  # noqa: E402

C.setup_matplotlib()
import matplotlib.pyplot as plt  # noqa: E402
from media import data as Dt, draw as Dr  # noqa: E402

N = Dt.Numbers()


def save(fig, name, transparent_copy=False):
    C.DECK.mkdir(parents=True, exist_ok=True)
    fig.savefig(C.DECK / f"{name}.png", dpi=C.DPI, facecolor=C.BG)
    if transparent_copy:
        bg = []
        for ax in fig.axes:
            bg.append(ax.get_facecolor())
            ax.set_facecolor("none")
        fig.savefig(C.DECK / f"{name}_transparent.png", dpi=C.DPI, transparent=True)
        for ax, c in zip(fig.axes, bg):
            ax.set_facecolor(c)
    plt.close(fig)


def miss_step(ro):
    """Attempt-1 step where the truth is furthest outside the 95% ellipse (mid-route only)."""
    n = len(ro["inside"])
    best, kbest = -1.0, None
    for k in range(n):
        if ro["inside"][k] or not (0.25 * n <= k <= 0.75 * n):
            continue
        Cw = C.world_pos_cov(ro["est"][k], ro["cov"][k])
        d = ro["truth"][k, :2] - ro["est"][k, :2]
        m = float(d @ np.linalg.solve(Cw, d))
        if m > best:
            best, kbest = m, k
    return kbest, best


def featured_rollouts():
    sel = C.featured_seed()
    a = C.rollout("attempt1", sel["route"], sel["trial_index"])
    b = C.rollout("final", sel["route"], sel["trial_index"])
    for name, ro in (("attempt1", a), ("final", b)):
        rec = C.recorded_outcome(name, sel["route"], sel["trial_index"])
        assert ro["outcome"]["success"] == rec["success"] and ro["outcome"]["collision"] == rec["collision"]
    return sel, a, b


# ---------------------------------------------------------------- stills

def s01_cover_hero(sel, a):
    k, _ = miss_step(a)
    spec = C.task_spec()
    fig = Dr.new_fig(880, 1080)
    p = a["truth"][k]
    hw, hh = 1.5, 1.5 * 1080 / 880
    ax = Dr.map_axes(fig, (0, 0, 1, 1), xlim=(p[0] - hw, p[0] + hw), ylim=(p[1] - hh, p[1] + hh))
    Dr.draw_hallway(ax, spec, label=False)
    Dr.draw_walls(ax, spec)
    Dr.draw_rollout(ax, spec, a, k)
    save(fig, "s01_cover_hero")
    N.add("s01_cover_hero.png", "attempt-1 rollout step shown (ellipse misses truth)", int(k),
          "build/media_cache rollout replay of " + C.rel(C.FEATURED / "1" / "solution.py"))
    return k


def s04_task_map():
    spec = C.task_spec()
    fig = Dr.new_fig()
    ax = Dr.map_axes(fig)
    Dr.draw_world(ax, spec)
    Dr.draw_start(ax, spec)
    save(fig, "s04_task_map")


def bars(ax, rows, key_first, key_after, labels, annotate=True):
    x = np.arange(len(rows))
    w = 0.36
    for off, key, color, lab in ((-w / 2, key_first, C.GRAY, labels[0]), (w / 2, key_after, C.AMBER, labels[1])):
        k = np.array([r[key] for r in rows], float)
        n = np.array([r["n"] for r in rows], float)
        rate = k / n
        lo = np.array([r[key + "_wilson95"][0] for r in rows])
        hi = np.array([r[key + "_wilson95"][1] for r in rows])
        ax.bar(x + off, rate, w * 0.92, color=color, label=lab, zorder=3)
        ax.errorbar(x + off, rate, yerr=[rate - lo, hi - rate], fmt="none", ecolor=C.FG, elinewidth=2,
                    capsize=6, capthick=2, zorder=4)
        if annotate:
            for xi, ki, ni, h in zip(x + off, k, n, hi):
                ax.text(xi, h + 0.03, f"{int(ki)}/{int(ni)}", ha="center", va="bottom", family=C.MONO,
                        fontsize=C.px(24), color=C.FG)
    ax.set_xticks(x)
    ax.set_xticklabels([r["task"] for r in rows], family=C.MONO, fontsize=C.px(24), color=C.FG)
    ax.set_ylim(0, 1.22)
    ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_yticklabels(["0%", "25%", "50%", "75%", "100%"])
    ax.grid(axis="y", color=C.GRID, lw=1.5, zorder=0)
    ax.tick_params(axis="x", length=0, pad=14)


def s06_results(rows, tot, tot_src):
    fig = Dr.new_fig()
    ax = fig.add_axes((0.07, 0.13, 0.9, 0.66))
    bars(ax, rows, "first_try", "after", ("first try", "after feedback, fresh seeds"))
    ax.set_ylabel("pass rate (Wilson 95%)", labelpad=14)
    fig.text(0.07, 0.93, "Pass rate per task, Gemini 4 Argon", fontsize=C.px(44), weight="semibold", color=C.FG)
    fig.text(0.07, 0.875, "first try vs after SimCheck feedback (re-graded on fresh seeds)", fontsize=C.px(28),
             color=C.GRAY)
    fig.text(0.97, 0.93, f"{tot['first_try']}/{tot['n']} → {tot['after_loop']}/{tot['after_loop_n']}",
             ha="right", fontsize=C.px(56), weight="semibold", color=C.AMBER)
    fig.text(0.97, 0.875, "all runs", ha="right", family=C.MONO, fontsize=C.px(24), color=C.GRAY)
    leg = ax.legend(loc="upper left", ncol=2, bbox_to_anchor=(0.0, 1.08), handlelength=1.2, columnspacing=2)
    for t in leg.get_texts():
        t.set_color(C.FG)
    save(fig, "s06_results_first_try_vs_feedback", transparent_copy=True)
    for r in rows:
        N.add("s06_results_first_try_vs_feedback.png", f"{r['task']} first try", f"{r['first_try']}/{r['n']}",
              r["first_try_source"])
        N.add("s06_results_first_try_vs_feedback.png", f"{r['task']} first try Wilson 95%", r["first_try_wilson95"],
              r["first_try_source"])
        N.add("s06_results_first_try_vs_feedback.png", f"{r['task']} after feedback (fresh seeds)",
              f"{r['after']}/{r['n']}", r["after_source"])
        N.add("s06_results_first_try_vs_feedback.png", f"{r['task']} after feedback Wilson 95%",
              r["after_wilson95"], "computed from the counts above with scripts/summarize.py wilson()")
    N.add("s06_results_first_try_vs_feedback.png", "headline",
          f"{tot['first_try']}/{tot['n']} -> {tot['after_loop']}/{tot['after_loop_n']}", tot_src)


S07_XLIM, S07_YLIM = (3.6, 8.6), (-2.0, 2.0)


def s07_before_after(sel, a, b):
    k, m = miss_step(a)
    spec = C.task_spec()
    fig = Dr.new_fig()
    for i, (ro, title) in enumerate(((a, "Attempt 1"), (b, "After feedback"))):
        ax = Dr.map_axes(fig, (0.01 + i * 0.5, 0.1, 0.48, 0.7), xlim=S07_XLIM, ylim=S07_YLIM)
        Dr.draw_hallway(ax, spec, label=False)
        Dr.draw_walls(ax, spec)
        kk = min(k, len(ro["truth"]) - 1)
        Dr.draw_rollout(ax, spec, ro, kk)
        inside = bool(ro["inside"][kk])
        fig.text(0.03 + i * 0.5, 0.9, title, fontsize=C.px(44), weight="semibold", color=C.FG)
        fig.text(0.03 + i * 0.5, 0.84, "truth inside 95% ellipse" if inside else "truth outside 95% ellipse",
                 family=C.MONO, fontsize=C.px(26), color=C.AMBER if inside else C.RED)
    fig.text(0.5, 0.05, f"same eval seed {sel['seed']}, route A, step {k}", ha="center", family=C.MONO,
             fontsize=C.px(24), color=C.GRAY)
    fig.add_artist(plt.Line2D([0.5, 0.5], [0.12, 0.92], color=C.GRID, lw=2))
    save(fig, "s07_before_after_unknown_noise")
    N.add("s07_before_after_unknown_noise.png", "step shown", int(k), "rollout replay (max attempt-1 Mahalanobis miss)")
    N.add("s07_before_after_unknown_noise.png", "seed", sel["seed"], "build/media_cache/selection.json")
    N.add("s07_before_after_unknown_noise.png", "attempt-1 squared Mahalanobis distance at that step", round(m, 3),
          "rollout replay")
    return k


def feedback_card(name, header, lines, src):
    fig = Dr.new_fig()
    from matplotlib.patches import FancyBboxPatch
    ax = fig.add_axes((0, 0, 1, 1))
    ax.axis("off")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.add_patch(FancyBboxPatch((0.08, 0.2), 0.84, 0.56, boxstyle="round,pad=0,rounding_size=0.012",
                                facecolor="#1B1D21", edgecolor=C.GRID, lw=2))
    fig.text(0.1, 0.84, header, fontsize=C.px(40), weight="semibold", color=C.FG)
    body = "\n\n".join(textwrap.fill(l, 62) for l in lines)
    fig.text(0.11, 0.71, body, family=C.MONO, fontsize=C.px(34), color=C.FG, va="top", linespacing=1.45)
    fig.text(0.1, 0.13, src, family=C.MONO, fontsize=C.px(24), color=C.GRAY)
    save(fig, name)
    N.add(f"{name}.png", "feedback text (verbatim)", lines, src)


def s09_coverage(runs):
    fig = Dr.new_fig()
    fig.text(0.07, 0.92, "unknown_noise: truth inside the 95% region", fontsize=C.px(44), weight="semibold",
             color=C.FG)
    fig.text(0.07, 0.865, "coverage per attempt, 30 eval seeds, two runs", fontsize=C.px(28), color=C.GRAY)
    times = [("end_of_dropout", "end of GPS dropout"), ("end_of_trajectory", "end of trajectory")]
    styles = {"r1": (C.GRAY, "o", "run r1"), "r2": (C.AMBER, "o", "run r2 (featured)")}
    for j, (t, tlabel) in enumerate(times):
        ax = fig.add_axes((0.07 + j * 0.47, 0.14, 0.4, 0.62))
        ax.axhline(0.95, color=C.FG, lw=2, ls=(0, (6, 4)), zorder=2)
        ax.text(3.28, 0.962, "0.95 target", va="bottom", ha="right", family=C.MONO, fontsize=C.px(24), color=C.FG)
        for run in runs:
            color, mk, lab = styles[run["run"]]
            xs, ys = [], []
            for at in run["attempts"]:
                if t in at:
                    xs.append(at["attempt"]); ys.append(at[t]["coverage95"])
                    ax.annotate(f"{at[t]['covered']}/{at[t]['K']}", (at["attempt"], at[t]["coverage95"]),
                                xytext=(0, 22), textcoords="offset points", ha="center", va="bottom",
                                family=C.MONO, fontsize=C.px(24), color=color)
                    N.add("s09_unknown_noise_coverage.png", f"{run['run']} attempt {at['attempt']} {t} coverage95",
                          f"{at[t]['covered']}/{at[t]['K']}", at["source"])
                else:
                    ax.text(at["attempt"], 0.06 + (0.08 if run["run"] == "r2" else 0), f"{run['run']}: did not run",
                            ha="center", family=C.MONO, fontsize=C.px(24), color=color)
                    N.add("s09_unknown_noise_coverage.png", f"{run['run']} attempt {at['attempt']}",
                          "did not run (execution failed)", at["source"])
            ax.plot(xs, ys, color=color, lw=3, marker=mk, ms=14, label=lab, zorder=3)
        ax.set_xlim(0.6, 3.3)
        ax.set_ylim(0, 1.13)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels(["attempt 1", "attempt 2", "attempt 3"])
        ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(["0%", "25%", "50%", "75%", "100%"])
        ax.grid(axis="y", color=C.GRID, lw=1.5, zorder=0)
        ax.set_title(tlabel, loc="left", color=C.FG, pad=16, fontsize=C.px(30))
        if j == 0:
            leg = ax.legend(loc="lower left", bbox_to_anchor=(0.0, 0.16))
            for tx in leg.get_texts():
                tx.set_color(C.FG)
    save(fig, "s09_unknown_noise_coverage", transparent_copy=True)


def main():
    sel, a, b = featured_rollouts()
    rows = Dt.per_task()
    tot, tot_src = Dt.headline_totals()
    s01_cover_hero(sel, a)
    s04_task_map()
    s06_results(rows, tot, tot_src)
    s07_before_after(sel, a, b)
    fb_before, src_before = Dt.v31_feedback_with_rate()
    fb_after, src_after = Dt.featured_feedback()
    feedback_card("s08_feedback_before", "v3.1 feedback: states the measured rate", fb_before, src_before)
    feedback_card("s08_feedback_after", "v3.2 feedback: direction only", fb_after, src_after)
    s09_coverage(Dt.unknown_noise_coverage())
    out = {"featured_run": C.rel(C.FEATURED), "attempts": C.ATTEMPTS, "seed_selection": sel,
           "numbers": N.rows}
    (C.DECK / "deck_numbers.json").write_text(json.dumps(out, indent=2, default=float))
    print("wrote", sorted(p.name for p in C.DECK.iterdir()))


if __name__ == "__main__":
    main()
