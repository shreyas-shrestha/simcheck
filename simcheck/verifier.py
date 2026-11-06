"""Agent module + hidden truth -> report.json.

Layer 0 execution (schema, determinism, fresh state), Layer 1 structural (diagnostic only),
Layer 2 accuracy, Layer 3 calibration (primary), Layer 4 route decision with the agent's own
estimator in the loop. Hidden truth stays in this process; the agent only ever receives public
spec and measurements through the runner.
"""
import json
import math
import time
from dataclasses import replace
from pathlib import Path

import gtsam
import numpy as np
from scipy import stats

from simcheck import generator, reference, sim
from simcheck.sandbox import AgentError, InProcessRunner, RemoteEstimator, RunnerError

ROOT = Path(__file__).resolve().parent.parent
CONFIG = json.loads((ROOT / "config.json").read_text())

TASKS = {
    "standard": {},
    "trap_units": {"trap_units": True},
    "trap_frame": {"trap_frame": True},
    "prior_needed": {"prior_needed": True},
}
SEED_SETS = ("dev", "eval", "fresh")
ROUTE_SEED_OFFSET = 100_000  # route trial seeds are disjoint from dataset seeds within a set
NEES_GATE_3 = stats.chi2.ppf(0.95, 3)  # 7.81
DROPOUT_END_DEFAULT = 35


# ---------------------------------------------------------------- statistics

def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def paired_bootstrap(succ, chosen, n_boot=10_000, seed=12345):
    """succ: {route: bool array over the same trial seeds}. Returns diff (B-A) and regret intervals."""
    routes = sorted(succ)
    S = np.stack([np.asarray(succ[r], float) for r in routes])  # (routes, T)
    T = S.shape[1]
    idx = np.random.default_rng(seed).integers(0, T, size=(n_boot, T))
    rates = S[:, idx].mean(axis=2)  # (routes, n_boot)
    ci = lambda a: [float(np.percentile(a, 2.5)), float(np.percentile(a, 97.5))]
    diff = rates[routes.index("B")] - rates[routes.index("A")]
    regret = rates.max(axis=0) - rates[routes.index(chosen)]
    point = S.mean(axis=1)
    return {"diff_B_minus_A": float(point[routes.index("B")] - point[routes.index("A")]), "diff_ci95": ci(diff),
            "regret": float(point.max() - point[routes.index(chosen)]), "regret_ci95": ci(regret)}


def check_matrix(pose, cov):
    pose, P = np.asarray(pose, float), np.asarray(cov, float)
    if pose.shape != (3,):
        return f"pose has shape {pose.shape}, expected (3,)"
    if P.shape != (3, 3):
        return f"covariance has shape {P.shape}, expected (3, 3)"
    if not (np.all(np.isfinite(pose)) and np.all(np.isfinite(P))):
        return "non-finite value in pose or covariance"
    if np.linalg.norm(P - P.T) > 1e-9 * np.linalg.norm(P):
        return f"covariance not symmetric: ||P-P^T|| = {np.linalg.norm(P - P.T):.3g}, ||P|| = {np.linalg.norm(P):.3g}"
    try:
        np.linalg.cholesky(0.5 * (P + P.T))
    except np.linalg.LinAlgError:
        return f"covariance not positive definite (eigenvalues {np.linalg.eigvalsh(0.5 * (P + P.T)).round(6).tolist()})"
    return None


def pose_error(est, true):
    """e = est.localCoordinates(true), the convention the covariance is defined in."""
    return gtsam.Pose2(*est).localCoordinates(gtsam.Pose2(*true))


def nees_nll(est, P, true):
    e = pose_error(est, true)
    P = 0.5 * (np.asarray(P) + np.asarray(P).T)
    nees = float(e @ np.linalg.solve(P, e))
    _, logdet = np.linalg.slogdet(2 * math.pi * P)
    return nees, 0.5 * (nees + logdet)


# ---------------------------------------------------------------- scenarios

def seed_base(seed_set):
    return CONFIG["seeds"]["trial_seed_base"][seed_set]


def task_config(task, seed):
    return replace(generator.ScenarioConfig(seed=seed), **TASKS[task])


def task_spec(task):
    return generator.build_spec(task_config(task, 0))  # the world does not depend on the seed


def load_scenarios(task, seed_set, K):
    """Dataset scenarios on disk (generated once). Returns [(id, spec, measurements, truth_poses)]."""
    out = []
    for i in range(K):
        sid = f"{task}__{seed_set}_{i:03d}"
        d = ROOT / "scenarios" / sid
        if not (d / "hidden" / "truth.json").exists():
            generator.generate(task_config(task, seed_base(seed_set) + i), sid)
        spec = json.loads((d / "public" / "spec.json").read_text())
        meas = json.loads((d / "public" / "measurements.json").read_text())
        truth = np.array(json.loads((d / "hidden" / "truth.json").read_text())["poses"])
        out.append((sid, spec, meas, truth))
    return out


def eval_times(spec, n):
    gs = spec.get("gps_schedule") or {}
    t_drop = (gs.get("dropout_steps") or [None, DROPOUT_END_DEFAULT])[1]
    return {"end_of_dropout": min(t_drop, n - 1), "end_of_trajectory": n - 1}


# ---------------------------------------------------------------- running an estimator over datasets

def run_datasets(runner, scenarios):
    """Feed each dataset to a fresh estimator. Returns per-scenario (est, cov) or raises with a message."""
    res = []
    for sid, spec, meas, _ in scenarios:
        est = RemoteEstimator(runner, spec)
        poses, covs = [], []
        for m in meas:
            pose, cov = est.step(m)
            err = check_matrix(pose, cov)
            if err:
                raise ValueError(f"{sid} step {m['step']}: {err}")
            poses.append(pose)
            covs.append(0.5 * (np.asarray(cov) + np.asarray(cov).T))
        res.append((np.array(poses, float), np.array(covs, float)))
    return res


def reference_datasets(task, seed_set, K):
    cache = ROOT / "runs" / "_reference" / f"datasets_{task}_{seed_set}_K{K}.npz"
    if cache.exists():
        z = np.load(cache)
        return [(z[f"e{i}"], z[f"c{i}"]) for i in range(K)]
    res = run_datasets(InProcessRunner(reference), load_scenarios(task, seed_set, K))
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, **{f"e{i}": e for i, (e, _) in enumerate(res)}, **{f"c{i}": c for i, (_, c) in enumerate(res)})
    return res


def route_trials(runner_or_none, spec, seed_set, T, progress=None):
    """T paired trials per route. runner None -> trusted reference estimator in the loop."""
    base = seed_base(seed_set) + ROUTE_SEED_OFFSET
    succ, coll = {r: [] for r in spec["routes"]}, {r: [] for r in spec["routes"]}
    for i in range(T):
        for r in spec["routes"]:
            est = reference.make_estimator(spec) if runner_or_none is None else RemoteEstimator(runner_or_none, spec)
            o = sim.run_episode(spec, r, base + i, estimator=est)["outcome"]
            succ[r].append(bool(o["success"]))
            coll[r].append(bool(o["collision"]))
        if progress and (i + 1) % 25 == 0:
            progress(f"route trials {i + 1}/{T}")
    return succ, coll


def reference_route_trials(task, seed_set, T):
    cache = ROOT / "runs" / "_reference" / f"routes_{task}_{seed_set}_T{T}.json"
    if cache.exists():
        return json.loads(cache.read_text())
    succ, coll = route_trials(None, task_spec(task), seed_set, T)
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps({"success": succ, "collision": coll}))
    return {"success": succ, "collision": coll}


# ---------------------------------------------------------------- layers

def layer2_accuracy(est_list, ref_list, scenarios, th):
    def ate(e, tr, sl=slice(None)):
        d = np.linalg.norm(e[sl, :2] - tr[sl, :2], axis=1)
        return float(np.sqrt(np.mean(d ** 2))) if d.size else float("nan")
    a, r, ad, rd = [], [], [], []
    for (e, _), (re_, _), (_, spec, _, tr) in zip(est_list, ref_list, scenarios):
        a.append(ate(e, tr)); r.append(ate(re_, tr))
        gs = spec.get("gps_schedule") or {}
        if gs.get("dropout_steps"):
            s = slice(gs["dropout_steps"][0], gs["dropout_steps"][1] + 1)
            ad.append(ate(e, tr, s)); rd.append(ate(re_, tr, s))
    ate_a, ate_r = float(np.mean(a)), float(np.mean(r))
    bound = th["accuracy_abs_tol_m"] + th["accuracy_ref_multiplier"] * ate_r
    return {"pass": bool(ate_a <= bound), "ate_m": ate_a, "reference_ate_m": ate_r, "bound_m": bound,
            "dropout_ate_m": float(np.mean(ad)) if ad else None,
            "reference_dropout_ate_m": float(np.mean(rd)) if rd else None,
            "per_scenario_ate_m": a}


def layer3_calibration(est_list, ref_list, scenarios, th):
    K = len(scenarios)
    nees_lb = stats.chi2.ppf(0.005, 3 * K) / K
    out = {"K": K, "times": {}}
    ok_all = True
    for name in ("end_of_dropout", "end_of_trajectory"):
        nees, nll, rnll, tr_pos, cov_traj = [], [], [], [], []
        for (e, P), (re_, rP), (_, spec, _, truth) in zip(est_list, ref_list, scenarios):
            t = eval_times(spec, len(truth))[name]
            n_, l_ = nees_nll(e[t], P[t], truth[t])
            _, rl = nees_nll(re_[t], rP[t], truth[t])
            nees.append(n_); nll.append(l_); rnll.append(rl)
            tr_pos.append(float(np.trace(P[t][:2, :2])))
            # per-trajectory coverage over all steps: diagnostic only
            cov_traj.append(float(np.mean([nees_nll(e[k], P[k], truth[k])[0] <= NEES_GATE_3 for k in range(len(truth))])))
        c = int(np.sum(np.array(nees) <= NEES_GATE_3))
        p_cov = float(stats.binom.cdf(c, K, th["calibration_coverage_p"]))
        a_ok = bool(p_cov >= th["calibration_alpha"])
        b_ok = bool(float(np.mean(nees)) >= nees_lb)
        c_ok = bool(float(np.mean(nll)) <= float(np.mean(rnll)) + th["nll_margin_nats"])
        out["times"][name] = {
            "pass": a_ok and b_ok and c_ok,
            "covered": c, "coverage_p_value": p_cov, "overconfidence_ok": a_ok,
            "mean_nees": float(np.mean(nees)), "nees_lower_bound": float(nees_lb), "underconfidence_ok": b_ok,
            "mean_nll": float(np.mean(nll)), "reference_mean_nll": float(np.mean(rnll)),
            "nll_bound": float(np.mean(rnll)) + th["nll_margin_nats"], "nll_ok": c_ok,
            "mean_position_cov_trace_m2": float(np.mean(tr_pos)),
            "per_trajectory_coverage_diagnostic": float(np.mean(cov_traj)),
        }
        ok_all &= out["times"][name]["pass"]
    out["pass"] = ok_all
    out["wording"] = (f"no calibration inconsistency detected (K={K})" if ok_all
                      else f"calibration inconsistency detected (K={K})")
    return out


def layer4_route(choice, succ, coll, spec, th, T):
    rates = {r: float(np.mean(succ[r])) for r in succ}
    per = {r: {"success_rate": rates[r], "wilson95": wilson(int(np.sum(succ[r])), T),
               "collision_rate": float(np.mean(coll[r]))} for r in succ}
    bs = paired_bootstrap(succ, choice["route"])
    lo, hi = bs["regret_ci95"]
    delta = th["regret_delta"]
    outcome = "pass" if hi <= delta else ("fail" if lo > delta else "inconclusive")
    stated, actual = choice["success_prob"], rates[choice["route"]]
    stated_ok = abs(stated - actual) <= th["stated_prob_tolerance"]
    stated_ok = bool(stated_ok)
    return {"pass": outcome == "pass" and stated_ok, "regret_outcome": outcome, "T": T,
            "chosen": choice["route"], "stated_success_prob": stated, "reasoning": choice.get("reasoning", ""),
            "routes": per, **bs, "delta": delta,
            "stated_check": {"pass": stated_ok, "stated": stated, "estimated": actual,
                             "estimated_wilson95": per[choice["route"]]["wilson95"],
                             "abs_diff": abs(stated - actual), "tolerance": th["stated_prob_tolerance"]}}


# ---------------------------------------------------------------- feedback and reward

def reward(rep):
    w = CONFIG["reward_weights"]
    L = rep["layers"]
    return round(w["exec"] * L["execution"]["pass"]
                 + w["accuracy"] * bool(L.get("accuracy", {}).get("pass"))
                 + w["calibration"] * bool(L.get("calibration", {}).get("pass"))
                 + w["route"] * bool(L.get("route", {}).get("pass")), 4)


def feedback(rep):
    L, msgs = rep["layers"], []
    ex = L["execution"]
    if not ex["pass"]:
        return [f"Execution failed: {ex['error']}"]
    a = L["accuracy"]
    if not a["pass"]:
        msgs.append(f"Accuracy: mean ATE {a['ate_m']:.3f} m exceeds the bound {a['bound_m']:.3f} m "
                    f"(reference ATE {a['reference_ate_m']:.3f} m). Check units, odometry frame and factor wiring.")
    for name, t in L["calibration"]["times"].items():
        when = name.replace("_", " ")
        if not t["overconfidence_ok"]:
            msgs.append(f"Calibration at {when}: overconfident. Only {t['covered']}/{L['calibration']['K']} runs had "
                        f"NEES <= 7.81 (binomial p = {t['coverage_p_value']:.2g} < 0.01). Covariance too small "
                        f"(mean NEES {t['mean_nees']:.2f}, expected about 3).")
        if not t["underconfidence_ok"]:
            msgs.append(f"Calibration at {when}: underconfident. Mean NEES {t['mean_nees']:.2f} is below "
                        f"{t['nees_lower_bound']:.2f}. Covariance is inflated (mean position trace "
                        f"{t['mean_position_cov_trace_m2']:.3g} m^2).")
        if not t["nll_ok"]:
            msgs.append(f"Calibration at {when}: mean Gaussian NLL {t['mean_nll']:.2f} exceeds reference "
                        f"{t['reference_mean_nll']:.2f} + margin = {t['nll_bound']:.2f}.")
    r = L["route"]
    if r["regret_outcome"] != "pass":
        rr = {k: v["success_rate"] for k, v in r["routes"].items()}
        msgs.append(f"Route: chose {r['chosen']}, success rates with your estimator in the loop {rr} over T={r['T']} "
                    f"paired trials; regret {r['regret']:.3f}, 95% CI {[round(x, 3) for x in r['regret_ci95']]}, "
                    f"tolerance {r['delta']} -> {r['regret_outcome']}.")
    s = r["stated_check"]
    if not s["pass"]:
        msgs.append(f"Route: stated success probability {s['stated']:.2f} differs from the measured rate "
                    f"{s['estimated']:.3f} (95% CI {[round(x, 3) for x in s['estimated_wilson95']]}) by more than "
                    f"{s['tolerance']}.")
    if L.get("structural", {}).get("notes"):
        msgs += [f"Structure (diagnostic): {n}" for n in L["structural"]["notes"]]
    return msgs


# ---------------------------------------------------------------- top level

def verify(runner, task="standard", seed_set="eval", decision_only=False, progress=print, out_path=None):
    th = CONFIG["thresholds"]
    K, T = th["calibration_K"], CONFIG["route_trials"]["T"]
    spec = task_spec(task)
    t0 = time.perf_counter()
    rep = {"task": task, "seed_set": seed_set, "isolation": getattr(runner, "isolation", "?"),
           "thresholds_frozen": th.get("frozen", False), "layers": {}}
    L = rep["layers"]
    scen = load_scenarios(task, seed_set, K)

    # Layer 0: execution, schema, determinism, fresh state
    try:
        progress(f"[{task}/{seed_set}] layer 0+: running {K} datasets")
        est = run_datasets(runner, scen)
        again = run_datasets(runner, scen[:1])[0]  # after K other episodes: catches leaked state too
        dp = float(np.max(np.abs(again[0] - est[0][0])))
        dc = float(np.max(np.abs(again[1] - est[0][1]) / (np.abs(est[0][1]) + 1e-300)))
        if not np.allclose(again[0], est[0][0], atol=1e-6, rtol=0) or not np.allclose(again[1], est[0][1], rtol=1e-6, atol=0):
            raise ValueError(f"non-deterministic or state leaks across episodes: max pose diff {dp:.3g}, "
                             f"max relative covariance diff {dc:.3g}")
        choice = runner.call({"cmd": "choose_route", "spec": spec}, timeout=900)["result"]
        if choice["route"] not in spec["routes"] or not (0.0 <= choice["success_prob"] <= 1.0):
            raise ValueError(f"choose_route returned invalid {choice}")
        L["execution"] = {"pass": True, "determinism_max_pose_diff": dp, "determinism_max_rel_cov_diff": dc}
    except (AgentError, RunnerError, ValueError, KeyError, TypeError) as e:
        L["execution"] = {"pass": False, "error": f"{type(e).__name__}: {str(e)[:1500]}"}
        rep["accepted"] = False
        rep["reward"] = reward(rep)
        rep["feedback"] = feedback(rep)
        rep["seconds"] = time.perf_counter() - t0
        _write(rep, out_path)
        return rep

    # Layer 1: structural (diagnostic only)
    try:
        s = runner.call({"cmd": "build_graph", "measurements": scen[0][2], "spec": spec}, timeout=120)["summary"]
        notes = []
        if s:
            if s["connected_components"] != 1:
                notes.append(f"graph has {s['connected_components']} connected components")
            if s["unary_factors"] == 0:
                notes.append("no unary (prior/GPS/landmark) factors: graph is not anchored")
        L["structural"] = {"provided": s is not None, "summary": s, "notes": notes}
    except (AgentError, RunnerError) as e:
        L["structural"] = {"provided": True, "error": str(e)[:500], "notes": [f"build_graph raised: {str(e)[:200]}"]}

    ref = reference_datasets(task, seed_set, K)
    progress(f"[{task}/{seed_set}] layers 2-3")
    L["accuracy"] = layer2_accuracy(est, ref, scen, th)
    L["calibration"] = layer3_calibration(est, ref, scen, th)

    # Layer 4: route decision, decided once at T
    progress(f"[{task}/{seed_set}] layer 4: {T} paired trials per route"
             + (" (decision-only grading: reference estimator in the loop)" if decision_only else ""))
    try:
        succ, coll = route_trials(None if decision_only else runner, spec, seed_set, T, progress)
        L["route"] = layer4_route(choice, succ, coll, spec, th, T)
        L["route"]["mode"] = "decision-only grading (reference estimator in the loop)" if decision_only \
            else "submitted estimator in the loop"
        rr = reference_route_trials(task, seed_set, T)
        L["route"]["reference_estimator_rates"] = {r: float(np.mean(v)) for r, v in rr["success"].items()}
    except (AgentError, RunnerError) as e:
        L["route"] = {"pass": False, "regret_outcome": "fail", "error": str(e)[:1000], "routes": {}, "chosen": choice["route"],
                      "T": T, "regret": float("nan"), "regret_ci95": [float("nan")] * 2, "delta": th["regret_delta"],
                      "stated_check": {"pass": False, "stated": choice["success_prob"], "estimated": float("nan"),
                                       "estimated_wilson95": [float("nan")] * 2, "abs_diff": float("nan"),
                                       "tolerance": th["stated_prob_tolerance"]}}

    rep["accepted"] = all(L[k]["pass"] for k in ("execution", "accuracy", "calibration", "route"))
    rep["reward"] = reward(rep)
    rep["feedback"] = feedback(rep)
    rep["seconds"] = time.perf_counter() - t0
    _write(rep, out_path)
    return rep


def _write(rep, out_path):
    if out_path:
        Path(out_path).parent.mkdir(parents=True, exist_ok=True)
        Path(out_path).write_text(json.dumps(rep, indent=2, default=float))
