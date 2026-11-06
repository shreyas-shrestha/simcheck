"""Runs INSIDE the sandbox. Loads the agent's solution.py and serves JSON lines on stdin/stdout.

Self-contained on purpose (stdlib + numpy + gtsam only): nothing from simcheck, no hidden truth.
Requests:  {"cmd": "make", "spec": {...}}            -> {"ok": true}   (replaces the estimator)
           {"cmd": "step", "m": {...}}               -> {"ok": true, "pose": [...], "cov": [[...]]}
           {"cmd": "choose_route", "spec": {...}}    -> {"ok": true, "result": {...}}
           {"cmd": "build_graph", "measurements": [...], "spec": {...}} -> {"ok": true, "summary": {...} | null}
Errors come back as {"ok": false, "error": "..."}; the worker keeps serving.
"""
import importlib.util
import json
import sys
import traceback

PROTO = sys.stdout
sys.stdout = sys.stderr  # agent prints must never corrupt the protocol stream


def _jsonable(x):
    try:
        import numpy as np
        return np.asarray(x, dtype=float).tolist()
    except Exception:
        return x


def _graph_summary(graph, values):
    import gtsam
    types, keys, edges = {}, set(), []
    for i in range(graph.size()):
        f = graph.at(i)
        if f is None:
            continue
        name = type(f).__name__
        types[name] = types.get(name, 0) + 1
        ks = list(f.keys())
        keys.update(ks)
        if len(ks) >= 2:
            edges.append(ks)
    parent = {k: k for k in keys}

    def find(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]]
            k = parent[k]
        return k
    for ks in edges:
        for k in ks[1:]:
            parent[find(k)] = find(ks[0])
    unary = sum(1 for i in range(graph.size()) if graph.at(i) is not None and len(graph.at(i).keys()) == 1)
    first = gtsam.Symbol(min(keys)).string() if keys else None
    return {"n_factors": int(graph.size()), "factor_types": types, "n_variables": len(keys),
            "n_values": int(values.size()), "connected_components": len({find(k) for k in keys}),
            "unary_factors": unary, "first_key": first}


def main(solution_path):
    spec_ = importlib.util.spec_from_file_location("solution", solution_path)
    sol = importlib.util.module_from_spec(spec_)
    spec_.loader.exec_module(sol)
    est = None
    for line in sys.stdin:
        try:
            req = json.loads(line)
            cmd = req["cmd"]
            if cmd == "make":
                est = sol.make_estimator(req["spec"])
                resp = {"ok": True}
            elif cmd == "step":
                pose, cov = est.step(req["m"])
                resp = {"ok": True, "pose": _jsonable(pose), "cov": _jsonable(cov)}
            elif cmd == "choose_route":
                r = sol.choose_route(req["spec"])
                resp = {"ok": True, "result": {"route": r["route"], "success_prob": float(r["success_prob"]),
                                               "reasoning": str(r.get("reasoning", ""))}}
            elif cmd == "build_graph":
                if not hasattr(sol, "build_graph"):
                    resp = {"ok": True, "summary": None}
                else:
                    g, v = sol.build_graph(req["measurements"], req["spec"])
                    resp = {"ok": True, "summary": _graph_summary(g, v)}
            else:
                resp = {"ok": False, "error": f"unknown cmd {cmd}"}
        except Exception as e:  # report and keep serving
            resp = {"ok": False, "error": f"{type(e).__name__}: {e}", "trace": traceback.format_exc()[-2000:]}
        PROTO.write(json.dumps(resp) + "\n")
        PROTO.flush()


if __name__ == "__main__":
    main(sys.argv[1])
