"""ScenarioConfig + seed -> scenarios/<id>/{public/{task.md,spec.json,measurements.json}, hidden/truth.json}.

World: start at the origin, goal at (corridor_length, 0).
  Route A: straight through a narrow featureless gap (short).
  Route B: detour over the gap past a row of landmarks (longer).
Landmarks sit far enough from route A that none is ever within max_range of it.
"""
import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from simcheck import sim

ROOT = Path(__file__).resolve().parent.parent


@dataclass
class ScenarioConfig:
    seed: int = 0
    n_poses: int = 120               # cap on dataset length; dataset stops on arrival
    odom_noise: tuple = (0.02, 0.02, 0.01)
    actuator_noise: tuple = (0.05, 0.05)
    n_landmarks: int = 6
    landmark_range: float = 5.0
    range_noise: float = 0.10
    bearing_noise: float = 0.03
    gps: bool = True
    gps_noise: float = 0.15
    gps_dropout: tuple | None = (20, 35)
    trap_units: bool = False
    trap_frame: bool = False
    corridor_length: float = 12.0
    prior_needed: bool = False
    gap_half_width: float = 0.45     # wall offset from the route-A centreline
    detour_height: float = 3.0
    dataset_route: str = "B"         # B has turns, so frame/unit bugs are visible
    extra: dict = field(default_factory=dict)


def build_spec(cfg: ScenarioConfig) -> dict:
    L, h, g = cfg.corridor_length, cfg.detour_height, cfg.gap_half_width
    gap_x = (0.45 * L, 0.75 * L)
    walls = [[gap_x[0], g, gap_x[1], g], [gap_x[0], -g, gap_x[1], -g]]
    route_a = [[0.0, 0.0], [L, 0.0]]
    route_b = [[0.0, 0.0], [0.2 * L, h], [0.8 * L, h], [L, 0.0]]

    # landmarks on a row above the detour: visible from B, never from A
    ly = h + 0.5 * cfg.landmark_range
    assert ly > cfg.landmark_range, "landmarks must be invisible from route A"
    # prior_needed: no GPS and no landmarks, so the initial prior is the only absolute anchor
    n_lm = 0 if cfg.prior_needed else cfg.n_landmarks
    lx = np.linspace(0.1 * L, 0.9 * L, n_lm) if n_lm else []
    landmarks = {str(i): [round(float(x), 3), ly] for i, x in enumerate(lx)}

    gps_schedule = None
    if cfg.gps and not cfg.prior_needed:
        gps_schedule = {"rate_steps": 1, "dropout_steps": list(cfg.gps_dropout) if cfg.gps_dropout else None}

    return {
        "units_out": {"position": "m", "angle": "rad"},
        "dt": 0.5,
        "pose_indexing": "N poses x_0..x_{N-1}, N-1 odometry transitions",
        "initial_pose": {"mean": [0.0, 0.0, 0.0], "cov_diag_variances": [0.01, 0.01, 0.001]},
        "initial_truth_rule": "true x_0 is sampled from N(mean, diag(cov_diag_variances))",
        "landmarks": landmarks,
        "landmark_visibility": {"max_range": cfg.landmark_range, "rule": "observed every step when within max_range, no occlusion"},
        "sensor_noise_std": {"odom": list(cfg.odom_noise), "range": cfg.range_noise,
                             "bearing": cfg.bearing_noise, "gps": cfg.gps_noise},
        "actuator_noise_std": {"v": cfg.actuator_noise[0], "omega": cfg.actuator_noise[1]},
        "gps_schedule": gps_schedule,
        "input_units": {"odom": "mm" if cfg.trap_units else "m", "bearing": "deg" if cfg.trap_units else "rad"},
        "odom_frame": "odom" if cfg.trap_frame else "body",
        "controller": {"type": "pure_pursuit", "lookahead": 0.8, "v": 0.5, "omega_max": 1.0},
        "max_steps": 120,
        "arrival_tolerance": 0.3,
        "routes": {"A": route_a, "B": route_b},
        "robot_radius": 0.2,
        "walls": walls,
        "success_definition": "reach within arrival_tolerance of route end within max_steps with zero collisions",
        "objective": "maximize collision-free arrival probability; distance breaks close ties",
    }


def task_md(spec: dict) -> str:
    notes = []
    if spec["input_units"]["odom"] != "m" or spec["input_units"]["bearing"] != "rad":
        notes.append(f"- INPUT UNITS: odometry dx/dy arrive in {spec['input_units']['odom']}, "
                     f"bearings in {spec['input_units']['bearing']}. Outputs must still be metres and radians.")
    if spec["odom_frame"] == "odom":
        notes.append("- ODOMETRY FRAME: dx/dy are expressed in the odometry frame, i.e. the body frame at step 0 "
                     "rotated by the running sum of the reported dtheta values up to (not including) this "
                     "transition. dtheta is unaffected.")
    if spec["gps_schedule"] is None:
        notes.append("- There is no GPS in this scenario.")
    if not spec["landmarks"]:
        notes.append("- There are no landmarks in this scenario.")
    trap = "\n".join(notes) or "- None."
    return f"""# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

Conventions:
- The true initial pose x_0 is sampled from the stated prior N(initial_pose.mean, diag(cov_diag_variances));
  it is not known exactly.
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION and is
  always in SI units (metres, radians), even when measurements arrive in mm or degrees (`input_units`).
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{{k-1}} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (the GTSAM Pose2 marginal
  convention, ordered x, y, theta), not in the world frame.

Grading: trajectory accuracy, consistency of the reported uncertainty across independent runs, and
collision-free arrival rate of the chosen route in closed-loop simulation driven by your estimator.

Scenario-specific notes:
{trap}
"""


def generate(cfg: ScenarioConfig, scenario_id: str, out_root: Path = ROOT / "scenarios") -> Path:
    spec = build_spec(cfg)
    ep = sim.run_episode(spec, cfg.dataset_route, cfg.seed, oracle=True, max_steps=cfg.n_poses - 1)
    d = out_root / scenario_id
    (d / "public").mkdir(parents=True, exist_ok=True)
    (d / "hidden").mkdir(parents=True, exist_ok=True)
    (d / "public" / "spec.json").write_text(json.dumps(spec, indent=2))
    (d / "public" / "measurements.json").write_text(json.dumps(ep["measurements"]))
    (d / "public" / "task.md").write_text(task_md(spec))
    cfg_d = asdict(cfg)
    (d / "hidden" / "truth.json").write_text(json.dumps(
        {"config": cfg_d, "poses": ep["truth"].tolist(), "outcome": ep["outcome"]}))
    return d


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--id", default=None)
    a = ap.parse_args()
    print(generate(ScenarioConfig(seed=a.seed), a.id or f"dev_{a.seed:04d}"))
