"""ScenarioConfig + seed -> scenarios/<id>/{public/{task.md,spec.json,measurements.json}, hidden/truth.json}.

World: start at the origin, goal at (corridor_length, 0).
  Route A: straight through a narrow featureless gap (short).
  Route B: detour over the gap past a row of landmarks (longer).
Landmarks sit far enough from route A that none is ever within max_range of it.
"""
import argparse
import json
import os
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
    unknown_noise: bool = False      # v3.2: true odometry std = nominal x scale, scale ~ U(noise_scale_range)
    noise_scale_range: tuple = (0.5, 3.0)
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
        **({"odom_noise_scale_range": list(cfg.noise_scale_range)} if cfg.unknown_noise else {}),
    }


def odom_noise_scale(spec, seed):
    """unknown_noise: the hidden per-scenario (or per-trial) odometry noise scale; 1.0 otherwise."""
    lo_hi = spec.get("odom_noise_scale_range")
    if not lo_hi:
        return 1.0
    return float(np.random.default_rng([int(seed), 0x5CA1E]).uniform(*lo_hi))


def true_spec(spec, seed):
    """The spec the simulator runs: public spec with the hidden odometry noise scale applied."""
    k = odom_noise_scale(spec, seed)
    if k == 1.0:
        return spec
    out = dict(spec, sensor_noise_std=dict(spec["sensor_noise_std"]))
    out["sensor_noise_std"]["odom"] = [k * x for x in spec["sensor_noise_std"]["odom"]]
    return out


# The original v3.1 conventions block, kept byte-for-byte so repeat runs get the exact v3.1 prompt.
_CONVENTIONS_V31 = """Conventions:
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION in SI
  units (metres, radians), regardless of `input_units`.
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{k-1} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (GTSAM convention).
"""

# v3.2: states the x_0 prior, SI noise units and the covariance frame explicitly, plus a grading summary.
_CONVENTIONS_V32 = """Conventions:
- The true initial pose x_0 is sampled from the stated prior N(initial_pose.mean, diag(cov_diag_variances));
  it is not known exactly.
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION and is
  always in SI units (metres, radians), even when measurements arrive in mm or degrees (`input_units`).
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{k-1} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (the GTSAM Pose2 marginal
  convention, ordered x, y, theta), not in the world frame.

Grading: trajectory accuracy, consistency of the reported uncertainty across independent runs, and
collision-free arrival rate of the chosen route in closed-loop simulation driven by your estimator.
"""

TASK_MD_VERSIONS = {"v3.1": _CONVENTIONS_V31, "v3.2": _CONVENTIONS_V32}


def config_path() -> Path:
    """$SIMCHECK_CONFIG, else config_v3.2.json on the v3.2 branch, else config.json."""
    env = os.environ.get("SIMCHECK_CONFIG")
    if env:
        return Path(env) if Path(env).is_absolute() else ROOT / env
    v32 = ROOT / "config_v3.2.json"
    return v32 if v32.exists() else ROOT / "config.json"


def task_md_version() -> str:
    """The config's "task_md_version"; "v3.1" (the original text) when unset."""
    cfg = config_path()
    return (json.loads(cfg.read_text()).get("task_md_version") if cfg.exists() else None) or "v3.1"


def task_md(spec: dict, version: str | None = None) -> str:
    version = version or task_md_version()
    if version not in TASK_MD_VERSIONS:
        raise ValueError(f"unknown task_md_version {version!r}; expected one of {sorted(TASK_MD_VERSIONS)}")
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
    if spec.get("odom_noise_scale_range"):
        lo, hi = spec["odom_noise_scale_range"]
        notes.append(f"- ODOMETRY NOISE: odometry noise std is between {lo:g}x and {hi:g}x the nominal value in "
                     "`sensor_noise_std.odom`; the exact value is not given (`odom_noise_scale_range`). Other noise "
                     "values are exact.")
    trap = "\n".join(notes) or "- None."
    return f"""# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

{TASK_MD_VERSIONS[version]}
Scenario-specific notes:
{trap}
"""


def generate(cfg: ScenarioConfig, scenario_id: str, out_root: Path = ROOT / "scenarios") -> Path:
    spec = build_spec(cfg)
    ep = sim.run_episode(true_spec(spec, cfg.seed), cfg.dataset_route, cfg.seed, oracle=True,
                         max_steps=cfg.n_poses - 1)
    d = out_root / scenario_id
    (d / "public").mkdir(parents=True, exist_ok=True)
    (d / "hidden").mkdir(parents=True, exist_ok=True)
    (d / "public" / "spec.json").write_text(json.dumps(spec, indent=2))
    (d / "public" / "measurements.json").write_text(json.dumps(ep["measurements"]))
    (d / "public" / "task.md").write_text(task_md(spec))
    cfg_d = asdict(cfg)
    (d / "hidden" / "truth.json").write_text(json.dumps(
        {"config": cfg_d, "poses": ep["truth"].tolist(), "outcome": ep["outcome"],
         "odom_noise_scale": odom_noise_scale(spec, cfg.seed)}))
    return d


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--id", default=None)
    a = ap.parse_args()
    print(generate(ScenarioConfig(seed=a.seed), a.id or f"dev_{a.seed:04d}"))
