# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

Conventions:
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION in SI
  units (metres, radians), regardless of `input_units`.
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{k-1} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (GTSAM convention).

Scenario-specific notes:
- There is no GPS in this scenario.
- There are no landmarks in this scenario.


## spec.json
```json
{
  "units_out": {
    "position": "m",
    "angle": "rad"
  },
  "dt": 0.5,
  "pose_indexing": "N poses x_0..x_{N-1}, N-1 odometry transitions",
  "initial_pose": {
    "mean": [
      0.0,
      0.0,
      0.0
    ],
    "cov_diag_variances": [
      0.01,
      0.01,
      0.001
    ]
  },
  "initial_truth_rule": "true x_0 is sampled from N(mean, diag(cov_diag_variances))",
  "landmarks": {},
  "landmark_visibility": {
    "max_range": 5.0,
    "rule": "observed every step when within max_range, no occlusion"
  },
  "sensor_noise_std": {
    "odom": [
      0.02,
      0.02,
      0.01
    ],
    "range": 0.1,
    "bearing": 0.03,
    "gps": 0.15
  },
  "actuator_noise_std": {
    "v": 0.05,
    "omega": 0.05
  },
  "gps_schedule": null,
  "input_units": {
    "odom": "m",
    "bearing": "rad"
  },
  "odom_frame": "body",
  "controller": {
    "type": "pure_pursuit",
    "lookahead": 0.8,
    "v": 0.5,
    "omega_max": 1.0
  },
  "max_steps": 120,
  "arrival_tolerance": 0.3,
  "routes": {
    "A": [
      [
        0.0,
        0.0
      ],
      [
        12.0,
        0.0
      ]
    ],
    "B": [
      [
        0.0,
        0.0
      ],
      [
        2.4000000000000004,
        3.0
      ],
      [
        9.600000000000001,
        3.0
      ],
      [
        12.0,
        0.0
      ]
    ]
  },
  "robot_radius": 0.2,
  "walls": [
    [
      5.4,
      0.45,
      9.0,
      0.45
    ],
    [
      5.4,
      -0.45,
      9.0,
      -0.45
    ]
  ],
  "success_definition": "reach within arrival_tolerance of route end within max_steps with zero collisions",
  "objective": "maximize collision-free arrival probability; distance breaks close ties"
}
```

## measurements.json (one example episode; your estimator is graded on many others)
```json
[{"step": 0, "t": 0.0, "odom": null, "gps": null, "landmarks": []}, {"step": 1, "t": 0.5, "odom": {"dx": 0.21640307377794948, "dy": 0.059559631003158, "dtheta": 0.4886025372423295}, "gps": null, "landmarks": []}, {"step": 2, "t": 1.0, "odom": {"dx": 0.217545008846335, "dy": 0.05643189171546602, "dtheta": 0.3766346508481456}, "gps": null, "landmarks": []}, {"step": 3, "t": 1.5, "odom": {"dx": 0.2270974756173022, "dy": 0.02593173228203815, "dtheta": 0.20184937466053937}, "gps": null, "landmarks": []}, {"step": 4, "t": 2.0, "odom": {"dx": 0.18543273848013794, "dy": 0.02147294001119913, "dtheta": 0.06436396310085864}, "gps": null, "landmarks": []}, {"step": 5, "t": 2.5, "odom": {"dx": 0.21406636227251327, "dy": 0.00016461387480697, "dtheta": -0.047912010269036874}, "gps": null, "landmarks": []}, {"step": 6, "t": 3.0, "odom": {"dx": 0.2665084805751088, "dy": 0.023813984368249188, "dtheta": -0.021243083356897816}, "gps": null, "landmarks": []}, {"step": 7, "t": 3.5, "odom": {"dx": 0.2853138631292763, "dy": -0.017945895184682537, "dtheta": -0.09997408017050695}, "gps": null, "landmarks": []}, {"step": 8, "t": 4.0, "odom": {"dx": 0.24139980602078484, "dy": -0.01898423859456941, "dtheta": 0.00832135016571419}, "gps": null, "landmarks": []}, {"step": 9, "t": 4.5, "odom": {"dx": 0.25349302672588875, "dy": -0.007457190695313472, "dtheta": 0.017503459848943363}, "gps": null, "landmarks": []}, {"step": 10, "t": 5.0, "odom": {"dx": 0.23325090035613621, "dy": 0.0005927966003238432, "dtheta": -0.057166262686729724}, "gps": null, "landmarks": []}, {"step": 11, "t": 5.5, "odom": {"dx": 0.248359579259954, "dy": -0.01504653183794382, "dtheta": 0.013040317958357382}, "gps": null, "landmarks": []}, {"step": 12, "t": 6.0, "odom": {"dx": 0.2031977033320157, "dy": 0.0230189036889416, "dtheta": 0.007115274937243212}, "gps": null, "landmarks": []}, {"step": 13, "t": 6.5, "odom": {"dx": 0.28556153195046907, "dy": -0.014323721723801383, "dtheta": -0.017047931706411668}, "gps": null, "landmarks": []}, {"step": 14, "t": 7.0, "odom": {"dx": 0.26032323949645897, "dy": 0.026838804036254223, "dtheta": -0.0014011700279694562}, "gps": null, "landmarks": []}, {"step": 15, "t": 7.5, "odom": {"dx": 0.2826271022109707, "dy": -0.022958995042837948, "dtheta": -0.144181226142696}, "gps": null, "landmarks": []}, {"step": 16, "t": 8.0, "odom": {"dx": 0.29727328954621374, "dy": -0.05965191884449461, "dtheta": -0.28155246550421653}, "gps": null, "landmarks": []}, {"step": 17, "t": 8.5, "odom": {"dx": 0.22157681438206173, "dy": -0.012621647740679493, "dtheta": -0.21204895089148484}, "gps": null, "landmarks": []}, {"step": 18, "t": 9.0, "odom": {"dx": 0.27565149529702093, "dy": -0.037070842749163196, "dtheta": -0.1874706084565553}, "gps": null, "landmarks": []}, {"step": 19, "t": 9.5, "odom": {"dx": 0.26532041956947483, "dy": -0.013470207593736058, "dtheta": -0.10826510161616905}, "gps": null, "landmarks": []}, {"step": 20, "t": 10.0, "odom": {"dx": 0.24722035976617882, "dy": -0.014276507588035107, "dtheta": 0.0005819721211717758}, "gps": null, "landmarks": []}, {"step": 21, "t": 10.5, "odom": {"dx": 0.24946489457709536, "dy": -0.00015634171702163732, "dtheta": -0.03374805736890688}, "gps": null, "landmarks": []}, {"step": 22, "t": 11.0, "odom": {"dx": 0.20414893829599864, "dy": 0.0005303792931219526, "dtheta": -0.012557332040064177}, "gps": null, "landmarks": []}, {"step": 23, "t": 11.5, "odom": {"dx": 0.2787915538381812, "dy": -0.024134155730181243, "dtheta": 0.026121449140689975}, "gps": null, "landmarks": []}, {"step": 24, "t": 12.0, "odom": {"dx": 0.29392468495206114, "dy": -0.02748591618104597, "dtheta": 0.01570479103224044}, "gps": null, "landmarks": []}, {"step": 25, "t": 12.5, "odom": {"dx": 0.24065291340275025, "dy": 0.004937815895373218, "dtheta": -0.03546465289036823}, "gps": null, "landmarks": []}, {"step": 26, "t": 13.0, "odom": {"dx": 0.24728060275770145, "dy": 0.00920026831147802, "dtheta": -0.017375696505819764}, "gps": null, "landmarks": []}, {"step": 27, "t": 13.5, "odom": {"dx": 0.2764047973197916, "dy": 0.027063512634430506, "dtheta": 0.044778858765874084}, "gps": null, "landmarks": []}, {"step": 28, "t": 14.0, "odom": {"dx": 0.28668905866861943, "dy": -0.013863118475994179, "dtheta": -0.04856830749462169}, "gps": null, "landmarks": []}, {"step": 29, "t": 14.5, "odom": {"dx": 0.22681732647844396, "dy": -0.00031938055195913093, "dtheta": 0.026931397131921855}, "gps": null, "landmarks": []}, {"step": 30, "t": 15.0, "odom": {"dx": 0.2210536672720894, "dy": 0.010868839430869831, "dtheta": 0.036581841640107804}, "gps": null, "landmarks": []}, {"step": 31, "t": 15.5, "odom": {"dx": 0.248455822473009, "dy": -0.0028892588361147543, "dtheta": 0.0052345823210797416}, "gps": null, "landmarks": []}, {"step": 32, "t": 16.0, "odom": {"dx": 0.23418878132935533, "dy": -0.012684394321679676, "dtheta": -0.043711644100941124}, "gps": null, "landmarks": []}, {"step": 33, "t": 16.5, "odom": {"dx": 0.2073919485377766, "dy": -0.060902508170888284, "dtheta": -0.04652224447199079}, "gps": null, "landmarks": []}, {"step": 34, "t": 17.0, "odom": {"dx": 0.23897673169562547, "dy": 0.00973842416978339, "dtheta": 0.0199650721723089}, "gps": null, "landmarks": []}, {"step": 35, "t": 17.5, "odom": {"dx": 0.2560534050125987, "dy": -0.02851418161027209, "dtheta": 0.0256032260784932}, "gps": null, "landmarks": []}, {"step": 36, "t": 18.0, "odom": {"dx": 0.32868595231336717, "dy": 0.016482217752745176, "dtheta": -0.048591477637486585}, "gps": null, "landmarks": []}, {"step": 37, "t": 18.5, "odom": {"dx": 0.23816074864743017, "dy": -0.030168423808396877, "dtheta": 0.02606213499923244}, "gps": null, "landmarks": []}, {"step": 38, "t": 19.0, "odom": {"dx": 0.29868184844177176, "dy": -0.014830055095765036, "dtheta": 0.010663099656742371}, "gps": null, "landmarks": []}, {"step": 39, "t": 19.5, "odom": {"dx": 0.24830950603059218, "dy": 0.0351509498531715, "dtheta": 0.0060400913651236865}, "gps": null, "landmarks": []}, {"step": 40, "t": 20.0, "odom": {"dx": 0.2333766229147251, "dy": -0.005390476030513906, "dtheta": 0.02668192618661623}, "gps": null, "landmarks": []}, {"step": 41, "t": 20.5, "odom": {"dx": 0.2753090377892652, "dy": -0.015178036261365796, "dtheta": -0.04369722573837817}, "gps": null, "landmarks": []}, {"step": 42, "t": 21.0, "odom": {"dx": 0.2645469267890417, "dy": 0.015452115375311791, "dtheta": -0.0068402157563811575}, "gps": null, "landmarks": []}, {"step": 43, "t": 21.5, "odom": {"dx": 0.23364932261203863, "dy": 0.0054928905010615744, "dtheta": -0.16555983225960982}, "gps": null, "landmarks": []}, {"step": 44, "t": 22.0, "odom": {"dx": 0.26139289886170475, "dy": -0.03686421730132198, "dtheta": -0.1642576682005381}, "gps": null, "landmarks": []}, {"step": 45, "t": 22.5, "odom": {"dx": 0.22530319984881356, "dy": -0.06547500296089168, "dtheta": -0.2509842558697724}, "gps": null, "landmarks": []}, {"step": 46, "t": 23.0, "odom": {"dx": 0.28024578370036735, "dy": -0.010941655010782812, "dtheta": -0.09185223248575022}, "gps": null, "landmarks": []}, {"step": 47, "t": 23.5, "odom": {"dx": 0.22437351304707537, "dy": -0.020623232275335537, "dtheta": -0.14708765237079674}, "gps": null, "landmarks": []}, {"step": 48, "t": 24.0, "odom": {"dx": 0.18408000957200923, "dy": -0.010642807074925785, "dtheta": -0.0396207893848193}, "gps": null, "landmarks": []}, {"step": 49, "t": 24.5, "odom": {"dx": 0.24006443968434082, "dy": 0.016487454229412984, "dtheta": -0.10080461597521118}, "gps": null, "landmarks": []}, {"step": 50, "t": 25.0, "odom": {"dx": 0.24754920052863325, "dy": -0.038957678418157296, "dtheta": -0.04192465441849101}, "gps": null, "landmarks": []}, {"step": 51, "t": 25.5, "odom": {"dx": 0.2745816346590286, "dy": -0.012805767161139164, "dtheta": 0.033392646933696334}, "gps": null, "landmarks": []}, {"step": 52, "t": 26.0, "odom": {"dx": 0.26953460771416193, "dy": -0.016550455343191373, "dtheta": 0.010412469656588888}, "gps": null, "landmarks": []}, {"step": 53, "t": 26.5, "odom": {"dx": 0.19666946170213073, "dy": 0.007666716623619797, "dtheta": -0.006785628769774892}, "gps": null, "landmarks": []}, {"step": 54, "t": 27.0, "odom": {"dx": 0.24152159022198816, "dy": 0.011710367538477097, "dtheta": 0.05548826342641534}, "gps": null, "landmarks": []}, {"step": 55, "t": 27.5, "odom": {"dx": 0.2600842178537674, "dy": 0.021469385572850518, "dtheta": 0.029172913801995205}, "gps": null, "landmarks": []}, {"step": 56, "t": 28.0, "odom": {"dx": 0.260841997977341, "dy": 0.02140509115197769, "dtheta": -0.06299638591540903}, "gps": null, "landmarks": []}, {"step": 57, "t": 28.5, "odom": {"dx": 0.24952542899357794, "dy": -0.010325050771330996, "dtheta": -0.030646700245097518}, "gps": null, "landmarks": []}, {"step": 58, "t": 29.0, "odom": {"dx": 0.23936937545663112, "dy": -0.024518039617676443, "dtheta": 0.01912706586210473}, "gps": null, "landmarks": []}, {"step": 59, "t": 29.5, "odom": {"dx": 0.2446358247597065, "dy": 0.0032828157113811043, "dtheta": 0.02511416208358789}, "gps": null, "landmarks": []}]
```

## Contract
Implement solution.py with exactly this interface:

def make_estimator(public_spec):
    """Return a NEW estimator object. One object per episode; the verifier creates a fresh one for every
    trial, so no state may persist across episodes (no module-level caches or globals)."""

class Estimator:
    def step(self, new_measurements):
        """new_measurements is a dict, always containing "step" (int) and "t" (seconds):
          step 0:  {"step": 0, "t": 0.0, "odom": None, "gps": {"x","y"} or None, "landmarks": [...]}
          step k:  {"step": k, "t": k*dt, "odom": {"dx","dy","dtheta"} for transition k-1 -> k,
                    "gps": {"x","y"} or None, "landmarks": [{"id","range","bearing"}, ...] (possibly empty)}
        Returns (pose [x, y, theta], covariance 3x3 in local tangent frame of pose)."""

def choose_route(public_spec):
    """Returns {"route": "A" or "B", "success_prob": float, "reasoning": str}."""

Incremental (e.g. iSAM2) or batch inside step() are both allowed. Optional:
build_graph(measurements, public_spec) -> (gtsam.NonlinearFactorGraph, gtsam.Values) for diagnostics.
Same measurements fed to two fresh estimators must give identical outputs (deterministic).
The robot is driven by a pure-pursuit controller that uses YOUR estimator's pose, so choose_route should
reflect the collision-free arrival probability with your own estimator in the loop.
Available in the sandbox: Python 3.12, numpy, scipy, gtsam. No network, no files except what you write.
Answer with ONE ```python fenced block containing the complete solution.py.

## Revision 1 of 2: your previous solution.py
```python
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, L


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _unit_scale_pos(unit_str: str) -> float:
    if unit_str in ("m", "meter", "meters", None):
        return 1.0
    if unit_str in ("cm", "centimeter", "centimeters"):
        return 0.01
    if unit_str in ("mm", "millimeter", "millimeters"):
        return 0.001
    if unit_str in ("km", "kilometer", "kilometers"):
        return 1000.0
    return 1.0


def _unit_scale_ang(unit_str: str) -> float:
    if unit_str in ("rad", "radian", "radians", None):
        return 1.0
    if unit_str in ("deg", "degree", "degrees"):
        return math.pi / 180.0
    return 1.0


def _make_gps_factor(key, gx: float, gy: float, noise_model):
    def error_func(this: gtsam.CustomFactor, values: gtsam.Values, jacobians):
        pose: gtsam.Pose2 = values.atPose2(this.keys()[0])
        err = np.array([pose.x() - gx, pose.y() - gy], dtype=float)
        if jacobians is not None:
            c = math.cos(pose.theta())
            s = math.sin(pose.theta())
            jacobians[0] = np.array([[c, -s, 0.0], [s, c, 0.0]], dtype=float)
        return err

    return gtsam.CustomFactor(noise_model, [key], error_func)


def build_graph(measurements, public_spec):
    """Build a GTSAM NonlinearFactorGraph and initial Values from a sequence of measurements."""
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    input_units = public_spec.get("input_units", {})
    odom_pos_scale = _unit_scale_pos(input_units.get("odom", "m"))
    bearing_scale = _unit_scale_ang(input_units.get("bearing", "rad"))

    init_spec = public_spec["initial_pose"]
    init_mean = init_spec["mean"]
    init_vars = np.array(init_spec["cov_diag_variances"], dtype=float)

    pose0 = gtsam.Pose2(float(init_mean[0]), float(init_mean[1]), float(init_mean[2]))
    prior_noise = gtsam.noiseModel.Diagonal.Variances(init_vars)

    noise_spec = public_spec["sensor_noise_std"]
    odom_sigmas = np.array(noise_spec["odom"], dtype=float)
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)

    gps_std = noise_spec.get("gps")
    gps_noise = (
        gtsam.noiseModel.Isotropic.Sigma(2, float(gps_std))
        if gps_std is not None
        else None
    )

    bearing_std = noise_spec.get("bearing")
    range_std = noise_spec.get("range")
    br_noise = (
        gtsam.noiseModel.Diagonal.Sigmas(
            np.array([float(bearing_std), float(range_std)], dtype=float)
        )
        if (bearing_std is not None and range_std is not None)
        else None
    )

    landmarks_spec = public_spec.get("landmarks", {}) or {}
    added_landmarks = set()

    current_pose = pose0

    for m in measurements:
        k = int(m["step"])
        key_k = X(k)

        if k == 0:
            graph.add(gtsam.PriorFactorPose2(key_k, pose0, prior_noise))
            initial.insert(key_k, pose0)
        else:
            odom = m.get("odom")
            if odom is not None:
                dx = float(odom["dx"]) * odom_pos_scale
                dy = float(odom["dy"]) * odom_pos_scale
                dtheta = float(odom["dtheta"])
                rel_pose = gtsam.Pose2(dx, dy, dtheta)
            else:
                rel_pose = gtsam.Pose2(0.0, 0.0, 0.0)

            current_pose = current_pose.compose(rel_pose)
            graph.add(gtsam.BetweenFactorPose2(X(k - 1), key_k, rel_pose, odom_noise))
            initial.insert(key_k, current_pose)

        gps = m.get("gps")
        if gps is not None and gps_noise is not None:
            gx = float(gps["x"])
            gy = float(gps["y"])
            graph.add(_make_gps_factor(key_k, gx, gy, gps_noise))

        lms = m.get("landmarks") or []
        if lms and br_noise is not None:
            for lm in lms:
                lid_raw = lm["id"]
                lid_str = str(lid_raw)
                if lid_str in landmarks_spec:
                    lm_coords = landmarks_spec[lid_str]
                elif lid_raw in landmarks_spec:
                    lm_coords = landmarks_spec[lid_raw]
                else:
                    continue

                if isinstance(lid_raw, int) or (isinstance(lid_raw, str) and lid_raw.isdigit()):
                    l_key = L(int(lid_raw))
                else:
                    l_key = L( abs(hash(lid_str)) % (10**9) )

                if l_key not in added_landmarks:
                    pt = gtsam.Point2(float(lm_coords[0]), float(lm_coords[1]))
                    lm_prior = gtsam.noiseModel.Constrained.All(2)
                    graph.add(gtsam.PriorFactorPoint2(l_key, pt, lm_prior))
                    initial.insert(l_key, pt)
                    added_landmarks.add(l_key)

                r_val = float(lm["range"]) * odom_pos_scale
                b_val = float(lm["bearing"]) * bearing_scale
                graph.add(
                    gtsam.BearingRangeFactor2D(
                        key_k, l_key, gtsam.Rot2(b_val), r_val, br_noise
                    )
                )

    return graph, initial


class Estimator:
    def __init__(self, public_spec):
        self.public_spec = public_spec
        self.measurements = []

        out_units = public_spec.get("units_out", {})
        self.out_pos_scale = 1.0 / _unit_scale_pos(out_units.get("position", "m"))
        self.out_ang_scale = 1.0 / _unit_scale_ang(out_units.get("angle", "rad"))

    def step(self, new_measurements):
        self.measurements.append(new_measurements)
        k = int(new_measurements["step"])
        key_k = X(k)

        graph, initial = build_graph(self.measurements, self.public_spec)

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
        result = optimizer.optimize()

        marginals = gtsam.Marginals(graph, result)
        cov_local = marginals.marginalCovariance(key_k)

        pose_k: gtsam.Pose2 = result.atPose2(key_k)
        pose_out = [
            float(pose_k.x() * self.out_pos_scale),
            float(pose_k.y() * self.out_pos_scale),
            float(_wrap_angle(pose_k.theta()) * self.out_ang_scale),
        ]
        S = np.diag([self.out_pos_scale, self.out_pos_scale, self.out_ang_scale])
        cov_out = S @ np.asarray(cov_local, dtype=float) @ S

        return pose_out, cov_out


def make_estimator(public_spec):
    return Estimator(public_spec)


def _point_to_segment_dist_sq(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq <= 1e-12:
        return (px - x1) ** 2 + (py - y1) ** 2
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return (px - proj_x) ** 2 + (py - proj_y) ** 2


def _pure_pursuit_Lookahead(x, y, waypoints, lookahead):
    # Find closest point on path segments and advance by lookahead distance
    best_dist_sq = float("inf")
    best_seg = 0
    best_t = 0.0

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        len_sq = dx * dx + dy * dy
        if len_sq <= 1e-12:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / len_sq))
        px = x1 + t * dx
        py = y1 + t * dy
        d_sq = (x - px) ** 2 + (y - py) ** 2
        if d_sq < best_dist_sq:
            best_dist_sq = d_sq
            best_seg = i
            best_t = t

    rem = lookahead
    seg = best_seg
    x1, y1 = waypoints[seg]
    x2, y2 = waypoints[seg + 1]
    seg_len = math.hypot(x2 - x1, y2 - y1)
    dist_left_on_seg = (1.0 - best_t) * seg_len

    if rem <= dist_left_on_seg and seg_len > 1e-12:
        frac = best_t + rem / seg_len
        return x1 + frac * (x2 - x1), y1 + frac * (y2 - y1)

    rem -= dist_left_on_seg
    seg += 1
    while seg < len(waypoints) - 1:
        x1, y1 = waypoints[seg]
        x2, y2 = waypoints[seg + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        if rem <= seg_len and seg_len > 1e-12:
            frac = rem / seg_len
            return x1 + frac * (x2 - x1), y1 + frac * (y2 - y1)
        rem -= seg_len
        seg += 1

    return waypoints[-1][0], waypoints[-1][1]


def _simulate_route_mc(waypoints, public_spec, num_trials=400, seed=42):
    rng = np.random.default_rng(seed)
    dt = float(public_spec["dt"])
    max_steps = int(public_spec["max_steps"])
    arrival_tol = float(public_spec["arrival_tolerance"])
    robot_radius = float(public_spec["robot_radius"])
    walls = public_spec.get("walls", [])

    ctrl = public_spec["controller"]
    lookahead = float(ctrl["lookahead"])
    v_cmd = float(ctrl["v"])
    omega_max = float(ctrl["omega_max"])

    init_mean = np.array(public_spec["initial_pose"]["mean"], dtype=float)
    init_stds = np.sqrt(np.array(public_spec["initial_pose"]["cov_diag_variances"], dtype=float))

    odom_stds = np.array(public_spec["sensor_noise_std"]["odom"], dtype=float)
    act_v_std = float(public_spec["actuator_noise_std"]["v"])
    act_w_std = float(public_spec["actuator_noise_std"]["omega"])

    goal_x, goal_y = float(waypoints[-1][0]), float(waypoints[-1][1])
    r_sq = robot_radius * robot_radius

    def check_collision(rx, ry):
        for w in walls:
            if _point_to_segment_dist_sq(rx, ry, w[0], w[1], w[2], w[3]) <= r_sq:
                return True
        return False

    successes = 0
    for _ in range(num_trials):
        # True initial pose sampled from N(mean, diag(cov_diag_variances))
        true_x = init_mean[0] + rng.normal(0.0, init_stds[0])
        true_y = init_mean[1] + rng.normal(0.0, init_stds[1])
        true_th = _wrap_angle(init_mean[2] + rng.normal(0.0, init_stds[2]))

        # Estimator starts at prior mean (since no GPS/landmarks at step 0)
        est_x, est_y, est_th = init_mean[0], init_mean[1], init_mean[2]

        if check_collision(true_x, true_y):
            continue

        if math.hypot(true_x - goal_x, true_y - goal_y) <= arrival_tol:
            successes += 1
            continue

        arrived = False
        for _step in range(1, max_steps + 1):
            tx, ty = _pure_pursuit_Lookahead(est_x, est_y, waypoints, lookahead)
            alpha = _wrap_angle(math.atan2(ty - est_y, tx - est_x) - est_th)
            ld = max(math.hypot(tx - est_x, ty - est_y), 1e-6)
            omega_cmd = 2.0 * v_cmd * math.sin(alpha) / ld
            omega_cmd = max(-omega_max, min(omega_max, omega_cmd))

            v_true = v_cmd + rng.normal(0.0, act_v_std)
            w_true = omega_cmd + rng.normal(0.0, act_w_std)

            # True relative motion in body frame
            dth_true = w_true * dt
            if abs(dth_true) < 1e-6:
                dx_true = v_true * dt
                dy_true = 0.0
            else:
                dx_true = v_true * dt * math.sin(dth_true) / dth_true
                dy_true = v_true * dt * (1.0 - math.cos(dth_true)) / dth_true

            c_t = math.cos(true_th)
            s_t = math.sin(true_th)
            next_x = true_x + c_t * dx_true - s_t * dy_true
            next_y = true_y + s_t * dx_true + c_t * dy_true
            next_th = _wrap_angle(true_th + dth_true)

            # Check collision along step
            if check_collision(0.5 * (true_x + next_x), 0.5 * (true_y + next_y)) or check_collision(next_x, next_y):
                break

            true_x, true_y, true_th = next_x, next_y, next_th

            if math.hypot(true_x - goal_x, true_y - goal_y) <= arrival_tol:
                arrived = True
                break

            # Odom measurement in body frame (Pose2 composition with tangent noise)
            xi = rng.normal(0.0, odom_stds)
            rel_true = gtsam.Pose2(dx_true, dy_true, dth_true)
            rel_meas = rel_true.compose(gtsam.Pose2.Expmap(xi))

            est_pose = gtsam.Pose2(est_x, est_y, est_th).compose(rel_meas)
            est_x, est_y, est_th = est_pose.x(), est_pose.y(), est_pose.theta()

        if arrived:
            successes += 1

    return successes / float(num_trials)


def choose_route(public_spec):
    routes = public_spec["routes"]
    probs = {}
    lengths = {}

    for r_name, waypoints in routes.items():
        probs[r_name] = _simulate_route_mc(waypoints, public_spec, num_trials=500, seed=12345)
        lengths[r_name] = sum(
            math.hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1])
            for i in range(len(waypoints) - 1)
        )

    # Sort routes: maximize collision-free arrival probability; distance breaks close ties (within 0.02)
    route_names = sorted(routes.keys())
    best_route = route_names[0]
    for r_name in route_names[1:]:
        p_diff = probs[r_name] - probs[best_route]
        if p_diff > 0.02:
            best_route = r_name
        elif abs(p_diff) <= 0.02 and lengths[r_name] < lengths[best_route]:
            best_route = r_name

    other_routes = [r for r in route_names if r != best_route]
    other_info = ", ".join(f"route {r} ({probs[r]:.2f}, len {lengths[r]:.1f}m)" for r in other_routes)

    reasoning = (
        f"With pure odometry dead-reckoning (no GPS or landmarks), pose uncertainty grows unboundedly "
        f"from initial variance and odometry drift. Route A passes through a narrow 0.90m wall corridor "
        f"(0.25m clearance after 0.20m robot radius) at x=[5.4, 9.0], leading to frequent wall collisions. "
        f"Route {best_route} achieves a much higher collision-free arrival probability of "
        f"{probs[best_route]:.2f} (length {lengths[best_route]:.1f}m) vs {other_info}."
    )

    return {
        "route": best_route,
        "success_prob": float(probs[best_route]),
        "reasoning": reasoning,
    }

```

## Verifier feedback on that solution
- Route: stated success probability 0.70 differs from the measured rate 0.505 (95% CI [0.436, 0.574]) by more than 0.15.

Fix the failed checks. Return the complete revised solution.py.