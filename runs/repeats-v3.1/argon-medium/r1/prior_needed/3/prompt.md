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

## Revision 2 of 2: your previous solution.py
```python
import math
import numpy as np
import gtsam
from gtsam import symbol_shorthand

X = symbol_shorthand.X


def _unit_scale(unit_name):
    if unit_name in ("m", "rad", None):
        return 1.0
    if unit_name == "cm":
        return 0.01
    if unit_name == "mm":
        return 0.001
    if unit_name == "deg":
        return math.pi / 180.0
    raise ValueError(f"Unknown unit: {unit_name}")


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.current_estimates = gtsam.Values()

        init_mean = public_spec["initial_pose"]["mean"]
        init_vars = public_spec["initial_pose"]["cov_diag_variances"]
        self.init_pose = gtsam.Pose2(
            float(init_mean[0]), float(init_mean[1]), float(init_mean[2])
        )
        self.init_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array(init_vars, dtype=float)
        )

        odom_std = public_spec["sensor_noise_std"]["odom"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(odom_std, dtype=float)
        )

        self.has_gps = public_spec.get("gps_schedule") is not None
        if (
            "gps" in public_spec.get("sensor_noise_std", {})
            and public_spec["sensor_noise_std"]["gps"] is not None
        ):
            gps_std = float(public_spec["sensor_noise_std"]["gps"])
            self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(2, gps_std)
        else:
            self.gps_noise = None

        self.landmarks = public_spec.get("landmarks", {})
        if (
            "bearing" in public_spec.get("sensor_noise_std", {})
            and "range" in public_spec.get("sensor_noise_std", {})
            and public_spec["sensor_noise_std"]["bearing"] is not None
            and public_spec["sensor_noise_std"]["range"] is not None
        ):
            b_std = float(public_spec["sensor_noise_std"]["bearing"])
            r_std = float(public_spec["sensor_noise_std"]["range"])
            self.br_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([b_std, r_std], dtype=float)
            )
        else:
            self.br_noise = None

        in_units = public_spec.get("input_units", {})
        self.odom_pos_scale = _unit_scale(in_units.get("odom", "m"))
        self.bearing_scale = _unit_scale(in_units.get("bearing", "rad"))

        self.last_step = -1

    def _add_measurements_for_step(self, k, meas):
        key_k = X(k)

        if k == 0:
            self.graph.add(
                gtsam.PriorFactorPose2(key_k, self.init_pose, self.init_noise)
            )
            self.initial_estimates.insert(key_k, self.init_pose)
            self.current_estimates.insert(key_k, self.init_pose)
        else:
            odom = meas.get("odom")
            dx = float(odom["dx"]) * self.odom_pos_scale
            dy = float(odom["dy"]) * self.odom_pos_scale
            dtheta = float(odom["dtheta"])
            rel_pose = gtsam.Pose2(dx, dy, dtheta)

            prev_key = X(k - 1)
            self.graph.add(
                gtsam.BetweenFactorPose2(prev_key, key_k, rel_pose, self.odom_noise)
            )
            prev_est = self.current_estimates.atPose2(prev_key)
            pred_pose = prev_est.compose(rel_pose)
            self.initial_estimates.insert(key_k, pred_pose)
            self.current_estimates.insert(key_k, pred_pose)

        gps = meas.get("gps")
        if gps is not None and self.gps_noise is not None:
            gx = float(gps["x"])
            gy = float(gps["y"])
            self.graph.add(
                gtsam.GPSFactorPose2(key_k, gtsam.Point2(gx, gy), self.gps_noise)
            )

        lms = meas.get("landmarks")
        if lms and self.br_noise is not None:
            for obs in lms:
                lm_id = str(obs["id"])
                if lm_id not in self.landmarks:
                    continue
                lm_pos = self.landmarks[lm_id]
                lm_pt = gtsam.Point2(float(lm_pos[0]), float(lm_pos[1]))
                bearing = gtsam.Rot2(float(obs["bearing"]) * self.bearing_scale)
                rng = float(obs["range"]) * self.odom_pos_scale
                self.graph.add(
                    gtsam.BearingRangeFactor2D(
                        key_k, lm_pt, bearing, rng, self.br_noise
                    )
                )

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        self._add_measurements_for_step(k, new_measurements)
        self.last_step = k

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, self.current_estimates, params
        )
        self.current_estimates = optimizer.optimize()

        key_k = X(k)
        pose_k = self.current_estimates.atPose2(key_k)
        marginals = gtsam.Marginals(self.graph, self.current_estimates)
        cov_k = marginals.marginalCovariance(key_k)

        out_pose = [float(pose_k.x()), float(pose_k.y()), float(pose_k.theta())]
        out_cov = np.array(cov_k, dtype=float)
        return out_pose, out_cov


def make_estimator(public_spec):
    return Estimator(public_spec)


def build_graph(measurements, public_spec):
    est = make_estimator(public_spec)
    for m in measurements:
        est.step(m)
    return est.graph, est.current_estimates


def _point_to_segment_dist_sq(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0.0:
        return (px - x1) ** 2 + (py - y1) ** 2
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return (px - proj_x) ** 2 + (py - proj_y) ** 2


def _check_collision(rx, ry, walls, radius):
    r_sq = radius * radius
    for w in walls:
        if _point_to_segment_dist_sq(rx, ry, w[0], w[1], w[2], w[3]) <= r_sq:
            return True
    return False


def _pure_pursuit_control(pose, waypoints, seg_idx, lookahead, v_cmd, omega_max):
    px, py, ptheta = pose
    n_seg = len(waypoints) - 1

    while seg_idx < n_seg - 1:
        wx, wy = waypoints[seg_idx + 1]
        if math.hypot(wx - px, wy - py) < lookahead:
            seg_idx += 1
        else:
            break

    target = waypoints[-1]
    for i in range(seg_idx, n_seg):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        len_seg = math.hypot(dx, dy)
        if len_seg < 1e-9:
            continue
        t_proj = ((px - x1) * dx + (py - y1) * dy) / (len_seg * len_seg)
        t_proj = max(0.0, min(1.0, t_proj))
        proj_x = x1 + t_proj * dx
        proj_y = y1 + t_proj * dy
        dist_proj = math.hypot(px - proj_x, py - proj_y)
        if dist_proj <= lookahead:
            rem = math.sqrt(max(0.0, lookahead * lookahead - dist_proj * dist_proj))
            t_look = t_proj + rem / len_seg
            if t_look <= 1.0 or i == n_seg - 1:
                t_look = min(1.0, t_look)
                target = (x1 + t_look * dx, y1 + t_look * dy)
                seg_idx = i
                break
        else:
            target = (proj_x, proj_y)
            seg_idx = i
            break

    tx, ty = target
    angle_to_target = math.atan2(ty - py, tx - px)
    alpha = math.atan2(
        math.sin(angle_to_target - ptheta), math.cos(angle_to_target - ptheta)
    )
    dist = max(1e-6, math.hypot(tx - px, ty - py))
    L_eff = min(lookahead, dist) if seg_idx == n_seg - 1 else lookahead
    L_eff = max(0.1, L_eff)
    curvature = 2.0 * math.sin(alpha) / L_eff
    omega = v_cmd * curvature
    omega = max(-omega_max, min(omega_max, omega))
    return v_cmd, omega, seg_idx


def _simulate_route(route_waypoints, public_spec, n_trials=400, seed=42):
    rng = np.random.default_rng(seed)
    dt = float(public_spec["dt"])
    max_steps = int(public_spec["max_steps"])
    tol = float(public_spec["arrival_tolerance"])
    radius = float(public_spec["robot_radius"])
    walls = public_spec.get("walls", [])

    init_mean = np.array(public_spec["initial_pose"]["mean"], dtype=float)
    init_std = np.sqrt(
        np.array(public_spec["initial_pose"]["cov_diag_variances"], dtype=float)
    )

    odom_std = np.array(public_spec["sensor_noise_std"]["odom"], dtype=float)
    v_std = float(public_spec["actuator_noise_std"]["v"])
    w_std = float(public_spec["actuator_noise_std"]["omega"])

    ctrl = public_spec["controller"]
    lookahead = float(ctrl["lookahead"])
    v_cmd = float(ctrl["v"])
    omega_max = float(ctrl["omega_max"])

    goal = route_waypoints[-1]
    successes = 0

    for _ in range(n_trials):
        true_vec = init_mean + rng.normal(0.0, init_std)
        true_pose = gtsam.Pose2(true_vec[0], true_vec[1], true_vec[2])
        est_pose = gtsam.Pose2(init_mean[0], init_mean[1], init_mean[2])
        seg_idx = 0

        if _check_collision(true_pose.x(), true_pose.y(), walls, radius):
            continue

        arrived = False
        for _step in range(1, max_steps + 1):
            if math.hypot(true_pose.x() - goal[0], true_pose.y() - goal[1]) <= tol:
                arrived = True
                break

            # If estimated pose has already reached the goal, controller stops making progress
            if math.hypot(est_pose.x() - goal[0], est_pose.y() - goal[1]) <= tol:
                break

            est_tuple = (est_pose.x(), est_pose.y(), est_pose.theta())
            v, omega, seg_idx = _pure_pursuit_control(
                est_tuple, route_waypoints, seg_idx, lookahead, v_cmd, omega_max
            )

            v_actual = v + rng.normal(0.0, v_std)
            w_actual = omega + rng.normal(0.0, w_std)

            rel_true = gtsam.Pose2.Expmap(
                np.array([v_actual * dt, 0.0, w_actual * dt], dtype=float)
            )
            next_true = true_pose.compose(rel_true)

            mid_x = 0.5 * (true_pose.x() + next_true.x())
            mid_y = 0.5 * (true_pose.y() + next_true.y())
            if _check_collision(mid_x, mid_y, walls, radius) or _check_collision(
                next_true.x(), next_true.y(), walls, radius
            ):
                break

            true_pose = next_true

            noise_xi = rng.normal(0.0, odom_std)
            rel_meas = rel_true.compose(gtsam.Pose2.Expmap(noise_xi))
            est_pose = est_pose.compose(rel_meas)

            if math.hypot(true_pose.x() - goal[0], true_pose.y() - goal[1]) <= tol:
                arrived = True
                break

        if arrived:
            successes += 1

    return successes / float(n_trials)


def choose_route(public_spec):
    routes = public_spec["routes"]
    probs = {}
    lengths = {}
    for name, wps in routes.items():
        probs[name] = _simulate_route(wps, public_spec, n_trials=500, seed=12345)
        length = sum(
            math.hypot(wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1])
            for i in range(len(wps) - 1)
        )
        lengths[name] = length

    best_route = max(routes.keys(), key=lambda r: (round(probs[r], 2), -lengths[r]))
    return {
        "route": best_route,
        "success_prob": float(probs[best_route]),
        "reasoning": (
            f"With dead-reckoning only (no GPS or landmarks), pose uncertainty grows "
            f"along the trajectory. Route A passes through a narrow wall corridor "
            f"(clearance 0.25m) where lateral drift causes frequent collisions "
            f"(estimated success prob {probs.get('A', 0.0):.2f}), whereas Route B detours "
            f"around the walls in open space and reaches the goal within arrival_tolerance "
            f"about half the time despite terminal drift (estimated success prob {probs.get('B', 0.0):.2f})."
        ),
    }

```

## Verifier feedback on that solution
- Route: stated success probability 0.23 differs from the measured rate 0.505 (95% CI [0.436, 0.574]) by more than 0.15.

Fix the failed checks. Return the complete revised solution.py.