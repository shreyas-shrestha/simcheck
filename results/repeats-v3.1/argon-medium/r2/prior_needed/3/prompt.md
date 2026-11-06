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
L = symbol_shorthand.L


def _unit_scale(unit_name):
    if unit_name in ("m", "rad", None):
        return 1.0
    if unit_name == "cm":
        return 0.01
    if unit_name == "mm":
        return 0.001
    if unit_name == "deg":
        return math.pi / 180.0
    return 1.0


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.current_estimate = gtsam.Values()
        self.initialized_landmarks = set()

        init_mean = public_spec["initial_pose"]["mean"]
        init_vars = public_spec["initial_pose"]["cov_diag_variances"]
        self.init_pose = gtsam.Pose2(
            float(init_mean[0]), float(init_mean[1]), float(init_mean[2])
        )
        self.init_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array(init_vars, dtype=float)
        )

        sensor_std = public_spec["sensor_noise_std"]
        odom_std = sensor_std["odom"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(odom_std, dtype=float)
        )

        bearing_std = float(sensor_std.get("bearing", 0.03))
        range_std = float(sensor_std.get("range", 0.1))
        self.br_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([bearing_std, range_std], dtype=float)
        )

        gps_std = float(sensor_std.get("gps", 0.15))
        # Large sigma on theta so PriorFactorPose2 acts solely as an (x, y) prior
        self.gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([gps_std, gps_std, 1e6], dtype=float)
        )

        input_units = public_spec.get("input_units", {})
        self.odom_pos_scale = _unit_scale(input_units.get("odom", "m"))
        self.bearing_scale = _unit_scale(input_units.get("bearing", "rad"))

        self.landmarks_spec = public_spec.get("landmarks", {})
        self.last_pose = self.init_pose
        self.last_step = -1

    def _add_known_landmark(self, lm_id_str):
        if lm_id_str not in self.initialized_landmarks and lm_id_str in self.landmarks_spec:
            lm_pos = self.landmarks_spec[lm_id_str]
            lm_key = L(int(lm_id_str))
            pt = gtsam.Point2(float(lm_pos[0]), float(lm_pos[1]))
            prior_noise = gtsam.noiseModel.Constrained.All(2)
            self.graph.add(gtsam.PriorFactorPoint2(lm_key, pt, prior_noise))
            self.initial_estimate.insert(lm_key, pt)
            self.initialized_landmarks.add(lm_id_str)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        pose_key = X(k)

        if k == 0:
            self.graph.add(
                gtsam.PriorFactorPose2(pose_key, self.init_pose, self.init_noise)
            )
            self.initial_estimate.insert(pose_key, self.init_pose)
            pred_pose = self.init_pose
        else:
            odom = new_measurements.get("odom")
            if odom is not None:
                dx = float(odom["dx"]) * self.odom_pos_scale
                dy = float(odom["dy"]) * self.odom_pos_scale
                dtheta = float(odom["dtheta"])
                rel_pose = gtsam.Pose2(dx, dy, dtheta)
            else:
                rel_pose = gtsam.Pose2(0.0, 0.0, 0.0)

            prev_key = X(k - 1)
            self.graph.add(
                gtsam.BetweenFactorPose2(
                    prev_key, pose_key, rel_pose, self.odom_noise
                )
            )
            pred_pose = self.last_pose.compose(rel_pose)
            self.initial_estimate.insert(pose_key, pred_pose)

        gps = new_measurements.get("gps")
        if gps is not None:
            gx = float(gps["x"]) * self.odom_pos_scale
            gy = float(gps["y"]) * self.odom_pos_scale
            gps_pose = gtsam.Pose2(gx, gy, pred_pose.theta())
            self.graph.add(
                gtsam.PriorFactorPose2(pose_key, gps_pose, self.gps_noise)
            )

        landmarks = new_measurements.get("landmarks") or []
        for lm in landmarks:
            lm_id_str = str(lm["id"])
            self._add_known_landmark(lm_id_str)
            lm_key = L(int(lm_id_str))
            r = float(lm["range"]) * self.odom_pos_scale
            b = float(lm["bearing"]) * self.bearing_scale
            self.graph.add(
                gtsam.BearingRangeFactor2D(
                    pose_key, lm_key, gtsam.Rot2(b), r, self.br_noise
                )
            )

        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, self.initial_estimate, params
        )
        self.current_estimate = optimizer.optimize()
        # Keep initial_estimate synced with optimized values for fast convergence on next step
        self.initial_estimate = gtsam.Values(self.current_estimate)

        est_pose = self.current_estimate.atPose2(pose_key)
        self.last_pose = est_pose
        self.last_step = k

        marginals = gtsam.Marginals(self.graph, self.current_estimate)
        cov = marginals.marginalCovariance(pose_key)

        pose_out = [
            float(est_pose.x()),
            float(est_pose.y()),
            float(est_pose.theta()),
        ]
        return pose_out, np.array(cov, dtype=float)


def make_estimator(public_spec):
    return Estimator(public_spec)


def build_graph(measurements, public_spec):
    est = make_estimator(public_spec)
    for m in measurements:
        est.step(m)
    return est.graph, est.current_estimate


def _dist_point_to_segment(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / len_sq))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def _check_collision(x, y, walls, radius):
    for w in walls:
        if _dist_point_to_segment(x, y, w[0], w[1], w[2], w[3]) <= radius:
            return True
    return False


def _pure_pursuit_control(pose, waypoints, lookahead, v_cmd, omega_max):
    x, y, theta = pose
    best_dist = float("inf")
    best_seg = 0
    best_t = 0.0
    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx, dy = x2 - x1, y2 - y1
        len_sq = dx * dx + dy * dy
        t = 0.0 if len_sq == 0.0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / len_sq))
        px, py = x1 + t * dx, y1 + t * dy
        d = math.hypot(x - px, y - py)
        if d < best_dist:
            best_dist = d
            best_seg = i
            best_t = t

    rem = lookahead
    seg = best_seg
    t_cur = best_t
    target_x, target_y = waypoints[-1]
    while seg < len(waypoints) - 1:
        x1, y1 = waypoints[seg]
        x2, y2 = waypoints[seg + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        avail = (1.0 - t_cur) * seg_len
        if rem <= avail and seg_len > 0:
            t_target = t_cur + rem / seg_len
            target_x = x1 + t_target * (x2 - x1)
            target_y = y1 + t_target * (y2 - y1)
            break
        rem -= avail
        seg += 1
        t_cur = 0.0

    dx_w = target_x - x
    dy_w = target_y - y
    local_x = math.cos(theta) * dx_w + math.sin(theta) * dy_w
    local_y = -math.sin(theta) * dx_w + math.cos(theta) * dy_w
    L_sq = local_x * local_x + local_y * local_y
    if L_sq < 1e-9:
        omega = 0.0
    else:
        curvature = 2.0 * local_y / L_sq
        omega = v_cmd * curvature
    omega = max(-omega_max, min(omega_max, omega))
    return v_cmd, omega


def _simulate_route(route_waypoints, public_spec, num_trials=400, seed=42):
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
    init_std = np.sqrt(np.array(public_spec["initial_pose"]["cov_diag_variances"], dtype=float))
    odom_std = np.array(public_spec["sensor_noise_std"]["odom"], dtype=float)
    act_v_std = float(public_spec["actuator_noise_std"]["v"])
    act_w_std = float(public_spec["actuator_noise_std"]["omega"])

    goal_x, goal_y = route_waypoints[-1]
    success_count = 0

    for _ in range(num_trials):
        true_pose = gtsam.Pose2(*(init_mean + rng.normal(0.0, init_std)))
        est_pose = gtsam.Pose2(*init_mean)

        if _check_collision(true_pose.x(), true_pose.y(), walls, robot_radius):
            continue

        arrived = False
        for _step in range(1, max_steps + 1):
            est_tuple = (est_pose.x(), est_pose.y(), est_pose.theta())
            v, omega = _pure_pursuit_control(est_tuple, route_waypoints, lookahead, v_cmd, omega_max)

            v_true = v + rng.normal(0.0, act_v_std)
            w_true = omega + rng.normal(0.0, act_w_std)
            true_rel = gtsam.Pose2.Expmap(np.array([v_true * dt, 0.0, w_true * dt]))
            prev_true = true_pose
            true_pose = true_pose.compose(true_rel)

            mid_pose = prev_true.compose(
                gtsam.Pose2.Expmap(np.array([0.5 * v_true * dt, 0.0, 0.5 * w_true * dt]))
            )
            if _check_collision(mid_pose.x(), mid_pose.y(), walls, robot_radius) or _check_collision(
                true_pose.x(), true_pose.y(), walls, robot_radius
            ):
                break

            xi_noise = rng.normal(0.0, odom_std)
            meas_rel = true_rel.compose(gtsam.Pose2.Expmap(xi_noise))
            est_pose = est_pose.compose(meas_rel)

            if math.hypot(true_pose.x() - goal_x, true_pose.y() - goal_y) <= arrival_tol:
                arrived = True
                break

            # Once the estimated pose reaches the goal, the controller has finished the trajectory
            if math.hypot(est_pose.x() - goal_x, est_pose.y() - goal_y) <= arrival_tol:
                break

        if arrived:
            success_count += 1

    return success_count / float(num_trials)


def choose_route(public_spec):
    routes = public_spec["routes"]
    probs = {}
    lengths = {}
    for name, wypts in routes.items():
        probs[name] = _simulate_route(wypts, public_spec, num_trials=400, seed=12345)
        lengths[name] = sum(
            math.hypot(wypts[i + 1][0] - wypts[i][0], wypts[i + 1][1] - wypts[i][1])
            for i in range(len(wypts) - 1)
        )

    best_route = max(routes.keys(), key=lambda r: (round(probs[r], 2), -lengths[r]))
    reasoning = (
        f"Without GPS or landmarks, pose estimation relies purely on dead-reckoning odometry, "
        f"causing lateral uncertainty to grow over distance. Route A passes through a narrow wall "
        f"corridor (y in [-0.45, 0.45] for x in [5.4, 9.0]) with only 0.25m clearance, resulting "
        f"in high collision risk (estimated success probability {probs.get('A', 0.0):.2f}). "
        f"Route B detours safely around the walls via y=3.0 and achieves a higher "
        f"collision-free arrival probability ({probs.get('B', 0.0):.2f})."
    )
    return {
        "route": best_route,
        "success_prob": float(probs[best_route]),
        "reasoning": reasoning,
    }

```

## Verifier feedback on that solution
- Route: chose A, success rates with your estimator in the loop {'A': 0.295, 'B': 0.505} over T=200 paired trials; regret 0.210, 95% CI [0.135, 0.285], tolerance 0.05 -> fail.

Fix the failed checks. Return the complete revised solution.py.