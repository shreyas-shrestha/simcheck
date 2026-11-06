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
"""GTSAM odometry estimator and deterministic closed-loop route evaluation."""

import copy
import math

import gtsam
import numpy as np


def _wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    """Prior plus Pose2 odometry factors; covariances use local coordinates."""

    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        )
        prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.asarray(initial["cov_diag_variances"], dtype=float)
        )

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPose2(0, self.pose, prior_noise))
        values.insert(0, self.pose)
        self.isam.update(graph, values)

        self.last_step = -1
        self.last_result = None
        units = self.spec["input_units"]
        self.translation_scale = {
            "m": 1.0, "cm": 0.01, "mm": 0.001
        }[units["odom"]]
        self.angle_scale = {
            "rad": 1.0, "deg": math.pi / 180.0
        }[units["bearing"]]

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k == self.last_step:
            return tuple(a.copy() for a in self.last_result)
        if k != self.last_step + 1:
            raise ValueError("Expected consecutive measurements starting at step 0.")
        if new_measurements.get("gps") is not None:
            raise ValueError("This scenario specifies no GPS.")
        if new_measurements.get("landmarks"):
            raise ValueError("This scenario specifies no landmarks.")

        odom = new_measurements.get("odom")
        if k == 0:
            if odom is not None:
                raise ValueError("Step 0 must have null odometry.")
        else:
            if odom is None:
                raise ValueError("Missing odometry transition.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]) * self.angle_scale,
            )
            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(k - 1, k, relative, self.odom_noise)
            )
            values.insert(k, self.pose.compose(relative))
            self.isam.update(graph, values)

        self.pose = self.isam.calculateEstimatePose2(k)
        covariance = np.asarray(self.isam.marginalCovariance(k), dtype=float)
        covariance = (covariance + covariance.T) * 0.5
        pose = np.array(
            [self.pose.x(), self.pose.y(), self.pose.theta()], dtype=float
        )
        self.last_step = k
        self.last_result = (pose.copy(), covariance.copy())
        return pose, covariance


def _point_segment_distance_squared(point, start, end):
    delta = end - start
    denominator = np.sum(delta * delta, axis=-1)
    numerator = np.sum((point - start) * delta, axis=-1)
    fraction = np.clip(
        numerator / np.maximum(denominator, np.finfo(float).tiny), 0.0, 1.0
    )
    difference = point - (start + fraction[..., None] * delta)
    return np.sum(difference * difference, axis=-1)


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _swept_collision(start, end, walls, radius):
    collision = np.zeros(len(start), dtype=bool)
    for wall in walls:
        a, b = wall[:2], wall[2:]
        motion, barrier = end - start, b - a
        c1 = _cross(motion, a - start)
        c2 = _cross(motion, b - start)
        c3 = _cross(barrier, start - a)
        c4 = _cross(barrier, end - a)
        box_overlap = np.all(
            np.maximum(np.minimum(start, end), np.minimum(a, b))
            <= np.minimum(np.maximum(start, end), np.maximum(a, b)),
            axis=1,
        )
        intersects = (c1 * c2 <= 0.0) & (c3 * c4 <= 0.0) & box_overlap
        distance_squared = np.minimum.reduce([
            _point_segment_distance_squared(start, a, b),
            _point_segment_distance_squared(end, a, b),
            _point_segment_distance_squared(a, start, end),
            _point_segment_distance_squared(b, start, end),
        ])
        collision |= intersects | (distance_squared <= radius * radius)
    return collision


def _route_probability(spec, waypoints, trials):
    """Simulate feedback using the exact mean of the odometry-only graph.

    Arrival is tested on truth. An estimated arrival is neither a true arrival
    nor an episode-termination condition in the supplied success definition.
    In particular, it must not discard trials that can arrive later.
    """
    rng = np.random.default_rng(1729)
    route = np.asarray(waypoints, dtype=float)
    if route.ndim != 2 or route.shape[1] != 2 or len(route) == 0:
        raise ValueError("Expected a nonempty list of 2D waypoints.")
    keep = np.r_[True, np.linalg.norm(np.diff(route, axis=0), axis=1) > 0.0]
    route = route[keep]
    segments = np.diff(route, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    total_length = float(cumulative[-1])

    initial = spec["initial_pose"]
    mean = np.asarray(initial["mean"], dtype=float)
    initial_std = np.sqrt(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    truth = mean + rng.standard_normal((trials, 3)) * initial_std
    truth[:, 2] = _wrap(truth[:, 2])
    estimate = np.tile(mean, (trials, 1))

    dt = float(spec["dt"])
    controller = spec["controller"]
    if controller["type"] != "pure_pursuit":
        raise ValueError("Expected pure_pursuit.")
    speed = float(controller["v"])
    lookahead = float(controller["lookahead"])
    omega_max = float(controller["omega_max"])
    actuator = spec["actuator_noise_std"]
    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    tolerance = float(spec["arrival_tolerance"])
    radius = float(spec["robot_radius"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)
    goal = route[-1]

    collided = _swept_collision(truth[:, :2], truth[:, :2], walls, radius)
    success = (
        np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    ) & ~collided
    active = ~collided & ~success
    progress = np.zeros(trials)
    rows = np.arange(trials)

    for _ in range(max(0, int(spec["max_steps"]) - 1)):
        if not np.any(active):
            break

        if len(lengths):
            displacement = estimate[:, None, :2] - route[None, :-1, :]
            fraction = np.clip(
                np.sum(displacement * segments[None, :, :], axis=2)
                / lengths[None, :] ** 2,
                0.0,
                1.0,
            )
            projection = (
                route[None, :-1, :]
                + fraction[:, :, None] * segments[None, :, :]
            )
            distance_squared = np.sum(
                (estimate[:, None, :2] - projection) ** 2, axis=2
            )
            closest = np.argmin(distance_squared, axis=1)
            projected_progress = (
                cumulative[closest]
                + fraction[rows, closest] * lengths[closest]
            )
            progress = np.maximum(progress, projected_progress)
            target_progress = np.minimum(progress + lookahead, total_length)
            target_segment = np.clip(
                np.searchsorted(cumulative, target_progress, side="right") - 1,
                0,
                len(lengths) - 1,
            )
            target_fraction = (
                target_progress - cumulative[target_segment]
            ) / lengths[target_segment]
            target = (
                route[target_segment]
                + target_fraction[:, None] * segments[target_segment]
            )
        else:
            target = np.broadcast_to(goal, (trials, 2))

        toward = target - estimate[:, :2]
        squared_distance = np.sum(toward * toward, axis=1)
        lateral = (
            -np.sin(estimate[:, 2]) * toward[:, 0]
            + np.cos(estimate[:, 2]) * toward[:, 1]
        )
        commanded_omega = np.clip(
            2.0 * speed * lateral / np.maximum(squared_distance, 1e-12),
            -omega_max,
            omega_max,
        )
        realized_v = speed + float(actuator["v"]) * rng.standard_normal(trials)
        realized_omega = (
            commanded_omega
            + float(actuator["omega"]) * rng.standard_normal(trials)
        )
        turn = realized_omega * dt

        # Constant-twist SE(2) integration of the realized actuator motion.
        forward = realized_v * dt * np.sinc(turn / np.pi)
        sideways = (
            realized_v * dt * (0.5 * turn)
            * np.sinc(turn / (2.0 * np.pi)) ** 2
        )
        ct, st = np.cos(truth[:, 2]), np.sin(truth[:, 2])
        next_truth = truth.copy()
        next_truth[:, 0] += ct * forward - st * sideways
        next_truth[:, 1] += st * forward + ct * sideways
        next_truth[:, 2] = _wrap(truth[:, 2] + turn)
        next_truth[~active] = truth[~active]

        # Pose2 right-local measurement perturbation. Noise is already SI.
        noise = rng.standard_normal((trials, 3)) * odom_std
        cr, sr = np.cos(turn), np.sin(turn)
        measured_x = forward + cr * noise[:, 0] - sr * noise[:, 1]
        measured_y = sideways + sr * noise[:, 0] + cr * noise[:, 1]
        measured_theta = turn + noise[:, 2]

        # This composition equals the GTSAM chain's zero-residual MAP estimate.
        ce, se = np.cos(estimate[:, 2]), np.sin(estimate[:, 2])
        next_estimate = estimate.copy()
        next_estimate[:, 0] += ce * measured_x - se * measured_y
        next_estimate[:, 1] += se * measured_x + ce * measured_y
        next_estimate[:, 2] = _wrap(estimate[:, 2] + measured_theta)
        next_estimate[~active] = estimate[~active]

        collided = _swept_collision(
            truth[:, :2], next_truth[:, :2], walls, radius
        ) & active
        reached = (
            np.linalg.norm(next_truth[:, :2] - goal, axis=1) <= tolerance
        ) & active & ~collided
        success |= reached
        active &= ~collided & ~reached
        truth, estimate = next_truth, next_estimate

    return float(np.mean(success)), total_length


def choose_route(public_spec):
    spec = copy.deepcopy(public_spec)
    trials = 4096
    results = {
        name: _route_probability(spec, points, trials)
        for name, points in spec["routes"].items()
    }
    best_probability = max(p for p, _ in results.values())
    standard_error = math.sqrt(
        best_probability * (1.0 - best_probability) / trials
    )
    candidates = [
        name for name, (probability, _) in results.items()
        if best_probability - probability <= 2.0 * standard_error
    ]
    chosen = min(candidates, key=lambda name: (results[name][1], name))
    summary = "; ".join(
        f"{name}: success approximately {p:.3f}, length {length:.2f} m"
        for name, (p, length) in sorted(results.items())
    )
    return {
        "route": chosen,
        "success_prob": results[chosen][0],
        "reasoning": (
            "Deterministic closed-loop Monte Carlo using the odometry-only "
            "GTSAM estimator's mean, configured pure pursuit, and initial, "
            "actuator, and odometry uncertainties. Collision checks include "
            "robot radius and swept motion. Arrival is checked on true position "
            "throughout the time budget: estimated proximity alone does not "
            "terminate a trial. Without absolute observations, heading drift "
            "makes the narrow corridor risky. Distance breaks statistically "
            "close ties. " + summary + ". Probabilities are approximate because "
            "the detailed pure-pursuit implementation is not specified."
        ),
    }

```

## Verifier feedback on that solution
- Route: stated success probability 0.70 differs from the measured rate 0.505 (95% CI [0.436, 0.574]) by more than 0.15.

Fix the failed checks. Return the complete revised solution.py.