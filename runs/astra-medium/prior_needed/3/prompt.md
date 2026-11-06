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
"""Odometry-only GTSAM estimation and deterministic closed-loop route assessment."""

import copy
import math

import gtsam
import numpy as np


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)
        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))

        variances = np.asarray(initial["cov_diag_variances"], dtype=float)
        sigmas = np.asarray(self.spec["sensor_noise_std"]["odom"], dtype=float)
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)

        if self.spec["odom_frame"] != "body":
            raise ValueError("Expected body-frame relative-pose odometry.")
        unit = self.spec["input_units"]["odom"].lower()
        scales = {"m": 1.0, "cm": 0.01, "mm": 0.001}
        if unit not in scales:
            raise ValueError("Unsupported odometry translation unit: " + unit)
        self.translation_scale = scales[unit]

        self.isam = gtsam.ISAM2()
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(
            gtsam.PriorFactorPose2(
                0, self.pose, gtsam.noiseModel.Diagonal.Variances(variances)
            )
        )
        values.insert(0, self.pose)
        self.isam.update(graph, values)
        self.last_step = -1

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must arrive once, in step order.")
        if new_measurements.get("gps") is not None:
            raise ValueError("This scenario has no GPS.")
        if new_measurements.get("landmarks"):
            raise ValueError("This scenario has no landmarks.")

        if k == 0:
            if new_measurements.get("odom") is not None:
                raise ValueError("Step zero must not contain odometry.")
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("Missing odometry transition.")
            relative = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]),
            )
            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(
                    k - 1, k, relative, self.odom_noise
                )
            )
            values.insert(k, self.pose.compose(relative))
            self.isam.update(graph, values)

        self.pose = self.isam.calculateEstimatePose2(k)
        covariance = np.asarray(self.isam.marginalCovariance(k), dtype=float)
        covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k
        return (
            [self.pose.x(), self.pose.y(), self.pose.theta()],
            covariance.tolist(),
        )


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _compose(pose, relative):
    """Vectorized Pose2 composition, with one pose per row."""
    c = np.cos(pose[:, 2])
    s = np.sin(pose[:, 2])
    result = np.empty_like(pose)
    result[:, 0] = pose[:, 0] + c * relative[:, 0] - s * relative[:, 1]
    result[:, 1] = pose[:, 1] + s * relative[:, 0] + c * relative[:, 1]
    result[:, 2] = _wrap(pose[:, 2] + relative[:, 2])
    return result


def _cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _point_segment_distance_squared(point, start, end):
    """Supports broadcasting a fixed point or segment over a batch."""
    delta = end - start
    denominator = np.sum(delta * delta, axis=-1)
    numerator = np.sum((point - start) * delta, axis=-1)
    fraction = np.divide(
        numerator,
        denominator,
        out=np.zeros_like(numerator, dtype=float),
        where=denominator > 0.0,
    )
    fraction = np.clip(fraction, 0.0, 1.0)
    difference = point - (start + fraction[..., None] * delta)
    return np.sum(difference * difference, axis=-1)


def _collisions(previous, current, walls, radius):
    """Swept-disc collision test against finite wall segments."""
    collided = np.zeros(len(current), dtype=bool)
    motion = current - previous

    for wall in walls:
        a, b = wall[:2], wall[2:]
        edge = b - a
        d2 = np.minimum(
            _point_segment_distance_squared(previous, a, b),
            _point_segment_distance_squared(current, a, b),
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(a, previous, current)
        )
        d2 = np.minimum(
            d2, _point_segment_distance_squared(b, previous, current)
        )

        # Distances to endpoints alone do not detect crossing segments.
        denominator = _cross(motion, edge)
        offset = a - previous
        nonparallel = np.abs(denominator) > 1e-14
        t = np.divide(
            _cross(offset, edge),
            denominator,
            out=np.zeros_like(denominator),
            where=nonparallel,
        )
        u = np.divide(
            _cross(offset, motion),
            denominator,
            out=np.zeros_like(denominator),
            where=nonparallel,
        )
        crossing = (
            nonparallel
            & (t >= 0.0)
            & (t <= 1.0)
            & (u >= 0.0)
            & (u <= 1.0)
        )
        collided |= crossing | (d2 <= radius * radius)

    return collided


def _prepare_path(waypoints):
    points = np.asarray(waypoints, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("A route must contain two-dimensional waypoints.")
    if len(points) > 1:
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(points, axis=0), axis=1) > 0.0)
        )
        points = points[keep]

    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    return points, delta, lengths, cumulative


def _pursuit_target(position, progress, path, lookahead):
    points, delta, lengths, cumulative = path
    if len(lengths) == 0:
        return np.broadcast_to(points[-1], position.shape), progress

    # Project the estimated position onto the polyline, then advance by the
    # lookahead distance. Progress is monotone along the selected route.
    differences = position[:, None, :] - points[None, :-1, :]
    fraction = np.sum(differences * delta[None, :, :], axis=2)
    fraction /= lengths[None, :] ** 2
    fraction = np.clip(fraction, 0.0, 1.0)
    projection = points[None, :-1, :] + fraction[:, :, None] * delta
    distance2 = np.sum(
        (position[:, None, :] - projection) ** 2, axis=2
    )
    indices = np.argmin(distance2, axis=1)
    rows = np.arange(len(position))
    projected_progress = (
        cumulative[indices] + fraction[rows, indices] * lengths[indices]
    )
    progress = np.maximum(progress, projected_progress)
    target_progress = np.minimum(progress + lookahead, cumulative[-1])
    segment = np.searchsorted(cumulative, target_progress, side="right") - 1
    segment = np.clip(segment, 0, len(lengths) - 1)
    alpha = (target_progress - cumulative[segment]) / lengths[segment]
    target = points[segment] + alpha[:, None] * delta[segment]
    return target, progress


def _assess_route(spec, waypoints, samples=4096):
    """Approximate collision-free TRUE arrival using closed-loop rollouts.

    In this observation-free scenario, the GTSAM factor-chain solution is
    exactly the prior mean composed with all measured relative poses. The
    vectorized rollouts use that same estimator mean, not perfect localization.

    Numerical integration and pure-pursuit implementation details can differ
    from the verifier; this is a model-based probability estimate.
    """
    # Local RNG: repeated calls are deterministic, with no persistent state.
    # The same seed for each route supplies common random numbers.
    rng = np.random.default_rng(47129)

    dt = float(spec["dt"])
    steps = int(spec["max_steps"])
    controller = spec["controller"]
    speed = float(controller["v"])
    omega_limit = float(controller["omega_max"])
    lookahead = float(controller["lookahead"])
    radius = float(spec["robot_radius"])
    tolerance = float(spec["arrival_tolerance"])
    walls = np.asarray(spec["walls"], dtype=float).reshape(-1, 4)

    initial = spec["initial_pose"]
    mean = np.asarray(initial["mean"], dtype=float)
    initial_std = np.sqrt(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    odom_std = np.asarray(spec["sensor_noise_std"]["odom"], dtype=float)
    actuator_v = float(spec["actuator_noise_std"]["v"])
    actuator_omega = float(spec["actuator_noise_std"]["omega"])

    path = _prepare_path(waypoints)
    goal = path[0][-1]
    length = float(path[3][-1])
    if dt <= 0.0 or steps < 1:
        return 0.0, length, 0.0

    estimate = np.tile(mean, (samples, 1))
    # initial_truth_rule specifies a Gaussian in the supplied pose coordinates.
    truth = mean + rng.normal(size=(samples, 3)) * initial_std
    truth[:, 2] = _wrap(truth[:, 2])
    progress = np.zeros(samples)

    alive = ~_collisions(truth[:, :2], truth[:, :2], walls, radius)
    arrived = alive & (
        np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
    )
    alive &= ~arrived

    for _ in range(1, steps):
        if not np.any(alive):
            break

        target, progress = _pursuit_target(
            estimate[:, :2], progress, path, lookahead
        )
        displacement = target - estimate[:, :2]
        distance = np.linalg.norm(displacement, axis=1)
        alpha = _wrap(
            np.arctan2(displacement[:, 1], displacement[:, 0])
            - estimate[:, 2]
        )
        # Geometric pure pursuit: curvature = 2*sin(alpha)/target distance.
        omega = 2.0 * speed * np.sin(alpha) / np.maximum(distance, 1e-12)
        omega = np.clip(omega, -omega_limit, omega_limit)

        v_true = speed + actuator_v * rng.normal(size=samples)
        omega_true = omega + actuator_omega * rng.normal(size=samples)
        turn = omega_true * dt

        # Exact constant-twist integration, including the zero-turn limit.
        travel = v_true * dt * np.sinc(turn / (2.0 * np.pi))
        relative = np.column_stack(
            (
                travel * np.cos(0.5 * turn),
                travel * np.sin(0.5 * turn),
                turn,
            )
        )
        next_truth = _compose(truth, relative)

        # Right-local Pose2 perturbation of the measured relative pose.
        # Noise standard deviations are already SI, independent of input_units.
        noise = rng.normal(size=(samples, 3)) * odom_std
        measured_relative = _compose(relative, noise)
        estimate = _compose(estimate, measured_relative)

        hit = _collisions(
            truth[:, :2], next_truth[:, :2], walls, radius
        )
        alive &= ~hit
        truth = next_truth

        reached = alive & (
            np.linalg.norm(truth[:, :2] - goal, axis=1) <= tolerance
        )
        arrived |= reached
        alive &= ~reached

    probability = float(np.mean(arrived))
    sampling_error = math.sqrt(
        probability * (1.0 - probability) / samples
    )
    return probability, length, sampling_error


def choose_route(public_spec):
    if public_spec["gps_schedule"] is not None or public_spec["landmarks"]:
        raise ValueError("Route assessment is for the odometry-only scenario.")

    assessments = {
        name: _assess_route(public_spec, waypoints)
        for name, waypoints in public_spec["routes"].items()
    }
    best = max(assessments, key=lambda name: assessments[name][0])

    # Treat differences below Monte Carlo resolution as close ties, then use
    # the specified distance tie-breaker.
    candidates = []
    for name, (probability, length, error) in assessments.items():
        best_probability, _, best_error = assessments[best]
        resolution = 2.0 * math.hypot(error, best_error)
        if best_probability - probability <= resolution:
            candidates.append(name)
    selected = min(
        candidates,
        key=lambda name: (assessments[name][1], -assessments[name][0], name),
    )
    probability, length, error = assessments[selected]
    comparison = ", ".join(
        f"{name}: {result[0]:.3f}" for name, result in sorted(assessments.items())
    )

    return {
        "route": selected,
        "success_prob": probability,
        "reasoning": (
            f"Choose {selected} (polyline length {length:.2f} m). "
            f"Deterministic closed-loop rollout estimates are {comparison}. "
            "Rollouts drive pure pursuit using the same odometry-composition "
            "mean as the GTSAM estimator, sample initial, actuator, and "
            "relative-pose measurement uncertainty, and test swept-disc "
            "wall collisions and true arrival within the step budget. "
            "In particular, avoiding walls is not counted as success unless "
            "the true robot reaches the goal: without absolute observations, "
            "endpoint drift remains significant even on the detour. "
            f"Monte Carlo standard error is approximately {error:.3f}; "
            "controller and simulator implementation differences introduce "
            "additional model uncertainty."
        ),
    }

```

## Verifier feedback on that solution
- Route: stated success probability 0.70 differs from the measured rate 0.505 (95% CI [0.436, 0.574]) by more than 0.15.

Fix the failed checks. Return the complete revised solution.py.