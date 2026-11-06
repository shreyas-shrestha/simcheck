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
"""GTSAM dead-reckoning estimator and uncertainty-aware route selection."""

import copy
import math

import gtsam
import numpy as np
from scipy.special import ndtr


def make_estimator(public_spec):
    return Estimator(public_spec)


class Estimator:
    def __init__(self, public_spec):
        self.spec = copy.deepcopy(public_spec)

        initial = self.spec["initial_pose"]
        self.pose = gtsam.Pose2(*map(float, initial["mean"]))
        variances = np.asarray(initial["cov_diag_variances"], dtype=float)
        odom_sigmas = np.asarray(
            self.spec["sensor_noise_std"]["odom"], dtype=float
        )

        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(odom_sigmas)
        self.isam = gtsam.ISAM2()

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(
            gtsam.PriorFactorPose2(
                0,
                self.pose,
                gtsam.noiseModel.Diagonal.Variances(variances),
            )
        )
        values.insert(0, self.pose)
        self.isam.update(graph, values)

        self.last_step = -1
        self.covariance = np.diag(variances)

        # Only measured translation is converted. Noise is already in SI.
        unit = self.spec["input_units"]["odom"].lower()
        if unit == "m":
            self.translation_scale = 1.0
        elif unit == "cm":
            self.translation_scale = 0.01
        elif unit == "mm":
            self.translation_scale = 0.001
        else:
            raise ValueError("Unsupported odometry translation unit: " + unit)

        if self.spec["odom_frame"] != "body":
            raise ValueError("This estimator requires body-frame odometry.")

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        if k != self.last_step + 1:
            raise ValueError("Measurements must be supplied once, in step order.")

        # This scenario has no absolute observations. Do not silently discard
        # observations if a different scenario is accidentally supplied.
        if new_measurements.get("gps") is not None:
            raise ValueError("GPS is not part of this scenario.")
        if new_measurements.get("landmarks"):
            raise ValueError("Landmarks are not part of this scenario.")

        if k == 0:
            if new_measurements.get("odom") is not None:
                raise ValueError("Step zero must not contain odometry.")
        else:
            odom = new_measurements["odom"]
            if odom is None:
                raise ValueError("An odometry transition is required.")

            relative_pose = gtsam.Pose2(
                float(odom["dx"]) * self.translation_scale,
                float(odom["dy"]) * self.translation_scale,
                float(odom["dtheta"]),
            )
            prediction = self.pose.compose(relative_pose)

            graph = gtsam.NonlinearFactorGraph()
            values = gtsam.Values()
            graph.add(
                gtsam.BetweenFactorPose2(
                    k - 1, k, relative_pose, self.odom_noise
                )
            )
            values.insert(k, prediction)
            self.isam.update(graph, values)

        self.pose = self.isam.calculateEstimatePose2(k)
        covariance = np.asarray(self.isam.marginalCovariance(k), dtype=float)
        self.covariance = 0.5 * (covariance + covariance.T)
        self.last_step = k

        # marginalCovariance is already in this Pose2's local tangent frame.
        return (
            [float(self.pose.x()), float(self.pose.y()), float(self.pose.theta())],
            self.covariance.tolist(),
        )


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _nearest_on_segment(point, a, b):
    direction = b - a
    squared_length = float(direction @ direction)
    if squared_length == 0.0:
        return a
    fraction = float((point - a) @ direction) / squared_length
    return a + np.clip(fraction, 0.0, 1.0) * direction


def _route_assessment(spec, waypoints):
    """Approximate route risk, not a calibrated closed-loop success model.

    Propagate the same local-tangent uncertainty as the odometry factor chain.
    Use Gaussian separating-plane bounds at nominal travel samples. Summing
    the bounds is deliberately conservative about temporal correlation.
    """
    points = np.asarray(waypoints, dtype=float)
    dt = float(spec["dt"])
    speed = float(spec["controller"]["v"])
    omega_max = float(spec["controller"]["omega_max"])
    available_time = (int(spec["max_steps"]) - 1) * dt
    radius = float(spec["robot_radius"])

    initial = spec["initial_pose"]
    pose = gtsam.Pose2(*map(float, initial["mean"]))
    covariance = np.diag(
        np.asarray(initial["cov_diag_variances"], dtype=float)
    )
    noise_covariance = np.diag(
        np.square(np.asarray(spec["sensor_noise_std"]["odom"], dtype=float))
    )
    walls = [
        (np.asarray(w[:2], dtype=float), np.asarray(w[2:], dtype=float))
        for w in spec["walls"]
    ]

    if speed <= 0.0 or dt <= 0.0:
        return 0.0, float("inf"), float("inf")

    length = 0.0
    turn_time = 0.0
    risk_sum = 0.0

    def collision_bound(current_pose, local_covariance):
        position = np.array([current_pose.x(), current_pose.y()])
        theta = current_pose.theta()
        c, s = math.cos(theta), math.sin(theta)
        rotation = np.array([[c, -s], [s, c]])
        world_covariance = (
            rotation @ local_covariance[:2, :2] @ rotation.T
        )
        total = 0.0
        for a, b in walls:
            closest = _nearest_on_segment(position, a, b)
            displacement = position - closest
            distance = float(np.linalg.norm(displacement))
            clearance = distance - radius

            if clearance <= 0.0:
                total += 1.0
                continue

            normal = displacement / distance
            variance = max(float(normal @ world_covariance @ normal), 0.0)
            if variance > 0.0:
                total += float(ndtr(-clearance / math.sqrt(variance)))
        return total

    risk_sum += collision_bound(pose, covariance)

    # Travel from the specified initial mean through the supplied waypoints.
    for target in points:
        start = np.array([pose.x(), pose.y()])
        displacement = target - start
        distance = float(np.linalg.norm(displacement))
        if distance <= np.finfo(float).eps:
            continue

        heading = math.atan2(displacement[1], displacement[0])
        turn = abs(_wrap(heading - pose.theta()))
        if turn > 0.0:
            if omega_max <= 0.0:
                return 0.0, float("inf"), float("inf")
            duration = turn / omega_max
            turn_time += duration
        else:
            duration = 0.0

        # A nominal heading change transports the existing local covariance.
        rotated_pose = gtsam.Pose2(pose.x(), pose.y(), heading)
        relative = pose.between(rotated_pose)
        adjoint = np.asarray(relative.inverse().AdjointMap())
        covariance = (
            adjoint @ covariance @ adjoint.T
            + (duration / dt) * noise_covariance
        )
        pose = rotated_pose

        count = max(1, int(math.ceil(distance / (speed * dt))))
        increment = distance / count
        transition = gtsam.Pose2(increment, 0.0, 0.0)
        adjoint = np.asarray(transition.inverse().AdjointMap())
        noise_fraction = increment / (speed * dt)

        for _ in range(count):
            covariance = (
                adjoint @ covariance @ adjoint.T
                + noise_fraction * noise_covariance
            )
            pose = pose.compose(transition)
            risk_sum += collision_bound(pose, covariance)

        length += distance

    # Turning in place overestimates nominal time for a moving pure-pursuit
    # controller, providing a useful feasibility screen.
    travel_time = length / speed + turn_time
    probability = float(np.clip(1.0 - risk_sum, 0.0, 1.0))
    if travel_time > available_time:
        probability = 0.0
    return probability, length, travel_time


def choose_route(public_spec):
    assessments = {
        name: _route_assessment(public_spec, waypoints)
        for name, waypoints in public_spec["routes"].items()
    }
    # Favor estimated safety; use distance to resolve equal safety scores.
    selected = max(
        assessments,
        key=lambda name: (
            assessments[name][0],
            -assessments[name][1],
            name,
        ),
    )
    probability, length, travel_time = assessments[selected]

    return {
        "route": selected,
        "success_prob": float(probability),
        "reasoning": (
            f"Choose {selected}: nominal length {length:.2f} m and "
            f"conservative travel-time estimate {travel_time:.1f} s. "
            "Without GPS or landmarks, this estimator integrates odometry "
            "and cannot correct heading or lateral drift. The detour avoids "
            "the narrow wall corridor, whose clearance is small compared "
            "with accumulated positional uncertainty. The reported probability "
            "is a nominal-path Gaussian clearance approximation using the "
            "estimator's tangent covariance, not a calibrated closed-loop "
            "Monte Carlo probability; pure-pursuit corner cutting, between-"
            "sample collisions, and true endpoint error are not modeled."
        ),
    }

```

## Verifier feedback on that solution
- Route: stated success probability 1.00 differs from the measured rate 0.505 (95% CI [0.436, 0.574]) by more than 0.15.

Fix the failed checks. Return the complete revised solution.py.