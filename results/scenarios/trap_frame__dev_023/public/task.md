# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

Conventions:
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

Scenario-specific notes:
- ODOMETRY FRAME: dx/dy are expressed in the odometry frame, i.e. the body frame at step 0 rotated by the running sum of the reported dtheta values up to (not including) this transition. dtheta is unaffected.
