## With vs without SimCheck

| Version | Model | First try (no feedback) | After SimCheck feedback (fresh seeds) | Fixed / lost by feedback | McNemar exact p | Accuracy-only grader accepts | ...of which actually failing | Copied feedback numbers |
|---|---|---|---|---|---|---|---|---|
| v3.1 | argon-medium | 10/20 [0.30, 0.70] | 17/20 [0.64, 0.95] | 7 / 0 | 0.0156 | 13/20 | 3 | - |
| v3.2 | argon-medium | 3/5 [0.23, 0.88] | 4/5 [0.38, 0.96] | 1 / 0 | 1 | 4/5 | 1 | - |
| all | argon-medium | 13/25 [0.33, 0.70] | 21/25 [0.65, 0.94] | 8 / 0 | 0.00781 | 17/25 | 4 | - |

## First-attempt pass rate per task

| Version | Model | Task | Passed | Rate | Wilson 95% | Mean reward | Failed layers |
|---|---|---|---|---|---|---|---|
| v3.1 | argon-medium | prior_needed | 2/5 | 0.40 | [0.12, 0.77] | 0.82 | route 3 |
| v3.1 | argon-medium | standard | 3/5 | 0.60 | [0.23, 0.88] | 0.60 | execution 2 |
| v3.1 | argon-medium | trap_frame | 2/5 | 0.40 | [0.12, 0.77] | 0.40 | execution 3 |
| v3.1 | argon-medium | trap_units | 3/5 | 0.60 | [0.23, 0.88] | 0.60 | execution 2 |
| v3.2 | argon-medium | close_routes | 3/3 | 1.00 | [0.44, 1.00] | 1.00 | - |
| v3.2 | argon-medium | unknown_noise | 0/2 | 0.00 | [0.00, 0.66] | 0.30 | execution 1, calibration 1 |

## Closed loop (secondary)

| Model | Task | Attempts | Reward per attempt | Fresh reward | Copied feedback numbers |
|---|---|---|---|---|---|
| argon-medium | prior_needed | 1 | 1.00 | 1.00 | - |
| argon-medium | standard | 2 | 0.00 → 1.00 | 1.00 | - |
| argon-medium | trap_frame | 2 | 0.00 → 1.00 | 1.00 | - |
| argon-medium | trap_units | 2 | 0.00 → 1.00 | 1.00 | - |

## v3.2 unknown_noise calibration

| Model | Time | Runs | coverage95 | Mean NEES |
|---|---|---|---|---|
| argon-medium | end_of_dropout | 1 | 0.967 | 2.87 |
| argon-medium | end_of_trajectory | 1 | 0.800 | 3.92 |

## v3.2 close_routes stated vs measured

| Model | Chosen | Stated | Measured [95%] | Pass |
|---|---|---|---|---|
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |

## Spend

| Model | USD |
|---|---|
| argon-medium | 4.63 |
