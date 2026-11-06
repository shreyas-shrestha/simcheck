## First-attempt pass rate (headline)

| Version | Model | Task | Passed | Rate | Wilson 95% | Mean reward | Failed layers |
|---|---|---|---|---|---|---|---|
| v3.1 | argon-medium | prior_needed | 2/5 | 0.40 | [0.12, 0.77] | 0.82 | route 3 |
| v3.1 | argon-medium | standard | 3/5 | 0.60 | [0.23, 0.88] | 0.60 | execution 2 |
| v3.1 | argon-medium | trap_frame | 2/5 | 0.40 | [0.12, 0.77] | 0.40 | execution 3 |
| v3.1 | argon-medium | trap_units | 2/4 | 0.50 | [0.15, 0.85] | 0.50 | execution 2 |
| v3.1 | astra-medium | prior_needed | 0/1 | 0.00 | [0.00, 0.79] | 0.70 | route 1 |
| v3.1 | astra-medium | standard | 3/3 | 1.00 | [0.44, 1.00] | 1.00 | - |
| v3.1 | astra-medium | trap_frame | 1/1 | 1.00 | [0.21, 1.00] | 1.00 | - |
| v3.1 | astra-medium | trap_units | 2/2 | 1.00 | [0.34, 1.00] | 1.00 | - |
| v3.2 | argon-medium | close_routes | 3/3 | 1.00 | [0.44, 1.00] | 1.00 | - |
| v3.2 | argon-medium | unknown_noise | 0/3 | 0.00 | [0.00, 0.56] | 0.40 | execution 1, calibration 2 |
| v3.2 | astra-medium | close_routes | 1/1 | 1.00 | [0.21, 1.00] | 1.00 | - |

## Closed loop (secondary)

| Model | Task | Attempts | Reward per attempt | Fresh reward | Copied feedback numbers |
|---|---|---|---|---|---|
| argon-medium | prior_needed | 1 | 1.00 | 1.00 | - |
| argon-medium | standard | 2 | 0.00 → 1.00 | 1.00 | - |
| argon-medium | trap_frame | 2 | 0.00 → 1.00 | 1.00 | - |
| argon-medium | trap_units | 2 | 0.00 → 1.00 | 1.00 | - |
| astra | prior_needed | 3 | 0.70 → 0.70 → 1.00 | 1.00 | 0.505, 0.70 |
| astra | standard | 1 | 1.00 | 1.00 | - |
| astra | trap_frame | 1 | 1.00 | 1.00 | - |
| astra-medium | prior_needed | 3 | 0.70 → 0.70 → 1.00 | 1.00 | - |
| astra-medium | standard | 1 | 1.00 | 1.00 | - |
| astra-medium | trap_frame | 1 | 1.00 | 1.00 | - |
| astra-medium | trap_units | 1 | 1.00 | 1.00 | - |

## v3.2 unknown_noise calibration

| Model | Time | Runs | coverage95 | Mean NEES |
|---|---|---|---|---|
| argon-medium | end_of_dropout | 2 | 0.800 | 5.93 |
| argon-medium | end_of_trajectory | 2 | 0.583 | 10.96 |

## v3.2 close_routes stated vs measured

| Model | Chosen | Stated | Measured [95%] | Pass |
|---|---|---|---|---|
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |
| argon-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |
| astra-medium | B | 1.00 | 1.000 [0.981, 1.000] | yes |

## Spend

| Model | USD |
|---|---|
| argon-medium | 3.00 |
| astra | 1.43 |
| astra-medium | 4.25 |
