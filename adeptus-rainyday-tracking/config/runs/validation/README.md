# Validation Test Run Files

These run files are used by `scripts/runTestPlan.m` to execute the automated validation test plan (v0.7). They are **not** user experiments — do not edit or delete them unless updating the test plan itself.

## Usage

```matlab
addpath("scripts");
runTestPlan          % runs all 9 test cases (27 checks)
```

## Test Cases

| File | TC | What It Tests |
|------|----|---------------|
| `tc01_template_user.json` | TC-01 | Template usability — user can create and run a config |
| `tc02_baseline_clear.json` | TC-02 | All 3 trackers under ideal conditions |
| `tc03_rain_sband.json` | TC-03 | S-band (2.8 GHz) in 16 mm/hr rain |
| `tc04_rain_xband.json` | TC-04 | X-band (9 GHz) in 16 mm/hr rain (same rate as TC-03) |
| `tc05_rcs_demo.json` | TC-05 | RCS verification — 20 dBsm vs -10 dBsm at 100km |
| `tc06_crossing_swap.json` | TC-06 | GNN vs JPDA track swap on crossing targets |
| `tc07_compound_stress.json` | TC-07 | TOMHT + rain + mountain + mixed RCS |
| `tc08a_missing_sensor.json` | TC-08 | Error: nonexistent sensor file |
| `tc08b_missing_target.json` | TC-08 | Error: nonexistent target file |
| `tc08c_missing_terrain.json` | TC-08 | Error: nonexistent terrain file |
| `tc08d_malformed_json.json` | TC-08 | Error: invalid JSON syntax |
| `tc08e_missing_tracker.json` | TC-08 | Error: nonexistent tracker file |

TC-09 uses `scripts/verifySimulation.m` directly — no run file needed.
