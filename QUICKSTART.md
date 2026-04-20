# Rainy Day — Quickstart

All the commands you need, one page.
Full docs: `README.md` · Long-form reference: `Cheatsheet.txt`

---

## Setup (first time on your machine)

```matlab
cd("path\to\adeptus-rainyday-tracking")
```

## Every MATLAB session

```matlab
addpath("scripts"); addpath(genpath("src"))
```

**If you edit any `.m` file in `src/+trackbench/`** — run this or your changes won't take effect:
```matlab
clear classes; clear all; clc; close all
```

---

## Run a scenario

```matlab
runSingleScenario("my_run")                   % your custom run
runSingleScenario("dasr_baseline")            % PSR + SSR, clear weather
runSingleScenario("rain_demo_xband")          % X-band in heavy rain
runSingleScenario("demo_tuned_performance")   % Boeing demo run
runSingleScenario("compound_demo")            % all physics + rain
```

**To make your own run:** copy `config/runs/run_template.json` → `config/runs/my_run.json`, edit, save, run.

## NASA real-flight demos

```matlab
viewNASAFlightGlobe("nasa_multi_target")      % replay cached results on globe
scanNASAFlights                               % profile all .mat files in Tail_687_1/
```

---

## Compare trackers head-to-head

Run the same cached detections through several trackers and see which wins.

**Step 1 — cache detections (do this once per scenario):**
```matlab
runSingleScenario("my_run")                   % saves cache/my_run.mat
```

**Step 2 — in `config/runs/my_run.json`, add a `compare_trackers` list:**
```json
"compare_trackers": [
    "GNN/default_GNN",
    "JPDA/default_JPDA",
    "TOMHT/default_TOMHT",
    "autotuned/my_run/GNN_IMM"
]
```

**Step 3 — run the comparison (fast, no detection regen):**
```matlab
compareTrackers("my_run")                     % prefer this
compareAllTrackers("my_run")                  % legacy: auto-picks 6 combos
runComparisonDemo                             % Boeing clean-vs-rain side-by-side
```

Prints a ranked table sorted by composite score (lower = better).

---

## Auto-tune a tracker

Sweeps tracker + filter params against cached detections to find the best config. Fast (~0.1–0.5s per iteration since detection generation is skipped).

**Step 1 — cache detections first:**
```matlab
runSingleScenario("my_run")
```

**Step 2 — tune:**
```matlab
autoTuneTracker("my_run", "GNN")                    % GNN with IMM filter (default)
autoTuneTracker("my_run", "GNN", "CV")              % GNN with CV filter
autoTuneTracker("my_run", "JPDA", "IMM")            % JPDA with IMM
autoTuneTracker("my_run", "TOMHT", "CV")            % TOMHT with CV

% Advanced: also compare against the OTHER filter model (CV vs IMM)
autoTuneTracker("my_run", "GNN", "IMM", struct('compareModels', true))
```

Saves the best config to `config/trackers/autotuned/<run_name>/<TYPE>_<MODEL>.json`.
You can then reference that path in a run file or `compare_trackers` list.

---

## Path editor (draw custom flight paths)

Interactive UI — click on a map to place waypoints, export to a target JSON.

```matlab
pathEditor                                    % launches the UI
```

Left-click on the map = add waypoint. "Export JSON" writes to `config/targets/waypoints/<name>.json`.
Then reference it in a run file like any other target: `"targets": "waypoints/<name>"`.

---

## Tests & validation

```matlab
runTestPlan                                   % 27 automated checks across 9 test cases
verifySimulation                              % 40+ diagnostic checks across 8 phases
```

## Diagnostics (when something looks wrong)

```matlab
diagBeamLimits                                % check sensor beam/elevation coverage
diagnoseBadDetections                         % investigate missing/bad detections
diagVerifyTargetIndex                         % verify PlatformID → TargetIndex mapping
```

---

## Where stuff lives

```
config/runs/       the "recipe" — what sensors + targets + terrain + trackers + weather
config/sensors/    radars: PSR, SSR, AESA, FLIR, ... (19 types)
config/targets/    what the aircraft do: crossing_pair, orbit, s_maneuver, waypoints, recorded_flight
config/terrain/    water, rural, urban, mountain, desert
config/trackers/   GNN (fast) · JPDA (medium) · TOMHT (best accuracy)
                   tracker_globals.json     → shared params (Pd, max tracks, filter init)
                   autotuned/<run>/         → outputs from autoTuneTracker
config/weather/    rain, snow, fog, icing (each with storm window profile)

scripts/           entry-point MATLAB scripts (run these from command window)
src/+trackbench/   engine code (edit only if you know why)
cache/             saved detections per run (skip detection regen — gitignored)
results/           saved tracker outputs (gitignored)
```

Each sensor/target/terrain/tracker folder has three files:
- `default_*.json` — works out of the box, don't edit
- `*_template.json` — documented, read this to learn the params
- `my_*.json` — **your** copy, edit freely

---

## When something breaks

```matlab
clear classes; clear all; clc; close all     % reset MATLAB cache (fixes 90% of weirdness)
```

| Error | Fix |
|---|---|
| "Function not found" / "Undefined..." | You forgot `addpath("scripts"); addpath(genpath("src"))` |
| Red error about `+trackbench` | `clear classes` and retry |
| JSON error on run file | Missing one of: `sensors`, `targets`, `terrain`, `trackers` |
| "No cached detections at..." | Run `runSingleScenario("<run>")` first before `autoTuneTracker`/`compareTrackers` |
| NASA flight can't find .mat | `Tail_687_1/` must be a **sibling** of the project folder, not inside it |
| Changes to .m file do nothing | `clear classes` — MATLAB caches `+package` code aggressively |
