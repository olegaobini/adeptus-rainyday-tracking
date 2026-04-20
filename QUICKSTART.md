# Rainy Day — Quickstart

**For teammates.** One page. All the commands you need.
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

That's it. Now you can run anything below.

---

## Run a scenario

```matlab
runSingleScenario("my_run")                   % your custom run
runSingleScenario("dasr_baseline")            % PSR + SSR, clear weather
runSingleScenario("rain_demo_xband")          % X-band in heavy rain
runSingleScenario("demo_tuned_performance")   % Boeing demo run
```

**To make your own run:** copy `config/runs/run_template.json` → `config/runs/my_run.json`, edit, save, run.

## NASA real-flight demos

```matlab
runNASAFlight                                 % real flight + simulated PSR
runNASAFlightGlobe                            % same, on a 3D Earth globe
viewNASAFlightGlobe("nasa_multi_target")      % replay cached results on globe
```

## Tests & tuning

```matlab
runTestPlan                                   % 27 automated checks
verifySimulation                              % 40+ diagnostic checks
autoTuneTracker("my_run", "GNN")              % auto-tune GNN params
```

---

## Where stuff lives

```
config/runs/       the "recipe" — what sensors + targets + terrain + trackers + weather
config/sensors/    radars: PSR, SSR, AESA, FLIR, ... (19 types)
config/targets/    what the aircraft do: crossing_pair, orbit, s_maneuver, ...
config/terrain/    water, rural, urban, mountain, desert
config/trackers/   GNN (fast) · JPDA (medium) · TOMHT (best accuracy)
config/weather/    rain, snow, fog, icing
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

**"Function not found"** → you forgot `addpath("scripts"); addpath(genpath("src"))`
**JSON error** → your run file is missing one of: `sensors`, `targets`, `terrain`, `trackers`
**Red error about `+trackbench`** → run `clear classes` and try again
**NASA flight can't find .mat file** → check `Tail_687_1/` folder is a **sibling** of the project folder, not inside it
