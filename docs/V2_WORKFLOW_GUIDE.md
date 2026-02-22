# V2 Sensor Fusion & Tracking — Workflow Guide

## What This System Does

You have a **radar tracking simulation** that:
1. Places **sensors** (radars, IR, sonar, etc.) on platforms (towers, aircraft, ships)
2. Flies **targets** (aircraft with different flight behaviors) through the sensor coverage
3. Generates **detections** (what the sensors see, including noise and false alarms)
4. Runs **trackers** (GNN, TOMHT, JPDA) with different motion models (CV, IMM) to see which combination tracks targets best
5. Compares results with **metrics** (track accuracy, swaps, breaks)

---

## Project Structure

```
V2/
├── config/
│   ├── default.json                 ← Base configuration (tracker params, output settings)
│   ├── sensors.json                 ← Sensor catalog (enable/disable sensors here)
│   └── scenarios/
│       └── scenario_catalog.json    ← All scenario definitions (targets, weather, overrides)
│
├── src/
│   ├── trackingWithWeather.m        ← ORIGINAL main driver (still works, uses old flow)
│   └── helpers/
│       ├── loadScenario.m           ← NEW: Loads scenario + sensors + targets from catalog
│       ├── loadScenarioCatalog.m    ← NEW: Lists all available scenarios
│       ├── loadSensors.m            ← NEW: Builds sensors from sensors.json
│       ├── buildSensor.m            ← NEW: Universal sensor factory (24 types)
│       ├── load_config.m            ← Loads and merges JSON configs
│       ├── createScenario3D.m       ← ORIGINAL scenario builder (still works)
│       ├── buildCustomFusionRadarSensor.m  ← Original PSR builder
│       ├── buildIFFSensor.m         ← Original SSR/MSSR builder
│       ├── runDetections.m          ← Runs scenario, generates detection log
│       ├── buildTracker.m           ← Creates tracker objects from config
│       ├── helperRunTracker.m       ← Runs tracker against detections, collects metrics
│       ├── initCVFilter.m           ← Constant Velocity filter
│       ├── initIMMFilter.m          ← Interacting Multiple Model filter
│       └── analyzeTrackSwaps.m      ← Track swap detection
│
├── cache/                           ← Saved detection logs (.mat files)
├── results/                         ← Saved run results (.mat files)
│
├── testBuildSensor.m                ← Test: all 24 sensor types build correctly
├── testLoadSensors.m                ← Test: sensor catalog loads, groups by platform
└── testLoadScenario.m               ← Test: all 9 scenarios load and run
```

---

## Quick Start — 3 Commands

Open MATLAB, `cd` to the V2 folder, then:

```matlab
% 1. See what scenarios are available
loadScenarioCatalog
```
This prints:
```
  dasr_ideal          | 2t |  50s | IDEAL    | DASR baseline, clear weather
  dasr_degraded       | 2t |  50s | DEGRADED | DASR in rain
  crossing_targets    | 2t |  60s | IDEAL    | Two targets crossing paths
  head_on             | 2t |  40s | IDEAL    | Head-on approach
  high_density        | 5t |  60s | IDEAL    | 5 targets, stress test
  maneuvering_evasive | 1t |  60s | IDEAL    | S-maneuver, tests IMM vs CV
  storm_window        | 2t |  60s | DEGRADED | Heavy rain, max degradation
  long_range_arsr     | 3t | 120s | IDEAL    | ARSR at 250nm
  approach_pattern    | 2t |  90s | IDEAL    | Landing approach
```

```matlab
% 2. Load a scenario (builds sensors + targets + config automatically)
[scenario, config, sensors, metas] = loadScenario("crossing_targets");
```

```matlab
% 3. Generate detections and run trackers
dataLog = runDetections(scenario, config.degradation.enabled);
tracker = buildTracker('GNN', 'IMM', config.active_params, ...
    config.tracker_global, config.filter_params, ...
    config.active_params.pd, numel(sensors.tower));
[trackSummary, truthSummary] = helperRunTracker(dataLog, tracker);
```

That's it. You have tracks.

---

## How To: Choose Your Sensors

Open **`config/sensors.json`** in any text editor. You'll see a list of sensors. Each one has `"enabled": true` or `"enabled": false`.

### Default Setup (DASR = PSR + SSR)
```json
{ "name": "DASR Primary Search Radar",  "type": "PSR", "enabled": true  },
{ "name": "DASR Secondary Surveillance Radar", "type": "SSR", "enabled": true  },
{ "name": "Airport Surveillance Radar", "type": "ASR", "enabled": false },
...
```

### To add a sensor: change `false` → `true`
### To remove a sensor: change `true` → `false`
### To tweak a sensor: edit the `"params"` block

Example — make the PSR spin faster and see farther:
```json
{
  "name": "DASR Primary Search Radar",
  "type": "PSR",
  "enabled": true,
  "params": {
    "rpm": 25,
    "rangeLimits": [0, 200000]
  }
}
```

### Available Sensor Types
| Type | What It Is | Range |
|------|-----------|-------|
| PSR | Primary Search Radar | 60 nm |
| SSR | Secondary Surveillance (IFF) | 120 nm |
| ASR | Airport Surveillance | 60 nm |
| ARSR | Air Route Surveillance | 250 nm |
| PAR | Precision Approach | 20 nm |
| TWS | Track-While-Scan Phased Array | 200 km |
| AESA | Active Electronic Scan Array | 300 km |
| FIRE_CONTROL | Fire Control Radar | 150 km |
| WEATHER | Weather Radar (NEXRAD) | 250 nm |
| MARITIME | Maritime Surface Search | 40 nm |
| IRST | IR Search & Track | 100 km |
| IR_STARING | Staring IR Sensor | 50 km |
| FLIR | Forward-Looking Infrared | 30 km |
| ACTIVE_SONAR | Active Sonar | 20 km |
| PASSIVE_SONAR | Passive Sonar | 50 km |
| TOWED_ARRAY | Towed Array Sonar | 80 km |
| LIDAR | Lidar Point Cloud | 200 m |
| ADSB_TX | ADS-B Transponder | — |
| ADSB_RX | ADS-B Receiver | — |

---

## How To: Choose Your Scenario

Open **`config/scenarios/scenario_catalog.json`**. Each scenario defines:
- **targets** — how many, what behavior, where they start, how fast
- **overrides** — duration, degradation, tracker tuning
- **sensor_config** — which sensor JSON to use (default: `"sensors"`)

### Pre-Built Scenarios (18 total)

**Airport / ATC Radar (PSR + SSR)**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `dasr_ideal` | PSR+SSR | Baseline — normal operations, 2 turning targets |
| `dasr_degraded` | PSR+SSR | Rain — tracker robustness with reduced Pd |
| `crossing_targets` | PSR+SSR | Track swap — two targets crossing paths |
| `head_on` | PSR+SSR | Identity — targets approaching each other |
| `high_density` | PSR+SSR | Stress test — 5 targets, mixed behaviors |
| `maneuvering_evasive` | PSR+SSR | Filter comparison — S-maneuver, IMM vs CV |
| `storm_window` | PSR+SSR | Max degradation — heavy rain, Pd=0.5 |
| `approach_pattern` | PSR+SSR | Close tracking — two aircraft on approach |

**Specialized Radar**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `long_range_arsr` | ARSR+SSR | En-route surveillance — 250nm, slow scan |
| `par_approach` | PSR+SSR+PAR | Precision approach — narrow beam final tracking |
| `phased_array_intercept` | TWS+AESA | Phased array fusion — ground+airborne arrays |
| `fire_control_engagement` | PSR+Fire Control | Search-to-track handoff |

**Infrared Fusion**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `ir_radar_fusion` | PSR+IRST+Staring IR | Passive+active complementarity |
| `ir_degraded_weather` | PSR+IRST+Staring IR | Rain — radar degrades, IR doesn't |

**Maritime / Sonar**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `maritime_surface` | Maritime+Active/Passive Sonar+Towed Array | Surface+subsurface tracking |

**Airborne Fighter**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `fighter_intercept` | AESA+FLIR | Fighter tracking bogey at range |

**Multi-Sensor Layered Defense**
| Scenario | Sensors | What It Tests |
|----------|---------|---------------|
| `layered_defense` | PSR+SSR+IRST+Fire Control+TWS | 5-sensor fusion, 4 targets |
| `layered_defense_storm` | PSR+SSR+IRST+Fire Control+TWS | Same in heavy rain — which sensors degrade? |

### Target Behaviors
| Behavior | Description |
|----------|------------|
| `constant_velocity` | Straight line, fixed speed |
| `gentle_turn` | Gradual heading changes |
| `s_maneuver` | S-shaped evasive turns |
| `crossing` | Straight line from start_pos to end_pos |
| `orbit` | Circular holding pattern |
| `approach` | Descending straight-in approach |
| `departure` | Outbound climb |

---

## How To: Add a New Scenario

Add a new entry inside `"scenarios"` in `scenario_catalog.json`:

```json
"my_custom_scenario": {
  "description": "My custom test — 3 targets, one maneuvering",
  "sensor_config": "sensors",
  "overrides": {
    "scenario.duration_s": 80,
    "scenario.num_targets": 3,
    "degradation.enabled": false,
    "data_logging.use_saved_datalog": false,
    "data_logging.datalog_file": "cache/my_custom.mat"
  },
  "targets": [
    {
      "behavior": "constant_velocity",
      "speed_kmh": 900,
      "start_pos": [-5000, -20000, -3000],
      "heading_deg": 90,
      "altitude_m": 3000
    },
    {
      "behavior": "s_maneuver",
      "speed_kmh": 800,
      "start_pos": [3000, -22000, -4000],
      "turn_rate_dps": 3,
      "altitude_m": 4000
    },
    {
      "behavior": "orbit",
      "speed_kmh": 600,
      "start_pos": [0, -18000, -3500],
      "orbit_radius_m": 4000,
      "altitude_m": 3500
    }
  ]
}
```

Then run it:
```matlab
[scenario, config, sensors, metas] = loadScenario("my_custom_scenario");
```

---

## How To: Create a Custom Sensor Config

Copy `config/sensors.json` → `config/sensors_maritime.json`, edit it to enable maritime/sonar sensors, then reference it in your scenario:

```json
"sensor_config": "sensors_maritime"
```

---

## How To: Run the Full Tracker Comparison (Original Flow)

The original driver still works and runs ALL enabled trackers:

```matlab
trackingWithWeather("default_reRunDetections")
```

This uses `config/default_reRunDetections.json` which controls:
- Which trackers to run (`gnn_cv`, `gnn_imm`, `tomht_cv`, etc.)
- Whether to use cached detections or generate fresh ones
- Ideal vs degraded mode
- Visualization on/off

---

## How To: Run Step-by-Step (Full Control)

```matlab
%% Setup
addpath(genpath(fullfile(pwd, 'src', 'helpers')));
addpath(genpath(fullfile(pwd, 'src', 'visualization')));

%% 1. Load scenario
[scenario, config, sensors, metas] = loadScenario("crossing_targets");

%% 2. Generate detections
dataLog = runDetections(scenario, config.degradation.enabled);

%% 3. Visualize the scene
plotInitialScenario(dataLog, true);  % true = animate

%% 4. Build a tracker
params     = config.active_params;
numSensors = numel(sensors.tower);  % or count across all platforms

tracker = buildTracker('GNN', 'IMM', params, ...
    config.tracker_global, config.filter_params, ...
    params.pd, numSensors);

%% 5. Run tracker
[trackSummary, truthSummary, trackMetrics, truthMetrics, ...
    time, assignLog, swapReport] = helperRunTracker(dataLog, tracker, false, true, true);

%% 6. View results
disp(trackSummary);
disp(truthSummary);

if swapReport.swapFree
    fprintf('No track swaps detected.\n');
else
    fprintf('%d swaps detected.\n', swapReport.totalSwaps);
end
```

---

## How To: Compare Multiple Trackers on Same Scenario

```matlab
[scenario, config, sensors, ~] = loadScenario("maneuvering_evasive");
dataLog = runDetections(scenario, config.degradation.enabled);

params     = config.active_params;
numSensors = numel(sensors.tower);

combos = {
    'GNN',   'CV';
    'GNN',   'IMM';
    'TOMHT', 'IMM';
    'JPDA',  'IMM';
};

for i = 1:size(combos, 1)
    fprintf('\n=== %s + %s ===\n', combos{i,1}, combos{i,2});
    tracker = buildTracker(combos{i,1}, combos{i,2}, params, ...
        config.tracker_global, config.filter_params, params.pd, numSensors);
    [trkSum, truthSum] = helperRunTracker(dataLog, tracker);
    disp(trkSum);
    disp(truthSum);
end
```

---

## How To: Save and Reload Detections

Generating detections is slow. Save them once, then rerun trackers instantly:

```matlab
% Generate and save
[scenario, config, sensors, ~] = loadScenario("high_density");
dataLog = runDetections(scenario, config.degradation.enabled);
save('cache/high_density.mat', 'dataLog', '-v7.3');

% Later — reload without regenerating
load('cache/high_density.mat', 'dataLog');
% Now run any tracker against the same detections
```

---

## How To: Run Tests

From the V2 root folder:

```matlab
testBuildSensor     % 28/28 — all sensor types build
testLoadSensors     % 6/6  — sensor catalog loads correctly
testLoadScenario    % 10/10 — all scenarios load and run
```

---

## Key Config File: default.json

This is the **base config** that every scenario starts from. Key sections:

| Section | What It Controls |
|---------|-----------------|
| `scenario` | Mode (3D), num_targets, duration |
| `degradation` | enabled, type (rain/heavy_rain) |
| `data_logging` | Use cached detections, save path |
| `tracker_global` | max_tracks, volume, beta, Pd (ideal + degraded) |
| `tracker_params.ideal` | Gate size, FAR, confirm/delete thresholds (clear weather) |
| `tracker_params.degraded` | Same but tuned for rain (wider gates, lower thresholds) |
| `filter_params` | Initial speed, IMM transition prob, accel/turn scaling |
| `trackers_to_run` | Which tracker combos to enable |
| `output` | Diagnostics, visuals, animation, save results |

Scenario overrides in `scenario_catalog.json` override any of these fields using dot-notation:
```json
"tracker_params.degraded.gate": 100
```

---

## Cheat Sheet

| I want to... | Do this |
|--------------|---------|
| See available scenarios | `loadScenarioCatalog` |
| Load a scenario | `[scen,cfg,sens,m] = loadScenario("name")` |
| List available sensors | Open `config/sensors.json` |
| Enable/disable a sensor | Change `"enabled": true/false` in sensors.json |
| Add a new scenario | Add entry to `scenario_catalog.json` |
| Run full comparison | `trackingWithWeather("default_reRunDetections")` |
| Generate detections | `dataLog = runDetections(scenario, degraded)` |
| Build a tracker | `tracker = buildTracker(type, model, params, global, filter, pd, nSens)` |
| Run a tracker | `[trkS,trS] = helperRunTracker(dataLog, tracker)` |
| Run tests | `testBuildSensor`, `testLoadSensors`, `testLoadScenario` |
