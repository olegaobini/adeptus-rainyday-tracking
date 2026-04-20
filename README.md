# Rainy Day: Advanced Radar Tracking in Degraded Weather

**Version 3.5.0** — Boeing-sponsored senior capstone project, University of Washington.

A modular MATLAB framework for evaluating radar target tracking performance under
real-world degraded conditions. Users compose custom scenarios by combining
independent JSON configuration files for sensors, targets, terrain, trackers,
and weather — no MATLAB code required.

## Quick Start

```matlab
% cd to the adeptus-rainyday-tracking folder, then:
addpath("scripts"); addpath(genpath("src"));

% First time only — creates config folder structure:
buildModularConfig   % (only if config/ is missing)

% Run a simulation:
runSingleScenario("dasr_baseline")       % PSR+SSR, rural, GNN+JPDA
runSingleScenario("demo_mountain")       % PSR, 5 targets, mountain terrain
runSingleScenario("fighter_intercept")   % AESA+FLIR on aircraft
runSingleScenario("dasr_storm")          % PSR+SSR, mountain, heavy rain
runSingleScenario("my_run")              % your custom run file

% NASA real flight data (new in v3.4.0):
runNASAFlight                  % Track real Tail-687 flight with simulated radar
viewNASAFlightGlobe            % Earth-centered 3D view of real flights
scanNASAFlights                % Profile all flight files in a folder

% Automated parameter sweep (new in v3.4.0):
autoTuneTracker("my_run", "GNN")   % Two-pass GNN tuning on cached detections

% Validation suite (new in v3.3.0):
runTestPlan                    % All 9 test cases, 27 assertions
verifySimulation               % 40+ diagnostic checks across 8 phases

% Demo video (1080p MP4):
recordDemoVideo

% Interactive path editor (new in v3.5.0):
pathEditor                     % click-and-drag target path drawing → JSON
```

## How It Works

Each simulation is defined by a **run file** (`config/runs/*.json`) that
assembles five independent components:

```
config/runs/my_run.json
  ├── sensors  → config/sensors/PSR/default_PSR.json
  │              config/sensors/SSR/default_SSR.json
  ├── targets  → config/targets/crossing_pair/default_crossing_pair.json
  ├── terrain  → config/terrain/mountain/default_mountain.json
  ├── trackers → config/trackers/GNN/default_GNN.json
  │              config/trackers/JPDA/default_JPDA.json
  └── weather  → config/weather/rain/default_rain.json   (new in v3.4.0)
```

`runSingleScenario` reads the run file, loads each component, builds the MATLAB
`trackingScenario`, generates detections through the environment pipeline, runs
each specified tracker, and reports metrics with 3D visualization.

### Run file format (v3.4.2)

```json
{
  "description": "My experiment",
  "sensors": ["PSR/default_PSR", "SSR/default_SSR"],
  "targets": "crossing_pair/default_crossing_pair",
  "terrain": "mountain/default_mountain",
  "trackers": ["GNN/default_GNN", "JPDA/default_JPDA"],
  "degradation": {
    "terrain_occlusion": true,
    "horizon_masking":   true,
    "ground_clutter":    true,
    "doppler_fade":      true,
    "rcs_range_filter":  false,
    "weather":           "rain/default_rain"
  },
  "cache":    { "use_cached_detections": false, "save_detections": true },
  "platforms": {},
  "output":   { "show_visuals": true, "animate_visuals": true, "save_results": true }
}
```

Legacy `"degradation": { "enabled": true, "type": "rain" }` format still works.
The `weather` key supersedes inline rain params — see `config/weather/`.

See `config/runs/run_template.json` for the fully documented version.

## Building a Custom Simulation

### 1. Pick or create sensors
Each sensor type has its own folder under `config/sensors/<TYPE>/` with three
files: `default_<TYPE>.json` (works out of box), `<TYPE>_template.json`
(documented), `my_<TYPE>.json` (your copy).

```
config/sensors/PSR/my_PSR.json     — edit rpm, range, Pd, mounting
config/sensors/AESA/my_AESA.json   — edit sector, range (needs aircraft platform)
```

### 2. Pick or create targets
Target folders define flight behaviors and duration:

```
config/targets/crossing_pair/my_crossing_pair.json  — two targets crossing
config/targets/s_maneuver/my_s_maneuver.json        — evasive S-turn
config/targets/recorded_flight/my_recorded.json     — real NASA flight data (v3.4.0)
```

### 3. Pick terrain
Terrain folders define environment effects:

```
config/terrain/mountain/my_mountain.json  — ridges, occlusion ON, clutter ON
config/terrain/water/default_water.json   — flat ocean, all effects OFF
```

### 4. Pick weather (new in v3.4.0)
Weather configs live under `config/weather/<TYPE>/`. Four types: `rain`, `snow`,
`fog`, `icing`. Each has a storm-window profile (step / ramp / pulse) that
controls when degradation fires during the scenario.

```
config/weather/rain/default_rain.json      — 16 mm/hr moderate
config/weather/rain/heavy_rain.json        — 40 mm/hr
config/weather/rain/tropicalStorm.json     — 60+ mm/hr
config/weather/snow/default_snow.json      — equivalent precip rate
config/weather/fog/default_fog.json        — visibility-based
config/weather/icing/default_icing.json    — antenna hardware degradation
config/weather/validation_rain.json        — short storm window for validation tests
```

### 5. Pick trackers
Tracker folders define algorithm + tuning. Shared parameters (Pd, filter init)
live in `tracker_globals.json`.

```
config/trackers/GNN/my_GNN.json     — gate, volume, beta, thresholds
config/trackers/JPDA/my_JPDA.json   — JPDA-specific tuning
config/trackers/TOMHT/my_TOMHT.json — multi-hypothesis tuning
```

### 6. Create a run file
Copy `config/runs/run_template.json` → `config/runs/my_experiment.json`, fill
in references to the components above.

### 7. Run it
```matlab
runSingleScenario("my_experiment")
```

### Detection caching
After the first run, set `"use_cached_detections": true` in the run file. This
skips detection generation and goes straight to the tracker — much faster when
tuning gates, thresholds, or comparing algorithms. The auto-tuner uses this
exclusively.

## Project Structure

```
adeptus-rainyday-tracking/
├── config/
│   ├── runs/                        ← Run files (entry points)
│   │   ├── run_template.json        ← Documented template (copy + edit)
│   │   ├── dasr_baseline.json       ← PSR+SSR, rural, GNN+JPDA
│   │   ├── demo_first_run.json      ← Boeing demo
│   │   ├── fighter_intercept.json   ← Airborne AESA+FLIR
│   │   ├── recorded_flight/         ← NASA flight scenarios (v3.4.0)
│   │   ├── validation/              ← TC-01..TC-09 test run files (v3.3.0)
│   │   └── showcase/                ← Pre-built scenarios
│   ├── sensors/                     ← Per-type sensor folders (19 types)
│   │   ├── PSR/                     ← default_PSR, sband_PSR, xband_PSR,
│   │   │                              tc05_PSR_longrange (v3.4.2),
│   │   │                              dead_zone_*, my_PSR, template
│   │   ├── SSR/  AESA/  PAR/  TWS/  FIRE_CONTROL/  ARSR/
│   │   ├── IRST/ FLIR/   MARITIME/  WEDGE/
│   │   └── ... (19 sensor types total)
│   ├── targets/                     ← Per-behavior target folders
│   │   ├── crossing_pair/           ← Two targets crossing
│   │   ├── gentle_turn/             ← Gradual heading change
│   │   ├── crossing_5way/           ← 5 targets (Boeing demo)
│   │   ├── s_maneuver/              ← Evasive S-turn
│   │   ├── orbit/                   ← Circular holding pattern
│   │   ├── approach/ head_on/ high_density/ dead_zone/
│   │   ├── rcs_demo/                ← 20 dBsm vs -10 dBsm (TC-05)
│   │   └── recorded_flight/         ← Real NASA flight data (v3.4.0)
│   ├── terrain/                     ← water / rural / urban / mountain / desert
│   ├── trackers/                    ← GNN / JPDA / TOMHT + tracker_globals
│   └── weather/                     ← NEW in v3.4.0
│       ├── rain/   (default, heavy, tropicalStorm, validation, nasa)
│       ├── snow/   (default, nasa)
│       ├── fog/    (default)
│       ├── icing/  (default)
│       └── weather_template.json
│
├── src/+trackbench/
│   ├── +config/
│   │   └── loadRunFile.m            ← Modular run-file loader
│   ├── +detections/
│   │   ├── runDetections.m          ← Detection generator (main pipeline)
│   │   ├── createDetections.m
│   │   └── getWeather.m             ← Storm window severity (v3.4.0)
│   ├── +scenario/
│   │   ├── addTargetFromDef.m       ← Universal target builder from JSON
│   │   └── validateScanCoverage.m
│   ├── +tracking/
│   │   ├── buildTracker.m           ← GNN/JPDA/TOMHT factory
│   │   ├── initCVFilter.m           ← Constant Velocity filter
│   │   ├── initIMMFilter.m          ← IMM filter
│   │   └── runTracker.m             ← Run + metrics + 3D plots
│   ├── +sensors/
│   │   ├── buildSensor.m            ← Universal sensor factory (19 types)
│   │   └── buildIFFSensor.m
│   ├── +environment/
│   │   ├── isAboveHorizon.m         ← 4/3 Earth horizon masking
│   │   ├── generateGroundClutter.m  ← Frequency-dependent clutter (v3.4.1)
│   │   ├── generateTerrain.m        ← Procedural heightmap generator
│   │   ├── applyDopplerFade.m       ← MTI clutter notch (v3.2.0, MDV fix v3.4.1)
│   │   ├── applyWeatherDegradation.m← 4-type weather (v3.4.0)
│   │   ├── applyRainDegradation.m   ← ITU-R P.838-3 via rainpl() (v3.2.0)
│   │   ├── applyRCSFilter.m         ← Probabilistic R⁴ Pd scaling (opt-in, v3.4.2)
│   │   └── buildRCSProfile.m        ← Aspect-dependent RCS patterns (v3.2.0)
│   ├── +reporting/
│   │   ├── plotInitialScenario.m    ← 3D animated truth + detections + terrain
│   │   ├── plotPlatformToTrackAssignment.m
│   │   ├── plotTrackSwapAnalysis.m
│   │   ├── drawSensorCoverage.m     ← Range rings + sector wedges
│   │   ├── drawBeamEnvelope.m       ← Exact scan-limit cones (v3.4.1)
│   │   └── tabbedAxes.m
│   ├── +flightdata/
│   │   └── loadNASAFlight.m         ← FDR telemetry → NED waypoints (v3.4.0)
│   ├── +validation/
│   │   └── validateScenarioConfig.m ← 10 pre-flight checks
│   ├── +analysis/
│   │   └── analyzeTrackSwaps.m
│   └── +editor/                     ← Interactive path editor (v3.5.0)
│       ├── EditorState.m            ← Mutable state (waypoints, undo/redo)
│       ├── buildUI.m                ← uifigure + side panel + toolbar
│       ├── drawMap.m                ← 2D/3D axes, grid, waypoint markers
│       ├── exportToJSON.m           ← Writes config/targets/waypoints/<n>.json
│       ├── loadFromJSON.m           ← Re-opens an exported path for editing
│       ├── previewWindow.m          ← Animated preview of the drawn flight
│       ├── computeScaleBar.m        ← Zoom-aware scale bar
│       ├── interpPos.m              ← Spline / linear interpolation
│       └── zoomAroundPoint.m        ← Scroll-wheel zoom target
│
├── scripts/
│   ├── runSingleScenario.m          ← Main entry point
│   ├── runTestPlan.m                ← Validation suite (v3.3.0)
│   ├── verifySimulation.m           ← 40+ diagnostic checks (v3.3.0)
│   ├── autoTuneTracker.m            ← 2-pass parameter sweep (v3.4.0)
│   ├── runNASAFlight.m              ← Real flight demo (v3.4.0)
│   ├── runNASAFlightGlobe.m         ← Earth-centered view (v3.4.0)
│   ├── scanNASAFlights.m            ← Flight data profiler (v3.4.0)
│   ├── viewNASAFlightGlobe.m        ← Globe viewer
│   ├── compareTrackers.m
│   ├── compareAllTrackers.m
│   ├── runComparisonDemo.m
│   ├── recordDemoVideo.m            ← 1080p MP4 export
│   ├── buildModularConfig.m         ← One-time setup
│   ├── runAllPresentationPrep.m
│   ├── diagBeamLimits.m             ← Beam envelope diagnostic (v3.4.1)
│   ├── diagnoseBadDetections.m      ← Detection debugging (v3.4.1)
│   ├── diagVerifyTargetIndex.m      ← TargetIndex audit (v3.4.1)
│   ├── pathEditor.m                 ← Interactive editor launcher (v3.5.0)
│   ├── testPathEditor_M1.m          ← Editor M1 (click+export) tests
│   ├── testPathEditor_M2.m          ← Editor M2 (select/drag/undo) tests
│   ├── testPathEditor_M3.m          ← Editor M3 (2D/3D, grid, altitude) tests
│   ├── testPathEditor_M3_preview.m  ← Preview-window tests
│   ├── testPathEditor_shortcuts.m   ← Keyboard shortcut tests
│   └── cleanup_showcase.m
│
├── _cleanup_backup/                 ← Archived legacy / removed modules
├── tests/
│   └── testBuildSensor.m
│
├── docs/                            ← Design spec, physics audit, etc.
├── Cheatsheet.txt
├── README.md                        ← This file
├── cache/                           ← Saved detection logs (gitignored)
└── results/                         ← Saved run results (gitignored)
```

## Available Sensor Types (19)

| Type | What It Is | Range | Platform |
|------|-----------|-------|----------|
| PSR | Primary Search Radar | 60 nm | tower |
| SSR | Secondary Surveillance (IFF) | 120 nm | tower |
| ASR | Airport Surveillance | 60 nm | tower |
| ARSR | Air Route Surveillance | 250 nm | tower |
| PAR | Precision Approach | 20 nm | tower |
| TWS | Track-While-Scan Phased Array | 200 km | tower |
| AESA | Active Electronic Scan Array | 300 km | aircraft |
| FIRE_CONTROL | Fire Control Radar | 150 km | tower |
| WEATHER | Weather Radar (NEXRAD) | 250 nm | tower |
| MARITIME | Maritime Surface Search | 40 nm | ship |
| IRST | IR Search & Track | 100 km | tower |
| IR_STARING | Staring IR Sensor | 50 km | tower |
| FLIR | Forward-Looking Infrared | 30 km | aircraft |
| ACTIVE_SONAR | Active Sonar | 20 km | ship |
| PASSIVE_SONAR | Passive Sonar | 50 km | ship |
| TOWED_ARRAY | Towed Array Sonar | 80 km | ship |
| LIDAR | Lidar Point Cloud | 200 m | tower |
| ADSB_TX | ADS-B Transponder | — | aircraft |
| ADSB_RX | ADS-B Receiver | — | tower |

## Target Behaviors

| Behavior | Description | Extra Fields |
|----------|-------------|-------------|
| `constant_velocity` | Straight line, fixed heading | `heading_deg` |
| `gentle_turn` | Gradual heading change | — |
| `s_maneuver` | Evasive S-shaped path | `turn_rate_dps` (2–5 typical) |
| `crossing` | Straight line start→end | `end_pos` |
| `orbit` | Circular holding pattern | `orbit_radius_m` (3000–5000) |
| `approach` | Descending to runway | `end_pos` (near threshold) |
| `departure` | Climbing away from airport | `end_pos` |
| `waypoints` | Fully user-defined 3D path (v3.2.0) | `waypoints: [{pos, time_s}, …]` |
| `recorded_flight` | NASA FDR telemetry (v3.4.0) | `source_file`, `ref_lat`, `ref_lon` |
| `head_on` / `parallel` | Aliased to constant_velocity | `heading_deg` |

All targets require: `speed_kmh`, `start_pos` [x,y,z NED], `altitude_m`.

Optional physical properties (v3.2.0+):

- `rcs_dbsm` — isotropic RCS in dBsm
- `rcs_profile` — named preset (`stealth`, `fighter`, `airliner`, `drone`,
  `missile`) for aspect-dependent RCS pattern
- `dimensions` — `{length_m, width_m, height_m}` cuboid model
- `class_id` — classification for SSR/IFF

## Tracker Algorithms

| Tracker | Strength | Best For |
|---------|----------|----------|
| GNN | Fast, deterministic | Sparse, low-clutter |
| JPDA | Probabilistic, soft assignment | Medium clutter, real-time |
| TOMHT | Multiple hypotheses, optimal | Dense scenarios, highest accuracy |

| Motion Model | Assumption | Use Case |
|-------------|-----------|----------|
| CV | Straight line | Fast targets, baseline |
| IMM | Switches between CV + Coordinated Turn | Maneuvering aircraft |

### Tracker globals vs per-tracker params

- **`tracker_globals.json`** — shared: max tracks, detection probability
  (ideal/degraded), filter init (speed, IMM transition prob, process noise)
- **Per-tracker JSON** — algorithm-specific: volume, beta, gate sizes,
  confirm/delete thresholds, branch limits

Volume and beta live per-tracker because GNN and JPDA use them with opposite
effects.

### Auto-Tune Tracker (v3.4.0)

`autoTuneTracker("run_name", "TRACKER")` sweeps parameters in two passes:

1. **Pass 1 — Tracker params**: gate, volume, beta, confirm/delete thresholds
   (~60–100 combinations, smart grid varying 2 params at a time + corners).
2. **Pass 2 — Filter params**: using best tracker params from Pass 1, sweeps
   IMM process noise (accel H/V, omega dot), transition probability, init
   speed. Centered on baseline with 0.5x–4x variations.

Composite score: `0.50·posRMS/maxRange + 0.25·swapCount +
0.15·log(1+falseTracks) + 0.10·breakCount`.

Uses cached detections exclusively (~0.1–0.5s per iteration). Saves best
config to `config/trackers/<TYPE>/autotuned_<TYPE>_<run>.json`.

## Environment Modeling

Five independent layers of physically-motivated effects plus a weather
degradation system. Each layer is toggled separately in the run file's
`degradation` block.

### 1. Horizon Masking
4/3 effective Earth radius model. Targets below radar horizon are invisible.
*Ref: Skolnik Ch. 2, MATLAB `horizonrange()`.*

### 2. Ground Clutter (frequency-dependent as of v3.4.1)
Two-component model: surface clutter (Poisson-distributed, inverse-square
density) + discrete clutter from terrain features. Surface clutter scales as
**freq²** (Rayleigh); discrete clutter as **freq^0.8**.

| Terrain | Returns/scan | Noise (m) |
|---------|-------------|-----------|
| water | ~1–2 | 50 |
| rural | ~3–6 | 100 |
| urban | ~8–15 | 150 |
| mountain | ~5–10 | 200 |

*Ref: Skolnik Ch. 7, Nathanson 2nd ed.*

### 3. Terrain Occlusion
Procedural heightmaps (200×200 grid) via MATLAB `groundSurface` API.
`SurfaceManager.occlusion()` performs LOS checks between every sensor-target
pair at every timestep.

### 4. Doppler/MTI Fade (v3.2.0, MDV fix in v3.4.1)
Radial-velocity check per detection: `v_radial = dot(v_target, unit_radar_to_target)`.
If `|v_radial| < MDV`, Pd scales linearly from 0 to 1 with a 5% sidelobe floor.
MDV is auto-computed from radar frequency: **MDV = λ × PRF / 4** (S-band 2.8 GHz
→ MDV ≈ 27 m/s, X-band 9 GHz → MDV ≈ 8 m/s).
*Ref: Skolnik Ch. 3 (MTI and Pulse Doppler).*

### 5. RCS Signatures (v3.2.0, native sensor path + opt-in filter v3.4.2)
`fusionRadarSensor` natively reads `platform.Signatures` via the radar equation
with `ReferenceRange` / `ReferenceRCS`. Five preset profiles in
`buildRCSProfile.m`: stealth, fighter, airliner, drone, missile.

For validation scenarios that need a strong, observable RCS-vs-range
differential (which the sensor-native Swerling model doesn't expose at typical
scan counts), `applyRCSFilter` offers an **opt-in** probabilistic filter:
`Pd_scale = max(0.05, min(1, (σ/σ_ref)·(R_ref/R)⁴))`. Enable per-run with
`"rcs_range_filter": true` in the degradation block. TC-05 uses it; user demos
leave it OFF.
*Ref: Knott/Shaeffer/Tuley, Radar Cross Section.*

### 6. Weather Degradation (new in v3.4.0)
Four weather types in `config/weather/<TYPE>/`, each with distinct physics:

- **Rain** — MATLAB `rainpl()` (ITU-R P.838-3). Three effects: range-dependent
  Pd reduction (two-way), measurement noise inflation (wet radome), and
  Poisson-distributed weather clutter in the FOV scaling as freq².
- **Snow** — `rainpl()` at 25% of equivalent precipitation rate (Gunn & East
  1954). Lower RF attenuation than rain, reduced volume clutter.
- **Fog** — visibility-based, primarily affects IR/optical. Negligible RF
  effect below 10 GHz (uses `fogpl()` above X-band). ITU-R P.840 / Koschmieder.
- **Icing** — antenna-hardware gain loss (2–6 dB), **not** a path effect.
  Range-independent. Models ice on the radome/antenna.

Each weather config carries a **storm window** with three profile types:
- `step` — binary on/off at start/end times
- `ramp` — linear 0→1 over first half of window, then 1→0
- `pulse` — sharp 20% burst then clear

### Terrain Presets

| terrain_type | Occlusion | Masking | Clutter | Density | Max Elevation |
|-------------|-----------|---------|---------|---------|--------------|
| `water` | OFF | OFF | OFF | 0 | 0m |
| `rural` | ON | ON | ON | 0.3 | ~73m |
| `urban` | ON | ON | ON | 0.6 | ~150m |
| `mountain` | ON | ON | ON | 0.5 | ~1960m |
| `desert` | ON | ON | ON | 0.2 | ~40m |

### Beam Envelope Visualization (v3.4.1)
`drawBeamEnvelope.m` queries `coverageConfig()` for exact scan limits and
renders green (included) / red (excluded) cones for any radar or IR sensor.
Automatically called by `plotInitialScenario`. Provides immediate visual
feedback on where the sensor can and cannot see.

## Real Flight Data (v3.4.0)

`loadNASAFlight.m` ingests NASA DASHlink Flight Data Recorder (FDR) telemetry
(ARINC 717 decoded parameters: LATP, LONP, ALT, GS, TRK at 1–16 Hz), extracts
the airborne portion, converts lat/lon to NED waypoints relative to a
reference point, and resamples to uniform spacing. The target behavior
`"recorded_flight"` feeds these waypoints to `waypointTrajectory`, making a
real flight a scenario target.

Multiple flights can be loaded simultaneously for multi-target scenarios, each
with different RCS values. `viewNASAFlightGlobe` provides an Earth-centered 3D
view of real flights against simulated radar sites.

The framework bridges simulation and reality: the radar and trackers are
simulated (allowing controlled experiments), but the flight path is real
(stressing trackers with actual turbulence, pilot corrections, and ATC
maneuvers that synthetic trajectories cannot reproduce).

## Interactive Path Editor (v3.5.0)

`pathEditor` launches a uifigure-based map editor for drawing custom target
flight paths by clicking on a 2D or 3D view. Exports directly to the
`waypoints` target behavior that `addTargetFromDef.m` already consumes
(added in v3.2.0), so drawn paths plug into any run file without code changes.

```matlab
clear classes; clear all                         % namespace cache reset
addpath("scripts"); addpath(genpath("src"))
pathEditor                                       % launch
```

**Drawing**
- **Left-click** on map → append waypoint at default altitude + default speed
- **Drag** a waypoint → move it (undo granular per drag, not per pixel)
- **Shift+click** on a segment → insert a new waypoint at the click point
- **Delete / Backspace** → remove selected waypoint
- **Ctrl+Z / Ctrl+Y** → undo / redo (50 levels)
- **V** → toggle 2D ↔ 3D view
- **Middle-click + drag** → pan (2D)
- **Scroll wheel** → zoom around cursor

**Sidebar controls**
- Target name, description, RCS (dBsm + preset), default altitude and speed
- Per-waypoint edit of x / y / altitude / leg speed / time
- Color-by-altitude toggle, grid spacing, editable radar marker position
- Spline preview, animation preview window, load-from-JSON to re-edit

**Export**
Writes `config/targets/waypoints/<target_name>.json` with schema:

```json
{
  "description": "...",
  "duration_s": 780,
  "targets": [{
    "name": "my_path", "label": "my_path",
    "behavior": "waypoints",
    "rcs_dbsm": 10, "rcs_profile": "airliner",
    "waypoints": [
      { "pos": [0, 0, -3000], "time_s": 0, "speed_kmh": 900 },
      { "pos": [5000, 12000, -3500], "time_s": 70, "speed_kmh": 900 },
      ...
    ]
  }]
}
```

Reference the exported file from a run file:

```json
{ "targets": "waypoints/my_path", ... }
```

The editor is **fully portable** — resolves its project root from
`mfilename('fullpath')` so it works from any clone location without
configuration.

## Validation & Testing

`scripts/runTestPlan.m` executes 9 test cases containing 27 assertions.
Current status (v3.4.2): **27/27 passing**.

| TC | Name | What It Validates |
|----|------|-------------------|
| 01 | Template Usability | User-created config loads and runs end-to-end |
| 02 | Baseline Clear | All 3 trackers produce valid metrics |
| 03 | Rain S-band | S-band tracking holds under 16 mm/hr rain |
| 04 | Rain X-band | X-band fewer target dets than S-band at same rate |
| 05 | RCS Verification | 20 dBsm vs -10 dBsm detection ratio at 100 km |
| 06 | Crossing Swap | JPDA swap count ≤ GNN swap count |
| 07 | Compound Stress | TOMHT + rain + mountain + mixed RCS completes |
| 08 | Error Paths | 5 malformed configs produce clear error messages |
| 09 | Verification Suite | `verifySimulation.m` (40+ checks) completes |

Run files for the test cases live in `config/runs/validation/` to keep the
main runs folder clean for user experiments. `loadRunFile` supports
subdirectory paths natively. Results are saved to
`results/test_plan_results_<timestamp>.mat`.

`verifySimulation.m` is a standalone 40+ check diagnostic covering: toolbox
availability, file system, JSON integrity, sensor build, detection
generation, tracker build, visualization, and cleanup.

## Data Flow

```
runSingleScenario("my_run")
  └── trackbench.config.loadRunFile("my_run")
      ├── Read config/runs/my_run.json
      ├── Load each config/sensors/<TYPE>/<file>.json
      │   └── trackbench.sensors.buildSensor (per sensor)
      ├── Load config/targets/<PATTERN>/<file>.json
      │   └── trackbench.scenario.addTargetFromDef (per target)
      │       └── trackbench.flightdata.loadNASAFlight (if recorded_flight)
      ├── Load config/terrain/<TYPE>/<file>.json
      │   └── trackbench.environment.generateTerrain
      ├── Load config/weather/<TYPE>/<file>.json  (if weather set)
      ├── Load config/trackers/<TYPE>/<file>.json (per tracker)
      ├── trackbench.scenario.validateScanCoverage
      └── trackbench.validation.validateScenarioConfig
            ↓
  trackbench.detections.runDetections
      ├── Radar / IR sensor step loop
      ├── trackbench.environment.isAboveHorizon       (per target)
      ├── SurfaceManager.occlusion                    (terrain LOS)
      ├── trackbench.environment.applyRCSFilter       (opt-in v3.4.2)
      ├── trackbench.environment.applyDopplerFade     (per detection)
      ├── trackbench.environment.applyWeatherDegradation
      │   └── trackbench.detections.getWeather        (storm severity)
      └── trackbench.environment.generateGroundClutter (per scan)
            ↓
  For each enabled tracker:
    trackbench.tracking.buildTracker
      ├── trackbench.tracking.initCVFilter
      └── trackbench.tracking.initIMMFilter
    trackbench.tracking.runTracker
      ├── trackbench.analysis.analyzeTrackSwaps
      ├── trackbench.reporting.tabbedAxes
      ├── trackbench.reporting.drawSensorCoverage
      ├── trackbench.reporting.drawBeamEnvelope       (v3.4.1)
      ├── trackbench.reporting.plotPlatformToTrackAssignment
      └── trackbench.reporting.plotTrackSwapAnalysis
            ↓
  Save results to results/ and detections to cache/
```

## Known Issues

- **Sonar sensors**: Maritime scenarios build sonar sensors but `runDetections`
  skips them (sonar uses `sonarEmission` interface). Only maritime radar
  generates detections.
- **Terrain visibility at small scale**: Rural terrain (73m peaks) is
  physically present but visually flat at aircraft altitude. Use mountain
  terrain to see the mesh.
- **MATLAB namespace caching**: `+trackbench` package functions are
  aggressively cached. After editing any `.m` file, run `clear classes`
  then `clear all` — `clc` and `close all` alone are insufficient.

## Required Toolboxes

- **Sensor Fusion and Tracking Toolbox** (R2024a+) — trackers, sensors,
  theaterPlot, metrics
- **Radar Toolbox** — `horizonrange`, `rainpl`, `fogpl`, `landroughness`,
  `earthSurfacePermittivity`, `refractiveidx`
- **Mapping Toolbox** — `groundSurface`, `SurfaceManager` (terrain occlusion)
- **Phased Array System Toolbox** *(optional fallback)* — alternate path for
  `rainpl` if Radar Toolbox isn't present

Tested on MATLAB **R2025b**.

## Team

Team Adeptus — Boeing-sponsored senior capstone project, University of
Washington.
Daniel Trofimchik · James Gallegos · Kaz Foster · Michael Harding · Olega Obini

## License

Boeing Proprietary.

---

## Change Log

### v3.5.0 — April 17, 2026 (current)

**Interactive Path Editor.** Merged the `InteractiveSexyMapTry` branch onto
the v3.4.2 base to produce a single presentation-ready build.

- **New package `src/+trackbench/+editor/`** (9 files, ~117 KB) — `EditorState`,
  `buildUI`, `drawMap`, `exportToJSON`, `loadFromJSON`, `previewWindow`,
  `computeScaleBar`, `interpPos`, `zoomAroundPoint`.
- **New script `scripts/pathEditor.m`** — one-call launcher. Resolves project
  root from `mfilename('fullpath')` so the editor runs unchanged on any
  clone of the repo.
- **Click-and-drag waypoint drawing** in 2D or 3D, with altitude colormap,
  zoom-aware hit-testing, grid overlay, editable radar marker, spline
  preview, and an animated preview window.
- **50-level undo/redo** with per-drag granularity (not per mouse-move).
- **JSON round-trip**: exports to `config/targets/waypoints/<name>.json`
  matching the schema `addTargetFromDef.m` already consumed (the
  `case "waypoints"` branch added in v3.2.0 required no changes).
- **Editor test suite** (`testPathEditor_M1.m` … `_M3.m`, `_shortcuts.m`,
  `_M3_preview.m`) plus milestone docs `TESTING_M{1,2,3}.md` carried over.
- **No changes** to the core simulation pipeline, validation suite, or any
  existing run/target/sensor/tracker/weather configs. All v3.4.2 test
  results (27/27) remain valid.
- **No hardcoded paths** anywhere in the editor package or merge tooling —
  verified by grep against `C:\`, `Admin`, branch folder names, and
  `Documents\` patterns.

### v3.4.2 — April 16, 2026

**Validation pass fixes** — restored 27/27 test plan pass rate after
regressions from the v3.4.1 session.

- **`applyRCSFilter.m` restored** as an **opt-in probabilistic** filter in
  `src/+trackbench/+environment/`. Applies `Pd_scale = max(0.05, min(1,
  (σ/σ_ref)·(R_ref/R)⁴))` per detection. Default OFF to preserve sensor-native
  Swerling behavior for user demos; TC-05 enables it via
  `"rcs_range_filter": true` in its degradation block.
- **`runDetections.m`** reads the new `envConfig.rcs_range_filter` flag and
  calls `applyRCSFilter` when ON. Diagnostics line added.
- **`loadRunFile.m`** propagates `rcs_range_filter` from the run file's
  `degradation` block into `config.environment`.
- **`sband_PSR.json`** — symmetric S-band counterpart to `xband_PSR.json`
  (2.8 GHz, 111 km rangeLimits). TC-03/TC-04 now compare S-band vs X-band with
  identical geometry — the only variable is `CenterFrequency`, isolating
  ITU-R P.838-3 attenuation cleanly.
- **`validation_rain.json`** — rain config with storm window (5–50s) aligned
  to the 50s gentle_turn scenario. The `default_rain` storm window (50–130s)
  never fired during the validation scenarios, making TC-03/TC-04 effectively
  clear-sky. TC-03 and TC-04 now reference this file.
- **TC-05 run file**: disables `doppler_fade` (targets fly tangentially and
  the fade would confound the RCS test) and turns on `rcs_range_filter`.

**Result:** 27/27 tests pass. TC-05 airliner=24, stealth=7, ratio=3.43x.
TC-04 X-band=6, S-band=14 target dets at matched 16 mm/hr rain rate.

### v3.4.1 — April 15, 2026

**Physics & geometry fixes, auto-tuner, beam envelope visualization.**

- **`buildSensor.m` elevation limits** fixed. Old formula
  `[-(fov+2), 2]-tilt` pushed the PSR beam below the horizon. New formula
  `[-fov/2, fov/2]-tilt` centers the beam on the horizon. PSR now covers
  −17° to +13° elevation.
- **Frequency-dependent ground clutter** in `generateGroundClutter.m`:
  surface clutter scales as freq² (Rayleigh), discrete clutter as freq^0.8
  relative to S-band baseline.
- **Doppler fade MDV** corrected to use `λ × PRF / 4` (was hardcoded to 40 m/s).
  S-band MDV ≈ 27 m/s, X-band ≈ 8 m/s now computed from each sensor's frequency.
- **RCS range filter removed** (*later restored as opt-in in v3.4.2*) on the
  assumption that `fusionRadarSensor` handled RCS natively via
  `platform.Signatures`. Empirically it doesn't produce observable differentials
  at typical scan counts — see v3.4.2.
- **VCP propagation model removed** from the detection pipeline.
- **TargetIndex semantics** clarified: uses `PlatformID` (tower=1, targets
  start at 2), **not** array index. False alarms use `-1` not `0`.
- **`drawBeamEnvelope.m`** renders exact green/red scan-limit cones for any
  radar or IR sensor by querying `coverageConfig()`. Automatically called by
  `plotInitialScenario`.
- **`autoTuneTracker.m`** — two-pass parameter sweep (tracker params then
  filter params) with composite scoring. Saves optimal config to
  `config/trackers/<TYPE>/autotuned_<TYPE>_<run>.json`.
- **Diagnostic scripts** added: `diagBadDetections`, `diagVerifyTargetIndex`,
  `diagBeamLimits` in `scripts/`.

### v3.4.0 — April 13, 2026

**NASA real flight data + 4-type weather system.**

- **NASA flight data integration**: `loadNASAFlight.m`, `runNASAFlight`,
  `scanNASAFlights`, `viewNASAFlightGlobe`. Target behavior `recorded_flight`
  feeds FDR waypoints to `waypointTrajectory`.
- **Weather system**: `config/weather/<TYPE>/` folder. Four types: **rain**
  (ITU-R P.838-3 via `rainpl`), **snow** (Gunn & East 25% equivalent), **fog**
  (visibility-based, IR/optical), **icing** (antenna hardware gain loss).
- **Storm windows**: each weather config specifies `storm_start_s`,
  `storm_end_s`, and `active_type` (step/ramp/pulse). `getWeather.m` computes
  severity `w ∈ [0,1]` at each timestep.
- **`applyWeatherDegradation.m`** unified the four types behind one interface
  (replaces type-specific functions in the per-detection path).
- **Consolidated degradation block**: run files now use a single `degradation`
  block with terrain toggles + `"weather": "rain/default_rain"` reference.
  Legacy `"degradation": { "enabled": true, "type": "rain" }` still works.

### v3.3.0 — March 22, 2026

**Automated validation test plan.**

- `scripts/runTestPlan.m` — 9 test cases, 27 assertions, pass/fail reporting.
- `scripts/verifySimulation.m` — 40+ standalone diagnostic checks across 8
  phases.
- Test run files moved to `config/runs/validation/` to keep the main folder
  clean for user experiments. `loadRunFile` supports subdirectory paths
  natively.
- RCS demo geometry fix: targets moved from 30 km to 100 km range to expose
  the R⁴ dependence across the 30 dB RCS gap.
- IEEE 829-compliant test plan document (v0.8) in `docs/`.

### v3.2.0 — March 21, 2026

**Rain attenuation via MATLAB `rainpl()` + aspect-dependent RCS + Doppler fade.**

- Replaced hand-coded ITU coefficient table with MATLAB's official
  `rainpl()` (ITU-R P.838-3). Fallback to Phased Array Toolbox or internal
  table if Radar Toolbox unavailable.
- **`buildRCSProfile.m`** creates full azimuth×elevation `rcsSignature`
  pattern matrices. Five preset profiles: stealth, fighter, airliner, drone,
  missile. `fusionRadarSensor` natively interpolates at each scan.
- **`applyDopplerFade.m`** — tangential targets fall into the MTI clutter
  notch and lose detections. Per-detection radial-velocity check with 5%
  sidelobe floor.
- **Custom waypoint trajectories**: new `"behavior": "waypoints"` for
  user-defined 3D flight paths.

### v3.1.x — March 20, 2026

- Physics validation audit (`docs/physics_validation_audit.md`)
- Sandbox run templates
- Auto-terrain environment resolution

### v3.0.0 — March 15–19, 2026

**Modular Config Architecture** — complete restructure from monolithic
`default.json` to individual component files.

- Run files (`config/runs/`) assemble sensors + targets + terrain + trackers
  by reference
- Per-sensor type folders with default/template/user configs
- Per-terrain type folders with environment flag presets
- Per-tracker algorithm folders with algorithm-specific tuning
- Per-behavior target folders
- Detection caching per run file for fast tracker tuning
- `tracker_globals.json` separates shared params from per-tracker volume/beta
- Legacy catalog system removed (copies preserved in `_cleanup_backup/`)

### v2.x — February 23 – March 15, 2026

- v2.6: auto-terrain resolution, pre-flight scenario validator
- v2.5: demo video recording (`recordDemoVideo.m`)
- v2.4: one-command entrypoint, sector scanner fix, terrain occlusion toggle
- v2.3: terrain occlusion, 3D visualization fixes, sensor coverage viz
- v2.2: propagation model (VCP), environment modeling, ground clutter
- v2.1: JSON-driven scenario loading, 19 sensor types, batch runner
- v2.0: +trackbench namespace, JSON config, DASR sensor model

---

**Last Updated:** April 17, 2026
**Version:** 3.5.0
