# Rainy Day: Advanced Radar Tracking in Degraded Weather

  **Version 3.7.3** — Boeing-sponsored senior capstone project, University of Washington.

> **Just here to test the app?** See **`TESTING.md`** in the repo root. It's a one-page guide for installing the .exe and trying it out as an end user — no MATLAB required.

A modular MATLAB framework for evaluating radar target tracking performance under
real-world degraded conditions. Users compose custom scenarios by combining
independent JSON configuration files for sensors, targets, terrain, trackers,
and weather — no MATLAB code required.

> **Experimental build (this branch).** Beyond the production radar core, this build adds **sonar**, **infrared (IRST)**, and **moving-platform** sensing as proofs-of-concept. They run end-to-end but are **not validated** to the radar standard - treat them as capability demonstrations. See [Experimental Build](#experimental-build) below and the `Experimental Features` disclosure doc. Radar remains production-grade.

## Launching the App

Three ways to launch, in order of "least technical":

### 1. Pre-built EXE (recommended for end users)

Double-click **`trackbench.vbs`** (silent launch) or **`trackbench.bat`** (shows a
brief console window). Either one starts `mainMenu.exe` and opens a 3-button menu:
**Path Editor** / **Run Simulation** / **Validation & Documentation**.

> **First-time setup:** install the **MATLAB Compiler Runtime (MCR) for R2025b**.
> Free download: <https://www.mathworks.com/products/compiler/mcr/index.html>
> Without MCR the EXE won't run. The launcher .bat will fall back to MATLAB if
> MATLAB is installed but MCR is missing.

Optional: run `install_shortcut.vbs` once to drop a "Rainy Day Tracker" shortcut
on your Desktop pointing back at this folder.

### 2. From MATLAB directly

```matlab
% cd to this folder, then:
addpath("scripts"); addpath(genpath("src"));
mainMenu
```

Same 3-button menu. Use this when you want to also run scripts from the console.

### 3. Build the EXE yourself

If you have MATLAB Compiler:

```matlab
addpath("scripts"); addpath(genpath("src"));
build_executable
```

Output: `trackbench/mainMenu.exe` (also copied to project root). Compile time
3–7 min depending on installed toolboxes.

## Quick Start (MATLAB console)

```matlab
% cd to the adeptus-rainyday-tracking folder, then:
addpath("scripts"); addpath(genpath("src"));

% Run a simulation:
runSingleScenario("dasr_baseline")       % PSR+SSR, rural, GNN+JPDA
runSingleScenario("demo_mountain")       % PSR, 5 targets, mountain terrain
runSingleScenario("fighter_intercept")   % AESA+FLIR on aircraft
runSingleScenario("dasr_storm")          % PSR+SSR, mountain, heavy rain
runSingleScenario("my_run")              % your custom run file

% NASA real flight data (v3.4.0):
runNASAFlight                  % Track real Tail-687 flight with simulated radar
viewNASAFlightGlobe            % Earth-centered 3D view of real flights
scanNASAFlights                % Profile all flight files in a folder

% Automated parameter sweep (v3.4.0):
autoTuneTracker("my_run", "GNN")   % Two-pass GNN tuning on cached detections

% Validation suite (v3.3.0):
runTestPlan                    % All 9 test cases, 27 assertions
verifySimulation               % 40+ diagnostic checks across 8 phases

% GUI entry points (v3.5.0+):
mainMenu                       % 4-button launcher (Path Editor / Run Sim / Validation / Flight Data Mgr)
pathEditor                     % click-and-drag scenario builder → JSON
runSimGUI                      % Run Simulation window with in-app Tracker Editor
validationDocsGUI              % 27-assertion test plan + diagnostics + docs
flightDataManagerGUI           % browse NASA flights, build multi-flight batches (v3.6.0)
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
  └── weather  → config/weather/rain/default_rain.json   (v3.4.0)
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

## Project Structure

```
<repo root>/
├── adeptus-rainyday-tracking/        ← MATLAB project (this folder)
│   ├── trackbench.vbs                ← double-click to launch (silent)
│   ├── trackbench.bat                ← same, shows brief console
│   ├── install_shortcut.vbs          ← run once: Desktop shortcut
│   ├── mainMenu.exe                  ← compiled launcher (1.95 MB)
│   ├── README.md                     ← this file
│   ├── CHECKPOINT.md                 ← in-progress dev state
│   ├── config/                       ← JSON configs (run files, sensors, …)
│   ├── src/+trackbench/              ← +trackbench package + sub-packages
│   ├── scripts/                      ← .m entry points + helpers
│   ├── docs/                         ← Rainy Day.pdf + reference markdowns
│   └── tests/                        ← regression tests
│
└── Tail_687_1/                       ← NASA DASHlink FDR data (sibling of project)
    ├── 687200104121330.mat
    ├── 687200107261425.mat
    └── 687200107282131.mat
```

`runNASAFlight.m` resolves `Tail_687_1/` as a sibling of the project root, so
keep both folders together if you want NASA flight features to work.

### Inside `config/`

```
config/
├── runs/                       ← Run files (entry points)
│   ├── run_template.json
│   ├── dasr_baseline.json
│   ├── fighter_intercept.json
│   ├── recorded_flight/        ← NASA flight scenarios (v3.4.0)
│   ├── validation/             ← TC-01..TC-09 test run files
│   └── showcase/               ← Pre-built scenarios
├── sensors/                    ← Per-type folders (19 types)
│   └── PSR/ SSR/ AESA/ PAR/ TWS/ FIRE_CONTROL/ ARSR/ IRST/ FLIR/ MARITIME/ WEDGE/ ...
├── targets/                    ← Per-behavior folders
│   └── crossing_pair/ gentle_turn/ s_maneuver/ orbit/ approach/ rcs_demo/ recorded_flight/ ...
├── terrain/                    ← none / water / rural / urban / mountain / desert
├── trackers/                   ← GNN / JPDA / TOMHT + tracker_globals.json
└── weather/                    ← rain / snow / fog / icing
```

### Inside `src/+trackbench/`

```
+trackbench/
├── +config/         loadRunFile.m
├── +detections/     runDetections.m, getWeather.m, ...
├── +scenario/       addTargetFromDef.m, validateScanCoverage.m
├── +tracking/       buildTracker.m, initCVFilter.m, initIMMFilter.m, runTracker.m
├── +sensors/        buildSensor.m, buildIFFSensor.m
├── +environment/    isAboveHorizon.m, generateGroundClutter.m, generateTerrain.m,
│                    applyDopplerFade.m, applyWeatherDegradation.m, applyRainDegradation.m,
│                    applyRCSFilter.m, buildRCSProfile.m
├── +reporting/      plotInitialScenario.m, drawSensorCoverage.m, drawBeamEnvelope.m, ...
├── +flightdata/     loadNASAFlight.m
├── +validation/     validateScenarioConfig.m
├── +analysis/       analyzeTrackSwaps.m
└── +editor/         EditorState, buildUI, drawMap, exportToJSON, loadFromJSON,
                     previewWindow, computeScaleBar, interpPos, zoomAroundPoint,
                     plus exportTerrainToJSON, exportWeatherToJSON, exportSingleSensorToJSON
```

### Inside `scripts/`

| Script | What it does |
|--------|--------------|
| `mainMenu.m` | 3-button main menu (this is the EXE entry point) |
| `pathEditor.m` | Interactive scenario builder (Path Editor window) |
| `runSimGUI.m` | Run Simulation window with cache/dirty tracking and in-app Tracker Editor |
| `validationDocsGUI.m` | Test plan + diagnostic suite + docs window |
| `runSingleScenario.m` | Run a scenario from the MATLAB console |
| `runTestPlan.m` | Validation suite (27 assertions across 9 test cases) |
| `verifySimulation.m` | 40+ standalone diagnostic checks |
| `autoTuneTracker.m` | Two-pass parameter sweep |
| `compareTrackers.m` | Side-by-side tracker comparison |
| `runNASAFlight.m`, `scanNASAFlights.m`, `viewNASAFlightGlobe.m` | NASA real flight data tools |
| `build_executable.m`, `build_installer.m` | Compile `mainMenu.exe` and the Web/Offline installer |
| `diagBeamLimits.m`, `diagnoseBadDetections.m`, `diagVerifyTargetIndex.m` | Detection diagnostics |
| `legacy/` | Milestone test scripts, presentation-prep helpers, superseded launchers (kept for archeology, excluded from CTF bundle) |

## Available Sensor Types (19)

> **Maturity (this build):** the 10 **radar** types are production-grade. **Sonar** and **IR** are **experimental** (functional, not validated); **LIDAR** and **ADS-B** are **scaffolded** (objects build but are not wired into detection generation). See [Experimental Build](#experimental-build).

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

## Experimental Build

This branch (`Michael-Testing-Maritime`) is the **Experimental** release that ships alongside the **Stable** radar build. The radar pipeline is unchanged; the capabilities below are demonstrated functional but **not validated** to the Stable standard - treat them as capability demonstrations, not sign-off output.

| Family | Status | Functions today | Needs continued work |
|--------|--------|-----------------|----------------------|
| **Radar** (10 types) | Stable | Full pipeline: detection, GNN/JPDA/TOMHT x CV/IMM, 3D coverage, editor authoring | Production baseline |
| **Sonar** (3) | Experimental | Acoustic emit/propagate/detect; GNN+IMM tracking (~single-digit-m RMS on a clean contact); editor depth-target authoring; depth-vs-range plot | Surface mirror ambiguity; tracker tuning for slow contacts |
| **Infrared** (3) | Experimental | Angle-only detection; range-parameterized MSC-EKF + per-scan observer input; runs on a maneuvering ownship | False-alarm/FOV tuning; range needs a maneuvering observer; no editor authoring |
| **Moving platform** | Experimental | Any sensor on a waypoint-trajectory ownship; world-frame detections; animated 3D coverage that follows the platform | Authored in JSON (no editor path); banking not fully modeled |
| **LIDAR**, **ADS-B** | Scaffold | Sensor/transponder objects build | Not wired into detection generation |

**Try it** (run `clear classes; clear all; rehash` first):

| Scenario | Command |
|----------|---------|
| Sonar | `runSingleScenario('SONAR_TEST')` |
| Airborne IR | `runSingleScenario('test_IRST_airborne')` |
| Airborne radar | `runSingleScenario('AirborneRadar_demo')` |
| Sonar authoring | Main Menu -> Path Editor -> Maritime Sonar |

Full detail, screenshots, and known limitations are in the **Experimental Features** disclosure document (in the Final Deliverable).

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

Both per-tracker and global params can be edited directly from the Run
Simulation window via *Edit Tracker* and *Edit Globals…* (v3.5.2) — no JSON
editing required. The in-app editor enforces field constraints, shows
typical-value tooltips, and emits save-time warnings for inverted
thresholds or non-monotonic TOMHT multipliers.

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

### 6. Weather Degradation (v3.4.0)
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
| `none` | OFF | OFF | OFF | 0 | 0m |
| `water` | OFF | OFF | OFF | 0 | 0m |
| `rural` | ON | ON | ON | 0.3 | ~73m |
| `urban` | ON | ON | ON | 0.6 | ~150m |
| `mountain` | ON | ON | ON | 0.5 | ~1960m |
| `desert` | ON | ON | ON | 0.2 | ~40m |

`none` and `water` both produce a perfectly flat surface; `none` is the
default for fresh editor sessions and is the right pick when you want a
baseline clean run with no terrain contribution at all.

### Beam Envelope Visualization (v3.4.1)
`drawBeamEnvelope.m` queries `coverageConfig()` for exact scan limits and
renders green (included) / red (excluded) cones for any radar or IR sensor.
Automatically called by `plotInitialScenario`.

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

**Data location:** `Tail_687_1/` is a sibling of `adeptus-rainyday-tracking/`.
The repo includes three flight files (~7.8 MB total). Additional Tail-687
flights are available from
<https://c3.ndc.nasa.gov/dashlink/resources/664/>.

## Interactive Path Editor (v3.5.0)

The Path Editor button in the main menu (or `pathEditor` from MATLAB) launches
an interactive uifigure-based scenario builder. Click waypoints onto a 2D or
3D map; sidebar lets you build complete scenarios (sensors + targets + terrain
+ weather) without touching JSON.

Exports directly to the run-file format that `runSingleScenario` consumes —
the editor's "Export Scenario" button writes a complete bundle (sensors +
targets + terrain + weather + run file).

**Drawing**
- **Left-click** on map → append waypoint at default altitude + default speed
- **Drag** a waypoint → move it (undo granular per drag, not per pixel)
- **Shift+click** on a segment → insert a new waypoint at the click point
- **Delete / Backspace** → remove selected waypoint
- **Ctrl+Z / Ctrl+Y** → undo / redo (50 levels)
- **V** → toggle 2D ↔ 3D view
- **Middle-click + drag** → pan (2D)
- **Scroll wheel** → zoom around cursor

**Sidebar controls (mode-specific show/hide as of v3.5)**
- **Targets mode** — dropdown + collection mgmt + Load/Save Target + Import NASA
  Flight + Reference overlay + Clear waypoints
- **Sensors mode** — dropdown + collection mgmt + Load/Save Sensor +
  Sensor parameters
- **Environment mode** — Terrain panel (with Save) + Weather panel (with Save)

The editor is **fully portable** — resolves its project root from
`mfilename('fullpath')` so it works from any clone location without
configuration.

## Validation & Testing

`scripts/runTestPlan.m` executes 9 test cases containing 27 assertions.
Current status (v3.5.2): **27/27 passing**. The Validation & Documentation
window in the main menu runs the same suite with an inline result grid.

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

Run files for the test cases live in `config/runs/validation/`. Results are
saved to `results/test_plan_results_<timestamp>.mat`.

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

## Required Toolboxes (for source / build)

EXE users need only **MATLAB Compiler Runtime (MCR) R2025b** (free download,
linked above). The toolboxes below are required only if you're running from
MATLAB or rebuilding the EXE:

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

### Experimental - June 1, 2026 (Michael-Testing-Maritime branch)

**Stretch sensing: sonar, infrared, and moving-platform - proof-of-concept.**

Adds end-to-end **sonar** (acoustic emit/propagate/detect, GNN+IMM tracking, editor depth-target authoring, depth-view plot), **passive IR ranging** on a maneuvering ownship (MSC-RPEKF + per-scan observer input), and **moving sensor platforms** (any sensor on a waypoint-trajectory ownship, world-frame detections, animated 3D coverage). Demo scenarios: `SONAR_TEST`, `test_IRST_airborne`, `AirborneRadar_demo`. A `%#function` manifest in `mainMenu.m` forces `mcc` to bundle the new toolbox calls into the compiled EXE (verified: all three run in the deployed app). Robustness: `runTracker` returns gracefully on a zero-detection scenario instead of indexing an empty log. Radar core unchanged. **Experimental: functional but not validated.**

### v3.6.15 — May 26, 2026 (current — SENSORS v3.6.x line)

**IRST scan-envelope override + production-runnable test scenario.**

This release bundles Phase 2 (scan-envelope override) and Phase 3 (test_IRST scaffolding) of the 2026-05-26 demo-prep dispatch. First v3.6.x line release since v3.6.14, landing in parallel with the v3.7.0 BUGHUNT release of the same day. Closes the SENSORS-side IRST plumbing gap (sub-horizon class-default scan envelope) and supplies the canonical end-to-end smoke-test scenario that exercises v3.7.0's `trackbenchFilterInit` wrapper through the full production pipeline. No `.m` files modified; PosterDemo bit-identical to the v3.6.14 / v3.7.0 baseline by construction. Per `HANDOFF_COORDINATION.md`, the SENSORS and BUGHUNT lines remain in parallel until the v3.8.0 demo-lock merge (Thu 2026-05-28 EOD).

**Phase 2 — IRST scan-envelope JSON override.**

- **Symptom.** Tactical IRST scenarios need upward elevation coverage; `irSensor`'s R2025b class default for `MechanicalScanLimits` is `[0 360; -10 0]` — elevation entirely at or below the horizon. Doesn't fit incoming-airborne-threat geometry.
- **Root cause.** `config/sensors/IRST/default_IRST.json` declared no `MechanicalScanLimits` override; the runtime IRST inherited the class default. `buildSensor.m`'s formula-computed override at line 577 is gated on `sectorSpan < 359` and doesn't fire for default 360°-rotator IRST (sector = `[0, 360]`). The v1.3 matrix entry's framing of this finding incorrectly attributed the default to the JSON file; corrected in matrix v1.4 (committed earlier this session).
- **Fix.** Add `"MechanicalScanLimits": [[0, 360], [-5, 15]]` to the `params` block of `config/sensors/IRST/default_IRST.json`. The value flows through `loadRunFile.m:87-116`'s nvPair construction → `buildSensor` varargin → `buildIR`'s `inputParser` (`KeepUnmatched=true`) → `p.Unmatched.MechanicalScanLimits` → `applyUnmatched(ir, p.Unmatched)` at line 581 → `safeSet(ir, 'MechanicalScanLimits', value)`. R2025b `irSensor` doc confirms `MechanicalScanLimits` accepts 2×2 matrix `[minAz maxAz; minEl maxEl]` when `ScanMode='Mechanical'` (default), with azimuth span ≤ 360° and elevation in [-90°, 90°]. Our `[0 360; -5 +15]` is in spec.
- **Validation expected.** PosterDemo bit-identical to v3.6.14 / v3.7.0 baseline (PSR-only scenario, never builds IRST). Confirmed empirically before this entry was written via the supplementary debug script reproducing `loadRunFile.m:87-116`'s nvPair construction: `ir.MechanicalScanLimits = [0 360; -5 15]` after build. End-to-end validation lands with the Phase 3 `test_IRST` smoke test.

**Phase 3 — Canonical IRST smoke-test scenario (test_IRST).**

Three new JSON files supplied as a production-runnable end-to-end verification of the IRST pipeline — exercising the full chain from JSON load through the v3.7.0 `trackbenchFilterInit` wrapper and into `trackerGNN`. Not a v3.7.0 inline empirical bypass; runs through the canonical `loadRunFile → buildSensor → buildIR → runDetections → tracker` pipeline that production scenarios use.

- **`config/targets/ir_low_altitude_demo/default_ir_low_altitude_demo.json`** — Single `crossing` target, 100m altitude, 5km lateral offset from the IRST tower, 10km traversal in 60s at 800 km/h. Range stays 5-7km throughout (well under the IRST's 100km rangeLimits); elevation stays 0.73°-1.03° (well inside the new [-5°, +15°] envelope). 5 dBsm RCS, `class_id=1`.
- **`config/runs/test_IRST.json`** — References the new target preset + `IRST/default_IRST` + `none/default_none` terrain + `GNN/test_IRST_GNN`. All degradations off for minimum-friction verification. Expected console signature: `Primary sensors: 1 | Beacon sensors: 0` (IR is classified as Primary per v3.6.8 labeling), per-scan `Primary=N, Beacon=0, total=N (clutter=0)`. Expected visualization: magenta dashed coverage ring (`drawSensorCoverage` `isIR` branch) + magenta beam cone (`drawBeamEnvelope` `isIR` gate). Success gate: end-to-end run without error + ≥1 confirmed track.
- **`config/trackers/GNN/test_IRST_GNN.json`** — Identical to `default_GNN.json` except `confirm_threshold: 5` (lenient Score-mode scalar). See Process findings below for why this substitutes for the dispatch-specified `[2 3]` history-based form. Detection-count math: at IRST 1 Hz scan rate, pd 0.7, 60s run → ~42 detection opportunities; Score-mode threshold 5 confirms in ~3 successful detections at Pd≈0.99 (score ~1.5 per detection per MATLAB doc). Achievable even at the lower observed detection rate from the v3.7.0 gate-6 empirical (2 detections in 60s with comparable angle-only geometry).

**Matrix v1.4 → v1.5 (verification block correction).**

`docs/SENSOR_VALIDATION_MATRIX.md` bumped to v1.5. The v1.3/v1.4 verification command block in the IRST row contained a verification script `[ir, ~] = trackbench.sensors.buildSensor(1, 'IRST')` with comment `% After Phase 2 commit: [0 360; -5 15]` — but the script doesn't load the JSON. With empty `varargin`, `buildSensor` builds with `getDefaults('IRST')` only, `p.Unmatched` is empty, `applyUnmatched` is a no-op, and the irSensor inherits the class default `[0 360; -10 0]`. The script was testing a sibling path (direct `buildSensor` invocation) that happens to produce a similar-looking surface, not the JSON-override path it claimed to test. v1.5 replaces the block with: (1) canonical end-to-end verification via `runSingleScenario("test_IRST")`, and (2) a corrected supplementary debug script reproducing `loadRunFile.m:87-116`'s nvPair construction inline.

**Process findings.**

- **Verification scripts must exercise the specific code path they claim to test.** During Phase 2 verification, the matrix v1.3 inline script appeared to fail (returned class default `[0 360; -10 0]` instead of expected override `[0 360; -5 15]`). Initial reaction was to trace a downstream defect through `buildSensor` → `buildIR` → `applyUnmatched` → `safeSet`. Discovery on read-only tracing of `loadRunFile.m`'s nvPair construction (lines 87-116) was that the verification script *bypassed* this path entirely — `trackbench.sensors.buildSensor(1, 'IRST')` with no varargin means empty `p.Unmatched`, and the JSON override is never seen. The script wasn't testing the path it claimed to test; it was testing direct `buildSensor` invocation which produces class default. The JSON override mechanism was working correctly all along; only the verification was misaligned. Same shape of failure mode as BUGHUNT's "comprehensive-looking checks that mask bugs" category: a check that looks like it tests behavior X but actually tests behavior Y. Banking this as a methodology discipline rule: every verification script must reproduce the same code path the production scenario uses, not a shortcut path with a similar-looking surface. Adds to the v3.6.x pattern (alongside the PosterDemo bit-identical canary coverage gap from v3.6.14): the harder the verification, the more important it is that the verification *matches reality*.

- **History-logic `[M N]` ConfirmationThreshold is locked out across all three trackers — three different mechanisms.** During Phase 3 planning, the dispatch's specified `confirm_threshold: [2 3]` (1×2 history-based form) was discovered incompatible with the current `buildTracker.m`. R2025b `trackerGNN.ConfirmationThreshold` semantics depend on `TrackLogic`: `'Score'` mode takes a scalar score threshold; `'History'` mode takes a 1×2 vector `[M N]` (confirm after M hits in last N attempts); `'Integrated'` mode takes a probability scalar. Empirical grep of `buildTracker.m` shows the History form is blocked across all three tracker types but **via three different mechanisms** — worth recording precisely because the BUGHUNT post-demo fix is per-tracker, not one-size-fits-all: (1) `trackerGNN` at line 67 **explicitly hardcodes** `'TrackLogic', 'Score'`, so a `[M N]` value flowing through would error at construction. (2) `trackerTOMHT` (lines 78–92) **doesn't set `TrackLogic` at all** — falls back to the SDK default, which is also `'Score'`. Same effective behavior as GNN but via inheritance, not hardcoding. (3) `trackerJPDA` at line 114 **hardcodes `'TrackLogic', 'Integrated'`** — a different mode entirely that takes probability values (line 100–101: `jpda_confirm = 0.95`, `jpda_delete = 0.05`); also incompatible with `[M N]` but for a different reason. Net result: the `[M N]` history form is locked out across all three tracker types, but the fix for each tracker would touch a different code site. Workaround for v3.6.15: scalar `confirm_threshold: 5` (lenient Score-mode) in `test_IRST_GNN.json`, with the JSON description block explicitly documenting the substitution. BUGHUNT-side fix would enable type-dispatch on the JSON shape (scalar → Score, vector → History, opt-in probability → Integrated for JPDA) to expose all three modes — small change per tracker, touching `+tracking/buildTracker.m`. Out of demo-week scope; documented for the project record.

- **`default_IRST.json` joins the CRLF→LF artifact list as the 7th file.** Previously CRLF (12 CR characters); `Filesystem:edit_file` converted to LF on the Phase 2 commit, as it has for every prior file that crossed it. Full list (7 files): `drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`, `buildSensor.m`, `buildTracker.m`, `default_IRST.json`. Review with `git diff -w` to filter line-ending noise. First non-`.m` file on the list; the pattern generalizes.

**Validation expected (overall).**

- PosterDemo bit-identical to v3.6.14 / v3.7.0 baseline. No `.m` files modified in this release; only JSON (`default_IRST.json` and three Phase 3 net-new files) and markdown (`SENSOR_VALIDATION_MATRIX.md` and this README entry).
- `runSingleScenario("test_IRST")` runs end-to-end without error AND produces ≥1 confirmed track. This is the Phase 3 success gate. Expected console: `Primary sensors: 1 | Beacon sensors: 0`, per-scan `Primary=N, Beacon=0, total=N (clutter=0)`. Expected visualization: magenta dashed IRST coverage ring at the tower location + magenta beam cone in the 3D view.
- Phase 2 sub-validation (supplementary): reproducing `loadRunFile.m:87-116`'s nvPair construction inline → `ir.MechanicalScanLimits = [0 360; -5 15]` (the supplementary debug script in matrix v1.5).

**Out of scope this release.**

- BUGHUNT-side per-tracker fix to `buildTracker.m` enabling History-logic confirmation (three different code sites). Post-demo or separate v3.7.x release.
- After BUGHUNT lands the fix, `test_IRST_GNN.json` should be updated to use the literal dispatch-specified `[2 3]` form; the substitution to scalar 5 is a workaround.
- IRST G3/G5/G7 matrix promotions — these close after `runSingleScenario("test_IRST")` empirical success (an orchestrator-validated runtime check, not a SENSORS-side commit). Next matrix bump after success.
- Proper IR-aware weather degradation (fog physics for `irSensor`'s lens-optics sensitivity model) — post-demo framework work, BUGHUNT scope.

### v3.7.5 — May 27, 2026 (current — visualization refactor)

**Faithful sensor coverage volume rendering replaces generic green/red beam-edge cones in the Scenario Truth and Detections plot, plus a tactical viz filter for an `HasINS=false` body-frame artifact surfaced by the new reference frame. PosterDemo + test_IRST bit-identical to v3.7.3 baseline.**

Demo-week visualization refactor. Replaces `drawBeamEnvelope.m`'s open upper/lower beam-edge cones with closed swept-volume rendering at Site 1 (`plotInitialScenario.m`), so the demo audience can see what the sensor actually detects — the geometric region defined by the sensor's true `MechanicalScanLimits` and `RangeLimits` — rather than two abstract boundary surfaces. Sites 2 (`runTracker.m`, BUGHUNT-owned) and 3 (`drawMap.m`, Path Editor, on the CRLF→LF artifact list) defer to Thursday by design — both touch ownership/CRLF boundaries that aren't worth crossing in a tonight-locked session. New function `computeSensorCoverageVolume.m` is call-site-portable: takes a `cov` struct (one element of `dataLog.SensorCoverage`) and returns patch-ready vertices + faces in world-frame NED meters, so the same function backs all three sites when the BUGHUNT/Path-Editor work lands.

**Site 1 — sensor coverage volume in plotInitialScenario.m.**

- **New function `src/+trackbench/+reporting/computeSensorCoverageVolume.m`.** Returns `[V, F]` describing the closed boundary of the 3D swept region. For 360°-rotator sensors (PSR, IRST, MSSR): outer spherical cap at `r = rMax` over an `(az, el)` ndgrid, plus top cone at `el = elMax` and bottom cone at `el = elMin` over `(az, r)` grids — three closed surfaces, no side walls (the volume wraps around 360°). For sector sensors: two additional flat walls at `az = azMin` and `az = azMax` over `(el, r)` grids. Default sampling 30 az × 10 el; ~1000 vertices and ~1000 quads per typical 2-sensor scene, well below the interactive-frame budget. Geometry computed in WORLD-frame NED meters; caller is responsible for the m→km scale and NED→altitude-up Z-negation, mirroring `drawBeamEnvelope.m`'s pattern of doing the unit conversion at the call site.
- **Property accessors per R2025b doc.** `MountingAngles` is `[z-yaw, y-pitch, x-roll]` per the fusionRadarSensor Sensor Mounting block; only `yaw` is propagated through to `cov.mountingYaw` by `runDetections.m:329` today, so pitch/roll are treated as 0 (correct for tower-mounted radars and roughly-upright aircraft; documented as a Site 3 / post-demo follow-up when the Path Editor wants tilted IRSTs). `RangeLimits` defaults to `[0 Inf]` on `fusionRadarSensor`; the `cov.maxRange` try/catch in `runDetections.m:328` doesn't catch `Inf` because no exception is thrown for it, so `computeSensorCoverageVolume` caps explicitly via `opts.rMaxCap = 120000m` per the `drawBeamEnvelope.m:47` precedent. `MechanicalScanLimits` are read indirectly via `cov.scanElLimits` (preferred — true beam edges from `coverageConfig`) with `cov.elLimits` (`MechanicalElevationLimits` direct) as fallback.
- **Call-site integration at `plotInitialScenario.m:41-48`.** Replaces the single-call try/catch around `drawBeamEnvelope` with a per-sensor loop calling `computeSensorCoverageVolume(cov)`, converting NED meters → display km + altitude-up, and `patch`ing with `FaceAlpha=0.18, EdgeAlpha=0.10`. Color selection mirrors `drawSensorCoverage.m` v3.6.15 (FROZEN, see `HANDOFF_COORDINATION.md`) per-type conventions: PSR blue `[0.2 0.6 1.0]`, IR magenta `[0.9 0.2 0.8]`, sector radar green `[0.1 0.9 0.3]`. MSSR skipped (matching `drawBeamEnvelope`'s gate at line 23). Same `cov.azLimits + cov.mountingYaw` world-frame azimuth convention as `drawSensorCoverage.m:68`, so the new coverage volume aligns with its existing ground ring rather than rotating independently.
- **`drawBeamEnvelope.m` deprecated.** Header comment added: `DEPRECATED v3.7.5+ — superseded by computeSensorCoverageVolume in +reporting`. File retained as fallback only; not deleted, not called from the primary plotting path. Will be removed once Sites 2 + 3 land and the new function has soaked through a demo cycle.

**Tactical fix — underground-detection visualization filter (deferred root cause).**

- **Symptom.** With the new translucent coverage volume providing a visual reference frame for "where the sensor can see," PosterDemo screenshots showed a substantial subset of detections rendering at display altitudes below ground (−5 to −10 km in the static 3D plot, well below the terrain mesh peak at 3898 m). Not caused by this release's edits — `plotInitialScenario.m` detection rendering at lines 147 / 157 has been unchanged for many releases before v3.7.5 — but newly visible against the coverage-volume reference. PosterDemo's `MountainSensor` mounting altitude 4053 m makes the artifact prominent; the same mechanism is presumably present in any non-trivial-altitude sensor scenario.
- **Root cause hypothesis (deferred to v3.7.4).** `plotInitialScenario.m:147` and `:157` treat `det.Measurement(1:3)` directly as world-frame NED coordinates, but `fusionRadarSensor.HasINS=false` (the SDK default) reports `objectDetection.Measurement` in sensor body frame. For a high-altitude mounted sensor, body-frame z-axis points up from the sensor, so ground-level targets project to NED z ≈ sensor-mounting-altitude when treated as world coords — display altitude `-meas(3)` then maps to a substantial negative value. Three candidate root causes filed for v3.7.4 BUGHUNT (see follow-up list below).
- **Fix this release (viz-only mitigation).** One-line guard added at both detection scatter sites (`plotInitialScenario.m:147` cell-array branch, `:157` struct-array branch): `if meas(3) <= 1` before `addpoints` — drops detections whose NED z is more than 1 m below ground (equivalently, display altitude < −1 m). Surface-level legit detections (clutter at z ≈ 0) pass through; phantom underground detections are suppressed. Does not touch the underlying detection generation, storage, or sensor frame convention — those land in v3.7.4 per the candidate list below.

**Validation expected.**

- **PosterDemo bit-identical to v3.7.3 baseline.** No edits touch detection generation, tracker math, or assignment metric. Empirical confirmation (Michael IDE, 2026-05-27 17:59): T1=1233.1m, T2=340.0m, T3=2043.7m, T4=544.9m, T5=4932.4m, T6=1760.7m, T7=1654.7m, Avg=1787.1m, Tracked%=98/98/73, Swaps=CLEAN, Truth3 est-fail=27% (23/86), per-scan canary `t=4.80: Primary=8, Beacon=0, total=8 (clutter=3)` — all exact match scan-by-scan.
- **test_IRST Tracked%=96% holds** (TruID 1 → AssocTrk 8 → Tracked%=96%, 23 scans), consistent with v3.7.3 MSC-RPEKF stationary-geometry result.
- **Visual gates passed.** PSR coverage volume renders as blue translucent enclosed swept shape with outer shell + top/bottom cones, aligned with the existing dashed range ring at ground level; IRST coverage volume renders magenta. Tracker plot at `runTracker.m:118` (Site 2, BUGHUNT-owned) intentionally unchanged this release; ground ring still renders correctly via the unmodified `drawSensorCoverage.m` call.

**Process findings.**

- **Replace before introspecting at FROZEN-file boundaries.** The brief's `(sensor, mountingLoc, mountingAngles, platformPos)` signature for `computeSensorCoverageVolume` would have required threading live sensor objects through `runDetections.m:308-355` (BUGHUNT-owned) to reach the Site 1 call site. The pragmatic deviation — function takes the already-populated `cov` struct — kept the edit footprint to a single non-owned package (`+reporting/`) and avoided crossing the BUGHUNT boundary in a 24-hour-lock-window session. Cost: no live-sensor overload for Site 3 (Path Editor) yet; benefit: tonight ships. Same calculus as the v3.6.x scoping pattern where SENSORS / BUGHUNT lines deliberately run in parallel rather than serializing through merge contention.
- **New reference frames expose pre-existing anomalies.** Underground detections were rendering in every prior release; the new translucent coverage volume just made them visible. Second time in v3.7.x where adding "what should be" structure (v3.7.0 `trackbenchFilterInit` wrapper; v3.7.5 coverage volume) surfaced a pre-existing "what is" violation. Worth banking as a methodology principle: any visualization that establishes a geometric / spatial reference for a derived quantity (coverage envelope, expected measurement frame, scan rate) will surface upstream bugs invisible without the reference. Future visualization work should expect this and triage discovered anomalies as scoping decisions (fix-now vs deferred-with-mitigation), not as regressions from the visualization itself.
- **Two-occurrence anchor uniqueness verified empirically before each Edit call.** The detection scatter site has the same `addpoints(hDetect, meas(1)*s, meas(2)*s, -meas(3)*s);` line in both cell-array and struct-array branches (different surrounding context, identical inner text). Per project's anchor-uniqueness-empirical rule (v3.6.12 commit was lucky, v3.6.14 first-revert was caught at diff review), `grep -c` confirmed 2 occurrences in the file, and each Edit used multi-line context anchored on the distinguishing branch structure (`elseif numel(meas) == 2` for the cell-array branch, bare `else` for the struct-array branch).

**Doc-fetch (load-bearing for fix design).**

- *`fusionRadarSensor` System object* — <https://www.mathworks.com/help/fusion/ref/fusionradarsensor-system-object.html> — confirmed `MountingAngles` form `[z-yaw, y-pitch, x-roll]` (Sensor Mounting block); `RangeLimits` default `[0 100e3]` but commonly observed `[0 Inf]` for unbounded sensors (Detection Reporting block); `MechanicalScanLimits` 2×2 matrix when `ScanMode='Mechanical'` (Scanning Settings block).
- *`irSensor` System object* — <https://www.mathworks.com/help/fusion/ref/irsensor-system-object.html> — confirmed `MechanicalScanLimits` accepts 2×2 matrix `[minAz maxAz; minEl maxEl]`, same convention as `fusionRadarSensor`; relevant for the v3.6.15 IRST `[0 360; -5 15]` override which feeds `cov.scanElLimits` via `coverageConfig` and is consumed unchanged by the new function.

**v3.7.4 follow-up queue (this release expands the detection-visualization audit).**

- **Detection visualization spatial-frame audit — three candidates, BUGHUNT scope.**
  - (a) **`HasINS=false` body-frame measurement convention at `plotInitialScenario.m:147` and `:157`.** Treats `meas(1:3)` directly as world coords, but for HasINS=false sensors the measurement is in sensor body frame and needs transform via `cov.mountingLocation + cov.mountingYaw + cov.position` before plotting. v3.7.5 tactical filter suppresses the visible symptom; proper fix here.
  - (b) **Ground clutter Z-coordinate sampling in `generateGroundClutter.m`.** Hypothesis: places surface clutter at `z = 0` NED (sea-level reference) rather than sampling the terrain heightmap at clutter `(x, y)` coords. Affects scenarios with non-trivial terrain — PosterDemo's MountainSensor at 4053 m mounting altitude is the canonical case where the artifact would compound with (a).
  - (c) **Detection-vs-coverage-envelope membership not enforced in visualization.** Detections rendered outside `MechanicalScanLimits` envelope — clutter / false alarms not gated by sensor geometric visibility. Optional v3.7.4 viz filter candidate: drop a detection if its `(az, el, r)` projection back into the sensor frame falls outside `cov.azLimits / cov.elLimits / cov.maxRange`. Lower priority than (a) and (b); only worth doing if (a) doesn't naturally eliminate the visual noise.
- **Existing v3.7.4 queue items from CHECKPOINT.md preserved** (Airborne IRST ObserverInput investigation, X-F8 trackErrorMetrics, mixed-sensor visualization, multi-ownship support).
- **Sites 2 + 3 visualization refactor follow-on.** Site 2 (`runTracker.m`, BUGHUNT) and Site 3 (`+editor/drawMap.m`, Path Editor) both adopt the new `computeSensorCoverageVolume` without code changes to the function itself. Site 3 may benefit from a live-sensor overload of the function (cov struct isn't populated until `runDetections` runs).
- **`drawBeamEnvelope.m` removal.** After Sites 2 + 3 land and the new function has soaked through a demo cycle.

**Out of scope this release.**

- **Sites 2 + 3.** Tracker visualization (`runTracker.m`, BUGHUNT-owned) and Path Editor (`drawMap.m`, CRLF→LF artifact) deferred to Thursday with proper line scoping.
- **Terrain occlusion of coverage volume.** The new volume renders the full geometric envelope without clipping at terrain LOS. In PosterDemo the bottom cone visibly dips below the terrain at far range — geometrically correct but physically the radar wouldn't see those returns. Terrain occlusion of the volume mesh would require LOS sampling through the heightmap per vertex; post-demo enhancement.
- **Animated mode for coverage volume.** Static rendering only this release. The current call site lives inside the figure setup before the animation loop, which is the right architectural slot — animated coverage updates would require either re-patching every scan (expensive) or alpha-toggling pre-built per-scan volumes (storage cost). Defer until a real use case emerges.
- **Live-sensor overload of `computeSensorCoverageVolume`.** Current signature takes a `cov` struct; the brief's `(sensor, mountingLoc, mountingAngles, platformPos)` form is achievable as an `isstruct(arg1)`-dispatched overload when Path Editor (Site 3) wants to render coverage from a live sensor object before `runDetections` has populated `dataLog.SensorCoverage`. Adds ~15 lines if/when needed.
- **Underground-detection proper fix.** The tactical viz filter is symptomatic only; root cause (HasINS=false body-frame measurement convention) lands in v3.7.4 per the follow-up queue above.

### v3.7.3 — May 27, 2026 (superseded by v3.7.5)

**MSC-RPEKF filter pivot for moving-ownship passive ranging + X-F7 metric-axis fix. PARTIAL SUCCESS: stationary IRST (`test_IRST`) reaches 96% Tracked%; airborne IRST (`test_IRST_airborne`) end-to-end runs cleanly with 86 tracks formed (several confirmed) but still 0% Tracked% — ObserverInput plumbing investigation deferred to v3.7.4.**

Sister release to v3.7.2's X-F6 truth-log fix. Phase 3 empirical for v3.6.16 airborne_IRST after v3.7.2 confirmed Hypothesis B from the orchestrator's diagnostic matrix: `initrpekf` (Cartesian RPEKF) is the wrong filter for moving-sensor angle-only passive ranging — tracks form along correct bearings but range hypotheses spread vertically and never collapse (29 scans, 89 tracks, 0% Tracked%). MSC-RPEKF (range-parameterized modified-spherical-coordinate EKF bank) with per-scan `ObserverInput` propagation is the canonical fix per the MathWorks "Passive Ranging Using a Single Maneuvering Sensor" example. The filter pivot exposed a new Cartesian-state-assumption site (X-F7) at the `trackAssignmentMetrics` axis, patched in the same release. One further state-extraction site (X-F8, `trackErrorMetrics:200`) flagged for v3.7.4 per the X-F7-only patch contract.

- **Three-file commit — record-then-replay architecture.** Dispatch as originally written assumed live `scenario.Platforms` access in `runTracker.m` (canonical passive ranging example runs detection + tracking in a single `advance(scene)` loop); trackbench's staged pipeline (`runDetections → cached dataLog → runTracker`) doesn't carry the scenario object through to the tracker. Resolution: record-then-replay via three new `dataLog` fields populated in `runDetections.m` and consumed in `runTracker.m`.
  - **`runDetections.m`** — New local helper `ownshipPoseWorld(scenario)` mirrors v3.7.2's `targetPosesWorld` with inverted predicate (`~isempty(plat.Sensors)` for sensor hosts vs target platforms). Per-scan append to new `dataLog.OwnshipPose` struct array (PlatformID/Position/Velocity), parallel to `dataLog.Truth`. Scenario-level routing flag `dataLog.UsingMSCTracking = any([activeInfos.isIR])` set at dataLog init — always recorded regardless of sensor type, keeping dataLog shape consistent across runs.
  - **`trackbenchFilterInit.m`** — Angle-only branch routes to new `initMSCRPEKF` (near-verbatim from canonical example, only `rMin`/`rMax` adjusted from canonical `[3e4, 8e4]` long-range tactical geometry to `[1e3, 2e4]` for `test_IRST_airborne`'s ~5 km target range). 10 filters, `trackingGSF` wrapper, 400× velocity covariance scaling per canonical.
  - **`runTracker.m`** — Pre-loop state init (`usingMSC`, `prevPose`, `lastCorrectionTime`, `currentPose`). Per-scan `ObserverInput` propagation block via `getTrackFilterProperties(tracker, ..., 'TrackingFilters')` fires before `step(tracker, ...)`. 2-output tracker call upgraded to 3-output form so `allTracks` is populated for the propagation loop. `prevPose`/`lastCorrectionTime` update only on successful correction (non-empty scan) — maneuver accumulates across prediction-only scans, mirroring canonical pattern. Visualization at line 352 (the confirmed-track plot) branches on `usingMSC`: `getTrackPositionsMSC(tracks, currentPose.Position(:))` (verbatim from canonical example, uses `cvmeasmsc(state, 'rectangular')` + observer position) for MSC tracks, original Cartesian PositionSelector for radar tracks. Three new local helpers: `readOwnshipPose` (backward-compat fallback), `calculateManeuver`, `getTrackPositionsMSC` — all verbatim from canonical except the empty-tracks defensive early-return in the last one.

- **X-F7 metric-axis fix.** Phase 3 retest after the three-file commit produced 86 tracks (several confirmed) but `Tracked%=0%, Truth1 AssocTrk=NaN` — numerically near-identical to the pre-pivot v3.7.0 result (89 tracks, 0%). Filter math was actually sound: a one-shot diagnostic dump (X-F7 TEMP-DIAG block at `runTracker.m:356`) of first-track State + first-truth Position for the first 3 scans where both existed confirmed MSC state magnitudes consistent with truth range — scan 2: State[5]=1/r=0.000157 → range estimate 6370 m vs true 5704 m (~12% error, well within range-parameterized bank pre-collapse spread). The failure was at the metric site: `trackAssignmentMetrics` with `'MotionModel', 'constvel', 'AssignmentDistance', 'posabserr'` extracts track state elements [1,3,5] as Cartesian `[x,y,z]`; for MSC state those are `[az, el, 1/r]` ≈ `[0.5, 0.1, 0.0002]` — dimensional gibberish vs truth Cartesian magnitudes ~5000 m, so every track-to-truth distance exceeds `assignThresh=1000m` → no truth assignment ever succeeds. Resolution at `runTracker.m:190`:
  - Branch on `usingMSC` flag (same scenario-level flag pattern as the visualization branch added earlier in this release).
  - **MSC path**: `DistanceFunctionFormat='custom'` with `AssignmentDistanceFcn` and `DivergenceDistanceFcn` both bound to new file-local helper `trackbenchMSCAssignmentDistance(track, truth)` returning Euclidean meters via `cvmeasmsc(track.State, 'rectangular') + observerPosition` (world Cartesian) minus `truth.Position`.
  - **Observer position plumbing**: second file-local helper `trackbenchMSCObsPos(newPos)` with persistent slot (setter/getter dual mode). Setter call fires before each `step(tam, tracks, truths)` per scan, reading `currentPose.Position` (already updated for this scan by the MSC propagation block earlier in the loop). Persistent-variable pattern is the standard escape hatch for the System object's nontunable property lock — once `step(tam, ...)` has been called the function handle slot can't be re-bound.
  - **Cartesian path**: bit-identical to v3.7.2 baseline, preserved in the `else` branch.
  - **Empirical retest**: PosterDemo bit-identical to v3.7.2 (T1=1233.1m, T2=340.0m, T3=2043.7m, Avg=1787.1m, swaps clean). `test_IRST` (stationary IRST) reaches Tracked%=96%, AssocTrk=8 — the metric is now functional and the bank's lucky range hypothesis lands within `assignThresh=1000m` for the stationary geometry (per Fogel-Gavish 1988 a stationary angle-only observer cannot achieve true observability, but the bank's discrete hypothesis spread provides occasional in-threshold landings). `test_IRST_airborne` remains 0% (see v3.7.4 follow-up).

- **Ordering hotfix.** First commit of the X-F7 patch crashed PosterDemo immediately on `Unrecognized function or variable 'usingMSC'`. Root cause: `usingMSC = isfield(dataLog, 'UsingMSCTracking') && dataLog.UsingMSCTracking;` was defined inside the v3.7.3 MSC-RPEKF state init block at ~line 269 (just before the main loop) but read by the new tam construction at line 190. Hotfix: moved the single `usingMSC = ...` assignment up to just before line 190 (with a comment noting why the variable lives there rather than with the other v3.7.3 init vars). Remaining loop-scoped variables (`prevPose`, `lastCorrectionTime`, `currentPose`) stayed in their original location — they're only consumed inside the main loop. Comments in the original init block describing `usingMSC` semantics remain accurate context for the loop-scoped variables that follow.

- **Process findings (methodology principles banked).**
  - **State-read before dispatch.** The architectural blocker — staged-pipeline trackbench vs canonical example's live-scenario tracker loop — could not have been surfaced from doc-fetch alone. Reading the actual codebase state before designing the dispatch caught it before any commit effort was spent. Joins v3.7.2's doc-fetch-load-bearing rule. Future dispatches touching unfamiliar code regions should include state-read in the design phase, not just before commit.
  - **Always-record dataLog fields beat conditionally-record.** `OwnshipPose` and `UsingMSCTracking` are written every run regardless of sensor type. Degenerate cases handle themselves: stationary tower → zero-velocity OwnshipPose → `calculateManeuver` returns `zeros(6,1)` → MSC-RPEKF degrades cleanly to plain MSC-EKF; PSR-only run → `UsingMSCTracking=false` → all v3.7.3 code paths skipped. Backward-compat for pre-v3.7.3 cached dataLogs falls out naturally via `isfield` guards in `runTracker.m`'s `readOwnshipPose` helper.
  - **Verbatim-first canonical-pattern adoption with documented departures.** `calculateManeuver`, `initMSCRPEKF`, `getTrackPositionsMSC`, and the per-scan ObserverInput propagation pattern are all verbatim from the MathWorks Passive Ranging example. Three departures, each documented and justified in inline comments: (1) `rMin`/`rMax` scenario-tuned from canonical `[3e4, 8e4]` to `[1e3, 2e4]`; (2) record-then-replay dataLog plumbing for the staged pipeline; (3) scenario-level `usingMSC` flag for the X-F7 metric branch and visualization branch. No speculative deviations from canonical.
  - **State-extraction sites carry assumptions invisibly.** The Cartesian-state-assumption family is now confirmed at three sites (`runTracker.m:332` raw-detection viz — v3.7.1; `runTracker.m:352` confirmed-track viz — v3.7.3; `runTracker.m:190` assignment metric — X-F7, v3.7.3) with one further site flagged (`runTracker.m:200` `trackErrorMetrics` — X-F8, deferred to v3.7.4). When pivoting from one state shape to another, every site that touches `track.State` or applies a `MotionModel` string parameter must be audited — including MathWorks SDK calls. `'constvel'` in any property slot is a Cartesian-state assertion.
  - **When a diagnostic confirms cleanly, don't compound fixes.** v3.7.3's X-F7 confirmation diagnostic showed both (a) MSC state magnitudes unambiguous, and (b) range estimates in the right ballpark (~12% error vs truth). High confidence in the single-site patch. Resisted the temptation to also patch X-F8 in the same release — staying disciplined paid off when the ordering hotfix needed a second iteration without compounding cleanup.

- **Doc-fetch (load-bearing for fix design).** MathWorks R2025b documentation read before approving the dispatch design AND before approving the X-F7 patch design:
  - *Passive Ranging Using a Single Maneuvering Sensor* example: <https://www.mathworks.com/help/fusion/ug/passive-ranging-using-a-single-maneuvering-sensor.html> — canonical MSC-RPEKF pattern, ObserverInput propagation, `calculateManeuver`, `getTrackPositionsMSC`.
  - `initcvmscekf`: <https://www.mathworks.com/help/fusion/ref/initcvmscekf.html> — confirmed 3D state layout `[az; azRate; el; elRate; 1/r; rDot/r]`.
  - `trackingMSCEKF`: <https://www.mathworks.com/help/fusion/ref/trackingmscekf.html> — confirmed `ObserverInput` accepts both 3-element (acceleration) and 6-element (maneuver) forms; canonical example uses the 6-element form via `calculateManeuver`.
  - `trackAssignmentMetrics`: <https://www.mathworks.com/help/fusion/ref/trackassignmentmetrics-system-object.html> — confirmed `'constvel'` extracts position from state elements [1,3,5], `DistanceFunctionFormat='custom'` interface for `AssignmentDistanceFcn(onetrack, onetruth)` callbacks, properties nontunable post-lock.

**v3.7.4 follow-up list.**

- **X-F8 (trackErrorMetrics).** `runTracker.m:200` `trackErrorMetrics('MotionModel', 'constvel')` has the same Cartesian-state assumption as the X-F7 site. Not load-bearing for the Tracked% gate (that comes from `trackAssignmentMetrics`) but affects `posRMS`/`velRMS`/`posANEES` quality summary for MSC tracks — currently empty in test_IRST_airborne console summary. Patch path similar to X-F7 (custom mode + meters-domain distance function). Deferred per the X-F7-only patch contract; doc-fetch `trackErrorMetrics` documentation needed first.
- **Airborne IRST ObserverInput plumbing investigation.** `test_IRST_airborne` reaches the success gate's *runs end-to-end without error AND ≥1 confirmed track* parts (29 scans, 86 tracks, several confirmed) but Tracked%=0%. Bonus signal from v3.7.3 X-F7 diagnostic: track 1 azimuth `−0.063 rad` vs truth bearing `+61°` at scan 2 — range estimate plausible (~6.4 km vs true 5.7 km) but bearing-side init was off. Suggests false-alarm track initialization, or `ObserverInput` plumbing not landing on first-scan correctly. Likely candidates: (1) `prevPose` initialization to scan-1 ownship pose may need offset; (2) `getTrackFilterProperties` cell-array indexing on `trackingGSF`-wrapped filters may need adjustment for `setTrackFilterProperties` write-back; (3) per-scan tracker filter rebinding timing. Triage in v3.7.4.
- **Mixed-sensor visualization.** `dataLog.UsingMSCTracking = any([activeInfos.isIR])` is a scenario-level flag — radar + IR simultaneously would need per-track filter-class inspection for the visualization branch. Post-demo scope.
- **Multi-ownship support.** `ownshipPoseWorld(scenario)` returns all sensor host platforms but downstream consumes only the first (`OwnshipPose(end+1) = ownshipNow(1)`). Sufficient for airborne_IRST's single-ownship geometry but doesn't handle bistatic or multistatic configurations. Post-demo scope.

**Out of scope this release.**

- X-F8 `trackErrorMetrics` fix (deferred to v3.7.4 per X-F7-only patch contract).
- Airborne IRST Tracked%>0% (filter-init/plumbing investigation deferred — empirical evidence of MSC-RPEKF working at stationary geometry is banked as v3.7.3 success).
- v3.6.16 SENSORS Batch 3 cleanup (separate workstream, signaled after v3.7.3 lands).

### v3.7.2 — May 26, 2026 (superseded by v3.7.3)

**X-F6 truth-log axis closure — `targetPoses(observer)` plat-local NED replaced with world-frame helper for moving-ownship support.**

Third release in the X-F6 cross-cutting series. v3.7.0 closed the framework-wide "all sensors emit 3-vec position measurements in world frame" assumption on the **detection axis** (`runDetections.m:401` `minMeas` branch + `trackbenchFilterInit.m` angle-only routing wrapper); v3.7.1 closed it on the **visualization axis** (`runTracker.m:332` `meas(:,jj) = m(1:3)` indexing past 2-vec IR measurements); v3.7.2 closes it on the **truth-log axis** for non-canonical observer geometries. SENSORS' Phase 1 investigation for airborne IRST identified the truth-log frame mismatch as the load-bearing blocker for any moving-ownship scenario; this release ships the fix so SENSORS' v3.6.16 work (`addOwnshipFromDef.m` + `airborne_IRST.json`) can land cleanly on top.

- **Symptom.** Any scenario with a moving sensor host (or stationary host with non-origin Position / non-identity Orientation) would silently produce 0% Tracked% in `runTracker`. Per-scan detection generation, sensor classification, scan log signature, and tracker filter construction completed normally; the failure was localized to `trackAssignmentMetrics` returning empty assignments, which masqueraded as "tracker can't form tracks" rather than "tracks form but can't be associated with truth." Stationary tower at origin with identity orientation worked correctly by accident — the v3.6.5 §1 comment in `loadRunFile.m` documented the assumption explicitly: *"keeping the platform at (0,0,0) means targetPoses(plat) returns target positions in absolute scenario coords."*

- **Root cause.** `runDetections.m` lines 272 and 698 called `targetPoses(activeInfos(1).platform)` to populate `dataLog.Truth`, which `runTracker.m` then fed to `trackAssignmentMetrics` for position-based track↔truth assignment. The R2025b `targetPoses(observer)` doc returns poses in `observer`'s local NED frame, not scenario NED. For airborne ownship with translation + AutoBank/AutoPitch rotation, plat-local ≠ world. Tracker output is scenario-NED Cartesian (via `initrpekf` + `MeasurementParameters.OriginPosition` compensation in the IR path, world-frame natively in the radar path); truth log was observer-local-NED. Mixed frames → assignment metric distances blow up → no truth assignment ever succeeds → 0% Tracked%.

- **Fix.** New local helper `targetPosesWorld(scenario)` at the bottom of `runDetections.m`:
  - Walks `scenario.Platforms` (R2025b doc: cell array of Platform objects — note the indexing nuance below).
  - Filters out sensor-host platforms via `isempty(plat.Sensors)` predicate. Sensor hosts created via `platform(scenario, 'Sensors', sensorList)` in `loadRunFile.m:488` have non-empty `Sensors`; targets created via bare `platform(scenario)` in `addTargetFromDef.m:121` have empty `Sensors`. Clean structural distinction at the right semantic level.
  - For each target platform, calls `pose(plat, 'true')` returning kinematic state in scenario NED (per R2025b doc, contrasted with `targetPoses(observer)` returning observer-local NED).
  - Skips expired-trajectory platforms via NaN-Position check (see hotfix below).
  - Returns a struct array carrying `PlatformID`, `Position`, `Velocity` — exactly the field subset downstream consumers (`runTracker.m:283/285`, `analyzeTrackSwaps.m:213-235`) actually read from `dataLog.Truth`. Other targetPoses fields (`Orientation`, `Acceleration`, `AngularVelocity`, `Signatures`, etc.) are unused downstream — confirmed by empirical grep across the codebase.

  Both call sites updated (line 272 init, line 698 per-scan). Site 3 at line 358 (`targets = targetPoses(plat)` inside the per-sensor visibility-masking + sensor-step loop) **left untouched** — see "Out of scope" below.

- **Iteration history during close — three commits, honest accounting.**
  - **Commit 1 (initial helper + call-site replacements).** Structurally correct fix; broke PosterDemo with `Dot indexing is not supported for variables of this type` runtime error. Iteration mechanism was wrong: `scenario.Platforms` is a **cell array** of Platform objects, not an object array, so `plat = plats(k)` returned a 1×1 cell and the next `plat.Sensors` dot-access failed. Doc-fetch oversight in Phase 1 design — assumed object-array iteration based on the `platform()` constructor's per-call return type, not the property's collection shape.
  - **Commit 2 (cell-array hotfix).** One-character edit: `plat = plats(k)` → `plat = plats{k}`. R2025b `trackingScenario.Platforms` doc confirmed cell-array shape. PosterDemo ran to completion, but T1 (TrkID 1, Truth 1 trajectory expires t=122s) and T4 (TrkID 202, Truth 2 trajectory expires t=264s, T4 lives across that boundary) regressed from numeric posRMS to NaN posRMS. All other tracks bit-identical. Velocity RMS bit-identical for all tracks including T1/T4.
  - **Commit 3 (NaN-skip hotfix).** Diagnosis: the v3.7.0 truth-log code was **designed around** `targetPoses(plat)` dropping expired-trajectory targets from its return, with the slot-matching loop holding the last-cached pose in `truthLastByPlatform` for missing slots (lines 263–272 comment block documents this contract explicitly). The new helper instead emitted NaN-bearing poses for expired targets, per the R2025b `waypointTrajectory.lookupPose` doc: *"If any sample time is beyond the duration of the trajectory, the corresponding pose information is returned as NaN."* Cache poisoned; NaN propagated to all scans past expiry for tracks alive across that boundary. T2 (also a Truth 2 track) was fine because it died before Truth 2's expiry, so saw no NaN scans. Fix: `if any(isnan(p.Position(:))); continue; end` inside the helper restores the drop-expired contract. Bit-identical PosterDemo confirmed.

- **Validation.** PosterDemo bit-identical to v3.7.0 / v3.6.8 baseline (every metric matches scan-by-scan: T1=1233.1m, T2=340.0m, T3=2043.7m, T4=544.9m, T5=4932.4m, T6=1760.7m, T7=1654.7m, Avg=1787.1m, Tracked%=98/98/73, 0 swaps, Truth3 est-fail=27%, all 7 Velocity RMS values, per-scan Primary counts and clutter counts, maxRange=103104m, assignThresh=5155m). `runSingleScenario("test_IRST")` runs end-to-end without error (23 scans, IR pipeline through `trackbenchFilterInit` wrapper into `trackerGNN`, 27 transient tracks, Tracked%=0% as expected for the observability-limited geometry — SENSORS' v3.6.16 closes that with proper airborne scenario design). Three-axis X-F6 closure verified compatible with itself.

**X-F6 closure scope after v3.7.2.** The v3.6.5 §1 comment in `loadRunFile.m` enumerates four consumers of `targetPoses` output that assumed world-frame: (1) terrain occlusion LOS, (2) horizon masking, (3) truth log, (4) tracker association via `dataLog.Truth`. v3.7.0+v3.7.2 close (3)+(4). Consumers (1)+(2) live at `runDetections.m:358` visibility-masking sub-block, left untouched in v3.7.2's tight scope per dispatch and Thursday 11am hard fallback. See "Out of scope".

**Process findings.**

- **Doc-fetch hard rule expanded: "load-bearing assumptions" count.** Project rule (since v3.6.x) was "doc-fetch the R2025b SDK doc BEFORE designing fixes that touch SDK class behavior." v3.7.2 demonstrated that "behavior we depend on as load-bearing" also counts — even when we don't directly touch it. Phase 1 design claimed `pose(plat,'true')` returns "the platform's last position" after trajectory expiry (citation: vague "MATLAB doc"); the actual R2025b doc says NaN, and the PosterDemo regression caught it. Methodology elaboration banked: **doc-fetch any property/method behavior load-bearing to a fix design, not just the surface being touched.** Future regressions of this shape are now anticipated.

- **Dispatch enumeration is a hint, not a contract.** The v3.7.2 dispatch listed "at least 2" `targetPoses` calls and gave the truth-log axis as scope. Empirical `grep` inventory during Phase 1 surfaced a **third call site at line 358** — inside the per-sensor visibility-masking + sensor-step loop — with a related but architecturally distinct bug shape (the sensor step *correctly* wants plat-local `targets` per `fusionRadarSensor.step` / `irSensor.step` API contracts, the visibility-masking sub-block *incorrectly* pairs plat-local target positions with world-frame `sensorPos`). Site 3 was flagged in the read report, framed as "Framing A vs Framing B" scope decision, and explicitly deferred to v3.7.3 contingent on v3.6.16 empirical. Without the empirical inventory, the dispatch's enumeration would have been taken as exhaustive and a partial X-F6 closure would have shipped.

- **MCP timeout safety: empirical state check before retry.** The initial three-edit commit (helper insertion + two call-site replacements) timed out at 4 minutes with no result. Could have shipped, could have failed, could have partially applied. Naïve retry would have been unsafe — if hunks 1+2 had been consumed but hunk 3 was still pending, the helper-insertion anchor (`function isMSSR = classifyAsMSSR(sensor, metas)`) would still be present, and retry would have applied hunk 3 again, producing a duplicate helper function. Re-pull confirmed pristine pre-commit state (all three anchors `grep -c = 1`, file unchanged at 823 lines) — safe to retry, which then succeeded. Rule banked: **anchor-uniqueness empirical check is necessary before initial commit AND before any retry after timeout. Partial application is theoretically possible and would produce duplicate helpers on retry.**

- **Cell-array vs object-array iteration shape — doc-check the property, not just the constructor.** `platform(scenario)` constructor returns a Platform object (used in `addTargetFromDef.m:121`). `scenario.Platforms` property returns a cell array of Platform objects (R2025b doc: *"Platforms in the scenario, returned as a cell or cell array of Platform objects"*). Different shapes, same element type — the constructor doc was previously fetched and verified, the property doc was not. Cost: one runtime error on first PosterDemo. Rule banked: **doc-check shape of any property being iterated, not just the constructor that produces individual elements.**

**Out of scope this release (deferred).**

- **Site 3 visibility-masking world-framing** — `runDetections.m:358` third `targetPoses(plat)` call. The sensor step `si.sensor(targets, ins, simTime)` correctly wants `targets` in plat-local frame; the visibility-masking sub-block in the same iteration incorrectly pairs `targets(tt).Position` (plat-local) with `sensorPos = ins.Position + sensor.MountingLocation` (world frame). Dormant for stationary-tower-at-origin (plat-local ≡ world); will silently surface as wrong occlusion / horizon-mask counts the first time an airborne scenario runs with either degradation on. SENSORS' v3.6.16 `test_IRST.json` and `airborne_IRST.json` should keep `terrain_occlusion: false` + `horizon_masking: false` for the demo path. **v3.7.3 candidate** contingent on v3.6.16 empirical surfacing wrong occlusion/horizon counts.

- **Velocity-via-fallback-only at `runTracker.m:284`.** `isprop(targets(k),'Velocity')` returns false for STRUCT inputs in R2025b (empirically: Velocity RMS bit-identical between the old `targetPoses` struct output and the new `targetPosesWorld` struct output, confirming the `else` branch fires in both cases), so `truths(k).Velocity` is hard-coded to `[0 0 0]` for every truth row. Has been true since the runTracker code was written; not a v3.7.x regression. Tracker assignment uses position-based distance only by default, so this doesn't degrade Tracked% in PosterDemo. Cleanup post-demo: replace `isprop` with `isfield` (struct-compatible) so future scenarios that want velocity-weighted assignment can consume world-frame truth Velocity (the helper provides it correctly — it's just unread).

- **Cartesian-ize IR bearing-only detections for raw-detection visualization** — from v3.7.1 out-of-scope. Framework feature.

- **Extract shared `isAngleOnlyDetection` predicate helper** — from v3.7.1 out-of-scope. Post-demo cleanup.

**CRLF→LF artifact list unchanged.** `runDetections.m` is LF and stayed LF through v3.7.0, v3.7.1, v3.7.2 edits. 7 files total (`drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`, `buildSensor.m`, `buildTracker.m`, `default_IRST.json`).

### v3.7.1 — May 26, 2026 (superseded by v3.7.2 — README entry deferred during demo-prep cascade)

**X-F6 visualization axis closure — `runTracker.m:332` `meas(:,jj) = m(1:3)` indexing past 2-vec IR measurements.**

Second release in the X-F6 cross-cutting series, sealed Tue 2026-05-26 PM but with README + CHECKPOINT.md updates deferred during the same-day pivot to v3.7.2 (orchestrator's Phase 1 investigation for airborne IRST surfaced the truth-log axis bug downstream of v3.7.0; README cascade caught up in v3.7.2). Caught when running `runSingleScenario("test_IRST")` as the v3.6.15 success gate: the pipeline produced 23 scans of valid 2-vec `[az;el]` IR detections, all routing through v3.7.0's `trackbenchFilterInit` wrapper correctly into `trackerGNN`, then errored at `runTracker.m:332`.

- **Symptom.** Any scenario using an IR sensor with `Frame='Spherical'` + `HasRange=false` (R2025b `irSensor` default) errored at `runTracker.m:332` during the `if showVisuals` block's raw-detection 3D scatter construction. v3.7.0 closed the detection path (line 401 `minMeas` branch accepting 2-vec for IR) and the tracker filter path (`trackbenchFilterInit` routing to `initrpekf` for angle-only detections), but the visualization-side `meas` matrix construction at lines 326-333 unconditionally hard-indexed `m(1:3)` for every detection, including the 2-vec IR ones the pipeline now correctly accepts upstream. Confirmed tracks (which include IR-contributed state via `getTrackPositions` at line 336) plotted normally; raw-detection scatter (line 340 `addpoints(detLine, meas(1,:), meas(2,:), -meas(3,:))`) blew up at the `(1:3)` indexing of 2-vec measurements.

- **Root cause.** Same X-F6 framework assumption extending into the tracker analysis phase, not just the detection pipeline. The `if showVisuals` block at `runTracker.m:325-353` was the only remaining site in tracker-analysis code that assumed 3-vec position vectors per detection. Other sites in `runTracker.m` (`normalizeDetectionDimensions` at line 626 gated on `nMeas > 3`, `transformToSensorFrame` helper at 602-617 reading `MeasurementParameters` for `OriginPosition`/`Orientation` only) were already safe.

- **Fix.** Single-site edit at `runTracker.m:325-353`. Filter `scanCells` to keep only detections with `numel(Measurement) >= 3` before the `meas` matrix construction, building `plotCells = scanCells(keepDet)` first. 2-vec IR detections are skipped from the raw-detection scatter (they can't be plotted as Cartesian XYZ on the theaterPlot anyway, since their measurement frame is sensor-local spherical without range — Cartesian-izing would require a fixed-range bearing ray, framework feature deferred to post-demo). Confirmed tracks that consume the IR detections via `getTrackPositions` plot normally — the visualization path for tracks is independent of the scatter path for raw detections.

  **`numel(m) >= 3` shape guard chosen over the full `Frame='Spherical' && ~HasRange` predicate** for consistency with v3.7.0's `runDetections.m:401` `minMeas` pattern (same family of "shape gate before indexing"), and for robustness — any future sensor type emitting angle-only or odd-shaped measurements (sonar bearings-only, AoA-only) automatically falls through correctly without needing a predicate update.

- **Validation.** PosterDemo bit-identical to v3.7.0 baseline (all PosterDemo detections are 3-vec radar, `keepDet(jj)=true` for all, behavior identical, T1/T2/T3 match scan-by-scan). `runSingleScenario("test_IRST")` ran end-to-end without error, the v3.6.15 success gate.

**Process finding.**

- **Dry-run diff review caught indent miscount before commit.** First v3.7.1 dry-run had the new comment block at 8 spaces while the surrounding else-branch content was at 12. The `oldText`/`newText` matched whitespace-tolerantly during anchor lookup but pasted `newText` verbatim, leaving indentation drift visible in the diff output. Re-craft with correct 12-space base indent (matching the file's 8/12/16/20 progression for that block) produced a clean diff. **Rule reinforced: byte-level indent verification via `cat -A` against the actual file is mandatory for any insertion deeper than the file's top level.**

**Out of scope (deferred to post-demo).**

- **Cartesian-ize IR bearing-only detections for raw-detection scatter** — framework feature requiring fixed-range bearing ray from sensor's `OriginPosition` + `Orientation`. Not a demo-week blocker.

- **Extract shared `isAngleOnlyDetection` predicate helper** — currently the angle-only classification (`Frame='Spherical' && ~HasRange` against 4-shape `MeasurementParameters` defense) lives in `trackbenchFilterInit.m`. v3.7.1's `numel >= 3` shape guard at line 332 is intentionally simpler (the visualization site only needs "is this Cartesian-plottable?", not "is this angle-only?"). Post-demo cleanup if a third call site appears.

**CRLF→LF artifact list unchanged.** `runTracker.m` was already LF; remained LF. List stays at 6 files at v3.7.1 close (the 7th, `default_IRST.json`, was added in v3.6.15 which landed concurrently).

### v3.7.0 — May 26, 2026 (superseded by v3.7.1)

**IRST gate 6 closure — `runDetections` 2-vec angle-only measurements accepted; `+tracking/` wrapper routes IR detections to `initrpekf` (`trackingGSF` range-parameterized EKF bank).**

This release closes the framework-wide gap identified as cross-cutting finding **X-F6** (v3.6.12 README entry): the codebase declared four IR sensor types (`IRST`, `IR_STARING`, `FLIR`, `CUSTOM_IR`) but the tracker pipeline was hard-coded for 3-vec position-vector measurements at two distinct gates — the angle-only detection filter in `runDetections.m` and the `FilterInitializationFcn` in `buildTracker.m`. After SENSORS confirmed in v3.6.14 that `irSensor` produces valid 2-vec `[az;el]` Measurements with full `MeasurementParameters` per the R2025b `irSensor` doc, the remaining work was glue: routing those detections past the filter and into a tracker filter that can construct angle-only innovation. MATLAB's `initrpekf` is the supplied infrastructure (R2025b doc example matches our use case verbatim: `Frame='spherical'`, `HasRange=false`, 2-vec `[az;el]` detection → `trackingGSF`); this release wires it in. First entry on the BUGHUNT v3.7.x version line (SENSORS continues v3.6.x per `HANDOFF_COORDINATION.md`).

- **Symptom.** IRST end-to-end produced zero tracks. Detections were silently dropped at `runDetections.m` line 396 (the `numel(dets{ii}.Measurement) < 3` filter); even if they had made it past, the tracker's `FilterInitializationFcn` (a CV/IMM lambda baked in `buildTracker.m`) would have errored at construction trying to read a 3-vec position from a 2-vec `[az;el]` Measurement. Both gates needed to change, and they sit in different ownership scopes — `runDetections.m` in `+detections/`, the init function in `+tracking/`. Phase A3 IRST validation (SENSORS v3.6.14) confirmed gates 1, 2, 4 ✅ (build, introspect, raw detection), gates 3, 5, 7 ⏳ pending non-blocking, and gate 6 ❌ blocked here.

- **Root cause.** The framework was originally written assuming all sensors emit position-vector measurements; the line 396 hard cut and the `initCVFilter` / `initIMMFilter` 3-vec assumption together encoded that assumption in two places. Cross-cutting finding **X-F6** from the v3.6.12 README entry called this out: the `buildRadar` / `buildSonarSensor` `DetectionCoordinates='Scenario'` / `'Sensor spherical'` setters made it explicit for radar/sonar; `buildIR` originally omitted it (also touched in v3.6.12, then re-discovered to be a SetAccess-read-only no-op in v3.6.14, leaving `irSensor` on its R2025b default `'Sensor spherical'` + 2-vec `[az;el]`). The fix at the line-396 site is one branch; the tracker-side fix is routing angle-only detections to `initrpekf` instead of the CV/IMM lambda. Both are project-side framework plumbing, not novel filter math.

- **Fix.** Four surgical changes across three files:

  1. **`runDetections.m` line ~390 (Edit 3a)** — the `numel(Measurement) < 3` hard cut becomes `numel(Measurement) < minMeas` where `minMeas = 3` by default and drops to `2` when `si.isIR` is true (using the existing IR classification computed at line 162). Non-IR sensors with <3-vec measurements are still treated as malformed and dropped, unchanged from prior behavior.

  2. **`runDetections.m` lines ~488 and ~506 (Edits 3b, 3c)** — defensive `if numel(dets{dd}.Measurement) < 3; continue; end` guards added to both weather-degradation loops (Pd drop, noise scaling). The weather block is gated `(si.isRadar || si.isIR)` per the v1.3 SENSOR_VALIDATION_MATRIX correction (matrix Doppler-gating doc-bug also fixed there), so IR detections do enter the block; the guards skip them cleanly because they have no `(x,y)` for the per-region resolve and no slant range for the Pd lookup. Proper IR-aware weather (fog primarily — rain has negligible IR effect) is post-demo scope.

  3. **NEW `+tracking/trackbenchFilterInit.m`** — detection-aware filter init router. Branches on `MeasurementParameters.Frame == 'Spherical' && ~MeasurementParameters.HasRange` → `initrpekf(detection, 6, [1e3 5e4])` (6 EKFs over 1–50 km range hypotheses, sized for tactical IRST geometry, returns `trackingGSF`); else falls through to a `baseInitFcn` closure-captured argument (the configured CV/IMM lambda from `buildTracker.m`). Defends four `MeasurementParameters` shapes: empty (default), single struct, cell array of structs, and **struct array** — `irSensor` emits a 2×1 struct array (outer spherical frame + inner rectangular, sensor-side first). The `mp = mp(1)` line picks the sensor-side first frame; without it, accessing `mp.Frame` on a struct array expands to a comma-separated list and `strcmpi` errors at runtime. See "Process findings" below — this shape was caught by code review against SENSORS' empirical v3.6.14 observation, not by dry-run.

  4. **`buildTracker.m`** — rename `initFcn` → `baseInitFcn` for the CV/IMM lambda; add `initFcn = @(det) trackbench.tracking.trackbenchFilterInit(det, baseInitFcn)` wrapping closure. The tracker constructors (`trackerGNN`, `trackerTOMHT`, `trackerJPDA`) consume the wrapper, which is bit-identical to the bare lambda for any detection lacking the spherical+no-range combo.

- **Validation expected.** PosterDemo bit-identical to v3.6.14 / v3.6.8 baseline. PSR detections from `fusionRadarSensor` produce 3-vec `Measurement` with `MeasurementParameters` either empty or `Frame='Rectangular'` (per `DetectionCoordinates='Scenario'`); both gates fall through unchanged: `si.isIR=false` → `minMeas=3` → identical filter behavior; `isAngleOnly=false` in the wrapper → `baseInitFcn(det)` called identically to the pre-v3.7.0 lambda. **Confirmed empirically before this entry was written:** T1=1233.1m, T2=340.0m, T3=2043.7m, Avg=1787.1m, Tracked%=98/98/73, 0 swaps, Truth3 est-fail=27%; per-scan log signature also identical scan-by-scan to the v3.6.7 baseline. Gate-6 empirical closure (a standalone inline script wiring `trackerGNN` + IRST + scan-envelope override + low-altitude target through the new wrapper to confirm a track is generated and confirmed) is the immediately following work, not part of this commit; if successful, SENSORS workstream takes the matrix-row upgrade from DEMO-READY-LIMITED → DEMO-READY (their ownership per `HANDOFF_COORDINATION.md`).

**X-F6 closure scope.** This release resolves the cross-cutting framework assumption *for the IR path*. The IR sensor side (DetectionCoordinates default, signature handling) was closed by v3.6.13 (auto-attach `irSignature` for forward-compat). The tracker side is closed here. **Sonar and lidar paths remain doc-trusted only** — `buildSonarSensor`'s line 675 `DetectionCoordinates` set and `buildLidar`'s line 735 equivalent are not empirically exercised by any current scenario (Maritime sonars are skipped by `runDetections` per the v3.6.8 known-issue note; lidar has never been run end-to-end through a tracker). Both remain on the post-demo cleanup list. If either SDK class shares `irSensor`'s read-only `DetectionCoordinates` behavior, those `safeSet` calls will error when first exercised by a real scenario; the dormant risk is unchanged from v3.6.14.

**Process findings.**

- **Struct-array shape of `irSensor` MeasurementParameters caught by code review, not by dry-run.** First draft of `trackbenchFilterInit.m` handled empty / single-struct / cell-array shapes but not struct-array. Orchestrator review caught it from SENSORS' empirical v3.6.14 observation that `irSensor` emits `2×1 struct array` (outer spherical + inner rectangular). Without the `mp = mp(1)` fix, the wrapper would have errored silently at dry-run (PSR detections never enter the angle-only branch) and only failed at the first IRST detection. Adds to the v3.6.x pattern: PosterDemo canary bit-identical is necessary but not sufficient evidence of correctness for code paths PosterDemo doesn't exercise.

- **`buildTracker.m` joins the CRLF→LF artifact list as the 6th file.** Previously CRLF (120 CR characters); `Filesystem:edit_file` converted to LF on the wrapping edit, as it has for every prior file that crossed it. Full list (6 files): `drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`, `buildSensor.m`, `buildTracker.m`. Review these with `git diff -w` to filter line-ending noise.

- **Empirical anchor-uniqueness verification (project hard rule since v3.6.14) worked as designed.** Both `runDetections.m` `Measurement(1:3)` lines are identical in content but at different indentation; the bare line is NOT a unique anchor. The disambiguating comment headers above each loop (`% Drop detections using range-dependent Pd`, `% Scale measurement noise (re-resolve`) ARE uniquely `grep -c = 1` and were the actual anchors used. Pattern-recognition against memory would have picked the bare line and produced a non-unique-anchor failure on the first dry-run attempt.

**Out of scope this release (deferred to post-demo or separate workstream).**

- `trackerJPDA` and `trackerTOMHT` consumption of `trackingGSF` — only `trackerGNN` is doc-confirmed (R2025b `initrpekf` example explicitly wires through `trackerGNN`). PosterDemo and the gate-6 empirical use GNN; JPDA/TOMHT + IR is post-demo.
- IRST-F1 (`getDefaults('IRST')` dead defaults — `pd`, `rangeLimits`, `rangeRes`, `hasRangeRate` that `irSensor` doesn't have) — SENSORS-side cleanup, deferred.
- `runDetections.m` line 386 `try [dets, ~, sensorCfg] = si.sensor(...); catch; continue; end` silent-typo-masker pattern — Phase 2 BUGHUNT audit scope.
- Proper IR-aware weather degradation (fog physics for `irSensor`'s lens-optics sensitivity model) — post-demo, framework work.

### v3.6.14 — May 25, 2026 (superseded by v3.6.15)

**Revert v3.6.12 — `irSensor.DetectionCoordinates` is read-only in R2025b; clarify v3.6.13 framing.**

This release is a doc-fetch-gap correction. The v3.6.12 design assumed `irSensor.DetectionCoordinates` was both writable and would convert `Measurement` from `[az; el]` to a 3-vector position — premises that an R2025b `irSensor` doc fetch would have invalidated had it been performed before commit, per the project hard rule "Doc lookup is MANDATORY before approving any fix that touches SDK class behavior." The doc fetch happened after v3.6.12 commit, during Phase A3 G2-round-2 introspection; it surfaced findings that materially affected the plan, and empirical confirmation showed every `buildSensor(idx, 'IRST' | 'IR_STARING' | 'FLIR' | 'CUSTOM_IR')` call had been failing at construction since v3.6.12 with a `MATLAB:class:SetProhibited` error in the `safeSet(ir, 'DetectionCoordinates', char(S.detCoords))` call. The bit-identical PosterDemo canary that "confirmed v3.6.12 safe" was false reassurance: PosterDemo is a PSR-only scenario, never exercises the `buildIR` code path, and the construction error was invisible to it. The canary correctly measured what it tested; the gap was canary coverage, not canary execution.

**v3.6.12 revert.**

- **Symptom.** Every `buildSensor(idx, 'IRST' | 'IR_STARING' | 'FLIR' | 'CUSTOM_IR')` call errored at construction with `MATLAB:class:SetProhibited` during the v3.6.12-introduced `safeSet(ir, 'DetectionCoordinates', char(S.detCoords))` call at `+trackbench/+sensors/buildSensor.m` line 559. All four IR sensor types were completely non-functional in v3.6.12 and v3.6.13. PosterDemo canary on v3.6.12 reported bit-identical numerics because it never built an IR sensor, masking the failure.
- **Root cause.** v3.6.12 mirrored `buildRadar`'s `DetectionCoordinates` plumbing pattern onto `buildIR` under the premise that the property worked symmetrically across both SDK classes. The R2025b `fusionRadarSensor` doc confirms `DetectionCoordinates` is writable on that class; the R2025b `irSensor` doc lists `Measurement` as `[az; el]` for `HasElevation=true` and does not list `DetectionCoordinates` as a configurable property at all — the property exists at runtime (`isprop(ir, 'DetectionCoordinates')` returns TRUE) but is read-only. `safeSet`'s implementation checks `isprop` for existence but not `findprop(obj, name).SetAccess` for writability; the assignment was attempted and raised. `safeSet` has no try/catch wrap, so the error propagated up through `buildIR` and `buildSensor` to the caller. The SetAccess picture across SDK classes used in the framework: `irSensor`'s `DetectionCoordinates` is read-only (empirically confirmed this session); `fusionRadarSensor`'s is writable (empirically validated by every PSR canary in the project's history). `sonarSensor`'s and `lidarPointCloudGenerator`'s SetAccess hasn't been empirically tested in this codebase — their build paths exist but no current validation scenario exercises them. The empirical-vs-doc-trusted split is worth recording for future SDK plumbing decisions.
- **Fix.** Remove the two lines added by v3.6.12: `addParameter(p, 'detCoords', 'Scenario')` at line ~534 of `buildIR`'s inputParser, and `safeSet(ir, 'DetectionCoordinates', char(S.detCoords))` at line ~559 of `buildIR`'s safeSet block. Returns `buildIR` to its pre-v3.6.12 state. `buildRadar`'s `DetectionCoordinates` set at line 491 is empirically validated by every PosterDemo PSR canary. `buildSonarSensor`'s line 675 and `buildLidar`'s line 735 sets remain in code, doc-trusted but not empirically exercised by any scenario in this codebase (per v3.6.8 audit note: "Maritime scenarios build sonar sensors but runDetections skips them"). If either SDK class shares `irSensor`'s read-only behavior, the bug is currently dormant — won't surface until those paths are exercised.
- **Validation expected.** PosterDemo bit-identical to v3.6.8 baseline (PSR-only, never calls `buildIR`, so revert is invisible to this canary). Confirmed empirically before this entry was written: T1=1233.1m, T2=340.0m, T3=2043.7m posRMS, Avg=1787.1m, Tracked%=98/98/73, 0 swaps, Truth3 est-fail=27%. IRST construction now succeeds; empirical observation of `Measurement` vector format is the next step in Phase A3 validation.

**v3.6.13 re-framing (no code change, clarification only).**

The v3.6.13 entry described the auto-attached default `irSignature` as the second half of unblocking `irSensor` detection in `runDetections`. Per the R2025b `irSensor` doc fetched during this release's investigation, that framing is inaccurate for the canonical `irSensor` class. Its sensitivity model is **lens-optics-based**: detection is governed by `Detectivity`, `DetectorArea`, `FocalLength`, `LensDiameter`, `NoiseEquivalentBandwidth`, and target apparent radiance derived from `Position` — not from `Platform.Signatures{}` entries. R2025b documentation examples produce detections from bare `(PlatformID, Position)` targets with empty `Signatures`. Concretely: the `irSignature` auto-attached by v3.6.13 is **not load-bearing** for the canonical `irSensor` class. That sensor would have detected `Platform.Signatures = {}` targets via its lens-optics path equally well.

The auto-attach is **forward-compatibility plumbing** for custom IR sensor classes (e.g. user-authored subclasses of `irSensor` or `matlab.System` that choose to consume `Platform.Signatures{}` to compute detection probability against an explicit IR signature). v3.6.13 remains a valid release: the auto-attach costs ~zero (one cell-array append per target) and provides forward-compat for custom sensors that we may add post-demo. The PosterDemo canary on v3.6.13 correctly confirmed `fusionRadarSensor`'s `isa()`-based `Signatures` enumeration is order-independent (the cosmetic `irSignature` in slot 2 has no effect on RCS-based radar physics) — that confirmation is still valid; just don't read the v3.6.13 entry as "this is what unblocks canonical `irSensor` detection."

**A3 IRST path forward.** The combination of v3.6.12 revert + v3.6.13 re-framing reveals that the real tracker-pipeline integration gap is at `runDetections.m` line ~398 (the `numel(Measurement) < 3` hard cut, which drops every angle-only IR detection by design) plus the absence of an angle-only tracker init path in `+tracking/`. That's framework work, BUGHUNT-scope, deferred to v3.7.x. A3 IRST validation pivots to **gates 1–4 only** (build, classify, scan-mechanics, raw detection generation outside the tracker pipeline); gates 5–7 (tracker integration through to track output) are explicitly out of scope until BUGHUNT lands the framework changes. The matrix entry for IRST will reflect this scope split when written.

**Notes for the CHECKPOINT proposal pool.**

Three process findings carry forward from this round-trip:

- **safeSet `isprop`-only weakness.** Current `safeSet(obj, propName, value)` in `buildSensor.m` checks `isprop(obj, propName)` before assignment but not `findprop(obj, name).SetAccess`. Read-only properties pass the gate, then error at assignment, then propagate uncaught (no try/catch). Same shape of issue as the Category A try/catch typo-mask pattern BUGHUNT is hunting — a check that looks comprehensive but silently masks a category of failure mode. Post-demo cleanup: add a `findprop`-based writability check, or wrap the assignment in a typed try/catch catching `MATLAB:class:SetProhibited` specifically (re-raising anything else, so legitimate value-type errors aren't masked).
- **Canary coverage gap.** PosterDemo canary validates only what PosterDemo exercises (PSR build + GNN tracker + mountain terrain). Orthogonal sensor build paths (IR, MSSR, sonar, lidar, ADS-B) require orthogonal smoke-tests. Recommendation post-demo: add `scripts/test_all_sensor_builds.m` that calls `trackbench.sensors.buildSensor(idx, type)` for each declared type and reports success/failure with the resulting object's class name. Pure construction test, no scenarios run, ~5 seconds wall clock. Would have caught v3.6.12 in seconds rather than via the doc-fetch + empirical loop that took this session.
- **Anchor-uniqueness empirical verification (now project hard rule).** The existing rule "anchor on unique single-line statements only" was mental discipline; per orchestrator escalation this turn, `grep -c` (or equivalent empirical search) of the anchor against the current file state is now required before every dry-run. Two near-misses in one session — v3.6.12's commit matched `buildIR` uniquely only by luck (`buildRadar` happened to have `detCoords` between its `hasINS` and `mountingLoc` lines that `buildIR` didn't), and the original v3.6.14 revert dry-run would have removed `buildRadar`'s pre-existing line 430 `detCoords` if the diff function-context hadn't been reviewed carefully — set the bar. Anchors that were unique in a prior edit may have become non-unique due to intervening changes; the only safe assumption is to verify current uniqueness empirically.

**CRLF→LF artifact list unchanged.** `buildSensor.m` was already on the list from v3.6.12; this revert kept it LF (verified post-commit: 0 CR characters in file). Five files total.

### v3.6.13 — May 25, 2026 (superseded by v3.6.14)

**Targets had no `irSignature` — `addTargetFromDef` now auto-attaches one alongside the RCS signature.**

- **Symptom.** Even after v3.6.12 fixed `buildIR`'s `DetectionCoordinates` plumbing so IR detections survive the `runDetections.m` ~line 398 angle-only filter, an irSensor pointed at any target built through the scenario pipeline still produced **zero detections** — because the targets themselves had no IR signature for the sensor to detect. Platforms came out of `addTargetFromDef` with `Signatures = {rcsSignature(...)}` (a 1-element cell) when RCS was configured, or with the documented `Platform.Signatures = {}` default when neither `rcs_dbsm` nor `rcs_profile` was specified. Either way: nothing for `irSensor` to detect. The two existing IR-themed target presets (`default_ir_radar_fusion.json`, `default_ir_degraded_weather.json`) both fell in the second case — their JSON declares targets but never set RCS or IR signature fields, so post-`addTargetFromDef` they had empty `Signatures` cells.
- **Root cause.** `addTargetFromDef.m` (the universal target-builder used by `loadRunFile` for every target across every scenario) has a `Signatures` block at lines 125–141 that handles `rcs_dbsm` and `rcs_profile` only — no IR-related schema field, no `irSignature` attached automatically. The build sub-functions (`buildRecordedFlight`, `buildFromWaypoints`, `buildConstantVelocity`, `buildGentleTurn`, `buildSManeuver`, `buildCrossing`) only return `[wp, t, vel]` and don't touch signatures, so the gap is single-site. This is a second F2-shape gap parallel to v3.6.12's `buildIR` issue: the framework declared four IR sensor types but only ever plumbed the sensor side of the IR detection pipeline, never the target side. Identified during Phase A3 G3 prep when picking a target preset for `test_IRST.json` — the natural follow-up question "do existing IR-themed presets actually have IR signatures?" had a no-hit `grep` answer in `addTargetFromDef.m`.
- **Fix.** Append a default `irSignature()` to every target's `Signatures` cell unconditionally, immediately after the existing RCS block (insertion at lines 144–161). `fusionRadarSensor` and `sonarSensor` enumerate `Signatures` by `isa()`-match (documented SDK behavior — see e.g. the `fusionRadarSensor.step` documentation), so a trailing `irSignature` is inert for radar/sonar physics. Construction defended with a try/catch fallback to `irSignature('Pattern', 1000)` for R2025b API uncertainty about the no-arg constructor. The fetch of `tgt.Signatures` is **not** wrapped in try/catch — `Platform.Signatures` is a documented property that always returns a cell array (empty `{}` if unset), and the BUGHUNT workstream is actively hunting exactly the kind of silent typo-masking try/catch pattern we'd be introducing.
- **Validation expected.** PosterDemo bit-identical to v3.6.8 baseline. PosterDemo's three radar targets will now have `{rcsSignature, irSignature}` in their `Signatures` cells — if `fusionRadarSensor` enumerates by `isa()` match (documented), bit-identical numerics. **If anything drifts: that is an SDK behavior discovery** (`fusionRadarSensor` has hidden index-1-dependence on `Signatures{1}` rather than `isa()`-matching), **not a regression** to revert. The reversion direction would be either prepending the `irSignature` instead of appending, or switching to an opt-in `ir_signature` schema field with per-JSON authoring — both are larger architectural choices requiring orchestrator triage rather than unilateral SENSORS adjustment.

**Other notes.**

- **Unblocks A3 IRST end-to-end validation.** v3.6.12 fixed the sensor wiring; v3.6.13 fixes the target wiring. Both are required for an irSensor to produce non-zero detections in `runDetections`. The G3–G6 empirical pass via `config/runs/test_IRST.json` + a new low-altitude target preset (Path A from this session's investigation) follows in the next substep, pending the canary clearing.
- **Architectural framing.** v3.6.12 and v3.6.13 together close the original framework-design gap identified as X-F6 (scope-of-framework: IR sensor types declared but tracker-pipeline integration incomplete). Bearings-only IR tracking (where targets are detected by direction only, no range, requiring a range-parameterized or angle-only EKF tracker filter) remains explicitly out of scope; the v3.6.12 `'Scenario'` default produces 3-vector position measurements that the existing tracker pipeline consumes natively, which is the framework's intended IR detection model.
- **CRLF→LF artifact list unchanged.** `Filesystem:edit_file` was tested empirically on this `addTargetFromDef.m` edit; the file was already LF and remains LF. Five files total on the CRLF-converted-by-edit-file list (`drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`, `buildSensor.m`).
- **No schema additions.** Existing target JSONs across `config/targets/**` continue to work unchanged. The `ir_signature` schema field mentioned in the inline comment is a deliberate post-demo follow-up, not a current requirement — every target gets a default IR signature regardless of whether its JSON acknowledges IR.

### v3.6.12 — May 25, 2026 (superseded by v3.6.14, reverted)

**IRST / IR_STARING / FLIR / CUSTOM_IR silently produced angle-only measurements — `buildIR` now matches `buildRadar`'s `DetectionCoordinates` plumbing.**

- **Symptom.** The framework has declared four IR sensor types (`IRST`, `IR_STARING`, `FLIR`, `CUSTOM_IR`) since the universal sensor factory landed, and the gating logic correctly routes IR sensors into the weather-degradation path (`runDetections.m` line ~437, gated `si.isRadar || si.isIR`). But the tracker pipeline received **zero IR detections in practice** — every IR detection was dropped before the buffer flush, at the angle-only filter `runDetections.m` ~line 398 (`numel(dets{ii}.Measurement) < 3`). End-to-end an IRST scenario looked indistinguishable from a misconfigured-geometry failure ("Only N scan(s)" warning) when the wiring was the real cause.
- **Root cause.** `buildRadar` plumbs `DetectionCoordinates` via its `inputParser` (`addParameter(p, 'detCoords', 'Scenario')` line ~462) and applies it via `safeSet(radar, 'DetectionCoordinates', char(S.detCoords))` (line ~491). `buildSonarSensor` does the same (line ~675). `buildIR` omits **both** — no `detCoords` parameter, no `safeSet` call. `irSensor` therefore falls back to its R2025b default `DetectionCoordinates = 'Sensor spherical'`, which combined with `HasElevation = true` and no range estimation produces 2-element `[az; el]` angular measurements. The line 398 hard cut at `numel(Measurement) < 3` then drops every IR detection silently. Hidden since the IR sensor types were declared because no test scenario had ever exercised the full IRST → buffer → tracker path through the actual `runDetections` pipeline. Phase A3 G1 inspection surfaced the asymmetry between `buildRadar`'s and `buildIR`'s `DetectionCoordinates` handling; G2 introspection confirmed `DetectionCoordinates='Sensor spherical'` empirically.
- **Fix.** Add `'detCoords'` parameter (default `'Scenario'`) to `buildIR`'s `inputParser` between `'hasINS'` and `'mountingLoc'`, exactly mirroring `buildRadar`'s declaration; add `safeSet(ir, 'DetectionCoordinates', char(S.detCoords))` between the `HasINS` and `MountingLocation` `safeSet` calls. Two-line patch, isolated to `buildIR` (lines ~538–602). `buildRadar`, `buildSonarSensor`, `buildLidar`, ADS-B builders, all `getDefaults` blocks untouched. Pre-existing IRST/IR_STARING/FLIR/CUSTOM_IR JSONs (`config/sensors/IRST/default_IRST.json` etc.) are unaffected — they don't override `params.detCoords`, so the new `'Scenario'` default kicks in. Users wanting different coordinates can now set `"detCoords": "Sensor rectangular"` (or similar) under `params` in their sensor JSON.
- **Validation expected.** PosterDemo bit-identical to v3.6.8 baseline (PSR-only scenario, no IR involvement, `buildIR` never called — confirmed empirically before this entry was written: T1=1233.1m, T2=340.0m, T3=2043.7m posRMS, Tracked%=98/98/73, 0 swaps, Truth3 est-fail=27%). New `config/runs/test_IRST.json` smoke-test exercises IRST end-to-end: should now produce non-zero `Primary=N` counts across scans in the `runDetections` log, where v3.6.11 would have produced `Primary=0` on every scan despite the IRST being correctly built.

**Other notes for the matrix.**

- **Two new IRST findings cataloged.** **IRST-F1** — dead defaults in `getDefaults('IRST')` (`D.pd`, `D.rangeLimits`, `D.rangeRes`, `D.hasRangeRate` never reach the sensor because `irSensor` doesn't have those properties; `safeSet` not even attempted; same shape as MSSR-F2 but silent rather than inert). Verified via `isprop` returning 0 for all four in G2 introspection. Deferred to post-demo cleanup. **IRST-F2** — the `DetectionCoordinates` plumbing gap fixed in this release.
- **`SENSOR_VALIDATION_MATRIX.md` Phase A3 row update will follow on successful `test_IRST` run** (this release fixes the wiring, not the verification). The matrix row addition is part of the upcoming SENSORS-workstream `[CHECKPOINT PROPOSAL]` to the orchestrator, after G3–G6 empirical verification this session.
- **Matrix Doppler-gating doc bug corrected.** The matrix's existing prose (Phase A step 3 row, also referenced in `docs/HANDOFF_SENSORS.md`) states "the codebase already gates Doppler and weather degradation to `isRadar || isIR`." Empirical reading of `runDetections.m` shows this is only half-true: **weather** is gated `(si.isRadar || si.isIR)` at line ~437 ✓ matches matrix; **Doppler fade** is gated `si.isRadar` at line ~417 ✗ matrix wrong — IR is excluded, which is physics-correct (IR doesn't measure radial velocity). The actual gating across all four post-step filters is: RCS filter radar-only, Doppler fade radar-only, weather degradation radar+IR, ground clutter radar-only (driven by the first primary radar). All four are physics-correct as implemented; the matrix doc was the source-of-truth error, not the code. Correction will land in the matrix as part of the checkpoint package.
- **New cross-cutting finding X-F6 (scope-of-framework).** The combination of `buildRadar`/`buildSonarSensor` setting `DetectionCoordinates='Scenario'`/`'Sensor spherical'` explicitly, `buildIR` omitting it entirely (until this release), and `runDetections.m` line 398 hard-cutting at `numel(Measurement) < 3` together suggest the framework was originally written assuming all sensors emit position-vector measurements — IR was declared in `buildSensor` switch table but never intended to flow through the tracker as angles-only. This release closes that asymmetry for the `'Scenario'` coords default. Bearings-only IR tracking (which would require a different tracker filter, e.g. range-parameterized EKF or angle-only EKF) remains out of scope.
- **CRLF→LF artifact list grows by one.** `buildSensor.m` joins the existing CRLF-converted-by-edit-file list (`drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`). Five files total. `git diff -w` for review.

### v3.6.11 — May 25, 2026 (superseded by v3.6.12)

**Phase A step 2 verification run complete; promoted MSSR gates G3/G4/G6 → ✅; added MSSR-F4 + X-F5 elevation-geometry compatibility notes.** Doc-only release. No `.m` source files touched, so `PosterDemo` remains bit-identical (re-run on 2026-05-25 confirmed: T1=1233.1m, T2=340.0m, T3=2043.7m posRMS, Tracked%=98/98/73, 0 swaps).

- **MSSR audit gates promoted to ✅** based on the `runSingleScenario("test_MSSR")` console output:
  - **G3 (Classification)** — confirmed by `[runDetections] MSSR detected: SensorIndex=1` and `Primary sensors: 0 \| Beacon sensors: 1`. The metadata-`type`-tag classifier route fired (not the heuristic FAR/Pd/refRange fallback), so the JSON's `"type":"SSR"` is plumbing correctly through `loadRunFile → meta.type → classifyAsMSSR`.
  - **G4 (Detection physics)** — beacon routing confirmed by per-scan `Primary=0, Beacon=N, total=N (clutter=0)` lines. ObjectClassID stamping at `runDetections.m` line 419 ran without error.
  - **G6 (Tracker integration)** — GNN+IMM consumed beacon detections cleanly through the `mergedDets = [mssrBuffer; detBuffer]` handoff path, ran without error, executed swap analysis (0 swaps).
  - **G2 (Build & introspect)** — marked ⚠ implicit pass; explicit `disp(ssr)` introspection still recommended for promotion to ✅.
  - **G5 (Visualization)** — stays ⏳ pending visual eyeball of the 3D plot for orange ring + absent beam cone.
  - **G1 / G7** — stay ⚠ with their existing findings (MSSR-F1 orphan, etc.). Same status, deferred to post-demo.

- **`docs/SENSOR_VALIDATION_MATRIX.md` — new finding MSSR-F4.** During the verification run, `test_MSSR` produced only 4 scans across 60 s (the 5-scan tracker minimum was missed, warning fired). Root cause: `crossing_pair` default targets at 5000m altitude vs SSR's narrow 10° elevation FOV `MechanicalElevationLimits=[-7 3]` — target elevation angles fall in the 5.7°–11.3° range at 25–50 km, above the +3° upper edge. **This is correct ASR-11 MSSR behavior** (narrow elevation by design, for en-route transponder traffic, not overhead coverage). The sensor pipeline itself is fully verified; only the test scenario's target geometry doesn't fit the SSR's elevation envelope. Not a regression, not a bug.

- **`docs/SENSOR_VALIDATION_MATRIX.md` — new cross-cutting finding X-F5.** Generalizes MSSR-F4 across all narrow-elevation sensor types (AESA 3°, FIRE_CONTROL 2°, PAR 1°, WEATHER 1°, ASR 5°). PSR's 30° elBW absorbs most reasonable airborne target geometries; the narrower-FOV types have stricter altitude-vs-range envelopes. The defaults match real-world radar physics. Three optional post-demo polish ideas listed (elevation envelope visualization in `drawSensorCoverage`, pre-flight target-in-beam coverage validator, Path Editor altitude hint panel) — none required for the Boeing demo.

- **`config/runs/test_MSSR.json` description revised** to flag the elevation constraint up front. The "Only 4 scan(s)" warning is now documented as expected behavior for this scenario, so future runs of the smoke test won't mistake it for a regression. The expected console signature is spelled out in the JSON `description` field.

- **Matrix document version bumped to v1.2.** Status line, gate table, findings section, and verification status block all updated to reflect the verification run.

**Validation expected** (optional re-run since no `.m` files changed):
- `runSingleScenario("test_MSSR")`: same 4-scan output, `Primary sensors: 0 \| Beacon sensors: 1`, `MSSR detected: SensorIndex=1`, "Only 4 scan(s)" warning expected per MSSR-F4 (not a regression).
- `runSingleScenario("PosterDemo")`: T1/T2/T3 posRMS = 1233/340/2044m, Tracked% = 98/98/73, 0 swaps. Already re-confirmed this release.

### v3.6.10 — May 25, 2026 (superseded by v3.6.11)

**Phase A step 2 — MSSR/SSR Gate 1 audit complete; three new findings
documented.** Doc-only release. No `.m` source files touched, so
`PosterDemo` remains bit-identical to v3.6.8/v3.6.9 by construction
(T1/T2/T3 posRMS = 1233/340/2044 m, Tracked% = 98/98/73, 0 track swaps).

- **`docs/SENSOR_VALIDATION_MATRIX.md` — MSSR/SSR/IFF row expanded.**
  Replaced the v3.6.9 placeholder row (⏳ PENDING, single G1–G7 stub)
  with a full Gate 1 audit — status ⚠, properties table per gate,
  three findings, and a verification command block. Document
  version bumped to v1.1; linked README version updated to 3.6.10.
- **`config/runs/test_MSSR.json` — canonical MSSR smoke-test.**
  Mirrors `test_PSR.json`: single tower, `SSR/default_SSR` (120 nm,
  canonical ASR-11 MSSR spec), `crossing_pair/default_crossing_pair`
  targets at 10–50 km, flat `none` terrain, GNN tracker. All
  degradations off except horizon (Doppler fade auto-skipped by the
  `~si.isMSSR` gate at `runDetections.m` line ~451; weather
  attenuation same gate at line ~468; ground clutter generated from
  primary radar only, line ~497). Lets the next validation session
  exercise the SSR build-classify-beacon-track pipeline end-to-end.

- **Three non-blocking findings flagged in the matrix** (no fix this
  release — same bit-identical guarantee as v3.6.9):
  - **MSSR-F1** — `src/+trackbench/+sensors/buildIFFSensor.m` is an
    orphan file. The active pipeline routes MSSR sensors through
    `buildSensor → buildRadar` (verified by reading the full
    `loadRunFile.m` and checking every script in `scripts/` and
    `scripts/legacy/` for callers — none found). The orphan file's
    elevation formula `[-fov(2) 0] - tilt` produces `[-12 -2]` by
    default — the entire beam below horizon. Inert today, hazard if
    someone restores it as a factory in a future refactor.
    Recommended post-demo fix: delete the file or convert to a thin
    documented shim around `buildSensor(idx, 'SSR', varargin{:})`.
  - **MSSR-F2** — `default_SSR.json`'s top-level `frequency_hz: 1.03e9`
    reaches `meta.frequency` (used downstream for clutter freq
    scaling and the heuristic MDV lookup) but does **not** reach the
    actual `fusionRadarSensor.CenterFrequency` property — that comes
    from `buildSensor.getDefaults('SSR').centerFreq = 1.06e9`. Silent
    mismatch (both L-band, no warning fires). Low impact for MSSR
    specifically because most freq-dependent physics paths (clutter,
    Doppler, rain) auto-skip MSSR-classified sensors via `~si.isMSSR`
    gates in `runDetections.m`.
  - **MSSR-F3** — beacon-path `ObjectClassID = TargetIndex + platformIdx`
    formula at `runDetections.m` line ~419 can collide in multi-tower
    scenarios (target_pid=2 + tower_pid=1 = target_pid=1 + tower_pid=2
    = 3). PosterDemo single-tower geometry cannot trigger this. Defer
    to post-demo. Recommended fix: `tgtIdx * 1000 + platformIdx`, or
    a true `(tgtPID, sensorPID)` tuple in a separate
    `ObjectAttributes` field.

- **`docs/SENSOR_VALIDATION_MATRIX.md` — new cross-cutting finding
  X-F4.** Generalizes MSSR-F2: top-level `frequency_hz` in any sensor
  JSON splits to `meta.frequency` (which reaches physics) but never
  to `sensor.CenterFrequency` (which stays at the `getDefaults` value
  for the three types — PSR/SSR/MARITIME — that set `centerFreq`, and
  the SDK default for the other seven radar types). Workaround for
  users today: put `centerFreq` under `params`. Recommended fix:
  one-line plumbing addition in `loadRunFile.m` § 2.

**Validation expected** (re-run after `clear classes; clear all; rehash`):
- All numeric values identical to v3.6.8/v3.6.9 (no `.m` files touched).
- `runSingleScenario("test_MSSR")` runs end-to-end producing real
  `posRMS` numbers and `Tracked%` above 80% over flat terrain with
  only horizon masking active. Per-scan log signature: `Primary
  sensors: 0 \| Beacon sensors: 1` followed by `Primary=0,
  Beacon=N, total=N (clutter=0)` lines, plus `[runDetections] MSSR
  detected: SensorIndex=1` confirming metadata-route classification.

### v3.6.9 — May 25, 2026 (superseded by v3.6.10)

**Sensor validation pass — PSR gates 1–7 complete; matrix established.**
Doc-only release. No `.m` source files touched, so `PosterDemo`
remains bit-identical to v3.6.8 by construction (T1/T2/T3 posRMS =
1233/340/2044 m, Tracked% = 98/98/73, 0 track swaps).

- **New: `docs/SENSOR_VALIDATION_MATRIX.md`** — Boeing-presentable
  per-sensor-type status table across all 23 declared sensor types
  in `buildSensor.m`. PSR fully filled in across gates G1–G7 with
  verification commands; Phase A steps 2 (MSSR) and 3 (IR) and all
  of Phase B / Phase C marked ⏳ PENDING with priority phase. The
  matrix is intended to evolve sensor-by-sensor — the PSR row is
  the template for everything that follows.
- **New: `config/runs/test_PSR.json`** — canonical PSR smoke-test
  independent of `PosterDemo`. Uses `sband_PSR` (60 nm, S-band 2.8 GHz
  canonical ASR-11 spec) + `default_crossing_pair` (two crossing
  targets at 10–50 km) + `none` terrain + GNN tracker. All
  degradations off except horizon and Doppler. Lets the next
  validation session smoke-test PSR end-to-end without depending on
  PosterDemo's lock geometry.
- **Two non-blocking findings flagged in the matrix** (no fix this
  release — both are low-risk pre-demo, and code edits would
  invalidate the PosterDemo bit-identical guarantee):
  - **PSR-F1**: `safeSet(radar, 'HasRCSSignature', true)` in
    `buildSensor.m`'s `buildRadar()` is a silent no-op —
    `HasRCSSignature` is not a documented R2025b property of
    `fusionRadarSensor` (cross-checked against
    `@fusionRadarSensor/fusionRadarSensor.m` header in
    `C:\Program Files\MATLAB\R2025b\toolbox\fusion\core\fusion`).
    `fusionRadarSensor` reads `platform.Signatures` natively. Dead
    code; remove in a post-demo cleanup sweep.
  - **PSR-F2**: the v3.6.7 changelog's claim that
    `MountingAngles(2) = +2°` for PosterDemo's PSR is inaccurate.
    `buildRadar()` never assigns `MountingAngles`, and neither
    `MountainSensor.json` nor `default_PSR.json` set it in params,
    so the default `[0 0 0]` survives. `MountingAngles(2) = 0` for
    PosterDemo's PSR, not `+2°`. The v3.6.7 fix is still correct in
    principle (the old `ElectronicScanAngle` typo also returned the
    fallback 2°), but the "bit-identical" reason is different: PSR's
    elBW = 30° is wide enough that both `tilt=0°` and `tilt=2°`
    produce `lowerEdgeDeg < 0` in `generateGroundClutter`, hitting
    the same `else` branch. For narrower-FOV Phase B types (PAR
    elBW=1°, AESA elBW=3°, FIRE_CONTROL elBW=2°, WEATHER elBW=1°),
    the `tilt=0` vs `tilt=2` difference WILL change clutter
    geometry — flag for verification when validating those types.

**Validation expected** (re-run `runSingleScenario("PosterDemo")`
after `clear classes; clear all; rehash`):
- All numeric values identical to v3.6.8 (no `.m` files touched).
- New: `runSingleScenario("test_PSR")` runs end-to-end producing
  real `posRMS` numbers and `Tracked%` in the 80–100% range over
  flat terrain with horizon + Doppler the only active degradations.

### v3.6.8 — May 25, 2026 (superseded by v3.6.9)

**Per-scan log labels: PSR/MSSR → Primary/Beacon.** The per-scan and
sensor-count log lines hardcoded the labels `PSR` and `MSSR`, but the
underlying buffers (`detBuffer`, `mssrBuffer`) actually split by
beacon-vs-skin classification, not by specific sensor type.
`detBuffer` collected detections from *any* primary-return sensor —
PSR, IR, sonar, LIDAR, custom radar, etc. — anything that isn't a
transponder reply. `mssrBuffer` collected MSSR/SSR/IFF/ADSB beacon
replies. So an IR-only scenario would log `PSR=8, MSSR=0` even with
zero PSRs in the scenario, which is confusing and inaccurate.

Renamed both log lines:
- `PSR count: N | MSSR: N` → `Primary sensors: N | Beacon sensors: N`
- `t=X.XX: PSR=N, MSSR=N, total=N (clutter=N)` →
  `t=X.XX: Primary=N, Beacon=N, total=N (clutter=N)`

The terminology is standard radar engineering: "primary surveillance"
is any skin-echo / direct return (PSR, IR, sonar all qualify);
"secondary surveillance" / "beacon" is transponder-aided (MSSR, SSR,
IFF, ADSB). The counts have always been correct; only the labels
were wrong.

**Fundamentals audit — known limitations documented.** As part of
pre-demo validation, the following were reviewed and either verified
or flagged:

- **Sensor types declared:** `buildSensor.m` supports 23 named
  variants (PSR, ASR, ARSR, PAR, TWS, AESA, FIRE_CONTROL, WEATHER,
  MARITIME, CUSTOM_RADAR, IRST, IR_STARING, FLIR, CUSTOM_IR,
  ACTIVE_SONAR, PASSIVE_SONAR, TOWED_ARRAY, CUSTOM_SONAR, LIDAR,
  CUSTOM_LIDAR, ADSB_TX, ADSB_RX, CUSTOM). **Actively validated in
  current demo path:** PSR (mechanical rotator, S-band, monostatic).
  Other types have code paths but have not been smoke-tested in
  the v3.6.5–v3.6.8 sessions — they may work, but no recent
  end-to-end confirmation.

- **Beam envelope rendering (`drawBeamEnvelope.m`)** is gated to
  `isRadar || isIR`. Sonar, LIDAR, and ADSB sensors get the coverage
  ring but no 3D cone. MSSR is also skipped (line 23) since beacon
  sensors don't have a directional beam shape worth visualizing.
  This is intentional but worth knowing for multi-modal scenarios.

- **`drawSensorCoverage.m` color assignment:** PSR=blue, sector
  radar=green, MSSR=orange, IR=magenta. Sonar, LIDAR, ADSB, and
  custom sensors fall through to cycled defaults (no dedicated
  color). Not a bug; just inconsistent in multi-modal plots.

- **Terrain generation (`generateTerrain.m`):** deterministic
  (`rng(42, 'twister')`), 6 terrain types (none, water, rural,
  urban, mountain, desert), 200x200 grid. The "radar hilltop
  clearing" block (~lines 155-175) assumes the radar platform is at
  scenario origin and creates a smooth 50m hilltop there. For
  editor-style scenarios where sensors live at world coords via
  `MountingLocation`, this hilltop is a leftover feature at
  `(0,0)` that doesn't affect physics but does create an unexpected
  terrain bump at origin. Worth refactoring post-demo to make the
  clearing optional or tied to actual platform positions.

- **Degradation physics paths:**
  - Terrain occlusion: ✓ exercised (PosterDemo)
  - Horizon masking: ✓ exercised (PosterDemo)
  - Ground clutter: ✓ exercised (PosterDemo, fixed in v3.6.7)
  - Doppler fade: ✓ exercised (PosterDemo)
  - RCS range filter: ✓ exercised (PosterDemo)
  - Rain attenuation: ⚠ not exercised in this session
    (`degradation.enabled=false` for PosterDemo). Code paths exist
    in `applyRainDegradation.m`. Recommended pre-demo: run
    PosterDemo once with rain enabled to confirm.
  - Snow/fog/icing: ⚠ not exercised. Code exists in
    `applyWeatherDegradation.m`.

**Validation expected** (re-run PosterDemo after `clear classes`):
- Per-scan log shows `Primary=N, Beacon=N` instead of `PSR/MSSR`.
- Sensor-count line shows `Primary sensors: 1 | Beacon sensors: 0`.
- All numeric values identical to v3.6.7 (label-only change).

### v3.6.7 — May 25, 2026 (superseded by v3.6.8)

**Silent try/catch audit — one more typo found.**

Following v3.6.5's `InitialPosition` postmortem, audited every
`try X.Property; catch; ... = default; end` pattern across the 12
most SDK-touching files in `+trackbench` (`runDetections.m`,
`createDetections.m`, `drawBeamEnvelope.m`, `drawSensorCoverage.m`,
`plotInitialScenario.m`, `plotScenarioAndDetections.m`,
`addTargetFromDef.m`, `buildSensor.m`,
`buildCustomFusionRadarSensor.m`, `runTracker.m`,
`applyDopplerFade.m`, `applyRCSFilter.m`, plus
`generateGroundClutter.m`). Every property name was cross-checked
against the R2025b MathWorks docs for `fusionRadarSensor`,
`monostaticRadarSensor`, `irSensor`, `sonarSensor`,
`fusion.scenario.Platform`, and `objectDetection`.

**One bug confirmed**, fixed in this version. Everything else
checks out as legitimate defensive coding.

**The bug — `runDetections.m` §ground-clutter `sParams` block:**
```
try sParams.tilt = cSensor.ElectronicScanAngle(2); catch; sParams.tilt = 2; end
```
`ElectronicScanAngle` is not a property of `fusionRadarSensor` in
R2025b. The actual documented properties are `ElectronicScanLimits`,
`LookAngle`, `MechanicalAngle`, and `MountingAngles`. The silent
catch has therefore fired on every sensor on every run for the
project's lifetime, leaving `sParams.tilt` pinned to the constant
`2°` fallback regardless of the actual sensor's mount geometry.

`sParams.tilt` is the beam-center elevation passed into
`generateGroundClutter.m`, where it sets the lower beam edge
(`lowerEdgeDeg = tiltDeg - elBW/2`) that determines how much of the
beam intersects terrain. Wrong tilt → wrong clutter geometry.

**Lucky-but-fragile detail:** PosterDemo's PSR has
`MountingAngles(2) = +2°` (the +2 from the 4/15 elevation fix), which
happens to match the broken fallback exactly. So the clutter
geometry has been *accidentally correct* for this specific scenario.
A sensor with mount pitch of, say, +5° or 0° would have had clutter
generated at the wrong elevation footprint.

**Fix:** read `cSensor.MountingAngles(2)` (antenna mount pitch in
degrees, positive = tilted up). For a pure mechanical rotator that's
the beam-center elevation. Fallback to `2°` is retained for sensor
types that lack a `MountingAngles` property. The PosterDemo run will
be bit-identical to v3.6.6 because the new and old values match;
other scenarios (any sensor not at +2° mount pitch) will now
produce slightly different but geometrically correct clutter
footprints.

**The other 30-plus `try X.Property; catch` blocks** in the audited
files are all reading documented properties with sensible fallbacks
for sensor types that don't have that particular property (e.g. an
IR sensor doesn't have `FalseAlarmRate`). These are legitimate
defensive coding patterns, not latent typos. The audit list:
`RangeLimits`, `MountingAngles`, `MountingLocation`, `FieldOfView`,
`MechanicalElevationLimits`, `MechanicalAzimuthLimits`,
`MechanicalScanLimits`, `ElectronicAzimuthLimits`,
`coverageConfig`+`ScanLimits`, `ScanMode`, `UpdateRate`,
`RangeResolution`, `FalseAlarmRate`, `DetectionProbability`,
`ReferenceRange`, `ReferenceRCS`, `CenterFrequency`, `Signatures`,
`Pattern`, `PlatformID`, `ObjectAttributes`,
`SurfaceManager.Surfaces`, `SurfaceManager.UseOcclusion`,
`Trajectory.Position` (v3.6.5 fix), `Trajectory.Waypoints` — all
verified against the docs.

**Validation expected** (re-run PosterDemo after `clear classes`):
- Per-scan detection log and clutter counts identical to v3.6.6.
- Total detection count identical (same `Terrain occluded 0`,
  `Horizon masked 0`).
- Tracker output `posRMS`, `Tracked%%`, `Avg posRMS` all identical.
- Cache stays valid (no source-physics change for this scenario;
  if you flip `cache.use_cached_detections` back to true the
  cached `.mat` from v3.6.6 will still load).

### v3.6.6 — May 25, 2026 (superseded by v3.6.7)

**Two cleanup fixes on top of v3.6.5**, both surfaced by the first
working post-v3.6.5 PosterDemo run.

**1. Sensor marker at correct altitude (3D plots).** With v3.6.5's
`cov.position` fix the blue-star marker correctly resolved in XY,
but its Z coordinate was still hardcoded to `gz = 0.05` in
`drawSensorCoverage.m` (ground-plane offset). The 3D scenario plot
showed the green/red beam-envelope cone apex at the sensor's true
altitude (~4 km for an editor-placed mountaintop sensor) while the
blue-star marker sat at ground level beneath it — visually
inconsistent. `drawSensorCoverage.m` now sets the marker Z to
`-cov.position(3) * sf` (negate NED to altitude-up), matching
`drawBeamEnvelope.m`'s convention. In top-down views the Z change
is invisible; in 3D views the star now sits at the cone apex where
it belongs.

The coverage *ring*, *wedge*, and *label* in `drawSensorCoverage.m`
stay at `gz` (ground level) because they're ground-plane projections
of the sensor's reachable area, not features of the sensor body.

**2. Validator Check 1 — underground check uses composed sensor
position.** `validateScenarioConfig.m` Check 1 ("Sensor Platform
Underground") was reading `platObj.Trajectory.Position` directly and
comparing it to terrain at that XY. For editor-style scenarios that
put the platform at scenario origin by design, the check sampled
terrain at `(0, 0)` — typically a valley floor — and reported the
platform as buried even though the actual sensor world position was
on a mountain peak via `MountingLocation`. Every editor-exported
scenario therefore showed a spurious `1 CRITICAL` in the pre-flight
validation block.

Fixed by composing the sensor world position for each attached
sensor: `sensorWorld = pPos + sObj.MountingLocation`, then checking
that against terrain at `(sensorWorld(1), sensorWorld(2))`. This is
the same position `fusionRadarSensor` uses internally for detection
geometry, so it's the correct ground truth for the buried-check.
For body-frame sensors with small `MountingLocation` offsets the
composed position is essentially identical to the platform position
and the check behaves the same as before. For editor-style sensors
the check now correctly samples terrain at the editor-placed XY —
e.g. the mountaintop — and the sensor is correctly reported as above
terrain. The issue label was tightened from "Sensor Platform
Underground" to "Sensor Underground" since we're now checking the
sensor itself, not the platform that hosts it.

**Validation expected** (re-run PosterDemo after `clear classes`):
- 3D scenario plot: blue star renders at the FOV cone apex (mountain
  peak at altitude ≈4 km), not at ground level beneath it.
- Pre-flight validation: `0 CRITICAL` for editor-style scenarios that
  used to fire the false-positive (PosterDemo specifically).
- All other physics and tracker results unchanged from v3.6.5 — the
  visualization and validator are independent of detection physics.

### v3.6.5 — May 25, 2026 (superseded by v3.6.6)

**The actual bug — and an honest postmortem.**

The symptom that triggered the entire v3.6.1–3.6.4 saga was that
editor-placed sensors rendered at the scenario origin (blue star and
green/red FOV cones both stuck on the valley floor) regardless of
where the user clicked in the path editor. I spent the afternoon
assuming that meant the *physics* had the sensor at the wrong place
and kept trying to move the platform up to the anchor. Three
increasingly heroic attempts later (post-hoc `Trajectory.Position`
write, constructor `'Position'` arg, full `Trajectory` handle
replacement with `kinematicTrajectory`), the diagnostic prints in
v3.6.4 finally showed that the platform *had* moved successfully on
attempt #3 — `plat.Trajectory.Position`, `pose(plat,'true').Position`,
and the composed sensor world position all read `[21065, 73, -4053]`
on the first scan. **The physics had been right**; my fix had been
working. The visualization was *still* broken because it reads from a
different code path entirely.

The real bug is one line in `runDetections.m` §coverage build:
```
try cov.position = si.platform.InitialPosition(:)' + s.MountingLocation(:)'; catch; cov.position = [0 0 0]; end
```
`Platform.InitialPosition` is not a property of
`fusion.scenario.Platform` in R2025b. The documented Platform
properties are `PlatformID, ClassID, Dimensions, Mesh, Position,
Velocity, Acceleration, Orientation, AngularVelocity, Trajectory,
Sensors, Emitters, Signatures, PoseEstimator` — no `InitialPosition`.
The try has therefore been throwing `"no appropriate method,
property, or field 'InitialPosition'"` on every sensor on every run
since this code was written, and the silent catch fallback has been
leaving `cov.position` at the origin universally. Every sensor in
every scenario has been rendering at origin because of that one
property-name typo, *not* because of any platform-positioning logic.

**Worse**, moving the platform to the anchor (v3.6.2/3/4) introduced a
separate real bug: `targetPoses(plat)` returns target positions **in
the platform's local NED frame**, per the MathWorks doc. When the
platform was at origin, local frame = scenario frame and everything
downstream that consumes `targetPoses` (occlusion LOS, horizon
masking, truth log, tracker-to-truth association) silently worked. As
soon as v3.6.2 moved the platform to `(21065, 73, -4053)`, all four
of those code paths started receiving target positions shifted by
the anchor and produced bit-identical garbage — 11179 LOS
occlusions, 100%% truth-est failure — even though the physics inside
`fusionRadarSensor` was happy.

**v3.6.5 fix — two surgical changes:**

1. **`loadRunFile.m` §7**: revert to v3.6.1 behavior. Editor sensors
   keep their world coordinates in `MountingLocation`, the platform
   stays at scenario origin, no Trajectory replacement, no rebase.
   The composed sensor world position
   `(plat.Position + sensor.MountingLocation)` is correctly used by
   `fusionRadarSensor` internally (this has always worked), and
   `targetPoses(plat)` returns absolute scenario coords because the
   platform's local NED frame coincides with the scenario frame.
2. **`loadRunFile.m` §9**: restore the v3.6.1 editor-style guard in
   the buried-check raise loop. Platforms whose attached sensors
   carry XY-offsets in `MountingLocation` larger than 100 m are
   flagged as editor-anchored and skipped — raising them to the
   terrain at origin would shift the composed sensor world position
   by the valley floor elevation, pulling editor-placed sensors
   below their intended altitude.
3. **`runDetections.m` §coverage build**: change
   `si.platform.InitialPosition` to `si.platform.Trajectory.Position`.
   `Trajectory.Position` is a real Platform property and defaults to
   `[0 0 0]` for editor-anchored sensors (the platform stays at
   origin), so `cov.position` correctly resolves to `MountingLocation`
   — the sensor's actual editor-placed world position. The blue star
   in `drawSensorCoverage.m` and the green/red beam-envelope cones in
   `drawBeamEnvelope.m` both read `cov.position`, so both renderings
   move to the correct location with this single fix.
4. **Diagnostic prints removed**: the three `DIAG-v3.6.4` prints in
   `loadRunFile.m §7/§9` and `runDetections.m` main loop served their
   purpose (they're what finally pinpointed where the value was — and
   wasn't — wrong) and have been torn out.

**Multi-sensor implication.** The scenario-origin convention scales
cleanly to multi-sensor scenarios: each editor-placed sensor lives on
its own platform at origin with its own `MountingLocation` holding
its world coords. Each blue star and FOV cone renders at its own
world position because `cov.position` resolves per-sensor as
`[0,0,0] + that_sensor's_MountingLocation`. No per-sensor frame
conversions anywhere in the pipeline.

**Validation criteria** (what should change vs the v3.6.4 run):
- The `[DIAG §...]` lines disappear from the console.
- `Detections generation complete. (Terrain occluded N)` with `N`
  near `0`, matching v3.6.1's working behavior.
- Tracker summary with real `posRMS` numbers, `Tracked%%` in the
  70–98%% range, `Est failure` percentages low.
- **2D/3D plots now show the blue star at the editor-placed sensor
  position** — the cosmetic bug that started this whole thing, finally
  fixed.
- Green/red FOV cones radiate from the editor-placed sensor position
  with the correct shape, tilt, range, and azimuth wedge.

**Postmortem note.** The lesson from this saga is that silent
`try/catch` fallbacks on property reads can hide critical bugs for
years. The `InitialPosition` typo had been in the codebase since
before v3.0; nothing failed loudly because the fallback `[0 0 0]`
produced a plausible-looking (but wrong) result. A future cleanup
pass should grep `+trackbench` for similar `try X.SomeProperty;
catch` patterns and either replace them with explicit `isprop`
checks or remove the catch so failures become visible.

### v3.6.4 — May 25, 2026 (superseded by v3.6.5)

**Bugfix on v3.6.3 — anchor via `Trajectory` replacement, not constructor `'Position'`.**

v3.6.3 was also wrong. `platform(scenario, 'Position', anchor, 'Sensors', sensorList)`
is valid MATLAB syntax per the MathWorks docs (see
[plotPlatform example](https://www.mathworks.com/help/fusion/ref/trackingglobeviewer.plotplatform.html)
showing `platform(s,'Position',[100 100 0])`), but in practice it did
not propagate the anchor through to `pose(plat,'true').Position` during
simulation. The smoking gun: the first post-v3.6.3 PosterDemo run
produced **per-scan detection counts and clutter draws bit-identical**
to the failed v3.6.2 run (`PSR=6, clutter=2` at `t=4.80`, `PSR=7,
clutter=4` at `t=9.61`, ... and `Terrain occluded 11179` again). Same
physics output → same physics input → the sensor was still effectively
at the origin in v3.6.3, just as in v3.6.2. Same 100%% truth-est
failure.

**v3.6.4 fix** — `src/+trackbench/+config/loadRunFile.m` §7:
- Create the platform with the default trajectory:
  `plat = platform(scenario, 'Sensors', sensorList)`.
- Then, if an editor anchor was detected, **replace the entire
  `Trajectory` handle** with a fresh `kinematicTrajectory` at the
  anchor:
  ```
  plat.Trajectory = kinematicTrajectory('Position', anchor);
  ```
  This is the same idiom already used in
  `+scenario/addTargetFromDef.m` (which replaces `Trajectory` with a
  `waypointTrajectory` for moving targets and is empirically known to
  work — every target trajectory in every run renders at its correct
  world position) and in the moving-platform branch of §7 itself.
- The `MountingLocation` rebase (subtract anchor from each attached
  sensor's `MountingLocation`) is unchanged from v3.6.2/v3.6.3 — still
  correct, since `MountingLocation` on a fusionRadarSensor *does*
  persist across `sn.MountingLocation = ...` assignments.
- Diagnostic line updated:
  `[RUN] Platform "tower": positioned at sensor world coords [E=21065, N=73, alt=4053]m (Trajectory replaced + rebased mountingLocs)`.

**Why three attempts** — the underlying MATLAB behavior here is
underspecified in the docs. The `Position` property of
`kinematicTrajectory` *is* listed as a settable property, and the
MathWorks doc examples show both `plat.Trajectory.Position = ...` and
`platform(s, 'Position', ...)` patterns. Empirically, neither writes
through to the trackingScenario simulation's effective pose for a
stationary platform — only **replacing the Trajectory handle
wholesale** does. The codebase already used this pattern for moving
platforms and targets, so the lesson is: when MATLAB System object
property writes don't seem to propagate, the safe move is to
reconstruct the System object rather than mutate its properties.

**Validation criteria** (carry-over from v3.6.3, which never met them):
- `Detections generation complete. (Terrain occluded N)` with `N`
  near `0`, not near `11179`. From the sensor's intended altitude of
  4053 m, looking out over terrain peaks at ≈3898 m, almost no LOS
  rays should hit mountains.
- Tracker summary with real `posRMS` numbers (not `N/A`), `Tracked%%`
  in the 70–98%% range, `Est failure` percentages low.
- 2D/3D track viewer's blue star renders near `(X≈21 km, Y≈0 km,
  alt≈4 km)`, not at the origin.

If v3.6.4 still produces 11179 occlusions and 100%% est failure, the
next step is to add diagnostic prints inside `runDetections.m` to
actually log `pose(plat,'true').Position` and rule out further MATLAB
API surprises before any more code edits.

### v3.6.3 — May 25, 2026 (superseded by v3.6.4)

**Bugfix on v3.6.2 — hoist now writes `InitialPosition`, not `Trajectory.Position`.**

v3.6.2 was wrong. Setting `plat.Trajectory.Position = anchor`
*post-creation* updated only the live trajectory state — the
simulation's effective static-platform pose is composed from
`Platform.InitialPosition`, which is captured at `platform(...)`
construction time and isn't refreshed by later `Trajectory.Position`
writes. Combined with v3.6.2's `MountingLocation ← mLoc - anchor`
rebase (which zeroed the offset on the anchor sensor), the radar's
effective world position used by `fusionRadarSensor` collapsed from
`(21065, 73, -4053)` back to the origin.

The failure mode was empirically obvious in the first post-v3.6.2 run:
  - `Detections generation complete. (Terrain occluded 11179)` —
    11179 LOS rays now blocked, vs **0** in the v3.6.1 run with the
    same scenario. From the origin a low-altitude sensor sees through
    mountain ridges on every scan.
  - `Truth1/2/3 est@86/86 (100%)` est failure — every truth track
    100%% miss-associated, vs 0–27%% in v3.6.1. The few detections
    that did make it through were geometrically inconsistent with the
    target truth positions because the sensor frame they were
    referenced to was at the wrong place.
  - `posRMS: N/A` in the results summary.

The validator was happy because Check 1 reads
`platObj.Trajectory.Position`, which *was* updated. So the symptom the
user was tracking (validator critical → cleared) gave the appearance
of progress while the physics silently broke. Lesson: when a fix
changes one read path and not another, the dial-of-truth becomes
whichever path the validator happens to use.

**v3.6.3 fix** — `src/+trackbench/+config/loadRunFile.m` §7:
- Restructured the per-platform loop so the editor-anchor detection
  and `MountingLocation` rebase happen **before** the `platform(...)`
  call.
- When an anchor is found, the platform is constructed via
  `platform(scenario, 'Position', anchor, 'Sensors', sensorList)`, so
  `InitialPosition` is `anchor` from the start. The radar's effective
  world position used by physics is therefore
  `InitialPosition + MountingLocation = anchor + 0 = anchor` — the
  original editor intent, but written into the construction-time pose
  rather than mutated post-hoc.
- When no anchor is found, behavior is bit-for-bit identical to the
  pre-v3.6.1 code path (default platform at origin, raise loop in §9
  decides whether to auto-lift onto buried terrain).
- A new local `isMoving` flag captures the moving-platform branch
  condition once so it can be reused for the anchor-skip check (moving
  platforms get their pose from a `waypointTrajectory` replacement
  after construction; the anchor logic shouldn't run for them).
- Diagnostic line updated to clarify the mechanism:
  `[RUN] Platform "tower": positioned at sensor world coords [E=21065, N=73, alt=4053]m (InitialPosition + rebased mountingLocs)`.

**§9 raise loop unchanged** — still the simple buried-check from
v3.6.2. With InitialPosition correct, `platObj.Trajectory.Position`
reads the same anchor, the buried check fires only for true ground
platforms below terrain, and editor-anchored platforms at altitude
are correctly skipped.

**Compatibility** — unchanged from v3.6.2 in spirit. No JSON schema,
run-file, or `fusionRadarSensor` property changes. Legacy non-editor
sensors with body-frame mountingLocs (small XY offset) bypass the
hoist entirely.

**Validation expected** — occlusion count drops back to the v3.6.1
level (~0 for the PosterDemo scenario), Tracked%% returns to the
73–98%% range, posRMS reports real numbers, and the 2D/3D track
viewer's blue star renders at the editor-clicked world position.

### v3.6.2 — May 25, 2026 (superseded by v3.6.3)

**Editor sensors hoisted to platform position — finishes the v3.6.1 fix.**

v3.6.1 stopped the auto-raise loop from clobbering editor-placed
sensors but left the platform at the origin, which surfaced two
downstream symptoms:
  - `validateScenarioConfig` §1 read `platform.Trajectory.Position` for
    its underground check, saw `z=0` at the scenario origin where
    terrain happened to be `80 m` ASL, and emitted a `CRITICAL` "Sensor
    Platform Underground" even though the *sensor* was at `4053 m`.
  - The 3D scenario plot rendered the radar marker at the origin
    instead of on the intended peak, because every viz path is
    platform-position-centric.

The v3.6.1 "don't raise editor sensors" guard was correct as far as
it went, but it left a deeper architectural inconsistency in place:
the physics path (`runDetections.m` line 294) composes
`platform.InitialPosition + sensor.MountingLocation` for the sensor's
world position, while every visualization and validation path treats
`platform.Position` *as if* it were the sensor world position. The
editor convention of "platform at origin, world XYZ in MountingLocation"
broke that assumption silently.

**v3.6.2 fix** — `src/+trackbench/+config/loadRunFile.m`:
- **§7 hoist (new)** — immediately after building each stationary
  platform, scan its attached sensors for a `MountingLocation` whose
  horizontal component (`hypot(mLoc(1), mLoc(2))`) exceeds 100 m. The
  first such hit becomes the *anchor*: the platform's
  `Trajectory.Position` is set to that world-frame vector, and every
  attached sensor's `MountingLocation` is rebased by subtracting the
  anchor. The composed world position
  (`platform.Position + sensor.MountingLocation`) is mathematically
  unchanged for every sensor, but `Trajectory.Position` now equals the
  primary sensor's world position, so the validator, the 3D plot, and
  the auto-raise loop all see what they expect.
  - Multi-sensor case: anchoring on the first editor sensor means
    additional sensors on the same platform end up with non-trivial
    body-frame `MountingLocation`s (the offset between their world
    position and the anchor). The composed world position is still
    correct. Multi-sensor-per-platform is unusual for editor exports;
    each editor sensor typically gets its own `platform` group via the
    `platform` field in its JSON.
- **§9 raise loop (simplified)** — the v3.6.1 `hasWorldPosSensor`
  guard is removed. With the hoist upstream, every stationary platform
  reaching this code is already at its true world position, so the
  loop reduces to a single buried-check: `pPos(3) > terrZ + 1` (NED).
  Real body-frame-mount platforms whose `Trajectory.Position(3)` is at
  default `0` continue to auto-lift onto non-trivial terrain exactly
  as before.
- **New diagnostic line** — hoisted platforms print
  `[RUN] Platform "tower": hoisted to sensor world position [E=21065, N=73, alt=4053]m (mountingLocs rebased)`
  so the user can confirm the editor's intent was honored.

**Compatibility** — no JSON schema, run-file, or `fusionRadarSensor`
property changes. The path editor and `exportSingleSensorToJSON` are
untouched; the existing "world XYZ in mountingLoc" export convention
still works exactly as it did, the hoist just normalizes it at load
time. Legacy non-editor sensors with body-frame `MountingLocation`s
(small XY offsets) bypass the hoist entirely and follow the
identical buried-check path as before.

**Validation expected after this fix** — the
`✗ [CRITICAL] Sensor Platform Underground` line from
`validateScenarioConfig` disappears for editor scenarios. The 3D
plot's radar marker, beam-envelope cone apex, and coverage ring all
render at the editor-clicked world position. Cached detections from
before this fix should be regenerated (`cache.use_cached_detections: false`)
because `SensorCoverage.position` is stored at detection time.

### v3.6.1 — May 25, 2026

**Hotfix — editor-placed sensors no longer dragged to ground.**

- `src/+trackbench/+config/loadRunFile.m` (§9 terrain attach) — the
  "raise stationary platforms to terrain" block was overwriting every
  stationary platform's `Trajectory.Position(3)` with the interpolated
  terrain Z at the platform's `(X,Y)` whenever that terrain was above
  1 m, with no check for whether the platform was actually buried.
  Path-Editor-exported scenarios are particularly affected because the
  editor's contract (per `exportSingleSensorToJSON` lines 154–158) is
  *tower platform stays at origin, sensor world XY+Z baked into
  `MountingLocation`* — so the raise step shifted every editor-placed
  sensor's effective altitude by the raise amount, dropping mountaintop
  PSRs onto whatever the ground happened to be at scenario origin.
  Repro: PosterDemo scenario with `MountainSensor` at 4053 m — console
  printed `Platform 1 raised to terrain surface (80m ASL)` and the 3D
  scenario plot rendered the sensor cone apex at ≈ 0 m instead of
  on the intended peak.
- **Fix** — two guards on the raise loop:
  1. Detect editor-style placement by scanning the platform's attached
     sensors for a `MountingLocation` whose horizontal component
     (`hypot(mLoc(1), mLoc(2))`) exceeds 100 m. Body-frame mounts on a
     tower, ship, or aircraft are at most a few tens of meters; an
     offset larger than that is unambiguously a world-coordinate mount
     from the editor. Those platforms are left at the origin.
  2. For non-editor platforms, raise only when the platform is actually
     buried: `pPos(3) > terrZ + 1` in NED (more positive Z = lower
     altitude). The original behavior of auto-lifting a default-Z
     platform onto a non-trivial terrain surface is preserved for true
     body-frame mounts.
- **Compatibility** — no run-file or JSON schema changes. Legacy
  scenarios where the platform was supposed to be auto-lifted continue
  to work identically (their sensors have body-frame mountingLocs, so
  the editor-detection guard does not trigger, and the buried-check
  guard fires exactly when the old `terrZ < -1` check did, since a
  Z=0 default platform is by definition buried below any terrain that
  was non-trivially above sea level).
- **No other files touched.** Rebuild not required for source/dev use
  (`clear classes; clear all` then re-run). For the deployed `.exe`,
  rebuild with `build_executable` to ship the fix.

### v3.6.0 — May 25, 2026

**Flight Data Manager — browse / globe / batch / export for NASA real-flight data.**

A new 4th main menu option (`Flight Data Manager`) for composing multi-flight
scenarios from NASA DASHlink Flight Data Recorder (`.mat`) files. Pulled
forward from post-demo per Boeing demo prep (May 29).

- **`scripts/flightDataManagerGUI.m`** — 3-pane workspace:
  - *Left* — `uitable` of `.mat` files with duration / max altitude / max
    ground speed / total-turn columns. Checkbox column drives the batch.
  - *Center* — single-flight lat/lon preview when a row is clicked, or
    merged NED layout preview when *Preview NED Layout* is pressed.
  - *Right* — batch builder with per-flight controls (RCS, max duration,
    waypoint interval, **start_offset_s**), global batch name, and ref
    origin lat/lon (auto = mean midpoint of checked flights, or manual).
  - *Top toolbar* — folder picker with Browse / Rescan / `?` Help.
    Default folder: `<userRoot>/flight_data/Tail_687_1/` in deployed
    mode (writable per-user dir, seeded from the installer on first
    launch); `<projectParent>/Tail_687_1/` in dev mode.
  - *Bottom actions* — View on Globe (`trackingGlobeViewer` +
    `geoTrajectory` per checked flight, no simulation), Preview NED
    Layout, Export to Path Editor, Close.
- **`scripts/mainMenu.m`** — grid grew 8 → 9 rows; new "4. Flight Data
  Manager" button + `launchFlightDataMgr` nested function. Window taller
  (460 → 526 px) to fit the extra button.
- **`src/+trackbench/+flightdata/scanFlightFolder.m`** — callable refactor
  of `scripts/scanNASAFlights.m`'s profiling loop. Returns a struct array
  (`file`, `fullPath`, `duration_s`, lat/lon bounds, `maxAlt_ft`,
  `maxGS_kts`, heading-change metrics). No console output — the original
  CLI script is preserved for ad-hoc folder profiling.
- **`src/+trackbench/+flightdata/buildBatchTargetJSON.m`** — emits a
  `recorded_flight` multi-target JSON matching the existing schema (e.g.
  `nasa_multi_target.json`). Computes a portable forward-slash relative
  path (`../Tail_687_1/<file>.mat` style) from absolute `fullPath` so the
  exported file works on any machine with the standard sibling-of-project
  `Tail_687_1` layout. Falls back to absolute path when no common
  ancestor (different drive letters on Windows).
- **`start_offset_s` plumbed through the runtime.**
  `trackbench.flightdata.loadNASAFlight` accepts a new `StartOffset`
  name-value option that shifts every `timeOfArrival` by the given
  seconds. `trackbench.scenario.addTargetFromDef` passes the JSON's
  `start_offset_s` field through. **Caveat:** the target sits at its
  first waypoint position from scenario `t=0` to `t=offset` (MATLAB's
  `waypointTrajectory` default), then begins moving. For NASA flights
  whose start is outside radar range this is fine; otherwise the radar
  will see a stationary blip during the pre-offset window. Documented in
  both docstrings and in the in-GUI `?` Help.
- **Installer integration.** `build_installer.m` now ships the 3 NASA
  Tail_687_1 sample flights as `AdditionalFiles` (`687200104121330.mat`,
  `687200107261425.mat`, `687200107282131.mat`; ~7.8 MB total).
  `mainMenu`'s `seedUserDataRoot` extends to (a) always create
  `<userRoot>/flight_data/` (writable, like cache/results) and (b) seed
  `<installRoot>/Tail_687_1/` → `<userRoot>/flight_data/Tail_687_1/` on
  first launch. FDM's `defaultFlightFolder` branches on `isdeployed`:
  the deployed default lands on the writable per-user folder, dev mode
  unchanged. Installer default version bumped to `3.6.0.0`. Rebuild
  with `build_executable; build_installer('both')` to ship.

The exported JSON drops in alongside `nasa_multi_target.json` /
`nasa_tail687_sample.json` under `config/targets/recorded_flight/`. Use
it by editing a run file's `targets` field to reference the new batch
name, or by opening it in Path Editor for further editing.

### v3.5.2 — May 2026

**Tracker Editor, "none" terrain type, EXE bug-fix sweep, physics audit refresh.**

- **Tracker Editor** in the Run Simulation window — click any tracker in the
  list and press *Edit Tracker* to edit its full parameter set (gate, FAR,
  beta, volume, confirm/delete thresholds, JPDA probabilities, TOMHT branch
  multipliers, …) without touching JSON. Save As with name validation,
  overwrite confirm, and round-trip preservation of cross-tracker fields.
  *Edit Globals…* edits `tracker_globals.json` (max tracks, ideal/degraded
  Pd, filter init params). Per-field tooltips with typical-value hints;
  save-time cross-field warnings flag inverted threshold/probability
  orderings or non-monotonic TOMHT multipliers.
- **"none" terrain type** added as the default for fresh editor sessions.
  Truly flat (no heightmap, no clutter, no occlusion). Lives at
  `config/terrain/none/default_none.json` and round-trips through the
  editor and run-file pipelines identically to the other terrain types.
- **Deployed-executable bug fixes** — five separate issues fixed in the
  `.exe` build:
  - Path resolution: scripts using `mfilename('fullpath')` were writing
    into the read-only CTF cache. Replaced with a deployed-mode-aware
    helper.
  - `degEqual` crash on polymorphic weather field (string vs. struct).
  - Degenerate-polygon warning from `parseRegion` in `loadRunFile`.
  - Auto-tune silently defaulting to IMM regardless of the JSON's
    `filter_model`.
  - Hardcoded `maxRange = 111120` over-normalized short-range scenarios.
- **Path Editor polish** — 3D view toggle now reachable in Sensors and
  Environment modes (was scenario-mode only), `V` shortcut documented in
  the help overlay, `setIfGraphics` crash on map-click sensor placement
  fixed.
- **Physics validation audit refresh** — `docs/physics_validation_audit.md`
  gained a v3.4+/v3.5 addendum covering ground clutter, horizon masking,
  the snow/fog/icing weather dispatchers, and the full `runDetections`
  orchestration (every toggle traced to its physics code path). Two
  defensive doc-only notes added in source: ITU-R P.838-3 fallback table
  caveat in `applyRainDegradation.m`, hardcoded-PRF caveat in
  `applyDopplerFade.m`. No functional physics changes.
- **Codebase cleanup** — milestone test scripts, presentation-prep helpers,
  and superseded launchers moved to `scripts/legacy/` (excluded from the
  CTF bundle). Active `scripts/` now contains only what end-users and
  developers actually run.

### v3.5.1 — April 25, 2026

**Per-user data directory for installed builds.** Fixes a Windows permissions trap where standard users (no admin elevation) could not save anything when the app was installed to `C:\Program Files\` via the Web/Offline installer.

- On first launch in deployed mode, `mainMenu` resolves a writable per-user
  data dir at `%LOCALAPPDATA%\RainyDay\` and seeds it from the read-only
  install dir. Items copied: `config/`, `docs/`, `README.md`,
  `CHECKPOINT.md`. Items always created: `cache/`, `results/`.
- `mainMenu` then `cd`s into that user data dir and uses it as `projectRoot`,
  so all sub-windows (`pathEditor`, `runSimGUI`, `validationDocsGUI`) and
  `runSingleScenario` write to a place the standard user actually owns.
- Subsequent launches preserve any user edits — the seed step is idempotent
  (only copies items that don't already exist).
- A `.rainyday_userdata` marker file documents the seed time and source.
- Reset to factory defaults: delete `%LOCALAPPDATA%\RainyDay\` and relaunch.
- Dev mode (running `mainMenu` from MATLAB) is unchanged — the source tree
  is still the working tree.
- Installer version bumped to 3.5.1.0.

No changes outside `scripts/mainMenu.m` and the installer version field.
Three new local helpers live at the bottom of `mainMenu.m`:
`findDeployedInstallRoot`, `resolveUserDataRoot`, `seedUserDataRoot`.

### v3.5.0 — April 2026

**3-button main menu + GUI rebuild + EXE entry point.**

- **`mainMenu.m`** — top-level launcher with three options: Path Editor,
  Run Simulation, Validation & Documentation. Replaces the previous 4-button
  menu (the orphaned Scenario Runner was subsumed by the path editor).
- **`runSimGUI.m`** — Run Simulation window with smart cache, dirty-tracking,
  and inline weather sub-grid.
- **`validationDocsGUI.m`** — 3-tab window: Validation Test Plan (27-row
  inline grid), Diagnostic Suite, and Documentation links.
- **Path Editor v3.5** — major refactor:
  - Mode-specific show/hide (Targets / Sensors / Environment buttons collapse
    irrelevant panels to 0px)
  - NASA Flight import as a dedicated target type
  - In-panel Save buttons across Targets / Sensors / Terrain / Weather; File
    panel shrunk to scenario-level Open/Export only
  - Three new exporters: `exportTerrainToJSON`, `exportWeatherToJSON`,
    `exportSingleSensorToJSON`
- **Compiled EXE** — `mainMenu.exe` is now the EXE entry point. The launchers
  (`trackbench.bat` / `trackbench.vbs`) check both project root and
  `trackbench/` for the EXE, falling back to MATLAB if neither exists.
- **`build_executable.m`** — auto-copies `mainMenu.exe` to project root after
  build so double-clicking it directly works.

### v3.4.2 — April 16, 2026

**Validation pass fixes** — restored 27/27 test plan pass rate.

- `applyRCSFilter.m` restored as opt-in via `"rcs_range_filter": true` in the
  run file's degradation block. Default OFF for user demos; TC-05 enables it.
- `sband_PSR.json` — symmetric S-band counterpart to `xband_PSR.json` for
  TC-03/TC-04 (only `CenterFrequency` differs).
- `validation_rain.json` — storm window aligned to validation scenario duration.
- TC-05 disables `doppler_fade` and turns on `rcs_range_filter`.

### v3.4.1 — April 15, 2026

**Physics & geometry fixes, auto-tuner, beam envelope visualization.**

- `buildSensor.m` elevation limits fixed (was pushing PSR beam below horizon).
- Frequency-dependent ground clutter (freq² Rayleigh, freq^0.8 discrete).
- Doppler fade MDV corrected to `λ × PRF / 4`.
- RCS range filter removed (later restored as opt-in in v3.4.2).
- VCP propagation model removed.
- `TargetIndex` clarified: uses `PlatformID`, not array index.
- `drawBeamEnvelope.m` renders exact scan-limit cones.
- `autoTuneTracker.m` two-pass parameter sweep.
- Diagnostic scripts: `diagBadDetections`, `diagVerifyTargetIndex`, `diagBeamLimits`.

### v3.4.0 — April 13, 2026

**NASA real flight data + 4-type weather system.**

- NASA flight integration: `loadNASAFlight`, `runNASAFlight`, `scanNASAFlights`,
  `viewNASAFlightGlobe`. Target behavior `recorded_flight` for FDR data.
- Weather system: `config/weather/<TYPE>/`. Four types: rain (ITU-R P.838-3
  via `rainpl`), snow (Gunn & East 25%), fog (visibility-based), icing
  (antenna hardware loss).
- Storm windows with step / ramp / pulse profiles.
- `applyWeatherDegradation` unified the four types behind one interface.
- Consolidated degradation block in run files.

### v3.3.0 — March 22, 2026

**Automated validation test plan.**

- `runTestPlan.m` — 9 test cases, 27 assertions.
- `verifySimulation.m` — 40+ standalone diagnostic checks.
- Test run files moved to `config/runs/validation/`.
- RCS demo geometry: targets moved from 30 km to 100 km.

### v3.2.0 — March 21, 2026

**Rain attenuation, aspect-dependent RCS, Doppler fade.**

- `rainpl()` (ITU-R P.838-3) replacement for hand-coded ITU table.
- `buildRCSProfile.m` — full az×el `rcsSignature` patterns. Five profiles.
- `applyDopplerFade.m` — MTI clutter notch with 5% sidelobe floor.
- Custom `"behavior": "waypoints"` for user-defined 3D paths.

### v3.1.x — March 20, 2026

- Physics validation audit (`docs/physics_validation_audit.md`).
- Sandbox run templates.
- Auto-terrain environment resolution.

### v3.0.0 — March 15–19, 2026

**Modular Config Architecture** — restructured from monolithic `default.json`
to per-component JSON files.

- Run files assemble sensors + targets + terrain + trackers by reference.
- Per-type folders for sensors, targets, terrain, trackers.
- Detection caching per run file.
- `tracker_globals.json` separates shared params from per-tracker tuning.
- Legacy catalog system removed.

### v2.x — February 23 – March 15, 2026

- v2.6: auto-terrain resolution, pre-flight scenario validator.
- v2.5: demo video recording.
- v2.4: one-command entrypoint, terrain occlusion toggle.
- v2.3: terrain occlusion, 3D visualization fixes.
- v2.2: propagation model (VCP), environment modeling, ground clutter.
- v2.1: JSON-driven scenario loading, 19 sensor types.
- v2.0: +trackbench namespace, JSON config, DASR sensor model.

---
 **Last Updated:** May 27, 2026
 **Version:** 3.7.3
