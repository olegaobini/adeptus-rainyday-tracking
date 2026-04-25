# Rainy Day: Advanced Radar Tracking in Degraded Weather

**Version 3.5.0** — Boeing-sponsored senior capstone project, University of Washington.

A modular MATLAB framework for evaluating radar target tracking performance under
real-world degraded conditions. Users compose custom scenarios by combining
independent JSON configuration files for sensors, targets, terrain, trackers,
and weather — no MATLAB code required.

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

% Demo video (1080p MP4):
recordDemoVideo

% Interactive path editor (v3.5.0):
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
├── terrain/                    ← water / rural / urban / mountain / desert
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
| `runSimGUI.m` | Run Simulation window with cache/dirty tracking |
| `validationDocsGUI.m` | Test plan + diagnostic suite + docs window |
| `runSingleScenario.m` | Run a scenario from the MATLAB console |
| `runTestPlan.m` | Validation suite (27 assertions across 9 test cases) |
| `verifySimulation.m` | 40+ standalone diagnostic checks |
| `autoTuneTracker.m` | Two-pass parameter sweep |
| `runNASAFlight.m` | Real flight demo (uses Tail_687_1) |
| `build_executable.m` | Compile mainMenu.exe via mcc |
| `compareTrackers.m`, `compareAllTrackers.m`, `runComparisonDemo.m` | Tracker comparison tools |
| `recordDemoVideo.m` | 1080p MP4 export |
| `diagBeamLimits.m`, `diagnoseBadDetections.m`, `diagVerifyTargetIndex.m` | Detection diagnostics |
| `testPathEditor_M*.m` | Editor regression tests |
| `verifyM*_endToEnd.m` | Milestone end-to-end checks |

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
| `water` | OFF | OFF | OFF | 0 | 0m |
| `rural` | ON | ON | ON | 0.3 | ~73m |
| `urban` | ON | ON | ON | 0.6 | ~150m |
| `mountain` | ON | ON | ON | 0.5 | ~1960m |
| `desert` | ON | ON | ON | 0.2 | ~40m |

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
Current status (v3.4.2): **27/27 passing**. The Validation & Documentation
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

### v3.5.0 — April 2026 (current)

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

**Last Updated:** April 25, 2026
**Version:** 3.5.0
