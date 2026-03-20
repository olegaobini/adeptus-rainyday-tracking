# Rainy Day: Advanced Radar Tracking in Degraded Weather

## Quick Start

```matlab
% cd to the adeptus-rainyday-tracking folder, then:
addpath("scripts");

% First time only — creates config folder structure:
buildModularConfig

% Run a simulation:
runSingleScenario("my_run")              % your custom run file
runSingleScenario("dasr_baseline")       % PSR+SSR, rural, GNN+JPDA
runSingleScenario("demo_mountain")       % PSR, 5 targets, mountain terrain
runSingleScenario("fighter_intercept")   % AESA+FLIR on aircraft
runSingleScenario("dasr_storm")          % PSR+SSR, mountain, heavy rain

% Demo video (1080p MP4):
recordDemoVideo
```

## How It Works

Each simulation is defined by a **run file** (`config/runs/*.json`) that assembles four independent components:

```
config/runs/my_run.json
  ├── sensors  → config/sensors/PSR/default_PSR.json
  │              config/sensors/SSR/default_SSR.json
  ├── targets  → config/targets/crossing_pair/default_crossing_pair.json
  ├── terrain  → config/terrain/mountain/default_mountain.json
  └── trackers → config/trackers/GNN/default_GNN.json
                  config/trackers/JPDA/default_JPDA.json
```

`runSingleScenario` reads the run file, loads each component, builds the MATLAB `trackingScenario`, generates detections, runs trackers, and reports metrics.

### Run file format
```json
{
  "description": "My experiment",
  "sensors": ["PSR/default_PSR", "SSR/default_SSR"],
  "targets": "crossing_pair/default_crossing_pair",
  "terrain": "mountain/default_mountain",
  "trackers": ["GNN/default_GNN", "JPDA/default_JPDA"],
  "degradation": { "enabled": false, "type": "rain" },
  "cache": { "use_cached_detections": false, "save_detections": true },
  "platforms": {},
  "output": { "show_visuals": true, "animate_visuals": true, "save_results": true }
}
```

See `config/runs/run_template.json` for the fully documented version.

## Building a Custom Simulation

### 1. Pick or create sensors
Each sensor type has its own folder in `config/sensors/<TYPE>/` with three files: `default_<TYPE>.json` (works out of box), `<TYPE>_template.json` (documented), `my_<TYPE>.json` (your copy).

```
config/sensors/PSR/my_PSR.json     — edit rpm, range, Pd, mounting location
config/sensors/AESA/my_AESA.json   — edit sector, range (needs aircraft platform)
```

### 2. Pick or create targets
Target folders define flight behaviors and duration:
```
config/targets/crossing_pair/my_crossing_pair.json   — two targets crossing
config/targets/s_maneuver/my_s_maneuver.json         — evasive S-turn
```

### 3. Pick terrain
Terrain folders define environment effects:
```
config/terrain/mountain/my_mountain.json   — ridges, occlusion ON, clutter ON
config/terrain/water/default_water.json    — flat ocean, all effects OFF
```

### 4. Pick trackers
Tracker folders define algorithm + tuning. Shared parameters (Pd, filter init) live in `tracker_globals.json`.
```
config/trackers/GNN/my_GNN.json     — gate, volume, beta, thresholds
config/trackers/JPDA/my_JPDA.json   — JPDA-specific tuning
```

### 5. Create a run file
Copy `config/runs/run_template.json` → `config/runs/my_experiment.json`, fill in references.

### 6. Run it
```matlab
runSingleScenario("my_experiment")
```

### Detection caching
After your first run, set `"use_cached_detections": true` in the run file. This skips detection generation and goes straight to the tracker — much faster when tuning gates, thresholds, or trying different algorithms.

## Project Structure

```
adeptus-rainyday-tracking/
├── config/
│   ├── runs/                        ← Run files (pass to runSingleScenario)
│   │   ├── run_template.json        ← Documented template (copy + edit)
│   │   ├── my_run.json              ← Your custom run
│   │   ├── dasr_baseline.json       ← PSR+SSR, rural, GNN+JPDA
│   │   ├── demo_first_run.json      ← Boeing demo
│   │   ├── demo_tuned_performance.json
│   │   ├── fighter_intercept.json
│   │   ├── dasr_storm.json
│   │   ├── README.md
│   │   └── showcase/                ← Pre-built scenarios (converted from catalog)
│   ├── sensors/                     ← Per-type sensor folders
│   │   ├── PSR/                     ← Primary Search Radar
│   │   │   ├── default_PSR.json
│   │   │   ├── PSR_template.json
│   │   │   └── my_PSR.json
│   │   ├── SSR/                     ← Secondary Surveillance (IFF)
│   │   ├── AESA/                    ← Active Electronic Scan Array
│   │   ├── PAR/                     ← Precision Approach Radar
│   │   ├── TWS/                     ← Track-While-Scan Phased Array
│   │   ├── FIRE_CONTROL/            ← Fire Control Radar
│   │   ├── ARSR/                    ← Air Route Surveillance
│   │   ├── IRST/                    ← IR Search and Track
│   │   ├── FLIR/                    ← Forward-Looking Infrared
│   │   ├── MARITIME/                ← Maritime Surface Search
│   │   └── WEDGE/                   ← Original wedge radar
│   ├── targets/                     ← Per-behavior target folders
│   │   ├── crossing_pair/           ← Two targets crossing
│   │   ├── gentle_turn/             ← Gradual heading change
│   │   ├── crossing_5way/           ← 5 targets (Boeing demo)
│   │   ├── s_maneuver/              ← Evasive S-turn
│   │   ├── head_on/                 ← Two targets converging
│   │   ├── approach/                ← Landing approach (descending)
│   │   ├── orbit/                   ← Circular holding pattern
│   │   ├── high_density/            ← 5 mixed-behavior targets
│   │   ├── dead_zone/              ← 6 targets, multi-site radar gap
│   │   └── (more from catalog...)
│   ├── terrain/                     ← Per-type terrain folders
│   │   ├── water/                   ← Flat sea, all effects OFF
│   │   ├── rural/                   ← Rolling hills, light clutter
│   │   ├── urban/                   ← Building clusters, moderate clutter
│   │   ├── mountain/                ← Ridges + peaks, heavy occlusion
│   │   └── desert/                  ← Gentle dunes, light clutter
│   └── trackers/                    ← Per-algorithm tracker folders
│       ├── tracker_globals.json     ← Shared: Pd, filter, max tracks
│       ├── GNN/                     ← Global Nearest Neighbor
│       ├── JPDA/                    ← Joint Probabilistic Data Association
│       └── TOMHT/                   ← Track-Oriented Multi-Hypothesis
│
├── src/+trackbench/
│   ├── +config/
│   │   └── loadRunFile.m            ← Modular run file loader
│   ├── +detections/
│   │   └── runDetections.m          ← Detection generator
│   ├── +scenario/
│   │   ├── addTargetFromDef.m       ← Universal target builder from JSON
│   │   └── validateScanCoverage.m   ← Pre-flight scan count check
│   ├── +tracking/
│   │   ├── buildTracker.m           ← Tracker factory (GNN/TOMHT/JPDA)
│   │   ├── initCVFilter.m           ← Constant Velocity filter
│   │   ├── initIMMFilter.m          ← IMM filter
│   │   └── runTracker.m             ← Run tracker + metrics + 3D plots
│   ├── +sensors/
│   │   └── buildSensor.m            ← Universal sensor factory (19 types)
│   ├── +reporting/
│   │   ├── plotInitialScenario.m    ← 3D animated truth + detections + terrain
│   │   ├── plotPlatformToTrackAssignment.m ← Assignment timeline
│   │   ├── plotTrackSwapAnalysis.m  ← Swap analysis figure
│   │   ├── drawSensorCoverage.m     ← Range rings + sector wedges
│   │   └── tabbedAxes.m            ← Tabbed figure manager
│   ├── +environment/
│   │   ├── isAboveHorizon.m         ← 4/3 Earth horizon masking
│   │   ├── generateGroundClutter.m  ← Terrain-dependent clutter model
│   │   ├── computeVerticalCoverage.m ← radarvcd-based VCP
│   │   ├── applyVCPMask.m          ← Per-detection VCP range check
│   │   └── generateTerrain.m       ← Procedural heightmap generator
│   ├── +validation/
│   │   └── validateScenarioConfig.m ← Pre-flight checks (10 categories)
│   └── +analysis/
│       └── analyzeTrackSwaps.m      ← Track swap detection
│
├── scripts/
│   ├── runSingleScenario.m          ← Main entry point
│   ├── buildModularConfig.m         ← One-time setup (creates config folders)
│   ├── recordDemoVideo.m            ← Record MP4 demo video
│   └── cleanupLegacy.m             ← Removes V2 legacy files (run once)
│
├── tests/
│   └── testBuildSensor.m            ← Sensor factory test
│
├── Cheatsheet.txt                   ← Quick-reference command sheet
├── README.md                        ← This file
├── .gitignore
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
| `head_on` / `parallel` | Mapped to constant_velocity | `heading_deg` |

All targets require: `speed_kmh`, `start_pos` [x,y,z NED], `altitude_m`.

## Tracker Algorithms

| Tracker | Strength | Best For |
|---------|----------|----------|
| GNN | Fast, deterministic | Sparse, low-clutter |
| JPDA | Probabilistic, soft assignment | Medium clutter, real-time |
| TOMHT | Multiple hypotheses, optimal | Dense scenarios, highest accuracy |

| Motion Model | Assumption | Use Case |
|-------------|-----------|----------|
| CV | Straight line | Fast targets, baseline |
| IMM | Switches between straight & turn | Maneuvering aircraft |

### Tracker globals vs per-tracker params
- **`tracker_globals.json`** — shared: max tracks, detection probability (ideal/degraded), filter initialization (speed, IMM transition prob, process noise)
- **Per-tracker JSON** — algorithm-specific: volume, beta, gate sizes, confirm/delete thresholds, branch limits

Volume and beta live per-tracker because GNN and JPDA use them with opposite effects.

## Environment Modeling

Five layers of physically-motivated effects, each independently toggleable in terrain configs.

### 1. Horizon Masking
4/3 effective Earth radius model. Targets below radar horizon are invisible.

### 2. Ground Clutter
Terrain-dependent false returns at low elevation angles.

| Terrain | Returns/scan | Noise (m) |
|---------|-------------|-----------|
| water | ~1–2 | 50 |
| rural | ~3–6 | 100 |
| urban | ~8–15 | 150 |
| mountain | ~5–10 | 200 |

### 3. Propagation Model (VCP)
`radarvcd`-based vertical coverage patterns. Multipath ground-bounce creates interference nulls.

### 4. Terrain Occlusion
Procedural heightmaps via `groundSurface` API. Line-of-sight checks between every sensor-target pair.

### 5. Sensor Coverage Visualization
Range rings (360° rotators) and sector wedges (PAR, FLIR) drawn on 3D ground plane.

### Terrain Presets

| terrain_type | Occlusion | Masking | Clutter | Density | Max Elevation |
|-------------|-----------|---------|---------|---------|--------------|
| `water` | OFF | OFF | OFF | 0 | 0m |
| `rural` | ON | ON | ON | 0.3 | ~73m |
| `urban` | ON | ON | ON | 0.6 | ~150m |
| `mountain` | ON | ON | ON | 0.5 | ~1960m |
| `desert` | ON | ON | ON | 0.2 | ~40m |

## Data Flow

```
runSingleScenario("my_run")
  └── trackbench.config.loadRunFile("my_run")
      ├── Read config/runs/my_run.json
      ├── Load each config/sensors/<TYPE>/<file>.json
      │   └── trackbench.sensors.buildSensor (per sensor)
      ├── Load config/targets/<PATTERN>/<file>.json
      │   └── trackbench.scenario.addTargetFromDef (per target)
      ├── Load config/terrain/<TYPE>/<file>.json
      │   └── trackbench.environment.generateTerrain
      ├── Load config/trackers/<TYPE>/<file>.json (per tracker)
      ├── trackbench.scenario.validateScanCoverage
      └── trackbench.validation.validateScenarioConfig
            ↓
  trackbench.detections.runDetections
      ├── Radar sensor step loop
      ├── trackbench.environment.isAboveHorizon
      ├── trackbench.environment.computeVerticalCoverage
      ├── trackbench.environment.applyVCPMask
      ├── trackbench.environment.generateGroundClutter
      └── SurfaceManager.occlusion (terrain LOS)
            ↓
  For each enabled tracker:
    trackbench.tracking.buildTracker
      ├── trackbench.tracking.initCVFilter
      └── trackbench.tracking.initIMMFilter
    trackbench.tracking.runTracker
      ├── trackbench.analysis.analyzeTrackSwaps
      ├── trackbench.reporting.tabbedAxes
      ├── trackbench.reporting.drawSensorCoverage
      ├── trackbench.reporting.plotPlatformToTrackAssignment
      └── trackbench.reporting.plotTrackSwapAnalysis
            ↓
  Save results to results/ and detections to cache/
```

## Known Issues

- **Sonar sensors**: Maritime scenario builds sonar sensors but `runDetections` skips them (sonar uses `sonarEmission` step interface). Only maritime radar generates detections.
- **Terrain visibility at small scale**: Rural terrain (73m peaks) is physically present but visually flat at aircraft altitude. Use mountain terrain to see mesh.
- **Truth trajectory plotter**: `showTruth` in `runTracker` hardcodes 2 targets — scenarios with different target counts may error.

## Required Toolboxes

- **Sensor Fusion and Tracking Toolbox** (R2024a+) — trackers, sensors, theaterPlot, metrics
- **Radar Toolbox** — `horizonrange`, `radarvcd`, `landroughness`, `earthSurfacePermittivity`, `refractiveidx`
- **Mapping Toolbox** — `groundSurface`, `SurfaceManager` for terrain occlusion

## Team

Boeing-sponsored senior capstone project.

## License

Boeing Proprietary.

---

## Change Log

### v3.0.0 — March 15–19, 2026 (current)

**Modular Config Architecture** — Complete restructure from monolithic `default.json` to individual component files.
- Run files (`config/runs/`) assemble sensors + targets + terrain + trackers by reference
- Per-sensor type folders with default/template/user configs (PSR, SSR, AESA, etc.)
- Per-terrain type folders with environment flag presets (water, rural, urban, mountain, desert)
- Per-tracker algorithm folders with algorithm-specific tuning (GNN, JPDA, TOMHT)
- Per-behavior target folders (crossing_pair, s_maneuver, orbit, etc.)
- Detection caching per run file for fast tracker tuning
- `tracker_globals.json` separates shared params from per-tracker volume/beta

**New Source Files**
- `loadRunFile.m` — modular config loader (reads run file, builds scenario from components)
- `addTargetFromDef.m` — universal target builder supporting 8 flight behaviors
- `buildModularConfig.m` — one-time setup script for folder structure
- `cleanupLegacy.m` — removes V2 catalog files no longer needed

**Simplified `runSingleScenario.m`** — single path through modular `loadRunFile`, no legacy routing.

**Legacy Removal** — catalog system (`scenario_catalog.json`, `loadScenario`, `createScenario`, `loadSensors`, `loadConfig`, `runScenario`, flat sensor configs, batch runner, compat shims, patch scripts) removed. Copies preserved in backup.

### v2.6.0 — March 15, 2026
Auto-terrain environment resolution, pre-flight scenario validator, sandbox templates.

### v2.5.0 — March 14, 2026
Demo video recording (`recordDemoVideo.m`).

### v2.4.1 — February 24, 2026
Dedicated sensor configs per scenario, batch runner showcase/my-sensors modes.

### v2.4 — February 24, 2026
One-command entrypoint, sector scanner fix, terrain occlusion toggle, scan coverage safeguard.

### v2.3 — February 23, 2026
Terrain occlusion, 3D visualization fixes, sensor coverage visualization.

### v2.2 — February 23, 2026
Propagation model (VCP), environment modeling, ground clutter.

### v2.1
JSON-driven scenario loading, 19 sensor types, batch runner, track swap analysis.

### v2.0
Initial V2: +trackbench namespace, JSON config, DASR sensor model.

---

**Last Updated:** March 19, 2026
**Version:** 3.0.0
