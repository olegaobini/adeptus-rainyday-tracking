# CLAUDE.md — Rainy Day Project Context

## Project Overview
**Rainy Day** is a Boeing-sponsored senior capstone project (University of Washington) that provides a modular MATLAB sandbox for simulating radar tracking across scenarios with degradation from weather, terrain, targets, and equipment. Based on the MathWorks "Tracking Closely Spaced Targets Under Ambiguity" example, extended into a full scenario simulation framework.

**Version:** 3.0.0 (March 2026)
**MATLAB:** R2025b
**Branch:** Michael (do NOT push to remote — local changes only until manually reviewed)

---

## Critical Rules

1. **Never push to git.** All changes stay local. The user reviews and commits manually.
2. **Check MATLAB documentation before making large changes** to simulation logic, scenario construction, sensor models, or tracker algorithms. Key toolbox docs:
   - Sensor Fusion & Tracking: https://www.mathworks.com/help/fusion/
   - Radar Toolbox: https://www.mathworks.com/help/radar/
   - Phased Array: https://www.mathworks.com/help/phased/
   - Mapping Toolbox: https://www.mathworks.com/help/map/
3. **Adhere to existing project structure.** All source code lives in `src/+trackbench/` using MATLAB package namespaces. Config is modular JSON in `config/`. Scripts in `scripts/`.
4. **Preserve the user-facing workflow.** Users run simulations via:
   ```matlab
   addpath("scripts");
   runSingleScenario("my_run")
   ```
5. **Update the changelog** in README.md when making meaningful changes.

---

## Project Structure

```
adeptus-rainyday-tracking/
├── config/
│   ├── runs/          ← Run files (entry point configs)
│   ├── sensors/       ← Per-type sensor folders (PSR/, SSR/, AESA/, etc.)
│   ├── targets/       ← Per-behavior target folders (crossing_pair/, orbit/, etc.)
│   ├── terrain/       ← Per-type terrain folders (water/, rural/, mountain/, etc.)
│   └── trackers/      ← Per-algorithm tracker folders (GNN/, JPDA/, TOMHT/)
│       └── tracker_globals.json  ← Shared params (Pd, filter init)
├── src/+trackbench/
│   ├── +config/       ← loadRunFile.m (modular config loader)
│   ├── +detections/   ← runDetections.m (detection generator)
│   ├── +scenario/     ← addTargetFromDef.m, validateScanCoverage.m
│   ├── +tracking/     ← buildTracker.m, runTracker.m, initCVFilter.m, initIMMFilter.m
│   ├── +sensors/      ← buildSensor.m (universal sensor factory, 19 types)
│   ├── +environment/  ← isAboveHorizon.m, generateGroundClutter.m, computeVerticalCoverage.m,
│   │                     applyVCPMask.m, generateTerrain.m, computePropFactor.m
│   ├── +reporting/    ← plotInitialScenario.m, drawSensorCoverage.m, tabbedAxes.m, etc.
│   ├── +validation/   ← validateScenarioConfig.m (pre-flight checks)
│   └── +analysis/     ← analyzeTrackSwaps.m
├── scripts/
│   ├── runSingleScenario.m    ← MAIN ENTRY POINT
│   ├── buildModularConfig.m   ← One-time folder setup
│   └── recordDemoVideo.m      ← Demo video capture
├── docs/              ← SENSOR_REFERENCE.md, V2_WORKFLOW_GUIDE.md
├── tests/             ← testBuildSensor.m
├── MathWorks_*.md     ← Reference links to MATLAB examples
├── cache/             ← Saved detection logs (gitignored)
└── results/           ← Saved run results (gitignored)
```

---

## Data Flow

```
runSingleScenario("run_name")
  → trackbench.config.loadRunFile  (reads run JSON, loads sensor/target/terrain/tracker JSONs)
  → trackbench.detections.runDetections  (radar step loop with environment effects)
  → For each tracker:
      trackbench.tracking.buildTracker  (factory: GNN/JPDA/TOMHT × CV/IMM)
      trackbench.tracking.runTracker    (runs tracker, computes metrics, plots)
  → Save results to results/ and detections to cache/
```

---

## How the Modular Config Works

A **run file** (`config/runs/*.json`) assembles four independent components by reference:
- `sensors`: array of `"TYPE/filename"` strings → loads from `config/sensors/<TYPE>/<file>.json`
- `targets`: `"PATTERN/filename"` string → loads from `config/targets/<PATTERN>/<file>.json`
- `terrain`: `"TYPE/filename"` string → loads from `config/terrain/<TYPE>/<file>.json`
- `trackers`: array of `"ALGO/filename"` strings → loads from `config/trackers/<ALGO>/<file>.json`

**Volume and beta** are per-tracker (not in globals) because GNN and JPDA use them with opposite effects.

**Detection caching:** Set `"use_cached_detections": true` in run file to skip detection generation and go straight to tracker — fast for tuning.

---

## Required MATLAB Toolboxes

- **Sensor Fusion and Tracking Toolbox** (R2024a+) — trackers, sensors, theaterPlot, metrics
  - Key classes: `trackingScenario`, `radarDataGenerator`, `trackerGNN`, `trackerJPDA`, `trackerTOMHT`
  - Key functions: `trackGOSPAMetric`, `trackErrorMetrics`
- **Radar Toolbox** — propagation, clutter, environment
  - Key functions: `horizonrange`, `radarvcd`, `landroughness`, `earthSurfacePermittivity`, `refractiveidx`, `radarpropfactor`, `rainelres`, `atmositu`, `surfacegamma`
- **Phased Array System Toolbox** — antenna models, beamforming, waveforms
- **Mapping Toolbox** — terrain occlusion
  - Key classes: `groundSurface`, `SurfaceManager`

The full MATLAB R2025b installation with all toolboxes is at:
`C:\Program Files\MATLAB\R2025b\toolbox\`

Key toolbox subdirectories:
- `fusion/` — Sensor Fusion and Tracking Toolbox
- `radar/radar/` — Radar Toolbox functions (horizonrange.m, radarvcd.m, radarpropfactor.m, etc.)
- `phased/` — Phased Array System Toolbox
- `map/` — Mapping Toolbox

---

## Available Sensor Types (19)

| Type | Description | Platform |
|------|-------------|----------|
| PSR | Primary Search Radar (60 nm) | tower |
| SSR | Secondary Surveillance / IFF (120 nm) | tower |
| ASR | Airport Surveillance (60 nm) | tower |
| ARSR | Air Route Surveillance (250 nm) | tower |
| PAR | Precision Approach (20 nm) | tower |
| TWS | Track-While-Scan Phased Array (200 km) | tower |
| AESA | Active Electronic Scan Array (300 km) | aircraft |
| FIRE_CONTROL | Fire Control Radar (150 km) | tower |
| WEATHER | Weather Radar / NEXRAD (250 nm) | tower |
| MARITIME | Maritime Surface Search (40 nm) | ship |
| IRST | IR Search & Track (100 km) | tower |
| IR_STARING | Staring IR Sensor (50 km) | tower |
| FLIR | Forward-Looking Infrared (30 km) | aircraft |
| ACTIVE_SONAR | Active Sonar (20 km) | ship |
| PASSIVE_SONAR | Passive Sonar (50 km) | ship |
| TOWED_ARRAY | Towed Array Sonar (80 km) | ship |
| LIDAR | Lidar Point Cloud (200 m) | tower |
| ADSB_TX | ADS-B Transponder | aircraft |
| ADSB_RX | ADS-B Receiver | tower |

---

## Target Behaviors

`constant_velocity`, `gentle_turn`, `s_maneuver`, `crossing`, `orbit`, `approach`, `departure`, `head_on`, `parallel`

All require: `speed_kmh`, `start_pos` [x,y,z NED], `altitude_m`.

---

## Tracker Algorithms

- **GNN** (Global Nearest Neighbor) — fast, deterministic, best for sparse/low-clutter
- **JPDA** (Joint Probabilistic Data Association) — soft probabilistic assignment, medium clutter
- **TOMHT** (Track-Oriented Multi-Hypothesis) — multiple hypotheses, optimal for dense scenarios

Motion models: **CV** (Constant Velocity) or **IMM** (Interacting Multiple Model — switches between straight and turning)

---

## Environment Modeling (5 layers)

1. **Horizon Masking** — 4/3 Earth radius model (`isAboveHorizon.m`)
2. **Ground Clutter** — terrain-dependent false returns (`generateGroundClutter.m`)
3. **Propagation / VCP** — `radarvcd`-based vertical coverage patterns (`computeVerticalCoverage.m`)
4. **Terrain Occlusion** — procedural heightmaps via `groundSurface` API (`generateTerrain.m`)
5. **Sensor Coverage Visualization** — range rings and sector wedges (`drawSensorCoverage.m`)

---

## Known Issues

- Sonar sensors build but `runDetections` skips them (sonar uses `sonarEmission` step interface)
- Rural terrain is physically present but visually flat at aircraft altitude — use mountain terrain to see mesh
- `showTruth` in `runTracker` hardcodes 2 targets — scenarios with different counts may error

---

## Reference Material

MathWorks example reference files are in the project root:
- `MathWorks_Broader_Examples_Reference.md` — Radar, Phased Array, Aerospace/Defense, AI examples
- `MathWorks_Fusion_Toolbox_Examples_Reference.md` — Sensor Fusion & Tracking Toolbox examples
- `MathWorks_Radar_Examples_Comprehensive.md` — All Radar Toolbox + Phased Array examples (77+)

The original MathWorks example this project is based on:
https://www.mathworks.com/help/fusion/ug/tracking-closely-spaced-targets-under-ambiguity.html

Key weather/environment examples for the "Rainy Day" degradation modeling:
- Modeling Propagation of Radar Signals (rain, fog, multipath): https://www.mathworks.com/help/radar/ug/modeling-the-propagation-of-rf-signals.html
- Radar Surface Clutter Simulation: https://www.mathworks.com/help/radar/ug/clutter-topic.html
- Simulating Polarimetric Radar Returns for Weather: https://www.mathworks.com/help/radar/ug/simulating-a-polarimetric-radar-return-for-weather-observation.html
- Improving Weather Radar Moment Estimation with CNNs: https://www.mathworks.com/help/radar/ug/improving-weather-radar-moment-estimation-with-convolutional-neural-networks.html

---

## Style & Conventions

- All source code uses the `+trackbench` MATLAB package namespace
- JSON configs use snake_case for field names
- Functions use camelCase (MATLAB convention)
- Avoid adding files to project root — use the existing folder structure
- New sensor types go in `config/sensors/<NEW_TYPE>/` with default, template, and my_ files
- New target behaviors go in `config/targets/<BEHAVIOR>/`
- New terrain types go in `config/terrain/<TYPE>/`
