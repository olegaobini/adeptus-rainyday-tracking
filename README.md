# Rainy Day: Advanced Radar Tracking in Degraded Weather (V2)

## Quick Start

```matlab
% cd to the adeptus-rainyday-tracking folder, then:
addpath("scripts");

% ── SINGLE SCENARIO ──────────────────────────────────────────────
% Uses YOUR sensors from config/components/sensors/sensors.json + default.json:
runSingleScenario("default")

% Run a specific catalog scenario (uses that scenario's dedicated sensors):
runSingleScenario("crossing_targets")
runSingleScenario("fighter_intercept")

% ── BATCH ────────────────────────────────────────────────────────
% SHOWCASE MODE — each scenario uses its dedicated sensor config:
runAllScenarios

% MY SENSORS MODE — force YOUR sensors.json on every scenario:
runAllScenarios(true)
```

Configure in two files:
- **`config/components/sensors/sensors.json`** — toggle sensors `"enabled": true/false`
- **`config/default.json`** — scenarios_to_run, trackers_to_run, duration, degradation, environment, visuals


Advanced usage (manual pipeline):
```matlab
addpath(genpath(fullfile(pwd, 'src')));
[scenario, config, sensors, metas] = trackbench.scenario.loadScenario("crossing_targets");
dataLog = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas, config.environment);
tracker = trackbench.tracking.buildTracker('GNN', 'IMM', config.active_params, ...
    config.tracker_global, config.filter_params, config.active_params.pd, numel(sensors.tower));
[trkSum, trSum] = trackbench.tracking.runTracker(dataLog, tracker);
```

## What Changed from V1 → V2

1. **DASR sensor architecture** — PSR + MSSR co-rotating (not single generic radar)
2. **19 sensor types** — Radar, IR, Sonar, Lidar, ADS-B via universal `buildSensor` factory
3. **21 pre-built scenarios** — Airport, military, maritime, airborne, layered defense
4. **JSON-driven config** — Scenarios, sensors, tracker params all in JSON catalogs
5. **Track swap analysis** — Detects and reports identity swaps between targets
6. **Detectable track IDs** — GNN/JPDA don't penalize tracks outside sensor FOV
7. **Assignment timeline plots** — Visual track-to-truth assignment with swap overlay
8. **Batch runner** — `runAllScenarios` with showcase mode (dedicated sensors) or my-sensors mode (your picks)
9. **+trackbench namespace** — Clean MATLAB package structure
10. **No truth-based pre-filtering** — All detections (incl. clutter/outliers) pass directly to trackers for honest performance evaluation
11. **Horizon masking** — 4/3 Earth radius model prevents detections for targets below the radar horizon
12. **Ground clutter model** — Terrain-dependent false returns (water/rural/urban/mountain) concentrated at low elevation angles
13. **Environment config** — Per-scenario terrain type, clutter density, and refraction factor in `default.json`
14. **Multipath propagation (VCP)** — `radarvcd`-based vertical coverage patterns with terrain-dependent surface reflection
15. **Terrain occlusion** — Procedural terrain generation + SurfaceManager LOS blocking per terrain type
16. **Sensor coverage visualization** — Range rings (360° rotators) and sector wedges (PAR, FLIR) in 3D plots
17. **3D altitude display** — Correct NED-to-altitude conversion; terrain mesh rendering in tracker and scenario plots

## Project Structure

```
adeptus-rainyday-tracking/
├── config/
│   ├── default.json                 ← Base config (tracker params, toggles, environment)
│   ├── scenarios/
│   │   └── scenario_catalog.json    ← All 21 scenario definitions
│   ├── components/
│   │   ├── sensors/
│   │   │   ├── sensors.json             ← User-configurable (runSingleScenario "default")
│   │   │   ├── sensors_dasr.json        ← PSR+SSR (DASR scenarios)
│   │   │   ├── sensors_approach.json    ← PSR+SSR+PAR (par_approach)
│   │   │   ├── sensors_default_wedge.json ← 40° sector wedge (main-branch baseline)
│   │   │   ├── sensors_fighter.json     ← AESA+FLIR (fighter_intercept)
│   │   │   ├── sensors_maritime.json    ← Maritime+Sonar (maritime_surface)
│   │   │   ├── sensors_fire_control.json
│   │   │   ├── sensors_ir_fusion.json
│   │   │   ├── sensors_layered_defense.json
│   │   │   ├── sensors_long_range.json
│   │   │   └── sensors_phased_array.json
│   ├── templates/
│   │   └── standard_crossing.json    ← Reusable scenario pattern
│   └── sweeps/
│       └── weather_study.json         ← Batch sweep definition
│
├── src/+trackbench/
│   ├── +loader/
│   │   └── loadConfig.m             ← JSON config loader with overrides
│   ├── +detections/
│   │   └── runDetections.m          ← Detection generator (PSR+MSSR+terrain+VCP)
│   ├── +scenario/
│   │   ├── createScenario.m         ← DASR scenario builder (legacy)
│   │   ├── loadScenario.m           ← Load scenario from catalog + terrain
│   │   └── loadScenarioCatalog.m    ← List available scenarios
│   ├── +tracking/
│   │   ├── buildTracker.m           ← Tracker factory (GNN/TOMHT/JPDA)
│   │   ├── initCVFilter.m           ← Constant Velocity filter
│   │   ├── initIMMFilter.m          ← IMM filter
│   │   └── runTracker.m             ← Run tracker + metrics + 3D visualization
│   ├── +sensors/
│   │   ├── buildSensor.m            ← Universal sensor factory (19 types)
│   │   ├── buildCustomFusionRadarSensor.m
│   │   ├── buildIFFSensor.m         ← MSSR/IFF sensor builder
│   │   ├── customSensorTemplate.m
│   │   └── loadSensors.m            ← Load sensors from JSON catalog
│   ├── +reporting/
│   │   ├── plotInitialScenario.m    ← 3D animated truth + detections + terrain
│   │   ├── plotScenarioAndDetections.m
│   │   ├── plotPlatformToTrackAssignment.m ← Assignment timeline
│   │   ├── plotTrackSwapAnalysis.m  ← Swap analysis figure
│   │   ├── drawSensorCoverage.m     ← Range rings + sector wedges on 3D plot
│   │   └── tabbedAxes.m            ← Tabbed figure manager
│   ├── +environment/
│   │   ├── isAboveHorizon.m         ← 4/3 Earth horizon masking
│   │   ├── generateGroundClutter.m  ← Terrain-dependent clutter model
│   │   ├── computeVerticalCoverage.m ← radarvcd-based VCP computation
│   │   ├── applyVCPMask.m          ← Per-detection VCP range check
│   │   ├── computePropFactor.m      ← Propagation factor utility
│   │   └── generateTerrain.m       ← Procedural heightmap (water/rural/urban/mountain/desert)
│   └── +analysis/
│       └── analyzeTrackSwaps.m      ← Track swap detection
│
├── scripts/
│   ├── runSingleScenario.m          ← Single scenario (smart router)
│   ├── runAllScenarios.m            ← Batch runner (showcase or my-sensors)
│   ├── setupTrackbench.m            ← One-call path/root bootstrap
│   └── debug/
│       └── debugFireControl.m       ← Fire-control tracking diagnostics
│
├── tests/
│   ├── testBuildSensor.m
│   ├── testLoadScenario.m
│   └── testLoadSensors.m
│
├── cache/                           ← Saved detection logs (.mat per scenario)
└── outputs/                         ← Saved run results (.mat per batch)
```

## How To: Run Everything

All commands start with:
```matlab
addpath("scripts");
```

### Option A: Single scenario — your sensors
```matlab
runSingleScenario("default")          % uses sensors.json + default.json
```
Edit `config/components/sensors/sensors.json` to toggle which sensors run. Edit `config/default.json` for duration, trackers, environment, visuals.

### Option B: Batch showcase — dedicated sensors per scenario
```matlab
runAllScenarios                       % each scenario uses its own sensor config
```
Each scenario loads its dedicated sensors (PSR+SSR for DASR, AESA+FLIR for fighter, etc.). Toggle which scenarios run in `default.json` → `scenarios_to_run`. Trackers, environment, and visuals all come from `default.json`.

### Option C: Batch — your sensors on every scenario
```matlab
runAllScenarios(true)                 % forces sensors.json on all scenarios
```
Same targets/durations/degradation as showcase, but uses whatever sensors you have enabled in `sensors.json`. Good for testing one sensor setup across many target geometries.

### Option D: Single catalog scenario
```matlab
runSingleScenario("fighter_intercept") % uses that scenario's dedicated sensors
runSingleScenario("crossing_targets") % DASR PSR+SSR, two crossing targets
```

## Available Scenarios (21)

| Scenario | Sensors | Terrain | What It Tests |
|----------|---------|---------|---------------|
| **default** | **sensors.json** | **default.json** | **User-configured — reads sensor toggles from sensors.json** |
| default_wedge_ideal | Wedge Radar | default.json | Original main-branch 40° sector radar, clear |
| default_wedge_degraded | Wedge Radar | default.json | Original 40° sector radar, rain degradation |
| dasr_ideal | PSR+SSR (sensors_dasr) | rural | Baseline clear weather |
| dasr_degraded | PSR+SSR (sensors_dasr) | rural | Rain — tracker robustness |
| crossing_targets | PSR+SSR (sensors_dasr) | rural | Track swap — two targets crossing |
| head_on | PSR+SSR (sensors_dasr) | rural | Identity — targets approaching |
| high_density | PSR+SSR (sensors_dasr) | rural | Stress test — 5 targets |
| maneuvering_evasive | PSR+SSR (sensors_dasr) | rural | S-maneuver, IMM vs CV |
| storm_window | PSR+SSR (sensors_dasr) | rural | Max degradation — heavy rain |
| approach_pattern | PSR+SSR (sensors_dasr) | rural | Landing approach |
| long_range_arsr | ARSR+SSR | rural | 250nm en-route surveillance |
| par_approach | PSR+SSR+PAR | rural | Precision approach |
| phased_array_intercept | TWS+AESA | rural | Phased array fusion |
| fire_control_engagement | PSR+Fire Control | rural | Search-to-track handoff |
| ir_radar_fusion | PSR+IRST+IR | rural | Passive+active complementarity |
| ir_degraded_weather | PSR+IRST+IR | rural | Rain — radar degrades, IR doesn't |
| maritime_surface | Maritime+Sonar | water | Surface+subsurface tracking |
| fighter_intercept | AESA+FLIR | mountain | Fighter tracking bogey (terrain occlusion) |
| layered_defense | PSR+SSR+IRST+FC+TWS | rural | 5-sensor fusion |
| layered_defense_storm | PSR+SSR+IRST+FC+TWS | rural | Same in heavy rain |

## Available Sensor Types (19)

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

## Environment Modeling

The simulation includes five layers of physically-motivated environment effects that degrade sensor performance realistically. All layers are toggled independently in config.

### 1. Horizon Masking
Uses the standard 4/3 effective Earth radius model for atmospheric refraction. Before each sensor step, targets below the radar horizon are removed — the sensor never gets a chance to detect them.

**Impact by scenario:**
- High-altitude targets (>3km) at DASR range (60nm): always visible
- Low-altitude approach (50m) at 30nm: visible. At 50nm: masked
- Maritime (surface, ~0m altitude): horizon at ~16km from a 15m tower

The refraction factor is configurable per-scenario (default 4/3). Set to 1.0 for optical line-of-sight, >4/3 for ducting.

File: `+environment/isAboveHorizon.m`

### 2. Ground Clutter
Terrain-dependent false returns concentrated at low elevation angles where the radar beam intersects the ground.

| Terrain | Returns/scan | Noise (m) | Use Case |
|---------|-------------|-----------|----------|
| water | ~1-2 | 50 | Maritime, coastal |
| rural | ~3-6 | 100 | Default, airport |
| urban | ~8-15 | 150 | City approach |
| mountain | ~5-10 | 200 | Mountainous terrain |

All clutter passes through to the tracker unfiltered — the tracker's own gating handles rejection.

File: `+environment/generateGroundClutter.m`

### 3. Propagation Model (Multipath Lobing / VCP)
Uses MATLAB Radar Toolbox's `radarvcd` to compute vertical coverage patterns for each radar sensor. Multipath ground-bounce creates constructive/destructive interference — targets at certain elevation angles sit in nulls where detection is impossible.

Physics modelled:
- Multipath interference (direct + ground-reflected path)
- Surface roughness scattering (terrain-dependent via `landroughness`/`searoughness`)
- Surface permittivity (frequency + terrain via `earthSurfacePermittivity`)
- Vegetation attenuation
- Atmospheric refraction (effective Earth radius via `refractiveidx`)

| Terrain | Typical Effect | Notes |
|---------|---------------|-------|
| water | Strong lobing, peaks can exceed free-space | Smooth surface = strong reflection |
| rural | Moderate lobing, ~30% avg range reduction | Vegetation scatters some energy |
| urban | Mild lobing, surface roughness breaks coherence | Rough surface = weak reflection |
| mountain | Minimal lobing, terrain scatter dominates | Very rough = near free-space |

MSSR/SSR sensors are exempt (transponder link budget follows a different model).

Files: `+environment/computeVerticalCoverage.m`, `+environment/applyVCPMask.m`

### 4. Terrain Occlusion (LOS Blocking)
Procedurally generated terrain heightmaps attach to the tracking scenario via MATLAB's `groundSurface` API. At each timestep, `SurfaceManager.occlusion()` performs line-of-sight checks between every sensor-target pair — if terrain blocks the path, the detection is suppressed.

**Terrain profiles** (200×200 grid, seeded RNG for reproducibility):

| Type | Features | Max Elevation | Occlusion Impact |
|------|----------|--------------|-----------------|
| water | Flat (z=0) | 0m | None — open ocean |
| rural | Rolling hills, gaussian bumps | ~73m | Low — blocks at very low angles/long range |
| urban | Flat with building clusters (50-150m) | ~150m | Moderate near clusters |
| mountain | Ridge lines + isolated peaks | ~1960m | High — significant LOS blocking |
| desert | Gentle dunes + occasional mesa | ~40m | Low |

The terrain heightmap is also rendered as a 3D surface mesh in both `plotInitialScenario` and `runTracker` plots when available.

**Verified results:**
- `dasr_ideal` (rural, high-altitude targets): 0 terrain-occluded
- `par_approach` (rural, approach targets): 9644 terrain-occluded pairs (~2% of checks)
- `maritime_surface` (water): 0 terrain-occluded, 15760 horizon-masked (surface targets)
- `fighter_intercept` (mountain): 2402 terrain-occluded (ridge lines block low-angle LOS)

Files: `+environment/generateTerrain.m`, `+scenario/loadScenario.m` (Section 7)

### 5. Sensor Coverage Visualization
Range rings (360° rotators like PSR/SSR) and sector wedges (PAR, FLIR, fire control) are drawn on the ground plane of all 3D plots. Labels show sensor name, range in nm, and beam width.

File: `+reporting/drawSensorCoverage.m`

### Configuration
In `default.json` (overridable per-scenario in `scenario_catalog.json`):
```json
"environment": {
    "terrain_occlusion": false,
    "horizon_masking": false,
    "ground_clutter": false,
    "propagation_model": false,
    "terrain_type": "water",
    "clutter_density": 0.5
}
```
All five effects are independently toggleable. Set any to `true` to enable.

## Key Algorithms

| Tracker | Strength | Best For |
|---------|----------|----------|
| GNN | Fast, deterministic | Sparse, low-clutter |
| JPDA | Probabilistic, soft assignment | Medium clutter, real-time |
| TOMHT | Multiple hypotheses, optimal | Dense scenarios, highest accuracy |

| Motion Model | Assumption | Use Case |
|-------------|-----------|----------|
| CV | Straight line | Fast targets, baseline |
| IMM | Switches between straight & turn | Maneuvering aircraft |

## Known Issues / In Progress

- **Sonar sensors**: Maritime scenario builds sonar sensors but `runDetections` currently skips them (sonar uses `sonarEmission` step interface, not `fusionRadarSensor`). Only the maritime radar generates detections.
- **Terrain visibility at small scale**: Rural terrain (73m peaks) is physically present but visually indistinguishable from flat ground when viewed at aircraft altitude scale (3+ km). Use `fighter_intercept` (mountain, 1960m peaks) to see terrain mesh in plots.
- **Truth trajectory plotter**: The `showTruth` option in `runTracker` hardcodes 2 targets for `trajectoryPlotter` — scenarios with different target counts may error.

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

### v2.4.1 — February 24, 2026 (current)

**Dedicated Sensor Configs Per Scenario**
- Created `sensors_dasr.json` — PSR + SSR always-on. All DASR-based scenarios
  (`dasr_ideal`, `crossing_targets`, `head_on`, `high_density`, etc.) now point
  to this instead of the shared `sensors.json`. Ensures showcase mode always
  uses the right sensors regardless of what the user has toggled.
- Only `"default"` still reads from `sensors.json` (user-configurable).

**Batch Runner: Showcase vs My-Sensors Mode**
- `runAllScenarios` (no args) = SHOWCASE: each scenario uses its dedicated sensor config.
- `runAllScenarios(true)` = MY SENSORS: forces `sensors.json` on every scenario.
- `loadScenario` now accepts optional `sensorOverride` argument.
- Run plan header shows which mode is active.

**scripts/runAllScenarios.m Wrapper**
- Added `scripts/runAllScenarios.m` so users just need `addpath("scripts")` —
  no package paths required. Matches the `runSingleScenario` pattern.

**default.json: terrain_occlusion field added**
- Was implemented in `runDetections.m` but missing from the actual JSON.
  Now present and set to `false` by default.

### v2.4 — February 24, 2026

**One-Command Entrypoint**
- Rewrote `scripts/runSingleScenario.m` as a smart router. It checks the scenario
  catalog first — if the name exists there, it uses the full V2 pipeline
  (`loadScenario` → custom sensors/targets/environment). Otherwise it falls back
  to the simple `loadConfig` → `createScenario` (DASR) path.
- Added `"default"` entry to `scenario_catalog.json` that reads `sensors.json`.
  This means `runSingleScenario("default")` now honors sensor toggles in
  `config/components/sensors/sensors.json` instead of using the hardcoded DASR.
- All scenarios are now accessible via one command:
  ```matlab
  addpath("scripts");
  runSingleScenario("default")              % sensors.json + default.json
  runSingleScenario("crossing_targets")     % catalog scenario
  runSingleScenario("default_wedge_ideal")  % original wedge radar
  runSingleScenario("fighter_intercept")    % AESA on aircraft
  ```

**Default Wedge Radar Restored**
- Added "Default Wedge Radar (Main Branch Baseline)" to `sensors.json`
  (`enabled: false` by default). This is the original main-branch 40° sector
  radar: 25 RPM, FOV [1.5, 10], sector [250, 290], Pd 0.8.
- Created `config/components/sensors/sensors_default_wedge.json` (standalone sensor config).
- Added `default_wedge_ideal` and `default_wedge_degraded` to scenario catalog.

**Sector Scanner Fix (`runDetections.m`)**
- Added `isMechanical` flag to sensor classification. Previously only 360°
  rotators (`isRotator`) were recognized as having `IsScanDone`. Sector
  scanners (like the wedge radar, PAR) also produce `IsScanDone` on each
  sweep completion but were falling to the 1-second time-based flush.
  This caused ~4 detections per target per scan (multiple sweeps merged),
  leading to duplicate tracks.
- Scan master selection now uses `isMechanical` (true for both rotators
  AND sector scanners) instead of `isRotator`.
- Result: wedge radar now correctly produces ~2 detections/scan (one per
  target) and tracker forms exactly 2 tracks for 2 targets.

**Terrain Occlusion Toggle**
- Added `terrain_occlusion` field to environment config in `runDetections.m`.
  Set `"terrain_occlusion": false` in `default.json` to disable LOS checks
  against the terrain heightmap.
- All five environment effects are now independently toggleable from JSON:
  ```json
  "environment": {
      "terrain_occlusion": false,
      "horizon_masking": false,
      "ground_clutter": false,
      "propagation_model": false,
      "terrain_type": "water",
      "clutter_density": 0.2
  }
  ```

**Scan Coverage Safeguard**
- NEW `+scenario/validateScanCoverage.m` — pre-flight check that verifies
  scenario duration produces enough scans for the configured sensor(s).
  Prints a per-sensor diagnostic table and warns with recommended minimum
  duration if insufficient.
- Integrated into active entry paths: `runScenario` and `loadScenario`.
- Post-detection scan count check in `runSingleScenario` catalog path.

**Main-Branch Compatibility Shims**
- `+loader/loadConfig.m` — canonical JSON config loader
- `+detections/createDetections.m` — forwards to `runDetections.m` with defaults
- `runScenario.m` (package root) — matches main's `[results, detections]` signature
- `+results/ResultsSchema.m` — standard results struct factory
- Environment effects disabled in compat path for clean baseline behavior

**Misc Fixes**
- Populated `clear_weather.json` and `storm_window.json` (were empty/broken).
- Fixed `default.json` duration: 10s → 50s (DASR at 12.5 RPM needs 4.8s/scan).

### v2.3 — February 23, 2026

**Terrain Occlusion**
- NEW `generateTerrain.m` — Procedural heightmap generator for 5 terrain types (water/rural/urban/mountain/desert). 200×200 grid, seeded RNG, NED z-negative convention.
- `loadScenario.m` — Section 7: attaches terrain via `groundSurface(scenario, 'Terrain', Z, 'Boundary', bounds)` after target creation. Terrain type read from scenario config.
- `runDetections.m` — Two-stage visibility masking: (1) `SurfaceManager.occlusion()` for terrain LOS, (2) `isAboveHorizon()` for Earth curvature. Terrain grid passed through `envConfig.terrainGrid` to `dataLog.TerrainGrid` for visualization.
- `plotInitialScenario.m` — Renders terrain as `surf()` mesh with earth-tone colormap when `dataLog.TerrainGrid` exists; falls back to flat ground plane otherwise.
- `runTracker.m` — Same terrain mesh rendering in tracker 3D plots.

**3D Visualization Fixes**
- `runTracker.m` — Fixed NED Z-axis display: removed broken `ZDir='reverse'` (theaterPlot overrides it) and instead negate all Z values manually for plotting. Altitude now displays positive/upward across all plot elements (detections, tracks, terrain, ground plane).
- `runTracker.m` — Added axis labels with units (`X (km)`, `Y (km)`, `Altitude (km)`) applied *after* theaterPlot setup (which overwrites labels set earlier).
- `runTracker.m` — ROI Z-limits now computed from display altitude (negated NED Z) so theaterPlot axis range is correct.

**Bugfixes**
- `applyVCPMask.m` — Fixed crash on AESA 6-element measurements (`[x,y,z,vx,vy,vz]`). Now extracts `meas(1:3)` position-only before computing relative position to sensor. Previously failed with "Arrays have incompatible sizes" on `fighter_intercept`.
- `runTracker.m` — Fixed `cat(2, allDets.Measurement)` crash when scan contains mixed measurement sizes (e.g., AESA 6-element + clutter 3-element). Now extracts position-only per detection via loop.

**Sensor Coverage Visualization**
- NEW `drawSensorCoverage.m` — Draws range rings (360° rotators) and sector wedges (PAR, FLIR, fire control) on 3D ground plane with per-type color coding.
- Integrated into both `plotInitialScenario.m` and `runTracker.m`.
- Coverage metadata stored in `dataLog.SensorCoverage` during `runDetections`.

### v2.2 — February 23, 2026

**Propagation Model (VCP)**
- NEW `computeVerticalCoverage.m` — Computes vertical coverage diagrams using `radarvcd` with terrain-dependent surface parameters.
- NEW `applyVCPMask.m` — Per-detection range check against VCP. Soft fade zone at 90-100% of VCP max range.
- NEW `computePropFactor.m` — Propagation factor utility.
- Integrated into `runDetections.m` detection pipeline.

**Environment Modeling**
- NEW `isAboveHorizon.m` — 4/3 Earth radius horizon masking.
- NEW `generateGroundClutter.m` — Terrain-dependent false returns.
- `default.json` — Added `environment` config block with toggles.

**3D Visualization**
- Ground plane rendering in `plotInitialScenario.m` and `runTracker.m`.
- Dark theme styling for plots.

### v2.1

- `loadScenario.m` — JSON-driven scenario loading from catalog.
- `loadSensors.m` — Multi-platform sensor loading from JSON.
- `buildSensor.m` — Universal sensor factory (19 types).
- `runAllScenarios.m` — Batch runner with formatted results summary.
- Track swap analysis (`analyzeTrackSwaps.m`, `plotTrackSwapAnalysis.m`).
- Detectable track ID support for GNN/JPDA.

### v2.0

- Initial V2 architecture: +trackbench namespace, JSON config, DASR sensor model.

---

**Last Updated:** February 24, 2026
**Version:** 2.4.1
