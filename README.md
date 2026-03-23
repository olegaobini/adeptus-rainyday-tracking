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

% Physics comparison (clean vs rain, all 3 trackers):
runComparisonDemo

% Automated validation (27 checks, 9 test cases):
runTestPlan

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

## Validation Test Plan

An automated test suite validates the full pipeline across 9 test cases (27 individual checks). Test run files live in `config/runs/validation/` — separate from user experiments.

```matlab
addpath("scripts");
runTestPlan          % runs all 9 test cases, prints PASS/FAIL summary
```

| TC | What It Tests | Key Checks |
|----|---------------|------------|
| TC-01 | Template usability | User-created config loads + runs end-to-end |
| TC-02 | Baseline clear weather | All 3 trackers (GNN, JPDA, TOMHT) produce metrics |
| TC-03 | Rain S-band (16 mm/hr) | S-band PSR degrades minimally, tracking holds |
| TC-04 | Rain X-band (16 mm/hr) | X-band fewer target detections than S-band (same rain rate) |
| TC-05 | RCS verification | 20 dBsm airliner detected more than -10 dBsm stealth at 100km |
| TC-06 | Crossing target swap | JPDA swap count ≤ GNN swap count |
| TC-07 | Compound stress | TOMHT + rain + mountain + mixed RCS completes without crash |
| TC-08 | Config error paths | Missing/malformed configs produce clear error messages |
| TC-09 | Verification suite | `verifySimulation.m` (40+ checks) passes all phases |

Results are saved to `results/test_plan_results_<timestamp>.mat`.

## Project Structure

```
adeptus-rainyday-tracking/
├── config/
│   ├── runs/                        ← Run files (pass to runSingleScenario)
│   │   ├── run_template.json        ← Documented template (copy + edit)
│   │   ├── my_run.json              ← Your custom run
│   │   ├── rcs_demo.json            ← Airliner vs stealth RCS comparison
│   │   ├── crossing_test.json       ← Crossing targets, swap analysis
│   │   ├── compound_demo.json       ← Multi-degradation demo
│   │   ├── rain_demo_sband.json     ← S-band rain comparison
│   │   ├── rain_demo_xband.json     ← X-band rain comparison
│   │   ├── README.md
│   │   ├── showcase/                ← Pre-built scenarios
│   │   └── validation/              ← Test plan run files (do not edit)
│   │       ├── tc01–tc08e .json     ← 12 test case configs
│   │       └── README.md
│   ├── sensors/                     ← Per-type sensor folders
│   │   ├── PSR/                     ← Primary Search Radar
│   │   │   ├── default_PSR.json
│   │   │   ├── xband_PSR.json       ← 9 GHz X-band variant (rain tests)
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
│   │   ├── rcs_demo/               ← Airliner vs stealth at 100km
│   │   ├── range_rcs_test/         ← 747 vs stealth bomber compound test
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
│   │   └── runDetections.m          ← Detection generator (6 env layers)
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
│   │   ├── applyRainDegradation.m  ← ITU-R P.838-3 via rainpl()
│   │   ├── applyDopplerFade.m      ← MTI clutter notch
│   │   ├── buildRCSProfile.m       ← Aspect-dependent RCS patterns
│   │   └── generateTerrain.m       ← Procedural heightmap generator
│   ├── +validation/
│   │   └── validateScenarioConfig.m ← Pre-flight checks (10 categories)
│   └── +analysis/
│       └── analyzeTrackSwaps.m      ← Track swap detection
│
├── scripts/
│   ├── runSingleScenario.m          ← Main entry point
│   ├── runTestPlan.m                ← Automated validation (27 checks)
│   ├── runComparisonDemo.m          ← Clean vs rain, all 3 trackers
│   ├── verifySimulation.m           ← 40+ checks across 8 phases
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
| `waypoints` | User-defined flight path | `waypoints` array with pos + time_s |
| `head_on` / `parallel` | Mapped to constant_velocity | `heading_deg` |

All targets require: `speed_kmh`, `start_pos` [x,y,z NED], `altitude_m`.

Optional per-target fields: `rcs_dbsm` (scalar RCS), `rcs_profile` (stealth/fighter/airliner/drone/missile), `dimensions` (for visualization), `class_id`, `label`.

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

Six layers of physically-motivated effects, each independently toggleable in terrain configs.

### 1. Horizon Masking
4/3 effective Earth radius model (`isAboveHorizon.m`). Configurable `refraction_factor`. Targets below radar horizon are invisible.

### 2. Ground Clutter
Terrain-dependent false returns at low elevation angles (`generateGroundClutter.m`).

| Terrain | Returns/scan | Noise (m) |
|---------|-------------|-----------|
| water | ~1–2 | 50 |
| rural | ~3–6 | 100 |
| urban | ~8–15 | 150 |
| mountain | ~5–10 | 200 |

### 3. Propagation Model (VCP)
`radarvcd`-based vertical coverage patterns (`computeVerticalCoverage.m`, `applyVCPMask.m`). Multipath ground-bounce creates interference nulls. Configurable: `"propagation_model": "vcp"` or `"propfactor"` or `false`.

### 4. Terrain Occlusion
Procedural heightmaps via `groundSurface` API (`generateTerrain.m`). Line-of-sight checks between every sensor-target pair via `SurfaceManager.occlusion()`.

### 5. Doppler/MTI Fade
Targets flying tangentially lose detections in the clutter notch (`applyDopplerFade.m`). MDV auto-computed from radar frequency. Configurable: `"doppler_fade": true/false`, `"mdv_ms": 40`.

### 6. Rain Degradation
ITU-R P.838-3 frequency-dependent attenuation via `rainpl()` (`applyRainDegradation.m`). S-band barely affected; X-band severely degraded. Weather clutter generated in sensor FOV. Noise scales with rain rate + wet radome loss. Configurable: `"degradation": {"enabled": true, "rain_rate_mmhr": 16}`.

### Terrain Presets

| terrain_type | Occlusion | Masking | Clutter | Density | Max Elevation |
|-------------|-----------|---------|---------|---------|--------------|
| `water` | OFF | OFF | OFF | 0 | 0m |
| `rural` | ON | ON | ON | 0.3 | ~73m |
| `urban` | ON | ON | ON | 0.6 | ~150m |
| `mountain` | ON | ON | ON | 0.5 | ~1960m |
| `desert` | ON | ON | ON | 0.2 | ~40m |

### Sensor Coverage Visualization
Range rings (360° rotators) and sector wedges (PAR, FLIR) drawn on 3D ground plane.

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
      ├── trackbench.environment.applyDopplerFade
      ├── trackbench.environment.applyRainDegradation (if enabled)
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

## Technical Notes

### Frequency vs RCS — How They Affect Detection

**Frequency** (`frequency_hz` in sensor JSON) is **metadata only** at the `fusionRadarSensor` level. It has zero effect on detections under clear weather. The frequency tag is consumed by `applyRainDegradation.m` when rain is enabled — S-band (2.8 GHz) gets ~0.2 dB attenuation at 100km while X-band (9 GHz) gets ~8 dB. To observe the frequency effect, rain degradation must be active.

**RCS** (`rcs_dbsm` in target JSON) is a **first-class sensor property**. It sets `platform.Signatures = rcsSignature(...)` which `fusionRadarSensor` reads natively in its internal radar equation to compute detection probability per scan. A -10 dBsm stealth target genuinely gets fewer raw detections than a 20 dBsm airliner — but only at ranges where the SNR difference matters (typically >80% of the sensor's reference range). At short range, both targets have overwhelming SNR and both get detected every scan.

## Known Issues

- **Sonar sensors**: Maritime scenario builds sonar sensors but `runDetections` skips them (sonar uses `sonarEmission` step interface). Only maritime radar generates detections.
- **Terrain visibility at small scale**: Rural terrain (73m peaks) is physically present but visually flat at aircraft altitude. Use mountain terrain to see mesh.
- **RCS effect at short range**: At ranges <50% of radar reference range, even very low RCS targets (-10 dBsm) have near-100% Pd due to R⁴ radar equation scaling. Place targets at >80km from PSR (111km ref range) to observe meaningful RCS differential.

## Required Toolboxes

- **Sensor Fusion and Tracking Toolbox** (R2024a+) — trackers, sensors, theaterPlot, metrics
- **Radar Toolbox** — `horizonrange`, `radarvcd`, `landroughness`, `earthSurfacePermittivity`, `refractiveidx`, `rainpl`
- **Mapping Toolbox** — `groundSurface`, `SurfaceManager` for terrain occlusion

## Team

Boeing-sponsored senior capstone project — Team Adeptus (Daniel Trofimchik, James Gallegos, Kaz Foster, Michael Harding, Olega Obini).

## License

Boeing Proprietary.

---

## Change Log

### v3.3.0 — March 22, 2026 (current)

**Automated Validation Test Plan** — New `scripts/runTestPlan.m` executes 9 test cases (27 individual checks) with automated pass/fail reporting.
- TC-01: Template usability — user-created config runs end-to-end
- TC-02: Baseline clear — all 3 trackers (GNN, JPDA, TOMHT) produce metrics
- TC-03: Rain S-band — 2.8 GHz at 16 mm/hr, tracking holds
- TC-04: Rain X-band — 9 GHz at 16 mm/hr (same rate as TC-03), fewer target detections than S-band
- TC-05: RCS verification — 20 dBsm airliner vs -10 dBsm stealth at 100km, detection ratio >1.15x
- TC-06: Crossing swap — JPDA swap count ≤ GNN swap count
- TC-07: Compound stress — TOMHT + 50 mm/hr rain + mountain terrain + mixed RCS
- TC-08: Config error paths — 5 malformed/missing configs each produce clear error messages
- TC-09: Verification suite — `verifySimulation.m` completes all 8 phases
- Results saved to `results/test_plan_results_<timestamp>.mat`

**Test Run Files** — 12 validation-specific run files moved to `config/runs/validation/` to keep the main runs folder clean for user experiments. `runTestPlan` references them via `"validation/tc01_template_user"` etc. — `loadRunFile` supports subdirectory paths natively.

**RCS Demo Geometry Fix** — `config/targets/rcs_demo/default_rcs_demo.json` targets moved from 30km to 100km range. At 30km, both 20 dBsm and -10 dBsm targets had overwhelming SNR (Pd ≈ 1.0 for both). At 100km near the PSR reference range, the 30 dB RCS gap produces a measurable Pd differential (confirmed: Airliner=21 dets, Stealth=18 dets, ratio=1.17x).

### v3.2.0 — March 21, 2026

**Rain Attenuation via MATLAB `rainpl()`** — Replaced hand-coded ITU coefficient table with MATLAB's official Phased Array Toolbox `rainpl()` function (ITU-R P.838-3). Falls back gracefully if toolbox unavailable. Full inline citations for Boeing briefing.

**Aspect-Dependent RCS Profiles** — Targets can now have realistic angle-varying radar cross sections.
- New `buildRCSProfile.m` in `+environment/`: creates full azimuth×elevation `rcsSignature` pattern matrices
- 5 preset profiles: `stealth`, `fighter`, `airliner`, `drone`, `missile`
- Stealth profile: -10 dBsm nose-on, +7 dBsm broadside, -1 dBsm rear
- `fusionRadarSensor` natively interpolates the pattern at each scan — no custom code needed
- Target JSON: `"rcs_profile": "stealth"` with `"rcs_dbsm": -10` as base value

**Doppler/MTI Fade** — Targets flying tangentially (across the radar beam) now lose detections.
- New `applyDopplerFade.m` in `+environment/`: computes radial velocity per detection
- Targets with |v_radial| < MDV (Minimum Detectable Velocity) fall into the clutter notch
- MDV auto-computed from radar frequency and assumed PRF (~40 m/s for S-band)

**Custom Waypoint Trajectories** — New `"behavior": "waypoints"` for user-defined flight paths.
- Define arbitrary [x,y,z] waypoints with arrival times in target JSON
- Uses MATLAB `waypointTrajectory` directly — any path, any timing, any complexity

**Enhanced Target Definitions** — All target types now support optional fields:
- `rcs_dbsm`, `rcs_profile`, `dimensions`, `class_id`, `label`
- New `config/targets/target_template.json` with documented examples

**Vertical Process Noise Fix** — IMM/CV vertical process noise = 400 m/s² (was 1 m/s²).

**Range-Adaptive Performance Metrics** — Track-to-truth assignment threshold = 5% of max truth range. Quality score: posRMS as percentage of scenario range.

**Tracker Improvements** — `ConfirmationThreshold` and `DeletionThreshold` now wired from JSON configs. JPDA probability-based thresholds. Crossing-pair tracker configs with wider gates.

**New Demo Scenarios** — `rain_demo_baseline/sband/xband`, `crossing_test`, `rcs_demo`, `range_rcs_test`.

### v3.1.0 — March 20, 2026

**Physics-Based Rain Degradation Model** — ITU-R P.838-3 rain attenuation with frequency-dependent specific attenuation coefficients. Weather clutter in sensor FOV. Measurement noise scales with rain rate.

**Verification Suite** — `scripts/verifySimulation.m` runs 40+ automated checks across 8 phases.

**showTruth N-Target Fix** — `runTracker.m` loops over actual target count instead of hardcoding 2.

**Legacy Cleanup** — Removed 38 legacy V2 files.

### v3.0.0 — March 15–19, 2026

**Modular Config Architecture** — Complete restructure from monolithic config to individual component files. Run files assemble sensors + targets + terrain + trackers by reference. Detection caching per run file. `tracker_globals.json` separates shared params from per-tracker tuning.

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

**Last Updated:** March 22, 2026
**Version:** 3.3.0
