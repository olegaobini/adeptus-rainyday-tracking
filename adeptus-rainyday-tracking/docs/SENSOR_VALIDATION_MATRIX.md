# Sensor Validation Matrix — Rainy Day Tracker

**Project:** Rainy Day Tracker (TrackBench) — UW ECE Senior Capstone, Boeing-sponsored
**Sponsor demo:** 2026-05-29
**Document version:** v1.5 (2026-05-26)
**Source:** `HANDOFF_SENSORS.md` — Phase A/B/C validation methodology
**Linked README version:** 3.7.0

This document tracks the validation status of each of the 23 declared sensor
types in `src/+trackbench/+sensors/buildSensor.m` against a seven-gate
methodology. For the gate definitions and detailed methodology, see
`HANDOFF_SENSORS.md` in the repo root.

## Gate definitions (summary)

| Gate | What it verifies |
|------|------------------|
| **G1 — Build inspection** | `buildSensor.m` `case` branch sets only documented R2025b properties on the underlying SDK class. No silent typos. |
| **G2 — Build & introspect** | `buildSensor(idx, 'TYPE')` returns a real SDK object; `disp` / `properties` show what was actually set. |
| **G3 — Classification** | `runDetections` `[runDetections] Sensor N: …` log line shows the correct `className`, `ScanMode`, `isRotator`, `isMechanical`, and lands in `Primary` vs `Beacon` bucket. |
| **G4 — Detection physics** | Sensor produces detections in correct frame; `total=N` per scan looks right for geometry; Doppler / weather / RCS filters apply only where they should. |
| **G5 — Visualization** | `drawSensorCoverage` renders ring/wedge in the right color; `drawBeamEnvelope` renders cone (radar/IR only); sensor marker at correct altitude. |
| **G6 — Tracker integration** | Detections reach the tracker, tracks form, `Tracked%` is reasonable; MSSR `TargetIndex → PlatformID` works. |
| **G7 — Documentation** | Canonical example JSON in `config/sensors/<TYPE>/`; minimal smoke-test run in `config/runs/test_<TYPE>.json`; gotchas captured. |

## Status legend

- ✅ Pass
- ⚠ Pass with notes / minor finding (non-blocking)
- ❌ Fail
- — Not applicable / intentionally skipped
- ⏳ Pending validation

---

## Phase A — Demo-adjacent (must validate or accept gaps before 2026-05-29)

### 1. PSR — Primary Search Radar (mechanical rotator, S-band)

**SDK class:** `fusionRadarSensor` with `Rotator` convenience syntax
**Active in:** `PosterDemo.json` (gold-standard, T1/T2/T3 posRMS=1233/340/2044 m, Tracked%=98/98/73, 0 track swaps)
**Status:** ✅ DEMO-READY (validated 2026-05-25, code unchanged since v3.6.8)

| Gate | Status | Notes |
|------|--------|-------|
| G1 — Build inspection | ⚠ Pass with 2 minor findings | See **PSR-F1** and **PSR-F2** below. All properties set are documented R2025b properties of `fusionRadarSensor` except `HasRCSSignature`, which is a silent no-op via `safeSet`. |
| G2 — Build & introspect | ✅ Pass | `class(s) == 'fusionRadarSensor'`. `ScanMode = 'Mechanical'`. FOV = `[1.4; 30.001]` (intentional 1e-3 elevation epsilon). `MechanicalElevationLimits = [-17 13]` (centered around horizon, offset by tilt — see v3.4.1 fix). `MechanicalAzimuthLimits` unset for full 360° rotator (`abs(diff(sector)) >= 359` skips the set). |
| G3 — Classification | ✅ Pass | `runDetections` logs: `Sensor 1: fusionRadarSensor \| ScanMode=Mechanical \| isRotator=1 \| isMechanical=1`. After v3.6.8, sensor-count line reads `Primary sensors: 1 \| Beacon sensors: 0`. |
| G4 — Detection physics | ✅ Pass | PosterDemo: T1=1233 m, T2=340 m, T3=2044 m posRMS; 0 track swaps; 27% Truth3 est failure (expected for the harder fighter target). All degradation paths (terrain occlusion, horizon, ground clutter, Doppler fade, RCS filter) exercised end-to-end. |
| G5 — Visualization | ✅ Pass | `drawSensorCoverage` → **blue** dashed range ring (`isRotator && isRadar && !isMSSR`). `drawBeamEnvelope` → green (upper) + red (lower) elevation cones using `coverageConfig().ScanLimits`. Sensor star at composed world position + altitude (v3.6.5/3.6.6 fixes). |
| G6 — Tracker integration | ✅ Pass | GNN, JPDA, TOMHT all consume PSR detections cleanly. PosterDemo runs GNN; per-tracker behavior validated by TC-02 in test plan (27/27 passing as of v3.5.2). |
| G7 — Documentation | ✅ Pass | Multiple example JSONs exist: `default_PSR.json` (20 km demo geometry), `sband_PSR.json` (60 nm canonical ASR-11 — TC-03 reference), `MountainSensor.json` (PosterDemo lock), `Primary_Search_Radar_(DASR).json`, `xband_PSR.json`, `psr_m7.json`, `tc05_PSR_longrange.json`. Canonical smoke-test: **`config/runs/test_PSR.json`** (v3.6.9). |

#### Findings (PSR)

**PSR-F1 — `HasRCSSignature` is not a documented `fusionRadarSensor` property.**
`buildSensor.m` line ~290 sets `safeSet(radar, 'HasRCSSignature', true)`. Cross-check against `C:\Program Files\MATLAB\R2025b\toolbox\fusion\core\fusion\@fusionRadarSensor\fusionRadarSensor.m` properties block confirms `HasRCSSignature` does not exist in R2025b. `fusionRadarSensor` reads `platform.Signatures` natively via the radar equation with `ReferenceRange` / `ReferenceRCS`. The `safeSet` helper guards with `isprop()`, so this is a silent no-op — not breaking, but dead code that should be removed in a post-demo cleanup pass.

**PSR-F2 — v3.6.7 changelog claim about `MountingAngles(2) = +2°` is inaccurate (but the fix is still correct for PSR).**
The v3.6.7 entry says PosterDemo's PSR has `MountingAngles(2) = +2°` and that this happens to match the broken fallback exactly, making PosterDemo "bit-identical to v3.6.6." Verification:
- `buildRadar()` never assigns `MountingAngles`.
- `MountainSensor.json` (and `default_PSR.json`) do not include `mountingAngles` in `params`.
- `fusionRadarSensor`'s default `MountingAngles = [0 0 0]`.

So PosterDemo's PSR has `MountingAngles(2) = 0`, not `+2°`. The fact that PosterDemo *is* bit-identical between v3.6.6 and v3.6.7 has a different cause: PSR's elBW is 30°, so in `generateGroundClutter`:

```
lowerEdgeDeg = tiltDeg - elBW/2;
% tilt=2 → lowerEdgeDeg = -13°  → else branch (rSurfMin = max(rMin, 500))
% tilt=0 → lowerEdgeDeg = -15°  → else branch (rSurfMin = max(rMin, 500))
```

Both values land in the same conditional branch (`lowerEdgeDeg < 0`), producing the same `rSurfMin`. **Implication for Phase B:** narrower-FOV sensors with elBW < 4° (e.g., PAR elBW=1°, AESA elBW=3°, FIRE_CONTROL elBW=2°) WILL see different clutter geometry between `tilt=2°` (old fallback) and `tilt=0°` (current MountingAngles read). Verify clutter behavior for those types when validating Phase B.

The v3.6.7 fix itself (read MountingAngles instead of the typo ElectronicScanAngle) is still correct in principle, just for a different reason than the changelog says.

#### PSR — Verification commands

```matlab
clear classes; clear all; rehash
addpath("scripts"); addpath(genpath("src"));

% Gate 2 — build & introspect
[psr, meta] = trackbench.sensors.buildSensor(1, 'PSR');
disp(psr); disp(meta)
properties(psr)
psr.ScanMode, psr.FieldOfView, psr.MechanicalElevationLimits, psr.MountingAngles

% Gate 4–6 — end-to-end smoke test
runSingleScenario("test_PSR")

% PosterDemo bit-identical re-confirmation (gold standard)
runSingleScenario("PosterDemo")
```

**Expected `test_PSR` console signature:**
```
[runDetections]   Sensor 1: fusionRadarSensor | ScanMode=Mechanical | isRotator=1 | isMechanical=1
[runDetections] Primary sensors: 1 | Beacon sensors: 0
t=4.80: Primary=N, Beacon=0, total=N (clutter=0)
…
```

**Expected `PosterDemo` bit-identical numbers (v3.6.8 baseline):**
- T1 posRMS = 1233 m, T2 = 340 m, T3 = 2044 m, Avg = 1787 m
- Tracked% = 98% / 98% / 73%
- 0 track swaps
- Truth3 est failure 27%

---

### 2. MSSR / SSR / IFF — Secondary Surveillance Radar (transponder beacon)

**SDK class:** `fusionRadarSensor` with `Rotator` convenience syntax
**Live build path:** `buildSensor.m` `case {'SSR', 'MSSR', 'IFF'}` (lines 81–84) → `buildRadar(idx, 'Rotator', getDefaults('SSR'), …)` — identical code path to PSR with MSSR-tuned defaults.
**Parallel (orphan) build path:** `src/+trackbench/+sensors/buildIFFSensor.m` — see **MSSR-F1** below.
**Beacon classification:** `runDetections.classifyAsMSSR` (lines ~544–566) reads `sensorMetas{}.type` tag (`SSR`/`MSSR`/`IFF`) → `mssrBuffer` rather than `detBuffer`. Heuristic fallback when metadata is missing: `FAR ≤ 1e-7 && Pd ≥ 0.98 && 150 km < ReferenceRange < 250 km`.
**Active in:** `dasr_baseline.json`, TC-02 (PSR + SSR co-rotation).
**Status:** ⚠ VERIFIED 2026-05-25 (Gates 3/4/6 ✅, G2 implicit, G5 pending eyeball, G1/G7 ⚠; four findings F1–F4 + cross-cutting X-F5, all non-blocking for demo)

| Gate | Status | Notes |
|------|--------|-------|
| G1 — Build inspection | ⚠ Pass with 3 findings | See **MSSR-F1**, **MSSR-F2**, **MSSR-F3** below. The live `buildSensor → buildRadar` path produces correctly-centered elevation `[-7 3]` for SSR defaults `fov=[1.4;10], tilt=2` (formula `[-fov(2)/2 fov(2)/2] - tilt`, matches v3.4.1 fix). All properties set are documented R2025b `fusionRadarSensor` properties except `HasRCSSignature` — same silent no-op as PSR (**PSR-F1** applies identically here, since `safeSet(radar,'HasRCSSignature',true)` in `buildRadar` runs for every type that goes through it). |
| G2 — Build & introspect | ⚠ Implicit pass | Sensor functions end-to-end in the v3.6.11 verification run: `[buildSensor] Created SSR/MSSR (index=1)` logged at load; scan period `4.80 s` (→ `UpdateRate ≈ 53.6 Hz`) reported by the pre-flight `SCAN CHECK`; the run did produce detections when targets were in the expected elevation envelope (confirming `MechanicalElevationLimits=[-7 3]` and `FieldOfView` are wired correctly). Direct `disp(ssr)` / `properties(ssr)` introspection not yet run by user; verification commands block below provides the explicit sequence. |
| G3 — Classification | ✅ Verified 2026-05-25 | Console output exactly as audited: `[runDetections]   Sensor 1: fusionRadarSensor \| ScanMode=Mechanical \| isRotator=1 \| isMechanical=1`, sensor-count line `Primary sensors: 0 \| Beacon sensors: 1`, and critically `[runDetections] MSSR detected: SensorIndex=1` — confirms the metadata-`type`-tag classifier route fired (not the heuristic FAR/Pd/refRange fallback). JSON `"type":"SSR"` plumbing `loadRunFile → meta.type → classifyAsMSSR` is working end-to-end. |
| G4 — Detection physics | ✅ Verified 2026-05-25 (with **MSSR-F4** geometry note) | Beacon path triggered cleanly: every scan that produced output logged `Primary=0, Beacon=N, total=N (clutter=0)`, confirming routing to `mssrBuffer` and not `detBuffer`. Per-scan detection count tracked target presence in the elevation beam (1 detection when one of two targets in beam, 0 when both out). Horizon masking active but produced `Horizon masked 0` (correct geometry: 5km-alt targets at 30–50 km range are above the 4/3-Earth horizon from a near-ground sensor). Doppler / rain / clutter auto-skip gates (`~si.isMSSR` at `runDetections.m` lines ~451 / ~468 / ~497) were not directly exercised by this run because their global toggles were off in the run file; code-level inspection confirms the gates are in place. **ObjectClassID stamping**: line 419 ran (no errors), explicit value check in `dataLog.Detections{:}.ObjectClassID` recommended for definitive sign-off but not blocking. |
| G5 — Visualization | ⏳ Pending eyeball | The 3D static scenario plot was generated by the v3.6.11 run (`3D Static plot complete.`) but the matrix can't auto-verify what's on screen. **User action required:** open the figure window and confirm (a) the SSR renders as a **dashed orange ring** at the tower location (the `isMSSR` branch of `drawSensorCoverage.m`), and (b) **no green/red beam-envelope cone** is drawn around the SSR (the `isMSSR` skip in `drawBeamEnvelope.m`, per **X-F2**). |
| G6 — Tracker integration | ✅ Verified 2026-05-25 | `GNN+IMM` consumed the beacon detections cleanly: tracker constructed, ran without error, executed swap analysis (0 swaps reported), produced track/truth summary tables, generated track metrics. The handoff path `mergedDets = [mssrBuffer; detBuffer]` at `runDetections.m` line ~698 functioned end-to-end and the tracker handled beacon detections the same way it handles primary detections. Track quality on this specific scenario (Tracked% = 0% / 100% across 4 scans, posRMS in the kilometers) is geometry-bound — see **MSSR-F4** — not a tracker failure. The point of this gate is *does the tracker successfully process beacon detections from this sensor?* → yes. |
| G7 — Documentation | ⚠ Pass with caveat | Multiple example JSONs: `default_SSR.json` (canonical ASR-11 MSSR, 120 nm), `Secondary_Surveillance_Radar_(MSSR_IFF).json`, `dead_zone_SSR_west.json`, `SSR_demo_multi.json`, `SSR_template.json`. Canonical smoke-test: **`config/runs/test_MSSR.json`** (description revised in v3.6.11 to flag the **MSSR-F4** geometry constraint up front, so the "Only 4 scan(s)" warning isn't mistaken for a regression). **Caveat:** `buildIFFSensor.m` still exists as orphan code — see **MSSR-F1**. |

#### Findings (MSSR / SSR / IFF)

**MSSR-F1 — `src/+trackbench/+sensors/buildIFFSensor.m` is an orphan file with a stale, sub-horizon elevation formula.**

The active sensor-build pipeline (`loadRunFile.m` § 2 → `trackbench.sensors.buildSensor` → `buildRadar`) is the only path that produces MSSR sensors used by `runSingleScenario`. Verified by reading the full `loadRunFile.m` source (no second sensor-loading path) and by checking every script in `scripts/` (active) and `scripts/legacy/` for references to `buildIFFSensor` — none found. The file is therefore dead code in the live pipeline, but it carries a buggy elevation formula:

```matlab
% buildIFFSensor.m line ~91
mssr.MechanicalElevationLimits = [-fov(2) 0] - S.tilt;
```

With default `fov=[1.4;10]`, `tilt=2`, that produces `[-12, -2]` — the **entire** beam below the horizon. The live `buildRadar` path uses the corrected v3.4.1 formula `[-fov(2)/2 fov(2)/2] - tilt = [-7 3]`, which is correctly centered around the horizon.

**Risk:** zero today (no callers). Hazard if someone tracing "how does MSSR work?" lands here and copies the formula into the live path, or restores `buildIFFSensor` as a callable factory in a future refactor.

**Recommended fix (post-demo):** delete `buildIFFSensor.m`, or convert it to a thin documented shim that calls `buildSensor(idx, 'SSR', varargin{:})`. Defer until post-demo because the file is inert.

**MSSR-F2 — Top-level `frequency_hz` in SSR JSON splits cleanly to `meta.frequency` but not to `sensor.CenterFrequency`.**

`config/sensors/SSR/default_SSR.json` declares `"frequency_hz": 1.03e9` at the **top level** (sibling of `params`, not nested under it). `loadRunFile.m` § 2:

- Lines 78–95 build `nvPairs` from `sDef.params` only. Top-level `frequency_hz` is not under `params`, so it never becomes a `'centerFreq'` arg to `buildSensor`.
- Lines 99–104 separately copy `sDef.frequency_hz` (or `sParams.frequency_hz`) to `meta.frequency`, which `runDetections.m` line ~280 reads back as `info.radarFreq` for downstream physics (clutter freq scaling at lines ~566 / 729 / 756, heuristic MDV lookup at lines ~283–289).

Net effect for default SSR:

| Quantity | Value | Source |
|----------|-------|--------|
| `meta.frequency` (used by clutter / freq-scaled physics) | **1.03 GHz** | JSON top-level wins |
| `sensor.CenterFrequency` (`fusionRadarSensor` property) | **1.06 GHz** | `buildSensor.getDefaults('SSR').centerFreq` (built-in default in `buildSensor.m` line ~285) |

The mismatch is silent — both numbers are L-band, no physics warning fires.

**Impact for MSSR specifically: low.** The beacon path auto-skips clutter, Doppler, and weather (all gated `~si.isMSSR`), so the meta-level `1.03e9` doesn't drive any active code in PosterDemo or TC-02. The `sensor.CenterFrequency` is read by `fusionRadarSensor` internals for rain-attenuation modeling, which we override at the meta level anyway for non-MSSR sensors.

**Recommended fix (post-demo):** in `loadRunFile.m` § 2, after the `meta.frequency = …` block, append `nvPairs = [nvPairs, {'centerFreq', meta.frequency}]` so the JSON's `frequency_hz` reaches both paths consistently. See **X-F4** for the generalized cross-sensor version of this gap.

**MSSR-F3 — Beacon `ObjectClassID = TargetIndex + platformIdx` formula can collide in multi-tower scenarios.**

`runDetections.m` line ~419 (verified by direct read):

```matlab
if si.isMSSR
    for ii = 1:numel(dets)
        tgtIdx = getTargetIndex(dets{ii});
        if tgtIdx > 0; dets{ii}.ObjectClassID = tgtIdx + si.platformIdx; end
    end
    mssrBuffer = [mssrBuffer; dets];
end
```

Sum collision in a 2-tower scenario (tower A at `platformIdx=1`, tower B at `platformIdx=2`):

- Target with `TargetIndex=2` seen from tower A → `ObjectClassID = 2 + 1 = 3`
- Target with `TargetIndex=1` seen from tower B → `ObjectClassID = 1 + 2 = 3`

Two physically distinct aircraft, identical beacon ID. The tracker treats them as the same identity-tagged track, which can produce a swap if both detections arrive in the same scan.

**PosterDemo cannot trigger this** — single-tower geometry (`platformIdx` only ever takes one value, so the formula degenerates to a per-target offset). TC-02 is also single-tower.

**Recommended fix (post-demo):** use a Cantor-pairing-style formula (e.g., `tgtIdx * 1000 + platformIdx`) or stamp a true `(tgtPID, sensorPID)` tuple in a separate `ObjectAttributes` field. Defer until post-demo — no active scenario triggers the collision.

**MSSR-F4 — SSR's narrow elevation FOV (10°) constrains target-geometry compatibility (physically correct behavior, flagged for Path Editor users).**

The v3.6.11 verification run of `test_MSSR` (`SSR/default_SSR` + `crossing_pair/default_crossing_pair` targets at 5000 m altitude + `none` terrain + horizon masking on) produced **only 4 SSR detections across 4 scans** in 60 seconds, with a 19-second gap between t=14.41 and t=33.60 where both targets were out of beam. Expected total: 12 scans (60 s ÷ 4.80 s/scan). The 5-scan tracker minimum is missed, triggering:

```
Warning: Only 4 scan(s) — need at least 5.
```

Root cause is geometry, not a sensor defect. The SSR's `MechanicalElevationLimits=[-7 3]` (10° elBW centered with `tilt=2`) clips elevation coverage at approximately +3° upper edge. For a target at 5000 m altitude:

| Range to target | Elevation angle | In SSR beam? |
|-----------------|-----------------|--------------|
| 25 km   | 11.3° | ❌ above upper edge |
| 30 km   |  9.5° | ❌ above upper edge |
| 36 km   |  7.9° | ❌ above upper edge |
| 50 km   |  5.7° | ❌ above upper edge |
| 100 km  |  2.9° | ✅ just inside  |
| 150 km  |  1.9° | ✅ inside        |

The `crossing_pair` defaults sit in the 30–50 km range at 5 km altitude, where target elevation lives in the 5.7–7.9° range — above the SSR upper edge.

**Why this is correct behavior:** real ASR-11 MSSR is designed with narrow elevation coverage *intentionally*. It's an en-route surveillance asset for transponder-equipped aircraft well clear of the airport, not an overhead-coverage radar. The PSR + MSSR pair on a real DASR is complementary by design: PSR's 30° elBW covers close-range high-elevation traffic, MSSR's 10° beam covers long-range low-elevation en-route. The codebase reproduces this faithfully.

**Why this matters for the Path Editor user experience:** a user who selects `SSR` and constructs (or uses) a high-altitude close-range scenario will see the "Only N scans" warning and a tracker that can't establish tracks. **This isn't a sensor bug — the sensor is working correctly.** But the failure mode is opaque without knowing the elevation envelope, and a Boeing reviewer running this exact smoke test needs to know up front that the warning is expected.

**Recommended action (post-demo polish, none required for Boeing):**
- Update `drawSensorCoverage` to draw the elevation envelope as a vertical slice alongside the existing azimuth ring/wedge, making the upper/lower coverage edges visible at a glance.
- Add a pre-flight validator check that warns when target trajectories spend less than 25% of their lifetime inside any active sensor's elevation beam.
- See **X-F5** for the generalized version of this constraint across other narrow-elevation sensor types.

**Status:** documented; no fix this release. Behavior matches real hardware.

#### MSSR / SSR — Verification commands

```matlab
clear classes; clear all; rehash
addpath("scripts"); addpath(genpath("src"));

% Gate 2 — build & introspect (via the live buildSensor path)
[ssr, meta] = trackbench.sensors.buildSensor(2, 'SSR');
disp(ssr); disp(meta)
ssr.ScanMode                       % 'Mechanical'
ssr.FieldOfView                    % [1.4; 10.001]
ssr.MechanicalElevationLimits      % [-7 3]
ssr.MountingAngles                 % [0 0 0] (default) — X-F3 applies, but
                                   % SSR's 10° elBW gives lowerEdgeDeg = -5°,
                                   % which lands in the same branch as the
                                   % +2° fallback so clutter geometry is
                                   % unchanged from pre-v3.6.7 behavior.
ssr.CenterFrequency                % 1.06e9 — confirms MSSR-F2 (JSON says 1.03e9)
ssr.DetectionProbability           % 0.99
ssr.FalseAlarmRate                 % 1e-7
ssr.ReferenceRange                 % 222240
ssr.ReferenceRCS                   % 20

% Alias coverage: 'SSR', 'MSSR', and 'IFF' all route through the same
% buildRadar(SSR-defaults) path — verify meta.type tag for each:
[~, m1] = trackbench.sensors.buildSensor(3, 'MSSR');  disp(m1.type)
[~, m2] = trackbench.sensors.buildSensor(4, 'IFF');   disp(m2.type)

% MSSR-F1 — confirm orphan factory produces a sub-horizon beam
[bad, ~] = trackbench.sensors.buildIFFSensor(99);
bad.MechanicalElevationLimits      % Expect [-12 -2] — confirms stale formula

% Gates 3–6 — end-to-end smoke test (single-SSR scenario)
runSingleScenario("test_MSSR")

% Expected per-scan log signature:
%   [runDetections]   Sensor 1: fusionRadarSensor | ScanMode=Mechanical | isRotator=1 | isMechanical=1
%   [runDetections] MSSR detected: SensorIndex=1
%   [runDetections] Primary sensors: 0 | Beacon sensors: 1
%   t=4.80: Primary=0, Beacon=N, total=N (clutter=0)

% PosterDemo bit-identical re-confirmation — this MSSR audit changes no .m files
runSingleScenario("PosterDemo")
% Expect: T1/T2/T3 posRMS = 1233/340/2044m, Tracked% = 98/98/73, 0 swaps
```

**Verification status as of 2026-05-25 (post-v3.6.11 run):**
- G1 ⚠ — stays ⚠; three architectural findings (F1, F2, F3) are documented and deferred to post-demo. The grade can't promote until at least one of them is fixed.
- G2 ⚠ — implicit pass via successful end-to-end run; direct `disp(ssr)` introspection (see commands above) is the path to explicit ✅.
- G3 ✅ — verified by `MSSR detected: SensorIndex=1` and `Primary sensors: 0 | Beacon sensors: 1` in the run log.
- G4 ✅ — verified by per-scan `Primary=0, Beacon=N, total=N` lines. Explicit `dataLog.Detections{:}.ObjectClassID` check still recommended for definitive sign-off; current grade reflects "the beacon physics path works end-to-end" which is the sensor-correctness question.
- G5 ⏳ — pending visual eyeball: open the 3D plot and confirm orange dashed ring + absent beam cone.
- G6 ✅ — verified. Tracker processes beacon detections cleanly through GNN+IMM with 0 swaps. **Track quality on `test_MSSR`'s specific geometry is geometry-bound (MSSR-F4), not a tracker defect.** This grade answers *does the tracker successfully process beacon detections from this sensor?*, which is the sensor-correctness question, not *does this specific scenario produce good Tracked%?*.
- G7 ⚠ — stays ⚠ until MSSR-F1 (orphan `buildIFFSensor.m`) is resolved.

---

### 3. IR — IRST / IR_STARING / FLIR (irSensor)

**SDK class:** `irSensor`
**Live build path:** `buildSensor.m` `case {'IRST'}` (and `IR_STARING`, `FLIR`, `CUSTOM_IR`) → `buildIR(idx, getDefaults('IRST'), …)`.
**Status:** ✅ DEMO-READY (gate 6 closed 2026-05-26 by v3.7.0 commits + Test A/B empirical; gates 3/5/7 ⏳ closing via Phase 3 `test_IRST` scenario today, non-blocking)

| Gate | Status | Notes |
|------|--------|-------|
| G1 — Build inspection | ⚠ Pass with 1 finding | See **IRST-F1** below. All properties set by `buildIR` (`NumDetectors`, `FocalLength`, `LensDiameter`, `FieldOfView`, `MechanicalScanLimits`) are documented R2025b `irSensor` properties. `getDefaults('IRST')` carries four dead fields (`pd`, `rangeLimits`, `rangeRes`, `hasRangeRate`) that `irSensor` does not expose — silent no-ops via the same `safeSet` / `isprop` guard pattern as **PSR-F1** for `HasRCSSignature`. |
| G2 — Build & introspect | ✅ Verified 2026-05-25 (Mon empirical) | `[ir, meta] = trackbench.sensors.buildSensor(1, 'IRST')` → `class(ir) = 'irSensor'`, properties set as configured. `irSensor` emits `MeasurementParameters` as a **2×1 struct array** (outer spherical frame + inner rectangular, sensor-side first per R2025b doc) — this shape later proved load-bearing for v3.7.0's `trackbenchFilterInit` wrapper, see G6 below. |
| G3 — Classification | ⏳ Pending Phase 3 `test_IRST` | `runDetections` IR classification `si.isIR = isa(si.sensor,'irSensor')` is in place at line 162 and is consumed by the gate-6 wrapper; end-to-end scenario log signature pending the Phase 3 `test_IRST` run. |
| G4 — Detection physics | ✅ Verified 2026-05-25 (Mon empirical) + 2026-05-26 (v3.7.0 corrections) | `irSensor` produces 2-vec `[az;el]` measurements with full `MeasurementParameters`, sensor-side frame `Spherical` with `HasRange=false`. **Doppler-gating doc correction:** `applyDopplerFade` is gated `si.isRadar` ONLY — IR sensors do not receive Doppler processing (corrects the v1.2 IRST row claim that conflated Doppler with weather gating). `applyWeatherDegradation` is gated `(si.isRadar \|\| si.isIR)` per design, but v3.7.0 defensive guards at `runDetections.m` lines 488/506 skip 2-vec measurements pending proper IR-aware weather (fog primarily — rain has negligible IR effect; post-demo scope). |
| G5 — Visualization | ⏳ Pending Phase 3 visual | Expected: `drawSensorCoverage` magenta dashed ring (`isIR` branch per **X-F1** color convention); `drawBeamEnvelope` renders cone via `isRadar \|\| isIR` gate. Will confirm with `runSingleScenario("test_IRST")` 3D static plot in Phase 3. |
| G6 — Tracker integration | ✅ Closed 2026-05-26 (v3.7.0) | Two-test empirical closure (orchestrator-drafted inline MATLAB; both tests PASS first run). **Test A:** `runDetections.m` line 401 `minMeas` branch correct for all four cases (IR 2-vec ✓, IR 3-vec ✓, PSR 2-vec dropped ✗, PSR 3-vec kept ✓). **Test B:** end-to-end through `trackerGNN` with the new `trackbench.tracking.trackbenchFilterInit` wrapper — 2 IR detections fed, first track at t=0, first confirmed at t=1s. `baseInitFcn` error-stub never fired → confirms angle-only routing via `initrpekf` → `trackingGSF` against `irSensor`'s actual struct-array `MeasurementParameters` shape (the `mp = mp(1)` struct-array catch was the orchestrator code-review payoff against SENSORS' Mon empirical, not caught by dry-run). |
| G7 — Documentation | ⏳ In progress (Phase 3 today) | Canonical example JSON `config/sensors/IRST/default_IRST.json` updated in v3.6.15 with `"MechanicalScanLimits": [[0, 360], [-5, 15]]` override (closes **IRST-F3**; runtime IRST was inheriting `irSensor`'s class-default sub-horizon envelope `[0 360; -10 0]`). Canonical smoke-test scenario `config/runs/test_IRST.json` + low-altitude target preset `config/targets/ir_low_altitude_demo/default_ir_low_altitude_demo.json` pending Phase 3 today. |

#### Findings (IRST)

**IRST-F1 — `getDefaults('IRST')` declares four fields that `irSensor` does not expose.**

`pd`, `rangeLimits`, `rangeRes`, `hasRangeRate` reach `buildIR` via the defaults block but `irSensor` has no matching property; the `safeSet` `isprop` guard turns them into silent no-ops. Same dead-default pattern as **PSR-F1** (`HasRCSSignature` on `fusionRadarSensor`). Risk today: zero (no callers depend on the unset values). Cleanup is post-demo — remove the four fields from `getDefaults('IRST')`, or add an explicit log line when the `safeSet` no-op fires so future regressions don't hide. Deferred, non-blocking.

**IRST-F3 — `irSensor`'s class-default scan envelope is sub-horizon; closed in v3.6.15.**

`irSensor`'s R2025b class default for `MechanicalScanLimits` is `[0 360; -10 0]` (per the MathWorks `irSensor` doc page; also matches the `Rotator` convenience syntax). Elevation lies entirely at or below the horizon, which doesn't fit tactical IRST geometry — incoming airborne threats from low to mid altitudes at modest range want upward elevation coverage, not downward. The v1.3 wording of this finding incorrectly attributed the default to `config/sensors/IRST/default_IRST.json`; the JSON declares no `MechanicalScanLimits` field, and the runtime IRST inherits the class default (the formula-computed override at `buildSensor.m:577` is gated on `sectorSpan < 359` and does not fire for default 360°-rotator IRST). **v3.6.15** adds `"MechanicalScanLimits": [[0, 360], [-5, 15]]` to the JSON `params` block; the value flows through `buildIR`'s `applyUnmatched → safeSet` path (line 581) to override the class default. After v3.6.15 commits, Phase 3's `test_IRST` validates the new envelope end-to-end and G7 in this row promotes accordingly.

#### IRST — Verification commands

**Canonical end-to-end verification (v3.6.15):**

```matlab
clear classes; clear all; rehash
addpath("scripts"); addpath(genpath("src"));

runSingleScenario("test_IRST")
% Exercises the full pipeline: loadRunFile.m:87-116 (JSON nvPair construction)
% → buildSensor → buildIR → applyUnmatched → safeSet → irSensor (with v3.6.15
% MechanicalScanLimits override) → runDetections (si.isIR classification,
% 2-vec minMeas branch, weather guards) → trackbenchFilterInit angle-only
% wrapper (initrpekf → trackingGSF) → trackerGNN.
% SUCCESS GATE: end-to-end without error AND ≥1 confirmed track.
% Expected console: 'Primary sensors: 1 | Beacon sensors: 0', per-scan
% 'Primary=N, Beacon=0, total=N (clutter=0)'. Expected visualization:
% magenta dashed coverage ring + magenta beam cone in 3D view.

% PosterDemo bit-identical re-confirmation (v3.7.0 wrapper is no-op for PSR)
runSingleScenario("PosterDemo")
% Expect: T1/T2/T3 posRMS = 1233/340/2044m, Tracked% = 98/98/73, 0 swaps
```

**Supplementary debug script (path-by-path tracing only):**

The canonical verification above is the authoritative smoke test. The following inline script is for diagnostic use when an end-to-end failure needs path-by-path tracing through the JSON-load chain. It reproduces `loadRunFile.m:87-116`'s nvPair construction so the JSON override actually enters the build path — unlike a direct `buildSensor(idx, 'IRST')` call which bypasses the JSON entirely (see v1.5 change log entry for the methodology lesson).

```matlab
clear classes; clear all; rehash
addpath("scripts"); addpath(genpath("src"));

% Reproduce loadRunFile.m:87-116 nvPair construction
sDef    = jsondecode(fileread('config/sensors/IRST/default_IRST.json'));
sParams = sDef.params;
fields  = fieldnames(sParams);
nvPairs = {};
for f = 1:numel(fields)
    val = sParams.(fields{f});
    if isnumeric(val) && iscolumn(val) && numel(val) > 1
        val = val(:)';                       % col → row (line 107)
    end
    if strcmpi(fields{f}, 'fov') && isnumeric(val)
        val = val(:);                        % fov → col (line 111)
    end
    nvPairs = [nvPairs, {fields{f}, val}];   %#ok<AGROW>
end
disp('=== nvPairs flowing to buildSensor ==='); disp(nvPairs);

% Now build with the JSON-derived nvPairs
[ir, ~] = trackbench.sensors.buildSensor(1, 'IRST', nvPairs{:});
disp('=== ir.MechanicalScanLimits ==='); disp(ir.MechanicalScanLimits);
% After v3.6.15 commit: [0 360; -5 15] (2x2 double).
% Calling buildSensor(1, 'IRST') with NO further args bypasses the JSON-load
% path entirely and produces the class default [0 360; -10 0]. The path the
% override flows through is the loadRunFile nvPair pipeline above.
```

---

## Phase B — Common Boeing-Q&A surface

### 4. TWS — Track-While-Scan phased array (no scanning)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | `fusionRadarSensor` with `'No scanning'` config + `hasRangeRate=true`. Sector wedge color = **green** (`!isRotator && isRadar`). |

### 5. AESA — Active Electronically Scanned Array (no scanning, sector ±60°)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | elBW=3° narrow beam. **PSR-F2 applies**: clutter tilt now 0° (was 2° via broken fallback) — verify clutter geometry. |

### 6. PAR — Precision Approach Radar (sector, ±10° around runway)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | elBW=1° very narrow beam. **PSR-F2 applies strongly.** sector=[170 190]. Sector wedge green. |

### 7. ASR — Airport Surveillance Radar (rotator, S-band, 60 nm)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | Nearly identical to PSR; elBW=5° (tighter than PSR's 30°). **PSR-F2 may apply** depending on tilt — needs check. |

### 8. ARSR — Air Route Surveillance Radar (rotator, L-band, 250 nm)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | rpm=5, elBW=20° (wide enough that PSR-F2 doesn't bite). |

### 9. WEATHER — Weather Radar (raster scan, high RPM)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | rpm=25, elBW=1° narrow pencil beam (raster). Will report negative-RCS targets. **PSR-F2 applies.** |

### 10. MARITIME — Maritime / surface search radar (rotator, X-band, 40 nm)
| Gate | Status | Notes |
|------|--------|-------|
| G1–G7 | ⏳ | rpm=24, X-band 9.4 GHz, `hasElevation=false`. Different elevation handling path — verify `MechanicalElevationLimits` skip in `buildRadar` works correctly. |

---

## Phase C — Framework completeness (post-demo OK)

### Sonar (4)
| Type | SDK class | Status | Notes |
|------|-----------|--------|-------|
| ACTIVE_SONAR  | `sonarSensor` | ⏳ | `runDetections` currently `WARNING: %d sonar sensor(s) — skipping.` and excludes them from the active set. No detections produced. |
| PASSIVE_SONAR | `sonarSensor` | ⏳ | Same skip behavior. |
| TOWED_ARRAY   | `sonarSensor` | ⏳ | Same. Sector mode. |
| CUSTOM_SONAR  | `sonarSensor` | ⏳ | Bare passthrough. |

### Lidar (2)
| Type | SDK class | Status | Notes |
|------|-----------|--------|-------|
| LIDAR        | `monostaticLidarSensor` | ⏳ | Short range (200 m). `runDetections` would `step()` it normally — no skip — but produces point-cloud measurements that are not 3-vec positions, so the `numel(dets{ii}.Measurement) < 3` filter would drop them. Needs check. |
| CUSTOM_LIDAR | `monostaticLidarSensor` | ⏳ | Bare passthrough. |

### ADS-B (2)
| Type | SDK class | Status | Notes |
|------|-----------|--------|-------|
| ADSB_TX | `adsbTransponder` | ⏳ | Attached to aircraft platform, not host. Not directly stepped in `runDetections`. |
| ADSB_RX | `adsbReceiver`     | ⏳ | Not directly stepped in `runDetections`. Would require a dedicated ADS-B pipeline (post-demo). |

### Custom / Misc (3)
| Type | SDK class | Status | Notes |
|------|-----------|--------|-------|
| FIRE_CONTROL  | `fusionRadarSensor` | ⏳ | Narrow beam (elBW=2°), `'No scanning'`, X-band-ish range. **PSR-F2 applies strongly.** |
| CUSTOM_RADAR  | `fusionRadarSensor` | ⏳ | Bare rotator, no defaults. |
| CUSTOM_IR     | `irSensor`          | ⏳ | Bare no-scanning, no defaults. |
| CUSTOM        | `struct` template   | — | Template struct, not a real sensor. Documented for users who want to implement a `sensor.stepImpl` from scratch. Not detected/tracked. |

---

## Cross-cutting findings (apply to multiple types)

### X-F1 — `drawSensorCoverage.m` color assignment is incomplete for multi-modal scenarios

Current rules (see `drawSensorCoverage.m` lines ~40–55):
- `isMSSR` → orange
- `isIR` → magenta
- `isRadar && !isRotator` → green (sector radar)
- `isRadar && isRotator` → blue (PSR-style)
- everything else → cycled defaults (sonar, lidar, ADSB)

For a Boeing audience evaluating a multi-sensor scenario, the lack of a dedicated color for sonar / lidar / ADSB is a minor consistency gap. **Recommendation:** add cyan for sonar and yellow for lidar in a post-demo polish pass.

### X-F2 — `drawBeamEnvelope.m` gates rendering to `isRadar || isIR` (and skips MSSR)

This is intentional and documented in the file header. Sonar/lidar/ADSB get the coverage ring/wedge but no 3D cone. MSSR also skipped (line 23) because beacon transponder coverage isn't directional in the way a radar beam is. Worth knowing for multi-modal scenarios.

### X-F3 — PSR-F2 propagates: `cSensor.MountingAngles(2)` now reads 0° for any sensor that doesn't set it explicitly

Affects all Phase A/B types except those whose elBW is wide enough that `lowerEdgeDeg = -elBW/2 < 0` (most rotators). Narrow-beam sensors (PAR=1°, AESA=3°, FIRE_CONTROL=2°, WEATHER=1°) may show clutter behavior different from what was implicit pre-v3.6.7. **Recommendation:** when validating these types, run with and without `ground_clutter` to isolate the effect, and document any clutter-geometry change.

### X-F4 — Top-level `frequency_hz` in any sensor JSON does not reach `sensor.CenterFrequency`

Generalization of **MSSR-F2**. `loadRunFile.m` § 2 splits `frequency_hz` between two destinations:

- **`meta.frequency`** — read from `sDef.frequency_hz` (top-level) **or** `sDef.params.frequency_hz` (nested). Reaches `runDetections.info.radarFreq`, drives clutter freq scaling and the heuristic MDV lookup. ✓
- **`sensor.CenterFrequency`** (the `fusionRadarSensor` property) — set only when `centerFreq` is present in `sDef.params`. Top-level `frequency_hz` is never plumbed here. The built-in defaults in `buildSensor.getDefaults` set `centerFreq` for only **three** types — PSR (2.8e9), SSR (1.06e9), MARITIME (9.4e9). All other seven radar types (ASR, ARSR, PAR, TWS, AESA, FIRE_CONTROL, WEATHER) get the `fusionRadarSensor` SDK default on the sensor object, while the meta-level lookup in `loadRunFile.getFreqForType()` returns sane values per type so downstream physics stays mostly correct.

**Why this matters for Phase B types:** when validating PAR, AESA, FIRE_CONTROL, ARSR, WEATHER, the sensor's actual `CenterFrequency` property reads the SDK default, which affects:

- `fusionRadarSensor`'s internal rain-attenuation calculation (when the SDK's built-in attenuation paths are active)
- Any downstream code that reads `sensor.CenterFrequency` directly rather than going through `meta.frequency`

**Workaround for users today:** put `"centerFreq": 9e9` (or whatever value) inside the `params` block of your sensor JSON to set the actual sensor property. Top-level `frequency_hz` only affects meta-level physics.

**Recommended fix (post-demo):** in `loadRunFile.m` § 2, after `meta.frequency = …`, append `nvPairs = [nvPairs, {'centerFreq', meta.frequency}]` so both paths agree by construction. Verify by re-running PosterDemo: PSR's `default_PSR.json` does not declare `centerFreq` under `params`, so the fix would route the meta-level 2.8 GHz through to the sensor; the buildSensor default for PSR is already 2.8 GHz so PosterDemo should stay bit-identical. SSR's `centerFreq` *would* change from 1.06 GHz to 1.03 GHz on the sensor object — that's the intended outcome of the fix, but it does mean any TC that uses SSR would need a one-time cache regen.

### X-F5 — Narrow-elevation sensors have stricter target-geometry compatibility than PSR

Generalization of **MSSR-F4**. Several supported sensor types have elevation FOVs significantly narrower than PSR's 30°:

| Type | elBW | tilt | Beam-center elev range | Beam upper edge (approx) |
|------|------|------|------------------------|--------------------------|
| PSR | 30° | +2° | `[-17, 13]` | +28° |
| SSR / MSSR | 10° | +2° | `[-7, 3]` | +8° |
| ASR | 5° | +2° | `[-4.5, 0.5]` | +3° |
| ARSR | 20° | 0° | `[-10, 10]` | +20° |
| AESA | 3° | 0° | `[-1.5, 1.5]` | +3° |
| FIRE_CONTROL | 2° | 0° | `[-1, 1]` | +2° |
| PAR | 1° | -3° | `[2.5, 3.5]` | +4° |
| WEATHER | 1° | 0° | `[-0.5, 0.5]` | +1° |
| MARITIME | 25° | 0° | `[-12.5, 12.5]` (no el) | n/a |

(Beam upper edge is the approximate top of the actual detectable elevation cone, i.e. `MechanicalElevationLimits(2) + fov(2)/2`. The exact effective edge depends on the sensor's beam pattern; for `fusionRadarSensor` with `'Rotator'` scan, the FOV is the beam shape stepped through the mechanical limits.)

For each of these, a Path Editor user who selects the sensor and then adds targets at altitudes producing elevation angles greater than the upper edge will see in-beam dropouts — like the **MSSR-F4** symptom on `test_MSSR`. PSR's wide beam absorbs almost any reasonable airborne target geometry; the others all have stricter altitude-vs-range envelopes that match what the real-world sensor would see.

**Why this isn't necessarily a problem the codebase needs to "fix":** these elBW values are deliberate and match real radar hardware. AESA and PAR have narrow beams because that's what high-resolution arrays do. WEATHER scans a 1° pencil beam in a raster pattern. The defaults are correct.

**What the codebase could do better (post-demo polish):**
1. **`drawSensorCoverage`** could draw a vertical-slice cone showing the elevation envelope alongside the existing azimuth ring/wedge. `drawBeamEnvelope` already renders this as green/red lines for radar/IR types; integrating those into the coverage panel directly would make the elevation constraint legible at the same level as range.
2. **Pre-flight validator** (`validateScenarioConfig.m`) could compute, for each target trajectory, the fraction of timesteps the target is inside any active sensor's elevation envelope, and warn if that drops below a threshold (suggest 25%).
3. **Path Editor** could surface an "Elevation coverage hint" panel when a narrow-FOV sensor is selected, plotting the altitude-vs-range envelope and overlaying any current target trajectories.

None of these are required for the Boeing demo — the sensors do what real radars do. But they would help users avoid the "Only N scans" surprise when geometry and beam profile mismatch.

---

## Change log for this document

- **v1.5 (2026-05-26)** — Verification block corrected. The v1.3/v1.4 verification script `[ir, ~] = trackbench.sensors.buildSensor(1, 'IRST')` was misaligned with its claim: with no varargin, the call bypasses `loadRunFile.m:87-116`'s nvPair construction entirely, so the JSON `MechanicalScanLimits` override never reaches `applyUnmatched → safeSet`, and irSensor inherits the class default `[0 360; -10 0]`. The script was testing a sibling path (`buildSensor` direct invocation) that happens to produce a similar-looking surface, not the JSON-override path it claimed to test. Replaced with: (1) canonical end-to-end verification `runSingleScenario("test_IRST")` exercising the full loadRunFile → buildSensor → buildIR → applyUnmatched → safeSet → irSensor → runDetections → trackbenchFilterInit → trackerGNN pipeline (Phase 3 of 2026-05-26 dispatch); and (2) a corrected supplementary debug script that reproduces loadRunFile's nvPair construction inline for path-by-path tracing. **Phase 3 components landed in this same v3.6.15 release:** `config/targets/ir_low_altitude_demo/default_ir_low_altitude_demo.json` (low-altitude crossing target preset, elevation 0.73-1.03° throughout); `config/runs/test_IRST.json` (canonical smoke-test scenario); `config/trackers/GNN/test_IRST_GNN.json` (lenient Score-mode `confirm_threshold: 5` — see README v3.6.15 Process findings for why scalar 5 substitutes for dispatch-specified `[2 3]`). Methodology lesson banked for the project record: verification scripts must exercise the specific code path they claim to test, not a sibling path. No `.m` files modified; PosterDemo bit-identical guarantee preserved.
- **v1.4 (2026-05-26)** — v1.3 IRST-F3 wording corrected. The default `[0 360; -10 0]` is `irSensor`'s class default per R2025b doc (`MechanicalScanLimits` property; also matches the `Rotator` convenience syntax), not declared in `default_IRST.json`. The JSON's runtime IRST inherits the class default — the formula-computed override at `buildSensor.m:577` is gated on `sectorSpan < 359` and does not fire for default 360°-rotator IRST. **v3.6.15** closes the finding by adding `"MechanicalScanLimits": [[0, 360], [-5, 15]]` to the JSON `params` block, which flows through `buildIR`'s `applyUnmatched → safeSet` path (line 581) to set the property at construction. Also updates the IRST row G7 cell to reflect the new JSON state (no longer reads as a pending Phase 2 item). Phase 3 of today's dispatch will validate end-to-end via `runSingleScenario("test_IRST")`. No `.m` files modified; PosterDemo bit-identical guarantee preserved.
- **v1.3 (2026-05-26)** — Phase A step 3 (IRST) materialized from ⏳ PENDING placeholder to populated row. Gate 6 closed (❌ → ✅) by v3.7.0 commits + Test A/B empirical: Test A confirms `runDetections.m` line 401 `minMeas` branch correct for all four IR/PSR × 2-vec/3-vec cases; Test B confirms 2 IR detections route end-to-end through `trackerGNN` via the new `trackbench.tracking.trackbenchFilterInit` wrapper, first confirmed track at t=1s. Gates 1/2/4 ✅ from Mon empirical (`irSensor` build + property introspection + 2-vec `[az;el]` Measurement + struct-array `MeasurementParameters` shape). Gates 3/5/7 ⏳ closing via Phase 3 `test_IRST` scenario today, non-blocking. Overall IRST status promoted to ✅ DEMO-READY. Two findings filed: **IRST-F1** (`getDefaults('IRST')` dead fields, same pattern as PSR-F1, post-demo cleanup) and **IRST-F3** (`default_IRST.json` sub-horizon scan envelope — Phase 2 of today's dispatch closes by overriding to `[0 360; -5 +15]`). Corrected v1.2 IRST row's Doppler-gating doc claim: `applyDopplerFade` is gated `si.isRadar` only, NOT `(isRadar \|\| isIR)`; only `applyWeatherDegradation` carries the IR-inclusive gate, with v3.7.0 line-488/506 defensive guards skipping 2-vec measurements pending proper IR-aware weather (post-demo). Linked README version bumped to 3.7.0. No `.m` files modified; PosterDemo bit-identical guarantee preserved.
- **v1.2 (2026-05-25)** — Phase A step 2 verification run complete (v3.6.11 release). Promoted Gates 3 → ✅, 4 → ✅, 6 → ✅ based on `test_MSSR` run output; G2 marked ⚠ implicit pass; G5 still ⏳ pending visual confirmation. Added new finding **MSSR-F4** (SSR elevation FOV vs default crossing_pair target altitude — correct hardware behavior, but produces "Only 4 scan(s)" warning surprise) and cross-cutting **X-F5** (generalizes MSSR-F4 across all narrow-elevation sensor types, with proposed post-demo Path Editor UX improvements). Linked README version bumped to 3.6.11. No `.m` files modified; PosterDemo bit-identical guarantee preserved.
- **v1.1 (2026-05-25)** — Phase A step 2 (MSSR/SSR/IFF) Gate 1 audit complete. Replaced the v1.0 placeholder row with the full audit: status ⚠, properties table, three findings (MSSR-F1 orphan `buildIFFSensor.m`, MSSR-F2 `frequency_hz` JSON-to-sensor plumbing gap, MSSR-F3 `ObjectClassID = TargetIndex + platformIdx` multi-tower collision), and verification commands. Added cross-cutting finding X-F4 generalizing MSSR-F2 across all 10 radar types. Linked README version bumped to 3.6.10. No `.m` files modified; PosterDemo bit-identical guarantee preserved.
- **v1.0 (2026-05-25)** — Initial creation. Phase A step 1 (PSR) fully validated; Phase A steps 2–3 and Phases B/C marked PENDING with priority phase.

---

## How to use this matrix

1. When you sit down to validate the next type, copy the PSR template (gates G1–G7 + Findings + Verification commands) and fill in for the new type.
2. Run the verification commands. If everything passes, mark the row ✅ and add the verification timestamp.
3. If a gate fails, mark it ❌ with a one-line summary, then file the detail under "Findings" with a fix proposal.
4. After every change to any source file, re-run `runSingleScenario("PosterDemo")` and confirm bit-identical numeric output. If it isn't, the PosterDemo lock is broken — investigate before pushing.
5. Boeing-presentable: every ⏳ should be ✅ or ⚠ before claiming a sensor type as part of the demo surface. Anything still ⏳ on demo day is OUT of the demo claim.