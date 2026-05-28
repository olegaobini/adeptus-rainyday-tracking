# HANDOFF — Sensor Validation & Perfection

**Project:** Rainy Day Tracker (TrackBench) — UW ECE Senior Capstone, Boeing-sponsored
**Repo:** `C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking`
**Working branch:** `Michael---Working_on_pathEditor`
**Active version when this handoff was written:** v3.6.8
**MATLAB environment:** R2025b, Windows
**Boeing demo deadline:** **2026-05-29** (read this date, plan accordingly)
**Sole developer:** Michael Harding

---

## Mission for this chat

Bring sensor types from "framework declared" to "demo validated."
`buildSensor.m` exposes **23 named sensor variants** but only **PSR** has
been actively exercised end-to-end in the recent sessions. Your job is
to systematically validate each declared type, fix what's broken,
improve the visualization parity, and produce a per-type status
matrix that can stand up to Boeing engineers asking "what happens
when I configure an AESA / IRST / passive sonar / ADSB receiver?"

You are **not** responsible for codebase-wide bug hunting outside the
sensor pipeline — that's the parallel `HANDOFF_BUGHUNT.md` chat. If
you find non-sensor bugs incidentally, document them and flag for the
user; don't fix them yourself unless they directly block sensor
validation.

---

## Context Michael will not repeat (read this carefully)

### The 23 declared sensor types

In `src/+trackbench/+sensors/buildSensor.m` switch statement:

**Radar (10):**
`PSR`, `ASR`, `ARSR`, `PAR`, `TWS`, `AESA`, `FIRE_CONTROL`, `WEATHER`,
`MARITIME`, `CUSTOM_RADAR`

**Beacon (effectively 3):**
`ADSB_TX`, `ADSB_RX`, plus MSSR/SSR/IFF (created via
`buildIFFSensor.m` or `buildSensor` with metadata-tagged type)

**IR (4):**
`IRST`, `IR_STARING`, `FLIR`, `CUSTOM_IR`

**Sonar (4):**
`ACTIVE_SONAR`, `PASSIVE_SONAR`, `TOWED_ARRAY`, `CUSTOM_SONAR`

**LIDAR (2):**
`LIDAR`, `CUSTOM_LIDAR`

**Generic (1):**
`CUSTOM`

### What's already known to work

**PSR (Primary Surveillance Radar, mechanical rotator, S-band):**
this is the canonical demo path. PosterDemo runs PSR end-to-end with
all four exercised degradations (terrain occlusion, horizon masking,
ground clutter, Doppler fade) and produces real tracker metrics:
- T1 posRMS = 1233m (Good)
- T2 posRMS = 340m (Excellent)
- T3 posRMS = 2044m (Good)
- Avg posRMS = 1787m
- 98%/98%/73% Tracked%
- 0 track swaps
- Truth3 est failure 27% (expected — harder fighter target)

This is your gold standard. Anything you change must NOT degrade
PosterDemo. After every fix:
```
clear classes; clear all; rehash
runSingleScenario("PosterDemo")
```
and confirm the numeric output is bit-identical (or document why a
non-zero diff is correct).

### Architectural conventions (DO NOT BREAK)

1. **Editor-style stationary sensors live at scenario origin.**
   `platform.Trajectory.Position = [0, 0, 0]`, with world coordinates
   stored in `sensor.MountingLocation`. The radar's effective world
   position is composed by `fusionRadarSensor` as
   `plat.Position + sensor.MountingLocation`. This was hard-won in
   v3.6.5 — don't try to "hoist" the platform.

2. **`targetPoses(plat)` returns target positions in platform's local
   NED frame.** Because platforms are at origin, local frame coincides
   with scenario frame and the entire downstream pipeline (occlusion,
   horizon, truth log, tracker association) reads `targetPoses` as
   absolute coords. Any new sensor type that breaks this assumption
   (e.g., a moving sensor) needs frame transforms applied carefully.

3. **`Platform.InitialPosition` does NOT exist.** Use
   `Trajectory.Position`. This typo cost 5 fix iterations on 2026-05-25.

4. **`fusionRadarSensor.ElectronicScanAngle` does NOT exist.**
   Documented properties: `ElectronicScanLimits`, `LookAngle`,
   `MechanicalAngle`, `MountingAngles`. This typo was silently
   producing wrong ground-clutter geometry until v3.6.7.

5. **Silent `try/catch` on property reads is dangerous.** If you
   write `try X.SomeProperty; catch; Y = default; end` and the
   property doesn't exist, the fallback fires forever and the bug
   stays hidden. Use `isprop(X, 'SomeProperty')` instead, or remove
   the catch and let it throw loudly.

6. **After any `+trackbench/` package edit, MATLAB caches classes
   aggressively.** Always run `clear classes; clear all; rehash`
   before testing.

7. **`drawSensorCoverage.m` color convention:**
   - PSR (rotating radar) = blue
   - Sector radar (non-rotating) = green
   - MSSR/IFF = orange
   - IR = magenta
   - Everything else (sonar, lidar, ADSB) = cycled default

8. **`drawBeamEnvelope.m` gates rendering:** only draws cones for
   `isRadar || isIR` and skips MSSR. Sonar/LIDAR/ADSB get coverage
   rings but no 3D beam cones. This is intentional but worth being
   aware of.

### What v3.6.5–v3.6.8 changed (2026-05-25 session)

| Version | File | Change |
|---|---|---|
| v3.6.5 | `+config/loadRunFile.m` | Reverted to v3.6.1 platform-at-origin convention |
| v3.6.5 | `+detections/runDetections.m` | `cov.position` bugfix: `InitialPosition` → `Trajectory.Position` |
| v3.6.6 | `+reporting/drawSensorCoverage.m` | Sensor marker Z now at sensor altitude (was ground level) |
| v3.6.6 | `+validation/validateScenarioConfig.m` | Underground check uses composed sensor world position |
| v3.6.7 | `+detections/runDetections.m` | Clutter `sParams.tilt`: `ElectronicScanAngle(2)` → `MountingAngles(2)` |
| v3.6.8 | `+detections/runDetections.m` | Per-scan log labels PSR/MSSR → Primary/Beacon |

Full changelog in `README.md` under "Change Log" section.

### Files modified this session that may have CRLF→LF line-ending side effects
- `loadRunFile.m`
- `runDetections.m`
- `drawSensorCoverage.m`
- `validateScenarioConfig.m`

Use `git diff -w` for review.

---

## Methodology — per-sensor-type validation gate

For each sensor type, work through this gate **in order**. Document
each step's outcome in a per-type findings file (see "Output" below).

### Gate 1: Build inspection
- Open `src/+trackbench/+sensors/buildSensor.m`, find the relevant
  `case` branch
- Read what properties get set on the sensor object
- For each property write, cross-check against the MATLAB R2025b doc
  for the underlying SDK class (`fusionRadarSensor`, `irSensor`,
  `sonarSensor`, etc.). The audit on 2026-05-25 specifically checked
  `fusionRadarSensor` properties; do the same for IR and sonar.
- Note any defaults that look implausible (e.g., FieldOfView=[1.4 30]
  for a beam radar — that's PSR's value, may not fit other types)

### Gate 2: Build & introspect
- Construct one sensor of this type in the MATLAB workspace using
  `trackbench.sensors.buildSensor(idx, 'TYPE')` or via JSON load
- Inspect the resulting object: what class is it? What properties did
  actually get set? Use `disp(sensor)` and `properties(sensor)` to
  see
- Compare to a built PSR for parity

### Gate 3: Classification
- Add the sensor to a minimal test scenario (one platform, one target)
- Run `runSingleScenario` and look at the `[runDetections] Sensor N:`
  line — does it print the right `className`, `ScanMode`, `isRotator`,
  `isMechanical`?
- After v3.6.8 the sensor-count line reads
  `Primary sensors: N | Beacon sensors: N` — confirm your sensor
  lands in the right bucket

### Gate 4: Detection physics
- Does the sensor produce detections at all?
- Are detections in the right coordinate frame (scenario absolute)?
- Does the `total=N` count per scan look reasonable for the geometry?
- For radar/IR: does Doppler fade / RCS filter / weather degradation
  apply correctly? (These are gated by `isRadar`, `isIR`, etc.)
- For MSSR/beacon: does the `TargetIndex → PlatformID` mapping
  produce correct `ObjectClassID`?

### Gate 5: Visualization
- 3D scenario plot: does `drawSensorCoverage` render a coverage
  ring/wedge? In what color?
- Does `drawBeamEnvelope` render a 3D cone? If skipped (sonar, lidar,
  ADSB, MSSR), is that intentional or a gap?
- Does the sensor marker render at the correct altitude (v3.6.6 fix)?
- For non-radar sensors that get cycled default colors: should they
  have a dedicated color? Boeing audience may find lack of
  color coding confusing in a multi-sensor scenario.

### Gate 6: Tracker integration
- Do detections from this sensor reach the tracker?
- Does the tracker produce sensible tracks?
- For MSSR-typed sensors: does the track-ID-from-PlatformID path work?
- Does `Tracked%` look reasonable given the scenario geometry?

### Gate 7: Documentation
- Update or create a config JSON in `config/sensors/<TYPE>/` that
  serves as a canonical example for this sensor type
- Note any "gotchas" — quirks of MATLAB's SDK that surprised you
- Add to the per-type validation matrix (see "Output" below)

---

## Phased priority queue

### Phase A — Demo-adjacent (must validate or accept gaps before 2026-05-29)

1. **PSR** — ✓ already validated, just confirm gates 1–7 still pass
2. **MSSR / SSR / IFF** — handled via metadata tag, the classifier
   already routes them through the beacon pipeline. Find/create a
   canonical MSSR config and run gates 3–6.
3. **IR (IRST or FLIR)** — the codebase already gates Doppler and
   weather degradation to `isRadar || isIR`, so it's been partially
   integrated. Validate end-to-end.

### Phase B — Common Boeing-Q&A surface

4. **TWS (Track-While-Scan)** — important for airborne radar use cases
5. **AESA (Active Electronically Scanned Array)** — modern radars,
   would be a natural Boeing question
6. **PAR (Precision Approach Radar)** — sector radar with tight FOV
7. **ASR (Airport Surveillance Radar)** — primary radar, S-band
8. **ARSR (Air Route Surveillance Radar)** — long-range, L-band
9. **WEATHER** — special-purpose radar
10. **MARITIME** — special-purpose radar

### Phase C — Framework completeness (post-demo OK)

11. **ACTIVE_SONAR**
12. **PASSIVE_SONAR**
13. **TOWED_ARRAY**
14. **CUSTOM_SONAR**
15. **LIDAR**
16. **CUSTOM_LIDAR**
17. **ADSB_TX / ADSB_RX**
18. **FIRE_CONTROL**
19. **CUSTOM_RADAR / CUSTOM_IR / CUSTOM**

Don't push Phase C work into a Boeing demo claim unless you've
actually validated them. Better to ship a smaller, real, defensible
scope than a wider claimed surface that breaks under questioning.

---

## Output expected from this chat

1. **`docs/SENSOR_VALIDATION_MATRIX.md`** — a per-sensor-type table
   showing pass/fail/N-A status across all 7 gates. Boeing-presentable.
2. **`config/sensors/<TYPE>/example.json`** — canonical JSON config
   for each Phase A and Phase B type (some may already exist).
3. **`config/runs/test_<TYPE>.json`** — minimal test scenario for
   each Phase A and Phase B type. These let the user quickly smoke-
   test by running `runSingleScenario("test_<TYPE>")`.
4. **Code fixes** — for any bugs found in build paths, classification,
   visualization, or integration.
5. **README change-log entries** — bump version per non-trivial fix,
   following the v3.6.5–v3.6.8 entry style.

---

## What is OUT of scope for this chat

- **PosterDemo.json** — don't change it. It's the demo lock.
- **Path editor / Flight Data Manager** — separate workstream.
- **Codebase-wide bug hunting unrelated to sensors** — that's
  `HANDOFF_BUGHUNT.md`'s job. Flag findings but don't chase them.
- **New sensor types** — work with the 23 already declared.
- **`generateTerrain.m` "radar hilltop clearing" refactor** — known
  legacy assumption, deferred to post-demo (see v3.6.8 README entry).
- **EXE rebuild** — Michael will do that after all today's fixes
  land. Don't trigger `build_executable` yourself.

---

## Verification before declaring done

- [ ] PosterDemo still produces bit-identical numeric output to v3.6.8
- [ ] Each Phase A type passes all 7 gates or has its failures clearly
      documented
- [ ] Validation matrix has no question marks for Phase A and Phase B
- [ ] Color and beam-envelope rendering is at least *consistent* for
      a multi-sensor scenario (even if some types fall back to cycled
      defaults, that should be intentional and noted)
- [ ] Any new test scenarios pass `runSingleScenario` without error
- [ ] README change log reflects the work

---

## Suggested opening message for the new chat

> I'm working on the Rainy Day Tracker capstone project (Boeing-sponsored,
> demo 2026-05-29). The full handoff context is at
> `HANDOFF_SENSORS.md` in the project root
> (`C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking`).
> Please read that file first, then start with Phase A, step 1 (PSR
> verification gate pass).
>
> Use `Filesystem:read_text_file` to read the handoff and any source
> files. The full repo is at the path above. After any code change in
> `+trackbench/`, advise me to run `clear classes; clear all; rehash`
> in MATLAB before testing.
>
> Boeing demo is Friday — keep PosterDemo bit-identical or flag any
> deltas explicitly.

---

## Git workflow

When ready to push:
```
cd "C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git"
git add adeptus-rainyday-tracking/
git commit -m "Sensor validation pass: <summary>"
git push origin Michael:main --force-with-lease
```

Coordinate with the BUGHUNT chat via Michael — don't push without
checking if the other chat has uncommitted work in the same files.
