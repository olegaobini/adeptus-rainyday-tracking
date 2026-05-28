# HANDOFF — Codebase Bug Hunt

**Project:** Rainy Day Tracker (TrackBench) — UW ECE Senior Capstone, Boeing-sponsored
**Repo:** `C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking`
**Working branch:** `Michael---Working_on_pathEditor`
**Active version when this handoff was written:** v3.6.8
**MATLAB environment:** R2025b, Windows
**Boeing demo deadline:** **2026-05-29**
**Sole developer:** Michael Harding

---

## Mission for this chat

Systematic codebase audit for latent bugs. Today's session (2026-05-25)
found **three real bugs** — `InitialPosition` typo (v3.6.5),
`ElectronicScanAngle` typo (v3.6.7), and PSR/MSSR label confusion
(v3.6.8) — all of which were silently masked by `try/catch` fallbacks
that produced plausible-looking-but-wrong output for years. Your job
is to find more of these, file by file, applying the patterns we've
learned.

You are **not** responsible for sensor-type validation — that's the
parallel `HANDOFF_SENSORS.md` chat. If you find sensor-specific bugs
incidentally, document them and flag for the user; the sensor chat
will deal with them. Focus on the rest of the codebase.

---

## The bug pattern to hunt

Every bug we found today fits the same shape:

```matlab
try
    X = obj.SomeProperty(...);
catch
    X = some_default_value;
end
```

Where `SomeProperty` either (a) is a typo for a real property,
(b) was renamed in a MATLAB SDK version, or (c) never existed at all.
The `catch` makes the failure silent. The fallback produces output
that looks plausible. The bug stays hidden for the life of the
project.

**Examples we found:**

1. `si.platform.InitialPosition` — `Platform` class has no
   `InitialPosition`. Correct: `Trajectory.Position`.
   Effect: every sensor's coverage rendered at scenario origin
   regardless of placement, for the project's entire history.

2. `cSensor.ElectronicScanAngle(2)` — `fusionRadarSensor` has no
   `ElectronicScanAngle`. Correct: `MountingAngles(2)` or `LookAngle(2)`.
   Effect: ground clutter beam tilt pinned at constant 2°,
   geometrically wrong for any sensor not at +2° mount pitch.

3. Per-scan log `"PSR=%d, MSSR=%d"` — counts were correct but labels
   wrong for non-radar scenarios. Effect: an IR scenario would log
   `PSR=8, MSSR=0` despite having zero PSRs. Less dangerous than #1
   and #2 but visibly misleading.

**The lesson:** silent `try/catch` on property reads hides bugs
indefinitely. The audit method is to (a) grep for the pattern,
(b) verify every property name against the R2025b MathWorks docs,
(c) decide whether the fallback is legitimate defensive coding (some
sensor types lack a property by design) or a typo-masking smell.

---

## What's already been audited (DO NOT re-audit)

The 2026-05-25 session audited these 14 files thoroughly:

| File | Status |
|---|---|
| `src/+trackbench/+detections/runDetections.m` | ✓ Audited, 3 bugs fixed |
| `src/+trackbench/+detections/createDetections.m` | ✓ Clean |
| `src/+trackbench/+reporting/drawSensorCoverage.m` | ✓ Audited + v3.6.6 fix |
| `src/+trackbench/+reporting/drawBeamEnvelope.m` | ✓ Clean (no try/catch) |
| `src/+trackbench/+reporting/plotInitialScenario.m` | ✓ Clean (catch with fprintf, not silent) |
| `src/+trackbench/+reporting/plotScenarioAndDetections.m` | ✓ Clean |
| `src/+trackbench/+scenario/addTargetFromDef.m` | ✓ Clean |
| `src/+trackbench/+sensors/buildSensor.m` | ✓ Clean (defensive sonar fallbacks are legit) |
| `src/+trackbench/+sensors/buildCustomFusionRadarSensor.m` | ✓ Clean |
| `src/+trackbench/+tracking/runTracker.m` | ✓ Clean (uses isprop, catch ME with warnings) |
| `src/+trackbench/+environment/applyDopplerFade.m` | ✓ Clean (no try/catch) |
| `src/+trackbench/+environment/applyRCSFilter.m` | ✓ Clean |
| `src/+trackbench/+environment/generateGroundClutter.m` | ✓ Clean |
| `src/+trackbench/+config/loadRunFile.m` | Partially audited (§7 and §9 only) — REST NEEDS AUDIT |

---

## Files still to audit (your priority queue)

### Priority 1 — Physics-touching code (highest impact)

These files run during detection generation and tracker execution.
Bugs here directly affect demo output.

- `src/+trackbench/+config/loadRunFile.m` — the rest of the file
  beyond §7 and §9 (sensor build, target build, weather attachment,
  tracker config loading)
- `src/+trackbench/+detections/getWeather.m` — storm window
  evaluation, weather severity over time
- `src/+trackbench/+environment/applyRainDegradation.m` — rain
  attenuation physics (ITU-R P.838-3 baseline)
- `src/+trackbench/+environment/applyWeatherDegradation.m` — snow,
  fog, icing physics
- `src/+trackbench/+environment/composeHeightmap.m` — multi-region
  terrain compositing
- `src/+trackbench/+environment/generateTerrain.m` — terrain
  generation per-type (note the "radar hilltop clearing" assumption
  flagged in v3.6.8 README — not in scope to fix, but flag if other
  similar assumptions exist)
- `src/+trackbench/+environment/isAboveHorizon.m` — horizon masking
  formula (4/3 Earth, refraction=1.333)
- `src/+trackbench/+environment/resolveRegionIdx.m` — first-wins
  polygon mask for multi-region scenarios
- `src/+trackbench/+environment/resolveTerrainAt.m`
- `src/+trackbench/+environment/resolveWeatherAt.m`
- `src/+trackbench/+environment/buildRCSProfile.m` — RCS aspect
  table construction
- `src/+trackbench/+tracking/buildTracker.m` — tracker construction
  (GNN / JPDA / TOMHT)
- `src/+trackbench/+tracking/initCVFilter.m`
- `src/+trackbench/+tracking/initIMMFilter.m`
- `src/+trackbench/+scenario/validateScanCoverage.m`
- `src/+trackbench/+validation/validateScenarioConfig.m` — Check 1
  was audited and fixed in v3.6.6, but Checks 2–10 still need audit

### Priority 2 — Reporting & analysis (medium impact)

- `src/+trackbench/+reporting/printCompactTrackSummary.m`
- `src/+trackbench/+reporting/printCompactTruthSummary.m`
- `src/+trackbench/+reporting/plotPlatformToTrackAssignment.m`
- `src/+trackbench/+reporting/plotTrackSwapAnalysis.m`
- `src/+trackbench/+reporting/tabbedAxes.m`
- `src/+trackbench/+analysis/analyzeSensitivity.m`
- `src/+trackbench/+analysis/analyzeTrackSwaps.m`
- `src/+trackbench/+analysis/computeTunerScore.m`
- `src/+trackbench/+analysis/extractTunerMetrics.m`
- `src/+trackbench/+sensors/buildIFFSensor.m`
- `src/+trackbench/+sensors/customSensorTemplate.m`
- `src/+trackbench/+sensors/loadSensors.m`

### Priority 3 — Editor & flight data (lower-impact at runtime, but
GUI quality matters for demo)

- `src/+trackbench/+editor/*` (28 files) — these mostly handle plain
  data structs, lower SDK-typo risk. Skim for hardcoded values and
  frame-of-reference issues.
- `src/+trackbench/+flightdata/buildBatchTargetJSON.m`
- `src/+trackbench/+flightdata/loadNASAFlight.m`
- `src/+trackbench/+flightdata/scanFlightFolder.m`

### Priority 4 — Top-level scripts (entry points for the user)

- `scripts/mainMenu.m`
- `scripts/runSingleScenario.m`
- `scripts/runSimGUI.m`
- `scripts/pathEditor.m`
- `scripts/compareTrackers.m`
- `scripts/autoTuneTracker.m`
- `scripts/verifySimulation.m`
- `scripts/runTestPlan.m`
- `scripts/viewSavedResults.m`
- `scripts/validationDocsGUI.m`
- `scripts/runNASAFlight.m`, `scanNASAFlights.m`, `viewNASAFlightGlobe.m`
- `scripts/flightDataManagerGUI.m`
- `scripts/build_executable.m`, `build_installer.m`
- `scripts/diagBeamLimits.m`, `diagnoseBadDetections.m`, `diagVerifyTargetIndex.m`

---

## Audit methodology

For each file (in priority order):

### Step 1: Locate all try/catch blocks
```bash
grep -n "try" <file>
```
For multi-line tries, read the full block. For single-line tries,
the catch is usually on the same line.

### Step 2: Classify each try/catch
For each block, decide which category it belongs to:

**Category A: Silent typo masker (BUG)**
- Reads a property that doesn't exist on the underlying class
- Catch silently substitutes a default
- Action: fix the property name, document in code comment, bump
  version per the v3.6.5–v3.6.8 pattern

**Category B: Legitimate defensive coding (OK)**
- Reads a property that exists on *some* sensor types but not others
  (e.g., `FalseAlarmRate` on radar but not IR)
- Catch provides a sensible fallback for the absent case
- Action: leave alone, but consider adding an `isprop` check
  upgrade for explicitness

**Category C: Wrapping a function call with known failure modes (OK)**
- Wraps a MATLAB SDK function that can fail for environmental
  reasons (e.g., `occlusion(SurfaceManager, ...)` returns empty for
  flat terrain)
- Action: leave alone if the failure mode is genuinely expected

**Category D: Catch ME with warning (good practice)**
- `catch ME` with `fprintf` or `warning` on `ME.message`
- Action: leave alone, this is the right pattern

### Step 3: Cross-check property names
For Category A candidates, verify the property name exists. References:

- **`fusionRadarSensor` properties** — MathWorks docs URL:
  `https://www.mathworks.com/help/fusion/ref/fusionradarsensor-system-object.html`
  Documented: `SensorIndex, UpdateRate, ScanMode, MountingLocation,
  MountingAngles, FieldOfView, RangeLimits, RangeRateLimits,
  AzimuthResolution, ElevationResolution, RangeResolution,
  RangeRateResolution, ReferenceRange, ReferenceRCS, CenterFrequency,
  Bandwidth, DetectionMode, HasElevation, HasRangeRate, HasINS,
  HasFalseAlarms, FalseAlarmRate, DetectionProbability,
  MechanicalScanLimits, ElectronicScanLimits, MaxMechanicalScanRate,
  MechanicalAngle, LookAngle, TargetReportFormat, DetectionCoordinates,
  Signatures (read from platform), ...`

- **`fusion.scenario.Platform` properties** — MathWorks docs:
  `https://www.mathworks.com/help/fusion/ref/fusion.scenario.platform-class.html`
  Documented: `PlatformID, ClassID, Dimensions, Mesh, Position,
  Velocity, Acceleration, Orientation, AngularVelocity, Trajectory,
  Sensors, Emitters, Signatures, PoseEstimator`
  **NOT a property:** `InitialPosition` (was the v3.6.5 bug)

- **`kinematicTrajectory` properties** — `Position, Velocity,
  Acceleration, Orientation, AngularVelocity`

- **`waypointTrajectory` properties** — `Waypoints, TimeOfArrival,
  Velocities, Orientation, WaitTime, AutoPitch, AutoBank, ReferenceFrame`

- **`objectDetection` fields** — `Time, Measurement, MeasurementNoise,
  SensorIndex, ObjectClassID, ObjectAttributes, MeasurementParameters`

- **`irSensor`, `sonarSensor`, `monostaticLidarSensor`** — see
  MathWorks docs per class.

When uncertain, **search the docs**, don't guess.

### Step 4: Verify, fix, document, test
- If you confirm a Category A bug:
  - Apply the fix with a clear v-bumped comment in the code
  - Update `README.md` change log with a new version entry
  - Run `clear classes; clear all; rehash; runSingleScenario("PosterDemo")`
    and confirm output is bit-identical (or document why diff is correct)

### Step 5: Document patterns found
For each file, note in a working doc:
- Number of try/catch blocks
- Category breakdown
- Any Category A bugs (fixed)
- Any Category B blocks that should be upgraded to `isprop` checks
  post-demo

---

## Other audit dimensions beyond try/catch

While reading each file, also watch for:

### Hardcoded values that should be config-driven
- Look for `magic_number` constants in physics paths
- Cross-reference with run config schema — should this be tunable?

### Frame-of-reference assumptions
- Does the code assume platform-at-origin? (no longer always true)
- Does it assume NED vs altitude-up consistently?
- Does it mix coordinate frames implicitly?

### Multi-platform / multi-sensor edge cases
- Does code that loops over platforms or sensors handle `numel > 1`
  correctly?
- Does it assume platform index 1 is always the sensor platform?

### Cache invalidation
- The `cache/*.mat` files have no source-hash stamp (from project
  memory). If a file's source changes, the cache silently goes
  stale. Look for places where this could bite: detection cache,
  terrain cache, RCS table cache.

### CRLF / line-ending issues
- Files known to have CRLF→LF conversion from `Filesystem:edit_file`:
  `drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`,
  `runSimGUI.m`, plus today's edited files. Use `git diff -w` for
  review. If you edit one of these, expect line-ending noise in
  the diff.

### Determinism / RNG
- `generateTerrain.m` uses `rng(42, 'twister')` and restores afterward — good
- Verify other randomized paths do the same (do they save/restore RNG state?)

---

## Output expected from this chat

1. **`docs/CODEBASE_AUDIT_NOTES.md`** — per-file audit log with
   try/catch categorization, findings, and TODOs.
2. **Code fixes** for any Category A bugs found, with clear
   v3.6.x version bumps in code comments and README change log.
3. **Post-demo TODO list** — patterns/upgrades you spotted that are
   *not* bugs but should be cleaned up later (e.g., upgrade
   Category B silent catches to explicit isprop checks).

---

## What is OUT of scope for this chat

- **Sensor-type validation** — that's `HANDOFF_SENSORS.md`. Flag
  sensor bugs you find but don't chase them.
- **PosterDemo.json** — never change it.
- **EXE rebuild** — Michael does that himself.
- **Path editor refactors** — separate workstream.
- **Performance optimization** — only fix demonstrable bugs.
- **The `generateTerrain.m` "radar hilltop clearing"** legacy
  assumption — known issue, deferred post-demo. Flag any *other*
  similar legacy assumptions you find.
- **Cache stamp/invalidation mechanism** — known issue, deferred
  post-demo. Document if you find a concrete case where stale cache
  caused real confusion, but don't build the invalidation system.

---

## Verification

- [ ] PosterDemo produces bit-identical numeric output to v3.6.8
      after every fix
- [ ] Each fix has a README change log entry
- [ ] Audit notes doc covers all Priority 1 and 2 files
- [ ] Any Category A bugs found have been fixed AND tested
- [ ] Post-demo TODO list is concrete and prioritized

---

## Suggested opening message for the new chat

> I'm working on the Rainy Day Tracker capstone project (Boeing-sponsored,
> demo 2026-05-29). The full handoff context is at
> `HANDOFF_BUGHUNT.md` in the project root
> (`C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking`).
> Please read that file first, then start with Priority 1, first file:
> the rest of `src/+trackbench/+config/loadRunFile.m` beyond §7/§9.
>
> Use `Filesystem:read_text_file` to read the handoff and any source
> files. After any code change in `+trackbench/`, advise me to run
> `clear classes; clear all; rehash` in MATLAB before testing.
>
> Boeing demo is Friday — keep PosterDemo bit-identical or flag any
> deltas explicitly. The 2026-05-25 session log has a working
> example of the audit methodology in the README v3.6.7 change log.

---

## Git workflow

When ready to push:
```
cd "C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git"
git add adeptus-rainyday-tracking/
git commit -m "Audit pass: <summary>"
git push origin Michael:main --force-with-lease
```

Coordinate with the SENSORS chat via Michael — don't push without
checking if the other chat has uncommitted work in the same files.
The files `+detections/runDetections.m`, `+sensors/buildSensor.m`,
and `+reporting/drawSensorCoverage.m` are most likely to be touched
by both chats.

---

## Helpful patterns from the 2026-05-25 session

### How to run the audit on a batch of files
1. List the files: `grep -l "try" src/+trackbench/+somedir/*.m`
2. Pull all try lines at once: `grep -n -H "try" src/+trackbench/+somedir/*.m`
3. For multi-line tries, follow up with `sed -n 'N,Mp' <file>`
4. Cross-check property names against MathWorks docs.
5. Flag suspicious ones, dig deeper.

### How to verify a fix is bit-identical
After applying a fix:
```matlab
clear classes; clear all; rehash
runSingleScenario("PosterDemo")
```
Look at the per-scan log values and compare to a known-good run.
For PosterDemo at v3.6.8, the per-scan output should match the
log in the README v3.6.7 entry exactly. Numeric drift = the fix
changed behavior; investigate before committing.

### How to write a good README change log entry
Pattern (see v3.6.5–v3.6.8 entries):
1. **Honest title** describing the bug class
2. **Symptom** — what was visible to the user
3. **Root cause** — the actual line of code and why it was wrong
4. **Fix** — what the new line does and why
5. **Validation expected** — concrete checks after re-running
6. Mark the previous version as `(superseded by vX.Y.Z)`

Don't shy away from postmortem-style honesty. If a bug was hidden
for years, say so. Boeing values engineering maturity over polish.
