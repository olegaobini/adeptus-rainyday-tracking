# BUGHUNT Phase 2 Audit Report

Generated: 2026-05-26 (Tue PM)
Scope: Priority 1 file queue (per HANDOFF_BUGHUNT.md), plus the runDetections.m
line ~386 try/catch line flagged in CHECKPOINT.md post-demo cleanup list.
Methodology: Read-only pattern scan for Category A (try/catch typo-maskers)
and Category B (comprehensive-looking checks that mask bugs). No SDK
behavior verification — pattern matching only.

Auditor: Cowork (read-only pass; no source files modified outside this report).

## Summary

- 8 Category A findings across 4 files (loadRunFile, applyRainDegradation,
  applyWeatherDegradation, runDetections); 2 audited files file-clean for
  Category A (getWeather, isAboveHorizon)
- 6 Category B findings across 4 files (loadRunFile, getWeather,
  applyRainDegradation, applyWeatherDegradation); 2 audited files
  file-clean for Category B (isAboveHorizon, runDetections-line-386
  context)
- 0 Category C findings
- File-clean (no findings in either category): `getWeather.m` Category A,
  `isAboveHorizon.m` both categories, `applyWeatherDegradation.m` Category B
  (other than the cross-file getField pattern noted under loadRunFile).

High-priority recommendations (suggested triage order for BUGHUNT chat):

1. **A7 — `runDetections.m:386`** (HIGH). Already flagged in CHECKPOINT.md
   as a Phase 2 audit target. Wraps the per-sensor System-object step call
   in a bare `catch; continue; end`. ANY failure in the sensor step
   (typo, class behavior change, malformed targets) silently drops the
   entire scan-sensor result. PosterDemo (PSR-only) would not catch a
   failure that affected only non-PSR sensor classes.

2. **A3 / A4 — `loadRunFile.m:610` and `:613`** (HIGH). Both validation
   calls (`validateScanCoverage` and `validateScenarioConfig`) are wrapped
   in bare `try ... catch; end`. The whole point of these blocks is to
   warn the user about scenario problems; a silent catch defeats the
   purpose and silently masks ANY error inside the validators (including
   typos, class behavior shifts, or missing dependencies). High concern
   given Phase 2's interest in `validateScenarioConfig.m` itself.

3. **B1 — `loadRunFile.m:639-647` (`getFreqForType`)** (MED-HIGH).
   `switch upper(sType)` with `otherwise; freq = 2.8e9;`. IRST, IR, SONAR,
   LIDAR sensor types all silently get S-band 2.8 GHz returned. If
   anything downstream uses `meta.frequency` for physics (e.g., rain
   attenuation in `applyRainDegradation.m`), an IR sensor gets treated as
   S-band radar for rain physics. CHECKPOINT.md already flags "Proper
   IR-aware weather degradation" as post-demo cleanup — this is the
   upstream root cause.

4. **A1 — `loadRunFile.m:129`** (MED). `try meta.maxRange =
   sObj.RangeLimits(2); catch; meta.maxRange = 111120; end`. Per project
   memory ("maxRange: computeMaxRangeFromTruth(). 111120 is fallback
   only."), the 111120 fallback is supposed to be exceptional, but
   `irSensor` lacks a `RangeLimits` property in R2025b — every IR sensor
   silently hits the fallback. Needs doc-fetch verification on irSensor
   property set before reclassification.

5. **B2 — `loadRunFile.m:610-613`** (MED). Cross-cutting `try ... catch;
   end` pattern shared by both validate calls — see triage #2. Pattern
   also appears in the `computeScenarioBounds` helper at lines 624 & 628
   with the same silent-swallow shape.

6. **A2 / A5 / A6 — `rainpl` / `fogpl` / `RangeLimits(2)` fallbacks**
   (LOW). Documented Category-B-style defensive fallbacks per the brief's
   exclusion list (comments explain rationale; failure mode is plausible
   in deployed environments without the Phased Array Toolbox). Upgrade
   to `catch ME; warning(...)` so the first occurrence is visible, but
   the design itself is sound.

## File: src/+trackbench/+config/loadRunFile.m

File length: 839 lines. Audited scope: everything outside §7 (line 425)
and §9 (line 535) per HANDOFF_BUGHUNT.md note. §6 sits between them but
was not on the audited list; included here since the brief specified
"everything else is in scope."

Five try blocks total in audited regions: lines 129, 130, 605, 610, 624,
628 (in helper). (Line 539 is the §9 terrain block, partially audited
already — excluded.)

### Category A — try/catch typo-maskers

**Finding A1** — `meta.maxRange` fallback to hardcoded sentinel
- Line: 129
- Snippet:
  ```matlab
  meta.name = '';
  if isfield(sDef, 'name'); meta.name = char(sDef.name); end
  try meta.maxRange = sObj.RangeLimits(2); catch; meta.maxRange = 111120; end
  try meta.frequency = getFreqForType(sType); catch; meta.frequency = 2.8e9; end
  ```
- Context: in the §2 sensor-loading loop, builds the per-sensor metadata
  struct used by downstream physics (clutter generation, weather
  degradation). Reads `RangeLimits(2)` (upper bound of the range window).
- Catch behavior: silent default to 111120 m (~60 nmi). No log, no warning.
- Risk: MEDIUM. `fusionRadarSensor` and `sonarSensor` both expose
  `RangeLimits`, but `irSensor` (per Phase 1 audit notes and v3.7.0 IRST
  work) does NOT. Every IR sensor silently gets 111120 m maxRange — which
  then drives `rangeFactor = rMax / 111120` (= 1.0) in
  `applyRainDegradation.m:170` and bounds clutter range at
  `rMax * 0.7 ≈ 78 km` in `applyRainDegradation.m:190`. Real IR detection
  ranges are typically 5-30 km. Needs doc-fetch verification on the
  R2025b `irSensor` property set. Per the project's empirical-uniqueness
  rule, this should be reverified against current sensor builds, not
  memory.

**Finding A2** — `meta.frequency` fallback never reachable
- Line: 130
- Snippet:
  ```matlab
  try meta.maxRange = sObj.RangeLimits(2); catch; meta.maxRange = 111120; end
  try meta.frequency = getFreqForType(sType); catch; meta.frequency = 2.8e9; end
  % Allow sensor config to override default frequency
  if isfield(sDef, 'frequency_hz')
  ```
- Context: per-sensor metadata frequency assignment, used by rain/snow/fog
  physics for ITU-R coefficient lookup.
- Catch behavior: silent default 2.8 GHz. `getFreqForType` (line 639-647)
  is a local switch with `otherwise; freq = 2.8e9` — it cannot throw on
  any string input. The catch is dead code.
- Risk: LOW. Dead defensive code; no real masking risk because
  `getFreqForType` won't throw. The real frequency-bug exposure is in
  `getFreqForType` itself (see B1). Consider deleting the try/catch and
  letting `getFreqForType` be the single source of truth.

**Finding A3** — `validateScanCoverage` silently swallowed
- Line: 605-610
- Snippet:
  ```matlab
  %% 10. Validate
  try
      [scanOk, scanInfo] = trackbench.scenario.validateScanCoverage(scenario, config.scenario.duration_s);
      if ~scanOk
          warning('loadRunFile:fewScans', '%s', scanInfo.message);
      end
  catch; end
  ```
- Context: §10 "Validate" — the only point in `loadRunFile.m` that audits
  whether the scenario will produce enough scans for tracking.
- Catch behavior: bare `catch; end`. Any error inside `validateScanCoverage`
  (typo, API drift on `scenario.UpdateRate`, struct-shape change in
  `scanInfo`) is swallowed with no notice. The conditional warning becomes
  unreachable.
- Risk: HIGH. This is the validator block — silent failure here means
  scenarios with broken scan coverage proceed through detection generation
  with no warning. PosterDemo's bit-identical canary would not catch a
  regression here unless the regression also broke detection counts.
  Recommend `catch ME; warning('loadRunFile:scanValidatorFailed', '%s',
  ME.message); end`.

**Finding A4** — `validateScenarioConfig` silently swallowed
- Line: 611-613
- Snippet:
  ```matlab
  try
      trackbench.validation.validateScenarioConfig(config, scenario, sensors, metas);
  catch; end
  ```
- Context: §10 "Validate" continued — calls the project's main
  scenario-config validator. Per HANDOFF_BUGHUNT.md, this file's Checks
  2–10 are themselves on the Phase 2 audit list.
- Catch behavior: bare `catch; end`. Any error in any of Checks 1-10 is
  silently swallowed.
- Risk: HIGH. This is the SAME pattern as A3, on an even more
  consequential validator. If any one of Checks 2-10 has the
  property-typo class of bug Phase 1 found in Check 1, the catch here
  silently masks it. Recommend `catch ME; warning('loadRunFile:configValidatorFailed',
  '%s', ME.message); end` and consider rethrowing for hard validation
  failures (criterion: any error vs. any `warning('validateScenarioConfig:*')`
  raised by the validator itself).

**Finding A5** — `computeScenarioBounds` helper: nested silent catches
- Lines: 624-633 (helper function `computeScenarioBounds`, well outside §7/§9)
- Snippet:
  ```matlab
  function bounds = computeScenarioBounds(scenario)
      allPos = [0 0 0];
      plats = scenario.Platforms;
      for p = 1:numel(plats)
          try
              pos0 = plats{p}.Trajectory.Position(:)';
              allPos = [allPos; pos0]; %#ok<AGROW>
          catch; end
          try
              traj = plats{p}.Trajectory;
              if isprop(traj, 'Waypoints')
                  allPos = [allPos; traj.Waypoints]; %#ok<AGROW>
              end
          catch; end
      end
      maxExtent = max(vecnorm(allPos(:,1:2), 2, 2));
      halfSpan = max(maxExtent * 1.15, 130000);
      bounds = [-halfSpan, halfSpan; -halfSpan, halfSpan];
  end
  ```
- Context: computes the bounding box used to generate terrain extent. Two
  back-to-back tries: first reads `Trajectory.Position`, second reads
  `Trajectory.Waypoints` (guarded by `isprop`).
- Catch behavior: silent. Errors in either branch are dropped.
- Risk: MEDIUM. The second try is defensible (mixed `kinematicTrajectory`
  / `waypointTrajectory` types — only the latter has `Waypoints`). The
  first try, however, swallows real failures: if `Trajectory` is some
  custom type without `Position`, every platform's bounds contribution
  is silently dropped, and the floor of `max(maxExtent*1.15, 130000)` at
  line 636 silently absorbs the loss with a 130 km bounding box. Could
  cause wrong-extent terrain generation for moving-platform scenarios
  with non-standard trajectories. Recommend: gate first try with `isprop(plats{p}.Trajectory, 'Position')`
  rather than catching; the catch should not be bare.

### Category B — comprehensive-looking checks

**Finding B1** — `getFreqForType` `otherwise` silently accepts unknown types
- Lines: 639-648
- Snippet:
  ```matlab
  function freq = getFreqForType(sType)
      switch upper(sType)
          case {'PSR','ASR','WEATHER'};     freq = 2.8e9;
          case {'SSR','MSSR'};              freq = 1.03e9;
          case 'ARSR';                      freq = 1.3e9;
          case {'PAR','FIRE_CONTROL','AESA','TWS'}; freq = 9.0e9;
          case 'MARITIME';                  freq = 9.4e9;
          otherwise;                        freq = 2.8e9;
      end
  end
  ```
- Context: maps sensor type string to RF frequency. The result populates
  `meta.frequency` (line 130) which downstream physics consumes.
- Why it looks comprehensive: covers 11 sensor-type strings explicitly,
  has a fallthrough default.
- What it misses: any sensor type not in the case list — including 'IRST',
  'IR', 'SONAR', 'LIDAR', 'CUSTOM' — silently gets 2.8 GHz returned. For
  IRST in particular, this is a meaningful physics-level mistake:
  - `applyRainDegradation.m:67` reads `sensorInfo.radarFreq` (driven by
    `meta.frequency`) and applies `rainpl()` at that frequency. An IRST
    routed through this path gets ITU-R P.838-3 attenuation evaluated at
    2.8 GHz — but IR doesn't use RF attenuation at all.
  - `applyRainDegradation.m:165-166` early-returns for `isIR`, so in the
    rain path this is currently inert — BUT only if `isIR` propagation is
    correct. The frequency lookup is wrong regardless; the inertness is
    accidental.
  - CHECKPOINT.md "Post-demo cleanup" already lists "Proper IR-aware
    weather degradation". This is the upstream defect.
- Risk: MEDIUM-HIGH. Recommend either: (a) add explicit `'IR'`, `'IRST'`,
  `'SONAR'`, `'LIDAR'` cases returning meaningful values (or `NaN` /
  `[]` to signal "not an RF sensor"); (b) change `otherwise` to
  `error('getFreqForType:unknownType', ...)` so unknown types fail loudly
  during the v3.7.0+ sensor-build sweep.

**Finding B2** — duplicate `volume`/`beta` field-fill block
- Lines: 313-317 (inside §6, included in scope per brief)
- Snippet:
  ```matlab
      if isfield(tg, 'volume');         config.tracker_global.volume = tg.volume; end
      if isfield(tg, 'beta');           config.tracker_global.beta = tg.beta; end
      ...
      % volume/beta are per-tracker (GNN/JPDA use them differently),
      % but we keep defaults here for backward compatibility
      if isfield(tg, 'volume'); config.tracker_global.volume = tg.volume;
      else;                     config.tracker_global.volume = 1e9; end
      if isfield(tg, 'beta');   config.tracker_global.beta = tg.beta;
      else;                     config.tracker_global.beta = 1e-14; end
  ```
- Context: §6 assemble unified config — loading `tracker_globals.json`.
- Why it looks comprehensive: explicit defaults via `else` branch.
- What it misses: lines 304-305 already set `volume`/`beta` if present.
  Lines 314-317 repeat the same `isfield` then-branch and add the
  defaults. The defaults only matter when the field is MISSING — which
  means a typo in `tracker_globals.json` (`volumne`, `Beta`) silently
  falls through to 1e9 / 1e-14 rather than raising. Same pattern as a
  silent typo-mask, but for config field names.
- Risk: LOW (redundant code, mild typo-mask), but cleaning it up would
  reduce confusion. Could be replaced with a single block that explicitly
  errors on misspelled fields (`structfun(@(...) ...)` or whitelist
  check). Note: the duplicate ALSO means the first block's writes (lines
  304-305) are immediately overwritten by the second — the first block
  is essentially dead code.

## File: src/+trackbench/+detections/getWeather.m

File length: 118 lines. Author: Daniel Trofimchik. No try/catch blocks
in this file.

### Category A — try/catch typo-maskers

None. File contains zero try/catch blocks.

### Category B — comprehensive-looking checks

**Finding B3** — `getOrDefault` helper silently substitutes on missing field
- Lines: 31-33 (call sites) and 112-118 (helper)
- Snippet (helper):
  ```matlab
  function val = getOrDefault(s, field, default)
      if isstruct(s) && isfield(s, field)
          val = s.(field);
      else
          val = default;
      end
  end
  ```
  And call sites:
  ```matlab
  storm_start = getOrDefault(d, 'storm_start_s', 5);
  storm_end   = getOrDefault(d, 'storm_end_s',   45);
  active_type = getOrDefault(d, 'active_type',   'step');
  ```
- Context: parses storm-window config from `cfg.degradation`.
- Why it looks comprehensive: explicit defaults, isstruct/isfield guard.
- What it misses: any typo in the run-file degradation fieldname (e.g.,
  `storm_start` instead of `storm_start_s`, `start_storm_s`) silently
  yields the default (5 s start). The user sees the storm starting at
  5 s and may not realize their override isn't being applied. Same class
  of bug as the original v3.6.5 `InitialPosition` typo — silent
  substitution masking a mismatch between what was written and what's
  read.
- Risk: MEDIUM. Recommend: log when the default fires (`fprintf('[WEATHER]
  Using default %s = %g (field missing)', field, default);`) at least
  on first call per scenario. Or whitelist known fields and warn on
  unknown ones in the parent `cfg.degradation` struct.

(The `switch lower(active_type)` `otherwise` branch at line 88-92 DOES
emit a `warning` before defaulting to step — clean and not a finding.
The window-validation block at lines 38-58 is similarly clean.)

## File: src/+trackbench/+environment/applyRainDegradation.m

File length: 281 lines.

### Category A — try/catch typo-maskers

**Finding A6** — `rainpl()` fallback silently triggers on any error
- Lines: 223-234 (inside `computeRadarPd` helper)
- Snippet:
  ```matlab
  try
      % rainpl(range_m, freq_Hz, rainRate_mmhr) -> one-way loss in dB
      % Ref: https://www.mathworks.com/help/phased/ref/rainpl.html
      L_oneway_dB = rainpl(slantRange, freq, rainRate);
      L_twoway_dB = 2 * L_oneway_dB;
  catch
      % Fallback: hand-coded ITU-R P.838-3 coefficients
      % (in case Phased Array Toolbox is not installed)
      [k_coeff, alpha_coeff] = getITU838Fallback(freq);
      gamma_dBperkm = k_coeff * rainRate^alpha_coeff;
      L_twoway_dB = 2 * gamma_dBperkm * slantRange / 1000;
  end
  ```
- Context: per-detection rain attenuation. Borderline Category C
  per the brief (documented fallback, sound rationale: missing Phased
  Array Toolbox).
- Catch behavior: silent fallback to local table. Comments explain
  rationale.
- Risk: LOW. Documented exclusion criterion per brief, BUT (a) the catch
  is undifferentiated — any error in `rainpl()` (negative range, bad
  freq, malformed inputs) ALSO triggers the fallback, not just "toolbox
  missing"; (b) the fallback table itself is flagged in its own
  docstring as having 1-7 GHz coefficients ~50-300% off from strict
  ITU horizontal-pol values. So a silently-triggered fallback at S-band
  would produce wrong attenuation by a factor of ~2-4. Recommend
  upgrade to `catch ME; persistent warned; if isempty(warned);
  warning(...); warned = true; end` or gate by `exist('rainpl', 'file')`
  once at function-init.

### Category B — comprehensive-looking checks

**Finding B4** — `getField` helper at line 280-282, same pattern as B3
- Lines: 60-62 (call sites) and 280-282 (helper)
- Snippet:
  ```matlab
  function val = getField(s, field, default)
      if isstruct(s) && isfield(s, field); val = s.(field); else; val = default; end
  end
  ```
- Context: parses rain config fields (rainRate, pdFloor, clutterMult).
- Same risk profile as B3 in `getWeather.m`. Silent default on missing
  field — typo in run-file weather config yields default values without
  notice.
- Risk: MEDIUM (compounded across the codebase — this helper pattern
  appears in at least three files: `getWeather.m:112`,
  `applyRainDegradation.m:280`, `applyWeatherDegradation.m:261`).

**Finding B5** — inconsistent `rangeLimits` handling
- Lines: 79-81 vs. 189
- Snippet:
  ```matlab
  rMax = 111120;
  if isfield(sensorParams, 'rangeLimits')
      rMax = sensorParams.rangeLimits(2);
  end
  ...
  for ii = 1:nClutter
      az = rand() * 360;
      rMin_c = max(sensorParams.rangeLimits(1), 2000);
      ...
  ```
- Context: line 79-81 sets `rMax` with an `isfield` guard (silent
  default to 111120); line 189 reads `sensorParams.rangeLimits(1)`
  with no guard.
- Why it looks comprehensive: the up-front guard implies `rangeLimits`
  is optional.
- What it misses: line 189 will hard-error if `rangeLimits` is missing,
  while line 79-81 silently defaults. Inconsistent — either the field
  is required (in which case 79-81 should also error) or optional (in
  which case 189 should also guard). Currently a sensor without
  `rangeLimits` would pass through 79-81, then crash on 189, but only
  in the radar/non-IR/non-MSSR clutter-generation path (since 165
  early-returns for IR/MSSR). So the inconsistency is masked by an
  early return.
- Risk: LOW (currently masked by line 165 early-return; would surface
  for a future code change that removed the IR/MSSR shortcut).

## File: src/+trackbench/+environment/applyWeatherDegradation.m

File length: 263 lines. Author: James Gallegos.

### Category A — try/catch typo-maskers

**Finding A7** — `fogpl()` fallback silently triggers (same pattern as A6)
- Lines: 244-256 (inside `computeFogPdRadar` helper)
- Snippet:
  ```matlab
  function pd = computeFogPdRadar(slantRange, freq, lwc, pdFloor)
      if slantRange < 500; pd = 1.0; return; end
      try
          % fogpl(range, freq, temperature, liquidWaterDensity)
          L_oneway = fogpl(slantRange, freq, 15, lwc);  % 15°C typical
          L_twoway = 2 * L_oneway;
          pd = max(pdFloor, 10^(-L_twoway / 12));
      catch
          % Fallback if fogpl not available
          % Approximate: fog attenuation ~0.01-0.1 dB/km at 10-30 GHz
          freqGHz = freq / 1e9;
          gamma = 0.005 * lwc * freqGHz^1.5;  % dB/km approximate
          L_twoway = 2 * gamma * slantRange / 1000;
          pd = max(pdFloor, 10^(-L_twoway / 12));
      end
  end
  ```
- Context: fog attenuation for radar above 10 GHz.
- Risk: LOW. Same risk profile as A6. Documented fallback. Recommend
  the same `catch ME; persistent warned; ...` upgrade so silent regressions
  in `fogpl` surface.

### Category B — comprehensive-looking checks

**Finding B6** — `applySnowDegradation` uses non-seeded random sampling
- Lines: 88-91
- Snippet:
  ```matlab
  if ~isempty(clutter)
      keepMask = rand(numel(clutter), 1) < 0.4;
      clutter = clutter(keepMask);
  end
  ```
- Context: in `applySnowDegradation`, drops ~60% of the rain-equivalent
  clutter returns to model snow's lower reflectivity.
- Why it looks comprehensive: the comment above (line 86-87) explains
  the reasoning ("Snow reflectivity lower than rain").
- What it misses: `rand()` here is NOT seeded — calls to
  `applySnowDegradation` will produce different clutter sets each run,
  violating the project's determinism baseline (per CHECKPOINT.md
  "PosterDemo canary baseline must stay bit-identical"). PosterDemo
  uses rain, not snow, so this is currently dormant — but any
  snow-using scenario would have non-reproducible detection counts
  scan-to-scan, which would break the v3.7.0 bit-identical canary
  pattern.
- Risk: LOW for current demo, MEDIUM post-demo if snow scenarios are
  added to validation. Recommend save/restore RNG state around this
  block, mirroring `generateTerrain.m`'s `rng(42, 'twister')` pattern
  noted in HANDOFF_BUGHUNT.md "Determinism / RNG".

(The `switch lower(weatherType)` `otherwise` at line 53-58 DOES emit a
warning before defaulting to rain — clean, not a finding. `nargin < 6`
guard at line 31-33 is standard.)

## File: src/+trackbench/+environment/isAboveHorizon.m

File length: 75 lines.

### Category A — try/catch typo-maskers

None. File contains zero try/catch blocks.

### Category B — comprehensive-looking checks

None of consequence. One minor cosmetic note (not a finding): the
single-target-reshape branch at lines 48-50 is logically a no-op for
all input shapes — `if size(targetPos, 1) == 1` matches an already-1x3
row, in which case `targetPos(:)'` returns the same 1x3; a 3x1 column
input fails the size check and is interpreted as 3 single-coordinate
targets, which then crashes at line 58 on `targetPos(k, 3)` rather than
producing silent wrong output. So this branch neither helps the
intended case nor silently masks the misuse case — it just doesn't do
anything useful. Cosmetic, not a Category A/B finding.

The horizon formula at lines 55-72 implements the standard 4/3-Earth
LOS sum (sensor + target horizon distances) and compares against ground
range. The MathWorks `horizonrange` function returns the slant distance
to the horizon, which is conventionally treated as approximately equal
to ground range for small altitudes relative to Earth radius. For
typical ATC altitudes (≤15 km), the slant/ground delta is <0.05% — not
a concern. Needs doc-fetch verification of `horizonrange` return
convention if the project starts modeling space-domain sensors.

## File: src/+trackbench/+detections/runDetections.m (line ~386 only)

Per the brief, only the immediate context around the flagged
line is in scope. The rest of `runDetections.m` is BUGHUNT-owned.

### Category A — try/catch typo-maskers

**Finding A8** — Sensor-step call silently dropped on any error
- Line: 386 (in current file state — verified empirically below)
- Snippet (lines 383-392):
  ```matlab
          if any(~visible); targets = targets(visible); end
      end

      % Step the sensor
      try [dets, ~, sensorCfg] = si.sensor(targets, ins, simTime); catch; continue; end
      dets = dets(:);

      if hasMechanical && k == masterIdx
          try if sensorCfg.IsScanDone; scanDone = true; end; catch; end
      end
      if isempty(dets); continue; end
  ```
- Context: per-scan, per-sensor inner loop. After the visibility filter
  (terrain occlusion + horizon masking) yields the filtered `targets`,
  this is the call into the System-object sensor (`fusionRadarSensor`,
  `irSensor`, `sonarSensor`, etc.) that produces detections for the
  current scan. The 3-output form `[dets, ~, sensorCfg]` is the
  "detections + config" signature used to also pull `IsScanDone` (read
  three lines down at line 390).
- Catch behavior: bare `catch; continue;`. Skips the rest of the loop
  iteration for this sensor at this time step. NO log, NO warning.
- Risk: HIGH. This is the most consequential silent catch in the
  audited set. Failure modes it masks:
  1. **Sensor-class API drift.** If a future R2025c renames a property
     or argument shape, every detection silently drops; PosterDemo
     would show zero detections and "fail" tracking but with no error
     pointing at the cause.
  2. **Per-sensor targets-format bugs.** If the visibility filter
     produces malformed targets for some sensor type (e.g., empty
     struct vs. empty struct array), the sensor errors and silently
     drops. Already partially mitigated downstream by the `minMeas`
     branch at line 401, but THAT only triggers AFTER detections come
     back — this catch fires before detections exist.
  3. **Non-PSR class behavior changes.** PosterDemo (PSR-only) cannot
     catch this. An IR/IRST scenario could silently produce zero
     detections per scan and the user would see "tracking failed" with
     no diagnostic.
  4. **Property-typo masking** of the v3.6.5/6.7 class. If a v3.7.x
     change introduced a typo'd property read on `si.sensor` (e.g., via
     the visibility-filter code at line 360 / 412 that already calls
     `si.sensor.MountingLocation`), the typo's error path could route
     through line 386's catch on the next iteration and be invisible.
- Already flagged in CHECKPOINT.md "Post-demo cleanup" line 65:
  `runDetections.m line 386 try/catch ... continue ... end — silent
  typo-masker, Phase 2 BUGHUNT audit scope (flagged explicitly so it
  doesn't get skipped).` This audit confirms the classification.
- Recommended fix: `catch ME; warning('runDetections:sensorStepFailed',
  'Sensor %d step failed at t=%.3f: %s', si.sensorIndex, simTime,
  ME.message); continue; end`. Once. Per-sensor-per-run rate-limited
  if necessary via persistent flag.
- Doc-fetch needed before fix DESIGN: `fusionRadarSensor`, `irSensor`,
  and `sonarSensor` step signatures and documented error modes. The
  catch may currently mask a documented "warm-up returns empty"
  behavior for some sensor classes, in which case the fix shape
  matters.

### Surrounding context (informational, not findings)

The same file region contains two other silent catches at lines 360 and
390:
- Line 360: `try sensorPos = sensorPos + si.sensor.MountingLocation(:)'; catch; end`
  — adjusts sensor position by mounting offset. Silent failure leaves
  sensorPos at the platform origin (similar root cause shape as the
  v3.6.5 `InitialPosition` bug — exactly the kind of pattern Phase 1
  flagged).
- Line 390: `try if sensorCfg.IsScanDone; scanDone = true; end; catch; end`
  — reads `IsScanDone` off `sensorCfg`. Silent failure means master-clock
  advancement could stall for the scan that errored.

These are not in the brief's explicit scope (the rest of `runDetections.m`
is BUGHUNT-owned), but they share the same pattern shape as A8 and the
v3.6.5/v3.6.7 fixed bugs. Flagging here for BUGHUNT chat triage
awareness; not classified as findings in this report.

## Cross-file observations

1. **`getField` / `getOrDefault` pattern is project-wide.** The "silent
   default on missing field" helper appears as `getOrDefault` in
   `getWeather.m:112` and as `getField` in `applyRainDegradation.m:280`
   and `applyWeatherDegradation.m:261`. Same shape, same risk: typo in
   a config field name → silent default value → user thinks their
   override is applied. Triage: consider a project-level
   `+trackbench/+util/configGet.m` helper that (a) logs first-time
   default fires, and/or (b) accepts a whitelist of known field names
   and warns on unknowns. Out of scope for demo week; post-demo
   cleanup candidate.

2. **`rainpl` / `fogpl` toolbox-fallback pattern is consistent.** Both
   `applyRainDegradation.m:223` and `applyWeatherDegradation.m:244` use
   the same `try <toolbox-call>; catch <local-fallback>` shape. Both
   are documented. Both should grow a `catch ME; persistent warned;
   ...` upgrade together so a silent regression in either toolbox call
   surfaces consistently.

3. **Silent validators in §10 of `loadRunFile.m` (A3, A4) are the
   highest-leverage fix in this audit.** Both validator wrappers are
   `try ... catch; end` — the whole point of validators is to surface
   problems, so silencing them inverts the design intent. Recommend
   these be the first two fixes BUGHUNT triages.

4. **The "frequency lookup silently returns S-band for unknown
   sensor types" defect (B1) is the most consequential
   cross-cutting Category B finding.** It connects directly to the
   v3.7.0 IRST work and CHECKPOINT.md's "Proper IR-aware weather
   degradation" post-demo cleanup item. Fixing `getFreqForType` to
   `error` on unknown types would cause IR sensors to fail loudly
   during sensor metadata construction rather than silently
   misrepresenting their physics downstream.

5. **CHECKPOINT.md `safeSet` `isprop`-only weakness** (post-demo
   cleanup line 67) is a related Category B pattern not in this audit's
   file list but worth noting: `isprop(obj, name)` returns true for
   read-only properties; `findprop(obj, name).SetAccess` would catch
   them. Same pattern shape as Category B "isprop without further
   checks." Confirms the brief's pattern taxonomy is sound.

6. **Anchor-uniqueness rule applied to line citations.** Per CHECKPOINT.md
   "Anchor uniqueness is EMPIRICAL," all line numbers in this report
   were verified by reading the actual current file state, not memory.
   Spot-verified `runDetections.m` line 386 is indeed `try [dets, ~,
   sensorCfg] = si.sensor(targets, ins, simTime); catch; continue; end`
   in the current branch state (`Michael---Working_on_pathEditor`).
