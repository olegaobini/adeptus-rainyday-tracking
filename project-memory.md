Purpose & context
Michael is a UW ECE senior (graduating 2026) and sole developer on Team Adeptus, working on a Boeing-sponsored capstone project called "Rainy Day" / TrackBench — a configurable MATLAB radar tracking simulation sandbox for evaluating airborne radar tracking performance under variable environmental degradation conditions. The project evaluates tracker algorithms (GNN, JPDA, TOMHT) with filter models (CV, IMM) across five environment degradation layers: terrain occlusion, horizon masking, ground clutter, Doppler/MTI fade, and weather attenuation.
Team: Michael (primary developer, ~21 of 23 git commits), Daniel Trofimchik (getWeather.m, plotInitialScenario.m), James Gallegos (applyWeatherDegradation.m), Kaz Foster, Olega Obini. Boeing demo day target: 2026-05-29.
Active project root (as of 4/24):
C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking
(Created from "After Presentation" merged with exeAttempt contents. Prior locations — exeAttempt, After Presentation, Adding Flight Data — are reference-only.)
Git workflow: Force-push via git push origin Michael:main --force-with-lease from the parent Git directory. Active branch: Michael---Working_on_pathEditor.
MATLAB environment: R2025b, Windows, with Sensor Fusion and Tracking, Radar, and Phased Array System Toolboxes.

Current state
Deployed executable (latest session, ~May 2026): Five bugs were diagnosed and resolved in the deployed .exe:

Path resolution — compareTrackers.m, autoTuneTracker.m, compareAllTrackers.m used mfilename('fullpath') pointing to read-only CTF cache; fixed with resolveRootFromThisFile() helper + if ~isdeployed guard on addpath
degEqual crash — polymorphic weather field (string vs. {fallback, regions[]} struct) in runSimGUI.m; fixed with weatherEqual() helper + weatherFallbackOf()/weatherRegionsOf() accessors
Degenerate polygon warning — loadRunFile.m parseRegion returning bad records; fixed with rec.degenerate flag + continue guards in parseTerrainField/parseWeatherField loops
AutoTune silently defaulting to IMM — autoTuneAction in runSimGUI.m ignored tracker JSON; fixed by reading tracker_type/filter_model from JSON, adding multi-select guard, surfacing three-way uiconfirm dialog
Hardcoded maxRange = 111120 — over-normalized short-range scenarios; fixed with computeMaxRangeFromTruth() using max(vecnorm(allTruthPos, 2, 2)), 111120 as fallback

All five confirmed working in rebuilt executable.
Codebase cleanup (in progress): Full 42-file audit of scripts/ completed. Classified orphan list:

Delete (4 files): launch_trackbench.m, runScenarioGUI.m, compareAllTrackers.m, cleanup_showcase.m
Archive to scripts/legacy/ (19 files): Path editor milestone tests, per-milestone verification scripts, Boeing one-shot prep helpers, runNASAFlightGlobe.m
Keep as CLI tools (5 files): runNASAFlight.m, scanNASAFlights.m, three diag scripts
Pending: build_executable.m patch to exclude scripts/legacy/ from CTF bundle; awaiting user confirmation to execute moves

GUI — Path Editor and Simulation (previously active):

runSimGUI.m tracker editor fully implemented: GNN (6 fields), JPDA (7 fields), TOMHT (9 fields), Save-As with name validation, overwrite confirmation, JSON field preservation
Path editor 2D/3D view bugs resolved (view-state flags, R-key reset, arrow-key 3D rotation)
Live terrain heightmap rendering (Step 5e) implemented in drawMap.m via drawTerrainHeightmap2D/3D, hypsometric tint for 2D, surf() with FaceAlpha for 3D, persistent-variable cache

Version: v3.4.0 (README update pending).

On the horizon
Pending work (per 4/15 plan):

README update to v3.4.0+
Codebase cleanup: remove dead code/orphan scripts, update all JSONs (remove propagation_model, document rcs_range_filter OFF default)
Test and debug all five degradation features (weather types, Doppler, clutter, terrain, horizon)
Physics verification against MATLAB docs and published references (ITU-R P.838-3, Skolnik, Gunn & East)
Boeing presentation with reference-backed feature explanations (prior rough drafts in Cowork)

Tracker editor remaining substeps: Globals editor (2.4) and validation polish (2.5)
Post-demo feature — Flight Data Manager:

New mainMenu item, separate from Path Editor
Browse NASA files, view on globe (true positions), build multi-flight batch, import to Path Editor
Time-zero + per-flight start_offset_s; translate to editor reference origin
Reference code: scanNASAFlights, viewNASAFlightGlobe, loadNASAFlight, runNASAFlightGlobe.m (in scripts/legacy/)


Key learnings & principles
MATLAB R2025b specifics (critical):

HasRCSSignature property does not exist — fusionRadarSensor reads platform.Signatures natively; external RCS filter (applyRCSFilter) defaults OFF to avoid double-counting
coverageConfig(sensor) returns ScanLimits [az; el] with actual beam scan range
TargetReportFormat = 'Clustered detections'; TargetIndex in detections = PlatformID (tower=1, targets start at 2); false alarm TargetIndex = -1
Beam elevation: FOV-sized, centered within MechanicalElevationLimits
After any +trackbench/ package edit: clear classes; clear all mandatory before testing (MATLAB aggressively caches package members)

Sensor elevation fix (4/15): Old formula [-(fov+2), 2]-tilt pushed PSR beam below horizon. Correct: [-fov/2, fov/2]-tilt → PSR now covers [-17°, +13°]
Key architecture decisions:

propagation_model (VCP) fully removed from detection pipeline — fusionRadarSensor internal model + VCP post-filter double-dip on detection probability; VCP retained as visualization only (plotVCP.m, plotVCPOverlay3D.m)
Run file degradation block: terrain_occlusion, horizon_masking, ground_clutter, doppler_fade, rcs_range_filter toggles + "weather": "rain/default_rain" ref to config/weather/; legacy "enabled": true still supported
runDetections takes 5 args (5th = cfg for storm window)
Deployed-mode path resolution must use resolveRootFromThisFile() pattern, not mfilename('fullpath'), to avoid CTF cache writes

Physics:

Doppler fade MDV uses λ × PRF/4 (not hardcoded 40)
Ground clutter scales with freq² (surface) and freq^0.8 (discrete) relative to S-band baseline
Weather: rain (rainpl, ITU-R P.838-3), snow (rainpl at 25% rate, Gunn & East 1954), fog (visibility-based, fogpl ≥10 GHz), icing (flat antenna gain loss)
autoTuneTracker 2-pass sweep: tracker params first, then filter params; saves to config/trackers/<TYPE>/autotuned_<TYPE>_<runName>.json

Document/presentation tooling: PPTX edit workflow: unpack → edit XML via inline Python content.replace() (not str_replace tool after clean.py/pack.py transform) → clean.py → pack.py --original → soffice --headless --convert-to pdf → pdftoppm -jpeg for visual QA. All spatial coordinates in EMUs (914400 = 1 inch).

Approach & patterns

Dry-run-then-commit workflow: dryRun: true preview before every Filesystem:edit_file commit
One substep at a time with explicit verification gates; confirms with brief status updates before proceeding
Cowork (Opus) handles longer implementation tasks; Claude handles verification, debugging, architectural decisions, and handoff documentation. Cowork sessions kept short and focused
Fresh chat per major milestone to maintain context quality
CHECKPOINT.md at project root maintained as living document tracking completed steps, open questions, and verification checklists
File edits anchored on unique single-line statements (never box-drawing comment headers or $' sequences); large files split into 5–6 smaller edit_file calls (~80–120 lines each)
Git: force-push from parent Git directory; git diff -w to ignore CRLF→LF side effects from Filesystem:edit_file


Tools & resources

Filesystem MCP tool — primary file access (all Windows paths); head/tail parameters for files >400 lines; copy_file_user_to_claude + bash grep/sed for verification on large files
MATLAB MCP — times out after ~4 minutes; not suitable for long-running operations (autoTuneTracker, full runSingleScenario with visualization)
Cowork (agentic coding, Opus) — implementation of larger feature blocks
Key MATLAB toolboxes: Sensor Fusion and Tracking, Radar, Phased Array System
References: ITU-R P.838-3 (rain attenuation), Gunn & East 1954 (snow), Skolnik (radar handbook), FAA ASR-11 specs, MATLAB R2025b documentation
GitHub: https://github.com/team-adeptus/trackbench, username Hardimic
Four files known to have CRLF→LF converted by edit_file: drawMap.m, exportSensorsToJSON.m, openScenarioFromJSON.m, runSimGUI.m — use git diff -w for review