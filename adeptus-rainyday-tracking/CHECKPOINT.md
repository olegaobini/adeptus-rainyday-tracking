# Build Order Checkpoint — v3.5 GUI Rebuild

**Last checkpoint:** 2026-04-25, immediately after step 4 complete + EXE rebuild + git cleanup.

## What's done

- **3-button main menu** (`scripts/mainMenu.m`) — Path Editor / Run Simulation / Validation & Documentation
- **Run Simulation** (`scripts/runSimGUI.m`) — smart cache, dirty-tracking, weather sub-grid
- **Validation & Documentation** (`scripts/validationDocsGUI.m`) — 3-tab: 27-row test plan inline grid, diagnostic suite, docs links
- **Path Editor step 4 (FULLY COMPLETE):**
  - **4a** — mode-specific show/hide: Targets / Sensors / Environment buttons collapse irrelevant panels to 0 px height (`buildUI.applyEditMode` rewrites parent grid `RowHeight`)
  - **4b** — NASA flight as a target type: "Import NASA Flight (.mat)…" button in Targets panel → `loadNASAFlight` → `EditorState.addNasaFlightTarget`
  - **4c** — context-aware Save (Option A: in-panel Save buttons):
    - Targets panel: 6-row layout (dropdown / collection mgmt / Load+Save / NASA / Load Ref+Unload / Clear)
    - Sensors panel: 3-row layout (dropdown / collection mgmt / Load+Save)
    - Terrain panel: row 10 added for Save
    - Weather panel: Save next to Load on row 8
    - File panel shrunk to scenario-only (Open Scenario / Export Scenario)
    - 3 new exporter files: `exportTerrainToJSON.m`, `exportWeatherToJSON.m`, `exportSingleSensorToJSON.m`
    - Bugfix in `refreshTerrainPanel`: explicitly enable Load/Save/Overlay/4 degradation checkboxes (the prior "intentionally stay on" comment was lying — `applyEditMode` was disabling them on mode-leave and `refreshTerrainPanel` wasn't re-enabling)
- **EXE rebuilt** with `mainMenu` as the entry point. Now lives at project root + `trackbench/` (build output dir); `trackbench.bat` checks both locations.
- **Git-ready folder layout** — repo root contains `adeptus-rainyday-tracking/` + `Tail_687_1/` (NASA flight data sibling). `runNASAFlight.m` resolves Tail_687_1 relative to project parent, so this layout works as a clone.

## What's NEXT (resume here)

### Step 5 — Multi-region terrain & weather

User chose option (c) at the time of planning: **multi-region**, not single polygon.

- **5a** — data model + JSON shape: extend `TerrainRecord` / `WeatherRecord` (or introduce sibling collection records) so a single scenario can carry multiple terrain regions and multiple weather regions, each with its own polygon footprint. Update `loadRunFile` to parse the new shape. Backward-compat: a scalar legacy file still loads as a single full-map region.
- **5b** — sim engine: `runDetections` queries terrain/weather per (x, y) location instead of using a single global record. Touch points: `applyWeatherDegradation`, `generateGroundClutter`, terrain occlusion path.
- **5c** — polygon drawing for terrain regions in path editor.
- **5d** — polygon drawing for weather regions.

### Step 6 — Save/load figures per run / session / tracker

Last item on the build order list. Persistence layer for the visual outputs (not just data).

## Other deferred / pending

- **Diagnostic suite stale references** — `verifySimulation.m` still has refs to removed VCP code (45 PASS / 11 FAIL / 2 WARN); failures are not GUI bugs, just stale checks. Cleanup deferred.
- **TC-05** in test plan still at 1.10× ratio (threshold 1.15×) — geometry/threshold tuning issue, not a code bug.
- **Original 4-button main menu / `runScenarioGUI.m`** is orphaned but kept on disk in case it's useful as reference. Same for `launch_trackbench.m`.

## Key gotchas to keep in mind

- After ANY edit to a file in `+trackbench/` or `+trackbench/+editor/`: run `clear classes; clear all` before `mainMenu` — MATLAB caches package members aggressively.
- When adding a new `state.<field> = uibutton(...)` in `buildUI`, the matching `<field> = gobjects(1)` MUST be added to `EditorState.m` properties block. `checkcode` does NOT catch missing-property errors at lint time (runtime check only). The strict classdef throws mid-build, leaving downstream panels un-built — that's the "empty panels" failure mode.
- Value-class assignment chains like `state.terrain.field = X` work in modern MATLAB but the codebase prefers explicit read-mutate-writeback (`tr = state.terrain; tr.field = X; state.terrain = tr;`). Match the existing style for maintainability.
- `runNASAFlight.m` resolves `Tail_687_1/` as a SIBLING of the project root (`fileparts(projectRoot)/Tail_687_1`). The git layout (repo root containing both `adeptus-rainyday-tracking/` and `Tail_687_1/`) preserves this. Don't move them apart.

## Boeing meeting

Next Friday from the date of this checkpoint. v3.5 should be ready to demo.
