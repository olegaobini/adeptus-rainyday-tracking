# Build Order Checkpoint — v3.5 GUI Rebuild

**Last checkpoint:** 2026-04-25, after Step 5a (data model) + 5b (sim engine) complete.
**Active project root:** `C:\Users\Admin\Documents\RAINY DAY GIT COPY\GitUpdate\Test\adeptus-rainyday-tracking`
**Boeing meeting:** Friday 2026-05-01.

---

## What's done

### Step 4 — Path Editor refinements (FULLY COMPLETE)
- **3-button main menu** (`scripts/mainMenu.m`) — Path Editor / Run Simulation / Validation & Documentation
- **Run Simulation** (`scripts/runSimGUI.m`) — smart cache, dirty-tracking, weather sub-grid
- **Validation & Documentation** (`scripts/validationDocsGUI.m`) — 3-tab: 27-row test plan inline grid, diagnostic suite, docs links
- **4a** — mode-specific show/hide: Targets / Sensors / Environment buttons collapse irrelevant panels (`buildUI.applyEditMode` rewrites parent grid `RowHeight`)
- **4b** — NASA flight as a target type: "Import NASA Flight (.mat)…" button → `loadNASAFlight` → `EditorState.addNasaFlightTarget`
- **4c** — context-aware Save (Option A: in-panel Save buttons):
  - Targets / Sensors / Terrain / Weather panels each gained their own Save button
  - File panel shrunk to scenario-only (Open Scenario / Export Scenario)
  - 3 new exporter files: `exportTerrainToJSON.m`, `exportWeatherToJSON.m`, `exportSingleSensorToJSON.m`

### Installer pipeline (FULLY COMPLETE — verified working)
- `scripts/build_executable.m` — bakes `.m` files into `mainMenu.exe` (~5 min)
- `scripts/build_installer.m` — wraps EXE → web + offline installers (~12 min)
- **Web installer:** `installer/web/RainyDayTrackerInstaller_web.exe` (~3.4 MB)
- **Offline installer:** `installer/offline/RainyDayTrackerInstaller_mcr.exe` (~1.6 GB) — bundles MCR
- Installs to `C:\Program Files\Rainy_Day_Tracker\application\` with Start Menu shortcut + Apps & Features registration
- `config/`, `docs/`, `README.md`, `CHECKPOINT.md` ship alongside the EXE via `AdditionalFiles` (editable post-install)
- `mainMenu.m` deployed-mode resolution tries `pwd` → `ctfroot` → `fullfile(ctfroot, 'adeptus-rainyday-tracking')` to find `config/`
- MCR cached at `C:\Users\Admin\AppData\Local\MathWorks\MatlabRuntimeCache\MCRInstaller25.2\` (one-time `compiler.runtime.download()`)

### Per-file author headers (FULLY COMPLETE — ~70 files)
Standard 4-line header inserted after H1, attributing each file to its author. Format:
```matlab
%   Author:  <name> (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
```

Coverage by package:
- `scripts/` — 24 files, all Michael
- `src/+trackbench/+analysis/` — 1 file, Michael
- `src/+trackbench/+config/` — 1 file, Michael
- `src/+trackbench/+detections/` — 3 files (Michael ×2, Daniel ×1: `getWeather.m`)
- `src/+trackbench/+editor/` — 25 files, all Michael
- `src/+trackbench/+environment/` — 8 files (Michael ×7, James ×1: `applyWeatherDegradation.m`)
- `src/+trackbench/+flightdata/` — 1 file, Michael
- `src/+trackbench/+reporting/` — 7 files (Michael ×6, Daniel ×1: `plotInitialScenario.m` ⚠️ flagged uncertain — verify with Daniel)
- `src/+trackbench/+scenario/` — 2 files, Michael
- `src/+trackbench/+sensors/` — 5 files, Michael
- `src/+trackbench/+tracking/` — 4 files, Michael
- `src/+trackbench/+validation/` — 1 file, Michael

Skipped on purpose: `launch_trackbench.m`, `runScenarioGUI.m` (orphaned legacy), `.bat`/`.md` files.

### Git-ready folder layout
Repo root contains `adeptus-rainyday-tracking/` + `Tail_687_1/` (NASA flight data sibling). `runNASAFlight.m` resolves `Tail_687_1/` relative to project parent, so this layout works as a clone.

### Step 5a — Multi-region data model + JSON shape (FULLY COMPLETE)

**Decisions (signed off):**
- **Region overlap → first-listed wins.** Order in JSON `regions[]` array is the resolution order. No priority field, no blending, no smallest-wins.
- **No-region fallback → explicit `fallback` record.** Backward compatible: a legacy single-string scalar (`"terrain": "mountain/default_mountain"`) is treated as `{fallback: "mountain/default_mountain", regions: []}`. New JSON shape uses an explicit `fallback` field. Terrain fallback is required (every scenario must resolve to some heightmap); weather fallback may be `"none"`.
- **Reference-only regions** for 5a — each region points at a saved component file via `config:` path. No inline override yet (that can be added later as a `terrain_override` field without breaking compat).
- **Polygon coords = scenario NED meters**, Nx2 array literal `[[x,y],[x,y],...]`. Implicitly closed.
- **Two new region-record value classes** (one per concern), not a single generic — each carries a typed inner record (`TerrainRecord` / `WeatherRecord`).

**New files:**
- `src/+trackbench/+editor/TerrainRegionRecord.m` — value class. Fields: `name`, `configPath`, `polygonXY` (Nx2), `terrain` (resolved cache), `readOnly`. Methods: `isValidPolygon`, `xy`.
- `src/+trackbench/+editor/WeatherRegionRecord.m` — same shape with `weather` (resolved cache).
- `src/+trackbench/+environment/resolveTerrainAt.m` — editor-side resolver (typed, takes `TerrainRegionRecord` array).
- `src/+trackbench/+environment/resolveWeatherAt.m` — same.

**Edited files:**
- `EditorState.m` — added `terrainRegions`, `weatherRegions` properties (parallel to `terrain`, `weather` fallback records). Snapshot/restore plumbing handles them and degrades cleanly with pre-5a snapshots.
- `loadRunFile.m` — polymorphic terrain (`runDef.terrain`) and weather (`runDef.degradation.weather`) field parsing. Helper functions: `parseTerrainField`, `parseWeatherField`, `parseRegion`, `loadTerrainFile`, `loadWeatherFile`. Outputs:
  - `config.environment` — fallback terrain def (unchanged contract)
  - `config.environmentRegions` — cell-array-of-structs `{config_path, name, polygon_xy, def}` (empty for legacy)
  - `config.degradation.*` — fallback weather (unchanged contract)
  - `config.degradationRegions` — same shape

**Sim engine (5b) consumes the cell-array shape**; the typed editor records (`TerrainRegionRecord`/`WeatherRegionRecord`) are for the editor side once 5c/5d wire up polygon drawing.

### Step 5b — Sim engine multi-region (FULLY COMPLETE)

**Heightmap strategy (decided):** option (i) — composite heightmap stamped at scenario load, MATLAB `groundSurface`/`SurfaceManager` sees one grid, occlusion runs through the existing radar pipeline. Polygon edges are hard steps (no smooth blending — fine for radar physics, visually consistent with the existing terrain hilltop step).

**New files:**
- `src/+trackbench/+environment/composeHeightmap.m` — one-time multi-region heightmap composer. First-wins via a `claimed` mask; each region's `generateTerrain` is called with the same `scenBounds` so the grids match cell-wise.
- `src/+trackbench/+environment/resolveRegionIdx.m` — sim-engine resolver returning 0 (fallback) or 1..N (region index). Index-based shape lets `runDetections` cache per-region effects in a `cell{N+1}` array indexed by `[idx + 1]`.

**Edited files:**
- `loadRunFile.m`:
  - Added `config.degradation.has_fallback` flag (set true when fallback weather is configured, false in the region-only case). Distinguishes "weather everywhere" from "regions-only, clear elsewhere."
  - Step 9 calls `composeHeightmap` after `generateTerrain` if `terrainRegions` is non-empty. Empty regions → bit-for-bit identical to legacy.
- `runDetections.m`:
  - Pulls `terrainRegions`, `weatherRegions`, `hasFallbackWeather` from `cfg` near top.
  - Diagnostic prints show `+ N terrain region(s)` / `+ N weather region(s)` lines under the existing OFF/ON status when present.
  - **Per-detection weather:** pre-computes `(pdMult, noiseMult)` per region+fallback ONCE per sensor per scan (avoids per-detection function-handle construction), then resolves per-(x,y) via `resolveRegionIdx` to look up the cached effects.
  - **Per-region ground clutter:** Pass 1 generates clutter per region with that region's terrain config, masking by polygon. Pass 2 fills the fallback for points outside all regions. Legacy fast path when `terrainRegions` is empty (Pass 1 is empty loop, Pass 2 keeps everything).
  - **Per-region weather clutter:** same two-pass pattern at flush time. Pass 2 only runs when `hasFallbackWeather` is true.

**Known scope cuts (intentional, not bugs):**
- Per-region storm window — uses global `cfg.degradation.storm_start_s/end_s`. Regions can't have different storm timings yet.
- Per-region refraction — horizon masking uses fallback `refraction_factor` only.
- Per-region toggles — `terrain_occlusion`, `horizon_masking`, `ground_clutter`, `doppler_fade` are global flags.
- Hard polygon edges in heightmap — no smooth blending. Visually fine, physically clean.
- Doppler fade is global — uses fallback only.

**Backward compat verified by design:** every change reduces to the legacy single-record code path when both regions cells are empty. No behavior change is expected for any existing run file. (User-confirmed: legacy run files are no longer needed; only the new framework matters.)

---

## What's NEXT (resume here)

### Step 5c — Polygon drawing UI for terrain regions
- Wire `EditorState.terrainRegions` collection into the editor UI:
  - Region list panel (similar to targets/sensors panels): add/duplicate/delete/rename, dropdown to switch active region.
  - Mouse handler in Environment mode: click to add polygon vertex, double-click (or Enter) to close. Drag a vertex to edit. Esc to cancel.
  - `drawMap.m` updates: render region polygons as semi-transparent fills tinted by terrain type (mountain=brown, water=blue, etc.).
  - File-I/O: extend `exportToJSON` / `openScenarioFromJSON` to round-trip the regions block. Region records persist as `{name, config, polygon_xy}` per the 5a JSON shape.
- Mode dispatch: a new sub-mode under "Environment" for polygon editing, or a fourth top-level mode? Lean toward sub-mode.

### Step 5d — Polygon drawing UI for weather regions
- Same as 5c but for `weatherRegions`. Probably reuses 5c's polygon-editing helpers — extract a shared `+trackbench/+editor/+polygon/` package if the code grows.

### Step 6 — Save/load figures per run / session / tracker
Last item on the build order list. Persistence layer for the visual outputs (not just data).

---

## Other deferred / pending

- **Per-region storm window / refraction / toggles** (5b scope cuts above) — implement when needed for a specific demo scenario.
- **Diagnostic suite stale references** — `verifySimulation.m` still has refs to removed VCP code (45 PASS / 11 FAIL / 2 WARN); failures are not GUI bugs, just stale checks. Cleanup deferred.
- **TC-05** in test plan still at 1.10× ratio (threshold 1.15×) — geometry/threshold tuning issue, not a code bug.
- **Tail_687_1/ not bundled in installer** — NASA flight features won't work from installed app. Quick fix when needed: include as `AdditionalFiles` in `build_installer.m` + update `runNASAFlight.m` path resolution.
- **`compiler.package.installer` mention in `verifySimulation.m`** triggers a benign mcc-warning during `build_executable`. Safe to leave, or wrap body with `if isdeployed; error; end` to silence.
- **AUTHORS.md draft** — deferred. Will list the 5 team members with their scope (Michael: lead/sim engine/GUI/installer; Daniel: weather sensor effects + initial scenario rendering; James: weather dispatch routing; Kaz/Olega: scope TBD).
- **Original 4-button main menu / `runScenarioGUI.m`** is orphaned but kept on disk as reference. Same for `launch_trackbench.m`.

---

## Release workflow (confirmed, for any code change shipped to Boeing)

```matlab
% Dev work (no rebuild needed):
mainMenu

% Code change → fresh installers:
build_executable          % ~5 min  — bakes .m files into mainMenu.exe
build_installer('both')   % ~12 min — wraps EXE → web + offline installers
% Web flavor only ~30s:    build_installer('web')
```

Both required for a release. EXE alone has no MCR/Start Menu/uninstall registration.

---

## Key gotchas to keep in mind

- After ANY edit to a file in `+trackbench/` or `+trackbench/+editor/`: run `clear classes; clear all` before `mainMenu` — MATLAB caches package members aggressively.
- When adding a new `state.<field> = uibutton(...)` in `buildUI`, the matching `<field> = gobjects(1)` MUST be added to `EditorState.m` properties block. `checkcode` does NOT catch missing-property errors at lint time (runtime check only). The strict classdef throws mid-build, leaving downstream panels un-built — that's the "empty panels" failure mode.
- Value-class assignment chains like `state.terrain.field = X` work in modern MATLAB but the codebase prefers explicit read-mutate-writeback (`tr = state.terrain; tr.field = X; state.terrain = tr;`). Match the existing style. Same applies to the new `terrainRegions(idx).polygonXY = ...` pattern in 5c.
- `runNASAFlight.m` resolves `Tail_687_1/` as a SIBLING of the project root. Don't move them apart.
- `compiler.package.installer` for mcc-built apps requires TWO positional args: `(exePath, productsFile, NameValue...)`. R2025a+ prefers `buildresult.json` over legacy `requiredMCRProducts.txt`.
- mcc artifacts (`buildresult.json`) live next to `trackbench/mainMenu.exe`, NOT a project-root copy.
- `Filesystem:edit_file` is fragile with multi-line replacements; verify with read-back after each edit.
- For files over ~400–500 lines: copy-to-Claude → `sed` edit in bash → read back and write (more reliable than inline rewrites).
- **5a/5b region representations are PARALLEL but distinct:**
  - Editor side uses typed value-class arrays: `EditorState.terrainRegions` (`TerrainRegionRecord[]`), `EditorState.weatherRegions` (`WeatherRegionRecord[]`). Inner record is a `TerrainRecord` / `WeatherRecord` cache. Resolvers: `resolveTerrainAt` / `resolveWeatherAt`.
  - Sim engine side uses cell-array-of-structs: `config.environmentRegions{i}` and `config.degradationRegions{i}` are structs with fields `{config_path, name, polygon_xy, def}` where `def` is the parsed JSON terrainDef/weatherDef. Resolver: `resolveRegionIdx` (returns 0 or 1..N for cache indexing).
  - **Both serialize to/from the same JSON shape.** 5c/5d will need exporters that read editor records → JSON, and importers that read JSON → editor records. The sim-engine cell shape is a load-time intermediate.
