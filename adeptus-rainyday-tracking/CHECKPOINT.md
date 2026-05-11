# Build Order Checkpoint — v3.5 GUI Rebuild

**Last checkpoint:** 2026-05-10, after Step 5e end-to-end user verification — FULLY COMPLETE. Heightmap renders correctly in 2D (hypsometric tint via truecolor `image()`) and 3D (`surf()` with FaceAlpha 0.7) in the editor. Multi-region composition in the editor exactly matches what the sim engine produces — same `composeHeightmap` call, same `[Heightmap] Region N: "..." -> X / Y cells` output. End-to-end smoke test: user manually built water-fallback + mountain-region scenario in the editor, exported as `Step_5e_verification`, ran via `runSingleScenario`. Editor preview showed `10273 / 40000` mountain cells; sim console reported the same; GNN+IMM tracker completed cleanly (posRMS=956m, no swaps). Two follow-up bugs surfaced and triaged below (1 fixed, 1 deferred). Demo-day deadline = 2026-05-29; remaining phases: 5c.6 → tracker editor → manual → video.

**Demo day:** 2026-05-29 (19 days out). Tentative phase plan:
  - **5c.6 vertex-drag polygon editing** — May 10–14
  - **Tracker editor (Run Simulation GUI)** — May 15–20
  - **Install/usage manual** — May 21–23
  - **Merge manual + technical report** — May 24–25
  - **Demo video (5–7 min walkthrough)** — May 26–28
  - **Buffer / demo prep** — May 29

**Active project root:** `C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking` (branch `Michael---Working_on_pathEditor`)
**Boeing meeting:** Friday 2026-05-01 — PASSED. No active deadline; iteration mode.

## 5e verification — bugs surfaced + triage (2026-05-10)

**Two pre-existing issues were exposed by 5e verification, neither is a 5e bug:**

1. **`buildUI.m` line 1145: `terrainOverlayCB` defaulted to `Value: false`** — contradicted the documented "Default ON so users see terrain on first open" decision. Heightmap was invisible until the user manually checked the box, masking 5e success on first launch. **FIXED**: Value: true, plus tooltip updated to reflect heightmap behavior (was: "Tint the map with a type-coloured terrain overlay (2D only)" → now: "Render procedural terrain heightmap on the map (2D hypsometric tint, 3D surface). Uncheck if rendering ever lags."). buildUI.m: 6298 → 6300 lines.

2. **Editor cannot import target files with non-waypoint behaviors** — `loadFromJSON.m` only handles `behavior="waypoints"` but the rest of `config/targets/` includes `crossing`, `orbit`, `s_maneuver`, `head_on`, `approach`, etc. Hit when trying to round-trip `config/runs/test_multi_region.json` (its target file used `behavior="crossing"`). **PARTIAL FIX**: converted `config/targets/crossing_pair/region_test_target.json` from `crossing` → `waypoints` (2-point trajectory, same straight-line motion). The test fixture now round-trips through the editor. **DEFERRED (post-5c.6 polish):** Extend `loadFromJSON.m` to synthesize waypoints from `crossing`/`constant_velocity`/`orbit`/etc. so the editor can open ANY existing showcase scenario. ~1 hour of work; tracks all behaviors in `+trackbench/+scenario/addTargetFromDef.m`. Worth doing for step 2 (tracker editor) era to align with the "editor is the primary authoring tool" goal.

3. **3D toggle button is mode-specific** — only shown in the Targets sub-panel; Sensors/Environment modes hide it. `V` keyboard shortcut works globally (per Help text). **DEFERRED (post-5c.6 polish):** add a global 3D toggle button somewhere mode-independent (toolbar above the map, or in a top-level row of the editor panel).

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

### Step 5a + 5b smoke test — VERIFIED END-TO-END (2026-04-26)

**Test files (committed):**
- `config/runs/test_multi_region.json` — exercises both new shapes (terrain `{fallback, regions[]}` + weather `{fallback: "none", regions[]}`)
- `config/targets/crossing_pair/region_test_target.json` — single airliner, 240s due-east transit y=0 from x=-30km to +30km

**Geometry:**
- Fallback terrain: `rural/default_rural` (61m peaks)
- Mountain region polygon: `[-15,+15] × [-15,+15] km` referencing `mountain/default_mountain` (1487m peaks)
- No weather fallback — `"none"` (clear outside regions)
- Rain region polygon: `[+5,+35] × [-15,+15] km` referencing `rain/default_rain` (16 mm/hr step)
- Storm window 0–240s spans the full scenario

**Verified behaviors:**
- ✅ `parseTerrainField` / `parseWeatherField` polymorphic dispatch fired (legacy string + new struct shape both parse)
- ✅ `composeHeightmap` stamped 484 / 40000 cells = ~1.2% (matches the 30×30 km rectangle in a 260×260 km auto-bounded grid)
- ✅ `runDetections` recognized both region collections (`+ 1 terrain region(s)` / `+ 1 weather region(s)` lines printed)
- ✅ `has_fallback=false` correctly suppressed Pass 2 fallback weather clutter ("no global fallback — clear sky outside regions" message)
- ✅ Mountain visible in 3D scenario plot as elevated terrain in the rural plain
- ✅ Detections completed cleanly (22 scans, target tracked through both regions)
- ✅ Tracker (GNN+IMM) ran to completion with results saved
- ✅ Visual evidence in tracker output: T08→Truth1 association during t=60-95s mountain entry; gap during t=95-200s overlap of mountain exit + rain region; T59→Truth1 picks up after t=200s — region effects propagate visibly through to tracking

**Tracker quality is poor** (posRMS 1100-1683m, Quality 4-6%) but that's expected — clutter spike at polygon edges is what we wanted to demonstrate. Track quality is a separate tuning problem; 5b correctness is unrelated.

**One transient issue:** Initial run hit a MATLAB graphics handshake timeout at `plotInitialScenario:211`. NOT a 5a/5b bug — known MATLAB OpenGL/GPU issue. Resolved by computer reboot. Workaround if it recurs: `opengl software` then re-run.

### Step 5c.1 — EditorState region mutator API (FULLY COMPLETE — 2026-04-26)

Foundation work for 5c. No UI yet; this is the data-layer API the upcoming Regions sub-panel (5c.2) and polygon-edit sub-mode (5c.3) will call into.

**Decision (signed off):** Option B inheritance for new regions. `addTerrainRegion` / `addWeatherRegion` seed the new region with the scenario's FALLBACK record so the new region is immediately resolvable (composeHeightmap stamps the same terrain inside the polygon — visually a no-op until the user changes configPath via `loadTerrainRegionConfig`). Mirrors the duplicate pattern in targets/sensors. Synthesizes a `<TYPE>/default_<TYPE>` configPath when the fallback's own configPath is empty.

**Edited file:** `src/+trackbench/+editor/EditorState.m`

**New properties:**
- `activeTerrainRegionIdx (1,1) double = 0`
- `activeWeatherRegionIdx (1,1) double = 0`

No separate `regionsDirty` flag — region mutations bump `environmentDirty` since regions are conceptually environment edits.

**16 new methods (8 mirrored pairs, all pushUndo + bump anyDirty/environmentDirty):**
- `hasActiveTerrainRegion` / `hasActiveWeatherRegion` (predicates)
- `addTerrainRegion(name?)` / `addWeatherRegion(name?)` — seeded from fallback (Option B); auto-name `region_<n+1>` if name omitted
- `duplicateActiveTerrainRegion` / `duplicateActiveWeatherRegion` — `_copy` suffix, polygon offset +2km east
- `deleteActiveTerrainRegion` / `deleteActiveWeatherRegion` — manages activeIdx clamping
- `renameActiveTerrainRegion(newName)` / `renameActiveWeatherRegion(newName)` — validates non-empty + unique within own collection (collections independent)
- `setActiveTerrainRegionIdx(newIdx)` / `setActiveWeatherRegionIdx(newIdx)` — view state, no undo
- `setActiveTerrainRegionPolygon(polyXY, commit?)` / `setActiveWeatherRegionPolygon(polyXY, commit?)` — commit=true pushes undo (drag start / vertex commit), commit=false is the live-update path. Accepts any Nx2 (validation in `isValidPolygon` skips invalid <3-vertex polygons at runtime)
- `loadTerrainRegionConfig(relPath)` / `loadWeatherRegionConfig(relPath)` — swap inner record via `loadTerrainFromJSON` / `loadWeatherFromJSON`; preserves polygon; rolls back undo on parse error

**Snapshot/restore:** Both new active-index properties captured + restored with backward-compat clamping. Pre-5c snapshots restore as `activeIdx=0` (no active region). Restored indices are clamped to the restored collection size to handle Add→undo across deletes safely.

**6 new file-scope helpers** (mirrors `sensorNameExists`/`uniquifySensorName` pattern):
- `terrainRegionNameExists(obj, name)` / `weatherRegionNameExists(obj, name)` (excludes active idx)
- `anyTerrainRegionHasName(obj, name)` / `anyWeatherRegionHasName(obj, name)` (full scan)
- `uniquifyTerrainRegionName(obj, base)` / `uniquifyWeatherRegionName(obj, base)` (appends `_2`, `_3`…)

The collections are INDEPENDENT — a terrain region named `storm1` does not collide with a weather region named `storm1`.

**Smoke-test verified end-to-end (2026-04-26):**
- `addTerrainRegion("test_region")` → idx=1, name="test_region", configPath="rural/default_rural" (Option B inheritance)
- `setActiveTerrainRegionPolygon([4 vertex square])` → polygonXY 4x2, isValidPolygon=true
- `duplicateActiveTerrainRegion` → 2 regions, copy named "test_region_copy", polygon east-shifted by +2000m
- 2x undo → 1 region, polygon vertices=0 (back to empty original — both Add and setPolygon individually undoable)
- `setWeatherType("rain")` then `addWeatherRegion("storm1")` → idx=1, configPath="rain/default_rain" (inherits from rain fallback)
- `deleteActiveWeatherRegion` → 0 regions, activeWeatherRegionIdx=0

**MCP edit tactic learned:** large `Filesystem:edit_file` payloads can hit a 4-min server timeout (file untouched). Splitting into 5–6 smaller `edit_file` calls (each ~80–120 lines of new content) avoids the timeout reliably. Anchor each edit on the END of the previously-inserted block to keep oldText unique. Confirmed working approach for files >1500 lines.

**Cosmetic note:** the `loadTerrainRegionConfig`/`loadWeatherRegionConfig` methods landed between rename* and setActive*Idx in the methods block (oldText boilerplate matched first occurrence). Functionally identical to a tail-of-region-block placement — MATLAB doesn't care about method ordering inside a `methods` block. Not worth a corrective edit.

### Step 5c.2 — Regions sub-panel UI (FULLY COMPLETE — 2026-04-26)

First end-user-visible piece of the regions feature. Adds an Environment-mode sub-mode toggle and two new sub-panels (Terrain Regions, Weather Regions) wired to the 5c.1 mutator API.

**Decisions (signed off):**
- **Option A (sub-mode under Environment).** A `[Fallback] [Regions]` toggle inside Environment mode picks between the existing fallback panels and the new regions panels. Not a fourth top-level mode — keeps the top-level mode toggle stable.
- **Q1: 250 px Regions panels.** Visual symmetry with the existing 280 px Terrain panel; 7 internal rows fit comfortably.
- **Q2: sub-mode toggle hidden until Environment mode.** Matches `applyEditMode`'s existing show/hide pattern; no greyed-out version in Targets/Sensors mode.
- **Inline rename via name field** (no explicit Rename button) — cleaner row of action buttons (Add / Duplicate / Delete only).
- **Edit Polygon button greyed in 5c.2.** Full polygon-edit state machine arrives in 5c.3; the button is constructed with `Enable='off'` so this is enforced at the widget level.
- **Change Config wired immediately.** Hooks into `loadTerrainRegionConfig` / `loadWeatherRegionConfig` from 5c.1 — cheap to do here, no point deferring.

**Edited files:**

*`EditorState.m`*
- New property: `envSubMode (1,1) string = "fallback"` (after `editMode`).
- New method: `setEnvSubMode(mode)` — view-state setter, no undo, mirrors `setEditMode`. Accepts `"fallback"`/`"regions"` only.
- 26 new UI handle properties (all `gobjects(1)` with comment headers explaining ownership):
  - `envSubModePanel`, `envSubModeFallbackBtn`, `envSubModeRegionsBtn`
  - `terrainRegionsPanel`, `terrainRegionsDD`, `terrainRegionsBtn{Add,Duplicate,Delete}`, `terrainRegionsNameField`, `terrainRegionsConfigLabel`, `terrainRegionsPolygonStatusLabel`, `terrainRegionsBtn{EditPolygon,ChangeConfig}`
  - Same parallel set for `weatherRegions*`.
- snapshot/restore now captures + restores `envSubMode` with backward-compat for pre-5c.2 snapshots (missing field → `"fallback"` default).

*`buildUI.m` (5119 → 5931 lines, +812)*
- Inner grid expanded **11 → 14 rows**: `{85, 145, 378, 230, 378, **50**, 280, 340, **250, 250**, 240, 65, 60, 115}`. New rows: 6 (sub-mode toggle, 50 px), 9 (Terrain Regions, 250 px), 10 (Weather Regions, 250 px). Selection/File/Undo/Help shifted from rows 8/9/10/11 to 11/12/13/14.
- Three new builders: `buildEnvSubModeTogglePanel` (state-button radio pair, mirrors `buildModeTogglePanel`), `buildTerrainRegionsPanel`, `buildWeatherRegionsPanel`.
- Three new refresh helpers: `refreshEnvSubModePanel`, `refreshTerrainRegionsPanel`, `refreshWeatherRegionsPanel`. Plus `polygonStatusText` shared helper that produces strings like `"4 vertices  ✓ valid"` / `"2 vertices  ⚠ needs ≥3"` / `"empty (draw a polygon to define this region)"`.
- 14 new callbacks: 2 sub-mode toggle (mutual-exclusion radio pair), 6 terrain region (DD change, Add, Dup, Del, NameField inline rename, ChangeConfig + EditPolygon stub for 5c.3), and 6 mirrors for weather.
- `applyEditMode` updated:
  - `envFallbackOn = environmentOn && envSubMode=="fallback"`
  - `envRegionsOn  = environmentOn && envSubMode=="regions"`
  - Panel visibility: `envSubModePanel` shown when `environmentOn`, Terrain/Weather panels gated on `envFallbackOn`, Regions panels gated on `envRegionsOn`.
  - Row-collapse logic updated for the new row indices.
  - Environment mode also calls the 3 new refreshers (regardless of which sub-mode is active) so flipping the toggle shows current data on first reveal.
- buildUI seed-time call wires `refreshTerrainRegionsPanel`, `refreshWeatherRegionsPanel`, `refreshEnvSubModePanel` before `applyEditMode`.
- `refreshAfterEnvironmentChange` (called from undo/redo / Open Scenario / Load Terrain or Weather) extended to call the 3 new refreshers.

**Smoke-test verified end-to-end (2026-04-26):**
12-step manual walkthrough all green:
1. Default state — sub-mode toggle hidden in Targets mode ✅
2. Click Environment → sub-mode toggle visible, Fallback active, Terrain + Weather panels show ✅
3. Click Regions → Terrain/Weather hidden, Terrain Regions + Weather Regions panels show ✅
4. Add terrain region → region_1 created, configPath inherited from rural fallback, all expected widgets enabled ✅
5. Inline rename via name field → dropdown updates with `(active)` tag ✅
6. Duplicate → `_copy` suffix ✅
7. Delete with confirm ✅
8. Change config… file picker → loadTerrainRegionConfig → polygon preserved across config swap ✅
9. Toggle back to Fallback → Terrain + Weather show again, region preserved in state ✅
10. Mirror operations on weather region ✅
11. Switch to Targets → sub-mode toggle + all 4 environment panels collapse to 0 ✅
12. Ctrl+Z walks back through region operations correctly ✅

**One layout-edit gotcha during build:** the initial RowHeight + uigridlayout-dimension edit silently failed when anchored on a multi-line comment block with variable-width box-drawing characters. Re-anchoring on the unique `inner.RowHeight = {85, 145, ...}` line itself worked first try. Lesson: anchor on stable single-line statements, not box-drawing comment headers, when working with this file's CRLF + Unicode comment layout.

### Step 5c.3a — Polygon-edit state machine + click dispatch + UI lockdown (FULLY COMPLETE — 3 bug fixes applied)

**Decision recap (signed off in chat):**
- Q1 = B — entering edit mode seeds `polygonEditDraft` with the active region's existing `polygonXY`. User edits in place, doesn't redraw from scratch.
- Q2 = A — UI is locked during edit. Mode toggle, sub-mode toggle, both regions' Add/Dup/Del/Name/EditPolygon/ChangeConfig, both regions dropdowns, File panel buttons, Undo/Redo buttons all disabled. Esc + Enter remain live for abort/commit.
- Step 5c.3 was split into 5c.3a (state machine + dispatch + lockdown — done here) and 5c.3b (drawMap rendering — next).
- Ctrl+Z mid-edit refused with status nag (rather than auto-aborting) — less surprising than discarding the user's draft on muscle-memory undo.
- `<3` vertices on commit attempt → refused with status, edit stays active.
- Vertex-by-vertex undo + Delete-key vertex removal deferred to 5c.6.
- Polygon-edit transient state intentionally NOT in snapshot/restore — interrupted edit vanishes if editor closed mid-draw.

**New EditorState API (+108 lines):**
- 3 new transient properties: `polygonEditActive (1,1) logical`, `polygonEditTarget (1,1) string`, `polygonEditDraft (:,2) double`.
- `beginPolygonEdit(target)` — seeds draft from active region's polygonXY, sets active=true. Errors if no active region for target.
- `appendPolygonDraftVertex(x, y)` — pushes vertex, silent no-op if active=false.
- `commitPolygonEdit()` returns ok — refuses with ok=false when <3 vertices (edit stays active). Calls `setActive*RegionPolygon(poly, true)` so undo captures one entry per edit. Clears draft + active=false on success.
- `abortPolygonEdit()` — clears draft + active=false. No undo entry pushed (stored polygon was never touched).

**buildUI.m additions (+285 lines initial + 3 bug-fix follow-ups):**
- Edit Polygon stubs replaced with real `state.beginPolygonEdit("terrain"/"weather")` + `applyPolygonEditLockdown(state)` calls. Status reports vertex count (existing seed) or empty start.
- `onAxesClick` polygon-edit branch (priority: after middle-click pan, before sensor/waypoint dispatch). Single-click appends with ~10px world-meters near-duplicate guard. Double-click (`SelectionType=='open'`) commits via `tryCommitPolygonEdit`. Right-click swallowed. 3D guard posts status nag.
- `onKeyPress` extensions: Escape → `abortPolygonEdit` + lockdown release; Enter → `tryCommitPolygonEdit`; Ctrl+Z/Y → refused with status; Delete/Backspace → status nag pointing at 5c.6.
- New helpers: `tryCommitPolygonEdit` (handles <3-vertex refusal + panel refresh + lockdown release on success), `applyPolygonEditLockdown` (toggles Enable on every locked widget based on `polygonEditActive`).
- Refresh helpers updated: n>0 branches now enable EditPolygon button; n==0 branches now explicitly disable it (Bug A fix).

**Bug fixes applied during 5c.3a verification:**

*Bug A — EditPolygon button stayed enabled after region deletion.* `refreshTerrainRegionsPanel` and `refreshWeatherRegionsPanel` n==0 branches disabled Duplicate/Delete/NameField/ChangeConfig but never touched EditPolygon. After Ctrl+Z removed a region, the button looked clickable but the early-return on `~hasActive*Region` made it appear broken. Fix: explicit `setPropIfGraphics(... 'Enable', 'off')` for EditPolygon in both n==0 branches. Lines 5622, 5693.

*Bug B — Environment-mode map clicks added waypoints to active target.* Pre-existing issue exposed by 5c.3a testing. `onAxesClick` had no environment-mode branch — clicks fell through to targets-mode waypoint logic, accumulating phantom yellow waypoints since the Targets/Selection panels weren't visible in env mode. Fix: env-mode click guard added at line ~1569, after polygon-edit branch and before sensor-mode branch. Left-click posts coordinates with hint about Regions sub-mode + Edit polygon. Right-click swallowed.

*Bug C — Lockdown release stuck mode toggle, sub-mode toggle, File panel, and Undo/Redo buttons disabled.* The trap: after committing a polygon, `applyPolygonEditLockdown`'s release branch only called `applyEditMode`, which only handles panel visibility + region widgets. It never re-enabled the mode/sub-mode/File/Undo-Redo widgets that the lock branch had disabled. User could navigate undo history via Ctrl+Z/Y keyboard shortcuts but couldn't click any buttons or switch modes. Fix: release branch now explicitly re-enables every widget the lock branch disabled (mode toggle, sub-mode toggle, region widgets, File panel buttons via findall walk, Undo/Redo buttons via findall walk) BEFORE delegating to `applyEditMode` for fine-grained rules. Lines ~6230–6280.

**Test sequence verified by user (2026-04-26):**
1. Add region → Edit Polygon enables ✓
2. Click Edit Polygon → lockdown applies (mode toggle, sub-mode, all region widgets, File panel, Undo/Redo all greyed) ✓
3. Click 12 vertices → status updates per click, near-duplicate guard skips dupes ✓
4. Press Enter → commit fires, polygon stores, region panel shows "12 vertices ✓ valid" ✓
5. After commit, ALL buttons re-enable correctly (Bug C fix verified) ✓
6. Ctrl+Z walks back through commit + region creation, ending at no-region state. EditPolygon button correctly grey at n==0 (Bug A fix verified) ✓
7. Ctrl+Y walks forward, region + polygon restored ✓
8. Env-mode map click outside polygon-edit posts "Environment mode — pick Regions sub-mode..." status; no waypoint added (Bug B fix verified) ✓

**Files modified:**
- `src/+trackbench/+editor/EditorState.m` — 2527 → 2635 lines
- `src/+trackbench/+editor/buildUI.m` — 5931 → 6238 lines (after 3 bug fixes)

**Lessons:**
- `applyEditMode` does NOT re-enable mode toggle, sub-mode toggle, File panel, or Undo/Redo buttons. It only handles panel visibility + region widget refresh. Any code path that disables those widgets MUST explicitly re-enable them. The lockdown release path is the canonical example. (Bug C lesson.)
- Env mode lacked any click handler before 5c.3a. Adding regions surfaced the pre-existing fall-through-to-targets-mode bug because the user now expects map clicks to mean *something* in env mode. Worth keeping the env-mode click guard for future env-mode features. (Bug B lesson.)
- Refresh functions that have n==0 vs n>0 branches need to TOUCH EVERY GATED WIDGET in BOTH branches. Asymmetric coverage (only enabling in one branch, leaving stale state in the other) is the failure mode. (Bug A lesson.)
- MCP edit tactic still works at 6000+ lines: split large edits into ≤6 anchored sub-edits, anchor on unique single-line statements (not box-drawing comment headers), verify each diff before continuing.

### Step 5c.3b — Polygon rendering on `drawMap.m` (FULLY COMPLETE — verified end-to-end)

State machine and lockdown were wired in 5c.3a but the user was flying blind during edit (status bar text only). 5c.3b makes the edit visible.

**Style decisions (signed off in chat):**
- **Terrain region palette:** reuse existing `terrainTintColor()` lookup, multiplied by 0.7 (darken 30%) so a region reads against a same-type background tint. Edge color = fill color × 0.6.
- **Weather region palette:** single canonical blue `[0.20 0.45 0.85]` for all weather types, matches storm timeline + path line.
- **Fill opacity:** `FaceAlpha = 0.30`. Sits between bg tint (0.20) and storm timeline patch (0.35).
- **Active region (DD-selected, env mode + regions sub-mode):** outline LineWidth 2.0 vs 1.2 inactive, plus vertex circles MarkerSize 6 (filled fill-color, edge `[0.10 0.10 0.10]`). No numeric labels (would clutter at 12+ vertices).
- **In-progress draft:** dashed `'--'` LineWidth 1.6 in the active region's color, vertex markers MarkerSize 6 at every appended point, most-recent vertex highlighted with a hollow ring MarkerSize 10 / LineWidth 2.0 / EdgeColor `[0.10 0.55 0.25]` (matches existing waypoint selection halo green). Open polyline (no closing edge until commit).
- **Z-order:** terrain bg tint → committed terrain regions → committed weather regions → in-progress draft → sensors/targets/etc. Terrain below weather (ground beneath atmosphere). Drawn after `drawTerrainTint2D` and before the radar marker in `drawMap2D`.
- **3D mode skipped** — same rationale as `drawTerrainTint2D` being skipped in 3D (the 3D view has its own ground plane and editing happens in 2D).

**`drawMap.m` additions (1669 → 1908 lines, +239):**
- New section header `v3.5 §5c.3b — REGION POLYGON RENDERING` at end of file.
- `drawRegionPolygons(ax, state)` — top-level entry point. `isprop` guards on `terrainRegions`/`weatherRegions`/`polygonEditActive` so pre-5c snapshots don't blow up. Computes `inRegionsContext = (editMode=="environment" && envSubMode=="regions")` so the "active region" highlight only applies in that context — in Targets/Sensors mode the regions still render but no single one is highlighted (they're scenario context, not the editing focus).
- `drawCommittedRegion(ax, rec, kind, isActive)` — single-region renderer. Skips records with <3 vertices. During an active polygon edit on the same target, suppresses the COMMITTED render so the draft (which was seeded from the committed polygon) takes its place — prevents two stacked polygons disagreeing as the user edits.
- `drawDraftPolygon(ax, state)` — dashed open polyline + vertex markers + most-recent green ring.
- `regionFillColor(rec, kind)` — `terrain` branch calls `terrainTintColor(rec.terrain.terrainType) * 0.7`; `weather` branch returns canonical blue.
- `draftPolygonColor(state)` — looks up the active region's fill color so the user sees what they're committing to.
- One-line call site insertion in `drawMap2D`: `drawRegionPolygons(ax, state)` between the terrain bg tint block and the radar marker block.

**`buildUI.m` additions (6238 → 6298 lines, +15 drawMap call sites — every region/polygon-edit state mutation now retriggers a redraw):**
- **Polygon-edit transitions (3):** after `appendPolygonDraftVertex` in onAxesClick, after `abortPolygonEdit` in Esc handler, after the panel refresh in `tryCommitPolygonEdit`.
- **Terrain region callbacks (6):** end of `onTerrainRegionDDChanged`, `onTerrainRegionsAdd`, `onTerrainRegionsDuplicate`, `onTerrainRegionsDelete`, `onTerrainRegionsEditPolygon` (after `applyPolygonEditLockdown`), `onTerrainRegionsChangeConfig`.
- **Weather region callbacks (6):** mirror set.
- Skipped: `onTerrainRegionsNameFieldChanged` / `onWeatherRegionsNameFieldChanged` — pure rename, no map representation. Skipped: `refreshTerrainRegionsPanel` / `refreshWeatherRegionsPanel` themselves — follows the established contract that panel-refresh helpers do not redraw (callers are responsible). Undo/redo path already triggers `drawMap` via `refreshAfterActiveTargetChange`, so region mutations under undo/redo redraw automatically.

**Verified by user:**
1. Add region → Edit Polygon → click N vertices: each click should now draw a vertex marker; most-recent shows green ring; ≥2 vertices show dashed connecting polyline.
2. Press Enter → dashed line + ring vanish, region fills with translucent darkened-type color + darker outline. Active vertex circles still show while in env+regions context.
3. Switch to Fallback sub-mode → fill stays, vertex circles disappear (no longer "active").
4. Switch to Targets mode → fill stays (regions are scenario context), still no vertex circles.
5. Two overlapping regions of different types: both fills layered (first-listed wins for sim-engine resolution per 5a, but visually both fills are drawn).
6. Weather region on top of terrain region: blue fill above terrain fill.
7. Ctrl+Z back through everything: undo redraws correctly each step.
8. 3D view-mode toggle: regions vanish (matches terrain-tint skip).

**One side-effect to flag:** `Filesystem:edit_file` converted `drawMap.m` from CRLF to LF during the helper-function append. MATLAB on Windows reads either fine — no functional issue — but `git diff` will likely show every line as changed. `git diff -w` ignores whitespace; alternatively `unix2dos drawMap.m` or VSCode "Change End of Line Sequence: CRLF" restores the prior convention. `buildUI.m` and `EditorState.m` were already LF — no churn there.

**Files modified:**
- `src/+trackbench/+editor/drawMap.m` — 1669 → 1908 lines (line endings: CRLF → LF, see flag above).
- `src/+trackbench/+editor/buildUI.m` — 6238 → 6298 lines.

### Step 5c.5 — JSON round-trip for regions block (FULLY COMPLETE — verified end-to-end)

5a/5b already round-tripped the new `{fallback, regions[]}` shape on the SIM ENGINE side via `loadRunFile.parseTerrainField` / `parseWeatherField`. 5c.5 closes the loop on the EDITOR side so a scenario authored with regions, exported, and re-opened in the editor preserves polygons + configPaths.

**Decisions (signed off in chat):**
- **(1A) Backward-compat string scalar when no regions.** When both `state.terrainRegions` and `state.weatherRegions` are empty, exporter emits the legacy string scalar (`"terrain": "<TYPE>/<stem>"`, `"weather": "<TYPE>/<stem>"` or `"none"`) so files exported from no-regions sessions look byte-identical-ish to v3.4 exports. The choice is INDEPENDENT per side: terrain can be a struct while weather stays a string scalar (and vice versa).
- **(2B) Round polygon coordinates to 0.1 m on export.** Clean JSON output without IEEE-754 trailing-digit ugliness, lossless for any realistic editing workflow (the click handler resolves at world-meters-per-pixel which sits in the meter range at typical zooms).

**`exportSensorsToJSON.m` additions (509 → 616 lines, +107):**
- New section header `v3.5 §5c.5 — Multi-region run-file emitters` at end of file.
- `buildTerrainField(fallbackRef, state)` — returns char (legacy) when no regions, struct (new) otherwise. `isprop` guard so pre-5c snapshots fail cleanly to legacy.
- `buildWeatherField(fallbackRef, state)` — same logic. The fallback string may be the literal `"none"` when state.weather is empty but state.weatherRegions is not (regions-only case) — the sim engine accepts that inside the struct shape so no special handling.
- `buildRegionsArray(regions)` — cell-of-structs wrapper. Cell rather than direct struct array keeps each region as a JSON object even at N=1.
- `buildRegionStruct(rec)` — single region serializer. Field names match `loadRunFile.parseRegion`: `name`, `config` (REQUIRED), `polygon_xy` (Nx2). Polygon emitted as a CELL OF ROW VECTORS so jsonencode produces `[[x,y],[x,y],...]` consistently — direct emission of an Nx2 matrix degenerates to `[x,y]` at N=1 (no outer wrap), breaking the schema. Cell wrap costs nothing and avoids the edge case.
- Two-line edit in `writeRunFile`: `runFile.terrain = buildTerrainField(...)` and `deg.weather = buildWeatherField(...)`.

**`openScenarioFromJSON.m` additions (286 → 555 lines, +269):**
- New section header `v3.5 §5c.5 — Multi-region run-file parsers` between `defaultDegradationLocal` and the `Path resolvers` section.
- `parseTerrainFieldEditor(field, state)` — polymorphic dispatch (string → legacy single-component, struct → fallback + regions). Returns `(TerrainRecord, TerrainRegionRecord 1xN)`. Mirrors the sim engine's `parseTerrainField`.
- `parseWeatherFieldEditor(field, state)` — same shape; fallback may be empty `WeatherRecord.empty` (when string is `"none"` or struct.fallback is `"none"`/missing).
- `parseRegionsToRecords(regs, kind, state)` — normalizes the jsondecode struct-array-vs-cell-array quirk (`if isstruct(regs); regs = num2cell(regs); end`), iterates and builds typed value-class arrays. Per-region failures (missing config, bad polygon, unresolvable config path) warn-and-skip rather than aborting the open — more forgiving than the sim engine's `parseRegion`. Inner record (`rec.terrain` / `rec.weather`) loaded via `loadTerrainFromJSON` / `loadWeatherFromJSON`; failure leaves the default and warns so the user can fix Change Config.
- `normalizePolygonXY(raw, regionName)` — coerces jsondecode quirks: `[]` → `zeros(0,2)`, single-pair vector → 1x2, transposed 2xN → Nx2.
- `loadEnvironmentFromRun` body rewritten: terrain branch and weather branch both delegate to the polymorphic parsers; region-collection reset moved to top of function for partial-failure consistency; active-idx clamp added at end (defaults to 1 if regions exist, 0 otherwise; envSubMode intentionally NOT reset).
- `state.terrain` / `state.weather` are touched on the existing assignment paths; `state.terrainRegions` / `state.weatherRegions` / `state.activeTerrainRegionIdx` / `state.activeWeatherRegionIdx` are touched on the new paths.

**Round-trip behavior:**
- Existing scenario without regions → export emits legacy string scalar → byte-identical-ish to v3.4 export.
- Existing scenario with regions → export emits new struct shape → sim engine accepts it (5b) → editor importer accepts it back (this step).
- Mixed: terrain has regions, weather doesn't → `terrain` becomes struct, `weather` stays string. Sim engine and editor both tolerate.
- `"weather": {"fallback": "none", "regions": [...]}` → "regions only, clear sky elsewhere" case from 5b. Round-trips cleanly.
- Polygon coordinates: jsonencode emits e.g. `[[1234.5,2345.7],[3456.8,4567.9]]` (clean 0.1 m precision).

**Verified by user:**
1. Open `config/runs/test_multi_region.json` (the 5b smoke-test fixture) in the editor. Both regions (mountain terrain region, rain weather region) should appear in their respective dropdowns. Polygons should render on the map with the 5c.3b styling.
2. Save scenario via Export Scenario → inspect the run JSON. `terrain` should be `{fallback, regions: [...]}`; `degradation.weather` similarly. Polygon coordinates rounded to 0.1 m precision. Each region has `name`, `config`, `polygon_xy` fields.
3. Open the freshly-exported run JSON in the editor again — polygons + configPaths preserved.
4. Open a v3.4 / no-regions scenario (e.g. `dasr_storm.json`) → round-trip without regions → export shape is the legacy string scalar (no `regions: []` clutter).
5. Manually craft a malformed run file (e.g. region with missing `config` field) → open in editor → should warn and skip the bad region without aborting the whole open.
6. Active region indices: open a scenario with 3 terrain regions → `activeTerrainRegionIdx == 1`; open a scenario with no terrain regions → `activeTerrainRegionIdx == 0`.

**Side-effect to flag (same as 5c.3b):** `Filesystem:edit_file` converted both `exportSensorsToJSON.m` and `openScenarioFromJSON.m` from CRLF to LF during the helper-function appends. MATLAB on Windows reads either fine. Same remediation options as for `drawMap.m`: `git diff -w` ignores whitespace; `unix2dos <file>` or VSCode "Change End of Line Sequence: CRLF" restores the prior convention. Three files now affected total (drawMap.m, exportSensorsToJSON.m, openScenarioFromJSON.m) — might be worth running `git diff -w` to extract real changes for review, then renormalizing line endings before commit if you want a cleaner blame history.

**Files modified:**
- `src/+trackbench/+editor/exportSensorsToJSON.m` — 509 → 616 lines (line endings: CRLF → LF).
- `src/+trackbench/+editor/openScenarioFromJSON.m` — 286 → 555 lines (line endings: CRLF → LF).

### Step 5c.5 follow-up — `runSimGUI.m` polymorphic weather field (BUG FIX, FULLY COMPLETE — verified end-to-end)

The user's first multi-region export RAN successfully end-to-end (the 3-terrain-region + 2-weather-region scenario produced a clean GNN+IMM track with `[RUN] Weather: 2 region(s), no fallback (clear outside regions)` confirming the new struct shape made it all the way through `trackbench.config.loadRunFile` to `runDetections`). But when they opened the same run file from `mainMenu` → Run Simulation, `runSimGUI` crashed:

```
Error using string
Conversion to string from struct is not possible.
Error in runSimGUI/loadRunFile (line 352)
    weatherVal = char(string(getFieldOrDefault(deg, 'weather', 'none')));
```

`runSimGUI` does its OWN `jsondecode` of the run file (separate from `trackbench.config.loadRunFile`) for display purposes, and that local code was still assuming `deg.weather` is a string scalar. The polymorphic struct shape that 5c.5 introduced broke the `string(struct)` cast.

**Two bugs, two fixes:**

1. **Read crash (line 352).** Polymorphic unwrap: if `deg.weather` is a struct, extract `.fallback` for the dropdown; if string/char, use as-is; otherwise default to 'none'. The dropdown only shows the fallback — region editing remains in the Path Editor.

2. **Latent write data-loss (line 500).** `cfg.degradation.weather = char(ddWeather.Value);` would have unconditionally replaced any regions struct with a flat string on Save Changes. Fixed to preserve regions: if the original cfg's weather was `{fallback, regions[]}`, keep the regions and only mutate the fallback. Else emit a legacy string scalar.

**Codebase audit (no further fixes needed):**
- `runSingleScenario.m` — consumes `trackbench.config.loadRunFile`'s parsed output (`config.environment.terrain_type`), not raw JSON. Polymorphic-safe.
- `validationDocsGUI.m` — no `jsondecode` calls; delegates to `runTestPlan` / `verifySimulation` which use `trackbench.config.loadRunFile`. Polymorphic-safe.
- `mainMenu.m` — dispatcher only, no JSON parsing. Polymorphic-safe.
- `runSimGUI.m` was the lone stray that read run files directly with the old shape assumption — fixed in this entry.

**Architectural takeaway:** anything that does raw `jsondecode` on a run file needs to be polymorphic-aware. Anything that consumes the output of `trackbench.config.loadRunFile` is automatically safe — the sim engine's parser is the single point of truth. Future GUI helpers should prefer the latter pattern.

**Files modified:**
- `scripts/runSimGUI.m` — 803 → 836 lines (+33). Line endings: CRLF → LF (same Filesystem:edit_file conversion as the other v3.5 §5c work). Four files now CRLF-converted total: `drawMap.m`, `exportSensorsToJSON.m`, `openScenarioFromJSON.m`, `runSimGUI.m`.

### Step 5e — Terrain heightmap preview in editor (IMPLEMENTED, pending end-to-end verification)

User flagged that authoring multi-region terrain scenarios was effectively blind — the editor showed flat tints + polygon outlines, but the actual elevation (rural rolling hills vs mountain peaks vs desert ripples) only became visible after a run completed. 5e closes that gap by rendering the same procedural heightmap the sim engine uses, directly in the editor's 2D and 3D views.

**Decisions (signed off in chat):**
- **(A) 2D presentation:** hypsometric tint (deep blue water → green lowlands → tan/brown foothills → white peaks). Standard topo-map look. Replaces the old flat color tint when the heightmap helper succeeds.
- **(D) 3D presentation:** `surf()` with FaceAlpha 0.7 so waypoints/sensors above stay readable, EdgeColor='none' for a smooth surface (200x200 grid would otherwise be lined out by every cell edge).
- **(G) Multi-region merge:** uses the existing `trackbench.environment.composeHeightmap` (already standalone, callable from outside `runDetections`/`loadRunFile`) to stamp regions onto the fallback layer. First-listed-wins resolution rule — same as the sim engine, bit-for-bit.
- **Cache via persistent variable:** keyed on (terrain type, terrain scale, region hash, scenario bounds). Hashes via per-region `(terrainType, scale, polygon vertex count, polygon vertex sum)` so a single dragged vertex invalidates correctly. Cheap to compute per draw, exact to compare.
- **Always-on with existing checkbox gate:** the existing 2D "Overlay on map" checkbox now controls both 2D and 3D heightmap rendering. Default ON so users see terrain on first open; uncheck to disable if rendering ever lags.
- **Truecolor RGB rendering:** both 2D `image()` and 3D `surf()` use precomputed MxNx3 RGB CData rather than the axes colormap, so the heightmap doesn't fight with `drawWaypointsByAltitude` (which sets `parula` + `clim` on the same axes). Same hypsometric ramp is reused in both views via `elevationToRgb` + `hypsometricColormap`.
- **Failure-safe:** 2D falls back to the old flat tint via `drawTerrainTint2D` when `getOrComputeEditorHeightmap` throws; 3D silently no-ops (matches pre-5e behavior where 3D had no terrain).

**Reused infrastructure (no changes needed):**
- `trackbench.environment.generateTerrain(type, bounds, scale)` — procedural heightmap, 200x200 grid, deterministic (rng 42), ~50 ms. Includes the radar hilltop clearing tweak so what the editor previews matches the sim's geometry exactly.
- `trackbench.environment.composeHeightmap(fallbackZ, regions, Xg, Yg, scenBounds)` — already a standalone callable function (separate from `runDetections`/`loadRunFile`). Uses the cell-array-of-structs region shape with fields `{config_path, name, polygon_xy, def}`.

**`drawMap.m` additions (1908 → 2208 lines, +300):**
- Two call-site changes: drawMap2D's flat-tint call became `drawTerrainHeightmap2D` with fallback to `drawTerrainTint2D`; drawMap3D got a brand-new `drawTerrainHeightmap3D` call inserted after the radar marker, before the sensor pass.
- Eight new helpers in a new `v3.5 §5e — TERRAIN HEIGHTMAP PREVIEW` section (placed before the existing `v3.5 §5c.3b — REGION POLYGON RENDERING` section):
  - `drawTerrainHeightmap2D(ax, state)` — returns ok/fail logical for caller's fallback decision; uses `image('Parent', ax, 'XData', xData, 'YData', yData, 'CData', rgb, 'AlphaData', 0.85, ...)` then `set(ax, 'YDir', 'normal')` to undo image()'s default Y-flip.
  - `drawTerrainHeightmap3D(ax, state)` — simpler; `surf(ax, Xg, Yg, elev, rgb, 'EdgeColor', 'none', 'FaceAlpha', 0.7, ...)`.
  - `getOrComputeEditorHeightmap(state)` — cached entry point, persistent struct keyed by string fingerprint.
  - `computeHeightmapCacheKey(state, bounds)` — cheap-to-compute string fingerprint.
  - `currentTerrainSettings(state)` — pull terrainType + terrainScale with safe defaults.
  - `computeEditorScenBounds(state)` — mirror of `loadRunFile/computeScenarioBounds` (radar pos + every target's waypoints, max extent × 1.15, floor 130km).
  - `editorRegionsToConfigRegions(terrainRegions)` — adapter from the editor's `TerrainRegionRecord[]` value-class array to the cell-of-structs format `composeHeightmap` expects. Skips records with <3 vertices.
  - `elevationToRgb(elev, cmap)` + `hypsometricColormap()` — precompute MxNx3 truecolor RGB so heightmap rendering bypasses the axes colormap.

**Cache invalidation triggers:**
- Any change to `state.terrain.terrainType` or `state.terrain.terrainScale`.
- Any add/remove/reorder of `state.terrainRegions`.
- Any change to a region's terrain (config swap) or polygon (vertex add/move/delete).
- Any change to scenario bounds (e.g. waypoints moved far enough to shift halfSpan past the 130km floor).

**Known limitations to document for follow-ups:**
- 3D Z-axis limits don't auto-fit the surface peaks. Mountain peaks at 1500m are fine when waypoints fly above them, but a low-flying scenario with mountain terrain could clip the terrain ceiling at the waypoint zMax. Fix would extend `apply3DLimits` to include the heightmap's max elevation.
- Heightmap colorbar: 2D doesn't show one (only the altitude colorbar from `colorByAltitude` mode). Adding one would require a second colorbar instance or layered alpha; deferred.
- During waypoint drag, drawMap fires repeatedly. The cache makes this fast (key compare + return cached arrays) but if it ever does feel sluggish, an explicit "is dragging" flag could skip heightmap rendering entirely mid-drag.
- Polygons that extend past `scenBounds` are silently clipped (cells outside bounds aren't queried by `composeHeightmap`). Matches sim behavior (`computeScenarioBounds` doesn't include polygon vertices), so editor preview equals run output — but if a user authors a region polygon way outside the radar range, the part outside bounds shows fallback terrain. Edge case, document and move on.

**File modified:**
- `src/+trackbench/+editor/drawMap.m` — 1908 → 2208 lines (+300). File was already CRLF-converted in 5c.3b; remains LF after this edit (no new line-ending churn).

**To verify (NEXT user session):**
1. `clear classes; clear all; mainMenu` — then open the editor.
2. Default scenario (no regions, e.g. mountain fallback) — 2D should show hypsometric tint with visible ridge lines; 3D should show the surface where previously empty space.
3. Toggle the "Overlay on map" checkbox off — heightmap disappears, 2D shows old flat tint, 3D shows nothing.
4. Open the 3-region test scenario from earlier today — should show rural fallback with mountain peaks visible inside region 2's polygon and desert ripples inside region 3's. Polygon outlines from 5c.3b should overlay on top.
5. Switch between Targets / Sensors / Environment modes — heightmap should persist (it's gated on the checkbox, not the mode).
6. Toggle 2D ↔ 3D rapidly — should not lag (cache hit on every redraw after the first).
7. Run an existing scenario via `runSingleScenario` — the run's `[Heightmap] Region N` log lines should match what the editor previewed (same generator, same merger, same `claimed` mask).
8. Watch for any colormap fights with `colorByAltitude` mode — truecolor bypass should prevent this, but worth confirming.

---

## What's NEXT (resume here)

### Step 5c.6 — Vertex-drag to edit existing polygon (IMMEDIATE NEXT)
Until now, polygon editing is append-only — click-to-add-vertex during a polygon-edit session, no way to move or remove an existing vertex without aborting and starting over. 5c.6 adds vertex-level interaction:
- **Drag a vertex** of the active region's polygon to move it. Live update via `setActive*RegionPolygon(poly, false)` (no undo per frame); final `commit=true` on mouseup. Mirrors the existing waypoint-drag pattern in `onAxesClick`.
- **Right-click vertex → delete vertex.** Must keep polygon valid — ≥3 remaining vertices required. If the user tries to drop below 3, refuse with a status nag. (Below-3 polygons are still drawable as drafts but invalid for commit, so this is consistent with the existing commit guard.)
- **Shift-click an edge → insert vertex** at the projection point. Mirrors the existing waypoint `insertAfter` pattern. Visual feedback during shift-hover would be nice but is out of scope for the MVP.
- New EditorState methods to add: `moveVertexLive(idx, x, y, commit)`, `deleteVertex(idx)` (errors if it would leave <3), `insertVertexOnEdge(edgeIdx, x, y)`. All push undo on commit.
- Hit-testing: a 10-px vertex-pick threshold matches the existing waypoint-pick threshold. New helper in `buildUI` (or inline in `onAxesClick`): `findRegionVertexAt(state, x, y)` returns `(regionKind, regionIdx, vertexIdx)` or 0 for miss.
- Click dispatch order in `onAxesClick`: polygon-edit branch (existing) → vertex-pick branch (new) → sensor branch (existing) → waypoint branch (existing). Vertex-pick is only active when the editor is in env-mode + regions sub-mode + polygon-edit-NOT-active.

### Step 5c.7 — "Edit existing polygon" entrypoint via direct vertex click (post-MVP polish)
Currently the user must click the Edit Polygon button to enter polygon-edit mode. Once 5c.6 lands, an alternative gesture would be "click a vertex of an inactive committed polygon → select that region as active AND enter polygon-edit mode in one click." Saves a UI roundtrip. Defer until 5c.6 is solid.

### Step 5d — (covered by 5c.3b/5c.5/5c.6)
The original split between 5c (terrain regions) and 5d (weather regions) hasn't really materialized as a separate step — every 5c work item has been built to handle both terrain and weather symmetrically (parallel mutator API, parallel UI panels, parallel rendering, parallel JSON round-trip). 5d is effectively merged into 5c work. Leaving the placeholder here for completeness.

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
