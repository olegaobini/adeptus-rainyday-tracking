# TESTING — pathEditor Milestone 1

Manual test checklist for the click-and-export minimum viable editor.
Takes ~5 minutes end-to-end. Run in MATLAB R2025b with the repo on the path.

## 0. Prerequisites

```matlab
cd('<path to>\adeptus-rainyday-tracking')
clear classes; clear all
addpath('scripts');
```

The `clear classes` is not optional — MATLAB caches classdefs in `+trackbench`
aggressively and will otherwise run stale code.

## 1. Programmatic test first (no GUI)

Before touching the UI, run the non-interactive tests:

```matlab
testPathEditor_M1
```

Expected: `Result: 6 PASS / 0 FAIL` (or `5 PASS` if the decoded waypoints
come back as a cell array — that branch is noted as SKIP, not a failure).

If anything fails here, **stop** — the UI build is going to surface the
same problem less cleanly. Report the failure before proceeding.

## 2. Launch the editor

```matlab
pathEditor
```

Verify the window opens with:

- [ ] Title: "Rainy Day — Path Editor (M1)"
- [ ] Left pane: empty map axes with a red triangle at the origin labeled `radar`
- [ ] Right sidebar: Scenario panel with fields (Target name, Default speed,
      Default altitude, RCS, RCS profile), a waypoints counter, Export and Clear
      buttons, and a help line
- [ ] Bottom status bar: "Ready. Click on the map to add a waypoint."

## 3. Click-to-add

- [ ] Left-click somewhere on the map. A numbered yellow marker appears at
      the click location. Status bar reports the new waypoint count and its
      (x, y) in meters.
- [ ] Click four more times at spread-out locations. Blue lines connect the
      waypoints in click order.
- [ ] Waypoint counter in the sidebar reads `5`.
- [ ] Title updates to show the waypoint count and the computed duration.

## 4. Scenario fields

- [ ] Edit **Target name** → the axes title updates on next redraw.
      Confirm the editor strips non-alphanumeric characters (try typing
      `hello world!` → should become `hello_world_`).
- [ ] Change **Default speed** to `450` km/h. The duration shown in the
      title should roughly double (since time = distance / speed).
- [ ] Change **RCS profile** to `stealth`. No visible change in the map;
      the change is persisted for export.
- [ ] **Default altitude** change does NOT retroactively rewrite existing
      waypoint altitudes (M2 will add per-waypoint editing). This is
      intentional — verify that exported altitudes match the altitude in
      effect *when each waypoint was added*.

## 5. Export

- [ ] Click **Export JSON**.
- [ ] A green success dialog appears: *"Wrote waypoint file: ... / Reference
      it in a run file as: 'waypoints/<name>'"*.
- [ ] The file exists at `config/targets/waypoints/<target_name>.json`.
- [ ] Open the file. Top-level fields: `description`, `duration_s`, `targets`.
      The `targets` array has one entry with `behavior: "waypoints"`,
      correct `name`/`label`, `rcs_dbsm`, `rcs_profile`, and an array of
      waypoints with `pos: [x, y, z]` and `time_s`.
- [ ] Every `pos[2]` (Z, the third component) is **negative** — altitude
      below ground in NED convention.
- [ ] `time_s` values are strictly increasing, starting at 0.

## 6. End-to-end: run the simulation

Create `config/runs/m1_test.json` by copying `config/runs/run_template.json`
and changing one line:

```json
  "targets": "waypoints/<your_target_name>",
```

Then in MATLAB:

```matlab
runSingleScenario("m1_test")
```

- [ ] Scenario runs without errors from `addTargetFromDef`.
- [ ] The visualization shows the target following the clicked path at the
      configured altitude and speed.
- [ ] Log line `Target 1: <label> (<N> waypoints, <duration>s)` appears.

If `addTargetFromDef:nonMonotonicTime` or `:tooFewWaypoints` appears, the
editor lost a waypoint or had a divide-by-zero speed — re-run
`testPathEditor_M1` and report which assertion failed.

## 7. Edge cases (worth a quick pass)

- [ ] Export with 0 waypoints: Export button produces an alert "Need at
      least 2 waypoints", does NOT write a file.
- [ ] Export with exactly 1 waypoint: same behavior as 0.
- [ ] Export with 2 waypoints: succeeds, writes valid JSON that
      `runSingleScenario` accepts.
- [ ] **Clear all waypoints** button: map empties, counter returns to 0,
      title resets to "click on map to add waypoints".
- [ ] Close the window with unsaved waypoints: confirmation dialog prompts
      before closing.

## What M1 does NOT do (save feedback for later milestones)

- No waypoint selection, drag, delete, or right-click menu (M2).
- No undo/redo (M2).
- No load existing `waypoints/*.json` for editing (M2).
- No 3D view, altitude color coding, or scale bar (M3).
- No animation preview (M3).
- No spline smoothing or kinematic warnings (M4).
- No NASA flight overlay (M5).

Flag anything else that surprises you — ambiguity in the spec is easier to
resolve now than mid-M3.
