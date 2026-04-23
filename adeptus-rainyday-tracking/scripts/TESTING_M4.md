# Path Editor — Milestone 4 Manual Testing Checklist

M4 is the Catmull-Rom spline curve milestone (spec: `PROGRESS_M3_FINAL_M4_START.md` §3).
Everything has been implemented and programmatic tests pass; this checklist
exercises the interactive behavior (tension dropdown, live redraw, preview
animation, 3D curved view) that the test suite cannot assert.

> Before starting: `clear classes; clear all` to flush cached classdefs, then
> `pathEditor` from the project root.

---

## 0. Programmatic tests (run first)

```matlab
addpath('scripts');
testPathEditor_M1          % regression — expect all PASS
testPathEditor_M2          % regression — expect all PASS
testPathEditor_M3          % regression — expect all PASS
testPathEditor_shortcuts   % regression — expect all PASS
testPathEditor_M4          % M4 suite   — expect 20/20 PASS
```

The M4 suite covers: Catmull-Rom math across α ∈ {0.0, 0.5, 1.0}, endpoint
phantom handling, straight↔curved mode switching, JSON round-trip in both
modes, M3-era backward compat (no `curve_mode` field → loads as straight),
dense-waypoint export (5 controls → 201 dense), and preview-timeline sanity.

If any test fails, stop — do not bother with manual checks until the suite
is green.

---

## 1. Launch + layout (new row)

- [ ] Figure title reads `Rainy Day — Path Editor (M4)` (or the current
      milestone tag).
- [ ] Scenario panel now shows, in order:
      default speed, default altitude, target name, RCS dBsm, RCS profile,
      **Color by altitude**, **Grid spacing**, **View mode**, **Curve mode**,
      **Curve tension**, **Apply default altitude to all**, Preview button.
- [ ] Nothing in the sidebar is clipped at the default window size. The
      scenario panel grew by one row (≈28 px) compared to M3.

## 2. Straight mode (M3 regression)

- [ ] Add 4+ waypoints by clicking the map. Path draws as straight legs.
- [ ] Change **Curve tension** — the dropdown value updates but the
      rendered path does **not** change (straight mode ignores α).
- [ ] Drag a waypoint. Single Ctrl+Z restores it (no per-redraw spam).
- [ ] Export → JSON has `"curve_mode": "straight"` and **no**
      `control_waypoints` / `curve_tension_alpha` keys (open the JSON and
      confirm).

## 3. Curved mode — centripetal (M4.3.1 + 3.2)

- [ ] Switch **Curve mode** to `Curved`. Status bar acknowledges the
      change and states the current tension (e.g.
      `Curved mode on (Centripetal, α=0.5)`).
- [ ] The rendered path becomes a smooth spline passing through every
      waypoint exactly — the polyline corners at each control round off.
- [ ] Add another waypoint by clicking. The curve re-fits live; no console
      errors.
- [ ] Drag a waypoint. The curve re-fits continuously as you drag (not
      just on mouse-up). Release → single undo snapshot (Ctrl+Z restores
      the pre-drag position in one keypress).
- [ ] Select a waypoint. The selection ring snaps to the **control
      point**, not the nearest spline sample (proves selection still
      operates on `state.waypoints`, not on the dense list).

## 4. Tension dropdown (M4.3.3)

With 5+ waypoints in curved mode:

- [ ] Switch tension to **Uniform** (α=0.0).
      - Corners tighten; near-coincident controls can produce small loops /
        overshoot — this is expected Uniform behavior and is why 0.5 is the
        default.
      - Status bar shows `Curve tension: Uniform (α=0.0)`.
- [ ] Switch to **Centripetal** (α=0.5). Curve relaxes to the
      "no cusps, no loops" shape.
- [ ] Switch to **Chordal** (α=1.0). Curve sags toward the straight-line
      connecting edges — closer to piecewise-linear than centripetal.
- [ ] Switching tension pushes an undo snapshot: Ctrl+Z restores the
      previous tension (and visually, the previous curve shape).

## 5. 3D view with curved mode (M3.3 interaction)

- [ ] Switch to 3D view while in curved mode. The spline renders in 3D —
      it passes through every waypoint at its actual altitude, with
      vertical stems from each control down to z=0 (same as M3).
- [ ] Rotate the 3D camera; the curve re-renders on the new viewpoint
      without resetting to the default azimuth.
- [ ] Flip curve mode Curved ↔ Straight while in 3D. The curve morphs
      between smooth and piecewise-linear; the camera stays put.
- [ ] Toggle **Color by altitude**. Control-point markers recolor on the
      parula ramp; the spline polyline itself is unaffected (this is by
      design — the markers are what the user is editing).

## 6. Animation preview (M4.3.5)

- [ ] Click **Preview** with curved mode on. A second window opens
      showing a marker animating along the dense Catmull-Rom curve — it
      should visibly sweep through rounded corners rather than hopping
      from control to control.
- [ ] Click Preview with straight mode on. The marker snaps between
      control points along straight segments (original M3 behavior).
- [ ] Preview window Play/Pause/Reset still work; closing the window
      does not crash the editor.
- [ ] The preview's marker position at t = waypoint_N.time_s lies visibly
      on top of control waypoint N (confirms the dense curve preserves
      timing — see automated test #19 for the numeric proof).

## 7. JSON round-trip (M4.3.4)

- [ ] Export a curved-mode file (5+ waypoints, tension = Uniform).
      Open the JSON by hand. Confirm:
      - [ ] `"curve_mode": "curved"`
      - [ ] `"curve_tension_alpha": 0`
      - [ ] `"control_waypoints": [ ... 5 entries ... ]`
      - [ ] `"waypoints": [ ... (5-1)*50 + 1 = 201 entries ... ]` — the
            dense simulator-consumable list.
      - [ ] Every waypoint `pos[2]` is negative (NED convention; UI shows
            positive altitude but on-disk z is negative-down).
- [ ] Reopen the same file via the editor's Load. The sidebar repopulates
      with **5 control points**, not 201 dense ones. Curve mode is set
      to `Curved` and the tension dropdown reads `Uniform`.
- [ ] Hand-edit a JSON to remove the `curve_mode` and `control_waypoints`
      keys (simulating an M3-era file). Reload → opens as straight mode,
      tension defaults to Centripetal (α=0.5). No errors in console.

## 8. End-to-end simulator consumption (M4.3.5, critical)

This is the check Michael specifically asked for: the exported curved
JSON must be consumable by `runSingleScenario` and the simulated target
must fly a smooth path — not piecewise-linear.

```matlab
cd <project root>
addpath('scripts');
verifyM4_endToEnd        % builds m4_curved_demo target + run file
                         % then calls runSingleScenario
```

Expected behavior:
- Target JSON is written to `config/targets/waypoints/m4_curved_demo.json`.
- Run file is written to `config/runs/m4_curved_demo.json`.
- Round-trip assertions pass (5 controls restored, curveMode=curved,
  α=0.5).
- `runSingleScenario("m4_curved_demo")` runs to completion. The 3D
  truth-and-detections figure shows the target flying a smooth S-curve
  with a vertical arc in altitude — not a zig-zag between 5 straight
  legs.
- Screenshots are saved under
  `After Presentation/PROGRESS_M4_screenshots/`. The reference captures
  from the first successful run are:
  - `m4_curved_02_Scenario_Truth_and_Detections_3D.png` — the 3D truth
    trajectory from `runSingleScenario`, showing the smooth flight path
    with detection returns inside the PSR coverage cone.
  - `m4_curved_03_BirdsEye_ControlsVsDense.png` — bird's-eye comparison
    of the 5 control points + linear connection vs. the 201-point
    Catmull-Rom dense curve. This is the clearest proof that the export
    densified correctly.
  - `m4_curved_04_AltitudeProfile.png` — altitude-over-time with the
    control kinks (dashed) vs. the smooth Catmull-Rom interpolation
    (solid).

Note: the full sim run can take 60–120 s depending on machine speed —
roughly the target's 211 s flight time divided by MATLAB's real-time
factor, minus rendering overhead. Expect the command window to be busy
for a minute or so before the figure appears. If it appears to hang,
check Task Manager — MATLAB is almost certainly still working.

If you need to regenerate just the static control-vs-dense plots
without running the full simulator (much faster), run:

```matlab
verifyM4_renderTruth     % ~2 s, produces the 03/04 PNGs only
```

## 9. M1 + M2 + M3 regression

Everything that worked before must still work. Sanity pass:

- [ ] Click-to-add waypoint works in 2D (both modes).
- [ ] Undo / redo still works (Ctrl+Z, Ctrl+Y).
- [ ] Drag-to-move is one undo-per-drag in both modes.
- [ ] Delete selected, insert-on-segment, Esc-to-deselect all still work.
- [ ] Apply-default-altitude bulk updates still work and are undoable.
- [ ] Export → load round-trip produces identical waypoints in straight
      mode (see automated test #18), and identical *control points* in
      curved mode (see automated test #16).
- [ ] `testPathEditor_shortcuts` still passes end-to-end.

---

## Known cosmetic items (not blockers)

- Tension dropdown shows raw α values in the status bar as well as the
  preset name — this is intentional, for users who want to verify the
  math behind the label.
- In Uniform mode with near-coincident control waypoints, small loops can
  appear in the curve. This is a property of uniform Catmull-Rom, not a
  bug. Centripetal (the default) does not have this issue.
- The preview window's timer granularity matches M3's — about 25 FPS. The
  dense curve has 201 samples over ≈200 s of flight, so the marker moves
  smoothly at that frame rate.

---

## Sign-off

If every box in sections 1–9 is ticked and `verifyM4_endToEnd` produced
the three reference screenshots, M4 is complete.
