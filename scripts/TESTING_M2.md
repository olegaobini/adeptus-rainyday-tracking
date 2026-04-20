# Path Editor — Milestone 2 Manual Testing Checklist

M2 adds selection, drag-to-move, delete, insert-on-segment, undo/redo,
load-from-JSON, and a per-waypoint properties panel. The GUI surface is
much larger than M1, so please actually exercise every item below and
flag anything that feels off.

> Before starting: `clear classes; clear all` in MATLAB to flush any
> cached classdefs from the M1 session, then `pathEditor` from the
> project root.

---

## 0. Programmatic tests (run first)

```matlab
addpath('scripts');
testPathEditor_M1   % regression — must still pass
testPathEditor_M2   % new
```

Expected: `10 PASS / 0 FAIL` on M2, `6 PASS / 0 FAIL` on M1 carry-over.

---

## 1. Launch + layout

- [ ] `pathEditor` opens a window titled "Rainy Day — Path Editor (M2)"
- [ ] Sidebar shows three sub-panels: **Scenario**, **Selected waypoint**,
      **File**, plus **History** and **Help** rows
- [ ] Help text wraps cleanly at narrow sidebar widths (no horizontal cut-off)
- [ ] "Selected waypoint" fields start disabled (gray) with "Index: —"

## 2. Adding waypoints (M1 regression)

- [ ] Left-click empty map → marker appears, numbered sequentially
- [ ] Waypoint count in sidebar updates
- [ ] Underscores in target names (try `boeing_747`) render literally in
      the axes title — NOT as TeX subscripts

## 3. Selection

- [ ] Left-click within ~500 m of an existing marker selects it
- [ ] Selected marker shows a green halo ring and larger green dot
- [ ] Sidebar "Selected waypoint" panel populates: index, x, y, alt,
      leg speed, time
- [ ] Pressing **Esc** clears the selection (halo disappears, panel re-disables)
- [ ] Clicking empty space near an existing waypoint (>500 m away) adds a
      new waypoint rather than selecting the far one

## 4. Drag to move

- [ ] Left-click-and-hold on a selected marker, drag around — the waypoint
      follows the cursor in real time
- [ ] Release → status bar reports "Moved waypoint #N"
- [ ] One drag = one undoable action (Ctrl+Z restores the pre-drag position,
      not a halfway-frame)

## 5. Insert-on-segment

- [ ] Shift+click on the line between two waypoints → a new waypoint
      appears at the perpendicular projection point, renumbered in place
- [ ] Shift+click far from any segment → status bar says "missed any path
      segment"; no waypoint added

## 6. Right-click context menu

- [ ] Right-clicking near a waypoint opens a menu with: **Insert waypoint
      here / Delete selected waypoint / Clear selection**
- [ ] "Insert waypoint here" on a segment → inserts at projected point
- [ ] "Insert waypoint here" on empty space with a selection → appends
- [ ] "Delete selected waypoint" removes the current selection
- [ ] "Clear selection" deselects without modifying waypoints

## 7. Delete

- [ ] With a waypoint selected, press **Delete** (or **Backspace**) →
      waypoint disappears, numbering recomputes
- [ ] Sidebar "Delete" button does the same
- [ ] Deleting the last selected waypoint clears the selection panel

## 8. Insert after

- [ ] With a mid-path waypoint selected, click "Insert after" →
      new waypoint appears at the midpoint to the next one
- [ ] With the last waypoint selected, "Insert after" → new waypoint
      1 km east of it

## 9. Per-waypoint editing (sidebar)

- [ ] Change **East x** → marker jumps; path re-renders; times recompute
- [ ] Change **Altitude** → no visible XY change, but time recomputes
      (higher altitude + same speed = slightly longer leg due to the
      Z component of distance)
- [ ] Try setting altitude to a negative number → it clamps to 0
- [ ] Change **Leg speed** → time field on THAT waypoint updates
- [ ] Time field is read-only (cannot be typed into)

## 10. Apply default altitude to all

- [ ] Set some waypoints to mixed altitudes
- [ ] Change **Default altitude (m)** in the Scenario panel — existing
      waypoints are NOT mutated (this is intentional M1 behavior)
- [ ] Click "Apply default altitude to all waypoints" — confirm dialog
      appears; on accept, every waypoint's altitude becomes the new default
- [ ] Ctrl+Z undoes the bulk update in a single step

## 11. Undo / redo

- [ ] Click "Undo" button (or Ctrl+Z) — reverses the most recent action
- [ ] Click "Redo" (or Ctrl+Y) — reapplies
- [ ] A fresh edit after undoing invalidates the redo future (Redo button
      reports "Nothing to redo")
- [ ] Undo works across: add, drag, delete, insert, bulk-altitude, load,
      clear, sidebar field edits
- [ ] Undo stack is capped at 50 entries (no memory blowup after 100
      clicks)

## 12. Load JSON

- [ ] Click "Load JSON…" → file picker opens at `config/targets/waypoints/`
- [ ] Select a file you just exported — map re-populates, sidebar shows
      loaded target name, waypoint count, altitudes, speeds
- [ ] Title bar shows `(loaded from <filename>.json)`
- [ ] Ctrl+Z after load wipes the loaded path and restores the prior state
- [ ] Try a hand-written minimal JSON (missing rcs_profile, integer pos)
      — loads without error

## 13. Export (M1 regression)

- [ ] With ≥2 waypoints, "Export JSON" writes to
      `config/targets/waypoints/<targetName>.json`
- [ ] Open the file in a text editor — `speed_kmh` now appears on each
      waypoint (new in M2; `addTargetFromDef` ignores it)
- [ ] Run the exported file end-to-end with `runTracker('runs/<yours>')`
      — should still produce a valid `waypointTrajectory`

## 14. Clear + safety

- [ ] "Clear all waypoints" prompts for confirmation
- [ ] Clearing resets the map and disables the selection panel
- [ ] Ctrl+Z restores the cleared waypoints

## 15. Close with unsaved

- [ ] Add waypoints without exporting → close the window → prompt warns
      about unsaved changes
- [ ] Clicking "Cancel" keeps the window open

---

## Known M2 scope limits (deferred — NOT bugs)

- **No 3D view.** `viewMode` property exists but isn't wired up until M3.
- **No spline preview.** Lines are always straight; smoothing ships in M4.
- **No manual timing mode.** `timingMode == "auto"` everywhere — manual
  per-waypoint time entry is a future feature.
- **No multi-select.** Exactly one waypoint selected at a time.
- **No NASA flight overlay.** Ships in M5.

---

## What to report back

- Anything that looks broken visually (stacking order, truncated labels,
  ghost halos after undo)
- Any action that should be undoable but isn't
- Any case where the sidebar becomes out-of-sync with the map
- Cases where the status bar text is misleading or empty when it
  shouldn't be
- Edge cases around Shift+click near waypoint endpoints (the hit-test
  biases toward the waypoint, but report if this feels wrong)
