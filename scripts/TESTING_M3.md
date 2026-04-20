# Path Editor — Milestone 3 Manual Testing Checklist (CHECK-IN 1)

M3 is being built in **two check-in windows** (per `PROGRESS_M2_FINAL_M3_START.md`
§7). This checklist covers the first window: **sub-tasks 3.1 altitude color,
3.2 scale bar + grid dropdown, 3.3 2D/3D view toggle**. Sub-tasks 3.4
(animation preview) and 3.5 (editable radar marker) are gated behind
your sign-off, so their checklist lives in a separate file that will
appear alongside CHECK-IN 2.

> Before starting: `clear classes; clear all` to flush cached classdefs,
> then `pathEditor` from the project root.

---

## 0. Programmatic tests (run first)

```matlab
addpath('scripts');
testPathEditor_M1          % regression — 6 PASS / 0 FAIL
testPathEditor_M2          % regression — 10 PASS / 0 FAIL
testPathEditor_M3          % CHECK-IN 1 — 11 PASS / 0 FAIL
testPathEditor_shortcuts   % shortcuts  — 13 PASS / 0 FAIL (see §7)
```

> **Patch C note.** Test 11 briefly opens a hidden uifigure to verify
> `ax.ButtonDownFcn` survives `cla(ax,'reset')` — expect a window to
> flash open and close. This is the first M3 test that touches a real
> figure and was added because the walk-through caught a regression
> where left-click-to-add and drag / reselect stopped working after
> the M3 drawMap rewrite. Section 6 ("Click-to-add still works in 2D",
> "Drag-to-move still single-undo per drag") is the manual counterpart
> — exercise those first.

If any test fails, stop — do not bother with manual checks until the suite
is green.

---

## 1. Launch + layout

- [ ] Figure title reads "Rainy Day — Path Editor (M3)"
- [ ] Scenario panel is visibly taller than M2 (now has 10 rows instead of 7)
- [ ] Scenario panel now shows, in order:
      default speed, default altitude, target name, RCS dBsm, RCS profile,
      **Color by altitude** checkbox, **Grid spacing** dropdown, **View
      mode** toggle button, **Apply default altitude to all** button
- [ ] Nothing in the sidebar is clipped or overlapping at the default
      window size

## 2. Altitude colormap (M3.1)

- [ ] Add 4+ waypoints; set several to distinct altitudes (e.g. 1000,
      3000, 7000, 12000 m) via the "Selected waypoint" panel
- [ ] Check the **Color by altitude** checkbox
      - [ ] Waypoint markers recolor on a parula ramp (deep blue = low,
            yellow = high)
      - [ ] A **colorbar** appears on the right side of the axes, labeled
            "Altitude (m)"
      - [ ] Colorbar range spans the min→max altitude of the current path
            (not a fixed 0–12 km)
      - [ ] Path line between markers is dimmed/de-emphasized so the
            colored dots are what you notice first
- [ ] Uncheck the checkbox
      - [ ] Colorbar disappears (not just blanked — actually removed)
      - [ ] Markers revert to plain blue
- [ ] **Hover tooltip** (Patch A): with the checkbox on OR off,
      moving the mouse over the map (not dragging) continuously
      updates the status bar with the cursor's east/north, e.g.
      `x=2400 m, y=-500 m`
      - [ ] When within the zoom-aware hit radius of a waypoint the
            status bar appends
            `(near wp #N: alt=... m, speed=... km/h, t=... s)`
      - [ ] Moving the cursor OFF the axes (into the sidebar) stops
            the update — no stale "near wp #..." message appears
            while editing sidebar fields
- [ ] Underscored target names (e.g. `boeing_747`) still render literally
      in the axes title — no TeX subscripts

## 3. Grid + scale bar (M3.2)

- [ ] **Scale bar** is visible in the bottom-left of the map
      - [ ] Has a filled black/white bar and a text label like `5 km` or
            `500 m`
      - [ ] Scrolling the zoom (mouse wheel / zoom tool) makes the label
            update through "nice" values: 100 m → 200 m → 500 m → 1 km
            → 2 km → 5 km → 10 km, never a weird number like 347 m
      - [ ] Scale bar never overlaps the path in the corner; it sits on
            a light background box and stays readable
- [ ] **Grid spacing dropdown**
      - [ ] Options are: Off, 1 km, 5 km (default), 10 km
      - [ ] Selecting **Off** removes minor gridlines; the axis box
            remains visible
      - [ ] Selecting **1 km** adds a dense minor grid; **10 km** a
            sparse one
      - [ ] Grid is visibly behind the path (gray, not black)

## 4. 2D / 3D view toggle (M3.3)

- [ ] **View mode** button reads "Switch to 3D" initially
- [ ] Click it → map flips to an oblique 3D perspective
      - [ ] Waypoints appear as markers at their altitude above a ground
            plane, with vertical stems dropping to z=0
      - [ ] Altitude axis is positive-up (12 km waypoint is above a 3 km
            waypoint)
      - [ ] Z-exaggeration is reasonable — altitude is clearly visible
            without crushing the horizontal layout
      - [ ] Path line in 3D connects waypoints at their actual altitudes
      - [ ] Button text becomes "Switch to 2D"
      - [ ] Axes title appends `[3D view-only]`
      - [ ] Status bar shows a brief note like "Switched to 3D view (view-only)"
- [ ] In 3D, the default MATLAB figure-toolbar **rotate/pan/zoom**
      interactions work (drag to rotate the camera)
- [ ] **Camera persistence** (Patch B): rotate the 3D view to a
      non-default angle, then:
      - [ ] Click a waypoint to select it → camera stays put (no
            snap back to default azimuth/elevation)
      - [ ] Toggle "Color by altitude" → camera stays put
      - [ ] Change grid spacing → camera stays put
      - [ ] Edit the selected waypoint's altitude in the sidebar →
            camera stays put
- [ ] Switch to 2D and back to 3D → camera autofits to the current
      scene (mode switch is allowed to reset the camera)
- [ ] In 3D, **left-click on a waypoint** still selects it (sidebar
      populates), but **left-click on empty space does NOT add a waypoint**
      - [ ] Status bar says something like "Switch to 2D to add or
            insert waypoints (3D is view-only)"
- [ ] In 3D, dragging a selected marker does nothing (drag is disabled)
- [ ] In 3D, **right-click → Insert waypoint here** is also disabled or
      shows the same "switch to 2D" message
- [ ] Click "Switch to 2D" → returns to top-down view with no rotation
      artifacts
      - [ ] Scale bar + grid reappear correctly
      - [ ] Colorbar (if color-by-altitude was on) reappears correctly

## 5. Interaction between features

- [ ] Turn ON color-by-altitude in 2D, switch to 3D → coloring carries
      over (markers still colored by altitude in 3D, colorbar still
      present)
- [ ] Switch to 3D, switch back to 2D → grid + scale bar return (they
      were hidden in 3D)
- [ ] Change grid spacing while in 3D → no crash, no visible grid change
      (grid is 2D-only); switching back to 2D shows the new spacing
- [ ] Load an existing JSON file → view defaults to 2D, color-by-altitude
      defaults to OFF (fresh session behavior)

## 6. M1 + M2 regression

- [ ] Click-to-add still works in 2D
- [ ] Undo / redo still works (Ctrl+Z, Ctrl+Y)
- [ ] Drag-to-move still single-undo per drag
- [ ] Delete selected, insert-on-segment, Esc-to-deselect all still work
- [ ] Apply-default-altitude bulk updates still work and are undoable
- [ ] Export → load round-trip produces identical waypoints

---

## 7. Keyboard + mouse shortcuts (shortcuts pass, 2026-04-16)

The shortcuts pass landed after CHECK-IN 1 sign-off and before M3.4.
Run `testPathEditor_shortcuts` first — expect 13 PASS / 0 FAIL. Manual
verification below.

Before exercising this section, place 3+ waypoints on the map so there
is something to nudge / toggle / zoom on.

### 7.1 V key — toggle 2D / 3D

- [ ] With nothing focused in the sidebar, press **V** → map flips to 3D
      (view-mode button label also flips to "Switch to 2D")
- [ ] Press **V** again → returns to 2D
- [ ] Click into the "Target name" field, type a letter, press **V** →
      the letter `v` appears in the field, view mode does NOT change
      (focus guard)
- [ ] Tab / click out of the name field, press **V** → view toggles again
- [ ] Ctrl+V and Shift+V are intentionally no-ops (reserved for future
      paste / batch-toggle bindings)

### 7.2 Arrow-key nudge on selected waypoint

- [ ] Select a waypoint (click it). Press **→** → waypoint moves +100 m
      east; status bar reflects the new position
- [ ] Press **↑** / **←** / **↓** → waypoint moves ±100 m in the
      expected direction
- [ ] Each arrow keystroke is exactly **one** undo step — press Ctrl+Z
      N times after N arrow presses and the waypoint returns to its
      original (x, y)
- [ ] **Shift + arrow** moves 1000 m (1 km) per keystroke — same
      direction mapping, same "one undo per keystroke"
- [ ] With NO waypoint selected, arrow keys do nothing and the status
      bar says `No waypoint selected — arrow keys need a selection.`;
      the undo stack does NOT grow
- [ ] Arrow keys while typing in a sidebar numeric field move the caret
      inside the field (not the waypoint) — MATLAB consumes them before
      the figure-level handler sees them

### 7.3 PageUp / PageDown — altitude of selected waypoint

- [ ] Select a waypoint with altitude ≥ 100 m. **PageUp** → altitude
      increases by 100 m; altitude field in the sidebar updates; path
      redraws (colormap, if on, re-ramps)
- [ ] **PageDown** → altitude decreases by 100 m
- [ ] PageDown far enough that altitude would go below 0 → clamps at
      exactly 0 m (never negative); repeating PageDown at 0 stays at 0
- [ ] **Shift + PageUp / PageDown** moves ±1000 m per keystroke, same
      clamp at 0 behavior
- [ ] Each keystroke is one undo step (Ctrl+Z reverses in the expected
      granularity)

### 7.4 Mouse-wheel zoom (2D)

- [ ] Move the cursor to an identifiable feature (a specific waypoint
      or a corner of the path). Scroll the wheel **down** once → view
      zooms out ~1.2× around that cursor point; the feature stays under
      the cursor (cursor anchor, not figure center)
- [ ] Scroll **up** → zooms in ~1/1.2× around the cursor; feature stays
      pinned
- [ ] Repeated scrolls compound smoothly (no visual stutter, no jump to
      a default view)
- [ ] Scale bar label updates to a "nice" value on the 1 / 2 / 5
      ladder as the zoom changes (100 m / 500 m / 1 km / 5 km etc.) —
      same §3 contract
- [ ] Zoom-out limit: scrolling down many times leaves the view
      readable; no axes collapse or NaN limits
- [ ] Zoom still works after a 2D redraw (e.g. toggle color-by-altitude
      then scroll — limits are preserved, scroll anchors still correctly)

### 7.5 Middle-click drag pan (2D)

- [ ] Middle-click and **hold** on an empty area of the map, drag, and
      release → the map pans: the world point under the cursor when you
      pressed stays under the cursor as you drag
- [ ] Scale bar stays visible and updates on release (or mid-pan — both
      are acceptable)
- [ ] Releasing the middle button anywhere (on or off the axes) ends
      the pan cleanly; no stuck "still panning" state
- [ ] Start a middle-click pan, then switch to 3D mid-drag → pan state
      clears, no ghost panning after mode switch

### 7.6 Mouse-wheel + rotate in 3D

- [ ] Switch to 3D. Scroll the wheel → camera zooms in/out (via
      `CameraViewAngle`), not XLim/YLim — scene stays framed, altitude
      perspective feels natural
- [ ] Middle-click-drag in 3D does NOT invoke the 2D pan path (the 2D
      pan is explicitly gated off in 3D); MATLAB's built-in figure
      interactions still work via the toolbar rotate/pan/zoom buttons

### 7.7 Universal bindings still live under the new dispatcher

- [ ] **Delete** / Backspace still removes the selected waypoint
- [ ] **Escape** still clears the selection
- [ ] **Ctrl+Z** / **Ctrl+Y** still undo / redo (Cmd+Z / Cmd+Y also
      works on macOS — tested via modifier alias)
- [ ] These four bindings work **regardless of focus** — even while the
      name / altitude field is focused, Delete still edits the field
      text (MATLAB intercepts that) and Escape / Ctrl-Z still route to
      the figure handler (no interference observed)

### 7.8 Help-panel cheat sheet

- [ ] The Help panel in the bottom-left of the sidebar lists the new
      shortcuts (V, arrows, PageUp / PageDown, wheel zoom) alongside
      the existing M1 / M2 bindings
- [ ] Nothing in the Help panel is clipped at the default window size

---

## Things that may intentionally NOT work yet (coming in CHECK-IN 2)

- Editable radar site marker fields — state exists, no UI wired up yet

If you find bugs, please jot them in `CHECK_IN_1_NOTES.md` in the repo
root (or Slack them my way) and I will address them before touching
3.4 / 3.5.

---

## §8 · M3.4 Animation Preview — Manual Checklist

**Prereq:** M3.1–M3.3 already signed off; CHECK-IN 1 + shortcuts pass
green; M3.4 integration applied (44 / 44 automated tests).

### 8.1 Launch

- [ ] Start MATLAB at the project root and run
      `addpath(genpath('src')); addpath('scripts')`.
- [ ] Launch the editor: `pathEditor`.
- [ ] Scenario panel (right sidebar) shows a new **Preview Animation**
      button at the bottom (light-blue fill, bold text). Panel is
      still scrollable — bottom row visible without clipping.
- [ ] Load any saved path with ≥ 2 waypoints, or click 3–4 waypoints
      fresh.

### 8.2 Zero / one waypoint guard

- [ ] Brand-new editor with no waypoints: click **Preview Animation**.
- [ ] Secondary window opens showing the message
      *"Add at least 2 waypoints in the editor to preview playback."*
- [ ] No MATLAB error in the command window.
- [ ] `timerfindall('Tag','RainyDayPathPreviewTimer')` returns empty.
- [ ] Close the preview window — main editor unaffected.
- [ ] Repeat with exactly one waypoint placed. Same guard message.

### 8.3 Happy path — play / pause / restart

- [ ] With a 3+ waypoint path, click **Preview Animation**.
- [ ] Preview window shows the full blue polyline, orange radar star
      (at the radar position), and a red marker at the first waypoint.
- [ ] Click **Play**: marker advances smoothly along the path; `t = ...`
      status label increments; button text flips to **Pause**.
- [ ] Click **Pause**: marker freezes mid-segment; button text flips
      back to **Play**.
- [ ] Click **Play** again: resumes from the paused time (not start).
- [ ] Let playback run to the end: marker stops at the last waypoint,
      button text is **Play**, status label shows `t = t_end / t_end`.
- [ ] Click **Play** once more with marker at end: auto-rewinds to
      `t_start` and plays from there.
- [ ] Click **Restart** mid-playback: marker snaps back to waypoint 1,
      timer stops, button text is **Play**.

### 8.4 Speed dropdown

- [ ] With playback running at **1x**, change to **2x** — marker roughly
      doubles in speed; no stutter or flicker.
- [ ] Change to **0.5x** — marker noticeably slower.
- [ ] Change to **5x** — marker visibly faster than 2x; still smooth.
- [ ] Change to **0.5x → Pause → 5x → Play** — new speed applies on
      resume (no stale period).

### 8.5 Close-safety

- [ ] Start playback at 2x. Close the preview window with the X.
      Main editor remains fully responsive.
- [ ] Run `timerfindall('Tag','RainyDayPathPreviewTimer')` — empty.
- [ ] Open preview again, click **Play**, then close the **main**
      editor window. Both windows disappear together. No leftover
      figures, no leftover timers.
- [ ] Open preview, leave on **Pause**, close main editor. Same result.

### 8.6 View-only guarantee

- [ ] Try to click on the polyline in the preview window — nothing
      happens (`PickableParts='none'` on every graphics object).
- [ ] No right-click context menu on waypoints inside the preview.
- [ ] Resizing the preview window is smooth; axes rescale without
      warping the marker trajectory.

### 8.7 Interaction with the main editor

- [ ] Open preview. Without closing it, drag a waypoint in the main
      editor. Click **Preview Animation** again — the existing window
      raises (does NOT spawn a second one) and still shows the OLD
      path. Close it and re-open to see the new path. This matches
      the spec's view-only / open-time-snapshot rule; no hot-reload
      is required in M3.4.
- [ ] Open preview, click **Play**, then switch focus back to the main
      editor and add a waypoint via click. Main editor stays
      responsive (confirms the `timer` object didn't block the UI
      thread — no `while` loop).

### 8.8 Automated tests

- [ ] Run `testPathEditor_M3_preview` — all 4 cases PASS:
  - `timer_created_and_cleaned_on_close`
  - `interpolation_linear_and_exact_at_waypoint`
  - `speed_factor_inverse_on_timer_period`
  - `zero_and_one_waypoint_dont_crash`
- [ ] `timerfindall('Tag','RainyDayPathPreviewTimer')` empty after
      tests.

### 8.9 Out of scope for M3.4 — do **not** test here

The following belong to M3.5 or M4 and are explicitly not part of
the M3.4 gate:

- Editable radar marker in the main editor  → M3.5
- 3-D preview with altitude-coloured marker  → M3.5 / M4
- Spline-smoothed path preview  → M4
- NASA flight overlay inside the preview  → M5
- Hot-reload of the preview when waypoints change mid-playback  → not planned

---

**Sign-off:** Once all boxes above are checked, record the result in
`PROGRESS_M3.md` (§M3.4 pass) and **park the session** for CHECK-IN 2.
M3.5 (editable radar marker) starts in a separate fresh session — do
not roll it into the same branch without a CHECK-IN 2 approval first.
