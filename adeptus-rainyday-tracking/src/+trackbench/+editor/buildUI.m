function buildUI(state)
%buildUI  Create the uifigure + controls for the interactive path editor.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Lays out:
%    [  map axes (uiaxes)            |  sidebar  ]
%    [  status bar                                ]
%
%  Sidebar (Milestone 2):
%    - Scenario: target name, default speed, default altitude, RCS, profile
%    - "Apply default altitude to all" button (user feedback from M1)
%    - Selection panel: waypoint #, x, y, altitude, leg speed, time (readonly)
%    - Delete + Insert-after buttons on the selection
%    - File row: Load JSON, Export JSON, Clear
%    - Undo / Redo buttons
%    - Help text (wrapped for narrow windows)
%
%  INTERACTIONS (M2, per spec §4.1 / §6.3)
%    - Left-click empty space   : add waypoint
%    - Left-click near waypoint : select
%    - Left-click + drag marker : move waypoint (live preview, one undo step)
%    - Shift+click on a segment : insert waypoint at projection point
%    - Right-click              : context menu (Insert after / Delete)
%    - Delete / Backspace key   : delete selected
%    - Ctrl+Z / Ctrl+Y          : undo / redo
%    - Escape                   : clear selection
%
%  MATLAB GOTCHA
%    uiaxes click callbacks use ButtonDownFcn on the uiaxes itself; we read
%    the click location from evt.IntersectionPoint. Children's hit-test
%    would steal clicks from the empty map, so drawMap does NOT rely on
%    per-child callbacks. Drag motion is routed through the figure-level
%    WindowButtonMotionFcn/WindowButtonUpFcn so a brief excursion off the
%    marker while dragging doesn't cancel the gesture.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.drawMap,
%            trackbench.editor.exportToJSON, trackbench.editor.loadFromJSON

    arguments
        state (1,1) trackbench.editor.EditorState
    end

    % ── Root figure ─────────────────────────────────────────────────
    % WindowScrollWheelFcn is the shortcuts-pass addition — it routes
    % mousewheel events to onScrollWheel, which zooms around the cursor
    % in 2D (updating XLim/YLim) or modifies CameraViewAngle in 3D.
    fig = uifigure('Name', 'Rainy Day — Scenario Editor (M6)', ...
                   'Position', [100 80 1200 860], ...
                   'CloseRequestFcn', @(src, ~) onClose(src, state), ...
                   'KeyPressFcn',     @(src, evt) onKeyPress(src, evt, state), ...
                   'WindowButtonMotionFcn', @(src, ~) onMouseMove(src, state), ...
                   'WindowButtonUpFcn',     @(src, ~) onMouseUp(src, state), ...
                   'WindowScrollWheelFcn',  @(src, evt) onScrollWheel(src, evt, state));
    state.fig = fig;

    % ── Top-level grid: map | sidebar, storm-timeline strip, status bar
    %
    %  M7 §3.3 ROW INSERT — a thin uiaxes strip lives between the main
    %  map and the status bar, showing the storm window against the
    %  scenario duration. The sidebar column spans both the map row and
    %  the timeline row (RowSpan [1 2]) so the sidebar remains contiguous
    %  and the timeline only consumes width below the map.
    outer = uigridlayout(fig, [3 2]);
    outer.RowHeight   = {'1x', 40, 28};
    % Sidebar width bumped 300 → 400 (M6 §3.3 follow-up). The 4-column
    % Sensor Params panel packs label/field/label/field quads per row;
    % at 300 px the two '1x' field cols collapsed to ~19 px each and
    % numeric values weren't readable. 400 px gives each field ~76 px
    % after the fixed label cols (110 + 95), the grid padding (12), and
    % the tightened ColumnSpacing (4 — see buildSensorParamsPanel). Map
    % loses ~100 px but is still ~780 px wide at the default window size.
    outer.ColumnWidth = {'1x', 400};
    outer.Padding     = [8 8 8 8];
    outer.RowSpacing  = 6;
    outer.ColumnSpacing = 8;

    % ── Map axes ────────────────────────────────────────────────────
    ax = uiaxes(outer);
    ax.Layout.Row = 1;
    ax.Layout.Column = 1;
    ax.Box = 'on';
    disableDefaultInteractivity(ax);
    ax.Toolbar.Visible = 'on';
    ax.ButtonDownFcn = @(src, evt) onAxesClick(src, evt, state);
    % Patch C: stash the handle so drawMap can restore it after
    % cla(ax,'reset'), which otherwise wipes ButtonDownFcn back to ''.
    state.axesClickFcn = ax.ButtonDownFcn;
    state.ax = ax;

    % Right-click context menu on the axes (M2).
    cm = uicontextmenu(fig);
    uimenu(cm, 'Text', 'Insert waypoint here', ...
        'MenuSelectedFcn', @(~, ~) onContextInsertHere(state));
    uimenu(cm, 'Text', 'Delete selected waypoint', ...
        'MenuSelectedFcn', @(~, ~) onContextDelete(state));
    uimenu(cm, 'Text', 'Clear selection', ...
        'MenuSelectedFcn', @(~, ~) onContextClearSelection(state));
    ax.ContextMenu = cm;

    % ── Sidebar column (scrolling panel) ────────────────────────────
    %  Spans rows [1 2] in the outer grid so it runs full-height from the
    %  top of the map down past the storm-timeline strip.
    side = uipanel(outer, 'Title', 'Editor', 'FontWeight', 'bold', ...
                   'Scrollable', 'on');
    side.Layout.Row = [1 2];
    side.Layout.Column = 2;

    sg = uigridlayout(side, [1 1]);
    sg.RowHeight   = {'1x'};
    sg.ColumnWidth = {'1x'};
    sg.Padding     = [0 0 0 0];
    sg.RowSpacing  = 0;
    sg.Scrollable  = 'on';

    % 5-row grid with FIXED row heights. Using 'fit' here caused the
    % last two sub-panels (History, Help) to be squeezed out of view
    % and the parent's Scrollable='on' never engaged because the child
    % grid sized itself to the available space. Fixed heights force the
    % total to exceed the panel and the scrollbar appears.
    %
    % Row heights (M3/M4):
    %   Scenario panel grew from 7→11 rows (M3.1 checkbox + M3.2 grid
    %   dropdown + M3.3 view-mode button + M3.4 preview button), so
    %   bumped from 240→320. M4.3.2 adds the Curve: Straight/Smooth
    %   toggle on a new row (12 rows total → 350 px). M4.3.3 adds the
    %   tension dropdown (Uniform/Centripetal/Chordal) on another new
    %   row (13 rows total → 378 px). If this window feels cramped
    %   after M3.5 (radar X/Y add 2 more scenario rows), grow Scenario
    %   to ~420 and/or enlarge the figure.
    inner = uigridlayout(sg, [14 1]);
    % Help panel (last row) stays at 115 from the M5 shortcuts pass.
    %
    % M5 §3.1 ROW INSERT
    %   Row 1 was the Targets sub-panel (dropdown + 4 buttons, ~110 px).
    %
    % M5 §3.2 FILE-PANEL GROWTH
    %   File sub-panel went from 2 button-rows to 3 (added "Load as
    %   Reference…" and "Unload all references"). Row bumped 95→125 px.
    %
    % M6 §3.2 LAYOUT EXPANSION — 9 rows total.
    %   The sidebar now hosts both a multi-sensor workflow and the
    %   existing multi-target workflow. Order from top:
    %     1. Mode toggle   ( 85 px) — Targets / Sensors radio pair
    %     2. Sensors       (110 px) — dropdown + Add/Dup/Del/Rename
    %     3. Sensor Params (378 px) — type-driven fields (Freq, RPM, …)
    %     4. Targets       (110 px) — existing M5 panel
    %     5. Scenario      (378 px) — existing M5 panel
    %     6. Selection     (240 px) — existing M5 panel
    %     7. File          (125 px) — existing M5 panel
    %     8. Undo/Redo     ( 60 px) — existing panel
    %     9. Help          (115 px) — existing panel
    %   applyEditMode() grays/un-grays the Sensors pair (#2/#3) and the
    %   Targets/Scenario/Selection trio (#4/#5/#6) based on state.editMode.
    %   Total height grew ~575 px; parent is Scrollable='on' so the user
    %   just scrolls a little farther on narrow windows.
    %
    % M7 §3.2 LAYOUT EXPANSION — 11 rows total.
    %   Two new sub-panels (Terrain, Weather) inserted between Scenario
    %   and Selection so environment authoring sits between "what's in
    %   the scenario" (targets/sensors/scenario) and "what's selected on
    %   the map" (selection/file). Order from top:
    %     1. Mode toggle   ( 85 px) — Targets / Sensors / Environment (3 buttons)
    %     2. Sensors       (110 px)
    %     3. Sensor Params (378 px)
    %     4. Targets       (110 px)
    %     5. Scenario      (378 px)
    %     6. Terrain       (220 px) — NEW (type, desc, scale, clutter, refr, 4 deg toggles, overlay, load)
    %     7. Weather       (340 px) — NEW (type, desc, rate, storm start/end, profile, pd_floor, clutter_mult, sparkline, load)
    %     8. Selection     (240 px)
    %     9. File          (157 px)
    %    10. Undo/Redo     ( 60 px)
    %    11. Help          (115 px)
    %   applyEditMode() now has three modes — environment mode enables
    %   Terrain + Weather (rows 6/7) and disables the rest.
    %
    %   ROW-3 SIZING NOTE  The Sensor Params panel packs 12 rows (name,
    %   type, east/north, alt/tilt, freq/maxRange, rangeRes, azFov/elFov,
    %   scan banner, rpm, sectorLo/Hi, pd/far, place-on-map button). At
    %   24 px per row it needs ~344 px inside the panel; 378 matches
    %   Scenario and leaves room for panel-title chrome without
    %   clipping the Place-on-map button.
    %
    %   TERRAIN ROW SIZING  220 px holds: type DD + desc + scale/clutter
    %   pair + refraction + 4 degradation checkboxes in 2 cols + Load +
    %   Overlay toggle + inline hint. ~9 rows at 22 px plus panel title.
    %
    %   WEATHER ROW SIZING  340 px holds: type DD + desc + rate (labeled
    %   per-type) + storm start/end pair + profile DD + pd_floor +
    %   clutter_mult (fog/icing disabled) + sparkline (36 px) + Load.
    %   ~12 rows at ~22 px plus 36 px for the sparkline plus panel title.
    %
    %   v3.5 §5c.2 LAYOUT EXPANSION — 14 rows total.
    %     Inserted 3 new rows for the Environment-mode regions UX:
    %       Row 6  (50 px)  : Env sub-mode toggle (Fallback / Regions)
    %       Row 9  (250 px) : Terrain Regions sub-panel
    %       Row 10 (250 px) : Weather Regions sub-panel
    %     The Selection / File / Undo / Help rows shifted from 8/9/10/11
    %     to 11/12/13/14. applyEditMode collapses rows to 0 px based on:
    %       editMode == environment           → row 6 visible
    %       envSubMode == fallback (env on)   → rows 7+8 visible
    %       envSubMode == regions  (env on)   → rows 9+10 visible
    %     The sub-mode toggle is hidden entirely when not in environment
    %     mode (no greyed-out version) — matches existing top-level mode
    %     toggle behavior.
    % v3.5 §5d: Mode Toggle (row 1) grew 85 → 120 px to host the
    % relocated 2D/3D view toggle; Scenario (row 5) shrank 378 → 350 px
    % since that toggle is no longer one of its rows. Net +7 px overall.
    inner.RowHeight   = {120, 145, 378, 230, 350, 50, 280, 340, 250, 250, 240, 65, 60, 115};
    inner.ColumnWidth = {'1x'};
    inner.RowSpacing  = 4;
    inner.Scrollable  = 'on';

    % v3.5 step 4a — capture handles for mode-specific show/hide.
    %  applyEditMode rewrites inner.RowHeight on every mode switch,
    %  zeroing out rows whose panels don't belong to the active mode.
    %  We snapshot the original row-height cell now because there's no
    %  way to recover it later — reading inner.RowHeight after a hide
    %  cycle would yield the rewritten cell with zeros in it. Storing
    %  the cell directly (not a copy) is fine: cell-array assignment
    %  in MATLAB copies-on-write, so future mutations of
    %  inner.RowHeight don't bleed back into our saved snapshot.
    state.editorInnerGrid              = inner;
    state.editorInnerOriginalRowHeights = inner.RowHeight;

    % ── Mode-toggle sub-panel (M6 §3.2, row 1) ──────────────────────
    buildModeTogglePanel(inner, state);

    % ── Sensors sub-panel (M6 §3.2, row 2) ──────────────────────────
    buildSensorsPanel(inner, state);

    % ── Sensor Parameters sub-panel (M6 §3.2, row 3) ────────────────
    buildSensorParamsPanel(inner, state);

    % ── Targets sub-panel (M5 §3.1, row 4) ──────────────────────────
    buildTargetsPanel(inner, state);

    % ── Scenario sub-panel (row 5) ──────────────────────────────────
    buildScenarioPanel(inner, state);

    % ── Env Sub-mode toggle (5c.2, row 6) ─────────────────────────
    buildEnvSubModeTogglePanel(inner, state);

    % ── Terrain sub-panel (M7 §3.2, row 7) ──────────────────────────
    buildTerrainPanel(inner, state);

    % ── Weather sub-panel (M7 §3.2, row 8) ──────────────────────────
    buildWeatherPanel(inner, state);

    % ── Terrain Regions sub-panel (5c.2, row 9) ───────────────────
    buildTerrainRegionsPanel(inner, state);

    % ── Weather Regions sub-panel (5c.2, row 10) ──────────────────
    buildWeatherRegionsPanel(inner, state);

    % ── Selection sub-panel (row 11) ─────────────────────────────────
    buildSelectionPanel(inner, state);

    % ── File sub-panel (row 12) ──────────────────────────────────────
    buildFilePanel(inner, state);

    % ── Undo/redo sub-panel (row 13) ────────────────────────────────
    buildUndoPanel(inner, state);

    % ── Help sub-panel (row 14) ─────────────────────────────────────
    buildHelpPanel(inner);

    % ── Storm-window timeline strip (M7 §3.3) ───────────────────────
    %  Thin uiaxes stretching under the map showing scenario duration
    %  with the storm window highlighted. Visible flips off when
    %  state.weather is empty. PickableParts='none' so the strip never
    %  steals clicks intended for the main map axes.
    stormAx = uiaxes(outer);
    stormAx.Layout.Row = 2;
    stormAx.Layout.Column = 1;
    stormAx.XTick = [];
    stormAx.YTick = [];
    stormAx.Box = 'on';
    stormAx.XColor = [0.55 0.55 0.55];
    stormAx.YColor = [0.55 0.55 0.55];
    stormAx.HitTest = 'off';
    stormAx.PickableParts = 'none';
    disableDefaultInteractivity(stormAx);
    stormAx.Toolbar.Visible = 'off';
    stormAx.Visible = 'off';   % flips on in drawStormTimeline when weather exists
    state.weatherStormTimelineAx = stormAx;

    % ── Status bar ──────────────────────────────────────────────────
    state.statusLabel = uilabel(outer, ...
        'Text', 'Ready. Click to add · drag to move · Shift+click a segment to insert · Del to remove.', ...
        'FontColor', [0.25 0.25 0.25]);
    state.statusLabel.Layout.Row = 3;
    state.statusLabel.Layout.Column = [1 2];

    % ── Initial draw ────────────────────────────────────────────────
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    refreshTargetsDropdown(state);
    % M6 §3.2 — seed the Sensors widgets and apply edit-mode gating
    % (Targets mode is default, so the Sensors panels start disabled).
    refreshSensorsDropdown(state);
    refreshSensorParamsPanel(state);
    % M7 §3.2 — seed the Environment widgets. Terrain always has content
    % (rural default); Weather renders its (none) empty state initially.
    refreshTerrainPanel(state);
    refreshWeatherPanel(state);
    % v3.5 §5c.2 — seed the new Regions sub-panels and the sub-mode
    % toggle so they show correct values even before the user enters
    % Environment mode. (applyEditMode also re-runs these inside its
    % environmentOn branch, but seeding here covers the case where the
    % initial editMode is targets/sensors and the user later flips into
    % environment — the panels are already current.)
    refreshTerrainRegionsPanel(state);
    refreshWeatherRegionsPanel(state);
    refreshEnvSubModePanel(state);
    applyEditMode(state);
end


%% ========================================================================
%  SIDEBAR BUILDERS
%% ========================================================================

function buildModeTogglePanel(parent, state)
%buildModeTogglePanel  M6 §3.2 — Mode toggle sub-panel (row 1, ~120 px).
%
%  v3.5 §5d (3D-toggle relocation): the 2D/3D view toggle used to
%  live in the Scenario sub-panel (row 9). It was moved up here so it
%  stays visible — and clickable — in Sensors and Environment edit
%  modes too. View mode is global; it doesn't belong to any one
%  edit mode.
%
%  Layout:
%    ┌─ Edit mode ──────────────────────────────────┐
%    │ ┌───────┐ ┌───────┐ ┌──────────────────┐     │
%    │ │Targets│ │Sensors│ │   Environment    │     │
%    │ └───────┘ └───────┘ └──────────────────┘     │
%    │ ┌──────────────────────────────────────────┐ │
%    │ │   View: 2D (click to switch to 3D)       │ │
%    │ └──────────────────────────────────────────┘ │
%    │ Click a button to switch editable sub-panels │
%    └──────────────────────────────────────────────┘
%
%  Two uibutton('state') widgets acting as a mutually-exclusive radio
%  pair. The "mutual exclusion" is enforced in the callbacks, not by
%  MATLAB — when one flips to Value=true its onMode* callback flips the
%  other to Value=false before calling state.setEditMode and
%  applyEditMode.
%
%  WHY STATE-BUTTONS (not a uibuttongroup)
%    uibuttongroup radio behavior is a classic-figure concept; in
%    R2025b uifigure the webfigure backend does not support it inside
%    scrollable panels. A pair of state-buttons is the idiomatic
%    replacement and also mirrors the existing curveModeBtn / viewModeBtn
%    pattern elsewhere in this file.
    pnl = uipanel(parent, 'Title', 'Edit mode');
    g = uigridlayout(pnl, [3 1]);
    g.RowHeight   = {30, 28, 22};
    g.ColumnWidth = {'1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 2;

    % Row 1 — button triple (nested 1x3 grid, M7 §3.2 expansion)
    btns = uigridlayout(g, [1 3]);
    btns.Layout.Row    = 1;
    btns.Layout.Column = 1;
    btns.RowHeight   = {30};
    btns.ColumnWidth = {'1x', '1x', '1x'};
    btns.Padding     = [0 0 0 0];
    btns.ColumnSpacing = 6;

    mode0        = char(state.editMode);
    isTargets    = strcmp(mode0, 'targets');
    isSensors    = strcmp(mode0, 'sensors');
    isEnvironment= strcmp(mode0, 'environment');
    state.modeTargetsBtn = uibutton(btns, 'state', ...
        'Text', 'Targets', ...
        'FontWeight', 'bold', ...
        'Value', isTargets, ...
        'Tooltip', 'Edit target waypoints and scenario metadata.', ...
        'ValueChangedFcn', @(src, ~) onModeTargetsPressed(src, state));
    state.modeSensorsBtn = uibutton(btns, 'state', ...
        'Text', 'Sensors', ...
        'FontWeight', 'bold', ...
        'Value', isSensors, ...
        'Tooltip', 'Edit sensor placement and radar parameters.', ...
        'ValueChangedFcn', @(src, ~) onModeSensorsPressed(src, state));
    state.modeEnvironmentBtn = uibutton(btns, 'state', ...
        'Text', 'Environment', ...
        'FontWeight', 'bold', ...
        'Value', isEnvironment, ...
        'Tooltip', 'Edit terrain, weather, and physics degradation toggles.', ...
        'ValueChangedFcn', @(src, ~) onModeEnvironmentPressed(src, state));

    % Row 2 — 2D/3D view toggle. View applies to ALL edit modes (not
    % just Targets), so this lives in the always-visible Mode Toggle
    % panel rather than down inside the Scenario sub-panel (where it
    % used to live in M3.3). State-button pattern matches curveModeBtn
    % so the "you are HERE" affordance is consistent.
    state.viewModeBtn = uibutton(g, 'state', ...
        'Text', viewButtonText(state.viewMode), ...
        'Value', strcmp(char(state.viewMode), '3d'), ...
        'Tooltip', 'Toggle between 2D map view and 3D camera view (V key).', ...
        'ValueChangedFcn', @(src, ~) onViewModeChanged(src, state));
    state.viewModeBtn.Layout.Row    = 2;
    state.viewModeBtn.Layout.Column = 1;

    % Row 3 — hint label
    hint = uilabel(g, ...
        'Text', 'Click a button to switch editable sub-panels.', ...
        'FontColor', [0.4 0.4 0.4], ...
        'FontSize', 11);
    hint.Layout.Row = 3;
    hint.Layout.Column = 1;
end


function buildSensorsPanel(parent, state)
%buildSensorsPanel  M6 §3.2 — Sensors sub-panel (row 2, 110 px).
%
%  Layout:
%    ┌─ Sensors ────────────────────────────────────┐
%    │ [ sensor_1 (PSR, active)            ] ▼      │
%    │ [ + Add ] [ Duplicate ] [ Delete ] [ Rename ]│
%    └──────────────────────────────────────────────┘
%
%  Parallel to buildTargetsPanel — same 2×1 layout, same button
%  order, so muscle memory transfers between Targets and Sensors modes.
%
%  Dropdown label format (similar to Targets):
%    writable + inactive    : "name (PSR)"
%    writable + active      : "name (PSR, active)"
%    read-only + inactive   : "name (PSR, passthrough)"
%    read-only + active     : "name (PSR, active, passthrough)"
%  The (passthrough) tag is the user-visible signal that a sensor is
%  locked (UNKNOWN type loaded from a JSON file — see §7 loadRunFile).
%
%  WHY THE "+" PREFIX ON ADD
%    The +Add button opens a modal with 8 type choices. The prefix
%    differentiates it visually from the Targets sub-panel's "New"
%    button (which takes no type argument), so users don't mis-click
%    expecting the same behavior.
    pnl = uipanel(parent, 'Title', 'Sensors');
    state.sensorsPanel = pnl;   % v3.5 step 4a — mode-specific show/hide
    % v3.5 step 4c — expanded from 2 rows to 3 to host file-I/O.
    %   Row 1: dropdown                                 30 px
    %   Row 2: + Add | Duplicate | Delete | Rename      28 px
    %   Row 3: Load Sensor… | Save Sensor…               28 px
    g = uigridlayout(pnl, [3 1]);
    g.RowHeight   = {30, 28, 28};
    g.ColumnWidth = {'1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;

    % Row 1 — sensors dropdown (placeholder until refreshSensorsDropdown)
    state.sensorsDD = uidropdown(g, ...
        'Items', {'(no sensors)'}, 'ItemsData', {0}, 'Value', 0, ...
        'Tooltip', 'Active sensor. Pick another to edit its parameters.', ...
        'ValueChangedFcn', @(src, ~) onSensorsDropdownChanged(src, state));

    % Row 2 — collection-management button strip
    btns = uigridlayout(g, [1 4]);
    btns.Layout.Row    = 2;
    btns.Layout.Column = 1;
    btns.RowHeight   = {28};
    btns.ColumnWidth = {'1x', '1x', '1x', '1x'};
    btns.Padding     = [0 0 0 0];
    btns.ColumnSpacing = 4;

    state.sensorsBtnAdd = uibutton(btns, 'push', 'Text', '+ Add', ...
        'Tooltip', 'Add a new sensor. A modal asks you to pick a type.', ...
        'ButtonPushedFcn', @(~, ~) onSensorsAdd(state));
    state.sensorsBtnDuplicate = uibutton(btns, 'push', 'Text', 'Duplicate', ...
        'Tooltip', 'Copy the active sensor, offset 2 km east so it does not overlap.', ...
        'ButtonPushedFcn', @(~, ~) onSensorsDuplicate(state));
    state.sensorsBtnDelete = uibutton(btns, 'push', 'Text', 'Delete', ...
        'BackgroundColor', [0.90 0.40 0.40], 'FontColor', 'white', ...
        'Tooltip', 'Remove the active sensor from the scenario (confirmable, undoable).', ...
        'ButtonPushedFcn', @(~, ~) onSensorsDelete(state));
    state.sensorsBtnRename = uibutton(btns, 'push', 'Text', 'Rename', ...
        'Tooltip', 'Rename the active sensor (alphanumeric; must be unique).', ...
        'ButtonPushedFcn', @(~, ~) onSensorsRename(state));

    % Row 3 — file-I/O strip (v3.5 step 4c). Save is bold + blue to
    % match the styling of Save Target in the Targets panel — makes
    % the primary commit verb visually consistent across modes.
    %   onLoadSensors APPENDS to the collection (existing semantic);
    %   onSaveSensor writes the ACTIVE sensor only via
    %   exportSingleSensorToJSON.
    fioRow = uigridlayout(g, [1 2]);
    fioRow.Layout.Row    = 3;
    fioRow.Layout.Column = 1;
    fioRow.RowHeight    = {28};
    fioRow.ColumnWidth  = {'1x', '1x'};
    fioRow.Padding      = [0 0 0 0];
    fioRow.ColumnSpacing = 4;
    state.sensorsBtnLoad = uibutton(fioRow, 'push', 'Text', 'Load Sensor…', ...
        'Tooltip', 'Load a sensor JSON file from config/sensors/. Appends to the current collection.', ...
        'ButtonPushedFcn', @(~, ~) onLoadSensors(state));
    state.sensorsBtnSave = uibutton(fioRow, 'push', 'Text', 'Save Sensor…', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.25 0.55 0.85], 'FontColor', 'white', ...
        'Tooltip', 'Export the active sensor to a single JSON file in config/sensors/.', ...
        'ButtonPushedFcn', @(~, ~) onSaveSensor(state));
end


function buildSensorParamsPanel(parent, state)
%buildSensorParamsPanel  M6 §3.2 — Sensor Parameters sub-panel (row 3).
%
%  Type-driven fields. The layout is a 12-row × 4-column grid where
%  most rows pack a (label, value, label, value) quad so two numeric
%  pairs fit per row without stretching the sidebar. The scan-mode
%  banner and the Place-on-map button span all 4 columns.
%
%  ROWS
%    1. Name           (label + field, col-span 3)
%    2. Type           (label + dropdown, col-span 3)
%    3. East  | North
%    4. Altitude | Tilt
%    5. Freq (GHz) | Max range (m)
%    6. Range res (m) | (empty)
%    7. Az FOV (deg) | El FOV (deg)
%    8. Scan-mode banner (col-span 4, italic, informational)
%    9. RPM (label + field; always visible — 0 = staring)
%   10. Sector start | Sector end  (visible for sector + no-scan)
%   11. Pd | FAR
%   12. Place on map   (disabled in §3.2 — wired in §3.6)
%
%  TYPE-DRIVEN VISIBILITY
%    refreshSensorParamsPanel() calls setPropIfGraphics(h, 'Visible', …)
%    for the Sector widgets based on the active sensor's scan kind
%    (isRotator / isSector / isNoScan from SensorRecord). The RPM
%    widget stays visible across all three so the user can transition
%    between scan modes by just editing RPM — hiding it would make
%    "TWS ⇒ PSR" require the dropdown path.
%
%  LIMITS-BEFORE-VALUE
%    All numeric fields have Limits set at construct time. The
%    refresh path only sets Value. Values from SensorRecord.m respect
%    these limits (they share their ranges with buildSensor.m); if
%    that invariant ever breaks, uieditfield raises with a loud error
%    rather than silently clamping — which is what we want.
%
%  FREQUENCY UNIT HANDLING
%    The on-disk format stores Hz (matching buildSensor). The UI
%    displays GHz for readability (2.8 vs 2_800_000_000). onSensorFieldChanged
%    and refreshSensorParamsPanel do the *1e9 / /1e9 conversion.

    pnl = uipanel(parent, 'Title', 'Sensor parameters');
    state.sensorParamsPanel = pnl;

    nRows = 12;
    g = uigridlayout(pnl, [nRows 4]);
    g.RowHeight     = repmat({24}, 1, nRows);
    g.ColumnWidth   = {110, '1x', 95, '1x'};
    g.Padding       = [6 6 6 6];
    g.RowSpacing    = 4;
    % ColumnSpacing tightened default 10 → 4 so the two '1x' field
    % columns keep enough pixel width to show numeric values. At the
    % 400 px sidebar with default ColumnSpacing=10 the three inter-
    % column gaps consume 30 px; dropping to 4 reclaims 18 px, giving
    % each field ~76 px after labels and padding. If the sidebar ever
    % widens again, this can relax back to 10 for visual breathing room.
    g.ColumnSpacing = 4;

    % Row 1 — Name
    lName = uilabel(g, 'Text', 'Name');
    lName.Layout.Row = 1; lName.Layout.Column = 1;
    state.sensorNameField = uieditfield(g, 'text', 'Value', '', ...
        'ValueChangedFcn', @(src, ~) onSensorNameChanged(src, state));
    state.sensorNameField.Layout.Row = 1;
    state.sensorNameField.Layout.Column = [2 4];

    % Row 2 — Type
    lType = uilabel(g, 'Text', 'Type');
    lType.Layout.Row = 2; lType.Layout.Column = 1;
    domSel     = trackbench.editor.sensorDomain(state.domain);
    domTypes   = cellstr(domSel.sensorTypes);
    domDefault = char(domSel.defaultSensor);
    state.sensorTypeDD = uidropdown(g, ...
        'Items', domTypes, ...
        'Value', domDefault, ...
        'Tooltip', 'Changing type resets per-type defaults (keeps name, position, altitude).', ...
        'ValueChangedFcn', @(src, ~) onSensorTypeChanged(src, state));
    state.sensorTypeDD.Layout.Row = 2;
    state.sensorTypeDD.Layout.Column = [2 4];

    % Row 3 — East | North
    lEast = uilabel(g, 'Text', 'East (m)');
    lEast.Layout.Row = 3; lEast.Layout.Column = 1;
    state.sensorEastField = uieditfield(g, 'numeric', 'Value', 0, ...
        'Limits', [-1e7 1e7], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'east'));
    state.sensorEastField.Layout.Row = 3;
    state.sensorEastField.Layout.Column = 2;
    lNorth = uilabel(g, 'Text', 'North (m)');
    lNorth.Layout.Row = 3; lNorth.Layout.Column = 3;
    state.sensorNorthField = uieditfield(g, 'numeric', 'Value', 0, ...
        'Limits', [-1e7 1e7], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'north'));
    state.sensorNorthField.Layout.Row = 3;
    state.sensorNorthField.Layout.Column = 4;

    % Row 4 — Altitude | Tilt
    lAlt = uilabel(g, 'Text', 'Altitude (m)');
    lAlt.Layout.Row = 4; lAlt.Layout.Column = 1;
    state.sensorAltField = uieditfield(g, 'numeric', 'Value', 15, ...
        'Limits', [0 5000], ...
        'Tooltip', 'Height above ground (m). Stored as -alt on mountingLoc(3) (NED).', ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'altitude'));
    state.sensorAltField.Layout.Row = 4;
    state.sensorAltField.Layout.Column = 2;
    lTilt = uilabel(g, 'Text', 'Tilt (deg)');
    lTilt.Layout.Row = 4; lTilt.Layout.Column = 3;
    state.sensorTiltField = uieditfield(g, 'numeric', 'Value', 2, ...
        'Limits', [-90 90], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'tilt'));
    state.sensorTiltField.Layout.Row = 4;
    state.sensorTiltField.Layout.Column = 4;

    % Row 5 — Frequency (GHz) | Max range (m)
    lFreq = uilabel(g, 'Text', 'Frequency (GHz)');
    lFreq.Layout.Row = 5; lFreq.Layout.Column = 1;
    state.sensorFreqField = uieditfield(g, 'numeric', 'Value', 2.8, ...
        'Limits', [0.1 40], ...
        'Tooltip', 'Center frequency in GHz. Converted to Hz on export.', ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'frequency'));
    state.sensorFreqField.Layout.Row = 5;
    state.sensorFreqField.Layout.Column = 2;
    lMaxR = uilabel(g, 'Text', 'Max range (m)');
    lMaxR.Layout.Row = 5; lMaxR.Layout.Column = 3;
    state.sensorMaxRangeField = uieditfield(g, 'numeric', 'Value', 111120, ...
        'Limits', [1 1e7], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'maxRange'));
    state.sensorMaxRangeField.Layout.Row = 5;
    state.sensorMaxRangeField.Layout.Column = 4;

    % Row 6 — Range res (m)
    lRRes = uilabel(g, 'Text', 'Range res (m)');
    lRRes.Layout.Row = 6; lRRes.Layout.Column = 1;
    state.sensorRangeResField = uieditfield(g, 'numeric', 'Value', 93, ...
        'Limits', [0.1 1e5], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'rangeRes'));
    state.sensorRangeResField.Layout.Row = 6;
    state.sensorRangeResField.Layout.Column = 2;

    % Row 7 — Az FOV | El FOV
    lAzFov = uilabel(g, 'Text', 'Az FOV (deg)');
    lAzFov.Layout.Row = 7; lAzFov.Layout.Column = 1;
    state.sensorAzFovField = uieditfield(g, 'numeric', 'Value', 1.4, ...
        'Limits', [0.01 180], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'azFov'));
    state.sensorAzFovField.Layout.Row = 7;
    state.sensorAzFovField.Layout.Column = 2;
    lElFov = uilabel(g, 'Text', 'El FOV (deg)');
    lElFov.Layout.Row = 7; lElFov.Layout.Column = 3;
    state.sensorElFovField = uieditfield(g, 'numeric', 'Value', 30, ...
        'Limits', [0.01 180], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'elFov'));
    state.sensorElFovField.Layout.Row = 7;
    state.sensorElFovField.Layout.Column = 4;

    % Row 8 — Scan-mode banner (informational; derived from RPM + sector)
    state.sensorScanModeLbl = uilabel(g, ...
        'Text', 'Scan mode: —', ...
        'FontAngle', 'italic', ...
        'FontColor', [0.35 0.35 0.35]);
    state.sensorScanModeLbl.Layout.Row = 8;
    state.sensorScanModeLbl.Layout.Column = [1 4];

    % Row 9 — RPM (0 = no-scan / staring)
    state.sensorRpmLabel = uilabel(g, 'Text', 'RPM');
    state.sensorRpmLabel.Layout.Row = 9;
    state.sensorRpmLabel.Layout.Column = 1;
    state.sensorRpmField = uieditfield(g, 'numeric', 'Value', 12.5, ...
        'Limits', [0 120], ...
        'Tooltip', 'RPM = 0 ⇒ staring / no-scan. > 0 ⇒ rotator (full circle) or sector scan.', ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'rpm'));
    state.sensorRpmField.Layout.Row = 9;
    state.sensorRpmField.Layout.Column = 2;

    % Row 10 — Sector start | Sector end (hidden for pure rotator)
    state.sensorSectorLoLabel = uilabel(g, 'Text', 'Sector start (deg)');
    state.sensorSectorLoLabel.Layout.Row = 10;
    state.sensorSectorLoLabel.Layout.Column = 1;
    state.sensorSectorLoField = uieditfield(g, 'numeric', 'Value', 0, ...
        'Limits', [-360 720], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'sectorLo'));
    state.sensorSectorLoField.Layout.Row = 10;
    state.sensorSectorLoField.Layout.Column = 2;
    state.sensorSectorHiLabel = uilabel(g, 'Text', 'Sector end (deg)');
    state.sensorSectorHiLabel.Layout.Row = 10;
    state.sensorSectorHiLabel.Layout.Column = 3;
    state.sensorSectorHiField = uieditfield(g, 'numeric', 'Value', 360, ...
        'Limits', [-360 720], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'sectorHi'));
    state.sensorSectorHiField.Layout.Row = 10;
    state.sensorSectorHiField.Layout.Column = 4;

    % Row 11 — Pd | FAR
    lPd = uilabel(g, 'Text', 'Pd');
    lPd.Layout.Row = 11; lPd.Layout.Column = 1;
    state.sensorPdField = uieditfield(g, 'numeric', 'Value', 0.9, ...
        'Limits', [0 1], ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'pd'));
    state.sensorPdField.Layout.Row = 11;
    state.sensorPdField.Layout.Column = 2;
    lFar = uilabel(g, 'Text', 'FAR');
    lFar.Layout.Row = 11; lFar.Layout.Column = 3;
    state.sensorFarField = uieditfield(g, 'numeric', 'Value', 1e-6, ...
        'Limits', [0 1], ...
        'Tooltip', 'False-alarm rate (unitless).', ...
        'ValueChangedFcn', @(src, ~) onSensorFieldChanged(src, state, 'far'));
    state.sensorFarField.Layout.Row = 11;
    state.sensorFarField.Layout.Column = 4;

    % Row 12 — Place-on-map button (M6 §3.5A)
    %   Wired: click button → state.sensorPlacePending = true → next
    %   axes click teleports the active sensor to the click point and
    %   clears the pending flag. Enable gate: sensors mode + an active
    %   sensor exists + it is not read-only (applyEditMode handles this).
    state.sensorPlaceOnMapBtn = uibutton(g, 'push', ...
        'Text', 'Place on map', ...
        'Enable', 'off', ...
        'Tooltip', 'Click, then left-click on the map to set East/North.', ...
        'ButtonPushedFcn', @(~, ~) onSensorPlaceOnMap(state));
    state.sensorPlaceOnMapBtn.Layout.Row = 12;
    state.sensorPlaceOnMapBtn.Layout.Column = [1 4];
end


function buildTargetsPanel(parent, state)
%buildTargetsPanel  M5 §3.1 — Targets sub-panel (dropdown + 4 buttons).
%
%  Layout:
%    ┌─ Targets ────────────────────────────────────┐
%    │ [ ● target_1 (active)              ] ▼       │
%    │ [ New ] [ Duplicate ] [ Delete ] [ Rename ]  │
%    └──────────────────────────────────────────────┘
%
%  Why a dropdown and not a listbox: the panel only needs to show the
%  active target and let the user pick another. A dropdown saves
%  vertical real estate (the sidebar is already busy with Scenario and
%  Selection). §3.2 may revisit this if we need a persistent view of
%  which targets are references vs editable.
%
%  Why refreshTargetsDropdown rebuilds Items + ItemsData together:
%  uidropdown raises if the current Value isn't in ItemsData. Rebuild
%  in the order Items → ItemsData → Value to avoid transient
%  inconsistency. See M5 handoff §8 gotcha #3.
    pnl = uipanel(parent, 'Title', 'Targets');
    state.targetsPanel = pnl;   % v3.5 step 4a — mode-specific show/hide
    % 6 rows × 1 column layout (v3.5 step 4c expanded the panel from 3
    % rows to 6). Row groups by operation class:
    %   Row 1: dropdown                     30 px — active-target picker
    %   Row 2: New|Duplicate|Delete|Rename  28 px — collection mgmt
    %   Row 3: Load Target | Save Target    28 px — standard file I/O
    %   Row 4: Import NASA Flight…          28 px — special importer
    %   Row 5: Load as Reference | Unload   28 px — reference overlay
    %   Row 6: Clear all waypoints          28 px — destructive (alone)
    %
    % Total content: 30 + 5*28 + 5*4 spacings + 12 padding + ~22 title
    %              = 224 px. Outer slot in inner.RowHeight is 230 px.
    g = uigridlayout(pnl, [6 1]);
    g.RowHeight   = {30, 28, 28, 28, 28, 28};
    g.ColumnWidth = {'1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;

    % Row 1: target dropdown. Populated by refreshTargetsDropdown on
    % first draw; we supply a placeholder here so uidropdown accepts
    % the construct call without Value-not-in-Items errors.
    state.targetsDD = uidropdown(g, ...
        'Items', {'(no targets)'}, 'ItemsData', {0}, 'Value', 0, ...
        'Tooltip', 'Active target. Pick another to edit it.', ...
        'ValueChangedFcn', @(src, ~) onTargetsDropdownChanged(src, state));

    % Row 2: button strip (New / Duplicate / Delete / Rename) — nested
    % 1×4 grid inside row 2.
    btns = uigridlayout(g, [1 4]);
    btns.Layout.Row    = 2;
    btns.Layout.Column = 1;
    btns.RowHeight   = {28};
    btns.ColumnWidth = {'1x', '1x', '1x', '1x'};
    btns.Padding     = [0 0 0 0];
    btns.ColumnSpacing = 4;

    state.targetsBtnNew = uibutton(btns, 'push', 'Text', 'New', ...
        'Tooltip', 'Create a new empty target with default scenario fields.', ...
        'ButtonPushedFcn', @(~, ~) onTargetsNew(state));
    state.targetsBtnDuplicate = uibutton(btns, 'push', 'Text', 'Duplicate', ...
        'Tooltip', 'Copy the active target (waypoints + scenario fields).', ...
        'ButtonPushedFcn', @(~, ~) onTargetsDuplicate(state));
    state.targetsBtnDelete = uibutton(btns, 'push', 'Text', 'Delete', ...
        'BackgroundColor', [0.90 0.40 0.40], 'FontColor', 'white', ...
        'Tooltip', 'Remove the active target from this edit session.', ...
        'ButtonPushedFcn', @(~, ~) onTargetsDelete(state));
    state.targetsBtnRename = uibutton(btns, 'push', 'Text', 'Rename', ...
        'Tooltip', 'Rename the active target (same rules as Target name).', ...
        'ButtonPushedFcn', @(~, ~) onTargetsRename(state));

    % Row 3: Load + Save target file I/O (v3.5 step 4c). Nested 1×2
    % grid for clean side-by-side spacing. Save is the highlighted
    % primary action (blue + bold, mirroring the old File-panel
    % "Export JSON" emphasis) so it reads as the main commit verb.
    fioRow = uigridlayout(g, [1 2]);
    fioRow.Layout.Row    = 3;
    fioRow.Layout.Column = 1;
    fioRow.RowHeight    = {28};
    fioRow.ColumnWidth  = {'1x', '1x'};
    fioRow.Padding      = [0 0 0 0];
    fioRow.ColumnSpacing = 4;
    state.targetsBtnLoad = uibutton(fioRow, 'push', 'Text', 'Load Target…', ...
        'Tooltip', 'Load a target waypoints JSON file (replaces or appends to current targets).', ...
        'ButtonPushedFcn', @(~, ~) onLoad(state));
    state.targetsBtnSave = uibutton(fioRow, 'push', 'Text', 'Save Target…', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.25 0.55 0.85], 'FontColor', 'white', ...
        'Tooltip', 'Export all writable targets to a single waypoints JSON file.', ...
        'ButtonPushedFcn', @(~, ~) onExport(state));

    % Row 4: NASA Flight importer (v3.5 step 4b). Spans full width —
    % see step-4b commit message for layout rationale.
    state.targetsBtnNasaFlight = uibutton(g, 'push', ...
        'Text', 'Import NASA Flight (.mat)…', ...
        'Tooltip', 'Load a NASA DASHlink Flight Data Recorder .mat file as a target trajectory.', ...
        'ButtonPushedFcn', @(~, ~) onTargetsAddNasaFlight(state));
    state.targetsBtnNasaFlight.Layout.Row    = 4;
    state.targetsBtnNasaFlight.Layout.Column = 1;

    % Row 5: reference-overlay buttons (v3.5 step 4c). Reference
    % targets are read-only path overlays loaded for context; users
    % Duplicate one to get an editable copy.
    refRow = uigridlayout(g, [1 2]);
    refRow.Layout.Row    = 5;
    refRow.Layout.Column = 1;
    refRow.RowHeight    = {28};
    refRow.ColumnWidth  = {'1x', '1x'};
    refRow.Padding      = [0 0 0 0];
    refRow.ColumnSpacing = 4;
    state.targetsBtnLoadRef = uibutton(refRow, 'push', 'Text', 'Load as Reference…', ...
        'Tooltip', 'Load a target file as a read-only reference overlay (display only — not exported).', ...
        'ButtonPushedFcn', @(~, ~) onLoadReference(state));
    state.targetsBtnUnloadRefs = uibutton(refRow, 'push', 'Text', 'Unload References', ...
        'Tooltip', 'Remove all reference targets currently overlaid on the map.', ...
        'ButtonPushedFcn', @(~, ~) onUnloadReferences(state));

    % Row 6: destructive action (v3.5 step 4c). Kept on its own row
    % away from the file-I/O cluster so a stray click can't accidentally
    % land on Clear when aiming for Save. No bold styling — we want it
    % visually quieter than Save, not louder.
    state.targetsBtnClear = uibutton(g, 'push', 'Text', 'Clear all waypoints', ...
        'Tooltip', 'Wipe the active target''s waypoints. Does not affect scenario fields.', ...
        'ButtonPushedFcn', @(~, ~) onClear(state));
    state.targetsBtnClear.Layout.Row    = 6;
    state.targetsBtnClear.Layout.Column = 1;
end


function buildScenarioPanel(parent, state)
    pnl = uipanel(parent, 'Title', 'Scenario');
    % M5 §3.2 — capture handle so refreshScenarioPanel can change the
    % panel title between "Scenario" and the read-only banner text when
    % the active target is a reference.
    state.scenarioPanel = pnl;
    % 12 rows: the five scenario-field pairs, waypoint count, M3.1
    % colormap checkbox, M3.2 grid dropdown, the M4.3.2 curve-mode
    % toggle, the M4.3.3 curve-tension dropdown, the Apply-altitude
    % button, and the M3.4 preview button.
    %
    % v3.5 §5d: the M3.3 2D/3D toggle was moved up to the Mode Toggle
    % panel so it stays visible in Sensors and Environment edit modes
    % too. That dropped this panel from 13 rows to 12.
    %
    % Height: 12*24 + 11*4 + 2*6 = 344; the inner grid row for this
    % panel in buildUI is 350.
    %
    % GOTCHA (M2 feedback): Do NOT use 'fit' for row heights here —
    % the parent is scrollable and 'fit' triggers MATLAB's layout
    % recursion. Explicit pixel heights are required.
    nRows = 12;
    g = uigridlayout(pnl, [nRows 2]);
    g.RowHeight   = repmat({24}, 1, nRows);
    g.ColumnWidth = {120, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;

    uilabel(g, 'Text', 'Target name');
    state.nameField = uieditfield(g, 'text', 'Value', char(state.targetName), ...
        'ValueChangedFcn', @(src, ~) onNameChanged(src, state));

    % M5 §3.1 — capture handles so refreshScenarioPanel() can re-sync
    % these fields when the active target changes (per-target storage
    % means the scenario panel mirrors a different TargetRecord every
    % time the user picks something else from the Targets dropdown).
    uilabel(g, 'Text', 'Default speed (km/h)');
    state.speedField = uieditfield(g, 'numeric', 'Value', state.defaultSpeedKmh, ...
        'Limits', [1 5000], ...
        'ValueChangedFcn', @(src, ~) onSpeedChanged(src, state));

    uilabel(g, 'Text', 'Default altitude (m)');
    state.altField = uieditfield(g, 'numeric', 'Value', state.defaultAltitudeM, ...
        'Limits', [0 30000], ...
        'ValueChangedFcn', @(src, ~) onAltitudeChanged(src, state));

    uilabel(g, 'Text', 'RCS (dBsm)');
    state.rcsField = uieditfield(g, 'numeric', 'Value', state.rcsDbsm, ...
        'Limits', [-50 50], ...
        'ValueChangedFcn', @(src, ~) onRcsDbsmChanged(src, state));

    uilabel(g, 'Text', 'RCS profile');
    state.rcsProfileDD = uidropdown(g, ...
        'Items', {'none','stealth','fighter','airliner','drone','missile'}, ...
        'Value', char(state.rcsProfile), ...
        'ValueChangedFcn', @(src, ~) onRcsProfileChanged(src, state));

    uilabel(g, 'Text', 'Waypoints:');
    state.waypointCountLbl = uilabel(g, 'Text', '0');

    % M3.1 — "Color by altitude" checkbox. Off by default so existing
    % users see the same yellow markers they had in M2.
    state.colorByAltCheckbox = uicheckbox(g, ...
        'Text', 'Color waypoints by altitude', ...
        'Value', state.colorByAltitude, ...
        'ValueChangedFcn', @(src, ~) onColorByAltitudeChanged(src, state));
    state.colorByAltCheckbox.Layout.Row    = 7;
    state.colorByAltCheckbox.Layout.Column = [1 2];

    % M3.2 — Grid spacing dropdown. Items map to kilometers (or 0 = off).
    uilabel(g, 'Text', 'Grid spacing');
    state.gridSpacingDD = uidropdown(g, ...
        'Items', {'off', '1 km', '5 km', '10 km'}, ...
        'ItemsData', [0, 1, 5, 10], ...
        'Value', state.gridSpacingKm, ...
        'ValueChangedFcn', @(src, ~) onGridSpacingChanged(src, state));

    % M4.3.2 — Curve-mode toggle. Straight is the M3 default; flipping
    % this on enables a centripetal Catmull-Rom interpolation through
    % the control waypoints for rendering (and, once §3.4 lands, for
    % export). Uses the same state-button pattern as the 2D/3D toggle
    % so the active mode is obvious at a glance.
    state.curveModeBtn = uibutton(g, 'state', ...
        'Text', curveButtonText(state.curveMode), ...
        'Value', strcmp(char(state.curveMode), 'curved'), ...
        'ValueChangedFcn', @(src, ~) onCurveModeChanged(src, state));
    state.curveModeBtn.Layout.Row    = 9;
    state.curveModeBtn.Layout.Column = [1 2];

    % M4.3.3 — Curve tension dropdown. Controls the Catmull-Rom alpha:
    %   Uniform      (alpha=0.0) tightest, can overshoot at cusps.
    %   Centripetal  (alpha=0.5) default; no self-intersections.
    %   Chordal      (alpha=1.0) loosest, most rounded.
    % The selection always pushes an undo snapshot so Ctrl+Z returns to
    % the prior alpha. Selecting a new tension triggers a redraw even
    % when curveMode=="straight" — that way flipping to Smooth
    % immediately shows the chosen family without a second click. The
    % catmullRomCurve implementation accepts alpha in [0,1] and defaults
    % to 0.5 if nothing is wired in.
    uilabel(g, 'Text', 'Curve tension');
    state.curveTensionDD = uidropdown(g, ...
        'Items', {'Uniform', 'Centripetal', 'Chordal'}, ...
        'ItemsData', [0.0, 0.5, 1.0], ...
        'Value', state.curveTensionAlpha, ...
        'Tooltip', ['Catmull-Rom parameterization. Uniform = tight, can ' ...
                    'overshoot. Centripetal (default) = no self-intersections. ' ...
                    'Chordal = loosest, most rounded.'], ...
        'ValueChangedFcn', @(src, ~) onCurveTensionChanged(src, state));

    % "Apply default altitude to all" — spans both columns.
    applyAltBtn = uibutton(g, 'push', ...
        'Text', 'Apply default altitude to all waypoints', ...
        'ButtonPushedFcn', @(~, ~) onApplyDefaultAltitude(state));
    applyAltBtn.Layout.Column = [1 2];
    applyAltBtn.Layout.Row    = 11;

    % M3.4 — Preview Animation button. Opens a secondary uifigure that
    % animates a marker along the current waypoint path using the
    % auto-computed time_s timeline. View-only; no editing in the
    % preview window.
    state.previewBtn = uibutton(g, 'push', ...
        'Text', 'Preview Animation', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.88 0.92 0.98], ...
        'Tooltip', ['Open a secondary window and animate a marker along ' ...
                    'the current waypoint path (view-only).'], ...
        'ButtonPushedFcn', @(~, ~) onPreviewRequest(state));
    state.previewBtn.Layout.Column = [1 2];
    state.previewBtn.Layout.Row    = 12;
end


function t = curveButtonText(mode)
%curveButtonText  Human-readable label for the curve toggle. Always tells
%                 the user where they ARE, not where they're going.
    if strcmp(char(mode), 'curved')
        t = 'Curve: Smooth (click for Straight)';
    else
        t = 'Curve: Straight (click for Smooth)';
    end
end


function t = viewButtonText(mode)
%viewButtonText  Human-readable label for the view toggle. Always tells
%                the user where they ARE, not where they're going.
    if strcmp(char(mode), '3d')
        t = 'View: 3D (click to return to 2D)';
    else
        t = 'View: 2D (click to switch to 3D)';
    end
end


function buildTerrainPanel(parent, state)
%buildTerrainPanel  M7 §3.2 — Terrain sub-panel (row 6, 220 px).
%
%  Layout:
%    ┌─ Terrain ────────────────────────────────────┐
%    │ Type        [ rural           ] ▼            │
%    │ Description [ Rolling farmland, 80m peaks, …]│
%    │ Scale       [ 1.00 ]  Clutter    [ 0.30  ]   │
%    │ Refraction  [ 1.333]  (4/3 Earth)            │
%    │ Degradation [x] Terrain occlusion            │
%    │             [x] Horizon masking              │
%    │             [x] Ground clutter               │
%    │             [x] Doppler fade                 │
%    │ [ Load from file… ]   [x] Overlay on map     │
%    └──────────────────────────────────────────────┘
%
%  The Type dropdown is the primary control. Changing it cascades a
%  reset via state.setTerrainType (see onTerrainTypeChanged); a
%  uiconfirm dialog warns the user first when the current terrain
%  diverges from the disk defaults.
%
%  The four degradation checkboxes map 1:1 onto the run file's
%  degradation block booleans. They live in the Terrain panel (not a
%  dedicated Degradation panel) because three of the four are
%  terrain-physics toggles and only one — doppler_fade — is not. The
%  handoff calls this out: a single Degradation group is simpler
%  than splitting it.
%
%  The "Overlay on map" checkbox drives drawMap's terrain tint render
%  (§3.3). Kept local to this panel because it's a view-only concern
%  that shouldn't clutter the View group in Scenario.
    pnl = uipanel(parent, 'Title', 'Terrain');
    state.terrainPanel = pnl;

    g = uigridlayout(pnl, [10 4]);
    g.RowHeight   = {22, 22, 22, 22, 20, 20, 20, 20, 26, 26};
    % 4-col: label | value | label | value — gives room for paired
    % fields on rows 3 (Scale/Clutter) and for the degradation
    % checkboxes which span 3 columns so the 4-wide row still reads.
    %
    % v3.5 step 4c — row 10 added for the Save Terrain button. The
    % previous 9-row layout had Load + Overlay paired on row 9; rather
    % than disrupt that visual pairing the new Save button gets its
    % own row spanning the full panel width.
    g.ColumnWidth = {80, '1x', 80, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 2;
    g.ColumnSpacing = 4;

    % Row 1 — Type
    lblType = uilabel(g, 'Text', 'Type', 'HorizontalAlignment', 'right');
    lblType.Layout.Row = 1; lblType.Layout.Column = 1;
    state.terrainTypeDD = uidropdown(g, ...
        'Items', {'none','water','rural','urban','mountain','desert'}, ...
        'Value', char(state.terrain.terrainType), ...
        'Tooltip', 'Terrain type. Changing this resets the fields below to that type''s disk defaults.', ...
        'ValueChangedFcn', @(src, ~) onTerrainTypeChanged(src, state));
    state.terrainTypeDD.Layout.Row = 1;
    state.terrainTypeDD.Layout.Column = [2 4];

    % Row 2 — Description
    lblDesc = uilabel(g, 'Text', 'Description', 'HorizontalAlignment', 'right');
    lblDesc.Layout.Row = 2; lblDesc.Layout.Column = 1;
    state.terrainDescField = uieditfield(g, 'text', ...
        'Value', char(state.terrain.description), ...
        'Tooltip', 'Free-form description. Loaded from disk for built-in types; editable.', ...
        'ValueChangedFcn', @(src, ~) onTerrainFieldChanged(src, state, 'description'));
    state.terrainDescField.Layout.Row = 2;
    state.terrainDescField.Layout.Column = [2 4];

    % Row 3 — Scale + Clutter (paired on one row)
    lblScale = uilabel(g, 'Text', 'Scale', 'HorizontalAlignment', 'right');
    lblScale.Layout.Row = 3; lblScale.Layout.Column = 1;
    state.terrainScaleField = uieditfield(g, 'numeric', ...
        'Limits', [0 10], ...
        'Value', state.terrain.terrainScale, ...
        'Tooltip', 'Elevation scaling factor (1.0 = nominal per-type heights).', ...
        'ValueChangedFcn', @(src, ~) onTerrainFieldChanged(src, state, 'terrainScale'));
    state.terrainScaleField.Layout.Row = 3;
    state.terrainScaleField.Layout.Column = 2;
    lblClut = uilabel(g, 'Text', 'Clutter', 'HorizontalAlignment', 'right');
    lblClut.Layout.Row = 3; lblClut.Layout.Column = 3;
    state.terrainClutterField = uieditfield(g, 'numeric', ...
        'Limits', [0 1], ...
        'Value', state.terrain.clutterDensity, ...
        'Tooltip', 'Ground-clutter density, 0..1 (0 = none, 1 = saturated).', ...
        'ValueChangedFcn', @(src, ~) onTerrainFieldChanged(src, state, 'clutterDensity'));
    state.terrainClutterField.Layout.Row = 3;
    state.terrainClutterField.Layout.Column = 4;

    % Row 4 — Refraction
    lblRefr = uilabel(g, 'Text', 'Refraction', 'HorizontalAlignment', 'right');
    lblRefr.Layout.Row = 4; lblRefr.Layout.Column = 1;
    state.terrainRefractionField = uieditfield(g, 'numeric', ...
        'Limits', [1 2], ...
        'Value', state.terrain.refractionFactor, ...
        'Tooltip', 'Atmospheric refraction factor. 1.333 = 4/3 Earth standard model.', ...
        'ValueChangedFcn', @(src, ~) onTerrainFieldChanged(src, state, 'refractionFactor'));
    state.terrainRefractionField.Layout.Row = 4;
    state.terrainRefractionField.Layout.Column = 2;
    lblRefrHint = uilabel(g, 'Text', '(4/3 Earth)', ...
        'FontColor', [0.45 0.45 0.45], 'FontSize', 11);
    lblRefrHint.Layout.Row = 4; lblRefrHint.Layout.Column = [3 4];

    % Row 5 — "Degradation" group header + first checkbox
    lblDeg = uilabel(g, 'Text', 'Degradation', 'HorizontalAlignment', 'right', ...
        'FontWeight', 'bold');
    lblDeg.Layout.Row = 5; lblDeg.Layout.Column = 1;
    state.degTerrainOcclusionCB = uicheckbox(g, ...
        'Text', 'Terrain occlusion', ...
        'Value', state.degradation.terrain_occlusion, ...
        'Tooltip', 'Block line-of-sight through terrain features.', ...
        'ValueChangedFcn', @(src, ~) onDegradationToggled(src, state, 'terrain_occlusion'));
    state.degTerrainOcclusionCB.Layout.Row = 5;
    state.degTerrainOcclusionCB.Layout.Column = [2 4];

    % Row 6 — Horizon masking
    state.degHorizonMaskingCB = uicheckbox(g, ...
        'Text', 'Horizon masking', ...
        'Value', state.degradation.horizon_masking, ...
        'Tooltip', 'Apply horizon / Earth curvature masking.', ...
        'ValueChangedFcn', @(src, ~) onDegradationToggled(src, state, 'horizon_masking'));
    state.degHorizonMaskingCB.Layout.Row = 6;
    state.degHorizonMaskingCB.Layout.Column = [2 4];

    % Row 7 — Ground clutter
    state.degGroundClutterCB = uicheckbox(g, ...
        'Text', 'Ground clutter', ...
        'Value', state.degradation.ground_clutter, ...
        'Tooltip', 'Generate ground-clutter returns.', ...
        'ValueChangedFcn', @(src, ~) onDegradationToggled(src, state, 'ground_clutter'));
    state.degGroundClutterCB.Layout.Row = 7;
    state.degGroundClutterCB.Layout.Column = [2 4];

    % Row 8 — Doppler fade
    state.degDopplerFadeCB = uicheckbox(g, ...
        'Text', 'Doppler fade', ...
        'Value', state.degradation.doppler_fade, ...
        'Tooltip', 'Apply Doppler-notch fading for slow-moving targets.', ...
        'ValueChangedFcn', @(src, ~) onDegradationToggled(src, state, 'doppler_fade'));
    state.degDopplerFadeCB.Layout.Row = 8;
    state.degDopplerFadeCB.Layout.Column = [2 4];

    % Row 9 — Load button + Overlay toggle
    state.terrainLoadBtn = uibutton(g, 'push', 'Text', 'Load from file…', ...
        'Tooltip', 'Load a terrain definition from config/terrain/. Replaces the current terrain.', ...
        'ButtonPushedFcn', @(~, ~) onLoadTerrain(state));
    state.terrainLoadBtn.Layout.Row = 9;
    state.terrainLoadBtn.Layout.Column = [1 2];
    state.terrainOverlayCB = uicheckbox(g, ...
        'Text', 'Overlay on map', ...
        'Value', true, ...
        'Tooltip', ['Render procedural terrain heightmap on the map ' ...
                    '(2D hypsometric tint, 3D surface). Uncheck if ' ...
                    'rendering ever lags.'], ...
        'ValueChangedFcn', @(~, ~) onTerrainOverlayToggled(state));
    state.terrainOverlayCB.Layout.Row = 9;
    state.terrainOverlayCB.Layout.Column = [3 4];

    % Row 10 — Save button (v3.5 step 4c). Full-width to match the
    % weight of "Load from file…" + "Overlay on map" on row 9 above.
    state.terrainSaveBtn = uibutton(g, 'push', 'Text', 'Save to file…', ...
        'Tooltip', 'Save the current terrain configuration to a JSON file.', ...
        'ButtonPushedFcn', @(~, ~) onSaveTerrain(state));
    state.terrainSaveBtn.Layout.Row = 10;
    state.terrainSaveBtn.Layout.Column = [1 4];
end


function buildWeatherPanel(parent, state)
%buildWeatherPanel  M7 §3.2 — Weather sub-panel (row 7, 340 px).
%
%  Layout:
%    ┌─ Weather ────────────────────────────────────┐
%    │ Type        [ (none)          ] ▼            │
%    │ Description [ Moderate rain — 16 mm/hr, …  ] │
%    │ Rain rate   [ 16.00 ] mm/hr                  │
%    │ Storm start [ 50.0 ]  Storm end  [ 130.0 ]   │
%    │ Profile     [ step            ] ▼            │
%    │ Pd floor    [ 0.15 ]  Clutter ×  [ 1.00 ]    │
%    │ ┌──────────────────────────────────────────┐ │
%    │ │ [ storm-profile sparkline, 36 px tall  ] │ │
%    │ └──────────────────────────────────────────┘ │
%    │ [ Load from file… ]                          │
%    └──────────────────────────────────────────────┘
%
%  Type dropdown's first entry is "(none)" and the default state on a
%  fresh scenario is "(none)" (state.weather is empty). Picking a
%  supported type seeds a fresh WeatherRecord via
%  state.setWeatherType; switching BACK to "(none)" clears the record.
%
%  The Rain rate label text is re-written per type by
%  refreshWeatherPanel (see the rain_rate_mmhr reinterpretation note
%  in WeatherRecord.m). Same storage, different label:
%    rain  → "Rain rate"        mm/hr
%    snow  → "Snow rate (eq)"   mm/hr
%    fog   → "Density"          (5/15/30 = light/mod/dense)
%    icing → "Severity"         (5/15/30 = light/mod/severe)
%  The uieditfield itself reuses weatherRateField for all four.
%
%  Clutter × is hidden (Visible='off') for fog/icing because those
%  types omit clutter_multiplier on disk (see WeatherRecord.
%  emitsClutterField). Keeping the widget alive and just flipping
%  Visible is simpler than destroying/recreating it.
%
%  Sparkline: small uiaxes that draws the three storm profiles
%  (step/ramp/pulse) over the [storm_start..storm_end] window. It is
%  an explicit render artifact of refreshWeatherPanel; the user
%  cannot click on it (HitTest off).
    pnl = uipanel(parent, 'Title', 'Weather');
    state.weatherPanel = pnl;

    g = uigridlayout(pnl, [9 4]);
    g.RowHeight   = {22, 22, 22, 22, 22, 22, 40, 26, 4};
    g.ColumnWidth = {80, '1x', 80, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 2;
    g.ColumnSpacing = 4;

    % Row 1 — Type dropdown (includes "(none)")
    lblType = uilabel(g, 'Text', 'Type', 'HorizontalAlignment', 'right');
    lblType.Layout.Row = 1; lblType.Layout.Column = 1;
    state.weatherTypeDD = uidropdown(g, ...
        'Items', {'(none)','rain','snow','fog','icing'}, ...
        'Value', '(none)', ...
        'Tooltip', 'Weather type. (none) means no weather degradation.', ...
        'ValueChangedFcn', @(src, ~) onWeatherTypeChanged(src, state));
    state.weatherTypeDD.Layout.Row = 1;
    state.weatherTypeDD.Layout.Column = [2 4];

    % Row 2 — Description
    lblDesc = uilabel(g, 'Text', 'Description', 'HorizontalAlignment', 'right');
    lblDesc.Layout.Row = 2; lblDesc.Layout.Column = 1;
    state.weatherDescField = uieditfield(g, 'text', ...
        'Value', '', ...
        'Tooltip', 'Free-form description. Loaded from disk for built-in types; editable.', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'description'));
    state.weatherDescField.Layout.Row = 2;
    state.weatherDescField.Layout.Column = [2 4];

    % Row 3 — Rate (label rewritten per-type by refreshWeatherPanel)
    state.weatherRateLabel = uilabel(g, 'Text', 'Rain rate', ...
        'HorizontalAlignment', 'right');
    state.weatherRateLabel.Layout.Row = 3; state.weatherRateLabel.Layout.Column = 1;
    % Limits-BEFORE-Value is required by R2025b uieditfield (otherwise
    % a subsequent Limits update would silently clip a valid initial
    % Value). refreshWeatherPanel also sets Limits before Value per type.
    state.weatherRateField = uieditfield(g, 'numeric', ...
        'Limits', [0 200], ...
        'Value', 16, ...
        'Tooltip', 'Rate / density / severity (per-type meaning — see label).', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'rainRateMmhr'));
    state.weatherRateField.Layout.Row = 3;
    state.weatherRateField.Layout.Column = 2;
    lblRateUnits = uilabel(g, 'Text', 'mm/hr (or as labeled)', ...
        'FontColor', [0.45 0.45 0.45], 'FontSize', 11);
    lblRateUnits.Layout.Row = 3; lblRateUnits.Layout.Column = [3 4];

    % Row 4 — Storm window start/end
    lblStart = uilabel(g, 'Text', 'Storm start', 'HorizontalAlignment', 'right');
    lblStart.Layout.Row = 4; lblStart.Layout.Column = 1;
    state.weatherStormStartField = uieditfield(g, 'numeric', ...
        'Limits', [0 100000], ...
        'Value', 50, ...
        'Tooltip', 'Storm onset in scenario seconds.', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'stormStartS'));
    state.weatherStormStartField.Layout.Row = 4;
    state.weatherStormStartField.Layout.Column = 2;
    lblEnd = uilabel(g, 'Text', 'Storm end', 'HorizontalAlignment', 'right');
    lblEnd.Layout.Row = 4; lblEnd.Layout.Column = 3;
    state.weatherStormEndField = uieditfield(g, 'numeric', ...
        'Limits', [0 100000], ...
        'Value', 130, ...
        'Tooltip', 'Storm end in scenario seconds. Must exceed start.', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'stormEndS'));
    state.weatherStormEndField.Layout.Row = 4;
    state.weatherStormEndField.Layout.Column = 4;

    % Row 5 — Profile
    lblProf = uilabel(g, 'Text', 'Profile', 'HorizontalAlignment', 'right');
    lblProf.Layout.Row = 5; lblProf.Layout.Column = 1;
    state.weatherProfileDD = uidropdown(g, ...
        'Items', {'step','ramp','pulse'}, ...
        'Value', 'step', ...
        'Tooltip', 'Storm temporal profile: step (binary), ramp (triangle), pulse (first 20%).', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'activeType'));
    state.weatherProfileDD.Layout.Row = 5;
    state.weatherProfileDD.Layout.Column = [2 4];

    % Row 6 — pd_floor and clutter_multiplier
    lblPd = uilabel(g, 'Text', 'Pd floor', 'HorizontalAlignment', 'right');
    lblPd.Layout.Row = 6; lblPd.Layout.Column = 1;
    state.weatherPdFloorField = uieditfield(g, 'numeric', ...
        'Limits', [0 1], ...
        'Value', 0.15, ...
        'Tooltip', 'Minimum detection probability during the storm window.', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'pdFloor'));
    state.weatherPdFloorField.Layout.Row = 6;
    state.weatherPdFloorField.Layout.Column = 2;
    state.weatherClutterMultLabel = uilabel(g, 'Text', 'Clutter ×', ...
        'HorizontalAlignment', 'right');
    state.weatherClutterMultLabel.Layout.Row = 6;
    state.weatherClutterMultLabel.Layout.Column = 3;
    state.weatherClutterMultField = uieditfield(g, 'numeric', ...
        'Limits', [0 10], ...
        'Value', 1.0, ...
        'Tooltip', 'Multiplier on ground-clutter density during the storm (rain/snow only).', ...
        'ValueChangedFcn', @(src, ~) onWeatherFieldChanged(src, state, 'clutterMultiplier'));
    state.weatherClutterMultField.Layout.Row = 6;
    state.weatherClutterMultField.Layout.Column = 4;

    % Row 7 — sparkline
    state.weatherStormSparkline = uiaxes(g);
    state.weatherStormSparkline.Layout.Row = 7;
    state.weatherStormSparkline.Layout.Column = [1 4];
    state.weatherStormSparkline.XTick = [];
    state.weatherStormSparkline.YTick = [];
    state.weatherStormSparkline.Box = 'on';
    state.weatherStormSparkline.XColor = [0.55 0.55 0.55];
    state.weatherStormSparkline.YColor = [0.55 0.55 0.55];
    disableDefaultInteractivity(state.weatherStormSparkline);
    state.weatherStormSparkline.Toolbar.Visible = 'off';
    state.weatherStormSparkline.HitTest = 'off';
    state.weatherStormSparkline.PickableParts = 'none';

    % Row 8 — Load + Save buttons (v3.5 step 4c added Save)
    state.weatherLoadBtn = uibutton(g, 'push', 'Text', 'Load from file…', ...
        'Tooltip', 'Load a weather definition from config/weather/. Replaces the current weather.', ...
        'ButtonPushedFcn', @(~, ~) onLoadWeather(state));
    state.weatherLoadBtn.Layout.Row = 8;
    state.weatherLoadBtn.Layout.Column = [1 2];

    % Save weather lives next to Load per v3.5 step 4c "context-aware
    % Save" (Option A: in-panel save buttons). Disabled while
    % state.weather is empty (refreshWeatherPanel handles this).
    state.weatherSaveBtn = uibutton(g, 'push', 'Text', 'Save to file…', ...
        'Tooltip', 'Save the current weather configuration to a JSON file.', ...
        'ButtonPushedFcn', @(~, ~) onSaveWeather(state));
    state.weatherSaveBtn.Layout.Row = 8;
    state.weatherSaveBtn.Layout.Column = [3 4];
end


function buildSelectionPanel(parent, state)
    pnl = uipanel(parent, 'Title', 'Selected waypoint');
    state.selectedPanel = pnl;

    g = uigridlayout(pnl, [7 2]);
    g.RowHeight   = repmat({24}, 1, 7);
    g.ColumnWidth = {120, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;

    uilabel(g, 'Text', 'Index');
    state.selLabelIndex = uilabel(g, 'Text', '—');

    uilabel(g, 'Text', 'East x (m)');
    state.selFieldX = uieditfield(g, 'numeric', 'Value', 0, 'Enable', 'off', ...
        'ValueChangedFcn', @(src, ~) onSelFieldChanged(src, state, 'x'));

    uilabel(g, 'Text', 'North y (m)');
    state.selFieldY = uieditfield(g, 'numeric', 'Value', 0, 'Enable', 'off', ...
        'ValueChangedFcn', @(src, ~) onSelFieldChanged(src, state, 'y'));

    uilabel(g, 'Text', 'Altitude (m)');
    state.selFieldAlt = uieditfield(g, 'numeric', 'Value', 0, 'Enable', 'off', ...
        'Limits', [0 30000], ...
        'ValueChangedFcn', @(src, ~) onSelFieldChanged(src, state, 'altitude'));

    uilabel(g, 'Text', 'Leg speed (km/h)');
    state.selFieldSpeed = uieditfield(g, 'numeric', 'Value', state.defaultSpeedKmh, 'Enable', 'off', ...
        'Limits', [1 5000], ...
        'ValueChangedFcn', @(src, ~) onSelFieldChanged(src, state, 'speed'));

    uilabel(g, 'Text', 'Time (s)');
    state.selFieldTime = uieditfield(g, 'numeric', 'Value', 0, ...
        'Enable', 'off', 'Editable', 'off');  % readonly — auto timing

    % Delete + Insert-after side-by-side on the last row.
    state.selBtnDelete = uibutton(g, 'push', 'Text', 'Delete', ...
        'BackgroundColor', [0.85 0.30 0.30], 'FontColor', 'white', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @(~, ~) onDeleteSelected(state));
    state.selBtnInsertAfter = uibutton(g, 'push', 'Text', 'Insert after', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @(~, ~) onInsertAfterSelected(state));
end


function buildFilePanel(parent, state)
%buildFilePanel  Scenario-level file actions (v3.5 step 4c shrink).
%
%  Pre-v3.5 this panel hosted EVERYTHING file-related: target Load/
%  Export, sensor Load, reference overlay management, target Clear,
%  AND the full-scenario Open/Export. Step 4c moved each operation
%  into its respective mode-specific panel (Targets / Sensors /
%  Terrain / Weather), leaving this panel with just the two scenario-
%  level commands that genuinely don't belong to a single mode.
%
%  Layout:
%    [ Open Scenario… ] [ Export Scenario ]
%
%  WHY KEEP IT AS A PANEL
%    A two-button strip without a panel border would feel orphaned in
%    the always-visible bottom row of the sidebar. The "File" title
%    keeps the operation class explicit and signals "these run
%    regardless of which edit mode you're in".
    pnl = uipanel(parent, 'Title', 'File');
    g = uigridlayout(pnl, [1 2]);
    g.RowHeight   = {28};
    g.ColumnWidth = {'1x', '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;
    g.ColumnSpacing = 4;

    openScenarioBtn = uibutton(g, 'push', 'Text', 'Open Scenario…', ...
        'Tooltip', 'Open a full scenario from a run file (replaces sensors + targets + environment).', ...
        'ButtonPushedFcn', @(~, ~) onOpenScenario(state));
    openScenarioBtn.Layout.Row    = 1;
    openScenarioBtn.Layout.Column = 1;

    exportScenarioBtn = uibutton(g, 'push', 'Text', 'Export Scenario', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.20 0.50 0.30], 'FontColor', 'white', ...
        'Tooltip', 'Save the full scenario bundle: sensors, targets, terrain, weather, and run file.', ...
        'ButtonPushedFcn', @(~, ~) onExportScenario(state));
    exportScenarioBtn.Layout.Row    = 1;
    exportScenarioBtn.Layout.Column = 2;
end


function buildUndoPanel(parent, state)
    pnl = uipanel(parent, 'Title', 'History');
    g = uigridlayout(pnl, [1 2]);
    g.RowHeight   = {28};
    g.ColumnWidth = {'1x', '1x'};
    g.Padding     = [6 6 6 6];
    g.ColumnSpacing = 4;

    uibutton(g, 'push', 'Text', 'Undo (Ctrl+Z)', ...
        'ButtonPushedFcn', @(~, ~) onUndo(state));
    uibutton(g, 'push', 'Text', 'Redo (Ctrl+Y)', ...
        'ButtonPushedFcn', @(~, ~) onRedo(state));
end


function buildHelpPanel(parent)
    pnl = uipanel(parent, 'Title', 'Help');
    g = uigridlayout(pnl, [1 1]);
    g.RowHeight   = {'fit'};
    g.ColumnWidth = {'1x'};
    g.Padding     = [6 6 6 6];

    uilabel(g, 'Text', ...
        ['Click: add  |  Drag: move  |  Shift+click segment: insert' newline ...
         'Del: remove  |  Ctrl+Z/Y: undo/redo  |  Esc: clear selection' newline ...
         'V: 2D/3D  |  Arrows: nudge 100 m (Shift = 1 km)' newline ...
         'PgUp/PgDn: altitude ±100 m (Shift = ±1 km)' newline ...
         'Wheel: zoom  |  Middle-drag: pan (2D; 3D via built-in)'], ...
        'WordWrap', 'on', 'FontColor', [0.3 0.3 0.3], 'FontSize', 11);
end


%% ========================================================================
%  CLICK + DRAG
%% ========================================================================

function onAxesClick(~, evt, state)
%onAxesClick  Decide between add / select / insert-on-segment based on
%             click position and modifiers.
%
%  M3.3: 3D view is "view-only" — click-to-add and drag are disabled
%  because the 3D IntersectionPoint isn't a meaningful (x, y) without
%  an assumed altitude. Clicking in 3D still performs hit-test selection
%  (using x, y only, ignoring z) so the sidebar can populate and the
%  user can edit via the sidebar fields if they want.
    try
        pt = evt.IntersectionPoint;
    catch
        cp = state.ax.CurrentPoint;
        pt = cp(1, 1:3);
    end
    if isempty(pt) || numel(pt) < 2
        return;
    end

    isRightClick  = isprop(evt, 'Button') && evt.Button == 3;
    isMiddleClick = isprop(evt, 'Button') && evt.Button == 2;
    sel = getSelectionType(state.fig);
    isShift = strcmpi(sel, 'extend');  % Shift+left or middle-click
    in3D = state.viewMode == "3d";

    % Middle-click = pan-begin (2D only). In 3D the axes' built-in
    % interactivity owns middle-click-drag, so we just let it through.
    %
    % v3.5 §5f post-mortem: tried five iterations of custom 3D pan
    % (camera translation, axis-limit shift with view projection,
    % auto-mode forcing, camdolly) and each had bugs around either
    % CameraPosition mutation from uifigure's built-in 3D interactions
    % or wheel-zoom reversal afterward. Reverted to letting MATLAB's
    % built-in pan tool own middle-click in 3D. Custom shortcuts
    % (arrows = rotate, wheel = zoom, R = reset) cover the rest;
    % the pan parity nice-to-have is parked as post-demo work.
    if isMiddleClick && ~in3D
        state.panActive     = true;
        state.panStartFigPt = state.fig.CurrentPoint;
        state.panStartXLim  = state.ax.XLim;
        state.panStartYLim  = state.ax.YLim;
        setStatus(state, 'Panning — release middle button to stop.');
        return;
    end

    % ── v3.5 §5c.3 — polygon-edit click branch ─────────────────
    %  Active-edit clicks come BEFORE sensor / waypoint dispatch so the
    %  polygon-edit overlay owns the map while the user is drawing.
    %  Right-click is consumed (no context menu in edit mode); left-click
    %  on a 'normal' selection appends a vertex; left-click on an 'open'
    %  selection (double-click) commits.
    %
    %  3D guard: in 3D view we don't have a meaningful (x, y) for vertex
    %  placement, so just post a status nag and bail. The lockdown helper
    %  doesn't touch the view-mode toggle (which is a Scenario-panel
    %  control hidden in env mode anyway), so this case is rare — only
    %  reachable if the user enters polygon edit then somehow flips into
    %  3D via keyboard 'V' shortcut.
    if state.polygonEditActive
        if isRightClick
            return;   % swallow right-click; no context menu while editing
        end
        if in3D
            setStatus(state, 'Switch to 2D to add polygon vertices (3D is view-only).');
            return;
        end
        isOpen = strcmpi(sel, 'open');
        if isOpen
            % Double-click → commit. The first click of the double already
            % appended a vertex via the 'normal' branch below in a prior
            % onAxesClick invocation; no need to re-append here.
            tryCommitPolygonEdit(state);
            return;
        end
        % Near-duplicate guard: skip clicks that land within ~10 px of
        % the most recent draft vertex (prevents double-vertex from a
        % jittery click+release). Threshold is in WORLD meters, sized
        % from the current axes pixel-per-meter.
        if size(state.polygonEditDraft, 1) >= 1
            last = state.polygonEditDraft(end, :);
            xs = state.ax.XLim;
            axPix = getpixelposition(state.ax, true);
            if axPix(3) > 0
                metersPerPx = (xs(2) - xs(1)) / axPix(3);
                jitterM = 10 * metersPerPx;
                if hypot(pt(1) - last(1), pt(2) - last(2)) < jitterM
                    return;   % silently swallow the dupe click
                end
            end
        end
        state.appendPolygonDraftVertex(pt(1), pt(2));
        trackbench.editor.drawMap(state);
        nv = size(state.polygonEditDraft, 1);
        if nv < 3
            setStatus(state, sprintf( ...
                'Polygon vertex #%d at (%.0f, %.0f) m. Need %d more before commit.', ...
                nv, pt(1), pt(2), 3 - nv));
        else
            setStatus(state, sprintf( ...
                'Polygon vertex #%d at (%.0f, %.0f) m. Enter to commit, Esc to cancel.', ...
                nv, pt(1), pt(2)));
        end
        return;
    end

    % ── v3.5 §5c.6 — Region vertex drag + edge shift-click insert ──
    %  Priority: AFTER polygon-edit (append mode wins), BEFORE all other
    %  branches. "Any visible region" UX (per design Q1) — a click on a
    %  region polygon vertex grabs that vertex regardless of editMode /
    %  envSubMode. Auto-promotes the region to active so panel state +
    %  drawMap visual feedback + undo all stay consistent.
    %
    %  3D guard: vertex drag is 2D-only (matches waypoint/sensor drag).
    %  In 3D, vertex clicks fall through to the existing branches and
    %  end up at the 3D "switch to 2D" status nag.
    %
    %  Read-only guard: regions whose inner record is readOnly cannot
    %  be edited. The pick still hits visually, but we post a status
    %  nag and don't start a drag (or insert).
    if ~in3D && ~isMiddleClick
        [vKind, vReg, vVtx] = state.findRegionVertexAt(pt(1), pt(2));
        if vVtx > 0 && ~isRightClick
            if isRegionReadOnly(state, vKind, vReg)
                setStatus(state, sprintf( ...
                    '%s region is read-only — cannot edit vertices.', vKind));
                return;
            end
            beginRegionVertexDrag(state, vKind, vReg, vVtx);
            return;
        end
        if isShift && ~isRightClick && vVtx == 0
            [eKind, eReg, eIdx, eProj] = state.findRegionEdgeAt(pt(1), pt(2));
            if eIdx > 0
                if isRegionReadOnly(state, eKind, eReg)
                    setStatus(state, sprintf( ...
                        '%s region is read-only — cannot insert vertices.', eKind));
                    return;
                end
                insertRegionVertexFromClick(state, eKind, eReg, eIdx, eProj);
                return;
            end
        end
        % No vertex or edge hit — if the user had a region vertex
        % selected (for Delete-key targeting), clear it on a left-click
        % miss so Delete can't target a stale vertex they think they
        % deselected. Right-click and middle-click leave selection alone.
        if ~isRightClick && state.selectedVertexIdx > 0
            state.selectedRegionKind = "";
            state.selectedRegionIdx = 0;
            state.selectedVertexIdx = 0;
            % No drawMap here — selected-vertex highlight is added in
            % 5c.6.6, so there's nothing visual to refresh yet.
        end
    end

    % ── v3.5 §5c.3 fix — environment-mode map-click guard ─────────
    %  Pre-fix, env mode had no click branch, so map clicks fell through
    %  to the targets-mode waypoint logic and added waypoints to the
    %  active target. The Targets/Selection panels aren't even visible
    %  in env mode, so the user couldn't see what was happening — just
    %  scattered yellow dots accumulating on the map.
    %
    %  Now: a left-click in env mode just posts the click coordinates
    %  with a hint about how to actually interact (pick Regions sub-mode
    %  and click "Edit polygon…"). Right-click is swallowed (no context
    %  menu in env mode — the menu's actions all target waypoints).
    %
    %  Note: the polygon-edit branch above already returned for the
    %  active-edit case, so we don't need to re-check polygonEditActive
    %  here.
    if state.editMode == "environment"
        if isRightClick
            return;
        end
        setStatus(state, sprintf( ...
            ['x=%.0f m, y=%.0f m  (Environment mode — pick Regions sub-mode ' ...
             'and click "Edit polygon…" to draw on the map.)'], ...
            pt(1), pt(2)));
        return;
    end

    % ── M6 §3.5 — sensor-mode click routing ─────────────────────────
    %  Priority 1 (Place-on-map pending): a previous Place-on-map press
    %  armed a one-shot teleport. Consume the click and return to normal.
    %  Priority 2 (click-to-select / drag-begin): if the click lands
    %  within hit-radius of a sensor marker, select it and — in 2D —
    %  begin a drag. Otherwise fall through to the targets logic so that
    %  sensor mode doesn't swallow clicks the user intends for waypoints
    %  on visible target paths. (Targets are still rendered; users just
    %  won't be able to MOVE a waypoint while in sensors mode because the
    %  targets-side drag branch requires editMode=="targets" — enforced
    %  in the hit branch below.)
    if state.editMode == "sensors" && ~isRightClick && ~isMiddleClick
        if state.sensorPlacePending
            if ~state.hasActiveSensor()
                state.sensorPlacePending = false;
                setStatus(state, 'Place cancelled — no active sensor.');
                return;
            end
            if state.activeSensorIsReadOnly()
                state.sensorPlacePending = false;
                setStatus(state, 'Place cancelled — sensor is read-only.');
                return;
            end
            state.moveActiveSensorTo(pt(1), pt(2), true);
            state.sensorPlacePending = false;
            trackbench.editor.drawMap(state);
            refreshSensorParamsPanel(state);
            sr = state.activeSensor();
            setStatus(state, sprintf('Placed %s at (%.0f, %.0f) m.', ...
                sr.sensorName, pt(1), pt(2)));
            return;
        end

        sensorHit = state.findSensorAt(pt(1), pt(2));
        if sensorHit > 0
            state.setActiveSensorIdx(sensorHit);
            refreshSensorsDropdown(state);
            refreshSensorParamsPanel(state);
            if ~in3D && ~state.activeSensorIsReadOnly()
                state.sensorDragActive = true;
                state.pushUndo();
                sr = state.sensors(sensorHit);
                state.sensorDragStart = [sr.positionEastM, sr.positionNorthM];
            end
            trackbench.editor.drawMap(state);
            sr = state.sensors(sensorHit);
            if in3D
                setStatus(state, sprintf( ...
                    'Selected sensor %s (3D view-only; switch to 2D to drag).', ...
                    sr.sensorName));
            elseif state.activeSensorIsReadOnly()
                setStatus(state, sprintf( ...
                    'Selected sensor %s (read-only; duplicate to edit).', ...
                    sr.sensorName));
            else
                setStatus(state, sprintf('Selected sensor %s (drag to move).', ...
                    sr.sensorName));
            end
            return;
        end
        % No sensor hit — don't add/insert waypoints while in sensors
        % mode. Just report the click coordinates and stop.
        setStatus(state, sprintf( ...
            'x=%.0f m, y=%.0f m (sensors mode; no sensor at click)', ...
            pt(1), pt(2)));
        return;
    end

    % Hit-test against existing waypoints first.
    hitIdx = state.findWaypointAt(pt(1), pt(2));

    if hitIdx > 0
        state.selectedIndex = hitIdx;
        if ~in3D
            % Begin drag — motion routed through WindowButtonMotionFcn.
            state.dragActive = true;
            state.pushUndo();
            state.dragStartWP = state.waypoints(hitIdx, :);
        end
        trackbench.editor.drawMap(state);
        refreshSelectionPanel(state);
        if in3D
            setStatus(state, sprintf( ...
                'Selected waypoint #%d (3D view-only; switch to 2D to edit).', ...
                hitIdx));
        else
            setStatus(state, sprintf('Selected waypoint #%d (drag to move).', hitIdx));
        end
        return;
    end

    if isRightClick
        % Right-click on empty space just opens the context menu.
        return;
    end

    if in3D
        % View-only mode: don't modify the path from empty-space clicks.
        setStatus(state, ...
            'Switch to 2D to add or insert waypoints (3D is view-only).');
        return;
    end

    if isShift
        % Shift+click: if close to a segment, insert there; else ignore.
        [segIdx, projXY] = state.findSegmentAt(pt(1), pt(2));
        if segIdx > 0
            state.insertAfter(segIdx, projXY(1), projXY(2));
            trackbench.editor.drawMap(state);
            updateWaypointCount(state);
            refreshSelectionPanel(state);
            setStatus(state, sprintf('Inserted waypoint at (%.0f, %.0f) m on segment %d-%d.', ...
                projXY(1), projXY(2), segIdx, segIdx+1));
            return;
        end
        setStatus(state, 'Shift+click missed any path segment (try closer to a line).');
        return;
    end

    % Plain click on empty space: append a waypoint.
    state.addWaypoint(pt(1), pt(2));
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Added waypoint #%d at (%.0f, %.0f) m.', ...
        state.count(), pt(1), pt(2)));
end


function onMouseMove(~, state)
%onMouseMove  Three behaviors, in priority order:
%    1. While dragging in 2D: update the selected waypoint's position
%       live from the axes CurrentPoint. Drag continues even if the
%       cursor briefly leaves the axes.
%    2. Cursor not over the map axes (e.g. moved to the sidebar): do
%       nothing. Without this guard the stale ax.CurrentPoint can post
%       spurious "near waypoint" messages while the user edits fields.
%    3. Hovering the map (M3.1 tooltip): always post the cursor's
%       east/north to the status bar. If within the zoom-aware hit
%       radius of a waypoint, append index / altitude / speed / time.
%
%  Hover tooltip is 2D-only — in 3D ax.CurrentPoint is the near-plane
%  ray intersection, which is not a meaningful ground-plane (x, y).
    ax = state.ax;
    if ~isgraphics(ax); return; end
    cp = ax.CurrentPoint;
    if isempty(cp); return; end
    x = cp(1, 1);
    y = cp(1, 2);

    % ── 0. Pan branch (middle-click drag, 2D only) ───────────────────
    % Pan is computed in FIGURE pixel space (not world meters) because
    % ax.CurrentPoint during a pan gives us the point in the *current*
    % (moving) limits — which would feed back on itself. fig.CurrentPoint
    % is stable pixel coordinates, so we convert pixel delta to world
    % delta against the pan-start snapshot of XLim/YLim.
    if state.panActive && state.viewMode ~= "3d"
        if isgraphics(state.fig)
            figPt = state.fig.CurrentPoint;
            axPix = getpixelposition(ax, true);
            if axPix(3) > 0 && axPix(4) > 0 && ~isempty(state.panStartFigPt)
                dxPix = figPt(1) - state.panStartFigPt(1);
                dyPix = figPt(2) - state.panStartFigPt(2);
                xSpan = state.panStartXLim(2) - state.panStartXLim(1);
                ySpan = state.panStartYLim(2) - state.panStartYLim(1);
                dxWorld = dxPix * xSpan / axPix(3);
                dyWorld = dyPix * ySpan / axPix(4);
                % Drag-the-map direction: cursor-right moves content right,
                % so XLim shifts LEFT by dxWorld.
                ax.XLim = state.panStartXLim - dxWorld;
                ax.YLim = state.panStartYLim - dyWorld;
                drawnow limitrate;
            end
        end
        return;
    end

    % ── v3.5 §5c.6 — Region vertex drag live update (2D only) ────────
    %  Mirrors the waypoint drag branch below. commit=false: pushUndo
    %  was already taken in beginRegionVertexDrag, so we don't burn an
    %  undo slot per cursor frame. moveRegionVertex's no-op for
    %  out-of-range indices means a stale drag survives an undo /
    %  region-delete-during-drag without throwing.
    if state.vertexDragActive && state.viewMode ~= "3d"
        state.moveRegionVertex(state.vertexDragKind, ...
                               state.vertexDragRegionIdx, ...
                               state.vertexDragVertexIdx, ...
                               x, y, false);
        trackbench.editor.drawMap(state);
        return;
    end

    % ── 1. Drag branch (2D only) ─────────────────────────────────────
    if state.dragActive && state.selectedIndex >= 1 && state.viewMode ~= "3d"
        state.moveSelectedTo(x, y, false);   % commit=false: no undo per frame
        trackbench.editor.drawMap(state);
        refreshSelectionPanel(state);
        return;
    end

    % ── 1b. Sensor drag branch (M6 §3.5C, 2D only) ───────────────────
    %  Parallel to the waypoint drag above. Separate state field so
    %  the dispatch is unambiguous regardless of editMode. pushUndo was
    %  already taken at drag-start in onAxesClick, so we pass commit=false
    %  to avoid stacking an undo entry per cursor frame.
    if state.sensorDragActive && state.hasActiveSensor() && state.viewMode ~= "3d"
        state.moveActiveSensorTo(x, y, false);
        trackbench.editor.drawMap(state);
        refreshSensorParamsPanel(state);
        return;
    end

    % ── 2. Bounds guard ─────────────────────────────────────────────
    % Only post hover status when the cursor is actually over the map
    % axes. fig.CurrentPoint is in figure pixels; getpixelposition with
    % the 'true' flag returns [left bottom width height] in the same
    % coordinate system.
    if ~isgraphics(state.fig); return; end
    figPt = state.fig.CurrentPoint;
    axPix = getpixelposition(ax, true);
    inAxes = figPt(1) >= axPix(1) && figPt(1) <= axPix(1) + axPix(3) && ...
             figPt(2) >= axPix(2) && figPt(2) <= axPix(2) + axPix(4);
    if ~inAxes
        return;
    end

    % ── v3.5 §5c.6 — Region vertex / edge hover state ((5+6 combined) ──
    %  Polled per mousemove (2D only — 3D screen-projected coords don't
    %  map cleanly to polygon hit-tests). Cheap hit-test (just distance
    %  math), then compare against previous hover state and only redraw
    %  if changed. Shift-tracking via fig.CurrentModifier means shift-
    %  press-without-mousemove won't update the edge highlight instantly
    %  — user has to nudge the cursor. Acceptable trade-off vs adding a
    %  full WindowKeyReleaseFcn (noted as polish in CHECKPOINT).
    if state.viewMode ~= "3d"
        shiftHeld = false;
        if isgraphics(state.fig)
            shiftHeld = any(strcmp(state.fig.CurrentModifier, 'shift'));
        end
        [newHoverKind, newHoverRegion, newHoverVertex] = state.findRegionVertexAt(x, y);
        newHoverEdge = 0;
        if shiftHeld && newHoverVertex == 0
            [eKind, eReg, eIdx, ~] = state.findRegionEdgeAt(x, y);
            if eIdx > 0
                newHoverKind   = eKind;
                newHoverRegion = eReg;
                newHoverEdge   = eIdx;
            end
        end
        hoverChanged = (newHoverKind   ~= state.hoverRegionKind) || ...
                       (newHoverRegion ~= state.hoverRegionIdx) || ...
                       (newHoverVertex ~= state.hoverVertexIdx) || ...
                       (newHoverEdge   ~= state.hoverEdgeIdx);
        if hoverChanged
            state.hoverRegionKind = newHoverKind;
            state.hoverRegionIdx  = newHoverRegion;
            state.hoverVertexIdx  = newHoverVertex;
            state.hoverEdgeIdx    = newHoverEdge;
            trackbench.editor.drawMap(state);
        end
    end

    % Hover tooltip is 2D-only — skip in 3D where (x, y) isn't meaningful.
    if state.viewMode == "3d"
        return;
    end

    % ── 3. Hover tooltip ─────────────────────────────────────────────
    hoverIdx = state.findWaypointAt(x, y);
    if hoverIdx >= 1
        row = state.waypoints(hoverIdx, :);
        setStatus(state, sprintf( ...
            'x=%.0f m, y=%.0f m  (near wp #%d: alt=%.0f m, speed=%.0f km/h, t=%.1f s)', ...
            x, y, hoverIdx, row(3), row(5), row(4)));
    else
        setStatus(state, sprintf('x=%.0f m, y=%.0f m', x, y));
    end
    state.hoverIndex = hoverIdx;
end


function onMouseUp(~, state)
%onMouseUp  Terminate drag OR middle-click pan. Drag-end keeps the
%           single pushUndo we did on drag start; pan-end does NOT
%           create an undo entry (pan is a view change, not a data edit).
    if state.vertexDragActive
        state.vertexDragActive = false;
        rname = getRegionName(state, state.vertexDragKind, state.vertexDragRegionIdx);
        vtx   = state.vertexDragVertexIdx;
        kind  = state.vertexDragKind;
        setStatus(state, sprintf('Moved %s region "%s" vertex #%d.', kind, rname, vtx));
        state.vertexDragKind      = "";
        state.vertexDragRegionIdx = 0;
        state.vertexDragVertexIdx = 0;
        % v3.5 §5c.6 — refresh region panels in case auto-promote moved
        % the active region. The dropdown shows "(active)" next to the
        % region the user just grabbed; without this refresh it would
        % stay stuck on whatever was active before mousedown.
        refreshTerrainRegionsPanel(state);
        refreshWeatherRegionsPanel(state);
        % A final drawMap to flush any cache-skip-during-drag state in
        % the heightmap renderer (deferred optimization — see CHECKPOINT
        % "During waypoint drag, drawMap fires repeatedly"). No-op today
        % since the heightmap cache is invalidated per-vertex hash, but
        % belt-and-suspenders for the future skip-during-drag tweak.
        trackbench.editor.drawMap(state);
    end
    if state.dragActive
        state.dragActive = false;
        state.dragStartWP = [];
        setStatus(state, sprintf('Moved waypoint #%d.', state.selectedIndex));
    end
    if state.sensorDragActive
        state.sensorDragActive = false;
        state.sensorDragStart  = [];
        if state.hasActiveSensor()
            sr = state.activeSensor();
            setStatus(state, sprintf('Moved sensor %s to (%.0f, %.0f) m.', ...
                sr.sensorName, sr.positionEastM, sr.positionNorthM));
        end
    end
    if state.panActive
        state.panActive     = false;
        state.panStartFigPt = [];
        state.panStartXLim  = [];
        state.panStartYLim  = [];
        % v3.5 §5f — 3D pan camera-state cleanup
        state.panStartCamPos    = [];
        state.panStartCamTarget = [];
        state.panStartCamUp     = [];
        state.panStartCamVA     = 0;
        % Full redraw once at pan end so the scale bar label and minor
        % grid catch up to the new visible span. During the drag we
        % skipped drawMap for smoothness.
        trackbench.editor.drawMap(state);
        setStatus(state, 'Pan complete.');
    end
end


%% ========================================================================
%  KEYBOARD
%% ========================================================================

function onKeyPress(~, evt, state)
%onKeyPress  Figure-level keyboard dispatcher.
%
%  Existing M1/M2 bindings: Delete/Backspace (delete selected), Escape
%  (clear selection), Ctrl+Z / Ctrl+Y (undo / redo).
%
%  Shortcuts pass (2026-04-16) adds:
%    - V                    toggle 2D / 3D view
%    - Arrow keys           nudge selected waypoint  100 m (X or Y)
%    - Shift + Arrow        nudge selected waypoint  1000 m (X or Y)
%    - PageUp / PageDown    nudge selected altitude  ±100 m  (clamped ≥ 0)
%    - Shift + PageUp/Down  nudge selected altitude  ±1000 m (clamped ≥ 0)
%
%  FOCUS GUARD
%    These shortcuts should NOT fire while the user is typing in a
%    sidebar uieditfield (target name, default speed, x/y/alt editors,
%    etc.). editingTextField() checks fig.CurrentObject and bails out
%    for V / arrow / page keys. The existing Delete/Escape/Ctrl bindings
%    are preserved unconditionally — those are universal and field
%    interaction won't normally consume them (Delete in an edit field
%    already removes characters, which MATLAB handles before the
%    figure KeyPressFcn runs).
    ctrl = any(strcmp(evt.Modifier, 'control')) || any(strcmp(evt.Modifier, 'command'));
    shift = any(strcmp(evt.Modifier, 'shift'));
    inSensors = (state.editMode == "sensors");

    % Universal (no focus guard) — safe inside or outside text fields.
    switch evt.Key
        case {'delete', 'backspace'}
            % v3.5 §5c.3 — in polygon-edit mode, Delete/Backspace are
            % swallowed. Vertex deletion during APPEND is intentionally
            % left out: append mode is for laying down a fresh polygon
            % left-to-right, not editing back into it. Escape aborts;
            % Enter commits and gives the user the post-commit editing
            % tools (vertex drag / delete / insert from 5c.6).
            if state.polygonEditActive
                setStatus(state, ['Polygon edit (append) — Delete is disabled. ' ...
                                  'Esc to abort, Enter to commit then edit vertices.']);
                return;
            end
            % v3.5 §5c.6 — Region vertex delete. Priority over the
            % sensor / waypoint delete paths so the status message
            % posted by beginRegionVertexDrag ("...Del to remove")
            % stays a true promise. If both a region vertex AND a
            % waypoint are selected (possible across modes — see the
            % "mutual exclusion" comment on selectedRegionKind in
            % EditorState), region vertex wins. Click empty space
            % first to clear region selection and reroute Delete to
            % the waypoint.
            if state.selectedVertexIdx > 0
                handleRegionVertexDelete(state);
                return;
            end
            % M6 §3.5C — Delete in sensors mode routes through the
            % Delete-button confirm path so the user doesn't lose a
            % sensor from a stray keypress. In targets mode, the existing
            % waypoint-delete path.
            if inSensors
                onSensorsDelete(state);
            else
                onDeleteSelected(state);
            end
            return;
        case 'escape'
            % v3.5 §5c.3 — polygon-edit abort takes priority over every
            % other Escape branch. The stored polygon was never touched,
            % so this is a clean rollback (no undo entry).
            if state.polygonEditActive
                state.abortPolygonEdit();
                applyPolygonEditLockdown(state);   % releases the lock
                trackbench.editor.drawMap(state);
                setStatus(state, 'Polygon edit cancelled.');
                return;
            end
            % M6 §3.6C — Escape during an active sensor drag aborts the
            % drag: reverts the sensor position to its drag start, pops
            % the undo snapshot (so the aborted drag leaves no history),
            % and clears the drag state. Takes priority over the
            % Place-pending and selection-clear branches because an
            % in-flight drag is the most recent user action.
            if state.sensorDragActive
                if state.abortSensorDrag()
                    trackbench.editor.drawMap(state);
                    refreshSensorParamsPanel(state);
                    setStatus(state, 'Sensor drag cancelled.');
                end
                return;
            end
            % Escape cancels a pending Place-on-map AND clears the
            % current selection, regardless of mode. Covers the case
            % where the user armed Place, then decided not to.
            if state.sensorPlacePending
                state.sensorPlacePending = false;
                setStatus(state, 'Place cancelled.');
                return;
            end
            % v3.5 §5c.6 — Escape also clears region-vertex selection.
            % Pairs with the empty-click clear in onAxesClick so users
            % have two ways to bail out of an accidental vertex select.
            hadRegionVtx = (state.selectedVertexIdx > 0);
            state.selectedRegionKind = "";
            state.selectedRegionIdx  = 0;
            state.selectedVertexIdx  = 0;
            state.selectedIndex = 0;
            trackbench.editor.drawMap(state);
            refreshSelectionPanel(state);
            if hadRegionVtx
                setStatus(state, 'Region vertex selection cleared.');
            else
                setStatus(state, 'Selection cleared.');
            end
            return;
        case 'z'
            % v3.5 §5c.3 — Ctrl+Z is refused while polygon-edit is active.
            % Refusing rather than aborting (option B from design): less
            % surprising than discarding the user's draft on a
            % muscle-memory undo press. Vertex-by-vertex undo is 5c.6.
            if ctrl && state.polygonEditActive
                setStatus(state, ['Polygon edit active — Esc to cancel, Enter to commit. ' ...
                                  'Undo is refused mid-edit.']);
                return;
            end
            if ctrl; onUndo(state); end
            return;
        case 'y'
            if ctrl && state.polygonEditActive
                setStatus(state, ['Polygon edit active — Esc to cancel, Enter to commit. ' ...
                                  'Redo is refused mid-edit.']);
                return;
            end
            if ctrl; onRedo(state); end
            return;
        case 'return'
            % v3.5 §5c.3 — Enter commits a polygon edit. Outside polygon-
            % edit mode, Enter is unbound (matches pre-5c.3 behavior).
            if state.polygonEditActive
                tryCommitPolygonEdit(state);
                return;
            end
            return;
    end

    % Shortcut keys that conflict with text editing — bail if a field has focus.
    if editingTextField(state.fig)
        return;
    end

    % Nudge step sizes (spec: 100 m fine, 1 km coarse).
    stepM = 100;
    if shift; stepM = 1000; end

    % In sensors mode, arrow keys nudge the active sensor; everything
    % else (V / PageUp / PageDown) falls through to the targets-side
    % behavior because it's still useful (toggle view, per-waypoint alt
    % bump on the active target).
    if inSensors
        switch evt.Key
            case 'v'
                if ctrl || shift; return; end
                toggleViewModeViaKey(state);
                return;
            case 'leftarrow'
                nudgeSensorXY(state, -stepM, 0, stepM);
                return;
            case 'rightarrow'
                nudgeSensorXY(state,  stepM, 0, stepM);
                return;
            case 'uparrow'
                nudgeSensorXY(state, 0,  stepM, stepM);
                return;
            case 'downarrow'
                nudgeSensorXY(state, 0, -stepM, stepM);
                return;
        end
        % Fall through for any unhandled key — no-op.
        return;
    end

    switch evt.Key
        case 'v'
            if ctrl || shift; return; end  % leave Ctrl+V / Shift+V for future use
            toggleViewModeViaKey(state);
        case 'r'
            % v3.5 fix — "reset view" recovery shortcut. Forces the
            % next drawMap to take the firstDraw branch, which
            % re-autofits limits + aspect. Works in both 2D and 3D so
            % users have a consistent way out of any zoomed/panned/
            % rotated state.
            if ctrl || shift; return; end
            if state.viewMode == "3d"
                state.has3DViewState = false;
                trackbench.editor.drawMap(state);
                setStatus(state, '3D view reset to autofit.');
            else
                state.has2DViewState = false;
                trackbench.editor.drawMap(state);
                setStatus(state, '2D view reset to autofit.');
            end
        case 'leftarrow'
            if state.viewMode == "3d"
                rotate3DView(state, -5, 0, shift);
            else
                nudgeSelectedXY(state, -stepM, 0, stepM);
            end
        case 'rightarrow'
            if state.viewMode == "3d"
                rotate3DView(state, +5, 0, shift);
            else
                nudgeSelectedXY(state,  stepM, 0, stepM);
            end
        case 'uparrow'
            if state.viewMode == "3d"
                rotate3DView(state, 0, +5, shift);
            else
                nudgeSelectedXY(state, 0,  stepM, stepM);
            end
        case 'downarrow'
            if state.viewMode == "3d"
                rotate3DView(state, 0, -5, shift);
            else
                nudgeSelectedXY(state, 0, -stepM, stepM);
            end
        case 'pageup'
            bumpSelectedAltitude(state,  stepM);
        case 'pagedown'
            bumpSelectedAltitude(state, -stepM);
    end
end


function nudgeSensorXY(state, dx, dy, stepM)
%nudgeSensorXY  M6 §3.5C — keyboard nudge of the active sensor by
%               (dx, dy) metres. Each call is ONE undo entry (mirrors
%               nudgeSelectedXY's fine-granularity convention).
    if ~state.hasActiveSensor()
        setStatus(state, 'No active sensor — arrow keys need an active sensor.');
        return;
    end
    if state.activeSensorIsReadOnly()
        setStatus(state, 'Active sensor is read-only — duplicate to edit.');
        return;
    end
    state.nudgeActiveSensor(dx, dy);
    trackbench.editor.drawMap(state);
    refreshSensorParamsPanel(state);
    sr = state.activeSensor();
    setStatus(state, sprintf('Nudged sensor %s to (%.0f, %.0f) m [step %d m].', ...
        sr.sensorName, sr.positionEastM, sr.positionNorthM, stepM));
end


function tf = editingTextField(fig)
%editingTextField  True if the keyboard focus is on an edit field (so
%                  keyboard shortcuts should not fire — the user is typing).
%  Covers text uieditfield and numeric uieditfield. Defensive in case
%  CurrentObject is empty or deleted.
    tf = false;
    try
        co = fig.CurrentObject;
        if isempty(co) || ~isgraphics(co); return; end
        tf = isa(co, 'matlab.ui.control.EditField') || ...
             isa(co, 'matlab.ui.control.NumericEditField');
    catch
        tf = false;
    end
end


function toggleViewModeViaKey(state)
%toggleViewModeViaKey  V-key equivalent of clicking the view-mode toggle
%                      button. Routes through the button's
%                      ValueChangedFcn by flipping its Value so the
%                      button UI updates in sync.
    if ~isgraphics(state.viewModeBtn); return; end
    state.viewModeBtn.Value = ~state.viewModeBtn.Value;
    onViewModeChanged(state.viewModeBtn, state);
end


function nudgeSelectedXY(state, dx, dy, stepM)
%nudgeSelectedXY  Move the selected waypoint by (dx, dy) world meters.
%                 Each call is ONE undo entry (option 1 in the design
%                 notes — fine granularity for precision editing).
    idx = state.selectedIndex;
    if idx < 1 || idx > state.count()
        setStatus(state, 'No waypoint selected — arrow keys need a selection.');
        return;
    end
    newX = state.waypoints(idx, 1) + dx;
    newY = state.waypoints(idx, 2) + dy;
    state.pushUndo();
    state.waypoints(idx, 1) = newX;
    state.waypoints(idx, 2) = newY;
    state.recomputeTimes();
    state.isDirty = true;
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Nudged wp #%d to (%.0f, %.0f) m [step %d m].', ...
        idx, newX, newY, stepM));
end


function bumpSelectedAltitude(state, dAlt)
%bumpSelectedAltitude  Change the selected waypoint's altitude by dAlt
%                      meters. Clamps at 0 (matches setWaypointProperty).
    idx = state.selectedIndex;
    if idx < 1 || idx > state.count()
        setStatus(state, 'No waypoint selected — PageUp/Down needs a selection.');
        return;
    end
    oldAlt = state.waypoints(idx, 3);
    newAlt = max(0, oldAlt + dAlt);
    state.pushUndo();
    state.waypoints(idx, 3) = newAlt;
    state.recomputeTimes();
    state.isDirty = true;
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
    if dAlt < 0 && newAlt == 0 && oldAlt + dAlt < 0
        setStatus(state, sprintf('Altitude clamped at 0 m for wp #%d.', idx));
    else
        setStatus(state, sprintf('Altitude of wp #%d: %.0f m (%+d m).', ...
            idx, newAlt, dAlt));
    end
end


function onScrollWheel(~, evt, state)
%onScrollWheel  Mousewheel zoom handler.
%    2D : compute new XLim/YLim via trackbench.editor.zoomAroundPoint,
%         anchored to the cursor position so zoom feels "natural". Call
%         drawMap afterwards to refresh scale bar + grid at the new span.
%    3D : adjust ax.CameraViewAngle. The built-in 3D interactivity
%         already handles zoom on mouse drag, but the scroll wheel is
%         more natural. Clamp angle to [1, 120] degrees.
%
%  Guards: only act when the cursor is actually over the axes (not the
%  sidebar). Otherwise scrolling the sidebar would zoom the map, which
%  is a surprising UX.
    ax = state.ax;
    if ~isgraphics(ax); return; end
    if ~isgraphics(state.fig); return; end

    % Bounds guard: cursor inside axes pixel rect?
    figPt = state.fig.CurrentPoint;
    axPix = getpixelposition(ax, true);
    inAxes = figPt(1) >= axPix(1) && figPt(1) <= axPix(1) + axPix(3) && ...
             figPt(2) >= axPix(2) && figPt(2) <= axPix(2) + axPix(4);
    if ~inAxes; return; end

    try
        scrollCount = double(evt.VerticalScrollCount);
    catch
        return;
    end
    if scrollCount == 0; return; end

    if state.viewMode == "3d"
        % v3.5 fix — axis-limit zoom in 3D, matching 2D behavior.
        %
        % The previous mechanism adjusted ax.CameraViewAngle (perspective
        % field-of-view, 1°–120°). At wide angles the perspective went
        % fisheye-distorted, and the CameraViewAngle persisted across
        % view-mode toggles which left the camera stuck in a warped
        % state. Axis-limit zoom is what MATLAB's built-in magnifying-
        % glass tool does, so the behavior matches what users expect.
        %
        % Zoom is centered on the current view center, not the cursor.
        % Cursor-anchored 3D zoom requires a ray-cast through the
        % camera into the scene to find the world point under the
        % cursor; deferred as future polish. For tens-of-km horizontal
        % spans, center-zoom is close enough to feel natural.
        factor = 1.2 ^ scrollCount;  % +tick (scroll-down) = wider
        cx = mean(ax.XLim);
        cy = mean(ax.YLim);
        cz = mean(ax.ZLim);
        halfX = 0.5 * (ax.XLim(2) - ax.XLim(1)) * factor;
        halfY = 0.5 * (ax.YLim(2) - ax.YLim(1)) * factor;
        halfZ = 0.5 * (ax.ZLim(2) - ax.ZLim(1)) * factor;
        ax.XLim = [cx - halfX, cx + halfX];
        ax.YLim = [cy - halfY, cy + halfY];
        ax.ZLim = [cz - halfZ, cz + halfZ];
        setStatus(state, sprintf('3D zoom: %.1f km span (press R to reset view).', ...
            (ax.XLim(2) - ax.XLim(1)) / 1000));
        return;
    end

    % 2D: zoom around the cursor's world (x, y).
    cp = ax.CurrentPoint;
    if isempty(cp); return; end
    cx = cp(1, 1);
    cy = cp(1, 2);

    [newXLim, newYLim] = trackbench.editor.zoomAroundPoint( ...
        ax.XLim, ax.YLim, cx, cy, scrollCount);
    ax.XLim = newXLim;
    ax.YLim = newYLim;
    % drawMap2D captures ax.XLim/YLim before the cla and restores them
    % after (see drawMap.m lines 60-64 & 159-164), so passing through
    % drawMap refreshes the scale bar + minor grid at the new span
    % without losing our zoom.
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Zoomed (span %.1f km).', ...
        (newXLim(2) - newXLim(1)) / 1000));
end


%% ========================================================================
%  BUTTON CALLBACKS
%% ========================================================================

function onExport(state)
%onExport  Export writable targets via exportToJSON (M5 §3.2).
%
%  Multi-target export rules (implemented in exportToJSON.m):
%    - ALL writable targets are emitted to root.targets[].
%    - Reference targets (readOnly==true) are filtered out — they came
%      from disk and stay on disk unmodified.
%    - Error if there are ZERO writable targets (all are references).
%    - Per-target pre-flight: need >=2 waypoints, monotonic time_s.
%
%  The guard below checks the ACTIVE target's waypoint count for the
%  common "I just opened the editor with an empty target" case. The
%  stricter per-target pre-flight inside exportToJSON catches the edge
%  cases (an inactive writable with <2 waypoints, a duplicated then
%  emptied target, etc.) and raises a named error we surface verbatim.
    nWritable = countWritableTargets(state);
    if nWritable == 0
        setStatus(state, ...
            'Export blocked: no writable targets (all are references).');
        uialert(state.fig, ...
            ['No writable targets to export. ', ...
             'All targets are references (read-only). ', ...
             'Duplicate a reference target first to create an editable copy.'], ...
            'Export blocked');
        return;
    end
    if state.count() < 2
        setStatus(state, ...
            sprintf('Need at least 2 waypoints on the active target (have %d).', state.count()));
        uialert(state.fig, ...
            sprintf('The active target has %d waypoints — need at least 2 to export.', ...
                    state.count()), ...
            'Export blocked');
        return;
    end
    try
        [outPath, excludedRefCount] = trackbench.editor.exportToJSON(state);
    catch ME
        setStatus(state, sprintf('Export failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Export failed');
        return;
    end
    % Build the "(M reference target(s) were not included.)" message
    % exactly once — the dialog and status bar reuse the same text.
    if excludedRefCount == 1
        refMsg = sprintf('(1 reference target was not included.)');
    elseif excludedRefCount > 1
        refMsg = sprintf('(%d reference targets were not included.)', ...
                         excludedRefCount);
    else
        refMsg = '';
    end
    if nWritable == 1
        statusCore = sprintf('Exported 1 target (%d waypoints) → %s', ...
                              state.count(), outPath);
    else
        statusCore = sprintf('Exported %d targets → %s', nWritable, outPath);
    end
    if ~isempty(refMsg)
        setStatus(state, sprintf('%s  %s', statusCore, refMsg));
    else
        setStatus(state, statusCore);
    end
    % Compose dialog body. Single-target exports still show the
    % "targets": "waypoints/<n>" hint because it's the common case
    % and matches the M4 dialog wording. Multi-target exports point at
    % the filename itself.
    %
    % When references were filtered out, lead with an explicit summary
    % line — "N writable exported, M reference(s) not included" — so the
    % user sees both counts in one phrase before the file path/run-file
    % hint. Without refs present we skip that line to keep the M4
    % single-target dialog unchanged.
    [~, fileStem, ~] = fileparts(outPath);
    if ~isempty(refMsg)
        if excludedRefCount == 1
            summaryLine = sprintf( ...
                '%d writable target(s) exported, 1 reference target not included.', ...
                nWritable);
        else
            summaryLine = sprintf( ...
                '%d writable target(s) exported, %d reference targets not included.', ...
                nWritable, excludedRefCount);
        end
    else
        summaryLine = '';
    end
    if nWritable == 1
        dlgCore = sprintf(['Wrote waypoint file:\n\n%s\n\n' ...
                           'Reference it in a run file as:\n' ...
                           '  "targets": "waypoints/%s"'], ...
                          outPath, fileStem);
    else
        dlgCore = sprintf(['Wrote multi-target waypoint file (%d targets):\n\n%s\n\n' ...
                           'Reference it in a run file as:\n' ...
                           '  "targets": "waypoints/%s"'], ...
                          nWritable, outPath, fileStem);
    end
    if ~isempty(summaryLine)
        dlgBody = sprintf('%s\n\n%s', summaryLine, dlgCore);
    else
        dlgBody = dlgCore;
    end
    uialert(state.fig, dlgBody, 'Export complete', 'Icon', 'success');
end


function n = countWritableTargets(state)
%countWritableTargets  Count targets with readOnly==false. Mirrors the
%                      partitioning that exportToJSON does internally
%                      but without duplicating its error path — the UI
%                      uses this for a cheaper pre-flight.
    n = 0;
    for k = 1:numel(state.targets)
        if ~state.targets(k).readOnly
            n = n + 1;
        end
    end
end


function onLoad(state)
%onLoad  Open a file picker at config/targets/waypoints and load the chosen
%        JSON into the EditorState.
    startDir = pwd;
    if state.outputDir ~= "" && isfolder(state.outputDir)
        startDir = char(state.outputDir);
    end
    [file, path] = uigetfile({'*.json','JSON waypoint files'}, ...
        'Load waypoint file', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    try
        trackbench.editor.loadFromJSON(state, full);
    catch ME
        setStatus(state, sprintf('Load failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Load failed');
        return;
    end
    % M5 §3.1: a load can change the active target's name, scenario
    % fields, and (once §3.1's loadFromJSON multi-target refactor lands
    % in task #10) the targets collection itself. Full refresh covers
    % all of those at once.
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Loaded %d waypoints from %s', state.count(), full));
end


function onLoadReference(state)
%onLoadReference  M5 §3.2 — append the chosen JSON's writable targets to
%                 the current edit session as READ-ONLY references.
%
%  Differs from onLoad ("replace" mode) in three ways:
%    1. Calls loadFromJSON with mode="reference", which APPENDS targets
%       instead of replacing the targets collection.
%    2. Each appended target is marked readOnly=true with sourceFile set
%       to the file we loaded. The reference colors cycle through a
%       distinct palette (see loadFromJSON.nextReferenceColor) so they
%       don't collide visually with the canonical-blue active target.
%    3. activeIdx is unchanged — the user's current edit context is
%       preserved. Reference targets render dimmed in the background.
%
%  Reference targets cannot be edited in place (every mutator on
%  EditorState short-circuits when activeIsReadOnly() is true). To edit
%  a reference, the user Duplicates it (creates a writable copy that
%  inherits the geometry and scenario fields).
    startDir = pwd;
    if state.outputDir ~= "" && isfolder(state.outputDir)
        startDir = char(state.outputDir);
    end
    [file, path] = uigetfile({'*.json','JSON waypoint files'}, ...
        'Load waypoint file as reference', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    nBefore = numel(state.targets);
    try
        trackbench.editor.loadFromJSON(state, full, "reference");
    catch ME
        setStatus(state, sprintf('Reference load failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Reference load failed');
        return;
    end
    nAdded = numel(state.targets) - nBefore;
    refreshAfterActiveTargetChange(state);
    if nAdded == 1
        msg = sprintf('Loaded 1 reference target from %s', full);
    else
        msg = sprintf('Loaded %d reference targets from %s', nAdded, full);
    end
    setStatus(state, msg);
end


function onUnloadReferences(state)
%onUnloadReferences  M5 §3.2 — remove every reference (readOnly==true)
%                    target from the current session in one click.
%
%  No file is touched on disk — references were never going to be
%  written by export anyway. activeIdx accounting is handled inside
%  EditorState.unloadAllReferences (it shifts down or falls back to 1
%  if the active target itself was a reference).
%
%  Confirms with the user when there are 2+ references; for 0 or 1
%  reference the action is fast enough that a confirm dialog is more
%  annoying than helpful (and 0 just no-ops with a status message).
    nRefs = 0;
    for k = 1:numel(state.targets)
        if state.targets(k).readOnly; nRefs = nRefs + 1; end
    end
    if nRefs == 0
        setStatus(state, 'No reference targets loaded — nothing to unload.');
        return;
    end
    if nRefs >= 2
        sel = uiconfirm(state.fig, ...
            sprintf('Unload all %d reference target(s)?', nRefs), ...
            'Unload references', ...
            'Options', {'Unload', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel == "Cancel"; return; end
    end
    nRemoved = state.unloadAllReferences();
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Unloaded %d reference target(s).', nRemoved));
end


function onLoadSensors(state)
%onLoadSensors  M6 §3.4 — APPEND mode sensor load. Auto-detects single-
%               sensor, bundle, and run-file shapes via loadSensorsFromJSON's
%               shape-detect path. Adds the parsed sensors to state.sensors
%               without disturbing state.targets.
%
%  Differs from Open Scenario in two ways:
%    1. Append, not replace — the user's existing sensors stay put.
%    2. Sensors only — targets are untouched even if the chosen file is a
%       run file that references both.
%
%  Usability: start the file picker at config/sensors/ under the project
%  root so the common case ("merge in another PSR from the library") is
%  one click. If the user navigates to config/runs/ and picks a run file
%  the shape-detect path still handles it — only the sensors half is
%  consumed.
    startDir = defaultSensorPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON sensor/run files'}, ...
        'Load sensors (append)', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    try
        nLoaded = trackbench.editor.loadSensorsFromJSON( ...
            state, full, "append");
    catch ME
        setStatus(state, sprintf('Load Sensors failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Load Sensors failed');
        return;
    end
    refreshAfterActiveSensorChange(state);
    if nLoaded == 1
        setStatus(state, sprintf('Loaded 1 sensor from %s', full));
    else
        setStatus(state, sprintf('Loaded %d sensors from %s', nLoaded, full));
    end
end


function onSaveSensor(state)
%onSaveSensor  v3.5 step 4c — single-sensor save (active sensor only).
%
%  Counterpart to onLoadSensors. Where Load APPENDS to the collection
%  (and accepts single-file, bundle, or run-file shapes via the shape
%  detector), Save writes JUST the active sensor to JUST a single
%  sensor JSON file. For "save my whole scenario" the user wants
%  Export Scenario in the File panel — different operation class.
%
%  Default folder: config/sensors/<currentType>/ (mirrors
%  exportSensorsToJSON's bundle layout so the new save lands beside
%  the type-specific defaults).
%  Default filename: source-file stem if set (Load → edit → Save
%  overwrites by default), otherwise <sensorName>.json which matches
%  the convention exportSensorsToJSON uses for fresh sensors.
    if ~state.hasActiveSensor()
        setStatus(state, 'No active sensor to save — add one first.');
        return;
    end
    sr = state.activeSensor();
    typeStr = char(sr.sensorType);
    if isempty(typeStr); typeStr = 'UNKNOWN'; end

    % Default directory — config/sensors/<type>/. Falls back to
    % config/sensors/ if the type-specific folder doesn't exist.
    defaultDir = pwd;
    if state.projectRoot ~= ""
        typeDir = fullfile(state.projectRoot, "config", "sensors", typeStr);
        rootDir = fullfile(state.projectRoot, "config", "sensors");
        if isfolder(typeDir)
            defaultDir = char(typeDir);
        elseif isfolder(rootDir)
            defaultDir = char(rootDir);
        end
    end

    if sr.sourceFile ~= ""
        [~, defStem, ~] = fileparts(char(sr.sourceFile));
        defaultName = [defStem '.json'];
    else
        defaultName = [char(sr.sensorName) '.json'];
    end

    [file, path] = uiputfile({'*.json','JSON sensor files'}, ...
        'Save sensor', fullfile(defaultDir, defaultName));
    if isequal(file, 0); return; end
    full = fullfile(path, file);

    try
        outPath = trackbench.editor.exportSingleSensorToJSON(state, string(full));
    catch ME
        setStatus(state, sprintf('Save sensor failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Save sensor failed');
        return;
    end
    [~, savedStem, savedExt] = fileparts(char(outPath));
    setStatus(state, sprintf('Saved sensor "%s" to %s%s', ...
        char(sr.sensorName), savedStem, savedExt));
end


function onOpenScenario(state)
%onOpenScenario  M6 §3.4 — REPLACE mode full-scenario load. Reads the
%                run file's "sensors": [...] and "targets": "..." fields,
%                resolves them under config/sensors/ and config/targets/,
%                and replaces both state.sensors and state.targets.
%
%  This is the "undo all my editor setup and open the thing I ran last
%  week" path. If the user has unsaved work the confirm dialog below
%  gives them a chance to back out.
%
%  Picker opens at config/runs/ so the path matches the simulator's
%  loadRunFile entry point.
    if state.anyDirty
        sel = uiconfirm(state.fig, ...
            ['Opening a scenario will replace the current sensors and targets. ', ...
             'Unsaved edits will be lost (Undo will still restore them). Continue?'], ...
            'Open scenario', ...
            'Options', {'Open', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel == "Cancel"; return; end
    end
    startDir = defaultRunPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON run files'}, ...
        'Open scenario (run file)', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    try
        [nSensors, nTargets] = trackbench.editor.openScenarioFromJSON( ...
            state, full);
    catch ME
        setStatus(state, sprintf('Open scenario failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Open scenario failed');
        return;
    end
    % Full refresh — sensors, targets, AND environment all changed. The
    % target refresh already calls drawMap (which renders the new
    % terrain tint / weather badge / timeline); add the env panel
    % re-sync so the Terrain/Weather sidebars match the loaded state.
    refreshAfterActiveTargetChange(state);
    refreshAfterActiveSensorChange(state);
    refreshAfterEnvironmentChange(state);
    setStatus(state, sprintf( ...
        'Opened scenario: %d sensor(s), %d target(s) from %s', ...
        nSensors, nTargets, full));
end


function onExportScenario(state)
%onExportScenario  M6 §3.4 — full bundle export.
%
%  Prompts for a scenario name (used for the targets JSON stem AND the
%  run-file stem — each sensor keeps its own sensorName as its filename).
%  Emits:
%     config/sensors/<TYPE>/<sensorName>.json    (one per sensor)
%     config/targets/waypoints/<scenario>.json   (via existing exportToJSON)
%     config/runs/<scenario>.json                (wires the above together)
%
%  Preflight mirrors onExport:
%    - At least one writable target (error from exportToJSON if not).
%    - At least one sensor.
%    - Active target needs >=2 waypoints (stricter per-target check is
%      inside exportToJSON, surfaced verbatim below).
    if numel(state.sensors) == 0
        setStatus(state, 'Export Scenario blocked: no sensors configured.');
        uialert(state.fig, ...
            'Add at least one sensor before exporting a scenario.', ...
            'Export Scenario blocked');
        return;
    end
    nWritable = countWritableTargets(state);
    if nWritable == 0
        setStatus(state, ...
            'Export Scenario blocked: no writable targets (all are references).');
        uialert(state.fig, ...
            ['No writable targets to export. ', ...
             'All targets are references (read-only). ', ...
             'Duplicate a reference target first to create an editable copy.'], ...
            'Export Scenario blocked');
        return;
    end
    if state.count() < 2
        setStatus(state, sprintf( ...
            'Need at least 2 waypoints on the active target (have %d).', ...
            state.count()));
        uialert(state.fig, sprintf( ...
            'The active target has %d waypoints — need at least 2 to export.', ...
            state.count()), 'Export Scenario blocked');
        return;
    end

    % ── Prompt for scenario name ──────────────────────────────────────
    %  Default to the loaded-from stem or the active target's name so
    %  "Open Scenario → edit → Export Scenario" round-trips overwrite
    %  the same file set. Otherwise fall back to a timestamped name.
    defaultName = defaultScenarioName(state);
    answer = inputdlg({'Scenario name (used for run + targets files):'}, ...
        'Export Scenario', [1 48], {defaultName});
    if isempty(answer); return; end
    name = strtrim(answer{1});
    if isempty(name)
        setStatus(state, 'Export Scenario cancelled: empty name.');
        return;
    end

    % ── M6 §3.5 fold-in — overwrite confirmation ──────────────────────
    %  Collect every path exportSensorsToJSON would write for this name
    %  and prompt before clobbering any that already exist. The export
    %  is done per-sensor, per-target, per-run so a blind overwrite of
    %  one file with the same name would silently replace someone
    %  else's config. uiconfirm gives the user a chance to bail.
    collisions = collectScenarioCollisions(state, string(name));
    if ~isempty(collisions)
        if isscalar(collisions)
            msg = sprintf('Overwrite existing file?\n\n%s', collisions(1));
        else
            msg = sprintf('Overwrite %d existing file(s)?\n\n%s', ...
                numel(collisions), strjoin("  " + collisions, newline));
        end
        sel = uiconfirm(state.fig, msg, 'Overwrite existing scenario?', ...
            'Options', {'Overwrite', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel == "Cancel"
            setStatus(state, 'Export Scenario cancelled (would overwrite).');
            return;
        end
    end

    try
        [sensorPaths, runPath, targetsPath, excludedRefCount, envPaths] = ...
            trackbench.editor.exportSensorsToJSON(state, string(name));
    catch ME
        setStatus(state, sprintf('Export Scenario failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Export Scenario failed');
        return;
    end

    % ── Build the summary dialog & status line ────────────────────────
    %  Show the run file as the headline artifact (it's what the user
    %  re-opens next time) and list each sensor/target/environment file
    %  beneath.
    nSensors = numel(sensorPaths);
    lines = strings(0);
    lines(end+1) = sprintf('Wrote run file:');
    lines(end+1) = "  " + string(runPath);
    lines(end+1) = "";
    if nSensors == 1
        lines(end+1) = sprintf('Wrote 1 sensor file:');
    else
        lines(end+1) = sprintf('Wrote %d sensor files:', nSensors);
    end
    for k = 1:nSensors
        lines(end+1) = "  " + sensorPaths(k); %#ok<AGROW>
    end
    lines(end+1) = "";
    lines(end+1) = 'Wrote waypoints file:';
    lines(end+1) = "  " + string(targetsPath);
    % M7 §3.4 — environment files. Terrain is always written; weather
    % only when state.weather is non-empty.
    if isfield(envPaths, 'terrain') && envPaths.terrain ~= ""
        lines(end+1) = "";
        lines(end+1) = 'Wrote terrain file:';
        lines(end+1) = "  " + string(envPaths.terrain);
    end
    if isfield(envPaths, 'weather') && envPaths.weather ~= ""
        lines(end+1) = 'Wrote weather file:';
        lines(end+1) = "  " + string(envPaths.weather);
    end
    if excludedRefCount > 0
        lines(end+1) = "";
        if excludedRefCount == 1
            lines(end+1) = '(1 reference target was not included.)';
        else
            lines(end+1) = sprintf( ...
                '(%d reference targets were not included.)', excludedRefCount);
        end
    end
    uialert(state.fig, strjoin(lines, newline), ...
        'Export Scenario complete', 'Icon', 'success');

    [~, runStem, ~] = fileparts(char(runPath));
    setStatus(state, sprintf( ...
        'Exported scenario "%s" (%d sensor(s), %d target(s)).', ...
        runStem, nSensors, nWritable));
end


function dir = defaultSensorPickerDir(state)
%defaultSensorPickerDir  Start Load Sensors… at config/sensors/ under
%                         the project root, or pwd as a fallback.
    dir = pwd;
    if state.projectRoot ~= ""
        candidate = fullfile(state.projectRoot, "config", "sensors");
        if isfolder(candidate); dir = char(candidate); end
    end
end


function dir = defaultRunPickerDir(state)
%defaultRunPickerDir  Start Open Scenario… at config/runs/ under the
%                      project root. Mirrors loadRunFile's expected root.
    dir = pwd;
    if state.projectRoot ~= ""
        candidate = fullfile(state.projectRoot, "config", "runs");
        if isfolder(candidate); dir = char(candidate); end
    end
end


function name = defaultScenarioName(state)
%defaultScenarioName  Pick a sensible pre-filled name for the Export
%                     Scenario dialog. Round-trip a previously-loaded
%                     file stem if present; otherwise the active target's
%                     name; otherwise a timestamped fallback.
    if state.loadedFrom ~= ""
        [~, stem, ~] = fileparts(char(state.loadedFrom));
        name = stem;
        return;
    end
    if state.hasActiveTarget()
        tr = state.activeTarget();
        if strlength(tr.targetName) > 0
            name = char(tr.targetName);
            return;
        end
    end
    name = sprintf('scenario_%s', ...
        char(datetime("now", "Format", "yyyyMMdd_HHmm")));
end


function paths = collectScenarioCollisions(state, scenarioName)
%collectScenarioCollisions  Return the set of files on disk that an
%                            Export Scenario with this name would
%                            overwrite. Mirrors exportSensorsToJSON's
%                            path layout:
%                               config/runs/<name>.json
%                               config/targets/waypoints/<name>.json
%                               config/sensors/<TYPE>/<sensorName>.json
%                                  for each writable sensor
%                               config/terrain/<type>/<name>_terrain.json
%                               config/weather/<type>/<name>_weather.json
%                                  (M7 §3.4 — terrain always written,
%                                  weather only when state.weather is
%                                  non-empty)
%  Returns a string[] (may be empty). Read-only "UNKNOWN passthrough"
%  sensors aren't re-exported by exportSensorsToJSON, so they don't
%  contribute to collisions here either.
    paths = strings(0);
    root = state.projectRoot;
    if root == ""; root = string(pwd); end
    runP     = fullfile(root, "config", "runs",    scenarioName + ".json");
    tgtP     = fullfile(root, "config", "targets", "waypoints", ...
                         scenarioName + ".json");
    if isfile(runP); paths(end+1) = string(runP); end
    if isfile(tgtP); paths(end+1) = string(tgtP); end
    for k = 1:numel(state.sensors)
        sr = state.sensors(k);
        if sr.readOnly; continue; end
        sensorP = fullfile(root, "config", "sensors", ...
                           upper(sr.sensorType), sr.sensorName + ".json");
        if isfile(sensorP); paths(end+1) = string(sensorP); end %#ok<AGROW>
    end
    % M7 §3.4 — terrain + weather collision candidates.
    if ~isempty(state.terrain) && state.terrain.terrainType ~= ""
        terrainP = fullfile(root, "config", "terrain", ...
                            char(state.terrain.terrainType), ...
                            scenarioName + "_terrain.json");
        if isfile(terrainP); paths(end+1) = string(terrainP); end
    end
    if ~isempty(state.weather)
        weatherP = fullfile(root, "config", "weather", ...
                            char(state.weather.weatherType), ...
                            scenarioName + "_weather.json");
        if isfile(weatherP); paths(end+1) = string(weatherP); end
    end
end


function onClear(state)
    if state.count() == 0
        return;
    end
    sel = uiconfirm(state.fig, ...
        sprintf('Clear all %d waypoints? This can be undone.', state.count()), ...
        'Clear waypoints', ...
        'Options', {'Clear', 'Cancel'}, ...
        'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
    if sel == "Cancel"; return; end
    state.clear();
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, 'Cleared all waypoints.');
end


function onDeleteSelected(state)
    if state.selectedIndex < 1
        setStatus(state, 'No waypoint selected.');
        return;
    end
    idx = state.selectedIndex;
    state.removeSelected();
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Deleted waypoint #%d.', idx));
end


function onInsertAfterSelected(state)
%onInsertAfterSelected  Insert a new waypoint halfway between the selected
%                      one and its successor (or offset slightly if last).
    if state.selectedIndex < 1
        setStatus(state, 'Select a waypoint first.');
        return;
    end
    idx = state.selectedIndex;
    n = state.count();
    if idx < n
        x1 = state.waypoints(idx,   1); y1 = state.waypoints(idx,   2);
        x2 = state.waypoints(idx+1, 1); y2 = state.waypoints(idx+1, 2);
        xNew = (x1 + x2) / 2;
        yNew = (y1 + y2) / 2;
    else
        % Last waypoint — offset 1 km east by default.
        xNew = state.waypoints(idx, 1) + 1000;
        yNew = state.waypoints(idx, 2);
    end
    state.insertAfter(idx, xNew, yNew);
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Inserted waypoint at position #%d.', idx+1));
end


function onApplyDefaultAltitude(state)
    if state.count() == 0
        setStatus(state, 'No waypoints to update.');
        return;
    end
    sel = uiconfirm(state.fig, ...
        sprintf('Set altitude on all %d waypoints to %.0f m?', ...
            state.count(), state.defaultAltitudeM), ...
        'Apply default altitude', ...
        'Options', {'Apply', 'Cancel'}, ...
        'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'question');
    if sel == "Cancel"; return; end
    state.applyDefaultAltitudeToAll();
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Applied %.0f m altitude to all %d waypoints.', ...
        state.defaultAltitudeM, state.count()));
end


function onUndo(state)
    if state.undo()
        % M5 §3.1: undo can restore a DIFFERENT active target (New,
        % Duplicate, Delete, Rename, activeIdx change all snapshot the
        % collection). Full active-target refresh covers all of those
        % cases plus the common "undo a waypoint edit" path.
        %
        % M7 §3.3: undo can also restore a different terrain/weather/
        % degradation state, so the Environment panels need a re-sync too.
        % refreshAfterActiveTargetChange already calls drawMap, which
        % handles the terrain-tint/badge/timeline overlays.
        refreshAfterActiveTargetChange(state);
        refreshAfterEnvironmentChange(state);
        setStatus(state, 'Undid last change.');
    else
        setStatus(state, 'Nothing to undo.');
    end
end


function onRedo(state)
    if state.redo()
        % M5 §3.1: see onUndo — redo can also change the active target.
        % M7 §3.3: redo also restores terrain/weather/degradation state.
        refreshAfterActiveTargetChange(state);
        refreshAfterEnvironmentChange(state);
        setStatus(state, 'Redid change.');
    else
        setStatus(state, 'Nothing to redo.');
    end
end


function refreshAfterEnvironmentChange(state)
%refreshAfterEnvironmentChange  Re-sync the Environment panels and map
%                                overlays after a mutation that changed
%                                terrain, weather, or degradation state
%                                (undo/redo, Open Scenario, Load
%                                Terrain/Weather). drawMap is called by
%                                refreshAfterActiveTargetChange in the
%                                undo/redo path, so don't redraw here.
%                                The standalone callers (Open Scenario,
%                                Load Terrain/Weather) call drawMap on
%                                their own paths.
%
%  v3.5 §5c.2 — also re-syncs the new Regions sub-panels and sub-mode
%  toggle so undo/redo of region operations (Add, Dup, Del, Rename,
%  Polygon edits, Change Config) updates the UI. The toggle Value
%  state is captured + restored by snapshot/restore so undo CAN bring
%  back the previous sub-mode along with everything else.
    refreshTerrainPanel(state);
    refreshWeatherPanel(state);
    refreshTerrainRegionsPanel(state);
    refreshWeatherRegionsPanel(state);
    refreshEnvSubModePanel(state);
end


%% ========================================================================
%  CONTEXT MENU
%% ========================================================================

function onContextInsertHere(state)
%onContextInsertHere  Insert a waypoint at the current click location. If
%                     a path segment is near, projects onto it; otherwise
%                     appends to the end.
%
%  M3.3: blocked in 3D view because the CurrentPoint(x, y) projected from
%  a 3D camera isn't a meaningful insertion location.
    if state.viewMode == "3d"
        setStatus(state, 'Switch to 2D to insert waypoints.');
        return;
    end
    ax = state.ax;
    if ~isgraphics(ax); return; end
    cp = ax.CurrentPoint;
    x = cp(1,1); y = cp(1,2);
    if state.count() >= 2
        [segIdx, projXY] = state.findSegmentAt(x, y);
        if segIdx > 0
            state.insertAfter(segIdx, projXY(1), projXY(2));
        else
            state.addWaypoint(x, y);
        end
    else
        state.addWaypoint(x, y);
    end
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Inserted waypoint at (%.0f, %.0f) m.', x, y));
end


function onContextDelete(state)
    onDeleteSelected(state);
end


function onContextClearSelection(state)
    state.selectedIndex = 0;
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
    setStatus(state, 'Selection cleared.');
end


%% ========================================================================
%  SIDEBAR FIELD CALLBACKS
%% ========================================================================

function onNameChanged(src, state)
%onNameChanged  Scenario panel "Target name" field callback.
%
%  M5 §3.2 fix: also refresh the Targets dropdown so the per-target label
%  updates as soon as the user finishes editing the name. Previously the
%  dropdown showed the stale name until something else triggered a
%  refresh (e.g. switching active target and switching back).
%
%  Reference-target guard: if the active target is read-only, refuse the
%  rename, restore the displayed value, and post a status message. The
%  Scenario panel is supposed to be disabled when a reference is active
%  so this branch is defense-in-depth against UI state drift.
    if state.hasActiveTarget() && state.activeTarget().readOnly
        src.Value = char(state.targetName);
        setStatus(state, ...
            'Reference target — name is read-only. Duplicate to edit.');
        return;
    end
    newName = strtrim(string(src.Value));
    if newName == ""
        src.Value = char(state.targetName);
        return;
    end
    newName = regexprep(newName, '[^A-Za-z0-9_\-]', '_');
    src.Value = char(newName);
    state.targetName = newName;
    trackbench.editor.drawMap(state);
    refreshTargetsDropdown(state);
end


function onSpeedChanged(src, state)
    state.defaultSpeedKmh = src.Value;
    % Back-fill any rows whose leg speed was left at the (old) default.
    % Heuristic: we don't track which rows are "auto" vs "explicit" in M2;
    % callers who want finer control should edit via the selection panel.
    state.recomputeTimes();
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
end


function onAltitudeChanged(src, state)
    state.defaultAltitudeM = src.Value;
    % Do NOT mutate existing altitudes — per M1 decision. User must press
    % "Apply default altitude to all waypoints" to bulk-update.
end


function onRcsDbsmChanged(src, state)
    state.rcsDbsm = src.Value;
end


function onRcsProfileChanged(src, state)
    state.rcsProfile = string(src.Value);
end


function onColorByAltitudeChanged(src, state)
%onColorByAltitudeChanged  Toggle M3.1 colormap rendering on/off.
    state.colorByAltitude = logical(src.Value);
    trackbench.editor.drawMap(state);
    if state.colorByAltitude
        setStatus(state, 'Colormap: waypoint fill = altitude (parula).');
    else
        setStatus(state, 'Colormap off (solid yellow markers).');
    end
end


function onGridSpacingChanged(src, state)
%onGridSpacingChanged  M3.2 — switch grid-line spacing. 0 disables the
%                      minor grid; any positive value is read as
%                      kilometers between grid lines.
    state.gridSpacingKm = double(src.Value);
    trackbench.editor.drawMap(state);
    if state.gridSpacingKm <= 0
        setStatus(state, 'Grid off.');
    else
        setStatus(state, sprintf('Grid spacing: %g km.', state.gridSpacingKm));
    end
end


function onPreviewRequest(state)
%onPreviewRequest  M3.4 — open (or raise) the animation preview window.
%
%  Passes only what the preview needs: Nx2 positions, Nx1 times, and
%  the radar site. Waypoints are column-sliced from state.waypoints
%  (the internal Nx5 matrix stores altitude/time/leg_speed in cols 3-5
%  which the 2D preview doesn't need). The preview window reads state
%  at open-time and is intentionally static thereafter — re-open to
%  see edits reflected. This matches the spec's "view-only preview"
%  rule and keeps the preview code fully decoupled from drawMap.
%
%  M4.3.5 — in curved mode we pass the DENSIFIED Nx2 curve to the
%  preview so the marker walks a smooth path. The preview window's
%  interpPos helper already does linear interpolation between adjacent
%  samples — between dense samples of a centripetal Catmull-Rom curve,
%  that approximation is visually indistinguishable from the true curve.
%  Straight mode still passes the raw control points, preserving the
%  M3.4 behavior and keeping the view-only contract intact.
    if isgraphics(state.previewFig)
        % A preview is already open. Bring it forward rather than
        % spawning a second window that would fight over the same timer.
        figure(state.previewFig);
        return;
    end
    if state.count() < 2
        setStatus(state, ...
            'Need at least 2 waypoints before previewing (still opens a guard window).');
    end
    if state.curveMode == "curved" && state.count() >= 2
        [densePts, denseT] = trackbench.editor.catmullRomCurve( ...
            state.waypoints(:, 1:5), ...
            state.curveDensityPerSeg, ...
            state.curveTensionAlpha);
        wp = densePts(:, 1:2);
        ts = denseT;
    else
        wp = state.waypoints(:, 1:2);
        ts = state.waypoints(:, 4);
    end
    rxy = [state.radarEastM, state.radarNorthM];
    state.previewFig = trackbench.editor.previewWindow( ...
        wp, ts, rxy, state.fig);
end


function onCurveModeChanged(src, state)
%onCurveModeChanged  M4.3.2 — toggle between straight-segment and
%                    centripetal Catmull-Rom rendering of the path.
%
%  The flip is an undoable state change (pushUndo before mutation) so
%  Ctrl+Z rolls the view back to whatever mode it was in, matching the
%  behavior of every other scenario-panel change. The button label
%  always describes the current state, not the state it would switch
%  to — same pattern as the 2D/3D view toggle.
%
%  Control waypoints are NOT touched. Only the render path changes.
    state.pushUndo();
    if src.Value
        state.curveMode = "curved";
    else
        state.curveMode = "straight";
    end
    src.Text = curveButtonText(state.curveMode);
    trackbench.editor.drawMap(state);
    if state.curveMode == "curved"
        tensionLabel = tensionNameForAlpha(state.curveTensionAlpha);
        setStatus(state, sprintf( ...
            'Curve mode: Smooth (%s, alpha=%.2f). Waypoints still editable.', ...
            tensionLabel, state.curveTensionAlpha));
    else
        setStatus(state, 'Curve mode: Straight segments.');
    end
end


function onCurveTensionChanged(src, state)
%onCurveTensionChanged  M4.3.3 — switch Catmull-Rom tension parameter.
%
%  Pushes an undo snapshot, updates state.curveTensionAlpha, and
%  triggers a redraw so the visible curve updates immediately. We
%  redraw even when curveMode=="straight" — that's a no-op for the
%  rendered lines but keeps the axes consistent and primes the cache
%  so flipping to Smooth shows the right family with no lag.
%
%  The dropdown's ItemsData carries the literal alpha value (0.0, 0.5,
%  1.0), so src.Value IS the alpha. catmullRomCurve validates alpha in
%  [0,1]; these three presets always satisfy the contract.
    state.pushUndo();
    state.curveTensionAlpha = double(src.Value);
    trackbench.editor.drawMap(state);
    tensionLabel = tensionNameForAlpha(state.curveTensionAlpha);
    if state.curveMode == "curved"
        setStatus(state, sprintf( ...
            'Curve tension: %s (alpha=%.2f).', ...
            tensionLabel, state.curveTensionAlpha));
    else
        setStatus(state, sprintf( ...
            'Curve tension: %s (alpha=%.2f). Flip Curve to Smooth to see it.', ...
            tensionLabel, state.curveTensionAlpha));
    end
end


function name = tensionNameForAlpha(a)
%tensionNameForAlpha  Human-readable label for the three alpha presets.
%
%  Used by both the curve-mode toggle and the tension-dropdown callbacks
%  when they post status. Intermediate values (which the UI can't reach
%  today but might in the future) fall back to a generic label.
    if abs(a - 0.0) < 1e-9
        name = 'Uniform';
    elseif abs(a - 0.5) < 1e-9
        name = 'Centripetal';
    elseif abs(a - 1.0) < 1e-9
        name = 'Chordal';
    else
        name = 'Custom';
    end
end


function onViewModeChanged(src, state)
%onViewModeChanged  M3.3 — switch between 2D and 3D rendering.
%
%  3D is intentionally view-only: click-to-add and drag are disabled
%  so users don't edit waypoints at a bogus altitude. Sidebar editing
%  still works so users can tweak the selected waypoint's altitude and
%  see the change reflected in 3D.
%
%  We always force an autofit on mode switch (reset has3DViewState) so
%  that re-entering 3D after editing in 2D frames the new waypoints
%  sensibly. Within a single 3D session, drawMap3D preserves the user's
%  rotation/zoom across redraws.
    if src.Value
        state.viewMode = "3d";
    else
        state.viewMode = "2d";
    end
    src.Text = viewButtonText(state.viewMode);
    % Cancel any in-progress drag or pan — would otherwise survive the
    % view change and cause a surprise move / pan on the next mouse-up.
    state.dragActive = false;
    state.dragStartWP = [];
    state.panActive = false;
    state.panStartFigPt = [];
    state.panStartXLim = [];
    state.panStartYLim = [];
    % v3.5 §5f — also clear 3D pan camera state on view-mode switch
    state.panStartCamPos    = [];
    state.panStartCamTarget = [];
    state.panStartCamUp     = [];
    state.panStartCamVA     = 0;
    % Force autofit on the destination view so a mode switch always
    % frames the current scene cleanly. v3.5 fix — has2DViewState added
    % parallel to has3DViewState: stale axis limits from the
    % just-departed view would otherwise leak into the new view and
    % render the scene at the wrong scale/aspect.
    state.has2DViewState = false;
    state.has3DViewState = false;
    trackbench.editor.drawMap(state);
    if state.viewMode == "3d"
        setStatus(state, ...
            '3D view: middle-drag to pan, arrows to rotate, wheel to zoom. Switch to 2D to edit.');
    else
        setStatus(state, '2D view: click to add, drag to move.');
    end
end


function onSelFieldChanged(src, state, field)
    if state.selectedIndex < 1; return; end
    state.setWaypointProperty(state.selectedIndex, field, src.Value);
    trackbench.editor.drawMap(state);
    refreshSelectionPanel(state);
end


%% ========================================================================
%  TARGETS SUB-PANEL CALLBACKS (M5 §3.1)
%% ========================================================================

function onTargetsDropdownChanged(src, state)
%onTargetsDropdownChanged  User picked a different active target.
%
%  ItemsData are 1-based target indices (1..numel(state.targets)) so
%  src.Value is exactly the new activeIdx. setActiveIdx clears
%  selectedIndex and is a no-op if the choice didn't change.
    newIdx = double(src.Value);
    if newIdx < 1 || newIdx > numel(state.targets)
        return;
    end
    state.setActiveIdx(newIdx);
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Active target: %s', state.targetName));
end


function onTargetsNew(state)
%onTargetsNew  Append a fresh empty target and make it active.
    state.addNewTarget();
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Created target: %s (active).', state.targetName));
end


function onTargetsAddNasaFlight(state)
%onTargetsAddNasaFlight  Import a NASA DASHlink FDR .mat file as a target.
%
%  v3.5 step 4b — "Import NASA Flight (.mat)..." button in the Targets
%  sub-panel. Workflow:
%    1. uigetfile dialog rooted at projectRoot (best guess; user can
%       navigate freely).
%    2. trackbench.flightdata.loadNASAFlight parses the file.
%    3. state.addNasaFlightTarget builds a TargetRecord from the
%       returned struct and makes it active.
%    4. refreshAfterActiveTargetChange repaints the map + sidebar so
%       the imported flight shows up immediately.
%
%  ERROR HANDLING
%    Three failure modes are surfaced to the user:
%      (a) User cancels the file picker  — silent return, no message.
%      (b) loadNASAFlight throws         — uialert with the parse error
%                                          (likely "missing field LATP"
%                                          or "no airborne data").
%      (c) addNasaFlightTarget throws    — uialert with the validation
%                                          error (notEnoughWaypoints).
%
%  Pre-condition: this callback is wired only in Targets edit mode
%  (button is hidden via the panel-level mode gate when not in Targets
%  mode), so we don't re-check editMode here.

    % Default starting directory: try Tail_687_1/ if it exists at the
    % project root or one level up (matches the user's known data
    % location), else fall back to projectRoot.
    candidates = { ...
        fullfile(char(state.projectRoot), '..', 'Tail_687_1'), ...
        fullfile(char(state.projectRoot), 'Tail_687_1'), ...
        char(state.projectRoot)};
    startDir = '';
    for k = 1:numel(candidates)
        if isfolder(candidates{k})
            startDir = candidates{k};
            break;
        end
    end
    if isempty(startDir); startDir = pwd; end

    [fileName, pathName] = uigetfile( ...
        {'*.mat', 'NASA DASHlink FDR data (*.mat)'; ...
         '*.*',   'All files'}, ...
        'Select NASA flight data .mat file', startDir);
    if isequal(fileName, 0)
        % User cancelled — not an error, just no-op.
        return;
    end
    matPath = fullfile(pathName, fileName);

    % Show "working" status so the user knows the click registered;
    % loadNASAFlight can take a couple of seconds for large files.
    setStatus(state, sprintf('Loading NASA flight: %s ...', fileName));
    drawnow;

    try
        flightData = trackbench.flightdata.loadNASAFlight(string(matPath));
    catch ME
        uialert(state.fig, sprintf( ...
            'Could not parse NASA flight file:\n%s\n\n%s', ...
            matPath, ME.message), 'NASA Flight Load Failed', 'Icon', 'error');
        setStatus(state, 'NASA flight load failed (see dialog).');
        return;
    end

    try
        state.addNasaFlightTarget(flightData);
    catch ME
        uialert(state.fig, sprintf( ...
            'Could not import flight as target:\n\n%s', ME.message), ...
            'NASA Flight Import Failed', 'Icon', 'error');
        setStatus(state, 'NASA flight import failed (see dialog).');
        return;
    end

    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf( ...
        'Imported NASA flight: %s (%d waypoints, %.0fs duration).', ...
        state.targetName, size(state.waypoints, 1), state.durationS));
end


function onTargetsDuplicate(state)
%onTargetsDuplicate  Copy the active target → "<n>_copy", make active.
    if ~state.hasActiveTarget()
        setStatus(state, 'Nothing to duplicate — create a target first.');
        return;
    end
    state.duplicateActiveTarget();
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Duplicated → %s (active).', state.targetName));
end


function onTargetsDelete(state)
%onTargetsDelete  Confirm + delete the active target.
%
%  Confirmation guard so a stray click can't lose work. Skips the
%  prompt only when the active target is empty (no waypoints) AND has
%  the default name — same heuristic the Clear button uses for "are
%  you sure".
    if ~state.hasActiveTarget()
        setStatus(state, 'Nothing to delete.');
        return;
    end
    tr = state.activeTarget();
    needsConfirm = size(tr.waypoints, 1) > 0;
    if needsConfirm
        sel = uiconfirm(state.fig, ...
            sprintf('Delete target "%s" (%d waypoints)?', tr.targetName, size(tr.waypoints, 1)), ...
            'Delete target', ...
            'Options', {'Delete', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel ~= "Delete"
            return;
        end
    end
    deletedName = tr.targetName;
    state.deleteActiveTarget();
    refreshAfterActiveTargetChange(state);
    if state.hasActiveTarget()
        setStatus(state, sprintf('Deleted "%s". Active: %s.', deletedName, state.targetName));
    else
        setStatus(state, sprintf('Deleted "%s". No targets remain — create one to continue.', deletedName));
    end
end


function onTargetsRename(state)
%onTargetsRename  Prompt for a new name and apply it.
%
%  uiinputdlg returns char on OK, empty on Cancel. renameActiveTarget
%  validates non-empty + alphanumeric + uniqueness; we surface those
%  errors via uialert rather than letting them propagate to the
%  command window.
    if ~state.hasActiveTarget()
        setStatus(state, 'Nothing to rename.');
        return;
    end
    current = char(state.activeTarget().targetName);
    answer = inputdlg('New target name:', 'Rename target', ...
        [1 50], {current});
    if isempty(answer)
        return;   % user hit Cancel
    end
    newName = strtrim(answer{1});
    if isempty(newName)
        setStatus(state, 'Rename cancelled (empty name).');
        return;
    end
    try
        state.renameActiveTarget(newName);
    catch ME
        uialert(state.fig, ME.message, 'Rename failed', 'Icon', 'warning');
        return;
    end
    refreshAfterActiveTargetChange(state);
    setStatus(state, sprintf('Renamed → %s', state.targetName));
end


%% ========================================================================
%  MODE-TOGGLE + SENSOR CALLBACKS (M6 §3.2)
%% ========================================================================

function onModeTargetsPressed(src, state)
%onModeTargetsPressed  Targets state-button handler. Enforces mutual
%                      exclusion with the Sensors and Environment
%                      buttons — the trio acts as a radio group even
%                      though MATLAB uifigure has no uibuttongroup
%                      equivalent.
%
%  Clicking an already-on button would normally flip it to off. We bounce
%  it back on instead: the editor must always be in exactly one of
%  "targets" / "sensors" / "environment" mode; "none" is not a valid
%  state. See the matching logic in onModeSensorsPressed and
%  onModeEnvironmentPressed.
    if ~src.Value
        src.Value = true;   % bounce an accidental off-click back on
        return;
    end
    if isgraphics(state.modeSensorsBtn)
        state.modeSensorsBtn.Value = false;
    end
    if isgraphics(state.modeEnvironmentBtn)
        state.modeEnvironmentBtn.Value = false;
    end
    state.setEditMode("targets");
    applyEditMode(state);
    setStatus(state, 'Edit mode: Targets');
end


function onModeSensorsPressed(src, state)
%onModeSensorsPressed  Sensors state-button handler. Mirror of
%                       onModeTargetsPressed — see that docstring.
    if ~src.Value
        src.Value = true;
        return;
    end
    if isgraphics(state.modeTargetsBtn)
        state.modeTargetsBtn.Value = false;
    end
    if isgraphics(state.modeEnvironmentBtn)
        state.modeEnvironmentBtn.Value = false;
    end
    state.setEditMode("sensors");
    applyEditMode(state);
    setStatus(state, 'Edit mode: Sensors');
end


function onModeEnvironmentPressed(src, state)
%onModeEnvironmentPressed  Environment state-button handler. M7 §3.2.
%                           Mirror of onModeTargetsPressed.
    if ~src.Value
        src.Value = true;
        return;
    end
    if isgraphics(state.modeTargetsBtn)
        state.modeTargetsBtn.Value = false;
    end
    if isgraphics(state.modeSensorsBtn)
        state.modeSensorsBtn.Value = false;
    end
    state.setEditMode("environment");
    applyEditMode(state);
    setStatus(state, 'Edit mode: Environment');
end


% ========================================================================
% M7 §3.2 — ENVIRONMENT MODE CALLBACKS + REFRESH HELPERS
% ========================================================================
%  Kept contiguous so the whole Environment-panel surface can be scanned
%  in one block. refreshTerrainPanel / refreshWeatherPanel are the single
%  source-of-truth re-syncers (type dropdown, numeric fields, read-only
%  banner, per-type rate label, clutter visibility, sparkline). The
%  callbacks below all pushUndo → mutate via EditorState → refresh.

function onTerrainTypeChanged(src, state)
%onTerrainTypeChanged  Cascade-reset the terrain fields for the new type.
%                       A uiconfirm guards the reset when the current
%                       terrain's numeric fields have drifted from the
%                       disk defaults — otherwise switching types would
%                       silently wipe the user's edits.
    newType = string(src.Value);
    curType = string(state.terrain.terrainType);
    if lower(newType) == lower(curType)
        return;
    end
    if isDivergentFromTerrainDefaults(state.terrain)
        choice = uiconfirm(state.fig, ...
            sprintf(['Change terrain type to %s?\n\nAll terrain fields ' ...
                     '(Description, Scale, Clutter, Refraction) will ' ...
                     'reset to %s defaults — losing your current ' ...
                     'edits.\n\nThis pushes an undo step.'], ...
                     newType, newType), ...
            'Change terrain type', ...
            'Options', {'Change', 'Cancel'}, ...
            'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
            'Icon', 'warning');
        if ~strcmp(choice, 'Change')
            src.Value = char(curType);
            return;
        end
    end
    % setTerrainType pushes undo itself — do NOT pushUndo here, or the
    % user sees a spurious empty undo slot between each edit (Ctrl+Z
    % would have to be pressed twice to revert one type change).
    try
        state.setTerrainType(newType);
    catch ME
        src.Value = char(curType);
        uialert(state.fig, ME.message, 'Change terrain type');
        return;
    end
    refreshTerrainPanel(state);
    trackbench.editor.drawMap(state);  % tint color changes with type
    setStatus(state, sprintf('Terrain type → %s (defaults reset)', newType));
end


function onTerrainFieldChanged(src, state, field)
%onTerrainFieldChanged  Description / Scale / Clutter / Refraction edits.
%                        Mutator silently no-ops on readOnly terrain, so
%                        the refresh reverts the widget back to the stored
%                        value and the user sees the edit didn't take.
%
%  Undo is pushed by setTerrainField itself — do NOT pushUndo here.
    state.setTerrainField(field, src.Value);
    refreshTerrainPanel(state);
end


function onDegradationToggled(src, state, key)
%onDegradationToggled  Flip one of the four degradation booleans. These
%                       live in the Terrain panel visually but write to
%                       state.degradation, not state.terrain (the fifth
%                       run-file key — "weather" — is derived on export
%                       from isempty(state.weather) and has no checkbox).
%
%  Undo is pushed by setDegradationToggle itself — do NOT pushUndo here.
%  §3.5 — status-bar feedback since these toggles have no visual map
%  effect (they're sim-pipeline behaviors, not rendered graphics).
    state.setDegradationToggle(key, logical(src.Value));
    % Humanize the key for the status message.
    human = replace(lower(string(key)), "_", " ");
    if src.Value
        setStatus(state, sprintf('%s: enabled', capFirstString(human)));
    else
        setStatus(state, sprintf('%s: disabled', capFirstString(human)));
    end
end


function s = capFirstString(str)
%capFirstString  "horizon masking" -> "Horizon masking". String-safe.
    s = char(str);
    if isempty(s); return; end
    s(1) = upper(s(1));
end


function onLoadTerrain(state)
%onLoadTerrain  File-picker for config/terrain/. Uses
%                loadTerrainFromFile which delegates to loadTerrainFromJSON
%                (§3.4 work). If the file is outside config/terrain/ the
%                absolute path is passed verbatim; loader errors surface
%                as uialert + status message.
%
%  Undo is pushed by loadTerrainFromFile itself (and rolled back on
%  parse failure so a failed load leaves no empty-diff undo slot).
%  Do NOT pushUndo here.
    startDir = defaultTerrainPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON terrain files'}, ...
        'Load terrain', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    relPath = extractConfigRelPath(full, state, "terrain");
    try
        state.loadTerrainFromFile(relPath);
    catch ME
        setStatus(state, sprintf('Load terrain failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Load terrain failed');
        return;
    end
    refreshTerrainPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Loaded terrain from %s', file));
end


function onSaveTerrain(state)
%onSaveTerrain  uiputfile dialog → exportTerrainToJSON. v3.5 step 4c.
%
%  Default folder: config/terrain/<currentType>/  (the natural home
%  for files of this terrain type — same convention the library files
%  use, so a saved "my_mountain.json" lands beside the existing
%  default_mountain.json).
%
%  Default filename: stem of state.terrain.sourceFile if set (so a
%  Load → edit → Save round-trip overwrites the source by default),
%  otherwise "my_<type>.json" matching the per-type-folder convention.
%
%  Errors from exportTerrainToJSON surface as uialert + status. The
%  function itself flips environmentDirty=false on success.
    tr = state.terrain;
    typeStr = char(tr.terrainType);
    if isempty(typeStr); typeStr = 'unknown'; end

    % Default directory — config/terrain/<currentType>/. Fall back to
    % config/terrain/ if the type-specific folder doesn't exist (e.g.
    % UNKNOWN passthrough type), then to pwd as the last resort.
    defaultDir = pwd;
    if state.projectRoot ~= ""
        typeDir = fullfile(state.projectRoot, "config", "terrain", typeStr);
        rootDir = fullfile(state.projectRoot, "config", "terrain");
        if isfolder(typeDir)
            defaultDir = char(typeDir);
        elseif isfolder(rootDir)
            defaultDir = char(rootDir);
        end
    end

    % Default filename: source-file stem (preserving subfolder match)
    % or my_<type> for never-saved terrains.
    if tr.sourceFile ~= ""
        [~, defStem, ~] = fileparts(char(tr.sourceFile));
        defaultName = [defStem '.json'];
    else
        defaultName = ['my_' typeStr '.json'];
    end

    [file, path] = uiputfile({'*.json','JSON terrain files'}, ...
        'Save terrain', fullfile(defaultDir, defaultName));
    if isequal(file, 0); return; end   % user cancelled
    full = fullfile(path, file);

    try
        outPath = trackbench.editor.exportTerrainToJSON(state, string(full));
    catch ME
        setStatus(state, sprintf('Save terrain failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Save terrain failed');
        return;
    end
    [~, savedStem, savedExt] = fileparts(char(outPath));
    setStatus(state, sprintf('Saved terrain to %s%s', savedStem, savedExt));
end


function onTerrainOverlayToggled(state)
%onTerrainOverlayToggled  View-only toggle. NO undo step — overlay is a
%                          view preference, not a data edit (matches
%                          colorByAltitude checkbox behavior from M3.1).
%                          The redraw picks up the new checkbox value
%                          and renders / skips the tint accordingly.
    if ~isgraphics(state.terrainOverlayCB); return; end
    if state.terrainOverlayCB.Value
        setStatus(state, 'Terrain overlay enabled.');
    else
        setStatus(state, 'Terrain overlay disabled.');
    end
    trackbench.editor.drawMap(state);
end


function onWeatherTypeChanged(src, state)
%onWeatherTypeChanged  Swap weather type, including the (none) sentinel
%                       which clears state.weather entirely. Same
%                       divergence-warning pattern as terrain, but only
%                       when the CURRENT weather is a non-default
%                       configured record (switching from (none) never
%                       prompts).
    newTypeStr = char(src.Value);   % may be '(none)'
    if isempty(state.weather)
        oldTypeStr = '(none)';
    else
        oldTypeStr = char(state.weather.weatherType);
    end
    if strcmp(newTypeStr, oldTypeStr)
        return;
    end
    mustConfirm = ~isempty(state.weather) && ...
                  isDivergentFromWeatherDefaults(state.weather);
    if mustConfirm
        choice = uiconfirm(state.fig, ...
            sprintf(['Change weather to %s?\n\nAll weather fields ' ...
                     'will reset, losing your current edits.\n\n' ...
                     'This pushes an undo step.'], newTypeStr), ...
            'Change weather type', ...
            'Options', {'Change', 'Cancel'}, ...
            'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
            'Icon', 'warning');
        if ~strcmp(choice, 'Change')
            src.Value = oldTypeStr;
            return;
        end
    end
    % setWeatherType pushes undo itself — do NOT pushUndo here.
    try
        if strcmp(newTypeStr, '(none)')
            state.setWeatherType("none");
        else
            state.setWeatherType(newTypeStr);
        end
    catch ME
        src.Value = oldTypeStr;
        uialert(state.fig, ME.message, 'Change weather type');
        return;
    end
    refreshWeatherPanel(state);
    trackbench.editor.drawMap(state);  % badge + timeline change with type
    setStatus(state, sprintf('Weather → %s', newTypeStr));
end


function onWeatherFieldChanged(src, state, field)
%onWeatherFieldChanged  Rate / storm start / storm end / profile /
%                        pd_floor / clutter_mult edits. No-op when
%                        state.weather is empty (the field would be
%                        disabled anyway but belt-and-braces). Full
%                        panel refresh keeps the sparkline in sync
%                        with profile + storm-window changes.
%
%  Undo is pushed by setWeatherField itself — do NOT pushUndo here.
%
%  §3.5 — storm-window validation:
%    * If user sets stormEndS < stormStartS, clamp end to start+1 and
%      post a status message. The widget value is rewritten by the
%      subsequent refresh.
%    * If stormEndS > the longest target durationS, allow it but warn
%      (storm just finishes after the scenario ends — no crash).
    if isempty(state.weather); return; end
    value = src.Value;
    if strcmp(field, 'stormStartS')
        if value > state.weather.stormEndS
            % Start pushed past current end → bump end to preserve a
            % 1-second minimum window. Clearer than clamping start down.
            state.setWeatherField('stormEndS', value + 1);
            setStatus(state, sprintf( ...
                'Storm end bumped to %gs to stay after new start.', value + 1));
        end
    elseif strcmp(field, 'stormEndS')
        if value <= state.weather.stormStartS
            value = state.weather.stormStartS + 1;
            setStatus(state, sprintf( ...
                'Storm end must exceed start — clamped to %gs.', value));
        end
    end
    state.setWeatherField(field, value);
    % Warn (status-bar only, not alert) when end exceeds scenario dur.
    if strcmp(field, 'stormEndS') || strcmp(field, 'stormStartS')
        maxDur = 0;
        for k = 1:numel(state.targets)
            d = 0;
            if isprop(state.targets(k), 'durationS') && ...
               ~isempty(state.targets(k).durationS)
                d = state.targets(k).durationS;
            end
            if d > maxDur; maxDur = d; end
        end
        if maxDur > 0 && state.weather.stormEndS > maxDur
            setStatus(state, sprintf( ...
                'Warning — storm end (%gs) is after max target duration (%gs).', ...
                state.weather.stormEndS, maxDur));
        end
    end
    refreshWeatherPanel(state);
    % Rate/storm/profile all affect the badge text or timeline shape.
    trackbench.editor.drawMap(state);
end


function onLoadWeather(state)
%onLoadWeather  File-picker for config/weather/. Mirrors onLoadTerrain.
%
%  Undo is pushed by loadWeatherFromFile itself (and rolled back on
%  parse failure). Do NOT pushUndo here.
    startDir = defaultWeatherPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON weather files'}, ...
        'Load weather', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    relPath = extractConfigRelPath(full, state, "weather");
    try
        state.loadWeatherFromFile(relPath);
    catch ME
        setStatus(state, sprintf('Load weather failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Load weather failed');
        return;
    end
    refreshWeatherPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Loaded weather from %s', file));
end


function onSaveWeather(state)
%onSaveWeather  uiputfile dialog → exportWeatherToJSON. v3.5 step 4c.
%
%  Mirrors onSaveTerrain. Refuses to launch the picker when state.weather
%  is empty; the Save button itself is disabled in that state
%  (refreshWeatherPanel), but the guard here is defense-in-depth in case
%  a stale Enable lets the click through.
%
%  Default folder: config/weather/<currentType>/.
%  Default filename: source-file stem if set, else my_<type>.json.
    if isempty(state.weather)
        setStatus(state, 'Pick a weather type first — nothing to save when (none).');
        return;
    end
    wr = state.weather;
    typeStr = char(wr.weatherType);
    if isempty(typeStr); typeStr = 'unknown'; end

    defaultDir = pwd;
    if state.projectRoot ~= ""
        typeDir = fullfile(state.projectRoot, "config", "weather", typeStr);
        rootDir = fullfile(state.projectRoot, "config", "weather");
        if isfolder(typeDir)
            defaultDir = char(typeDir);
        elseif isfolder(rootDir)
            defaultDir = char(rootDir);
        end
    end

    if wr.sourceFile ~= ""
        [~, defStem, ~] = fileparts(char(wr.sourceFile));
        defaultName = [defStem '.json'];
    else
        defaultName = ['my_' typeStr '.json'];
    end

    [file, path] = uiputfile({'*.json','JSON weather files'}, ...
        'Save weather', fullfile(defaultDir, defaultName));
    if isequal(file, 0); return; end
    full = fullfile(path, file);

    try
        outPath = trackbench.editor.exportWeatherToJSON(state, string(full));
    catch ME
        setStatus(state, sprintf('Save weather failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Save weather failed');
        return;
    end
    [~, savedStem, savedExt] = fileparts(char(outPath));
    setStatus(state, sprintf('Saved weather to %s%s', savedStem, savedExt));
end


function refreshTerrainPanel(state)
%refreshTerrainPanel  Re-sync Terrain sub-panel widgets from
%                     state.terrain + state.degradation.
%
%  Called at buildUI seed-time, after every mutator that touches terrain
%  or degradation, after undo/redo, and from applyEditMode when the user
%  switches into environment mode.
%
%  READ-ONLY BEHAVIOR
%    UNKNOWN-passthrough terrain (readOnly=true) dims the four editable
%    terrain fields (type, description, scale, clutter, refraction) and
%    swaps the panel title to a banner. The Load button and degradation
%    checkboxes STAY ENABLED — Load is the escape hatch out of readOnly,
%    and the four degradation booleans are scenario-level flags that
%    apply regardless of terrain source.
    tr = state.terrain;

    % ── Terrain field sync ────────────────────────────────────────────
    if isgraphics(state.terrainTypeDD)
        state.terrainTypeDD.Value = char(tr.terrainType);
    end
    if isgraphics(state.terrainDescField)
        state.terrainDescField.Value = char(tr.description);
    end
    if isgraphics(state.terrainScaleField)
        state.terrainScaleField.Value = tr.terrainScale;
    end
    if isgraphics(state.terrainClutterField)
        state.terrainClutterField.Value = tr.clutterDensity;
    end
    if isgraphics(state.terrainRefractionField)
        state.terrainRefractionField.Value = tr.refractionFactor;
    end

    % ── Degradation checkbox sync (lives visually in Terrain panel) ──
    if isgraphics(state.degTerrainOcclusionCB)
        state.degTerrainOcclusionCB.Value = state.degradation.terrain_occlusion;
    end
    if isgraphics(state.degHorizonMaskingCB)
        state.degHorizonMaskingCB.Value = state.degradation.horizon_masking;
    end
    if isgraphics(state.degGroundClutterCB)
        state.degGroundClutterCB.Value = state.degradation.ground_clutter;
    end
    if isgraphics(state.degDopplerFadeCB)
        state.degDopplerFadeCB.Value = state.degradation.doppler_fade;
    end

    % ── Read-only banner + field enable cascade ──────────────────────
    isRO = tr.readOnly;
    if isgraphics(state.terrainPanel)
        if isRO
            state.terrainPanel.Title = ...
                'Terrain  —  UNKNOWN  (read-only; Load a supported type)';
            state.terrainPanel.ForegroundColor = [0.70 0.30 0.10];
        else
            state.terrainPanel.Title = 'Terrain';
            state.terrainPanel.ForegroundColor = [0 0 0];
        end
    end
    enableState = 'on'; if isRO; enableState = 'off'; end
    setPropIfGraphics(state.terrainTypeDD,          'Enable', enableState);
    setPropIfGraphics(state.terrainDescField,       'Enable', enableState);
    setPropIfGraphics(state.terrainScaleField,      'Enable', enableState);
    setPropIfGraphics(state.terrainClutterField,    'Enable', enableState);
    setPropIfGraphics(state.terrainRefractionField, 'Enable', enableState);
    % Degradation checkboxes + Load + Save + Overlay are explicitly
    % enabled here (v3.5 step 4c bugfix). The prior comment claimed
    % these "intentionally stay on" but applyEditMode's else-branch
    % had been turning them off whenever the user left Environment
    % mode, and refreshTerrainPanel wasn't flipping them back on when
    % returning. Result: Load Terrain stopped working after the first
    % mode round-trip. Setting them explicitly here fixes that and
    % also catches the new Save Terrain button.
    setPropIfGraphics(state.terrainLoadBtn,         'Enable', 'on');
    setPropIfGraphics(state.terrainSaveBtn,         'Enable', 'on');
    setPropIfGraphics(state.terrainOverlayCB,       'Enable', 'on');
    setPropIfGraphics(state.degTerrainOcclusionCB,  'Enable', 'on');
    setPropIfGraphics(state.degHorizonMaskingCB,    'Enable', 'on');
    setPropIfGraphics(state.degGroundClutterCB,     'Enable', 'on');
    setPropIfGraphics(state.degDopplerFadeCB,       'Enable', 'on');
end


function refreshWeatherPanel(state)
%refreshWeatherPanel  Re-sync Weather sub-panel widgets from
%                     state.weather (may be empty → (none)).
%
%  HANDLES PER-TYPE MEANING
%    1. Rate label swaps text per type (Rain rate / Snow rate (eq) /
%       Density / Severity) and the rate field Limits update accordingly.
%       Limits-before-Value ordering per R2025b uieditfield gotcha.
%    2. Clutter × label + field flip Visible=off for fog/icing (those
%       types omit clutter_multiplier on disk — see
%       WeatherRecord.emitsClutterField).
%    3. The sparkline is re-rendered from stormStart / stormEnd /
%       activeType via drawStormSparkline.
%
%  EMPTY-WEATHER STATE
%    When state.weather is empty we show a (none) sparkline, blank the
%    rate label text back to "Rain rate", and gray all editable widgets.
%    The Type dropdown and Load button STAY ENABLED so the user can pick
%    a type or load from disk.
%
%  READ-ONLY BEHAVIOR
%    UNKNOWN-passthrough weather (readOnly=true) grays the editable
%    widgets and banners the panel title. The Type dropdown stays on
%    so the user can flip back to (none).
    hasW = ~isempty(state.weather);
    if hasW
        wr = state.weather;
        typeStr = char(wr.weatherType);
    else
        typeStr = '(none)';
    end

    % ── Type dropdown sync ────────────────────────────────────────────
    if isgraphics(state.weatherTypeDD)
        state.weatherTypeDD.Value = typeStr;
    end

    if hasW
        % Per-type rate label + limits.
        [rateLbl, rateLimits] = weatherRateMeta(wr.weatherType);
        if isgraphics(state.weatherRateLabel)
            state.weatherRateLabel.Text = rateLbl;
        end
        if isgraphics(state.weatherRateField)
            % Limits-BEFORE-Value to avoid silent clipping on type
            % switch when the new Limits would reject the old value.
            state.weatherRateField.Limits = rateLimits;
            clamped = max(min(wr.rainRateMmhr, rateLimits(2)), rateLimits(1));
            state.weatherRateField.Value  = clamped;
        end
        setIfGraphics(state.weatherDescField,         char(wr.description));
        setIfGraphics(state.weatherStormStartField,   wr.stormStartS);
        setIfGraphics(state.weatherStormEndField,     wr.stormEndS);
        if isgraphics(state.weatherProfileDD)
            state.weatherProfileDD.Value = char(wr.activeType);
        end
        setIfGraphics(state.weatherPdFloorField,      wr.pdFloor);
        setIfGraphics(state.weatherClutterMultField,  wr.clutterMultiplier);

        % Clutter × visibility — rain/snow only, hidden for fog/icing.
        if wr.emitsClutterField()
            setPropIfGraphics(state.weatherClutterMultLabel, 'Visible', 'on');
            setPropIfGraphics(state.weatherClutterMultField, 'Visible', 'on');
        else
            setPropIfGraphics(state.weatherClutterMultLabel, 'Visible', 'off');
            setPropIfGraphics(state.weatherClutterMultField, 'Visible', 'off');
        end
    else
        % (none) — reset widgets to neutral placeholder values. Keep
        % rate label at the plain default so returning to (none) doesn't
        % leave "Severity" hanging.
        if isgraphics(state.weatherRateLabel)
            state.weatherRateLabel.Text = 'Rain rate';
        end
        if isgraphics(state.weatherRateField)
            state.weatherRateField.Limits = [0 200];
            state.weatherRateField.Value  = 0;
        end
        setIfGraphics(state.weatherDescField,        '');
        setIfGraphics(state.weatherStormStartField,  0);
        setIfGraphics(state.weatherStormEndField,    0);
        if isgraphics(state.weatherProfileDD)
            state.weatherProfileDD.Value = 'step';
        end
        setIfGraphics(state.weatherPdFloorField,     0.15);
        setIfGraphics(state.weatherClutterMultField, 1.0);
        setPropIfGraphics(state.weatherClutterMultLabel, 'Visible', 'on');
        setPropIfGraphics(state.weatherClutterMultField, 'Visible', 'on');
    end

    % ── Enable cascade ────────────────────────────────────────────────
    % Editable fields require a configured, writable, supported weather.
    editable = hasW && state.weather.isSupportedType() && ~state.weather.readOnly;
    editableState = 'on'; if ~editable; editableState = 'off'; end
    setPropIfGraphics(state.weatherDescField,        'Enable', editableState);
    setPropIfGraphics(state.weatherRateField,        'Enable', editableState);
    setPropIfGraphics(state.weatherStormStartField,  'Enable', editableState);
    setPropIfGraphics(state.weatherStormEndField,    'Enable', editableState);
    setPropIfGraphics(state.weatherProfileDD,        'Enable', editableState);
    setPropIfGraphics(state.weatherPdFloorField,     'Enable', editableState);
    setPropIfGraphics(state.weatherClutterMultField, 'Enable', editableState);
    % Type DD + Load stay on — the escape hatch.
    setPropIfGraphics(state.weatherTypeDD,  'Enable', 'on');
    setPropIfGraphics(state.weatherLoadBtn, 'Enable', 'on');
    % Save button (v3.5 step 4c): enabled whenever weather is configured
    % (even readOnly UNKNOWN — it round-trips via originalDef). Disabled
    % when state.weather is empty since there's nothing to write.
    if hasW
        setPropIfGraphics(state.weatherSaveBtn, 'Enable', 'on');
    else
        setPropIfGraphics(state.weatherSaveBtn, 'Enable', 'off');
    end

    % ── Read-only banner ──────────────────────────────────────────────
    if isgraphics(state.weatherPanel)
        if hasW && state.weather.readOnly
            state.weatherPanel.Title = ...
                'Weather  —  UNKNOWN  (read-only; Load a supported type)';
            state.weatherPanel.ForegroundColor = [0.70 0.30 0.10];
        else
            state.weatherPanel.Title = 'Weather';
            state.weatherPanel.ForegroundColor = [0 0 0];
        end
    end

    % ── Sparkline ─────────────────────────────────────────────────────
    if isgraphics(state.weatherStormSparkline)
        drawStormSparkline(state.weatherStormSparkline, state.weather);
    end
end


function [lbl, lims] = weatherRateMeta(weatherType)
%weatherRateMeta  Per-type label + Limits for the rate field. The
%                  rain_rate_mmhr slot stores different meanings per
%                  type (see WeatherRecord docstring); this helper
%                  picks the right UI label and a sensible Limits range.
    switch lower(string(weatherType))
        case "rain"
            lbl  = 'Rain rate';      lims = [0 200];
        case "snow"
            lbl  = 'Snow rate (eq)'; lims = [0 200];
        case "fog"
            lbl  = 'Density';        lims = [0 100];
        case "icing"
            lbl  = 'Severity';       lims = [0 100];
        otherwise
            lbl  = 'Rate';           lims = [0 200];
    end
end


function drawStormSparkline(ax, wr)
%drawStormSparkline  Tiny storm-profile preview for the Weather panel.
%                     Renders one of three shapes (step / ramp / pulse)
%                     bracketed by pre/post-storm zero padding.
%
%  When wr is empty a neutral "(no weather)" placeholder is drawn. The
%  axes has Toolbar off, HitTest off, and PickableParts='none' (set at
%  construct time) so this render can be called repeatedly without
%  leaking interactivity handles.
    cla(ax);
    hold(ax, 'on');
    if isempty(wr)
        plot(ax, [0 1], [0 0], '--', 'Color', [0.75 0.75 0.75]);
        text(ax, 0.5, 0.5, '(no weather)', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', 10, ...
            'Color', [0.55 0.55 0.55]);
        ax.XLim = [0 1];
        ax.YLim = [-0.15 1.15];
        hold(ax, 'off');
        return;
    end
    t0 = wr.stormStartS;
    t1 = wr.stormEndS;
    if t1 <= t0
        t1 = t0 + 1;  % guard against zero-or-negative window
    end
    % Padding on either side so the on/off edges are visible.
    pad = max(5, (t1 - t0) * 0.15);
    a = t0 - pad;
    b = t1 + pad;
    prof = lower(string(wr.activeType));
    switch prof
        case "step"
            x = [a, t0, t0, t1, t1, b];
            y = [0,  0,   1,  1,  0,  0];
        case "ramp"
            tm = (t0 + t1) / 2;
            x = [a, t0, tm, t1, b];
            y = [0,  0,  1,  0,  0];
        case "pulse"
            tp = t0 + (t1 - t0) * 0.20;
            x = [a, t0, t0, tp, tp, b];
            y = [0,  0,   1,  1,  0,  0];
        otherwise
            x = [a, t0, t0, t1, t1, b];
            y = [0,  0,   1,  1,  0,  0];
    end
    plot(ax, x, y, 'LineWidth', 1.5, 'Color', [0.2 0.45 0.85]);
    % Faint shaded band showing the storm window itself.
    patch(ax, 'XData', [t0 t1 t1 t0], 'YData', [0 0 1 1], ...
        'FaceColor', [0.2 0.45 0.85], 'FaceAlpha', 0.08, ...
        'EdgeColor', 'none');
    ax.XLim = [a b];
    ax.YLim = [-0.15 1.15];
    hold(ax, 'off');
end


function dir = defaultTerrainPickerDir(state)
%defaultTerrainPickerDir  Start Load Terrain… at config/terrain/.
    dir = pwd;
    if state.projectRoot ~= ""
        candidate = fullfile(state.projectRoot, "config", "terrain");
        if isfolder(candidate); dir = char(candidate); end
    end
end


function dir = defaultWeatherPickerDir(state)
%defaultWeatherPickerDir  Start Load Weather… at config/weather/.
    dir = pwd;
    if state.projectRoot ~= ""
        candidate = fullfile(state.projectRoot, "config", "weather");
        if isfolder(candidate); dir = char(candidate); end
    end
end


function rel = extractConfigRelPath(fullPath, state, subdir)
%extractConfigRelPath  Strip projectRoot/config/<subdir>/ prefix and the
%                       .json extension so loadTerrainFromFile /
%                       loadWeatherFromFile get the library-style
%                       "<typeDir>/<name>" path. Falls back to the
%                       absolute path when the file is outside the
%                       config tree — the loader error messaging takes
%                       over from there.
    rel = fullPath;
    if state.projectRoot == ""; return; end
    rootPrefix = [char(fullfile(state.projectRoot, "config", subdir)) filesep];
    if startsWith(fullPath, rootPrefix)
        tail = fullPath(numel(rootPrefix)+1:end);
        [sub, name, ~] = fileparts(tail);
        if ~isempty(sub)
            rel = fullfile(sub, name);
        else
            rel = name;
        end
    end
end


function tf = isDivergentFromTerrainDefaults(tr)
%isDivergentFromTerrainDefaults  True iff tr's user-editable numeric /
%                                 descriptive fields differ from the
%                                 disk default for its current type.
%                                 Drives the uiconfirm on type change.
    if tr.readOnly; tf = false; return; end
    try
        fresh = trackbench.editor.terrainDefaults(tr.terrainType);
    catch
        tf = true; return;   % unknown current type → always confirm
    end
    tf = (tr.terrainScale      ~= fresh.terrainScale)    || ...
         (tr.clutterDensity    ~= fresh.clutterDensity)  || ...
         (tr.refractionFactor  ~= fresh.refractionFactor) || ...
         (string(tr.description) ~= string(fresh.description));
end


function tf = isDivergentFromWeatherDefaults(wr)
%isDivergentFromWeatherDefaults  True iff wr's user-editable fields
%                                 differ from the disk default for its
%                                 current type. Drives the uiconfirm on
%                                 weather-type change. Returns false
%                                 for readOnly UNKNOWN passthrough
%                                 (re-typing one errors out via the
%                                 mutator anyway).
    if wr.readOnly; tf = false; return; end
    try
        fresh = trackbench.editor.weatherDefaults(wr.weatherType);
    catch
        tf = true; return;
    end
    tf = (wr.rainRateMmhr      ~= fresh.rainRateMmhr)      || ...
         (wr.stormStartS       ~= fresh.stormStartS)       || ...
         (wr.stormEndS         ~= fresh.stormEndS)         || ...
         (wr.pdFloor           ~= fresh.pdFloor)           || ...
         (wr.clutterMultiplier ~= fresh.clutterMultiplier) || ...
         (string(wr.activeType)   ~= string(fresh.activeType)) || ...
         (string(wr.description) ~= string(fresh.description));
end


function onSensorsDropdownChanged(src, state)
%onSensorsDropdownChanged  Switch active sensor when user picks from
%                           the Sensors dropdown. Parallels
%                           onTargetsDropdownChanged.
    newIdx = src.Value;
    if ~isnumeric(newIdx) || newIdx < 1 || newIdx > numel(state.sensors)
        return;
    end
    state.setActiveSensorIdx(newIdx);
    refreshAfterActiveSensorChange(state);
    setStatus(state, sprintf('Active sensor: %s', state.sensors(newIdx).sensorName));
end


function onSensorsAdd(state)
%onSensorsAdd  Prompt for a sensor type, then addNewSensor(type).
%              The modal restricts to the 8 supported types — UNKNOWN
%              passthrough sensors are only created by the load path,
%              never by the user.
    types = cellstr(trackbench.editor.sensorDomain(state.domain).sensorTypes);
    [sel, ok] = listdlg('PromptString', 'Pick sensor type:', ...
                        'SelectionMode', 'single', ...
                        'ListString', types, ...
                        'Name', 'Add sensor', ...
                        'ListSize', [220 170]);
    if ~ok || isempty(sel)
        return;
    end
    typeStr = types{sel};
    try
        state.addNewSensor(typeStr);
    catch ME
        uialert(state.fig, ME.message, 'Add sensor failed', 'Icon', 'error');
        return;
    end
    refreshAfterActiveSensorChange(state);
    setStatus(state, sprintf('Added sensor: %s (%s)', ...
        state.activeSensor().sensorName, typeStr));
end


function onSensorsDuplicate(state)
%onSensorsDuplicate  Copy the active sensor, offset 2 km east. Fails
%                     loudly via uialert if there's no active sensor
%                     (shouldn't happen because applyEditMode disables
%                     the button, but defensive is cheap).
    if ~state.hasActiveSensor()
        setStatus(state, 'Nothing to duplicate.');
        return;
    end
    try
        state.duplicateActiveSensor();
    catch ME
        uialert(state.fig, ME.message, 'Duplicate sensor failed', 'Icon', 'error');
        return;
    end
    refreshAfterActiveSensorChange(state);
    setStatus(state, sprintf('Duplicated → %s', state.activeSensor().sensorName));
end


function onSensorsDelete(state)
%onSensorsDelete  Delete active sensor with a uiconfirm guard. The
%                  operation is undoable — EditorState.deleteActiveSensor
%                  pushes an undo step before removing.
    if ~state.hasActiveSensor()
        setStatus(state, 'Nothing to delete.');
        return;
    end
    nm = char(state.activeSensor().sensorName);
    sel = uiconfirm(state.fig, ...
        sprintf('Delete sensor "%s"? You can Ctrl+Z to bring it back.', nm), ...
        'Delete sensor', 'Options', {'Delete', 'Cancel'}, ...
        'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
        'Icon', 'warning');
    if ~strcmp(sel, 'Delete')
        return;
    end
    state.deleteActiveSensor();
    refreshAfterActiveSensorChange(state);
    setStatus(state, sprintf('Deleted: %s', nm));
end


function onSensorsRename(state)
%onSensorsRename  Prompt for a new name; delegate to
%                  state.renameActiveSensor for validation + uniqueness.
    if ~state.hasActiveSensor()
        setStatus(state, 'Nothing to rename.');
        return;
    end
    current = char(state.activeSensor().sensorName);
    answer = inputdlg('New sensor name:', 'Rename sensor', ...
                      [1 50], {current});
    if isempty(answer)
        return;
    end
    newName = strtrim(answer{1});
    if isempty(newName)
        setStatus(state, 'Rename cancelled (empty name).');
        return;
    end
    try
        state.renameActiveSensor(newName);
    catch ME
        uialert(state.fig, ME.message, 'Rename failed', 'Icon', 'warning');
        return;
    end
    refreshAfterActiveSensorChange(state);
    setStatus(state, sprintf('Renamed → %s', state.activeSensor().sensorName));
end


function onSensorNameChanged(src, state)
%onSensorNameChanged  Inline rename via the Name edit-field. Uses the
%                      same renameActiveSensor mutator as the Rename
%                      button — validation errors surface through
%                      uialert and the field snaps back to the old name.
    if ~state.hasActiveSensor()
        src.Value = '';
        return;
    end
    newName = strtrim(src.Value);
    current = char(state.activeSensor().sensorName);
    if isempty(newName) || strcmp(newName, current)
        src.Value = current;
        return;
    end
    try
        state.renameActiveSensor(newName);
    catch ME
        uialert(state.fig, ME.message, 'Rename failed', 'Icon', 'warning');
        src.Value = current;   % snap back
        return;
    end
    refreshSensorsDropdown(state);
    refreshSensorParamsPanel(state);
    setStatus(state, sprintf('Renamed → %s', state.activeSensor().sensorName));
end


function onSensorTypeChanged(src, state)
%onSensorTypeChanged  Changing type re-seeds per-type defaults. Name,
%                      position, and altitude are preserved so the user
%                      doesn't have to re-place the sensor after
%                      switching PSR→PAR (for example).
%
%  A uiconfirm guards the reset because it clobbers all the other
%  type-specific fields (freq, RPM, sector, FOV, tilt, Pd, FAR, range
%  limits). Cancel bounces the dropdown back to the previous type.
    if ~state.hasActiveSensor()
        return;
    end
    newType = string(src.Value);
    sr = state.activeSensor();
    if sr.sensorType == newType
        return;
    end
    choice = uiconfirm(state.fig, ...
        sprintf(['Change type to %s?\n\nAll type-specific parameters ' ...
                 '(frequency, RPM, sector, FOV, tilt, Pd, FAR, range ' ...
                 'limits) will reset to %s defaults.\n\nName, east, ' ...
                 'north, and altitude are preserved.\n\nThis pushes ' ...
                 'an undo step.'], newType, newType), ...
        'Change sensor type', ...
        'Options', {'Change', 'Cancel'}, ...
        'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
        'Icon', 'warning');
    if ~strcmp(choice, 'Change')
        src.Value = char(sr.sensorType);   % bounce back
        return;
    end
    % Preserve the user-authored fields we explicitly keep.
    keepName   = sr.sensorName;
    keepEast   = sr.positionEastM;
    keepNorth  = sr.positionNorthM;
    keepMount3 = sr.mountingLoc(3);
    % Build a fresh record seeded with the new type's defaults, then
    % re-apply the preserved fields. pushUndo captures the pre-change
    % sensors array so Ctrl+Z returns to the old type.
    state.pushUndo();
    % M6 §3.6C — shared defaults live in trackbench.editor.sensorDefaults;
    % the previous file-local applyTypeDefaults + EditorState.defaultsForType
    % pair was a maintenance trap (two switches in sync by comment only).
    reset = trackbench.editor.sensorDefaults(newType);
    reset.sensorName     = keepName;
    reset.positionEastM  = keepEast;
    reset.positionNorthM = keepNorth;
    reset.mountingLoc    = [0 0 keepMount3];
    state.setActiveSensor(reset);
    refreshSensorParamsPanel(state);
    refreshSensorsDropdown(state);
    setStatus(state, sprintf('Type → %s (per-type defaults reset)', newType));
end


% M6 §3.6C — applyTypeDefaults was moved to trackbench.editor.sensorDefaults
% (package-level) so it can be shared with EditorState.addNewSensor.
% See: src/+trackbench/+editor/sensorDefaults.m


function onSensorFieldChanged(src, state, field)
%onSensorFieldChanged  Generic numeric-field handler for the Sensor
%                       Parameters panel. Uses a tag to dispatch to the
%                       right SensorRecord slot, then re-runs the refresh
%                       so derived widgets (scan-mode banner, sector
%                       visibility) stay in sync.
%
%  UNITS
%    'frequency' is in GHz in the UI → multiply by 1e9 for storage.
%    'altitude' is +m above ground in the UI → stored as -Alt on
%    mountingLoc(3) (NED convention; negative = above ground).
%
%  UNDO CADENCE
%    uieditfield fires ValueChangedFcn only on commit (Enter or focus
%    lost), not per keystroke. Each commit pushes one undo step —
%    comfortable Ctrl+Z granularity for numeric edits.
    if ~state.hasActiveSensor()
        return;
    end
    state.pushUndo();
    sr = state.activeSensor();
    v = src.Value;
    switch field
        case 'east';       sr.positionEastM   = v;
        case 'north';      sr.positionNorthM  = v;
        case 'altitude';   sr.mountingLoc(3)  = -v;     % +UI → -stored (NED)
        case 'tilt';       sr.tilt            = v;
        case 'frequency';  sr.frequencyHz     = v * 1e9;  % GHz → Hz
        case 'maxRange';   sr.rangeLimits(2)  = v;
        case 'rangeRes';   sr.rangeResM       = v;
        case 'azFov';      sr.fov(1)          = v;
        case 'elFov';      sr.fov(2)          = v;
        case 'rpm';        sr.rpm             = v;
        case 'sectorLo';   sr.sectorDeg(1)    = v;
        case 'sectorHi';   sr.sectorDeg(2)    = v;
        case 'pd';         sr.pd              = v;
        case 'far';        sr.far             = v;
    end
    state.setActiveSensor(sr);
    % RPM and sectorDeg drive scan-mode classification; always refresh.
    refreshSensorParamsPanel(state);
end


function onSensorPlaceOnMap(state)
%onSensorPlaceOnMap  M6 §3.5A — arm the next axes click to teleport the
%                     active sensor. Guarded on editMode + hasActiveSensor
%                     so a stale Enable state (or keyboard-activated
%                     button) can't place a ghost sensor.
%
%  v3.5 §5g fix: previous version called setIfGraphics(h, 'Text', …)
%  to update the status label, but setIfGraphics is the 2-arg
%  Value-setter (handle, value) — the 3-arg call raised "Too many
%  input arguments" every time the user pressed Place-on-map. The
%  canonical helper for statusLabel is setStatus(state, msg); use it.
    if state.editMode ~= "sensors"; return; end
    if ~state.hasActiveSensor()
        setStatus(state, 'No active sensor to place — add one first.');
        return;
    end
    if state.activeSensorIsReadOnly()
        setStatus(state, 'Active sensor is read-only — duplicate it first.');
        return;
    end
    state.sensorPlacePending = true;
    sr = state.activeSensor();
    setStatus(state, sprintf( ...
        'Click on map to place %s. (Esc to cancel.)', sr.sensorName));
end


%% ========================================================================
%  HELPERS
%% ========================================================================

function refreshAfterActiveTargetChange(state)
%refreshAfterActiveTargetChange  Re-sync every UI element that mirrors
%                                the active TargetRecord.
%
%  Called from every Targets-sub-panel mutation (New, Duplicate, Delete,
%  Rename, Dropdown change) AND from Open Scenario (M6 §3.4) which
%  mutates both sensors and targets simultaneously. Order matters
%  slightly:
%    1. Rebuild the targets dropdown (Items + ItemsData + Value) so it
%       reflects the new collection.
%    2. Re-sync the Scenario panel fields from the new active target.
%    3. Update name field separately (it's bound through state.nameField).
%    4. Refresh the selection panel (selectedIndex was reset to 0 by
%       any active-target change — sidebar must reflect "no selection").
%    5. Update the waypoint count label.
%    6. Refresh the Sensors dropdown and params panel — harmless for
%       pure target mutations, essential for Open Scenario which also
%       replaces state.sensors.
%    7. Redraw the map (target colors / titles change; sensors may have
%       changed too).
    refreshTargetsDropdown(state);
    refreshScenarioPanel(state);
    refreshSelectionPanel(state);
    updateWaypointCount(state);
    refreshSensorsDropdown(state);
    refreshSensorParamsPanel(state);
    trackbench.editor.drawMap(state);
end


function refreshAfterActiveSensorChange(state)
%refreshAfterActiveSensorChange  Re-sync every UI element that mirrors
%                                the active SensorRecord. Called from
%                                every Sensors sub-panel mutation (Add,
%                                Duplicate, Delete, Rename, dropdown
%                                change) and from undo/redo.
%
%  Map redraw IS called here — §3.3 draws range rings, sector wedges,
%  and beam cones for the full sensor collection with the active sensor
%  highlighted, so any active-sensor mutation needs a fresh paint.
    refreshSensorsDropdown(state);
    refreshSensorParamsPanel(state);
    trackbench.editor.drawMap(state);
end


function refreshSensorsDropdown(state)
%refreshSensorsDropdown  Rebuild the Sensors dropdown from
%                         state.sensors + state.activeSensorIdx.
%
%  Same Items→ItemsData→Value ordering as refreshTargetsDropdown to
%  avoid "Value not in ItemsData" throws from uidropdown.
%
%  Display label: "name (TYPE)" with optional tags:
%    (active)       — active sensor
%    (passthrough)  — readOnly (UNKNOWN-type passthrough)
%  Combined tags: "(TYPE, active, passthrough)".
    if ~isgraphics(state.sensorsDD)
        return;
    end
    n = numel(state.sensors);
    if n == 0
        state.sensorsDD.Items     = {'(no sensors)'};
        state.sensorsDD.ItemsData = {0};
        state.sensorsDD.Value     = 0;
        toggleSensorsButtons(state, false);
        return;
    end
    items = cell(1, n);
    data  = cell(1, n);
    for k = 1:n
        sr = state.sensors(k);
        nm = char(sr.sensorName);
        tp = char(sr.sensorType);
        isActive = (k == state.activeSensorIdx);
        isRO     = sr.readOnly;
        if isRO && isActive
            items{k} = sprintf('%s (%s, active, passthrough)', nm, tp);
        elseif isRO
            items{k} = sprintf('%s (%s, passthrough)', nm, tp);
        elseif isActive
            items{k} = sprintf('%s (%s, active)', nm, tp);
        else
            items{k} = sprintf('%s (%s)', nm, tp);
        end
        data{k} = k;
    end
    state.sensorsDD.Items     = items;
    state.sensorsDD.ItemsData = data;
    if state.activeSensorIdx >= 1 && state.activeSensorIdx <= n
        state.sensorsDD.Value = state.activeSensorIdx;
    else
        state.sensorsDD.Value = 1;
    end
    toggleSensorsButtons(state, true);
end


function toggleSensorsButtons(state, hasActive)
%toggleSensorsButtons  Enable Duplicate / Delete / Rename / Save when an
%                      active sensor exists. Add + Load are always
%                      enabled (subject to applyEditMode's coarse mode
%                      gate). v3.5 step 4c added Save to the gated set.
    enable = 'off'; if hasActive; enable = 'on'; end
    if isgraphics(state.sensorsBtnDuplicate); state.sensorsBtnDuplicate.Enable = enable; end
    if isgraphics(state.sensorsBtnDelete);    state.sensorsBtnDelete.Enable    = enable; end
    if isgraphics(state.sensorsBtnRename);    state.sensorsBtnRename.Enable    = enable; end
    if isgraphics(state.sensorsBtnSave);      state.sensorsBtnSave.Enable      = enable; end
end


function refreshSensorParamsPanel(state)
%refreshSensorParamsPanel  Re-sync the Sensor Parameters panel to the
%                          active SensorRecord. Decides visibility of
%                          the Sector start/end widgets based on the
%                          sensor's scan kind.
%
%  NO ACTIVE SENSOR
%    When sensors is empty, the panel stays alive but fields are
%    disabled and the Name field is cleared. The scan-mode banner says
%    "— (no active sensor)" to make the empty state obvious.
%
%  READ-ONLY ENFORCEMENT
%    When the active sensor is readOnly (UNKNOWN passthrough) we
%    disable every editable field. applyEditMode re-asserts this on
%    mode switch, and refreshSensorParamsPanel re-asserts it after any
%    mutator that could change read-only status (load, duplicate).
%
%  LIMITS-BEFORE-VALUE
%    Fields have Limits set at construct time; we only write Values
%    here. SensorRecord values stay inside those ranges by default, so
%    no ordering hazard.
    if ~state.hasActiveSensor()
        setSensorFieldEnable(state, false);
        setPropIfGraphics(state.sensorNameField, 'Value', '');
        setPropIfGraphics(state.sensorScanModeLbl, 'Text', ...
            'Scan mode: — (no active sensor)');
        setPropIfGraphics(state.sensorRpmLabel,      'Visible', 'on');
        setPropIfGraphics(state.sensorRpmField,      'Visible', 'on');
        setPropIfGraphics(state.sensorSectorLoLabel, 'Visible', 'off');
        setPropIfGraphics(state.sensorSectorLoField, 'Visible', 'off');
        setPropIfGraphics(state.sensorSectorHiLabel, 'Visible', 'off');
        setPropIfGraphics(state.sensorSectorHiField, 'Visible', 'off');
        return;
    end

    sr = state.activeSensor();

    setPropIfGraphics(state.sensorNameField, 'Value', char(sr.sensorName));
    if isgraphics(state.sensorTypeDD)
        typeStr = char(sr.sensorType);
        if any(strcmp(typeStr, state.sensorTypeDD.Items))
            state.sensorTypeDD.Value = typeStr;
        else
            % UNKNOWN passthrough — pin to PSR visually; the field is
            % disabled below by the read-only branch so the mis-pin
            % cannot cause an accidental type change.
            state.sensorTypeDD.Value = 'PSR';
        end
    end
    setPropIfGraphics(state.sensorEastField,     'Value', sr.positionEastM);
    setPropIfGraphics(state.sensorNorthField,    'Value', sr.positionNorthM);
    setPropIfGraphics(state.sensorAltField,      'Value', -sr.mountingLoc(3));  % NED → +m
    setPropIfGraphics(state.sensorTiltField,     'Value', sr.tilt);
    setPropIfGraphics(state.sensorFreqField,     'Value', sr.frequencyHz / 1e9);  % Hz → GHz
    setPropIfGraphics(state.sensorMaxRangeField, 'Value', sr.rangeLimits(2));
    setPropIfGraphics(state.sensorRangeResField, 'Value', sr.rangeResM);
    setPropIfGraphics(state.sensorAzFovField,    'Value', sr.fov(1));
    setPropIfGraphics(state.sensorElFovField,    'Value', sr.fov(2));
    setPropIfGraphics(state.sensorRpmField,      'Value', sr.rpm);
    setPropIfGraphics(state.sensorSectorLoField, 'Value', sr.sectorDeg(1));
    setPropIfGraphics(state.sensorSectorHiField, 'Value', sr.sectorDeg(2));
    setPropIfGraphics(state.sensorPdField,       'Value', sr.pd);
    setPropIfGraphics(state.sensorFarField,      'Value', sr.far);

    % ── Scan-mode banner + conditional widget visibility ───────────────
    %  Dispatch order mirrors drawSensor2D: isRotator → isNoScan →
    %  isSector → else. isNoScan MUST come before isSector because the
    %  post-M6-checkin2 isSector() accepts rpm=0 (to let PAR classify
    %  correctly), which means TWS's sectorDeg=[-60 60] also satisfies
    %  isSector even though the visual is a beam cone. Keeping isNoScan
    %  first ensures the banner text matches what drawSensor2D draws.
    if sr.isRotator()
        scanText      = 'Scan mode: Rotator (360° sweep)';
        sectorVisible = 'off';
    elseif sr.isNoScan()
        scanText      = 'Scan mode: No-scan / staring (RPM = 0)';
        sectorVisible = 'on';
    elseif sr.isSector()
        scanText      = sprintf('Scan mode: Sector (%.0f° → %.0f°)', ...
                                sr.sectorDeg(1), sr.sectorDeg(2));
        sectorVisible = 'on';
    else
        scanText      = 'Scan mode: custom (check RPM and sector values)';
        sectorVisible = 'on';
    end
    % ── M6 §3.6B fold-in — altitude-window readout ─────────────────
    %  Append the lower/upper beam-edge altitudes at max range so users
    %  can sanity-check why a waypoint of a given altitude does or
    %  doesn't fall inside coverage. Formula matches the simulator's
    %  understanding of the beam geometry:
    %      alt_low  = mountAlt + maxRange * tand(-fov_el/2 + tilt)
    %      alt_high = mountAlt + maxRange * tand(+fov_el/2 + tilt)
    %  Down-tilt is negative tilt → lower edge stays lower. We display
    %  the raw physics (no ground clipping) — the 3D cone rendering in
    %  drawMap clips below 0 for cosmetic reasons, but the banner is a
    %  debug readout and should show the true edges.
    mountAltBan = -sr.mountingLoc(3);
    if mountAltBan <= 0; mountAltBan = 15; end
    fovEl       = max(0, sr.fov(2));
    rMaxBan     = sr.rangeLimits(2);
    % v3.7.9 - rotators/sectors now draw the sim's [-fov-tilt, +fov-tilt] band
    % (see beamAltEdgesM); no-scan staring cones keep the +/-fov/2+tilt edges
    % (drawBeamConeVolume3D centers the cone at tilt with half-angle fov/2).
    if sr.isRotator() || sr.isSector()
        altLowBan   = mountAltBan + rMaxBan * tand(-fovEl - sr.tilt);
        altHighBan  = mountAltBan + rMaxBan * tand(+fovEl - sr.tilt);
    else
        altLowBan   = mountAltBan + rMaxBan * tand(-fovEl/2 + sr.tilt);
        altHighBan  = mountAltBan + rMaxBan * tand(+fovEl/2 + sr.tilt);
    end
    scanText    = sprintf('%s  |  Alt @ max range: %.0f–%.0f m', ...
                          scanText, altLowBan, altHighBan);
    setPropIfGraphics(state.sensorScanModeLbl,   'Text',    scanText);
    setPropIfGraphics(state.sensorRpmLabel,      'Visible', 'on');
    setPropIfGraphics(state.sensorRpmField,      'Visible', 'on');
    setPropIfGraphics(state.sensorSectorLoLabel, 'Visible', sectorVisible);
    setPropIfGraphics(state.sensorSectorLoField, 'Visible', sectorVisible);
    setPropIfGraphics(state.sensorSectorHiLabel, 'Visible', sectorVisible);
    setPropIfGraphics(state.sensorSectorHiField, 'Visible', sectorVisible);

    % Read-only enforcement for UNKNOWN passthrough sensors.
    ro = state.activeSensorIsReadOnly();
    setSensorFieldEnable(state, ~ro);
end


function setSensorFieldEnable(state, onoff)
%setSensorFieldEnable  Coarse enable/disable on every editable widget
%                       in the Sensor Parameters panel. Place-on-map
%                       stays disabled regardless until §3.6.
    list = {state.sensorNameField, state.sensorTypeDD, ...
            state.sensorEastField, state.sensorNorthField, ...
            state.sensorAltField, state.sensorFreqField, ...
            state.sensorMaxRangeField, state.sensorRangeResField, ...
            state.sensorAzFovField, state.sensorElFovField, ...
            state.sensorTiltField, state.sensorRpmField, ...
            state.sensorSectorLoField, state.sensorSectorHiField, ...
            state.sensorPdField, state.sensorFarField};
    enableStr = 'off'; if onoff; enableStr = 'on'; end
    for k = 1:numel(list)
        if isgraphics(list{k}); list{k}.Enable = enableStr; end
    end
end


function applyEditMode(state)
%applyEditMode  Coarse mode-gate across the sidebar. Three exclusive
%                modes: Targets, Sensors, Environment. Only the active
%                mode's panels are visible — the other two are hidden
%                outright (rows collapsed to 0 px in the sidebar grid)
%                so the panel relevant to the active mode dominates.
%
%  v3.5 step 4a CHANGE — hide-rather-than-gray.
%    Pre-v3.5, non-active panels stayed visible but disabled. Users
%    found the dimmed-but-present panels confusing: "What am I
%    supposed to interact with right now?" The new behavior collapses
%    the parent grid's RowHeight entry for hidden panels to 0 AND
%    flips Visible='off' on the panel itself (defense in depth). The
%    Sensors/Targets/Environment buttons act as a true context switch.
%
%  M7 CHANGE — three modes, not two. Environment mode (new in M7) gates
%  the Terrain + Weather sub-panels. The pre-M7 two-mode layout lives on
%  via targetsOn/sensorsOn; environmentOn is the new axis.
%
%  ROW-TO-MODE MAPPING (must match buildUI's inner.RowHeight order):
%    Row  1: Mode toggle    — always visible
%    Row  2: Sensors        — sensors mode only
%    Row  3: Sensor Params  — sensors mode only
%    Row  4: Targets        — targets mode only
%    Row  5: Scenario       — targets mode only
%    Row  6: Terrain        — environment mode only
%    Row  7: Weather        — environment mode only
%    Row  8: Selection      — targets mode only
%    Row  9: File           — always visible
%    Row 10: Undo           — always visible
%    Row 11: Help           — always visible
%
%  Fine-grained refreshes (refreshScenarioPanel / refreshSelectionPanel /
%  refreshSensorParamsPanel / refreshTerrainPanel / refreshWeatherPanel)
%  run at the end to re-assert stricter rules (reference read-only,
%  has-active, readOnly passthrough). They're still useful even though
%  the panels are hidden — a panel becoming visible again should land
%  with all its read-only/has-active state already applied, so the user
%  doesn't see a flash of incorrectly-enabled controls.
%
%  ROUTING NOTE
%    Every touched handle is isgraphics-guarded via setEnableIfGraphics
%    so snapshot-restore races (restore fires before buildUI finishes)
%    cannot throw.
    mode          = lower(string(state.editMode));
    targetsOn     = (mode == "targets");
    sensorsOn     = (mode == "sensors");
    environmentOn = (mode == "environment");

    % ── Panel-level show/hide (v3.5 step 4a) ───────────────────────
    %  First flip the panels' Visible flag, then collapse the parent
    %  grid's row heights. We do both because either one alone would
    %  leave artifacts:
    %    - Visible-only: row stays its full height, leaving a blank gap.
    %    - RowHeight-only: panel can flicker as MATLAB tries to render
    %      it before the grid resizes; also some uipanel ancestors keep
    %      reserving space when row is 0 unless Visible is also off.
    %  Doing both gives a clean instant context switch.
    setPanelVisibleIfGraphics(state.targetsPanel,      targetsOn);
    setPanelVisibleIfGraphics(state.scenarioPanel,     targetsOn);
    setPanelVisibleIfGraphics(state.selectedPanel,     targetsOn);
    setPanelVisibleIfGraphics(state.sensorsPanel,      sensorsOn);
    setPanelVisibleIfGraphics(state.sensorParamsPanel, sensorsOn);
    % v3.5 §5c.2 — environment splits into two sub-modes:
    %   fallback : edits state.terrain / state.weather (Terrain + Weather panels)
    %   regions  : edits terrainRegions / weatherRegions (the two new panels)
    % The sub-mode toggle itself is shown whenever environmentOn.
    envFallbackOn = environmentOn && (state.envSubMode == "fallback");
    envRegionsOn  = environmentOn && (state.envSubMode == "regions");
    setPanelVisibleIfGraphics(state.envSubModePanel,       environmentOn);
    setPanelVisibleIfGraphics(state.terrainPanel,          envFallbackOn);
    setPanelVisibleIfGraphics(state.weatherPanel,          envFallbackOn);
    setPanelVisibleIfGraphics(state.terrainRegionsPanel,   envRegionsOn);
    setPanelVisibleIfGraphics(state.weatherRegionsPanel,   envRegionsOn);

    % Collapse hidden rows to 0 px in the parent grid so visible panels
    % shift up to fill the space. Originals are restored from the
    % snapshot captured at build time.
    if isgraphics(state.editorInnerGrid) && ~isempty(state.editorInnerOriginalRowHeights)
        rh = state.editorInnerOriginalRowHeights;
        if ~sensorsOn
            rh{2} = 0;  % Sensors
            rh{3} = 0;  % Sensor Params
        end
        if ~targetsOn
            rh{4}  = 0;  % Targets
            rh{5}  = 0;  % Scenario
            rh{11} = 0;  % Selection (was row 8 pre-5c.2)
        end
        if ~environmentOn
            rh{6}  = 0;  % Env sub-mode toggle (5c.2)
        end
        if ~envFallbackOn
            rh{7} = 0;  % Terrain (was row 6 pre-5c.2)
            rh{8} = 0;  % Weather (was row 7 pre-5c.2)
        end
        if ~envRegionsOn
            rh{9}  = 0;  % Terrain Regions (5c.2)
            rh{10} = 0;  % Weather Regions (5c.2)
        end
        state.editorInnerGrid.RowHeight = rh;
    end

    % M6 §3.6C — Leaving sensors mode disarms any pending Place-on-map.
    % Without this, the user can arm Place in Sensors mode, switch to
    % Targets or Environment, click the map (intending to add a waypoint
    % or interact with the overlay), and instead teleport the armed
    % sensor to that point — very confusing. An in-flight drag is a
    % transient pointer operation that cannot survive a mode switch
    % either; just clear the state.
    if ~sensorsOn
        state.sensorPlacePending = false;
        if state.sensorDragActive
            state.sensorDragActive = false;
            state.sensorDragStart  = [];
        end
    end

    % ── Sensors sub-panel gate ─────────────────────────────────────
    setEnableIfGraphics(state.sensorsDD,      sensorsOn);
    setEnableIfGraphics(state.sensorsBtnAdd,  sensorsOn);
    setEnableIfGraphics(state.sensorsBtnLoad, sensorsOn);   % v3.5 step 4c
    if sensorsOn
        toggleSensorsButtons(state, state.hasActiveSensor());
    else
        setEnableIfGraphics(state.sensorsBtnDuplicate, false);
        setEnableIfGraphics(state.sensorsBtnDelete,    false);
        setEnableIfGraphics(state.sensorsBtnRename,    false);
        setEnableIfGraphics(state.sensorsBtnSave,      false);   % v3.5 step 4c
    end

    % ── Sensor-params gate ─────────────────────────────────────────
    setSensorFieldEnable(state, sensorsOn && state.hasActiveSensor());
    % Place-on-map (§3.5A) — only enabled in sensors mode with a writable
    % active sensor. Defense-in-depth: the callback re-checks editMode
    % and readOnly, so a stale Enable won't do damage.
    placeOn = sensorsOn && state.hasActiveSensor() && ~state.activeSensorIsReadOnly();
    setEnableIfGraphics(state.sensorPlaceOnMapBtn, placeOn);

    % ── Targets sub-panel gate ─────────────────────────────────────
    setEnableIfGraphics(state.targetsDD,            targetsOn);
    setEnableIfGraphics(state.targetsBtnNew,        targetsOn);
    setEnableIfGraphics(state.targetsBtnNasaFlight, targetsOn);   % v3.5 step 4c
    setEnableIfGraphics(state.targetsBtnLoad,       targetsOn);   % v3.5 step 4c
    setEnableIfGraphics(state.targetsBtnLoadRef,    targetsOn);   % v3.5 step 4c
    setEnableIfGraphics(state.targetsBtnUnloadRefs, targetsOn);   % v3.5 step 4c
    if targetsOn
        toggleTargetsButtons(state, state.hasActiveTarget());
    else
        setEnableIfGraphics(state.targetsBtnDuplicate, false);
        setEnableIfGraphics(state.targetsBtnDelete,    false);
        setEnableIfGraphics(state.targetsBtnRename,    false);
        setEnableIfGraphics(state.targetsBtnSave,      false);   % v3.5 step 4c
        setEnableIfGraphics(state.targetsBtnClear,     false);   % v3.5 step 4c
    end

    % ── Scenario panel gate ────────────────────────────────────────
    %  v3.5 §5d: state.viewModeBtn was moved out of the Scenario panel
    %  and up into the always-visible Mode Toggle panel, so it is no
    %  longer in this gate list — it must stay enabled in every mode.
    scenarioFields = {state.nameField,       state.speedField, ...
                      state.altField,        state.rcsField, ...
                      state.rcsProfileDD,    state.curveModeBtn, ...
                      state.curveTensionDD,  state.colorByAltCheckbox, ...
                      state.gridSpacingDD,   state.previewBtn};
    for k = 1:numel(scenarioFields)
        setEnableIfGraphics(scenarioFields{k}, targetsOn);
    end

    % ── Selection panel gate ───────────────────────────────────────
    selFields = {state.selFieldX, state.selFieldY, ...
                 state.selFieldAlt, state.selFieldSpeed};
    hasSel = state.selectedIndex >= 1;
    for k = 1:numel(selFields)
        setEnableIfGraphics(selFields{k}, targetsOn && hasSel);
    end

    % ── Environment (Terrain + Weather) gate ────────────────────────
    % Coarse gate — disable every environment widget when not in env
    % mode. refreshTerrainPanel / refreshWeatherPanel layer fine-grained
    % rules (readOnly UNKNOWN passthrough, empty-weather) on top when
    % env mode is active.
    terrainFields = {state.terrainTypeDD,        state.terrainDescField, ...
                     state.terrainScaleField,    state.terrainClutterField, ...
                     state.terrainRefractionField, state.terrainLoadBtn, ...
                     state.terrainSaveBtn,       state.terrainOverlayCB};
    degFields = {state.degTerrainOcclusionCB, state.degHorizonMaskingCB, ...
                 state.degGroundClutterCB,    state.degDopplerFadeCB};
    weatherFields = {state.weatherTypeDD,          state.weatherDescField, ...
                     state.weatherRateField,       state.weatherStormStartField, ...
                     state.weatherStormEndField,   state.weatherProfileDD, ...
                     state.weatherPdFloorField,    state.weatherClutterMultField, ...
                     state.weatherLoadBtn,         state.weatherSaveBtn};
    if environmentOn
        % Fine-grained refresh takes over enable logic for these widgets.
        refreshTerrainPanel(state);
        refreshWeatherPanel(state);
        % v3.5 §5c.2 — regions panels + sub-mode toggle re-sync. Refresh
        % regardless of which sub-mode is active so when the user flips
        % the toggle, the newly-shown panel is already current. The
        % buildUI seed-time call also goes through here when env mode is
        % the initial editMode.
        refreshTerrainRegionsPanel(state);
        refreshWeatherRegionsPanel(state);
        refreshEnvSubModePanel(state);
    else
        for k = 1:numel(terrainFields)
            setEnableIfGraphics(terrainFields{k}, false);
        end
        for k = 1:numel(degFields)
            setEnableIfGraphics(degFields{k}, false);
        end
        for k = 1:numel(weatherFields)
            setEnableIfGraphics(weatherFields{k}, false);
        end
    end

    % ── Re-run finer-grained refreshers so read-only / has-active
    %     rules apply on top of the coarse mode gate.
    if targetsOn
        refreshScenarioPanel(state);
        refreshSelectionPanel(state);
    elseif sensorsOn
        refreshSensorParamsPanel(state);
    end
    % Environment mode's fine-grained refresh already ran above.
end


function setEnableIfGraphics(h, onoff)
%setEnableIfGraphics  Flip Enable on a graphics handle if it's alive.
%                     Accepts boolean/numeric (true/1 → 'on') or string
%                     ('on'/'off') for onoff. Silent no-op on dead
%                     handles so snapshot-restore races don't throw.
    if ~isgraphics(h); return; end
    if islogical(onoff) || isnumeric(onoff)
        if onoff; h.Enable = 'on'; else; h.Enable = 'off'; end
    else
        h.Enable = char(onoff);
    end
end


function setPanelVisibleIfGraphics(h, onoff)
%setPanelVisibleIfGraphics  Flip Visible on a panel handle if alive.
%                           Sister to setEnableIfGraphics; used by
%                           applyEditMode to hide/show entire sub-
%                           panels for the v3.5 step 4a mode-specific
%                           layout. Silent no-op on dead handles.
    if ~isgraphics(h); return; end
    if islogical(onoff) || isnumeric(onoff)
        if onoff; h.Visible = 'on'; else; h.Visible = 'off'; end
    else
        h.Visible = char(onoff);
    end
end


function refreshTargetsDropdown(state)
%refreshTargetsDropdown  Rebuild the Targets dropdown to match
%                        state.targets and state.activeIdx.
%
%  uidropdown is fussy about Items / ItemsData / Value consistency —
%  setting Value to something not in ItemsData throws. Build them in
%  the safe order: Items first, ItemsData second, Value last. Use
%  ItemsData = numeric indices so the callback can map directly to
%  setActiveIdx without parsing display text.
%
%  Display text format: "<n>" plus an "(active)" tag for the
%  currently-active target (visual confirmation that the dropdown
%  selection matches what's in the editor).
    if ~isgraphics(state.targetsDD)
        return;
    end
    n = numel(state.targets);
    if n == 0
        state.targetsDD.Items     = {'(no targets)'};
        state.targetsDD.ItemsData = {0};
        state.targetsDD.Value     = 0;
        toggleTargetsButtons(state, false);
        return;
    end
    items = cell(1, n);
    data  = cell(1, n);
    for k = 1:n
        nm = char(state.targets(k).targetName);
        isActive = (k == state.activeIdx);
        isRef    = state.targets(k).readOnly;
        %  M5 §3.2 — dropdown label format:
        %    writable + inactive : "name"
        %    writable + active   : "name  (active)"
        %    reference + inactive: "name  (ref)"
        %    reference + active  : "name  (active, ref)"
        %  The (ref) tag is the only signal — besides the panel banner —
        %  that a target is read-only, so keep it concise but visible.
        if isActive && isRef
            items{k} = sprintf('%s  (active, ref)', nm);
        elseif isActive
            items{k} = sprintf('%s  (active)', nm);
        elseif isRef
            items{k} = sprintf('%s  (ref)', nm);
        else
            items{k} = nm;
        end
        data{k} = k;
    end
    state.targetsDD.Items     = items;
    state.targetsDD.ItemsData = data;
    if state.activeIdx >= 1 && state.activeIdx <= n
        state.targetsDD.Value = state.activeIdx;
    else
        state.targetsDD.Value = 1;
    end
    toggleTargetsButtons(state, true);
end


function toggleTargetsButtons(state, hasActive)
%toggleTargetsButtons  Enable / disable Duplicate, Delete, Rename, Save,
%                      Clear based on whether there's an active target.
%                      New, Import NASA Flight, Load Target, Load as
%                      Reference are always enabled. v3.5 step 4c added
%                      Save and Clear to the gated set.
    enable = 'off'; if hasActive; enable = 'on'; end
    if isgraphics(state.targetsBtnDuplicate); state.targetsBtnDuplicate.Enable = enable; end
    if isgraphics(state.targetsBtnDelete);    state.targetsBtnDelete.Enable    = enable; end
    if isgraphics(state.targetsBtnRename);    state.targetsBtnRename.Enable    = enable; end
    if isgraphics(state.targetsBtnSave);      state.targetsBtnSave.Enable      = enable; end
    if isgraphics(state.targetsBtnClear);     state.targetsBtnClear.Enable     = enable; end
end


function refreshScenarioPanel(state)
%refreshScenarioPanel  Re-sync Scenario panel fields from the active
%                      TargetRecord.
%
%  Called whenever the active target changes (active-idx switch, New,
%  Duplicate, Delete, Rename, Load with replace). The fields here all
%  proxy through dependent properties, but the UI widgets cache their
%  Value — without explicit re-sync, the Scenario panel would show the
%  PREVIOUS active target's values until the user typed in each field.
%
%  M5 §3.2 — READ-ONLY HANDLING
%    When the active target is a reference (readOnly==true) we:
%      1. Swap the panel title to a banner that explains the state
%         ("Scenario — REFERENCE (read-only; Duplicate to edit)").
%      2. Disable every editable widget in the panel (name, speed,
%         altitude, RCS, RCS profile, curve mode toggle, curve tension,
%         color-by-altitude checkbox, grid spacing dropdown).
%    The view-only widgets (waypoint count, view-mode toggle, preview
%    button) stay enabled because they don't mutate the target. The
%    Apply-default-altitude button is disabled because it would mutate
%    waypoints.
%
%    This is the third layer in the readOnly defense:
%       UI disable (this) + EditorState mutator guards + dependent-setter
%       guards. Any one layer would in theory be enough, but together
%       they make read-only enforcement obvious to both users and any
%       future maintainer reading the code.
%
%  M5 GOTCHA — Limits-before-Value: uieditfield raises if Value falls
%  outside Limits. Our editfields all have Limits set at construct
%  time and the TargetRecord defaults respect them, so this is safe
%  here. If a future field gets a tighter Limits range, set Limits
%  before Value to be defensive.
    if ~state.hasActiveTarget()
        return;
    end
    tr = state.activeTarget();
    if isgraphics(state.nameField)
        state.nameField.Value = char(tr.targetName);
    end
    if isgraphics(state.speedField)
        state.speedField.Value = tr.defaultSpeedKmh;
    end
    if isgraphics(state.altField)
        state.altField.Value = tr.defaultAltitudeM;
    end
    if isgraphics(state.rcsField)
        state.rcsField.Value = tr.rcsDbsm;
    end
    if isgraphics(state.rcsProfileDD)
        state.rcsProfileDD.Value = char(tr.rcsProfile);
    end
    if isgraphics(state.curveModeBtn)
        % uibutton 'state' Value is logical — true ⇒ "curved"
        state.curveModeBtn.Value = strcmp(char(tr.curveMode), 'curved');
        state.curveModeBtn.Text  = curveButtonText(tr.curveMode);
    end
    if isgraphics(state.curveTensionDD)
        state.curveTensionDD.Value = tr.curveTensionAlpha;
    end

    % ── Read-only enable/disable + title banner ───────────────────────
    isRO = state.activeIsReadOnly();
    if isRO
        enableState = 'off';
        titleText   = 'Scenario  —  REFERENCE  (read-only; Duplicate to edit)';
    else
        enableState = 'on';
        titleText   = 'Scenario';
    end
    if isgraphics(state.scenarioPanel)
        state.scenarioPanel.Title = titleText;
        if isRO
            state.scenarioPanel.ForegroundColor = [0.70 0.30 0.10];
        else
            state.scenarioPanel.ForegroundColor = [0 0 0];
        end
    end
    setPropIfGraphics(state.nameField,          'Enable', enableState);
    setPropIfGraphics(state.speedField,         'Enable', enableState);
    setPropIfGraphics(state.altField,           'Enable', enableState);
    setPropIfGraphics(state.rcsField,           'Enable', enableState);
    setPropIfGraphics(state.rcsProfileDD,       'Enable', enableState);
    setPropIfGraphics(state.curveModeBtn,       'Enable', enableState);
    setPropIfGraphics(state.curveTensionDD,     'Enable', enableState);
    setPropIfGraphics(state.colorByAltCheckbox, 'Enable', enableState);
    setPropIfGraphics(state.gridSpacingDD,      'Enable', enableState);
end


function setPropIfGraphics(h, prop, value)
%setPropIfGraphics  Defensive property setter — no-op if the handle
%                   isn't valid. Used by refreshScenarioPanel so the
%                   read-only enable cascade survives the construction
%                   window during which handles haven't been populated
%                   yet.
%
%  NOTE: distinct from the single-value setIfGraphics(h, v) helper used
%  in refreshSelectionPanel. That one only sets Value; this one sets
%  any property by name. Kept separate to preserve both call sites'
%  historical signatures without forcing a rewrite of selection-panel
%  code.
    if ~isgraphics(h)
        return;
    end
    % Clamp a numeric Value into the component Limits before setting, so a
    % value outside a radar-shaped field's range (e.g. a sonar/IR sensor's
    % frequency, meaningless in the GHz field) does not throw "Value must be
    % within Limits" and crash the panel refresh. (Phase 2 gives sonar/IR
    % their own per-modality fields.)
    if strcmp(prop, 'Value') && isnumeric(value) && isscalar(value) && isprop(h, 'Limits')
        lim = h.Limits;
        if isnumeric(lim) && numel(lim) == 2 && all(isfinite(lim))
            value = min(max(value, lim(1)), lim(2));
        end
    end
    try
        h.(prop) = value;
    catch
        % Leave the control unchanged rather than crash the refresh.
    end
end

function refreshSelectionPanel(state)
%refreshSelectionPanel  Enable / disable and populate the sidebar fields
%                       based on state.selectedIndex.
    idx = state.selectedIndex;
    hasSel = idx >= 1 && idx <= state.count();
    enable = 'off'; if hasSel; enable = 'on'; end

    if isgraphics(state.selLabelIndex)
        if hasSel
            state.selLabelIndex.Text = sprintf('%d of %d', idx, state.count());
        else
            state.selLabelIndex.Text = '—';
        end
    end

    flds = {state.selFieldX, state.selFieldY, state.selFieldAlt, ...
            state.selFieldSpeed};
    for i = 1:numel(flds)
        if isgraphics(flds{i}); flds{i}.Enable = enable; end
    end
    if isgraphics(state.selFieldTime); state.selFieldTime.Enable = enable; end
    if isgraphics(state.selBtnDelete); state.selBtnDelete.Enable = enable; end
    if isgraphics(state.selBtnInsertAfter); state.selBtnInsertAfter.Enable = enable; end

    if hasSel
        row = state.waypoints(idx, :);
        setIfGraphics(state.selFieldX,     row(1));
        setIfGraphics(state.selFieldY,     row(2));
        setIfGraphics(state.selFieldAlt,   row(3));
        setIfGraphics(state.selFieldSpeed, row(5));
        setIfGraphics(state.selFieldTime,  row(4));
    else
        setIfGraphics(state.selFieldX,     0);
        setIfGraphics(state.selFieldY,     0);
        setIfGraphics(state.selFieldAlt,   0);
        % selFieldSpeed has Limits=[1 5000] — 0 would throw. Use default.
        setIfGraphics(state.selFieldSpeed, state.defaultSpeedKmh);
        setIfGraphics(state.selFieldTime,  0);
    end
end


function setIfGraphics(h, v)
    if isgraphics(h)
        h.Value = v;
    end
end


function updateWaypointCount(state)
    if isgraphics(state.waypointCountLbl)
        state.waypointCountLbl.Text = sprintf('%d', state.count());
    end
end


function setStatus(state, msg)
    if isgraphics(state.statusLabel)
        state.statusLabel.Text = msg;
    end
end


function sel = getSelectionType(fig)
%getSelectionType  uifigure SelectionType is empty until the first click;
%                  fall back to 'normal' in that case.
    sel = 'normal';
    try
        if ~isempty(fig.SelectionType)
            sel = fig.SelectionType;
        end
    catch
    end
end


function onClose(fig, state)
%onClose  Warn-before-close if there are unsaved waypoints OR unsaved
%         sensors. Pre-M6 this checked only `state.count() > 0` which is
%         the active target's waypoint count — so pure-sensor edits (e.g.
%         the user placed a sensor via §3.5A but didn't export) slipped
%         through silently. sensorsDirty closes that hole.
    targetsUnsaved = state.isDirty && state.count() > 0;
    sensorsUnsaved = state.sensorsDirty && ~isempty(state.sensors);
    if targetsUnsaved || sensorsUnsaved
        if targetsUnsaved && sensorsUnsaved
            msg = sprintf(['%d waypoint(s) and %d sensor(s) not yet exported. ' ...
                           'Close anyway?'], state.count(), numel(state.sensors));
            ttl = 'Unsaved waypoints and sensors';
        elseif targetsUnsaved
            msg = sprintf('%d waypoint(s) not yet exported. Close anyway?', ...
                state.count());
            ttl = 'Unsaved waypoints';
        else
            msg = sprintf('%d sensor(s) not yet exported. Close anyway?', ...
                numel(state.sensors));
            ttl = 'Unsaved sensors';
        end
        sel = uiconfirm(fig, msg, ttl, ...
            'Options', {'Close without exporting', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel == "Cancel"
            return;
        end
    end
    delete(fig);
end


%% ========================================================================
%  5c.2 — ENVIRONMENT SUB-MODE TOGGLE + REGIONS SUB-PANELS
%% ========================================================================
%  Three new sub-panels for the v3.5 §5c.2 region-editing UX. The sub-mode
%  toggle is hidden until Environment top-level mode is active; once it
%  is, the toggle picks between Fallback (existing Terrain + Weather
%  panels) and Regions (Terrain Regions + Weather Regions panels).
%
%  All builders here mirror existing patterns:
%    • buildEnvSubModeTogglePanel   — mirrors buildModeTogglePanel
%                                     (state-button radio pair, mutual-
%                                     exclusion enforced in callbacks).
%    • buildTerrainRegionsPanel     — dropdown + Add/Dup/Del strip plus
%      buildWeatherRegionsPanel       a read-only-ish active-region
%                                     section (editable name field,
%                                     read-only configPath label,
%                                     polygon vertex count badge,
%                                     Edit Polygon + Change Config
%                                     buttons). Edit Polygon is greyed
%                                     in 5c.2 — it activates in 5c.3
%                                     when polygon-edit sub-mode lands.
%
%  All callbacks push undo via the EditorState mutators (which already
%  do pushUndo internally for region operations — see 5c.1). The view-
%  state setters (setActive*RegionIdx, setEnvSubMode) do NOT push undo,
%  matching the existing setEditMode / setActiveSensorIdx patterns.

function buildEnvSubModeTogglePanel(parent, state)
%buildEnvSubModeTogglePanel  v3.5 §5c.2 — sub-mode toggle (row 6, 50 px).
%
%  Layout:
%    ┌─ Environment sub-mode ─────────────────────────────────┐
%    │ [ Fallback ]                      [ Regions      ]      │
%    └─────────────────────────────────────────────────────────────┘
%
%  Two state-buttons acting as a mutually-exclusive radio pair. Mutual
%  exclusion enforced in onEnvSubMode{Fallback,Regions} — same pattern
%  as the top-level mode toggle. Panel itself is shown/hidden by
%  applyEditMode based on environmentOn.
    pnl = uipanel(parent, 'Title', 'Environment sub-mode');
    state.envSubModePanel = pnl;

    g = uigridlayout(pnl, [1 2]);
    g.RowHeight   = {26};
    g.ColumnWidth = {'1x', '1x'};
    g.Padding     = [6 4 6 4];
    g.ColumnSpacing = 6;

    isFallback = (state.envSubMode == "fallback");
    state.envSubModeFallbackBtn = uibutton(g, 'state', ...
        'Text', 'Fallback', ...
        'FontWeight', 'bold', ...
        'Value', isFallback, ...
        'Tooltip', 'Edit the scenario-wide fallback terrain and weather.', ...
        'ValueChangedFcn', @(src, ~) onEnvSubModeFallback(src, state));
    state.envSubModeRegionsBtn = uibutton(g, 'state', ...
        'Text', 'Regions', ...
        'FontWeight', 'bold', ...
        'Value', ~isFallback, ...
        'Tooltip', 'Edit terrain and weather regions (polygons over the fallback).', ...
        'ValueChangedFcn', @(src, ~) onEnvSubModeRegions(src, state));
end


function buildTerrainRegionsPanel(parent, state)
%buildTerrainRegionsPanel  v3.5 §5c.2 — Terrain Regions sub-panel
%                          (row 9, 250 px).
%
%  Layout:
%    ┌─ Terrain Regions ───────────────────────────────┐
%    │ Region   [ <none>                 ] ▼                  │
%    │ [ + Add ] [ Duplicate ] [ Delete ]                     │
%    │ ──── Active region ───────────────────────────│
%    │ Name     [ <name>                                    ] │
%    │ Config:    rural/default_rural                         │
%    │ Polygon:   0 vertices  ⚠ invalid                        │
%    │ [ Edit polygon… ]   [ Change config… ]                  │
%    └────────────────────────────────────────────────────────────┘
%
%  Mirrors buildTargetsPanel / buildSensorsPanel structure: dropdown +
%  collection-management button strip, then a per-active-region read-out
%  pane with its own action buttons. Inline name editing (rename via
%  ValueChangedFcn on the name field) replaces the explicit Rename
%  button — simpler and matches the design we settled on for 5c.2.
%
%  Edit Polygon button is constructed with Enable='off' for 5c.2; 5c.3
%  will flip it on when polygon-edit sub-mode lands. Change Config is
%  fully wired to onTerrainRegionsChangeConfig.
    pnl = uipanel(parent, 'Title', 'Terrain Regions');
    state.terrainRegionsPanel = pnl;

    g = uigridlayout(pnl, [7 2]);
    g.RowHeight   = {26, 28, 4, 26, 22, 22, 28};
    g.ColumnWidth = {72, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;
    g.ColumnSpacing = 6;

    % Row 1 — Region picker
    lblRegion = uilabel(g, 'Text', 'Region', 'HorizontalAlignment', 'right');
    lblRegion.Layout.Row = 1; lblRegion.Layout.Column = 1;
    state.terrainRegionsDD = uidropdown(g, ...
        'Items', {'(no regions)'}, 'ItemsData', {0}, 'Value', 0, ...
        'Tooltip', 'Active terrain region. Pick another to edit it.', ...
        'ValueChangedFcn', @(src, ~) onTerrainRegionDDChanged(src, state));
    state.terrainRegionsDD.Layout.Row = 1;
    state.terrainRegionsDD.Layout.Column = 2;

    % Row 2 — Add / Dup / Delete strip
    btns = uigridlayout(g, [1 3]);
    btns.Layout.Row = 2; btns.Layout.Column = [1 2];
    btns.RowHeight = {28};
    btns.ColumnWidth = {'1x', '1x', '1x'};
    btns.Padding = [0 0 0 0];
    btns.ColumnSpacing = 4;
    state.terrainRegionsBtnAdd = uibutton(btns, 'push', 'Text', '+ Add', ...
        'Tooltip', 'Add a new terrain region seeded from the fallback.', ...
        'ButtonPushedFcn', @(~, ~) onTerrainRegionsAdd(state));
    state.terrainRegionsBtnDuplicate = uibutton(btns, 'push', 'Text', 'Duplicate', ...
        'Tooltip', 'Copy the active region (polygon offset 2 km east).', ...
        'ButtonPushedFcn', @(~, ~) onTerrainRegionsDuplicate(state));
    state.terrainRegionsBtnDelete = uibutton(btns, 'push', 'Text', 'Delete', ...
        'BackgroundColor', [0.90 0.40 0.40], 'FontColor', 'white', ...
        'Tooltip', 'Remove the active terrain region (undoable).', ...
        'ButtonPushedFcn', @(~, ~) onTerrainRegionsDelete(state));

    % Row 3 — thin separator (visual breathing room before the active-region readout)
    sep = uilabel(g, 'Text', '', 'BackgroundColor', [0.85 0.85 0.85]);
    sep.Layout.Row = 3; sep.Layout.Column = [1 2];

    % Row 4 — Name (inline editable; rename via ValueChangedFcn)
    lblName = uilabel(g, 'Text', 'Name', 'HorizontalAlignment', 'right');
    lblName.Layout.Row = 4; lblName.Layout.Column = 1;
    state.terrainRegionsNameField = uieditfield(g, 'text', 'Value', '', ...
        'Tooltip', 'Active region name. Press Enter to rename (must be unique).', ...
        'ValueChangedFcn', @(src, ~) onTerrainRegionsNameFieldChanged(src, state));
    state.terrainRegionsNameField.Layout.Row = 4;
    state.terrainRegionsNameField.Layout.Column = 2;

    % Row 5 — Config (read-only label)
    lblConfig = uilabel(g, 'Text', 'Config:', 'HorizontalAlignment', 'right');
    lblConfig.Layout.Row = 5; lblConfig.Layout.Column = 1;
    state.terrainRegionsConfigLabel = uilabel(g, 'Text', '—', ...
        'FontColor', [0.30 0.30 0.30]);
    state.terrainRegionsConfigLabel.Layout.Row = 5;
    state.terrainRegionsConfigLabel.Layout.Column = 2;

    % Row 6 — Polygon status (vertex count + valid/invalid)
    lblPoly = uilabel(g, 'Text', 'Polygon:', 'HorizontalAlignment', 'right');
    lblPoly.Layout.Row = 6; lblPoly.Layout.Column = 1;
    state.terrainRegionsPolygonStatusLabel = uilabel(g, 'Text', '—', ...
        'FontColor', [0.30 0.30 0.30]);
    state.terrainRegionsPolygonStatusLabel.Layout.Row = 6;
    state.terrainRegionsPolygonStatusLabel.Layout.Column = 2;

    % Row 7 — action buttons (Edit polygon greyed in 5c.2, Change config wired)
    actBtns = uigridlayout(g, [1 2]);
    actBtns.Layout.Row = 7; actBtns.Layout.Column = [1 2];
    actBtns.RowHeight = {28};
    actBtns.ColumnWidth = {'1x', '1x'};
    actBtns.Padding = [0 0 0 0];
    actBtns.ColumnSpacing = 4;
    state.terrainRegionsBtnEditPolygon = uibutton(actBtns, 'push', ...
        'Text', 'Edit polygon…', ...
        'Enable', 'off', ...
        'Tooltip', 'Click to draw a polygon on the map (activates in 5c.3).', ...
        'ButtonPushedFcn', @(~, ~) onTerrainRegionsEditPolygon(state));
    state.terrainRegionsBtnChangeConfig = uibutton(actBtns, 'push', ...
        'Text', 'Change config…', ...
        'Tooltip', 'Pick a different terrain config file for this region.', ...
        'ButtonPushedFcn', @(~, ~) onTerrainRegionsChangeConfig(state));
end


function buildWeatherRegionsPanel(parent, state)
%buildWeatherRegionsPanel  v3.5 §5c.2 — Weather Regions sub-panel
%                          (row 10, 250 px). Mirror of
%                          buildTerrainRegionsPanel for weatherRegions.
    pnl = uipanel(parent, 'Title', 'Weather Regions');
    state.weatherRegionsPanel = pnl;

    g = uigridlayout(pnl, [7 2]);
    g.RowHeight   = {26, 28, 4, 26, 22, 22, 28};
    g.ColumnWidth = {72, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;
    g.ColumnSpacing = 6;

    lblRegion = uilabel(g, 'Text', 'Region', 'HorizontalAlignment', 'right');
    lblRegion.Layout.Row = 1; lblRegion.Layout.Column = 1;
    state.weatherRegionsDD = uidropdown(g, ...
        'Items', {'(no regions)'}, 'ItemsData', {0}, 'Value', 0, ...
        'Tooltip', 'Active weather region. Pick another to edit it.', ...
        'ValueChangedFcn', @(src, ~) onWeatherRegionDDChanged(src, state));
    state.weatherRegionsDD.Layout.Row = 1;
    state.weatherRegionsDD.Layout.Column = 2;

    btns = uigridlayout(g, [1 3]);
    btns.Layout.Row = 2; btns.Layout.Column = [1 2];
    btns.RowHeight = {28};
    btns.ColumnWidth = {'1x', '1x', '1x'};
    btns.Padding = [0 0 0 0];
    btns.ColumnSpacing = 4;
    state.weatherRegionsBtnAdd = uibutton(btns, 'push', 'Text', '+ Add', ...
        'Tooltip', 'Add a new weather region seeded from the fallback.', ...
        'ButtonPushedFcn', @(~, ~) onWeatherRegionsAdd(state));
    state.weatherRegionsBtnDuplicate = uibutton(btns, 'push', 'Text', 'Duplicate', ...
        'Tooltip', 'Copy the active region (polygon offset 2 km east).', ...
        'ButtonPushedFcn', @(~, ~) onWeatherRegionsDuplicate(state));
    state.weatherRegionsBtnDelete = uibutton(btns, 'push', 'Text', 'Delete', ...
        'BackgroundColor', [0.90 0.40 0.40], 'FontColor', 'white', ...
        'Tooltip', 'Remove the active weather region (undoable).', ...
        'ButtonPushedFcn', @(~, ~) onWeatherRegionsDelete(state));

    sep = uilabel(g, 'Text', '', 'BackgroundColor', [0.85 0.85 0.85]);
    sep.Layout.Row = 3; sep.Layout.Column = [1 2];

    lblName = uilabel(g, 'Text', 'Name', 'HorizontalAlignment', 'right');
    lblName.Layout.Row = 4; lblName.Layout.Column = 1;
    state.weatherRegionsNameField = uieditfield(g, 'text', 'Value', '', ...
        'Tooltip', 'Active region name. Press Enter to rename (must be unique).', ...
        'ValueChangedFcn', @(src, ~) onWeatherRegionsNameFieldChanged(src, state));
    state.weatherRegionsNameField.Layout.Row = 4;
    state.weatherRegionsNameField.Layout.Column = 2;

    lblConfig = uilabel(g, 'Text', 'Config:', 'HorizontalAlignment', 'right');
    lblConfig.Layout.Row = 5; lblConfig.Layout.Column = 1;
    state.weatherRegionsConfigLabel = uilabel(g, 'Text', '—', ...
        'FontColor', [0.30 0.30 0.30]);
    state.weatherRegionsConfigLabel.Layout.Row = 5;
    state.weatherRegionsConfigLabel.Layout.Column = 2;

    lblPoly = uilabel(g, 'Text', 'Polygon:', 'HorizontalAlignment', 'right');
    lblPoly.Layout.Row = 6; lblPoly.Layout.Column = 1;
    state.weatherRegionsPolygonStatusLabel = uilabel(g, 'Text', '—', ...
        'FontColor', [0.30 0.30 0.30]);
    state.weatherRegionsPolygonStatusLabel.Layout.Row = 6;
    state.weatherRegionsPolygonStatusLabel.Layout.Column = 2;

    actBtns = uigridlayout(g, [1 2]);
    actBtns.Layout.Row = 7; actBtns.Layout.Column = [1 2];
    actBtns.RowHeight = {28};
    actBtns.ColumnWidth = {'1x', '1x'};
    actBtns.Padding = [0 0 0 0];
    actBtns.ColumnSpacing = 4;
    state.weatherRegionsBtnEditPolygon = uibutton(actBtns, 'push', ...
        'Text', 'Edit polygon…', ...
        'Enable', 'off', ...
        'Tooltip', 'Click to draw a polygon on the map (activates in 5c.3).', ...
        'ButtonPushedFcn', @(~, ~) onWeatherRegionsEditPolygon(state));
    state.weatherRegionsBtnChangeConfig = uibutton(actBtns, 'push', ...
        'Text', 'Change config…', ...
        'Tooltip', 'Pick a different weather config file for this region.', ...
        'ButtonPushedFcn', @(~, ~) onWeatherRegionsChangeConfig(state));
end


%% ------------------------------------------------------------------------
%  5c.2 — REGIONS REFRESH HELPERS
%% ------------------------------------------------------------------------
%  Single source-of-truth re-syncers for the new sub-panels. Called by:
%    • buildUI seed time (after the panels are built, before applyEditMode)
%    • every region mutation callback (Add, Dup, Del, Rename, ChangeConfig)
%    • sub-mode toggle (so the panels show fresh data on first reveal)
%    • undo/redo (via refreshAfterEnvironmentChange — wired in Edit 7)

function refreshEnvSubModePanel(state)
%refreshEnvSubModePanel  Sync the Fallback/Regions state-buttons to
%                        state.envSubMode. View state only — does not
%                        touch the sub-panels themselves; that's
%                        applyEditMode's job.
    if ~isgraphics(state.envSubModeFallbackBtn); return; end
    if ~isgraphics(state.envSubModeRegionsBtn);  return; end
    isFallback = (state.envSubMode == "fallback");
    state.envSubModeFallbackBtn.Value = isFallback;
    state.envSubModeRegionsBtn.Value  = ~isFallback;
end


function refreshTerrainRegionsPanel(state)
%refreshTerrainRegionsPanel  Rebuild the Terrain Regions dropdown and
%                            populate the active-region read-out.
%
%  Items→ItemsData→Value ordering avoids the uidropdown "Value not in
%  ItemsData" throw — same pattern as refreshTargetsDropdown.
%
%  EMPTY COLLECTION
%    Dropdown shows "(no regions)" and the action buttons are disabled.
%    The Add button stays enabled so the user can create the first region.
%
%  ACTIVE REGION READOUT
%    Name field gets the active region's name (or '' when none).
%    Config label gets the configPath (or '—' when none).
%    Polygon label gets "N vertices  ✓ valid" or "N vertices  ⚠ invalid"
%    based on isValidPolygon (≥3 vertices). 0 vertices → "empty".
    if ~isgraphics(state.terrainRegionsDD); return; end
    n = numel(state.terrainRegions);

    if n == 0
        state.terrainRegionsDD.Items     = {'(no regions)'};
        state.terrainRegionsDD.ItemsData = {0};
        state.terrainRegionsDD.Value     = 0;
        setPropIfGraphics(state.terrainRegionsBtnDuplicate,    'Enable', 'off');
        setPropIfGraphics(state.terrainRegionsBtnDelete,       'Enable', 'off');
        setPropIfGraphics(state.terrainRegionsNameField,       'Enable', 'off');
        setPropIfGraphics(state.terrainRegionsBtnChangeConfig, 'Enable', 'off');
        % v3.5 §5c.3 fix — Edit Polygon must also be disabled when there
        % are no regions. Pre-fix, this branch left it at whatever state
        % the n>0 branch had set it to ('on'), so a Ctrl+Z that removed
        % a region left the button looking clickable but doing nothing
        % when pressed (early-return on ~hasActiveTerrainRegion).
        setPropIfGraphics(state.terrainRegionsBtnEditPolygon,  'Enable', 'off');
        setPropIfGraphics(state.terrainRegionsNameField, 'Value', '');
        setPropIfGraphics(state.terrainRegionsConfigLabel, 'Text', '—');
        setPropIfGraphics(state.terrainRegionsPolygonStatusLabel, ...
            'Text', '—');
        return;
    end

    items = cell(1, n);
    data  = cell(1, n);
    for k = 1:n
        rec = state.terrainRegions(k);
        nm = char(rec.name);
        if isempty(nm); nm = sprintf('(unnamed %d)', k); end
        if k == state.activeTerrainRegionIdx
            items{k} = sprintf('%s  (active)', nm);
        else
            items{k} = nm;
        end
        data{k} = k;
    end
    state.terrainRegionsDD.Items     = items;
    state.terrainRegionsDD.ItemsData = data;
    if state.activeTerrainRegionIdx >= 1 && state.activeTerrainRegionIdx <= n
        state.terrainRegionsDD.Value = state.activeTerrainRegionIdx;
    else
        state.terrainRegionsDD.Value = 1;
    end

    setPropIfGraphics(state.terrainRegionsBtnDuplicate,    'Enable', 'on');
    setPropIfGraphics(state.terrainRegionsBtnDelete,       'Enable', 'on');
    setPropIfGraphics(state.terrainRegionsNameField,       'Enable', 'on');
    setPropIfGraphics(state.terrainRegionsBtnChangeConfig, 'Enable', 'on');
    % v3.5 §5c.3 — Edit Polygon button activates when there's an active
    % region. The lockdown helper (applyPolygonEditLockdown) overrides
    % this when polygon-edit mode is active for ANY region.
    setPropIfGraphics(state.terrainRegionsBtnEditPolygon,  'Enable', 'on');

    % Active-region read-out.
    if state.hasActiveTerrainRegion()
        rec = state.terrainRegions(state.activeTerrainRegionIdx);
        setPropIfGraphics(state.terrainRegionsNameField, 'Value', char(rec.name));
        cfg = char(rec.configPath);
        if isempty(cfg); cfg = '—'; end
        setPropIfGraphics(state.terrainRegionsConfigLabel, 'Text', cfg);
        setPropIfGraphics(state.terrainRegionsPolygonStatusLabel, ...
            'Text', polygonStatusText(rec));
    else
        setPropIfGraphics(state.terrainRegionsNameField, 'Value', '');
        setPropIfGraphics(state.terrainRegionsConfigLabel, 'Text', '—');
        setPropIfGraphics(state.terrainRegionsPolygonStatusLabel, ...
            'Text', '—');
    end
end


function refreshWeatherRegionsPanel(state)
%refreshWeatherRegionsPanel  Mirror of refreshTerrainRegionsPanel for
%                            the weather collection.
    if ~isgraphics(state.weatherRegionsDD); return; end
    n = numel(state.weatherRegions);

    if n == 0
        state.weatherRegionsDD.Items     = {'(no regions)'};
        state.weatherRegionsDD.ItemsData = {0};
        state.weatherRegionsDD.Value     = 0;
        setPropIfGraphics(state.weatherRegionsBtnDuplicate,    'Enable', 'off');
        setPropIfGraphics(state.weatherRegionsBtnDelete,       'Enable', 'off');
        setPropIfGraphics(state.weatherRegionsNameField,       'Enable', 'off');
        setPropIfGraphics(state.weatherRegionsBtnChangeConfig, 'Enable', 'off');
        % v3.5 §5c.3 fix — mirror of the terrain-side fix above.
        setPropIfGraphics(state.weatherRegionsBtnEditPolygon,  'Enable', 'off');
        setPropIfGraphics(state.weatherRegionsNameField, 'Value', '');
        setPropIfGraphics(state.weatherRegionsConfigLabel, 'Text', '—');
        setPropIfGraphics(state.weatherRegionsPolygonStatusLabel, ...
            'Text', '—');
        return;
    end

    items = cell(1, n);
    data  = cell(1, n);
    for k = 1:n
        rec = state.weatherRegions(k);
        nm = char(rec.name);
        if isempty(nm); nm = sprintf('(unnamed %d)', k); end
        if k == state.activeWeatherRegionIdx
            items{k} = sprintf('%s  (active)', nm);
        else
            items{k} = nm;
        end
        data{k} = k;
    end
    state.weatherRegionsDD.Items     = items;
    state.weatherRegionsDD.ItemsData = data;
    if state.activeWeatherRegionIdx >= 1 && state.activeWeatherRegionIdx <= n
        state.weatherRegionsDD.Value = state.activeWeatherRegionIdx;
    else
        state.weatherRegionsDD.Value = 1;
    end

    setPropIfGraphics(state.weatherRegionsBtnDuplicate,    'Enable', 'on');
    setPropIfGraphics(state.weatherRegionsBtnDelete,       'Enable', 'on');
    setPropIfGraphics(state.weatherRegionsNameField,       'Enable', 'on');
    setPropIfGraphics(state.weatherRegionsBtnChangeConfig, 'Enable', 'on');
    % v3.5 §5c.3 — Edit Polygon button activates when there's an active
    % region. The lockdown helper overrides this during active edit.
    setPropIfGraphics(state.weatherRegionsBtnEditPolygon,  'Enable', 'on');

    if state.hasActiveWeatherRegion()
        rec = state.weatherRegions(state.activeWeatherRegionIdx);
        setPropIfGraphics(state.weatherRegionsNameField, 'Value', char(rec.name));
        cfg = char(rec.configPath);
        if isempty(cfg); cfg = '—'; end
        setPropIfGraphics(state.weatherRegionsConfigLabel, 'Text', cfg);
        setPropIfGraphics(state.weatherRegionsPolygonStatusLabel, ...
            'Text', polygonStatusText(rec));
    else
        setPropIfGraphics(state.weatherRegionsNameField, 'Value', '');
        setPropIfGraphics(state.weatherRegionsConfigLabel, 'Text', '—');
        setPropIfGraphics(state.weatherRegionsPolygonStatusLabel, ...
            'Text', '—');
    end
end


function txt = polygonStatusText(rec)
%polygonStatusText  Human-readable polygon vertex count + validity.
%                   Used by both Terrain and Weather Regions panels.
%                   Works for either region-record class because both
%                   expose .polygonXY and .isValidPolygon().
    nv = size(rec.polygonXY, 1);
    if nv == 0
        txt = 'empty (draw a polygon to define this region)';
        return;
    end
    if rec.isValidPolygon()
        txt = sprintf('%d vertices  ✓ valid', nv);
    else
        txt = sprintf('%d vertices  ⚠ needs ≥3', nv);
    end
end


%% ------------------------------------------------------------------------
%  5c.2 — SUB-MODE TOGGLE CALLBACKS
%% ------------------------------------------------------------------------

function onEnvSubModeFallback(src, state)
%onEnvSubModeFallback  Fallback state-button handler. Mutual exclusion
%                      with the Regions button — mirrors
%                      onModeTargetsPressed (top-level mode toggle).
    if ~src.Value
        src.Value = true;   % bounce off-click back on
        return;
    end
    if isgraphics(state.envSubModeRegionsBtn)
        state.envSubModeRegionsBtn.Value = false;
    end
    state.setEnvSubMode("fallback");
    applyEditMode(state);
    setStatus(state, 'Environment: editing fallback terrain + weather.');
end


function onEnvSubModeRegions(src, state)
%onEnvSubModeRegions  Regions state-button handler. Mirror of
%                     onEnvSubModeFallback.
    if ~src.Value
        src.Value = true;
        return;
    end
    if isgraphics(state.envSubModeFallbackBtn)
        state.envSubModeFallbackBtn.Value = false;
    end
    state.setEnvSubMode("regions");
    applyEditMode(state);
    setStatus(state, 'Environment: editing terrain + weather regions.');
end


%% ------------------------------------------------------------------------
%  5c.2 — TERRAIN REGIONS CALLBACKS
%% ------------------------------------------------------------------------

function onTerrainRegionDDChanged(src, state)
%onTerrainRegionDDChanged  Switch active terrain region. ItemsData are
%                          1-based indices, so src.Value is the new idx.
%                          View state — no undo (mirrors
%                          onTargetsDropdownChanged → setActiveIdx).
    newIdx = double(src.Value);
    if newIdx < 1 || newIdx > numel(state.terrainRegions)
        return;
    end
    state.setActiveTerrainRegionIdx(newIdx);
    refreshTerrainRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.terrainRegions(newIdx);
    setStatus(state, sprintf('Active terrain region: %s', rec.name));
end


function onTerrainRegionsAdd(state)
%onTerrainRegionsAdd  Append a new region (Option B inheritance — seeded
%                     from the fallback record by EditorState.
%                     addTerrainRegion). Auto-named region_<n+1>; the
%                     user can rename via the inline name field.
    try
        idx = state.addTerrainRegion();
    catch ME
        uialert(state.fig, ME.message, 'Add terrain region failed', 'Icon', 'error');
        return;
    end
    refreshTerrainRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.terrainRegions(idx);
    setStatus(state, sprintf('Added terrain region "%s" (config: %s).', ...
        rec.name, rec.configPath));
end


function onTerrainRegionsDuplicate(state)
%onTerrainRegionsDuplicate  Copy active region with "_copy" suffix and
%                            polygon offset +2 km east.
    if ~state.hasActiveTerrainRegion()
        setStatus(state, 'Nothing to duplicate — add a region first.');
        return;
    end
    try
        state.duplicateActiveTerrainRegion();
    catch ME
        uialert(state.fig, ME.message, 'Duplicate failed', 'Icon', 'error');
        return;
    end
    refreshTerrainRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.terrainRegions(state.activeTerrainRegionIdx);
    setStatus(state, sprintf('Duplicated → %s', rec.name));
end


function onTerrainRegionsDelete(state)
%onTerrainRegionsDelete  Confirm + delete active terrain region.
%                        Always confirms (regions don't have a "clearly
%                        empty so skip prompt" heuristic the way targets
%                        do — a region with no polygon vertices is
%                        still meaningful work).
    if ~state.hasActiveTerrainRegion()
        setStatus(state, 'Nothing to delete.');
        return;
    end
    rec = state.terrainRegions(state.activeTerrainRegionIdx);
    sel = uiconfirm(state.fig, ...
        sprintf('Delete terrain region "%s"? You can Ctrl+Z to bring it back.', rec.name), ...
        'Delete terrain region', ...
        'Options', {'Delete', 'Cancel'}, ...
        'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
        'Icon', 'warning');
    if ~strcmp(sel, 'Delete'); return; end
    deletedName = rec.name;
    state.deleteActiveTerrainRegion();
    refreshTerrainRegionsPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Deleted terrain region "%s".', deletedName));
end


function onTerrainRegionsNameFieldChanged(src, state)
%onTerrainRegionsNameFieldChanged  Inline rename via ValueChangedFcn.
%                                  renameActiveTerrainRegion validates
%                                  non-empty + unique; failures snap
%                                  the field back to the old name and
%                                  surface via uialert.
    if ~state.hasActiveTerrainRegion()
        src.Value = '';
        return;
    end
    newName = strtrim(src.Value);
    cur = char(state.terrainRegions(state.activeTerrainRegionIdx).name);
    if isempty(newName) || strcmp(newName, cur)
        src.Value = cur;   % no-op or empty → snap back
        return;
    end
    try
        state.renameActiveTerrainRegion(newName);
    catch ME
        uialert(state.fig, ME.message, 'Rename failed', 'Icon', 'warning');
        src.Value = cur;
        return;
    end
    refreshTerrainRegionsPanel(state);
    setStatus(state, sprintf('Renamed terrain region → %s', newName));
end


function onTerrainRegionsEditPolygon(state)
%onTerrainRegionsEditPolygon  v3.5 §5c.3 — enter polygon-edit mode
%                              for the active terrain region. Seeds the
%                              draft with the existing polygonXY (Q1 =
%                              edit in place). Locks the rest of the UI
%                              via applyPolygonEditLockdown so the user
%                              can't switch regions / sub-modes / modes
%                              mid-edit (Q2 = lock).
    if ~state.hasActiveTerrainRegion()
        setStatus(state, 'No active terrain region to edit.');
        return;
    end
    if state.polygonEditActive
        setStatus(state, 'Already in polygon-edit mode — Enter to commit, Esc to abort.');
        return;
    end
    try
        state.beginPolygonEdit("terrain");
    catch ME
        uialert(state.fig, ME.message, 'Edit polygon failed', 'Icon', 'error');
        return;
    end
    applyPolygonEditLockdown(state);
    trackbench.editor.drawMap(state);
    nv = size(state.polygonEditDraft, 1);
    if nv == 0
        setStatus(state, ['Polygon edit — click on the map to add vertices. ' ...
                          'Enter or double-click to commit, Esc to cancel.']);
    else
        setStatus(state, sprintf(['Polygon edit — editing existing %d-vertex ' ...
            'polygon. Click to add more, Enter to commit, Esc to cancel.'], nv));
    end
end


function onTerrainRegionsChangeConfig(state)
%onTerrainRegionsChangeConfig  File picker → loadTerrainRegionConfig.
%                              Default folder: config/terrain/. Path is
%                              stripped to a library-style relative
%                              "<typeDir>/<n>" before being passed to
%                              the loader (mirrors onLoadTerrain).
%
%  Polygon is preserved across the swap (loadTerrainRegionConfig
%  rebuilds the inner record but keeps polygonXY).
    if ~state.hasActiveTerrainRegion()
        setStatus(state, 'No active terrain region to change config for.');
        return;
    end
    startDir = defaultTerrainPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON terrain files'}, ...
        'Change terrain region config', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    relPath = extractConfigRelPath(full, state, "terrain");
    try
        state.loadTerrainRegionConfig(relPath);
    catch ME
        setStatus(state, sprintf('Change config failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Change config failed');
        return;
    end
    refreshTerrainRegionsPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Loaded "%s" into active terrain region.', relPath));
end


%% ------------------------------------------------------------------------
%  5c.2 — WEATHER REGIONS CALLBACKS
%% ------------------------------------------------------------------------
%  Mirrors of the terrain-region callbacks above. Same shape, different
%  collection. Kept as separate functions (not parametric) for readability
%  — matches how the rest of the file pairs Targets/Sensors callbacks.

function onWeatherRegionDDChanged(src, state)
    newIdx = double(src.Value);
    if newIdx < 1 || newIdx > numel(state.weatherRegions)
        return;
    end
    state.setActiveWeatherRegionIdx(newIdx);
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.weatherRegions(newIdx);
    setStatus(state, sprintf('Active weather region: %s', rec.name));
end


function onWeatherRegionsAdd(state)
    try
        idx = state.addWeatherRegion();
    catch ME
        uialert(state.fig, ME.message, 'Add weather region failed', 'Icon', 'error');
        return;
    end
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.weatherRegions(idx);
    setStatus(state, sprintf('Added weather region "%s" (config: %s).', ...
        rec.name, rec.configPath));
end


function onWeatherRegionsDuplicate(state)
    if ~state.hasActiveWeatherRegion()
        setStatus(state, 'Nothing to duplicate — add a region first.');
        return;
    end
    try
        state.duplicateActiveWeatherRegion();
    catch ME
        uialert(state.fig, ME.message, 'Duplicate failed', 'Icon', 'error');
        return;
    end
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rec = state.weatherRegions(state.activeWeatherRegionIdx);
    setStatus(state, sprintf('Duplicated → %s', rec.name));
end


function onWeatherRegionsDelete(state)
    if ~state.hasActiveWeatherRegion()
        setStatus(state, 'Nothing to delete.');
        return;
    end
    rec = state.weatherRegions(state.activeWeatherRegionIdx);
    sel = uiconfirm(state.fig, ...
        sprintf('Delete weather region "%s"? You can Ctrl+Z to bring it back.', rec.name), ...
        'Delete weather region', ...
        'Options', {'Delete', 'Cancel'}, ...
        'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
        'Icon', 'warning');
    if ~strcmp(sel, 'Delete'); return; end
    deletedName = rec.name;
    state.deleteActiveWeatherRegion();
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Deleted weather region "%s".', deletedName));
end


function onWeatherRegionsNameFieldChanged(src, state)
    if ~state.hasActiveWeatherRegion()
        src.Value = '';
        return;
    end
    newName = strtrim(src.Value);
    cur = char(state.weatherRegions(state.activeWeatherRegionIdx).name);
    if isempty(newName) || strcmp(newName, cur)
        src.Value = cur;
        return;
    end
    try
        state.renameActiveWeatherRegion(newName);
    catch ME
        uialert(state.fig, ME.message, 'Rename failed', 'Icon', 'warning');
        src.Value = cur;
        return;
    end
    refreshWeatherRegionsPanel(state);
    setStatus(state, sprintf('Renamed weather region → %s', newName));
end


function onWeatherRegionsEditPolygon(state)
%onWeatherRegionsEditPolygon  v3.5 §5c.3 — mirror of
%                              onTerrainRegionsEditPolygon for the
%                              weather collection.
    if ~state.hasActiveWeatherRegion()
        setStatus(state, 'No active weather region to edit.');
        return;
    end
    if state.polygonEditActive
        setStatus(state, 'Already in polygon-edit mode — Enter to commit, Esc to abort.');
        return;
    end
    try
        state.beginPolygonEdit("weather");
    catch ME
        uialert(state.fig, ME.message, 'Edit polygon failed', 'Icon', 'error');
        return;
    end
    applyPolygonEditLockdown(state);
    trackbench.editor.drawMap(state);
    nv = size(state.polygonEditDraft, 1);
    if nv == 0
        setStatus(state, ['Polygon edit — click on the map to add vertices. ' ...
                          'Enter or double-click to commit, Esc to cancel.']);
    else
        setStatus(state, sprintf(['Polygon edit — editing existing %d-vertex ' ...
            'polygon. Click to add more, Enter to commit, Esc to cancel.'], nv));
    end
end


function onWeatherRegionsChangeConfig(state)
    if ~state.hasActiveWeatherRegion()
        setStatus(state, 'No active weather region to change config for.');
        return;
    end
    startDir = defaultWeatherPickerDir(state);
    [file, path] = uigetfile({'*.json','JSON weather files'}, ...
        'Change weather region config', startDir);
    if isequal(file, 0); return; end
    full = fullfile(path, file);
    relPath = extractConfigRelPath(full, state, "weather");
    try
        state.loadWeatherRegionConfig(relPath);
    catch ME
        setStatus(state, sprintf('Change config failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Change config failed');
        return;
    end
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Loaded "%s" into active weather region.', relPath));
end


%% ------------------------------------------------------------------------
%  5c.3 — POLYGON-EDIT COMMIT HELPER + UI LOCKDOWN
%% ------------------------------------------------------------------------
%  tryCommitPolygonEdit       — wraps state.commitPolygonEdit with the
%                                <3-vertex refusal status message and
%                                the panel-refresh / lockdown-release
%                                cascade on success.
%  applyPolygonEditLockdown   — toggles Enable on every widget that
%                                could let the user navigate away from
%                                the active edit. Called when entering
%                                edit mode (lock down) AND when
%                                committing/aborting (release).

function tryCommitPolygonEdit(state)
%tryCommitPolygonEdit  v3.5 §5c.3 — commit the in-progress polygon
%                       edit. <3 vertices → refuse with status warning,
%                       edit mode stays active. ≥3 vertices → commit,
%                       refresh the relevant Regions panel, release
%                       the UI lockdown.
    nv = size(state.polygonEditDraft, 1);
    if nv < 3
        setStatus(state, sprintf( ...
            ['Need at least 3 vertices (have %d) — keep clicking, ' ...
             'or Esc to cancel.'], nv));
        return;
    end
    target = char(state.polygonEditTarget);   % capture before commit clears
    ok = state.commitPolygonEdit();
    if ~ok
        % Should not happen — the <3 guard above should have caught it.
        % Defensive status message if commit refused for some other
        % reason (e.g., stored polygon validation downstream).
        setStatus(state, 'Polygon commit failed unexpectedly.');
        return;
    end
    applyPolygonEditLockdown(state);   % releases the lock
    switch target
        case 'terrain'
            refreshTerrainRegionsPanel(state);
        case 'weather'
            refreshWeatherRegionsPanel(state);
    end
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf('Polygon committed (%d vertices).', nv));
end


function applyPolygonEditLockdown(state)
%applyPolygonEditLockdown  v3.5 §5c.3 — lock or unlock the UI based on
%                          state.polygonEditActive.
%
%  When ACTIVE: every UI control that could navigate away from the
%  current edit gets disabled — mode toggle (Targets/Sensors/
%  Environment), sub-mode toggle (Fallback/Regions), both Regions
%  dropdowns, Add/Dup/Del/NameField/ChangeConfig in BOTH regions
%  panels, Edit Polygon on BOTH regions (no value in keeping the
%  active one enabled — Enter/Esc handle commit/abort), File panel
%  buttons (Open Scenario / Export Scenario), Undo/Redo buttons.
%
%  When INACTIVE: each gated widget is restored to whatever the normal
%  enable rule would set it to. Cleanest way to do this is to just
%  re-run applyEditMode — it knows the right rules for the current
%  editMode/envSubMode/active-region state. The recursive call is
%  guarded by polygonEditActive==false here, so we don't re-enter.
%
%  Idempotent on repeat calls in the same direction — safe to call
%  defensively from refresh paths.
    if state.polygonEditActive
        % Lock down. Top-level mode toggle.
        setEnableIfGraphics(state.modeTargetsBtn,     false);
        setEnableIfGraphics(state.modeSensorsBtn,     false);
        setEnableIfGraphics(state.modeEnvironmentBtn, false);
        % Sub-mode toggle.
        setEnableIfGraphics(state.envSubModeFallbackBtn, false);
        setEnableIfGraphics(state.envSubModeRegionsBtn,  false);
        % Both regions panels' interactive controls.
        regionWidgets = { ...
            state.terrainRegionsDD, ...
            state.terrainRegionsBtnAdd, ...
            state.terrainRegionsBtnDuplicate, ...
            state.terrainRegionsBtnDelete, ...
            state.terrainRegionsNameField, ...
            state.terrainRegionsBtnEditPolygon, ...
            state.terrainRegionsBtnChangeConfig, ...
            state.weatherRegionsDD, ...
            state.weatherRegionsBtnAdd, ...
            state.weatherRegionsBtnDuplicate, ...
            state.weatherRegionsBtnDelete, ...
            state.weatherRegionsNameField, ...
            state.weatherRegionsBtnEditPolygon, ...
            state.weatherRegionsBtnChangeConfig};
        for k = 1:numel(regionWidgets)
            setEnableIfGraphics(regionWidgets{k}, false);
        end
        % Walk the File panel children (Open Scenario, Export Scenario).
        % These are buttons inside a uipanel we don't have a handle for,
        % so search via fig descendants. Cheap — only ~8 buttons total.
        if isgraphics(state.fig)
            allBtns = findall(state.fig, 'Type', 'uibutton');
            for k = 1:numel(allBtns)
                txt = '';
                try; txt = char(allBtns(k).Text); catch; end %#ok<NOSEMI>
                if any(strcmp(txt, {'Open Scenario…', 'Export Scenario', ...
                                    'Undo (Ctrl+Z)', 'Redo (Ctrl+Y)'}))
                    allBtns(k).Enable = 'off';
                end
            end
        end
    else
        % Release. v3.5 §5c.3 fix — the lock branch disables widgets
        % across THREE distinct enable-domains: (a) mode + sub-mode
        % toggles, which applyEditMode never re-enables (it only
        % handles panel visibility and the region widgets); (b) the
        % regions widgets, which refreshTerrain/WeatherRegionsPanel
        % handles correctly; (c) File panel + Undo/Redo buttons,
        % which nothing else touches. Pre-fix this branch only called
        % applyEditMode, leaving (a) and (c) stuck at disabled — user
        % could still use Ctrl+Z/Y keyboard shortcuts but couldn't
        % switch modes or click File-panel buttons. The trap.
        %
        % Fix: explicitly re-enable everything the lock branch
        % disabled, then delegate to applyEditMode for the fine-
        % grained mode/state rules (which may legitimately disable
        % some of them again, e.g. EditPolygon when n==0 regions).
        setEnableIfGraphics(state.modeTargetsBtn,     true);
        setEnableIfGraphics(state.modeSensorsBtn,     true);
        setEnableIfGraphics(state.modeEnvironmentBtn, true);
        setEnableIfGraphics(state.envSubModeFallbackBtn, true);
        setEnableIfGraphics(state.envSubModeRegionsBtn,  true);
        regionWidgets = { ...
            state.terrainRegionsDD, ...
            state.terrainRegionsBtnAdd, ...
            state.terrainRegionsBtnDuplicate, ...
            state.terrainRegionsBtnDelete, ...
            state.terrainRegionsNameField, ...
            state.terrainRegionsBtnEditPolygon, ...
            state.terrainRegionsBtnChangeConfig, ...
            state.weatherRegionsDD, ...
            state.weatherRegionsBtnAdd, ...
            state.weatherRegionsBtnDuplicate, ...
            state.weatherRegionsBtnDelete, ...
            state.weatherRegionsNameField, ...
            state.weatherRegionsBtnEditPolygon, ...
            state.weatherRegionsBtnChangeConfig};
        for k = 1:numel(regionWidgets)
            setEnableIfGraphics(regionWidgets{k}, true);
        end
        if isgraphics(state.fig)
            allBtns = findall(state.fig, 'Type', 'uibutton');
            for k = 1:numel(allBtns)
                txt = '';
                try; txt = char(allBtns(k).Text); catch; end %#ok<NOSEMI>
                if any(strcmp(txt, {'Open Scenario…', 'Export Scenario', ...
                                    'Undo (Ctrl+Z)', 'Redo (Ctrl+Y)'}))
                    allBtns(k).Enable = 'on';
                end
            end
        end
        % Now delegate to applyEditMode for fine-grained rules.
        applyEditMode(state);
    end
end


%% ========================================================================
%  v3.5 §5c.6 — Region vertex drag / edge-insert helpers
%% ========================================================================
%
%  Four local helpers split out of onAxesClick/onMouseMove/onMouseUp to
%  keep those handlers readable. All four take state (the EditorState
%  handle) as the first argument and mutate it in place; none return
%  values. Status messages are posted via setStatus.
%
%  beginRegionVertexDrag        — mousedown-on-vertex sequence: auto-
%                                 promote, pushUndo, set drag/selection
%                                 state, refresh region panels, draw.
%  insertRegionVertexFromClick  — shift+mousedown-on-edge sequence:
%                                 insertRegionVertex (which auto-promotes
%                                 + pushUndo internally), select the new
%                                 vertex, refresh, draw.
%  isRegionReadOnly             — guard for both above: bail if the
%                                 target region's inner record is
%                                 readOnly (status nag posted by caller).
%  getRegionName                — short helper for status messages.
%                                 Returns "?" on out-of-range to keep
%                                 sprintf safe in transient drag states.

function beginRegionVertexDrag(state, kind, regionIdx, vertexIdx)
    % Auto-promote the picked region to active so:
    %   * drawRegionPolygons highlights it (vertex circles + thicker outline)
    %   * the regions sub-panel dropdown selection follows the drag
    %   * subsequent moveRegionVertex(commit=false) calls in onMouseMove
    %     just write polygon updates without re-pushing undo
    %
    % Auto-promote happens BEFORE pushUndo so the undo snapshot captures
    % the pre-drag active-idx — a single Ctrl+Z reverts both polygon and
    % active-idx together.
    if kind == "terrain" && state.activeTerrainRegionIdx ~= regionIdx
        state.activeTerrainRegionIdx = regionIdx;
    elseif kind == "weather" && state.activeWeatherRegionIdx ~= regionIdx
        state.activeWeatherRegionIdx = regionIdx;
    end
    state.pushUndo();
    state.vertexDragActive    = true;
    state.vertexDragKind      = kind;
    state.vertexDragRegionIdx = regionIdx;
    state.vertexDragVertexIdx = vertexIdx;
    % Selection tracks the dragged vertex so Delete-key targeting works
    % after drag-end without an extra click.
    state.selectedRegionKind = kind;
    state.selectedRegionIdx  = regionIdx;
    state.selectedVertexIdx  = vertexIdx;
    refreshTerrainRegionsPanel(state);
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rname = getRegionName(state, kind, regionIdx);
    setStatus(state, sprintf( ...
        'Selected %s region "%s" vertex #%d (drag to move, Del to remove).', ...
        kind, rname, vertexIdx));
end

function insertRegionVertexFromClick(state, kind, regionIdx, edgeIdx, projXY)
    % insertRegionVertex auto-promotes + pushes undo internally, so we
    % don't repeat that work here. The new vertex sits at row (edgeIdx+1)
    % — selection follows it so Delete can immediately revert if the user
    % regrets the insert.
    state.insertRegionVertex(kind, regionIdx, edgeIdx, projXY(1), projXY(2));
    newVtx = edgeIdx + 1;
    state.selectedRegionKind = kind;
    state.selectedRegionIdx  = regionIdx;
    state.selectedVertexIdx  = newVtx;
    refreshTerrainRegionsPanel(state);
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    rname = getRegionName(state, kind, regionIdx);
    setStatus(state, sprintf( ...
        'Inserted vertex #%d into %s region "%s" at (%.0f, %.0f).', ...
        newVtx, kind, rname, projXY(1), projXY(2)));
end

function tf = isRegionReadOnly(state, kind, regionIdx)
    tf = false;
    if kind == "terrain"
        if regionIdx >= 1 && regionIdx <= numel(state.terrainRegions)
            tf = state.terrainRegions(regionIdx).readOnly;
        end
    elseif kind == "weather"
        if regionIdx >= 1 && regionIdx <= numel(state.weatherRegions)
            tf = state.weatherRegions(regionIdx).readOnly;
        end
    end
end

function name = getRegionName(state, kind, regionIdx)
    name = "?";
    if kind == "terrain"
        if regionIdx >= 1 && regionIdx <= numel(state.terrainRegions)
            name = state.terrainRegions(regionIdx).name;
        end
    elseif kind == "weather"
        if regionIdx >= 1 && regionIdx <= numel(state.weatherRegions)
            name = state.weatherRegions(regionIdx).name;
        end
    end
end

function handleRegionVertexDelete(state)
%handleRegionVertexDelete  v3.5 §5c.6 — Delete-key dispatch for region
%                          vertex deletion. Reads current selection,
%                          calls deleteRegionVertex (which auto-promotes
%                          + pushes undo + refuses if polygon would
%                          drop below 3 vertices), then refreshes the
%                          UI. Status nag on refuse.
    kind = state.selectedRegionKind;
    rIdx = state.selectedRegionIdx;
    vIdx = state.selectedVertexIdx;
    if vIdx < 1 || rIdx < 1 || kind == ""
        return;  % defensive: caller already checked, but be safe
    end
    if isRegionReadOnly(state, kind, rIdx)
        setStatus(state, sprintf( ...
            '%s region is read-only — cannot delete vertices.', kind));
        return;
    end
    rname = getRegionName(state, kind, rIdx);
    ok = state.deleteRegionVertex(kind, rIdx, vIdx);
    if ~ok
        setStatus(state, sprintf( ...
            'Refused: %s region "%s" already has the minimum 3 vertices.', ...
            kind, rname));
        return;
    end
    % Successful delete — selection is now stale (vertex it pointed to
    % no longer exists). Clear it so the next Delete-key press doesn't
    % chain into another deletion of whatever vertex got shifted into
    % the same index slot.
    state.selectedRegionKind = "";
    state.selectedRegionIdx  = 0;
    state.selectedVertexIdx  = 0;
    refreshTerrainRegionsPanel(state);
    refreshWeatherRegionsPanel(state);
    trackbench.editor.drawMap(state);
    setStatus(state, sprintf( ...
        'Deleted vertex #%d from %s region "%s".', vIdx, kind, rname));
end

function rotate3DView(state, dAz, dEl, shift)
%rotate3DView  v3.5 fix — keyboard camera orbit for 3D view. dAz / dEl
%              are degrees of azimuth / elevation change. Shift
%              multiplier triples the step for fast traversal.
%              Elevation clamped to [-90, 90] (MATLAB's valid range).
%              Setting ax.View directly triggers MATLAB's renderer to
%              redraw without us calling drawMap — cheap, and avoids
%              the cla-and-restore round-trip drawMap3D does on every
%              redraw.
    ax = state.ax;
    if ~isgraphics(ax); return; end
    if shift
        dAz = dAz * 3;
        dEl = dEl * 3;
    end
    v = ax.View;
    newAz = v(1) + dAz;
    newEl = max(-90, min(90, v(2) + dEl));
    ax.View = [newAz, newEl];
    setStatus(state, sprintf( ...
        '3D view: az=%.0f° el=%.0f°  (←/→ az, ↑/↓ el, Shift=fast, R=reset)', ...
        newAz, newEl));
end
