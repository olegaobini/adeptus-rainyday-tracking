function buildUI(state)
%buildUI  Create the uifigure + controls for the interactive path editor.
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
    fig = uifigure('Name', 'Rainy Day — Path Editor (M3)', ...
                   'Position', [100 80 1200 860], ...
                   'CloseRequestFcn', @(src, ~) onClose(src, state), ...
                   'KeyPressFcn',     @(src, evt) onKeyPress(src, evt, state), ...
                   'WindowButtonMotionFcn', @(src, ~) onMouseMove(src, state), ...
                   'WindowButtonUpFcn',     @(src, ~) onMouseUp(src, state), ...
                   'WindowScrollWheelFcn',  @(src, evt) onScrollWheel(src, evt, state));
    state.fig = fig;

    % ── Top-level grid: map | sidebar, and a status bar at the bottom
    outer = uigridlayout(fig, [2 2]);
    outer.RowHeight   = {'1x', 28};
    outer.ColumnWidth = {'1x', 300};
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
    side = uipanel(outer, 'Title', 'Editor', 'FontWeight', 'bold', ...
                   'Scrollable', 'on');
    side.Layout.Row = 1;
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
    % Row heights (M3):
    %   Scenario panel grew from 7→11 rows (M3.1 checkbox + M3.2 grid
    %   dropdown + M3.3 view-mode button + M3.4 preview button), so
    %   bumped from 240→320. Selected panel unchanged. If this window
    %   feels cramped after M3.5 (radar X/Y add 2 more scenario rows),
    %   grow Scenario to ~370 and/or enlarge the figure.
    inner = uigridlayout(sg, [5 1]);
    % Help panel (row 5) grew from 85 → 115 when the shortcuts pass
    % added two extra lines to the cheat sheet (V/arrows/PgUp/Dn/wheel).
    inner.RowHeight   = {320, 240, 95, 60, 115};
    inner.ColumnWidth = {'1x'};
    inner.RowSpacing  = 4;
    inner.Scrollable  = 'on';

    % ── Scenario sub-panel ──────────────────────────────────────────
    buildScenarioPanel(inner, state);

    % ── Selection sub-panel ─────────────────────────────────────────
    buildSelectionPanel(inner, state);

    % ── File sub-panel ──────────────────────────────────────────────
    buildFilePanel(inner, state);

    % ── Undo/redo sub-panel ─────────────────────────────────────────
    buildUndoPanel(inner, state);

    % ── Help sub-panel ──────────────────────────────────────────────
    % Help sub-panel
    buildHelpPanel(inner);

    % ── Status bar ──────────────────────────────────────────────────
    state.statusLabel = uilabel(outer, ...
        'Text', 'Ready. Click to add · drag to move · Shift+click a segment to insert · Del to remove.', ...
        'FontColor', [0.25 0.25 0.25]);
    state.statusLabel.Layout.Row = 2;
    state.statusLabel.Layout.Column = [1 2];

    % ── Initial draw ────────────────────────────────────────────────
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
end


%% ========================================================================
%  SIDEBAR BUILDERS
%% ========================================================================

function buildScenarioPanel(parent, state)
    pnl = uipanel(parent, 'Title', 'Scenario');
    % 11 rows: the five scenario-field pairs, waypoint count, M3.1
    % colormap checkbox, M3.2 grid dropdown, M3.3 2D/3D toggle, the
    % Apply-altitude button, and the M3.4 preview button.
    % Height: 11*24 + 10*4 + 2*6 = 316; the inner grid row for this
    % panel in buildUI is 320.
    nRows = 11;
    g = uigridlayout(pnl, [nRows 2]);
    g.RowHeight   = repmat({24}, 1, nRows);
    g.ColumnWidth = {120, '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;

    uilabel(g, 'Text', 'Target name');
    state.nameField = uieditfield(g, 'text', 'Value', char(state.targetName), ...
        'ValueChangedFcn', @(src, ~) onNameChanged(src, state));

    uilabel(g, 'Text', 'Default speed (km/h)');
    uieditfield(g, 'numeric', 'Value', state.defaultSpeedKmh, ...
        'Limits', [1 5000], ...
        'ValueChangedFcn', @(src, ~) onSpeedChanged(src, state));

    uilabel(g, 'Text', 'Default altitude (m)');
    uieditfield(g, 'numeric', 'Value', state.defaultAltitudeM, ...
        'Limits', [0 30000], ...
        'ValueChangedFcn', @(src, ~) onAltitudeChanged(src, state));

    uilabel(g, 'Text', 'RCS (dBsm)');
    uieditfield(g, 'numeric', 'Value', state.rcsDbsm, ...
        'Limits', [-50 50], ...
        'ValueChangedFcn', @(src, ~) onRcsDbsmChanged(src, state));

    uilabel(g, 'Text', 'RCS profile');
    uidropdown(g, ...
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

    % M3.3 — 2D/3D view toggle. Uses a state-button so we can see at a
    % glance which mode is active. 2D is the editing mode; 3D is view-
    % only and disables click-to-add + drag.
    state.viewModeBtn = uibutton(g, 'state', ...
        'Text', viewButtonText(state.viewMode), ...
        'Value', strcmp(char(state.viewMode), '3d'), ...
        'ValueChangedFcn', @(src, ~) onViewModeChanged(src, state));
    state.viewModeBtn.Layout.Row    = 9;
    state.viewModeBtn.Layout.Column = [1 2];

    % "Apply default altitude to all" — spans both columns.
    applyAltBtn = uibutton(g, 'push', ...
        'Text', 'Apply default altitude to all waypoints', ...
        'ButtonPushedFcn', @(~, ~) onApplyDefaultAltitude(state));
    applyAltBtn.Layout.Column = [1 2];
    applyAltBtn.Layout.Row    = 10;

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
    state.previewBtn.Layout.Row    = 11;
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
    pnl = uipanel(parent, 'Title', 'File');
    g = uigridlayout(pnl, [2 2]);
    g.RowHeight   = {28, 28};
    g.ColumnWidth = {'1x', '1x'};
    g.Padding     = [6 6 6 6];
    g.RowSpacing  = 4;
    g.ColumnSpacing = 4;

    loadBtn = uibutton(g, 'push', 'Text', 'Load JSON…', ...
        'ButtonPushedFcn', @(~, ~) onLoad(state));
    loadBtn.Layout.Row    = 1;
    loadBtn.Layout.Column = 1;

    exportBtn = uibutton(g, 'push', 'Text', 'Export JSON', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.25 0.55 0.85], 'FontColor', 'white', ...
        'ButtonPushedFcn', @(~, ~) onExport(state));
    exportBtn.Layout.Row    = 1;
    exportBtn.Layout.Column = 2;

    clearBtn = uibutton(g, 'push', 'Text', 'Clear all waypoints', ...
        'ButtonPushedFcn', @(~, ~) onClear(state));
    clearBtn.Layout.Row    = 2;
    clearBtn.Layout.Column = [1 2];
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
         'Wheel: zoom  |  Middle-drag: pan (2D)'], ...
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
    if isMiddleClick && ~in3D
        state.panActive     = true;
        state.panStartFigPt = state.fig.CurrentPoint;
        state.panStartXLim  = state.ax.XLim;
        state.panStartYLim  = state.ax.YLim;
        setStatus(state, 'Panning — release middle button to stop.');
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

    % ── 1. Drag branch (2D only) ─────────────────────────────────────
    if state.dragActive && state.selectedIndex >= 1 && state.viewMode ~= "3d"
        state.moveSelectedTo(x, y, false);   % commit=false: no undo per frame
        trackbench.editor.drawMap(state);
        refreshSelectionPanel(state);
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
    if state.dragActive
        state.dragActive = false;
        state.dragStartWP = [];
        setStatus(state, sprintf('Moved waypoint #%d.', state.selectedIndex));
    end
    if state.panActive
        state.panActive     = false;
        state.panStartFigPt = [];
        state.panStartXLim  = [];
        state.panStartYLim  = [];
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

    % Universal (no focus guard) — safe inside or outside text fields.
    switch evt.Key
        case {'delete', 'backspace'}
            onDeleteSelected(state);
            return;
        case 'escape'
            state.selectedIndex = 0;
            trackbench.editor.drawMap(state);
            refreshSelectionPanel(state);
            setStatus(state, 'Selection cleared.');
            return;
        case 'z'
            if ctrl; onUndo(state); end
            return;
        case 'y'
            if ctrl; onRedo(state); end
            return;
    end

    % Shortcut keys that conflict with text editing — bail if a field has focus.
    if editingTextField(state.fig)
        return;
    end

    % Nudge step sizes (spec: 100 m fine, 1 km coarse).
    stepM = 100;
    if shift; stepM = 1000; end

    switch evt.Key
        case 'v'
            if ctrl || shift; return; end  % leave Ctrl+V / Shift+V for future use
            toggleViewModeViaKey(state);
        case 'leftarrow'
            nudgeSelectedXY(state, -stepM, 0, stepM);
        case 'rightarrow'
            nudgeSelectedXY(state,  stepM, 0, stepM);
        case 'uparrow'
            nudgeSelectedXY(state, 0,  stepM, stepM);
        case 'downarrow'
            nudgeSelectedXY(state, 0, -stepM, stepM);
        case 'pageup'
            bumpSelectedAltitude(state,  stepM);
        case 'pagedown'
            bumpSelectedAltitude(state, -stepM);
    end
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
        % 3D: narrower camera-view-angle = zoomed in.
        va = ax.CameraViewAngle;
        factor = 1.2 ^ scrollCount;           % +tick → wider (zoom out)
        newVA = max(1, min(120, va * factor));
        ax.CameraViewAngle = newVA;
        setStatus(state, sprintf('3D zoom: %.1f° view angle.', newVA));
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
    if state.count() < 2
        setStatus(state, ...
            sprintf('Need at least 2 waypoints to export (have %d).', state.count()));
        uialert(state.fig, ...
            sprintf('Need at least 2 waypoints to export. You have %d.', state.count()), ...
            'Export blocked');
        return;
    end
    try
        outPath = trackbench.editor.exportToJSON(state);
    catch ME
        setStatus(state, sprintf('Export failed: %s', ME.message));
        uialert(state.fig, ME.message, 'Export failed');
        return;
    end
    setStatus(state, sprintf('Exported %d waypoints → %s', ...
        state.count(), outPath));
    uialert(state.fig, ...
        sprintf(['Wrote waypoint file:\n\n%s\n\n' ...
                 'Reference it in a run file as:\n' ...
                 '  "targets": "waypoints/%s"'], ...
                 outPath, state.targetName), ...
        'Export complete', 'Icon', 'success');
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
    % Sync sidebar scenario fields that were updated by the load.
    if isgraphics(state.nameField)
        state.nameField.Value = char(state.targetName);
    end
    trackbench.editor.drawMap(state);
    updateWaypointCount(state);
    refreshSelectionPanel(state);
    setStatus(state, sprintf('Loaded %d waypoints from %s', state.count(), full));
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
        trackbench.editor.drawMap(state);
        updateWaypointCount(state);
        refreshSelectionPanel(state);
        setStatus(state, 'Undid last change.');
    else
        setStatus(state, 'Nothing to undo.');
    end
end


function onRedo(state)
    if state.redo()
        trackbench.editor.drawMap(state);
        updateWaypointCount(state);
        refreshSelectionPanel(state);
        setStatus(state, 'Redid change.');
    else
        setStatus(state, 'Nothing to redo.');
    end
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
    newName = strtrim(string(src.Value));
    if newName == ""
        src.Value = char(state.targetName);
        return;
    end
    newName = regexprep(newName, '[^A-Za-z0-9_\-]', '_');
    src.Value = char(newName);
    state.targetName = newName;
    trackbench.editor.drawMap(state);
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
    wp  = state.waypoints(:, 1:2);
    ts  = state.waypoints(:, 4);
    rxy = [state.radarEastM, state.radarNorthM];
    state.previewFig = trackbench.editor.previewWindow( ...
        wp, ts, rxy, state.fig);
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
    % Force autofit on the next drawMap3D so a view-mode switch always
    % frames the current scene.
    state.has3DViewState = false;
    trackbench.editor.drawMap(state);
    if state.viewMode == "3d"
        setStatus(state, ...
            '3D view: pan/zoom via the axes toolbar. Switch to 2D to edit.');
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
%  HELPERS
%% ========================================================================

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
    if state.isDirty && state.count() > 0
        sel = uiconfirm(fig, ...
            sprintf('%d waypoint(s) not yet exported. Close anyway?', state.count()), ...
            'Unsaved waypoints', ...
            'Options', {'Close without exporting', 'Cancel'}, ...
            'DefaultOption', 2, 'CancelOption', 2, 'Icon', 'warning');
        if sel == "Cancel"
            return;
        end
    end
    delete(fig);
end
