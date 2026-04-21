function drawMap(state)
%drawMap  Re-render the editor axes from the current EditorState.
%
%  Called after every state mutation (add/move/remove/select/undo) and
%  after every view-option change (colormap toggle, grid spacing, 2D/3D).
%  Dispatches to drawMap2D or drawMap3D based on state.viewMode.
%
%  2D mode renders:
%    - Top-down East/North axes, straight-line path, numbered markers
%    - Radar site marker at (radarEastM, radarNorthM)  [M3.5 — editable]
%    - Selection halo (M2)
%    - Altitude colormap + colorbar when state.colorByAltitude == true (M3.1)
%    - Minor grid at state.gridSpacingKm (M3.2)
%    - Auto-rescaling scale bar in the lower-left corner (M3.2)
%
%  3D mode renders (M3.3):
%    - Perspective view with positive-up altitude on Z
%    - Vertical stems from ground (z=0) to each waypoint
%    - 3D path line through the waypoints
%    - Same selection highlight at the waypoint's altitude
%    - Colorbar when colorByAltitude is on
%    - Scale bar skipped (doesn't map cleanly to an oblique camera)
%    - Rotate/pan/zoom enabled via the axes toolbar + interactions
%
%  PICKABLEPARTS RULE
%    Every plot/plot3/text/scatter/scatter3 call sets 'PickableParts','none'
%    so children do not intercept axes ButtonDownFcn clicks. See the M2
%    gotcha write-up in memory/feedback_matlab_uifigure_gotchas.md.
%
%  TEXT INTERPRETER RULE
%    All axis labels use 'Interpreter','none' because target names contain
%    underscores that TeX would render as subscripts.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.buildUI,
%            trackbench.editor.computeScaleBar

    arguments
        state (1,1) trackbench.editor.EditorState
    end

    ax = state.ax;
    if ~isgraphics(ax); return; end

    if state.viewMode == "3d"
        drawMap3D(state, ax);
    else
        drawMap2D(state, ax);
    end
end


%% ========================================================================
%  2D RENDERING (M1–M3.2)
%% ========================================================================

function drawMap2D(state, ax)
    % Capture current 2D axis limits so we don't jump on every redraw
    % once the user has zoomed/panned. On the very first draw (limits
    % still at MATLAB's default [0 1]), we fall back to autoFit.
    prevXLim = ax.XLim;
    prevYLim = ax.YLim;
    firstDraw = isequal(prevXLim, [0 1]) && isequal(prevYLim, [0 1]);

    cla(ax, 'reset');
    % Patch C: cla(ax,'reset') wipes ax.ButtonDownFcn back to ''. Re-
    % install the stashed handle immediately so every left-click routes
    % into onAxesClick (add / select / drag-begin). Without this, only
    % the right-click UIContextMenu works.
    if ~isempty(state.axesClickFcn)
        ax.ButtonDownFcn = state.axesClickFcn;
    end
    hold(ax, 'on');
    view(ax, 2);
    % 2D is the editing mode — keep the editor's custom click routing by
    % disabling MATLAB's default axes interactions.
    disableDefaultInteractivity(ax);

    % Colorbar is a sibling of the axes; cla() does NOT remove it. Turn
    % it off here; the altitude-color branch re-enables when appropriate.
    colorbar(ax, 'off');

    % ── Radar site marker (editable position, M3.5) ──────────────────
    rx = state.radarEastM;
    ry = state.radarNorthM;
    plot(ax, rx, ry, '^', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [0.85 0.15 0.15], ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'Tag', 'radarSite');
    text(ax, rx, ry, '  radar', ...
        'VerticalAlignment', 'middle', ...
        'HorizontalAlignment', 'left', ...
        'FontWeight', 'bold', ...
        'Color', [0.6 0.1 0.1], ...
        'PickableParts', 'none', ...
        'Interpreter', 'none');

    % ── Waypoint path ────────────────────────────────────────────────
    wp = state.waypoints;
    n = size(wp, 1);

    if n >= 2
        plot(ax, wp(:,1), wp(:,2), '-', ...
            'Color', [0.20 0.45 0.85], ...
            'LineWidth', 1.8, ...
            'PickableParts', 'none', ...
            'Tag', 'pathLine');
    end

    if n >= 1
        if state.colorByAltitude
            drawWaypointsByAltitude2D(ax, wp);
        else
            drawWaypointsDefault2D(ax, wp);
        end
        for k = 1:n
            text(ax, wp(k,1), wp(k,2), sprintf('  %d', k), ...
                'FontWeight', 'bold', ...
                'FontSize', 10, ...
                'VerticalAlignment', 'bottom', ...
                'HorizontalAlignment', 'left', ...
                'PickableParts', 'none', ...
                'Tag', 'waypointLabel', ...
                'Interpreter', 'none');
        end
    end

    % ── Selection highlight (M2) ─────────────────────────────────────
    sel = state.selectedIndex;
    if sel >= 1 && sel <= n
        plot(ax, wp(sel,1), wp(sel,2), 'o', ...
            'MarkerSize', 18, ...
            'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', [0.10 0.55 0.25], ...
            'LineWidth', 2.0, ...
            'PickableParts', 'none', ...
            'Tag', 'selectedHalo');
        plot(ax, wp(sel,1), wp(sel,2), 'o', ...
            'MarkerSize', 11, ...
            'MarkerFaceColor', [0.20 0.75 0.35], ...
            'MarkerEdgeColor', [0.05 0.25 0.10], ...
            'LineWidth', 1.4, ...
            'PickableParts', 'none', ...
            'Tag', 'selectedMarker');
    end

    hold(ax, 'off');

    % ── Axes cosmetics ───────────────────────────────────────────────
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'East (m)  —  +X', 'Interpreter', 'none');
    ylabel(ax, 'North (m) —  +Y', 'Interpreter', 'none');
    zlabel(ax, '');
    title(ax, buildTitle(state, n), 'Interpreter', 'none');

    if firstDraw
        applyAutoFit2D(ax, wp, state);
    else
        ax.XLim = prevXLim;
        ax.YLim = prevYLim;
    end

    applyMinorGrid(ax, state.gridSpacingKm);

    % M3.2 — scale bar in lower-left. Drawn last so it sits on top.
    drawScaleBar(ax);

    drawnow limitrate;
end


function drawWaypointsDefault2D(ax, wp)
    plot(ax, wp(:,1), wp(:,2), 'o', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', [1.0 0.85 0.25], ...
        'MarkerEdgeColor', [0.1 0.1 0.1], ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'Tag', 'waypointMarkers');
end


function drawWaypointsByAltitude2D(ax, wp)
    [lo, hi] = altitudeRange(wp);
    colormap(ax, parula);
    clim(ax, [lo hi]);
    scatter(ax, wp(:,1), wp(:,2), 80, wp(:,3), 'filled', ...
        'MarkerEdgeColor', [0.1 0.1 0.1], ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'Tag', 'waypointMarkers');
    cb = colorbar(ax);
    cb.Label.String = 'Altitude (m)';
    cb.Label.Interpreter = 'none';
end


%% ========================================================================
%  3D RENDERING (M3.3)
%% ========================================================================

function drawMap3D(state, ax)
%drawMap3D  Perspective view — positive-up altitude, stems from ground,
%           3D path line. View-only (edits blocked in buildUI/onAxesClick).
%
%  VIEW-STATE PRESERVATION (fixes the M3.3 "camera resets on every redraw"
%  regression). On every redraw after the first in a given 3D session
%  we capture axis limits + camera state before cla(ax,'reset') and
%  restore them after drawing. Without this, every select/colormap-
%  toggle/grid-change snaps the camera back to default azimuth/
%  elevation/zoom, which is a terrible demo experience.
%
%  firstDraw3D is driven by state.has3DViewState (set to true at the
%  end of this function, reset to false in onViewModeChanged). That
%  way switching 2D→3D or 3D→2D→3D always autofits on re-entry.

    % ── Capture previous view state (before cla blows it away) ──────
    prevXLim       = ax.XLim;
    prevYLim       = ax.YLim;
    prevZLim       = ax.ZLim;
    prevView       = ax.View;
    prevCamPos     = ax.CameraPosition;
    prevCamTarget  = ax.CameraTarget;
    prevCamUp      = ax.CameraUpVector;
    prevCamVA      = ax.CameraViewAngle;
    firstDraw3D    = ~state.has3DViewState;

    cla(ax, 'reset');
    % Patch C: same as 2D — cla(ax,'reset') wipes ax.ButtonDownFcn. In
    % 3D the onAxesClick handler still routes selection (x,y only), so
    % we need the handle alive here too. Without this, 3D selection
    % goes dead after the first redraw.
    if ~isempty(state.axesClickFcn)
        ax.ButtonDownFcn = state.axesClickFcn;
    end
    hold(ax, 'on');

    % Re-enable default interactions so rotate/pan/zoom via the axes
    % toolbar works. Editing click/drag is blocked in onAxesClick, so
    % this does not conflict with M2 behavior.
    enableDefaultInteractivity(ax);

    % Colorbar off by default; altitude branch re-enables it.
    colorbar(ax, 'off');

    % ── Radar site at z=0 ────────────────────────────────────────────
    rx = state.radarEastM;
    ry = state.radarNorthM;
    plot3(ax, rx, ry, 0, '^', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [0.85 0.15 0.15], ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'Tag', 'radarSite3D');
    text(ax, rx, ry, 0, '  radar', ...
        'VerticalAlignment', 'middle', ...
        'HorizontalAlignment', 'left', ...
        'FontWeight', 'bold', ...
        'Color', [0.6 0.1 0.1], ...
        'PickableParts', 'none', ...
        'Interpreter', 'none');

    wp = state.waypoints;
    n = size(wp, 1);

    if n >= 1
        stemColor = [0.55 0.55 0.55];
        for k = 1:n
            plot3(ax, [wp(k,1) wp(k,1)], [wp(k,2) wp(k,2)], [0 wp(k,3)], ...
                ':', 'Color', stemColor, 'LineWidth', 1.0, ...
                'PickableParts', 'none', 'Tag', 'waypointStem');
        end

        if n >= 2
            plot3(ax, wp(:,1), wp(:,2), wp(:,3), '-', ...
                'Color', [0.20 0.45 0.85], ...
                'LineWidth', 1.8, ...
                'PickableParts', 'none', ...
                'Tag', 'pathLine3D');
        end

        if state.colorByAltitude
            drawWaypointsByAltitude3D(ax, wp);
        else
            drawWaypointsDefault3D(ax, wp);
        end

        for k = 1:n
            text(ax, wp(k,1), wp(k,2), wp(k,3), sprintf('  %d', k), ...
                'FontWeight', 'bold', ...
                'FontSize', 10, ...
                'VerticalAlignment', 'bottom', ...
                'HorizontalAlignment', 'left', ...
                'PickableParts', 'none', ...
                'Tag', 'waypointLabel3D', ...
                'Interpreter', 'none');
        end
    end

    sel = state.selectedIndex;
    if sel >= 1 && sel <= n
        plot3(ax, wp(sel,1), wp(sel,2), wp(sel,3), 'o', ...
            'MarkerSize', 18, ...
            'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', [0.10 0.55 0.25], ...
            'LineWidth', 2.0, ...
            'PickableParts', 'none', ...
            'Tag', 'selectedHalo3D');
        plot3(ax, wp(sel,1), wp(sel,2), wp(sel,3), 'o', ...
            'MarkerSize', 11, ...
            'MarkerFaceColor', [0.20 0.75 0.35], ...
            'MarkerEdgeColor', [0.05 0.25 0.10], ...
            'LineWidth', 1.4, ...
            'PickableParts', 'none', ...
            'Tag', 'selectedMarker3D');
    end

    hold(ax, 'off');

    grid(ax, 'on');
    xlabel(ax, 'East (m)  —  +X', 'Interpreter', 'none');
    ylabel(ax, 'North (m) —  +Y', 'Interpreter', 'none');
    zlabel(ax, 'Altitude (m)  —  +Z up', 'Interpreter', 'none');
    title(ax, buildTitle(state, n), 'Interpreter', 'none');

    % ── Either autofit (first draw in this 3D session) or restore ───
    if firstDraw3D
        view(ax, 3);
        apply3DLimits(ax, wp, state);
        apply3DAspect(ax, wp, state);
        state.has3DViewState = true;
    else
        % Restore the user's rotation / zoom / pan from before cla.
        % Setting these explicitly forces the corresponding *Mode
        % properties back to 'manual'.
        ax.XLim            = prevXLim;
        ax.YLim            = prevYLim;
        ax.ZLim            = prevZLim;
        ax.View            = prevView;
        ax.CameraPosition  = prevCamPos;
        ax.CameraTarget    = prevCamTarget;
        ax.CameraUpVector  = prevCamUp;
        ax.CameraViewAngle = prevCamVA;
        % Aspect is deterministic from waypoint data; safe to reapply.
        apply3DAspect(ax, wp, state);
    end

    applyMinorGrid(ax, state.gridSpacingKm);

    drawnow limitrate;
end


function drawWaypointsDefault3D(ax, wp)
    plot3(ax, wp(:,1), wp(:,2), wp(:,3), 'o', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', [1.0 0.85 0.25], ...
        'MarkerEdgeColor', [0.1 0.1 0.1], ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'Tag', 'waypointMarkers3D');
end


function drawWaypointsByAltitude3D(ax, wp)
    [lo, hi] = altitudeRange(wp);
    colormap(ax, parula);
    clim(ax, [lo hi]);
    scatter3(ax, wp(:,1), wp(:,2), wp(:,3), 80, wp(:,3), 'filled', ...
        'MarkerEdgeColor', [0.1 0.1 0.1], ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'Tag', 'waypointMarkers3D');
    cb = colorbar(ax);
    cb.Label.String = 'Altitude (m)';
    cb.Label.Interpreter = 'none';
end


function apply3DLimits(ax, wp, state)
%apply3DLimits  Choose axis limits for the 3D view. Called only on the
%               first draw of a 3D session; subsequent redraws preserve
%               the user's camera via drawMap3D's capture/restore.
    if isempty(wp)
        rx = state.radarEastM;
        ry = state.radarNorthM;
        ax.XLim = [rx - 20000, rx + 20000];
        ax.YLim = [ry - 20000, ry + 20000];
        ax.ZLim = [0, 5000];
        return;
    end
    xs = [state.radarEastM;  wp(:,1)];
    ys = [state.radarNorthM; wp(:,2)];
    zs = [0; wp(:,3)];
    xMin = min(xs); xMax = max(xs);
    yMin = min(ys); yMax = max(ys);
    zMin = min(zs); zMax = max(zs);
    xSpan = max(xMax - xMin, 2000);
    ySpan = max(yMax - yMin, 2000);
    zSpan = max(zMax - zMin, 500);
    pad = 0.10;
    ax.XLim = [xMin - pad*xSpan, xMax + pad*xSpan];
    ax.YLim = [yMin - pad*ySpan, yMax + pad*ySpan];
    ax.ZLim = [min(0, zMin - 0.05*zSpan), zMax + pad*zSpan];
end


function apply3DAspect(ax, wp, state)
%apply3DAspect  Because altitudes (a few km) are tiny next to horizontal
%               extents (tens of km), a true 1:1:1 aspect makes the 3D
%               view look like a 2D sheet. Exaggerate Z so users can
%               actually see altitude variation while keeping X:Y = 1.
    if isempty(wp)
        daspect(ax, [1 1 0.25]);  % decent default for the empty view
        return;
    end
    xSpan = max(max(wp(:,1)) - min([state.radarEastM;  wp(:,1)]), 2000);
    ySpan = max(max(wp(:,2)) - min([state.radarNorthM; wp(:,2)]), 2000);
    zMax = max(max(wp(:,3)), 500);
    horizSpan = max(xSpan, ySpan);
    % Aim for altitude to render at ~40% of horizontal span so it reads
    % as a clearly 3D scene. Clamp to a reasonable [3, 30] exaggeration.
    target = 0.4 * horizSpan;
    if zMax <= 0
        zExag = 10;
    else
        zExag = min(max(target / zMax, 3), 30);
    end
    daspect(ax, [1 1 1/zExag]);
end


%% ========================================================================
%  SCALE BAR (M3.2, 2D only)
%% ========================================================================

function drawScaleBar(ax)
%drawScaleBar  Overlay a horizontal scale bar + label in the axes
%              lower-left corner. Auto-rescales as the user zooms.
    xLim = ax.XLim;
    yLim = ax.YLim;
    xSpan = xLim(2) - xLim(1);
    ySpan = yLim(2) - yLim(1);
    if ~isfinite(xSpan) || xSpan <= 0
        return;
    end

    [barLenM, labelStr] = trackbench.editor.computeScaleBar(xSpan);

    x0 = xLim(1) + 0.05 * xSpan;
    y0 = yLim(1) + 0.05 * ySpan;
    x1 = x0 + barLenM;

    hold(ax, 'on');
    plot(ax, [x0 x1], [y0 y0], '-', ...
        'Color', [0.15 0.15 0.15], 'LineWidth', 3.0, ...
        'PickableParts', 'none', 'Tag', 'scaleBar');
    tick = 0.015 * ySpan;
    plot(ax, [x0 x0], [y0 - tick, y0 + tick], '-', ...
        'Color', [0.15 0.15 0.15], 'LineWidth', 2.0, ...
        'PickableParts', 'none', 'Tag', 'scaleBarTick');
    plot(ax, [x1 x1], [y0 - tick, y0 + tick], '-', ...
        'Color', [0.15 0.15 0.15], 'LineWidth', 2.0, ...
        'PickableParts', 'none', 'Tag', 'scaleBarTick');
    text(ax, (x0 + x1) / 2, y0 + 0.025 * ySpan, labelStr, ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontWeight', 'bold', ...
        'FontSize', 10, ...
        'Color', [0.15 0.15 0.15], ...
        'BackgroundColor', [0.98 0.98 0.98], ...
        'Margin', 1, ...
        'PickableParts', 'none', ...
        'Interpreter', 'none', ...
        'Tag', 'scaleBarLabel');
    hold(ax, 'off');
end


%% ========================================================================
%  SHARED HELPERS
%% ========================================================================

function [lo, hi] = altitudeRange(wp)
%altitudeRange  CLim/colorbar range with a ±1 m widening when all
%               waypoints share a single altitude (fresh defaults).
    alt = wp(:, 3);
    lo = min(alt);
    hi = max(alt);
    if hi - lo < 1
        lo = lo - 1;
        hi = hi + 1;
    end
end


function applyMinorGrid(ax, gridSpacingKm)
    if gridSpacingKm > 0
        step = gridSpacingKm * 1000;
        ax.XAxis.MinorTick = 'on';
        ax.YAxis.MinorTick = 'on';
        ax.XAxis.MinorTickValues = ax.XLim(1):step:ax.XLim(2);
        ax.YAxis.MinorTickValues = ax.YLim(1):step:ax.YLim(2);
        grid(ax, 'minor');
    else
        ax.XAxis.MinorTick = 'off';
        ax.YAxis.MinorTick = 'off';
    end
end


function s = buildTitle(state, n)
    namePart = char(state.targetName);
    if state.loadedFrom ~= ""
        [~, baseName, ext] = fileparts(char(state.loadedFrom));
        loadedPart = sprintf('  (loaded from %s%s)', baseName, ext);
    else
        loadedPart = '';
    end
    modeSuffix = '';
    if state.viewMode == "3d"
        modeSuffix = '  [3D view-only]';
    end
    if n > 0
        s = sprintf('Path Editor — %s  (%d waypoints, %.0f s)%s%s', ...
            namePart, n, state.durationS, loadedPart, modeSuffix);
    else
        s = sprintf('Path Editor — %s  (click on map to add waypoints)%s%s', ...
            namePart, loadedPart, modeSuffix);
    end
end


function applyAutoFit2D(ax, wp, state)
    if isempty(wp)
        rx = state.radarEastM;
        ry = state.radarNorthM;
        ax.XLim = [rx - 20000, rx + 20000];
        ax.YLim = [ry - 20000, ry + 20000];
        return;
    end
    xs = [state.radarEastM;  wp(:,1)];
    ys = [state.radarNorthM; wp(:,2)];
    xMin = min(xs); xMax = max(xs);
    yMin = min(ys); yMax = max(ys);
    xSpan = max(xMax - xMin, 2000);
    ySpan = max(yMax - yMin, 2000);
    pad = 0.10;
    ax.XLim = [xMin - pad*xSpan, xMax + pad*xSpan];
    ax.YLim = [yMin - pad*ySpan, yMax + pad*ySpan];
end
