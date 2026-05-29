function drawMap(state)
%drawMap  Re-render the editor axes from the current EditorState.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Called after every state mutation (add/move/remove/select/undo) and
%  after every view-option change (colormap toggle, grid spacing, 2D/3D).
%  Dispatches to drawMap2D or drawMap3D based on state.viewMode.
%
%  M5 §3.1 MULTI-TARGET RENDERING
%    Iterates state.targets drawing inactive targets FIRST (dimmed,
%    using each target's displayColor, no numbered labels) then the
%    active target LAST (full styling — canonical blue, yellow markers
%    or altitude colormap, numbered labels, selection halo, curve).
%    Active-last means the active path always sits on top of its
%    neighbors and the user's eye follows the path they're editing.
%    Inactive-target rendering degrades gracefully to "one target"
%    when only one exists (the inactive loop has zero iterations).
%
%  2D mode renders:
%    - Top-down East/North axes, straight-line path, numbered markers
%    - Radar site marker at (radarEastM, radarNorthM)  [M3.5 — editable]
%    - Selection halo (M2) — active target only
%    - Altitude colormap + colorbar when state.colorByAltitude == true (M3.1)
%      (applies to active target waypoints; inactive targets keep their
%       displayColor so the user can still tell them apart)
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

    % M7 §3.3 — storm timeline is rendered into its own axes below the
    % main map (not into ax). Always call it; the helper takes care of
    % hiding the strip when state.weather is empty. Wrapped in
    % isgraphics because early-path-editor sessions predate the axes.
    if isgraphics(state.weatherStormTimelineAx)
        drawStormTimeline(state.weatherStormTimelineAx, state);
    end
end


%% ========================================================================
%  2D RENDERING (M1–M3.2)
%% ========================================================================

function drawMap2D(state, ax)
    % Capture current 2D axis limits so we don't jump on every redraw
    % once the user has zoomed/panned. has2DViewState is the
    % authoritative "have we autofit this 2D session yet?" flag (v3.5
    % fix — the old XLim==[0,1] heuristic broke after a 3D session
    % corrupted ax.XLim/YLim, leaving stale 3D limits leaking into the
    % 2D view).
    prevXLim = ax.XLim;
    prevYLim = ax.YLim;
    firstDraw = ~state.has2DViewState;

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

    % ── v3.5 §5e — procedural heightmap (drawn FIRST so it lives
    %     behind all other graphics). Hypsometric tint shows actual
    %     elevation across the scenario, including any per-region
    %     terrain stamps (mountain / desert cuts inside region
    %     polygons). Calls the same generator the sim uses
    %     (trackbench.environment.generateTerrain + composeHeightmap),
    %     so what the editor previews matches what runs produce.
    %     Falls back to drawTerrainTint2D's flat color tint when
    %     generation fails (e.g. environment helpers off path).
    if isgraphics(state.terrainOverlayCB) && state.terrainOverlayCB.Value
        if ~drawTerrainHeightmap2D(ax, state)
            drawTerrainTint2D(ax, state);
        end
    end

    % ── v3.5 §5c.3b — region polygons (terrain + weather + draft) ──
    %   Drawn after the terrain bg tint and before sensors/targets so
    %   committed regions sit on top of the tint but below the user's
    %   primary editing focus. The in-progress polygon-edit draft (when
    %   active) renders in this same pass, so a vertex appended via
    %   onAxesClick is visible after the next drawMap call.
    drawRegionPolygons(ax, state);

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

    % ── M6 §3.3: sensor collection pass (range rings / wedges / cones) ──
    %   Drawn BEFORE targets so target paths and markers overlay sensor
    %   geometry (targets are the editing focus; sensor coverage is
    %   decorative context). Every graphics object here is
    %   PickableParts='none' — §3.6 will wire click-to-select sensors
    %   through the axes ButtonDownFcn (state.findSensorAt), not via
    %   graphics children intercepting clicks. Leaving markers pickable
    %   now would break waypoint-add when a target clicks on top of one.
    if numel(state.sensors) >= 1
        for k = 1:numel(state.sensors)
            isActiveSensor = (k == state.activeSensorIdx) && ...
                             (state.editMode == "sensors");
            drawSensor2D(ax, state.sensors(k), state, isActiveSensor);
        end
    end

    % ── M5 §3.1: render inactive targets first (dimmed) ──────────────
    %   Draw order matters — MATLAB stacks later children on top of
    %   earlier ones. Drawing inactive paths first keeps the active
    %   target visually dominant (its line/markers/labels all sit on
    %   top). Inactive targets reuse their per-target displayColor so
    %   they remain distinguishable from each other.
    if numel(state.targets) > 1
        for k = 1:numel(state.targets)
            if k == state.activeIdx; continue; end
            drawInactiveTarget2D(ax, state.targets(k), state);
        end
    end

    % ── Active target (full styling, drawn last → on top) ────────────
    wp = state.waypoints;
    n = size(wp, 1);

    if n >= 2
        % M4.3.2: straight segments (M3 behavior) vs. centripetal
        % Catmull-Rom curve through the control waypoints. The curve
        % itself is NOT a click target — PickableParts='none' keeps
        % waypoint markers on top as the interactive layer.
        if isCurveMode(state)
            dp = trackbench.editor.catmullRomCurve( ...
                wp, state.curveDensityPerSeg, state.curveTensionAlpha);
            if state.colorByAltitude
                [lo, hi] = altitudeRange(wp);
                drawPathColoredByAltitude2D(ax, dp, lo, hi);
            else
                plot(ax, dp(:,1), dp(:,2), '-', ...
                    'Color', [0.20 0.45 0.85], ...
                    'LineWidth', 1.8, ...
                    'PickableParts', 'none', ...
                    'HitTest', 'off', ...
                    'Tag', 'pathCurve2D');
            end
        else
            plot(ax, wp(:,1), wp(:,2), '-', ...
                'Color', [0.20 0.45 0.85], ...
                'LineWidth', 1.8, ...
                'PickableParts', 'none', ...
                'Tag', 'pathLine');
        end
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
        % M5 §3.1: pass the union of all targets' waypoints so
        % autofit frames every visible path on first draw.
        applyAutoFit2D(ax, allTargetsWaypoints(state), state);
        state.has2DViewState = true;
    else
        ax.XLim = prevXLim;
        ax.YLim = prevYLim;
    end

    applyMinorGrid(ax, state.gridSpacingKm);

    % M3.2 — scale bar in lower-left. Drawn last so it sits on top.
    drawScaleBar(ax);

    % M7 §3.3 — weather badge in top-right corner. Drawn AFTER the scale
    % bar so it's on top of everything (including the tint). No-op when
    % state.weather is empty.
    drawWeatherBadge2D(ax, state);

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
    %
    % v3.5 §5f post-mortem: tried five iterations of replacing this
    % with disableDefaultInteractivity + custom 3D pan and the
    % uifigure 3D interaction internals fought every approach
    % (camera translation, axis-limit shift, auto-mode forcing,
    % camdolly). Reverted. 3D pan via middle-click goes through
    % MATLAB's built-in mechanism here; custom shortcuts (arrows,
    % wheel, R) own rotation, zoom, reset.
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

    % ── v3.5 §5e — procedural heightmap surface ─────────────────────
    %     Renders the same Zterrain grid the sim engine uses, so the
    %     editor preview matches the run-time visualization. Drawn
    %     before sensors/targets so MATLAB's per-pixel depth sorting
    %     keeps waypoints flying ABOVE terrain visible while terrain
    %     occludes anything below it. Gated on the same checkbox as
    %     the 2D heightmap (Environment panel "Overlay on map").
    if isgraphics(state.terrainOverlayCB) && state.terrainOverlayCB.Value
        drawTerrainHeightmap3D(ax, state);
    end

    % ── M6 §3.3 / §3.6A: sensor collection pass ────────────────
    %   Per-sensor 3D coverage volumes rendered via drawRotatorVolume3D /
    %   drawSectorVolume3D / drawBeamConeVolume3D (added in M6 §3.6A).
    %   Editor sensors use a custom record class with tilt-aware geometry
    %   (beamAltEdgesM + sensorMountAltitudeM); computeSensorCoverageVolume
    %   (Sites 1+2 plotting path, v3.7.5+) is NOT used here — Path Editor
    %   sensors don't flow through dataLog.SensorCoverage.
    if numel(state.sensors) >= 1
        for k = 1:numel(state.sensors)
            isActiveSensor = (k == state.activeSensorIdx) && ...
                             (state.editMode == "sensors");
            drawSensor3D(ax, state.sensors(k), state, isActiveSensor);
        end
    end

    % ── M5 §3.1: render inactive targets first (dimmed) ──────────────
    if numel(state.targets) > 1
        for k = 1:numel(state.targets)
            if k == state.activeIdx; continue; end
            drawInactiveTarget3D(ax, state.targets(k), state);
        end
    end

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
            % M4.3.2: straight-vs-curved path in 3D. The 3D view is
            % still view-only (editing happens in 2D) and the curve,
            % like the straight line, is non-pickable.
            if isCurveMode(state)
                dp = trackbench.editor.catmullRomCurve( ...
                    wp, state.curveDensityPerSeg, state.curveTensionAlpha);
                if state.colorByAltitude
                    [lo, hi] = altitudeRange(wp);
                    drawPathColoredByAltitude3D(ax, dp, lo, hi);
                else
                    plot3(ax, dp(:,1), dp(:,2), dp(:,3), '-', ...
                        'Color', [0.20 0.45 0.85], ...
                        'LineWidth', 1.8, ...
                        'PickableParts', 'none', ...
                        'HitTest', 'off', ...
                        'Tag', 'pathCurve3D');
                end
            else
                plot3(ax, wp(:,1), wp(:,2), wp(:,3), '-', ...
                    'Color', [0.20 0.45 0.85], ...
                    'LineWidth', 1.8, ...
                    'PickableParts', 'none', ...
                    'Tag', 'pathLine3D');
            end
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
        % M5 §3.1: size to the union of all targets' waypoints so
        % every path is visible in the initial 3D frame.
        allWp = allTargetsWaypoints(state);
        apply3DLimits(ax, allWp, state);
        apply3DAspect(ax, allWp, state);
        % v3.5 fix — reset CameraViewAngleMode to 'auto' so a wide
        % perspective angle from the previous 3D session (e.g. user
        % scroll-zoomed out to 60° then toggled to 2D and back) doesn't
        % leak through and warp this autofit. axis vis3d freezes the
        % aspect ratio against rotation so the box stays stable when the
        % user orbits the camera — without it, daspect can re-stretch
        % the box on each rotation and look like "the plot is breathing".
        ax.CameraViewAngleMode = 'auto';
        axis(ax, 'vis3d');
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
        % Use all-target union so aspect stays stable as targets come
        % and go — otherwise deleting the active target could shift
        % the Z-exaggeration even though other targets' altitude
        % ranges are unchanged.
        apply3DAspect(ax, allTargetsWaypoints(state), state);
    end

    applyMinorGrid(ax, state.gridSpacingKm);

    % M7 §3.3 — weather badge in 3D. Handoff §3.5: text objects work in
    % 3D axes. Terrain tint is deliberately skipped in 3D (see §3.3).
    drawWeatherBadge2D(ax, state);

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

    % v3.5 fix — degenerate horizontal aspect guard. apply3DAspect uses
    % daspect [1 1 1/zExag] (1:1 in X:Y), so if one horizontal span is
    % much narrower than the other (e.g. single waypoint at far east, or
    % a strictly east-west route at y≈0), the narrow axis collapses to
    % a thin sliver. Pad the smaller span to at least 1/5 of the larger
    % so the 3D box has reasonable dimensions in all directions.
    horizMax = max(xSpan, ySpan);
    minHoriz = horizMax / 5;
    if xSpan < minHoriz
        xMid = (xMin + xMax) / 2;
        xMin = xMid - minHoriz / 2;
        xMax = xMid + minHoriz / 2;
        xSpan = minHoriz;
    end
    if ySpan < minHoriz
        yMid = (yMin + yMax) / 2;
        yMin = yMid - minHoriz / 2;
        yMax = yMid + minHoriz / 2;
        ySpan = minHoriz;
    end

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
    % M5 §3.1 — title now mentions total target count when there's
    % more than one. Single-target case is unchanged from M4 so
    % existing test screenshots stay readable. §3.2 will extend
    % this to "(N targets, R refs)".
    nT = numel(state.targets);
    if nT > 1
        countSuffix = sprintf('  [%d targets]', nT);
    else
        countSuffix = '';
    end
    if n > 0
        s = sprintf('Path Editor — %s  (%d waypoints, %.0f s)%s%s%s', ...
            namePart, n, state.durationS, loadedPart, countSuffix, modeSuffix);
    else
        s = sprintf('Path Editor — %s  (click on map to add waypoints)%s%s%s', ...
            namePart, loadedPart, countSuffix, modeSuffix);
    end
end


function wpAll = allTargetsWaypoints(state)
%allTargetsWaypoints  Concatenate every target's Nx5 waypoint matrix.
%                     Used by autofit/aspect helpers so the initial
%                     view frames every visible path, not just the
%                     active one. Returns 0x5 if no targets exist.
    if isempty(state.targets)
        wpAll = zeros(0, 5);
        return;
    end
    parts = cell(numel(state.targets), 1);
    for k = 1:numel(state.targets)
        parts{k} = state.targets(k).waypoints;
    end
    wpAll = vertcat(parts{:});
    if isempty(wpAll)
        wpAll = zeros(0, 5);
    end
end


function drawInactiveTarget2D(ax, tr, state)
%drawInactiveTarget2D  Render a non-active target in the 2D map.
%
%  Visual contract (M5 §3.1):
%    * Path line at the target's displayColor, slightly dimmed by
%      blending toward white (so the active target's canonical blue
%      stays the dominant element).
%    * Smaller markers, no numbered labels (would be visual noise
%      when several targets co-exist).
%    * Curve mode honoured per-target — `tr.curveMode` decides
%      between straight lines and Catmull-Rom for THIS target.
%    * No hover/select interactivity on inactive targets — the user
%      switches via the Targets dropdown to edit a different one.
    wp = tr.waypoints;
    n = size(wp, 1);
    if n < 1; return; end

    col = dimColor(tr.displayColor, 0.45);   % ~55% toward white

    if n >= 2
        if isCurveModeFor(tr)
            dp = trackbench.editor.catmullRomCurve( ...
                wp, tr.curveDensityPerSeg, tr.curveTensionAlpha);
            plot(ax, dp(:,1), dp(:,2), '-', ...
                'Color', col, ...
                'LineWidth', 1.4, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', 'inactivePathCurve2D');
        else
            plot(ax, wp(:,1), wp(:,2), '-', ...
                'Color', col, ...
                'LineWidth', 1.4, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', 'inactivePath2D');
        end
    end

    plot(ax, wp(:,1), wp(:,2), 'o', ...
        'MarkerSize', 5, ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', [0.35 0.35 0.35], ...
        'LineWidth', 0.8, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'inactiveWaypoints2D');

    % Single label at the first waypoint with the target name — light
    % anchor so the user can identify the dimmed paths visually.
    text(ax, wp(1,1), wp(1,2), sprintf('  %s', char(tr.targetName)), ...
        'FontSize', 9, ...
        'FontAngle', 'italic', ...
        'Color', col * 0.6, ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left', ...
        'PickableParts', 'none', ...
        'Interpreter', 'none', ...
        'Tag', 'inactiveTargetLabel2D');
    %#ok<*MAXES> — state arg kept for symmetry with 3D variant
end


function drawInactiveTarget3D(ax, tr, state)
%drawInactiveTarget3D  Render a non-active target in the 3D map.
%
%  Same contract as drawInactiveTarget2D but with plot3 / no stems
%  (stems for several targets would clutter the scene irretrievably).
    wp = tr.waypoints;
    n = size(wp, 1);
    if n < 1; return; end

    col = dimColor(tr.displayColor, 0.45);

    if n >= 2
        if isCurveModeFor(tr)
            dp = trackbench.editor.catmullRomCurve( ...
                wp, tr.curveDensityPerSeg, tr.curveTensionAlpha);
            plot3(ax, dp(:,1), dp(:,2), dp(:,3), '-', ...
                'Color', col, ...
                'LineWidth', 1.4, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', 'inactivePathCurve3D');
        else
            plot3(ax, wp(:,1), wp(:,2), wp(:,3), '-', ...
                'Color', col, ...
                'LineWidth', 1.4, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', 'inactivePath3D');
        end
    end

    plot3(ax, wp(:,1), wp(:,2), wp(:,3), 'o', ...
        'MarkerSize', 5, ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', [0.35 0.35 0.35], ...
        'LineWidth', 0.8, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'inactiveWaypoints3D');

    text(ax, wp(1,1), wp(1,2), wp(1,3), sprintf('  %s', char(tr.targetName)), ...
        'FontSize', 9, ...
        'FontAngle', 'italic', ...
        'Color', col * 0.6, ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left', ...
        'PickableParts', 'none', ...
        'Interpreter', 'none', ...
        'Tag', 'inactiveTargetLabel3D');
    %#ok<*MAXES>
end


function out = dimColor(rgb, towardWhite)
%dimColor  Blend an RGB triple toward white by `towardWhite` ∈ [0,1].
%          0 → unchanged; 1 → pure white. Used to fade inactive
%          targets so the active one remains visually dominant.
    rgb = max(0, min(1, double(rgb(:)')));
    a = max(0, min(1, double(towardWhite)));
    out = rgb * (1 - a) + [1 1 1] * a;
end


function tf = isCurveModeFor(tr)
%isCurveModeFor  TargetRecord-equivalent of isCurveMode(state). Used by
%                the inactive-target renderers so each target's
%                straight/curved mode is honored independently.
    if ~isprop(tr, 'curveMode') || isempty(tr.curveMode)
        tf = false;
        return;
    end
    tf = (tr.curveMode == "curved") && (size(tr.waypoints, 1) >= 2);
end


function tf = isCurveMode(state)
%isCurveMode  True when the editor is in M4 curve-rendering mode AND
%             there are enough control points for a meaningful curve.
%             Defensive against missing fields for backward-compat with
%             snapshots saved before M4.
    if ~isprop(state, 'curveMode') || isempty(state.curveMode)
        tf = false;
        return;
    end
    tf = (state.curveMode == "curved") && (size(state.waypoints, 1) >= 2);
end


function drawPathColoredByAltitude2D(ax, dp, lo, hi)
%drawPathColoredByAltitude2D  Draw the densified Catmull-Rom curve as a
%  set of altitude-binned polylines. Reuses parula + clim so the curve
%  colors match the M3 waypoint-marker colors exactly.
%
%  Why bins (rather than one surface with interp edge color):
%    * Each bin is a single plot call with NaN-separated segments, so
%      the total number of line objects is <= nBins (not one per dense
%      segment). That keeps the draw cheap during drag.
%    * `colormap`+`clim` for the colorbar stay the single source of
%      truth; we sample them here and line up the bin edges.
    colormap(ax, parula);
    clim(ax, [lo hi]);
    drawBinnedColoredPolyline(ax, dp, lo, hi, false, 'pathCurveSeg2D');
end


function drawPathColoredByAltitude3D(ax, dp, lo, hi)
%drawPathColoredByAltitude3D  Same as the 2D version but uses plot3 so
%  the curve threads through altitude in 3D. Reuses parula + clim.
    colormap(ax, parula);
    clim(ax, [lo hi]);
    drawBinnedColoredPolyline(ax, dp, lo, hi, true, 'pathCurveSeg3D');
end


function drawBinnedColoredPolyline(ax, dp, lo, hi, is3D, tag)
%drawBinnedColoredPolyline  Shared worker that groups dense segments by
%  altitude bin and emits one plot call per bin.
%
%  Inputs
%    dp    : Mx3 dense curve points [x y alt]
%    lo/hi : colormap data range (matches scatter's clim)
%    is3D  : true → plot3 with Z=alt, false → plot with just x,y
%    tag   : tag string for the created line objects
    nSeg = size(dp, 1) - 1;
    if nSeg < 1; return; end

    nBins = 32;
    cmap = parula(nBins);

    % Per-segment mean altitude drives the bin choice.
    altSeg = 0.5 * (dp(1:end-1, 3) + dp(2:end, 3));
    if hi - lo < eps
        bin = ones(size(altSeg));
    else
        bin = round((altSeg - lo) ./ (hi - lo) * (nBins - 1)) + 1;
        bin = max(1, min(nBins, bin));
    end

    % For each populated bin, emit one plot call whose segments are
    % separated by NaN — much cheaper than one plot per dense segment.
    for b = unique(bin(:))'
        mask = (bin == b);
        idx  = find(mask);
        % Every selected segment contributes a (start, end, NaN) triple.
        triples = numel(idx);
        X = nan(3*triples, 1);
        Y = nan(3*triples, 1);
        Z = nan(3*triples, 1);
        r = 1;
        for kk = 1:triples
            i = idx(kk);
            X(r)   = dp(i,   1); X(r+1) = dp(i+1, 1);
            Y(r)   = dp(i,   2); Y(r+1) = dp(i+1, 2);
            Z(r)   = dp(i,   3); Z(r+1) = dp(i+1, 3);
            r = r + 3;
        end
        if is3D
            plot3(ax, X, Y, Z, '-', ...
                'Color', cmap(b, :), ...
                'LineWidth', 2.0, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', tag);
        else
            plot(ax, X, Y, '-', ...
                'Color', cmap(b, :), ...
                'LineWidth', 2.0, ...
                'PickableParts', 'none', ...
                'HitTest', 'off', ...
                'Tag', tag);
        end
    end
end


function applyAutoFit2D(ax, wp, state)
% M6 §3.5 fold-in — auto-fit now spans sensor positions too, so a user
% who placed a sensor far from any target path still sees it when the
% editor opens or after Fit-to-Waypoints. Prior behavior (pre-M6) only
% considered the radar origin and waypoints; a sensor dropped at
% (+50 km, 0) would fall outside the 20 km default and be invisible.
    sxs = double.empty(0, 1);
    sys = double.empty(0, 1);
    if isprop(state, 'sensors') && ~isempty(state.sensors)
        n = numel(state.sensors);
        sxs = zeros(n, 1);
        sys = zeros(n, 1);
        for k = 1:n
            sxs(k) = state.sensors(k).positionEastM;
            sys(k) = state.sensors(k).positionNorthM;
        end
    end
    if isempty(wp) && isempty(sxs)
        rx = state.radarEastM;
        ry = state.radarNorthM;
        ax.XLim = [rx - 20000, rx + 20000];
        ax.YLim = [ry - 20000, ry + 20000];
        return;
    end
    if isempty(wp)
        wpX = double.empty(0, 1);
        wpY = double.empty(0, 1);
    else
        wpX = wp(:, 1);
        wpY = wp(:, 2);
    end
    xs = [state.radarEastM;  wpX; sxs];
    ys = [state.radarNorthM; wpY; sys];
    xMin = min(xs); xMax = max(xs);
    yMin = min(ys); yMax = max(ys);
    xSpan = max(xMax - xMin, 2000);
    ySpan = max(yMax - yMin, 2000);
    pad = 0.10;
    ax.XLim = [xMin - pad*xSpan, xMax + pad*xSpan];
    ax.YLim = [yMin - pad*ySpan, yMax + pad*ySpan];
end


%% ========================================================================
%  SENSOR RENDERING  (M6 §3.3)
%% ========================================================================
%
%  Bearing convention (matches fusionRadarSensor body frame with identity
%  orientation): 0° = +X (East), 90° = +Y (North), ±180° = -X (West),
%  -90° / 270° = -Y (South). Angles increase counter-clockwise (standard
%  math convention, which is what MATLAB atan2 / cos / sin use).
%
%  Worked examples:
%    * PSR / ASR rotator, sectorDeg = [0 360]       → full ring
%    * PAR sector,       sectorDeg = [170 190]      → narrow wedge,
%                                                     near-West
%    * TWS no-scan,      sectorDeg = [-60 60]       → 120° forward stare,
%                                                     centered on East
%
%  Visual contract (per user §3.3 scope):
%    * rotator   → dashed range ring (no fill)
%    * sector    → filled translucent wedge + solid radial/arc outline
%    * no-scan   → unfilled "beam cone" — faint fill, dashed radials
%                  (distinct from the filled sector wedge to convey the
%                  "staring / no-scan" semantic)
%    * UNKNOWN passthrough (readOnly) → dimmed gray ring, no fill
%
%  Click routing: every sensor graphics object is PickableParts='none'.
%  Click-to-select sensors is §3.6 work and goes through onAxesClick +
%  state.findSensorAt, not through graphics child hit-testing (same
%  model the waypoint editor uses).


function drawSensor2D(ax, sr, state, isActive)
%drawSensor2D  Render one sensor in the 2D view: coverage geometry first,
%              then position marker + label + optional active halo.
    pos = [sr.positionEastM, sr.positionNorthM];
    col = sr.displayColor;
    if sr.readOnly
        col = dimColor(col, 0.30);   % UNKNOWN passthrough → dimmed
    end

    % ── Coverage geometry (drawn first so marker/label sit on top) ──
    rMax = max(sr.rangeLimits(2), 1);   % guard against 0
    if sr.isRotator()
        drawRangeRing2D(ax, pos, rMax, col);
    elseif sr.isNoScan()
        drawBeamCone2D(ax, pos, rMax, sr.sectorDeg, col);
    elseif sr.isSector()
        drawSectorWedge2D(ax, pos, rMax, sr.sectorDeg, col);
    else
        % UNKNOWN type or ill-formed sector — anchor ring only.
        drawRangeRing2D(ax, pos, max(rMax, 10000), [0.5 0.5 0.5]);
    end

    % ── Position marker (triangle, black edge, filled in sensor color) ──
    markerSize = 10;
    if isActive; markerSize = 14; end
    plot(ax, pos(1), pos(2), '^', ...
        'MarkerSize', markerSize, ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorMarker2D');

    % ── Label: "name (TYPE)" ───────────────────────────────────────
    text(ax, pos(1), pos(2), ...
        sprintf('  %s (%s)', char(sr.sensorName), char(sr.sensorType)), ...
        'FontWeight', 'bold', ...
        'FontSize', 9, ...
        'Color', col * 0.6, ...
        'VerticalAlignment', 'middle', ...
        'HorizontalAlignment', 'left', ...
        'PickableParts', 'none', ...
        'Interpreter', 'none', ...
        'Tag', 'sensorLabel2D');

    % ── Active halo (only when editMode == "sensors" + this is active) ──
    if isActive
        xSpan = ax.XLim(2) - ax.XLim(1);
        haloR = max(500, 0.015 * xSpan);
        phi = linspace(0, 2*pi, 64);
        plot(ax, pos(1) + haloR*cos(phi), pos(2) + haloR*sin(phi), '--', ...
            'Color', [0.15 0.65 0.20], ...
            'LineWidth', 2.0, ...
            'PickableParts', 'none', ...
            'HitTest', 'off', ...
            'Tag', 'sensorActiveHalo2D');
    end
end


%% ========================================================================
%  M7 §3.3 — ENVIRONMENT RENDERING (terrain tint, weather badge, timeline)
%% ========================================================================

function drawTerrainTint2D(ax, state)
%drawTerrainTint2D  Full-extent semi-transparent background tint for the
%                    active terrain type. Drawn FIRST (before any other
%                    graphics in drawMap2D) so the rest of the scene sits
%                    on top. PickableParts='none' so the tint never
%                    intercepts map clicks.
%
%  Per-type colors per handoff §3.3:
%    none     → skip entirely (no tint — clean blank background)
%    water    → pale blue
%    rural    → pale green
%    urban    → pale gray
%    mountain → pale brown
%    desert   → pale sand
%    otherwise (UNKNOWN passthrough) → neutral gray
%
%  Covers a generous rectangle beyond the current XLim/YLim so subsequent
%  autoFit expansions during the same draw don't reveal an untinted
%  border. The patch is re-issued every drawMap call, so stale coverage
%  from a prior viewport is cleared by cla(ax,'reset').
    tr = state.terrain;
    % v3.5 §5e: "none" means "no terrain modelling" — render absolutely
    % nothing so the editor opens with a blank canvas. Skipping the tint
    % is the simplest way to keep the map clean without special-casing
    % the heightmap renderer or the colorbar.
    if strcmpi(tr.terrainType, "none")
        return;
    end
    tint = terrainTintColor(tr.terrainType);

    % Pick a rectangle at least as wide as the active XLim/YLim, with a
    % 2× extent so pan doesn't expose untinted area until the next draw.
    xl = ax.XLim;  yl = ax.YLim;
    dx = xl(2) - xl(1);  dy = yl(2) - yl(1);
    x0 = xl(1) - dx;   x1 = xl(2) + dx;
    y0 = yl(1) - dy;   y1 = yl(2) + dy;
    patch(ax, [x0 x1 x1 x0], [y0 y0 y1 y1], tint, ...
        'FaceAlpha', 0.20, ...
        'EdgeColor', 'none', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'terrainTint2D');
end


function rgb = terrainTintColor(terrainType)
%terrainTintColor  Per-type background tint. Unknown types get neutral
%                   gray so UNKNOWN-passthrough terrain still renders.
%                   "none" is included for completeness but callers
%                   (drawTerrainTint2D) skip the patch entirely for
%                   "none" — the returned color is never actually drawn.
    switch lower(string(terrainType))
        case "none";     rgb = [1.00 1.00 1.00];  % never rendered; placeholder
        case "water";    rgb = [0.60 0.75 0.90];
        case "rural";    rgb = [0.60 0.75 0.50];
        case "urban";    rgb = [0.70 0.70 0.70];
        case "mountain"; rgb = [0.75 0.65 0.50];
        case "desert";   rgb = [0.90 0.85 0.65];
        otherwise;       rgb = [0.80 0.80 0.80];
    end
end


function drawWeatherBadge2D(ax, state)
%drawWeatherBadge2D  Compact text badge anchored to the top-right corner
%                     of the map axes. No-op when state.weather is empty.
%
%  Two lines:
%    Line 1: "<TypeCap>  <rate><unit>"
%    Line 2: "Storm <start>–<end>s · <Profile>"
%
%  ASCII labels (no Unicode glyphs) chosen per handoff §8 — R2025b text
%  rendering is reliable for plain ASCII; weather icons can tofu-box on
%  some font fallbacks. Keeping it ASCII avoids a demo-time surprise.
    if isempty(state.weather); return; end
    wr = state.weather;

    % Per-type rate unit / label meaning (see WeatherRecord docstring).
    tcap = capFirst(char(wr.weatherType));
    switch lower(string(wr.weatherType))
        case "rain";  rateStr = sprintf('%g mm/hr',  wr.rainRateMmhr);
        case "snow";  rateStr = sprintf('%g mm/hr',  wr.rainRateMmhr);
        case "fog";   rateStr = sprintf('%g density', wr.rainRateMmhr);
        case "icing"; rateStr = sprintf('%g severity', wr.rainRateMmhr);
        otherwise;    rateStr = sprintf('%g',         wr.rainRateMmhr);
    end
    profCap = capFirst(char(wr.activeType));
    line1 = sprintf('%s  %s', tcap, rateStr);
    line2 = sprintf('Storm %g-%gs  %s', wr.stormStartS, wr.stormEndS, profCap);
    if wr.readOnly
        line1 = sprintf('%s  (unknown)', tcap);
    end

    xRight = ax.XLim(2);
    yTop   = ax.YLim(2);
    text(ax, xRight, yTop, {line1; line2}, ...
        'HorizontalAlignment', 'right', ...
        'VerticalAlignment',   'top', ...
        'BackgroundColor',     [1 1 1], ...
        'EdgeColor',           [0.6 0.6 0.6], ...
        'Margin',              4, ...
        'FontSize',            11, ...
        'Interpreter',         'none', ...
        'PickableParts',       'none', ...
        'HitTest',             'off', ...
        'Tag',                 'weatherBadge2D');
end


function drawStormTimeline(ax, state)
%drawStormTimeline  Thin below-the-map strip showing the storm window
%                    against the scenario's duration. Hidden (Visible='off')
%                    when state.weather is empty.
%
%  Scenario duration = max over targets' durationS. With zero targets or
%  all-zero durations the strip falls back to [0, max(stormEnd, 60)] so
%  the weather window is still visible.
%
%  Profile rendering:
%    step  — solid rectangle filling [stormStart..stormEnd]
%    ramp  — trapezoid peaking at the midpoint
%    pulse — narrow bar in the first 20% of the window
%
%  PickableParts='none' on the patch/axes so the strip never steals
%  clicks. The axes itself has HitTest='off' at construct-time in
%  buildUI.
    if ~isgraphics(ax); return; end
    cla(ax);
    if isempty(state.weather)
        ax.Visible = 'off';
        return;
    end
    ax.Visible = 'on';
    wr = state.weather;

    % Scenario duration — union over every target.
    duration = 0;
    if ~isempty(state.targets)
        for k = 1:numel(state.targets)
            d = 0;
            tr = state.targets(k);
            if isprop(tr, 'durationS') && ~isempty(tr.durationS)
                d = tr.durationS;
            end
            if d > duration; duration = d; end
        end
    end
    if duration <= 0
        duration = max(wr.stormEndS, 60);
    end

    t0 = wr.stormStartS;
    t1 = wr.stormEndS;
    if t1 <= t0; t1 = t0 + 1; end

    hold(ax, 'on');

    % Duration baseline — light gray horizontal track.
    plot(ax, [0 duration], [0 0], '-', ...
        'Color', [0.75 0.75 0.75], 'LineWidth', 1.2, ...
        'PickableParts', 'none', 'HitTest', 'off');

    % Storm window shape per profile.
    prof = lower(string(wr.activeType));
    switch prof
        case "step"
            xs = [t0 t1 t1 t0];
            ys = [0  0  1  1];
        case "ramp"
            tm = (t0 + t1) / 2;
            xs = [t0 tm t1];
            ys = [0  1  0];
        case "pulse"
            tp = t0 + (t1 - t0) * 0.20;
            xs = [t0 tp tp t0];
            ys = [0  0  1  1];
        otherwise
            xs = [t0 t1 t1 t0];
            ys = [0  0  1  1];
    end
    patch(ax, xs, ys, [0.20 0.45 0.85], ...
        'FaceAlpha', 0.35, 'EdgeColor', [0.10 0.30 0.65], ...
        'LineWidth', 1.1, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'stormTimeline');

    % End-cap labels for 0 and duration. Drawn as small grey text.
    text(ax, 0, -0.05, '0s', 'FontSize', 8, ...
        'HorizontalAlignment', 'left',  'VerticalAlignment', 'top', ...
        'Color', [0.45 0.45 0.45], 'Interpreter', 'none', ...
        'PickableParts', 'none', 'HitTest', 'off');
    text(ax, duration, -0.05, sprintf('%gs', duration), 'FontSize', 8, ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
        'Color', [0.45 0.45 0.45], 'Interpreter', 'none', ...
        'PickableParts', 'none', 'HitTest', 'off');
    % Storm-start / end annotations above the bar.
    text(ax, t0, 1.05, sprintf('%gs', t0), 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'Color', [0.15 0.30 0.65], 'Interpreter', 'none', ...
        'PickableParts', 'none', 'HitTest', 'off');
    text(ax, t1, 1.05, sprintf('%gs', t1), 'FontSize', 8, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'Color', [0.15 0.30 0.65], 'Interpreter', 'none', ...
        'PickableParts', 'none', 'HitTest', 'off');

    ax.XLim = [0 duration];
    ax.YLim = [-0.2 1.4];
    hold(ax, 'off');
end


function s = capFirst(str)
%capFirst  "rain" -> "Rain". Single-char safe.
    if isempty(str); s = ''; return; end
    s = [upper(str(1)), lower(str(2:end))];
end


function drawSensor3D(ax, sr, state, isActive)
%drawSensor3D  3D sensor rendering with real beam volumes (M6 §3.6A).
%              Previously flat-on-ground at z=0 — replaced with:
%                * Rotator  → extruded cylinder (ground ring + ceiling
%                             ring + vertical connectors) up to the
%                             upper beam edge at max range.
%                * Sector   → vertical wedge (floor + ceiling + radial
%                             side walls + curved outer wall).
%                * No-scan  → oblique cone, apex at the mount top,
%                             half-angle = elFov/2, centerline tilted
%                             by `tilt` in elevation and aimed at
%                             mean(sectorDeg) in azimuth.
%              Mounting altitude is still shown as the vertical stem
%              from the ground marker up to the mounting height
%              (mountingLoc(3) is NED-negative for above-ground, so
%              flip sign for +Z up). The unknown-type fallback keeps
%              the old flat gray range ring — there's no beam geometry
%              to stand on for an UNKNOWN passthrough.
    pos = [sr.positionEastM, sr.positionNorthM];
    col = sr.displayColor;
    if sr.readOnly
        col = dimColor(col, 0.30);
    end

    rMax = max(sr.rangeLimits(2), 1);
    mountAlt = sensorMountAltitudeM(sr);   % shared with the volume drawers
    if sr.isRotator()
        drawRotatorVolume3D(ax, pos, rMax, sr, mountAlt, col);
    elseif sr.isNoScan()
        drawBeamConeVolume3D(ax, pos, rMax, sr, mountAlt, col);
    elseif sr.isSector()
        drawSectorVolume3D(ax, pos, rMax, sr, mountAlt, col);
    else
        drawRangeRing3D(ax, pos, max(rMax, 10000), [0.5 0.5 0.5]);
    end

    % ── Mounting stem: ground → tower height ───────────────────────
    plot3(ax, [pos(1) pos(1)], [pos(2) pos(2)], [0, mountAlt], ...
        ':', ...
        'Color', col * 0.6, ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorMountStem3D');

    % ── Ground marker + altitude marker ────────────────────────────
    markerSize = 10;
    if isActive; markerSize = 14; end
    plot3(ax, pos(1), pos(2), 0, '^', ...
        'MarkerSize', markerSize, ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorMarker3D');
    plot3(ax, pos(1), pos(2), mountAlt, 's', ...
        'MarkerSize', max(5, markerSize - 4), ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorMountTop3D');

    % ── Label at mount-top (readable above the stem) ───────────────
    text(ax, pos(1), pos(2), mountAlt, ...
        sprintf('  %s (%s)', char(sr.sensorName), char(sr.sensorType)), ...
        'FontWeight', 'bold', ...
        'FontSize', 9, ...
        'Color', col * 0.6, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'left', ...
        'PickableParts', 'none', ...
        'Interpreter', 'none', ...
        'Tag', 'sensorLabel3D');
end


function drawRangeRing2D(ax, pos, r, col)
%drawRangeRing2D  Dashed polyline circle at max range. 64 segments gives
%                 a smooth circle at typical zooms without flooding the
%                 scene with graphics children.
    phi = linspace(0, 2*pi, 64);
    plot(ax, pos(1) + r*cos(phi), pos(2) + r*sin(phi), '--', ...
        'Color', col, ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorRangeRing2D');
end


function drawRangeRing3D(ax, pos, r, col)
    phi = linspace(0, 2*pi, 64);
    plot3(ax, pos(1) + r*cos(phi), pos(2) + r*sin(phi), zeros(1, 64), '--', ...
        'Color', col, ...
        'LineWidth', 1.2, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorRangeRing3D');
end


function drawSectorWedge2D(ax, pos, r, sectorDeg, col)
%drawSectorWedge2D  Filled translucent wedge for rotating sector scanners
%                   (PAR, narrow sweep). FaceAlpha low enough that targets
%                   behind the sensor coverage are still clearly visible.
%                   Bearing convention: degrees CCW from +X (see module
%                   header).
    [arcX, arcY] = sectorArc(pos, r, sectorDeg, 48);
    patchX = [pos(1), arcX, pos(1)];
    patchY = [pos(2), arcY, pos(2)];
    patch(ax, patchX, patchY, col, ...
        'FaceAlpha', 0.18, ...
        'EdgeColor', col, ...
        'LineWidth', 1.2, ...
        'LineStyle', '-', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorSectorWedge2D');
end


function drawSectorVolume3D(ax, pos, rMax, sr, mountAlt, col)
%drawSectorVolume3D  Vertical wedge swept through the elevation FOV
%                    (M6 §3.6A). Replaces the old flat-on-ground
%                    drawSectorWedge3D.
%
%  Geometry:
%    * Floor wedge at z = altLow (lower beam edge altitude at rMax,
%      clipped to ≥ 0 so a downward-tilted beam doesn't sink below
%      the ground plane).
%    * Ceiling wedge at z = altHigh (upper beam edge altitude at rMax,
%      floored to mountAlt + 10 so horizontal beams still render with
%      non-zero height).
%    * Two radial side walls at sectorDeg(1) and sectorDeg(2),
%      quadrilaterals connecting floor to ceiling.
%    * Curved outer wall at r = rMax, rendered as a surf so the
%      sampling along the arc stays smooth (a single patch would be
%      a flat polygon warped in 3D).
%
%  Visual contract mirrors drawSectorWedge2D: translucent fill with
%  a solid-line edge for the "mechanical sweep" read.
    [arcX, arcY] = sectorArc(pos, rMax, sr.sectorDeg, 48);
    [altLow, altHigh] = beamAltEdgesM(sr, rMax, mountAlt);
    % Floor wedge.
    patchX = [pos(1), arcX, pos(1)];
    patchY = [pos(2), arcY, pos(2)];
    patch(ax, patchX, patchY, altLow * ones(size(patchX)), col, ...
        'FaceAlpha', 0.16, ...
        'EdgeColor', col, ...
        'LineWidth', 1.2, ...
        'LineStyle', '-', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorSectorFloor3D');
    % Ceiling wedge (slightly fainter so the viewer reads it as "top").
    patch(ax, patchX, patchY, altHigh * ones(size(patchX)), col, ...
        'FaceAlpha', 0.10, ...
        'EdgeColor', col, ...
        'LineWidth', 1.0, ...
        'LineStyle', '-', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorSectorCeiling3D');
    % Two radial side walls.
    for ang = sr.sectorDeg
        a = deg2rad(ang);
        xEnd = pos(1) + rMax * cos(a);
        yEnd = pos(2) + rMax * sin(a);
        wallX = [pos(1), xEnd, xEnd, pos(1)];
        wallY = [pos(2), yEnd, yEnd, pos(2)];
        wallZ = [altLow, altLow, altHigh, altHigh];
        patch(ax, wallX, wallY, wallZ, col, ...
            'FaceAlpha', 0.10, ...
            'EdgeColor', col, ...
            'LineWidth', 1.0, ...
            'LineStyle', '-', ...
            'PickableParts', 'none', ...
            'HitTest', 'off', ...
            'Tag', 'sensorSectorWall3D');
    end
    % Curved outer wall (ruled surface, r = rMax).
    surfX = [arcX; arcX];
    surfY = [arcY; arcY];
    surfZ = [altLow * ones(size(arcX)); altHigh * ones(size(arcX))];
    surf(ax, surfX, surfY, surfZ, ...
        'FaceColor', col, 'FaceAlpha', 0.08, ...
        'EdgeColor', col, 'EdgeAlpha', 0.25, ...
        'LineStyle', '-', 'LineWidth', 0.5, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorSectorOuter3D');
end


function drawRotatorVolume3D(ax, pos, rMax, sr, mountAlt, col)
%drawRotatorVolume3D  360° cylindrical coverage volume for mechanical
%                     rotators (M6 §3.6A). Replaces the old flat range
%                     ring in 3D. Ground ring at altLow, ceiling ring
%                     at altHigh, plus four vertical dashed connectors
%                     (every 90°) so the cylinder reads unambiguously
%                     in a perspective view.
    phi = linspace(0, 2*pi, 64);
    [altLow, altHigh] = beamAltEdgesM(sr, rMax, mountAlt);
    % Ground ring.
    plot3(ax, pos(1) + rMax*cos(phi), pos(2) + rMax*sin(phi), ...
        altLow * ones(1, 64), '--', ...
        'Color', col, 'LineWidth', 1.2, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorRotatorRingLow3D');
    % Ceiling ring.
    plot3(ax, pos(1) + rMax*cos(phi), pos(2) + rMax*sin(phi), ...
        altHigh * ones(1, 64), '--', ...
        'Color', col, 'LineWidth', 1.2, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorRotatorRingHigh3D');
    % Vertical connectors every 90° — four dashed lines give enough
    % visual scaffolding to read the volume without cluttering.
    for angDeg = [0 90 180 270]
        x = pos(1) + rMax * cosd(angDeg);
        y = pos(2) + rMax * sind(angDeg);
        plot3(ax, [x x], [y y], [altLow, altHigh], ':', ...
            'Color', col, 'LineWidth', 0.9, ...
            'PickableParts', 'none', 'HitTest', 'off', ...
            'Tag', 'sensorRotatorConnector3D');
    end
end


function drawBeamCone2D(ax, pos, r, sectorDeg, col)
%drawBeamCone2D  No-scan (staring) beam visualization for TWS / AESA /
%                FIRE_CONTROL. Rendered as an unfilled cone — distinct
%                from drawSectorWedge2D to convey "beam" vs. "mechanical
%                sweep". Dashed radial edges + solid arc at max range,
%                very faint fill so the user can still see coverage
%                footprint at a glance but the visual weight says
%                "fixed beam" not "scanning area".
    [arcX, arcY] = sectorArc(pos, r, sectorDeg, 48);
    patchX = [pos(1), arcX, pos(1)];
    patchY = [pos(2), arcY, pos(2)];
    % Faint translucent fill (optional — drop FaceAlpha to 0 to disable).
    patch(ax, patchX, patchY, col, ...
        'FaceAlpha', 0.06, ...
        'EdgeColor', 'none', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'sensorBeamConeFill2D');
    % Dashed radial edges.
    a1 = deg2rad(sectorDeg(1));
    a2 = deg2rad(sectorDeg(2));
    plot(ax, [pos(1), pos(1) + r*cos(a1)], [pos(2), pos(2) + r*sin(a1)], '--', ...
        'Color', col, 'LineWidth', 1.4, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorBeamConeEdge2D');
    plot(ax, [pos(1), pos(1) + r*cos(a2)], [pos(2), pos(2) + r*sin(a2)], '--', ...
        'Color', col, 'LineWidth', 1.4, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorBeamConeEdge2D');
    % Solid outer arc (distinguishes the "beam range envelope").
    plot(ax, arcX, arcY, '-', ...
        'Color', col, 'LineWidth', 1.4, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorBeamConeArc2D');
end


function drawBeamConeVolume3D(ax, pos, rMax, sr, mountAlt, col)
%drawBeamConeVolume3D  Oblique cone volume for no-scan / staring
%                      sensors (M6 §3.6A). Apex at the mount point
%                      (pos + mountAlt), axis aimed at az =
%                      sectorCenterDeg(sr.sectorDeg) (wrap-aware,
%                      §3.7B.2), el = tilt; half-angle = fov(2)/2
%                      (elevation FOV). The cone is circular — azimuth
%                      FOV is used only to set the centerline
%                      direction, per the §3.6 spec.
%
%  Visual contract mirrors drawBeamCone2D: faint translucent fill
%  (surf FaceAlpha 0.10), dashed side-edges (EdgeAlpha/LineStyle on
%  the surf), solid outer rim (the beam-envelope circle at rMax).
%
%  Ground clipping: the raw cone can dip below z=0 for beams that are
%  wide enough in el or tilted down far enough. We clip rim altitudes
%  to ≥ 0 so nothing renders under the earth. The clip is cosmetic —
%  the beam is still conceptually conical, but the visible shell stops
%  at the ground plane.
    halfAngEl = max(0.5, sr.fov(2) / 2);    % °, floor at 0.5° so the cone is drawable
    % Wrap-aware azimuth mean (§3.7B.2). Needed because a no-scan sector
    % like [350 10] has its forward axis at 0°, not 180°.
    centerAz  = trackbench.editor.sectorCenterDeg(sr.sectorDeg);
    centerEl  = sr.tilt;                     % centerline elevation, °
    nPhi      = 36;
    phi       = linspace(0, 2*pi, nPhi + 1);

    % Build cone rim in a sensor-local frame where the cone axis is +X.
    % Rim at distance rMax, radial offset = rMax * tand(halfAngEl).
    rRim   = rMax * tand(halfAngEl);
    localX = rMax * ones(1, nPhi + 1);
    localY = rRim * cos(phi);
    localZ = rRim * sin(phi);

    % Rotate: pitch up by centerEl around local Y, then yaw by centerAz
    % around world Z (axis convention: bearing CCW from +X).
    cEl = cosd(centerEl); sEl = sind(centerEl);
    cAz = cosd(centerAz); sAz = sind(centerAz);
    xP  = localX*cEl - localZ*sEl;
    yP  = localY;
    zP  = localX*sEl + localZ*cEl;
    rimX = pos(1) + xP*cAz - yP*sAz;
    rimY = pos(2) + xP*sAz + yP*cAz;
    rimZ = mountAlt + zP;
    rimZ = max(rimZ, 0);    % cosmetic ground clip

    % Cone shell as a ruled surface from apex to rim.
    apexX = pos(1) * ones(1, nPhi + 1);
    apexY = pos(2) * ones(1, nPhi + 1);
    apexZ = mountAlt * ones(1, nPhi + 1);
    shellX = [apexX; rimX];
    shellY = [apexY; rimY];
    shellZ = [apexZ; rimZ];
    surf(ax, shellX, shellY, shellZ, ...
        'FaceColor', col, 'FaceAlpha', 0.10, ...
        'EdgeColor', col, 'EdgeAlpha', 0.25, ...
        'LineStyle', '--', 'LineWidth', 0.5, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorBeamConeShell3D');
    % Solid outer rim (mirrors drawBeamCone2D's solid arc at rMax).
    plot3(ax, rimX, rimY, rimZ, '-', ...
        'Color', col, 'LineWidth', 1.4, ...
        'PickableParts', 'none', 'HitTest', 'off', ...
        'Tag', 'sensorBeamConeRim3D');
end


function mountAltM = sensorMountAltitudeM(sr)
%sensorMountAltitudeM  Mount altitude in +Z-up metres (for 3D drawing).
%                      mountingLoc(3) is stored in NED, so above-ground
%                      values are negative; flip the sign. Malformed
%                      records fall back to a 15 m tower default so
%                      drawSensor3D doesn't emit zero-height stems.
    mountAltM = -sr.mountingLoc(3);
    if mountAltM <= 0; mountAltM = 15; end
end


function [altLow, altHigh] = beamAltEdgesM(sr, rangeM, mountAltM)
%beamAltEdgesM  Altitudes (metres, +Z up) where the upper and lower
%                elevation-FOV edges reach `rangeM`. Formula:
%                    altEdge = mountAlt + range * tand(±fov_el/2 + tilt)
%                Lower edge is clipped to 0 (no below-ground volumes).
%                Upper edge is floored to mountAlt + 10 so horizontal
%                beams still render with non-zero cylinder height.
%                NOTE: this function is COSMETIC — the clipping/flooring
%                above is for 3D rendering only. The altitude-window
%                banner readout (refreshSensorParamsPanel, §3.6B)
%                intentionally computes the same ±fov_el/2+tilt formula
%                WITHOUT clipping, so it reports the true physics even
%                when the beam dips below ground. The parallel math is
%                deliberate; do not unify.
    fovEl    = max(0, sr.fov(2));
    tiltDeg  = sr.tilt;
    altLow   = mountAltM + rangeM * tand(-fovEl/2 + tiltDeg);
    altHigh  = mountAltM + rangeM * tand(+fovEl/2 + tiltDeg);
    altLow   = max(altLow, 0);
    altHigh  = max(altHigh, mountAltM + 10);
end


function [arcX, arcY] = sectorArc(pos, r, sectorDeg, nSeg)
%sectorArc  Sample the polar arc from sectorDeg(1) to sectorDeg(2) with
%           nSeg+1 points. Handles sectors that don't wrap around. If
%           startAz > endAz (e.g. user enters [350 10]), goes the "long
%           way" — for M6 the Add modal + sensor params UI constrain
%           inputs enough that pathological wraps shouldn't hit here,
%           but the simulation uses abs(diff()) semantics so this
%           matches.
    a1 = deg2rad(sectorDeg(1));
    a2 = deg2rad(sectorDeg(2));
    t = linspace(a1, a2, nSeg + 1);
    arcX = pos(1) + r * cos(t);
    arcY = pos(2) + r * sin(t);
end


%% ========================================================================
%  v3.5 §5e — TERRAIN HEIGHTMAP PREVIEW
%             (procedural elevation grid in 2D + 3D editor views)
%% ========================================================================
%
%  Renders the procedural terrain heightmap that the sim engine uses
%  (trackbench.environment.generateTerrain + composeHeightmap), so what
%  the editor previews matches what runs produce.
%
%  COMPOSITION
%    1. generateTerrain(state.terrain.terrainType, bounds, scale) builds
%       the fallback layer — same procedural functions, same rng seed
%       (42), same hilltop clearing tweak as the sim.
%    2. If state.terrainRegions is non-empty, composeHeightmap stamps
%       each region's heightmap into its polygon (first-listed-wins
%       resolution rule, identical to runDetections/loadRunFile §9).
%    3. Result is cached, keyed on (terrain type, terrain scale, region
%       hash, scenario bounds). Recompute only when the key changes —
%       generation is ~50ms for a single terrain, ~50ms × N for N
%       regions, so caching matters for interactive responsiveness
%       (drag a waypoint → drawMap fires repeatedly).
%
%  RENDERING
%    2D: image() with truecolor RGB (MxNx3), bypassing the axes
%        colormap so this never fights with drawWaypointsByAltitude2D
%        which sets parula on the same axes. Hypsometric tint: deep
%        blue (water) → green (lowlands) → tan/brown (foothills) →
%        white (snow caps). YDir forced to 'normal' since image()
%        flips it by default.
%    3D: surf() with truecolor CData and FaceAlpha 0.7, so waypoints/
%        sensors above stay readable. EdgeColor='none' for a smooth
%        terrain look (the 200x200 grid would otherwise be lined out
%        by every cell edge).
%
%  BOUNDS
%    Mirrors loadRunFile/computeScenarioBounds: max extent of radar +
%    all targets' waypoints, padded 15%, floored at 130km. Matches the
%    sim exactly so editor preview equals run output.
%
%  GATED BY state.terrainOverlayCB
%    The existing 2D "Overlay on map" checkbox now controls both 2D
%    and 3D heightmap rendering. Default ON so users see terrain
%    immediately on first open; uncheck to disable if it ever lags.


function ok = drawTerrainHeightmap2D(ax, state)
%drawTerrainHeightmap2D  Render the procedural heightmap as a hypsometric
%                        background image. Returns true on success, false
%                        if generation failed (caller falls back to
%                        drawTerrainTint2D for a flat color tint).
%
%  v3.5 §5e: when terrain type is "none", returns true WITHOUT drawing.
%  Returning true (not false) is intentional — it prevents the caller
%  from falling back to drawTerrainTint2D, so the result is a truly
%  blank background. drawTerrainTint2D ALSO bails early on "none" as a
%  belt-and-braces measure.
    ok = false;
    if ~isempty(state.terrain) && strcmpi(state.terrain.terrainType, "none")
        ok = true;   % skip rendering, suppress tint fallback
        return;
    end
    try
        [Z, Xg, Yg, ~] = getOrComputeEditorHeightmap(state);
    catch ME
        warning('drawMap:heightmap2D', 'Heightmap generation failed: %s', ME.message);
        return;
    end
    if isempty(Z); return; end

    elev = -Z;   % NED → +Z up so peaks are positive
    rgb  = elevationToRgb(elev, hypsometricColormap());
    xData = [Xg(1, 1), Xg(1, end)];
    yData = [Yg(1, 1), Yg(end, 1)];
    image('Parent', ax, ...
        'XData', xData, 'YData', yData, ...
        'CData', rgb, ...
        'AlphaData', 0.85, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'terrainHeightmap2D');
    set(ax, 'YDir', 'normal');   % image() inverts Y by default
    ok = true;
end


function drawTerrainHeightmap3D(ax, state)
%drawTerrainHeightmap3D  Render the procedural heightmap as a 3D surface
%                        with hypsometric coloring and partial transparency
%                        so waypoints/sensors above stay readable.
%                        Silently no-ops on generation failure — the 3D
%                        view stays usable without terrain (matches
%                        pre-§5e behavior where 3D had no terrain at all).
%
%  v3.5 §5e: when terrain type is "none", silently skips rendering so
%  the 3D scene matches the 2D "clean canvas" default.
    if ~isempty(state.terrain) && strcmpi(state.terrain.terrainType, "none")
        return;
    end
    try
        [Z, Xg, Yg, ~] = getOrComputeEditorHeightmap(state);
    catch ME
        warning('drawMap:heightmap3D', 'Heightmap generation failed: %s', ME.message);
        return;
    end
    if isempty(Z); return; end

    elev = -Z;
    rgb  = elevationToRgb(elev, hypsometricColormap());
    surf(ax, Xg, Yg, elev, rgb, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.7, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'terrainHeightmap3D');
end


function [Z, Xg, Yg, bounds] = getOrComputeEditorHeightmap(state)
%getOrComputeEditorHeightmap  Cached heightmap retrieval. Recomputes only
%                             when terrain type, scale, regions, or
%                             scenario bounds change. Persistent variable
%                             survives across drawMap calls within the
%                             same MATLAB session — `clear classes` wipes
%                             it, which is the right behavior on edits.
    persistent cache;

    bounds = computeEditorScenBounds(state);
    key = computeHeightmapCacheKey(state, bounds);
    if ~isempty(cache) && strcmp(cache.key, key)
        Z  = cache.Z;
        Xg = cache.Xg;
        Yg = cache.Yg;
        return;
    end

    [tType, tScale] = currentTerrainSettings(state);
    [Z, ~, Xg, Yg] = trackbench.environment.generateTerrain(tType, bounds, tScale);

    % Stamp regions on top via the same merger the sim uses (5b §i).
    if isprop(state, 'terrainRegions') && ~isempty(state.terrainRegions)
        regions = editorRegionsToConfigRegions(state.terrainRegions);
        if ~isempty(regions)
            Z = trackbench.environment.composeHeightmap(Z, regions, Xg, Yg, bounds);
        end
    end

    cache = struct('key', key, 'Z', Z, 'Xg', Xg, 'Yg', Yg);
end


function key = computeHeightmapCacheKey(state, bounds)
%computeHeightmapCacheKey  Compact string fingerprint of every input that
%                          affects the heightmap. Cheap to compute per
%                          draw, exact to compare. Includes a polygon
%                          "hash" (sum of vertex coordinates + count) so
%                          a single dragged vertex invalidates correctly.
    [tType, tScale] = currentTerrainSettings(state);
    parts = {sprintf('FB|%s|%g', tType, tScale)};
    if isprop(state, 'terrainRegions')
        for k = 1:numel(state.terrainRegions)
            rec = state.terrainRegions(k);
            rType = 'water';
            rScale = 1.0;
            if ~isempty(rec.terrain)
                rType = char(rec.terrain.terrainType);
                if isprop(rec.terrain, 'terrainScale')
                    rScale = rec.terrain.terrainScale;
                end
            end
            polyN = size(rec.polygonXY, 1);
            polyHash = 0;
            if polyN > 0
                polyHash = sum(rec.polygonXY(:));
            end
            parts{end+1} = sprintf('R%d|%s|%g|%d|%g', ...
                k, rType, rScale, polyN, polyHash); %#ok<AGROW>
        end
    end
    parts{end+1} = sprintf('B|%g', bounds(1, 2));
    key = strjoin(parts, '||');
end


function [tType, tScale] = currentTerrainSettings(state)
%currentTerrainSettings  Pull terrain type + scale from state.terrain
%                        with safe defaults if state.terrain is empty.
%                        v3.5 §5e: defaults to 'none' (was 'water') so a
%                        missing terrain produces no heightmap rather
%                        than a flat sea-blue background.
    tType = 'none';
    tScale = 1.0;
    if ~isempty(state.terrain)
        tType = char(state.terrain.terrainType);
        if isprop(state.terrain, 'terrainScale')
            tScale = state.terrain.terrainScale;
        end
    end
end


function bounds = computeEditorScenBounds(state)
%computeEditorScenBounds  Mirror of loadRunFile/computeScenarioBounds.
%                         Uses radar position + every target's waypoints
%                         (NOT polygon vertices — matches the sim, where
%                         polygons can extend past bounds without issue).
%                         maxExtent × 1.15, floored at 130km.
    pts = [state.radarEastM, state.radarNorthM];
    if ~isempty(state.targets)
        for k = 1:numel(state.targets)
            wp = state.targets(k).waypoints;
            if ~isempty(wp)
                pts = [pts; wp(:, 1:2)]; %#ok<AGROW>
            end
        end
    end
    if isempty(pts)
        pts = [0, 0];
    end
    maxExtent = max(vecnorm(pts, 2, 2));
    halfSpan = max(maxExtent * 1.15, 130000);
    bounds = [-halfSpan, halfSpan; -halfSpan, halfSpan];
end


function regions = editorRegionsToConfigRegions(terrainRegions)
%editorRegionsToConfigRegions  Adapter from editor's TerrainRegionRecord
%                              array to the cell-array-of-structs format
%                              that composeHeightmap expects.
%
%  Each output element has:
%    .config_path  reference path string (informational, used in log)
%    .name         display name (informational, used in log)
%    .polygon_xy   Nx2 polygon vertices in NED (East, North) metres
%    .def          inner struct {terrain_type, terrain_scale}
%
%  Skips records with malformed polygons (<3 vertices) so composeHeightmap
%  never sees a degenerate region. Returns 1x0 cell when nothing valid.
    n = numel(terrainRegions);
    out = cell(1, n);
    keep = false(1, n);
    for k = 1:n
        rec = terrainRegions(k);
        if size(rec.polygonXY, 1) < 3
            continue;
        end
        s = struct();
        s.config_path = char(rec.configPath);
        s.name = char(rec.name);
        s.polygon_xy = rec.polygonXY;
        s.def = struct();
        if ~isempty(rec.terrain)
            s.def.terrain_type = char(rec.terrain.terrainType);
            s.def.terrain_scale = 1.0;
            if isprop(rec.terrain, 'terrainScale')
                s.def.terrain_scale = rec.terrain.terrainScale;
            end
        else
            s.def.terrain_type = 'none';
            s.def.terrain_scale = 1.0;
        end
        out{k} = s;
        keep(k) = true;
    end
    regions = out(keep);
end


function rgb = elevationToRgb(elev, cmap)
%elevationToRgb  Map an MxN elevation grid to MxNx3 truecolor via a
%                colormap. Bypasses the axes colormap entirely so the
%                heightmap render doesn't fight with drawWaypointsByAltitude
%                which sets parula on the same axes.
%
%  Uniform-elevation case (all-water z=0) → solid color from row 1 of
%  the colormap (deep water blue for the hypsometric ramp).
    elevMin = min(elev(:));
    elevMax = max(elev(:));
    [H, W] = size(elev);
    if (elevMax - elevMin) < 1
        idx = ones(H, W);
    else
        nrm = (elev - elevMin) / (elevMax - elevMin);
        idx = round(nrm * (size(cmap, 1) - 1)) + 1;
        idx = max(1, min(size(cmap, 1), idx));
    end
    rgb = zeros(H, W, 3);
    flatIdx = idx(:);
    rgb(:,:,1) = reshape(cmap(flatIdx, 1), H, W);
    rgb(:,:,2) = reshape(cmap(flatIdx, 2), H, W);
    rgb(:,:,3) = reshape(cmap(flatIdx, 3), H, W);
end


function cm = hypsometricColormap()
%hypsometricColormap  Standard topographic-map color ramp:
%                       deep blue (water) → light blue → light green →
%                       yellow-green → tan → brown → dark brown →
%                       rocky → white (snow caps). 256-entry pchip
%                       interpolation so adjacent elevation bins blend
%                       smoothly with no visible banding.
    keyColors = [
        0.20 0.40 0.65;   % 0%   — deep water
        0.35 0.60 0.80;   % 12%  — shallow water
        0.45 0.70 0.45;   % 25%  — lowland green
        0.65 0.75 0.40;   % 38%  — grassland
        0.80 0.70 0.40;   % 50%  — tan
        0.65 0.50 0.30;   % 62%  — brown earth
        0.55 0.40 0.30;   % 75%  — dark brown
        0.75 0.65 0.55;   % 87%  — rocky
        0.95 0.95 0.95];  % 100% — snow caps
    keyPts = linspace(0, 1, size(keyColors, 1));
    qPts = linspace(0, 1, 256);
    cm = interp1(keyPts, keyColors, qPts, 'pchip');
    cm = max(0, min(1, cm));
end


%% ========================================================================
%  v3.5 §5c.3b — REGION POLYGON RENDERING
%                (terrain regions + weather regions + in-progress draft)
%% ========================================================================
%
%  Called from drawMap2D between the terrain background tint and the
%  radar marker. Three layers, drawn in order:
%
%    1. Committed terrain regions — semi-transparent fills tinted by
%       terrain type (mountain=brown, water=blue, urban=grey, rural=
%       green, desert=tan), darkened ~30% relative to the existing bg
%       tint so they read against a same-type background.
%    2. Committed weather regions — semi-transparent blue fills (single
%       palette for all weather types, matches the storm timeline).
%    3. In-progress draft — dashed open polyline + vertex markers in
%       the active region's color. Most-recent vertex highlighted with
%       a green ring. Replaces the active region's committed rendering
%       during edit-in-place (Q1 = seeded from existing polygon).
%
%  Active region (DD-selected, env mode + regions sub-mode) gets a
%  thicker outline at LineWidth 2.0 (vs 1.2 for inactive) and small
%  vertex circles. Inactive committed regions render with no vertex
%  markers — just the filled patch.
%
%  3D mode skipped — same rationale as drawTerrainTint2D being skipped
%  in 3D (the 3D view has its own ground plane and editing happens in
%  2D anyway).
%
%  PickableParts='none' on every graphics child, matching the rest of
%  the file. Click handling for polygon-edit lives in onAxesClick.


function drawRegionPolygons(ax, state)
%drawRegionPolygons  Render every committed region + the in-progress
%                    draft (if any). No-op when there are no regions
%                    and no active edit.
    % --- Defensive guards (pre-5c snapshots may lack these props) ----
    hasTerrainRegions = isprop(state, 'terrainRegions') && ...
                        ~isempty(state.terrainRegions);
    hasWeatherRegions = isprop(state, 'weatherRegions') && ...
                        ~isempty(state.weatherRegions);
    polyEdit = isprop(state, 'polygonEditActive') && state.polygonEditActive;
    if ~hasTerrainRegions && ~hasWeatherRegions && ~polyEdit
        return;
    end

    % "Active region" only if the user is currently in the regions
    % sub-panel of environment mode — outside that context the regions
    % are still visible (they're part of the scenario) but no single
    % region is highlighted as the editing focus.
    inRegionsContext = (state.editMode == "environment") && ...
                       (state.envSubMode == "regions");

    % --- 1. Committed terrain regions ----------------------------------
    if hasTerrainRegions
        for k = 1:numel(state.terrainRegions)
            isActive = inRegionsContext && ...
                       (k == state.activeTerrainRegionIdx);
            % During an active polygon edit on THIS region, suppress the
            % committed render so the draft (which was seeded from the
            % committed polygon) takes its place. Otherwise the user
            % would see two stacked polygons that disagree as they edit.
            if isActive && polyEdit && state.polygonEditTarget == "terrain"
                continue;
            end
            drawCommittedRegion(ax, state.terrainRegions(k), 'terrain', isActive);
        end
    end

    % --- 2. Committed weather regions ----------------------------------
    if hasWeatherRegions
        for k = 1:numel(state.weatherRegions)
            isActive = inRegionsContext && ...
                       (k == state.activeWeatherRegionIdx);
            if isActive && polyEdit && state.polygonEditTarget == "weather"
                continue;
            end
            drawCommittedRegion(ax, state.weatherRegions(k), 'weather', isActive);
        end
    end

    % --- 3. In-progress polygon-edit draft -----------------------------
    if polyEdit
        drawDraftPolygon(ax, state);
    end

    % --- 4. v3.5 §5c.6 — Selection + hover overlays --------------------
    %  Drawn LAST so they sit on top of the active-region vertex circles.
    %  Only active in 2D (3D click hit-tests don't make sense, so 5c.6
    %  bails on 3D in onAxesClick + onMouseMove). drawSelectedRegionVertex
    %  guards on a valid selection internally; same for drawHoveredEdge.
    if isprop(state, 'selectedVertexIdx') && state.selectedVertexIdx > 0
        drawSelectedRegionVertex(ax, state);
    end
    if isprop(state, 'hoverVertexIdx') && state.hoverVertexIdx > 0 && ...
            ~(state.hoverRegionKind == state.selectedRegionKind && ...
              state.hoverRegionIdx  == state.selectedRegionIdx && ...
              state.hoverVertexIdx  == state.selectedVertexIdx)
        % Don't double-paint hover ON TOP of selection — selection's
        % ring is more prominent and "hover === selected" is the common
        % case right after a click, where the cursor is still on the
        % just-clicked vertex.
        drawHoveredRegionVertex(ax, state);
    end
    if isprop(state, 'hoverEdgeIdx') && state.hoverEdgeIdx > 0
        drawHoveredRegionEdge(ax, state);
    end
end


function drawCommittedRegion(ax, rec, kind, isActive)
%drawCommittedRegion  Render one TerrainRegionRecord or WeatherRegionRecord
%                     as a filled polygon with an outline. Active region
%                     gets a thicker outline + small vertex circles.
%                     Records with <3 vertices are skipped (they cannot
%                     enclose area — freshly-added regions before any
%                     polygon has been drawn fall here).
    poly = rec.polygonXY;
    if size(poly, 1) < 3
        return;
    end
    fillCol = regionFillColor(rec, kind);
    edgeCol = fillCol * 0.6;     % darker tone for a readable outline
    if isActive
        lw = 2.0;
    else
        lw = 1.2;
    end
    tagFill   = sprintf('%sRegionFill2D',   kind);
    tagVertex = sprintf('%sRegionVertex2D', kind);
    if isActive
        tagFill   = [tagFill,   'Active'];
        tagVertex = [tagVertex, 'Active'];
    end
    % Filled polygon — patch closes the polyline implicitly so we don't
    % need to repeat the first vertex at the end.
    patch(ax, poly(:,1), poly(:,2), fillCol, ...
        'FaceAlpha', 0.30, ...
        'EdgeColor', edgeCol, ...
        'LineWidth', lw, ...
        'LineStyle', '-', ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', tagFill);
    % Vertex markers only for the active region (would clutter
    % otherwise when several regions are visible).
    if isActive
        plot(ax, poly(:,1), poly(:,2), 'o', ...
            'MarkerSize', 6, ...
            'MarkerFaceColor', fillCol, ...
            'MarkerEdgeColor', [0.10 0.10 0.10], ...
            'LineWidth', 1.0, ...
            'PickableParts', 'none', ...
            'HitTest', 'off', ...
            'Tag', tagVertex);
    end
end


function drawDraftPolygon(ax, state)
%drawDraftPolygon  Render the in-progress polygon-edit draft.
%
%  Visual contract:
%    * Open dashed polyline connecting the appended vertices in the
%      active region's color (no closing edge until commit — the user
%      sees they have not yet "closed" the polygon).
%    * Vertex marker at every appended point (filled circle, region
%      color).
%    * Most-recent vertex highlighted with a green hollow ring — same
%      green as the existing waypoint selection halo — so the user
%      can see exactly where their last click landed.
    draft = state.polygonEditDraft;
    n = size(draft, 1);
    if n < 1
        return;
    end
    col = draftPolygonColor(state);

    % Dashed open polyline (only meaningful at ≥2 vertices).
    if n >= 2
        plot(ax, draft(:,1), draft(:,2), '--', ...
            'Color', col, ...
            'LineWidth', 1.6, ...
            'PickableParts', 'none', ...
            'HitTest', 'off', ...
            'Tag', 'polygonDraftLine2D');
    end

    % All vertex markers.
    plot(ax, draft(:,1), draft(:,2), 'o', ...
        'MarkerSize', 6, ...
        'MarkerFaceColor', col, ...
        'MarkerEdgeColor', [0.10 0.10 0.10], ...
        'LineWidth', 1.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'polygonDraftVertex2D');

    % Most-recent vertex highlight — green hollow ring on top.
    plot(ax, draft(end,1), draft(end,2), 'o', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor', 'none', ...
        'MarkerEdgeColor', [0.10 0.55 0.25], ...   % matches selectedHalo
        'LineWidth', 2.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'polygonDraftLatest2D');
end


function rgb = regionFillColor(rec, kind)
%regionFillColor  Per-record region fill color.
%
%  Terrain → reuse terrainTintColor() lookup, darkened by 30% so the
%  region reads against an underlying same-type background tint.
%  Weather → single canonical blue (matches the storm timeline
%  patch and the path line).
    switch kind
        case 'terrain'
            % Inner record's terrain field carries the terrainType.
            if isprop(rec, 'terrain')
                rgb = terrainTintColor(rec.terrain.terrainType) * 0.7;
            else
                rgb = [0.50 0.50 0.50];
            end
        case 'weather'
            rgb = [0.20 0.45 0.85];
        otherwise
            rgb = [0.50 0.50 0.50];
    end
end


function rgb = draftPolygonColor(state)
%draftPolygonColor  Color used for the in-progress polygon-edit draft.
%                    Matches the active region's color so the user
%                    sees what they're committing to. Defensive
%                    fallback to neutral grey if the active region
%                    somehow vanished during edit (should not happen —
%                    beginPolygonEdit guards on hasActive*Region, and
%                    the lockdown prevents Add/Del during edit).
    rgb = [0.50 0.50 0.50];
    target = state.polygonEditTarget;
    if target == "terrain" && state.hasActiveTerrainRegion()
        rec = state.terrainRegions(state.activeTerrainRegionIdx);
        rgb = regionFillColor(rec, 'terrain');
    elseif target == "weather" && state.hasActiveWeatherRegion()
        rec = state.weatherRegions(state.activeWeatherRegionIdx);
        rgb = regionFillColor(rec, 'weather');
    end
end


function drawSelectedRegionVertex(ax, state)
%drawSelectedRegionVertex  v3.5 §5c.6 — Highlight the currently-
%                          selected region vertex with a yellow ring.
%                          Sized larger than the per-vertex circles in
%                          drawCommittedRegion so it's visible on top.
    poly = regionPolygonByKind(state, state.selectedRegionKind, state.selectedRegionIdx);
    if isempty(poly); return; end
    vIdx = state.selectedVertexIdx;
    if vIdx < 1 || vIdx > size(poly, 1); return; end
    plot(ax, poly(vIdx, 1), poly(vIdx, 2), 'o', ...
        'MarkerSize', 14, ...
        'MarkerFaceColor', 'none', ...
        'MarkerEdgeColor', [1.00 0.85 0.20], ...   % bright amber
        'LineWidth', 2.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'regionVertexSelected');
end


function drawHoveredRegionVertex(ax, state)
%drawHoveredRegionVertex  v3.5 §5c.6 — Subtle highlight on the vertex
%                         the cursor is hovering over. Sized SMALLER
%                         than the selected-vertex ring so the two
%                         are distinguishable when next to each other.
    poly = regionPolygonByKind(state, state.hoverRegionKind, state.hoverRegionIdx);
    if isempty(poly); return; end
    vIdx = state.hoverVertexIdx;
    if vIdx < 1 || vIdx > size(poly, 1); return; end
    plot(ax, poly(vIdx, 1), poly(vIdx, 2), 'o', ...
        'MarkerSize', 11, ...
        'MarkerFaceColor', 'none', ...
        'MarkerEdgeColor', [0.30 0.85 1.00], ...   % cyan
        'LineWidth', 1.5, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'regionVertexHover');
end


function drawHoveredRegionEdge(ax, state)
%drawHoveredRegionEdge  v3.5 §5c.6 — Render a bright green line over
%                       the polygon edge currently under shift+hover,
%                       plus a "+" marker at the projection point
%                       telling the user where the new vertex would
%                       land if they click. Edge i connects poly(i,:)
%                       to poly(i+1,:), wrapping N → 1 for the closing
%                       edge — same convention as pickClosestEdge.
    poly = regionPolygonByKind(state, state.hoverRegionKind, state.hoverRegionIdx);
    if isempty(poly); return; end
    eIdx = state.hoverEdgeIdx;
    n = size(poly, 1);
    if eIdx < 1 || eIdx > n; return; end
    eNext = eIdx + 1;
    if eNext > n; eNext = 1; end
    p1 = poly(eIdx,  :);
    p2 = poly(eNext, :);
    plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], '-', ...
        'Color', [0.20 1.00 0.30], ...   % bright green
        'LineWidth', 3.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'regionEdgeHover');
    % Projection point: cursor-on-edge "insert here" marker. The cursor
    % isn't exactly on the edge but the projection IS — use the midpoint
    % of the edge for the marker if the cursor's projection isn't
    % readily available (drawMap doesn't receive cursor coords). A
    % follow-up could store the projection in state.hoverEdgeProj for
    % pixel-precise placement, but mid-edge is good enough for the
    % "insert vertex here" affordance.
    midX = 0.5 * (p1(1) + p2(1));
    midY = 0.5 * (p1(2) + p2(2));
    plot(ax, midX, midY, '+', ...
        'MarkerSize', 12, ...
        'MarkerEdgeColor', [0.20 1.00 0.30], ...
        'LineWidth', 2.0, ...
        'PickableParts', 'none', ...
        'HitTest', 'off', ...
        'Tag', 'regionEdgeHoverPlus');
end


function poly = regionPolygonByKind(state, kind, idx)
%regionPolygonByKind  v3.5 §5c.6 helper — fetch the polygonXY of a
%                     terrain or weather region by index. Returns []
%                     for out-of-range or unknown kind so callers can
%                     bail with isempty().
    poly = [];
    if kind == "terrain"
        if idx >= 1 && idx <= numel(state.terrainRegions)
            poly = state.terrainRegions(idx).polygonXY;
        end
    elseif kind == "weather"
        if idx >= 1 && idx <= numel(state.weatherRegions)
            poly = state.weatherRegions(idx).polygonXY;
        end
    end
end
