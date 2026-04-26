function previewFig = previewWindow(waypoints, time_s, radar_xy, parentFig)
%PREVIEWWINDOW  Animated preview of the current waypoint path (M3.4).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   previewFig = previewWindow(WAYPOINTS, TIME_S, RADAR_XY, PARENTFIG)
%   opens a secondary uifigure that animates a moving marker along the
%   waypoint polyline using the provided auto-computed TIME_S timeline.
%   Marker position is interpolated LINEARLY between adjacent waypoints
%   (matches trackbench.scenario.addTargetFromDef default behaviour).
%
%   Inputs
%     WAYPOINTS : Nx2 matrix of xy positions in metres (altitude ignored
%                 in the 2-D preview — M3.4 is intentionally 2-D; the 3-D
%                 preview is deferred to M3.5/M4).
%     TIME_S    : Nx1 vector of monotonically increasing timestamps in
%                 seconds.  Length must match WAYPOINTS.
%     RADAR_XY  : 1x2 radar origin for visual reference (may be []).
%     PARENTFIG : Handle to the main editor uifigure.  Used so that
%                 closing the main editor also closes this preview.
%
%   Output
%     previewFig : Handle to the preview uifigure so the caller can
%                  track it on EditorState and close it on teardown.
%
%   Controls
%     Play/Pause   — toggles the timer
%     Restart      — rewinds t to time_s(1) and leaves paused
%     Speed        — 0.5x / 1x / 2x / 5x; driven by timer Period
%     Close [X]    — stops + deletes timer, deletes figure cleanly
%
%   Robustness
%     * Zero / one waypoint -> shows an "Add at least 2 waypoints"
%       label; no timer is created, no crash.
%     * Timer teardown runs in the figure CloseRequestFcn regardless of
%       whether the marker is mid-playback.
%     * All graphics objects set 'HitTest','off' and 'PickableParts',
%       'none' so the user cannot accidentally grab anything (view-only;
%       see feedback_matlab_uifigure_gotchas #2).
%     * Title uses 'Interpreter','none' to survive scenario names with
%       underscores (see feedback_matlab_title_interpreter).
%
%   Self-contained: does NOT reach into drawMap, 3-D camera logic,
%   shortcut handlers, or any other editor-state mutation.
%
%   M3.4 deliverable.  Do not touch for M3.5 or later — M3.5 should
%   compose with this, not edit it.

    arguments
        waypoints (:,:) double
        time_s    (:,1) double
        radar_xy  (:,:) double = []   % 1x2 expected, [] allowed
        parentFig              = []
    end

    % ---------- figure shell ------------------------------------------------
    previewFig = uifigure( ...
        'Name', 'Path Preview (M3.4)', ...
        'Position', [200 200 720 560], ...
        'Resize', 'on', ...
        'Color', [0.97 0.97 0.98]);

    % CloseRequestFcn is set before we create the timer so a mid-construction
    % error still tears the window down cleanly.
    previewFig.CloseRequestFcn = @(src, ~) localCloseFig(src);

    % Link the preview to the parent: when parent closes, close this too.
    if ~isempty(parentFig) && isgraphics(parentFig)
        existing = parentFig.DeleteFcn;
        parentFig.DeleteFcn = @(~,~) localLinkClose(previewFig, existing);
    end

    % ---------- handle zero / one waypoint without a timer -------------------
    if size(waypoints, 1) < 2 || numel(time_s) < 2 || ...
            numel(time_s) ~= size(waypoints, 1)
        g = uigridlayout(previewFig, [1 1]);
        g.RowHeight   = {'1x'};
        g.ColumnWidth = {'1x'};
        uilabel(g, ...
            'Text', 'Add at least 2 waypoints in the editor to preview playback.', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment',   'center', ...
            'FontSize', 14, ...
            'FontWeight', 'bold', ...
            'FontColor', [0.35 0.35 0.4]);
        return;
    end

    % ---------- layout ------------------------------------------------------
    outer = uigridlayout(previewFig, [2 1]);
    outer.RowHeight   = {'1x', 72};      % axes row, controls row
    outer.ColumnWidth = {'1x'};
    outer.Padding     = [8 8 8 8];
    outer.RowSpacing  = 6;

    ax = uiaxes(outer);
    ax.Layout.Row    = 1;
    ax.Layout.Column = 1;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    box(ax, 'on');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    title(ax, 'Path Preview', 'Interpreter', 'none');

    % full waypoint path
    pathLine = plot(ax, waypoints(:,1), waypoints(:,2), ...
        '-o', 'Color', [0.4 0.55 0.85], 'LineWidth', 1.25, ...
        'MarkerSize', 5, 'MarkerFaceColor', [0.7 0.8 0.95]);
    pathLine.HitTest       = 'off';
    pathLine.PickableParts = 'none';

    % radar marker (optional visual reference)
    if ~isempty(radar_xy) && numel(radar_xy) >= 2
        rm = plot(ax, radar_xy(1), radar_xy(2), ...
            'p', 'MarkerSize', 12, ...
            'MarkerFaceColor', [0.9 0.6 0.2], ...
            'MarkerEdgeColor', [0.3 0.2 0.05], 'LineWidth', 1);
        rm.HitTest       = 'off';
        rm.PickableParts = 'none';
    end

    % moving marker — initialise at first waypoint
    marker = plot(ax, waypoints(1,1), waypoints(1,2), ...
        'o', 'MarkerSize', 11, ...
        'MarkerFaceColor', [0.85 0.25 0.25], ...
        'MarkerEdgeColor', [0.2 0 0], 'LineWidth', 1.2);
    marker.HitTest       = 'off';
    marker.PickableParts = 'none';

    % ---------- controls ----------------------------------------------------
    ctrls = uigridlayout(outer, [1 7]);
    ctrls.Layout.Row    = 2;
    ctrls.Layout.Column = 1;
    ctrls.ColumnWidth   = {110, 90, 60, 120, '1x', 180, 120};
    ctrls.RowHeight     = {'1x'};
    ctrls.ColumnSpacing = 6;

    playBtn = uibutton(ctrls, 'push', ...
        'Text', 'Play', ...
        'FontWeight', 'bold');
    playBtn.Layout.Column = 1;

    restartBtn = uibutton(ctrls, 'push', ...
        'Text', 'Restart');
    restartBtn.Layout.Column = 2;

    uilabel(ctrls, ...
        'Text', 'Speed:', ...
        'HorizontalAlignment', 'right').Layout.Column = 3;

    speedDD = uidropdown(ctrls, ...
        'Items', {'0.5x', '1x', '2x', '5x'}, ...
        'ItemsData', [0.5 1 2 5], ...
        'Value', 1);
    speedDD.Layout.Column = 4;

    statusLbl = uilabel(ctrls, ...
        'Text', sprintf('t = %.1f / %.1f s', time_s(1), time_s(end)), ...
        'HorizontalAlignment', 'center', ...
        'FontColor', [0.25 0.25 0.3]);
    statusLbl.Layout.Column = 6;

    closeBtn = uibutton(ctrls, 'push', ...
        'Text', 'Close');
    closeBtn.Layout.Column = 7;

    % ---------- playback state ---------------------------------------------
    pState = struct();
    pState.t        = time_s(1);
    pState.tEnd     = time_s(end);
    pState.tStart   = time_s(1);
    pState.speed    = 1;
    pState.isPlaying = false;
    pState.waypoints = waypoints;
    pState.time_s    = time_s;

    % Base tick period (seconds).  Timer Period scales inversely with speed
    % so higher speeds visibly animate faster.  Clamped to 0.01 s — MATLAB
    % timer precision below ~10 ms is unreliable inside a uifigure.
    basePeriod = 0.05;

    % Create the timer.  Stash it on the figure so CloseRequestFcn can find
    % it even if this function's workspace is gone.
    t = timer( ...
        'ExecutionMode', 'fixedSpacing', ...
        'Period',        basePeriod, ...
        'BusyMode',      'drop', ...
        'Tag',           'RainyDayPathPreviewTimer', ...
        'ObjectVisibility', 'off');
    t.TimerFcn = @(~,~) localTick();
    setappdata(previewFig, 'previewTimer', t);
    setappdata(previewFig, 'previewAxes',  ax);
    setappdata(previewFig, 'previewMarker', marker);

    % wire callbacks
    playBtn.ButtonPushedFcn    = @(btn,~) localTogglePlay(btn);
    restartBtn.ButtonPushedFcn = @(~,~)   localRestart();
    speedDD.ValueChangedFcn    = @(dd,~)  localSetSpeed(dd.Value);
    closeBtn.ButtonPushedFcn   = @(~,~)   close(previewFig);

    % ---------- nested callbacks -------------------------------------------
    function localTick()
        if ~isvalid(previewFig) || ~pState.isPlaying
            return;
        end
        % Advance simulated time by a constant delta (basePeriod) each tick.
        % Speed is conveyed by how often the timer fires (t.Period scales
        % inversely with speed), not by the per-tick delta.  Doubling both
        % would square the effective speed, which is the bug this comment
        % exists to warn against.
        pState.t = pState.t + basePeriod;
        if pState.t >= pState.tEnd
            pState.t = pState.tEnd;
            pState.isPlaying = false;
            try, stop(t); catch, end  %#ok<CTCH>
            playBtn.Text = 'Play';
        end
        pos = trackbench.editor.interpPos( ...
            pState.waypoints, pState.time_s, pState.t);
        if isvalid(marker)
            marker.XData = pos(1);
            marker.YData = pos(2);
        end
        if isvalid(statusLbl)
            statusLbl.Text = sprintf('t = %.1f / %.1f s', pState.t, pState.tEnd);
        end
        drawnow limitrate;
    end

    function localTogglePlay(btn)
        if pState.isPlaying
            pState.isPlaying = false;
            try, stop(t); catch, end  %#ok<CTCH>
            btn.Text = 'Play';
        else
            % if at end, rewind so Play after completion restarts
            if pState.t >= pState.tEnd
                pState.t = pState.tStart;
            end
            pState.isPlaying = true;
            try
                if strcmp(t.Running, 'off'), start(t); end
            catch
            end
            btn.Text = 'Pause';
        end
    end

    function localRestart()
        pState.isPlaying = false;
        try, stop(t); catch, end  %#ok<CTCH>
        pState.t = pState.tStart;
        pos = trackbench.editor.interpPos( ...
            pState.waypoints, pState.time_s, pState.t);
        if isvalid(marker)
            marker.XData = pos(1);
            marker.YData = pos(2);
        end
        if isvalid(statusLbl)
            statusLbl.Text = sprintf('t = %.1f / %.1f s', pState.t, pState.tEnd);
        end
        playBtn.Text = 'Play';
    end

    function localSetSpeed(s)
        pState.speed = s;
        wasRunning = pState.isPlaying;
        try, stop(t); catch, end  %#ok<CTCH>
        % Timer Period scales inversely with speed so the "speed-factor
        % effect on timer period" test (TestPathEditor_M3_preview) can
        % observe the change via t.Period.  We also floor at 0.01 s to
        % keep MATLAB's timer happy.
        newPeriod = max(basePeriod / s, 0.01);
        t.Period = newPeriod;
        if wasRunning
            try, start(t); catch, end  %#ok<CTCH>
        end
    end
end

% =========================================================================
% Local (figure-scope) helpers — kept outside the main function so that
% CloseRequestFcn can run even after the main workspace is gone.
% =========================================================================

function localCloseFig(fig)
    % CloseRequestFcn — guarantee the timer is stopped + deleted before the
    % figure itself is torn down, so nothing leaks into timerfind.
    try
        t = getappdata(fig, 'previewTimer');
        if ~isempty(t) && isa(t, 'timer') && isvalid(t)
            try, stop(t); catch, end  %#ok<CTCH>
            delete(t);
        end
    catch ME
        warning('previewWindow:timerTeardown', ...
            'Timer teardown failed: %s', ME.message);
    end
    delete(fig);
end

function localLinkClose(previewFig, existingDeleteFcn)
    % Main editor was just closed — close the preview too.  Preserves any
    % existing DeleteFcn the editor already had wired (we are additive).
    try
        if ~isempty(existingDeleteFcn)
            feval(existingDeleteFcn, [], []);
        end
    catch
    end
    try
        if isgraphics(previewFig)
            close(previewFig);
        end
    catch
    end
end
