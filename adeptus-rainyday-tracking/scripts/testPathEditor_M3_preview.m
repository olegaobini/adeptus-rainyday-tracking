function results = testPathEditor_M3_preview()
%TESTPATHEDITOR_M3_PREVIEW  Automated tests for the M3.4 animation preview.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   Run from the project root:
%       >> cd adeptus-rainyday-tracking
%       >> addpath(genpath('src'));
%       >> results = scripts.testPathEditor_M3_preview()     % or cd scripts
%
%   Covers the four acceptance cases called out in the M3.4 spec
%   (PROGRESS_M2_FINAL_M3_START.md §7):
%       1. Timer is created when the preview opens and is fully cleaned
%          up when the preview closes (no leaks in timerfindall).
%       2. Interpolation math: a waypoint at a known time matches the
%          expected position exactly; a midpoint time matches the
%          analytic linear-interp value.
%       3. Speed-factor choice changes timer Period inversely (0.5x ->
%          longer period, 5x -> shorter period).
%       4. Zero-waypoint and one-waypoint inputs do NOT crash — the
%          preview renders a "need 2+ waypoints" message with no timer.
%
%   Returns a struct array with .name / .passed / .message fields so
%   runTestPlan.m (or a CI wrapper) can fail on non-green results.

    tests = { ...
        @test1_timerLifecycle,         'timer_created_and_cleaned_on_close'; ...
        @test2_interpolationMath,      'interpolation_linear_and_exact_at_waypoint'; ...
        @test3_speedFactorPeriod,      'speed_factor_inverse_on_timer_period'; ...
        @test4_zeroWaypointNoCrash,    'zero_and_one_waypoint_dont_crash'; ...
    };

    results = repmat(struct('name', '', 'passed', false, 'message', ''), 0, 1);

    for i = 1:size(tests, 1)
        fn   = tests{i, 1};
        name = tests{i, 2};
        fprintf('[M3.4] %-45s ', name);
        r = struct('name', name, 'passed', false, 'message', '');
        try
            fn();
            r.passed = true;
            r.message = 'OK';
            fprintf('PASS\n');
        catch ME
            r.passed = false;
            r.message = ME.message;
            fprintf('FAIL  (%s)\n', ME.message);
        end
        results(end+1) = r; %#ok<AGROW>

        % Paranoia: scrub any stray preview timers between tests so one
        % broken case doesn't cascade into the next.
        cleanupPreviewTimers();
        closePreviewFigures();
    end

    nPass = sum([results.passed]);
    fprintf('\n[M3.4] %d / %d passed\n', nPass, numel(results));
end

% =========================================================================
% Test 1 — timer lifecycle
% =========================================================================
function test1_timerLifecycle()
    cleanupPreviewTimers();

    % Simple 3-waypoint path over 10 seconds.
    wp = [0 0; 1000 0; 1000 1000];
    t  = [0; 5; 10];

    fig = trackbench.editor.previewWindow(wp, t, [0 0], []);
    assert(isgraphics(fig), 'previewWindow did not return a valid figure.');

    % Exactly one preview timer should exist.
    tmrs = timerfindall('Tag', 'RainyDayPathPreviewTimer');
    assert(numel(tmrs) == 1, ...
        'Expected 1 preview timer after open, got %d.', numel(tmrs));

    % Now close the preview and confirm the timer is gone.
    close(fig);
    drawnow;  % let CloseRequestFcn run
    tmrs = timerfindall('Tag', 'RainyDayPathPreviewTimer');
    assert(isempty(tmrs), ...
        'Preview timer leaked after close: %d still alive.', numel(tmrs));

    % Figure itself should also be gone.
    assert(~isgraphics(fig), 'Preview figure still valid after close.');
end

% =========================================================================
% Test 2 — interpolation math
% =========================================================================
function test2_interpolationMath()
    wp = [0 0; 1000 0; 1000 2000];    % 3-waypoint L-shape
    t  = [0; 10; 30];

    % (a) Query exactly at waypoint times -> waypoint positions.
    for k = 1:numel(t)
        got = trackbench.editor.interpPos(wp, t, t(k));
        assert(isequal(size(got), [1 2]), 'interpPos returned wrong shape.');
        assertClose(got, wp(k,:), 1e-9, sprintf('waypoint %d exact match', k));
    end

    % (b) Midpoint of segment 1 (t=5) -> halfway between (0,0) and (1000,0).
    got = trackbench.editor.interpPos(wp, t, 5);
    assertClose(got, [500 0], 1e-9, 'segment-1 midpoint');

    % (c) 1/4 into segment 2 (t = 10 + 0.25*20 = 15).
    got = trackbench.editor.interpPos(wp, t, 15);
    assertClose(got, [1000 500], 1e-9, 'segment-2 quarter point');

    % (d) Query below time_s(1) and above time_s(end) -> clamped.
    assertClose(trackbench.editor.interpPos(wp, t, -100), wp(1,:),   1e-9, 'clamp low');
    assertClose(trackbench.editor.interpPos(wp, t, 9999), wp(end,:), 1e-9, 'clamp high');
end

% =========================================================================
% Test 3 — speed-factor inverse effect on timer period
% =========================================================================
function test3_speedFactorPeriod()
    cleanupPreviewTimers();

    wp = [0 0; 1000 0; 1000 1000];
    t  = [0; 5; 10];

    fig = trackbench.editor.previewWindow(wp, t, [0 0], []);
    cleaner = onCleanup(@() safeCloseFig(fig));

    % Locate the timer and the speed dropdown.
    tmr = getappdata(fig, 'previewTimer');
    assert(isa(tmr, 'timer') && isvalid(tmr), 'No timer on preview figure.');
    basePeriod = tmr.Period;
    assert(abs(basePeriod - 0.05) < 1e-6, ...
        'Unexpected base period: %g (expected 0.05).', basePeriod);

    dd = findobj(fig, 'Type', 'uidropdown');
    assert(~isempty(dd), 'Could not find speed dropdown in preview.');
    dd = dd(1);

    % 2x -> half the base period
    setDropdown(dd, 2);
    assertClose(tmr.Period, basePeriod / 2, 1e-6, '2x period');

    % 0.5x -> double the base period
    setDropdown(dd, 0.5);
    assertClose(tmr.Period, basePeriod / 0.5, 1e-6, '0.5x period');

    % 5x -> 1/5 base, but floored at 0.01 s.  basePeriod/5 = 0.01 exactly,
    % so either value is acceptable.
    setDropdown(dd, 5);
    assert(tmr.Period <= basePeriod / 5 + 1e-6 && tmr.Period >= 0.01 - 1e-9, ...
        '5x period out of bounds: got %g.', tmr.Period);

    % 1x -> back to base
    setDropdown(dd, 1);
    assertClose(tmr.Period, basePeriod, 1e-6, '1x period');
end

% =========================================================================
% Test 4 — zero / one waypoint no-crash
% =========================================================================
function test4_zeroWaypointNoCrash()
    % (a) Zero waypoints.
    fig = trackbench.editor.previewWindow(zeros(0,2), zeros(0,1), [0 0], []);
    assert(isgraphics(fig), 'Zero-waypoint preview did not open.');
    lbls = findobj(fig, 'Type', 'uilabel');
    assert(~isempty(lbls), 'Zero-waypoint preview has no message label.');
    txt = strjoin(arrayfun(@(l) l.Text, lbls, 'UniformOutput', false), ' ');
    assert(contains(lower(txt), 'at least 2 waypoints'), ...
        'Zero-waypoint message text is missing: got "%s".', txt);
    % No timer should have been created.
    tmrs = timerfindall('Tag', 'RainyDayPathPreviewTimer');
    assert(isempty(tmrs), ...
        'Zero-waypoint preview created a timer (%d found).', numel(tmrs));
    close(fig); drawnow;

    % (b) One waypoint (still not animatable).
    fig = trackbench.editor.previewWindow([500 500], 0, [0 0], []);
    assert(isgraphics(fig), 'Single-waypoint preview did not open.');
    tmrs = timerfindall('Tag', 'RainyDayPathPreviewTimer');
    assert(isempty(tmrs), ...
        'Single-waypoint preview created a timer (%d found).', numel(tmrs));
    close(fig); drawnow;
end

% =========================================================================
% Helpers
% =========================================================================
function assertClose(a, b, tol, label)
    if ~isequal(size(a), size(b))
        error('assertClose:shape', ...
            '%s: shape mismatch (%s vs %s).', label, mat2str(size(a)), mat2str(size(b)));
    end
    d = max(abs(a(:) - b(:)));
    if d > tol
        error('assertClose:value', ...
            '%s: max abs diff %.3g > tol %.3g (got %s, want %s).', ...
            label, d, tol, mat2str(a,6), mat2str(b,6));
    end
end

function setDropdown(dd, value)
    % Programmatically change a uidropdown Value and fire its
    % ValueChangedFcn so the preview's speed handler runs.
    dd.Value = value;
    if ~isempty(dd.ValueChangedFcn)
        evt = struct('Value', value, 'PreviousValue', value, ...
                     'Source', dd, 'EventName', 'ValueChanged');
        try
            feval(dd.ValueChangedFcn, dd, evt);
        catch
            % Some MATLAB versions pass (src, evt); fall back.
            feval(dd.ValueChangedFcn, dd, []);
        end
    end
    drawnow;
end

function cleanupPreviewTimers()
    tmrs = timerfindall('Tag', 'RainyDayPathPreviewTimer');
    for k = 1:numel(tmrs)
        try, stop(tmrs(k)); catch, end  %#ok<CTCH>
        try, delete(tmrs(k)); catch, end %#ok<CTCH>
    end
end

function closePreviewFigures()
    figs = findall(0, 'Type', 'figure', 'Name', 'Path Preview (M3.4)');
    for k = 1:numel(figs)
        try, close(figs(k)); catch, end  %#ok<CTCH>
    end
    drawnow;
end

function safeCloseFig(fig)
    try
        if isgraphics(fig)
            close(fig); drawnow;
        end
    catch
    end
end
