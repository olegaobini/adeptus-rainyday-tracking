function testPathEditor_shortcuts()
%testPathEditor_shortcuts  Programmatic tests for the M3 keyboard + mouse
%                          shortcuts pass (2026-04-16).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Covers the shortcuts landed between CHECK-IN 1 sign-off and M3.4:
%     - V key toggles 2D/3D
%     - Arrow keys nudge selected waypoint (100 m / 1000 m with Shift)
%     - PageUp/PageDown alters altitude (100 m / 1000 m with Shift,
%       clamped ≥ 0)
%     - Each shortcut keystroke = exactly one undo step
%     - Focus guard: keys are inert while a uieditfield has focus
%     - Mousewheel zoom math (pure compute via zoomAroundPoint)
%     - M2 regression: Delete/Escape/Ctrl+Z still fire under the new
%       dispatcher
%     - Pan state fields exist on EditorState and default false/empty
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_shortcuts
%
%  EXPECTED
%      n PASS / 0 FAIL. Tests that spawn a hidden uifigure clean it
%      up via onCleanup on every path (PASS, FAIL, or error mid-test).
%
%  See also: trackbench.editor.zoomAroundPoint, trackbench.editor.buildUI,
%            testPathEditor_M3

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_shortcuts (keyboard + mouse) ====\n');
    nPass = 0; nFail = 0;

    % ── 1. zoomAroundPoint: zero scroll = identity ──────────────────
    try
        xL = [-1000, 1000]; yL = [-500, 500];
        [nX, nY] = trackbench.editor.zoomAroundPoint(xL, yL, 0, 0, 0);
        assert(isequal(nX, xL), 'zero scroll must preserve XLim');
        assert(isequal(nY, yL), 'zero scroll must preserve YLim');
        pass('1. zoomAroundPoint: zero scroll is no-op');
        nPass = nPass + 1;
    catch ME
        fail('1. zoomAroundPoint zero scroll', ME); nFail = nFail + 1;
    end

    % ── 2. zoomAroundPoint: anchor-point invariant ─────────────────
    % The cursor-anchor property is the WHOLE point of this helper.
    % Whatever (cx, cy) is passed in must stay at the same
    % fractional position inside the new viewport.
    try
        xL = [0, 1000]; yL = [0, 500];
        cx = 200; cy = 100;
        fracX = (cx - xL(1)) / (xL(2) - xL(1));   % 0.2
        fracY = (cy - yL(1)) / (yL(2) - yL(1));   % 0.2
        [nX, nY] = trackbench.editor.zoomAroundPoint(xL, yL, cx, cy, 1);
        newFracX = (cx - nX(1)) / (nX(2) - nX(1));
        newFracY = (cy - nY(1)) / (nY(2) - nY(1));
        assert(abs(newFracX - fracX) < 1e-9, ...
            sprintf('X anchor drifted (%.6g → %.6g)', fracX, newFracX));
        assert(abs(newFracY - fracY) < 1e-9, ...
            sprintf('Y anchor drifted (%.6g → %.6g)', fracY, newFracY));
        pass('2. zoomAroundPoint: cursor anchor stays fixed');
        nPass = nPass + 1;
    catch ME
        fail('2. zoomAroundPoint anchor', ME); nFail = nFail + 1;
    end

    % ── 3. zoomAroundPoint: span math, zoom in and out ─────────────
    try
        xL = [-1000, 1000]; yL = [-1000, 1000];
        % Positive scroll = zoom out → span × 1.2
        [nX, ~] = trackbench.editor.zoomAroundPoint(xL, yL, 0, 0, 1);
        assert(abs((nX(2) - nX(1)) - 2400) < 1e-9, ...
            sprintf('zoom-out span expected 2400, got %g', nX(2) - nX(1)));
        % Negative scroll = zoom in → span × 1/1.2
        [nX, ~] = trackbench.editor.zoomAroundPoint(xL, yL, 0, 0, -1);
        assert(abs((nX(2) - nX(1)) - 2000/1.2) < 1e-9, ...
            sprintf('zoom-in span expected %.4f, got %.4f', ...
                2000/1.2, nX(2) - nX(1)));
        % Two positive ticks → factor = 1.44
        [nX, ~] = trackbench.editor.zoomAroundPoint(xL, yL, 0, 0, 2);
        assert(abs((nX(2) - nX(1)) - 2000*1.44) < 1e-9, ...
            'two-tick span compound zoom');
        pass('3. zoomAroundPoint: 1.2× per tick, compounds correctly');
        nPass = nPass + 1;
    catch ME
        fail('3. zoomAroundPoint span', ME); nFail = nFail + 1;
    end

    % ── 4. zoomAroundPoint: degenerate inputs are safe ─────────────
    try
        % Zero-width span: return input unchanged
        xL = [500, 500]; yL = [0, 100];
        [nX, nY] = trackbench.editor.zoomAroundPoint(xL, yL, 0, 0, 1);
        assert(isequal(nX, xL) && isequal(nY, yL), ...
            'degenerate span should not produce NaNs');
        % Non-finite input: return input unchanged
        xL = [-Inf, 1000];
        [nX, ~] = trackbench.editor.zoomAroundPoint(xL, [0 100], 0, 0, 1);
        assert(isequal(nX, xL), 'non-finite input should not crash');
        pass('4. zoomAroundPoint: degenerate inputs safe');
        nPass = nPass + 1;
    catch ME
        fail('4. zoomAroundPoint degenerate', ME); nFail = nFail + 1;
    end

    % ── 5. Pan state fields default correctly on EditorState ───────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.panActive == false, 'panActive defaults false');
        assert(isempty(state.panStartFigPt), 'panStartFigPt defaults empty');
        assert(isempty(state.panStartXLim),  'panStartXLim defaults empty');
        assert(isempty(state.panStartYLim),  'panStartYLim defaults empty');
        pass('5. EditorState: pan state fields default safe');
        nPass = nPass + 1;
    catch ME
        fail('5. pan state defaults', ME); nFail = nFail + 1;
    end

    % ── 6. V key toggles viewMode ──────────────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        assert(state.viewMode == "2d", 'start in 2D');
        fireKey(state, 'v');
        assert(state.viewMode == "3d", 'V should switch to 3D');
        fireKey(state, 'v');
        assert(state.viewMode == "2d", 'second V switches back to 2D');
        pass('6. V key: toggles 2D ↔ 3D via KeyPressFcn');
        nPass = nPass + 1;
    catch ME
        fail('6. V toggle', ME); nFail = nFail + 1;
    end

    % ── 7. Arrow nudge: 100 m delta + one undo entry ───────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.addWaypoint(0, 0);
        state.selectedIndex = 1;
        undoDepthBefore = numel(state.undoStack);
        fireKey(state, 'rightarrow');
        assert(state.waypoints(1, 1) == 100, ...
            sprintf('rightarrow should add +100 m to x, got x=%g', ...
                state.waypoints(1, 1)));
        assert(state.waypoints(1, 2) == 0, 'y must not change');
        assert(numel(state.undoStack) == undoDepthBefore + 1, ...
            'one keystroke must push exactly one undo entry');
        fireKey(state, 'uparrow');
        assert(state.waypoints(1, 2) == 100, 'uparrow adds +100 m to y');
        fireKey(state, 'downarrow');
        fireKey(state, 'downarrow');
        assert(state.waypoints(1, 2) == -100, 'two downarrows → y=-100');
        pass('7. Arrow nudge: 100 m / keystroke + 1 undo / keystroke');
        nPass = nPass + 1;
    catch ME
        fail('7. arrow nudge', ME); nFail = nFail + 1;
    end

    % ── 8. Shift+Arrow = 1000 m step ───────────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.addWaypoint(0, 0);
        state.selectedIndex = 1;
        fireKey(state, 'rightarrow', {'shift'});
        assert(state.waypoints(1, 1) == 1000, ...
            sprintf('Shift+right should add +1000 m, got %g', ...
                state.waypoints(1, 1)));
        fireKey(state, 'leftarrow', {'shift'});
        fireKey(state, 'leftarrow', {'shift'});
        assert(state.waypoints(1, 1) == -1000, ...
            'two Shift+lefts from 1000 → -1000');
        pass('8. Shift+Arrow: 1000 m step');
        nPass = nPass + 1;
    catch ME
        fail('8. shift+arrow step', ME); nFail = nFail + 1;
    end

    % ── 9. Arrow with no selection = no-op, no undo entry ──────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        % No waypoints, no selection
        undoDepthBefore = numel(state.undoStack);
        fireKey(state, 'rightarrow');
        assert(numel(state.undoStack) == undoDepthBefore, ...
            'arrow with no selection must not push undo');
        % Add a waypoint but leave selection cleared
        state.addWaypoint(500, 500);
        state.selectedIndex = 0;
        undoDepthBefore = numel(state.undoStack);
        fireKey(state, 'uparrow');
        assert(numel(state.undoStack) == undoDepthBefore, ...
            'arrow with selectedIndex=0 must not push undo');
        assert(state.waypoints(1, 2) == 500, ...
            'waypoint must not move when unselected');
        pass('9. Arrow with no selection: no-op, no undo entry');
        nPass = nPass + 1;
    catch ME
        fail('9. arrow no-selection', ME); nFail = nFail + 1;
    end

    % ── 10. PageUp/PageDown: ±100 m altitude with clamp at 0 ───────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.defaultAltitudeM = 150;
        state.addWaypoint(0, 0);
        state.selectedIndex = 1;
        assert(state.waypoints(1, 3) == 150, 'starts at default alt');
        fireKey(state, 'pagedown');
        assert(state.waypoints(1, 3) == 50, 'PageDown -100 → 50');
        fireKey(state, 'pagedown');
        assert(state.waypoints(1, 3) == 0, ...
            'PageDown from 50 → clamp at 0 (not -50)');
        fireKey(state, 'pagedown');
        assert(state.waypoints(1, 3) == 0, ...
            'repeat PageDown at 0 stays at 0');
        fireKey(state, 'pageup');
        assert(state.waypoints(1, 3) == 100, 'PageUp +100 → 100');
        pass('10. PageUp/PageDown: 100 m step + clamp ≥ 0');
        nPass = nPass + 1;
    catch ME
        fail('10. pageup/down alt', ME); nFail = nFail + 1;
    end

    % ── 11. Shift+PageUp/PageDown = 1000 m altitude step ───────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.defaultAltitudeM = 2000;
        state.addWaypoint(0, 0);
        state.selectedIndex = 1;
        fireKey(state, 'pageup', {'shift'});
        assert(state.waypoints(1, 3) == 3000, 'Shift+PgUp +1000 → 3000');
        fireKey(state, 'pagedown', {'shift'});
        fireKey(state, 'pagedown', {'shift'});
        fireKey(state, 'pagedown', {'shift'});
        fireKey(state, 'pagedown', {'shift'});
        % 3000 - 4*1000 = -1000 → clamps at 0
        assert(state.waypoints(1, 3) == 0, ...
            'Shift+PgDn multiple past zero clamps to 0');
        pass('11. Shift+PageUp/Down: 1000 m step + clamp');
        nPass = nPass + 1;
    catch ME
        fail('11. shift+page alt', ME); nFail = nFail + 1;
    end

    % ── 12. Focus guard: keys inert while editing a text field ─────
    % NOTE on simulation: focus() alone does NOT populate
    % fig.CurrentObject on a hidden uifigure (MATLAB warns and no-ops),
    % and CurrentObject is what the production editingTextField() helper
    % reads. Real interactive use sets CurrentObject via mouse click.
    % In the test we stub that by setting fig.CurrentObject directly —
    % this is the exact state the guard is meant to detect.
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.fig.CurrentObject = state.nameField;
        drawnow;
        viewBefore = state.viewMode;
        fireKey(state, 'v');
        assert(state.viewMode == viewBefore, ...
            'V must not toggle viewMode while nameField has focus');
        % But Escape (universal) should still fire — no focus guard on it
        state.addWaypoint(0, 0);
        state.selectedIndex = 1;
        fireKey(state, 'escape');
        assert(state.selectedIndex == 0, ...
            'Escape must still clear selection regardless of focus');
        % And once CurrentObject is back to something non-editable, V fires
        state.fig.CurrentObject = state.ax;  % not an EditField
        drawnow;
        viewBefore = state.viewMode;
        fireKey(state, 'v');
        assert(state.viewMode ~= viewBefore, ...
            'V should fire once focus leaves the edit field');
        pass('12. Focus guard: V inert in text field, Escape survives');
        nPass = nPass + 1;
    catch ME
        fail('12. focus guard', ME); nFail = nFail + 1;
    end

    % ── 13. M2 regression: Delete / Ctrl+Z still fire ──────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 500);
        assert(state.count() == 2);
        state.selectedIndex = 2;
        fireKey(state, 'delete');
        assert(state.count() == 1, 'Delete must remove selected wp');
        fireKey(state, 'z', {'control'});
        assert(state.count() == 2, 'Ctrl+Z must undo the delete');
        fireKey(state, 'y', {'control'});
        assert(state.count() == 1, 'Ctrl+Y must redo');
        pass('13. M2 regression: Del + Ctrl+Z + Ctrl+Y still fire');
        nPass = nPass + 1;
    catch ME
        fail('13. Del/Ctrl-Z regression', ME); nFail = nFail + 1;
    end

    % ── Summary ────────────────────────────────────────────────────
    fprintf('\n---- Result: %d PASS / %d FAIL ----\n\n', nPass, nFail);
    if nFail > 0
        error('testPathEditor_shortcuts:failures', '%d test(s) failed.', nFail);
    end
end


function fireKey(state, keyName, modifiers)
%fireKey  Simulate a figure-level key press by constructing a mock event
%         and calling the installed KeyPressFcn directly.
%
%  INPUT
%    state      EditorState with state.fig populated by buildUI
%    keyName    evt.Key string, e.g. 'v', 'leftarrow', 'pageup'
%    modifiers  (optional) cell of modifier strings, e.g. {'shift'}
%
%  The real evt is a matlab.ui.eventdata.KeyData, but onKeyPress only
%  reads evt.Key and evt.Modifier — a struct with those fields works
%  equivalently for test purposes.
    if nargin < 3 || isempty(modifiers); modifiers = {}; end
    evt = struct('Key', keyName, 'Character', '', 'Modifier', {modifiers});
    cb = state.fig.KeyPressFcn;
    cb(state.fig, evt);
    drawnow;
end


function pass(label)
    fprintf('  PASS  %s\n', label);
end


function fail(label, ME)
    fprintf('  FAIL  %s\n', label);
    fprintf('        %s: %s\n', ME.identifier, ME.message);
end


function safeDelete(h)
%safeDelete  Delete a handle if it is valid; swallow errors on test teardown.
    try
        if isgraphics(h)
            delete(h);
        end
    catch
        % nothing — teardown best-effort only
    end
end
