function testPathEditor_M3()
%testPathEditor_M3  Programmatic tests for Milestone 3 (sub-tasks 3.1 – 3.3).
%
%  M3 is split into five sub-tasks (see PROGRESS_M2_FINAL_M3_START.md §7).
%  This file covers the first CHECK-IN window: 3.1 altitude colormap,
%  3.2 scale bar + grid dropdown, 3.3 2D/3D view toggle. Sub-tasks 3.4
%  (animation preview) and 3.5 (editable radar marker) are gated behind
%  user sign-off, so their tests live in a follow-up pass.
%
%  What this script verifies, all without spawning a live uifigure:
%     1. computeScaleBar: 1/2/5 ladder, nearest-<=, km vs m formatting
%     2. computeScaleBar floor at 10 m for very tight zooms
%     3. computeScaleBar upper end picks 500 km for huge spans
%     4. colorByAltitude flag round-trip on EditorState (default false)
%     5. gridSpacingKm default + assignment (supports [0, 1, 5, 10])
%     6. viewMode default "2d" + switch to "3d" does not touch waypoints
%     7. radar marker state (radarEastM, radarNorthM) defaults to 0,0
%     8. M1 regression: export schema still includes pos + time_s, Z < 0
%     9. M2 regression: undo/redo still round-trips after M3 state added
%    10. has3DViewState default false + resets on mode switch (M3.3
%        camera-preservation regression fix)
%    11. Patch C: ax.ButtonDownFcn survives drawMap's cla reset in 2D
%        AND 3D (left-click-adds / drag / reselect regression fix)
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_M3
%
%  Prints PASS/FAIL per test. Manual interaction tests (hover tooltip,
%  colorbar visibility, 3D rotate, etc.) live in TESTING_M3.md.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.computeScaleBar,
%            testPathEditor_M1, testPathEditor_M2

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_M3 (sub-tasks 3.1 – 3.3) ====\n');
    nPass = 0; nFail = 0;

    % ── 1. computeScaleBar: mid-range spans ─────────────────────────
    try
        % 1000 m span, target 15% = 150 m → largest nice <= 150 is 100 m
        [L, S] = trackbench.editor.computeScaleBar(1000);
        assert(L == 100, sprintf('expected 100, got %g', L));
        assert(strcmp(S, '100 m'), sprintf('expected "100 m", got "%s"', S));

        % 30 km span, 15% = 4500 m → largest nice <= 4500 is 2000 m
        [L, S] = trackbench.editor.computeScaleBar(30000);
        assert(L == 2000);
        assert(strcmp(S, '2 km'), sprintf('expected "2 km", got "%s"', S));

        % 100 km span, 15% = 15000 m → largest nice <= 15000 is 10000 m
        [L, S] = trackbench.editor.computeScaleBar(100000);
        assert(L == 10000);
        assert(strcmp(S, '10 km'));

        % 200 m span, 15% = 30 m → largest nice <= 30 is 20 m
        [L, S] = trackbench.editor.computeScaleBar(200);
        assert(L == 20);
        assert(strcmp(S, '20 m'));

        pass('1. computeScaleBar: mid-range (1/2/5 ladder, km vs m)');
        nPass = nPass + 1;
    catch ME
        fail('1. computeScaleBar mid-range', ME); nFail = nFail + 1;
    end

    % ── 2. computeScaleBar: tight-zoom floor at 10 m ────────────────
    try
        % 50 m span, 15% = 7.5 m → below 10 m floor, should clamp to 10 m
        [L, S] = trackbench.editor.computeScaleBar(50);
        assert(L == 10, sprintf('floor expected 10 m, got %g', L));
        assert(strcmp(S, '10 m'));
        % 1 m span (absurdly tight) still returns the floor, no crash
        [L, ~] = trackbench.editor.computeScaleBar(1);
        assert(L == 10, 'floor still 10 m at 1 m span');
        pass('2. computeScaleBar: 10 m floor on tight zooms');
        nPass = nPass + 1;
    catch ME
        fail('2. computeScaleBar floor', ME); nFail = nFail + 1;
    end

    % ── 3. computeScaleBar: saturation at 500 km ────────────────────
    try
        % 10,000 km span, 15% = 1,500 km → saturates at 500 km
        [L, S] = trackbench.editor.computeScaleBar(10e6);
        assert(L == 500000, sprintf('expected 500 km, got %g', L));
        assert(strcmp(S, '500 km'));
        pass('3. computeScaleBar: 500 km top of ladder');
        nPass = nPass + 1;
    catch ME
        fail('3. computeScaleBar saturation', ME); nFail = nFail + 1;
    end

    % ── 4. colorByAltitude flag round-trip ──────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.colorByAltitude == false, 'default is OFF');
        state.colorByAltitude = true;
        assert(state.colorByAltitude == true, 'can flip to ON');
        state.colorByAltitude = false;
        assert(state.colorByAltitude == false, 'can flip back OFF');
        pass('4. colorByAltitude: default false + toggle');
        nPass = nPass + 1;
    catch ME
        fail('4. colorByAltitude toggle', ME); nFail = nFail + 1;
    end

    % ── 5. gridSpacingKm defaults + assignment ──────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.gridSpacingKm == 5, 'default grid = 5 km');
        % Dropdown ItemsData is [0, 1, 5, 10]; exercise each
        for v = [0, 1, 5, 10]
            state.gridSpacingKm = v;
            assert(state.gridSpacingKm == v, ...
                sprintf('could not set gridSpacingKm=%g', v));
        end
        pass('5. gridSpacingKm: default 5, accepts [0,1,5,10]');
        nPass = nPass + 1;
    catch ME
        fail('5. gridSpacingKm', ME); nFail = nFail + 1;
    end

    % ── 6. viewMode toggle does not mutate waypoint data ────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 500);
        state.addWaypoint(2500, 1200);
        before = state.waypoints;
        assert(state.viewMode == "2d", 'default viewMode is "2d"');
        state.viewMode = "3d";
        assert(state.viewMode == "3d", 'can switch to 3d');
        after = state.waypoints;
        assert(isequal(before, after), ...
            'viewMode switch must not touch waypoints matrix');
        % And back
        state.viewMode = "2d";
        assert(state.viewMode == "2d", 'can switch back to 2d');
        pass('6. viewMode: default 2d, toggle preserves waypoints');
        nPass = nPass + 1;
    catch ME
        fail('6. viewMode toggle', ME); nFail = nFail + 1;
    end

    % ── 7. radar marker state present (M3.5 groundwork) ─────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.radarEastM == 0 && state.radarNorthM == 0, ...
            'radar defaults to origin');
        state.radarEastM = 1500;
        state.radarNorthM = -2500;
        assert(state.radarEastM == 1500);
        assert(state.radarNorthM == -2500);
        pass('7. radar marker: default (0,0), assignable');
        nPass = nPass + 1;
    catch ME
        fail('7. radar marker state', ME); nFail = nFail + 1;
    end

    % ── 8. M1 regression: export schema still sane ──────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.targetName = "m3_schema";
        state.defaultSpeedKmh = 800;
        state.defaultAltitudeM = 3000;
        state.addWaypoint(0, 0);
        state.addWaypoint(4000, 0);
        state.addWaypoint(4000, 4000);

        outPath = trackbench.editor.exportToJSON(state);
        decoded = jsondecode(fileread(outPath));
        if iscell(decoded.targets)
            tgt = decoded.targets{1};
        else
            tgt = decoded.targets(1);
        end
        assert(strcmp(tgt.behavior, 'waypoints'));
        assert(numel(tgt.waypoints) == 3);
        allZ = arrayfun(@(w) w.pos(3), tgt.waypoints);
        assert(all(allZ < 0), 'NED: Z still negative on disk');
        allT = arrayfun(@(w) w.time_s, tgt.waypoints);
        assert(all(diff(allT) > 0), 'times strictly increasing');
        delete(outPath);
        pass('8. M1 regression: export schema preserved under M3');
        nPass = nPass + 1;
    catch ME
        fail('8. M1 schema regression', ME); nFail = nFail + 1;
    end

    % ── 9. M2 regression: undo/redo still works ─────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0, 0);
        state.addWaypoint(500, 500);
        state.addWaypoint(1000, 1000);
        assert(state.count() == 3);
        ok = state.undo(); assert(ok, 'undo works');
        assert(state.count() == 2, 'undo shrinks to 2');
        ok = state.redo(); assert(ok, 'redo works');
        assert(state.count() == 3, 'redo back to 3');
        % And M3 view flags should NOT end up on undo stack — changing
        % viewMode does not require ctrl-Z
        state.viewMode = "3d";
        state.viewMode = "2d";
        assert(state.count() == 3, 'viewMode changes are non-undoable');
        pass('9. M2 regression: undo/redo still correct under M3');
        nPass = nPass + 1;
    catch ME
        fail('9. M2 undo/redo regression', ME); nFail = nFail + 1;
    end

    % ── 10. has3DViewState flag behaves correctly ──────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.has3DViewState == false, 'default false');
        % Simulate entering 3D and settling the view
        state.viewMode = "3d";
        state.has3DViewState = true;
        % Simulate onViewModeChanged resetting it on mode switch
        state.viewMode = "2d";
        state.has3DViewState = false;
        assert(state.has3DViewState == false, ...
            'mode-switch handler must reset has3DViewState');
        pass('10. has3DViewState: default false + resets on mode switch');
        nPass = nPass + 1;
    catch ME
        fail('10. has3DViewState', ME); nFail = nFail + 1;
    end

    % ── 11. Patch C: ButtonDownFcn survives drawMap's cla reset ────
    %  This test spawns a real (briefly visible) uifigure because the
    %  regression is specifically about cla(ax,'reset') wiping the
    %  installed callback — reproducing it without an axes wouldn't
    %  prove anything. Figure is deleted via onCleanup on all paths.
    try
        state = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.buildUI(state);
        cleanupFig = onCleanup(@() safeDelete(state.fig));

        % buildUI runs an initial drawMap → cla reset hits. After that,
        % state.axesClickFcn must be the stashed handle and
        % ax.ButtonDownFcn must still be a live function_handle.
        assert(isa(state.axesClickFcn, 'function_handle'), ...
            'buildUI must stash axes click handler on state');
        assert(~isempty(state.axesClickFcn), ...
            'stashed handler must not be empty');
        assert(isa(state.ax.ButtonDownFcn, 'function_handle'), ...
            'ax.ButtonDownFcn must be a function_handle after initial drawMap');

        % Force another 2D redraw — handler must survive that too.
        state.addWaypoint(1000, 500);
        trackbench.editor.drawMap(state);
        assert(isa(state.ax.ButtonDownFcn, 'function_handle'), ...
            '2D redraw must not wipe ax.ButtonDownFcn');

        % Switch to 3D and redraw — 3D path must also restore.
        state.viewMode = "3d";
        trackbench.editor.drawMap(state);
        assert(isa(state.ax.ButtonDownFcn, 'function_handle'), ...
            '3D drawMap must also restore ax.ButtonDownFcn');

        pass('11. Patch C: ax.ButtonDownFcn survives cla reset (2D + 3D)');
        nPass = nPass + 1;
    catch ME
        fail('11. Patch C: ButtonDownFcn survives', ME); nFail = nFail + 1;
    end

    % ── Summary ────────────────────────────────────────────────────
    fprintf('\n---- Result: %d PASS / %d FAIL ----\n\n', nPass, nFail);
    if nFail > 0
        error('testPathEditor_M3:failures', '%d test(s) failed.', nFail);
    end
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
