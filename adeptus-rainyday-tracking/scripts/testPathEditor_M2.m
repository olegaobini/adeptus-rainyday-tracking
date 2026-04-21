function testPathEditor_M2()
%testPathEditor_M2  Programmatic tests for Milestone 2 of the path editor.
%
%  Covers state-level behavior that does NOT require a live GUI:
%    1. pushUndo + undo + redo round-trip
%    2. insertAfter midpoint / beginning / end
%    3. removeSelected updates selection index sensibly
%    4. moveSelectedTo with commit=false does not pollute undo stack
%    5. applyDefaultAltitudeToAll bulk-sets altitude + is undoable
%    6. findWaypointAt / findSegmentAt hit-testing
%    7. loadFromJSON round-trip (export → load → compare)
%    8. loadFromJSON tolerates hand-authored files (no speed_kmh, no RCS)
%    9. setWaypointProperty x/y/altitude/speed recomputes times
%   10. M1 regression: export schema still includes pos + time_s, Z < 0
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_M2
%
%  Prints PASS/FAIL per test. Any catch errors out the run so CI-style
%  chaining surfaces the failure.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.loadFromJSON,
%            trackbench.editor.exportToJSON, testPathEditor_M1

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_M2 ====\n');
    nPass = 0; nFail = 0;

    % ── 1. undo/redo round-trip ─────────────────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.defaultSpeedKmh = 720;
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 0);
        state.addWaypoint(1000, 1000);
        assert(state.count() == 3, '3 after add');
        ok = state.undo(); assert(ok, 'undo should succeed');
        assert(state.count() == 2, '2 after undo');
        ok = state.undo(); assert(ok);
        assert(state.count() == 1, '1 after second undo');
        ok = state.redo(); assert(ok, 'redo should succeed');
        assert(state.count() == 2, '2 after redo');
        ok = state.redo(); assert(ok);
        assert(state.count() == 3, '3 after second redo');
        % A new edit invalidates redo future
        state.undo();
        state.addWaypoint(500, 500);
        ok = state.redo();
        assert(~ok, 'redo must be empty after a new edit');
        pass('1. undo/redo round-trip');
        nPass = nPass + 1;
    catch ME
        fail('1. undo/redo round-trip', ME); nFail = nFail + 1;
    end

    % ── 2. insertAfter at various indices ───────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0, 0);
        state.addWaypoint(2000, 0);
        state.addWaypoint(4000, 0);
        assert(state.count() == 3);
        state.insertAfter(1, 1000, 0);   % between #1 and #2
        assert(state.count() == 4, 'count should be 4 after insert');
        assert(abs(state.waypoints(2,1) - 1000) < 1e-6, 'insert at index 2');
        state.insertAfter(0, -500, 0);   % new head
        assert(abs(state.waypoints(1,1) + 500) < 1e-6, 'insertAfter(0) becomes head');
        state.insertAfter(state.count(), 9999, 0);  % tail
        assert(abs(state.waypoints(end,1) - 9999) < 1e-6, 'tail insert');
        % Out-of-range rejects
        threw = false;
        try state.insertAfter(-1, 0, 0); catch; threw = true; end
        assert(threw, 'negative idx rejected');
        pass('2. insertAfter: head, mid, tail, out-of-range');
        nPass = nPass + 1;
    catch ME
        fail('2. insertAfter', ME); nFail = nFail + 1;
    end

    % ── 3. removeSelected selection index behavior ──────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 0);
        state.addWaypoint(2000, 0);
        state.selectedIndex = 2;
        state.removeSelected();
        assert(state.count() == 2, 'remove shrinks count');
        assert(state.selectedIndex == 2 || state.selectedIndex == 1, ...
            'selection clamps within bounds');
        state.removeSelected();
        state.removeSelected();
        assert(state.count() == 0, 'can remove down to zero');
        assert(state.selectedIndex == 0, 'empty → selectedIndex=0');
        pass('3. removeSelected: selection clamps');
        nPass = nPass + 1;
    catch ME
        fail('3. removeSelected', ME); nFail = nFail + 1;
    end

    % ── 4. moveSelectedTo(commit=false) doesn't flood undo ──────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 0);
        state.selectedIndex = 2;
        undoDepthBefore = numel(state.undoStack);
        % Simulate a drag: one pushUndo at start, then many moves w/o commit
        state.pushUndo();
        for tick = 1:50
            state.moveSelectedTo(tick*10, tick*5, false);
        end
        undoDepthAfter = numel(state.undoStack);
        assert(undoDepthAfter - undoDepthBefore == 1, ...
            sprintf('drag should add exactly ONE undo entry (got %d)', ...
                undoDepthAfter - undoDepthBefore));
        assert(abs(state.waypoints(2,1) - 500) < 1e-6, 'final x is 50*10=500');
        state.undo();
        assert(abs(state.waypoints(2,1) - 1000) < 1e-6, 'undo restores pre-drag position');
        pass('4. drag: 50 moves + one undo entry');
        nPass = nPass + 1;
    catch ME
        fail('4. moveSelectedTo commit=false', ME); nFail = nFail + 1;
    end

    % ── 5. applyDefaultAltitudeToAll + undo ─────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.defaultAltitudeM = 3000;
        state.addWaypoint(0, 0);
        state.addWaypoint(1000, 0);
        state.waypoints(1, 3) = 500;     % manually edit to non-default
        state.waypoints(2, 3) = 500;
        state.defaultAltitudeM = 8000;   % changed default
        state.applyDefaultAltitudeToAll();
        assert(all(state.waypoints(:,3) == 8000), 'all alts updated to 8000');
        state.undo();
        assert(all(state.waypoints(:,3) == 500), 'undo restores 500m alts');
        pass('5. applyDefaultAltitudeToAll: bulk + undo');
        nPass = nPass + 1;
    catch ME
        fail('5. applyDefaultAltitudeToAll', ME); nFail = nFail + 1;
    end

    % ── 6. hit-testing ─────────────────────────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.addWaypoint(0,    0);
        state.addWaypoint(10000, 0);
        state.addWaypoint(10000, 10000);
        % Near waypoint #2 (400m away — within 500m radius)
        idx = state.findWaypointAt(10400, 0);
        assert(idx == 2, sprintf('expected wp 2, got %d', idx));
        % Far from any waypoint
        idx = state.findWaypointAt(5000, 5000, 500);
        assert(idx == 0, 'expected miss');
        % Segment-1 hit: halfway along the east leg, 100m off
        [segIdx, proj] = state.findSegmentAt(5000, 100);
        assert(segIdx == 1, sprintf('expected seg 1, got %d', segIdx));
        assert(abs(proj(1) - 5000) < 1 && abs(proj(2) - 0) < 1, ...
            'projection near midpoint of seg 1');
        % Far from any segment
        [segIdx, ~] = state.findSegmentAt(0, 5000, 500);
        assert(segIdx == 0, 'segment hit-test misses far points');
        pass('6. findWaypointAt / findSegmentAt');
        nPass = nPass + 1;
    catch ME
        fail('6. hit-testing', ME); nFail = nFail + 1;
    end

    % ── 7. loadFromJSON round-trip (export → load → compare) ───────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.targetName = "m2_roundtrip";
        state.defaultSpeedKmh = 900;
        state.defaultAltitudeM = 3500;
        state.rcsDbsm = 12;
        state.rcsProfile = "fighter";
        state.addWaypoint(   0,      0);
        state.addWaypoint(5000,      0);
        state.addWaypoint(5000,   5000);
        state.addWaypoint(   0,  10000);

        outPath = trackbench.editor.exportToJSON(state);
        origWP = state.waypoints;
        origTimes = origWP(:, 4);
        origName = state.targetName;
        origRcsProfile = state.rcsProfile;

        state2 = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.loadFromJSON(state2, outPath);

        assert(state2.count() == 4, 'loaded 4 waypoints');
        assert(state2.targetName == origName, 'target name preserved');
        assert(state2.rcsProfile == origRcsProfile, 'rcs profile preserved');
        assert(state2.rcsDbsm == 12, 'rcs dBsm preserved');
        % Altitudes round-trip as positive in UI
        assert(all(state2.waypoints(:,3) > 0), 'altitudes positive after load');
        assert(all(abs(state2.waypoints(:,3) - 3500) < 1e-6), 'altitudes = 3500');
        % Times should round-trip exactly (or within 1ms slop)
        assert(max(abs(state2.waypoints(:,4) - origTimes)) < 1e-2, ...
            'time_s round-trips within 10 ms');
        % Speed metadata round-trips (not default-fallback)
        assert(all(abs(state2.waypoints(2:end,5) - 900) < 1), ...
            'speed_kmh round-trips to 900');
        assert(state2.loadedFrom ~= "", 'loadedFrom set');
        assert(~state2.isDirty, 'freshly loaded state not dirty');

        % Cleanup
        delete(outPath);
        pass('7. loadFromJSON round-trip: export → load preserves everything');
        nPass = nPass + 1;
    catch ME
        fail('7. loadFromJSON round-trip', ME); nFail = nFail + 1;
    end

    % ── 8. loadFromJSON tolerates hand-authored minimal file ───────
    try
        tmpDir = fullfile(projectRoot, "config", "targets", "waypoints");
        if ~exist(tmpDir, "dir"); mkdir(tmpDir); end
        tmpPath = fullfile(tmpDir, "m2_minimal.json");
        minimalJson = sprintf([ ...
            '{\n' ...
            '  "duration_s": 30,\n' ...
            '  "targets": [\n' ...
            '    {\n' ...
            '      "name": "m2_minimal",\n' ...
            '      "behavior": "waypoints",\n' ...
            '      "rcs_dbsm": 5,\n' ...
            '      "waypoints": [\n' ...
            '        { "pos": [0, 0, -2000], "time_s": 0 },\n' ...
            '        { "pos": [3000, 0, -2000], "time_s": 15 }\n' ...
            '      ]\n' ...
            '    }\n' ...
            '  ]\n' ...
            '}\n' ]);
        fid = fopen(tmpPath, 'w');
        cleaner = onCleanup(@() closeAndDelete(fid, tmpPath));
        fwrite(fid, minimalJson, 'char');
        fclose(fid);

        state3 = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.loadFromJSON(state3, tmpPath);
        assert(state3.count() == 2, 'hand-authored 2-wp file loads');
        assert(state3.rcsDbsm == 5, 'rcs_dbsm read');
        assert(state3.rcsProfile == "none", 'missing rcs_profile defaults to none');
        assert(all(state3.waypoints(:,3) > 0), 'Z flipped to positive alt on load');
        assert(abs(state3.waypoints(1,3) - 2000) < 1e-6, 'alt = 2000');
        % Speed back-computed from 3km / 15s = 200 m/s = 720 km/h
        assert(abs(state3.waypoints(2,5) - 720) < 1, ...
            sprintf('back-computed speed should be ~720 km/h, got %.1f', ...
                state3.waypoints(2,5)));
        pass('8. loadFromJSON: hand-authored minimal file');
        nPass = nPass + 1;
    catch ME
        fail('8. loadFromJSON minimal file', ME); nFail = nFail + 1;
    end

    % ── 9. setWaypointProperty recomputes times ────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.defaultSpeedKmh = 720;  % 200 m/s
        state.addWaypoint(0, 0);
        state.addWaypoint(2000, 0);
        t1 = state.waypoints(2, 4);
        assert(abs(t1 - 10) < 1e-3, 't1 starts at 10 s (2000/200)');
        state.selectedIndex = 2;
        state.setWaypointProperty(2, 'x', 4000);
        t2 = state.waypoints(2, 4);
        assert(abs(t2 - 20) < 1e-3, 't2 = 20 s after x doubled to 4000');
        % Editing speed should also change time
        state.setWaypointProperty(2, 'speed', 360);  % halve speed
        t3 = state.waypoints(2, 4);
        assert(abs(t3 - 40) < 1e-3, 't3 = 40 s after halving speed');
        % Altitude edits stay positive
        state.setWaypointProperty(2, 'altitude', -500);
        assert(state.waypoints(2, 3) == 0, 'altitude clamped to >= 0');
        pass('9. setWaypointProperty: x/y/altitude/speed');
        nPass = nPass + 1;
    catch ME
        fail('9. setWaypointProperty', ME); nFail = nFail + 1;
    end

    % ── 10. Schema regression (still consumable by addTargetFromDef) ─
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.targetName = "m2_schema";
        state.defaultSpeedKmh = 800;
        state.defaultAltitudeM = 3000;
        state.addWaypoint(0, 0);
        state.addWaypoint(4000, 0);
        state.addWaypoint(4000, 4000);

        outPath = trackbench.editor.exportToJSON(state);
        raw = fileread(outPath);
        decoded = jsondecode(raw);

        if iscell(decoded.targets)
            tgt = decoded.targets{1};
        else
            tgt = decoded.targets(1);
        end
        assert(strcmp(tgt.behavior, 'waypoints'), 'behavior correct');
        assert(numel(tgt.waypoints) == 3, '3 exported waypoints');
        allZ = arrayfun(@(w) w.pos(3), tgt.waypoints);
        assert(all(allZ < 0), 'all Z negative (NED)');
        allT = arrayfun(@(w) w.time_s, tgt.waypoints);
        assert(all(diff(allT) > 0), 'times strictly increasing');
        % speed_kmh field is present but addTargetFromDef ignores it
        assert(isfield(tgt.waypoints, 'speed_kmh') || ...
            (iscell(tgt.waypoints) && isfield(tgt.waypoints{1}, 'speed_kmh')), ...
            'speed_kmh metadata present');
        delete(outPath);
        pass('10. M1 schema regression: Z<0, monotonic t, speed_kmh metadata');
        nPass = nPass + 1;
    catch ME
        fail('10. schema regression', ME); nFail = nFail + 1;
    end

    % ── Summary ────────────────────────────────────────────────────
    fprintf('\n---- Result: %d PASS / %d FAIL ----\n\n', nPass, nFail);
    if nFail > 0
        error('testPathEditor_M2:failures', '%d test(s) failed.', nFail);
    end
end


function pass(label)
    fprintf('  PASS  %s\n', label);
end

function fail(label, ME)
    fprintf('  FAIL  %s\n', label);
    fprintf('        %s: %s\n', ME.identifier, ME.message);
end

function closeAndDelete(fid, filePath)
%closeAndDelete  onCleanup handler — closes fid if still open, removes file.
    try
        if fid >= 0
            try fclose(fid); catch; end
        end
        if isfile(filePath)
            delete(filePath);
        end
    catch
    end
end
