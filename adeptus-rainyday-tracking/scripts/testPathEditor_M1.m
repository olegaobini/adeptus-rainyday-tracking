function testPathEditor_M1()
%testPathEditor_M1  Programmatic tests for Milestone 1 of the path editor.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Covers everything that does not require a live GUI:
%    1. EditorState construction + addWaypoint() + recomputeTimes()
%    2. exportToJSON schema (keys, field names, structure)
%    3. JSON round-trips through jsondecode back to a struct array that
%       trackbench.scenario.addTargetFromDef can consume.
%    4. NED sign convention on export (Z is negative-down).
%    5. Strict monotonic time_s after export.
%    6. Guard: export fails cleanly on < 2 waypoints.
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_M1
%
%  Prints PASS/FAIL per test, throws on the first hard failure so you
%  notice it in a chained run.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.exportToJSON,
%            trackbench.scenario.addTargetFromDef, pathEditor

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_M1 ====\n');
    nPass = 0; nFail = 0;

    % ── 1. EditorState construction + waypoint addition ─────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        assert(state.count() == 0, 'fresh state should have 0 waypoints');
        state.defaultSpeedKmh  = 720;   % 200 m/s — easy to verify by hand
        state.defaultAltitudeM = 3000;
        state.addWaypoint(0, 0);
        state.addWaypoint(2000, 0);   % 2 km east
        state.addWaypoint(2000, 2000);% 2 km north
        assert(state.count() == 3, 'expected 3 waypoints');
        % Times: leg1 = 2000m / 200 m/s = 10 s, leg2 = 10 s more = 20 s
        assert(abs(state.waypoints(1,4) - 0)  < 1e-6, 't0 should be 0');
        assert(abs(state.waypoints(2,4) - 10) < 1e-3, 't1 should be 10 s');
        assert(abs(state.waypoints(3,4) - 20) < 1e-3, 't2 should be 20 s');
        pass('1. EditorState: add + auto-time');
        nPass = nPass + 1;
    catch ME
        fail('1. EditorState: add + auto-time', ME);
        nFail = nFail + 1;
    end

    % ── 2. exportToJSON writes a file with the expected schema ──────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.targetName = "m1_unit_test";
        state.defaultSpeedKmh = 900;
        state.defaultAltitudeM = 3000;
        state.rcsDbsm = 10;
        state.rcsProfile = "airliner";
        state.addWaypoint(   0,      0);
        state.addWaypoint(5000,      0);
        state.addWaypoint(5000,   5000);
        state.addWaypoint(   0,  10000);

        outPath = trackbench.editor.exportToJSON(state);
        assert(isfile(outPath), 'export file was not created');

        raw = fileread(outPath);
        decoded = jsondecode(raw);

        assert(isfield(decoded, 'description'),  'missing description');
        assert(isfield(decoded, 'duration_s'),   'missing duration_s');
        assert(isfield(decoded, 'targets'),      'missing targets');
        assert(decoded.duration_s >= state.waypoints(end,4), ...
            'duration_s must be >= max time_s');

        % Targets is a 1-element cell array in the jsonencode output; on
        % decode that becomes either a struct or a struct-of-arrays.
        if iscell(decoded.targets)
            tgt = decoded.targets{1};
        else
            tgt = decoded.targets(1);
        end
        assert(strcmp(tgt.behavior, 'waypoints'), 'behavior must be "waypoints"');
        assert(strcmp(tgt.name,     'm1_unit_test'), 'name mismatch');
        assert(strcmp(tgt.label,    'm1_unit_test'), 'label mismatch');
        assert(tgt.rcs_dbsm == 10, 'rcs_dbsm mismatch');
        assert(strcmp(tgt.rcs_profile, 'airliner'), 'rcs_profile mismatch');
        assert(numel(tgt.waypoints) == 4, 'expected 4 exported waypoints');
        pass('2. exportToJSON: schema and top-level fields');
        nPass = nPass + 1;

        % ── 3. NED sign convention ──────────────────────────────────
        %  Every exported Z must be negative (altitude below-ground in NED).
        allZ = arrayfun(@(w) w.pos(3), tgt.waypoints);
        assert(all(allZ < 0), 'exported Z values must all be negative');
        assert(all(abs(allZ + 3000) < 1e-6), 'altitude should be -3000 m');
        pass('3. exportToJSON: NED sign (Z < 0)');
        nPass = nPass + 1;

        % ── 4. Strictly increasing time_s ───────────────────────────
        allT = arrayfun(@(w) w.time_s, tgt.waypoints);
        assert(all(diff(allT) > 0), 'time_s must be strictly increasing');
        pass('4. exportToJSON: monotonic time_s');
        nPass = nPass + 1;

        % ── 5. Consumer-shape check: addTargetFromDef buildFromWaypoints
        %      iterates with wk = wpDefs(k) and reads wk.pos / wk.time_s.
        %      Verify the decoded struct array exposes those fields.
        if isstruct(tgt.waypoints)
            wk = tgt.waypoints(1);
            assert(isfield(wk, 'pos') && numel(wk.pos) == 3, ...
                'consumer expects pos as 3-vector');
            assert(isfield(wk, 'time_s'), 'consumer expects time_s field');
            pass('5. exportToJSON: consumer-shape (pos + time_s)');
            nPass = nPass + 1;
        else
            fprintf('   SKIP 5. Decoded waypoints is not a struct array (%s)\n', ...
                class(tgt.waypoints));
        end

    catch ME
        fail('2-5. exportToJSON + schema', ME);
        nFail = nFail + 1;
    end

    % ── 6. Guard: too few waypoints ────────────────────────────────
    try
        state = trackbench.editor.EditorState(projectRoot);
        state.targetName = "m1_should_not_write";
        state.addWaypoint(0, 0);   % only one
        threw = false;
        try
            trackbench.editor.exportToJSON(state);
        catch
            threw = true;
        end
        assert(threw, 'export with <2 waypoints should throw');
        % And should NOT have written the file.
        badPath = fullfile(state.outputDir, "m1_should_not_write.json");
        assert(~isfile(badPath), 'export should not create a file for <2 waypoints');
        pass('6. exportToJSON: rejects <2 waypoints');
        nPass = nPass + 1;
    catch ME
        fail('6. exportToJSON: rejects <2 waypoints', ME);
        nFail = nFail + 1;
    end

    % ── Summary ────────────────────────────────────────────────────
    fprintf('\n---- Result: %d PASS / %d FAIL ----\n\n', nPass, nFail);
    if nFail > 0
        error('testPathEditor_M1:failures', '%d test(s) failed.', nFail);
    end
end


function pass(label)
    fprintf('  PASS  %s\n', label);
end

function fail(label, ME)
    fprintf('  FAIL  %s\n', label);
    fprintf('        %s: %s\n', ME.identifier, ME.message);
end
