function testPathEditor_M5()
%testPathEditor_M5  Programmatic tests for Milestone 5 §3.1-§3.2.
%
%  Scope of THIS test file (full M5 check-in):
%    §3.1 — Targets-collection state refactor:
%             - EditorState owns a TargetRecord array
%             - Per-target dependent property proxies
%             - Snapshot/restore captures the whole collection
%             - addNewTarget / duplicateActiveTarget /
%               deleteActiveTarget / setActiveIdx / renameActiveTarget
%             - Snapshot independence (value-class TargetRecord)
%
%    §3.2 — Multi-target export + reference-target overlay:
%             - exportToJSON writes ALL writable targets
%             - exportToJSON excludes readOnly targets and reports the
%               excluded count
%             - exportToJSON errors when no writables remain
%             - loadFromJSON "replace" mode rebuilds the collection
%             - loadFromJSON "reference" mode appends readOnly targets
%             - unloadAllReferences removes refs only
%             - readOnly defense: dependent-property writes through the
%               active-target proxy are no-ops when the active is a ref
%             - Backward compat: an M4-era single-target file still loads
%
%  Manual interaction tests (Targets sub-panel UI, Load-as-Reference
%  menu, banner repaint, end-to-end runSingleScenario render) live in
%  PROGRESS_M5_*.md and verifyM5_endToEnd.
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_M5
%
%  Prints PASS/FAIL per test.
%
%  See also: testPathEditor_M4,
%            trackbench.editor.EditorState,
%            trackbench.editor.TargetRecord,
%            trackbench.editor.exportToJSON,
%            trackbench.editor.loadFromJSON

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_M5 (sections 3.1 – 3.2) ====\n');
    nPass = 0; nFail = 0;

    % ── Ephemeral export directory ─────────────────────────────────
    %  Every export-touching test points outputDir here so we don't
    %  pollute config/targets/waypoints/ with throwaway files.
    tmpRoot = tempname;
    mkdir(tmpRoot);
    mkdir(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
    tmpCleaner = onCleanup(@() rmdirSafe(tmpRoot));
    tmpWPDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));

    % ── 1. Constructor seeds with one writable target ──────────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        assert(isa(s.targets, 'trackbench.editor.TargetRecord'), ...
            'targets must be a TargetRecord array');
        assert(isscalar(s.targets), ...
            sprintf('expected 1 seed target, got %d', numel(s.targets)));
        assert(s.activeIdx == 1, 'activeIdx should default to 1');
        assert(~s.targets(1).readOnly, 'seed target must be writable');
        assert(strlength(s.targets(1).targetName) > 0, ...
            'seed target must have a non-empty name');
        ok(1);
    catch err
        fail(1, err);
    end

    % ── 2. Dependent properties proxy to the active target ─────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName       = "alpha";
        s.rcsDbsm          = 17;
        s.defaultSpeedKmh  = 750;
        s.curveMode        = "curved";
        s.curveTensionAlpha = 0.0;
        % Dependent getters must surface what we just wrote.
        assert(s.targets(1).targetName == "alpha");
        assert(s.targets(1).rcsDbsm == 17);
        assert(s.targets(1).defaultSpeedKmh == 750);
        assert(s.targets(1).curveMode == "curved");
        assert(abs(s.targets(1).curveTensionAlpha - 0.0) < 1e-12);
        ok(2);
    catch err
        fail(2, err);
    end

    % ── 3. addNewTarget appends and re-points active ───────────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName = "first";
        idx = s.addNewTarget("second");
        assert(numel(s.targets) == 2, 'addNewTarget should grow the array');
        assert(idx == 2 && s.activeIdx == 2, ...
            'addNewTarget should make the new target active');
        assert(s.targets(2).targetName == "second");
        % Switching back must restore the per-target proxy values.
        s.setActiveIdx(1);
        assert(s.targetName == "first", ...
            'setActiveIdx did not re-point dependent proxy');
        ok(3);
    catch err
        fail(3, err);
    end

    % ── 4. duplicateActiveTarget produces an independent copy ──────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName = "orig";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        idx = s.duplicateActiveTarget();
        assert(idx == 2 && numel(s.targets) == 2, ...
            'duplicate should append and become active');
        assert(contains(string(s.targets(2).targetName), "copy"), ...
            'duplicate should suffix _copy in the name');
        % Mutate the copy; original must NOT change (value-class semantics).
        s.targetName = "renamed_copy";
        assert(s.targets(1).targetName == "orig", ...
            'mutating the copy bled into the original');
        assert(~s.targets(2).readOnly, 'duplicate must be writable');
        ok(4);
    catch err
        fail(4, err);
    end

    % ── 5. deleteActiveTarget shrinks and re-points safely ─────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.addNewTarget("two");
        s.addNewTarget("three");
        assert(numel(s.targets) == 3 && s.activeIdx == 3);
        s.deleteActiveTarget();
        assert(numel(s.targets) == 2, 'delete should shrink array');
        assert(s.activeIdx == 2, ...
            'after deleting the tail, active should clamp to new tail');
        % Delete remaining two — activeIdx must end at 0.
        s.deleteActiveTarget();
        s.deleteActiveTarget();
        assert(isempty(s.targets), 'all targets should be removed');
        assert(s.activeIdx == 0, ...
            'after last delete, activeIdx should be 0');
        ok(5);
    catch err
        fail(5, err);
    end

    % ── 6. Snapshot independence (value-class TargetRecord) ────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName = "snap_test";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        snap = s.snapshot();
        % Mutate the live state AFTER snapshotting.
        s.targetName = "mutated";
        s.addWaypoint(2000, 1500);
        % Restore — the live state must come back to snapshot values
        % even though we made post-snapshot mutations.
        s.restore(snap);
        assert(s.targetName == "snap_test", ...
            'restore did not bring back the snapshotted name');
        assert(size(s.waypoints, 1) == 2, ...
            'restore did not bring back the snapshotted waypoint count');
        ok(6);
    catch err
        fail(6, err);
    end

    % ── 7. Undo/redo across target-collection mutations ────────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName = "T1";
        s.addNewTarget("T2");                 % undo step recorded
        assert(numel(s.targets) == 2);
        assert(s.undo(), 'undo of addNewTarget should succeed');
        assert(isscalar(s.targets), ...
            'undo of addNewTarget did not remove the appended target');
        assert(s.activeIdx == 1, ...
            'undo of addNewTarget should restore active to original');
        assert(s.redo(), 'redo of addNewTarget should succeed');
        assert(numel(s.targets) == 2 && s.activeIdx == 2, ...
            'redo of addNewTarget did not reinstate the appended target');
        ok(7);
    catch err
        fail(7, err);
    end

    % ── 8. Multi-target export: 2 writables → both serialized ──────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        s.targetName = "multi_a";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.addNewTarget("multi_b");
        s.addWaypoint(0, 1000);
        s.addWaypoint(2000, 1000);

        [path, excluded] = trackbench.editor.exportToJSON(s, "multi_two");
        assert(isfile(path), 'export did not write file');
        assert(excluded == 0, 'no refs in scene → excludedRefCount must be 0');
        decoded = jsondecode(fileread(path));
        tgtCell = normalizeTargetsCell(decoded);
        assert(numel(tgtCell) == 2, ...
            sprintf('expected 2 targets in multi export, got %d', numel(tgtCell)));
        names = string(cellfun(@(t) string(t.name), tgtCell, ...
            'UniformOutput', false));
        assert(any(names == "multi_a") && any(names == "multi_b"), ...
            'multi-target export should preserve both names');
        ok(8);
    catch err
        fail(8, err);
    end

    % ── 9. Reference-filter export: ref excluded, count surfaced ───
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        % Mark the seed target as a reference (simulates a ref load).
        s.targetName = "the_ref";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.targets(1).readOnly = true;
        s.targets(1).sourceFile = "fake/ref.json";
        % Add a writable target to keep export non-empty.
        s.addNewTarget("the_writable");
        s.addWaypoint(0, 1000);
        s.addWaypoint(2000, 1000);

        [path, excluded] = trackbench.editor.exportToJSON(s, "filter_test");
        assert(excluded == 1, ...
            sprintf('expected 1 ref excluded, got %d', excluded));
        decoded = jsondecode(fileread(path));
        tgtCell = normalizeTargetsCell(decoded);
        assert(isscalar(tgtCell), ...
            sprintf('expected 1 writable in export, got %d', numel(tgtCell)));
        assert(string(tgtCell{1}.name) == "the_writable", ...
            'wrong target survived the ref filter');
        ok(9);
    catch err
        fail(9, err);
    end

    % ── 10. exportToJSON errors when zero writables remain ─────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        s.targetName = "only_ref";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.targets(1).readOnly = true;
        threw = false;
        try
            trackbench.editor.exportToJSON(s, "should_fail");
        catch ME
            threw = true;
            assert(contains(ME.identifier, 'noWritableTargets'), ...
                sprintf('wrong error id: %s', ME.identifier));
        end
        assert(threw, ...
            'exportToJSON should error when every target is readOnly');
        ok(10);
    catch err
        fail(10, err);
    end

    % ── 11. loadFromJSON replace mode: rebuilds the collection ─────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        s.targetName = "rep_a";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.addNewTarget("rep_b");
        s.addWaypoint(0, 1000);
        s.addWaypoint(2000, 1000);
        path = trackbench.editor.exportToJSON(s, "replace_src");

        s2 = trackbench.editor.EditorState(projectRoot);
        s2.addNewTarget("dirty_existing"); % seed target + 1 dirty extra
        trackbench.editor.loadFromJSON(s2, path);   % default = "replace"
        assert(numel(s2.targets) == 2, ...
            'replace-mode load should leave exactly 2 targets');
        assert(s2.activeIdx == 1, 'replace load should set active=1');
        assert(~any([s2.targets.readOnly]), ...
            'replace-mode load should not flag any target readOnly');
        ok(11);
    catch err
        fail(11, err);
    end

    % ── 12. loadFromJSON reference mode: appends + readOnly ────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        s.targetName = "ref_src_a";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.addNewTarget("ref_src_b");
        s.addWaypoint(0, 1000);
        s.addWaypoint(2000, 1000);
        path = trackbench.editor.exportToJSON(s, "reference_src");

        s2 = trackbench.editor.EditorState(projectRoot);
        s2.targetName = "kept_active";
        nBefore = numel(s2.targets);
        activeBefore = s2.activeIdx;
        trackbench.editor.loadFromJSON(s2, path, "reference");
        nAfter = numel(s2.targets);
        assert(nAfter == nBefore + 2, ...
            sprintf('reference load should APPEND 2; before=%d after=%d', ...
                nBefore, nAfter));
        % Active target must NOT change in reference mode.
        assert(s2.activeIdx == activeBefore, ...
            'reference load must not change activeIdx');
        % All appended must be readOnly with sourceFile populated.
        for k = (nBefore + 1):nAfter
            assert(s2.targets(k).readOnly, ...
                sprintf('appended target %d should be readOnly', k));
            assert(strlength(s2.targets(k).sourceFile) > 0, ...
                sprintf('appended target %d should have sourceFile', k));
        end
        ok(12);
    catch err
        fail(12, err);
    end

    % ── 13. unloadAllReferences strips refs, leaves writables ──────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = tmpWPDir;
        s.targetName = "writable_x";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.addNewTarget("writable_y");
        s.addWaypoint(0, 1000);
        s.addWaypoint(2000, 1000);
        path = trackbench.editor.exportToJSON(s, "unload_src");

        s2 = trackbench.editor.EditorState(projectRoot);
        s2.targetName = "kept";
        trackbench.editor.loadFromJSON(s2, path, "reference");
        nBefore = numel(s2.targets);
        nRemoved = s2.unloadAllReferences();
        assert(nRemoved == 2, ...
            sprintf('expected 2 refs removed, got %d', nRemoved));
        assert(numel(s2.targets) == nBefore - 2, ...
            'unloadAllReferences did not shrink by the right amount');
        assert(~any([s2.targets.readOnly]), ...
            'no writable target should be flagged readOnly afterwards');
        ok(13);
    catch err
        fail(13, err);
    end

    % ── 14. readOnly defense: dependent setters are no-ops on refs ─
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.targetName = "frozen";
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        wpBefore   = s.waypoints;
        nameBefore = s.targetName;
        rcsBefore  = s.rcsDbsm;
        % Promote the active target to a reference and try to mutate
        % every dependent property. Each assignment must be a no-op.
        s.targets(1).readOnly = true;
        s.waypoints  = [9999, 9999, 0, 0, 0; 8888, 8888, 0, 1, 0];
        s.targetName = "should_not_take";
        s.rcsDbsm    = 99;
        s.curveMode  = "curved";
        assert(isequal(s.waypoints, wpBefore), ...
            'waypoints setter should be a no-op on a readOnly target');
        assert(s.targetName == nameBefore, ...
            'targetName setter should be a no-op on a readOnly target');
        assert(s.rcsDbsm == rcsBefore, ...
            'rcsDbsm setter should be a no-op on a readOnly target');
        assert(s.curveMode == "straight", ...
            'curveMode setter should be a no-op on a readOnly target');
        ok(14);
    catch err
        fail(14, err);
    end

    % ── 15. Backward compat: M4-era single-target file still loads ─
    try
        % Build an M4-shape file by hand: a single object at root.targets
        % carrying just the M4 fields. No M5 fields anywhere.
        legacyPath = fullfile(tmpRoot, 'legacy_m4_singletarget.json');
        legacy = struct();
        legacy.description = char("Legacy M4 single-target");
        legacy.duration_s  = 60;
        tgt = struct();
        tgt.name        = 'legacy_solo';
        tgt.label       = 'legacy_solo';
        tgt.behavior    = 'waypoints';
        tgt.rcs_dbsm    = 10;
        tgt.rcs_profile = 'airliner';
        tgt.curve_mode  = 'straight';
        wp(1).pos = [0,    0,    -1000]; wp(1).time_s = 0;  wp(1).speed_kmh = 600;
        wp(2).pos = [5000, 3000, -2000]; wp(2).time_s = 30; wp(2).speed_kmh = 600;
        wp(3).pos = [12000, 6000, -1500]; wp(3).time_s = 60; wp(3).speed_kmh = 600;
        tgt.waypoints = wp;
        legacy.targets = {tgt};
        fid = fopen(legacyPath, 'w');
        fwrite(fid, jsonencode(legacy), 'char');
        fclose(fid);

        s = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.loadFromJSON(s, legacyPath);
        assert(isscalar(s.targets), ...
            'M4 single-target file should hydrate as exactly 1 target');
        assert(s.targets(1).targetName == "legacy_solo", ...
            'legacy target name lost in load');
        assert(s.targets(1).curveMode == "straight", ...
            'legacy file should load as straight');
        assert(~s.targets(1).readOnly, ...
            'legacy load (replace mode) should NOT set readOnly');
        assert(size(s.waypoints, 1) == 3, ...
            'expected 3 waypoints from legacy file');
        ok(15);
    catch err
        fail(15, err);
    end

    % ── Summary ─────────────────────────────────────────────────────
    total = nPass + nFail;
    fprintf('\n----\nPASSED %d/%d\n', nPass, total);
    if nFail > 0
        error('testPathEditor_M5:someFailed', '%d tests failed', nFail);
    end

    % ── Local helpers (closures capture nPass/nFail) ───────────────
    function ok(k)
        nPass = nPass + 1;
        fprintf('  [%2d] PASS\n', k);
    end
    function fail(k, err)
        nFail = nFail + 1;
        fprintf('  [%2d] FAIL  %s\n', k, err.message);
    end
end


%% ========================================================================
%  File-scope helpers (not nested — no access to nPass/nFail)
%% ========================================================================
function tgtCell = normalizeTargetsCell(decoded)
%normalizeTargetsCell  Always return a cell-array of target structs from
%                      a decoded export. jsondecode flip-flops between cell
%                      and struct-array depending on whether the per-target
%                      fields line up homogeneously, so we normalize once
%                      here and the tests don't have to care.
    assert(isstruct(decoded) && isfield(decoded, 'targets'), ...
        'normalizeTargetsCell: decoded.targets missing');
    arr = decoded.targets;
    if iscell(arr)
        tgtCell = arr;
    elseif isstruct(arr)
        tgtCell = num2cell(arr);
    else
        error('normalizeTargetsCell: unexpected targets shape (%s)', class(arr));
    end
end


function rmdirSafe(path)
%rmdirSafe  Best-effort cleanup of the test temp directory. Swallows
%           errors so a stuck file handle doesn't fail the test run.
    try
        if exist(path, 'dir')
            rmdir(path, 's');
        end
    catch
    end
end
