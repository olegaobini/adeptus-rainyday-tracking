function testPathEditor_M4()
%testPathEditor_M4  Programmatic tests for Milestone 4 sections 3.1-3.5.
%
%  Scope of THIS test file (full M4 check-in):
%    §3.1 — Centripetal Catmull-Rom math (interpolation, endpoints,
%           densification size, tension family, 3D behavior, time axis).
%    §3.2 — EditorState.curveMode plumbing (default, snapshot/restore,
%           undo/redo, coexistence with M3 state).
%    §3.3 — Curve-tension dropdown plumbing (alpha round-trip through
%           state changes the rendered dense curve shape).
%    §3.4 — exportToJSON / loadFromJSON curve round-trip
%           (control_waypoints + curve_mode + curve_tension_alpha);
%           legacy M3-era files load cleanly as straight mode.
%    §3.5 — Animation preview walks the densified curve (smoke test:
%           dense waypoints derivable from state for the preview call).
%
%  HOW TO RUN
%      addpath("scripts");
%      testPathEditor_M4
%
%  Prints PASS/FAIL per test. Manual interaction tests (toggle, drag,
%  3D, end-to-end runSingleScenario) live in TESTING_M4.md.
%
%  See also: testPathEditor_M3, trackbench.editor.catmullRomCurve,
%            trackbench.editor.EditorState,
%            trackbench.editor.exportToJSON,
%            trackbench.editor.loadFromJSON

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditor_M4 (sections 3.1 – 3.5) ====\n');
    nPass = 0; nFail = 0;

    % ── Fixtures ────────────────────────────────────────────────────
    % Four non-collinear 3D waypoints with varied altitudes and an
    % explicit time axis. Column order: [x y alt time speed].
    wp = [    0,    0, 1000,    0, 600
           5000, 3000, 2000,   40, 700
          12000, 6000, 1500,   90, 650
          18000, 2000, 3000,  150, 800];

    % ── 1. Interpolation: curve passes through every control point ──
    try
        dp = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
        % Dense size = (K-1)*N + 1 = 3*50 + 1 = 151 for K=4, N=50.
        assert(size(dp, 1) == 151, ...
            sprintf('expected 151 dense points, got %d', size(dp, 1)));
        % Control points appear at rows 1, 51, 101, 151.
        ctrlRows = [1 51 101 151];
        for r = 1:4
            d = norm(dp(ctrlRows(r), :) - wp(r, 1:3));
            assert(d <= 1e-10, sprintf( ...
                'control point %d not interpolated (err=%.3e)', r, d));
        end
        ok(1);
    catch err
        fail(1, err);
    end

    % ── 2. Endpoint phantom reflection produces finite, non-NaN ─────
    try
        dp = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
        assert(all(isfinite(dp(:))), 'NaN/Inf in dense curve');
        % Numerically approximate the forward tangent at the first
        % control point and the backward tangent at the last. Both
        % should be finite and non-zero — if the phantom reflection
        % degenerated, these would blow up or vanish.
        tanStart = dp(2,  :) - dp(1,     :);
        tanEnd   = dp(end,:) - dp(end-1, :);
        assert(all(isfinite(tanStart)) && norm(tanStart) > 0, ...
            'first-segment tangent is zero or non-finite');
        assert(all(isfinite(tanEnd))   && norm(tanEnd)   > 0, ...
            'last-segment tangent is zero or non-finite');
        ok(2);
    catch err
        fail(2, err);
    end

    % ── 3. Densification size scales as (K-1)*N + 1 ─────────────────
    try
        for Nn = [2 10 37 100]
            dp = trackbench.editor.catmullRomCurve(wp, Nn, 0.5);
            expected = (size(wp,1) - 1) * Nn + 1;
            assert(size(dp,1) == expected, sprintf( ...
                'N=%d: expected %d dense pts, got %d', ...
                Nn, expected, size(dp,1)));
        end
        % K=2 → straight line (no interior curvature) still interpolates.
        dp2 = trackbench.editor.catmullRomCurve(wp(1:2,:), 20, 0.5);
        assert(size(dp2,1) == 21, 'K=2 densification size wrong');
        assert(norm(dp2(1,:)   - wp(1,1:3)) <= 1e-10);
        assert(norm(dp2(end,:) - wp(2,1:3)) <= 1e-10);
        ok(3);
    catch err
        fail(3, err);
    end

    % ── 4. Tension family: uniform / centripetal / chordal differ ──
    try
        dpU = trackbench.editor.catmullRomCurve(wp, 100, 0.0);
        dpC = trackbench.editor.catmullRomCurve(wp, 100, 0.5);
        dpH = trackbench.editor.catmullRomCurve(wp, 100, 1.0);
        % Compare curves pointwise: different alphas produce visibly
        % different shapes even though the total polyline lengths can be
        % numerically very close (endpoints + control points are fixed).
        dUC = max(sqrt(sum((dpU - dpC).^2, 2)));
        dCH = max(sqrt(sum((dpC - dpH).^2, 2)));
        assert(dUC > 1.0, sprintf( ...
            'uniform vs centripetal curves nearly identical (max gap=%.3f)', dUC));
        assert(dCH > 1.0, sprintf( ...
            'centripetal vs chordal curves nearly identical (max gap=%.3f)', dCH));
        % All three must still interpolate the control points.
        for dp = {dpU, dpC, dpH}
            assert(norm(dp{1}(1,  :) - wp(1,  1:3)) <= 1e-10);
            assert(norm(dp{1}(end,:) - wp(end,1:3)) <= 1e-10);
        end
        ok(4);
    catch err
        fail(4, err);
    end

    % ── 5. Interpolates in 3D (altitude varies smoothly) ────────────
    try
        dp = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
        assert(size(dp,2) == 3, 'expected Nx3 dense points');
        % Altitude column must vary (the fixture has 4 distinct alts).
        assert(max(dp(:,3)) - min(dp(:,3)) > 500, ...
            'altitude did not vary along curve');
        % Between rows 51 (control 2: alt=2000) and 101 (control 3: alt=1500)
        % the altitude should decrease monotonically (fixture is monotone
        % on that leg and centripetal CR won't overshoot it).
        segA = dp(51:101, 3);
        assert(segA(1) > segA(end), 'segment 2-3 altitude wrong direction');
        ok(5);
    catch err
        fail(5, err);
    end

    % ── 6. Time parameterization matches control times ──────────────
    try
        [~, dt] = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
        assert(all(diff(dt) >= -1e-12), 'dense time not monotone');
        ctrlRows = [1 51 101 151];
        for r = 1:4
            err = abs(dt(ctrlRows(r)) - wp(r, 4));
            assert(err <= 1e-9, sprintf( ...
                'time at control %d off by %.3e', r, err));
        end
        ok(6);
    catch err
        fail(6, err);
    end

    % ── 7. Segment index labels dense points correctly ─────────────
    try
        [~, ~, seg] = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
        assert(numel(seg) == 151);
        % Segment 1 runs rows 1..50 (inclusive of start, exclusive of
        % right endpoint). Segment 2 runs rows 51..100. Segment 3 runs
        % rows 101..150. The very last row (151) is the final control
        % point, labelled inside segment 3.
        assert(all(seg(1:50)    == 1));
        assert(all(seg(51:100)  == 2));
        assert(all(seg(101:151) == 3));
        ok(7);
    catch err
        fail(7, err);
    end

    % ── 8. EditorState.curveMode defaults to "straight" ─────────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        assert(s.curveMode == "straight", ...
            sprintf('curveMode default wrong: %s', s.curveMode));
        assert(abs(s.curveTensionAlpha - 0.5) < 1e-12);
        assert(s.curveDensityPerSeg == 50);
        ok(8);
    catch err
        fail(8, err);
    end

    % ── 9. Snapshot/restore captures curveMode + alpha ──────────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.pushUndo();
        s.curveMode = "curved";
        s.curveTensionAlpha = 1.0;
        % Undo must roll back both the curveMode and the tension.
        assert(s.undo());
        assert(s.curveMode == "straight", 'undo did not restore curveMode');
        assert(abs(s.curveTensionAlpha - 0.5) < 1e-12, ...
            'undo did not restore curveTensionAlpha');
        % Redo must reapply both.
        assert(s.redo());
        assert(s.curveMode == "curved", 'redo did not reapply curveMode');
        assert(abs(s.curveTensionAlpha - 1.0) < 1e-12, ...
            'redo did not reapply curveTensionAlpha');
        ok(9);
    catch err
        fail(9, err);
    end

    % ── 10. Old snapshots (pre-M4) without curveMode restore safely ─
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.curveMode = "curved";
        s.curveTensionAlpha = 1.0;
        legacySnap = struct( ...
            'waypoints',        zeros(0,5), ...
            'targetName',       "legacy", ...
            'description',      "", ...
            'rcsDbsm',          10, ...
            'rcsProfile',       "airliner", ...
            'defaultSpeedKmh',  900, ...
            'defaultAltitudeM', 3000, ...
            'selectedIndex',    0, ...
            'loadedFrom',       "");
        s.restore(legacySnap);
        assert(s.curveMode == "straight", ...
            'legacy restore should fall back to straight');
        assert(abs(s.curveTensionAlpha - 0.5) < 1e-12, ...
            'legacy restore should fall back to alpha=0.5');
        ok(10);
    catch err
        fail(10, err);
    end

    % ── 11. M3 regression: toggling curveMode does not touch waypoints
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.addWaypoint(0, 0);
        s.addWaypoint(1000, 500);
        s.addWaypoint(2500, 2000);
        wpBefore = s.waypoints;
        s.pushUndo();
        s.curveMode = "curved";
        assert(isequal(s.waypoints, wpBefore), ...
            'curveMode flip mutated waypoints');
        ok(11);
    catch err
        fail(11, err);
    end

    % ── 12. catmullRomCurve gracefully handles K=0 and K=1 ──────────
    try
        d0 = trackbench.editor.catmullRomCurve(zeros(0,5));
        assert(isempty(d0));
        d1 = trackbench.editor.catmullRomCurve(wp(1,:));
        assert(isequal(size(d1), [1 3]));
        assert(norm(d1 - wp(1, 1:3)) <= 1e-12);
        ok(12);
    catch err
        fail(12, err);
    end

    % ── Ephemeral export directory — every §3.4 test writes here ────
    tmpRoot = tempname;
    mkdir(tmpRoot);
    mkdir(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
    tmpCleaner = onCleanup(@() rmdirSafe(tmpRoot));  %#ok<NASGU>

    % ── 13. §3.3: curveTensionAlpha change flows through to render ──
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        s.addWaypoint(18000, 2000);
        s.waypoints(:,3) = [1000; 2000; 1500; 3000];
        s.curveMode = "curved";
        s.curveTensionAlpha = 0.0;
        dpU = trackbench.editor.catmullRomCurve(s.waypoints, ...
            s.curveDensityPerSeg, s.curveTensionAlpha);
        s.curveTensionAlpha = 1.0;
        dpH = trackbench.editor.catmullRomCurve(s.waypoints, ...
            s.curveDensityPerSeg, s.curveTensionAlpha);
        gap = max(sqrt(sum((dpU - dpH).^2, 2)));
        assert(gap > 1.0, sprintf( ...
            'tension alpha change did not reshape curve (max gap=%.3f)', gap));
        % Both must still interpolate the controls exactly.
        ctrlRows = [1 51 101 151];
        for r = 1:4
            assert(norm(dpU(ctrlRows(r), :) - s.waypoints(r,1:3)) <= 1e-10);
            assert(norm(dpH(ctrlRows(r), :) - s.waypoints(r,1:3)) <= 1e-10);
        end
        ok(13);
    catch err
        fail(13, err);
    end

    % ── 14. §3.4 straight export: no control_waypoints / curve_tension_alpha
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
        s.targetName = "straight_only";
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        path = trackbench.editor.exportToJSON(s);
        assert(isfile(path), 'straight export did not write file');
        decoded = jsondecode(fileread(path));
        tgt = normalizeTarget(decoded);
        assert(strcmpi(tgt.curve_mode, 'straight'), ...
            'straight export should tag curve_mode="straight"');
        assert(~isfield(tgt, 'control_waypoints'), ...
            'straight export must not emit control_waypoints');
        assert(~isfield(tgt, 'curve_tension_alpha'), ...
            'straight export must not emit curve_tension_alpha');
        assert(numel(tgt.waypoints) == 3, ...
            'straight export should emit exactly the control waypoints');
        ok(14);
    catch err
        fail(14, err);
    end

    % ── 15. §3.4 curved export: control_waypoints + densified waypoints
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
        s.targetName = "curved_path";
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        s.addWaypoint(18000, 2000);
        s.waypoints(:,3) = [1000; 2000; 1500; 3000];
        s.curveMode = "curved";
        s.curveTensionAlpha = 1.0;
        path = trackbench.editor.exportToJSON(s);
        decoded = jsondecode(fileread(path));
        tgt = normalizeTarget(decoded);
        assert(strcmpi(tgt.curve_mode, 'curved'), 'curve_mode tag wrong');
        assert(isfield(tgt, 'control_waypoints') && ~isempty(tgt.control_waypoints), ...
            'curved export must emit control_waypoints');
        assert(abs(double(tgt.curve_tension_alpha) - 1.0) < 1e-12, ...
            'exported curve_tension_alpha differs from state');
        nCtrl = numel(tgt.control_waypoints);
        nDense = numel(tgt.waypoints);
        assert(nCtrl == 4, 'control_waypoints count wrong');
        expectedDense = (nCtrl - 1) * s.curveDensityPerSeg + 1;
        assert(nDense == expectedDense, sprintf( ...
            'dense waypoints count: expected %d, got %d', expectedDense, nDense));
        % Dense list's first and last positions match first/last controls.
        firstCtrl = ctrlStructAt(tgt.control_waypoints, 1);
        lastCtrl  = ctrlStructAt(tgt.control_waypoints, nCtrl);
        firstDen  = ctrlStructAt(tgt.waypoints, 1);
        lastDen   = ctrlStructAt(tgt.waypoints, nDense);
        assert(norm(firstCtrl.pos(:) - firstDen.pos(:)) < 1e-6);
        assert(norm(lastCtrl.pos(:)  - lastDen.pos(:))  < 1e-6);
        % NED convention: z must be negative (altitude stored positive-up in UI).
        for k = [1 2 nDense]
            w = ctrlStructAt(tgt.waypoints, k);
            assert(w.pos(3) <= 0, sprintf( ...
                'waypoint %d pos.z=%.3f should be <= 0 (NED)', k, w.pos(3)));
        end
        ok(15);
    catch err
        fail(15, err);
    end

    % ── 16. §3.4 round-trip: curved export → load restores editor state
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
        s.targetName = "roundtrip_curved";
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        s.addWaypoint(18000, 2000);
        s.waypoints(:,3) = [1000; 2000; 1500; 3000];
        originalWP = s.waypoints;
        s.curveMode = "curved";
        s.curveTensionAlpha = 0.0;  % uniform
        path = trackbench.editor.exportToJSON(s);

        s2 = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.loadFromJSON(s2, path);
        assert(size(s2.waypoints, 1) == 4, ...
            'loaded control waypoints should be the 4 originals, not dense');
        assert(max(abs(s2.waypoints(:,1) - originalWP(:,1))) < 1e-6);
        assert(max(abs(s2.waypoints(:,2) - originalWP(:,2))) < 1e-6);
        assert(max(abs(s2.waypoints(:,3) - originalWP(:,3))) < 1e-6);
        assert(s2.curveMode == "curved", 'round-trip lost curveMode');
        assert(abs(s2.curveTensionAlpha - 0.0) < 1e-12, ...
            'round-trip lost curve_tension_alpha');
        ok(16);
    catch err
        fail(16, err);
    end

    % ── 17. §3.4 legacy M3-era file loads as straight mode ─────────
    try
        legacyPath = fullfile(tmpRoot, 'legacy_m3.json');
        legacy.description = 'Legacy M3 file';
        legacy.duration_s  = 120;
        legacyTarget = struct();
        legacyTarget.name        = 'legacy_m3';
        legacyTarget.label       = 'legacy_m3';
        legacyTarget.behavior    = 'waypoints';
        legacyTarget.rcs_dbsm    = 10;
        legacyTarget.rcs_profile = 'airliner';
        legacyWP(1).pos    = [0, 0, -1000];
        legacyWP(1).time_s = 0;
        legacyWP(2).pos    = [5000, 3000, -2000];
        legacyWP(2).time_s = 30;
        legacyWP(3).pos    = [12000, 6000, -1500];
        legacyWP(3).time_s = 75;
        legacyTarget.waypoints = legacyWP;
        legacy.targets = {legacyTarget};
        fid = fopen(legacyPath, 'w');
        fwrite(fid, jsonencode(legacy), 'char');
        fclose(fid);

        s = trackbench.editor.EditorState(projectRoot);
        s.curveMode         = "curved";  % dirty these to prove restore resets them
        s.curveTensionAlpha = 1.0;
        trackbench.editor.loadFromJSON(s, legacyPath);
        assert(size(s.waypoints, 1) == 3, ...
            'legacy file should yield 3 editor waypoints');
        assert(s.curveMode == "straight", ...
            'legacy file (no curve_mode) must load as straight');
        assert(abs(s.curveTensionAlpha - 0.5) < 1e-12, ...
            'legacy file should restore alpha to default 0.5');
        ok(17);
    catch err
        fail(17, err);
    end

    % ── 18. §3.4 straight-mode round-trip regression (M3 contract) ─
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
        s.targetName = "roundtrip_straight";
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        originalWP = s.waypoints;
        path = trackbench.editor.exportToJSON(s);

        s2 = trackbench.editor.EditorState(projectRoot);
        trackbench.editor.loadFromJSON(s2, path);
        assert(size(s2.waypoints, 1) == 3);
        assert(max(abs(s2.waypoints(:,1) - originalWP(:,1))) < 1e-6);
        assert(max(abs(s2.waypoints(:,2) - originalWP(:,2))) < 1e-6);
        assert(s2.curveMode == "straight");
        ok(18);
    catch err
        fail(18, err);
    end

    % ── 19. §3.5 preview would get densified Nx2 when curveMode==curved
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        s.addWaypoint(18000, 2000);
        s.waypoints(:,3) = [1000; 2000; 1500; 3000];
        s.curveMode = "curved";
        [densePts, denseT] = trackbench.editor.catmullRomCurve( ...
            s.waypoints(:, 1:5), s.curveDensityPerSeg, s.curveTensionAlpha);
        assert(size(densePts, 1) > size(s.waypoints, 1), ...
            'dense preview input should have more samples than controls');
        assert(all(isfinite(denseT)));
        assert(issorted(denseT), 'preview time must be sorted');
        assert(denseT(1) == s.waypoints(1, 4));
        assert(abs(denseT(end) - s.waypoints(end, 4)) < 1e-9);
        % interpPos applied to the dense 2D list at mid-time must land on
        % or very near the dense polyline — validates what previewWindow
        % will animate.
        midT = (denseT(1) + denseT(end)) / 2;
        pos = trackbench.editor.interpPos(densePts(:, 1:2), denseT, midT);
        assert(all(isfinite(pos)) && numel(pos) == 2);
        ok(19);
    catch err
        fail(19, err);
    end

    % ── 20. §3.4 curved export speeds are finite + positive ─────────
    try
        s = trackbench.editor.EditorState(projectRoot);
        s.outputDir = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
        s.targetName = "speed_check";
        s.addWaypoint(0,    0);
        s.addWaypoint(5000, 3000);
        s.addWaypoint(12000, 6000);
        s.waypoints(:,3) = [1000; 2000; 1500];
        s.curveMode = "curved";
        path = trackbench.editor.exportToJSON(s);
        decoded = jsondecode(fileread(path));
        tgt = normalizeTarget(decoded);
        for k = 1:numel(tgt.waypoints)
            w = ctrlStructAt(tgt.waypoints, k);
            assert(isfield(w, 'speed_kmh'), ...
                sprintf('dense waypoint %d missing speed_kmh', k));
            assert(isfinite(w.speed_kmh) && w.speed_kmh > 0, ...
                sprintf('dense waypoint %d has non-positive speed %.3f', ...
                        k, w.speed_kmh));
        end
        ok(20);
    catch err
        fail(20, err);
    end

    % ── Summary ─────────────────────────────────────────────────────
    total = nPass + nFail;
    fprintf('\n----\nPASSED %d/%d\n', nPass, total);
    if nFail > 0
        error('testPathEditor_M4:someFailed', '%d tests failed', nFail);
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


% =========================================================================
%  File-scope helpers (not nested — no access to nPass/nFail)
% =========================================================================
function tgt = normalizeTarget(decoded)
%normalizeTarget  Extract the first target from a decoded JSON document.
%                 Mirrors the tolerance of loadFromJSON so the tests don't
%                 depend on MATLAB's flip-flopping between cell and struct
%                 arrays.
    if isstruct(decoded) && isfield(decoded, 'targets')
        targets = decoded.targets;
    elseif iscell(decoded)
        targets = decoded;
    elseif isstruct(decoded) && isfield(decoded, 'behavior')
        tgt = decoded;
        return;
    else
        error('normalizeTarget: unrecognized decoded shape');
    end
    if iscell(targets)
        tgt = targets{1};
    elseif isstruct(targets)
        tgt = targets(1);
    else
        error('normalizeTarget: unrecognized targets shape');
    end
end


function w = ctrlStructAt(arr, k)
%ctrlStructAt  Index into a waypoints array regardless of whether
%              jsondecode produced a struct-array or cell-array.
    if iscell(arr)
        w = arr{k};
    else
        w = arr(k);
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
