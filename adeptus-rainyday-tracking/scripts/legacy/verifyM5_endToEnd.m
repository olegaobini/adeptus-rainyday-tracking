function verifyM5_endToEnd()
%verifyM5_endToEnd  End-to-end sanity run for Milestone 5 (multi-target +
%                   reference targets in the path editor).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Mirrors the structure of verifyM4_endToEnd but exercises the new M5
%  surface area:
%
%    1. Build an EditorState, mutate the seed target into a NE-bound
%       airliner (straight, 3 waypoints, alt=10 km, RCS=15) and add a
%       SECOND target — a fighter on a centripetal-curved 5-waypoint
%       crossing path (alt=6 km, RCS=3, alpha=0.5).
%    2. Export via trackbench.editor.exportToJSON. Confirm the on-disk
%       schema has TWO entries in root.targets, the second is curved with
%       control_waypoints + dense waypoints, and excludedRefCount==0.
%    3. Round-trip with loadFromJSON in default ("replace") mode — both
%       targets restored, neither flagged readOnly, activeIdx==1.
%    4. Round-trip with loadFromJSON in "reference" mode — targets count
%       grows by 2, the appended pair are readOnly==true with sourceFile
%       populated.
%    5. Confirm `state.unloadAllReferences()` removes exactly the two
%       reference rows and leaves the writables alone.
%    6. Write config/runs/m5_multi_demo.json wired to PSR/default_PSR +
%       rural/default_rural + GNN/default_GNN.
%    7. Run the scenario via runSingleScenario("m5_multi_demo") with the
%       chained-call pause suppressed.
%    8. Save every open figure to ../PROGRESS_M5_screenshots/.
%
%  WHY MIRROR M4
%    Same harness, same screenshot folder convention, same fire-and-forget
%    runnability — so the existing PowerShell launcher pattern can drive
%    this script without changes.
%
%  OUTPUT ARTIFACTS (relative to project root)
%    config/targets/waypoints/m5_multi_demo.json     (multi-target export)
%    config/runs/m5_multi_demo.json                  (run file)
%    ../PROGRESS_M5_screenshots/<fig>.png            (saved figures)
%
%  See also: trackbench.editor.exportToJSON,
%            trackbench.editor.loadFromJSON,
%            trackbench.editor.EditorState,
%            runSingleScenario,
%            verifyM4_endToEnd

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));
    addpath(fullfile(projectRoot, 'scripts'));

    screenshotsDir = fullfile(fileparts(projectRoot), 'PROGRESS_M5_screenshots');
    if ~exist(screenshotsDir, 'dir')
        mkdir(screenshotsDir);
    end

    fprintf('\n==== verifyM5_endToEnd ====\n');
    fprintf('Project root : %s\n', projectRoot);
    fprintf('Screenshots  : %s\n', screenshotsDir);

    % ── Step 1: build a two-target scenario ─────────────────────────
    %  Seed target → airliner_ne (straight, 3 waypoints, NE bearing).
    %  Then addNewTarget → fighter_crosser (curved, 5 waypoints crossing
    %  the airliner's path roughly perpendicular).
    s = trackbench.editor.EditorState(projectRoot);
    s.description = "M5 §3.2 end-to-end: two-target scenario authored in the path editor.";

    % --- Target 1: airliner_ne (straight) ---
    s.setActiveIdx(1);
    s.targetName        = "airliner_ne";
    s.rcsDbsm           = 15;
    s.rcsProfile        = "airliner";
    s.defaultSpeedKmh   = 900;
    s.defaultAltitudeM  = 10000;
    s.curveMode         = "straight";

    airlinerWaypoints = [ ...
            0,     0, 10000;
        35000, 35000, 10000;
        70000, 70000, 10000];
    setTargetWaypoints(s, airlinerWaypoints);

    % --- Target 2: fighter_crosser (centripetal curve) ---
    s.addNewTarget();   % becomes active
    s.targetName        = "fighter_crosser";
    s.rcsDbsm           = 3;
    s.rcsProfile        = "fighter";
    s.defaultSpeedKmh   = 1500;
    s.defaultAltitudeM  = 6000;
    s.curveMode         = "curved";
    s.curveTensionAlpha = 0.5;

    fighterWaypoints = [ ...
        -20000,  30000, 6000;
              0,  20000, 6000;
         20000,  10000, 6000;
         40000,  -5000, 6000;
         60000, -20000, 6000];
    setTargetWaypoints(s, fighterWaypoints);

    fprintf('Targets in scene  : %d\n', numel(s.targets));
    for k = 1:numel(s.targets)
        tr = s.targets(k);
        fprintf('  [%d] %-20s  curve=%s  wp=%d  rcs=%g\n', ...
            k, char(tr.targetName), char(tr.curveMode), ...
            size(tr.waypoints,1), tr.rcsDbsm);
    end

    % ── Step 2: export and verify multi-target schema ───────────────
    [exported, excludedRefs] = trackbench.editor.exportToJSON(s, "m5_multi_demo");
    fprintf('Exported targets  : %s\n', exported);
    fprintf('Excluded refs     : %d (expected 0)\n', excludedRefs);
    assert(excludedRefs == 0, ...
        'no reference targets in the source state; export should exclude 0');

    decoded = jsondecode(fileread(exported));
    assert(isfield(decoded, 'targets'), ...
        'exported file must have a top-level "targets" array');

    % jsondecode returns either a cell array (heterogeneous structs) or a
    % struct array (homogeneous). Normalize to cell so indexing works.
    if iscell(decoded.targets)
        tgtCell = decoded.targets;
    else
        tgtCell = num2cell(decoded.targets);
    end
    assert(numel(tgtCell) == 2, ...
        sprintf('expected 2 targets in export, got %d', numel(tgtCell)));

    t1 = tgtCell{1};
    t2 = tgtCell{2};
    assert(strcmpi(t1.curve_mode, 'straight'), ...
        'target #1 (airliner_ne) should be straight');
    assert(strcmpi(t2.curve_mode, 'curved'), ...
        'target #2 (fighter_crosser) should be curved');
    assert(isfield(t2, 'control_waypoints'), ...
        'curved target must serialize control_waypoints');
    nDenseFighter = numel(t2.waypoints);
    nCtrlFighter  = numel(t2.control_waypoints);
    fprintf('Airliner wp       : %d\n', numel(t1.waypoints));
    fprintf('Fighter ctrl wp   : %d\n', nCtrlFighter);
    fprintf('Fighter dense wp  : %d (%.0fx densification)\n', ...
        nDenseFighter, nDenseFighter / nCtrlFighter);
    assert(nCtrlFighter == 5, ...
        sprintf('fighter should have 5 control waypoints, got %d', nCtrlFighter));

    % ── Step 3: round-trip in default (replace) mode ────────────────
    s2 = trackbench.editor.EditorState(projectRoot);
    trackbench.editor.loadFromJSON(s2, exported);
    assert(numel(s2.targets) == 2, ...
        'replace-mode load should produce exactly 2 targets');
    assert(~s2.targets(1).readOnly && ~s2.targets(2).readOnly, ...
        'replace-mode load should not flag any target readOnly');
    assert(s2.activeIdx == 1, 'replace-mode load should set activeIdx=1');
    fprintf('Round-trip (replace) OK : 2 targets, both writable, active=1\n');

    % ── Step 4: round-trip in reference mode (append + readOnly) ────
    trackbench.editor.loadFromJSON(s2, exported, "reference");
    assert(numel(s2.targets) == 4, ...
        sprintf('reference-mode load should append; expected 4, got %d', ...
        numel(s2.targets)));
    assert(s2.targets(3).readOnly && s2.targets(4).readOnly, ...
        'appended reference targets must be readOnly');
    assert(s2.targets(3).sourceFile ~= "" && s2.targets(4).sourceFile ~= "", ...
        'appended reference targets must have sourceFile populated');
    fprintf('Round-trip (reference) OK : appended 2 readOnly targets\n');

    % ── Step 5: unloadAllReferences strips refs only ────────────────
    nBefore = numel(s2.targets);
    s2.unloadAllReferences();
    nAfter = numel(s2.targets);
    assert(nAfter == 2, ...
        sprintf('unloadAllReferences should leave 2 writables; got %d', nAfter));
    assert(~any([s2.targets.readOnly]), ...
        'after unloadAllReferences no target should be readOnly');
    fprintf('unloadAllReferences OK : %d -> %d (refs removed)\n', nBefore, nAfter);

    % ── Step 6: write the run file ──────────────────────────────────
    %  Mirrors config/runs/m4_curved_demo.json so the rest of the sim
    %  stack is held constant — only the targets file differs.
    runDef = struct();
    runDef.description = char(['M5 §3.2 end-to-end test: two-target scenario ' ...
                               '(NE airliner straight + fighter centripetal-curve crosser) ' ...
                               'authored in the path editor.']);
    runDef.sensors     = {'PSR/default_PSR'};
    runDef.targets     = 'waypoints/m5_multi_demo';
    runDef.terrain     = 'rural/default_rural';
    runDef.trackers    = {'GNN/default_GNN'};
    runDef.degradation = struct( ...
        'terrain_occlusion', false, ...
        'horizon_masking',   false, ...
        'ground_clutter',    false, ...
        'doppler_fade',      false, ...
        'weather',           'none');
    runDef.cache   = struct('use_cached_detections', false, ...
                            'save_detections',       false);
    runDef.output  = struct('show_visuals',    true, ...
                            'animate_visuals', false, ...
                            'save_results',    false);

    runPath = fullfile(projectRoot, 'config', 'runs', 'm5_multi_demo.json');
    fid = fopen(runPath, 'w');
    fwrite(fid, jsonencode(runDef, 'PrettyPrint', true), 'char');
    fclose(fid);
    fprintf('Run file          : %s\n', runPath);

    % ── Step 7: run the scenario (suppress chained-call pause) ──────
    setappdata(0, 'trackbench_suppressPause', true);
    origCwd = pwd;
    cleaner = onCleanup(@() restoreState(origCwd));
    cd(projectRoot);

    fprintf('\n---- runSingleScenario("m5_multi_demo") ----\n');
    runSingleScenario("m5_multi_demo");
    fprintf('---- scenario complete ----\n\n');

    % ── Step 8: save every figure that's currently open ─────────────
    figs = findall(0, 'type', 'figure');
    fprintf('Figures to save : %d\n', numel(figs));
    for i = 1:numel(figs)
        f = figs(i);
        name = get(f, 'Name');
        if isempty(name)
            name = sprintf('figure_%d', i);
        end
        safeName = regexprep(char(name), '[^\w\-\.]+', '_');
        safeName = regexprep(safeName, '_+', '_');
        safeName = regexprep(safeName, '^_|_$', '');
        if isempty(safeName)
            safeName = sprintf('figure_%d', i);
        end
        outPath = fullfile(screenshotsDir, ...
            sprintf('m5_multi_%02d_%s.png', i, safeName));
        try
            exportgraphics(f, outPath, 'Resolution', 150);
            fprintf('  [save] %s\n', outPath);
        catch ME
            warning('verifyM5_endToEnd:saveFig', ...
                'Could not save fig %d (%s): %s', i, name, ME.message);
        end
    end

    fprintf('\nVERIFY DONE. Multi-target scenario ran and screenshots saved.\n');

    % Completion marker — independent of stdout/diary buffering so the
    % outside spawn poller can detect exit cleanly.
    markerPath = fullfile(screenshotsDir, '_VERIFY_DONE.marker');
    fid = fopen(markerPath, 'w');
    fprintf(fid, 'completed %s\nfigs=%d\n', char(datetime('now')), numel(figs));
    fclose(fid);
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function setTargetWaypoints(state, mat3)
%setTargetWaypoints  Bulk-load an Nx3 [x y alt] matrix onto the active
%                    target via addWaypoint + altitude assignment.
%
%  Uses the same pattern as verifyM4_endToEnd: per-row addWaypoint(x,y),
%  then write the explicit altitude. recomputeTimes() is called once at
%  the end so leg times reflect the updated geometry.
    for k = 1:size(mat3, 1)
        state.addWaypoint(mat3(k, 1), mat3(k, 2));
        state.waypoints(k, 3) = mat3(k, 3);
    end
    state.recomputeTimes();
end


function restoreState(origCwd)
%restoreState  onCleanup callback — restore pwd and clear suppress flag.
    setappdata(0, 'trackbench_suppressPause', false);
    try
        cd(origCwd);
    catch
    end
end
