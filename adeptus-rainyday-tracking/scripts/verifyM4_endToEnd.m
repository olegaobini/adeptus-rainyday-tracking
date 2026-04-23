function verifyM4_endToEnd()
%verifyM4_endToEnd  End-to-end sanity run for Milestone 4 (path editor curves).
%
%  Runs the COMPLETE path from editor state → exported JSON → loaded by
%  trackbench.config.loadRunFile → runSingleScenario consumes the dense
%  curve as the simulated target trajectory. Produces screenshots into
%      After Presentation/PROGRESS_M4_screenshots/
%  so the user can visually confirm the target flies a smooth path.
%
%  WHAT THIS SCRIPT DOES
%    1. Builds an EditorState with 5 control waypoints that describe a
%       sweeping 3D flight path (S-curve + altitude change).
%    2. Enables curved mode with the centripetal alpha (α=0.5) — the
%       default tension, and the one most users will hit first.
%    3. Exports via trackbench.editor.exportToJSON to
%          config/targets/waypoints/m4_curved_demo.json
%    4. Writes a run file at config/runs/m4_curved_demo.json referencing
%       PSR/default_PSR, rural terrain, GNN+CV tracker. Matches the
%       defaults in my_run.json so the result is comparable.
%    5. Re-opens the exported JSON with loadFromJSON to prove the
%       round-trip — curveMode should be "curved", α preserved, and
%       the 5 CONTROL waypoints restored (not the dense 201).
%    6. Runs runSingleScenario('m4_curved_demo') with visuals suppressed
%       so MATLAB's UI doesn't block, then re-enables figure rendering
%       for the screenshot pass.
%    7. Saves PNGs of the main figures under the screenshots folder.
%
%  OUTPUT ARTIFACTS (relative to project root)
%    config/targets/waypoints/m4_curved_demo.json     (editor export)
%    config/runs/m4_curved_demo.json                  (run file)
%    ../PROGRESS_M4_screenshots/<fig>.png             (saved figures)
%
%  See also: trackbench.editor.exportToJSON,
%            trackbench.editor.loadFromJSON,
%            runSingleScenario

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));
    addpath(fullfile(projectRoot, 'scripts'));

    screenshotsDir = fullfile(fileparts(projectRoot), 'PROGRESS_M4_screenshots');
    if ~exist(screenshotsDir, 'dir')
        mkdir(screenshotsDir);
    end

    fprintf('\n==== verifyM4_endToEnd ====\n');
    fprintf('Project root : %s\n', projectRoot);
    fprintf('Screenshots  : %s\n', screenshotsDir);

    % ── Step 1: editor state with control waypoints ─────────────────
    s = trackbench.editor.EditorState(projectRoot);
    s.targetName  = "m4_curved_demo";
    s.description = "M4.3 end-to-end curve test: 5-waypoint S-turn with altitude change.";
    s.rcsDbsm     = 10;
    s.rcsProfile  = "airliner";
    s.defaultSpeedKmh  = 900;
    s.defaultAltitudeM = 3000;

    % Five control points sketched to give a visibly curvy path.
    % x/y are in meters (east/north). The sim's default origin is the
    % radar site at (0,0). We place the aircraft roughly 20 km out and
    % fly it across the scope with two altitude changes and a heading
    % reversal — the centripetal spline will round those corners.
    controls = [ ...
        -18000, -10000, 2500;
         -8000,   4000, 3500;
          4000,   8000, 4000;
         14000,   2000, 3000;
         18000,  -8000, 2500 ];
    for k = 1:size(controls, 1)
        s.addWaypoint(controls(k, 1), controls(k, 2));
        s.waypoints(k, 3) = controls(k, 3);   % altitude
    end
    s.recomputeTimes();

    s.curveMode         = "curved";
    s.curveTensionAlpha = 0.5;
    fprintf('Control waypoints : %d\n', s.count());
    fprintf('Total duration    : %.1f s\n', s.waypoints(end, 4));

    % ── Step 2: export the curved-target JSON ───────────────────────
    exported = trackbench.editor.exportToJSON(s, "m4_curved_demo");
    fprintf('Exported target   : %s\n', exported);

    % Quick schema check: the exported file should have curve_mode
    % ="curved", control_waypoints with 5 entries, and a dense waypoints
    % array with (5-1)*50 + 1 = 201 entries.
    decoded = jsondecode(fileread(exported));
    if isstruct(decoded) && isfield(decoded, 'targets')
        if iscell(decoded.targets)
            tgt = decoded.targets{1};
        else
            tgt = decoded.targets(1);
        end
    else
        tgt = decoded;
    end
    assert(strcmpi(tgt.curve_mode, 'curved'), ...
        'exported file should tag curve_mode="curved"');
    nControls = numel(tgt.control_waypoints);
    nDense    = numel(tgt.waypoints);
    assert(nControls == 5, ...
        sprintf('expected 5 control waypoints, got %d', nControls));
    expectedDense = (nControls - 1) * s.curveDensityPerSeg + 1;
    assert(nDense == expectedDense, ...
        sprintf('expected %d dense waypoints, got %d', expectedDense, nDense));
    fprintf('Dense waypoints   : %d (%.0fx densification)\n', nDense, nDense/nControls);

    % ── Step 3: write the run file ──────────────────────────────────
    runDef = struct();
    runDef.description = char("M4 end-to-end test: simulator flies a curved editor path.");
    runDef.sensors     = {'PSR/default_PSR'};
    runDef.targets     = 'waypoints/m4_curved_demo';
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

    runPath = fullfile(projectRoot, 'config', 'runs', 'm4_curved_demo.json');
    fid = fopen(runPath, 'w');
    fwrite(fid, jsonencode(runDef, 'PrettyPrint', true), 'char');
    fclose(fid);
    fprintf('Run file          : %s\n', runPath);

    % ── Step 4: round-trip verify (control points survive) ──────────
    s2 = trackbench.editor.EditorState(projectRoot);
    trackbench.editor.loadFromJSON(s2, exported);
    assert(size(s2.waypoints, 1) == 5, ...
        'round-trip should restore 5 control points, not dense');
    assert(s2.curveMode == "curved", 'round-trip lost curveMode');
    assert(abs(s2.curveTensionAlpha - 0.5) < 1e-12);
    fprintf('Round-trip OK     : 5 controls, curveMode=curved, alpha=0.5\n');

    % ── Step 5: run the scenario (suppress chained-call pause) ──────
    % loadRunFile resolves paths relative to pwd, so cd to projectRoot
    % before calling. Restore the original cwd afterwards even on errors.
    setappdata(0, 'trackbench_suppressPause', true);
    origCwd = pwd;
    cleaner = onCleanup(@() restoreState(origCwd));  %#ok<NASGU>
    cd(projectRoot);

    fprintf('\n---- runSingleScenario("m4_curved_demo") ----\n');
    runSingleScenario("m4_curved_demo");
    fprintf('---- scenario complete ----\n\n');

    % ── Step 6: save every figure that's currently open ─────────────
    figs = findall(0, 'type', 'figure');
    fprintf('Figures to save : %d\n', numel(figs));
    for i = 1:numel(figs)
        f = figs(i);
        % Skip any figure without a name (usually transient / empty).
        name = get(f, 'Name');
        if isempty(name)
            name = sprintf('figure_%d', i);
        end
        % Sanitize the filename — strip characters that fail on Windows.
        safeName = regexprep(char(name), '[^\w\-\.]+', '_');
        safeName = regexprep(safeName, '_+', '_');
        safeName = regexprep(safeName, '^_|_$', '');
        if isempty(safeName)
            safeName = sprintf('figure_%d', i);
        end
        outPath = fullfile(screenshotsDir, ...
            sprintf('m4_curved_%02d_%s.png', i, safeName));
        try
            exportgraphics(f, outPath, 'Resolution', 150);
            fprintf('  [save] %s\n', outPath);
        catch ME
            warning('verifyM4_endToEnd:saveFig', ...
                'Could not save fig %d (%s): %s', i, name, ME.message);
        end
    end

    fprintf('\nVERIFY DONE. Curved-path scenario ran and screenshots saved.\n');
end


% ── helper: cleanup for onCleanup ─────────────────────────────────
function restoreState(origCwd)
    setappdata(0, 'trackbench_suppressPause', false);
    try
        cd(origCwd);
    catch
    end
end
