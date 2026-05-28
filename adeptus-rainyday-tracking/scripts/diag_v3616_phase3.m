function diag_v3616_phase3()
%DIAG_V3616_PHASE3  v3.6.16 Phase 3 triage diagnostic for airborne IRST.
%
%   Author:  Michael Harding (Team Adeptus) / SENSORS chat
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Loads the latest results_test_IRST_airborne_*.mat from results/ and
%  prints info to disambiguate paths 1/2/3 per v3.6.16 SENSORS chat
%  triage matrix:
%
%    PATH 2 — Is dataLog.Truth in world frame? (BUGHUNT v3.7.2 verification)
%    PATH 3 — Are tracks in world frame? (best-effort from trackSummary)
%    PATH 1 — Truth/track positions to inform retune sizing
%
%  PREREQUISITES
%    1. config/runs/test_IRST_airborne.json: output.save_results = true
%    2. From MATLAB: runSingleScenario("test_IRST_airborne")
%
%  USAGE
%    From MATLAB with project root on path:  diag_v3616_phase3

    %% Locate latest results file
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    pattern = fullfile(projectRoot, 'results', 'results_test_IRST_airborne_*.mat');
    files = dir(pattern);
    if isempty(files)
        error('diag_v3616_phase3:noResults', ...
            ['No results files matching %s.\n' ...
             'Setup: (1) output.save_results = true in test_IRST_airborne.json,\n' ...
             '       (2) run runSingleScenario("test_IRST_airborne").'], pattern);
    end
    [~, idx] = max([files.datenum]);
    matPath = fullfile(files(idx).folder, files(idx).name);

    bar = repmat('=', 1, 72);
    fprintf('\n%s\n', bar);
    fprintf(' v3.6.16 PHASE 3 DIAGNOSTIC — airborne IRST Tracked%% triage\n');
    fprintf('%s\n', bar);
    fprintf('Loaded: %s\n', matPath);
    fprintf('  size=%.1f KB, modified %s\n', files(idx).bytes/1024, datestr(files(idx).datenum));

    S = load(matPath);

    %% Inspect saved fields
    fprintf('\n--- Saved fields ---\n');
    fn = fieldnames(S);
    for i = 1:numel(fn)
        try
            v = S.(fn{i});
            if isstruct(v) && numel(v) == 1
                fprintf('  %-22s struct(1x1) fields: %s\n', fn{i}, strjoin(fieldnames(v)', ', '));
            elseif istable(v)
                fprintf('  %-22s table [%dx%d] cols: %s\n', fn{i}, height(v), width(v), ...
                    strjoin(v.Properties.VariableNames, ', '));
            else
                fprintf('  %-22s %s [%s]\n', fn{i}, class(v), num2str(size(v)));
            end
        catch
            fprintf('  %-22s (inspection failed)\n', fn{i});
        end
    end

    %% Locate dataLog
    dataLog = locateDataLog(S);
    if isempty(dataLog)
        fprintf('\n[FATAL] No dataLog found in mat. Cannot perform Path 2 check.\n');
        fprintf('  Saved results may not include dataLog. Consider modifying the\n');
        fprintf('  save_results branch in runSingleScenario.m to include dataLog.\n\n');
        return;
    end

    %% PATH 2 — dataLog.Truth world-frame check
    fprintf('\n--- PATH 2 CHECK: dataLog.Truth world-frame ---\n');
    truthSz = size(dataLog.Truth);
    fprintf('  Truth shape: [%d truths x %d scans]\n', truthSz(1), truthSz(2));
    if isfield(dataLog, 'Time') && ~isempty(dataLog.Time)
        fprintf('  Time(1)=%.2fs  Time(end)=%.2fs  scans=%d\n', ...
            dataLog.Time(1), dataLog.Time(end), numel(dataLog.Time));
    end

    % Expected target world position: ir_airborne_demo target started at
    % [4000, 5000, -2800], heading 180° at 150 m/s. With aviation
    % convention cosd->N, sind->E: velocity = [-150, 0, 0] m/s in NED world.
    pStart  = [4000, 5000, -2800];
    vTarget = [-150,    0,     0];

    t1   = dataLog.Time(1);
    tEnd = dataLog.Time(end);
    expT1   = pStart + vTarget * t1;
    expTEnd = pStart + vTarget * tEnd;

    truthT1   = dataLog.Truth(1, 1).Position(:)';
    truthTEnd = dataLog.Truth(1, end).Position(:)';

    fprintf('\n  Truth(1, 1).Position    (t=%6.2fs): [%9.1f %9.1f %9.1f] m\n', t1,   truthT1);
    fprintf('  Expected (world frame):              [%9.1f %9.1f %9.1f] m\n',       expT1);
    fprintf('  Distance observed-vs-expected:       %.1f m\n', norm(truthT1 - expT1));

    fprintf('\n  Truth(1, end).Position  (t=%6.2fs): [%9.1f %9.1f %9.1f] m\n', tEnd, truthTEnd);
    fprintf('  Expected (world frame):              [%9.1f %9.1f %9.1f] m\n',       expTEnd);
    fprintf('  Distance observed-vs-expected:       %.1f m\n', norm(truthTEnd - expTEnd));

    %% Sensor world-position context (for path 3 disambiguation)
    fprintf('\n--- Sensor world context (informational) ---\n');
    % Ownship waypoint interp: [0,0,-3000] @ t=0 → [3000,0,-3000] @ t=15
    sensorT1 = [0, 0, -3000] + (t1 / 15) * [3000, 0, 0];
    fprintf('  Sensor world position at t=%.2fs (expected): [%9.1f %9.1f %9.1f] m\n', t1, sensorT1);
    fprintf('  Target relative to sensor (world frame):     [%9.1f %9.1f %9.1f] m\n', expT1 - sensorT1);
    fprintf('  Target-to-sensor range:                      %.1f m\n', norm(expT1 - sensorT1));

    %% trackSummary inspection (path 3 best-effort)
    fprintf('\n--- trackSummary inspection (path 3 best-effort) ---\n');
    trackSummary = [];
    if isfield(S, 'trackSummary')
        trackSummary = S.trackSummary;
    elseif isfield(S, 'results') && isstruct(S.results) && isfield(S.results, 'trackSummary')
        trackSummary = S.results.trackSummary;
    end
    if istable(trackSummary) && height(trackSummary) > 0
        fprintf('  Columns: %s\n', strjoin(trackSummary.Properties.VariableNames, ', '));
        fprintf('  First %d rows:\n', min(5, height(trackSummary)));
        disp(head(trackSummary, min(5, height(trackSummary))));
    else
        fprintf('  No trackSummary table found — Path 3 inconclusive from saved .mat.\n');
        fprintf('  (Per-scan track State requires runTracker.m instrumentation.)\n');
    end

    %% Synthesis
    fprintf('\n--- DIAGNOSIS SYNTHESIS ---\n');
    dWorld     = norm(truthT1 - expT1);
    dPlatLocal = norm(truthT1 - (expT1 - sensorT1));
    fprintf('  Truth(1,1) vs world-frame expected: %.1f m\n', dWorld);
    fprintf('  Truth(1,1) vs plat-local expected:  %.1f m\n', dPlatLocal);
    if dWorld < 100
        fprintf('  [PATH 2] dataLog.Truth IS in WORLD frame.\n');
        fprintf('           BUGHUNT v3.7.2 fix verified working for airborne scenario.\n');
        fprintf('           Failure mode is Path 1 (filter convergence) or Path 3 (track frame).\n');
        fprintf('\n  NEXT STEPS:\n');
        fprintf('   a. Cheaper: Path 1 retune. Extend test_IRST_airborne.json duration\n');
        fprintf('      to ~180s with sustained-maneuver waypoints. Target ~30 scans.\n');
        fprintf('   b. Disambiguate Path 1 vs 3: instrument runTracker.m for per-scan\n');
        fprintf('      track State logging, OR inspect interactive MATLAB workspace.\n');
    elseif dPlatLocal < 100
        fprintf('  [PATH 2] dataLog.Truth APPEARS plat-local frame.\n');
        fprintf('           BUGHUNT v3.7.2 fix did NOT apply to airborne scenario.\n');
        fprintf('           DISPATCH to BUGHUNT for v3.7.2 moving-platform regression.\n');
    else
        fprintf('  [PATH 2] dataLog.Truth frame AMBIGUOUS — neither world nor plat-local\n');
        fprintf('           matches expected. Investigate: rotation? scale? wrong truth index?\n');
    end

    fprintf('\n%s\n\n', bar);
end


function dataLog = locateDataLog(S)
%LOCATEDATALOG  Find a dataLog struct in saved results .mat, possibly nested.
    dataLog = [];
    if isfield(S, 'dataLog') && isstruct(S.dataLog) && isfield(S.dataLog, 'Truth')
        dataLog = S.dataLog; return;
    end
    fn = fieldnames(S);
    for i = 1:numel(fn)
        v = S.(fn{i});
        if isstruct(v) && numel(v) == 1
            if isfield(v, 'Truth') && isfield(v, 'Time')
                dataLog = v; return;
            end
            sf = fieldnames(v);
            for j = 1:numel(sf)
                try
                    w = v.(sf{j});
                    if isstruct(w) && isfield(w, 'Truth') && isfield(w, 'Time')
                        dataLog = w; return;
                    end
                catch
                end
            end
        end
    end
end
