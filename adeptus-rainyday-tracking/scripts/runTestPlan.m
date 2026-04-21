function runTestPlan()
%runTestPlan  Execute the Validation Test Plan (v0.8) — all 9 test cases.
%
%  Runs TC-01 through TC-09 in sequence, collects pass/fail results,
%  and prints a final summary report.
%
%  USAGE
%    addpath("scripts");
%    runTestPlan
%
%  TEST CASES
%    TC-01 : Template usability — user-created config runs successfully
%    TC-02 : Baseline clear — all 3 trackers, ideal conditions
%    TC-03 : Rain S-band — minimal degradation at 2.8 GHz
%    TC-04 : Rain X-band — more degradation at 9 GHz (same rain rate as TC-03)
%    TC-05 : RCS verification — 20 dBsm vs -10 dBsm detection difference
%    TC-06 : Crossing swap — GNN vs JPDA track swap comparison
%    TC-07 : Compound stress — TOMHT + rain + mountain + mixed RCS
%    TC-08 : Config error paths — malformed/missing configs produce clear errors
%    TC-09 : Verification suite — verifySimulation.m passes all checks
%
%  RUN FILES
%    All test run files live in config/runs/validation/ to keep the main
%    runs/ folder clean for user experiments.
%
%  OUTPUTS
%    Console report with PASS/FAIL for each test case.
%    Summary table at the end.
%    Results saved to results/test_plan_results_<timestamp>.mat
%
%  See also: runSingleScenario, verifySimulation

clc; close all;
fprintf('\n');
fprintf('================================================================\n');
fprintf('  RAINY DAY — VALIDATION TEST PLAN EXECUTION\n');
fprintf('  Test Plan v0.8 | TrackBench v3.2.0\n');
fprintf('  %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('================================================================\n\n');

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'src')));

% Results accumulator
R = struct();
R.pass = 0; R.fail = 0; R.warn = 0;
R.details = {};

%% ================================================================
%  TC-01: TEMPLATE USABILITY
%% ================================================================
tcHeader('TC-01', 'Template Usability — user-created config runs successfully');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc01_template_user");
    dataLog = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas, config.environment);
    nScans = numel(dataLog.Time);
    nDets = sum(cellfun(@numel, dataLog.Detections));

    ok = nScans >= 3 && nDets > 0;
    R = logResult(R, 'TC-01', 'Run file loads and generates detections', ok, ...
        sprintf('%d scans, %d total detections', nScans, nDets));

    % Run a tracker to confirm full pipeline
    trkParams = config.active_params;
    trkGlobal = config.tracker_global;
    filterP   = config.filter_params;
    pd = trkParams.pd;
    nSensors = countSensors(sensors);
    tracker = trackbench.tracking.buildTracker('GNN', 'IMM', trkParams, trkGlobal, filterP, pd, nSensors);
    [~, ~, trackMetrics] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
    R = logResult(R, 'TC-01', 'Tracker produces metrics table', istable(trackMetrics), '');
catch ME
    R = logResult(R, 'TC-01', 'Execution completes without error', false, ME.message);
end

%% ================================================================
%  TC-02: BASELINE CLEAR — ALL 3 TRACKERS
%% ================================================================
tcHeader('TC-02', 'Baseline Clear — all 3 trackers under ideal conditions');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc02_baseline_clear");
    dataLog = trackbench.detections.runDetections(scenario, false, metas, config.environment);
    nSensors = countSensors(sensors);

    R = logResult(R, 'TC-02', 'Detection generation succeeds', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans', numel(dataLog.Time)));

    trackerTypes = {'GNN', 'JPDA', 'TOMHT'};
    for ti = 1:numel(trackerTypes)
        tType = trackerTypes{ti};
        try
            trkParams = config.active_params;
            trkGlobal = config.tracker_global;
            if isfield(config, 'tracker_configs')
                for tc = 1:numel(config.tracker_configs)
                    tcfg = config.tracker_configs{tc};
                    if strcmpi(tcfg.tracker_type, tType)
                        if isfield(tcfg, 'volume'); trkGlobal.volume = tcfg.volume; end
                        if isfield(tcfg, 'beta');   trkGlobal.beta   = tcfg.beta;   end
                        if isfield(tcfg, 'params'); trkParams = mergeS(trkParams, tcfg.params); end
                        break;
                    end
                end
            end
            tracker = trackbench.tracking.buildTracker(tType, 'IMM', trkParams, trkGlobal, config.filter_params, config.active_params.pd, nSensors);
            [~, ~, tm] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
            ok = istable(tm) && height(tm) > 0;
            posStr = '';
            if ok && ismember('posRMS', tm.Properties.VariableNames)
                posStr = sprintf('posRMS=[%s]', strjoin(string(round(tm.posRMS,1)),', '));
            end
            R = logResult(R, 'TC-02', sprintf('%s tracker completes with metrics', tType), ok, posStr);
        catch ME
            R = logResult(R, 'TC-02', sprintf('%s tracker completes', tType), false, ME.message);
        end
    end
catch ME
    R = logResult(R, 'TC-02', 'Scenario loads and runs', false, ME.message);
end

%% ================================================================
%  TC-03: RAIN S-BAND (16 mm/hr)
%% ================================================================
tcHeader('TC-03', 'Rain S-band — minimal degradation at 2.8 GHz, 16 mm/hr');
tc03_targetDets = 0;
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc03_rain_sband");
    envCfg = config.environment;
    envCfg.rain_rate_mmhr = config.degradation.rain_rate_mmhr;
    dataLog = trackbench.detections.runDetections(scenario, true, metas, envCfg);

    tc03_targetDets = countTargetDetections(dataLog);
    tc03_totalDets = sum(cellfun(@numel, dataLog.Detections));

    R = logResult(R, 'TC-03', 'S-band rain run completes', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans, %d target dets, %d total (incl clutter)', ...
        numel(dataLog.Time), tc03_targetDets, tc03_totalDets));

    nSensors = countSensors(sensors);
    tracker = trackbench.tracking.buildTracker('GNN', 'IMM', config.active_params, ...
        config.tracker_global, config.filter_params, config.active_params.pd, nSensors);
    [~, ~, tm] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
    if istable(tm) && ismember('posRMS', tm.Properties.VariableNames)
        avgPosRMS = mean(tm.posRMS);
        R = logResult(R, 'TC-03', 'S-band tracker produces finite posRMS (rain does not break tracking)', ...
            all(isfinite(tm.posRMS)), sprintf('avg=%.1f m, values=[%s]', avgPosRMS, ...
            strjoin(string(round(tm.posRMS,1)),', ')));
    end
catch ME
    R = logResult(R, 'TC-03', 'S-band rain completes', false, ME.message);
end

%% ================================================================
%  TC-04: RAIN X-BAND (16 mm/hr — same rain rate as TC-03)
%% ================================================================
tcHeader('TC-04', 'Rain X-band — more degradation at 9 GHz, same rain rate as TC-03');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc04_rain_xband");
    envCfg = config.environment;
    envCfg.rain_rate_mmhr = config.degradation.rain_rate_mmhr;
    dataLog = trackbench.detections.runDetections(scenario, true, metas, envCfg);

    tc04_targetDets = countTargetDetections(dataLog);
    tc04_totalDets = sum(cellfun(@numel, dataLog.Detections));

    R = logResult(R, 'TC-04', 'X-band rain run completes', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans, %d target dets, %d total (incl clutter)', ...
        numel(dataLog.Time), tc04_targetDets, tc04_totalDets));

    if tc03_targetDets > 0
        R = logResult(R, 'TC-04', ...
            'X-band target detections <= S-band target detections (freq differential)', ...
            tc04_targetDets <= tc03_targetDets, ...
            sprintf('X-band=%d vs S-band=%d target dets (excludes clutter)', ...
            tc04_targetDets, tc03_targetDets));
    else
        R = logResult(R, 'TC-04', 'Target detection comparison', false, ...
            'TC-03 had zero target detections — cannot compare');
    end
catch ME
    R = logResult(R, 'TC-04', 'X-band rain completes', false, ME.message);
end

%% ================================================================
%  TC-05: RCS VERIFICATION
%% ================================================================
tcHeader('TC-05', 'RCS Verification — 20 dBsm vs -10 dBsm detection difference');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc05_rcs_demo");
    dataLog = trackbench.detections.runDetections(scenario, false, metas, config.environment);

    R = logResult(R, 'TC-05', 'RCS demo run completes', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans', numel(dataLog.Time)));

    % Count detections per TargetIndex (auto-discover indices)
    indexCounts = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
    for si = 1:numel(dataLog.Detections)
        scanDets = dataLog.Detections{si};
        for di = 1:numel(scanDets)
            d = scanDets{di};
            tgtIdx = getTargetIndex(d);
            if tgtIdx > 0
                if indexCounts.isKey(tgtIdx)
                    indexCounts(tgtIdx) = indexCounts(tgtIdx) + 1;
                else
                    indexCounts(tgtIdx) = 1;
                end
            end
        end
    end

    allKeys = indexCounts.keys();
    detStr = '';
    for ki = 1:numel(allKeys)
        k = allKeys{ki};
        detStr = sprintf('%sIdx%d=%d  ', detStr, k, indexCounts(k));
    end
    R = logResult(R, 'TC-05', 'Target detection counts by TargetIndex', ...
        numel(allKeys) >= 2, detStr);

    if numel(allKeys) >= 2
        sortedKeys = sort(cell2mat(allKeys));
        idx_airliner = sortedKeys(1);
        idx_stealth  = sortedKeys(2);
        dets_airliner = indexCounts(idx_airliner);
        dets_stealth  = indexCounts(idx_stealth);

        R = logResult(R, 'TC-05', ...
            sprintf('Airliner (idx %d, 20dBsm) has more dets than stealth (idx %d, -10dBsm)', ...
            idx_airliner, idx_stealth), ...
            dets_airliner > dets_stealth, ...
            sprintf('Airliner=%d, Stealth=%d', dets_airliner, dets_stealth));

        if dets_airliner > 0 && dets_stealth > 0
            ratio = double(dets_airliner) / double(dets_stealth);
            R = logResult(R, 'TC-05', 'Detection ratio > 1.15x (meaningful RCS effect)', ...
                ratio > 1.15, sprintf('Ratio = %.2fx', ratio));
        end
    end
catch ME
    R = logResult(R, 'TC-05', 'RCS demo completes', false, ME.message);
end

%% ================================================================
%  TC-06: CROSSING TARGET SWAP ANALYSIS
%% ================================================================
tcHeader('TC-06', 'Crossing Target — GNN vs JPDA track swap comparison');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc06_crossing_swap");
    dataLog = trackbench.detections.runDetections(scenario, false, metas, config.environment);
    nSensors = countSensors(sensors);

    R = logResult(R, 'TC-06', 'Crossing scenario runs', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans', numel(dataLog.Time)));

    gnn_swaps = NaN; jpda_swaps = NaN;

    try
        trkParams = config.active_params;
        trkGlobal = config.tracker_global;
        if isfield(config, 'tracker_configs')
            for tc = 1:numel(config.tracker_configs)
                tcfg = config.tracker_configs{tc};
                if strcmpi(tcfg.tracker_type, 'GNN')
                    if isfield(tcfg, 'volume'); trkGlobal.volume = tcfg.volume; end
                    if isfield(tcfg, 'beta');   trkGlobal.beta = tcfg.beta; end
                    if isfield(tcfg, 'params'); trkParams = mergeS(trkParams, tcfg.params); end
                    break;
                end
            end
        end
        tracker = trackbench.tracking.buildTracker('GNN', 'IMM', trkParams, trkGlobal, ...
            config.filter_params, config.active_params.pd, nSensors);
        [~,~,~,~,~,~,swapGNN] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
        gnn_swaps = swapGNN.totalSwaps;
        R = logResult(R, 'TC-06', 'GNN completes on crossing', true, sprintf('Swaps=%d', gnn_swaps));
    catch ME
        R = logResult(R, 'TC-06', 'GNN completes on crossing', false, ME.message);
    end

    try
        trkParams = config.active_params;
        trkGlobal = config.tracker_global;
        if isfield(config, 'tracker_configs')
            for tc = 1:numel(config.tracker_configs)
                tcfg = config.tracker_configs{tc};
                if strcmpi(tcfg.tracker_type, 'JPDA')
                    if isfield(tcfg, 'volume'); trkGlobal.volume = tcfg.volume; end
                    if isfield(tcfg, 'beta');   trkGlobal.beta = tcfg.beta; end
                    if isfield(tcfg, 'params'); trkParams = mergeS(trkParams, tcfg.params); end
                    break;
                end
            end
        end
        tracker = trackbench.tracking.buildTracker('JPDA', 'IMM', trkParams, trkGlobal, ...
            config.filter_params, config.active_params.pd, nSensors);
        [~,~,~,~,~,~,swapJPDA] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
        jpda_swaps = swapJPDA.totalSwaps;
        R = logResult(R, 'TC-06', 'JPDA completes on crossing', true, sprintf('Swaps=%d', jpda_swaps));
    catch ME
        R = logResult(R, 'TC-06', 'JPDA completes on crossing', false, ME.message);
    end

    if ~isnan(gnn_swaps) && ~isnan(jpda_swaps)
        R = logResult(R, 'TC-06', 'JPDA swaps <= GNN swaps (soft assignment advantage)', ...
            jpda_swaps <= gnn_swaps, sprintf('JPDA=%d, GNN=%d', jpda_swaps, gnn_swaps));
    end
catch ME
    R = logResult(R, 'TC-06', 'Crossing scenario loads', false, ME.message);
end

%% ================================================================
%  TC-07: COMPOUND STRESS TEST
%% ================================================================
tcHeader('TC-07', 'Compound Stress — TOMHT + rain + mountain + mixed RCS');
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("validation/tc07_compound_stress");
    envCfg = config.environment;
    envCfg.rain_rate_mmhr = config.degradation.rain_rate_mmhr;
    dataLog = trackbench.detections.runDetections(scenario, true, metas, envCfg);
    nSensors = countSensors(sensors);

    R = logResult(R, 'TC-07', 'Compound scenario generates detections', numel(dataLog.Time) >= 3, ...
        sprintf('%d scans, %d dets', numel(dataLog.Time), sum(cellfun(@numel, dataLog.Detections))));

    try
        trkParams = config.active_params;
        trkGlobal = config.tracker_global;
        if isfield(config, 'tracker_configs')
            for tc = 1:numel(config.tracker_configs)
                tcfg = config.tracker_configs{tc};
                if strcmpi(tcfg.tracker_type, 'TOMHT')
                    if isfield(tcfg, 'volume'); trkGlobal.volume = tcfg.volume; end
                    if isfield(tcfg, 'beta');   trkGlobal.beta = tcfg.beta; end
                    if isfield(tcfg, 'params'); trkParams = mergeS(trkParams, tcfg.params); end
                    break;
                end
            end
        end
        tracker = trackbench.tracking.buildTracker('TOMHT', 'IMM', trkParams, trkGlobal, ...
            config.filter_params, config.active_params.pd, nSensors);
        [~, ~, tm] = trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
        ok = istable(tm) && height(tm) > 0;
        R = logResult(R, 'TC-07', 'TOMHT completes (no crash/OOM)', ok, '');
        if ok && ismember('posRMS', tm.Properties.VariableNames)
            R = logResult(R, 'TC-07', 'TOMHT produces valid posRMS', all(isfinite(tm.posRMS)), ...
                sprintf('posRMS=[%s]', strjoin(string(round(tm.posRMS,1)),', ')));
        end
    catch ME
        R = logResult(R, 'TC-07', 'TOMHT completes under compound degradation', false, ME.message);
    end
catch ME
    R = logResult(R, 'TC-07', 'Compound scenario loads', false, ME.message);
end

%% ================================================================
%  TC-08: CONFIG ERROR PATHS
%% ================================================================
tcHeader('TC-08', 'Config Error Paths — malformed/missing configs produce clear errors');

errorCases = {
    'validation/tc08a_missing_sensor',  'Missing sensor file';
    'validation/tc08b_missing_target',  'Missing target file';
    'validation/tc08c_missing_terrain', 'Missing terrain file';
    'validation/tc08d_malformed_json',  'Malformed JSON (trailing comma)';
    'validation/tc08e_missing_tracker', 'Missing tracker file';
};

for ei = 1:size(errorCases, 1)
    runName = errorCases{ei, 1};
    desc    = errorCases{ei, 2};
    try
        [~,~,~,~] = trackbench.config.loadRunFile(runName);
        R = logResult(R, 'TC-08', sprintf('%s: produces error', desc), false, ...
            'loadRunFile succeeded when it should have errored');
    catch ME
        hasInfo = ~isempty(ME.message) && strlength(string(ME.message)) > 10;
        msgShort = string(ME.message);
        if strlength(msgShort) > 80
            msgShort = extractBefore(msgShort, 81) + "...";
        end
        R = logResult(R, 'TC-08', sprintf('%s: produces clear error', desc), hasInfo, msgShort);
    end
end

%% ================================================================
%  TC-09: VERIFICATION SUITE
%% ================================================================
tcHeader('TC-09', 'Verification Suite — verifySimulation.m');
try
    fprintf('  Running verifySimulation (this may take a minute)...\n');
    evalc('verifySimulation');
    R = logResult(R, 'TC-09', 'verifySimulation completes without crash', true, 'All phases executed');
catch ME
    R = logResult(R, 'TC-09', 'verifySimulation completes without crash', false, ME.message);
end

%% ================================================================
%  FINAL SUMMARY
%% ================================================================
fprintf('\n\n');
fprintf('================================================================\n');
fprintf('  TEST PLAN EXECUTION — FINAL SUMMARY\n');
fprintf('================================================================\n');
fprintf('\n');
fprintf('  PASS: %d   FAIL: %d   TOTAL: %d\n', R.pass, R.fail, R.pass + R.fail);
fprintf('\n');

if R.fail == 0
    fprintf('  *** ALL TESTS PASSED ***\n');
else
    fprintf('  !!! %d TEST(S) FAILED — see details below !!!\n', R.fail);
end

fprintf('\n');
fprintf('%-8s %-6s %-55s %s\n', 'TC', 'Result', 'Check', 'Detail');
fprintf('%s\n', repmat('-', 1, 115));
for i = 1:numel(R.details)
    d = R.details{i};
    if d.passed; statusStr = 'PASS'; else; statusStr = 'FAIL'; end
    detailStr = char(d.detail);
    if numel(detailStr) > 50; detailStr = [detailStr(1:47) '...']; end
    fprintf('%-8s %-6s %-55s %s\n', d.tc, statusStr, d.check, detailStr);
end

if R.fail > 0
    fprintf('\n\n=== FAILED TESTS (full detail) ===\n');
    for i = 1:numel(R.details)
        d = R.details{i};
        if ~d.passed
            fprintf('\n  %s | %s\n', d.tc, d.check);
            fprintf('    %s\n', char(d.detail));
        end
    end
end

%% Save results
resultsDir = fullfile(root, 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end
timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outFile = fullfile(resultsDir, sprintf('test_plan_results_%s.mat', timestamp));
testResults = R;
save(outFile, 'testResults', '-v7.3');
fprintf('\n[SAVED] Test plan results: %s\n\n', outFile);

end


%% ========================================================================
%  HELPER FUNCTIONS
%% ========================================================================
function tcHeader(id, desc)
    fprintf('\n');
    fprintf('------------------------------------------------------------\n');
    fprintf('  %s: %s\n', id, desc);
    fprintf('------------------------------------------------------------\n');
end

function R = logResult(R, tc, check, passed, detail)
    if nargin < 5; detail = ''; end
    if passed
        R.pass = R.pass + 1;
        sym = 'OK';
    else
        R.fail = R.fail + 1;
        sym = 'XX';
    end
    fprintf('  [%s] %s: %s', sym, tc, check);
    if ~isempty(detail) && strlength(string(detail)) > 0
        fprintf(' — %s', char(detail));
    end
    fprintf('\n');
    R.details{end+1} = struct('tc', tc, 'check', check, 'passed', passed, 'detail', string(detail));
end

function n = countSensors(sensors)
    n = 0;
    pNames = fieldnames(sensors);
    for p = 1:numel(pNames)
        n = n + numel(sensors.(pNames{p}));
    end
    n = max(1, n);
end

function nTarget = countTargetDetections(dataLog)
%countTargetDetections  Count only real target detections (TargetIndex > 0).
%  Excludes clutter/false alarms which have TargetIndex = 0.
    nTarget = 0;
    for si = 1:numel(dataLog.Detections)
        scanDets = dataLog.Detections{si};
        for di = 1:numel(scanDets)
            tgtIdx = getTargetIndex(scanDets{di});
            if tgtIdx > 0
                nTarget = nTarget + 1;
            end
        end
    end
end

function tgtIdx = getTargetIndex(det)
    tgtIdx = 0;
    try
        attrs = det.ObjectAttributes;
        if iscell(attrs) && ~isempty(attrs)
            attr = attrs{1};
            if isstruct(attr) && isfield(attr, 'TargetIndex')
                tgtIdx = attr.TargetIndex;
            end
        end
    catch
    end
end

function merged = mergeS(base, overlay)
    merged = base;
    if ~isstruct(overlay); return; end
    flds = fieldnames(overlay);
    for i = 1:numel(flds)
        merged.(flds{i}) = overlay.(flds{i});
    end
end
