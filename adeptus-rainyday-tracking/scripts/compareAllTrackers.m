function results = compareAllTrackers(runName, opts)
%compareAllTrackers  [LEGACY] Runs all 6 tracker×model combos on cached detections.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  NOTE: This is the legacy comparison function. It auto-discovers tracker
%  configs from config/trackers/autotuned/<run>/ and hardcodes the list of
%  combinations to 3 trackers × 2 filter models. It also silently uses the
%  IMM auto-tuned file for CV runs, which can be misleading.
%
%  For new work, prefer compareTrackers("<run>"), which reads an explicit
%  "compare_trackers" list from the run file so you control exactly which
%  configs (including separately auto-tuned CV and IMM variants) are
%  compared.
%
%  This function is preserved for backward-compat only and may be removed
%  in a future cleanup.
%
%  See also: compareTrackers, autoTuneTracker
%
%  Loads cached detections and runs GNN, JPDA, TOMHT with both CV and IMM
%  filters, producing a side-by-side comparison table. Uses auto-tuned
%  configs from config/trackers/autotuned/<run_name>/ where available,
%  otherwise falls back to default configs from the run file.
%
%  USAGE
%    results = compareAllTrackers("range_rcs_test")
%    results = compareAllTrackers("range_rcs_storm")
%
%  For 3D visualization, use the winning tracker config with runSingleScenario.
%
%  See also: autoTuneTracker, runSingleScenario

if nargin < 2; opts = struct(); end
if ~isfield(opts, 'weights'); opts.weights = [0.5, 0.25, 0.15, 0.10]; end

fprintf('\n');
fprintf('================================================================\n');
fprintf('  TRACKER x MODEL COMPARISON\n');
fprintf('  Run: %s\n', runName);
fprintf('  %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('================================================================\n\n');

%% 1. Load cached detections
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'src')));

cacheFile = fullfile(root, 'cache', runName + ".mat");
if ~isfile(cacheFile)
    error('No cached detections at %s.\nRun the scenario first: runSingleScenario("%s")', ...
        cacheFile, runName);
end

fprintf('[CMP] Loading cached detections...\n');
loaded = load(cacheFile);
if isfield(loaded, 'detections'); dataLog = loaded.detections;
elseif isfield(loaded, 'dataLog'); dataLog = loaded.dataLog;
else; error('Cache file has no detections field.');
end
fprintf('[CMP] Loaded: %d scans, %d total detections\n', ...
    numel(dataLog.Time), sum(cellfun(@numel, dataLog.Detections)));

%% 2. Load config
[~, config, sensors, ~] = trackbench.config.loadRunFile(runName);
trkGlobal = config.tracker_global;
trkFilter = config.filter_params;
pd = config.active_params.pd;
nSensors = countSensors(sensors);
maxRange = 111120;
w = opts.weights;

%% 3. Resolve auto-tuned configs for each tracker
%  Looks in config/trackers/autotuned/<run_name>/ for per-run tuned configs.
%  Falls back to config/trackers/autotuned/<run_name>/ generic, then defaults.
trackerTypes = {'GNN', 'JPDA', 'TOMHT'};
filterModels = {'IMM', 'CV'};

runFolder = strrep(char(runName), '/', '_');
autoDir = fullfile(root, 'config', 'trackers', 'autotuned', runFolder);

trackerConfigs = struct();
for t = 1:numel(trackerTypes)
    tType = trackerTypes{t};
    
    % New location: config/trackers/autotuned/<run>/<TYPE>_IMM.json
    % (loads IMM version as base config — works for both IMM and CV runs)
    autoFile = fullfile(autoDir, sprintf('%s_IMM.json', tType));
    
    % Legacy location: config/trackers/<TYPE>/autotuned_<TYPE>_<run>.json
    legacyFile = fullfile(root, 'config', 'trackers', tType, ...
        sprintf('autotuned_%s_%s.json', tType, runFolder));
    legacyGeneric = fullfile(root, 'config', 'trackers', tType, ...
        sprintf('autotuned_%s.json', tType));
    
    if isfile(autoFile)
        fprintf('[CMP] %s: auto-tuned (autotuned/%s/)\n', tType, runFolder);
        trackerConfigs.(tType) = jsondecode(fileread(autoFile));
    elseif isfile(legacyFile)
        fprintf('[CMP] %s: auto-tuned (legacy location)\n', tType);
        trackerConfigs.(tType) = jsondecode(fileread(legacyFile));
    elseif isfile(legacyGeneric)
        fprintf('[CMP] %s: auto-tuned (generic legacy)\n', tType);
        trackerConfigs.(tType) = jsondecode(fileread(legacyGeneric));
    else
        fprintf('[CMP] %s: no auto-tuned config -- using defaults\n', tType);
        trackerConfigs.(tType) = [];
    end
end

%% 4. Run all 6 combinations (no visuals)
nCombos = numel(trackerTypes) * numel(filterModels);
comboResults = cell(nCombos, 1);
idx = 0;

fprintf('\n[CMP] Running %d combinations...\n\n', nCombos);
fprintf('  %-8s %-5s | %8s  %5s  %5s  %6s | %8s | Time\n', ...
    'Tracker', 'Model', 'posRMS', 'Swaps', 'False', 'Breaks', 'Score');
fprintf('  -------- ----- + -------- -----  -----  ------ + -------- + ----\n');

for t = 1:numel(trackerTypes)
    tType = trackerTypes{t};
    
    for f = 1:numel(filterModels)
        fModel = filterModels{f};
        idx = idx + 1;
        
        tic;
        try
            [params, globalMod, filterMod] = resolveParams(tType, fModel, ...
                trackerConfigs.(tType), config, trkGlobal, trkFilter);
            
            tracker = trackbench.tracking.buildTracker(tType, fModel, params, ...
                globalMod, filterMod, pd, nSensors);
            [trackSummary, truthSummary, trackMetrics, ~] = ...
                trackbench.tracking.runTracker(dataLog, tracker, false, false, false);
            
            m = extractMetrics(trackSummary, truthSummary, trackMetrics);
            m.score = w(1) * (m.avgPosRMS / maxRange) + ...
                      w(2) * m.swapCount + ...
                      w(3) * log1p(m.falseTracks) + ...
                      w(4) * m.breakCount;
            m.tracker = tType;
            m.model = fModel;
            m.elapsed = toc;
            
            comboResults{idx} = m;
            
            fprintf('  %-8s %-5s | %7.0fm  %5d  %5d  %6d | %8.4f | %.1fs\n', ...
                tType, fModel, m.avgPosRMS, m.swapCount, m.falseTracks, ...
                m.breakCount, m.score, m.elapsed);
            
            clear tracker;
        catch ME
            m = struct('avgPosRMS', Inf, 'swapCount', 99, 'falseTracks', 99, ...
                'breakCount', 99, 'score', Inf, 'tracker', tType, ...
                'model', fModel, 'elapsed', toc);
            comboResults{idx} = m;
            fprintf('  %-8s %-5s | FAILED: %s\n', tType, fModel, ME.message);
        end
    end
end

%% 5. Build comparison table
trackers = cell(nCombos, 1);
models   = cell(nCombos, 1);
posRMS   = zeros(nCombos, 1);
swaps    = zeros(nCombos, 1);
falses   = zeros(nCombos, 1);
breaks   = zeros(nCombos, 1);
scores   = zeros(nCombos, 1);
times    = zeros(nCombos, 1);

for i = 1:nCombos
    r = comboResults{i};
    trackers{i} = r.tracker;
    models{i}   = r.model;
    posRMS(i)   = r.avgPosRMS;
    swaps(i)    = r.swapCount;
    falses(i)   = r.falseTracks;
    breaks(i)   = r.breakCount;
    scores(i)   = r.score;
    times(i)    = r.elapsed;
end

compTable = table(trackers, models, posRMS, swaps, falses, breaks, scores, times, ...
    'VariableNames', {'Tracker','Model','posRMS_m','Swaps','FalseTracks','Breaks','Score','Time_s'});
compTable = sortrows(compTable, 'Score');

%% 6. Summary
fprintf('\n');
fprintf('================================================================\n');
fprintf('  FINAL RANKINGS (sorted by score, lower = better)\n');
fprintf('================================================================\n\n');
disp(compTable);

bestRow = compTable(1,:);
fprintf('  BEST: %s + %s -- posRMS=%.0fm, Score=%.4f\n', ...
    bestRow.Tracker{1}, bestRow.Model{1}, bestRow.posRMS_m, bestRow.Score);

fprintf('\n  Best per tracker:\n');
for t = 1:numel(trackerTypes)
    tType = trackerTypes{t};
    mask = strcmp(compTable.Tracker, tType);
    subset = compTable(mask, :);
    if ~isempty(subset)
        b = subset(1,:);
        fprintf('    %-6s: %s -- posRMS=%.0fm (Score=%.4f)\n', ...
            tType, b.Model{1}, b.posRMS_m, b.Score);
    end
end

fprintf('\n  Motion model summary:\n');
immMask = strcmp(compTable.Model, 'IMM');
cvMask  = strcmp(compTable.Model, 'CV');
immAvg  = mean(compTable.posRMS_m(immMask));
cvAvg   = mean(compTable.posRMS_m(cvMask));
fprintf('    IMM avg posRMS: %.0fm\n', immAvg);
fprintf('    CV  avg posRMS: %.0fm\n', cvAvg);
if immAvg < cvAvg
    fprintf('    -> IMM wins overall (%.1f%% better)\n', (cvAvg-immAvg)/cvAvg*100);
else
    fprintf('    -> CV wins overall (%.1f%% better)\n', (immAvg-cvAvg)/immAvg*100);
end

fprintf('\n================================================================\n');
fprintf('  COMPARISON COMPLETE: %s\n', runName);
fprintf('================================================================\n\n');

%% Package output
results.table = compTable;
results.raw = comboResults;
results.runName = runName;

end


%% ========================================================================
%  PARAMETER RESOLUTION
%% ========================================================================
function [params, globalMod, filterMod] = resolveParams(trackerType, filterModel, ...
    autoConfig, config, trkGlobal, trkFilter)

    globalMod = trkGlobal;
    filterMod = trkFilter;
    params = config.active_params;
    
    if ~isempty(autoConfig) && isstruct(autoConfig)
        if isfield(autoConfig, 'volume')
            globalMod.volume = autoConfig.volume;
        end
        if isfield(autoConfig, 'beta')
            globalMod.beta = autoConfig.beta;
        end
        if isfield(autoConfig, 'params')
            pNames = fieldnames(autoConfig.params);
            for i = 1:numel(pNames)
                params.(pNames{i}) = autoConfig.params.(pNames{i});
            end
        end
        if isfield(autoConfig, 'filter')
            fNames = fieldnames(autoConfig.filter);
            for i = 1:numel(fNames)
                filterMod.(fNames{i}) = autoConfig.filter.(fNames{i});
            end
        end
    else
        switch upper(trackerType)
            case 'GNN'
                if ~isfield(params, 'gate'); params.gate = 100; end
                if ~isfield(params, 'far_gnn'); params.far_gnn = 1e-6; end
                if ~isfield(params, 'confirm_threshold'); params.confirm_threshold = 20; end
                if ~isfield(params, 'delete_threshold'); params.delete_threshold = -5; end
            case 'JPDA'
                if ~isfield(params, 'gate_jpda'); params.gate_jpda = 100; end
                if ~isfield(params, 'far_jpda'); params.far_jpda = 1e-6; end
                if ~isfield(params, 'beta_jpda'); params.beta_jpda = globalMod.beta; end
                if ~isfield(params, 'jpda_confirm_prob'); params.jpda_confirm_prob = 0.95; end
                if ~isfield(params, 'jpda_delete_prob'); params.jpda_delete_prob = 0.05; end
                if ~isfield(params, 'time_tolerance_jpda'); params.time_tolerance_jpda = 0.05; end
                if ~isfield(params, 'num_tracks_jpda'); params.num_tracks_jpda = 500; end
            case 'TOMHT'
                if ~isfield(params, 'gate'); params.gate = 100; end
                if ~isfield(params, 'far_mht'); params.far_mht = 1e-6; end
                if ~isfield(params, 'max_branches'); params.max_branches = 5; end
                if ~isfield(params, 'confirm_threshold'); params.confirm_threshold = 15; end
                if ~isfield(params, 'delete_threshold'); params.delete_threshold = -5; end
                if ~isfield(params, 'tomht_threshold_multiplier')
                    params.tomht_threshold_multiplier = [0.2, 1, 1];
                end
        end
    end
end


%% ========================================================================
%  METRICS EXTRACTION
%% ========================================================================
function m = extractMetrics(trackSummary, truthSummary, trackMetrics)
    m.avgPosRMS = Inf;
    m.swapCount = 0;
    m.falseTracks = 0;
    m.breakCount = 0;
    
    if istable(trackMetrics) && ~isempty(trackMetrics)
        if ismember('posRMS', trackMetrics.Properties.VariableNames)
            validRMS = trackMetrics.posRMS(isfinite(trackMetrics.posRMS));
            if ~isempty(validRMS)
                m.avgPosRMS = mean(validRMS);
            end
        end
    end
    
    if istable(trackSummary) && ~isempty(trackSummary)
        if ismember('SwapCount', trackSummary.Properties.VariableNames)
            m.swapCount = sum(trackSummary.SwapCount);
        end
        if ismember('AssignedTruthID', trackSummary.Properties.VariableNames)
            m.falseTracks = sum(isnan(trackSummary.AssignedTruthID));
        end
    end
    
    if istable(truthSummary) && ~isempty(truthSummary)
        if ismember('BreakCount', truthSummary.Properties.VariableNames)
            m.breakCount = sum(truthSummary.BreakCount);
        end
    end
end


%% ========================================================================
function n = countSensors(sensors)
    n = 0;
    if isempty(sensors) || ~isstruct(sensors); n = 1; return; end
    pNames = fieldnames(sensors);
    for i = 1:numel(pNames)
        n = n + numel(sensors.(pNames{i}));
    end
    n = max(n, 1);
end
