function results = compareTrackers(runName, opts)
%compareTrackers  Compare an explicit list of tracker configs head-to-head.
%
%  Reads the run file's "compare_trackers" array — each entry pointing to a
%  tracker JSON config under config/trackers/ — and runs each one against
%  the cached detections from that run. Prints a ranked comparison table
%  sorted by composite score.
%
%  Unlike the older compareAllTrackers, this function makes NO assumptions
%  about which files to use. The user lists the exact files to compare in
%  the run file, so auto-tuned CV and IMM variants can both appear as
%  separate rows, along with any defaults you want as baselines.
%
%  USAGE
%    compareTrackers("range_rcs_test")
%    results = compareTrackers("my_run")
%
%  RUN FILE FORMAT
%    Add a "compare_trackers" array to any run file, e.g.:
%
%      "compare_trackers": [
%        "autotuned/my_run/GNN_IMM",
%        "autotuned/my_run/GNN_CV",
%        "autotuned/my_run/JPDA_IMM",
%        "GNN/default_GNN"
%      ]
%
%    Each entry is a path relative to config/trackers/, same convention
%    as the existing "trackers" field. The ".json" suffix is optional.
%
%  PREREQUISITES
%    - Cached detections must exist: run runSingleScenario("<run>") first
%      (or any run that produces cache/<run>.mat).
%    - Each listed tracker JSON must declare "tracker_type" (GNN/JPDA/TOMHT)
%      and "filter_model" (CV/IMM).
%
%  OPTS (optional struct)
%    .weights : [posRMS, swaps, falseTracks, breaks] weighting for score
%               (default [0.5, 0.25, 0.15, 0.10])
%
%  OUTPUT
%    results.table   : sorted comparison table (best first)
%    results.raw     : per-entry metric structs
%    results.runName : name of the run used
%
%  See also: autoTuneTracker, runSingleScenario, compareAllTrackers (legacy)

if nargin < 2; opts = struct(); end
if ~isfield(opts, 'weights'); opts.weights = [0.5, 0.25, 0.15, 0.10]; end

runName = string(runName);

fprintf('\n');
fprintf('================================================================\n');
fprintf('  TRACKER COMPARISON (explicit list from run file)\n');
fprintf('  Run: %s\n', runName);
fprintf('  %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('================================================================\n\n');

%% 1. Locate paths
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'src')));

runFileName = runName;
if ~endsWith(runFileName, ".json"); runFileName = runFileName + ".json"; end
runFilePath = fullfile(root, 'config', 'runs', runFileName);
if ~isfile(runFilePath)
    error('compareTrackers:runNotFound', ...
        'Run file not found: %s', runFilePath);
end

%% 2. Read "compare_trackers" list from the run file
runDef = jsondecode(fileread(runFilePath));
if ~isfield(runDef, 'compare_trackers') || isempty(runDef.compare_trackers)
    error('compareTrackers:noList', [ ...
        'Run file %s has no "compare_trackers" array.\n' ...
        'Add an array of tracker config paths, e.g.:\n' ...
        '  "compare_trackers": [\n' ...
        '      "autotuned/%s/GNN_IMM",\n' ...
        '      "autotuned/%s/GNN_CV",\n' ...
        '      "GNN/default_GNN"\n' ...
        '  ]'], runFilePath, runName, runName);
end

trkList = runDef.compare_trackers;
if ischar(trkList) || isstring(trkList)
    trkList = {char(trkList)};
elseif iscell(trkList)
    trkList = cellfun(@char, trkList, 'UniformOutput', false);
else
    trkList = cellstr(trkList);
end
nEntries = numel(trkList);
fprintf('[CMP] Comparing %d tracker config(s) from run file:\n', nEntries);
for i = 1:nEntries
    fprintf('        %d. %s\n', i, trkList{i});
end
fprintf('\n');

%% 3. Load cached detections
cacheFile = fullfile(root, 'cache', runName + ".mat");
if ~isfile(cacheFile)
    error('compareTrackers:noCachedData', [ ...
        'No cached detections at %s.\n' ...
        'Run the scenario first: runSingleScenario("%s")'], ...
        cacheFile, runName);
end
fprintf('[CMP] Loading cached detections...\n');
loaded = load(cacheFile);
if isfield(loaded, 'detections'); dataLog = loaded.detections;
elseif isfield(loaded, 'dataLog'); dataLog = loaded.dataLog;
else; error('compareTrackers:badCache', 'Cache file has no detections field.');
end
fprintf('[CMP] Loaded: %d scans, %d total detections\n\n', ...
    numel(dataLog.Time), sum(cellfun(@numel, dataLog.Detections)));

%% 4. Load environment (globals + sensor count + pd)
%   We only need the pieces that don't depend on tracker choice: the
%   detection probability, filter-init defaults, sensor count, and the
%   tracker globals (volume/beta/max_tracks). Individual tracker configs
%   will override volume/beta/params/filter per entry below.
[~, config, sensors, ~] = trackbench.config.loadRunFile(runName);
trkGlobalBase = config.tracker_global;
trkFilterBase = config.filter_params;
pd = config.active_params.pd;
nSensors = countSensors(sensors);
maxRange = 111120;  % 60 nm normalization for posRMS term in score
w = opts.weights;

%% 5. Run each listed tracker
entryResults = cell(nEntries, 1);

fprintf('  %-38s %-6s %-4s | %8s  %5s  %5s  %6s | %8s | Time\n', ...
    'Config', 'Type', 'Flt', 'posRMS', 'Swaps', 'False', 'Breaks', 'Score');
fprintf('  -------------------------------------- ------ ---- + --------  -----  -----  ------ + -------- + ----\n');

for i = 1:nEntries
    entryPath = trkList{i};
    entryLabel = entryPath;

    tStart = tic;
    try
        % Resolve JSON path under config/trackers/
        jsonRel = entryPath;
        if ~endsWith(jsonRel, '.json'); jsonRel = [jsonRel '.json']; end
        jsonFull = fullfile(root, 'config', 'trackers', jsonRel);
        if ~isfile(jsonFull)
            error('compareTrackers:cfgNotFound', 'Config not found: %s', jsonFull);
        end

        % Decode tracker config
        tc = jsondecode(fileread(jsonFull));
        if ~isfield(tc, 'tracker_type') || ~isfield(tc, 'filter_model')
            error('compareTrackers:badCfg', ...
                '%s is missing tracker_type or filter_model fields', jsonFull);
        end
        tType  = upper(char(string(tc.tracker_type)));
        fModel = upper(char(string(tc.filter_model)));

        % Resolve per-entry params (mirrors runSingleScenario merge logic)
        [params, gMod, fMod] = resolveEntryParams(tc, config, trkGlobalBase, trkFilterBase);

        % Build + run tracker (no visuals)
        tracker = trackbench.tracking.buildTracker(tType, fModel, params, ...
            gMod, fMod, pd, nSensors);
        [trackSummary, truthSummary, trackMetrics, ~] = ...
            trackbench.tracking.runTracker(dataLog, tracker, false, false, false);

        m = extractMetrics(trackSummary, truthSummary, trackMetrics);
        m.score = w(1) * (m.avgPosRMS / maxRange) + ...
                  w(2) * m.swapCount + ...
                  w(3) * log1p(m.falseTracks) + ...
                  w(4) * m.breakCount;
        m.configPath = entryPath;
        m.tracker = tType;
        m.model = fModel;
        m.elapsed = toc(tStart);

        entryResults{i} = m;
        fprintf('  %-38s %-6s %-4s | %7.0fm  %5d  %5d  %6d | %8.4f | %.1fs\n', ...
            truncLabel(entryLabel,38), tType, fModel, ...
            m.avgPosRMS, m.swapCount, m.falseTracks, m.breakCount, m.score, m.elapsed);

        clear tracker;
    catch ME
        m = struct('avgPosRMS', Inf, 'swapCount', 99, 'falseTracks', 99, ...
            'breakCount', 99, 'score', Inf, 'configPath', entryPath, ...
            'tracker', '?', 'model', '?', 'elapsed', toc(tStart), ...
            'error', ME.message);
        entryResults{i} = m;
        fprintf('  %-38s FAILED: %s\n', truncLabel(entryLabel,38), ME.message);
    end
end

%% 6. Build comparison table (sorted)
configs = cell(nEntries, 1);
trackers = cell(nEntries, 1);
models   = cell(nEntries, 1);
posRMS   = zeros(nEntries, 1);
swaps    = zeros(nEntries, 1);
falses   = zeros(nEntries, 1);
breaks   = zeros(nEntries, 1);
scores   = zeros(nEntries, 1);
times    = zeros(nEntries, 1);
for i = 1:nEntries
    r = entryResults{i};
    configs{i}  = r.configPath;
    trackers{i} = r.tracker;
    models{i}   = r.model;
    posRMS(i)   = r.avgPosRMS;
    swaps(i)    = r.swapCount;
    falses(i)   = r.falseTracks;
    breaks(i)   = r.breakCount;
    scores(i)   = r.score;
    times(i)    = r.elapsed;
end
compTable = table(configs, trackers, models, posRMS, swaps, falses, breaks, scores, times, ...
    'VariableNames', {'Config','Tracker','Model','posRMS_m','Swaps','FalseTracks','Breaks','Score','Time_s'});
compTable = sortrows(compTable, 'Score');

%% 7. Summary
fprintf('\n');
fprintf('================================================================\n');
fprintf('  FINAL RANKINGS (sorted by score, lower = better)\n');
fprintf('================================================================\n\n');
disp(compTable);

if ~isempty(compTable) && isfinite(compTable.Score(1))
    b = compTable(1,:);
    fprintf('  BEST: %s (%s+%s) -- posRMS=%.0fm, Score=%.4f\n', ...
        b.Config{1}, b.Tracker{1}, b.Model{1}, b.posRMS_m, b.Score);
end

fprintf('\n================================================================\n');
fprintf('  COMPARISON COMPLETE: %s\n', runName);
fprintf('================================================================\n\n');

results.table   = compTable;
results.raw     = entryResults;
results.runName = char(runName);

end


%% ========================================================================
%  HELPERS
%% ========================================================================
function [params, gMod, fMod] = resolveEntryParams(tc, config, gBase, fBase)
%resolveEntryParams  Apply a tracker config's overrides onto the globals.
%
%  Mirrors the merge logic in runSingleScenario so behavior matches:
%    - config.active_params is the starting point for params
%    - tc.params fields overlay onto active_params
%    - tc.volume / tc.beta overlay onto globals
%    - tc.filter fields overlay onto filter globals
    params = config.active_params;
    gMod   = gBase;
    fMod   = fBase;

    if isfield(tc, 'params') && isstruct(tc.params)
        names = fieldnames(tc.params);
        for i = 1:numel(names)
            params.(names{i}) = tc.params.(names{i});
        end
    end
    if isfield(tc, 'volume'); gMod.volume = tc.volume; end
    if isfield(tc, 'beta');   gMod.beta   = tc.beta;   end
    if isfield(tc, 'filter') && isstruct(tc.filter)
        names = fieldnames(tc.filter);
        for i = 1:numel(names)
            fMod.(names{i}) = tc.filter.(names{i});
        end
    end
end


function m = extractMetrics(trackSummary, truthSummary, trackMetrics)
    m.avgPosRMS = Inf;
    m.swapCount = 0;
    m.falseTracks = 0;
    m.breakCount = 0;

    if istable(trackMetrics) && ~isempty(trackMetrics)
        if ismember('posRMS', trackMetrics.Properties.VariableNames)
            valid = trackMetrics.posRMS(isfinite(trackMetrics.posRMS));
            if ~isempty(valid); m.avgPosRMS = mean(valid); end
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


function s = truncLabel(s, maxLen)
    s = char(s);
    if numel(s) > maxLen
        s = [s(1:maxLen-1) '...'];
    end
end


function n = countSensors(sensors)
    n = 0;
    if isempty(sensors) || ~isstruct(sensors); n = 1; return; end
    pNames = fieldnames(sensors);
    for i = 1:numel(pNames)
        n = n + numel(sensors.(pNames{i}));
    end
    n = max(n, 1);
end
