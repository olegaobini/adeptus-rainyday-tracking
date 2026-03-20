function runSingleScenario(runName)
% runSingleScenario  Run a modular simulation from a run file.
%
% USAGE
%   addpath("scripts");
%   runSingleScenario("my_run")              % your custom run file
%   runSingleScenario("dasr_baseline")       % PSR+SSR, rural, GNN+JPDA
%   runSingleScenario("demo_mountain")       % PSR, 5 targets, mountain
%   runSingleScenario("demo_first_run")      % Boeing demo, cached detections
%   runSingleScenario("fighter_intercept")   % AESA+FLIR on aircraft
%
% HOW IT WORKS
%   Reads config/runs/<runName>.json which references individual components:
%     sensors  → config/sensors/<TYPE>/  (one file per sensor)
%     targets  → config/targets/<PATTERN>/ (target definitions)
%     terrain  → config/terrain/<TYPE>/  (terrain + environment)
%     trackers → config/trackers/<TYPE>/ (tracker + filter params)
%
%   Run buildModularConfig to create the folder structure and templates.
%   See config/runs/run_template.json for the file format.
%
% See also: trackbench.config.loadRunFile, buildModularConfig

arguments
    runName (1,1) string = "my_run"
end

clc; close all;

%% Setup
root = resolveRootFromThisFile();
addpath(genpath(fullfile(root, "src")));

%% Validate run file exists
runPath = fullfile(root, "config", "runs", runName + ".json");
if ~isfile(runPath)
    fprintf('\n[ERROR] Run file not found: %s\n', runPath);
    fprintf('\nAvailable run files:\n');
    runsDir = fullfile(root, "config", "runs");
    files = dir(fullfile(runsDir, '*.json'));
    for i = 1:numel(files)
        [~, name] = fileparts(files(i).name);
        if ~startsWith(name, '_')
            fprintf('  runSingleScenario("%s")\n', name);
        end
    end
    fprintf('\nCreate a new run file: copy config/runs/run_template.json\n');
    fprintf('Or run buildModularConfig to regenerate defaults.\n\n');
    return;
end

%% Load via modular run file
[scenario, config, sensors, metas] = trackbench.config.loadRunFile(runName);
paths = buildPaths(root, config);
scenarioLabel = config.run_name;

%% Count sensors
platformNames = fieldnames(sensors);
totalSensors = 0;
for p = 1:numel(platformNames)
    totalSensors = totalSensors + numel(sensors.(platformNames{p}));
end

%% Build enabled tracker list for display
comboMap = {
    'gnn_cv','GNN','CV'; 'gnn_imm','GNN','IMM';
    'tomht_cv','TOMHT','CV'; 'tomht_imm','TOMHT','IMM';
    'jpda_cv','JPDA','CV'; 'jpda_imm','JPDA','IMM'
};
enabledTrackers = {};
for i = 1:size(comboMap, 1)
    key = comboMap{i, 1};
    if isfield(config.trackers_to_run, key) && ...
            (config.trackers_to_run.(key) == true || config.trackers_to_run.(key) == 1)
        enabledTrackers{end+1} = sprintf('%s+%s', comboMap{i,2}, comboMap{i,3}); %#ok<AGROW>
    end
end

%% Print run plan
sensorStr = config.sensor_config_name;
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              runSingleScenario — RUN PLAN               ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Run File  : %-44s║\n', scenarioLabel);
fprintf('║  Sensors   : %-44s║\n', sensorStr);
fprintf('║  Terrain   : %-44s║\n', config.environment.terrain_type);
fprintf('║  Degraded  : %-44s║\n', string(config.degradation.enabled));
fprintf('║  Trackers  : %-3d enabled                                 ║\n', numel(enabledTrackers));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  %s\n', strjoin(enabledTrackers, ', '));
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% Generate or load detections
dataLog = [];
if config.data_logging.use_saved_datalog
    dataLog = loadDetections(paths.data_log_file);
end

if isempty(dataLog)
    envCfg = config.environment;
    dataLog = trackbench.detections.runDetections( ...
        scenario, config.degradation.enabled, metas, envCfg);

    if config.data_logging.save_after_generation
        saveDetections(paths.data_log_file, dataLog);
    end
else
    totalSensors = max(totalSensors, 1);
end

if isfield(config, 'terrainGrid') && ~isempty(config.terrainGrid)
    dataLog.TerrainGrid = config.terrainGrid;
end

%% Scan check
numScans = numel(dataLog.Time);
if numScans < 5
    warning('runSingleScenario:fewScans', 'Only %d scan(s) — need at least 5.', numScans);
else
    fprintf('[SCAN CHECK] %d scans — OK.\n', numScans);
end

%% Visualization
showVis = true; animVis = true;
if isfield(config.output, 'show_visuals');    showVis = config.output.show_visuals;    end
if isfield(config.output, 'animate_visuals'); animVis = config.output.animate_visuals; end

if showVis
    trackbench.reporting.plotInitialScenario(dataLog, animVis);
end

%% Run trackers
params = config.active_params;
pd = params.pd;
results = struct();
results.run_id = scenarioLabel;
results.config = config;
summaryRows = {};

trackerCombos = {
    {'GNN','CV',config.trackers_to_run.gnn_cv};
    {'GNN','IMM',config.trackers_to_run.gnn_imm};
    {'TOMHT','CV',config.trackers_to_run.tomht_cv};
    {'TOMHT','IMM',config.trackers_to_run.tomht_imm};
    {'JPDA','CV',config.trackers_to_run.jpda_cv};
    {'JPDA','IMM',config.trackers_to_run.jpda_imm}
};

for c = 1:length(trackerCombos)
    tType = trackerCombos{c}{1}; fModel = trackerCombos{c}{2};
    enabled = trackerCombos{c}{3};
    if ~enabled; continue; end

    comboName = lower(sprintf('%s_%s', tType, fModel));
    fprintf('\n============ %s + %s ============\n', tType, fModel);

    % Use tracker-specific params if available
    trkParams = params;
    trkGlobal = config.tracker_global;
    trkFilter = config.filter_params;
    if isfield(config, 'tracker_configs')
        for tc = 1:numel(config.tracker_configs)
            tcfg = config.tracker_configs{tc};
            if strcmpi(tcfg.tracker_type, tType) && strcmpi(tcfg.filter_model, fModel)
                if isfield(tcfg, 'params'); trkParams = mergeStructs(trkParams, tcfg.params); end
                if isfield(tcfg, 'global'); trkGlobal = mergeStructs(trkGlobal, tcfg.global); end
                if isfield(tcfg, 'filter'); trkFilter = mergeStructs(trkFilter, tcfg.filter); end
                if isfield(tcfg, 'volume'); trkGlobal.volume = tcfg.volume; end
                if isfield(tcfg, 'beta');   trkGlobal.beta   = tcfg.beta;   end
                fprintf('  [%s] volume=%.0e, beta=%.0e\n', tType, trkGlobal.volume, trkGlobal.beta);
                break;
            end
        end
    end

    tracker = trackbench.tracking.buildTracker(tType, fModel, trkParams, ...
        trkGlobal, trkFilter, pd, totalSensors);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = ...
        trackbench.tracking.runTracker(dataLog, tracker, false, showVis, animVis);

    results.(comboName).trackSummary = trackSummary;
    results.(comboName).truthSummary = truthSummary;
    results.(comboName).trackMetrics = trackMetrics;
    results.(comboName).truthMetrics = truthMetrics;
    results.(comboName).time = time;
    results.(comboName).assignLog = assignLog;
    results.(comboName).swapReport = swapReport;

    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
    end

    swapStr = ternary(swapReport.swapFree, 'CLEAN', sprintf('%d SWAP(S)', swapReport.totalSwaps));
    posRMS_vals = []; velRMS_vals = [];
    if istable(trackMetrics) && ismember('posRMS', trackMetrics.Properties.VariableNames)
        posRMS_vals = trackMetrics.posRMS;
    end
    if istable(trackMetrics) && ismember('velRMS', trackMetrics.Properties.VariableNames)
        velRMS_vals = trackMetrics.velRMS;
    end

    fprintf('\n  ┌─────────────────────────────────────────────\n');
    fprintf('  │ %s + %s  SUMMARY\n', tType, fModel);
    fprintf('  ├─────────────────────────────────────────────\n');
    fprintf('  │ Track Swaps : %s\n', swapStr);
    if ~isempty(posRMS_vals)
        fprintf('  │ Position RMS: ');
        for ti = 1:numel(posRMS_vals); fprintf('T%d=%.1fm  ', ti, posRMS_vals(ti)); end
        fprintf('\n');
    end
    if ~isempty(velRMS_vals)
        fprintf('  │ Velocity RMS: ');
        for ti = 1:numel(velRMS_vals); fprintf('T%d=%.1fm/s  ', ti, velRMS_vals(ti)); end
        fprintf('\n');
    end
    if istable(trackMetrics) && ismember('Quality', trackMetrics.Properties.VariableNames)
        fprintf('  │ Quality     : ');
        for ti = 1:height(trackMetrics)
            fprintf('T%d=%.0f%%  ', ti, trackMetrics.Quality(ti));
        end
        fprintf('\n');
    end
    fprintf('  └─────────────────────────────────────────────\n');

    summaryRows{end+1} = {tType, fModel, swapStr, posRMS_vals, velRMS_vals}; %#ok<AGROW>
end

%% Final summary
if numel(summaryRows) >= 1
    fprintf('\n');
    fprintf('╔═══════════════════════════════════════════════════════╗\n');
    fprintf('║           %s — RESULTS SUMMARY             ║\n', upper(scenarioLabel));
    fprintf('╠═══════════════════════════════════════════════════════╣\n');
    fprintf('║  %-8s %-6s %-12s %-24s║\n', 'Tracker', 'Filter', 'Swaps', 'Avg posRMS (m)');
    fprintf('╠═══════════════════════════════════════════════════════╣\n');
    for ri = 1:numel(summaryRows)
        r = summaryRows{ri};
        avgPos = ternary(~isempty(r{4}), sprintf('%.1f', mean(r{4})), 'N/A');
        fprintf('║  %-8s %-6s %-12s %-24s║\n', r{1}, r{2}, r{3}, avgPos);
    end
    fprintf('╚═══════════════════════════════════════════════════════╝\n');
end

%% Save
if config.output.save_results
    if ~exist(paths.results_dir, "dir"); mkdir(paths.results_dir); end
    timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    resultsFile = fullfile(paths.results_dir, ...
        sprintf("results_%s_%s.mat", strrep(char(scenarioLabel),"/","_"), timestamp));
    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END | %s\n", scenarioLabel);
fprintf("==============================\n\n");

end


%% ========================================================================
%  HELPERS
%% ========================================================================
function root = resolveRootFromThisFile()
    thisFile = mfilename('fullpath');
    root = fileparts(fileparts(thisFile));
end

function paths = buildPaths(root, config)
    paths.root = root;
    paths.data_log_file = fullfile(root, config.data_logging.datalog_file);
    paths.results_dir   = fullfile(root, config.output.results_directory);
end

function detections = loadDetections(detectionsFile)
    if ~isfile(detectionsFile)
        detections = [];
        return;
    end
    loaded = load(detectionsFile);
    if isfield(loaded, "detections")
        detections = loaded.detections;
    elseif isfield(loaded, "dataLog")
        detections = loaded.dataLog;
    else
        detections = [];
    end
end

function saveDetections(detectionsFile, detections)
    detectionsDir = fileparts(detectionsFile);
    if ~exist(detectionsDir, "dir"); mkdir(detectionsDir); end
    save(detectionsFile, "detections", "-v7.3");
    fprintf("[INFO] Saved detections to %s\n", detectionsFile);
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function merged = mergeStructs(base, overlay)
    merged = base;
    if ~isstruct(overlay); return; end
    flds = fieldnames(overlay);
    for i = 1:numel(flds)
        merged.(flds{i}) = overlay.(flds{i});
    end
end
