function runSingleScenario(configName)
%runSingleScenario Run exactly one prepared config.
%
% Usage:
%   runSingleScenario("scenarios/my_test")

arguments
    configName (1,1) string = "scenarios/my_test"
end

clc; close all;
ctx = setupTrackbench();

runPlan = trackbench.loader.loadAndPrepare(configName);
if numel(runPlan) ~= 1
    error('runSingleScenario:expectedSingle', ...
        '"%s" expands to %d runs. Use runParameterSweep for sweeps.', ...
        configName, numel(runPlan));
end

config = runPlan(1);
paths = buildPaths(ctx.root, config);

fprintf("\n==============================\n");
fprintf(" RUN START | %s\n", config.run_id);
fprintf("==============================\n\n");

detections = [];
if isfield(config, 'data_logging') && isfield(config.data_logging, 'use_saved_datalog') && config.data_logging.use_saved_datalog
    detections = loadDetections(paths.data_log_file);
end

if isempty(detections)
    [results, detections] = trackbench.runScenario(config, string(config.run_id));
    if isfield(config, 'data_logging') && isfield(config.data_logging, 'save_after_generation') && config.data_logging.save_after_generation
        saveDetections(paths.data_log_file, detections);
    end
else
    [results, ~] = trackbench.runScenario(config, string(config.run_id), detections);
end

showVis = getOr(config, 'output.show_visuals', true);
animVis = getOr(config, 'output.animate_visuals', true);
if showVis
    trackbench.reporting.plotInitialScenario(detections, animVis);
end

if getOr(config, 'output.save_results', true)
    if ~exist(paths.results_dir, "dir")
        mkdir(paths.results_dir);
    end
    timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    resultsFile = fullfile(paths.results_dir, sprintf("results_%s_%s.mat", config.run_id, timestamp));
    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END\n");
fprintf("==============================\n\n");
end

function paths = buildPaths(root, config)
paths.root = root;
paths.results_dir = fullfile(root, getOr(config, 'output.results_directory', 'outputs'));
paths.data_log_file = fullfile(root, getOr(config, 'data_logging.datalog_file', 'cache/myRun1.mat'));
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
d = fileparts(detectionsFile);
if ~exist(d, "dir")
    mkdir(d);
end
save(detectionsFile, "detections", "-v7.3");
fprintf("[INFO] Saved detections to %s\n", detectionsFile);
end

function v = getOr(s, dotPath, fallback)
parts = strsplit(dotPath, '.');
cur = s;
for i = 1:numel(parts)
    if ~isstruct(cur) || ~isfield(cur, parts{i})
        v = fallback;
        return;
    end
    cur = cur.(parts{i});
end
v = cur;
end
