function runSingleScenario(configName)
% runSingleScenario: Top-level entrypoint for tracking + degradation experiments.
%
% 1.) Call the loader architecture -> get valid config
% 2.) call runScenario and pass config -> get results
% 3.) (optional) call visualization/reporting tools -> get visuals
%

arguments

    configName (1,1) string {mustBeMember(configName, ["default","storm"])} = "default"
end

clc; close all;

%% Setup
ctx = setupTrackbench();

%% Load Configuration
config = trackbench.loader.loadConfig(configName);

%% Resolve IO paths now that config is available
paths = buildPaths(ctx.root, config);

fprintf("\n==============================\n");
fprintf(" RUN START\n");
fprintf("==============================\n\n");

%% Load or generate detections
detections = [];
if config.data_logging.use_saved_datalog
    detections = loadDetections(paths.data_log_file);
end

if isempty(detections)
    [results, detections] = trackbench.runScenario(config, configName);
    if config.data_logging.save_after_generation
        saveDetections(paths.data_log_file, detections);
    end
else
    results = trackbench.runScenario(config, configName, detections);
end

%% Optional reporting/visualization
showVis = true;
animVis = true;

if isfield(config.output, 'show_visuals')
    showVis = config.output.show_visuals;
end

if isfield(config.output, 'animate_visuals')
    animVis = config.output.animate_visuals;
end

if showVis
    trackbench.reporting.plotInitialScenario(detections, animVis);
end

%% Save results if configured
if config.output.save_results
    if ~exist(paths.results_dir, "dir")
        mkdir(paths.results_dir);
    end

    timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    scenarioName = strrep(configName, "/", "_");
    resultsFile = fullfile(paths.results_dir, sprintf("results_%s_%s.mat", scenarioName, timestamp));

    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END\n");
fprintf("==============================\n\n");
end

function paths = buildPaths(root, config)
    paths.root = root;
    paths.data_log_file = fullfile(root, config.data_logging.datalog_file);
    paths.results_dir = fullfile(root, config.output.results_directory);
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
    if ~exist(detectionsDir, "dir")
        mkdir(detectionsDir);
    end

    save(detectionsFile, "detections", "-v7.3");
    fprintf("[INFO] Saved detections to %s\n", detectionsFile);
end
