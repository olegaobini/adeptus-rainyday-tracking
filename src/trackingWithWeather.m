function trackingWithWeather(configName)
% trackingWithWeather: Top-level driver for tracking + degradation experiments.
%
% PURPOSE
%   Compare tracking performance across multiple tracker types (GNN, TOMHT, JPDA)
%   and motion models (CV, IMM) under ideal and degraded (rainy) conditions.
%
% WORKFLOW
%   1. Load configuration (scenario parameters)
%   2. Create scenario and generate/load detections
%   3. Configure tracker parameters based on degradation mode
%   4. Run all tracker combinations and collect metrics
%
% BASELINE SOURCE
%   Adapted from MathWorks example:
%   https://www.mathworks.com/help/fusion/ug/tracking-closely-spaced-targets-under-ambiguity.html
% 
% NOTES FOR TEAM
%   - This is the top-level driver script. All "real work" happens in helpers:
%       helperCreateScenario3D   -> scenario definition (truth + radar)
%       helperRunDetections      -> generates detection logs
%       helperRunTracker         -> runs a tracker and produces metrics
%       initCVFilter/initIMMFilter -> defines the filter used per track
%   - If something looks "off", most debugging starts in helperRunDetections
%     (time stamps, measurement noise, clutter injection, etc.).

arguments
    configName (1,1) string = "default"
end

clc; close all;

%% Setup
addProjectPaths();

%% Load Configuration
config = load_config(configName);            % JSON loader from config dir

%% Extract Scenario Parameters
scenarioMode  = config.scenario.mode;        
numTargets    = config.scenario.num_targets; 
sceneDuration = config.scenario.duration_s;  

%% Extract Degradation Setting
enableDegradation = config.degradation.enabled;

%% Extract Global Tracker Parameters
volume    = config.tracker_global.volume;
beta      = config.tracker_global.beta;

% Extract active tracker parameters (already selected by load_config)
params = config.active_params;
pd = params.pd;

fprintf("\n==============================\n");
fprintf(" RUN START | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");

%% Generate/Load Detections
% IMPORTANT:
%   helperRunDetections is where "RAINY" degradation is injected:
%     - detection dropouts (effective Pd)
%     - inflated measurement noise
%     - added clutter (false alarms)

useSavedDataLog = config.data_logging.use_saved_datalog;
dataLogFile = fullfile(pwd, config.data_logging.datalog_file); % Look at this
dataLogDir  = fileparts(dataLogFile);

% Create directory if it doesn't exist
if ~exist(dataLogDir, "dir")
    mkdir(dataLogDir);
end

if useSavedDataLog
    load(dataLogFile, "dataLog");
    fprintf("[INFO] Loaded dataLog from %s\n", dataLogFile);
else
    if scenarioMode == "3D"
        scenario = createScenario3D( ...
            "NumTargets", numTargets, ...
            "SceneDuration", sceneDuration);
    else
        error("Only 3D scenarios are currently supported.");
    end

    dataLog = runDetections(scenario, enableDegradation);

    if config.data_logging.save_after_generation
        save(dataLogFile, "dataLog", "-v7.3");
        fprintf("[INFO] Saved dataLog to %s\n", dataLogFile);
    end
end

% Extract visualization flags with safe defaults
if isfield(config.output, 'show_visuals')
    showVis = config.output.show_visuals;
else
    showVis = true; 
end

if isfield(config.output, 'animate_visuals')
    animVis = config.output.animate_visuals;
else
    animVis = true; 
end

% Plot 3D Scenario if visuals are enabled
if showVis
    plotInitialScenario(dataLog, animVis);
end

%% Quick stats on detection count per scan
% Helps confirm degradation is actually happening:
%   IDEAL -> generally stable count close to number of targets
%   RAINY -> increased variability (clutter + dropouts)
if config.output.print_diagnostics
    nPerScan = cellfun(@numel, dataLog.Detections);
    fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
        min(nPerScan), mean(nPerScan), max(nPerScan));
end

fprintf("[Config] Scenario=%s | gate=%.1f | farGNN=%.2e | farMHT=%.2e | farJPDA=%.2e | pd=%.2f | volume=%.2e | beta=%.2e\n", ...
    ternary(enableDegradation,"RAINY","IDEAL"), params.gate, params.far_gnn, params.far_mht, params.far_jpda, pd, volume, beta);

%% Run all enabled trackers dynamically
results = struct();

% Define the combinations we want to check based on the config
trackerCombos = {
    {'GNN', 'CV',   config.trackers_to_run.gnn_cv};
    {'GNN', 'IMM',  config.trackers_to_run.gnn_imm};
    {'TOMHT','CV',  config.trackers_to_run.tomht_cv};
    {'TOMHT','IMM', config.trackers_to_run.tomht_imm};
    {'JPDA', 'CV',  config.trackers_to_run.jpda_cv};
    {'JPDA', 'IMM', config.trackers_to_run.jpda_imm}
};

for c = 1:length(trackerCombos)
    tType = trackerCombos{c}{1};
    fModel = trackerCombos{c}{2};
    isEnabled = trackerCombos{c}{3};
    
    if isEnabled
        comboName = lower(sprintf('%s_%s', tType, fModel));
        fprintf('\n============ %s + %s ============\n', tType, fModel);
        
        % 1. Use the new factory to build the tracker
        tracker = buildTracker(tType, fModel, params, config.tracker_global, config.filter_params, pd);
        
        % 2. Run the tracker
        [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog] = helperRunTracker(dataLog, tracker, false, showVis, animVis);
        
        % 3. Save results
        results.(comboName).trackSummary = trackSummary;
        results.(comboName).truthSummary = truthSummary;
        results.(comboName).trackMetrics = trackMetrics;
        results.(comboName).truthMetrics = truthMetrics;
        results.(comboName).time = time;
        results.(comboName).assignLog = assignLog;
        
        if config.output.print_diagnostics
            disp(trackSummary); disp(truthSummary);
            disp(trackMetrics); disp(truthMetrics);
        end
    end
end

%% Save results if configured
if config.output.save_results
    resultsDir = fullfile(pwd, config.output.results_directory);
    if ~exist(resultsDir, "dir")
        mkdir(resultsDir);
    end
    
    timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    scenarioName = strrep(configName, "/", "_");
    resultsFile = fullfile(resultsDir, sprintf("results_%s_%s.mat", scenarioName, timestamp));
    
    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");
end

%% -------- Local helper: ternary --------
% Small utility so we can write:
%   ternary(cond, "A", "B")
% instead of MATLAB's  if/else just for printing.
function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function addProjectPaths()
root = pwd;
addpath(genpath(fullfile(root, "src", "helpers")));
addpath(genpath(fullfile(root, "src", "visualization")));
end
