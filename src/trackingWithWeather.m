
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
numTracks = config.tracker_global.max_num_tracks;
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
% Datalog will contain
%    - detections measurements
%    - Ground Truth
useSavedDataLog = config.data_logging.use_saved_datalog;
dataLogFile = fullfile(pwd, config.data_logging.datalog_file); % Creates the path
dataLogDir  = fileparts(dataLogFile);

% Create directory if it doesn't exist
if ~exist(dataLogDir, "dir")
    mkdir(dataLogDir);
end

% load cached or generate new data
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

if config.output.plot_initial_scenario
    plotInitialScenario(dataLog);
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

%% Run all enabled trackers
% This structure makes it easy to enable/disable trackers via config

results = struct();

%% ============ GNN + CV ============
if config.trackers_to_run.gnn_cv
    fprintf("\n============ GNN + CV ============\n");
    tracker = trackerGNN( ...
        'FilterInitializationFcn', @initCVFilter, ...
        'MaxNumTracks', numTracks, ...
        'MaxNumSensors', 1, ...
        'AssignmentThreshold', params.gate, ...
        'TrackLogic', 'Score', ...
        'DetectionProbability', pd, ...
        'FalseAlarmRate', params.far_gnn, ...
        'Volume', volume, ...
        'Beta', beta);

    % helperRunTracker does:
    %   - step through dataLog detections
    %   - call tracker(detections, time)
    %   - compute metrics vs truth
    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);

    results.gnn_cv.trackSummary = trackSummary;
    results.gnn_cv.truthSummary = truthSummary;
    results.gnn_cv.trackMetrics = trackMetrics;
    results.gnn_cv.truthMetrics = truthMetrics;
    results.gnn_cv.time = time;

    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
        disp(trackMetrics); disp(truthMetrics);
    end
end

%% ============ GNN + IMM ============
if config.trackers_to_run.gnn_imm
    % IMM: Interacting Multiple Model (handles maneuvering better than CV)
    fprintf("\n============ GNN + IMM ============\n");
    tracker = trackerGNN( ...
        'FilterInitializationFcn', @initIMMFilter, ...
        'MaxNumTracks', numTracks, ...
        'MaxNumSensors', 1, ...
        'AssignmentThreshold', params.gate, ...
        'TrackLogic', 'Score', ...
        'DetectionProbability', pd, ...
        'FalseAlarmRate', params.far_gnn, ...
        'Volume', volume, ...
        'Beta', beta);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);
    
    results.gnn_imm.trackSummary = trackSummary;
    results.gnn_imm.truthSummary = truthSummary;
    results.gnn_imm.trackMetrics = trackMetrics;
    results.gnn_imm.truthMetrics = truthMetrics;
    results.gnn_imm.time = time;
    
    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
        disp(trackMetrics); disp(truthMetrics);
    end
end

%% ============ TOMHT + CV ============
if config.trackers_to_run.tomht_cv
fprintf("\n============ TOMHT + CV ============\n");
    
    tomhtThresh = params.tomht_threshold_multiplier * params.gate;
    
tracker = trackerTOMHT( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', tomhtThresh, ...
    'DetectionProbability', pd, ...
        'FalseAlarmRate', params.far_mht, ...
    'Volume', volume, ...
    'Beta', beta, ...
        'ConfirmationThreshold', params.confirm_threshold, ...
        'DeletionThreshold', params.delete_threshold, ...
    'MaxNumHistoryScans', 10, ...
        'MaxNumTrackBranches', params.max_branches, ...
    'NScanPruning', 'Hypothesis', ...
    'OutputRepresentation', 'Tracks');

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);
    
    results.tomht_cv.trackSummary = trackSummary;
    results.tomht_cv.truthSummary = truthSummary;
    results.tomht_cv.trackMetrics = trackMetrics;
    results.tomht_cv.truthMetrics = truthMetrics;
    results.tomht_cv.time = time;
    
    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
        disp(trackMetrics); disp(truthMetrics);
    end
end

%% ============ TOMHT + IMM ============
if config.trackers_to_run.tomht_imm
    fprintf("\n============ TOMHT + IMM ============\n");
    
    tomhtThresh = params.tomht_threshold_multiplier * params.gate;
    
    tracker = trackerTOMHT( ...
        'FilterInitializationFcn', @initIMMFilter, ...
        'MaxNumTracks', numTracks, ...
        'MaxNumSensors', 1, ...
        'AssignmentThreshold', tomhtThresh, ...
        'DetectionProbability', pd, ...
        'FalseAlarmRate', params.far_mht, ...
        'Volume', volume, ...
        'Beta', beta, ...
        'ConfirmationThreshold', params.confirm_threshold, ...
        'DeletionThreshold', params.delete_threshold, ...
        'MaxNumHistoryScans', 10, ...
        'MaxNumTrackBranches', params.max_branches, ...
        'NScanPruning', 'Hypothesis', ...
        'OutputRepresentation', 'Tracks');

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);
    
    results.tomht_imm.trackSummary = trackSummary;
    results.tomht_imm.truthSummary = truthSummary;
    results.tomht_imm.trackMetrics = trackMetrics;
    results.tomht_imm.truthMetrics = truthMetrics;
    results.tomht_imm.time = time;
    
    if config.output.print_diagnostics
    disp(trackSummary); disp(truthSummary);
    disp(trackMetrics); disp(truthMetrics);
    end
end

%% ============ JPDA + CV ============
if config.trackers_to_run.jpda_cv
    fprintf("\n============JPDA + CV==================\n");
    tracker = trackerJPDA( ...
        'FilterInitializationFcn', @initCVFilter, ...
        'MaxNumTracks', params.num_tracks_jpda, ...
        'MaxNumSensors', 1, ...
        'AssignmentThreshold', params.gate_jpda, ...
        'TrackLogic', 'Integrated', ...
        'DetectionProbability', pd, ...
        'ClutterDensity', params.far_jpda/volume, ...
        'NewTargetDensity', params.beta_jpda, ...
        'TimeTolerance', params.time_tolerance_jpda);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);
    
    results.jpda_cv.trackSummary = trackSummary;
    results.jpda_cv.truthSummary = truthSummary;
    results.jpda_cv.trackMetrics = trackMetrics;
    results.jpda_cv.truthMetrics = truthMetrics;
    results.jpda_cv.time = time;
    
    if config.output.print_diagnostics
    disp(trackSummary); disp(truthSummary);
    disp(trackMetrics); disp(truthMetrics);
    end
end

%% ============ JPDA + IMM ============
if config.trackers_to_run.jpda_imm
fprintf("\n============JPDA + IMM==================\n");
tracker = trackerJPDA( ...
    'FilterInitializationFcn', @initIMMFilter, ...
        'MaxNumTracks', params.num_tracks_jpda, ...
    'MaxNumSensors', 1, ...
        'AssignmentThreshold', params.gate_jpda, ...
    'TrackLogic', 'Integrated', ...
    'DetectionProbability', pd, ...
        'ClutterDensity', params.far_jpda/volume, ...
        'NewTargetDensity', params.beta_jpda, ...
        'TimeTolerance', params.time_tolerance_jpda);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog, tracker, false);
    
    results.jpda_imm.trackSummary = trackSummary;
    results.jpda_imm.truthSummary = truthSummary;
    results.jpda_imm.trackMetrics = trackMetrics;
    results.jpda_imm.truthMetrics = truthMetrics;
    results.jpda_imm.time = time;
    
    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
        disp(trackMetrics); disp(truthMetrics);
    end
end

%% Save results if configured
if config.output.save_results
    resultsDir = fullfile(pwd, config.output.results_directory);
    if ~exist(resultsDir, "dir")
        mkdir(resultsDir);
    end
    
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
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

