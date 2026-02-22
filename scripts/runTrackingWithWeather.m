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
%       createScenario3D     -> scenario definition (truth + radar + IFF)
%       runDetections        -> generates detection logs
%       helperRunTracker     -> runs a tracker and produces metrics
%       initCVFilter/initIMMFilter -> defines the filter used per track
%   - If something looks "off", most debugging starts in runDetections
%     (time stamps, measurement noise, clutter injection, etc.).

arguments
    configName (1,1) string = "default_reRunDetections"
end

clc; close all;

%% Setup
addProjectPaths();

%% Load Configuration
config = load_config(configName);

%% Extract Scenario Parameters
scenarioMode  = config.scenario.mode;
numTargets    = config.scenario.num_targets;
sceneDuration = config.scenario.duration_s;

% Number of primary search radars
numSensors = 1;
try
    if isfield(config, 'num_sensors')
        numSensors = config.num_sensors;
    elseif isfield(config, 'scenario') && isfield(config.scenario, 'num_sensors')
        numSensors = config.scenario.num_sensors;
    elseif isfield(config, 'sensor') && isfield(config.sensor, 'num_sensors')
        numSensors = config.sensor.num_sensors;
    end
catch
end

% IFF parameters from config.scenario.iff (all optional with safe defaults)
enableIFF     = false;
iffUpdateRate = 1;
iffRangeSigma = 50;
iffAzSigma    = 0.1;
iffPd         = 0.99;
try
    if isfield(config.scenario, 'iff')
        iff = config.scenario.iff;
        if isfield(iff, 'enabled');        enableIFF     = logical(iff.enabled);        end
        if isfield(iff, 'update_rate_hz'); iffUpdateRate = iff.update_rate_hz;          end
        if isfield(iff, 'range_sigma_m');  iffRangeSigma = iff.range_sigma_m;           end
        if isfield(iff, 'az_sigma_deg');   iffAzSigma    = iff.az_sigma_deg;            end
        if isfield(iff, 'pd');             iffPd         = iff.pd;                      end
    end
catch ME
    warning('trackingWithWeather:iffParse', ...
        'Could not parse IFF config: %s. IFF disabled.', ME.message);
    enableIFF = false;
end

%% Extract Degradation Setting
enableDegradation = config.degradation.enabled;

%% Extract Global Tracker Parameters
volume = config.tracker_global.volume;
beta   = config.tracker_global.beta;

% Extract active tracker parameters (selected by load_config)
params = config.active_params;
pd     = params.pd;

fprintf("\n==============================\n");
fprintf(" RUN START | Scenario = %s | IFF = %s\n", ...
    ternary(enableDegradation, "RAINY", "IDEAL"), ...
    ternary(enableIFF, "ON", "OFF"));
fprintf("==============================\n\n");

%% Generate/Load Detections
useSavedDataLog = config.data_logging.use_saved_datalog;
dataLogFile     = fullfile(pwd, config.data_logging.datalog_file);
dataLogDir      = fileparts(dataLogFile);

if ~exist(dataLogDir, "dir")
    mkdir(dataLogDir);
end

if useSavedDataLog
    load(dataLogFile, "dataLog");
    fprintf("[INFO] Loaded dataLog from %s\n", dataLogFile);
else
    if scenarioMode == "3D"
        scenario = createScenario3D( ...
            "NumTargets",    numTargets, ...
            "SceneDuration", sceneDuration, ...
            "NumSensors",    numSensors, ...
            "EnableIFF",     enableIFF, ...
            "IFFUpdateRate", iffUpdateRate, ...
            "IFFRangeSigma", iffRangeSigma, ...
            "IFFAzSigma",    iffAzSigma, ...
            "IFFPd",         iffPd);
    else
        error("Only 3D scenarios are currently supported.");
    end

    dataLog = runDetections(scenario, enableDegradation);

    if config.data_logging.save_after_generation
        save(dataLogFile, "dataLog", "-v7.3");
        fprintf("[INFO] Saved dataLog to %s\n", dataLogFile);
    end
end

% Total sensor count for tracker construction (primary + IFF if present)
% IFF sensor gets its own SensorIndex but the tracker needs to know about it
numSensorsTotal = numSensors + (enableIFF || ...
    (isfield(dataLog, 'HasIFF') && dataLog.HasIFF));

%% Visualization flags
showVis = true;
animVis = true;
if isfield(config.output, 'show_visuals');   showVis = config.output.show_visuals;   end
if isfield(config.output, 'animate_visuals'); animVis = config.output.animate_visuals; end

if showVis
    plotInitialScenario(dataLog, animVis);
end

%% Detection count diagnostics
if config.output.print_diagnostics
    nPerScan = cellfun(@numel, dataLog.Detections);
    fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
        min(nPerScan), mean(nPerScan), max(nPerScan));
    if isfield(dataLog, 'HasIFF') && dataLog.HasIFF
        fprintf("[INFO] IFF sensor index: %d\n", dataLog.IFFSensorIndex);
    end
end

fprintf("[Config] Scenario=%s | gate=%.1f | farGNN=%.2e | farMHT=%.2e | farJPDA=%.2e | pd=%.2f | volume=%.2e | beta=%.2e\n", ...
    ternary(enableDegradation,"RAINY","IDEAL"), params.gate, params.far_gnn, params.far_mht, params.far_jpda, pd, volume, beta);

%% Run all enabled trackers
results = struct();

trackerCombos = {
    {'GNN',   'CV',  config.trackers_to_run.gnn_cv};
    {'GNN',   'IMM', config.trackers_to_run.gnn_imm};
    {'TOMHT', 'CV',  config.trackers_to_run.tomht_cv};
    {'TOMHT', 'IMM', config.trackers_to_run.tomht_imm};
    {'JPDA',  'CV',  config.trackers_to_run.jpda_cv};
    {'JPDA',  'IMM', config.trackers_to_run.jpda_imm}
};

for c = 1:length(trackerCombos)
    tType   = trackerCombos{c}{1};
    fModel  = trackerCombos{c}{2};
    enabled = trackerCombos{c}{3};

    if ~enabled; continue; end

    comboName = lower(sprintf('%s_%s', tType, fModel));
    fprintf('\n============ %s + %s ============\n', tType, fModel);

    % Pass numSensorsTotal so tracker knows about IFF sensor index
    tracker = buildTracker(tType, fModel, params, config.tracker_global, ...
        config.filter_params, pd, numSensorsTotal);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = ...
        helperRunTracker(dataLog, tracker, false, showVis, animVis);

    results.(comboName).trackSummary = trackSummary;
    results.(comboName).truthSummary = truthSummary;
    results.(comboName).trackMetrics = trackMetrics;
    results.(comboName).truthMetrics = truthMetrics;
    results.(comboName).time         = time;
    results.(comboName).assignLog    = assignLog;
    results.(comboName).swapReport   = swapReport;

    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
        disp(trackMetrics); disp(truthMetrics);
        fprintf('\n--- Track Swap Analysis ---\n');
        if swapReport.swapFree
            fprintf('SWAP STATUS: CLEAN | 0 swaps | All tracks held correct identity\n');
        else
            fprintf('SWAP STATUS: %d SWAP(S) DETECTED | Max consecutive wrong: %d scans\n', ...
                swapReport.totalSwaps, swapReport.maxConsecutive);
            disp(swapReport.swapEvents);
            fprintf('Per-track breakdown:\n');
            disp(swapReport.perTrack);
        end
        fprintf('----------------------------\n');
    end
end

%% Save results
if config.output.save_results
    resultsDir = fullfile(pwd, config.output.results_directory);
    if ~exist(resultsDir, "dir"); mkdir(resultsDir); end

    timestamp    = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    scenarioName = strrep(configName, "/", "_");
    resultsFile  = fullfile(resultsDir, sprintf("results_%s_%s.mat", scenarioName, timestamp));

    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END | Scenario = %s\n", ternary(enableDegradation, "RAINY", "IDEAL"));
fprintf("==============================\n\n");
end

%% ---- Local helpers ----
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function addProjectPaths()
    root = pwd;
    addpath(genpath(fullfile(root, "src", "helpers")));
    addpath(genpath(fullfile(root, "src", "visualization")));
end