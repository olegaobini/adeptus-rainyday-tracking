function runExperiment(configName)
% runExperiment: Top-level driver for tracking + degradation experiments.
%
% PURPOSE
%   Compare tracking performance across multiple tracker types (GNN, TOMHT, JPDA)
%   and motion models (CV, IMM) under ideal and degraded (rainy) conditions.
%
% USAGE
%   runExperiment                    % uses default_reRunDetections config
%   runExperiment("default")         % uses default config
%
% REQUIRES
%   cd to project root first, then: addpath('src')
%   All functions are in the trackbench.* package.

arguments
    configName (1,1) string = "default_reRunDetections"
end

clc; close all;

%% Load Configuration
config = trackbench.config.loadConfig(configName);

%% Extract Scenario Parameters
scenarioMode  = config.scenario.mode;
numTargets    = config.scenario.num_targets;
sceneDuration = config.scenario.duration_s;
numSensors    = 1;

% IFF parameters
enableIFF = false;
iffUpdateRate = 1; iffRangeSigma = 50; iffAzSigma = 0.1; iffPd = 0.99;
try
    if isfield(config.scenario, 'iff')
        iff = config.scenario.iff;
        if isfield(iff, 'enabled');        enableIFF     = logical(iff.enabled);  end
        if isfield(iff, 'update_rate_hz'); iffUpdateRate = iff.update_rate_hz;    end
        if isfield(iff, 'range_sigma_m');  iffRangeSigma = iff.range_sigma_m;     end
        if isfield(iff, 'az_sigma_deg');   iffAzSigma    = iff.az_sigma_deg;      end
        if isfield(iff, 'pd');             iffPd         = iff.pd;                end
    end
catch
    enableIFF = false;
end

enableDegradation = config.degradation.enabled;
params = config.active_params;
pd     = params.pd;

fprintf("\n==============================\n");
fprintf(" RUN START | Scenario = %s | IFF = %s\n", ...
    ternary(enableDegradation, "RAINY", "IDEAL"), ternary(enableIFF, "ON", "OFF"));
fprintf("==============================\n\n");

%% Generate/Load Detections
useSavedDataLog = config.data_logging.use_saved_datalog;
dataLogFile     = fullfile(pwd, config.data_logging.datalog_file);

if useSavedDataLog && isfile(dataLogFile)
    load(dataLogFile, "dataLog");
    fprintf("[INFO] Loaded dataLog from %s\n", dataLogFile);
else
    scenario = trackbench.scenario.createScenario( ...
        "NumTargets",    numTargets, ...
        "SceneDuration", sceneDuration, ...
        "NumSensors",    numSensors, ...
        "EnableIFF",     enableIFF, ...
        "IFFUpdateRate", iffUpdateRate, ...
        "IFFRangeSigma", iffRangeSigma, ...
        "IFFAzSigma",    iffAzSigma, ...
        "IFFPd",         iffPd);

    % Pre-flight: validate that duration is long enough for the sensor config
    [scanOk, scanInfo] = trackbench.scenario.validateScanCoverage(scenario, sceneDuration);
    if ~scanOk
        warning('runExperiment:insufficientScans', ...
            '%s\nIncrease scenario.duration_s in your config.', scanInfo.message);
    end

    % Pass environment config for horizon masking and ground clutter
    envCfg = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                    'ground_clutter', true, 'terrain_type', 'rural', ...
                    'clutter_density', 0.5);
    if isfield(config, 'environment')
        envCfg = config.environment;
    end
    dataLog = trackbench.detections.runDetections(scenario, enableDegradation, [], envCfg);

    if config.data_logging.save_after_generation
        dDir = fileparts(dataLogFile);
        if ~exist(dDir, "dir"); mkdir(dDir); end
        save(dataLogFile, "dataLog", "-v7.3");
        fprintf("[INFO] Saved dataLog to %s\n", dataLogFile);
    end
end

numSensorsTotal = numSensors + (enableIFF || (isfield(dataLog,'HasIFF') && dataLog.HasIFF));

%% Visualization
showVis  = true;  animVis = true;
if isfield(config.output, 'show_visuals');    showVis = config.output.show_visuals;    end
if isfield(config.output, 'animate_visuals'); animVis = config.output.animate_visuals; end

if showVis
    trackbench.reporting.plotInitialScenario(dataLog, animVis);
end

%% Run all enabled trackers
results = struct();
trackerCombos = {
    {'GNN','CV',  config.trackers_to_run.gnn_cv};
    {'GNN','IMM', config.trackers_to_run.gnn_imm};
    {'TOMHT','CV',  config.trackers_to_run.tomht_cv};
    {'TOMHT','IMM', config.trackers_to_run.tomht_imm};
    {'JPDA','CV',  config.trackers_to_run.jpda_cv};
    {'JPDA','IMM', config.trackers_to_run.jpda_imm}
};

for c = 1:length(trackerCombos)
    tType   = trackerCombos{c}{1};
    fModel  = trackerCombos{c}{2};
    enabled = trackerCombos{c}{3};
    if ~enabled; continue; end

    comboName = lower(sprintf('%s_%s', tType, fModel));
    fprintf('\n============ %s + %s ============\n', tType, fModel);

    tracker = trackbench.tracking.buildTracker(tType, fModel, params, ...
        config.tracker_global, config.filter_params, pd, numSensorsTotal);

    [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = ...
        trackbench.tracking.runTracker(dataLog, tracker, false, showVis, animVis);

    results.(comboName).trackSummary = trackSummary;
    results.(comboName).truthSummary = truthSummary;
    results.(comboName).trackMetrics = trackMetrics;
    results.(comboName).truthMetrics = truthMetrics;
    results.(comboName).time         = time;
    results.(comboName).assignLog    = assignLog;
    results.(comboName).swapReport   = swapReport;

    if config.output.print_diagnostics
        disp(trackSummary); disp(truthSummary);
    end
end

%% Save results
if config.output.save_results
    resultsDir = fullfile(pwd, config.output.results_directory);
    if ~exist(resultsDir, "dir"); mkdir(resultsDir); end
    timestamp   = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    resultsFile = fullfile(resultsDir, sprintf("results_%s_%s.mat", strrep(configName,"/","_"), timestamp));
    save(resultsFile, "results", "config", "-v7.3");
    fprintf("[INFO] Saved results to %s\n", resultsFile);
end

fprintf("\n==============================\n");
fprintf(" RUN END | Scenario = %s\n", ternary(enableDegradation, "RAINY", "IDEAL"));
fprintf("==============================\n\n");
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end
