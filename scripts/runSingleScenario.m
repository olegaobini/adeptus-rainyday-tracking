function runSingleScenario(configName)
% runSingleScenario  ONE-COMMAND entrypoint for all scenarios.
%
% USAGE
%   runSingleScenario("default")              % DASR radar, IDEAL, default.json
%   runSingleScenario("clear_weather")        % DASR radar, IDEAL (override file)
%   runSingleScenario("storm_window")         % catalog: DASR + storm degradation
%   runSingleScenario("dasr_ideal")           % catalog: DASR baseline
%   runSingleScenario("crossing_targets")     % catalog: track swap test
%   runSingleScenario("default_wedge_ideal")  % catalog: original 40° wedge radar
%   runSingleScenario("fighter_intercept")    % catalog: AESA on moving aircraft
%   runSingleScenario("maritime_surface")     % catalog: X-band ship radar
%
% HOW IT WORKS
%   1. Checks if the name exists in the scenario catalog
%      → YES: Uses loadScenario (custom sensors, targets, environment)
%      → NO:  Uses loadConfig + createScenario (DASR default radar)
%   2. Generates or loads cached detections
%   3. Runs all enabled trackers from default.json
%   4. Saves results
%
% CONFIGURATION
%   Edit config/default.json to change:
%     scenario.duration_s       — sim length (must be long enough for sensor)
%     degradation.enabled       — true for RAINY, false for IDEAL
%     trackers_to_run.*         — toggle GNN/TOMHT/JPDA × CV/IMM
%     tracker_params.*          — gate, FAR, confirm/delete thresholds
%     output.*                  — visuals, diagnostics, save
%
%   Catalog scenarios override these via their own settings.
%   See: trackbench.scenario.loadScenarioCatalog  (lists all scenarios)
%
% See also: trackbench.scenario.loadScenarioCatalog, trackbench.scenario.loadScenario

arguments
    configName (1,1) string = "default"
end

clc; close all;

%% Setup
root = resolveRootFromThisFile();
addpath(genpath(fullfile(root, "src")));

%% Decide which path to take: catalog scenario vs simple config
isCatalogScenario = checkCatalog(root, configName);

if isCatalogScenario
    fprintf('\n[ROUTE] "%s" found in scenario catalog → full V2 pipeline\n', configName);
    runCatalogPath(root, configName);
else
    fprintf('\n[ROUTE] "%s" → config override path (DASR default radar)\n', configName);
    runConfigPath(root, configName);
end

end


%% ========================================================================
%  PATH 1: Catalog scenario (loadScenario → custom sensors/targets)
%% ========================================================================
function runCatalogPath(root, scenarioName)

    %% Load scenario from catalog (builds scenario + sensors + targets)
    [scenario, config, sensors, metas] = trackbench.scenario.loadScenario(scenarioName);

    %% Resolve paths
    paths = buildPaths(root, config);

    fprintf("\n==============================\n");
    fprintf(" RUN START | %s\n", scenarioName);
    fprintf("==============================\n\n");

    %% Load or generate detections
    dataLog = [];
    if config.data_logging.use_saved_datalog
        dataLog = loadDetections(paths.data_log_file);
    end

    if isempty(dataLog)
        % Count sensors for tracker
        platformNames = fieldnames(sensors);
        totalSensors = 0;
        for p = 1:numel(platformNames)
            totalSensors = totalSensors + numel(sensors.(platformNames{p}));
        end

        envCfg = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                        'ground_clutter', true, 'terrain_type', 'rural', ...
                        'clutter_density', 0.5);
        if isfield(config, 'environment')
            envCfg = config.environment;
        end

        dataLog = trackbench.detections.runDetections( ...
            scenario, config.degradation.enabled, metas, envCfg);

        if config.data_logging.save_after_generation
            saveDetections(paths.data_log_file, dataLog);
        end
    else
        totalSensors = 1;
        if isfield(dataLog, 'HasIFF') && dataLog.HasIFF
            totalSensors = 2;
        end
    end

    %% Scan check
    numScans = numel(dataLog.Time);
    if numScans < 5
        avgPeriod = config.scenario.duration_s / max(numScans, 1);
        warning('runSingleScenario:fewScans', ...
            'Only %d scan(s) — trackers need at least 5. Increase duration_s to %.0fs.', ...
            numScans, ceil(8 * avgPeriod));
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

    %% Detection stats
    params = config.active_params;
    pd     = params.pd;

    if config.output.print_diagnostics
        nPerScan = cellfun(@numel, dataLog.Detections);
        fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
            min(nPerScan), mean(nPerScan), max(nPerScan));
    end

    fprintf(" | gate=%.1f | pd=%.2f | volume=%.2e | beta=%.2e\n", ...
        params.gate, pd, config.tracker_global.volume, config.tracker_global.beta);

    %% Run enabled trackers
    results = struct();
    results.run_id = scenarioName;
    results.config = config;

    trackerCombos = {
        {'GNN', 'CV',   config.trackers_to_run.gnn_cv};
        {'GNN', 'IMM',  config.trackers_to_run.gnn_imm};
        {'TOMHT','CV',  config.trackers_to_run.tomht_cv};
        {'TOMHT','IMM', config.trackers_to_run.tomht_imm};
        {'JPDA', 'CV',  config.trackers_to_run.jpda_cv};
        {'JPDA', 'IMM', config.trackers_to_run.jpda_imm}
    };

    for c = 1:length(trackerCombos)
        tType   = trackerCombos{c}{1};
        fModel  = trackerCombos{c}{2};
        enabled = trackerCombos{c}{3};
        if ~enabled; continue; end

        comboName = lower(sprintf('%s_%s', tType, fModel));
        fprintf('\n============ %s + %s ============\n', tType, fModel);

        tracker = trackbench.tracking.buildTracker(tType, fModel, params, ...
            config.tracker_global, config.filter_params, pd, totalSensors);

        [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = ...
            trackbench.tracking.runTracker(dataLog, tracker, false, showVis, animVis);

        results.(comboName).trackSummary = trackSummary;
        results.(comboName).truthSummary = truthSummary;
        results.(comboName).trackMetrics = trackMetrics;
        results.(comboName).truthMetrics = truthMetrics;
        results.(comboName).time         = time;

        if config.output.print_diagnostics
            disp(trackSummary); disp(truthSummary);
        end
    end

    %% Save
    if config.output.save_results
        if ~exist(paths.results_dir, "dir"); mkdir(paths.results_dir); end
        timestamp   = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        resultsFile = fullfile(paths.results_dir, ...
            sprintf("results_%s_%s.mat", strrep(scenarioName,"/","_"), timestamp));
        save(resultsFile, "results", "config", "-v7.3");
        fprintf("[INFO] Saved results to %s\n", resultsFile);
    end

    fprintf("\n==============================\n");
    fprintf(" RUN END | %s\n", scenarioName);
    fprintf("==============================\n\n");
end


%% ========================================================================
%  PATH 2: Simple config override (loadConfig → DASR default)
%% ========================================================================
function runConfigPath(root, configName)

    config = trackbench.loader.loadConfig(configName);
    paths  = buildPaths(root, config);

    fprintf("\n==============================\n");
    fprintf(" RUN START\n");
    fprintf("==============================\n\n");

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

    if config.output.save_results
        if ~exist(paths.results_dir, "dir"); mkdir(paths.results_dir); end
        timestamp   = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        scenarioTag = strrep(configName, "/", "_");
        resultsFile = fullfile(paths.results_dir, ...
            sprintf("results_%s_%s.mat", scenarioTag, timestamp));
        save(resultsFile, "results", "config", "-v7.3");
        fprintf("[INFO] Saved results to %s\n", resultsFile);
    end

    fprintf("\n==============================\n");
    fprintf(" RUN END\n");
    fprintf("==============================\n\n");
end


%% ========================================================================
%  HELPERS
%% ========================================================================
function found = checkCatalog(root, scenarioName)
%checkCatalog  Return true if scenarioName exists in the scenario catalog.
    catPath = fullfile(root, "config", "scenarios", "scenario_catalog.json");
    found = false;
    if ~isfile(catPath); return; end
    try
        catalog = jsondecode(fileread(catPath));
        if isfield(catalog, 'scenarios') && isfield(catalog.scenarios, scenarioName)
            found = true;
        end
    catch
    end
end

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
