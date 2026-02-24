function allResults = runAllScenarios(configName, useMySensors)
%runAllScenarios  Run all enabled scenarios and tracker combinations.
%
% Reads scenarios_to_run and trackers_to_run from default.json (or specified
% config), then for each enabled scenario: loads it, generates detections,
% runs every enabled tracker, and collects results.
%
% USAGE
%   % SHOWCASE MODE — each scenario uses its dedicated sensor config:
%   trackbench.batch.runAllScenarios
%   trackbench.batch.runAllScenarios("default")
%
%   % MY SENSORS MODE — force all scenarios to use YOUR sensors.json:
%   trackbench.batch.runAllScenarios("default", true)
%
% INPUTS
%   configName   : base config name (default: "default")
%   useMySensors : logical. When true, overrides every scenario's sensor
%                  config with "sensors" (your sensors.json toggles).
%                  When false (default), each scenario uses its own
%                  dedicated sensor config (the "showcase" setup).
%
% See also: trackbench.scenario.loadScenario, trackbench.scenario.loadScenarioCatalog

arguments
    configName   (1,1) string  = "default"
    useMySensors (1,1) logical = false
end

clc; close all;

%% Load base config for toggles
baseConfig = trackbench.config.loadConfig(configName);

%% Get scenario toggles
scenToggle = baseConfig.scenarios_to_run;
scenNames  = fieldnames(scenToggle);

% Filter to enabled scenarios (skip _comment field)
enabledScenarios = {};
for i = 1:numel(scenNames)
    name = scenNames{i};
    if startsWith(name, '_'); continue; end
    if islogical(scenToggle.(name)) && scenToggle.(name)
        enabledScenarios{end+1} = name; %#ok<AGROW>
    elseif isnumeric(scenToggle.(name)) && scenToggle.(name) == 1
        enabledScenarios{end+1} = name; %#ok<AGROW>
    end
end

if isempty(enabledScenarios)
    fprintf('[runAll] No scenarios enabled in %s.json → scenarios_to_run\n', configName);
    fprintf('[runAll] Set at least one to true and re-run.\n');
    allResults = struct();
    return;
end

%% Get tracker toggles
trkToggle = baseConfig.trackers_to_run;
trackerCombos = {};
comboMap = {
    'gnn_cv',    'GNN',   'CV';
    'gnn_imm',   'GNN',   'IMM';
    'tomht_cv',  'TOMHT', 'CV';
    'tomht_imm', 'TOMHT', 'IMM';
    'jpda_cv',   'JPDA',  'CV';
    'jpda_imm',  'JPDA',  'IMM';
};

for i = 1:size(comboMap, 1)
    key = comboMap{i, 1};
    if isfield(trkToggle, key) && (trkToggle.(key) == true || trkToggle.(key) == 1)
        trackerCombos{end+1} = comboMap(i, :); %#ok<AGROW>
    end
end

if isempty(trackerCombos)
    fprintf('[runAll] No trackers enabled in %s.json → trackers_to_run\n', configName);
    allResults = struct();
    return;
end

%% Print run plan
sensorModeStr = 'SHOWCASE (dedicated sensors per scenario)';
if useMySensors
    sensorModeStr = 'MY SENSORS (sensors.json for all scenarios)';
end

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              runAllScenarios — RUN PLAN                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Sensors   : %-42s║\n', sensorModeStr);
fprintf('║  Scenarios : %-3d enabled                                ║\n', numel(enabledScenarios));
fprintf('║  Trackers  : %-3d enabled                                ║\n', numel(trackerCombos));
fprintf('║  Total runs: %-3d                                        ║\n', numel(enabledScenarios) * numel(trackerCombos));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
for i = 1:numel(enabledScenarios)
    fprintf('║  [%2d] %-50s║\n', i, enabledScenarios{i});
end
fprintf('╠══════════════════════════════════════════════════════════╣\n');
trkStr = '';
for i = 1:numel(trackerCombos)
    trkStr = [trkStr sprintf('%s+%s', trackerCombos{i}{2}, trackerCombos{i}{3})]; %#ok<AGROW>
    if i < numel(trackerCombos); trkStr = [trkStr ', ']; end %#ok<AGROW>
end
fprintf('║  Trackers: %-45s║\n', trkStr);
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

%% Output settings
showVis   = getOr(baseConfig.output, 'show_visuals', true);
animVis   = getOr(baseConfig.output, 'animate_visuals', true);
printDiag = getOr(baseConfig.output, 'print_diagnostics', true);
saveFigs  = getOr(baseConfig.output, 'save_figures', false);

%% Prepare figure output directory
if saveFigs
    timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    figDir = fullfile(pwd, getOr(baseConfig.output, 'results_directory', 'results'), ...
                      ['figures_' timestamp]);
    if ~exist(figDir, 'dir'); mkdir(figDir); end
    fprintf('[FIGURES] Saving .fig files to: %s\n\n', figDir);
end

%% Run each scenario
allResults = struct();
summaryRows = {};
totalTimer = tic;

for s = 1:numel(enabledScenarios)
    scenName = enabledScenarios{s};
    scenTimer = tic;

    fprintf('\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('  SCENARIO %d/%d: %s\n', s, numel(enabledScenarios), scenName);
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

    %% Load scenario
    try
        if useMySensors
            [scenario, config, sensors, metas] = trackbench.scenario.loadScenario(scenName, "scenario_catalog", "sensors");
        else
            [scenario, config, sensors, metas] = trackbench.scenario.loadScenario(scenName);
        end
    catch ME
        fprintf('[ERROR] Failed to load scenario "%s": %s\n', scenName, ME.message);
        continue;
    end

    %% Count sensors across all platforms
    platformNames = fieldnames(sensors);
    numSensors = 0;
    for p = 1:numel(platformNames)
        numSensors = numSensors + numel(sensors.(platformNames{p}));
    end

    %% Generate detections
    try
        % Pass environment config for horizon masking and ground clutter
        envCfg = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                        'ground_clutter', true, 'terrain_type', 'rural', ...
                        'clutter_density', 0.5);
        if isfield(config, 'environment')
            envCfg = config.environment;
        end
        dataLog = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas, envCfg);
    catch ME
        fprintf('[ERROR] Failed to generate detections for "%s": %s\n', scenName, ME.message);
        continue;
    end

    %% Cache detection log
    if isfield(config.data_logging, 'save_after_generation') && config.data_logging.save_after_generation
        cacheFile = fullfile(pwd, config.data_logging.datalog_file);
        cacheDir  = fileparts(cacheFile);
        if ~exist(cacheDir, 'dir'); mkdir(cacheDir); end
        save(cacheFile, 'dataLog', '-v7.3');
        fprintf('[CACHE] Saved detections → %s\n', cacheFile);
    end

    %% Visualize
    if showVis
        try
            trackbench.reporting.plotInitialScenario(dataLog, animVis);
            if saveFigs
                figFile = fullfile(figDir, sprintf('%s_scenario.fig', scenName));
                savefig(gcf, figFile);
                fprintf('[FIG] Saved → %s\n', figFile);
            end
        catch
        end
    end

    %% Detection diagnostics
    if printDiag
        nPerScan = cellfun(@numel, dataLog.Detections);
        fprintf('Detections/scan: min=%g, mean=%.1f, max=%g, scans=%d\n', ...
            min(nPerScan), mean(nPerScan), max(nPerScan), numel(nPerScan));
    end

    %% Run each tracker
    scenResult = struct();
    scenResult.config  = config;
    scenResult.dataLog = dataLog;
    scenResult.trackers = struct();
    params = config.active_params;

    for t = 1:numel(trackerCombos)
        trkKey  = trackerCombos{t}{1};
        trkType = trackerCombos{t}{2};
        fModel  = trackerCombos{t}{3};

        fprintf('\n  ┌── %s + %s ──\n', trkType, fModel);
        trkTimer = tic;

        try
            tracker = trackbench.tracking.buildTracker(trkType, fModel, params, ...
                config.tracker_global, config.filter_params, params.pd, numSensors);

            % Track which figures exist before tracker runs
            if saveFigs
                figsBefore = findall(0, 'Type', 'figure');
            end

            [trackSummary, truthSummary, trackMetrics, truthMetrics, ...
                tTime, assignLog, swapReport] = ...
                trackbench.tracking.runTracker(dataLog, tracker, false, showVis, animVis);

            % Save any new figures created by the tracker
            if saveFigs
                figsAfter = findall(0, 'Type', 'figure');
                newFigs = setdiff(figsAfter, figsBefore);
                for fi = 1:numel(newFigs)
                    figName = sprintf('%s_%s_%s_fig%d.fig', scenName, trkType, fModel, fi);
                    figFile = fullfile(figDir, figName);
                    try
                        savefig(newFigs(fi), figFile);
                        fprintf('[FIG] Saved → %s\n', figFile);
                    catch
                    end
                end
            end

            % Store results
            scenResult.trackers.(trkKey).trackSummary = trackSummary;
            scenResult.trackers.(trkKey).truthSummary = truthSummary;
            scenResult.trackers.(trkKey).trackMetrics = trackMetrics;
            scenResult.trackers.(trkKey).truthMetrics = truthMetrics;
            scenResult.trackers.(trkKey).time         = tTime;
            scenResult.trackers.(trkKey).assignLog    = assignLog;
            scenResult.trackers.(trkKey).swapReport   = swapReport;

            if printDiag
                disp(trackSummary);
                disp(truthSummary);
            end

            swapStr = 'CLEAN';
            if ~swapReport.swapFree
                swapStr = sprintf('%d SWAP(S)', swapReport.totalSwaps);
            end
            elapsed = toc(trkTimer);
            fprintf('  └── %s+%s: %s | %.1fs\n', trkType, fModel, swapStr, elapsed);
            summaryRows{end+1} = {scenName, trkType, fModel, swapStr, elapsed}; %#ok<AGROW>

        catch ME
            fprintf('  └── [ERROR] %s+%s: %s\n', trkType, fModel, ME.message);
            summaryRows{end+1} = {scenName, trkType, fModel, 'ERROR', 0}; %#ok<AGROW>
        end
    end

    allResults.(matlab.lang.makeValidName(scenName)) = scenResult;
    fprintf('\n  Scenario "%s" complete in %.1fs\n', scenName, toc(scenTimer));
end

%% Save all results
if baseConfig.output.save_results
    resultsDir = fullfile(pwd, baseConfig.output.results_directory);
    if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end
    if ~exist('timestamp', 'var')
        timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    end
    resultsFile = fullfile(resultsDir, sprintf('results_batch_%s.mat', timestamp));
    save(resultsFile, 'allResults', '-v7.3');
    fprintf('\n[SAVED] All results → %s\n', resultsFile);
end

%% Print summary table
totalElapsed = toc(totalTimer);
fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════╗\n');
fprintf('║                        RESULTS SUMMARY                           ║\n');
fprintf('╠════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  %-28s %-10s %-8s %-10s %5s ║\n', 'Scenario', 'Tracker', 'Filter', 'Swaps', 'Time');
fprintf('╠════════════════════════════════════════════════════════════════════╣\n');
for i = 1:numel(summaryRows)
    r = summaryRows{i};
    fprintf('║  %-28s %-10s %-8s %-10s %4.1fs ║\n', r{1}, r{2}, r{3}, r{4}, r{5});
end
fprintf('╠════════════════════════════════════════════════════════════════════╣\n');
fprintf('║  Total: %d scenario(s) × %d tracker(s) = %d runs in %.1fs        ║\n', ...
    numel(enabledScenarios), numel(trackerCombos), numel(summaryRows), totalElapsed);
fprintf('╚════════════════════════════════════════════════════════════════════╝\n');

if saveFigs
    fprintf('[FIGURES] All .fig files saved in: %s\n', figDir);
end
end

%% ---- Local helpers ----
function val = getOr(S, field, default)
    if isfield(S, field); val = S.(field); else; val = default; end
end
