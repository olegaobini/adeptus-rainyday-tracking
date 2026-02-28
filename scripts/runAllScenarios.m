% runAllScenarios  Batch entrypoint that routes all runs through runScenario.
%
% USAGE
%   runAllScenarios
%   runAllScenarios(true)
%
% CONFIGURE
%   config/default.json -> scenarios_to_run
%   config/default.json -> trackers_to_run
%
% MODE
%   useMySensors=false: each scenario uses its catalog sensor config.
%   useMySensors=true : every scenario forces sensors.json.
%
% See also: runSingleScenario, trackbench.runScenario

function allResults = runAllScenarios(useMySensors)
arguments
    useMySensors (1,1) logical = false
end

clc; close all;
ctx = setupTrackbench();
baseConfig = trackbench.loader.loadConfig("default");
[~, catalog] = trackbench.scenario.loadScenarioCatalog("scenario_catalog");

scenToggle = baseConfig.scenarios_to_run;
scenNames = fieldnames(scenToggle);
enabledScenarios = {};
for i = 1:numel(scenNames)
    name = scenNames{i};
    if startsWith(name, "_")
        continue;
    end
    if islogical(scenToggle.(name)) && scenToggle.(name)
        enabledScenarios{end+1} = name; %#ok<AGROW>
    elseif isnumeric(scenToggle.(name)) && scenToggle.(name) == 1
        enabledScenarios{end+1} = name; %#ok<AGROW>
    end
end

if isempty(enabledScenarios)
    fprintf("[runAll] No scenarios enabled in default.json -> scenarios_to_run\n");
    allResults = struct();
    return;
end

allResults = struct();

for s = 1:numel(enabledScenarios)
    scenName = enabledScenarios{s};
    fprintf("\n[%d/%d] Scenario: %s\n", s, numel(enabledScenarios), scenName);

    if useMySensors
        [scenario, config, ~, metas] = trackbench.scenario.loadScenario(scenName, "scenario_catalog", "sensors");
    else
        [scenario, config, ~, metas] = trackbench.scenario.loadScenario(scenName);
    end

    envCfg = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                    'ground_clutter', true, 'terrain_type', 'rural', ...
                    'clutter_density', 0.5);
    if isfield(config, "environment")
        envCfg = config.environment;
    end

    detections = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas, envCfg);
    [results, ~] = trackbench.runScenario(config, scenName, detections);

    if config.output.save_results
        resultsDir = fullfile(ctx.root, config.output.results_directory);
        if ~exist(resultsDir, "dir")
            mkdir(resultsDir);
        end
        timestamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        resultsFile = fullfile(resultsDir, sprintf("results_%s_%s.mat", scenName, timestamp));
        save(resultsFile, "results", "config", "-v7.3");
        fprintf("[INFO] Saved results to %s\n", resultsFile);
    end

    allResults.(scenName).results = results;
    allResults.(scenName).config = config;
end

catalogNames = fieldnames(catalog.scenarios);
skipped = setdiff(catalogNames, enabledScenarios);
if ~isempty(skipped)
    fprintf("\n[runAll] Skipped %d disabled scenario(s).\n", numel(skipped));
end
end
