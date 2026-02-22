function [scenarioList, catalog] = loadScenarioCatalog(catalogName)
%loadScenarioCatalog  Load and list available scenarios from the catalog.
%
% USAGE
%   % List all available scenarios
%   scenarioList = loadScenarioCatalog();
%
%   % Load a specific catalog
%   scenarioList = loadScenarioCatalog("scenario_catalog");
%
% OUTPUT
%   scenarioList : table with columns:
%       Name, Description, NumTargets, Duration, Degraded, SensorConfig
%   catalog : raw parsed JSON struct
%
% See also: loadScenario, loadSensors, load_config

arguments
    catalogName (1,1) string = "scenario_catalog"
end

if ~endsWith(catalogName, ".json")
    catalogName = catalogName + ".json";
end

jsonPath = fullfile(pwd, "config", "scenarios", catalogName);
if ~isfile(jsonPath)
    error('loadScenarioCatalog:notFound', 'Catalog not found: %s', jsonPath);
end

catalog = jsondecode(fileread(jsonPath));
scens   = catalog.scenarios;
names   = fieldnames(scens);

Name        = strings(numel(names), 1);
Description = strings(numel(names), 1);
NumTargets  = zeros(numel(names), 1);
Duration    = zeros(numel(names), 1);
Degraded    = false(numel(names), 1);
SensorConfig = strings(numel(names), 1);

for i = 1:numel(names)
    s = scens.(names{i});
    Name(i) = names{i};
    Description(i) = string(s.description);

    if isfield(s, 'overrides')
        ov = s.overrides;
        if isfield(ov, 'scenario_num_targets')
            NumTargets(i) = ov.scenario_num_targets;
        elseif isfield(ov, 'scenario')  && isstruct(ov.scenario) && isfield(ov.scenario, 'num_targets')
            NumTargets(i) = ov.scenario.num_targets;
        else
            % Try dot-notation keys
            NumTargets(i) = getOvField(ov, 'scenario.num_targets', numel(s.targets));
        end
        Duration(i) = getOvField(ov, 'scenario.duration_s', 50);
        Degraded(i) = logical(getOvField(ov, 'degradation.enabled', false));
    else
        NumTargets(i) = numel(s.targets);
        Duration(i)   = 50;
        Degraded(i)   = false;
    end

    if isfield(s, 'sensor_config')
        SensorConfig(i) = string(s.sensor_config);
    else
        SensorConfig(i) = "sensors";
    end
end

scenarioList = table(Name, Description, NumTargets, Duration, Degraded, SensorConfig);

fprintf('\n=== Available Scenarios ===\n');
for i = 1:height(scenarioList)
    degStr = "IDEAL";
    if scenarioList.Degraded(i); degStr = "DEGRADED"; end
    fprintf('  %-25s | %dt | %3.0fs | %-8s | %s\n', ...
        scenarioList.Name(i), scenarioList.NumTargets(i), ...
        scenarioList.Duration(i), degStr, scenarioList.Description(i));
end
fprintf('\nUsage: config = loadScenario("<name>")\n\n');
end

function val = getOvField(ov, dotKey, default)
    % Check for dot-notation keys in override struct
    % e.g. "scenario.duration_s" stored as field name with dots
    cleanKey = strrep(dotKey, '.', '_');
    if isfield(ov, dotKey)
        val = ov.(dotKey);
    elseif isfield(ov, cleanKey)
        val = ov.(cleanKey);
    else
        val = default;
    end
end
