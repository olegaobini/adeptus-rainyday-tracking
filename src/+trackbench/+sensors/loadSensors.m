function [sensors, metas, sensorCatalog] = loadSensors(sensorConfigName)
%loadSensors  Load sensor definitions from a JSON catalog and build enabled sensors.
%
% PURPOSE
%   Reads a sensor catalog JSON, builds all enabled sensors via buildSensor(),
%   and returns them ready to attach to a trackingScenario platform.
%
% USAGE
%   % Load default sensor catalog
%   [sensors, metas] = loadSensors();
%
%   % Load a specific sensor catalog
%   [sensors, metas] = loadSensors("sensors_maritime");
%
%   % Attach to scenario
%   tower = platform(scenario, 'Sensors', sensors.tower);
%
% INPUTS
%   sensorConfigName : (optional) name of JSON file in config/ (without .json)
%                      Default: "sensors"
%
% OUTPUTS
%   sensors : struct with fields grouped by platform name
%             sensors.tower    = {psr, ssr, ...}  (cell array of sensor objects)
%             sensors.aircraft = {aesa, ...}
%             sensors.ship     = {sonar, ...}
%   metas   : struct mirroring sensors, but containing metadata structs
%             metas.tower = {psrMeta, ssrMeta, ...}
%   sensorCatalog : the raw parsed JSON struct (for inspection/logging)
%
% JSON FORMAT
%   See config/sensors.json for the full catalog format. Key fields per sensor:
%     "name"     : human-readable label
%     "type"     : buildSensor type string (e.g. "PSR", "SSR", "IRST")
%     "enabled"  : true/false — only enabled sensors are built
%     "platform" : which platform to attach to ("tower", "aircraft", "ship")
%     "params"   : object of parameter overrides passed to buildSensor()
%
% SENSOR INDEX ASSIGNMENT
%   Indices are auto-assigned in order of enabled sensors: 1, 2, 3, ...
%   This ensures no index gaps or duplicates regardless of which sensors
%   are enabled. The tracker's MaxNumSensors should be set to numel of
%   all enabled sensors combined.
%
% See also: buildSensor, createScenario3D, trackingScenario

arguments
    sensorConfigName (1,1) string = "sensors"
end

%% Locate and load JSON
if ~endsWith(sensorConfigName, ".json")
    sensorConfigName = sensorConfigName + ".json";
end

% Look in config/sensors/ first, then fall back to config/ for compatibility
jsonPath = fullfile(pwd, "config", "sensors", sensorConfigName);
if ~isfile(jsonPath)
    jsonPath = fullfile(pwd, "config", sensorConfigName);
end
if ~isfile(jsonPath)
    error('loadSensors:fileNotFound', 'Sensor config not found: %s', jsonPath);
end

fprintf('[SENSORS] Loading: %s\n', jsonPath);
raw = jsondecode(fileread(jsonPath));

if ~isfield(raw, 'sensors') || isempty(raw.sensors)
    error('loadSensors:noSensors', 'Sensor config has no "sensors" array.');
end

sensorCatalog = raw;
entries = raw.sensors;

%% Build enabled sensors
sensors = struct();
metas   = struct();

sensorIdx = 0;
numEnabled = 0;
numSkipped = 0;

for i = 1:numel(entries)
    entry = entries(i);

    % Skip disabled sensors
    if ~isfield(entry, 'enabled') || ~entry.enabled
        numSkipped = numSkipped + 1;
        continue;
    end

    sensorIdx  = sensorIdx + 1;
    sType      = upper(string(entry.type));
    sName      = string(entry.name);
    platName   = "tower";  % default platform

    if isfield(entry, 'platform') && ~isempty(entry.platform)
        platName = lower(string(entry.platform));
    end

    % Convert params struct to name-value cell array
    nvPairs = {};
    if isfield(entry, 'params') && isstruct(entry.params)
        fields = fieldnames(entry.params);
        for f = 1:numel(fields)
            val = entry.params.(fields{f});
            % JSON arrays come in as column vectors — transpose row vectors
            if isnumeric(val) && iscolumn(val) && numel(val) > 1
                val = val(:)';  % keep as row for rangeLimits, sector, etc.
            end
            % fov needs to be column vector for buildSensor
            if strcmpi(fields{f}, 'fov') && isnumeric(val)
                val = val(:);
            end
            nvPairs = [nvPairs, {fields{f}, val}]; %#ok<AGROW>
        end
    end

    % Build the sensor
    try
        [sensor, meta] = trackbench.sensors.buildSensor(sensorIdx, sType, nvPairs{:});
        meta.catalogName = char(sName);
        meta.platformGroup = char(platName);
        meta.catalogIndex = i;

        % Make platform a valid MATLAB field name
        platField = matlab.lang.makeValidName(char(platName));

        % Append to platform group
        if ~isfield(sensors, platField)
            sensors.(platField) = {};
            metas.(platField)   = {};
        end
        sensors.(platField){end+1} = sensor;
        metas.(platField){end+1}   = meta;

        numEnabled = numEnabled + 1;

    catch ME
        warning('loadSensors:buildFailed', ...
            'Failed to build sensor "%s" (type=%s): %s', sName, sType, ME.message);
    end
end

%% Summary
platformNames = fieldnames(sensors);
fprintf('[SENSORS] Built %d sensors (%d skipped) across %d platform(s):\n', ...
    numEnabled, numSkipped, numel(platformNames));

for p = 1:numel(platformNames)
    pName = platformNames{p};
    nSens = numel(sensors.(pName));
    names = cellfun(@(m) m.catalogName, metas.(pName), 'UniformOutput', false);
    fprintf('  %s: %d sensor(s) — %s\n', pName, nSens, strjoin(string(names), ', '));
end

fprintf('[SENSORS] Total sensor count: %d (set tracker MaxNumSensors to this)\n', numEnabled);
end
