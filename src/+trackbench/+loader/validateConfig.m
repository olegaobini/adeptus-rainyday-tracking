function validateConfig(config)
%validateConfig Stage 2: validate structure/types/enums.

arguments
    config (1,1) struct
end

required = {'scenario', 'sensors', 'truth', 'trackers'};
for i = 1:numel(required)
    if ~isfield(config, required{i})
        error('validateConfig:missingField', 'Missing required field: %s', required{i});
    end
end

validateScenario(config.scenario);
validateSensors(config.sensors);
validateTruth(config.truth);
validateTrackers(config.trackers);

if isfield(config, 'sweep')
    validateSweepStructure(config.sweep);
end
end

function validateScenario(scenario)
if ~isstruct(scenario)
    error('validateScenario:type', 'scenario must be a struct.');
end

required = {'mode', 'duration_s'};
for i = 1:numel(required)
    if ~isfield(scenario, required{i})
        error('validateScenario:missingField', 'scenario.%s is required', required{i});
    end
end

if ~(isnumeric(scenario.duration_s) && isscalar(scenario.duration_s) && scenario.duration_s > 0)
    error('validateScenario:badDuration', ...
        'scenario.duration_s must be a positive number, got: %s', mat2str(scenario.duration_s));
end

validModes = {'2D', '3D'};
if ~ismember(char(string(scenario.mode)), validModes)
    error('validateScenario:badMode', ...
        'scenario.mode must be one of: %s. got: %s', strjoin(validModes, ', '), string(scenario.mode));
end
end

function validateSensors(sensors)
if ~isstruct(sensors)
    error('validateSensors:type', 'sensors must be a struct.');
end

if isfield(sensors, 'sensors')
    entries = sensors.sensors;
    if ~isstruct(entries)
        error('validateSensors:shape', 'sensors.sensors must be an array of sensor structs.');
    end
    for i = 1:numel(entries)
        e = entries(i);
        if ~isfield(e, 'type')
            error('validateSensors:missingType', 'sensors.sensors(%d).type is required.', i);
        end
        if ~isfield(e, 'enabled')
            error('validateSensors:missingEnabled', 'sensors.sensors(%d).enabled is required.', i);
        end
    end
end
end

function validateTruth(truth)
if ~isstruct(truth)
    error('validateTruth:type', 'truth must be a struct.');
end

if ~isfield(truth, 'targets')
    error('validateTruth:missingTargets', 'truth.targets is required.');
end

targets = truth.targets;
if ~(iscell(targets) || isstruct(targets))
    error('validateTruth:typeTargets', 'truth.targets must be a cell array or struct array.');
end
end

function validateTrackers(trackers)
if ~isstruct(trackers)
    error('validateTrackers:type', 'trackers must be a struct.');
end

req = {'global', 'params', 'run'};
for i = 1:numel(req)
    if ~isfield(trackers, req{i})
        error('validateTrackers:missingField', 'trackers.%s is required.', req{i});
    end
end

if ~isfield(trackers.params, 'ideal') || ~isfield(trackers.params, 'degraded')
    error('validateTrackers:missingParams', 'trackers.params must include ideal and degraded.');
end
end

function validateSweepStructure(sweep)
if ~isfield(sweep, 'mode')
    error('validateSweep:missingMode', 'sweep.mode is required.');
end

mode = char(string(sweep.mode));
valid = {'single', 'grid', 'list', 'monte_carlo'};
if ~ismember(mode, valid)
    error('validateSweep:badMode', 'sweep.mode must be one of: %s', strjoin(valid, ', '));
end

switch mode
    case 'single'
        if ~isfield(sweep, 'parameter') || ~isfield(sweep, 'values')
            error('validateSweep:single', 'single sweep requires "parameter" and "values".');
        end
    case 'grid'
        if ~isfield(sweep, 'parameters')
            error('validateSweep:grid', 'grid sweep requires "parameters".');
        end
    case 'list'
        if ~isfield(sweep, 'configs')
            error('validateSweep:list', 'list sweep requires "configs".');
        end
    case 'monte_carlo'
        if ~isfield(sweep, 'parameters') || ~isfield(sweep, 'num_samples')
            error('validateSweep:mc', 'monte_carlo sweep requires "parameters" and "num_samples".');
        end
end
end
