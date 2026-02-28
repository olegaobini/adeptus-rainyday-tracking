function validateScenario(scenario)
%validateScenario Validate scenario section.

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
