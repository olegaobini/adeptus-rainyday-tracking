function validateSweepStructure(sweep)
%validateSweepStructure Validate sweep shape by mode.

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
