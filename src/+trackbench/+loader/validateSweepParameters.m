function validateSweepParameters(config)
%validateSweepParameters Ensure sweep parameter paths exist.

if ~isfield(config, 'sweep')
    return;
end

sweep = config.sweep;
mode = char(string(sweep.mode));

switch mode
    case 'single'
        p = string(sweep.parameter);
        if ~trackbench.loader.parameterExists(config, p)
            error('validateSweepParameters:notFound', ...
                'Sweep parameter not found: %s', p);
        end
    case 'grid'
        params = fieldnames(sweep.parameters);
        for i = 1:numel(params)
            p = string(params{i});
            if ~trackbench.loader.parameterExists(config, p)
                error('validateSweepParameters:notFound', ...
                    'Sweep parameter not found: %s', p);
            end
        end
    case 'list'
        return;
    case 'monte_carlo'
        params = fieldnames(sweep.parameters);
        for i = 1:numel(params)
            p = string(params{i});
            if ~trackbench.loader.parameterExists(config, p)
                error('validateSweepParameters:notFound', ...
                    'Sweep parameter not found: %s', p);
            end
        end
end
end
