function runPlan = expandRunPlan(config)
%expandRunPlan Stage 4: sweep expansion into concrete run configs.

if ~isfield(config, 'sweep')
    config.run_id = 'single_run_001';
    runPlan = config;
    return;
end

trackbench.loader.validateSweepParameters(config);

sweep = config.sweep;
base = rmfield(config, 'sweep');
if isfield(base, 'sweep_name')
    sweepName = string(base.sweep_name);
    base = rmfield(base, 'sweep_name');
else
    sweepName = "sweep";
end

mode = char(string(sweep.mode));
switch mode
    case 'single'
        runPlan = trackbench.loader.expandSingleParam(base, sweep, sweepName);
    case 'grid'
        runPlan = trackbench.loader.expandGrid(base, sweep, sweepName);
    case 'list'
        runPlan = trackbench.loader.expandList(base, sweep, sweepName);
    case 'monte_carlo'
        runPlan = trackbench.loader.expandMonteCarlo(base, sweep, sweepName);
    otherwise
        error('expandRunPlan:badMode', 'Unknown sweep mode: %s', mode);
end
end
