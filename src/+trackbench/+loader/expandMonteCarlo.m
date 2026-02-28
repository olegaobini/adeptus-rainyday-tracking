function runPlan = expandMonteCarlo(baseConfig, sweep, sweepName)
%expandMonteCarlo Expand monte carlo sweep with deterministic default seed.

numSamples = double(sweep.num_samples);
if ~isscalar(numSamples) || numSamples < 1
    error('expandMonteCarlo:badSamples', 'sweep.num_samples must be >= 1');
end
numSamples = floor(numSamples);

seed = 42;
if isfield(sweep, 'seed')
    seed = double(sweep.seed);
end
rng(seed);

params = fieldnames(sweep.parameters);
runPlan = struct([]);

for i = 1:numSamples
    cfg = baseConfig;
    sampled = struct();
    for j = 1:numel(params)
        p = params{j};
        spec = sweep.parameters.(p);
        val = sampleValue(spec);
        cfg = trackbench.loader.setNestedField(cfg, string(p), val);
        sampled.(p) = val;
    end
    cfg = trackbench.loader.resolveComponentRefs(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    cfg.run_id = sprintf('%s_run_%03d', sweepName, i);
    cfg.sweep_iteration = i;
    cfg.sweep_config = sampled;
    if isempty(runPlan)
        runPlan = cfg;
    else
        runPlan(end+1, 1) = cfg; %#ok<AGROW>
    end
end
end

function val = sampleValue(spec)
if isnumeric(spec)
    vals = num2cell(spec);
    val = vals{randi(numel(vals))};
    return;
end
if iscell(spec)
    val = spec{randi(numel(spec))};
    return;
end
if isstruct(spec)
    if isfield(spec, 'values')
        vals = spec.values;
        if ~iscell(vals); vals = num2cell(vals); end
        val = vals{randi(numel(vals))};
        return;
    end
    if isfield(spec, 'min') && isfield(spec, 'max')
        a = double(spec.min);
        b = double(spec.max);
        val = a + (b-a)*rand();
        return;
    end
end
error('expandMonteCarlo:badParam', 'Unsupported monte_carlo parameter specification.');
end
