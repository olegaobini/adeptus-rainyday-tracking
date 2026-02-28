function runPlan = expandRunPlan(config)
%expandRunPlan Stage 4: sweep expansion into concrete run configs.

if ~isfield(config, 'sweep')
    config.run_id = 'single_run_001';
    runPlan = config;
    return;
end

validateSweepParameters(config);

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
        runPlan = expandSingleParam(base, sweep, sweepName);
    case 'grid'
        runPlan = expandGrid(base, sweep, sweepName);
    case 'list'
        runPlan = expandList(base, sweep, sweepName);
    case 'monte_carlo'
        runPlan = expandMonteCarlo(base, sweep, sweepName);
    otherwise
        error('expandRunPlan:badMode', 'Unknown sweep mode: %s', mode);
end
end

function validateSweepParameters(config)
if ~isfield(config, 'sweep')
    return;
end

sweep = config.sweep;
mode = char(string(sweep.mode));

switch mode
    case 'single'
        p = string(sweep.parameter);
        if ~parameterExists(config, p)
            error('validateSweepParameters:notFound', 'Sweep parameter not found: %s', p);
        end
    case 'grid'
        params = fieldnames(sweep.parameters);
        for i = 1:numel(params)
            p = string(params{i});
            if ~parameterExists(config, p)
                error('validateSweepParameters:notFound', 'Sweep parameter not found: %s', p);
            end
        end
    case 'list'
        return;
    case 'monte_carlo'
        params = fieldnames(sweep.parameters);
        for i = 1:numel(params)
            p = string(params{i});
            if ~parameterExists(config, p)
                error('validateSweepParameters:notFound', 'Sweep parameter not found: %s', p);
            end
        end
end
end

function exists = parameterExists(config, dotPath)
parts = strsplit(char(dotPath), '.');
current = config;

for i = 1:numel(parts)
    [field, idx, hasIdx] = parsePart(parts{i});

    if ~isstruct(current) || ~isfield(current, field)
        exists = false;
        return;
    end

    current = current.(field);

    if hasIdx
        if ~iscell(current) || idx < 1 || idx > numel(current)
            exists = false;
            return;
        end
        current = current{idx};
    end
end
exists = true;
end

function runPlan = expandSingleParam(baseConfig, sweep, sweepName)
param = string(sweep.parameter);
values = asCell(sweep.values);
n = numel(values);

runPlan = struct([]);
for i = 1:n
    cfg = trackbench.loader.setNestedField(baseConfig, param, values{i});
    cfg = trackbench.loader.resolveComponentRefs(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    cfg.run_id = sprintf('%s_run_%03d', sweepName, i);
    cfg.sweep_iteration = i;
    cfg.sweep_config = struct('parameter', param, 'value', values{i});
    if isempty(runPlan)
        runPlan = cfg;
    else
        runPlan(end+1, 1) = cfg; %#ok<AGROW>
    end
end
end

function runPlan = expandGrid(baseConfig, sweep, sweepName)
params = fieldnames(sweep.parameters);
valueSets = cell(1, numel(params));
for i = 1:numel(params)
    valueSets{i} = asCell(sweep.parameters.(params{i}));
end

indices = makeIndexCombos(cellfun(@numel, valueSets));
n = size(indices, 1);
runPlan = struct([]);

for i = 1:n
    cfg = baseConfig;
    chosen = cell(1, numel(params));
    for j = 1:numel(params)
        val = valueSets{j}{indices(i,j)};
        chosen{j} = val;
        cfg = trackbench.loader.setNestedField(cfg, string(params{j}), val);
    end
    cfg = trackbench.loader.resolveComponentRefs(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    cfg.run_id = sprintf('%s_run_%03d', sweepName, i);
    cfg.sweep_iteration = i;
    cfg.sweep_config = struct('parameters', {params}, 'values', {chosen});
    if isempty(runPlan)
        runPlan = cfg;
    else
        runPlan(end+1, 1) = cfg; %#ok<AGROW>
    end
end
end

function runPlan = expandList(baseConfig, sweep, sweepName)
cfgs = asCell(sweep.configs);
n = numel(cfgs);
runPlan = struct([]);

for i = 1:n
    cfg = trackbench.loader.mergeStructs(baseConfig, cfgs{i});
    cfg = trackbench.loader.resolveComponentRefs(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    cfg.run_id = sprintf('%s_run_%03d', sweepName, i);
    cfg.sweep_iteration = i;
    cfg.sweep_config = cfgs{i};
    if isempty(runPlan)
        runPlan = cfg;
    else
        runPlan(end+1, 1) = cfg; %#ok<AGROW>
    end
end
end

function runPlan = expandMonteCarlo(baseConfig, sweep, sweepName)
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

function [field, idx, hasIdx] = parsePart(part)
expr = regexp(part, '^(\w+)\[(\d+)\]$', 'tokens', 'once');
if isempty(expr)
    field = part;
    idx = 0;
    hasIdx = false;
else
    field = expr{1};
    idx = str2double(expr{2}) + 1;
    hasIdx = true;
end
end

function combos = makeIndexCombos(sizes)
N = numel(sizes);
vecs = cell(1, N);
for i = 1:N
    vecs{i} = 1:sizes(i);
end
[grids{1:N}] = ndgrid(vecs{:}); %#ok<CCAT>
rows = numel(grids{1});
combos = zeros(rows, N);
for i = 1:N
    combos(:, i) = grids{i}(:);
end
end

function c = asCell(v)
if iscell(v)
    c = v;
elseif isstruct(v)
    c = num2cell(v);
else
    c = num2cell(v);
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
