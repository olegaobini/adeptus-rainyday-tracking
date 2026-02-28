function runPlan = expandGrid(baseConfig, sweep, sweepName)
%expandGrid Expand cartesian grid sweep.

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
