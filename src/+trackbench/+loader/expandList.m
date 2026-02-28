function runPlan = expandList(baseConfig, sweep, sweepName)
%expandList Expand explicit list sweep.

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

function c = asCell(v)
if iscell(v)
    c = v;
elseif isstruct(v)
    c = num2cell(v);
else
    error('expandList:type', 'sweep.configs must be struct[] or cell.');
end
end
