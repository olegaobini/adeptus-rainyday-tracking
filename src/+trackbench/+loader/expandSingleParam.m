function runPlan = expandSingleParam(baseConfig, sweep, sweepName)
%expandSingleParam Expand single-parameter sweep.

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

function c = asCell(v)
if iscell(v)
    c = v;
elseif isstruct(v)
    c = num2cell(v);
else
    c = num2cell(v);
end
end
