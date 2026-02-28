function config = loadConfig(configName)
%loadConfig Compatibility wrapper for single-run configs.

arguments
    configName (1,1) string
end

runPlan = trackbench.loader.loadAndPrepare(configName);
if numel(runPlan) ~= 1
    error('loadConfig:multipleRuns', ...
        'Config "%s" expands to %d runs. Use trackbench.loader.loadAndPrepare.', ...
        configName, numel(runPlan));
end

config = runPlan(1);
end
