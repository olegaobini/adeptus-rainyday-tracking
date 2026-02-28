function runPlan = loadAndPrepare(configName)
%loadAndPrepare Full config preparation pipeline.
%   runPlan = trackbench.loader.loadAndPrepare("scenarios/my_test")
%   runPlan = trackbench.loader.loadAndPrepare("sweeps/weather_study")

arguments
    configName (1,1) string
end

fprintf('[LOADER] Ingest: %s\n', configName);
config = trackbench.loader.ingestConfig(configName);

fprintf('[LOADER] Validate\n');
trackbench.loader.validateConfig(config);

fprintf('[LOADER] Normalize\n');
config = trackbench.loader.normalizeConfig(config);

fprintf('[LOADER] Expand\n');
runPlan = trackbench.loader.expandRunPlan(config);

fprintf('[LOADER] Prepared %d run(s)\n', numel(runPlan));
end
