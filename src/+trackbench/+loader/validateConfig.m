function validateConfig(config)
%validateConfig Stage 2: validate structure/types/enums.

arguments
    config (1,1) struct
end

required = {'scenario', 'sensors', 'truth', 'trackers'};
for i = 1:numel(required)
    if ~isfield(config, required{i})
        error('validateConfig:missingField', 'Missing required field: %s', required{i});
    end
end

trackbench.loader.validateScenario(config.scenario);
trackbench.loader.validateSensors(config.sensors);
trackbench.loader.validateTruth(config.truth);
trackbench.loader.validateTrackers(config.trackers);

if isfield(config, 'sweep')
    trackbench.loader.validateSweepStructure(config.sweep);
end
end
