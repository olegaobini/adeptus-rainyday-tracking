function config = normalizeConfig(config)
%normalizeConfig Stage 3: canonical normalization.

if ~isfield(config.scenario, 'frame')
    config.scenario.frame = 'NED';
end

config = trackbench.loader.normalizeUnits(config);
config = trackbench.loader.resolveRelativePaths(config);

if isfield(config.scenario, 'duration_min')
    config.scenario.duration_s = double(config.scenario.duration_min) * 60;
end

% Runtime compatibility projection for current simulation stack.
if isfield(config, 'trackers')
    config.tracker_global = config.trackers.global;
    config.tracker_params = config.trackers.params;
    config.trackers_to_run = config.trackers.run;
end

if ~isfield(config, 'degradation')
    config.degradation = struct('enabled', false, 'type', 'none');
end
if ~isfield(config.degradation, 'enabled')
    config.degradation.enabled = false;
end

if config.degradation.enabled
    config.active_params = config.tracker_params.degraded;
    config.active_params.pd = config.tracker_global.detection_probability.degraded;
else
    config.active_params = config.tracker_params.ideal;
    config.active_params.pd = config.tracker_global.detection_probability.ideal;
end
end
