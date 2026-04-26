function [scenario, config, sensors, metas] = loadRunFile(runName)
%loadRunFile  Load a modular run file and build everything.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Reads a run file from config/runs/<runName>.json, which references
%  individual sensor, target, terrain, and tracker configs. Builds the
%  complete trackingScenario with all components assembled.
%
%  This is the NEW modular path — each component is independently defined
%  and combined through the run file. For the legacy catalog path, see
%  loadScenario.
%
%  USAGE
%    [scenario, config, sensors, metas] = loadRunFile("dasr_baseline");
%    [scenario, config, sensors, metas] = loadRunFile("my_run");
%
%  RUN FILE FORMAT (config/runs/<n>.json)
%    sensors   : ["PSR/default_PSR", "SSR/default_SSR"]  — sensor configs
%    targets   : "crossing_pair/default_crossing_pair"    — target config
%    terrain   : "mountain/default_mountain"              — terrain config
%    trackers  : ["GNN/default_GNN"]                      — tracker configs
%    degradation : {enabled, type}
%    platforms : {aircraft: {start_pos, heading_deg, speed_kmh}} (optional)
%    output    : {show_visuals, save_results, ...}
%
%  OUTPUTS
%    scenario : trackingScenario object ready for runDetections
%    config   : merged config struct (compatible with existing pipeline)
%    sensors  : struct grouped by platform (sensors.tower, sensors.aircraft)
%    metas    : sensor metadata struct
%
%  See also: loadScenario, buildSensor, runSingleScenario
arguments
    runName (1,1) string
end

% Dynamically resolve root for both console and deployed (.exe) environments
if isdeployed
    root = pwd;
else
    root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
end

configDir = fullfile(root, 'config');
%% 1. Load run file
if ~endsWith(runName, ".json")
    runName = runName + ".json";
end
runPath = fullfile(configDir, 'runs', runName);
if ~isfile(runPath)
    error('loadRunFile:notFound', 'Run file not found: %s', runPath);
end
fprintf('\n[RUN] Loading: %s\n', runPath);
runDef = jsondecode(fileread(runPath));
if isfield(runDef, 'description')
    fprintf('[RUN] %s\n', runDef.description);
end
%% 2. Load individual sensors
sensorPaths = {};
if isfield(runDef, 'sensors')
    if ischar(runDef.sensors) || isstring(runDef.sensors)
        sensorPaths = {char(runDef.sensors)};
    elseif iscell(runDef.sensors)
        sensorPaths = runDef.sensors;
    elseif isempty(runDef.sensors)
        % GUI-generated runs can emit "sensors": [] which jsondecode
        % returns as numeric [] — guard against cellstr([]) breaking.
        sensorPaths = {};
    else
        sensorPaths = cellstr(runDef.sensors);
    end
end
sensors = struct();
metas = struct();
sensorIndex = 0;
for i = 1:numel(sensorPaths)
    sPath = char(sensorPaths{i});
    if ~endsWith(sPath, '.json')
        sPath = [sPath '.json'];
    end
    fullPath = fullfile(configDir, 'sensors', sPath);
    if ~isfile(fullPath)
        error('loadRunFile:sensorNotFound', 'Sensor config not found: %s', fullPath);
    end
    sDef = jsondecode(fileread(fullPath));
    sensorIndex = sensorIndex + 1;
    % Determine platform
    platformName = 'tower';
    if isfield(sDef, 'platform')
        platformName = lower(char(sDef.platform));
    end
    % Build sensor using existing factory
    sType = 'PSR';
    if isfield(sDef, 'type'); sType = upper(char(sDef.type)); end
    sParams = struct();
    if isfield(sDef, 'params'); sParams = sDef.params; end
    % Convert params struct to name-value pairs (buildSensor uses varargin)
    nvPairs = {};
    if isstruct(sParams)
        fields = fieldnames(sParams);
        for f = 1:numel(fields)
            val = sParams.(fields{f});
            % JSON arrays come in as column vectors — transpose to row
            if isnumeric(val) && iscolumn(val) && numel(val) > 1
                val = val(:)';
            end
            % fov needs to be column vector for buildSensor
            if strcmpi(fields{f}, 'fov') && isnumeric(val)
                val = val(:);
            end
            nvPairs = [nvPairs, {fields{f}, val}]; %#ok<AGROW>
        end
    end
    [sObj, ~] = trackbench.sensors.buildSensor(sensorIndex, sType, nvPairs{:});
    % Add to platform group
    if ~isfield(sensors, platformName)
        sensors.(platformName) = {};
        metas.(platformName) = {};
    end
    sensors.(platformName){end+1} = sObj;
    % Build metadata
    meta = struct();
    meta.sensorIndex = sensorIndex;
    meta.type = sType;
    meta.name = '';
    if isfield(sDef, 'name'); meta.name = char(sDef.name); end
    try meta.maxRange = sObj.RangeLimits(2); catch; meta.maxRange = 111120; end
    try meta.frequency = getFreqForType(sType); catch; meta.frequency = 2.8e9; end
    % Allow sensor config to override default frequency
    if isfield(sDef, 'frequency_hz')
        meta.frequency = sDef.frequency_hz;
    elseif isfield(sParams, 'frequency_hz')
        meta.frequency = sParams.frequency_hz;
    end
    metas.(platformName){end+1} = meta;
    fprintf('[RUN] Sensor %d: %s (%s) → platform "%s"\n', ...
        sensorIndex, meta.name, sType, platformName);
end
%% 3. Load target config
targetDef = struct('duration_s', 50, 'targets', {{}});
if isfield(runDef, 'targets') && ~isempty(runDef.targets)
    tPath = char(runDef.targets);
    if ~endsWith(tPath, '.json'); tPath = [tPath '.json']; end
    fullPath = fullfile(configDir, 'targets', tPath);
    if ~isfile(fullPath)
        error('loadRunFile:targetsNotFound', 'Target config not found: %s', fullPath);
    end
    targetDef = jsondecode(fileread(fullPath));
    fprintf('[RUN] Targets: %s\n', fullPath);
end
%% 4. Load terrain config — supports legacy string OR new multi-region struct
%   v3.5 §5a:
%     Legacy:  "terrain": "mountain/default_mountain"
%     New:     "terrain": {"fallback": "...", "regions": [{...}]}
%   parseTerrainField handles both. For a legacy string it returns a
%   single-fallback / empty-regions tuple, preserving exact pre-5a
%   behavior for every existing run file.
terrainDef = struct('terrain_type','water','terrain_scale',1.0, ...
    'terrain_occlusion',false,'horizon_masking',false, ...
    'clutter_density',0,'refraction_factor',1.333);
terrainRegions = {};   % populated below in the multi-region branch
if isfield(runDef, 'terrain') && ~isempty(runDef.terrain)
    [terrainDef, terrainRegions] = parseTerrainField(runDef.terrain, configDir);
end
%% 5. Load tracker configs
trackerConfigs = {};
if isfield(runDef, 'trackers')
    trkPaths = {};
    if ischar(runDef.trackers) || isstring(runDef.trackers)
        trkPaths = {char(runDef.trackers)};
    elseif iscell(runDef.trackers)
        trkPaths = runDef.trackers;
    elseif isempty(runDef.trackers)
        % GUI-generated runs can emit "trackers": [] — guard against
        % cellstr([]) breaking the load. Mirrors the sensors case above.
        trkPaths = {};
    else
        trkPaths = cellstr(runDef.trackers);
    end
    for i = 1:numel(trkPaths)
        tPath = char(trkPaths{i});
        if ~endsWith(tPath, '.json'); tPath = [tPath '.json']; end
        fullPath = fullfile(configDir, 'trackers', tPath);
        if ~isfile(fullPath)
            error('loadRunFile:trackerNotFound', 'Tracker config not found: %s', fullPath);
        end
        tDef = jsondecode(fileread(fullPath));
        trackerConfigs{end+1} = tDef; %#ok<AGROW>
        fprintf('[RUN] Tracker: %s (%s + %s)\n', fullPath, ...
            tDef.tracker_type, tDef.filter_model);
    end
end
%% 6. Assemble unified config (compatible with existing pipeline)
config = struct();
% Scenario params from target file
config.scenario.mode = '3D';
config.scenario.duration_s = 50;
if isfield(targetDef, 'duration_s')
    config.scenario.duration_s = targetDef.duration_s;
end
config.scenario.num_targets = 0;
if isfield(targetDef, 'targets')
    if iscell(targetDef.targets)
        config.scenario.num_targets = numel(targetDef.targets);
    elseif isstruct(targetDef.targets)
        config.scenario.num_targets = numel(targetDef.targets);
    end
end
% Degradation
config.degradation.enabled = false;
config.degradation.type = 'rain';
config.degradation.rain_rate_mmhr = 16;
% v3.5 §5b — has_fallback flag tells runDetections whether the fallback
% weather record exists. Distinguishes "weather everywhere" from
% "region-only weather, clear elsewhere" so the per-detection resolver
% can skip fallback effects in the latter case.
config.degradation.has_fallback = false;
weatherRegions = {};   % v3.5 §5a — populated below if runDef has weather regions
if isfield(runDef, 'degradation')
    deg = runDef.degradation;
    
    % --- Weather config — supports legacy string OR new multi-region struct ---
    %   v3.5 §5a:
    %     Legacy:  "weather": "rain/default_rain"  (or "none")
    %     New:     "weather": {"fallback": "...", "regions": [{...}]}
    %   parseWeatherField handles both. The fallback may be "none"
    %   (or omitted) when only regions matter; in that case we still
    %   flag degradation enabled so 5b's per-detection resolver runs.
    if isfield(deg, 'weather') && ~isempty(deg.weather)
        [weatherFallbackDef, weatherRegions] = parseWeatherField(deg.weather, configDir);
        if ~isempty(fieldnames(weatherFallbackDef))
            weatherDef = weatherFallbackDef;
            config.degradation.enabled = true;
            config.degradation.has_fallback = true;
            if isfield(weatherDef, 'type');              config.degradation.type = char(weatherDef.type); end
            if isfield(weatherDef, 'rain_rate_mmhr');    config.degradation.rain_rate_mmhr = weatherDef.rain_rate_mmhr; end
            if isfield(weatherDef, 'pd_floor');          config.degradation.pd_floor = weatherDef.pd_floor; end
            if isfield(weatherDef, 'clutter_multiplier');config.degradation.clutter_multiplier = weatherDef.clutter_multiplier; end
            if isfield(weatherDef, 'storm_start_s');     config.degradation.storm_start_s = weatherDef.storm_start_s; end
            if isfield(weatherDef, 'storm_end_s');       config.degradation.storm_end_s = weatherDef.storm_end_s; end
            if isfield(weatherDef, 'active_type');       config.degradation.active_type = char(weatherDef.active_type); end
            fprintf('[RUN] Weather (fallback): %s\n', config.degradation.type);
        elseif ~isempty(weatherRegions)
            % Region-only weather (fallback="none" + non-empty regions).
            % Flag degradation enabled so 5b's resolver runs; has_fallback
            % stays FALSE so the resolver knows to leave outside-of-region
            % detections alone. legacy config.degradation.* fields keep
            % their no-op defaults.
            config.degradation.enabled = true;
            config.degradation.has_fallback = false;
            fprintf('[RUN] Weather: %d region(s), no fallback (clear outside regions)\n', ...
                numel(weatherRegions));
        end
    end
    
    % --- Legacy support: "enabled"/"type" inline (still works) ---
    if isfield(deg, 'enabled')
        config.degradation.enabled = logical(deg.enabled);
    end
    if isfield(deg, 'type')
        config.degradation.type = char(deg.type);
    end
    if isfield(deg, 'rain_rate_mmhr');    config.degradation.rain_rate_mmhr = deg.rain_rate_mmhr; end
    if isfield(deg, 'pd_floor');          config.degradation.pd_floor = deg.pd_floor; end
    if isfield(deg, 'noise_ceiling');     config.degradation.noise_ceiling = deg.noise_ceiling; end
    if isfield(deg, 'clutter_multiplier');config.degradation.clutter_multiplier = deg.clutter_multiplier; end
    if isfield(deg, 'storm_start_s');     config.degradation.storm_start_s = deg.storm_start_s; end
    if isfield(deg, 'storm_end_s');       config.degradation.storm_end_s = deg.storm_end_s; end
    if isfield(deg, 'active_type');       config.degradation.active_type = char(deg.active_type); end
end
% Storm window defaults
if ~isfield(config.degradation, 'storm_start_s'); config.degradation.storm_start_s = 5; end
if ~isfield(config.degradation, 'storm_end_s');   config.degradation.storm_end_s = 45; end
if ~isfield(config.degradation, 'active_type');   config.degradation.active_type = 'step'; end
% Environment from terrain file, then override with run file degradation toggles
config.environment = terrainDef;
% v3.5 §5a — multi-region collections. Both are empty cells in legacy
% runs (single-terrain / single-weather), so runDetections's per-detection
% resolver in 5b takes the fallback every time — bit-for-bit identical
% behavior to pre-5a for every existing run file.
config.environmentRegions = terrainRegions;
config.degradationRegions = weatherRegions;
if isfield(runDef, 'degradation')
    deg = runDef.degradation;
    % Run file degradation block overrides terrain file defaults
    if isfield(deg, 'terrain_occlusion'); config.environment.terrain_occlusion = logical(deg.terrain_occlusion); end
    if isfield(deg, 'horizon_masking');   config.environment.horizon_masking = logical(deg.horizon_masking); end
    if isfield(deg, 'ground_clutter');    config.environment.ground_clutter = logical(deg.ground_clutter); end
    if isfield(deg, 'doppler_fade');      config.environment.doppler_fade = logical(deg.doppler_fade); end
    % v3.4.2: rcs_range_filter restored as OPT-IN (default OFF). When ON,
    % runDetections applies a deterministic R_eff cutoff in addition to the
    % sensor-native Swerling model. Used by TC-05 RCS validation.
    if isfield(deg, 'rcs_range_filter');  config.environment.rcs_range_filter = logical(deg.rcs_range_filter); end
end
% Load shared tracker globals from config/trackers/tracker_globals.json
% These are environment-level params (volume, beta, Pd) shared by ALL trackers.
globalsPath = fullfile(configDir, 'trackers', 'tracker_globals.json');
if isfile(globalsPath)
    tg = jsondecode(fileread(globalsPath));
    config.tracker_global = struct();
    if isfield(tg, 'max_num_tracks'); config.tracker_global.max_num_tracks = tg.max_num_tracks; end
    if isfield(tg, 'volume');         config.tracker_global.volume = tg.volume; end
    if isfield(tg, 'beta');           config.tracker_global.beta = tg.beta; end
    if isfield(tg, 'detection_probability')
        config.tracker_global.detection_probability = tg.detection_probability;
    end
    if isfield(tg, 'filter')
        config.filter_params = tg.filter;
    end
    % volume/beta are per-tracker (GNN/JPDA use them differently),
    % but we keep defaults here for backward compatibility
    if isfield(tg, 'volume'); config.tracker_global.volume = tg.volume;
    else;                     config.tracker_global.volume = 1e9; end
    if isfield(tg, 'beta');   config.tracker_global.beta = tg.beta;
    else;                     config.tracker_global.beta = 1e-14; end
    fprintf('[RUN] Tracker globals: %s\n', globalsPath);
else
    % Fallback defaults if file is missing
    config.tracker_global = struct('max_num_tracks',500,'volume',1e9,'beta',1e-14, ...
        'detection_probability',struct('ideal',0.9,'degraded',0.7));
    config.filter_params = struct('init_speed_kmh',900,'imm_transition_prob',0.97, ...
        'scale_accel_horz',30,'scale_accel_vert',20,'scale_omega_dot',30);    
    fprintf('[RUN] Tracker globals: using built-in defaults (tracker_globals.json not found)\n');
end
% Tracker-specific params from individual tracker configs
if ~isempty(trackerConfigs)
    tc = trackerConfigs{1};
    % Filter override: per-tracker filter params override globals
    % This is critical for autoTuneTracker output — the tuner finds
    % optimal filter params for a specific scenario and saves them
    % in the tracker JSON's 'filter' block.
    if isfield(tc, 'filter')
        if ~isfield(config, 'filter_params')
            config.filter_params = tc.filter;
        else
            % Merge: tracker-specific values override globals
            fNames = fieldnames(tc.filter);
            for f = 1:numel(fNames)
                config.filter_params.(fNames{f}) = tc.filter.(fNames{f});
            end
        end
    end
    % Build tracker_params.ideal and .degraded from tracker params
    if isfield(tc, 'params')
        config.tracker_params.ideal = tc.params;
        config.tracker_params.degraded = tc.params;
        % Widen gates for degraded
        if isfield(tc.params, 'gate')
            config.tracker_params.degraded.gate = tc.params.gate * 1.8;
        end
        if isfield(tc.params, 'gate_jpda')
            config.tracker_params.degraded.gate_jpda = tc.params.gate_jpda * 1.8;
        end
    end
    % Build trackers_to_run from loaded tracker configs
    config.trackers_to_run = struct( ...
        'gnn_cv',false,'gnn_imm',false,'tomht_cv',false, ...
        'tomht_imm',false,'jpda_cv',false,'jpda_imm',false);
    for i = 1:numel(trackerConfigs)
        tc = trackerConfigs{i};
        key = lower(sprintf('%s_%s', tc.tracker_type, tc.filter_model));
        if isfield(config.trackers_to_run, key)
            config.trackers_to_run.(key) = true;
        end
    end
else
    % Defaults
    config.tracker_global = struct('max_num_tracks',500,'volume',1e9,'beta',1e-14, ...
        'detection_probability',struct('ideal',0.9,'degraded',0.7));
    config.tracker_params.ideal = struct('gate',45,'far_gnn',1e-6,'far_mht',1e-6, ...
        'far_jpda',1e-6,'confirm_threshold',20,'delete_threshold',-5, ...
        'tomht_threshold_multiplier',[0.2 1 1],'max_branches',5, ...
        'beta_jpda',1e-14,'gate_jpda',45,'time_tolerance_jpda',0.05,'num_tracks_jpda',500);
    config.tracker_params.degraded = config.tracker_params.ideal;
    config.filter_params = struct('init_speed_kmh',900,'imm_transition_prob',0.97, ...
        'scale_accel_horz',30,'scale_accel_vert',20,'scale_omega_dot',30);
    config.trackers_to_run = struct('gnn_cv',false,'gnn_imm',true, ...
        'tomht_cv',false,'tomht_imm',false,'jpda_cv',false,'jpda_imm',false);
end
% Select active params
if config.degradation.enabled
    config.active_params = config.tracker_params.degraded;
    config.active_params.pd = config.tracker_global.detection_probability.degraded;
    fprintf('[RUN] Using DEGRADED tracker parameters\n');
else
    config.active_params = config.tracker_params.ideal;
    config.active_params.pd = config.tracker_global.detection_probability.ideal;
    fprintf('[RUN] Using IDEAL tracker parameters\n');
end
% Output
config.output = struct('show_visuals',true,'animate_visuals',true, ...
    'save_results',true,'save_figures',true,'print_diagnostics',true, ...
    'results_directory','results');
if isfield(runDef, 'output')
    outFields = fieldnames(runDef.output);
    for f = 1:numel(outFields)
        config.output.(outFields{f}) = runDef.output.(outFields{f});
    end
end
% Data logging / cache
% Default: regenerate detections every run. Set cache.use_cached_detections
% in the run file to reuse saved detections (faster tracker tuning).
config.data_logging.use_saved_datalog = false;
config.data_logging.save_after_generation = true;
config.data_logging.datalog_file = sprintf('cache/%s.mat', erase(string(runName), '.json'));
if isfield(runDef, 'cache') && isstruct(runDef.cache)
    if isfield(runDef.cache, 'use_cached_detections')
        config.data_logging.use_saved_datalog = logical(runDef.cache.use_cached_detections);
    end
    if isfield(runDef.cache, 'save_detections')
        config.data_logging.save_after_generation = logical(runDef.cache.save_detections);
    end
end
if config.data_logging.use_saved_datalog
    fprintf('[RUN] Cache: will REUSE detections from %s (if exists)\n', config.data_logging.datalog_file);
else
    fprintf('[RUN] Cache: will REGENERATE detections fresh\n');
end
% Store tracker configs for runSingleScenario
config.tracker_configs = trackerConfigs;
config.run_name = erase(string(runName), '.json');
config.sensor_config_name = strjoin(string(sensorPaths), ' + ');
%% 7. Build trackingScenario
scenario = trackingScenario;
platformNames = fieldnames(sensors);
% Platform definitions from run file
% Handles three cases:
%   "platforms": {}              → jsondecode returns [] (empty), no moving platforms
%   "platforms": {"placeholder":0} → legacy workaround, strip it
%   "platforms": {"aircraft":{...}} → real moving platform definition
platformDefs = struct();
if isfield(runDef, 'platforms')
    p = runDef.platforms;
    if isstruct(p) && ~isempty(fieldnames(p))
        platformDefs = p;
        % Strip placeholder workaround for MATLAB empty struct encoding
        if isfield(platformDefs, 'placeholder')
            platformDefs = rmfield(platformDefs, 'placeholder');
        end
    end
    % If p is [] (from empty JSON {}), platformDefs stays as empty struct → all stationary
end
primaryUpdateRate = [];
for p = 1:numel(platformNames)
    pName = platformNames{p};
    sensorList = sensors.(pName);
    plat = platform(scenario, 'Sensors', sensorList);
    if isstruct(platformDefs) && ~isempty(fieldnames(platformDefs)) && isfield(platformDefs, pName)
        pDef = platformDefs.(pName);
        pStart = [0 0 0]; pHeading = 90; pSpeed = 250;
        if isfield(pDef, 'start_pos');   pStart = reshape(pDef.start_pos, 1, []); end
        if isfield(pDef, 'heading_deg'); pHeading = pDef.heading_deg; end
        if isfield(pDef, 'speed_kmh');   pSpeed = pDef.speed_kmh * 1000 / 3600; end
        dx = pSpeed * sind(pHeading);
        dy = pSpeed * cosd(pHeading);
        T = config.scenario.duration_s;
        pEnd = pStart + [dx dy 0] * T;
        plat.Trajectory = waypointTrajectory( ...
            'Waypoints', [pStart; pEnd], ...
            'TimeOfArrival', [0; T], ...
            'Velocities', [dx dy 0; dx dy 0], ...
            'AutoBank', true, 'AutoPitch', true);
        fprintf('[RUN] Platform "%s": %d sensor(s) — MOVING → hdg %.0f° @ %.0f km/h\n', ...
            pName, numel(sensorList), pHeading, pSpeed*3.6);
    else
        fprintf('[RUN] Platform "%s": %d sensor(s) — STATIONARY\n', pName, numel(sensorList));
    end
    if isempty(primaryUpdateRate) && ~isempty(sensorList)
        if isprop(sensorList{1}, 'UpdateRate')
            primaryUpdateRate = sensorList{1}.UpdateRate;
        end
    end
end
if ~isempty(primaryUpdateRate)
    scenario.UpdateRate = primaryUpdateRate;
end
%% 8. Build targets
if isfield(targetDef, 'targets')
    tgts = targetDef.targets;
    if isstruct(tgts); tgts = num2cell(tgts); end
    numTargets = numel(tgts);
    duration = config.scenario.duration_s;
    fprintf('[RUN] Building %d target(s) over %.0fs\n', numTargets, duration);
    for i = 1:numTargets
        if iscell(tgts); tDef = tgts{i}; else; tDef = tgts(i); end
        trackbench.scenario.addTargetFromDef(scenario, tDef, duration, i);
    end
end
%% 9. Attach terrain
terrainType = terrainDef.terrain_type;
elevScale = 1.0;
if isfield(terrainDef, 'terrain_scale'); elevScale = terrainDef.terrain_scale; end
try
    scenBounds = computeScenarioBounds(scenario);
    [Zterrain, boundary, Xg, Yg] = trackbench.environment.generateTerrain( ...
        terrainType, scenBounds, elevScale);
    % v3.5 §5b — stamp region heightmaps over the fallback layer so
    % MATLAB's groundSurface/SurfaceManager sees a single composite
    % grid. Empty terrainRegions → composeHeightmap returns Zterrain
    % unchanged (legacy bit-for-bit behavior).
    if ~isempty(terrainRegions)
        Zterrain = trackbench.environment.composeHeightmap( ...
            Zterrain, terrainRegions, Xg, Yg, scenBounds);
    end
    groundSurface(scenario, 'Terrain', Zterrain, 'Boundary', boundary);
    % Raise stationary platforms to terrain
    allPlats = scenario.Platforms;
    for pp = 1:numel(allPlats)
        platObj = allPlats{pp};
        if ~isa(platObj.Trajectory, 'waypointTrajectory')
            pPos = platObj.Trajectory.Position(:)';
            terrZ = interp2(Xg, Yg, Zterrain, pPos(1), pPos(2), 'linear', 0);
            if terrZ < -1
                platObj.Trajectory.Position = [pPos(1), pPos(2), terrZ];
                fprintf('[RUN] Platform %d raised to terrain surface (%.0fm ASL)\n', pp, -terrZ);
            end
        end
    end
    tg = struct('Z', Zterrain, 'boundary', boundary, 'X', Xg, 'Y', Yg);
    config.terrainGrid = tg;
    config.environment.terrainGrid = tg;
    fprintf('[RUN] Terrain: %s (occlusion=%d)\n', terrainType, ...
        scenario.SurfaceManager.UseOcclusion);
catch ME
    warning('loadRunFile:terrainFailed', 'Terrain failed: %s', ME.message);
    config.terrainGrid = [];
end
%% 10. Validate
try
    [scanOk, scanInfo] = trackbench.scenario.validateScanCoverage(scenario, config.scenario.duration_s);
    if ~scanOk
        warning('loadRunFile:fewScans', '%s', scanInfo.message);
    end
catch; end
try
    trackbench.validation.validateScenarioConfig(config, scenario, sensors, metas);
catch; end
%% Summary
totalSensors = sensorIndex;
fprintf('[RUN] Ready: %d sensor(s), %d target(s), %.0fs, terrain=%s\n', ...
    totalSensors, config.scenario.num_targets, config.scenario.duration_s, terrainType);
end % main function
%% ========================================================================
function bounds = computeScenarioBounds(scenario)
    allPos = [0 0 0];
    plats = scenario.Platforms;
    for p = 1:numel(plats)
        try
            pos0 = plats{p}.Trajectory.Position(:)';
            allPos = [allPos; pos0]; %#ok<AGROW>
        catch; end
        try
            traj = plats{p}.Trajectory;
            if isprop(traj, 'Waypoints')
                allPos = [allPos; traj.Waypoints]; %#ok<AGROW>
            end
        catch; end
    end
    maxExtent = max(vecnorm(allPos(:,1:2), 2, 2));
    halfSpan = max(maxExtent * 1.15, 130000);
    bounds = [-halfSpan, halfSpan; -halfSpan, halfSpan];
end
function freq = getFreqForType(sType)
    switch upper(sType)
        case {'PSR','ASR','WEATHER'};     freq = 2.8e9;
        case {'SSR','MSSR'};              freq = 1.03e9;
        case 'ARSR';                      freq = 1.3e9;
        case {'PAR','FIRE_CONTROL','AESA','TWS'}; freq = 9.0e9;
        case 'MARITIME';                  freq = 9.4e9;
        otherwise;                        freq = 2.8e9;
    end
end

%% ========================================================================
%  v3.5 §5a — multi-region terrain & weather parsers
%  parseTerrainField / parseWeatherField each accept either:
%    - a string  (legacy single-component shape)
%    - a struct  (new {fallback, regions[]} shape)
%  and return (fallbackDef, regions) where regions is a cell array of
%  region structs {.config_path, .name, .polygon_xy, .def}. The .def
%  field caches the loaded inner config so 5b's resolver doesn't have
%  to re-read referenced files per detection.
%% ========================================================================
function [fallbackDef, regions] = parseTerrainField(field, configDir)
%parseTerrainField  Parse a runDef.terrain value (legacy string or
%                   v3.5 multi-region struct). Returns:
%    fallbackDef  struct from the fallback terrain config file. Always
%                 populated when input is non-empty — terrain has no
%                 "none" sentinel; every scenario must resolve to some
%                 terrain to drive the heightmap.
%    regions      cell array of {config_path, name, polygon_xy, def}
%                 structs. Empty for legacy single-terrain runs.
    fallbackDef = struct();
    regions = {};

    if ischar(field) || isstring(field)
        % Legacy string shape — single terrain, no regions.
        fbStr = char(field);
        if ~isempty(fbStr)
            fallbackDef = loadTerrainFile(fbStr, configDir);
            fprintf('[RUN] Terrain: %s (%s)\n', fbStr, fallbackDef.terrain_type);
        end
        return;
    end

    if isstruct(field)
        % v3.5 §5a multi-region shape: {fallback, regions: [...]}
        if isfield(field, 'fallback') && ~isempty(field.fallback)
            fbStr = char(field.fallback);
            fallbackDef = loadTerrainFile(fbStr, configDir);
            fprintf('[RUN] Terrain (fallback): %s (%s)\n', ...
                fbStr, fallbackDef.terrain_type);
        else
            error('loadRunFile:noFallbackTerrain', ...
                'terrain.fallback is required (terrain has no "none" option).');
        end
        if isfield(field, 'regions') && ~isempty(field.regions)
            regs = field.regions;
            % jsondecode returns a struct array for homogeneous JSON
            % object arrays, or a cell array if shapes vary. Normalize.
            if isstruct(regs); regs = num2cell(regs); end
            for i = 1:numel(regs)
                rec = parseRegion(regs{i});
                rec.def = loadTerrainFile(char(rec.config_path), configDir);
                regions{end+1} = rec; %#ok<AGROW>
                fprintf('[RUN] Terrain region %d: "%s" -> %s (%d pts)\n', ...
                    i, rec.name, rec.config_path, size(rec.polygon_xy, 1));
            end
        end
        return;
    end

    warning('loadRunFile:badTerrainField', ...
        'Unrecognized terrain field type "%s" — using built-in defaults.', ...
        class(field));
end

function [fallbackDef, regions] = parseWeatherField(field, configDir)
%parseWeatherField  Parse a runDef.degradation.weather value (legacy
%                   string or v3.5 multi-region struct). Returns:
%    fallbackDef  struct from the fallback weather config file. Empty
%                 struct() (numel(fieldnames)==0) when fallback is
%                 "none" or omitted — the "no global weather" case.
%    regions      cell array of region structs (see parseRegion).
%                 Empty for legacy single-weather runs.
    fallbackDef = struct();
    regions = {};

    if ischar(field) || isstring(field)
        % Legacy string shape.
        fbStr = char(field);
        if isempty(fbStr) || strcmpi(fbStr, 'none')
            return;   % weather disabled
        end
        fallbackDef = loadWeatherFile(fbStr, configDir);
        return;
    end

    if isstruct(field)
        % v3.5 §5a multi-region shape.
        if isfield(field, 'fallback') && ~isempty(field.fallback)
            fbStr = char(field.fallback);
            if ~strcmpi(fbStr, 'none')
                fallbackDef = loadWeatherFile(fbStr, configDir);
            end
        end
        if isfield(field, 'regions') && ~isempty(field.regions)
            regs = field.regions;
            if isstruct(regs); regs = num2cell(regs); end
            for i = 1:numel(regs)
                rec = parseRegion(regs{i});
                rec.def = loadWeatherFile(char(rec.config_path), configDir);
                regions{end+1} = rec; %#ok<AGROW>
                fprintf('[RUN] Weather region %d: "%s" -> %s (%d pts)\n', ...
                    i, rec.name, rec.config_path, size(rec.polygon_xy, 1));
            end
        end
        return;
    end

    warning('loadRunFile:badWeatherField', ...
        'Unrecognized degradation.weather field type "%s" — treating as "none".', ...
        class(field));
end

function rec = parseRegion(rDef)
%parseRegion  Common region-struct parser shared by terrain & weather.
%             Extracts name, config path, and polygon_xy. Validates the
%             polygon shape (Nx2 double, ≥3 distinct points). Sets the
%             .def field to an empty struct — caller fills it via
%             loadTerrainFile/loadWeatherFile so this function stays
%             agnostic to the inner-record kind.
    if ~isstruct(rDef)
        error('loadRunFile:badRegion', ...
            'Region entry must be a JSON object, got %s.', class(rDef));
    end
    rec = struct('config_path', "", 'name', "", ...
                 'polygon_xy', zeros(0, 2), 'def', struct());
    if isfield(rDef, 'config'); rec.config_path = string(char(rDef.config)); end
    if isfield(rDef, 'name');   rec.name        = string(char(rDef.name));   end
    if isfield(rDef, 'polygon_xy')
        pxy = rDef.polygon_xy;
        if ~isnumeric(pxy)
            error('loadRunFile:badPolygon', ...
                'Region "%s" polygon_xy must be numeric, got %s.', ...
                rec.name, class(pxy));
        end
        % jsondecode of an Nx2 array of [x,y] pairs comes back as Nx2.
        % If somehow transposed (2xN), flip to Nx2 — a polygon of
        % exactly 2 points is degenerate either way (caught below).
        if size(pxy, 2) == 2
            rec.polygon_xy = pxy;
        elseif size(pxy, 1) == 2 && size(pxy, 2) >= 3
            rec.polygon_xy = pxy';
        else
            rec.polygon_xy = pxy;   % let validation below catch it
        end
    end
    if size(rec.polygon_xy, 1) < 3 || size(rec.polygon_xy, 2) ~= 2
        warning('loadRunFile:degeneratePolygon', ...
            'Region "%s" has degenerate polygon (%dx%d) — will be skipped at resolve time.', ...
            rec.name, size(rec.polygon_xy, 1), size(rec.polygon_xy, 2));
    end
    if strlength(rec.config_path) == 0
        error('loadRunFile:missingRegionConfig', ...
            'Region "%s" missing required "config" field.', rec.name);
    end
end

function def = loadTerrainFile(relPath, configDir)
%loadTerrainFile  Read a config/terrain/<TYPE>/<file>.json into a struct.
    if ~endsWith(relPath, '.json'); relPath = [relPath '.json']; end
    fullPath = fullfile(configDir, 'terrain', relPath);
    if ~isfile(fullPath)
        error('loadRunFile:terrainNotFound', ...
            'Terrain config not found: %s', fullPath);
    end
    def = jsondecode(fileread(fullPath));
end

function def = loadWeatherFile(relPath, configDir)
%loadWeatherFile  Read a config/weather/<TYPE>/<file>.json into a struct.
    if ~endsWith(relPath, '.json'); relPath = [relPath '.json']; end
    fullPath = fullfile(configDir, 'weather', relPath);
    if ~isfile(fullPath)
        error('loadRunFile:weatherNotFound', ...
            'Weather config not found: %s', fullPath);
    end
    def = jsondecode(fileread(fullPath));
end