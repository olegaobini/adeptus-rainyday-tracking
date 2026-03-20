function [scenario, config, sensors, metas] = loadScenario(scenarioName, catalogName, sensorOverride)
%loadScenario  Load a named scenario from the catalog and build everything.
%
% Reads the scenario definition from the catalog, loads the base config,
% applies overrides, loads the sensor catalog, builds a trackingScenario
% with platforms and targets.
%
% CHANGE LOG
%   - After terrain generation, stationary sensor platforms are raised to
%     sit on the terrain surface via Trajectory.Position (not InitialPosition,
%     which doesn't exist on fusion.scenario.Platform).
%   - auto_terrain_effects: when ON in default.json, changing terrain_type
%     auto-enables the correct environment flags (occlusion, masking,
%     clutter). Catalog overrides still take priority over auto-resolve.
%   - Pre-flight validateScenarioConfig runs after scenario build to catch
%     buried sensors, underground targets, and other red flags.

arguments
    scenarioName   (1,1) string
    catalogName    (1,1) string = "scenario_catalog"
    sensorOverride (1,1) string = ""
end

%% 1. Load base config
config = trackbench.config.loadConfig("default");

%% 2. Load scenario catalog
if ~endsWith(catalogName, ".json")
    catalogName = catalogName + ".json";
end
catPath = fullfile(pwd, "config", "scenarios", catalogName);
if ~isfile(catPath)
    error('loadScenario:catalogNotFound', 'Catalog not found: %s', catPath);
end

catalog = jsondecode(fileread(catPath));
if ~isfield(catalog.scenarios, scenarioName)
    available = strjoin(string(fieldnames(catalog.scenarios)), ', ');
    error('loadScenario:scenarioNotFound', ...
        'Scenario "%s" not in catalog. Available: %s', scenarioName, available);
end

scenDef = catalog.scenarios.(scenarioName);
fprintf('\n[SCENARIO] Loading: %s\n', scenarioName);
fprintf('[SCENARIO] %s\n', scenDef.description);

%% 3. Apply overrides (track which environment keys the catalog sets)
overriddenEnvKeys = {};
if isfield(scenDef, 'overrides') && isstruct(scenDef.overrides)
    ov = scenDef.overrides;
    fields = fieldnames(ov);
    for i = 1:numel(fields)
        key = fields{i};
        val = ov.(key);
        config = setNestedField(config, key, val);
        % Track environment keys so auto-resolve doesn't clobber them
        if startsWith(key, 'environment.')
            overriddenEnvKeys{end+1} = key; %#ok<AGROW>
        end
    end
    fprintf('[SCENARIO] Applied %d override(s)\n', numel(fields));
end

config = selectActiveParams(config);

%% 3.5 Auto-resolve environment flags from terrain_type
%  When auto_terrain_effects is ON (default), terrain_type drives:
%    water   → occlusion OFF, masking OFF, clutter OFF, propagation OFF
%    rural   → occlusion ON,  masking ON,  clutter ON  (density 0.3)
%    urban   → occlusion ON,  masking ON,  clutter ON  (density 0.6)
%    mountain→ occlusion ON,  masking ON,  clutter ON  (density 0.5)
%    desert  → occlusion ON,  masking ON,  clutter ON  (density 0.2)
%
%  Catalog overrides always win — if a scenario explicitly sets e.g.
%  "environment.terrain_occlusion": false, auto-resolve won't touch it.
config = resolveTerrainEnvironment(config, overriddenEnvKeys);

%% 4. Load sensors
if sensorOverride ~= ""
    sensorConfigName = sensorOverride;
    fprintf('[SCENARIO] Sensor config: %s (OVERRIDE)\n', sensorConfigName);
else
    sensorConfigName = "sensors";
    if isfield(scenDef, 'sensor_config') && ~isempty(scenDef.sensor_config)
        sensorConfigName = string(scenDef.sensor_config);
    end
    fprintf('[SCENARIO] Sensor config: %s\n', sensorConfigName);
end
config.sensor_config_name = sensorConfigName;
[sensors, metas] = trackbench.sensors.loadSensors(sensorConfigName);

%% 5. Build trackingScenario
scenario = trackingScenario;

platformNames = fieldnames(sensors);
primaryUpdateRate = [];

platformDefs = struct();
if isfield(scenDef, 'platforms') && isstruct(scenDef.platforms)
    platformDefs = scenDef.platforms;
end

for p = 1:numel(platformNames)
    pName = platformNames{p};
    sensorList = sensors.(pName);

    plat = platform(scenario, 'Sensors', sensorList);

    if isfield(platformDefs, pName)
        pDef = platformDefs.(pName);
        pStart = [0, 0, 0];
        pHeading = 90;
        pSpeed = 250;
        if isfield(pDef, 'start_pos');   pStart = reshape(pDef.start_pos, 1, []); end
        if isfield(pDef, 'heading_deg'); pHeading = pDef.heading_deg; end
        if isfield(pDef, 'speed_kmh');   pSpeed = pDef.speed_kmh * 1000 / 3600; end

        dx = pSpeed * sind(pHeading);
        dy = pSpeed * cosd(pHeading);
        T = config.scenario.duration_s;
        pEnd = pStart + [dx, dy, 0] * T;

        plat.Trajectory = waypointTrajectory( ...
            'Waypoints', [pStart; pEnd], ...
            'TimeOfArrival', [0; T], ...
            'Velocities', [dx dy 0; dx dy 0], ...
            'AutoBank', true, ...
            'AutoPitch', true);
        fprintf('[SCENARIO] Platform "%s": %d sensor(s) — MOVING [%.0f,%.0f,%.0f] → hdg %.0f° @ %.0f km/h\n', ...
            pName, numel(sensorList), pStart(1), pStart(2), pStart(3), pHeading, pSpeed*3.6);
    else
        fprintf('[SCENARIO] Platform "%s": %d sensor(s)\n', pName, numel(sensorList));
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

%% 6. Build targets
rawText = fileread(catPath);
rawCatalog = jsondecode(rawText);
rawScenDef = rawCatalog.scenarios.(scenarioName);

if isstruct(rawScenDef.targets)
    targets = num2cell(rawScenDef.targets);
else
    targets = rawScenDef.targets;
end
numTargets = numel(targets);
duration = config.scenario.duration_s;

fprintf('[SCENARIO] Building %d target(s) over %.0fs\n', numTargets, duration);

for i = 1:numTargets
    if iscell(targets)
        tDef = targets{i};
    else
        tDef = targets(i);
    end
    addTargetFromDef(scenario, tDef, duration, i);
end

%% 7. Attach terrain for occlusion modelling
terrainType = 'rural';
if isfield(config, 'environment') && isfield(config.environment, 'terrain_type')
    terrainType = config.environment.terrain_type;
end
elevScale = 1.0;
if isfield(config, 'environment') && isfield(config.environment, 'terrain_scale')
    elevScale = config.environment.terrain_scale;
end

try
    scenBounds = computeScenarioBounds(scenario, config.scenario.duration_s);
    [Zterrain, boundary, Xg, Yg] = trackbench.environment.generateTerrain( ...
        terrainType, scenBounds, elevScale);
    groundSurface(scenario, 'Terrain', Zterrain, 'Boundary', boundary);
    
    % ---- Raise stationary sensor platforms to terrain surface ----
    % Platforms at z=0 are buried underground when terrain is above sea level.
    % Fix: set platform Z = terrain Z so the sensor antenna (MountingLocation)
    % sits above the terrain surface, giving correct LOS for occlusion().
    allPlats = scenario.Platforms;
    for pp = 1:numel(allPlats)
        platObj = allPlats{pp};
        if ~isa(platObj.Trajectory, 'waypointTrajectory')
            % Stationary platform: use Trajectory.Position
            pPos = platObj.Trajectory.Position(:)';
            terrZ = interp2(Xg, Yg, Zterrain, pPos(1), pPos(2), 'linear', 0);
            if terrZ < -1  % terrain above sea level (NED: negative = up)
                oldZ = pPos(3);
                newZ = terrZ;
                platObj.Trajectory.Position = [pPos(1), pPos(2), newZ];
                fprintf('[SCENARIO] Platform %d raised from z=%.0f to z=%.0f (terrain=%.0fm ASL)\n', ...
                    pp, oldZ, newZ, -terrZ);
            end
        end
    end
    
    tg = struct('Z', Zterrain, 'boundary', boundary, 'X', Xg, 'Y', Yg);
    config.terrainGrid = tg;
    config.environment.terrainGrid = tg;
    fprintf('[SCENARIO] Terrain attached: %s (UseOcclusion=%d)\n', ...
        terrainType, scenario.SurfaceManager.UseOcclusion);
catch ME
    warning('loadScenario:terrainFailed', ...
        'Terrain generation failed: %s. Falling back to horizon model.', ME.message);
    config.terrainGrid = [];
end

%% 8. Pre-flight validation
[scanOk, scanInfo] = trackbench.scenario.validateScanCoverage(scenario, duration);
if ~scanOk
    warning('loadScenario:insufficientScans', ...
        '%s\nIncrease scenario.duration_s in the scenario overrides.', scanInfo.message);
end

% Comprehensive scenario validation (sensors, targets, terrain, params)
try
    trackbench.validation.validateScenarioConfig(config, scenario, sensors, metas);
catch ME
    warning('loadScenario:validationError', ...
        'Validator encountered an error: %s', ME.message);
end

%% Summary
totalSensors = 0;
for p = 1:numel(platformNames)
    totalSensors = totalSensors + numel(sensors.(platformNames{p}));
end

fprintf('[SCENARIO] Ready: %d platform(s), %d sensor(s), %d target(s), %.0fs duration\n', ...
    numel(platformNames), totalSensors, numTargets, duration);
fprintf('[SCENARIO] Environment: terrain=%s | occlusion=%s | masking=%s | clutter=%s\n', ...
    config.environment.terrain_type, ...
    ternary(config.environment.terrain_occlusion, 'ON', 'OFF'), ...
    ternary(config.environment.horizon_masking, 'ON', 'OFF'), ...
    ternary(config.environment.ground_clutter, 'ON', 'OFF'));
fprintf('[SCENARIO] Degradation: %s | UpdateRate: %.1f Hz\n', ...
    ternary(config.degradation.enabled, upper(config.degradation.type), 'OFF'), ...
    scenario.UpdateRate);
end


%% ========================================================================
%                    AUTO-RESOLVE TERRAIN ENVIRONMENT
%% ========================================================================
function config = resolveTerrainEnvironment(config, overriddenEnvKeys)
%resolveTerrainEnvironment  Auto-set environment flags from terrain_type.
%
%  When config.environment.auto_terrain_effects is true, this function
%  sets sensible defaults for terrain_occlusion, horizon_masking,
%  ground_clutter, propagation_model, and clutter_density based on the
%  terrain_type. Catalog overrides are never clobbered.

    if ~isfield(config, 'environment'); return; end
    env = config.environment;
    
    autoMode = true;
    if isfield(env, 'auto_terrain_effects')
        autoMode = logical(env.auto_terrain_effects);
    end
    if ~autoMode
        fprintf('[SCENARIO] auto_terrain_effects=OFF — environment flags unchanged\n');
        return;
    end
    
    terrainType = 'water';
    if isfield(env, 'terrain_type')
        terrainType = lower(string(env.terrain_type));
    end
    
    % Define terrain presets
    switch terrainType
        case "water"
            preset.terrain_occlusion = false;
            preset.horizon_masking   = false;
            preset.ground_clutter    = false;
            preset.propagation_model = false;
            preset.clutter_density   = 0;
        case "rural"
            preset.terrain_occlusion = true;
            preset.horizon_masking   = true;
            preset.ground_clutter    = true;
            preset.propagation_model = false;
            preset.clutter_density   = 0.3;
        case "urban"
            preset.terrain_occlusion = true;
            preset.horizon_masking   = true;
            preset.ground_clutter    = true;
            preset.propagation_model = false;
            preset.clutter_density   = 0.6;
        case "mountain"
            preset.terrain_occlusion = true;
            preset.horizon_masking   = true;
            preset.ground_clutter    = true;
            preset.propagation_model = false;
            preset.clutter_density   = 0.5;
        case "desert"
            preset.terrain_occlusion = true;
            preset.horizon_masking   = true;
            preset.ground_clutter    = true;
            preset.propagation_model = false;
            preset.clutter_density   = 0.2;
        otherwise
            fprintf('[SCENARIO] Unknown terrain_type "%s" — no auto-resolve\n', terrainType);
            return;
    end
    
    % Apply preset ONLY where the catalog didn't explicitly override
    presetFields = fieldnames(preset);
    applied = {};
    for i = 1:numel(presetFields)
        fld = presetFields{i};
        catalogKey = "environment." + fld;
        if ~any(strcmp(overriddenEnvKeys, catalogKey))
            config.environment.(fld) = preset.(fld);
            applied{end+1} = fld; %#ok<AGROW>
        end
    end
    
    if ~isempty(applied)
        fprintf('[SCENARIO] Auto-terrain (%s): set %s\n', ...
            terrainType, strjoin(string(applied), ', '));
    else
        fprintf('[SCENARIO] Auto-terrain (%s): all env flags set by catalog — no auto changes\n', ...
            terrainType);
    end
end


%% ========================================================================
%                         TARGET BUILDERS
%% ========================================================================
function addTargetFromDef(scenario, tDef, duration, idx)
    behavior = "constant_velocity";
    if isfield(tDef, 'behavior'); behavior = lower(string(tDef.behavior)); end
    speed_ms = 250;
    if isfield(tDef, 'speed_kmh'); speed_ms = tDef.speed_kmh * 1000 / 3600; end
    startPos = [-2000, -20000, -3000];
    if isfield(tDef, 'start_pos'); startPos = reshape(tDef.start_pos, 1, []); end
    altitude = abs(startPos(3));
    if isfield(tDef, 'altitude_m'); altitude = tDef.altitude_m; end
    startPos(3) = -abs(altitude);
    heading = 90;
    if isfield(tDef, 'heading_deg'); heading = tDef.heading_deg; end
    T = duration;

    switch behavior
        case "constant_velocity"
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);
        case "gentle_turn"
            [wp, t, vel] = buildGentleTurn(startPos, speed_ms, T, idx);
        case "s_maneuver"
            turnRate = 2;
            if isfield(tDef, 'turn_rate_dps'); turnRate = tDef.turn_rate_dps; end
            [wp, t, vel] = buildSManeuver(startPos, heading, speed_ms, turnRate, T);
        case "crossing"
            endPos = [5000, -20000, -3000];
            if isfield(tDef, 'end_pos'); endPos = reshape(tDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
        case "orbit"
            radius = 3000;
            if isfield(tDef, 'orbit_radius_m'); radius = tDef.orbit_radius_m; end
            [wp, t, vel] = buildOrbit(startPos, radius, speed_ms, T);
        case "approach"
            endPos = [0, -1000, -100];
            if isfield(tDef, 'end_pos'); endPos = reshape(tDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildApproach(startPos, endPos, T);
        case "departure"
            endPos = startPos + [0, -50000, -5000];
            if isfield(tDef, 'end_pos'); endPos = reshape(tDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
        case {"parallel", "head_on"}
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);
        otherwise
            endPos = startPos + 3000*(2*rand(1,3)-1);
            endPos(3) = startPos(3);
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
    end

    tgt = platform(scenario);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, 'TimeOfArrival', t, 'Velocities', vel);
    fprintf('  Target %d: %s | %.0f km/h | start=[%.0f,%.0f,%.0f]\n', ...
        idx, behavior, speed_ms*3.6, startPos(1), startPos(2), startPos(3));
end

function [wp, t, vel] = buildConstantVelocity(startPos, heading, speed, T)
    dx = speed * cosd(heading); dy = speed * sind(heading);
    endPos = startPos + [dx dy 0] * T;
    wp = [startPos; endPos]; t = [0; T];
    v = (endPos - startPos) / T; vel = [v; v];
end

function [wp, t, vel] = buildGentleTurn(startPos, speed, T, idx)
    nPts = 5; tNorm = linspace(0, 1, nPts); t = tNorm(:) * T;
    direction = (-1)^idx; wp = zeros(nPts, 3); wp(1,:) = startPos;
    for k = 2:nPts
        frac = tNorm(k); dt = t(k) - t(k-1);
        wp(k,:) = wp(k-1,:) + [speed*dt*0.8, direction*speed*dt*0.2*sin(pi*frac), -50*frac];
    end
    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildSManeuver(startPos, heading, speed, turnRate, T)
    nPts = 9; dt = T / (nPts - 1); t = (0:dt:T)'; t = t(1:nPts);
    wp = zeros(nPts, 3); wp(1,:) = startPos; hdg = heading;
    for k = 2:nPts
        frac = (k-1) / (nPts-1);
        if frac < 0.25; dhdg = 0;
        elseif frac < 0.5; dhdg = turnRate;
        elseif frac < 0.75; dhdg = -turnRate;
        else; dhdg = 0; end
        hdg = hdg + dhdg * dt;
        wp(k,:) = wp(k-1,:) + [speed*cosd(hdg)*dt, speed*sind(hdg)*dt, 0];
    end
    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildCrossing(startPos, endPos, T)
    wp = [startPos; endPos]; t = [0; T];
    v = (endPos - startPos) / T; vel = [v; v];
end

function [wp, t, vel] = buildOrbit(center, radius, speed, T)
    circumference = 2 * pi * radius; period = circumference / speed;
    nLaps = max(1, floor(T / period)); nPts = nLaps * 12 + 1;
    t = linspace(0, T, nPts)'; wp = zeros(nPts, 3);
    for k = 1:nPts
        theta = 2*pi * t(k) / period;
        wp(k,:) = center + [radius*cos(theta), radius*sin(theta), 0];
    end
    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildApproach(startPos, endPos, T)
    nPts = 5; t = linspace(0, T, nPts)'; wp = zeros(nPts, 3);
    for k = 1:nPts
        frac = (k-1) / (nPts-1); wp(k,:) = startPos + frac * (endPos - startPos);
    end
    vel = computeVelocities(wp, t);
end

function vel = computeVelocities(wp, t)
    nPts = size(wp, 1); vel = zeros(nPts, 3);
    for k = 1:nPts-1
        dt = t(k+1) - t(k);
        if dt > 0; vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt; end
    end
    vel(end,:) = vel(max(1, end-1),:);
end


%% ========================================================================
%                         HELPER FUNCTIONS
%% ========================================================================
function s = setNestedField(s, rawKey, val)
    rawKey = char(rawKey);
    if contains(rawKey, '.')
        parts = strsplit(rawKey, '.');
    else
        parts = greedyResolve(s, rawKey);
    end
    switch numel(parts)
        case 1; s.(parts{1}) = val;
        case 2
            if ~isfield(s, parts{1}); s.(parts{1}) = struct(); end
            s.(parts{1}).(parts{2}) = val;
        case 3
            if ~isfield(s, parts{1}); s.(parts{1}) = struct(); end
            if ~isfield(s.(parts{1}), parts{2}); s.(parts{1}).(parts{2}) = struct(); end
            s.(parts{1}).(parts{2}).(parts{3}) = val;
        case 4
            if ~isfield(s, parts{1}); s.(parts{1}) = struct(); end
            if ~isfield(s.(parts{1}), parts{2}); s.(parts{1}).(parts{2}) = struct(); end
            if ~isfield(s.(parts{1}).(parts{2}), parts{3}); s.(parts{1}).(parts{2}).(parts{3}) = struct(); end
            s.(parts{1}).(parts{2}).(parts{3}).(parts{4}) = val;
        otherwise; warning('setNestedField: path too deep (max 4): %s', rawKey);
    end
end

function parts = greedyResolve(s, key)
    parts = {}; tokens = strsplit(key, '_'); current = s; i = 1;
    while i <= numel(tokens)
        matched = false;
        for j = numel(tokens):-1:i
            candidate = strjoin(tokens(i:j), '_');
            if isstruct(current) && isfield(current, candidate)
                parts{end+1} = candidate; %#ok<AGROW>
                if isstruct(current.(candidate)); current = current.(candidate); end
                i = j + 1; matched = true; break;
            end
        end
        if ~matched
            parts{end+1} = strjoin(tokens(i:end), '_'); break; %#ok<AGROW>
        end
    end
end

function config = selectActiveParams(config)
    if config.degradation.enabled
        activeParams = config.tracker_params.degraded;
        activePd = config.tracker_global.detection_probability.degraded;
        fprintf('[SCENARIO] Using DEGRADED tracker parameters\n');
    else
        activeParams = config.tracker_params.ideal;
        activePd = config.tracker_global.detection_probability.ideal;
        fprintf('[SCENARIO] Using IDEAL tracker parameters\n');
    end
    config.active_params = activeParams;
    config.active_params.pd = activePd;
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function bounds = computeScenarioBounds(scenario, ~)
    allPos = [0 0 0];
    plats = scenario.Platforms;
    for p = 1:numel(plats)
        try
            pos0 = plats{p}.Trajectory.Position(:)';
            allPos = [allPos; pos0]; %#ok<AGROW>
        catch
        end
        try
            traj = plats{p}.Trajectory;
            if isprop(traj, 'Waypoints')
                allPos = [allPos; traj.Waypoints]; %#ok<AGROW>
            end
        catch
        end
    end
    maxExtent = max(vecnorm(allPos(:,1:2), 2, 2));
    halfSpan = max(maxExtent * 1.15, 130000);
    bounds = [-halfSpan, halfSpan; -halfSpan, halfSpan];
end