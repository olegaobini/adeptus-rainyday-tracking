function [scenario, config, sensors, metas] = loadScenario(scenarioName, catalogName, sensorOverride)
%loadScenario  Load a named scenario from the catalog and build everything.
%
% Reads the scenario definition from the catalog, loads the base config,
% applies overrides, loads the sensor catalog, builds a trackingScenario
% with platforms and targets.
%
% USAGE
%   [scenario, config, sensors, metas] = loadScenario("dasr_ideal");
%   [scenario, config, sensors, metas] = loadScenario("crossing_targets");
%   [scenario, config, sensors, metas] = loadScenario("storm_window");
%
%   % Force a specific sensor config (overrides what the catalog says):
%   [scenario, config, sensors, metas] = loadScenario("dasr_ideal", "scenario_catalog", "sensors");
%
% INPUTS
%   scenarioName   : string — key from scenario_catalog.json
%   catalogName    : (optional) catalog filename. Default "scenario_catalog"
%   sensorOverride : (optional) sensor config name to force instead of the
%                    catalog's sensor_config. e.g. "sensors" to use sensors.json.
%                    Default "" (empty = use catalog's sensor_config).
%
% OUTPUTS
%   scenario : trackingScenario object ready for runDetections()
%   config   : merged config struct (base + scenario overrides)
%   sensors  : struct from loadSensors, grouped by platform
%   metas    : sensor metadata struct from loadSensors
%
% WORKFLOW
%   1. Load base config (default.json)
%   2. Load scenario catalog, find named scenario
%   3. Apply scenario overrides to config
%   4. Load sensors from sensor catalog referenced by scenario
%   5. Build trackingScenario with platforms + sensors
%   6. Build targets from scenario target definitions
%
% See also: loadScenarioCatalog, loadSensors, load_config, createScenario3D

arguments
    scenarioName   (1,1) string
    catalogName    (1,1) string = "scenario_catalog"
    sensorOverride (1,1) string = ""
end

%% 1. Load base config
config = trackbench.loader.loadConfig("default");

%% 2. Load scenario catalog
if ~endsWith(catalogName, ".json")
    catalogName = catalogName + ".json";
end
catPath = trackbench.util.pathFromRoot("config", "scenarios", catalogName);
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

%% 3. Apply overrides
%  jsondecode stores dot-notation keys as literal field names.
%  e.g. "scenario.duration_s": 50 becomes ov.("scenario.duration_s") = 50
%  We need to treat these as nested paths.
if isfield(scenDef, 'overrides') && isstruct(scenDef.overrides)
    ov = scenDef.overrides;
    fields = fieldnames(ov);
    for i = 1:numel(fields)
        key = fields{i};
        val = ov.(key);
        config = setNestedField(config, key, val);
    end
    fprintf('[SCENARIO] Applied %d override(s)\n', numel(fields));
end

% Re-validate and select active params AFTER overrides
config = selectActiveParams(config);

%% 4. Load sensors
%  Priority: sensorOverride arg > catalog sensor_config > default "sensors"
if sensorOverride ~= ""
    sensorConfigName = sensorOverride;
    fprintf('[SCENARIO] Sensor config: %s (OVERRIDE — using your sensors.json)\n', sensorConfigName);
else
    sensorConfigName = "sensors";
    if isfield(scenDef, 'sensor_config') && ~isempty(scenDef.sensor_config)
        sensorConfigName = string(scenDef.sensor_config);
    end
    fprintf('[SCENARIO] Sensor config: %s\n', sensorConfigName);
end
[sensors, metas] = trackbench.sensors.loadSensors(sensorConfigName);

%% 5. Build trackingScenario
scenario = trackingScenario;

% Attach sensors to platforms
platformNames = fieldnames(sensors);
primaryUpdateRate = [];

% Check for platform definitions in the scenario (position/trajectory for
% non-stationary platforms like aircraft)
platformDefs = struct();
if isfield(scenDef, 'platforms') && isstruct(scenDef.platforms)
    platformDefs = scenDef.platforms;
end

for p = 1:numel(platformNames)
    pName = platformNames{p};
    sensorList = sensors.(pName);

    plat = platform(scenario, 'Sensors', sensorList);

    % If the scenario defines a trajectory for this platform, apply it
    if isfield(platformDefs, pName)
        pDef = platformDefs.(pName);
        pStart = [0, 0, 0];
        pHeading = 90;
        pSpeed = 250;  % m/s
        if isfield(pDef, 'start_pos');   pStart = reshape(pDef.start_pos, 1, []); end
        if isfield(pDef, 'heading_deg'); pHeading = pDef.heading_deg; end
        if isfield(pDef, 'speed_kmh');   pSpeed = pDef.speed_kmh * 1000 / 3600; end

        % NED: heading 0=North(+Y), 90=East(+X), 270=South(-Y)
        % MATLAB NED convention: X=North, Y=East, Z=Down
        % heading_deg measured from North (Y-axis) clockwise
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

    % Store first sensor's update rate as scenario rate
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
% jsondecode converts arrays of objects into struct arrays, but ONLY keeps
% fields common to ALL objects. To preserve per-target fields, we re-read
% the raw JSON and extract targets as a cell array of structs.
rawText = fileread(catPath);
rawCatalog = jsondecode(rawText);
rawScenDef = rawCatalog.scenarios.(scenarioName);

% Convert struct array to cell array to preserve per-element fields
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
%  Generates a procedural heightmap based on the environment terrain_type
%  and attaches it to the scenario via groundSurface(). The SurfaceManager
%  then provides occlusion() queries in runDetections, replacing the simpler
%  4/3 Earth horizon model with true line-of-sight checks against terrain.
terrainType = 'rural';
if isfield(config, 'environment') && isfield(config.environment, 'terrain_type')
    terrainType = config.environment.terrain_type;
end
elevScale = 1.0;
if isfield(config, 'environment') && isfield(config.environment, 'terrain_scale')
    elevScale = config.environment.terrain_scale;
end

try
    % Compute scenario bounds from target trajectories + margin
    scenBounds = computeScenarioBounds(scenario, config.scenario.duration_s);
    [Zterrain, boundary, Xg, Yg] = trackbench.environment.generateTerrain( ...
        terrainType, scenBounds, elevScale);
    groundSurface(scenario, 'Terrain', Zterrain, 'Boundary', boundary);
    tg = struct('Z', Zterrain, 'boundary', boundary, 'X', Xg, 'Y', Yg);
    config.terrainGrid = tg;
    config.environment.terrainGrid = tg;  % also in environment for runDetections pass-through
    fprintf('[SCENARIO] Terrain attached: %s (UseOcclusion=%d)\n', ...
        terrainType, scenario.SurfaceManager.UseOcclusion);
catch ME
    warning('loadScenario:terrainFailed', ...
        'Terrain generation failed: %s. Falling back to horizon model.', ME.message);
    config.terrainGrid = [];
end

%% Pre-flight: validate scan coverage
[scanOk, scanInfo] = trackbench.scenario.validateScanCoverage(scenario, duration);
if ~scanOk
    warning('loadScenario:insufficientScans', ...
        '%s\nIncrease scenario.duration_s in the scenario overrides.', scanInfo.message);
end

%% Summary
totalSensors = 0;
for p = 1:numel(platformNames)
    totalSensors = totalSensors + numel(sensors.(platformNames{p}));
end

fprintf('[SCENARIO] Ready: %d platform(s), %d sensor(s), %d target(s), %.0fs duration\n', ...
    numel(platformNames), totalSensors, numTargets, duration);
fprintf('[SCENARIO] Degradation: %s | UpdateRate: %.1f Hz\n', ...
    ternary(config.degradation.enabled, upper(config.degradation.type), 'OFF'), ...
    scenario.UpdateRate);
end


%% ========================================================================
%                         TARGET BUILDERS
%% ========================================================================
function addTargetFromDef(scenario, tDef, duration, idx)
%addTargetFromDef  Build a target platform from a scenario definition struct.

    behavior = "constant_velocity";
    if isfield(tDef, 'behavior'); behavior = lower(string(tDef.behavior)); end

    speed_ms = 250;  % default ~900 km/h
    if isfield(tDef, 'speed_kmh'); speed_ms = tDef.speed_kmh * 1000 / 3600; end

    startPos = [-2000, -20000, -3000];
    if isfield(tDef, 'start_pos'); startPos = reshape(tDef.start_pos, 1, []); end

    altitude = abs(startPos(3));
    if isfield(tDef, 'altitude_m'); altitude = tDef.altitude_m; end
    startPos(3) = -abs(altitude);  % NED convention: altitude is negative Z

    heading = 90;  % default east
    if isfield(tDef, 'heading_deg'); heading = tDef.heading_deg; end

    T = duration;

    switch behavior
        case "constant_velocity"
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);

        case "gentle_turn"
            [wp, t, vel] = buildGentleTurn(startPos, speed_ms, T, idx);

        case "s_maneuver"
            turnRate = 2;  % deg/s default
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
            [wp, t, vel] = buildCrossing(startPos, endPos, T);  % reuse crossing logic

        case "parallel"
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);

        case "head_on"
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);

        otherwise
            % Random fallback
            endPos = startPos + 3000*(2*rand(1,3)-1);
            endPos(3) = startPos(3);
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
    end

    tgt = platform(scenario);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, ...
        'TimeOfArrival', t, ...
        'Velocities', vel);

    fprintf('  Target %d: %s | %.0f km/h | start=[%.0f,%.0f,%.0f]\n', ...
        idx, behavior, speed_ms*3.6, startPos(1), startPos(2), startPos(3));
end


%% ---- Trajectory generators ----

function [wp, t, vel] = buildConstantVelocity(startPos, heading, speed, T)
    dx = speed * cosd(heading);
    dy = speed * sind(heading);
    endPos = startPos + [dx dy 0] * T;
    wp  = [startPos; endPos];
    t   = [0; T];
    v   = (endPos - startPos) / T;
    vel = [v; v];
end

function [wp, t, vel] = buildGentleTurn(startPos, speed, T, idx)
    % Gentle S-curve with mild heading changes (replicates original createScenario3D targets)
    nPts = 5;
    tNorm = linspace(0, 1, nPts);
    t = tNorm(:) * T;

    % Generate waypoints with gentle lateral drift
    direction = (-1)^idx;  % alternate left/right
    wp = zeros(nPts, 3);
    wp(1,:) = startPos;
    for k = 2:nPts
        frac = tNorm(k);
        dt = t(k) - t(k-1);
        wp(k,:) = wp(k-1,:) + [speed*dt*0.8, ...
                                 direction*speed*dt*0.2*sin(pi*frac), ...
                                 -50*frac];  % gentle descent
    end

    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildSManeuver(startPos, heading, speed, turnRate, T)
    % S-shaped maneuver: straight -> turn left -> turn right -> straight
    nPts = 9;
    dt = T / (nPts - 1);
    t = (0:dt:T)';
    t = t(1:nPts);

    wp = zeros(nPts, 3);
    wp(1,:) = startPos;
    hdg = heading;

    for k = 2:nPts
        frac = (k-1) / (nPts-1);
        if frac < 0.25
            dhdg = 0;           % straight
        elseif frac < 0.5
            dhdg = turnRate;    % turn left
        elseif frac < 0.75
            dhdg = -turnRate;   % turn right
        else
            dhdg = 0;           % straight
        end
        hdg = hdg + dhdg * dt;
        dx = speed * cosd(hdg) * dt;
        dy = speed * sind(hdg) * dt;
        wp(k,:) = wp(k-1,:) + [dx, dy, 0];
    end

    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildCrossing(startPos, endPos, T)
    wp  = [startPos; endPos];
    t   = [0; T];
    v   = (endPos - startPos) / T;
    vel = [v; v];
end

function [wp, t, vel] = buildOrbit(center, radius, speed, T)
    % Circular orbit (holding pattern)
    circumference = 2 * pi * radius;
    period = circumference / speed;
    nLaps = max(1, floor(T / period));
    nPts = nLaps * 12 + 1;  % 12 points per lap + closure
    t = linspace(0, T, nPts)';

    wp = zeros(nPts, 3);
    for k = 1:nPts
        theta = 2*pi * t(k) / period;
        wp(k,:) = center + [radius*cos(theta), radius*sin(theta), 0];
    end

    vel = computeVelocities(wp, t);
end

function [wp, t, vel] = buildApproach(startPos, endPos, T)
    % Descending approach — linear interpolation with altitude change
    nPts = 5;
    t = linspace(0, T, nPts)';
    wp = zeros(nPts, 3);
    for k = 1:nPts
        frac = (k-1) / (nPts-1);
        wp(k,:) = startPos + frac * (endPos - startPos);
    end
    vel = computeVelocities(wp, t);
end

function vel = computeVelocities(wp, t)
    % Compute velocity at each waypoint from finite differences
    nPts = size(wp, 1);
    vel = zeros(nPts, 3);
    for k = 1:nPts-1
        dt = t(k+1) - t(k);
        if dt > 0
            vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt;
        end
    end
    vel(end,:) = vel(max(1, end-1),:);
end


%% ========================================================================
%                         HELPER FUNCTIONS
%% ========================================================================
function s = setNestedField(s, rawKey, val)
%setNestedField  Set a nested struct field via dot-notation or underscore key.
%
%  jsondecode converts JSON dot-keys to underscores:
%    "scenario.duration_s" -> "scenario_duration_s"
%    "tracker_params.degraded.gate" -> "tracker_params_degraded_gate"
%
%  We resolve the path by greedily matching existing struct fields,
%  walking from the left. At each level, try the longest prefix that
%  matches an existing field, then recurse into the remainder.

    rawKey = char(rawKey);
    
    % If it contains dots, split on dots (original format)
    if contains(rawKey, '.')
        parts = strsplit(rawKey, '.');
    else
        % jsondecode converted dots to underscores.
        % Use greedy matching against existing struct fields.
        parts = greedyResolve(s, rawKey);
    end
    
    % Apply the resolved path
    switch numel(parts)
        case 1
            s.(parts{1}) = val;
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
        otherwise
            warning('setNestedField: path too deep (max 4): %s', rawKey);
    end
end

function parts = greedyResolve(s, key)
%greedyResolve  Split underscore key into nested path by matching struct fields.
%  Given s with fields {scenario, tracker_params, ...} and key 'scenario_duration_s',
%  finds that 'scenario' is a field of s, then 'duration_s' is a field of s.scenario.
%  Returns {'scenario', 'duration_s'}.
%
%  For 'tracker_params_degraded_gate', finds 'tracker_params' -> 'degraded' -> 'gate'.

    parts = {};
    tokens = strsplit(key, '_');
    current = s;
    i = 1;
    
    while i <= numel(tokens)
        matched = false;
        % Try longest prefix first (greedy)
        for j = numel(tokens):-1:i
            candidate = strjoin(tokens(i:j), '_');
            if isstruct(current) && isfield(current, candidate)
                parts{end+1} = candidate; %#ok<AGROW>
                if isstruct(current.(candidate))
                    current = current.(candidate);
                end
                i = j + 1;
                matched = true;
                break;
            end
        end
        if ~matched
            % No match found — treat the rest as a single field name
            remainder = strjoin(tokens(i:end), '_');
            parts{end+1} = remainder; %#ok<AGROW>
            break;
        end
    end
end

function config = selectActiveParams(config)
%selectActiveParams  Pick ideal or degraded params based on degradation flag.
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

function bounds = computeScenarioBounds(scenario, duration)
%computeScenarioBounds  Estimate XY extents from all platform trajectories.
%  Returns [xMin xMax; yMin yMax] with 20% margin. Used to size the terrain
%  grid so it covers the full scenario footprint.

    allPos = [0 0 0];  % always include origin (sensor platform default)
    
    % Sample each platform's trajectory at start and end
    plats = scenario.Platforms;
    for p = 1:numel(plats)
        try
            % Try start position
            pos0 = plats{p}.InitialPosition(:)';
            allPos = [allPos; pos0]; %#ok<AGROW>
        catch
        end
        try
            % Try to get position at end of scenario
            traj = plats{p}.Trajectory;
            if isprop(traj, 'Waypoints')
                allPos = [allPos; traj.Waypoints]; %#ok<AGROW>
            end
        catch
        end
    end
    
    xMin = min(allPos(:,1));
    xMax = max(allPos(:,1));
    yMin = min(allPos(:,2));
    yMax = max(allPos(:,2));
    
    % Ensure minimum span (at least 60km in each direction from center)
    cx = (xMin + xMax) / 2;
    cy = (yMin + yMax) / 2;
    halfSpan = max([abs(xMax-xMin)/2, abs(yMax-yMin)/2, 60000]);
    
    % Add 20% margin
    halfSpan = halfSpan * 1.2;
    
    bounds = [cx-halfSpan, cx+halfSpan; cy-halfSpan, cy+halfSpan];
end
