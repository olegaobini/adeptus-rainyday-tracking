function report = validateScenarioConfig(config, scenario, sensors, metas)
%validateScenarioConfig  Pre-flight check for simulation-breaking parameters.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Checks the built scenario for red flags before running detections.
%  Issues are categorized as CRITICAL (will break sim), WARNING (may cause
%  bad results), or INFO (worth knowing). Each issue includes a suggested fix.
%
%  CHECKS PERFORMED
%    1. Sensor platforms vs terrain — buried underground?
%    2. Targets vs terrain — starting below terrain surface?
%    3. Target altitude vs sensor range — targets out of range?
%    4. Target speed sanity — physically possible?
%    5. Terrain type valid
%    6. Degradation coherence — type set but not enabled?
%    7. Tracker parameter sanity — gate, FAR, thresholds
%    8. Airborne platform altitude — floating at z=0?
%    9. Duration vs target travel — targets leave sensor coverage?
%   10. Environment flag coherence — occlusion ON but no terrain?
%
%  USAGE
%    report = trackbench.validation.validateScenarioConfig(config, scenario, sensors, metas);
%
%  INPUTS
%    config   : merged config struct (from loadScenario)
%    scenario : trackingScenario object with platforms/targets attached
%    sensors  : struct from loadSensors (grouped by platform)
%    metas    : sensor metadata struct from loadSensors
%
%  OUTPUT
%    report : struct with fields:
%      .issues    — cell array of issue structs (severity, check, message, fix)
%      .critical  — number of CRITICAL issues
%      .warnings  — number of WARNING issues
%      .infos     — number of INFO issues
%      .passed    — true if no CRITICAL issues
%
%  See also: loadScenario, validateScanCoverage

arguments
    config   (1,1) struct
    scenario
    sensors  (1,1) struct = struct()
    metas    (1,1) struct = struct()
end

issues = {};

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              PRE-FLIGHT SCENARIO VALIDATION             ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n');

%% ---- 1. Sensor platforms vs terrain ----
%  Scope: only sensor-hosting platforms (the first N platforms, where N is
%  the number of sensor platform groups). Targets are platforms (N+1):end
%  and are validated separately in Check 2 — checking them here would
%  produce duplicate "Platform X Underground" + "Target Y Below Terrain"
%  warnings for the same object.
if isfield(config, 'terrainGrid') && ~isempty(config.terrainGrid)
    tg = config.terrainGrid;
    allPlats = scenario.Platforms;
    numSensorPlats = numel(fieldnames(sensors));

    for pp = 1:min(numSensorPlats, numel(allPlats))
        platObj = allPlats{pp};
        try
            if isa(platObj.Trajectory, 'waypointTrajectory')
                pPos = platObj.Trajectory.Waypoints(1,:);
            else
                pPos = platObj.Trajectory.Position(:)';
            end
        catch
            continue;
        end

        % v3.6.6 — Underground check uses COMPOSED sensor world position
        % (platform.Position + sensor.MountingLocation), not raw platform
        % Position. That's what fusionRadarSensor uses internally for
        % detection geometry, so it's also the correct value for the
        % "is the sensor buried?" check. The old code read Trajectory.Position
        % directly and fired a false-positive CRITICAL for every
        % editor-style scenario — those platforms sit at origin by
        % design (sensor world coords live in MountingLocation), and
        % terrain at origin is typically a valley floor, so the platform
        % was "buried" by construction even though the actual sensor
        % was on a mountain peak.
        if isempty(platObj.Sensors); continue; end
        for ss = 1:numel(platObj.Sensors)
            sObj = platObj.Sensors{ss};
            if ~isprop(sObj, 'MountingLocation'); continue; end
            sensorWorld = pPos + sObj.MountingLocation(:)';

            % Check if within terrain grid bounds
            if sensorWorld(1) >= tg.boundary(1,1) && sensorWorld(1) <= tg.boundary(1,2) && ...
               sensorWorld(2) >= tg.boundary(2,1) && sensorWorld(2) <= tg.boundary(2,2)
                terrZ = interp2(tg.X, tg.Y, tg.Z, sensorWorld(1), sensorWorld(2), 'linear', 0);

                if sensorWorld(3) > terrZ + 5  % sensor z MORE positive than terrain z (buried in NED)
                    issues{end+1} = makeIssue('CRITICAL', 'Sensor Underground', ...
                        sprintf(['Sensor on platform %d (composed world position ' ...
                            '[E=%.0f, N=%.0f, alt=%.0f]m) is %.0fm BELOW terrain ' ...
                            '(terrain=%.0fm ASL at that location).'], ...
                            pp, sensorWorld(1), sensorWorld(2), -sensorWorld(3), ...
                            sensorWorld(3)-terrZ, -terrZ), ...
                        'Either the editor-placed altitude is wrong, the terrain heightmap is higher than expected at that XY, or the platform auto-raise in loadRunFile needs to handle this case.'); %#ok<AGROW>
                    break;  % one issue per platform; avoid duplicate warnings on multi-sensor platforms
                end
            end
        end
    end
end

%% ---- 2. Targets vs terrain ----
%  Skip waypoints within a TAKEOFF/LANDING EXCLUSION ZONE around the
%  scenario origin. Procedural terrain (rng seed 42) doesn't know where
%  real airports are — a recorded NASA flight takes off from a real airport
%  (e.g. White Plains, 134m MSL) and climbs out over several minutes, but
%  the procedural heightmap at the origin coordinate may be arbitrarily
%  tall. Within the exclusion zone, terrain conflicts are expected and
%  not a real scenario bug. Beyond the zone, targets flying through
%  mountains mid-mission are still flagged.
ORIGIN_EXCLUSION_M = 10000;  % 10 km radius around origin
if isfield(config, 'terrainGrid') && ~isempty(config.terrainGrid)
    tg = config.terrainGrid;
    allPlats = scenario.Platforms;

    % Targets are platforms after sensor platforms
    numSensorPlats = numel(fieldnames(sensors));
    for pp = (numSensorPlats+1):numel(allPlats)
        tgtObj = allPlats{pp};
        tgtIdx = pp - numSensorPlats;
        try
            if isa(tgtObj.Trajectory, 'waypointTrajectory')
                waypoints = tgtObj.Trajectory.Waypoints;
            else
                waypoints = tgtObj.Trajectory.Position(:)';
            end
        catch
            continue;
        end

        for wp = 1:size(waypoints, 1)
            wPos = waypoints(wp,:);

            % Skip waypoints inside the origin exclusion zone (takeoff/landing)
            if norm(wPos(1:2)) < ORIGIN_EXCLUSION_M
                continue;
            end

            if wPos(1) >= tg.boundary(1,1) && wPos(1) <= tg.boundary(1,2) && ...
               wPos(2) >= tg.boundary(2,1) && wPos(2) <= tg.boundary(2,2)
                terrZ = interp2(tg.X, tg.Y, tg.Z, wPos(1), wPos(2), 'linear', 0);

                if wPos(3) > terrZ + 10  % target below terrain surface (NED)
                    altASL = -wPos(3);
                    terrASL = -terrZ;
                    rangeKm = norm(wPos(1:2)) / 1000;
                    issues{end+1} = makeIssue('CRITICAL', 'Target Below Terrain', ...
                        sprintf('Target %d waypoint %d at %.0fm ASL is BELOW terrain (%.0fm ASL) at [%.0f,%.0f] (%.1f km from origin).', ...
                            tgtIdx, wp, altASL, terrASL, wPos(1), wPos(2), rangeKm), ...
                        sprintf('Increase altitude_m to at least %.0f, or move the target away from high terrain.', terrASL + 500)); %#ok<AGROW>
                    break;  % one warning per target is enough
                end
            end
        end
    end
end

%% ---- 3. Targets vs sensor range ----
if ~isempty(metas) && isstruct(metas)
    maxSensorRange = 0;
    pNames = fieldnames(metas);
    for p = 1:numel(pNames)
        mList = metas.(pNames{p});
        for m = 1:numel(mList)
            if isfield(mList{m}, 'maxRange')
                maxSensorRange = max(maxSensorRange, mList{m}.maxRange);
            end
        end
    end
    
    if maxSensorRange > 0
        allPlats = scenario.Platforms;
        numSensorPlats = numel(fieldnames(sensors));
        for pp = (numSensorPlats+1):numel(allPlats)
            tgtObj = allPlats{pp};
            tgtIdx = pp - numSensorPlats;
            try
                if isa(tgtObj.Trajectory, 'waypointTrajectory')
                    startPos = tgtObj.Trajectory.Waypoints(1,:);
                else
                    startPos = tgtObj.Trajectory.Position(:)';
                end
            catch
                continue;
            end
            
            dist = norm(startPos(1:2));  % distance from origin (radar site)
            if dist > maxSensorRange * 1.5
                issues{end+1} = makeIssue('WARNING', 'Target Beyond Sensor Range', ...
                    sprintf('Target %d starts at %.0fkm from radar — max sensor range is %.0fkm (%.0f%% beyond).', ...
                        tgtIdx, dist/1000, maxSensorRange/1000, (dist/maxSensorRange - 1)*100), ...
                    'Move target closer or increase scenario duration so it enters range.'); %#ok<AGROW>
            end
        end
    end
end

%% ---- 4. Target speed sanity ----
allPlats = scenario.Platforms;
numSensorPlats = numel(fieldnames(sensors));
for pp = (numSensorPlats+1):numel(allPlats)
    tgtObj = allPlats{pp};
    tgtIdx = pp - numSensorPlats;
    try
        if isa(tgtObj.Trajectory, 'waypointTrajectory')
            vels = tgtObj.Trajectory.Velocities;
            speeds = vecnorm(vels, 2, 2);
            maxSpeed = max(speeds);
        else
            continue;
        end
    catch
        continue;
    end
    
    maxSpeedKmh = maxSpeed * 3.6;
    if maxSpeedKmh > 5000
        issues{end+1} = makeIssue('WARNING', 'Extreme Target Speed', ...
            sprintf('Target %d reaches %.0f km/h (Mach %.1f) — hypersonic.', ...
                tgtIdx, maxSpeedKmh, maxSpeedKmh/1235), ...
            'Verify this is intentional. Most aircraft scenarios use 200-2000 km/h.'); %#ok<AGROW>
    elseif maxSpeedKmh < 10
        issues{end+1} = makeIssue('INFO', 'Very Slow Target', ...
            sprintf('Target %d moves at only %.0f km/h — near-stationary.', tgtIdx, maxSpeedKmh), ...
            'Radar may have trouble establishing track on stationary targets.'); %#ok<AGROW>
    end
end

%% ---- 5. Terrain type valid ----
validTerrains = ["none", "water", "rural", "urban", "mountain", "desert"];
if isfield(config, 'environment') && isfield(config.environment, 'terrain_type')
    tt = lower(string(config.environment.terrain_type));
    if ~any(tt == validTerrains)
        issues{end+1} = makeIssue('CRITICAL', 'Invalid Terrain Type', ...
            sprintf('terrain_type="%s" is not recognized.', tt), ...
            sprintf('Use one of: %s', strjoin(validTerrains, ', ')));
    end
end

%% ---- 6. Degradation coherence ----
if isfield(config, 'degradation')
    deg = config.degradation;
    if isfield(deg, 'type') && ~isempty(deg.type) && ~deg.enabled
        issues{end+1} = makeIssue('INFO', 'Degradation Type Set But Disabled', ...
            sprintf('degradation.type="%s" but degradation.enabled=false.', deg.type), ...
            'Set degradation.enabled=true to activate weather effects, or clear the type.');
    end
end

%% ---- 7. Tracker parameter sanity ----
if isfield(config, 'active_params')
    ap = config.active_params;
    if isfield(ap, 'gate') && ap.gate <= 0
        issues{end+1} = makeIssue('CRITICAL', 'Invalid Gate Size', ...
            sprintf('Tracker gate=%.1f — must be positive.', ap.gate), ...
            'Set tracker_params.ideal.gate or .degraded.gate to a positive value (typical: 30-120).');
    end
    if isfield(ap, 'gate') && ap.gate > 500
        issues{end+1} = makeIssue('WARNING', 'Very Large Gate', ...
            sprintf('Tracker gate=%.0f — extremely permissive data association.', ap.gate), ...
            'Large gates accept distant detections. Typical range: 30-120. Reduce if seeing ghost tracks.');
    end
    if isfield(ap, 'pd') && (ap.pd <= 0 || ap.pd > 1)
        issues{end+1} = makeIssue('CRITICAL', 'Invalid Detection Probability', ...
            sprintf('pd=%.3f — must be in (0, 1].', ap.pd), ...
            'Check tracker_global.detection_probability.ideal/degraded.');
    end
    if isfield(ap, 'confirm_threshold') && ap.confirm_threshold <= 0
        issues{end+1} = makeIssue('WARNING', 'Non-positive Confirm Threshold', ...
            sprintf('confirm_threshold=%d — tracks confirm instantly.', ap.confirm_threshold), ...
            'Typical values: 5-20. Lower = faster confirmation but more false tracks.');
    end
end

%% ---- 8. Airborne platform at sea level ----
if ~isempty(metas) && isstruct(metas)
    pNames = fieldnames(metas);
    for p = 1:numel(pNames)
        pName = pNames{p};
        if contains(lower(pName), 'aircraft') || contains(lower(pName), 'fighter')
            % Check if the aircraft platform is at z=0
            allPlats = scenario.Platforms;
            for pp = 1:numel(allPlats)
                platObj = allPlats{pp};
                try
                    if isa(platObj.Trajectory, 'waypointTrajectory')
                        startPos = platObj.Trajectory.Waypoints(1,:);
                    else
                        startPos = platObj.Trajectory.Position(:)';
                    end
                catch
                    continue;
                end
                
                if abs(startPos(3)) < 50 && pp <= numel(fieldnames(sensors))
                    % Check if this platform hosts the aircraft sensors
                    mList = metas.(pName);
                    for m = 1:numel(mList)
                        if isfield(mList{m}, 'type')
                            typeStr = upper(string(mList{m}.type));
                            if contains(typeStr, 'AESA') || contains(typeStr, 'FLIR') || ...
                               contains(typeStr, 'FIGHTER')
                                issues{end+1} = makeIssue('WARNING', 'Airborne Sensor at Ground Level', ...
                                    sprintf('Platform "%s" has airborne sensors but starts at z=%.0fm (ground level).', ...
                                        pName, -startPos(3)), ...
                                    'Define a platforms.aircraft entry in the scenario catalog with start_pos altitude.'); %#ok<AGROW>
                                break;
                            end
                        end
                    end
                end
            end
        end
    end
end

%% ---- 9. Duration vs target travel ----
if isfield(config, 'scenario') && isfield(config.scenario, 'duration_s')
    dur = config.scenario.duration_s;
    if dur < 5
        issues{end+1} = makeIssue('CRITICAL', 'Duration Too Short', ...
            sprintf('duration_s=%g — trackers need at least 5 seconds.', dur), ...
            'Increase scenario.duration_s. Typical: 30-120s depending on sensor scan rate.');
    end
end

%% ---- 10. Environment flag coherence ----
if isfield(config, 'environment')
    env = config.environment;
    tt = lower(string(env.terrain_type));
    
    if (tt == "water" || tt == "none") && isfield(env, 'terrain_occlusion') && env.terrain_occlusion
        issues{end+1} = makeIssue('INFO', 'Occlusion On Flat Terrain', ...
            sprintf('terrain_occlusion=true but terrain_type="%s" — flat surface, no occlusion effect.', tt), ...
            'This is harmless but unnecessary. Set terrain_occlusion=false or change terrain_type.');
    end
    
    if isfield(env, 'clutter_density') && env.clutter_density > 1
        issues{end+1} = makeIssue('WARNING', 'Excessive Clutter Density', ...
            sprintf('clutter_density=%.1f — should be 0-1.', env.clutter_density), ...
            'Values above 1.0 generate unrealistic false alarm rates. Typical: 0.2-0.8.');
    end
end

%% ---- Build report ----
report.issues   = issues;
report.critical = sum(cellfun(@(i) strcmp(i.severity, 'CRITICAL'), issues));
report.warnings = sum(cellfun(@(i) strcmp(i.severity, 'WARNING'), issues));
report.infos    = sum(cellfun(@(i) strcmp(i.severity, 'INFO'), issues));
report.passed   = report.critical == 0;

%% ---- Print results ----
if isempty(issues)
    fprintf('  ✓ All checks passed — scenario is ready to run.\n\n');
else
    for i = 1:numel(issues)
        iss = issues{i};
        switch iss.severity
            case 'CRITICAL'; icon = '✗'; 
            case 'WARNING';  icon = '⚠';
            case 'INFO';     icon = 'ℹ';
            otherwise;       icon = '?';
        end
        fprintf('  %s [%s] %s\n', icon, iss.severity, iss.check);
        fprintf('    %s\n', iss.message);
        fprintf('    → FIX: %s\n\n', iss.fix);
    end
    
    fprintf('  ─────────────────────────────────────────\n');
    fprintf('  Results: %d CRITICAL, %d WARNING, %d INFO\n', ...
        report.critical, report.warnings, report.infos);
    if report.passed
        fprintf('  ✓ No critical issues — simulation can proceed.\n\n');
    else
        fprintf('  ✗ CRITICAL issues found — simulation may fail or produce invalid results.\n');
        fprintf('    Fix the items above before running.\n\n');
    end
end

end


%% ========================================================================
function iss = makeIssue(severity, check, message, fix)
    iss = struct('severity', severity, 'check', check, ...
                 'message', message, 'fix', fix);
end