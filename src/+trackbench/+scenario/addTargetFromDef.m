function addTargetFromDef(scenario, tDef, duration, idx)
%addTargetFromDef  Add a target platform to a trackingScenario from a JSON definition.
%
%  Creates a platform with waypointTrajectory and optional RCS signature,
%  physical dimensions, class ID, and label.
%
%  SUPPORTED BEHAVIORS
%    constant_velocity : Straight line at fixed heading
%    gentle_turn       : Gradual heading change
%    s_maneuver        : Evasive S-shaped path (set turn_rate_dps)
%    crossing          : Straight line from start_pos to end_pos
%    orbit             : Circular holding pattern (set orbit_radius_m)
%    approach          : Descending glide path to end_pos
%    departure         : Climbing away to end_pos
%    waypoints         : ★ Custom — user defines [x,y,z] waypoints + times
%
%  OPTIONAL FIELDS (all behaviors)
%    rcs_dbsm    : Radar cross section in dBsm (scalar). Sets platform.Signatures.
%                  Ref: https://www.mathworks.com/help/fusion/ref/rcssignature.html
%    dimensions  : struct with length_m, width_m, height_m. Sets platform.Dimensions.
%                  Ref: https://www.mathworks.com/help/fusion/ref/platform.html
%    class_id    : Integer classification (0=unknown, user-defined scheme).
%    label       : String label for logging and plots.
%
%  WAYPOINTS BEHAVIOR FORMAT
%    "waypoints": [
%      { "pos": [x, y, z], "time_s": 0 },
%      { "pos": [x, y, z], "time_s": 10 },
%      ...
%    ]
%    Uses MATLAB's waypointTrajectory directly — any path, any timing.
%    Ref: https://www.mathworks.com/help/fusion/ref/waypointtrajectory-system-object.html
%
%  See also: waypointTrajectory, rcsSignature, platform

    %% Parse common fields with defaults
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

    %% Parse label for logging
    label = behavior;
    if isfield(tDef, 'label'); label = string(tDef.label); end

    %% Build trajectory based on behavior
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
        case "waypoints"
            [wp, t, vel] = buildFromWaypoints(tDef);
        otherwise
            endPos = startPos + 3000*(2*rand(1,3)-1);
            endPos(3) = startPos(3);
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
    end

    %% Create platform
    tgt = platform(scenario);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, 'TimeOfArrival', t, 'Velocities', vel);

    %% ── RCS Signature (target size + aspect dependence) ───────────────
    % rcsSignature sets the radar cross section seen by fusionRadarSensor.
    % Two modes:
    %   rcs_dbsm  : scalar (isotropic) — same RCS from all angles
    %   rcs_profile : named preset — aspect-dependent pattern
    %     Options: 'stealth', 'fighter', 'airliner', 'drone', 'missile'
    %     Uses buildRCSProfile() which creates a full az×el pattern matrix.
    %     fusionRadarSensor automatically interpolates the pattern based on
    %     the current aspect angle between radar and target.
    % Ref: https://www.mathworks.com/help/fusion/ref/rcssignature.html
    if isfield(tDef, 'rcs_profile') && ~isempty(tDef.rcs_profile)
        baseRCS = 0;
        if isfield(tDef, 'rcs_dbsm'); baseRCS = tDef.rcs_dbsm; end
        tgt.Signatures = {trackbench.environment.buildRCSProfile(string(tDef.rcs_profile), baseRCS)};
    elseif isfield(tDef, 'rcs_dbsm')
        rcs_val = tDef.rcs_dbsm;
        tgt.Signatures = {rcsSignature('Pattern', rcs_val)};
    end

    %% ── Physical Dimensions ──────────────────────────────────────────
    % Sets the cuboid approximation for visualization and extended object tracking.
    % Ref: https://www.mathworks.com/help/fusion/ref/platform.html
    if isfield(tDef, 'dimensions') && isstruct(tDef.dimensions)
        d = tDef.dimensions;
        dimStruct = struct('Length', 0, 'Width', 0, 'Height', 0, 'OriginOffset', [0 0 0]);
        if isfield(d, 'length_m'); dimStruct.Length = d.length_m; end
        if isfield(d, 'width_m');  dimStruct.Width  = d.width_m;  end
        if isfield(d, 'height_m'); dimStruct.Height = d.height_m; end
        tgt.Dimensions = dimStruct;
    end

    %% ── Class ID ─────────────────────────────────────────────────────
    % Integer for target classification. 0 = unknown (default).
    % SSR/IFF sensors use this for identification.
    if isfield(tDef, 'class_id')
        tgt.ClassID = tDef.class_id;
    end

    %% Log
    rcsStr = '';
    if isfield(tDef, 'rcs_dbsm')
        rcsStr = sprintf(' | RCS=%.0f dBsm', tDef.rcs_dbsm);
    end
    dimStr = '';
    if isfield(tDef, 'dimensions') && isstruct(tDef.dimensions)
        d = tDef.dimensions;
        dimStr = sprintf(' | %.0fx%.0fx%.0fm', ...
            getFieldDef(d,'length_m',0), getFieldDef(d,'width_m',0), getFieldDef(d,'height_m',0));
    end

    if behavior == "waypoints"
        nWP = size(wp, 1);
        fprintf('  Target %d: %s (%d waypoints, %.0fs)%s%s\n', ...
            idx, label, nWP, t(end), rcsStr, dimStr);
    else
        fprintf('  Target %d: %s | %.0f km/h | start=[%.0f,%.0f,%.0f]%s%s\n', ...
            idx, label, speed_ms*3.6, startPos(1), startPos(2), startPos(3), rcsStr, dimStr);
    end
end


%% ========================================================================
%  WAYPOINT BUILDER (new — user-defined paths)
%% ========================================================================
function [wp, t, vel] = buildFromWaypoints(tDef)
%buildFromWaypoints  Build trajectory from user-defined waypoints in JSON.
%
%  Expects tDef.waypoints as an array of structs with fields:
%    pos    : [x, y, z] in NED meters (z negative for altitude)
%    time_s : arrival time in seconds
%
%  Velocities are auto-computed from waypoint spacing.

    if ~isfield(tDef, 'waypoints') || isempty(tDef.waypoints)
        error('addTargetFromDef:noWaypoints', ...
            'behavior="waypoints" requires a "waypoints" array with pos and time_s fields.');
    end

    wpDefs = tDef.waypoints;
    nPts = numel(wpDefs);
    if nPts < 2
        error('addTargetFromDef:tooFewWaypoints', ...
            'Need at least 2 waypoints, got %d.', nPts);
    end

    wp = zeros(nPts, 3);
    t  = zeros(nPts, 1);

    for k = 1:nPts
        wk = wpDefs(k);
        if isstruct(wk)
            wp(k,:) = reshape(wk.pos, 1, []);
            t(k) = wk.time_s;
        elseif iscell(wpDefs) && isstruct(wpDefs{k})
            wk = wpDefs{k};
            wp(k,:) = reshape(wk.pos, 1, []);
            t(k) = wk.time_s;
        end
    end

    % Ensure Z is negative for altitude (NED convention)
    for k = 1:nPts
        if isfield(tDef, 'altitude_m') && wp(k,3) >= 0
            % If user forgot NED sign convention, fix it
            wp(k,3) = -abs(wp(k,3));
        end
    end

    % Ensure times are monotonically increasing
    if any(diff(t) <= 0)
        error('addTargetFromDef:nonMonotonicTime', ...
            'Waypoint times must be strictly increasing.');
    end

    vel = computeVelocities(wp, t);
end


%% ========================================================================
%  EXISTING TRAJECTORY BUILDERS (unchanged)
%% ========================================================================
function [wp, t, vel] = buildConstantVelocity(startPos, heading, speed, T)
    dx = speed * cosd(heading); dy = speed * sind(heading);
    endPos = startPos + [dx dy 0] * T;
    wp = [startPos; endPos]; t = [0; T]; v = (endPos - startPos) / T; vel = [v; v];
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
    wp = [startPos; endPos]; t = [0; T]; v = (endPos - startPos) / T; vel = [v; v];
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


%% ========================================================================
%  SHARED HELPERS
%% ========================================================================
function vel = computeVelocities(wp, t)
    nPts = size(wp, 1); vel = zeros(nPts, 3);
    for k = 1:nPts-1
        dt = t(k+1) - t(k);
        if dt > 0; vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt; end
    end
    vel(end,:) = vel(max(1,end-1),:);
end

function val = getFieldDef(s, field, default)
    if isstruct(s) && isfield(s, field); val = s.(field); else; val = default; end
end