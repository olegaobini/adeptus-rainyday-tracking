function addTargetFromDef(scenario, tDef, duration, idx)
%addTargetFromDef  Add a target platform to a trackingScenario from a JSON definition.
%
%  Reads behavior, speed, position, and other fields from a target struct
%  (parsed from JSON) and creates the appropriate waypointTrajectory.
%
%  USAGE (called by loadScenario and loadRunFile)
%    trackbench.scenario.addTargetFromDef(scenario, tDef, 60, 1);
%
%  INPUTS
%    scenario : trackingScenario object to add the target to
%    tDef     : struct from JSON with behavior, speed_kmh, start_pos, etc.
%    duration : scenario duration in seconds
%    idx      : target index (for logging and turn direction)
%
%  See also: loadScenario, loadRunFile

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

%% Local trajectory builders
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

function vel = computeVelocities(wp, t)
    nPts = size(wp, 1); vel = zeros(nPts, 3);
    for k = 1:nPts-1
        dt = t(k+1) - t(k);
        if dt > 0; vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt; end
    end
    vel(end,:) = vel(max(1,end-1),:);
end