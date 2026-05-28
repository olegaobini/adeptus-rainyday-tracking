function plat = addOwnshipFromDef(scenario, oDef, duration, sensorList)
%addOwnshipFromDef  Add a sensor-carrying ownship platform with rich trajectory.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Mirrors addTargetFromDef's trajectory builders but for a SENSOR-CARRYING
%  ownship platform rather than a target. Differences from addTargetFromDef:
%    - Attaches sensorList to the platform at construction (target version
%      has no sensors).
%    - Skips user-specified rcsSignature and irSignature attach — the
%      ownship is the observer, not the observed. (The default Platform
%      object retains its built-in default Signatures; we just don't
%      override them with target-style RCS profiles or IR patterns.)
%    - Default trajectory uses AutoBank=true, AutoPitch=true, matching
%      MathWorks' fixed-wing aircraft pattern in the waypointTrajectory
%      doc Algorithms table (transverse plane aligns with net accel,
%      pitch aligns with path).
%
%  SUPPORTED BEHAVIORS (subset of addTargetFromDef)
%    constant_velocity : Straight line at fixed heading (legacy default)
%    gentle_turn       : Gradual heading change (always direction=+1 for
%                        ownship, since there's only one ownship per scenario)
%    s_maneuver        : Evasive S-shaped path (set turn_rate_dps)
%    crossing          : Straight line from start_pos to end_pos
%    orbit             : Circular holding pattern (set orbit_radius_m)
%    approach          : Descending glide path to end_pos
%    departure         : Climbing away to end_pos
%    waypoints         : Custom — user defines [x,y,z] waypoints + times
%
%  NOT SUPPORTED
%    recorded_flight   : NASA FDR data for an ownship platform is
%                        post-demo scope. Explicit error if requested.
%
%  HEADING CONVENTION
%    Aviation: heading_deg measured clockwise from north.
%      0° → +N (+X in NED)
%     90° → +E (+Y in NED)
%    Builder uses cosd(heading) for N component, sind(heading) for E
%    component, matching addTargetFromDef's buildConstantVelocity. The
%    pre-v3.6.16 loadRunFile.m §7 inline path used the swapped
%    convention (sind→N, cosd→E) — see v3.6.16 README Process findings.
%
%  POST-DEMO REFACTOR NOTE
%    Trajectory builders below are duplicated verbatim from
%    addTargetFromDef.m's local helper functions. Post-demo task is to
%    extract these to a shared +scenario/buildTrajectory.m module that
%    both addTargetFromDef and addOwnshipFromDef can consume. Duplicated
%    here for v3.6.16 to minimize refactor scope (touching
%    addTargetFromDef would risk PosterDemo bit-identicality).
%
%  INPUTS
%    scenario    : trackingScenario object
%    oDef        : struct from runDef.platforms.<name> with fields:
%                    behavior       : string (default "constant_velocity")
%                    speed_kmh      : scalar (default 250 → 69.4 m/s)
%                    start_pos      : [x, y, z] in NED meters
%                                     (default [0, 0, -3000])
%                    altitude_m     : scalar; overrides start_pos(3) sign
%                    heading_deg    : scalar (default 90 → east)
%                    turn_rate_dps  : scalar (s_maneuver only)
%                    end_pos        : [x, y, z] (crossing, approach,
%                                     departure)
%                    orbit_radius_m : scalar (orbit only)
%                    waypoints      : array of {pos, time_s} (waypoints
%                                     behavior only)
%                    label          : string for logging
%                    class_id       : integer for ClassID
%                    dimensions     : struct with length_m/width_m/height_m
%    duration    : scenario duration (s)
%    sensorList  : cell array of sensor objects to attach at construction
%
%  OUTPUT
%    plat : created Platform object
%
%  See also: addTargetFromDef, waypointTrajectory, platform

    %% Parse common fields with defaults (mirrors addTargetFromDef pattern)
    behavior = "constant_velocity";
    if isfield(oDef, 'behavior'); behavior = lower(string(oDef.behavior)); end
    speed_ms = 250 / 3.6;   % 250 km/h default → 69.4 m/s
    if isfield(oDef, 'speed_kmh'); speed_ms = oDef.speed_kmh * 1000 / 3600; end
    startPos = [0, 0, -3000];   % default airborne altitude
    if isfield(oDef, 'start_pos'); startPos = reshape(oDef.start_pos, 1, []); end
    altitude = abs(startPos(3));
    if isfield(oDef, 'altitude_m'); altitude = oDef.altitude_m; end
    startPos(3) = -abs(altitude);
    heading = 90;   % east default
    if isfield(oDef, 'heading_deg'); heading = oDef.heading_deg; end
    T = duration;

    %% Parse label for logging
    label = behavior;
    if isfield(oDef, 'label'); label = string(oDef.label); end

    %% Build trajectory based on behavior
    switch behavior
        case "constant_velocity"
            [wp, t, vel] = buildConstantVelocity(startPos, heading, speed_ms, T);
        case "gentle_turn"
            % idx=1 fixed for ownship (only one ownship per scenario;
            % addTargetFromDef uses idx parity for multi-target direction
            % alternation — irrelevant here).
            [wp, t, vel] = buildGentleTurn(startPos, speed_ms, T, 1);
        case "s_maneuver"
            turnRate = 2;
            if isfield(oDef, 'turn_rate_dps'); turnRate = oDef.turn_rate_dps; end
            [wp, t, vel] = buildSManeuver(startPos, heading, speed_ms, turnRate, T);
        case "crossing"
            endPos = startPos + [10000, 0, 0];   % default 10km north
            if isfield(oDef, 'end_pos'); endPos = reshape(oDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
        case "orbit"
            radius = 3000;
            if isfield(oDef, 'orbit_radius_m'); radius = oDef.orbit_radius_m; end
            [wp, t, vel] = buildOrbit(startPos, radius, speed_ms, T);
        case "approach"
            endPos = [0, 0, -100];   % default approach to near-ground
            if isfield(oDef, 'end_pos'); endPos = reshape(oDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildApproach(startPos, endPos, T);
        case "departure"
            endPos = startPos + [0, 50000, -5000];   % climb-out 50km east, +5km alt
            if isfield(oDef, 'end_pos'); endPos = reshape(oDef.end_pos, 1, []); end
            endPos(3) = -abs(endPos(3));
            [wp, t, vel] = buildCrossing(startPos, endPos, T);
        case "waypoints"
            [wp, t, vel] = buildFromWaypoints(oDef);
        case "recorded_flight"
            error('addOwnshipFromDef:unsupportedBehavior', ...
                ['behavior="recorded_flight" is not supported for ownship ' ...
                 'platforms in v3.6.16. NASA FDR for a sensor-carrying ' ...
                 'platform is post-demo scope.']);
        otherwise
            error('addOwnshipFromDef:unknownBehavior', ...
                'Unknown behavior "%s".', behavior);
    end

    %% Create platform WITH sensors attached at construction
    plat = platform(scenario, 'Sensors', sensorList);

    %% Assign trajectory — AutoBank+AutoPitch true for fixed-wing aircraft
    %  (per waypointTrajectory R2025b Algorithms truth table: roll aligns
    %  transverse plane with net acceleration, pitch aligns with path).
    plat.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, ...
        'TimeOfArrival', t, ...
        'Velocities', vel, ...
        'AutoBank', true, 'AutoPitch', true);

    %% Optional Platform metadata (rare for ownship, but allowed)
    if isfield(oDef, 'class_id')
        plat.ClassID = oDef.class_id;
    end
    if isfield(oDef, 'dimensions') && isstruct(oDef.dimensions)
        d = oDef.dimensions;
        dimStruct = struct('Length', 0, 'Width', 0, 'Height', 0, 'OriginOffset', [0 0 0]);
        if isfield(d, 'length_m'); dimStruct.Length = d.length_m; end
        if isfield(d, 'width_m');  dimStruct.Width  = d.width_m;  end
        if isfield(d, 'height_m'); dimStruct.Height = d.height_m; end
        plat.Dimensions = dimStruct;
    end

    %% Log
    nWP = size(wp, 1);
    fprintf('  Ownship: %s | %d sensor(s) | behavior=%s | %d waypoint(s), %.0fs\n', ...
        label, numel(sensorList), char(behavior), nWP, t(end));
end


%% ========================================================================
%  TRAJECTORY BUILDERS (duplicated verbatim from addTargetFromDef.m)
%
%  Post-demo cleanup: extract these to a shared +scenario/buildTrajectory.m
%  module so both addTargetFromDef and addOwnshipFromDef call a single
%  implementation. Duplicated here for v3.6.16 to keep the refactor
%  scope minimal and avoid touching addTargetFromDef.m (which would risk
%  PosterDemo bit-identicality).
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

function [wp, t, vel] = buildFromWaypoints(oDef)
%buildFromWaypoints  Build trajectory from user-defined waypoints in JSON.
%
%  Expects oDef.waypoints as an array of structs with fields:
%    pos    : [x, y, z] in NED meters (z negative for altitude)
%    time_s : arrival time in seconds
%
%  Velocities are auto-computed from waypoint spacing.

    if ~isfield(oDef, 'waypoints') || isempty(oDef.waypoints)
        error('addOwnshipFromDef:noWaypoints', ...
            'behavior="waypoints" requires a "waypoints" array with pos and time_s fields.');
    end

    wpDefs = oDef.waypoints;
    nPts = numel(wpDefs);
    if nPts < 2
        error('addOwnshipFromDef:tooFewWaypoints', ...
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

    % Ensure times are monotonically increasing
    if any(diff(t) <= 0)
        error('addOwnshipFromDef:nonMonotonicTime', ...
            'Waypoint times must be strictly increasing.');
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
