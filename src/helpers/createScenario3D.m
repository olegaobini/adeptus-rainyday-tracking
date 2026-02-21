function scenario = createScenario3D(varargin)
%helperCreateScenario3D  Create a trackingScenario with a tower-mounted rotating radar
%                        and a configurable number of 3D aircraft targets.
%
% PURPOSE
%   This helper builds the simulation "world" used by the tracking study:
%     - One stationary radar tower platform with a fusionRadarSensor
%     - N moving target platforms (aircraft) with simple waypoint trajectories
%     - A consistent scan/update rate derived from radar RPM and field-of-view
%
% WHY THIS FILE MATTERS
%   Scenario geometry (target paths, altitude changes, scan sector, FOV, etc.)
%   strongly determines how ambiguous the tracking problem is. If results look
%   odd, the first sanity-check is always: plot the truth trajectories and make
%   sure they match your intended "closely spaced / ambiguous" situation.
%
% COORDINATE FRAME NOTE (IMPORTANT)
%   This scenario uses the radarScenario "NED" convention:
%     x = North (meters)
%     y = East  (meters)
%     z = Down  (meters)
%   That means:
%     - Higher altitude => MORE NEGATIVE z
%     - "Climb" => z becomes more negative (e.g., -3000 -> -4000)
%
% USAGE
%   scenario = helperCreateScenario3D();                       % default (2 targets, 50s)
%   scenario = helperCreateScenario3D("NumTargets",3);         % 3 targets
%   scenario = helperCreateScenario3D("SceneDuration",60);     % 60 seconds
%   scenario = helperCreateScenario3D("NumTargets",4,"SceneDuration",45);
%
% NAME-VALUE PARAMETERS
%   "NumTargets"     : number of target platforms (>= 1). Default 2.
%   "SceneDuration"  : total scenario duration in seconds. Default 50.
%
% OUTPUT
%   scenario : trackingScenario object containing platforms + radar sensor
% -------------------------------------------------------------------------

%% Parse inputs
p = inputParser;
addParameter(p,"NumTargets",2,@(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p,"SceneDuration",50,@(x)isnumeric(x)&&isscalar(x)&&x>0);
parse(p,varargin{:});
numTargets = p.Results.NumTargets;
sceneDuration = p.Results.SceneDuration;

%% Create scenario container
% trackingScenario holds platforms and advances time via advance(scenario)
scenario = trackingScenario;

%% ---------------- Radar definition ----------------
% We keep radar settings close to the baseline example for comparability.
%
% rpm / scanrate:
%   Defines how quickly the radar sweeps azimuth.
% fov:
%   [azimuth; elevation] degrees
% updaterate:
%   The sensor returns detections once per "azimuth resolution cell" as it scans.
rpm = 25;
fov = [1.5;10];               % [Az; El] degrees
scanrate   = rpm*360/60;      % deg/s
updaterate = scanrate/fov(1); % Hz (updates per azimuth resolution)

% DetectionProbability and FalseAlarmRate are radar-level assumptions.
% NOTE: In our "RAINY" experiments we may also inject degradation at the
% detection level after sensor output (see helperRunDetections).
pd  = 0.8;
far = 1e-6;

% fusionRadarSensor configured as a mechanical rotator
radar = fusionRadarSensor(1,'Rotator', ...
    'UpdateRate', updaterate, ...
    'FieldOfView', fov, ...
    'MaxAzimuthScanRate', scanrate, ...
    'AzimuthResolution', fov(1), ...
    'ReferenceRange', 111e3, ...
    'ReferenceRCS', 0, ...
    'RangeResolution', 135, ...
    'HasINS', true, ...
    'MechanicalAzimuthLimits', [250 290], ...  % sector scan (deg)
    'DetectionCoordinates', 'Scenario', ...     % outputs in scenario NED coords
    'DetectionProbability', pd, ...
    'FalseAlarmRate', far);

% MountingLocation is relative to the platform body frame.
% Here we place the radar slightly "down" (positive Down in NED).
radar.MountingLocation = [0 0 -15];

% Create tower platform with the radar sensor attached.
% We don't assign a trajectory => tower stays fixed at origin.
tower = platform(scenario, 'Sensors', radar); %#ok<NASGU>

% Enable elevation scanning geometry.
% MechanicalElevationLimits: [min max] degrees relative to mount.
radar.HasElevation = true;
tilt = 2; % deg
radar.MechanicalElevationLimits = [-fov(2) 0] - tilt;
radar.FieldOfView(2) = fov(2) + 1e-3; % small epsilon prevents edge issues

% Set scenario update rate to match radar update rate for clean alignment.
scenario.UpdateRate = radar.UpdateRate;

%% ---------------- Targets ----------------
% We create target platforms with waypointTrajectory objects.
% The addAircraft helper contains example maneuver patterns.

switch numTargets
    case 1
        addAircraft(scenario, sceneDuration, 1);
    case 2
        addAircraft(scenario, sceneDuration, 1);
        addAircraft(scenario, sceneDuration, 2);
    otherwise
        for i = 1:numTargets
            addAircraft(scenario, sceneDuration, i);
        end
end
end

function addAircraft(scenario, T, idx)
%addAircraft  Add one aircraft platform with a simple waypoint trajectory.
%
% INPUTS
%   scenario : trackingScenario object to append the platform into
%   T        : scene duration (seconds)
%   idx      : which pattern to use (1,2 are hand-designed; others random-ish)
%
% DESIGN NOTES
%   - Waypoints are in NED meters [x y z] (z is Down).
%   - TimeOfArrival defines the time each waypoint is reached.
%   - We provide explicit velocities for clarity/repeatability (though
%     waypointTrajectory can infer them if omitted).
%
% If you want a different test, edit these waypoint patterns. The ambiguity
% of the tracking problem is mainly controlled by:
%   - How close the targets get in position
%   - Whether they cross / maneuver during the ambiguity window
%   - Whether the radar sector scan sees both simultaneously
% -------------------------------------------------------------------------

    switch idx
        case 1
            % Pattern 1: straight segment, then climb, then turn
            % Times scale with T
            tNorm = [0 0.3 0.5 0.8 1.0];  % Normalized times (0 to 1)
            t = tNorm * T;                 % Scale to actual duration
            
            wp = [ ...
                -2000    -20000   -3000;
                 0       -20000   -3000;
                 800     -19800   -3300;
                 1200    -19000   -3600;
                 1500    -18000   -4000];

            vel = zeros(numel(t), 3);
            for k = 1:numel(t)-1
                dt = t(k+1) - t(k);
                vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt;
            end
            vel(end,:) = vel(end-1,:);

        case 2
            % Pattern 2: opposite direction with gentle descent
            tNorm = [0 0.25 0.5 0.75 1.0];  % Normalized times
            t = tNorm * T;
            
            wp = [ ...
                2000     -19000   -3500;
                800      -19000   -3500;
                200      -19500   -3400;
                -600     -20000   -3300;
                -1200    -20500   -3200];

            vel = zeros(numel(t), 3);
            for k = 1:numel(t)-1
                dt = t(k+1) - t(k);
                if dt > 0
                    vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt;
                else
                    vel(k,:) = vel(k-1,:);  % Avoid division by zero
                end
            end
            vel(end,:) = vel(end-1,:);

        otherwise
            % Generic fallback
            t = [0 T]';
            wp = [ ...
                -1500 + 3000*rand,  -20500 + 3000*rand,  -2500 - 1500*rand;
                 1500 - 3000*rand,  -20500 + 3000*rand,  -2500 - 1500*rand];
            vel = repmat((wp(2,:) - wp(1,:)) / T, numel(t), 1);
    end

    % Create target platform
    tgt = platform(scenario);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, ...
        'TimeOfArrival', t, ...
        'Velocities', vel);
end
