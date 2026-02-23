function clutterDets = generateGroundClutter(simTime, sensorPos, sensorIndex, sensorParams, envConfig)
%generateGroundClutter  Generate realistic ground clutter returns.
%
% TWO-COMPONENT CLUTTER MODEL
%   1. Surface clutter: beam-ground intersection at short-medium range.
%      Governed by grazing angle geometry — concentrated near the radar.
%   2. Discrete clutter: buildings, vehicles, wind turbines, towers.
%      Elevated scatterers that return energy at longer ranges.
%      Density depends on terrain type (urban >> rural >> water).
%
% The model does NOT use truth positions — clutter is generated purely
% from sensor geometry and terrain type, the same information a real
% radar processor would have.
%
% INPUTS
%   simTime      : current simulation time (s)
%   sensorPos    : [x, y, z] sensor position in NED meters
%   sensorIndex  : scalar sensor index for objectDetection tagging
%   sensorParams : struct with fields:
%                    rangeLimits  : [rMin, rMax] in meters
%                    rangeRes     : range resolution (m)
%                    fov          : [azBW, elBW] in degrees
%                    tilt         : beam tilt angle (deg)
%                    mountingLoc  : [x,y,z] mounting offset (NED, m)
%   envConfig    : struct with fields:
%                    terrain_type : 'water' | 'rural' | 'urban' | 'mountain'
%                    clutter_density : 0-1 scale factor (default 0.5)
%
% OUTPUT
%   clutterDets : cell array of objectDetection objects
%
% TERRAIN TYPES (expected returns per scan at density=0.5)
%   water    : ~1-2  (almost no discrete scatterers, minimal surface return)
%   rural    : ~3-6  (farms, tree lines, vehicles, occasional structures)
%   urban    : ~8-15 (buildings, infrastructure, heavy traffic)
%   mountain : ~5-10 (ridgelines, rock faces, some structures in valleys)
%
% See also: trackbench.environment.isAboveHorizon

    clutterDets = {};

    if nargin < 5 || isempty(envConfig)
        envConfig = struct('terrain_type', 'rural', 'clutter_density', 0.5);
    end
    terrainType = lower(string(envConfig.terrain_type));
    densityScale = envConfig.clutter_density;

    % ================================================================
    %  TERRAIN PARAMETERS
    % ================================================================
    %                    surfaceLambda  discreteLambda  discreteMaxFrac  noiseFloor  scattererHeight
    % surfaceLambda   : Poisson mean for surface clutter (short range)
    % discreteLambda  : Poisson mean for discrete scatterers (long range)
    % discreteMaxFrac : fraction of sensor max range where discrete clutter extends
    % noiseFloor      : measurement noise sigma (m)
    % scattererHeight : typical height of discrete scatterers (m above ground)
    switch terrainType
        case 'water'
            surfaceLambda   = 0.5;
            discreteLambda  = 0.5;   % buoys, ships, offshore platforms
            discreteMaxFrac = 0.15;
            noiseFloor      = 50;
            scattererHeight = 5;
        case 'rural'
            surfaceLambda   = 1.5;
            discreteLambda  = 2.5;   % barns, silos, vehicles, tree clusters
            discreteMaxFrac = 0.25;
            noiseFloor      = 100;
            scattererHeight = 15;
        case 'urban'
            surfaceLambda   = 2.0;
            discreteLambda  = 8.0;   % buildings, cranes, towers, traffic
            discreteMaxFrac = 0.40;
            noiseFloor      = 150;
            scattererHeight = 30;
        case 'mountain'
            surfaceLambda   = 2.0;
            discreteLambda  = 4.0;   % ridgelines, cliff faces, ski lifts
            discreteMaxFrac = 0.30;
            noiseFloor      = 200;
            scattererHeight = 50;
        otherwise
            surfaceLambda   = 1.5;
            discreteLambda  = 2.5;
            discreteMaxFrac = 0.25;
            noiseFloor      = 100;
            scattererHeight = 15;
    end

    % Apply density scale
    surfaceLambda  = surfaceLambda  * densityScale;
    discreteLambda = discreteLambda * densityScale;

    % Sensor geometry
    sensorAlt = max(-sensorPos(3), 1);  % NED: -Z = altitude, min 1m
    rMin = sensorParams.rangeLimits(1);
    rMax = sensorParams.rangeLimits(2);
    tiltDeg = sensorParams.tilt;
    elBW = sensorParams.fov(2);

    Rclutter = eye(3) * noiseFloor^2;

    % ================================================================
    %  COMPONENT 1: SURFACE CLUTTER
    % ================================================================
    %  Beam-ground intersection — concentrated at short range where the
    %  grazing angle allows surface returns. Range extends from minimum
    %  range to the point where grazing angle drops below ~0.1°.
    lowerEdgeDeg = tiltDeg - elBW / 2;

    if lowerEdgeDeg > 0
        % Beam points above horizon — surface clutter only from sidelobes
        rSurfMin = sensorAlt / tand(lowerEdgeDeg);
        surfaceLambda = surfaceLambda * 0.2;
    else
        rSurfMin = max(rMin, 500);
    end

    % Surface clutter max range (grazing angle limit)
    minGrazingDeg = 0.1;  % real radar sees surface returns to very low angles
    rSurfMax = min(rMax, sensorAlt / tand(minGrazingDeg));
    rSurfMax = max(rSurfMax, rSurfMin + 500);
    rSurfMin = max(rSurfMin, rMin);
    rSurfMax = min(rSurfMax, rMax);

    nSurface = poissrnd(surfaceLambda);
    for ii = 1:nSurface
        az = 360 * rand();
        % Weighted toward closer range (inverse-square density)
        u = rand();
        rng_c = rSurfMin + (rSurfMax - rSurfMin) * sqrt(u);

        xC = sensorPos(1) + rng_c * sind(az);
        yC = sensorPos(2) + rng_c * cosd(az);
        zC = randn() * 30;  % ground level noise

        clutterDets{end+1, 1} = objectDetection(simTime, [xC; yC; zC], ...
            'MeasurementNoise', Rclutter, ...
            'SensorIndex', sensorIndex); %#ok<AGROW>
    end

    % ================================================================
    %  COMPONENT 2: DISCRETE CLUTTER
    % ================================================================
    %  Buildings, vehicles, towers, wind turbines, etc. These are elevated
    %  objects that scatter radar energy at ranges well beyond where surface
    %  grazing fails. Range extends to a terrain-dependent fraction of the
    %  sensor's maximum range. Density is uniform in azimuth but falls off
    %  with range (fewer structures per km² at longer range from site).
    rDiscreteMin = max(rMin, 1000);  % discrete scatterers start at ~1km
    rDiscreteMax = rMax * discreteMaxFrac;
    rDiscreteMax = max(rDiscreteMax, rDiscreteMin + 1000);

    nDiscrete = poissrnd(discreteLambda);
    for ii = 1:nDiscrete
        az = 360 * rand();
        % Density falls off with range (more structures close to site)
        u = rand();
        rng_c = rDiscreteMin + (rDiscreteMax - rDiscreteMin) * u^0.7;

        xC = sensorPos(1) + rng_c * sind(az);
        yC = sensorPos(2) + rng_c * cosd(az);
        % Discrete scatterers are above ground level
        zC = -(scattererHeight * (0.5 + rand())) + randn() * 20;

        clutterDets{end+1, 1} = objectDetection(simTime, [xC; yC; zC], ...
            'MeasurementNoise', Rclutter * 1.5, ...  % discrete returns are noisier
            'SensorIndex', sensorIndex); %#ok<AGROW>
    end
end
