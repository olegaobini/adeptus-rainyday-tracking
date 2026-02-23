function [F_twoway_dB, details] = computePropFactor(sensorPos, targetPos, freq, terrainType, refractionK)
%computePropFactor  Physics-based two-way radar propagation factor.
%
%  Uses MATLAB Radar Toolbox's radarpropfactor to compute the multipath
%  propagation factor for a given sensor–target geometry and terrain type.
%  The propagation factor accounts for ground-bounce multipath interference
%  that creates lobing patterns in the vertical plane — at some elevation
%  angles the signal is constructively enhanced (+dB), at others it is
%  destructively nullified (deep nulls, -20 dB or worse).
%
%  PHYSICS MODELLED:
%    - Flat-earth multipath interference (direct + ground-reflected path)
%    - Surface roughness scattering (terrain-dependent)
%    - Surface permittivity (terrain-dependent)
%    - Vegetation attenuation
%    - Atmospheric refraction (effective Earth radius model)
%    - Diffraction beyond the radar horizon
%
%  INPUTS
%    sensorPos   : [x y z] sensor position in NED (m). z is negative for
%                  above-ground (NED convention).
%    targetPos   : [x y z] target position in NED (m), or Nx3 matrix for
%                  multiple targets.
%    freq        : radar frequency in Hz (e.g. 2.8e9 for S-band)
%    terrainType : 'rural'|'urban'|'mountain'|'water'|'desert' (string)
%    refractionK : atmospheric refraction factor (default 4/3)
%
%  OUTPUTS
%    F_twoway_dB : Nx1 two-way propagation factor in dB for each target.
%                  Positive = constructive multipath (stronger than free space)
%                  Zero     = free space equivalent
%                  Negative = destructive multipath (weaker than free space)
%                  -Inf     = below horizon / completely shadowed
%    details     : struct with diagnostic fields (surfacePermittivity,
%                  surfaceRoughness, effectiveEarthRadius, ranges, heights)
%
%  EXAMPLE
%    sPos = [0, 0, -15];             % 15m tower (NED: z=-15 is 15m up)
%    tPos = [0, -30000, -3000];      % target 30km away, 3km altitude
%    F = trackbench.environment.computePropFactor(sPos, tPos, 2.8e9, 'rural');
%    fprintf('Two-way factor: %.1f dB\n', F);
%
%  REQUIRES: Radar Toolbox (radarpropfactor, landroughness, searoughness,
%            earthSurfacePermittivity, effearthradius, refractiveidx)
%
% See also: radarpropfactor, landroughness, earthSurfacePermittivity

    if nargin < 5 || isempty(refractionK)
        refractionK = 4/3;
    end
    
    % Ensure targetPos is Nx3
    if size(targetPos, 2) ~= 3
        targetPos = targetPos';
    end
    nTargets = size(targetPos, 1);
    
    %% Sensor antenna height (NED: height = -z)
    antennaHeight = max(-sensorPos(3), 1);  % at least 1m
    
    %% Terrain → surface properties (cached via persistent)
    persistent cachedTerrain cachedEpsc cachedHgtsd cachedSlope cachedVeg cachedRe
    
    terrainKey = lower(string(terrainType)) + "_" + string(freq);
    needRecalc = isempty(cachedTerrain) || ~strcmp(cachedTerrain, terrainKey);
    
    if needRecalc
        [epsc, hgtsd, slope, vegType, Re] = getSurfaceProperties(freq, terrainType, refractionK);
        cachedTerrain = terrainKey;
        cachedEpsc    = epsc;
        cachedHgtsd   = hgtsd;
        cachedSlope   = slope;
        cachedVeg     = vegType;
        cachedRe      = Re;
    else
        epsc    = cachedEpsc;
        hgtsd   = cachedHgtsd;
        slope   = cachedSlope;
        vegType = cachedVeg;
        Re      = cachedRe;
    end
    
    %% Compute propagation factor for each target
    F_twoway_dB = zeros(nTargets, 1);
    ranges_m    = zeros(nTargets, 1);
    heights_m   = zeros(nTargets, 1);
    
    for i = 1:nTargets
        % Slant range (meters)
        dx = targetPos(i,:) - sensorPos;
        R = norm(dx);
        ranges_m(i) = R;
        
        % Target height above ground (NED: height = -z)
        tgtHeight = max(-targetPos(i,3), 0.1);  % at least 0.1m
        heights_m(i) = tgtHeight;
        
        % Skip if range is too short (< 500m) — propagation model not valid
        if R < 500
            F_twoway_dB(i) = 0;  % free space at close range
            continue;
        end
        
        % Call radarpropfactor (range in METERS, heights in METERS)
        try
            F_oneway = radarpropfactor(R, freq, antennaHeight, tgtHeight, ...
                'SurfaceRelativePermittivity', epsc, ...
                'SurfaceHeightStandardDeviation', hgtsd, ...
                'SurfaceSlope', slope, ...
                'VegetationType', vegType, ...
                'EffectiveEarthRadius', Re);
            
            F_twoway_dB(i) = 2 * F_oneway;  % two-way = 2 × one-way (dB)
        catch
            % Graceful fallback — free space if radarpropfactor fails
            F_twoway_dB(i) = 0;
        end
    end
    
    %% Diagnostic output
    if nargout > 1
        details = struct();
        details.surfacePermittivity = epsc;
        details.surfaceRoughness    = hgtsd;
        details.surfaceSlope        = slope;
        details.vegetationType      = vegType;
        details.effectiveEarthRadius = Re;
        details.antennaHeight       = antennaHeight;
        details.ranges_m            = ranges_m;
        details.heights_m           = heights_m;
        details.F_oneway_dB         = F_twoway_dB / 2;
    end
end


%% ========================================================================
function [epsc, hgtsd, slope, vegType, Re] = getSurfaceProperties(freq, terrainType, refractionK)
%getSurfaceProperties  Map terrain type string to physical surface parameters.
%
%  Returns complex permittivity, surface height std dev, surface slope,
%  vegetation type, and effective Earth radius for the given terrain.

    terrainType = lower(string(terrainType));
    
    % Effective Earth radius (same for all terrain types)
    try
        [nidx, ~] = refractiveidx([0 1e3], 'LatitudeModel', 'Mid', 'Season', 'Summer');
        RGrad = (nidx(2) - nidx(1)) / 1e3;
        Re = effearthradius(RGrad);
    catch
        % Fallback: standard 4/3 Earth
        Re = refractionK * 6371e3;
    end
    
    switch terrainType
        case "rural"
            [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 21, 0.3);
            [hgtsd, slope] = landroughness("Farm");
            vegType = 'Grass';
            
        case "urban"
            [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 21, 0.1);
            [hgtsd, slope] = landroughness("Urban");
            vegType = 'None';
            
        case "mountain"
            [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 15, 0.4);
            [hgtsd, slope] = landroughness("Mountains");
            vegType = 'Trees';
            
        case "desert"
            % Dry soil: sand=70%, clay=10%, specific gravity=2.65, vwc=0.05
            [~,~,epsc] = earthSurfacePermittivity("soil", freq, 35, 70, 10, 2.65, 0.05);
            [hgtsd, slope] = landroughness("Desert");
            vegType = 'None';
            
        case "water"
            [~,~,epsc] = earthSurfacePermittivity("sea-water", freq, 15, 35);
            hgtsd = searoughness(3);  % sea state 3 (moderate)
            slope = 0.01;            % smooth sea
            vegType = 'None';
            
        otherwise
            % Default to rural
            [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 21, 0.3);
            [hgtsd, slope] = landroughness("Farm");
            vegType = 'Grass';
    end
end
