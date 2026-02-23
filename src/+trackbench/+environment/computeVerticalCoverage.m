function vcpData = computeVerticalCoverage(sensorInfos, terrainType, refractionK)
%computeVerticalCoverage  Pre-compute vertical coverage patterns for sensors.
%
%  Uses MATLAB Radar Toolbox's radarvcd to compute the maximum detection
%  range as a function of elevation angle for each radar sensor. The
%  vertical coverage pattern (VCP) accounts for multipath ground-bounce
%  interference that creates lobing in the vertical plane — constructive
%  at some elevations (range > free-space) and destructive at others
%  (deep nulls where detection is impossible).
%
%  PHYSICS MODELLED:
%    - Multipath interference (direct + ground-reflected path)
%    - Surface roughness scattering (terrain-dependent)
%    - Surface permittivity (terrain/frequency-dependent)
%    - Vegetation attenuation
%    - Atmospheric refraction (effective Earth radius)
%
%  INPUTS
%    sensorInfos : struct array from runDetections sensor discovery, with:
%                    .sensor     — fusionRadarSensor object
%                    .platform   — platform object
%                    .isRadar    — logical
%                    .isMSSR     — logical
%                    .radarFreq  — operating frequency (Hz)
%    terrainType : 'rural'|'urban'|'mountain'|'water'|'desert' (string)
%    refractionK : atmospheric refraction factor (default 4/3)
%
%  OUTPUT
%    vcpData : struct array (one per sensor) with:
%                .sensorIndex  — sensor index
%                .angles       — elevation angles (deg), 0° to 60°
%                .maxRange_m   — max detection range (m) at each angle
%                .freeSpaceRange_m — configured free-space range (m)
%                .antennaHeight_m  — antenna height above ground (m)
%                .terrainType  — terrain type used
%                .enabled      — false for MSSR/non-radar (VCP not applied)
%
%  DESIGN NOTES
%    - VCP is computed ONCE at simulation setup, not per-scan. The pattern
%      depends on fixed geometry (antenna height, frequency, terrain) which
%      doesn't change during a run.
%    - MSSR/SSR sensors skip VCP (transponder link budget is different from
%      radar multipath model).
%    - The VCP from radarvcd is in km; we convert to meters for internal use.
%    - Surface height std dev is capped at 90% of antenna height per the
%      radarvcd constraint (ANHT >= SurfaceHeightStandardDeviation).
%
%  REQUIRES: Radar Toolbox (radarvcd, landroughness, searoughness,
%            earthSurfacePermittivity, effearthradius, refractiveidx)
%
% See also: radarvcd, blakechart, landroughness, earthSurfacePermittivity

    if nargin < 3 || isempty(refractionK)
        refractionK = 4/3;
    end
    
    nSensors = numel(sensorInfos);
    vcpData  = repmat(struct('sensorIndex',0,'angles',[],'maxRange_m',[], ...
        'freeSpaceRange_m',0,'antennaHeight_m',0,'terrainType','','enabled',false), ...
        nSensors, 1);
    
    % Pre-compute effective Earth radius (shared across all sensors)
    try
        [nidx, ~] = refractiveidx([0 1e3], 'LatitudeModel', 'Mid', 'Season', 'Summer');
        RGrad = (nidx(2) - nidx(1)) / 1e3;
        Re = effearthradius(RGrad);
    catch
        Re = refractionK * 6371e3;
    end
    
    for k = 1:nSensors
        si = sensorInfos(k);
        vcpData(k).sensorIndex = si.sensorIndex;
        vcpData(k).terrainType = string(terrainType);
        
        % Skip non-radar and MSSR sensors
        if ~si.isRadar || si.isMSSR
            vcpData(k).enabled = false;
            continue;
        end
        
        %% Antenna height (NED: z-negative = altitude)
        platZ = 0;
        try platZ = si.platform.InitialPosition(3); catch; end
        mountZ = 0;
        try mountZ = si.sensor.MountingLocation(3); catch; end
        antennaHeight = max(-(platZ + mountZ), 5);  % at least 5m
        vcpData(k).antennaHeight_m = antennaHeight;
        
        %% Free-space range from sensor config
        freeSpaceRange_m = 111120;  % 60nm default
        try freeSpaceRange_m = si.sensor.RangeLimits(2); catch; end
        vcpData(k).freeSpaceRange_m = freeSpaceRange_m;
        freeSpaceRange_km = freeSpaceRange_m / 1000;
        
        %% Radar frequency
        freq = si.radarFreq;
        if isempty(freq) || freq <= 0
            freq = 2.8e9;  % S-band default
        end
        
        %% Surface properties from terrain type
        [epsc, hgtsd, slope, vegType] = getSurfaceParams(freq, terrainType);
        
        % Cap surface roughness at antenna height (model constraint)
        hgtsd = min(hgtsd, antennaHeight * 0.9);
        hgtsd = max(hgtsd, 0.01);  % minimum 1cm
        
        %% Compute VCP
        try
            nvargs = {'RangeUnit', 'km', 'HeightUnit', 'm', ...
                      'SurfaceHeightStandardDeviation', hgtsd, ...
                      'EffectiveEarthRadius', Re/1000};  % Re in km for radarvcd
            
            if ~isempty(epsc)
                nvargs = [nvargs, 'SurfaceRelativePermittivity', epsc]; %#ok<AGROW>
            end
            if slope > 0
                nvargs = [nvargs, 'SurfaceSlope', slope]; %#ok<AGROW>
            end
            if ~isempty(vegType)
                nvargs = [nvargs, 'VegetationType', vegType]; %#ok<AGROW>
            end
            
            [vcp_km, angles] = radarvcd(freq, freeSpaceRange_km, antennaHeight, nvargs{:});
            
            vcpData(k).angles      = angles;
            vcpData(k).maxRange_m  = vcp_km * 1000;  % convert km → m
            vcpData(k).enabled     = true;
            
            % Diagnostic summary
            posIdx = angles > 0 & angles < 30;
            if any(posIdx)
                avgRange = mean(vcp_km(posIdx));
                minRange = min(vcp_km(posIdx));
                maxRange = max(vcp_km(posIdx));
                fprintf('[VCP] Sensor %d (%.1f GHz, %.0fm ant, %s): avg=%.0f km, min=%.0f km, max=%.0f km (free-space=%.0f km)\n', ...
                    si.sensorIndex, freq/1e9, antennaHeight, terrainType, ...
                    avgRange, minRange, maxRange, freeSpaceRange_km);
            end
            
        catch ME
            warning('computeVerticalCoverage:vcpFailed', ...
                'VCP computation failed for sensor %d: %s. Propagation model disabled for this sensor.', ...
                si.sensorIndex, ME.message);
            vcpData(k).enabled = false;
        end
    end
end


%% ========================================================================
function [epsc, hgtsd, slope, vegType] = getSurfaceParams(freq, terrainType)
%getSurfaceParams  Map terrain type to physical surface parameters for VCP.

    terrainType = lower(string(terrainType));
    
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
            try
                [~,~,epsc] = earthSurfacePermittivity("soil", freq, 35, 70, 10, 2.65, 0.05);
            catch
                [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 35, 0.05);
            end
            [hgtsd, slope] = landroughness("Desert");
            vegType = 'None';
            
        case "water"
            try
                [~,~,epsc] = earthSurfacePermittivity("sea-water", freq, 15, 35);
            catch
                epsc = [];  % use radarvcd default sea model
            end
            [hgtsd, slope] = searoughness(3);
            vegType = 'None';
            
        otherwise
            [~,~,epsc] = earthSurfacePermittivity("vegetation", freq, 21, 0.3);
            [hgtsd, slope] = landroughness("Farm");
            vegType = 'Grass';
    end
end
