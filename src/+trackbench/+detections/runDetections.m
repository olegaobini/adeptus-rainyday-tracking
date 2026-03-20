function dataLog = runDetections(scenario, enableDegradation, sensorMetas, envConfig)
%runDetections  Run scenario and generate a detection log for tracking.
%
% GENERALISED MULTI-SENSOR DETECTION GENERATOR
%   Supports any combination of radar, IR, and sonar sensors across
%   multiple platforms. Handles rotating sensors (IsScanDone flush),
%   sector/no-scanning sensors (time-based flush), and mixed configs.
%
%   MSSR/IFF identification is based on sensor metadata type tag
%   (SSR/MSSR) from loadSensors, not on FAR threshold.
%
%   ENVIRONMENT MODELLING (when envConfig is provided):
%     - Terrain occlusion: if the scenario has a SurfaceManager (from
%       groundSurface/generateTerrain), LOS checks against the terrain
%       heightmap block targets hidden behind ridges, buildings, etc.
%     - Horizon masking: targets below the radar horizon (4/3 Earth model)
%       are removed. Supplements terrain occlusion with Earth curvature.
%     - Ground clutter: terrain-dependent false returns concentrated at low
%       elevation angles replace uniform false alarm scattering.
%
% INPUTS
%   scenario          : trackingScenario from loadScenario
%   enableDegradation : boolean. false=IDEAL, true=RAINY. Default false.
%   sensorMetas       : (optional) metas struct from loadSensors
%   envConfig         : (optional) struct from config.environment
%
% OUTPUT
%   dataLog struct with Time, Truth, Detections, SensorConfig, etc.
%
% CHANGE LOG
%   - Fixed weatherSeverity: now returns w=1.0 for constant rain effect
%     throughout the entire scenario (was hardcoded to t=15-30s only).
%   - Fixed radar Pd degradation: w=1 now gives Pd=0.50 (was 0.70).

if nargin < 2
    enableDegradation = false;
end
if nargin < 3
    sensorMetas = [];
end
if nargin < 4 || isempty(envConfig)
    envConfig = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                       'ground_clutter', true, 'terrain_type', 'rural', ...
                       'clutter_density', 0.5);
end

% Parse environment config with safe defaults
enableHorizon  = getOrDefault(envConfig, 'horizon_masking', true);
refractionK    = getOrDefault(envConfig, 'refraction_factor', 4/3);
enableClutter  = getOrDefault(envConfig, 'ground_clutter', true);
terrainType    = getOrDefault(envConfig, 'terrain_type', 'rural');
clutterDensity = getOrDefault(envConfig, 'clutter_density', 0.5);
enablePropModel = getOrDefault(envConfig, 'propagation_model', true);

% Detect terrain occlusion capability (SurfaceManager from loadScenario)
enableTerrainOcclusion = false;
hasSurfaceManager = false;
if getOrDefault(envConfig, 'terrain_occlusion', true)
    try
        sm = scenario.SurfaceManager;
        if ~isempty(sm) && ~isempty(sm.Surfaces)
            hasSurfaceManager = true;
            enableTerrainOcclusion = sm.UseOcclusion;
        end
    catch
    end
end

if enableTerrainOcclusion
    fprintf('[runDetections] Terrain occlusion: ON (SurfaceManager LOS checks)\n');
else
    fprintf('[runDetections] Terrain occlusion: OFF (no terrain attached)\n');
end

if enableHorizon
    fprintf('[runDetections] Horizon masking: ON (refraction=%.3f)\n', refractionK);
else
    fprintf('[runDetections] Horizon masking: OFF\n');
end
if enableClutter
    fprintf('[runDetections] Ground clutter: ON (terrain=%s, density=%.2f)\n', terrainType, clutterDensity);
else
    fprintf('[runDetections] Ground clutter: OFF\n');
end
if enablePropModel
    fprintf('[runDetections] Propagation model: ON (multipath lobing, terrain=%s)\n', terrainType);
else
    fprintf('[runDetections] Propagation model: OFF (free-space assumption)\n');
end

%% Discover all sensors across all platforms
allPlatforms = scenario.Platforms;
numPlats     = numel(allPlatforms);

sensorInfos = [];
sIdx = 0;

for pIdx = 1:numPlats
    plat = allPlatforms{pIdx};
    if isempty(plat.Sensors); continue; end
    
    for k = 1:numel(plat.Sensors)
        s = plat.Sensors{k};
        sIdx = sIdx + 1;
        
        info = struct();
        info.sensor       = s;
        info.sensorIndex  = s.SensorIndex;
        info.platformIdx  = pIdx;
        info.platform     = plat;
        info.className    = class(s);
        info.isMSSR       = false;
        info.isSonar      = contains(info.className, 'sonar', 'IgnoreCase', true);
        info.isIR         = contains(info.className, 'irSensor', 'IgnoreCase', true);
        info.isRadar      = contains(info.className, 'radar', 'IgnoreCase', true) || ...
                            contains(info.className, 'fusionRadar', 'IgnoreCase', true);
        info.isRotator    = false;
        info.isMechanical = false;
        info.scanMode     = '';
        
        try
            info.scanMode = string(s.ScanMode);
        catch
            info.scanMode = "unknown";
        end
        
        if contains(lower(info.scanMode), 'mechanical') || ...
           contains(lower(info.scanMode), 'rotat')
            azSpan = 360;
            try
                if isprop(s, 'MechanicalAzimuthLimits')
                    azLim = s.MechanicalAzimuthLimits;
                    azSpan = abs(diff(azLim));
                end
            catch
            end
            try
                if isprop(s, 'MechanicalScanLimits')
                    scanLim = s.MechanicalScanLimits;
                    if size(scanLim, 2) == 2
                        azSpan = abs(diff(scanLim(1,:)));
                    end
                end
            catch
            end
            
            info.isRotator = (azSpan >= 350);
            info.isMechanical = true;
        elseif contains(lower(info.scanMode), 'sector')
            info.isRotator    = false;
            info.isMechanical = true;
        elseif contains(lower(info.scanMode), 'no scanning') || ...
               contains(lower(info.scanMode), 'electronic')
            info.isRotator    = false;
            info.isMechanical = false;
        else
            info.isRotator    = false;
            info.isMechanical = false;
        end
        
        fprintf('[runDetections]   Sensor %d: %s | ScanMode=%s | isRotator=%d | isMechanical=%d\n', ...
            info.sensorIndex, info.className, info.scanMode, info.isRotator, info.isMechanical);
        
        info.isMSSR = classifyAsMSSR(s, sensorMetas);
        
        info.radarFreq = 2.8e9;
        if ~isempty(sensorMetas) && isstruct(sensorMetas)
            pNames = fieldnames(sensorMetas);
            for pp = 1:numel(pNames)
                mList = sensorMetas.(pNames{pp});
                for mm = 1:numel(mList)
                    if isfield(mList{mm}, 'sensorIndex') && mList{mm}.sensorIndex == s.SensorIndex
                        if isfield(mList{mm}, 'frequency')
                            info.radarFreq = mList{mm}.frequency;
                        elseif isfield(mList{mm}, 'type')
                            typeStr = upper(string(mList{mm}.type));
                            if contains(typeStr, 'PAR') || contains(typeStr, 'FIRE')
                                info.radarFreq = 9.0e9;
                            elseif contains(typeStr, 'SSR') || contains(typeStr, 'MSSR')
                                info.radarFreq = 1.03e9;
                            end
                        end
                    end
                end
            end
        end
        
        if sIdx == 1 || isempty(sensorInfos)
            sensorInfos = info;
        else
            sensorInfos(sIdx) = info;
        end
    end
end

numSensors = numel(sensorInfos);
if numSensors == 0
    error('runDetections:noSensors', 'No sensors found on any platform.');
end

%% Identify MSSR sensor(s)
mssrMask      = [sensorInfos.isMSSR];
hasMSSR       = any(mssrMask);
mssrSensorIdx = NaN;
if hasMSSR
    mssrInfo = sensorInfos(find(mssrMask, 1));
    mssrSensorIdx = mssrInfo.sensorIndex;
    fprintf('[runDetections] MSSR detected: SensorIndex=%d | Range=%.0f nm\n', ...
        mssrSensorIdx, mssrInfo.sensor.RangeLimits(2)/1852);
end

%% Identify sonar sensors (skip with warning)
sonarMask = [sensorInfos.isSonar];
if any(sonarMask)
    nSonar = sum(sonarMask);
    fprintf('[runDetections] WARNING: %d sonar sensor(s) detected — skipping.\n', nSonar);
end

activeMask  = ~sonarMask;
activeInfos = sensorInfos(activeMask);
numActive   = numel(activeInfos);

nonMSSR_mask = ~[activeInfos.isMSSR];
mssr_mask    = [activeInfos.isMSSR];

fprintf('[runDetections] PSR count: %d | MSSR: %d\n', sum(nonMSSR_mask), sum(mssr_mask));

%% Determine scan cadence
hasMechanical = any([activeInfos.isMechanical]);
hasRotator    = any([activeInfos.isRotator]);
masterIdx     = [];

if hasMechanical
    mechIdxs = find([activeInfos.isMechanical]);
    slowestRate = Inf;
    masterIdx = mechIdxs(1);
    for ri = 1:numel(mechIdxs)
        rIdx = mechIdxs(ri);
        try
            ur = activeInfos(rIdx).sensor.UpdateRate;
        catch
            ur = Inf;
        end
        if ur < slowestRate
            slowestRate = ur;
            masterIdx = rIdx;
        end
    end
    masterInfo = activeInfos(masterIdx);
    scanType = ternary(masterInfo.isRotator, 'rotator', 'sector');
    fprintf('[runDetections] Scan master: sensor %d (%.1f Hz) — slowest mechanical (%s)\n', ...
        masterInfo.sensorIndex, slowestRate, scanType);
else
    rates = zeros(numActive, 1);
    for k = 1:numActive
        try rates(k) = activeInfos(k).sensor.UpdateRate; catch; rates(k) = 1; end
    end
    minRate = max(min(rates(rates > 0)), 0.1);
    scanInterval = 1/minRate;
    scanInterval = max(scanInterval, 0.5);
    scanInterval = min(scanInterval, 15.0);
    fprintf('[runDetections] No mechanical scanner — time-based flush every %.2fs\n', scanInterval);
end

%% Initialise
restart(scenario);

detBuffer  = {};
mssrBuffer = {};
cfgBuffer  = {};

dataLog.Time              = [];
dataLog.Truth             = [];
dataLog.Detections        = {};
dataLog.SensorConfig      = {};
dataLog.ScanSensorIndices = {};
dataLog.SensorPlatformIDs = [];
dataLog.HasIFF            = hasMSSR;
dataLog.IFFSensorIndex    = mssrSensorIdx;
dataLog.HasRotator        = hasRotator;
terrainGrid = getOrDefault(envConfig, 'terrainGrid', []);
if ~isempty(terrainGrid)
    dataLog.TerrainGrid = terrainGrid;
else
    dataLog.TerrainGrid = [];
end

%% Build sensor coverage metadata
coverage = [];
for k = 1:numActive
    si = activeInfos(k);
    s  = si.sensor;
    cov = struct();
    cov.sensorIndex = si.sensorIndex;
    cov.isRotator   = si.isRotator;
    cov.isMSSR      = si.isMSSR;
    cov.isRadar     = si.isRadar;
    cov.isIR        = si.isIR;
    
    platPos = [0 0 0];
    try platPos = si.platform.InitialPosition(:)'; catch; end
    mountLoc = [0 0 0];
    try mountLoc = s.MountingLocation(:)'; catch; end
    cov.position = platPos + mountLoc;
    
    cov.maxRange = 111120;
    try cov.maxRange = s.RangeLimits(2); catch; end
    
    cov.mountingYaw = 0;
    try cov.mountingYaw = s.MountingAngles(1); catch; end
    
    cov.fov = [1.4 30];
    try cov.fov = s.FieldOfView(:)'; catch; end
    
    scanModeStr = '';
    try scanModeStr = lower(string(s.ScanMode)); catch; end
    
    if contains(scanModeStr, 'no scanning') || contains(scanModeStr, 'electronic')
        elecAz = [-45 45];
        try elecAz = s.ElectronicAzimuthLimits(:)'; catch; end
        cov.azLimits = cov.mountingYaw + elecAz;
    else
        cov.azLimits = [0 360];
        try cov.azLimits = s.MechanicalAzimuthLimits(:)'; catch; end
    end
    
    cov.label = si.className;
    if ~isempty(sensorMetas) && isstruct(sensorMetas)
        pNames = fieldnames(sensorMetas);
        for pp = 1:numel(pNames)
            mList = sensorMetas.(pNames{pp});
            for mm = 1:numel(mList)
                if isfield(mList{mm}, 'sensorIndex') && mList{mm}.sensorIndex == si.sensorIndex
                    if isfield(mList{mm}, 'type')
                        cov.label = string(mList{mm}.type);
                    end
                    if isfield(mList{mm}, 'name')
                        cov.label = string(mList{mm}.name);
                    end
                end
            end
        end
    end
    
    if isempty(coverage)
        coverage = cov;
    else
        coverage(end+1) = cov; %#ok<AGROW>
    end
end
dataLog.SensorCoverage = coverage;

%% Pre-compute VCP (propagation model)
vcpData = [];
if enablePropModel
    try
        vcpData = trackbench.environment.computeVerticalCoverage( ...
            activeInfos, terrainType, refractionK);
        dataLog.VCPData = vcpData;
    catch ME
        warning('runDetections:vcpFailed', ...
            'VCP computation failed: %s. Propagation model disabled.', ME.message);
        enablePropModel = false;
    end
else
    dataLog.VCPData = [];
end

s_rng = rng;
rng(2018);
disp('Please wait. Generating detections for scenario .....')

lastFlushTime = -Inf;
terrainOcclusionCount = 0;
horizonMaskCount = 0;
vcpMaskCount    = 0;

%% Main loop
while advance(scenario)
    
    simTime = scenario.SimulationTime;
    
    scanDone = false;
    
    for k = 1:numActive
        si   = activeInfos(k);
        plat = si.platform;
        
        targets = targetPoses(plat);
        ins     = pose(plat, 'true');
        
        % ---- Visibility masking (terrain + horizon) ----
        if ~isempty(targets)
            sensorPos = ins.Position(:)';
            try
                mountLoc = si.sensor.MountingLocation(:)';
                sensorPos = sensorPos + mountLoc;
            catch
            end
            
            visible = true(numel(targets), 1);
            
            % Stage 1: Terrain occlusion
            if enableTerrainOcclusion
                for tt = 1:numel(targets)
                    tgtPos = targets(tt).Position(:)';
                    try
                        [occ, ~] = occlusion(scenario.SurfaceManager, sensorPos, tgtPos);
                        if occ
                            visible(tt) = false;
                        end
                    catch
                    end
                end
                nTerrainMasked = sum(~visible);
                if nTerrainMasked > 0
                    terrainOcclusionCount = terrainOcclusionCount + nTerrainMasked;
                end
            end
            
            % Stage 2: Horizon masking
            if enableHorizon
                stillVisible = find(visible);
                if ~isempty(stillVisible)
                    tgtPositions = vertcat(targets(stillVisible).Position);
                    aboveHorizon = trackbench.environment.isAboveHorizon(sensorPos, tgtPositions, refractionK);
                    horizonBlocked = stillVisible(~aboveHorizon);
                    visible(horizonBlocked) = false;
                    horizonMaskCount = horizonMaskCount + numel(horizonBlocked);
                end
            end
            
            if any(~visible)
                targets = targets(visible);
            end
        end
        
        % Step the sensor
        try
            [dets, ~, cfg] = si.sensor(targets, ins, simTime);
        catch
            continue;
        end
        dets = dets(:);
        
        % Check scan done
        if hasMechanical && k == masterIdx
            try
                if cfg.IsScanDone
                    scanDone = true;
                end
            catch
            end
        end
        
        if isempty(dets); continue; end
        
        % Filter angle-only detections
        keepDets = true(numel(dets), 1);
        for ii = 1:numel(dets)
            m = dets{ii}.Measurement(:);
            if numel(m) < 3
                keepDets(ii) = false;
            end
        end
        if any(~keepDets)
            dets = dets(keepDets);
            if isempty(dets); continue; end
        end
        
        % VCP masking
        if enablePropModel && ~si.isMSSR && ~isempty(vcpData)
            sensorPos = ins.Position(:)';
            try sensorPos = sensorPos + si.sensor.MountingLocation(:)'; catch; end
            
            vcpMask = trackbench.environment.applyVCPMask(dets, sensorPos, vcpData(k));
            nMaskedVCP = sum(~vcpMask);
            if nMaskedVCP > 0
                vcpMaskCount = vcpMaskCount + nMaskedVCP;
                dets = dets(vcpMask);
                if isempty(dets); continue; end
            end
        end
        
        % Classify and buffer
        if si.isMSSR
            for ii = 1:numel(dets)
                tgtIdx = getTargetIndex(dets{ii});
                if tgtIdx > 0
                    dets{ii}.ObjectClassID = tgtIdx + si.platformIdx;
                end
            end
            mssrBuffer = [mssrBuffer; dets]; %#ok<AGROW>
        else
            % Weather degradation on non-MSSR sensors only
            if enableDegradation && (si.isRadar || si.isIR)
                w      = weatherSeverity(simTime);
                % IR sensors less affected by rain than radar
                if si.isIR
                    pdEff = (1-w)*0.9 + w*0.85;  % mild degradation
                    Rmult = 1 + 0.5*w;
                else
                    pdEff = (1-w)*0.95 + w*0.50;  % full rain -> Pd=0.50
                    Rmult = 1 + 2*w;
                end
                if ~isempty(dets)
                    keep  = rand(numel(dets), 1) < pdEff;
                    dets  = dets(keep);
                end
                for ii = 1:numel(dets)
                    dets{ii}.MeasurementNoise = dets{ii}.MeasurementNoise * Rmult;
                end
            end
            
            detBuffer = [detBuffer; dets]; %#ok<AGROW>
        end
        
        cfgBuffer{end+1} = cfg; %#ok<AGROW>
    end
    
    % ---- Scan complete check ----
    if hasMechanical
        % (scanDone already set above)
    else
        if (simTime - lastFlushTime) >= scanInterval
            scanDone = true;
        end
    end
    
    % ---- Flush scan buffer ----
    if scanDone
        if isempty(detBuffer) && isempty(mssrBuffer)
            lastFlushTime = simTime;
            continue;
        end
        
        if ~isempty(detBuffer)
            times = cellfun(@(d) d.Time, detBuffer);
            fprintf("Scan buffer time span (pre-snap) = %.6f s\n", ...
                max(times) - min(times));
        end
        
        nFalse = 0;
        
        % Find primary radar for clutter
        clutterSidx = 1;
        clutterSensorInfo = [];
        for kk = 1:numActive
            if activeInfos(kk).isRadar && ~activeInfos(kk).isMSSR
                clutterSidx = activeInfos(kk).sensorIndex;
                clutterSensorInfo = activeInfos(kk);
                break;
            end
        end
        
        % Ground clutter
        if enableClutter && ~isempty(clutterSensorInfo)
            sensorPlat = clutterSensorInfo.platform;
            sensorIns  = pose(sensorPlat, 'true');
            sPos = sensorIns.Position(:)';
            try
                sPos = sPos + clutterSensorInfo.sensor.MountingLocation(:)';
            catch
            end
            
            cSensor = clutterSensorInfo.sensor;
            sParams = struct();
            try sParams.rangeLimits = cSensor.RangeLimits;   catch; sParams.rangeLimits = [0 111120]; end
            try sParams.rangeRes    = cSensor.RangeResolution; catch; sParams.rangeRes = 93; end
            try sParams.fov         = cSensor.FieldOfView(:)'; catch; sParams.fov = [1.4 30]; end
            try sParams.tilt        = cSensor.ElectronicScanAngle(2); catch; sParams.tilt = 2; end
            try sParams.mountingLoc = cSensor.MountingLocation(:)'; catch; sParams.mountingLoc = [0 0 -15]; end
            
            clutterEnv = struct('terrain_type', terrainType, 'clutter_density', clutterDensity);
            clutterDets = trackbench.environment.generateGroundClutter( ...
                simTime, sPos, clutterSidx, sParams, clutterEnv);
            
            if ~isempty(clutterDets)
                detBuffer = [detBuffer; clutterDets]; %#ok<AGROW>
                nFalse = nFalse + numel(clutterDets);
            end
        end
        
        % Weather clutter (rain-driven false alarms)
        if enableDegradation
            wScan  = weatherSeverity(simTime);
            if wScan > 0
                weatherLambda = wScan * 3.0;
                nWeather = poissrnd(weatherLambda);
                clutterSigma = 150;
                Rclutter = eye(3) * clutterSigma^2;
                
                for ii = 1:nWeather
                    meas = falseMeasInSurveillanceVolume();
                    detBuffer{end+1,1} = objectDetection(simTime, meas, ...
                        'MeasurementNoise', Rclutter, ...
                        'SensorIndex', clutterSidx); %#ok<AGROW>
                end
                nFalse = nFalse + nWeather;
            end
        end
        
        % Snap timestamps
        for kk = 1:numel(detBuffer)
            detBuffer{kk}.Time = simTime;
        end
        for kk = 1:numel(mssrBuffer)
            mssrBuffer{kk}.Time = simTime;
        end
        
        % Get truth
        firstPlat = activeInfos(1).platform;
        targets = targetPoses(firstPlat);
        
        % Merge
        mergedDets = [mssrBuffer; detBuffer];
        
        if isempty(mergedDets)
            lastFlushTime = simTime;
            detBuffer  = {};
            mssrBuffer = {};
            cfgBuffer  = {};
            continue;
        end
        
        fprintf("t=%.2f: PSR=%d, MSSR=%d, total=%d (clutter=%d)\n", ...
            simTime, numel(detBuffer), numel(mssrBuffer), numel(mergedDets), nFalse);
        
        % Log
        dataLog.Time       = [dataLog.Time, simTime];
        dataLog.Truth      = [dataLog.Truth, targets];
        dataLog.Detections = [dataLog.Detections(:)', {mergedDets}];
        dataLog.SensorConfig = [dataLog.SensorConfig(:)', {cfgBuffer}];
        
        if isempty(mergedDets)
            scanSensors = [];
        else
            try
                scanSensors = unique(cellfun(@(d) d.SensorIndex, mergedDets));
            catch
                scanSensors = [];
            end
        end
        dataLog.ScanSensorIndices = [dataLog.ScanSensorIndices(:)', {scanSensors}];
        dataLog.SensorPlatformIDs = [dataLog.SensorPlatformIDs, ...
            ternary(~isempty(scanSensors), scanSensors(1), 0)];
        
        % Reset buffers
        detBuffer  = {};
        mssrBuffer = {};
        cfgBuffer  = {};
        lastFlushTime = simTime;
    end
end

rng(s_rng);
fprintf('Detections generation complete.');
if enableTerrainOcclusion
    fprintf(' (Terrain occluded %d target-sensor pairs)', terrainOcclusionCount);
end
if enableHorizon
    fprintf(' (Horizon masked %d target-sensor pairs)', horizonMaskCount);
end
if enablePropModel
    fprintf(' (VCP propagation masked %d detections)', vcpMaskCount);
end
fprintf('\n');

end  % end main function


%% ====================================================================
%  MSSR CLASSIFICATION
% =====================================================================
function isMSSR = classifyAsMSSR(sensor, metas)
    isMSSR = false;
    
    if ~isempty(metas) && isstruct(metas)
        platformNames = fieldnames(metas);
        for p = 1:numel(platformNames)
            metaList = metas.(platformNames{p});
            for m = 1:numel(metaList)
                meta = metaList{m};
                if isfield(meta, 'sensorIndex') && meta.sensorIndex == sensor.SensorIndex
                    if isfield(meta, 'type')
                        typeStr = upper(string(meta.type));
                        if contains(typeStr, 'SSR') || contains(typeStr, 'MSSR') || ...
                           contains(typeStr, 'IFF')
                            isMSSR = true;
                            return;
                        end
                    end
                end
            end
        end
    end
    
    if isempty(metas)
        try
            if sensor.FalseAlarmRate <= 1e-7
                if isprop(sensor, 'DetectionProbability') && sensor.DetectionProbability >= 0.98 && ...
                   isprop(sensor, 'ReferenceRange') && sensor.ReferenceRange > 150000 && ...
                   isprop(sensor, 'ReferenceRange') && sensor.ReferenceRange < 250000
                    isMSSR = true;
                end
            end
        catch
        end
    end
end


%% ====================================================================
%  HELPER FUNCTIONS
% =====================================================================
function w = weatherSeverity(~)
%weatherSeverity  Returns weather degradation severity (0-1).
%  When degradation is enabled, rain is constant throughout the scenario.
%  This function is only called when enableDegradation=true, so w=1.0
%  means "full rain effect" for the entire run.
    w = 1.0;
end

function meas = falseMeasInSurveillanceVolume()
    x = (-1.5e3) + (3.0e3)*rand;
    y = (-20.5e3) + (2.0e3)*rand;
    z = -3e3 + 700*randn;
    meas = [x; y; z];
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function tgtIdx = getTargetIndex(det)
    tgtIdx = 0;
    try
        attrs = det.ObjectAttributes;
        if iscell(attrs) && ~isempty(attrs)
            attr = attrs{1};
            if isstruct(attr) && isfield(attr, 'TargetIndex')
                tgtIdx = attr.TargetIndex;
            end
        end
    catch
    end
end

function val = getOrDefault(s, field, default)
    if isstruct(s) && isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end