function dataLog = runDetections(scenario, enableDegradation, sensorMetas, envConfig, cfg)
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
%     - Terrain occlusion: LOS checks against terrain heightmap
%     - Horizon masking: 4/3 Earth model
%     - Ground clutter: terrain-dependent false returns
%     - Rain degradation: ITU-R P.838-3 frequency-dependent attenuation
%
% INPUTS
%   scenario          : trackingScenario from loadRunFile
%   enableDegradation : boolean. false=IDEAL, true=RAINY. Default false.
%   sensorMetas       : (optional) metas struct from loadRunFile
%   envConfig         : (optional) struct from config.environment

if nargin < 2; enableDegradation = false; end
if nargin < 3; sensorMetas = []; end
if nargin < 4 || isempty(envConfig)
    envConfig = struct('horizon_masking', true, 'refraction_factor', 4/3, ...
                       'ground_clutter', true, 'terrain_type', 'rural', ...
                       'clutter_density', 0.5);
end

% cfg (5th arg): full config struct for getWeather storm window
if nargin < 5 || isempty(cfg)
    cfg = struct('degradation', struct('storm_start_s', 5, 'storm_end_s', 45, ...
                                       'active_type', 'step'), ...
                 'scenario', struct('duration_s', 50));
end

% Parse environment config with safe defaults
enableHorizon  = getOrDefault(envConfig, 'horizon_masking', true);
refractionK    = getOrDefault(envConfig, 'refraction_factor', 4/3);
enableClutter  = getOrDefault(envConfig, 'ground_clutter', true);
terrainType    = getOrDefault(envConfig, 'terrain_type', 'rural');
clutterDensity = getOrDefault(envConfig, 'clutter_density', 0.5);

% Propagation model (VCP/PropFactor): REMOVED in v3.4.0
% VCP post-filtering was architecturally flawed — VCP files removed in v3.4.0 cleanup.


% Parse rain degradation config
rainConfig = struct('rain_rate_mmhr', 16);
weatherType = 'rain';  % default
if isfield(cfg, 'degradation') && isfield(cfg.degradation, 'type')
    weatherType = lower(char(cfg.degradation.type));
end
if isstruct(envConfig)
    if isfield(envConfig, 'rain_rate_mmhr');     rainConfig.rain_rate_mmhr = envConfig.rain_rate_mmhr; end
    if isfield(envConfig, 'pd_floor');           rainConfig.pd_floor = envConfig.pd_floor; end
    if isfield(envConfig, 'clutter_multiplier'); rainConfig.clutter_multiplier = envConfig.clutter_multiplier; end
end

% Detect terrain occlusion capability
% NOTE (v3.4.2): External RCS range filter RESTORED as OPT-IN. Empirically,
% fusionRadarSensor's native Swerling detection does not produce an
% observable RCS-vs-range differential at typical scan counts (20-100 scans)
% for the reference geometries we use — both high-RCS and low-RCS targets
% saturate near DetectionProbability. The external filter applies a
% deterministic R_eff cutoff (R_eff = refRange * (sigma/sigma_ref)^(1/4))
% that exposes the R^4 dependence cleanly. Enabled per-run via
% envConfig.rcs_range_filter (default OFF to preserve statistical sensor
% behavior for user demos; TC-05 validation turns it ON).

enableTerrainOcclusion = false;
if getOrDefault(envConfig, 'terrain_occlusion', true)
    try
        sm = scenario.SurfaceManager;
        if ~isempty(sm) && ~isempty(sm.Surfaces)
            enableTerrainOcclusion = sm.UseOcclusion;
        end
    catch; end
end

% Diagnostics
fprintf('[runDetections] Terrain occlusion: %s\n', ternary(enableTerrainOcclusion, 'ON (SurfaceManager LOS checks)', 'OFF (no terrain attached)'));
fprintf('[runDetections] Horizon masking: %s\n', ternary(enableHorizon, sprintf('ON (refraction=%.3f)', refractionK), 'OFF'));
fprintf('[runDetections] Ground clutter: %s\n', ternary(enableClutter, sprintf('ON (terrain=%s, density=%.2f)', terrainType, clutterDensity), 'OFF'));

fprintf('[runDetections] Rain degradation: %s\n', ternary(enableDegradation, sprintf('ON (%s, rate=%.0f mm/hr)', weatherType, rainConfig.rain_rate_mmhr), 'OFF'));
if enableDegradation
    stormType = 'step';
    if isfield(cfg.degradation, 'active_type'); stormType = cfg.degradation.active_type; end
    stormStart = 5; stormEnd = 45;
    if isfield(cfg.degradation, 'storm_start_s'); stormStart = cfg.degradation.storm_start_s; end
    if isfield(cfg.degradation, 'storm_end_s');   stormEnd   = cfg.degradation.storm_end_s; end
    fprintf('[runDetections] Storm window: %.0f–%.0fs (%s profile)\n', stormStart, stormEnd, stormType);
end
enableRCSFilter = getOrDefault(envConfig, 'rcs_range_filter', false);
fprintf('[runDetections] RCS range filter: %s\n', ternary(enableRCSFilter, 'ON (deterministic R_eff cutoff)', 'OFF (sensor-native Swerling)'));
fprintf('[runDetections] Doppler fade (MTI): %s\n', ternary(getOrDefault(envConfig, 'doppler_fade', true), 'ON (tangential targets fade in clutter notch)', 'OFF'));

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
        try info.scanMode = string(s.ScanMode); catch; info.scanMode = "unknown"; end

        if contains(lower(info.scanMode), 'mechanical') || contains(lower(info.scanMode), 'rotat')
            azSpan = 360;
            try azLim = s.MechanicalAzimuthLimits; azSpan = abs(diff(azLim)); catch; end
            try scanLim = s.MechanicalScanLimits; if size(scanLim,2)==2; azSpan = abs(diff(scanLim(1,:))); end; catch; end
            info.isRotator = (azSpan >= 350);
            info.isMechanical = true;
        elseif contains(lower(info.scanMode), 'sector')
            info.isMechanical = true;
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

        if sIdx == 1 || isempty(sensorInfos); sensorInfos = info; else; sensorInfos(sIdx) = info; end
    end
end

numSensors = numel(sensorInfos);
if numSensors == 0; error('runDetections:noSensors', 'No sensors found.'); end

%% Identify MSSR and sonar
mssrMask = [sensorInfos.isMSSR];
hasMSSR = any(mssrMask);
mssrSensorIdx = NaN;
if hasMSSR
    mssrInfo = sensorInfos(find(mssrMask, 1));
    mssrSensorIdx = mssrInfo.sensorIndex;
    fprintf('[runDetections] MSSR detected: SensorIndex=%d\n', mssrSensorIdx);
end

sonarMask = [sensorInfos.isSonar];
if any(sonarMask); fprintf('[runDetections] WARNING: %d sonar sensor(s) — skipping.\n', sum(sonarMask)); end

activeMask  = ~sonarMask;
activeInfos = sensorInfos(activeMask);
numActive   = numel(activeInfos);
fprintf('[runDetections] PSR count: %d | MSSR: %d\n', sum(~[activeInfos.isMSSR]), sum([activeInfos.isMSSR]));

%% Determine scan cadence
hasMechanical = any([activeInfos.isMechanical]);
hasRotator    = any([activeInfos.isRotator]);
masterIdx     = [];
scanInterval  = 5;

if hasMechanical
    mechIdxs = find([activeInfos.isMechanical]);
    slowestRate = Inf;
    masterIdx = mechIdxs(1);
    for ri = 1:numel(mechIdxs)
        try ur = activeInfos(mechIdxs(ri)).sensor.UpdateRate; catch; ur = Inf; end
        if ur < slowestRate; slowestRate = ur; masterIdx = mechIdxs(ri); end
    end
    fprintf('[runDetections] Scan master: sensor %d (%.1f Hz) — %s\n', ...
        activeInfos(masterIdx).sensorIndex, slowestRate, ternary(activeInfos(masterIdx).isRotator, 'rotator', 'sector'));
else
    rates = zeros(numActive, 1);
    for k = 1:numActive; try rates(k) = activeInfos(k).sensor.UpdateRate; catch; rates(k) = 1; end; end
    scanInterval = max(0.5, min(15.0, 1/max(min(rates(rates>0)), 0.1)));
    fprintf('[runDetections] No mechanical scanner — time-based flush every %.2fs\n', scanInterval);
end

%% Initialise
restart(scenario);
detBuffer = {}; mssrBuffer = {}; cfgBuffer = {};

% ── M5 §3.2 multi-target Truth log shape contract ─────────────────────
%  Downstream consumers (runTracker, analyzeTrackSwaps, plotters) treat
%  dataLog.Truth as a [nTargets × nScans] struct array — they index it
%  as `Truth(:, ss)` (all targets at scan ss) and `Truth(tgt, :)` (one
%  target's history). To honor that shape we must:
%    1. Append a COLUMN of length nTargets per scan (not a horizontal
%       slab — that would corrupt size(Truth,1) once N>1).
%    2. Hold the per-scan column at a constant nTargets even when a
%       target's trajectory ends and targetPoses() drops it. We do this
%       by snapshotting the initial pose set as a "stale" template and
%       padding missing rows with the last seen pose for that PlatformID.
%
%  Single-target M4 scenarios keep working: nTargets==1, target never
%  drops out, and the column-append degenerates back to a 1×nScans row.
truthInitPoses = targetPoses(activeInfos(1).platform);
nExpectedTargets = numel(truthInitPoses);
truthLastByPlatform = truthInitPoses(:);   % column of last-known poses

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
dataLog.TerrainGrid = ternary(~isempty(terrainGrid), terrainGrid, []);

%% Build sensor coverage metadata
coverage = [];
for k = 1:numActive
    si = activeInfos(k); s = si.sensor;
    cov = struct('sensorIndex',si.sensorIndex,'isRotator',si.isRotator,...
        'isMSSR',si.isMSSR,'isRadar',si.isRadar,'isIR',si.isIR);
    try cov.position = si.platform.InitialPosition(:)' + s.MountingLocation(:)'; catch; cov.position = [0 0 0]; end
    try cov.maxRange = s.RangeLimits(2); catch; cov.maxRange = 111120; end
    try cov.mountingYaw = s.MountingAngles(1); catch; cov.mountingYaw = 0; end
    try cov.fov = s.FieldOfView(:)'; catch; cov.fov = [1.4 30]; end
    try cov.elLimits = s.MechanicalElevationLimits(:)'; catch; cov.elLimits = [-17 13]; end
    try cc = coverageConfig(s); cov.scanElLimits = cc.ScanLimits(2,:); catch; cov.scanElLimits = cov.elLimits; end
    scanModeStr = ''; try scanModeStr = lower(string(s.ScanMode)); catch; end
    if contains(scanModeStr, 'no scanning') || contains(scanModeStr, 'electronic')
        try elecAz = s.ElectronicAzimuthLimits(:)'; catch; elecAz = [-45 45]; end
        cov.azLimits = cov.mountingYaw + elecAz;
    else
        try cov.azLimits = s.MechanicalAzimuthLimits(:)'; catch; cov.azLimits = [0 360]; end
    end
    cov.label = si.className;
    if ~isempty(sensorMetas) && isstruct(sensorMetas)
        pNames = fieldnames(sensorMetas);
        for pp = 1:numel(pNames)
            mList = sensorMetas.(pNames{pp});
            for mm = 1:numel(mList)
                if isfield(mList{mm},'sensorIndex') && mList{mm}.sensorIndex == si.sensorIndex
                    if isfield(mList{mm},'name'); cov.label = string(mList{mm}.name);
                    elseif isfield(mList{mm},'type'); cov.label = string(mList{mm}.type); end
                end
            end
        end
    end
    if isempty(coverage); coverage = cov; else; coverage(end+1) = cov; end %#ok<AGROW>
end
dataLog.SensorCoverage = coverage;

dataLog.VCPData = [];

s_rng = rng; rng(2018);
disp('Please wait. Generating detections for scenario .....')

lastFlushTime = -Inf;
terrainOcclusionCount = 0;
horizonMaskCount = 0;

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
            try sensorPos = sensorPos + si.sensor.MountingLocation(:)'; catch; end
            visible = true(numel(targets), 1);

            if enableTerrainOcclusion
                for tt = 1:numel(targets)
                    try [occ,~] = occlusion(scenario.SurfaceManager, sensorPos, targets(tt).Position(:)');
                        if occ; visible(tt) = false; end
                    catch; end
                end
                terrainOcclusionCount = terrainOcclusionCount + sum(~visible);
            end

            if enableHorizon
                stillVis = find(visible);
                if ~isempty(stillVis)
                    tgtPos = vertcat(targets(stillVis).Position);
                    aboveH = trackbench.environment.isAboveHorizon(sensorPos, tgtPos, refractionK);
                    blocked = stillVis(~aboveH);
                    visible(blocked) = false;
                    horizonMaskCount = horizonMaskCount + numel(blocked);
                end
            end
            if any(~visible); targets = targets(visible); end
        end

        % Step the sensor
        try [dets, ~, sensorCfg] = si.sensor(targets, ins, simTime); catch; continue; end
        dets = dets(:);

        if hasMechanical && k == masterIdx
            try if sensorCfg.IsScanDone; scanDone = true; end; catch; end
        end
        if isempty(dets); continue; end

        % Filter angle-only
        keepDets = true(numel(dets), 1);
        for ii = 1:numel(dets); if numel(dets{ii}.Measurement) < 3; keepDets(ii) = false; end; end
        if any(~keepDets); dets = dets(keepDets); if isempty(dets); continue; end; end

        % RCS range filter (opt-in via envConfig.rcs_range_filter).
        % When ON, applies deterministic R_eff = refRange*(sigma/sigma_ref)^(1/4)
        % cutoff. When OFF, relies on fusionRadarSensor's native Swerling model
        % which in practice is too subtle to expose RCS differentials at
        % typical scan counts. See note in header.
        if enableRCSFilter && si.isRadar && ~si.isMSSR && ~isempty(dets) && ~isempty(targets)
            sPosRCS = ins.Position(:)';
            try sPosRCS = sPosRCS + si.sensor.MountingLocation(:)'; catch; end
            [dets, ~] = trackbench.environment.applyRCSFilter( ...
                dets, sPosRCS, si.sensor, targets, allPlatforms);
            if isempty(dets); continue; end
        end

        % Doppler fade: targets with low radial velocity fall into the
        % MTI clutter notch and become undetectable. This is a fundamental
        % radar phenomenon — tangential targets are harder to see.
        % Ref: MathWorks MTI example, Skolnik Ch. 3
        enableDopplerFade = getOrDefault(envConfig, 'doppler_fade', true);
        if enableDopplerFade && si.isRadar && ~si.isMSSR && ~isempty(dets) && ~isempty(targets)
            sPosDop = ins.Position(:)';
            try sPosDop = sPosDop + si.sensor.MountingLocation(:)'; catch; end
            dopplerCfg = struct();
            if isfield(envConfig, 'mdv_ms'); dopplerCfg.mdv_ms = envConfig.mdv_ms; end
            [dets, ~] = trackbench.environment.applyDopplerFade( ...
                dets, sPosDop, targets, si, dopplerCfg);
        end

        % Classify and buffer
        if si.isMSSR
            for ii = 1:numel(dets)
                tgtIdx = getTargetIndex(dets{ii});
                if tgtIdx > 0; dets{ii}.ObjectClassID = tgtIdx + si.platformIdx; end
            end
            mssrBuffer = [mssrBuffer; dets]; %#ok<AGROW>
        else
            % ── Rain degradation: Pd drop + noise inflation (per detection) ──
            % Weather clutter is generated ONCE per scan at flush time below.
            % Severity w ∈ [0,1] from getWeather scales the rain rate.
            if enableDegradation && (si.isRadar || si.isIR) && ~si.isMSSR
                % Compute weather severity at this time step
                w = computeWeatherSeverity(simTime, cfg);
                if w > 0
                sPos_rain = ins.Position(:)';
                try sPos_rain = sPos_rain + si.sensor.MountingLocation(:)'; catch; end

                sParams_rain = struct();
                try sParams_rain.rangeLimits = si.sensor.RangeLimits; catch; sParams_rain.rangeLimits = [0 111120]; end
                try sParams_rain.fov = si.sensor.FieldOfView(:)'; catch; sParams_rain.fov = [1.4 30]; end

                % Scale rain rate by weather severity
                scaledRainConfig = rainConfig;
                scaledRainConfig.rain_rate_mmhr = w * rainConfig.rain_rate_mmhr;

                % Get Pd multiplier (function handle) and noise multiplier (scalar)
                % Third output (weather clutter) is IGNORED here — generated at flush time
                [pdMult, noiseMult, ~] = trackbench.environment.applyWeatherDegradation( ...
                    simTime, scaledRainConfig, si, sPos_rain, sParams_rain, weatherType);

                % Drop detections using range-dependent Pd
                if ~isempty(dets)
                    keepRain = true(numel(dets), 1);
                    for dd = 1:numel(dets)
                        detPos = dets{dd}.Measurement(1:3);
                        slantRange = norm(detPos(:) - sPos_rain(:));
                        if rand() > pdMult(slantRange)
                            keepRain(dd) = false;
                        end
                    end
                    dets = dets(keepRain);
                end

                % Scale measurement noise
                for dd = 1:numel(dets)
                    dets{dd}.MeasurementNoise = dets{dd}.MeasurementNoise * noiseMult;
                end
                end  % if w > 0
            end

            detBuffer = [detBuffer; dets]; %#ok<AGROW>
        end
        cfgBuffer{end+1} = sensorCfg; %#ok<AGROW>
    end

    % ---- Scan complete check ----
    if ~hasMechanical && (simTime - lastFlushTime) >= scanInterval
        scanDone = true;
    end

    % ---- Flush scan buffer ----
    if scanDone
        if isempty(detBuffer) && isempty(mssrBuffer); lastFlushTime = simTime; continue; end

        if ~isempty(detBuffer)
            times = cellfun(@(d) d.Time, detBuffer);
            fprintf("Scan buffer time span (pre-snap) = %.6f s\n", max(times) - min(times));
        end

        nFalse = 0;

        % Find primary radar for clutter generation
        clutterSidx = 1;
        clutterSensorInfo = [];
        for kk = 1:numActive
            if activeInfos(kk).isRadar && ~activeInfos(kk).isMSSR
                clutterSidx = activeInfos(kk).sensorIndex;
                clutterSensorInfo = activeInfos(kk);
                break;
            end
        end

        % Ground clutter (terrain-dependent)
        if enableClutter && ~isempty(clutterSensorInfo)
            sPos_gc = pose(clutterSensorInfo.platform, 'true').Position(:)';
            try sPos_gc = sPos_gc + clutterSensorInfo.sensor.MountingLocation(:)'; catch; end
            cSensor = clutterSensorInfo.sensor;
            sParams = struct();
            try sParams.rangeLimits = cSensor.RangeLimits;   catch; sParams.rangeLimits = [0 111120]; end
            try sParams.rangeRes    = cSensor.RangeResolution; catch; sParams.rangeRes = 93; end
            try sParams.fov         = cSensor.FieldOfView(:)'; catch; sParams.fov = [1.4 30]; end
            try sParams.tilt        = cSensor.ElectronicScanAngle(2); catch; sParams.tilt = 2; end
            try sParams.mountingLoc = cSensor.MountingLocation(:)'; catch; sParams.mountingLoc = [0 0 -15]; end
            clutterEnv = struct('terrain_type', terrainType, 'clutter_density', clutterDensity, ...
                'radar_freq', clutterSensorInfo.radarFreq);
            clutterDets = trackbench.environment.generateGroundClutter(simTime, sPos_gc, clutterSidx, sParams, clutterEnv);
            if ~isempty(clutterDets)
                detBuffer = [detBuffer; clutterDets]; %#ok<AGROW>
                nFalse = nFalse + numel(clutterDets);
            end
        end

        % Weather clutter (rain-driven, ONCE per scan at flush time)
        % getWeather computes severity w and logs to dataLog.WeatherSeverity
        if enableDegradation && ~isempty(clutterSensorInfo)
            [wScan, ~, dataLog] = trackbench.detections.getWeather(simTime, cfg, dataLog);
            if wScan > 0
            sPos_wc = pose(clutterSensorInfo.platform, 'true').Position(:)';
            try sPos_wc = sPos_wc + clutterSensorInfo.sensor.MountingLocation(:)'; catch; end
            sParams_wc = struct();
            try sParams_wc.rangeLimits = clutterSensorInfo.sensor.RangeLimits; catch; sParams_wc.rangeLimits = [0 111120]; end
            try sParams_wc.fov = clutterSensorInfo.sensor.FieldOfView(:)'; catch; sParams_wc.fov = [1.4 30]; end
            scaledRainCfg = rainConfig;
            scaledRainCfg.rain_rate_mmhr = wScan * rainConfig.rain_rate_mmhr;
            [~, ~, weatherDets] = trackbench.environment.applyWeatherDegradation( ...
                simTime, scaledRainCfg, clutterSensorInfo, sPos_wc, sParams_wc, weatherType);
            if ~isempty(weatherDets)
                detBuffer = [detBuffer; weatherDets]; %#ok<AGROW>
                nFalse = nFalse + numel(weatherDets);
            end
            end  % if wScan > 0
        end

        % Snap timestamps
        for kk = 1:numel(detBuffer);  detBuffer{kk}.Time  = simTime; end
        for kk = 1:numel(mssrBuffer); mssrBuffer{kk}.Time = simTime; end

        % Get truth — targetPoses may return fewer targets than
        % nExpectedTargets if a trajectory has ended. Align the new scan
        % against the initial platform set by PlatformID and carry
        % forward the last-known pose for any missing target, so the
        % truth column stays at height nExpectedTargets across all scans.
        targetsRaw = targetPoses(activeInfos(1).platform);
        truthCol = truthLastByPlatform;   % start from last-known column
        if ~isempty(targetsRaw)
            for tg = 1:numel(targetsRaw)
                pid = targetsRaw(tg).PlatformID;
                slot = find([truthLastByPlatform.PlatformID] == pid, 1);
                if ~isempty(slot)
                    truthCol(slot) = targetsRaw(tg);
                end
            end
        end
        truthLastByPlatform = truthCol;   % cache for next scan's fallback
        targets = truthCol;               % keep name for downstream

        % Merge
        mergedDets = [mssrBuffer; detBuffer];
        if isempty(mergedDets); lastFlushTime = simTime; detBuffer = {}; mssrBuffer = {}; cfgBuffer = {}; continue; end

        fprintf("t=%.2f: PSR=%d, MSSR=%d, total=%d (clutter=%d)\n", ...
            simTime, numel(detBuffer), numel(mssrBuffer), numel(mergedDets), nFalse);

        % Log — append targets as a COLUMN so Truth grows as
        % [nExpectedTargets × nScans] (see §3.2 shape contract above).
        dataLog.Time       = [dataLog.Time, simTime];
        dataLog.Truth      = [dataLog.Truth, targets(:)];
        dataLog.Detections = [dataLog.Detections(:)', {mergedDets}];
        dataLog.SensorConfig = [dataLog.SensorConfig(:)', {cfgBuffer}];
        try scanSensors = unique(cellfun(@(d) d.SensorIndex, mergedDets)); catch; scanSensors = []; end
        dataLog.ScanSensorIndices = [dataLog.ScanSensorIndices(:)', {scanSensors}];
        dataLog.SensorPlatformIDs = [dataLog.SensorPlatformIDs, ternary(~isempty(scanSensors), scanSensors(1), 0)];

        detBuffer = {}; mssrBuffer = {}; cfgBuffer = {};
        lastFlushTime = simTime;
    end
end

rng(s_rng);
fprintf('Detections generation complete.');
if enableTerrainOcclusion; fprintf(' (Terrain occluded %d)', terrainOcclusionCount); end
if enableHorizon;          fprintf(' (Horizon masked %d)', horizonMaskCount); end
fprintf('\n');

end  % end main function


%% ====================================================================
%  LOCAL HELPER FUNCTIONS
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
                        if contains(typeStr, 'SSR') || contains(typeStr, 'MSSR') || contains(typeStr, 'IFF')
                            isMSSR = true; return;
                        end
                    end
                end
            end
        end
    end
    if isempty(metas)
        try
            if sensor.FalseAlarmRate <= 1e-7 && isprop(sensor, 'DetectionProbability') && ...
               sensor.DetectionProbability >= 0.98 && sensor.ReferenceRange > 150000 && sensor.ReferenceRange < 250000
                isMSSR = true;
            end
        catch; end
    end
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function tgtIdx = getTargetIndex(det)
    tgtIdx = 0;
    try attrs = det.ObjectAttributes;
        if iscell(attrs) && ~isempty(attrs)
            attr = attrs{1};
            if isstruct(attr) && isfield(attr, 'TargetIndex'); tgtIdx = attr.TargetIndex; end
        end
    catch; end
end

function val = getOrDefault(s, field, default)
    if isstruct(s) && isfield(s, field); val = s.(field); else; val = default; end
end

function w = computeWeatherSeverity(t, cfg)
%computeWeatherSeverity  Lightweight w computation (no logging).
%  Used in per-detection rain loop. Full getWeather is called at scan flush.
    d = cfg.degradation;
    storm_start = 5; storm_end = 45; active_type = 'step';
    if isfield(d, 'storm_start_s'); storm_start = d.storm_start_s; end
    if isfield(d, 'storm_end_s');   storm_end   = d.storm_end_s; end
    if isfield(d, 'active_type');   active_type = d.active_type; end
    switch lower(active_type)
        case 'step'
            w = double(t >= storm_start && t <= storm_end);
        case 'ramp'
            dur = storm_end - storm_start;
            if t < storm_start || t > storm_end; w = 0;
            elseif t <= storm_start + dur/2; w = (t - storm_start) / (dur/2);
            else; w = (storm_end - t) / (dur/2); end
        case 'pulse'
            w = double(t >= storm_start && t <= storm_start + 0.2*(storm_end - storm_start));
        otherwise
            w = double(t >= storm_start && t <= storm_end);
    end
    w = max(0, min(1, w));
end
