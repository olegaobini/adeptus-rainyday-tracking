function dataLog = runDetections(scenario, enableDegradation, sensorMetas)
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
% INPUTS
%   scenario          : trackingScenario from loadScenario
%   enableDegradation : boolean. false=IDEAL, true=RAINY. Default false.
%   sensorMetas       : (optional) metas struct from loadSensors — used to
%                       identify MSSR sensors by type. If omitted, falls
%                       back to FAR-based classification.
%
% OUTPUT
%   dataLog struct:
%     Time              : 1xN scan times (s)
%     Truth             : NtgtxN platformPose arrays
%     Detections        : 1xN cell of merged objectDetection arrays
%     SensorConfig      : 1xN cell of config structs
%     ScanSensorIndices : 1xN cell of contributing sensor indices per scan
%     SensorPlatformIDs : 1xN platform ID scalars
%     HasIFF            : scalar logical
%     IFFSensorIndex    : MSSR sensor index (or NaN)

if nargin < 2
    enableDegradation = false;
end
if nargin < 3
    sensorMetas = [];
end

%% ====================================================================
%  DISCOVER ALL SENSORS ACROSS ALL PLATFORMS
% =====================================================================
% Identify which platforms have sensors (vs targets which have none).
allPlatforms = scenario.Platforms;
numPlats     = numel(allPlatforms);

% Collect every sensor, its host platform index, and classify it
sensorInfos = [];  % struct array
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
        info.scanMode     = '';
        
        % Determine scan mode — distinguish true 360° rotators from sector/no-scan
        %
        % fusionRadarSensor: ScanMode is 'Mechanical' for BOTH rotators and sector,
        %   'Electronic' for some, 'No scanning' for staring. True rotators have
        %   MechanicalAzimuthLimits spanning 360° (or no limits set, defaulting to 360).
        %   Sector sensors have limits < 360°.
        %
        % irSensor: Uses MaxMechanicalScanRate / MechanicalScanLimits.
        %   Rotators scan full 360°.
        try
            info.scanMode = string(s.ScanMode);
        catch
            info.scanMode = "unknown";
        end
        
        % Check if it's a true 360° rotator
        if contains(lower(info.scanMode), 'mechanical') || ...
           contains(lower(info.scanMode), 'rotat')
            % Could be rotator OR sector — check azimuth limits
            azSpan = 360;  % default: full rotation
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
            
            % True rotator = 360° (or close to it)
            info.isRotator = (azSpan >= 350);
        elseif contains(lower(info.scanMode), 'no scanning') || ...
               contains(lower(info.scanMode), 'electronic')
            info.isRotator = false;
        else
            info.isRotator = false;
        end
        
        fprintf('[runDetections]   Sensor %d: %s | ScanMode=%s | isRotator=%d\n', ...
            info.sensorIndex, info.className, info.scanMode, info.isRotator);
        
        % Classify MSSR — first try metadata, then fall back to FAR
        info.isMSSR = classifyAsMSSR(s, sensorMetas);
        
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
    fprintf('[runDetections] WARNING: %d sonar sensor(s) detected — sonar uses a different\n', nSonar);
    fprintf('  step() interface (sonarEmission). Skipping sonar sensors for now.\n');
end

% Active sensors = everything except sonar
activeMask  = ~sonarMask;
activeInfos = sensorInfos(activeMask);
numActive   = numel(activeInfos);

nonMSSR_mask = ~[activeInfos.isMSSR];
mssr_mask    = [activeInfos.isMSSR];

fprintf('[runDetections] PSR count: %d | MSSR: %d\n', sum(nonMSSR_mask), sum(mssr_mask));

%% Determine scan cadence
%  For rotating sensors: use IsScanDone from the first rotator (master clock)
%  For non-rotating only: use time-based flush at fixed intervals
hasRotator  = any([activeInfos.isRotator]);
masterIdx   = [];  % index into activeInfos for the scan master

if hasRotator
    % Use SLOWEST rotator as scan master — fast rotators (IRST) trigger
    % IsScanDone too frequently if used as master, causing empty flushes
    rotatorIdxs = find([activeInfos.isRotator]);
    slowestRate = Inf;
    masterIdx = rotatorIdxs(1);
    for ri = 1:numel(rotatorIdxs)
        rIdx = rotatorIdxs(ri);
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
    fprintf('[runDetections] Scan master: sensor %d (%.1f Hz) — slowest rotator\n', ...
        activeInfos(masterIdx).sensorIndex, slowestRate);
else
    % No rotating sensor — use time-based scan flush
    % Use the SLOWEST sensor's period as the scan interval,
    % but clamp to [1s, 15s] to avoid overly frequent or slow flushes.
    rates = zeros(numActive, 1);
    for k = 1:numActive
        try rates(k) = activeInfos(k).sensor.UpdateRate; catch; rates(k) = 1; end
    end
    minRate = max(min(rates(rates > 0)), 0.1);
    scanInterval = 1/minRate;
    scanInterval = max(scanInterval, 1.0);   % minimum 1s between flushes
    scanInterval = min(scanInterval, 15.0);  % maximum 15s
    fprintf('[runDetections] No rotating sensor — time-based flush every %.2fs\n', scanInterval);
end

%% Initialise
restart(scenario);

detBuffer  = {};   % all non-MSSR detections buffered per scan
mssrBuffer = {};   % MSSR detections buffered per scan
cfgBuffer  = {};   % sensor configs per scan

dataLog.Time              = [];
dataLog.Truth             = [];
dataLog.Detections        = {};
dataLog.SensorConfig      = {};
dataLog.ScanSensorIndices = {};
dataLog.SensorPlatformIDs = [];
dataLog.HasIFF            = hasMSSR;
dataLog.IFFSensorIndex    = mssrSensorIdx;
dataLog.HasRotator        = hasRotator;

s_rng = rng;
rng(2018);
disp('Please wait. Generating detections for scenario .....')

lastFlushTime = -Inf;

%% Main loop
while advance(scenario)
    
    simTime = scenario.SimulationTime;
    
    scanDone = false;
    
    % Step each active sensor
    for k = 1:numActive
        si   = activeInfos(k);
        plat = si.platform;
        
        targets = targetPoses(plat);
        ins     = pose(plat, 'true');
        
        % Step the sensor
        try
            [dets, ~, cfg] = si.sensor(targets, ins, simTime);
        catch
            % Some sensors may not be ready yet (e.g. first step)
            continue;
        end
        dets = dets(:);
        
        % Check scan done — ONLY from the master rotator
        if hasRotator && k == masterIdx
            try
                if cfg.IsScanDone
                    scanDone = true;
                end
            catch
            end
        end
        
        if isempty(dets); continue; end
        
        % Filter out angle-only detections (e.g. FLIR [az,el] with no range)
        % These can't initialize the tracker and cause errors when mixed
        % with Cartesian or az+range detections
        keepDets = true(numel(dets), 1);
        for ii = 1:numel(dets)
            m = dets{ii}.Measurement(:);
            if numel(m) < 3
                keepDets(ii) = false;  % angle-only, skip
            end
        end
        if any(~keepDets)
            dets = dets(keepDets);
            if isempty(dets); continue; end
        end
        
        % Classify and buffer
        if si.isMSSR
            % Tag MSSR detections with platform IDs (IFF)
            for ii = 1:numel(dets)
                tgtIdx = getTargetIndex(dets{ii});
                if tgtIdx > 0
                    % For multi-platform: target platform ID = sensor platform count + tgtIdx
                    % But in trackingScenario, targets get IDs after sensor platforms
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
                    pdEff = (1-w)*0.95 + w*0.70;
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
        
        % Store config from each sensor for FOV-aware detectable track IDs
        cfgBuffer{end+1} = cfg; %#ok<AGROW>
    end
    
    % ----------------------------------------------------------------
    %  Scan complete check
    % ----------------------------------------------------------------
    if hasRotator
        % Rotating sensor master clock
        % (scanDone already set above)
    else
        % Time-based flush
        if (simTime - lastFlushTime) >= scanInterval
            scanDone = true;
        end
    end
    
    % ----------------------------------------------------------------
    %  Flush scan buffer
    % ----------------------------------------------------------------
    if scanDone
        % Early skip if both buffers truly empty (no sensor produced anything)
        if isempty(detBuffer) && isempty(mssrBuffer)
            lastFlushTime = simTime;
            continue;
        end
        
        if ~isempty(detBuffer)
            times = cellfun(@(d) d.Time, detBuffer);
            fprintf("Scan buffer time span (pre-snap) = %.6f s\n", ...
                max(times) - min(times));
        end
        
        % False alarms (weather clutter on radar only)
        nFalse = 0;
        if enableDegradation
            wScan  = weatherSeverity(simTime);
            lambda = (1-wScan)*0.0 + wScan*3.0;
            nFalse = poissrnd(lambda);
            
            haveMP = ~isempty(detBuffer) && isprop(detBuffer{1}, 'MeasurementParameters');
            if haveMP; mp = detBuffer{1}.MeasurementParameters; end
            
            clutterSigma = 150*(wScan>0) + 100*(wScan==0);
            Rclutter = eye(3) * clutterSigma^2;
            
            % Find a radar sensor index for clutter attribution
            clutterSidx = 1;
            for kk = 1:numActive
                if activeInfos(kk).isRadar && ~activeInfos(kk).isMSSR
                    clutterSidx = activeInfos(kk).sensorIndex;
                    break;
                end
            end
            
            for ii = 1:nFalse
                meas = falseMeasInSurveillanceVolume();
                if haveMP
                    detBuffer{end+1,1} = objectDetection(simTime, meas, ...
                        'MeasurementNoise', Rclutter, ...
                        'SensorIndex', clutterSidx, ...
                        'MeasurementParameters', mp);
                else
                    detBuffer{end+1,1} = objectDetection(simTime, meas, ...
                        'MeasurementNoise', Rclutter, ...
                        'SensorIndex', clutterSidx);
                end
            end
        end
        
        % Snap timestamps
        for kk = 1:numel(detBuffer)
            detBuffer{kk}.Time = simTime;
        end
        for kk = 1:numel(mssrBuffer)
            mssrBuffer{kk}.Time = simTime;
        end
        
        % Get truth target positions for logging (always needed)
        firstPlat = activeInfos(1).platform;
        targets = targetPoses(firstPlat);
        
        % ROI gate on non-MSSR detections (dynamic bounding box)
        % ONLY apply for rotating-sensor (DASR) scenarios where false alarms
        % are scattered across the full scan volume. For non-rotating sensors
        % (phased array, sector scan, FLIR, etc.), the sensors' own FOV and
        % detection logic provide sufficient gating — an additional ROI gate
        % causes coordinate-frame mismatches and filters valid detections.
        if hasRotator && ~isempty(detBuffer)
            
            % Compute ROI padding from max sensor range
            maxRange = 0;
            for kk = 1:numActive
                try
                    rl = activeInfos(kk).sensor.RangeLimits;
                    maxRange = max(maxRange, rl(2));
                catch
                end
            end
            roiPad = max(6000, maxRange * 0.1);
            
            nPreROI = numel(detBuffer);
            detBuffer = gateDetectionsROI(detBuffer, targets, roiPad);
            nPostROI = numel(detBuffer);
            if nPreROI > 0 && nPostROI == 0
                fprintf('[ROI] Filtered ALL %d detections — check coordinate frames.\n', nPreROI);
            end
        end
        
        % Merge: MSSR first so identity tags are processed before anonymous returns
        mergedDets = [mssrBuffer; detBuffer];
        
        % Post-merge empty check — skip scans where ROI gate removed everything
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
        % Truth: use targets from first sensor platform
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
disp('Detections generation complete.')

end  % end main function


%% ====================================================================
%  MSSR CLASSIFICATION
% =====================================================================
function isMSSR = classifyAsMSSR(sensor, metas)
%classifyAsMSSR  Determine if sensor is MSSR/SSR/IFF.
%  Uses metadata type tag if available, otherwise falls back to FAR check.

    isMSSR = false;
    
    % Method 1: Check metadata
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
    
    % Method 2: Fallback — FAR-based (original DASR logic)
    % ONLY used when metas are not available (legacy createScenario path).
    % Very conservative: requires FAR=1e-7 AND looks like a transponder SSR
    % (high Pd, co-rotating with PSR, moderate range).
    if isempty(metas)
        try
            if sensor.FalseAlarmRate <= 1e-7
                % Must look like SSR: high Pd AND moderate range (not AESA/fire control)
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
function w = weatherSeverity(t)
    stormStart = 15;
    stormEnd   = 30;
    w = double(t >= stormStart && t <= stormEnd);
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

function detsOut = gateDetectionsROI(detsIn, targets, pad)
%gateDetectionsROI  Remove detections outside a region of interest.
    if isempty(detsIn); detsOut = detsIn; return; end
    if nargin < 3; pad = 6000; end
    
    if nargin >= 2 && ~isempty(targets)
        nTgt = numel(targets);
        allPos = zeros(nTgt, 3);
        for ii = 1:nTgt
            allPos(ii,:) = targets(ii).Position;
        end
        xMin = min(allPos(:,1)) - pad;
        xMax = max(allPos(:,1)) + pad;
        yMin = min(allPos(:,2)) - pad;
        yMax = max(allPos(:,2)) + pad;
        zMin = min(allPos(:,3)) - pad;
        zMax = max(allPos(:,3)) + pad;
        zMax = max(zMax, 500);
    else
        xMin = -8000;  xMax =  8000;
        yMin = -26000; yMax = -16000;
        zMin = -8000;  zMax =  500;
    end
    
    keep = false(numel(detsIn), 1);
    for ii = 1:numel(detsIn)
        z = detsIn{ii}.Measurement(:);
        if numel(z) < 3
            % Angle-only (az/el) or 2D measurement — cannot ROI gate, keep it
            keep(ii) = true;
            continue;
        end
        % Check if measurement looks like Cartesian (large values) vs spherical (small angles)
        % Spherical: [az_deg, el_deg, range_m] — az/el typically <360
        % Cartesian: [x, y, z] — typically >1000m for any target
        % If all values < 360, likely spherical — skip ROI gate
        if all(abs(z) < 360)
            keep(ii) = true;
            continue;
        end
        keep(ii) = (z(1)>=xMin && z(1)<=xMax) && ...
                   (z(2)>=yMin && z(2)<=yMax) && ...
                   (z(3)>=zMin && z(3)<=zMax);
    end
    detsOut = detsIn(keep);
end
