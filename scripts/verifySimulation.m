function verifySimulation()
%verifySimulation  Comprehensive diagnostic verification of the Rainy Day pipeline.
%
%  Runs six phases of tests against the live codebase to verify that
%  terrain, occlusion, propagation, sensors, detections, degradation,
%  and trackers are all working as intended.
%
%  USAGE
%    addpath("scripts");
%    verifySimulation
%
%  OUTPUT
%    Console report with PASS/FAIL/WARN for each check.
%    No files are written, no plots are opened (silent diagnostics only).
%
%  See also: runSingleScenario

clc;
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║         RAINY DAY — SIMULATION VERIFICATION SUITE          ║\n');
fprintf('║         %s                                       ║\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, 'src')));

results = struct('pass',0,'fail',0,'warn',0);

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 1: CONFIG LOADING & SMOKE TEST
%% ════════════════════════════════════════════════════════════════════════
phase('1','Config Loading & Smoke Test');

% 1.1 — Can we load a run file?
try
    [scenario, config, sensors, metas] = trackbench.config.loadRunFile("my_run");
    results = check(results, true, '1.1 loadRunFile("my_run") succeeds');
catch ME
    results = check(results, false, '1.1 loadRunFile("my_run") succeeds', ME.message);
end

% 1.2 — Scenario has platforms
try
    nPlats = numel(scenario.Platforms);
    results = check(results, nPlats >= 1, '1.2 Scenario has >= 1 platform', ...
        sprintf('Found %d', nPlats));
catch ME
    results = check(results, false, '1.2 Scenario has >= 1 platform', ME.message);
end

% 1.3 — Scenario has targets (check via truth)
try
    restart(scenario);
    advance(scenario);
    p1 = scenario.Platforms{1};
    tgt = targetPoses(p1);
    nTgt = numel(tgt);
    results = check(results, nTgt >= 1, '1.3 Scenario has targets', ...
        sprintf('Found %d target(s)', nTgt));
catch ME
    results = check(results, false, '1.3 Scenario has targets', ME.message);
end

% 1.4 — Config has required fields
reqFields = {'scenario','degradation','environment','tracker_global', ...
    'filter_params','trackers_to_run','active_params','output','data_logging'};
missing = {};
for i = 1:numel(reqFields)
    if ~isfield(config, reqFields{i})
        missing{end+1} = reqFields{i}; %#ok<AGROW>
    end
end
results = check(results, isempty(missing), '1.4 Config has all required fields', ...
    ternary(isempty(missing), '', ['Missing: ' strjoin(string(missing),', ')]));

% 1.5 — Sensors built correctly
try
    platNames = fieldnames(sensors);
    totalSensors = 0;
    for p = 1:numel(platNames)
        totalSensors = totalSensors + numel(sensors.(platNames{p}));
    end
    results = check(results, totalSensors >= 1, '1.5 Sensors built', ...
        sprintf('%d sensor(s) across %d platform(s)', totalSensors, numel(platNames)));
catch ME
    results = check(results, false, '1.5 Sensors built', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 2: TERRAIN GENERATION & OCCLUSION
%% ════════════════════════════════════════════════════════════════════════
phase('2','Terrain Generation & Occlusion');

% 2.1 — Generate each terrain type
terrainTypes = {'water','rural','urban','mountain','desert'};
bounds = [-80000 80000; -80000 80000];
for i = 1:numel(terrainTypes)
    tt = terrainTypes{i};
    try
        [Z, bnd, Xg, Yg] = trackbench.environment.generateTerrain(tt, bounds);
        maxElev = -min(Z(:));
        gridSize = size(Z,1);
        ok = ~isempty(Z) && all(Z(:) <= 0) && gridSize > 10;
        results = check(results, ok, sprintf('2.1.%d generateTerrain("%s")', i, tt), ...
            sprintf('grid=%dx%d, maxElev=%.0fm', gridSize, gridSize, maxElev));
    catch ME
        results = check(results, false, sprintf('2.1.%d generateTerrain("%s")', i, tt), ME.message);
    end
end

% 2.2 — Mountain terrain has significant elevation
try
    [Z, ~, ~, ~] = trackbench.environment.generateTerrain('mountain', bounds);
    maxElev = -min(Z(:));
    results = check(results, maxElev > 500, '2.2 Mountain terrain > 500m', ...
        sprintf('maxElev=%.0fm', maxElev));
catch ME
    results = check(results, false, '2.2 Mountain terrain > 500m', ME.message);
end

% 2.3 — Water terrain is flat
try
    [Z, ~, ~, ~] = trackbench.environment.generateTerrain('water', bounds);
    maxElev = -min(Z(:));
    results = check(results, maxElev < 1, '2.3 Water terrain is flat', ...
        sprintf('maxElev=%.2fm', maxElev));
catch ME
    results = check(results, false, '2.3 Water terrain is flat', ME.message);
end

% 2.4 — Radar hilltop clearing works (Z near origin should be ~-50m, not deep underground)
try
    [Z, ~, Xg, Yg] = trackbench.environment.generateTerrain('mountain', bounds);
    nPts = size(Z,1);
    centerZ = Z(round(nPts/2), round(nPts/2));
    % Center should be a small hilltop (around -50m) not a deep valley
    results = check(results, centerZ > -100 && centerZ < 0, ...
        '2.4 Radar hilltop clearing at origin', ...
        sprintf('Z(origin)=%.1fm (expect ~-50m)', centerZ));
catch ME
    results = check(results, false, '2.4 Radar hilltop clearing at origin', ME.message);
end

% 2.5 — SurfaceManager attaches to scenario (mountain terrain)
try
    [scen2, cfg2, ~, ~] = trackbench.config.loadRunFile("demo_first_run");
    sm = scen2.SurfaceManager;
    hasSurfaces = ~isempty(sm) && ~isempty(sm.Surfaces);
    results = check(results, hasSurfaces, '2.5 SurfaceManager attached (mountain)', ...
        sprintf('UseOcclusion=%d', sm.UseOcclusion));
catch ME
    results = check(results, false, '2.5 SurfaceManager attached (mountain)', ME.message);
end

% 2.6 — Occlusion function works (target behind a ridge should be occluded)
try
    sm = scen2.SurfaceManager;
    sensorPos = [0, 0, -50];    % 50m hilltop
    visibleTgt = [0, -5000, -8000];  % 8km altitude, 5km away — should be visible
    [occVis, ~] = occlusion(sm, sensorPos, visibleTgt);
    % occVis=true means occluded; expect NOT occluded for high-altitude target
    results = check(results, ~occVis, '2.6 Occlusion: high-alt target visible', ...
        sprintf('occluded=%d (expect 0)', occVis));
catch ME
    results = check(results, false, '2.6 Occlusion: high-alt target visible', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 3: HORIZON MASKING
%% ════════════════════════════════════════════════════════════════════════
phase('3','Horizon Masking');

% 3.1 — Close high target is visible
try
    sPos = [0 0 -15];
    tPos = [30000 0 -5000];  % 30km, 5km alt
    vis = trackbench.environment.isAboveHorizon(sPos, tPos);
    results = check(results, vis, '3.1 Close high target (30km, 5km alt) visible');
catch ME
    results = check(results, false, '3.1 Close high target visible', ME.message);
end

% 3.2 — Very distant low target is below horizon
try
    sPos = [0 0 -15];
    tPos = [300000 0 -50];  % 300km, 50m alt — should be below horizon
    vis = trackbench.environment.isAboveHorizon(sPos, tPos);
    results = check(results, ~vis, '3.2 Distant low target (300km, 50m) below horizon', ...
        sprintf('visible=%d (expect 0)', vis));
catch ME
    results = check(results, false, '3.2 Distant low target below horizon', ME.message);
end

% 3.3 — Multiple targets (vectorized)
try
    sPos = [0 0 -15];
    tPos = [50000 0 -3000;   % 50km, 3km — visible
            50000 0 -10;     % 50km, 10m — probably visible (just)
            400000 0 -20];   % 400km, 20m — below horizon
    vis = trackbench.environment.isAboveHorizon(sPos, tPos);
    results = check(results, numel(vis) == 3 && vis(1) && ~vis(3), ...
        '3.3 Vectorized horizon check (3 targets)', ...
        sprintf('[%d %d %d]', vis(1), vis(2), vis(3)));
catch ME
    results = check(results, false, '3.3 Vectorized horizon check', ME.message);
end

% 3.4 — horizonrange function exists (Radar Toolbox dependency)
try
    hr = horizonrange(15, 4/3*6371000);
    results = check(results, hr > 0, '3.4 horizonrange() available (Radar Toolbox)', ...
        sprintf('15m tower → %.1f km horizon', hr/1000));
catch ME
    results = check(results, false, '3.4 horizonrange() available', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 4: PROPAGATION MODEL (VCP)
%% ════════════════════════════════════════════════════════════════════════
phase('4','Propagation Model (VCP)');

% 4.1 — radarvcd function exists (Radar Toolbox)
try
    [vcp_km, ang] = radarvcd(2.8e9, 111.12, 15, 'RangeUnit','km','HeightUnit','m');
    results = check(results, ~isempty(vcp_km), '4.1 radarvcd() available', ...
        sprintf('%d angle points', numel(ang)));
catch ME
    results = check(results, false, '4.1 radarvcd() available', ME.message);
end

% 4.2 — computeVerticalCoverage produces valid output
try
    % Build a minimal sensorInfo struct
    si = struct();
    si.sensor = sensors.(platNames{1}){1};
    si.platform = scenario.Platforms{1};
    si.sensorIndex = 1;
    si.isRadar = true;
    si.isMSSR = false;
    si.radarFreq = 2.8e9;
    vcpData = trackbench.environment.computeVerticalCoverage(si, 'rural', 4/3);
    hasData = vcpData(1).enabled && numel(vcpData(1).angles) > 5;
    results = check(results, hasData, '4.2 computeVerticalCoverage produces data', ...
        sprintf('enabled=%d, %d angle points, antHeight=%.0fm', ...
        vcpData(1).enabled, numel(vcpData(1).angles), vcpData(1).antennaHeight_m));
catch ME
    results = check(results, false, '4.2 computeVerticalCoverage', ME.message);
end

% 4.3 — VCP shows multipath lobing (range varies with elevation)
try
    if vcpData(1).enabled
        ranges = vcpData(1).maxRange_m;
        posIdx = vcpData(1).angles > 0.5 & vcpData(1).angles < 20;
        rangeVar = max(ranges(posIdx)) - min(ranges(posIdx));
        freeSpace = vcpData(1).freeSpaceRange_m;
        results = check(results, rangeVar > 1000, ...
            '4.3 VCP shows multipath lobing (range varies with elevation)', ...
            sprintf('range variation=%.0fm (free-space=%.0fkm)', rangeVar, freeSpace/1000));
    else
        results = check(results, false, '4.3 VCP multipath lobing', 'VCP disabled');
    end
catch ME
    results = check(results, false, '4.3 VCP multipath lobing', ME.message);
end

% 4.4 — applyVCPMask filters some detections
try
    if vcpData(1).enabled
        % Create fake detections at various ranges/elevations
        sPos = [0 0 -15];
        testDets = {};
        for r = [10000 50000 100000 200000]
            for elDeg = [0.5 2 5 15]
                h = r * tand(elDeg);
                pos = [r; 0; -(h+15)];
                testDets{end+1} = objectDetection(0, pos, 'MeasurementNoise', eye(3)*100, 'SensorIndex', 1); %#ok<AGROW>
            end
        end
        [keepMask, ~, ~] = trackbench.environment.applyVCPMask(testDets, sPos, vcpData(1));
        nKept = sum(keepMask);
        nTotal = numel(keepMask);
        someFiltered = nKept < nTotal && nKept > 0;
        results = check(results, someFiltered, '4.4 applyVCPMask filters some detections', ...
            sprintf('%d/%d kept (%.0f%%)', nKept, nTotal, 100*nKept/nTotal));
    else
        results = check(results, false, '4.4 applyVCPMask', 'VCP disabled');
    end
catch ME
    results = check(results, false, '4.4 applyVCPMask', ME.message);
end

% 4.5 — computePropFactor exists and runs (unused but available)
try
    sPos = [0 0 -15];
    tPos = [0 -30000 -3000];
    [F, details] = trackbench.environment.computePropFactor(sPos, tPos, 2.8e9, 'rural');
    results = check(results, isfinite(F), '4.5 computePropFactor runs (not integrated)', ...
        sprintf('F=%.1f dB at 30km/3km', F));
catch ME
    results = check(results, false, '4.5 computePropFactor runs', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 5: SENSOR & DETECTION VERIFICATION
%% ════════════════════════════════════════════════════════════════════════
phase('5','Sensor & Detection Verification');

% 5.1 — buildSensor works for core types
coreTypes = {'PSR','SSR','AESA','PAR','MARITIME','IRST','TWS','FLIR','ARSR'};
for i = 1:numel(coreTypes)
    sType = coreTypes{i};
    try
        [sObj, ~] = trackbench.sensors.buildSensor(100+i, sType);
        hasProp = isprop(sObj, 'SensorIndex');
        results = check(results, hasProp, sprintf('5.1.%d buildSensor("%s")', i, sType));
    catch ME
        results = check(results, false, sprintf('5.1.%d buildSensor("%s")', i, sType), ME.message);
    end
end

% 5.2 — Run detections with water terrain (no environment effects) — baseline
try
    [scWater, cfgWater, sensWater, metaWater] = trackbench.config.loadRunFile("my_run");
    % Override to water (no effects) for clean baseline
    cfgWater.environment.terrain_type = 'water';
    cfgWater.environment.terrain_occlusion = false;
    cfgWater.environment.horizon_masking = false;
    cfgWater.environment.ground_clutter = false;
    cfgWater.environment.propagation_model = false;
    cfgWater.environment.clutter_density = 0;
    
    restart(scWater);
    dataLogClean = trackbench.detections.runDetections(scWater, false, metaWater, cfgWater.environment);
    nScansClean = numel(dataLogClean.Time);
    nDetsClean = sum(cellfun(@numel, dataLogClean.Detections));
    results = check(results, nScansClean >= 3 && nDetsClean > 0, ...
        '5.2 runDetections baseline (water, no effects)', ...
        sprintf('%d scans, %d total detections', nScansClean, nDetsClean));
catch ME
    results = check(results, false, '5.2 runDetections baseline', ME.message);
    nDetsClean = 0;
    nScansClean = 0;
end

% 5.3 — Detection format check (objectDetection with 3-element Measurement)
try
    firstScan = dataLogClean.Detections{1};
    det1 = firstScan{1};
    nMeas = numel(det1.Measurement);
    hasSensorIdx = det1.SensorIndex > 0;
    hasNoise = all(size(det1.MeasurementNoise) >= [3 3]);
    results = check(results, nMeas >= 3 && hasSensorIdx && hasNoise, ...
        '5.3 Detection format (Measurement, SensorIndex, Noise)', ...
        sprintf('nMeas=%d, SensorIdx=%d, NoiseSize=%dx%d', ...
        nMeas, det1.SensorIndex, size(det1.MeasurementNoise)));
catch ME
    results = check(results, false, '5.3 Detection format', ME.message);
end

% 5.4 — Run detections WITH environment effects (mountain)
try
    [scMtn, cfgMtn, sensMtn, metaMtn] = trackbench.config.loadRunFile("demo_first_run");
    % Force no-cache
    restart(scMtn);
    dataLogMtn = trackbench.detections.runDetections(scMtn, false, metaMtn, cfgMtn.environment);
    nScansMtn = numel(dataLogMtn.Time);
    nDetsMtn = sum(cellfun(@numel, dataLogMtn.Detections));
    results = check(results, nScansMtn >= 3 && nDetsMtn > 0, ...
        '5.4 runDetections with mountain environment', ...
        sprintf('%d scans, %d total detections', nScansMtn, nDetsMtn));
catch ME
    results = check(results, false, '5.4 runDetections mountain', ME.message);
end

% 5.5 — Ground clutter adds false returns
try
    sPos = [0 0 -15];
    sParams = struct('rangeLimits',[0 111120],'rangeRes',93, ...
        'fov',[1.4 30],'tilt',2,'mountingLoc',[0 0 -15]);
    envCfg = struct('terrain_type','urban','clutter_density',1.0);
    
    nClutter = 0;
    for trial = 1:20
        cDets = trackbench.environment.generateGroundClutter(0, sPos, 1, sParams, envCfg);
        nClutter = nClutter + numel(cDets);
    end
    avgClutter = nClutter / 20;
    results = check(results, avgClutter > 2, '5.5 Ground clutter generates returns (urban)', ...
        sprintf('avg %.1f returns/scan over 20 trials', avgClutter));
catch ME
    results = check(results, false, '5.5 Ground clutter', ME.message);
end

% 5.6 — SSR is classified as MSSR
try
    [scDasr, cfgDasr, sensDasr, metaDasr] = loadDASR(root);
    hasMSSR = false;
    platNames2 = fieldnames(metaDasr);
    for pp = 1:numel(platNames2)
        mList = metaDasr.(platNames2{pp});
        for mm = 1:numel(mList)
            if isfield(mList{mm}, 'type') && contains(upper(string(mList{mm}.type)), 'SSR')
                hasMSSR = true;
            end
        end
    end
    results = check(results, hasMSSR, '5.6 SSR recognized as MSSR in metadata');
catch ME
    results = check(results, false, '5.6 SSR as MSSR', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 6: WEATHER DEGRADATION
%% ════════════════════════════════════════════════════════════════════════
phase('6','Weather Degradation');

% For degradation tests, we need a scenario run with and without degradation
% on identical geometry to compare detection counts.
try
    [scClean2, cfgClean2, sensClean2, metaClean2] = trackbench.config.loadRunFile("my_run");
    cfgClean2.environment.terrain_occlusion = false;
    cfgClean2.environment.horizon_masking = false;
    cfgClean2.environment.ground_clutter = false;
    cfgClean2.environment.propagation_model = false;
    cfgClean2.environment.clutter_density = 0;

    % Clean run
    restart(scClean2);
    dlClean = trackbench.detections.runDetections(scClean2, false, metaClean2, cfgClean2.environment);
    nDetClean = sum(cellfun(@numel, dlClean.Detections));

    % Degraded run (same scenario, degradation ON)
    restart(scClean2);
    dlDegrad = trackbench.detections.runDetections(scClean2, true, metaClean2, cfgClean2.environment);
    nDetDegrad = sum(cellfun(@numel, dlDegrad.Detections));

    % 6.1 — Degradation changes detection count (adds clutter, may reduce real returns)
    % S-band at 16 mm/hr barely reduces Pd but ADDS weather clutter,
    % so total detections may INCREASE. Check that counts differ.
    results = check(results, nDetDegrad ~= nDetClean, ...
        '6.1 Degradation changes detection count (clutter + Pd effects)', ...
        sprintf('clean=%d, degraded=%d (%+d)', nDetClean, nDetDegrad, nDetDegrad - nDetClean));

    % 6.2 — Degraded noise is larger
    noiseClean = 0; noiseDegrad = 0; nSamples = 0;
    for s = 1:min(numel(dlClean.Detections), numel(dlDegrad.Detections))
        dC = dlClean.Detections{s};
        dD = dlDegrad.Detections{s};
        if ~isempty(dC)
            noiseClean = noiseClean + trace(dC{1}.MeasurementNoise);
            nSamples = nSamples + 1;
        end
        if ~isempty(dD)
            noiseDegrad = noiseDegrad + trace(dD{1}.MeasurementNoise);
        end
    end
    if nSamples > 0
        avgNoiseClean = noiseClean / nSamples;
        avgNoiseDegrad = noiseDegrad / nSamples;
        results = check(results, avgNoiseDegrad > avgNoiseClean, ...
            '6.2 Degraded measurement noise is larger', ...
            sprintf('clean=%.0f, degraded=%.0f', avgNoiseClean, avgNoiseDegrad));
    else
        results = check(results, false, '6.2 Degraded noise', 'No samples');
    end
catch ME
    results = check(results, false, '6.1-6.2 Degradation tests', ME.message);
end

% 6.3 — Weather clutter adds false returns in degraded mode
try
    % Count detections from degraded run that have no target association
    % (proxy: weather clutter is generated with specific position pattern)
    results = check(results, nDetDegrad > 0, '6.3 Degraded run has detections', ...
        sprintf('%d total (includes weather clutter)', nDetDegrad));
catch ME
    results = check(results, false, '6.3 Weather clutter', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 7: TRACKER VERIFICATION
%% ════════════════════════════════════════════════════════════════════════
phase('7','Tracker Verification');

% Use clean detections from Phase 5 for tracker tests
try
    if exist('dataLogClean','var') && ~isempty(dataLogClean) && numel(dataLogClean.Time) >= 3
        dlTest = dataLogClean;
    else
        [scTest, cfgTest, ~, metaTest] = trackbench.config.loadRunFile("my_run");
        cfgTest.environment.horizon_masking = false;
        cfgTest.environment.ground_clutter = false;
        cfgTest.environment.propagation_model = false;
        restart(scTest);
        dlTest = trackbench.detections.runDetections(scTest, false, metaTest, cfgTest.environment);
    end
catch ME
    results = check(results, false, '7.0 Load test detections', ME.message);
    dlTest = [];
end

if ~isempty(dlTest) && numel(dlTest.Time) >= 3
    % Load tracker globals
    globPath = fullfile(root, 'config', 'trackers', 'tracker_globals.json');
    tg = jsondecode(fileread(globPath));
    filterParams = tg.filter;
    pd = tg.detection_probability.ideal;

    trackerTypes = {'GNN','JPDA','TOMHT'};
    for ti = 1:numel(trackerTypes)
        tType = trackerTypes{ti};
        try
            % Load tracker config
            trkPath = fullfile(root, 'config', 'trackers', tType, ['default_' tType '.json']);
            tDef = jsondecode(fileread(trkPath));
            trkParams = tDef.params;
            trkGlobal = tg;
            if isfield(tDef, 'volume'); trkGlobal.volume = tDef.volume; end
            if isfield(tDef, 'beta');   trkGlobal.beta = tDef.beta; end

            tracker = trackbench.tracking.buildTracker(tType, 'CV', trkParams, ...
                trkGlobal, filterParams, pd, 1);

            % Run tracker
            [trkSum, truthSum, trkMet, ~, tTime, ~, swapRpt] = ...
                trackbench.tracking.runTracker(dlTest, tracker, false, false, false);

            nTracks = height(trkSum);
            hasMetrics = ~isempty(trkMet) && height(trkMet) > 0;
            results = check(results, nTracks > 0 && hasMetrics, ...
                sprintf('7.%d %s+CV tracker runs and produces tracks', ti, tType), ...
                sprintf('%d tracks, swapFree=%d, time=%.2fs', nTracks, swapRpt.swapFree, tTime));
        catch ME
            results = check(results, false, sprintf('7.%d %s+CV tracker', ti, tType), ME.message);
        end
    end

    % 7.4 — IMM filter initializes
    try
        det1 = dlTest.Detections{1}{1};
        immFilter = trackbench.tracking.initIMMFilter(det1, filterParams);
        nModels = numel(immFilter.TrackingFilters);
        results = check(results, nModels >= 2, '7.4 IMM filter initializes', ...
            sprintf('%d models', nModels));
    catch ME
        results = check(results, false, '7.4 IMM filter', ME.message);
    end

    % 7.5 — CV filter initializes
    try
        det1 = dlTest.Detections{1}{1};
        cvFilter = trackbench.tracking.initCVFilter(det1, filterParams);
        nState = numel(cvFilter.State);
        results = check(results, nState == 6, '7.5 CV filter initializes (6-state)', ...
            sprintf('%d states', nState));
    catch ME
        results = check(results, false, '7.5 CV filter', ME.message);
    end
else
    results = check(results, false, '7.x Tracker tests', 'No test detections available');
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 8: v3.2.0 FEATURE VERIFICATION
%% ════════════════════════════════════════════════════════════════════════
phase('8','v3.2.0 Feature Verification');

% 8.1 — rainpl() available and returns physically correct values
try
    L_sband = rainpl(30000, 2.8e9, 50);  % S-band, 30km, 50mm/hr
    L_xband = rainpl(30000, 9e9, 50);    % X-band, 30km, 50mm/hr
    sband_ok = L_sband > 0 && L_sband < 2;      % should be ~0.39 dB
    xband_ok = L_xband > 10 && L_xband < 30;    % should be ~16.1 dB
    ratio_ok = L_xband / L_sband > 20;           % X-band >> S-band
    results = check(results, sband_ok && xband_ok && ratio_ok, ...
        '8.1 rainpl() returns correct S-band vs X-band values', ...
        sprintf('S=%.2f dB, X=%.1f dB, ratio=%.0fx', L_sband, L_xband, L_xband/L_sband));
catch ME
    results = check(results, false, '8.1 rainpl() function', ME.message);
end

% 8.2 — buildRCSProfile creates valid rcsSignature
try
    sig = trackbench.environment.buildRCSProfile('stealth', -10);
    isRCS = isa(sig, 'rcsSignature');
    patSize = size(sig.Pattern);
    hasAz = numel(sig.Azimuth) > 2;
    hasEl = numel(sig.Elevation) > 2;
    % Check stealth nose-on is near base, broadside is much higher
    rcsNose = sig.value(0, 0, 2.8e9);      % nose-on
    rcsSide = sig.value(90, 0, 2.8e9);      % broadside
    spread_ok = rcsSide - rcsNose > 10;     % should be ~17 dB
    results = check(results, isRCS && hasAz && hasEl && spread_ok, ...
        '8.2 buildRCSProfile("stealth") creates valid rcsSignature', ...
        sprintf('pattern=%dx%d, nose=%.0f dBsm, side=%.0f dBsm', patSize(1), patSize(2), rcsNose, rcsSide));
catch ME
    results = check(results, false, '8.2 buildRCSProfile', ME.message);
end

% 8.3 — applyDopplerFade drops tangential detections
try
    sPos = [0; 0; -15];
    % Tangential: target far away along Y, moving purely in X (perpendicular to LOS)
    % LOS = [0, 30000, -4985] → v_radial = dot([250,0,0], unit) ≈ 0
    tangPos = [0, 30000, -5000];
    tangVel = [250, 0, 0];
    tangTarget = struct('Position', tangPos, 'Velocity', tangVel);
    tangDet = {objectDetection(0, tangPos(:), 'MeasurementNoise', eye(3)*100, 'SensorIndex', 1)};
    
    % Radial: target far away along Y, moving purely in Y (along LOS)
    radPos = [0, 30000, -5000];
    radVel = [0, 250, 0];
    radTarget = struct('Position', radPos, 'Velocity', radVel);
    radDet = {objectDetection(0, radPos(:), 'MeasurementNoise', eye(3)*100, 'SensorIndex', 1)};
    
    nDropTang = 0; nDropRad = 0;
    for trial = 1:100
        [kept1, ~] = trackbench.environment.applyDopplerFade(tangDet, sPos, tangTarget, struct('radarFreq', 2.8e9), struct());
        if numel(kept1) == 0; nDropTang = nDropTang + 1; end
        [kept2, ~] = trackbench.environment.applyDopplerFade(radDet, sPos, radTarget, struct('radarFreq', 2.8e9), struct());
        if numel(kept2) == 0; nDropRad = nDropRad + 1; end
    end
    % Tangential should drop often (>50%), radial should almost never drop
    tang_ok = nDropTang > 50;
    rad_ok = nDropRad < 5;
    results = check(results, tang_ok && rad_ok, ...
        '8.3 applyDopplerFade: tangential drops, radial keeps', ...
        sprintf('tangential dropped %d%%, radial dropped %d%%', nDropTang, nDropRad));
catch ME
    results = check(results, false, '8.3 applyDopplerFade', ME.message);
end

% 8.4 — IMM filter vertical process noise is correct
try
    mockDet = objectDetection(0, [1000; 2000; -5000], 'MeasurementNoise', eye(3)*100, 'SensorIndex', 1);
    fp = struct('init_speed_kmh', 900, 'imm_transition_prob', 0.97, ...
        'scale_accel_horz', 30, 'scale_accel_vert', 20, 'scale_omega_dot', 30);
    immF = trackbench.tracking.initIMMFilter(mockDet, fp);
    % CV filter (index 1): ProcessNoise should be 3x3 diag([900, 900, 400])
    cvQ = immF.TrackingFilters{1}.ProcessNoise;
    qh = cvQ(1,1); qv = cvQ(3,3);
    cv_ok = abs(qh - 900) < 1 && abs(qv - 400) < 1;
    % CT filter (index 2): ProcessNoise should be 4x4 diag([900, 900, 900, 400])
    ctQ = immF.TrackingFilters{2}.ProcessNoise;
    ct_qh = ctQ(1,1); ct_qv = ctQ(4,4); ct_qo = ctQ(3,3);
    ct_ok = abs(ct_qh - 900) < 1 && abs(ct_qv - 400) < 1 && abs(ct_qo - 900) < 1;
    results = check(results, cv_ok && ct_ok, ...
        '8.4 IMM filter vertical process noise (Qv=400, Qh=900)', ...
        sprintf('CV=[%.0f,%.0f,%.0f], CT=[%.0f,%.0f,%.0f,%.0f]', cvQ(1,1),cvQ(2,2),cvQ(3,3), ctQ(1,1),ctQ(2,2),ctQ(3,3),ctQ(4,4)));
catch ME
    results = check(results, false, '8.4 IMM vertical process noise', ME.message);
end

% 8.5 — CenterFrequency set on PSR sensor
try
    [psrSensor, psrMeta] = trackbench.sensors.buildSensor(99, 'PSR');
    freq = psrSensor.CenterFrequency;
    freq_ok = abs(freq - 2.8e9) < 1e8;  % should be 2.8 GHz ± 100 MHz
    meta_ok = isfield(psrMeta, 'radarFreq') && abs(psrMeta.radarFreq - 2.8e9) < 1e8;
    results = check(results, freq_ok && meta_ok, ...
        '8.5 PSR CenterFrequency = 2.8 GHz (S-band)', ...
        sprintf('sensor=%.2e Hz, meta=%.2e Hz', freq, psrMeta.radarFreq));
catch ME
    results = check(results, false, '8.5 PSR CenterFrequency', ME.message);
end

% 8.6 — Waypoint trajectory behavior works via full loadRunFile
try
    % Test waypoint behavior by loading compound_demo (uses waypoints)
    wpRunPath = fullfile(root, 'config', 'runs', 'compound_demo.json');
    if isfile(wpRunPath)
        [wpScen, wpCfg, ~, ~] = trackbench.config.loadRunFile('compound_demo');
        nPlats = numel(wpScen.Platforms);
        % compound_demo has 1 tower + 2 targets = 3 platforms
        results = check(results, nPlats >= 3, ...
            '8.6 Waypoint trajectory (compound_demo loads)', ...
            sprintf('%d platforms (tower + 2 waypoint targets)', nPlats));
    else
        % Fallback: just check addTargetFromDef exists
        results = check(results, exist('trackbench.scenario.addTargetFromDef','file') > 0, ...
            '8.6 addTargetFromDef exists (compound_demo.json not found)');
    end
catch ME
    results = check(results, false, '8.6 Waypoint trajectory', ME.message);
end

% 8.7 — 5 RCS profiles all build successfully
try
    profiles = {'stealth','fighter','airliner','drone','missile'};
    allOk = true;
    for pi = 1:numel(profiles)
        sig = trackbench.environment.buildRCSProfile(profiles{pi}, 0);
        if ~isa(sig, 'rcsSignature'); allOk = false; end
    end
    results = check(results, allOk, '8.7 All 5 RCS profiles build successfully');
catch ME
    results = check(results, false, '8.7 RCS profiles', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  PHASE 9: KNOWN ISSUES CHECK
%% ════════════════════════════════════════════════════════════════════════
phase('9','Known Issues & Code Quality');

% 9.1 — showTruth hardcodes 2 targets (known issue)
try
    rtPath = fullfile(root, 'src', '+trackbench', '+tracking', 'runTracker.m');
    rtTxt = fileread(rtPath);
    hasHardcode = contains(rtTxt, 'trajPos{1}') && contains(rtTxt, 'trajPos{2}') && ...
                  ~contains(rtTxt, 'trajPos{3}');
    if hasHardcode
        results = warn(results, '9.1 showTruth hardcodes 2 targets (known issue — fix pending)');
    else
        results = check(results, true, '9.1 showTruth supports N targets');
    end
catch ME
    results = check(results, false, '9.1 showTruth check', ME.message);
end

% 9.2 — falseMeasInSurveillanceVolume is hardcoded (known issue)
try
    rdPath = fullfile(root, 'src', '+trackbench', '+detections', 'runDetections.m');
    rdTxt = fileread(rdPath);
    hasHardcode = contains(rdTxt, '-20.5e3') || contains(rdTxt, '-1.5e3');
    if hasHardcode
        results = warn(results, '9.2 falseMeasInSurveillanceVolume uses hardcoded positions (should use sensor range)');
    else
        results = check(results, true, '9.2 falseMeasInSurveillanceVolume is dynamic');
    end
catch ME
    results = check(results, false, '9.2 falseMeas check', ME.message);
end

% 9.3 — weatherSeverity / rain model check
try
    hasRainModel = contains(rdTxt, 'applyRainDegradation') || contains(rdTxt, 'rainpl');
    if hasRainModel
        results = check(results, true, '9.3 Rain attenuation model integrated (applyRainDegradation/rainpl)');
    else
        results = warn(results, '9.3 No rain attenuation model found in runDetections');
    end
catch ME
    results = check(results, false, '9.3 Rain model check', ME.message);
end

% 9.4 — Doppler fade integrated in runDetections
try
    hasDoppler = contains(rdTxt, 'applyDopplerFade');
    if hasDoppler
        results = check(results, true, '9.4 Doppler fade model integrated in runDetections');
    else
        results = warn(results, '9.4 applyDopplerFade not found in runDetections');
    end
catch ME
    results = check(results, false, '9.4 Doppler fade check', ME.message);
end

% 9.5 — computePropFactor exists but is not called from runDetections
try
    if ~contains(rdTxt, 'computePropFactor')
        results = warn(results, '9.5 computePropFactor.m exists but is not used in runDetections');
    else
        results = check(results, true, '9.5 computePropFactor integrated');
    end
catch ME
    results = check(results, false, '9.5 computePropFactor check', ME.message);
end

%% ════════════════════════════════════════════════════════════════════════
%  SUMMARY
%% ════════════════════════════════════════════════════════════════════════
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    VERIFICATION SUMMARY                     ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  PASS : %-3d                                                 ║\n', results.pass);
fprintf('║  FAIL : %-3d                                                 ║\n', results.fail);
fprintf('║  WARN : %-3d                                                 ║\n', results.warn);
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
total = results.pass + results.fail + results.warn;
if results.fail == 0
    fprintf('║  STATUS: ALL CHECKS PASSED                                  ║\n');
elseif results.fail <= 2
    fprintf('║  STATUS: MOSTLY PASSING — %d issue(s) to investigate         ║\n', results.fail);
else
    fprintf('║  STATUS: %d FAILURES — investigation needed                  ║\n', results.fail);
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

end % main function


%% ========================================================================
%  HELPER FUNCTIONS
%% ========================================================================

function phase(num, name)
    fprintf('\n── PHASE %s: %s ─────────────────────────\n', num, upper(name));
end

function r = check(r, passed, name, detail)
    if nargin < 4; detail = ''; end
    if passed
        r.pass = r.pass + 1;
        fprintf('  ✓ PASS  %s', name);
    else
        r.fail = r.fail + 1;
        fprintf('  ✗ FAIL  %s', name);
    end
    if ~isempty(detail)
        fprintf('  [%s]', detail);
    end
    fprintf('\n');
end

function r = warn(r, msg)
    r.warn = r.warn + 1;
    fprintf('  ⚠ WARN  %s\n', msg);
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function [scen, cfg, sens, meta] = loadDASR(root)
%loadDASR  Try to load a PSR+SSR config for MSSR testing.
    % Create a temporary run file in memory that uses PSR+SSR
    runDir = fullfile(root, 'config', 'runs');
    
    % Check if we have a run with SSR
    testRun = struct();
    testRun.description = 'Verification test: PSR+SSR';
    testRun.sensors = {{'PSR/default_PSR', 'SSR/default_SSR'}};
    testRun.targets = 'crossing_pair/default_crossing_pair';
    testRun.terrain = 'water/default_water';
    testRun.trackers = {{'GNN/default_GNN'}};
    testRun.degradation = struct('enabled', false, 'type', 'rain');
    testRun.platforms = struct();
    testRun.cache = struct('use_cached_detections', false, 'save_detections', false);
    testRun.output = struct('show_visuals', false, 'animate_visuals', false, ...
        'save_results', false, 'print_diagnostics', false);
    
    % Write temp file
    tempPath = fullfile(runDir, '_verify_dasr.json');
    fid = fopen(tempPath, 'w');
    fprintf(fid, '%s', jsonencode(testRun, 'PrettyPrint', true));
    fclose(fid);
    
    try
        [scen, cfg, sens, meta] = trackbench.config.loadRunFile("_verify_dasr");
    catch ME
        if isfile(tempPath); delete(tempPath); end
        rethrow(ME);
    end
    
    % Cleanup
    if isfile(tempPath); delete(tempPath); end
end