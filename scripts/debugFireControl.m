%% debugFireControl.m — Verify log(det(S)) and sweep gate values
cd('C:\Users\Admin\Documents\RAINY DAY GIT COPY\adeptus-rainyday-tracking');
addpath(genpath(fullfile(pwd, 'src')));

scenName = 'fire_control_engagement';
[scenario, config, sensors, metas] = trackbench.scenario.loadScenario(scenName);

platformNames = fieldnames(sensors);
numSensors = 0;
for p = 1:numel(platformNames)
    numSensors = numSensors + numel(sensors.(platformNames{p}));
end

dataLog = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas);

%% Manually compute assignment cost
det1 = dataLog.Detections{1}{1};
det3 = dataLog.Detections{3}{1};

fprintf('\n=== ASSIGNMENT COST ANALYSIS ===\n');
fprintf('Det1 (t=%.2f): [%.0f, %.0f, %.0f]\n', det1.Time, det1.Measurement(:)');
fprintf('Det3 (t=%.2f): [%.0f, %.0f, %.0f]\n', det3.Time, det3.Measurement(:)');

filterParams = config.filter_params;
filter = trackbench.tracking.initCVFilter(det1, filterParams);

dt = det3.Time - det1.Time;
fprintf('\ndt = %.2f s\n', dt);
predict(filter, dt);

% Innovation
z = det3.Measurement(:);
z_pred = filter.State(1:2:end);
innov = z - z_pred;
fprintf('Innovation: [%.0f, %.0f, %.0f]\n', innov');

% Innovation covariance S = H*P*H' + R
P = filter.StateCovariance;
H = zeros(3, 6);
H(1,1) = 1; H(2,3) = 1; H(3,5) = 1;
R = det3.MeasurementNoise;
S = H * P * H' + R;
fprintf('S diagonal: [%.0f, %.0f, %.0f]\n', S(1,1), S(2,2), S(3,3));

mahala2 = innov' * (S \ innov);
logdetS = log(det(S));
total = mahala2 + logdetS;
fprintf('\nMahalanobis^2:    %.4f\n', mahala2);
fprintf('log(det(S)):      %.4f\n', logdetS);
fprintf('TOTAL COST:       %.4f\n', total);
fprintf('Gate threshold:   %d\n', config.active_params.gate(1));
fprintf('Result: cost %.1f %s gate %d\n', total, ...
    ternary(total <= config.active_params.gate(1), '<=', '>'), ...
    config.active_params.gate(1));

%% Also check scan 1→2 (dt=4.8s, the typical interval)
det2 = dataLog.Detections{2}{1};
filter2 = trackbench.tracking.initCVFilter(det1, filterParams);
dt2 = det2.Time - det1.Time;
predict(filter2, dt2);
z2 = det2.Measurement(:);
z_pred2 = filter2.State(1:2:end);
innov2 = z2 - z_pred2;
P2 = filter2.StateCovariance;
S2 = H * P2 * H' + det2.MeasurementNoise;
mahala2_2 = innov2' * (S2 \ innov2);
logdetS2 = log(det(S2));
total2 = mahala2_2 + logdetS2;
fprintf('\n=== SCAN 1→2 (dt=%.2f) ===\n', dt2);
fprintf('S diagonal: [%.0f, %.0f, %.0f]\n', S2(1,1), S2(2,2), S2(3,3));
fprintf('Mahalanobis^2: %.4f\n', mahala2_2);
fprintf('log(det(S)):   %.4f\n', logdetS2);
fprintf('TOTAL COST:    %.4f\n', total2);

%% Now sweep gate values
fprintf('\n=== GATE SWEEP (fire_control_engagement) ===\n');
for testGate = [25, 30, 40, 45, 50, 55, 60, 80, 100]
    params2 = config.active_params;
    params2.gate = [testGate, Inf];
    
    tracker = trackbench.tracking.buildTracker('GNN', 'CV', params2, ...
        config.tracker_global, config.filter_params, params2.pd, numSensors);
    
    allTracks = objectTrack.empty(0,1);
    for i = 1:numel(dataLog.Time)
        simTime = dataLog.Time(i);
        scanCells = dataLog.Detections{i};
        if ~iscell(scanCells), scanCells = num2cell(scanCells(:)); end
        
        if isLocked(tracker) && ~isempty(allTracks)
            detectIDs = uint32([allTracks.TrackID]');
        else
            detectIDs = uint32([]);
        end
        
        [~, ~, allTracks] = tracker(scanCells, simTime, detectIDs);
    end
    
    nConf = sum([allTracks.IsConfirmed]);
    nTotal = numel(allTracks);
    maxAge = 0;
    if ~isempty(allTracks), maxAge = max([allTracks.Age]); end
    fprintf('  gate=%3d → %d confirmed, %d total, maxAge=%d\n', ...
        testGate, nConf, nTotal, maxAge);
end

%% Also test with MaxNumSensors=1 (ignore fire control for scoring)
fprintf('\n=== MaxNumSensors=1 SWEEP ===\n');
for testGate = [25, 30, 40, 45, 50, 55, 60]
    params2 = config.active_params;
    params2.gate = [testGate, Inf];
    
    tracker = trackbench.tracking.buildTracker('GNN', 'CV', params2, ...
        config.tracker_global, config.filter_params, params2.pd, 1);
    
    allTracks = objectTrack.empty(0,1);
    for i = 1:numel(dataLog.Time)
        simTime = dataLog.Time(i);
        scanCells = dataLog.Detections{i};
        if ~iscell(scanCells), scanCells = num2cell(scanCells(:)); end
        
        if isLocked(tracker) && ~isempty(allTracks)
            detectIDs = uint32([allTracks.TrackID]');
        else
            detectIDs = uint32([]);
        end
        
        [~, ~, allTracks] = tracker(scanCells, simTime, detectIDs);
    end
    
    nConf = sum([allTracks.IsConfirmed]);
    maxAge = 0;
    if ~isempty(allTracks), maxAge = max([allTracks.Age]); end
    fprintf('  gate=%3d, nSens=1 → %d confirmed, %d total, maxAge=%d\n', ...
        testGate, nConf, numel(allTracks), maxAge);
end

function r = ternary(cond, a, b)
    if cond, r = a; else, r = b; end
end
