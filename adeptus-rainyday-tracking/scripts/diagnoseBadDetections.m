%diagnoseBadDetections  Isolate why the 747 gets sparse detections.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  USAGE
%    clear classes; clear all; addpath("scripts"); diagnoseBadDetections

clc; close all;

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, "src")));

fprintf('\n========================================\n');
fprintf('  DETECTION DIAGNOSTIC — range_rcs_test\n');
fprintf('========================================\n\n');

%% Load scenario
[scenario, config, sensors, metas] = trackbench.config.loadRunFile("range_rcs_test");

%% Print target info
allPlats = scenario.Platforms;
fprintf('\n--- PLATFORMS ---\n');
for p = 1:numel(allPlats)
    plat = allPlats{p};
    hasSig = ~isempty(plat.Signatures);
    sigStr = 'none';
    if hasSig
        sig = plat.Signatures{1};
        if isa(sig, 'rcsSignature')
            pat = sig.Pattern;
            if isscalar(pat)
                sigStr = sprintf('scalar %.0f dBsm', pat);
            else
                sigStr = sprintf('matrix %dx%d, center=%.1f dBsm', size(pat,1), size(pat,2), pat(ceil(end/2), ceil(end/2)));
            end
        else
            sigStr = class(sig);
        end
    end
    nSensors = numel(plat.Sensors);
    fprintf('  Platform %d: %d sensors, Signatures: %s\n', p, nSensors, sigStr);
end

%% Print PSR config
tower = allPlats{1};
psr = tower.Sensors{1};
fprintf('\n--- PSR CONFIG ---\n');
fprintf('  UpdateRate: %.2f Hz\n', psr.UpdateRate);
fprintf('  FieldOfView: [%.1f, %.1f]\n', psr.FieldOfView(1), psr.FieldOfView(2));
fprintf('  RangeLimits: [%.0f, %.0f]\n', psr.RangeLimits(1), psr.RangeLimits(2));
fprintf('  ReferenceRange: %.0f\n', psr.ReferenceRange);
fprintf('  ReferenceRCS: %.1f dBsm\n', psr.ReferenceRCS);
fprintf('  DetectionProbability: %.2f\n', psr.DetectionProbability);
fprintf('  FalseAlarmRate: %.1e\n', psr.FalseAlarmRate);
fprintf('  ScanMode: %s\n', psr.ScanMode);
try fprintf('  MechanicalElevationLimits: [%.1f, %.1f]\n', psr.MechanicalElevationLimits(1), psr.MechanicalElevationLimits(2)); catch; end
try fprintf('  MechanicalAzimuthLimits: [%.0f, %.0f]\n', psr.MechanicalAzimuthLimits(1), psr.MechanicalAzimuthLimits(2)); catch; end
try fprintf('  MaxAzimuthScanRate: %.2f deg/s\n', psr.MaxAzimuthScanRate); catch; end
try fprintf('  CenterFrequency: %.2e Hz\n', psr.CenterFrequency); catch; end

% Check if HasRCSSignature exists
fprintf('\n--- HasRCSSignature CHECK ---\n');
if isprop(psr, 'HasRCSSignature')
    fprintf('  HasRCSSignature EXISTS: %s\n', string(psr.HasRCSSignature));
else
    fprintf('  HasRCSSignature: PROPERTY DOES NOT EXIST in R2025b\n');
end
propList = {'HasRCSSignature','HasTargetSignatures','TargetReportFormat'};
for pp = 1:numel(propList)
    if isprop(psr, propList{pp})
        fprintf('  %s = %s\n', propList{pp}, string(psr.(propList{pp})));
    end
end

%% Helper: extract TargetIndex from a detection
getTargetIdx = @(det) getTargetIndexFromDet(det);

%% ===== TEST 1: Raw sensor detections (step-by-step) =====
fprintf('\n\n========= TEST 1: RAW SENSOR DETECTIONS =========\n');
fprintf('(No RCS filter, no Doppler fade — just the sensor itself)\n\n');

restart(scenario);
scanNum = 0;
totalT1 = 0; totalT2 = 0; totalFalse = 0;
stepBufIdx = [];  % track target indices separately
stepBufDets = {};
totalSteps = 0;

while advance(scenario)
    simTime = scenario.SimulationTime;
    targets = targetPoses(tower);
    ins = pose(tower, 'true');
    
    [dets, ~, sensorCfg] = psr(targets, ins, simTime);
    totalSteps = totalSteps + 1;
    
    % Store detections and their target indices separately
    for dd = 1:numel(dets)
        tgtIdx = getTargetIdx(dets{dd});
        stepBufIdx(end+1) = tgtIdx; %#ok<AGROW>
        stepBufDets{end+1} = dets{dd}; %#ok<AGROW>
    end
    
    % Flush at scan done
    if sensorCfg.IsScanDone
        scanNum = scanNum + 1;
        nT1 = sum(stepBufIdx == 1);
        nT2 = sum(stepBufIdx == 2);
        nFalse = sum(stepBufIdx == 0);
        totalT1 = totalT1 + nT1;
        totalT2 = totalT2 + nT2;
        totalFalse = totalFalse + nFalse;
        
        % Get target positions at this time
        t1str = 'N/A'; t2str = 'N/A';
        if numel(targets) >= 1
            p1 = targets(1).Position;
            r1 = norm(p1); el1 = atan2d(-p1(3), norm(p1(1:2)));
            az1 = atan2d(p1(2), p1(1));
            t1str = sprintf('r=%5.0fm el=%4.1f° az=%6.1f°', r1, el1, az1);
        end
        if numel(targets) >= 2
            p2 = targets(2).Position;
            r2 = norm(p2); el2 = atan2d(-p2(3), norm(p2(1:2)));
            az2 = atan2d(p2(2), p2(1));
            t2str = sprintf('r=%5.0fm el=%4.1f° az=%6.1f°', r2, el2, az2);
        end
        
        fprintf('  Scan %2d (t=%6.1fs): T1(747)=%d [%s] | T2(stealth)=%d [%s] | FA=%d\n', ...
            scanNum, simTime, nT1, t1str, nT2, t2str, nFalse);
        stepBufIdx = [];
        stepBufDets = {};
    end
end

fprintf('\n  TOTALS (raw sensor): 747=%d | stealth=%d | false=%d | scans=%d | steps=%d\n', ...
    totalT1, totalT2, totalFalse, scanNum, totalSteps);

%% ===== TEST 2: With RCS filter =====
fprintf('\n\n========= TEST 2: WITH RCS FILTER =========\n');

restart(scenario);
scanNum2 = 0; rcsKept1 = 0; rcsKept2 = 0; rcsDropTotal = 0;
stepBuf2 = {}; stepIdx2 = [];

while advance(scenario)
    simTime = scenario.SimulationTime;
    targets = targetPoses(tower);
    ins = pose(tower, 'true');
    
    [dets, ~, sensorCfg] = psr(targets, ins, simTime);
    
    if ~isempty(dets)
        nBefore = numel(dets);
        sPos = ins.Position(:)';
        try sPos = sPos + psr.MountingLocation(:)'; catch; end
        [dets, nRCS] = trackbench.environment.applyRCSFilter(dets, sPos, psr, targets, allPlats);
        rcsDropTotal = rcsDropTotal + nRCS;
        if nRCS > 0
            fprintf('    t=%6.1f: RCS filter dropped %d of %d\n', simTime, nRCS, nBefore);
        end
    end
    
    for dd = 1:numel(dets)
        stepIdx2(end+1) = getTargetIdx(dets{dd}); %#ok<AGROW>
    end
    
    if sensorCfg.IsScanDone
        scanNum2 = scanNum2 + 1;
        rcsKept1 = rcsKept1 + sum(stepIdx2 == 1);
        rcsKept2 = rcsKept2 + sum(stepIdx2 == 2);
        stepIdx2 = [];
    end
end

fprintf('\n  After RCS filter: 747=%d | stealth=%d | total dropped=%d\n', rcsKept1, rcsKept2, rcsDropTotal);

%% ===== TEST 3: Remove RCS signatures entirely =====
fprintf('\n\n========= TEST 3: NO RCS SIGNATURES =========\n');
fprintf('(Remove all rcsSignature from target platforms)\n\n');

for p = 2:numel(allPlats)
    allPlats{p}.Signatures = {};
    fprintf('  Cleared Signatures from Platform %d\n', p);
end

restart(scenario);
scanNum3 = 0; noSigT1 = 0; noSigT2 = 0; noSigFA = 0;
stepIdx3 = [];

while advance(scenario)
    simTime = scenario.SimulationTime;
    targets = targetPoses(tower);
    ins = pose(tower, 'true');
    
    [dets, ~, sensorCfg] = psr(targets, ins, simTime);
    
    for dd = 1:numel(dets)
        stepIdx3(end+1) = getTargetIdx(dets{dd}); %#ok<AGROW>
    end
    
    if sensorCfg.IsScanDone
        scanNum3 = scanNum3 + 1;
        n1 = sum(stepIdx3 == 1); n2 = sum(stepIdx3 == 2); nF = sum(stepIdx3 == 0);
        noSigT1 = noSigT1 + n1; noSigT2 = noSigT2 + n2; noSigFA = noSigFA + nF;
        
        t1str = ''; 
        if numel(targets) >= 1; t1str = sprintf('r=%.0fm', norm(targets(1).Position)); end
        
        fprintf('  Scan %2d (t=%6.1fs): T1(747)=%d [%s] | T2(stealth)=%d | FA=%d\n', ...
            scanNum3, simTime, n1, t1str, n2, nF);
        stepIdx3 = [];
    end
end

fprintf('\n  No-sig totals: 747=%d | stealth=%d | false=%d | scans=%d\n', ...
    noSigT1, noSigT2, noSigFA, scanNum3);

%% ===== SUMMARY =====
fprintf('\n\n=================== DIAGNOSIS SUMMARY ===================\n');
fprintf('  TEST 1 (raw sensor):     747=%-3d  stealth=%-3d  FA=%-3d\n', totalT1, totalT2, totalFalse);
fprintf('  TEST 2 (+ RCS filter):   747=%-3d  stealth=%-3d  (dropped %d)\n', rcsKept1, rcsKept2, rcsDropTotal);
fprintf('  TEST 3 (no signatures):  747=%-3d  stealth=%-3d  FA=%-3d\n', noSigT1, noSigT2, noSigFA);
fprintf('=========================================================\n');
if totalT1 < 10
    fprintf('  >> TEST 1 sparse: sensor itself not generating 747 detections.\n');
    if noSigT1 > totalT1 * 1.5
        fprintf('  >> TEST 3 has MORE: rcsSignature on 747 platform is HURTING detection!\n');
        fprintf('     MATLAB reads platform.Signatures natively and the pattern\n');
        fprintf('     is reducing internal Pd. Try removing rcs_profile from target JSON.\n');
    else
        fprintf('  >> TEST 3 similar: issue is beam geometry or scan pattern, not RCS.\n');
    end
elseif rcsKept1 < totalT1 * 0.7
    fprintf('  >> RCS filter is dropping >30%% of 747 detections. Check applyRCSFilter.\n');
else
    fprintf('  >> Detection counts look OK at all stages. Check tracker or visualization.\n');
end
fprintf('\n');


%% ========================================================================
function tgtIdx = getTargetIndexFromDet(det)
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
