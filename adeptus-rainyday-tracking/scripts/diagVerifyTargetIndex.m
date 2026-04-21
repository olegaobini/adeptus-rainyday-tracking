%diagVerifyTargetIndex  Quick check: what TargetIndex values do detections actually have?
%
%  USAGE: clear classes; clear all; addpath("scripts"); diagVerifyTargetIndex

clc; close all;

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, "src")));

[scenario, config, sensors, metas] = trackbench.config.loadRunFile("range_rcs_test");

allPlats = scenario.Platforms;
tower = allPlats{1};
psr = tower.Sensors{1};

fprintf('\n--- Platform IDs ---\n');
for p = 1:numel(allPlats)
    fprintf('  Platform %d: PlatformID=%d\n', p, allPlats{p}.PlatformID);
end

fprintf('\n--- Target poses from tower ---\n');
restart(scenario); advance(scenario);
targets = targetPoses(tower);
for t = 1:numel(targets)
    fprintf('  targets(%d): PlatformID=%d, Position=[%.0f, %.0f, %.0f]\n', ...
        t, targets(t).PlatformID, targets(t).Position(1), targets(t).Position(2), targets(t).Position(3));
end

fprintf('\n--- Scanning for detections (first 5 scans) ---\n');
restart(scenario);
scanCount = 0;

while advance(scenario) && scanCount < 5
    simTime = scenario.SimulationTime;
    targets = targetPoses(tower);
    ins = pose(tower, 'true');
    [dets, ~, sensorCfg] = psr(targets, ins, simTime);
    
    for dd = 1:numel(dets)
        tgtIdx = 0;
        try
            attr = dets{dd}.ObjectAttributes;
            if iscell(attr); attr = attr{1}; end
            if isfield(attr, 'TargetIndex'); tgtIdx = attr.TargetIndex; end
        catch; end
        
        detPos = dets{dd}.Measurement(1:3);
        fprintf('  t=%.2f: TargetIndex=%d, Measurement=[%.0f, %.0f, %.0f]\n', ...
            simTime, tgtIdx, detPos(1), detPos(2), detPos(3));
    end
    
    if sensorCfg.IsScanDone
        scanCount = scanCount + 1;
    end
end

% Count with correct offset
fprintf('\n\n--- RE-COUNT with PlatformID offset ---\n');
restart(scenario);
tIdx_counts = containers.Map('KeyType','int32','ValueType','int32');

while advance(scenario)
    targets = targetPoses(tower);
    ins = pose(tower, 'true');
    simTime = scenario.SimulationTime;
    [dets, ~, ~] = psr(targets, ins, simTime);
    
    for dd = 1:numel(dets)
        tgtIdx = int32(0);
        try
            attr = dets{dd}.ObjectAttributes;
            if iscell(attr); attr = attr{1}; end
            if isfield(attr, 'TargetIndex'); tgtIdx = int32(attr.TargetIndex); end
        catch; end
        
        if tIdx_counts.isKey(tgtIdx)
            tIdx_counts(tgtIdx) = tIdx_counts(tgtIdx) + 1;
        else
            tIdx_counts(tgtIdx) = int32(1);
        end
    end
end

fprintf('\n  TargetIndex → Count:\n');
allKeys = keys(tIdx_counts);
for k = 1:numel(allKeys)
    idx = allKeys{k};
    fprintf('    TargetIndex=%d → %d detections', idx, tIdx_counts(idx));
    % Map to platform
    if idx > 0 && idx <= numel(allPlats)
        fprintf('  (Platform %d)', idx);
    end
    fprintf('\n');
end

fprintf('\n  Interpretation:\n');
fprintf('    TargetIndex=2 = Platform 2 = 747 (first target added)\n');
fprintf('    TargetIndex=3 = Platform 3 = Stealth (second target added)\n');
fprintf('    TargetIndex=0 = False alarm\n');
fprintf('\n  If TargetIndex=2 has detections, the 747 IS being detected!\n');
fprintf('  The bug is in TargetIndex interpretation, not detection generation.\n\n');
