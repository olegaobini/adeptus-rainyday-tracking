function plotScenarioAndDetections(dataLog)
%plotScenarioAndDetections Plot truth trajectories and detections from dataLog
%
% This version is robust when dataLog.Detections is a cell array where each
% element is a cell array of objectDetection objects (as produced by the
% modified helperRunDetections).

figure; hold on; grid on;
title('Scenario Truth and Detections');
xlabel('X (m)'); ylabel('Y (m)');

% ---- Plot Truth (all targets over time) ----
% dataLog.Truth is typically an array of platformPose structs
% with fields Position (3x1) and PlatformID (scalar).
truthPos = [];
truthID  = [];

if ~isempty(dataLog.Truth)
    for k = 1:numel(dataLog.Truth)
        tp = dataLog.Truth(k);

        if isfield(tp, 'Position')
            truthPos(:,end+1) = tp.Position(:); %#ok<AGROW>
        else
            continue;
        end

        if isfield(tp, 'PlatformID')
            truthID(end+1) = tp.PlatformID; %#ok<AGROW>
        else
            truthID(end+1) = NaN; %#ok<AGROW>
        end
    end

    % If we have IDs, plot each target separately
    if ~all(isnan(truthID))
        uIDs = unique(truthID(~isnan(truthID)));
        for i = 1:numel(uIDs)
            idx = (truthID == uIDs(i));
            plot(truthPos(1,idx), truthPos(2,idx), '-', 'LineWidth', 1.5);
        end
    else
        % Fallback: plot all truth points as one trajectory
        plot(truthPos(1,:), truthPos(2,:), '-', 'LineWidth', 1.5);
    end
end

% ---- Flatten detections across scans into a single cell array ----
% dataLog.Detections is 1xN cell, each cell contains a scanBuffer (cell array)
allDetCells = {};
if ~isempty(dataLog.Detections)
    for k = 1:numel(dataLog.Detections)
        scan = dataLog.Detections{k}; % expected: cell array of objectDetection
        if isempty(scan)
            continue;
        end
        % Ensure scan is a column cell array, then append
        scan = scan(:);
        allDetCells = [allDetCells; scan]; %#ok<AGROW>
    end
end

% ---- Extract detection measurements robustly ----
if isempty(allDetCells)
    legend({'Truth'}, 'Location', 'best');
    axis equal;
    return;
end

% Extract Measurement (should be 3x1 for DetectionCoordinates='Scenario')
meas = cell2mat(cellfun(@(d) d.Measurement(:), allDetCells, 'UniformOutput', false));

% Plot detections as points (x vs y)
plot(meas(1,:), meas(2,:), '.', 'MarkerSize', 10);

legend({'Truth', 'Detections'}, 'Location', 'best');
axis equal;
end
