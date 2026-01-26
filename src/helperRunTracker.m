function [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = helperRunTracker(dataLog,tracker,showTruth)
%helperRunTracker  Run a tracker on a logged detection sequence and compute metrics.
%
% PURPOSE
%   This helper is the "runner" for one tracker configuration (e.g., GNN+CV,
%   TOMHT+IMM, JPDA+CV, etc.). It:
%     1) Iterates through the logged detections scan-by-scan
%     2) Calls the tracker at each scan time
%     3) Updates assignment/error metrics against ground truth
%     4) Visualizes detections + tracks in a tabbed figure
%     5) Returns summary tables and cumulative metrics
%
% INPUTS
%   dataLog   : struct from helperRunDetections with fields:
%                - Time       : 1xN scan times (seconds)
%                - Detections : 1xN cell, each cell is scan detections (cell array)
%                - Truth      : truth target states per scan (platformPose array or similar)
%   tracker   : tracker object (trackerGNN / trackerTOMHT / trackerJPDA)
%               Note: validateattributes also allows 'numeric' for legacy/debug.
%   showTruth : boolean, if true plot truth trajectories after run
%
% OUTPUTS
%   trackSummary : table summarizing per-track continuity/ID behavior (from tam)
%   truthSummary : table summarizing per-truth continuity/coverage behavior (from tam)
%   trackMetrics : cumulative track error metrics (from tem)
%   truthMetrics : cumulative truth error metrics (from tem)
%   time         : total execution time spent in tracker(...) calls (seconds)
%
% DESIGN NOTES / "WHY" (read this if you're tuning things)
%   - Trackers operate on *batches* of detections at a scan time. Our detection
%     generator snaps all detections in a scan to the same simTime; this runner
%     assumes that is true (especially important for JPDA TimeTolerance).
%   - Under degraded conditions, clutter may explode. This helper includes:
%       (a) Adaptive ROI gating (optional) to suppress out-of-region clutter
%       (b) Optional hard cap on detections per scan to prevent runtime blow-up
%   - Metrics are stepped every scan regardless of whether detections exist.
%     This prevents "continuity crashes" when MaxUnreportedPeriod logic is used.
%
% DISPLAY NOTES
%   - Each call creates a NEW TAB in a single tabbedAxes popup window.
%   - This avoids spawning a new figure per run while keeping interactivity.
% -------------------------------------------------------------------------

% -------- Adaptive clutter gating settings --------
% enableAdaptiveROIGate:
%   If true, apply a simple measurement-space ROI gate when scans are "busy".
% gateThreshNumDets:
%   Only gate if more than this many detections exist in the scan.
enableAdaptiveROIGate = true;
gateThreshNumDets     = 3;

% ROI bounds (meters) for measurement-space box gate.
% These should match your scenario geometry and clutter generation region.
roi.xMin = -3000; roi.xMax =  3000;
roi.yMin = -22000; roi.yMax = -17000;
roi.zMin = -6000; roi.zMax = 0;
% ------------------------------------------------

% -------- Optional: cap detections per scan --------
% enableDetectionCap:
%   If true, limit the number of detections per scan to avoid worst-case runtime.
% maxDetsWhenBusy:
%   Only used if numel(dets) exceeds this value.
enableDetectionCap = true;
maxDetsWhenBusy    = 20;
% ---------------------------------------------------

% Validate tracker class so we catch mis-wiring early
validateattributes(tracker,{'trackerGNN','trackerTOMHT','trackerJPDA','numeric'},{},mfilename,'tracker');

% Extract readable labels for plot tab title (purely cosmetic)
trackerType = class(tracker);
trackerType = trackerType(8:end);  % strip "tracker" prefix (e.g., "GNN")
filterType = func2str(tracker.FilterInitializationFcn);
filterType = filterType(5:end-6);  % strip "@init" and "Filter" bits (hacky but readable)
plotTitle = ['Tracker: ',trackerType,'. Model: ',filterType];

%% Create Display (TABBED)
% tabbedAxes creates or reuses a single figure with tabs.
% Each run gets its own tab, so you can compare runs visually.
tpaxes = tabbedAxes(plotTitle);
title(tpaxes, plotTitle);

% theaterPlot is a convenient 2D/3D plotter for tracking scenarios.
% AxesUnits set to km for readability, but our measurements are in meters.
tp = theaterPlot('Parent', tpaxes, ...
    'AxesUnits', ["km" "km" "km"], ...
    'XLimits', [-2000 2000], ...        % meters
    'YLimits', [-20500 -17000]);        % meters

% Track plotter: draws track history and velocity arrows
trackP = trackPlotter(tp, ...
    'DisplayName','Tracks', ...
    'HistoryDepth',100, ...
    'ColorizeHistory','on', ...
    'ConnectHistory','on');

% Detection plotter: draws detection points and (optionally) uncertainty ellipses
detectionP = detectionPlotter(tp, ...
    'DisplayName','Detections', ...
    'MarkerSize',6, ...
    'MarkerFaceColor',[0.85 0.325 0.098], ...
    'MarkerEdgeColor','k', ...
    'History',1000);

%% Track Metrics (assignment + error)
% scanTime is inferred from dataLog.Time spacing.
% We use it to set MaxUnreportedPeriod to avoid false "breaks" due to small gaps.
scanTime = median(diff(dataLog.Time));
if isempty(scanTime) || ~isfinite(scanTime) || scanTime <= 0
    scanTime = 0.1;  % fallback if something is wrong with Time vector
end

% maxGap controls how long a truth/track can go without an assignment being
% considered "unreported". Larger tolerates missed detections; too large hides breaks.
maxGap = max(25*scanTime, 0.2);

% trackAssignmentMetrics (tam):
%   Tracks which track is assigned to which truth and detects events like divergence.
tam = trackAssignmentMetrics( ...
    'AssignmentThreshold', 300, ...       % distance threshold for assignment (units depend on truth/track format)
    'DivergenceThreshold', 350, ...       % when considered diverged
    'MaxUnreportedPeriod', maxGap);     % prevents continuity errors under missed detections

% trackErrorMetrics (tem):
%   Computes estimation errors (pos/vel error, RMSE-like metrics) for assigned pairs.
tem = trackErrorMetrics;

%% Run the tracker
% time accumulates only the time spent inside tracker(...) calls.
% This is a proxy for algorithm runtime (not including plotting/metrics overhead).
time = 0;
numSteps = numel(dataLog.Time);
i = 0;

% Loop until scenario ends OR the tab's axes gets deleted by user
while i < numSteps && isvalid(tpaxes)
    i = i + 1;

    % Scan time and detections for this step
    simTime    = dataLog.Time(i);
    scanBuffer = dataLog.Detections{i};

    % Normalize detections to a cell array of objectDetection objects
    % (Some upstream code paths may produce empty, cell, or object arrays.)
    if isempty(scanBuffer)
        scanCells = {};
    elseif iscell(scanBuffer)
        scanCells = scanBuffer(:);
    else
        scanCells = num2cell(scanBuffer(:));
    end

    % Adaptive ROI gate:
    % Only apply when scan is cluttered/busy. Reduces garbage detections
    % and helps keep tracker runtime bounded in degraded scenarios.
    if enableAdaptiveROIGate && numel(scanCells) > gateThreshNumDets
        scanCells = gateDetectionsROI(scanCells, roi);
    end

    % Optional detection cap:
    % Hard limit to avoid catastrophic runtime when clutter spikes.
    % NOTE: This is not "fair" scientifically unless applied consistently,
    % but it can be useful for debugging and preventing crashes.
    if enableDetectionCap && numel(scanCells) > maxDetsWhenBusy
        scanCells = scanCells(1:maxDetsWhenBusy);
    end

    % Update tracker with detections at this scan time.
    % Most tracker objects are callable: tracks = tracker(dets, time).
    tic
    tracks = tracker(scanCells, simTime);
    time = time + toc;

    % Truths for this scan.
    % dataLog.Truth is stored as platformPose array(s); we convert to the
    % struct format expected by trackAssignmentMetrics/trackErrorMetrics.
    targets = dataLog.Truth(:,i);

    % Wrap truths into struct array with fields:
    %   Time, PlatformID, Position (row), Velocity (row)
    % This avoids metrics tool errors caused by missing fields or wrong shapes.
    if isempty(targets)
        truths = struct('Time',{},'PlatformID',{},'Position',{},'Velocity',{});
    else
        truths = repmat(struct('Time', simTime, 'PlatformID', [], ...
            'Position', [0 0 0], 'Velocity', [0 0 0]), numel(targets), 1);

        for k = 1:numel(targets)
            truths(k).Time = simTime;

            % Prefer PlatformID if available; fallback to index
            if isprop(targets(k),'PlatformID')
                truths(k).PlatformID = targets(k).PlatformID;
            else
                truths(k).PlatformID = k;
            end

            % Ensure row vectors for metrics tools
            truths(k).Position = reshape(targets(k).Position, 1, []);
            if isprop(targets(k),'Velocity') && ~isempty(targets(k).Velocity)
                truths(k).Velocity = reshape(targets(k).Velocity, 1, []);
            else
                truths(k).Velocity = [0 0 0];
            end
        end
    end

    % ---- Metrics update (always step every scan) ----
    % Ensure tracks is an objectTrack array even when empty.
    if isempty(tracks)
        tracks = objectTrack.empty(0,1);
    end

    % Step assignment metrics
    step(tam, tracks, truths);

    % Use currentAssignment to map track IDs to truth IDs (for error metrics).
    [trackIDs, truthIDs] = currentAssignment(tam);

    % Step error metrics using the current assignment.
    tem(tracks, trackIDs, truths, truthIDs);

    %% Plotting: detections + tracks
    % Convert scanCells into an objectDetection array for plotting
    if isempty(scanCells)
        allDets = objectDetection.empty(0,1);
    else
        allDets = vertcat(scanCells{:});
    end

    % Extract measurements and measurement noise for detectionPlotter
    if isempty(allDets)
        meas = zeros(3,0);
        measCov = zeros(3,3,0);
    else
        meas = cat(2, allDets.Measurement);
        measCov = cat(3, allDets.MeasurementNoise);
    end
    detectionP.plotDetection(meas', measCov);

    % Extract track positions/velocities in XYZ using a selection matrix
    % (Assumes state ordering compatible with getTrackPositions/Velocities.)
    [pos,cov] = getTrackPositions(tracks, ...
        [1 0 0 0 0 0; ...
         0 0 1 0 0 0; ...
         0 0 0 0 1 0]);

    [vel,~] = getTrackVelocities(tracks, ...
        [0 1 0 0 0 0; ...
         0 0 0 1 0 0; ...
         0 0 0 0 0 1]);

    % Label tracks by TrackID for visual debugging
    labels = arrayfun(@(x)num2str(x.TrackID), tracks, 'UniformOutput', false);
    trackP.plotTrack(pos, vel, cov, labels);

    drawnow
end

%% Optional: plot truth trajectories (after run)
% Useful for visual sanity checks, but disabled during metric-only runs.
if showTruth
    trajectoryP = trajectoryPlotter(tp,'DisplayName','Trajectory');
    trajPos{1} = vertcat(dataLog.Truth(1,:).Position);
    trajPos{2} = vertcat(dataLog.Truth(2,:).Position);
    trajectoryP.plotTrajectory(trajPos);
end

%% Output metric tables
% Convert metric objects into tables for the main script to print/report.
trackSummary = trackMetricsTable(tam);
truthSummary = truthMetricsTable(tam);
trackMetrics = cumulativeTrackMetrics(tem);
truthMetrics = cumulativeTruthMetrics(tem);

% --- CUSTOM METRIC: Quality Score (0-100%) ---
% PURPOSE: 
%   Convert the raw "Root Mean Square Error" (RMSE) in meters into a 
%   human-readable percentage "grade" for the track.
%   - 100% Score = Perfect tracking (0 meters error).
%   - 0% Score   = Error exceeds the tolerance threshold.

% Define the "Failure Threshold" in meters.
% If a track's average error is >300m, it is considered a 0% quality track.
tolerance = 300; 

% Initialize a string array to hold the formatted scores (e.g., "95.2%")
scoreStrings = strings(height(trackMetrics), 1); 

% 1. Identify the Data Source
% We are looking for the 'posRMS' column in the metrics table.
% (Ensure this column name matches your specific MATLAB version's output)
rmseCol = 'posRMS';

% 2. Calculate Scores for Each Track
for k = 1:height(trackMetrics)
    % Safety Check: Ensure we actually found the column before trying to read it
    if isempty(rmseCol)
        scoreStrings(k) = "Error (Col Not Found)";
    else
        % Extract the RMSE value for the current track
        rmse = trackMetrics.(rmseCol)(k);
        
        % Handle "Ghost" or Unassigned Tracks
        % If RMSE is NaN, it means this track was never associated with a 
        % ground truth object (it tracked clutter or nothing).
        if isnan(rmse)
            scoreStrings(k) = "0.0% (Unassigned)";
        else
            % LINEAR SCORING FORMULA:
            % Score = 1.0 - (Current Error / Max Tolerable Error)
            val = 1 - (rmse / tolerance);
            
            % Clamp the result to ensure it stays between 0.0 and 1.0
            % (Prevents negative scores if error > tolerance)
            finalScore = max(0, min(1, val)) * 100;
            
            % Format as a percentage string with 1 decimal place
            scoreStrings(k) = sprintf("%.1f%%", finalScore);
        end
    end
end

% 3. Add to Table
% Convert to categorical so it prints without quotes
trackMetrics.Quality = categorical(scoreStrings);
% ---------------------------------------------

% Remove columns we don't use in our report to keep tables readable.
trVarsToRemove = {'DivergenceCount','DeletionStatus','DeletionLength','DivergenceLength', ...
    'RedundancyStatus','RedundancyCount','RedundancyLength','FalseTrackLength', ...
    'FalseTrackStatus','SwapCount'};
trackSummary = removevars(trackSummary,trVarsToRemove);

tuVarsToRemove = {'DeletionStatus','BreakStatus','BreakLength','InCoverageArea','EstablishmentStatus'};
truthSummary = removevars(truthSummary,tuVarsToRemove);
end

function detsOut = gateDetectionsROI(detsIn, roi)
%gateDetectionsROI  Simple ROI gate in measurement space.
%
% PURPOSE
%   Keep only detections whose (x,y,z) measurements fall inside the ROI bounds.
%   This is a pragmatic clutter limiter when scans get busy (many detections).
%
% INPUTS
%   detsIn : cell array of objectDetection
%   roi    : struct with fields xMin/xMax/yMin/yMax/zMin/zMax
%
% OUTPUT
%   detsOut : subset of detsIn inside ROI
%
% NOTE
%   This is not a statistical gate (no Mahalanobis distance).
%   If the scenario geometry changes, these bounds must be updated.

    if isempty(detsIn)
        detsOut = detsIn;
        return;
    end

    keep = false(numel(detsIn),1);
    for ii = 1:numel(detsIn)
        z = detsIn{ii}.Measurement(:);

        % Some dets may be 2D; pad to 3D so bounds checks don't crash.
        if numel(z) == 2
            z = [z; 0];
        end

        keep(ii) = (z(1) >= roi.xMin && z(1) <= roi.xMax) && ...
                   (z(2) >= roi.yMin && z(2) <= roi.yMax) && ...
                   (z(3) >= roi.zMin && z(3) <= roi.zMax);
    end
    detsOut = detsIn(keep);
end
