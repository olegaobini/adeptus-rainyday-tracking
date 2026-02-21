function [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog] = helperRunTracker(dataLog,tracker,showTruth, showVisuals, animateVisuals)
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

if nargin < 4
    showVisuals = true;
end
if nargin < 5
    animateVisuals = true;
end

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
% Extract readable labels for plot tab title (purely cosmetic)
trackerType = class(tracker);
trackerType = trackerType(8:end);  % strip "tracker" prefix (e.g., "GNN")

% Robustly extract the filter model even if it's an anonymous function
filterFcnStr = func2str(tracker.FilterInitializationFcn);
if contains(filterFcnStr, 'IMM', 'IgnoreCase', true)
    filterType = 'IMM';
elseif contains(filterFcnStr, 'CV', 'IgnoreCase', true)
    filterType = 'CV';
else
    filterType = 'Custom';
end

plotTitle = ['Tracker: ',trackerType,' | Model: ',filterType];


if showVisuals
    %% Create Display (TABBED)
    tpaxes = tabbedAxes(plotTitle);
    title(tpaxes, plotTitle);
    
    % Force the default camera view to 3D
    view(tpaxes, 3); 
    
    tp = theaterPlot('Parent', tpaxes, ...
        'AxesUnits', ["km" "km" "km"], ...
        'XLimits', [-2000 2000], ...        
        'YLimits', [-20500 -17000]);        
    
    % Dictionaries to hold unique plotters and custom history lines
    trackPlotters = containers.Map('KeyType', 'double', 'ValueType', 'any');
    trackHistLines = containers.Map('KeyType', 'double', 'ValueType', 'any'); 
    trackColors = lines(20); % Color palette for tracks
    
    % Use animatedline to accumulate ALL detections without a "(history)" legend!
    detLine = animatedline(tpaxes, 'DisplayName', 'Detections', ...
        'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 8, ...
        'Color', [0 0.8 0], 'MaximumNumPoints', 1e6); % 1e6 ensures no points ever fade
end

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

%% Initialize static buffers if not animating
time = 0;
numSteps = numel(dataLog.Time);
i = 0;

% NEW: assignment log buffers (one row per assigned track at each scan)
logTime  = [];
logPlat  = [];
logTrack = [];
logTruth = [];

% NEW: default so variable always exists (even if no assignments happen)
assignLog = table([],[],[],[], 'VariableNames', {'Time','PlatformID','TrackID','TruthID'});


%% Initialize static buffers if not animating
if showVisuals && ~animateVisuals
    allStaticDets = cell(1, numSteps); % Buffer for detections
    staticTrackHist = struct();
end

% Loop until scenario ends OR the tab's axes gets deleted by user
while i < numSteps
    if showVisuals && ~isvalid(tpaxes)
        break;
    end

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

    % NEW: log assignments (for timeline plot)
    if ~isempty(trackIDs)
        nA = numel(trackIDs);

        % PlatformID for this scan: use sensor platform if available
        if isfield(dataLog,'SensorPlatformIDs') && numel(dataLog.SensorPlatformIDs) >= i
            thisPlat = dataLog.SensorPlatformIDs(i);
        else
            thisPlat = 1;
        end

        logTime  = [logTime;  repmat(simTime, nA, 1)]; %#ok<AGROW>
        logPlat  = [logPlat;  repmat(thisPlat, nA, 1)]; %#ok<AGROW>
        logTrack = [logTrack; double(trackIDs(:))]; %#ok<AGROW>
        logTruth = [logTruth; double(truthIDs(:))]; %#ok<AGROW>
    end


    % Step error metrics using the current assignment.
    tem(tracks, trackIDs, truths, truthIDs);

    %% Plotting: Detections + Tracks
    if showVisuals
        % --- Extract Detections ---
        if isempty(scanCells)
            meas = zeros(3,0);
        else
            allDets = vertcat(scanCells{:});
            if isempty(allDets)
                meas = zeros(3,0);
            else
                meas = cat(2, allDets.Measurement);
            end
        end

        % --- Extract Tracks ---
        [pos,cov] = getTrackPositions(tracks, ...
            [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);

        if animateVisuals
            % 1. Plot the green detections cumulatively
            if ~isempty(meas)
                addpoints(detLine, meas(1,:), meas(2,:), meas(3,:));
            end
            
            % 2. Iterate through current tracks to plot them individually
            for tIdx = 1:numel(tracks)
                tID = double(tracks(tIdx).TrackID);
                
                % Create a new, uniquely colored plotter if this is a new track
                if ~isKey(trackPlotters, tID)
                    colorIdx = mod(tID - 1, size(trackColors, 1)) + 1;
                    trkColor = trackColors(colorIdx, :);
                    
                    % Create track plotter with history OFF to avoid double legend
                    trackPlotters(tID) = trackPlotter(tp, ...
                        'DisplayName', sprintf('Track %d', tID), ...
                        'ConnectHistory', 'off', ...   
                        'Marker', 's', ...             
                        'MarkerSize', 8, ...
                        'MarkerFaceColor', trkColor, ...
                        'MarkerEdgeColor', 'k');
                        
                    % Create our own custom history line!
                    hLine = animatedline(tpaxes, ...
                        'Color', trkColor, ...
                        'LineWidth', 1.5, ...
                        'LineStyle', '-', ...
                        'MaximumNumPoints', 100); 
                        
                    % Completely hide this history line from the legend!
                    hLine.Annotation.LegendInformation.IconDisplayStyle = 'off';
                    
                    trackHistLines(tID) = hLine;
                end
                
                % Grab the plotter for this track and update it
                thisP = trackPlotters(tID);
                
                % Pass zeros(1,3) instead of vel to hide the black velocity lines
                thisP.plotTrack(pos(tIdx, :), zeros(1, 3), cov(:, :, tIdx), {num2str(tID)});
                
                % Grab our custom history line and add the new point
                addpoints(trackHistLines(tID), pos(tIdx, 1), pos(tIdx, 2), pos(tIdx, 3));
            end
            
            drawnow limitrate;
        else
            % Buffer data in memory for instant plotting later
            if ~isempty(meas)
                allStaticDets{i} = meas; 
            end
            for tIdx = 1:numel(tracks)
                tID = tracks(tIdx).TrackID;
                fName = sprintf('T%d', tID);
                if ~isfield(staticTrackHist, fName)
                    staticTrackHist.(fName) = [];
                end
                
                % Extract the row for this specific track and append it
                thisPos = pos(tIdx, :)'; 
                staticTrackHist.(fName) = [staticTrackHist.(fName), thisPos];
            end
        end
    end
end % <--- End of while loop

% If visuals are ON but animation is OFF, draw everything instantly at the end
if showVisuals && ~animateVisuals
    hold(tpaxes, 'on');
    view(tpaxes, 3); % Ensure 3D view is kept
    
    % Plot all buffered detections at once
    validDets = allStaticDets(~cellfun('isempty', allStaticDets)); 
    if ~isempty(validDets)
        finalDets = cat(2, validDets{:}); 
        plot3(tpaxes, finalDets(1,:), finalDets(2,:), finalDets(3,:), ...
            '.', 'Color', [0 0.8 0], 'MarkerSize', 8, 'DisplayName', 'Detections');
    end
    
    % Plot all buffered tracks at once
    fNames = fieldnames(staticTrackHist);
    cmap = lines(numel(fNames)); % Give each track a distinct color
    
    for i = 1:numel(fNames)
        tPos = staticTrackHist.(fNames{i});
        
        % Draw the line with the matching square markers
        plot3(tpaxes, tPos(1,:), tPos(2,:), tPos(3,:), ...
            '-s', 'LineWidth', 2, 'Color', cmap(i,:), ...
            'MarkerSize', 6, 'MarkerFaceColor', cmap(i,:), ...
            'MarkerEdgeColor', 'k', ...
            'DisplayName', ['Track ', fNames{i}(2:end)]);
        
        % Add the Track ID label at the end of the line
        if ~isempty(tPos)
            text(tpaxes, tPos(1,end), tPos(2,end), tPos(3,end), ...
                fNames{i}(2:end), 'FontSize', 10, 'FontWeight', 'bold', 'Color', cmap(i,:));
        end
    end
    drawnow;
end


% ---------------- NEW: finalize assignment log table ----------------
if ~isempty(logTime)
    assignLog = table(logTime, logPlat, logTrack, logTruth, ...
        'VariableNames', {'Time','PlatformID','TrackID','TruthID'});
else
    assignLog = table([],[],[],[], 'VariableNames', {'Time','PlatformID','TrackID','TruthID'});
end

% NEW: Assignment timeline plot in its own tab
if showVisuals
    axAssign = tabbedAxes(string(plotTitle) + " | Assignment");
    plotPlatformToTrackAssignment(axAssign, assignLog, "Platform to Track Assignment");
end
% -------------------------------------------------------------------

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
        meas = detsIn{ii}.Measurement(:);
        
        % Extract X and Y (always present)
        xPos = meas(1);
        yPos = meas(2);
        
        % Extract Z, or default to 0 if it is a 2D measurement
        if numel(meas) >= 3
            zPos = meas(3);
        else
            zPos = 0;
        end

        % Evaluate bounds without array resizing
        keep(ii) = (xPos >= roi.xMin && xPos <= roi.xMax) && ...
                   (yPos >= roi.yMin && yPos <= roi.yMax) && ...
                   (zPos >= roi.zMin && zPos <= roi.zMax);
    end
    
    detsOut = detsIn(keep);
end
