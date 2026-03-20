function [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = runTracker(dataLog,tracker,showTruth, showVisuals, animateVisuals)
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
%                - Time          : 1xN scan times (seconds)
%                - Detections    : 1xN cell, each cell is scan detections (cell array)
%                - Truth         : truth target states per scan (platformPose array or similar)
%                - SensorConfig  : 1xN cell, each cell is sensor config(s) for that scan
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
%   assignLog    : table of per-scan track-to-truth assignments with sensor info
%
% DESIGN NOTES / "WHY" (read this if you're tuning things)
%   - Trackers operate on *batches* of detections at a scan time. Our detection
%     generator snaps all detections in a scan to the same simTime; this runner
%     assumes that is true (especially important for JPDA TimeTolerance).
%   - All detections (including clutter and outliers) are passed directly
%     to the tracker with no pre-filtering. The tracker's own gating logic
%     (AssignmentGate / ClutterDensity) handles clutter rejection. This
%     gives an honest picture of tracker performance under degradation.
%   - Metrics are stepped every scan regardless of whether detections exist.
%     This prevents "continuity crashes" when MaxUnreportedPeriod logic is used.
%   - HasDetectableTrackIDsInput: GNN and JPDA trackers are built with this flag.
%     Each scan we compute which tracks fall inside at least one sensor's FOV
%     (using the sensor configs stored in dataLog.SensorConfig) and pass only
%     those IDs to the tracker. Tracks outside the FOV coast on Kalman prediction
%     without any score penalty, preventing false deletions during beam gaps.
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

% ROI bounds (meters) computed from truth positions — used ONLY for plot
% axis limits (theaterPlot), never for filtering detections. The tracker's
% own gating logic (AssignmentGate) handles clutter rejection.
roi = computeDynamicROI(dataLog);

% -------- Detectable track ID support --------
% Determine whether this tracker was built with HasDetectableTrackIDsInput.
% If so, we compute and pass detectable IDs each scan to prevent false
% track deletions when targets exit the sensor FOV.
useDetectableIDs = false;
try
    if isprop(tracker, 'HasDetectableTrackIDsInput') && tracker.HasDetectableTrackIDsInput
        useDetectableIDs = true;
    end
catch
end
hasSensorConfigs = isfield(dataLog, 'SensorConfig') && ~isempty(dataLog.SensorConfig);

% If a rotating sensor exists (360 sweep), ALL tracks are detectable every
% scan — the rotator sees everything in range. Skip the FOV check to avoid
% false "not detectable" results caused by incomplete sensor config storage
% (e.g., only the fire-control's 1.4 FOV is stored, not the PSR's 360).
hasRotator = isfield(dataLog, 'HasRotator') && dataLog.HasRotator;
if useDetectableIDs && hasRotator
    fprintf('[runTracker] Rotating sensor present — all tracks detectable, skipping FOV gate.\n');
    useDetectableIDs = false;
end

% If the tracker wants detectable IDs but we have no configs, warn once and fall back.
if useDetectableIDs && ~hasSensorConfigs
    warning('helperRunTracker:noSensorConfig', ...
        ['Tracker has HasDetectableTrackIDsInput=true but dataLog.SensorConfig ' ...
         'is missing. Falling back to passing all tracks as detectable. ' ...
         'Re-run runDetections to regenerate dataLog with SensorConfig.']);
    useDetectableIDs = false;
end
% ---------------------------------------------

% Validate tracker class so we catch mis-wiring early
validateattributes(tracker,{'trackerGNN','trackerTOMHT','trackerJPDA','numeric'},{},mfilename,'tracker');

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
    tpaxes = trackbench.reporting.tabbedAxes(plotTitle);
    title(tpaxes, plotTitle);
    
    % Force the default camera view to 3D
    view(tpaxes, 3); 
    
    % Use dynamic ROI so the view always fits the full scenario
    tp = theaterPlot('Parent', tpaxes, ...
        'AxesUnits', ["km" "km" "km"], ...
        'XLimits', [roi.xMin roi.xMax], ...
        'YLimits', [roi.yMin roi.yMax], ...
        'ZLimits', [roi.zMin roi.zMax]);
    
    % NED Z-axis fix: theaterPlot overrides ZDir='reverse', so we
    % negate all Z values manually when plotting. Altitude displays upward.
    zlabel(tpaxes, 'Altitude (km)');
    
    % Draw terrain surface or flat ground plane
    % All Z values are NEGATED for display (NED z-down -> altitude up)
    if isfield(dataLog, 'TerrainGrid') && ~isempty(dataLog.TerrainGrid)
        tg = dataLog.TerrainGrid;
        Xterr = tg.X;  Yterr = tg.Y;
        Zterr = -tg.Z;  % NEGATE: NED z-down -> altitude up for display
        % Plot full terrain — no clipping (ROI already includes terrain bounds)
        surf(tpaxes, Xterr, Yterr, Zterr, ...
            'FaceColor', 'interp', 'EdgeColor', 'none', ...
            'FaceAlpha', 0.45, 'HandleVisibility', 'off');
        colormap(tpaxes, [0.15 0.12 0.08; 0.25 0.20 0.12; ...
                          0.35 0.30 0.15; 0.45 0.40 0.20; ...
                          0.55 0.50 0.30; 0.65 0.60 0.40]);
    else
        gx = [roi.xMin roi.xMax roi.xMax roi.xMin];
        gy = [roi.yMin roi.yMin roi.yMax roi.yMax];
        gz = [0 0 0 0];
        fill3(tpaxes, gx, gy, gz, [0.25 0.20 0.15], ...
            'FaceAlpha', 0.25, 'EdgeColor', [0.4 0.35 0.3], ...
            'EdgeAlpha', 0.4, 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
    end
    
    % Dictionaries to hold unique plotters and custom history lines
    trackPlotters = containers.Map('KeyType', 'double', 'ValueType', 'any');
    trackHistLines = containers.Map('KeyType', 'double', 'ValueType', 'any'); 
    trackColors = lines(20); % Color palette for tracks
    
    % Use animatedline to accumulate ALL detections without a "(history)" legend!
    detLine = animatedline(tpaxes, 'DisplayName', 'Detections', ...
        'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 8, ...
        'Color', [0 0.8 0], 'MaximumNumPoints', 1e6);

    % Draw sensor coverage (background layer — range rings and sectors)
    % theaterPlot axes use meters internally, so scaleKm=false
    if isfield(dataLog, 'SensorCoverage') && ~isempty(dataLog.SensorCoverage)
        trackbench.reporting.drawSensorCoverage(tpaxes, dataLog.SensorCoverage, false);
    end
    
    % Re-apply axis labels AFTER theaterPlot setup (theaterPlot overwrites them)
    xlabel(tpaxes, 'X (km)');
    ylabel(tpaxes, 'Y (km)');
    zlabel(tpaxes, 'Altitude (km)');
    
    % Zoom to match the Scenario Truth and Detections plot.
    % theaterPlot AxesUnits=["km"] labels ticks in km but internal
    % coordinates remain in METERS. So xlim/ylim/zlim take meters.
    padM = 3000;  % 3km padding in meters (matches plotInitialScenario)
    allTruthPos = [];
    for ss = 1:size(dataLog.Truth, 2)
        targets = dataLog.Truth(:, ss);
        for kk = 1:numel(targets)
            allTruthPos(:, end+1) = targets(kk).Position(:); %#ok<AGROW>
        end
    end
    if ~isempty(allTruthPos)
        xLimFit = [min(allTruthPos(1,:))-padM, max(allTruthPos(1,:))+padM];
        yLimFit = [min(allTruthPos(2,:))-padM, max(allTruthPos(2,:))+padM];
        % Z: negate for altitude display (NED z-down → positive altitude up)
        zAlt = -allTruthPos(3,:);  % flip sign: NED negative → positive altitude
        % Always include ground level (0) so terrain is visible,
        % with a small buffer below for terrain peaks.
        zLimFit = [min(-500, min(zAlt)-padM), max(zAlt)+padM];
        xlim(tpaxes, xLimFit);
        ylim(tpaxes, yLimFit);
        zlim(tpaxes, zLimFit);
    end
    set(tpaxes, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
end

%% Track Metrics (assignment + error)
% scanTime is inferred from dataLog.Time spacing.
% We use it to set MaxUnreportedPeriod to avoid false "breaks" due to small gaps.
scanTime = median(diff(dataLog.Time));
if isempty(scanTime) || ~isfinite(scanTime) || scanTime <= 0
    scanTime = 0.1;  % fallback if something is wrong with Time vector
end

% maxGap controls how long a truth/track can go without an assignment being
% considered "unreported". Use the full scenario time span so dead-zone
% gaps never crash the metrics. BreakCount still correctly counts re-acquisitions.
scenarioSpan = dataLog.Time(end) - dataLog.Time(1);
maxGap = max(scenarioSpan, 25*scanTime);

% trackAssignmentMetrics (tam):
%   Tracks which track is assigned to which truth and detects events like divergence.
tam = trackAssignmentMetrics( ...
    'AssignmentThreshold', 300, ...
    'DivergenceThreshold', 350, ...
    'MaxUnreportedPeriod', maxGap);

% trackErrorMetrics (tem):
%   Computes estimation errors (pos/vel error, RMSE-like metrics) for assigned pairs.
tem = trackErrorMetrics;

%% Initialize buffers
time = 0;
numSteps = numel(dataLog.Time);
i = 0;

% Assignment log buffers (one row per assigned track at each scan)
logTime  = [];
logPlat  = [];
logTrack = [];
logTruth = [];
logSensors = strings(0,1);

% Default so variable always exists (even if no assignments happen)
assignLog = table([],[],[],[],strings(0,1), 'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});

% Static buffers if not animating
if showVisuals && ~animateVisuals
    allStaticDets = cell(1, numSteps);
    staticTrackHist = struct();
end

% allTracks: needed for predictTracksToTime when using detectable IDs
allTracks = objectTrack.empty(0,1);

%% Main loop
while i < numSteps
    if showVisuals && ~isvalid(tpaxes)
        break;
    end

    i = i + 1;

    % Scan time and detections for this step
    simTime    = dataLog.Time(i);
    scanBuffer = dataLog.Detections{i};

    % Normalize detections to a cell array of objectDetection objects
    if isempty(scanBuffer)
        scanCells = {};
    elseif iscell(scanBuffer)
        scanCells = scanBuffer(:);
    else
        scanCells = num2cell(scanBuffer(:));
    end

    % -------- Normalize mixed measurement dimensions --------
    % Multi-sensor scenarios produce mixed 3/6-element detections.
    % TOMHT crashes on mixed sizes. Normalize all to position-only (3-element).
    scanCells = normalizeDetectionDimensions(scanCells);

    % No pre-filtering of detections. The tracker's own gating logic
    % (AssignmentGate / ClutterDensity) handles clutter rejection.
    % All detections pass through to give an honest performance picture.

    % -------- Compute detectable track IDs --------
    if useDetectableIDs && isLocked(tracker)
        try
            predictedTracks = predictTracksToTime(tracker, 'all', simTime);
            scanConfigs = getScanConfigs(dataLog, i);
            detectIDs = getDetectableTrackIDs(predictedTracks, scanConfigs);
        catch ME
            warning('helperRunTracker:detectableFailed', ...
                'getDetectableTrackIDs failed at t=%.2f: %s. Passing all IDs.', simTime, ME.message);
            detectIDs = uint32([allTracks.TrackID]');
        end
    elseif isLocked(tracker) && ~isempty(allTracks)
        detectIDs = uint32([allTracks.TrackID]');
    else
        detectIDs = uint32([]);
    end

    % Update tracker with detections at this scan time.
    tic
    trackerWants3Args = isprop(tracker, 'HasDetectableTrackIDsInput') && tracker.HasDetectableTrackIDsInput;
    if trackerWants3Args
        [tracks, ~, allTracks] = tracker(scanCells, simTime, detectIDs);
    else
        tracks = tracker(scanCells, simTime);
    end
    time = time + toc;

    % Truths for this scan.
    targets = dataLog.Truth(:,i);

    % Wrap truths into struct array with fields:
    %   Time, PlatformID, Position (row), Velocity (row)
    if isempty(targets)
        truths = struct('Time',{},'PlatformID',{},'Position',{},'Velocity',{});
    else
        truths = repmat(struct('Time', simTime, 'PlatformID', [], ...
            'Position', [0 0 0], 'Velocity', [0 0 0]), numel(targets), 1);

        for k = 1:numel(targets)
            truths(k).Time = simTime;

            if isprop(targets(k),'PlatformID')
                truths(k).PlatformID = targets(k).PlatformID;
            else
                truths(k).PlatformID = k;
            end

            truths(k).Position = reshape(targets(k).Position, 1, []);
            if isprop(targets(k),'Velocity') && ~isempty(targets(k).Velocity)
                truths(k).Velocity = reshape(targets(k).Velocity, 1, []);
            else
                truths(k).Velocity = [0 0 0];
            end
        end
    end

    % ---- Metrics update (always step every scan) ----
    if isempty(tracks)
        tracks = objectTrack.empty(0,1);
    end

    step(tam, tracks, truths);
    [trackIDs, truthIDs] = currentAssignment(tam);

    % Log assignments (for timeline plot)
    if ~isempty(trackIDs)
        nA = numel(trackIDs);

        if isfield(dataLog,'SensorPlatformIDs') && numel(dataLog.SensorPlatformIDs) >= i
            thisPlat = dataLog.SensorPlatformIDs(i);
        else
            thisPlat = 1;
        end

        if isfield(dataLog,'ScanSensorIndices') && numel(dataLog.ScanSensorIndices) >= i
            sset = dataLog.ScanSensorIndices{i};
            if isempty(sset)
                sensorsUsedStr = "";
            else
                sensorsUsedStr = strjoin(string(sset(:).'), ',');
            end
        else
            sensorsUsedStr = "";
        end

        logTime  = [logTime;  repmat(simTime, nA, 1)]; %#ok<AGROW>
        logPlat  = [logPlat;  repmat(thisPlat, nA, 1)]; %#ok<AGROW>
        logTrack = [logTrack; double(trackIDs(:))]; %#ok<AGROW>
        logTruth = [logTruth; double(truthIDs(:))]; %#ok<AGROW>
        logSensors = [logSensors; repmat(string(sensorsUsedStr), nA, 1)]; %#ok<AGROW>
    end

    % Step error metrics
    tem(tracks, trackIDs, truths, truthIDs);

    %% Plotting: Detections + Tracks
    if showVisuals
        % --- Extract Detections (position only, first 3 elements) ---
        if isempty(scanCells)
            meas = zeros(3,0);
        else
            meas = zeros(3, numel(scanCells));
            for jj = 1:numel(scanCells)
                m = scanCells{jj}.Measurement(:);
                meas(:,jj) = m(1:3);
            end
        end

        % --- Extract Tracks ---
        [pos,cov] = getTrackPositions(tracks, ...
            [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);

        if animateVisuals
            % Plot green detections cumulatively (negate Z: NED -> altitude up)
            if ~isempty(meas)
                addpoints(detLine, meas(1,:), meas(2,:), -meas(3,:));
            end
            
            % Iterate through current tracks
            for tIdx = 1:numel(tracks)
                tID = double(tracks(tIdx).TrackID);
                
                if ~isKey(trackPlotters, tID)
                    colorIdx = mod(tID - 1, size(trackColors, 1)) + 1;
                    trkColor = trackColors(colorIdx, :);
                    
                    trackPlotters(tID) = trackPlotter(tp, ...
                        'DisplayName', sprintf('Track %d', tID), ...
                        'ConnectHistory', 'off', ...   
                        'Marker', 's', ...             
                        'MarkerSize', 8, ...
                        'MarkerFaceColor', trkColor, ...
                        'MarkerEdgeColor', 'k');
                        
                    hLine = animatedline(tpaxes, ...
                        'Color', trkColor, ...
                        'LineWidth', 1.5, ...
                        'LineStyle', '-', ...
                        'MaximumNumPoints', 100); 
                    hLine.Annotation.LegendInformation.IconDisplayStyle = 'off';
                    trackHistLines(tID) = hLine;
                end
                
                thisP = trackPlotters(tID);
                % Negate Z for display (NED z-down -> altitude up)
                displayPos = pos(tIdx, :);
                displayPos(3) = -displayPos(3);
                displayCov = cov(:, :, tIdx);
                thisP.plotTrack(displayPos, zeros(1, 3), displayCov, {num2str(tID)});
                addpoints(trackHistLines(tID), displayPos(1), displayPos(2), displayPos(3));
            end
            
            drawnow limitrate;
        else
            % Buffer data for static plot (negate Z: NED -> altitude up)
            if ~isempty(meas)
                measDisp = meas;
                measDisp(3,:) = -measDisp(3,:);
                allStaticDets{i} = measDisp; 
            end
            for tIdx = 1:numel(tracks)
                tID = tracks(tIdx).TrackID;
                fName = sprintf('T%d', tID);
                if ~isfield(staticTrackHist, fName)
                    staticTrackHist.(fName) = [];
                end
                thisPos = pos(tIdx, :)';
                thisPos(3) = -thisPos(3);
                staticTrackHist.(fName) = [staticTrackHist.(fName), thisPos];
            end
        end
    end
end % <--- End of while loop

% If visuals are ON but animation is OFF, draw everything at the end
if showVisuals && ~animateVisuals
    hold(tpaxes, 'on');
    view(tpaxes, 3);
    
    validDets = allStaticDets(~cellfun('isempty', allStaticDets)); 
    if ~isempty(validDets)
        finalDets = cat(2, validDets{:}); 
        plot3(tpaxes, finalDets(1,:), finalDets(2,:), finalDets(3,:), ...
            '.', 'Color', [0 0.8 0], 'MarkerSize', 8, 'DisplayName', 'Detections');
    end
    
    fNames = fieldnames(staticTrackHist);
    cmap = lines(numel(fNames));
    
    for i = 1:numel(fNames)
        tPos = staticTrackHist.(fNames{i});
        plot3(tpaxes, tPos(1,:), tPos(2,:), tPos(3,:), ...
            '-s', 'LineWidth', 2, 'Color', cmap(i,:), ...
            'MarkerSize', 6, 'MarkerFaceColor', cmap(i,:), ...
            'MarkerEdgeColor', 'k', ...
            'DisplayName', ['Track ', fNames{i}(2:end)]);
        if ~isempty(tPos)
            text(tpaxes, tPos(1,end), tPos(2,end), tPos(3,end), ...
                fNames{i}(2:end), 'FontSize', 10, 'FontWeight', 'bold', 'Color', cmap(i,:));
        end
    end
    drawnow;
end

% ---------------- Finalize assignment log table ----------------
if ~isempty(logTime)
    assignLog = table(logTime, logPlat, logTrack, logTruth, logSensors, ...
        'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});
else
    assignLog = table([],[],[],[],strings(0,1), 'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});
end

% ---- Track Swap Analysis ----
swapReport = trackbench.analysis.analyzeTrackSwaps(assignLog, dataLog.Truth, dataLog.Time);

% Assignment timeline plot in its own tab (with swap overlay)
if showVisuals
    axAssign = trackbench.reporting.tabbedAxes(string(plotTitle) + " | Assignment");
    trackbench.reporting.plotPlatformToTrackAssignment(axAssign, assignLog, "Platform to Track Assignment", swapReport);

    % Standalone swap analysis figure (only when swaps detected)
    if ~swapReport.swapFree
        trackbench.reporting.plotTrackSwapAnalysis(swapReport, assignLog);
    end
end

%% Optional: plot truth trajectories (after run)
if showTruth
    trajectoryP = trajectoryPlotter(tp,'DisplayName','Trajectory');
    trajPos{1} = vertcat(dataLog.Truth(1,:).Position);
    trajPos{2} = vertcat(dataLog.Truth(2,:).Position);
    trajectoryP.plotTrajectory(trajPos);
end

%% Output metric tables
trackSummary = trackMetricsTable(tam);
truthSummary = truthMetricsTable(tam);
trackMetrics = cumulativeTrackMetrics(tem);
truthMetrics = cumulativeTruthMetrics(tem);

% --- CUSTOM METRIC: Quality Score (0-100%) ---
tolerance = 300; 
scoreStrings = strings(height(trackMetrics), 1); 
rmseCol = 'posRMS';

for k = 1:height(trackMetrics)
    if isempty(rmseCol)
        scoreStrings(k) = "Error (Col Not Found)";
    else
        rmse = trackMetrics.(rmseCol)(k);
        if isnan(rmse)
            scoreStrings(k) = "0.0% (Unassigned)";
        else
            val = 1 - (rmse / tolerance);
            finalScore = max(0, min(1, val)) * 100;
            scoreStrings(k) = sprintf("%.1f%%", finalScore);
        end
    end
end
trackMetrics.Quality = categorical(scoreStrings);

% Remove columns we don't use in our report
trVarsToRemove = {'DivergenceCount','DeletionStatus','DeletionLength','DivergenceLength', ...
    'RedundancyStatus','RedundancyCount','RedundancyLength','FalseTrackLength', ...
    'FalseTrackStatus'};
trackSummary = removevars(trackSummary,trVarsToRemove);

tuVarsToRemove = {'DeletionStatus','BreakStatus','BreakLength','InCoverageArea','EstablishmentStatus'};
truthSummary = removevars(truthSummary,tuVarsToRemove);
end

%% ========================================================================
%                         LOCAL HELPER FUNCTIONS
%% ========================================================================

function roi = computeDynamicROI(dataLog)
%computeDynamicROI  Compute ROI bounding box from truth positions AND terrain grid.
    pad = 6000;
    allPos = [];
    for s = 1:size(dataLog.Truth, 2)
        targets = dataLog.Truth(:, s);
        for k = 1:numel(targets)
            p = targets(k).Position(:)';
            allPos = [allPos; p]; %#ok<AGROW>
        end
    end
    if isempty(allPos)
        roi.xMin = -50000; roi.xMax = 50000;
        roi.yMin = -50000; roi.yMax = 50000;
        roi.zMin = -500;   roi.zMax = 20000;
    else
        roi.xMin = min(allPos(:,1)) - pad;
        roi.xMax = max(allPos(:,1)) + pad;
        roi.yMin = min(allPos(:,2)) - pad;
        roi.yMax = max(allPos(:,2)) + pad;
        altitudes = -allPos(:,3);
        roi.zMin = min(altitudes) - pad;
        roi.zMax = max(altitudes) + pad;
        roi.zMin = min(roi.zMin, -500);
    end

    % Expand ROI to include terrain grid so mountains fill the entire plot.
    % boundary is [xMin xMax; yMin yMax] — use explicit (row,col) indexing
    % because MATLAB linear indexing is column-major and gets it wrong.
    if isfield(dataLog, 'TerrainGrid') && ~isempty(dataLog.TerrainGrid)
        tg = dataLog.TerrainGrid;
        if isfield(tg, 'boundary') && numel(tg.boundary) >= 4
            roi.xMin = min(roi.xMin, tg.boundary(1,1));
            roi.xMax = max(roi.xMax, tg.boundary(1,2));
            roi.yMin = min(roi.yMin, tg.boundary(2,1));
            roi.yMax = max(roi.yMax, tg.boundary(2,2));
        elseif isfield(tg, 'X')
            roi.xMin = min(roi.xMin, min(tg.X(:)));
            roi.xMax = max(roi.xMax, max(tg.X(:)));
            roi.yMin = min(roi.yMin, min(tg.Y(:)));
            roi.yMax = max(roi.yMax, max(tg.Y(:)));
        end
    end

    % Cap altitude at 15km — PSR elevation noise produces 60km+ outliers
    roi.zMax = min(max(roi.zMax, 8000), 15000);
end

function configs = getScanConfigs(dataLog, scanIdx)
    raw = dataLog.SensorConfig{scanIdx};
    if iscell(raw)
        configs = raw;
    else
        configs = {raw};
    end
end

function detectableIDs = getDetectableTrackIDs(predictedTracks, configs)
    detectableIDs = uint32([]);
    if isempty(predictedTracks) || isempty(configs)
        return;
    end
    for iTrk = 1:numel(predictedTracks)
        posWorld = predictedTracks(iTrk).State(1:2:end);
        posWorld = posWorld(:);
        inFOV = false;
        for iSens = 1:numel(configs)
            cfg = configs{iSens};
            if isfield(cfg, 'IsValidTime') && ~cfg.IsValidTime
                continue;
            end
            posSensor = transformToSensorFrame(posWorld, cfg);
            [az, el] = cart2sph(posSensor(1), posSensor(2), posSensor(3));
            az = rad2deg(az);
            el = rad2deg(el);
            if isfield(cfg, 'FieldOfView') && numel(cfg.FieldOfView) >= 2
                fov = cfg.FieldOfView;
                if abs(az) <= fov(1)/2 && abs(el) <= fov(2)/2
                    inFOV = true;
                    break;
                end
            else
                inFOV = true;
                break;
            end
        end
        if inFOV
            detectableIDs(end+1) = predictedTracks(iTrk).TrackID; %#ok<AGROW>
        end
    end
    detectableIDs = uint32(detectableIDs(:));
end

function posSensor = transformToSensorFrame(posWorld, cfg)
    posSensor = posWorld(:);
    if ~isfield(cfg, 'MeasurementParameters') || isempty(cfg.MeasurementParameters)
        return;
    end
    mp = cfg.MeasurementParameters;
    for m = numel(mp):-1:1
        if ~isstruct(mp(m))
            continue;
        end
        if isfield(mp(m), 'OriginPosition') && ~isempty(mp(m).OriginPosition)
            posSensor = posSensor - mp(m).OriginPosition(:);
        end
        if isfield(mp(m), 'Orientation') && ~isempty(mp(m).Orientation)
            R = mp(m).Orientation;
            if isfield(mp(m), 'IsParentToChild') && ~mp(m).IsParentToChild
                R = R';
            end
            posSensor = R * posSensor;
        end
    end
end

function detsOut = normalizeDetectionDimensions(detsIn)
%normalizeDetectionDimensions  Normalize all detections to 3-element position-only.
    if isempty(detsIn); detsOut = detsIn; return; end
    detsOut = detsIn;
    for ii = 1:numel(detsOut)
        det = detsOut{ii};
        nMeas = numel(det.Measurement);
        if nMeas > 3
            det.Measurement = det.Measurement(1:3);
            R = det.MeasurementNoise;
            if ~isscalar(R) && size(R,1) >= 3 && size(R,2) >= 3
                det.MeasurementNoise = R(1:3, 1:3);
            end
            mp = det.MeasurementParameters;
            if isstruct(mp)
                for m = 1:numel(mp)
                    if isfield(mp(m), 'HasVelocity')
                        mp(m).HasVelocity = false;
                    end
                end
                det.MeasurementParameters = mp;
            elseif iscell(mp)
                for m = 1:numel(mp)
                    if isstruct(mp{m}) && isfield(mp{m}, 'HasVelocity')
                        mp{m}.HasVelocity = false;
                        det.MeasurementParameters = mp;
                    end
                end
            end
            detsOut{ii} = det;
        end
    end
end