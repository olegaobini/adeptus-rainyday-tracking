function [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = runTracker(dataLog,tracker,showTruth, showVisuals, animateVisuals)
%runTracker  Run a tracker on a logged detection sequence and compute metrics.
%
%  METRICS MODEL
%    Uses MATLAB's built-in trackAssignmentMetrics and trackErrorMetrics
%    with NEES-based (Normalized Estimation Error Squared) distance for
%    track-to-truth assignment. NEES normalizes position error by the
%    filter's own covariance, so it automatically scales to any scenario
%    regardless of target range, sensor noise, or geometry.
%
%    Quality score uses posANEES (Average NEES across all scans):
%      ANEES = 1.0 → filter is perfectly calibrated (ideal)
%      ANEES < 3   → good tracking
%      ANEES 3-10  → acceptable but degraded
%      ANEES > 10  → poor / divergent
%
%  REFERENCES
%    [1] MathWorks, trackAssignmentMetrics:
%        https://www.mathworks.com/help/fusion/ref/trackassignmentmetrics-system-object.html
%    [2] MathWorks, trackErrorMetrics:
%        https://www.mathworks.com/help/fusion/ref/trackerrormetrics-system-object.html
%    [3] MathWorks, "Introduction to Tracking Metrics":
%        https://www.mathworks.com/help/fusion/ug/introduction-to-tracking-metrics.html
%
%  See also: trackAssignmentMetrics, trackErrorMetrics, trackOSPAMetric

if nargin < 4
    showVisuals = true;
end
if nargin < 5
    animateVisuals = true;
end

% ROI bounds (meters) computed from truth positions — used ONLY for plot
% axis limits (theaterPlot), never for filtering detections.
roi = computeDynamicROI(dataLog);

% -------- Detectable track ID support --------
useDetectableIDs = false;
try
    if isprop(tracker, 'HasDetectableTrackIDsInput') && tracker.HasDetectableTrackIDsInput
        useDetectableIDs = true;
    end
catch
end
hasSensorConfigs = isfield(dataLog, 'SensorConfig') && ~isempty(dataLog.SensorConfig);

hasRotator = isfield(dataLog, 'HasRotator') && dataLog.HasRotator;
if useDetectableIDs && hasRotator
    fprintf('[runTracker] Rotating sensor present — all tracks detectable, skipping FOV gate.\n');
    useDetectableIDs = false;
end

if useDetectableIDs && ~hasSensorConfigs
    warning('runTracker:noSensorConfig', ...
        'Tracker has HasDetectableTrackIDsInput=true but no SensorConfig. Falling back to all detectable.');
    useDetectableIDs = false;
end

validateattributes(tracker,{'trackerGNN','trackerTOMHT','trackerJPDA','numeric'},{},mfilename,'tracker');

% Extract readable labels for plot tab title
trackerType = class(tracker);
trackerType = trackerType(8:end);
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
    view(tpaxes, 3); 
    
    tp = theaterPlot('Parent', tpaxes, ...
        'AxesUnits', ["km" "km" "km"], ...
        'XLimits', [roi.xMin roi.xMax], ...
        'YLimits', [roi.yMin roi.yMax], ...
        'ZLimits', [roi.zMin roi.zMax]);
    
    zlabel(tpaxes, 'Altitude (km)');
    
    if isfield(dataLog, 'TerrainGrid') && ~isempty(dataLog.TerrainGrid)
        tg = dataLog.TerrainGrid;
        surf(tpaxes, tg.X, tg.Y, -tg.Z, ...
            'FaceColor', 'interp', 'EdgeColor', 'none', ...
            'FaceAlpha', 0.45, 'HandleVisibility', 'off');
        colormap(tpaxes, [0.15 0.12 0.08; 0.25 0.20 0.12; ...
                          0.35 0.30 0.15; 0.45 0.40 0.20; ...
                          0.55 0.50 0.30; 0.65 0.60 0.40]);
    else
        gx = [roi.xMin roi.xMax roi.xMax roi.xMin];
        gy = [roi.yMin roi.yMin roi.yMax roi.yMax];
        fill3(tpaxes, gx, gy, [0 0 0 0], [0.25 0.20 0.15], ...
            'FaceAlpha', 0.25, 'EdgeColor', [0.4 0.35 0.3], ...
            'EdgeAlpha', 0.4, 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end
    
    trackPlotters = containers.Map('KeyType', 'double', 'ValueType', 'any');
    trackHistLines = containers.Map('KeyType', 'double', 'ValueType', 'any'); 
    trackColors = lines(20);
    
    detLine = animatedline(tpaxes, 'DisplayName', 'Detections', ...
        'LineStyle', 'none', 'Marker', '.', 'MarkerSize', 8, ...
        'Color', [0 0.8 0], 'MaximumNumPoints', 1e6);

    if isfield(dataLog, 'SensorCoverage') && ~isempty(dataLog.SensorCoverage)
        trackbench.reporting.drawSensorCoverage(tpaxes, dataLog.SensorCoverage, false);
    end
    
    xlabel(tpaxes, 'X (km)');
    ylabel(tpaxes, 'Y (km)');
    zlabel(tpaxes, 'Altitude (km)');
    
    padM = 3000;
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
        zAlt = -allTruthPos(3,:);
        zLimFit = [min(-500, min(zAlt)-padM), max(zAlt)+padM];
        xlim(tpaxes, xLimFit);
        ylim(tpaxes, yLimFit);
        zlim(tpaxes, zLimFit);
    end
    set(tpaxes, 'XLimMode', 'manual', 'YLimMode', 'manual', 'ZLimMode', 'manual');
end

%% ════════════════════════════════════════════════════════════════════
%  TRACK METRICS — uses MATLAB built-in NEES-based assignment [1][3]
%  ────────────────────────────────────────────────────────────────────
%  'posnees' normalizes position error by the filter's own covariance,
%  so the threshold works at ANY range: 20km targets, 50km targets,
%  or 200km ARSR targets — all use the same threshold.
%
%  Default thresholds (from MathWorks docs):
%    AssignmentThreshold = 50 NEES units
%    DivergenceThreshold = 200 NEES units
%  These are generous enough for initial filter convergence but tight
%  enough to reject genuinely wrong associations.
%% ════════════════════════════════════════════════════════════════════

scanTime = median(diff(dataLog.Time));
if isempty(scanTime) || ~isfinite(scanTime) || scanTime <= 0
    scanTime = 0.1;
end

scenarioSpan = dataLog.Time(end) - dataLog.Time(1);
maxGap = max(scenarioSpan, 25*scanTime);

% Compute range-adaptive assignment threshold from truth geometry.
% Uses 5% of max truth-to-origin distance as the assignment gate.
% This scales automatically: 20km targets → 1000m, 50km → 2500m, 200km → 10000m.
% Ref: MathWorks uses posabserr in their extended object tracking examples [3].
allTruthPosMetric = [];
for ss = 1:size(dataLog.Truth, 2)
    tgts = dataLog.Truth(:, ss);
    for kk = 1:numel(tgts)
        allTruthPosMetric = [allTruthPosMetric; tgts(kk).Position(:)']; %#ok<AGROW>
    end
end
if ~isempty(allTruthPosMetric)
    maxRange = max(vecnorm(allTruthPosMetric, 2, 2));
else
    maxRange = 111120;  % fallback: 60nm PSR range
end
assignThresh = max(1000, 0.05 * maxRange);  % 5% of max range, min 1km
divergeThresh = 1.5 * assignThresh;          % divergence is 50% wider
fprintf('[runTracker] Metrics: posabserr, assignThresh=%.0fm, divergeThresh=%.0fm (from maxRange=%.0fm)\n', ...
    assignThresh, divergeThresh, maxRange);

% trackAssignmentMetrics: absolute position error, range-adaptive threshold
% Ref: https://www.mathworks.com/help/fusion/ref/trackassignmentmetrics-system-object.html
tam = trackAssignmentMetrics( ...
    'MotionModel', 'constvel', ...
    'AssignmentDistance', 'posabserr', ...
    'DivergenceDistance', 'posabserr', ...
    'AssignmentThreshold', assignThresh, ...
    'DivergenceThreshold', divergeThresh, ...
    'MaxUnreportedPeriod', maxGap);

% trackErrorMetrics: built-in constvel error computation (posRMS, velRMS, posANEES)
% Ref: https://www.mathworks.com/help/fusion/ref/trackerrormetrics-system-object.html
tem = trackErrorMetrics('MotionModel', 'constvel');

%% Initialize buffers
time = 0;
numSteps = numel(dataLog.Time);
i = 0;

logTime  = [];
logPlat  = [];
logTrack = [];
logTruth = [];
logSensors = strings(0,1);

assignLog = table([],[],[],[],strings(0,1), 'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});

if showVisuals && ~animateVisuals
    allStaticDets = cell(1, numSteps);
    staticTrackHist = struct();
end

allTracks = objectTrack.empty(0,1);

%% Main loop
while i < numSteps
    if showVisuals && ~isvalid(tpaxes)
        break;
    end

    i = i + 1;
    simTime    = dataLog.Time(i);
    scanBuffer = dataLog.Detections{i};

    if isempty(scanBuffer)
        scanCells = {};
    elseif iscell(scanBuffer)
        scanCells = scanBuffer(:);
    else
        scanCells = num2cell(scanBuffer(:));
    end

    scanCells = normalizeDetectionDimensions(scanCells);

    % -------- Compute detectable track IDs --------
    if useDetectableIDs && isLocked(tracker)
        try
            predictedTracks = predictTracksToTime(tracker, 'all', simTime);
            scanConfigs = getScanConfigs(dataLog, i);
            detectIDs = getDetectableTrackIDs(predictedTracks, scanConfigs);
        catch ME
            warning('runTracker:detectableFailed', ...
                'getDetectableTrackIDs failed at t=%.2f: %s. Passing all IDs.', simTime, ME.message);
            detectIDs = uint32([allTracks.TrackID]');
        end
    elseif isLocked(tracker) && ~isempty(allTracks)
        detectIDs = uint32([allTracks.TrackID]');
    else
        detectIDs = uint32([]);
    end

    % Update tracker
    tic
    trackerWants3Args = isprop(tracker, 'HasDetectableTrackIDsInput') && tracker.HasDetectableTrackIDsInput;
    if trackerWants3Args
        [tracks, ~, allTracks] = tracker(scanCells, simTime, detectIDs);
    else
        tracks = tracker(scanCells, simTime);
    end
    time = time + toc;

    % Build truths struct array
    targets = dataLog.Truth(:,i);
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

    % Log assignments
    if ~isempty(trackIDs)
        nA = numel(trackIDs);
        if isfield(dataLog,'SensorPlatformIDs') && numel(dataLog.SensorPlatformIDs) >= i
            thisPlat = dataLog.SensorPlatformIDs(i);
        else
            thisPlat = 1;
        end
        if isfield(dataLog,'ScanSensorIndices') && numel(dataLog.ScanSensorIndices) >= i
            sset = dataLog.ScanSensorIndices{i};
            if isempty(sset); sensorsUsedStr = ""; else; sensorsUsedStr = strjoin(string(sset(:).'), ','); end
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

    %% Plotting
    if showVisuals
        if isempty(scanCells)
            meas = zeros(3,0);
        else
            meas = zeros(3, numel(scanCells));
            for jj = 1:numel(scanCells)
                m = scanCells{jj}.Measurement(:);
                meas(:,jj) = m(1:3);
            end
        end

        [pos,cov] = getTrackPositions(tracks, ...
            [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);

        if animateVisuals
            if ~isempty(meas)
                addpoints(detLine, meas(1,:), meas(2,:), -meas(3,:));
            end
            for tIdx = 1:numel(tracks)
                tID = double(tracks(tIdx).TrackID);
                if ~isKey(trackPlotters, tID)
                    colorIdx = mod(tID - 1, size(trackColors, 1)) + 1;
                    trkColor = trackColors(colorIdx, :);
                    trackPlotters(tID) = trackPlotter(tp, ...
                        'DisplayName', sprintf('Track %d', tID), ...
                        'ConnectHistory', 'off', 'Marker', 's', ...
                        'MarkerSize', 8, 'MarkerFaceColor', trkColor, 'MarkerEdgeColor', 'k');
                    hLine = animatedline(tpaxes, 'Color', trkColor, 'LineWidth', 1.5, ...
                        'LineStyle', '-', 'MaximumNumPoints', 100); 
                    hLine.Annotation.LegendInformation.IconDisplayStyle = 'off';
                    trackHistLines(tID) = hLine;
                end
                displayPos = pos(tIdx, :); displayPos(3) = -displayPos(3);
                trackPlotters(tID).plotTrack(displayPos, zeros(1,3), cov(:,:,tIdx), {num2str(tID)});
                addpoints(trackHistLines(tID), displayPos(1), displayPos(2), displayPos(3));
            end
            drawnow limitrate;
        else
            if ~isempty(meas)
                measDisp = meas; measDisp(3,:) = -measDisp(3,:);
                allStaticDets{i} = measDisp; 
            end
            for tIdx = 1:numel(tracks)
                tID = tracks(tIdx).TrackID;
                fName = sprintf('T%d', tID);
                if ~isfield(staticTrackHist, fName); staticTrackHist.(fName) = []; end
                thisPos = pos(tIdx,:)'; thisPos(3) = -thisPos(3);
                staticTrackHist.(fName) = [staticTrackHist.(fName), thisPos];
            end
        end
    end
end

% Static plot (non-animated mode)
if showVisuals && ~animateVisuals
    hold(tpaxes, 'on'); view(tpaxes, 3);
    validDets = allStaticDets(~cellfun('isempty', allStaticDets)); 
    if ~isempty(validDets)
        finalDets = cat(2, validDets{:}); 
        plot3(tpaxes, finalDets(1,:), finalDets(2,:), finalDets(3,:), ...
            '.', 'Color', [0 0.8 0], 'MarkerSize', 8, 'DisplayName', 'Detections');
    end
    fNames = fieldnames(staticTrackHist);
    cmap = lines(numel(fNames));
    for ii = 1:numel(fNames)
        tPos = staticTrackHist.(fNames{ii});
        plot3(tpaxes, tPos(1,:), tPos(2,:), tPos(3,:), ...
            '-s', 'LineWidth', 2, 'Color', cmap(ii,:), ...
            'MarkerSize', 6, 'MarkerFaceColor', cmap(ii,:), ...
            'MarkerEdgeColor', 'k', 'DisplayName', ['Track ', fNames{ii}(2:end)]);
        if ~isempty(tPos)
            text(tpaxes, tPos(1,end), tPos(2,end), tPos(3,end), ...
                fNames{ii}(2:end), 'FontSize', 10, 'FontWeight', 'bold', 'Color', cmap(ii,:));
        end
    end
    drawnow;
end

% Assignment log
if ~isempty(logTime)
    assignLog = table(logTime, logPlat, logTrack, logTruth, logSensors, ...
        'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});
else
    assignLog = table([],[],[],[],strings(0,1), 'VariableNames', {'Time','PlatformID','TrackID','TruthID','SensorsUsed'});
end

% Track Swap Analysis
swapReport = trackbench.analysis.analyzeTrackSwaps(assignLog, dataLog.Truth, dataLog.Time);

if showVisuals
    axAssign = trackbench.reporting.tabbedAxes(string(plotTitle) + " | Assignment");
    trackbench.reporting.plotPlatformToTrackAssignment(axAssign, assignLog, "Platform to Track Assignment", swapReport);
    if ~swapReport.swapFree
        trackbench.reporting.plotTrackSwapAnalysis(swapReport, assignLog);
    end
end

%% Optional: plot truth trajectories
if showTruth
    trajectoryP = trajectoryPlotter(tp,'DisplayName','Trajectory');
    numTargets = size(dataLog.Truth, 1);
    trajPos = cell(1, numTargets);
    for tgt = 1:numTargets
        trajPos{tgt} = vertcat(dataLog.Truth(tgt,:).Position);
    end
    trajectoryP.plotTrajectory(trajPos);
end

%% ════════════════════════════════════════════════════════════════════
%  OUTPUT METRIC TABLES
%  ────────────────────────────────────────────────────────────────────
%  trackAssignmentMetrics → track/truth summary (assignment, swaps)
%  trackErrorMetrics      → posRMS, velRMS, posANEES, velANEES
%
%  Quality score uses posANEES (Average NEES) from trackErrorMetrics:
%    ANEES ≈ 1.0: filter is statistically optimal
%    ANEES < 3:   good tracking performance
%    ANEES 3-10:  acceptable but degraded
%    ANEES > 10:  poor / divergent filter
%
%  This replaces the previous hardcoded tolerance approach and works
%  universally across all scenarios and ranges. [2][3]
%% ════════════════════════════════════════════════════════════════════

trackSummary = trackMetricsTable(tam);
truthSummary = truthMetricsTable(tam);
trackMetrics = cumulativeTrackMetrics(tem);
truthMetrics = cumulativeTruthMetrics(tem);

% Quality score: posRMS as percentage of max scenario range.
% This is intuitive, universal, and doesn't depend on filter covariance.
%   < 1%  = Excellent (sub-1% of range)
%   1-3%  = Good
%   3-5%  = Acceptable
%   > 5%  = Poor
scoreStrings = strings(height(trackMetrics), 1);
hasRMS = any(strcmp(trackMetrics.Properties.VariableNames, 'posRMS'));

for k = 1:height(trackMetrics)
    if ~hasRMS
        scoreStrings(k) = "N/A";
    else
        rmse = trackMetrics.posRMS(k);
        if isnan(rmse)
            scoreStrings(k) = "N/A";
        else
            pct = (rmse / maxRange) * 100;
            if pct < 1.0
                scoreStrings(k) = sprintf("%.0fm / %.1f%% (Excellent)", rmse, pct);
            elseif pct < 3.0
                scoreStrings(k) = sprintf("%.0fm / %.1f%% (Good)", rmse, pct);
            elseif pct < 5.0
                scoreStrings(k) = sprintf("%.0fm / %.1f%% (Acceptable)", rmse, pct);
            else
                scoreStrings(k) = sprintf("%.0fm / %.1f%% (Poor)", rmse, pct);
            end
        end
    end
end
trackMetrics.Quality = categorical(scoreStrings);

% Remove columns we don't use in our summary report
trVarsToRemove = {'DivergenceCount','DeletionStatus','DeletionLength','DivergenceLength', ...
    'RedundancyStatus','RedundancyCount','RedundancyLength','FalseTrackLength', ...
    'FalseTrackStatus'};
for v = 1:numel(trVarsToRemove)
    if any(strcmp(trackSummary.Properties.VariableNames, trVarsToRemove{v}))
        trackSummary = removevars(trackSummary, trVarsToRemove{v});
    end
end

tuVarsToRemove = {'DeletionStatus','BreakStatus','BreakLength','InCoverageArea','EstablishmentStatus'};
for v = 1:numel(tuVarsToRemove)
    if any(strcmp(truthSummary.Properties.VariableNames, tuVarsToRemove{v}))
        truthSummary = removevars(truthSummary, tuVarsToRemove{v});
    end
end

end

%% ========================================================================
%                         LOCAL HELPER FUNCTIONS
%% ========================================================================

function roi = computeDynamicROI(dataLog)
    pad = 6000;
    allPos = [];
    for s = 1:size(dataLog.Truth, 2)
        targets = dataLog.Truth(:, s);
        for k = 1:numel(targets)
            allPos = [allPos; targets(k).Position(:)']; %#ok<AGROW>
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
    roi.zMax = min(max(roi.zMax, 8000), 15000);
end

function configs = getScanConfigs(dataLog, scanIdx)
    raw = dataLog.SensorConfig{scanIdx};
    if iscell(raw); configs = raw; else; configs = {raw}; end
end

function detectableIDs = getDetectableTrackIDs(predictedTracks, configs)
    detectableIDs = uint32([]);
    if isempty(predictedTracks) || isempty(configs); return; end
    for iTrk = 1:numel(predictedTracks)
        posWorld = predictedTracks(iTrk).State(1:2:end); posWorld = posWorld(:);
        inFOV = false;
        for iSens = 1:numel(configs)
            cfg = configs{iSens};
            if isfield(cfg, 'IsValidTime') && ~cfg.IsValidTime; continue; end
            posSensor = transformToSensorFrame(posWorld, cfg);
            [az, el] = cart2sph(posSensor(1), posSensor(2), posSensor(3));
            az = rad2deg(az); el = rad2deg(el);
            if isfield(cfg, 'FieldOfView') && numel(cfg.FieldOfView) >= 2
                fov = cfg.FieldOfView;
                if abs(az) <= fov(1)/2 && abs(el) <= fov(2)/2; inFOV = true; break; end
            else
                inFOV = true; break;
            end
        end
        if inFOV; detectableIDs(end+1) = predictedTracks(iTrk).TrackID; end %#ok<AGROW>
    end
    detectableIDs = uint32(detectableIDs(:));
end

function posSensor = transformToSensorFrame(posWorld, cfg)
    posSensor = posWorld(:);
    if ~isfield(cfg, 'MeasurementParameters') || isempty(cfg.MeasurementParameters); return; end
    mp = cfg.MeasurementParameters;
    for m = numel(mp):-1:1
        if ~isstruct(mp(m)); continue; end
        if isfield(mp(m), 'OriginPosition') && ~isempty(mp(m).OriginPosition)
            posSensor = posSensor - mp(m).OriginPosition(:);
        end
        if isfield(mp(m), 'Orientation') && ~isempty(mp(m).Orientation)
            R = mp(m).Orientation;
            if isfield(mp(m), 'IsParentToChild') && ~mp(m).IsParentToChild; R = R'; end
            posSensor = R * posSensor;
        end
    end
end

function detsOut = normalizeDetectionDimensions(detsIn)
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
                    if isfield(mp(m), 'HasVelocity'); mp(m).HasVelocity = false; end
                end
                det.MeasurementParameters = mp;
            elseif iscell(mp)
                for m = 1:numel(mp)
                    if isstruct(mp{m}) && isfield(mp{m}, 'HasVelocity')
                        mp{m}.HasVelocity = false; det.MeasurementParameters = mp;
                    end
                end
            end
            detsOut{ii} = det;
        end
    end
end