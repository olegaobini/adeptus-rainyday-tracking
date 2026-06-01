function [trackSummary, truthSummary, trackMetrics, truthMetrics, time, assignLog, swapReport] = runTracker(dataLog,tracker,showTruth, showVisuals, animateVisuals)
%runTracker  Run a tracker on a logged detection sequence and compute metrics.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
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

        % v3.7.7 Site 2 — coverage volume overlay (mirror of v3.7.5 Site 1
        % in plotInitialScenario.m). NED meters → display: negate Z so
        % altitude points up; no m→km scale here (tracker plot uses NED
        % meters internally despite the "km" axis label, unlike Site 1's
        % plotInitialScenario which scales by 1/1000). FaceAlpha=0.12 —
        % lower than Site 1's 0.18 because tracks/detections dominate the
        % tracker plot visually; coverage is contextual here.
        try
            for kSensor = 1:numel(dataLog.SensorCoverage)
                cov = dataLog.SensorCoverage(kSensor);
                if ~cov.isRadar && ~cov.isIR; continue; end
                % v3.7.8 — MSSR/SSR now renders its swept coverage volume
                % (same path as PSR), built from this sensor's own
                % coverageConfig so it tracks the SSR JSON config dynamically.

                % Pass the sensor's real finite range so the SSR volume reaches
                % its full range (the 120 km cap is only for Inf range).
                volOpts = struct();
                if isfinite(cov.maxRange) && cov.maxRange > 0
                    volOpts.rMaxCap = cov.maxRange;
                end
                [V, F] = trackbench.reporting.computeSensorCoverageVolume(cov, volOpts);

                Vdisp = V;
                Vdisp(:, 3) = -Vdisp(:, 3);

                % Per-type color matches drawSensorCoverage.m (FROZEN, v3.6.15)
                if cov.isMSSR
                    col = [0.9 0.5 0.1];                 % orange  — MSSR/SSR
                elseif cov.isIR
                    col = [0.9 0.2 0.8];                 % magenta — IR
                elseif ~cov.isRotator && cov.isRadar
                    col = [0.1 0.9 0.3];                 % green   — sector radar
                else
                    col = [0.2 0.6 1.0];                 % blue    — PSR / rotator
                end

                patch(tpaxes, 'Vertices', Vdisp, 'Faces', F, ...
                    'FaceColor', col, 'FaceAlpha', 0.12, ...
                    'EdgeColor', col, 'EdgeAlpha', 0.10, ...
                    'LineWidth', 0.3, ...
                    'HandleVisibility', 'off');
            end
        catch ME
            fprintf('[WARN] Coverage volume skipped: %s\n', ME.message);
        end
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

% Guard: an empty run (0 detections -> 0 scans, e.g. a sonar contact that
% is airborne instead of submerged, or a sensor that never sees the target)
% has an empty dataLog.Time. Bail with empty results instead of indexing
% dataLog.Time(end) into an empty array (was: Array indices must be positive).
if isempty(dataLog.Time)
    warning('trackbench:runTracker:noDetections', ...
        ['No detections to track (0 scans). Check sensor/target geometry - ' ...
         'e.g. a sonar contact must be submerged, not airborne.']);
    trackSummary = table(); truthSummary = table();
    trackMetrics = table(); truthMetrics = table();
    time = dataLog.Time; assignLog = {};
    swapReport = struct('swapFree', true, 'totalSwaps', 0);
    return;
end

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
% v3.7.3 X-F7 patch — usingMSC must be defined BEFORE the tam construction
% branch below. Full v3.7.3 MSC-RPEKF init block (prevPose, lastCorrectionTime,
% currentPose, diagXF7Shots) remains in its original location just before the
% main loop — those variables are loop-scoped and don't need to move.
usingMSC = isfield(dataLog, 'UsingMSCTracking') && dataLog.UsingMSCTracking;

if usingMSC
    % v3.7.3 X-F7 patch — MSC-state-aware assignment metric. The built-in
    % 'constvel' MotionModel + 'posabserr' AssignmentDistance reads track
    % state elements [1,3,5] as Cartesian [x,y,z]. For an MSC-state track
    % those are [az, el, 1/r] — dimensional gibberish (rad/rad/m⁻¹ vs m),
    % producing distances that always exceed assignThresh → Tracked%=0%
    % even when the filter is producing plausible MSC estimates (v3.7.3
    % TEMP-DIAG X-F7 confirmed 1/r magnitudes consistent with truth range).
    %
    % Custom distance function reuses the same MSC→Cartesian conversion
    % as the visualization shim getTrackPositionsMSC: cvmeasmsc(state,
    % 'rectangular') returns position relative to observer in scenario
    % frame, plus observer position gives world Cartesian. Euclidean
    % distance to truth.Position then yields meters — same units as
    % assignThresh/divergeThresh, so thresholds stay in meters.
    %
    % Observer position is a per-scan input but the (track,truth)
    % distance fcn signature has no slot for it; nontunable property
    % lock on the System object prevents re-binding the handle each
    % scan. Standard escape hatch: a module-level persistent variable
    % set via setter call before step(tam,...). Same function handle
    % bound to both AssignmentDistanceFcn and DivergenceDistanceFcn
    % — both express absolute position error in meters.
    tam = trackAssignmentMetrics( ...
        'DistanceFunctionFormat', 'custom', ...
        'AssignmentDistanceFcn', @trackbenchMSCAssignmentDistance, ...
        'DivergenceDistanceFcn', @trackbenchMSCAssignmentDistance, ...
        'AssignmentThreshold', assignThresh, ...
        'DivergenceThreshold', divergeThresh, ...
        'MaxUnreportedPeriod', maxGap);
else
    tam = trackAssignmentMetrics( ...
        'MotionModel', 'constvel', ...
        'AssignmentDistance', 'posabserr', ...
        'DivergenceDistance', 'posabserr', ...
        'AssignmentThreshold', assignThresh, ...
        'DivergenceThreshold', divergeThresh, ...
        'MaxUnreportedPeriod', maxGap);
end

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

% v3.7.3 — MSC-RPEKF state for moving-sensor angle-only passive ranging.
% UsingMSCTracking branches:
%   - The per-scan ObserverInput propagation block in the main scan loop
%   - The getTrackPositionsMSC visualization shim at the confirmed-track
%     plot call below (line ~352)
% prevPose holds ownship pose at last correction (per canonical example).
% lastCorrectionTime accumulates dT across prediction-only scans.
% currentPose mirrors prevPose at init; updated per-scan inside the
% propagation block when MSC is active. Also consumed by line ~352's
% getTrackPositionsMSC call for the observer position.
% Backward-compat: pre-v3.7.3 dataLogs (and stationary scenarios with no
% MSC sensors) → usingMSC=false → block is a no-op, getTrackPositions
% takes the Cartesian path. Stationary-tower MSC scenarios (PosterDemo
% with hypothetical IR) → Velocity=[0,0,0] constant → maneuver=0 →
% MSC-RPEKF degrades to plain MSC-EKF behavior (still correct).
prevPose = readOwnshipPose(dataLog, 1);
lastCorrectionTime = 0;
currentPose = prevPose;

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

    % v3.7.3 — MSC-RPEKF ObserverInput propagation. The trackingMSCEKF
    % bank's state-transition function @constvelmsc expects an
    % ObserverInput vector capturing ownship maneuver since the last
    % correction. For stationary tower scenarios this is zero by
    % construction (constant zero Velocity → zero maneuver → no-op);
    % for airborne ownship it's the load-bearing mechanism that lets
    % the range-parameterized bank collapse to truth. Must fire BEFORE
    % the tracker call so the predict step in the next correction uses
    % the updated maneuver. Canonical: MathWorks "Passive Ranging Using
    % a Single Maneuvering Sensor" example, MSC-RPEKF section.
    if usingMSC
        currentPose = readOwnshipPose(dataLog, i);
        if ~isempty(allTracks)
            dT = simTime - lastCorrectionTime;
            observerManeuver = calculateManeuver(currentPose, prevPose, dT);
            for ot = 1:numel(allTracks)
                trackingFilters = getTrackFilterProperties(tracker, ...
                    allTracks(ot).TrackID, 'TrackingFilters');
                for m = 1:numel(trackingFilters{1})
                    trackingFilters{1}{m}.ObserverInput = observerManeuver;
                end
            end
        end
    end

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
        % v3.7.3 — 3-output form. Required so allTracks is populated
        % for the MSC-RPEKF ObserverInput propagation block above. SDK
        % supports the 3-output form on all three trackers (GNN, JPDA,
        % TOMHT); bit-identical to the prior 2-output form for tracks
        % consumption and metrics downstream.
        [tracks, ~, allTracks] = tracker(scanCells, simTime);
    end
    time = time + toc;

    % v3.7.3 — Update prevPose / lastCorrectionTime only on successful
    % correction (non-empty scan). Mirrors canonical passive ranging
    % example: maneuver accumulates across prediction-only scans, then
    % collapses on the next correction. No-op when usingMSC=false.
    if usingMSC && ~isempty(scanCells)
        prevPose = currentPose;
        lastCorrectionTime = simTime;
    end

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

    % v3.7.3 X-F7 patch — push current ownship position into the
    % module-level persistent slot used by trackbenchMSCAssignmentDistance.
    % Must fire BEFORE step(tam,...) so the System object's internal
    % (track,truth) distance evaluations read the correct world-frame
    % observer position for this scan. No-op when usingMSC=false.
    if usingMSC
        trackbenchMSCObsPos(currentPose.Position);
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
            % Filter angle-only [az;el] detections (IR sensors with
            % Frame='Spherical' + HasRange=false). Their 2-vec Measurement
            % can't be plotted as Cartesian XYZ here; the tracker has already
            % consumed them via initrpekf (v3.7.0) and any resulting confirmed
            % tracks render via getTrackPositions below.
            keepDet = false(1, numel(scanCells));
            for jj = 1:numel(scanCells)
                if numel(scanCells{jj}.Measurement) >= 3
                    keepDet(jj) = true;
                end
            end
            plotCells = scanCells(keepDet);
            if isempty(plotCells)
                meas = zeros(3,0);
            else
                meas = zeros(3, numel(plotCells));
                for jj = 1:numel(plotCells)
                    m = plotCells{jj}.Measurement(:);
                    meas(:,jj) = m(1:3);
                end
            end
        end

        % v3.7.3 — Branch on dataLog.UsingMSCTracking for the
        % visualization position-extraction call. MSC tracks have
        % state shape [az;azRate;el;elRate;1/r;rDot/r] — the Cartesian
        % PositionSelector would extract az/el/1-over-r as if they were
        % x/y/z (nonsense). getTrackPositionsMSC converts MSC state to
        % Cartesian via cvmeasmsc(state,'rectangular') + observer
        % position (currentPose.Position from this scan's MSC propagation
        % block above). Mixed-sensor scenarios (radar + IR simultaneously)
        % out of scope for v3.7.3; see README Process findings.
        if usingMSC
            [pos, cov] = getTrackPositionsMSC(tracks, currentPose.Position(:));
        else
            [pos,cov] = getTrackPositions(tracks, ...
                [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);
        end

        if animateVisuals
            if ~isempty(meas)
                % v3.7.7 — underground detection filter (Site 1 precedent,
                % plotInitialScenario.m:147/:157). HasINS=false body-frame
                % artifact can produce meas(3) > 1 (display altitude < -1m).
                % Root cause investigation in v3.7.4 follow-up queue item #9a
                % (HasINS body-frame proper fix); this is viz-only mitigation.
                keepUg = meas(3,:) <= 1;
                addpoints(detLine, meas(1,keepUg), meas(2,keepUg), -meas(3,keepUg));
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
                try
                    trackPlotters(tID).plotTrack(displayPos, zeros(1,3), cov(:,:,tIdx), {num2str(tID)});
                catch
                    % R2025b fallback: plot position only if full signature fails
                    try; trackPlotters(tID).plotTrack(displayPos); catch; end
                end
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
        trackbench.reporting.plotTrackSwapAnalysis(swapReport, assignLog, [], ...
            sprintf('%s + %s', trackerType, filterType));
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

% Add a TrackedPct column to truthSummary so readers can see at a
% glance how much of each truth's life was after its first track
% association. Computed as 100*(TotalLength - EstablishmentLength) /
% TotalLength, rounded to the nearest integer percent. This is an
% UPPER BOUND on actual tracking time — it assumes no track breaks
% after establishment, which is what the BreakCount column tracks.
% For the authoritative "who was tracking what at each scan" view,
% use plotPlatformToTrackAssignment.
if ~isempty(truthSummary) && height(truthSummary) > 0 && ...
        all(ismember({'TotalLength','EstablishmentLength'}, truthSummary.Properties.VariableNames))
    totLen = truthSummary.TotalLength;
    estLen = truthSummary.EstablishmentLength;
    pctTracked = NaN(height(truthSummary), 1);
    safe = totLen > 0;
    pctTracked(safe) = round(100 * (totLen(safe) - estLen(safe)) ./ totLen(safe));
    truthSummary.TrackedPct = pctTracked;
end

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

function pose = readOwnshipPose(dataLog, scanIdx)
% v3.7.3 — Read ownship pose for a given scan, with backward-compat
% fallback. Pre-v3.7.3 dataLogs lack the OwnshipPose field; return a
% zero-velocity stationary pose at origin so ObserverInput evaluates
% to zeros(6,1) → no maneuver → MSC-RPEKF degrades to plain MSC-EKF.
    if isfield(dataLog, 'OwnshipPose') && ~isempty(dataLog.OwnshipPose) ...
            && numel(dataLog.OwnshipPose) >= scanIdx
        pose = dataLog.OwnshipPose(scanIdx);
    else
        pose = struct('PlatformID', 0, 'Position', [0 0 0], 'Velocity', [0 0 0]);
    end
end

function maneuver = calculateManeuver(currentPose, prevPose, dT)
% Verbatim from MathWorks "Passive Ranging Using a Single Maneuvering
% Sensor" example. Computes the higher-order ownship motion (deviation
% from constant-velocity prediction) as a 6-element interleaved
% maneuver vector [dpx; dvx; dpy; dvy; dpz; dvz] consumed by
% trackingMSCEKF.ObserverInput. For dT=0 (first scan with no correction
% yet) and for stationary-ownship scans (currentPose == prevPose) the
% function naturally evaluates to zeros(6,1) — no maneuver.
    v = prevPose.Velocity;
    prevPos = prevPose.Position;
    prevVel = prevPose.Velocity;
    currentPos = currentPose.Position;
    currentVel = currentPose.Velocity;

    % position change apart from constant velocity motion
    deltaP = currentPos - prevPos - v*dT;
    % velocity change
    deltaV = currentVel - prevVel;

    maneuver = zeros(6, 1);
    maneuver(1:2:end) = deltaP;
    maneuver(2:2:end) = deltaV;
end

function [pos, cov] = getTrackPositionsMSC(tracks, observerPosition)
% Verbatim from MathWorks "Passive Ranging Using a Single Maneuvering
% Sensor" example. Converts MSC track state
% [az;azRate;el;elRate;1/r;rDot/r] to Cartesian position via
% cvmeasmsc(state, 'rectangular') and adds the observer's current
% world position. Plumbed at line ~352 via dataLog.UsingMSCTracking
% branch added in v3.7.3.
%
% Defensive deviation from canonical: empty-tracks early-return to
% match getTrackPositions' implicit "empty in → empty out" behavior.
    if isempty(tracks)
        pos = zeros(0, 3);
        cov = zeros(3, 3, 0);
        return;
    end
    if isstruct(tracks) || isa(tracks, 'objectTrack')
        % Track struct
        state = [tracks.State];
        stateCov = cat(3, tracks.StateCovariance);
    elseif isa(tracks, 'trackingMSCEKF')
        % Tracking Filter
        state = tracks.State;
        stateCov = tracks.StateCovariance;
    end

    % Get relative position using measurement function.
    relPos = cvmeasmsc(state, 'rectangular');

    % Add observer position
    pos = relPos + observerPosition;
    pos = pos';

    if nargout > 1
        cov = zeros(3, 3, numel(tracks));
        for ii = 1:numel(tracks)
            % Jacobian of position measurement
            jac = cvmeasmscjac(state(:, ii), 'rectangular');
            cov(:, :, ii) = jac * stateCov(:, :, ii) * jac';
        end
    end
end

function dist = trackbenchMSCAssignmentDistance(track, truth)
% v3.7.3 X-F7 — custom AssignmentDistanceFcn/DivergenceDistanceFcn for
% trackAssignmentMetrics when tracks carry MSC state. Converts MSC track
% state [az;daz;el;del;1/r;vr/r] to world-frame Cartesian position via
% cvmeasmsc(state,'rectangular') + persistent observer position (set by
% trackbenchMSCObsPos setter before each step(tam,...) call), then
% returns Euclidean distance to truth.Position in meters. Units match
% AssignmentThreshold/DivergenceThreshold (also meters).
%
% Signature matches the System object's expected (onetrack, onetruth)
% form. Returns a non-negative scalar. Bound to both Assignment and
% Divergence distance fcn slots — both quantities are absolute position
% error in meters in this configuration.
    obsPos = trackbenchMSCObsPos();
    relPos = cvmeasmsc(track.State(:), 'rectangular');
    trackPos = relPos(:) + obsPos(:);
    truthPos = truth.Position(:);
    dist = norm(trackPos - truthPos);
end

function p = trackbenchMSCObsPos(newPos)
% v3.7.3 X-F7 — module-level persistent observer-position holder for the
% custom MSC distance function. Two modes:
%   Setter:  trackbenchMSCObsPos(newPos)  — one arg, updates the slot.
%   Getter:  p = trackbenchMSCObsPos()    — zero args, returns current.
%
% Persistent var is scoped to this local function; survives across
% iterations of the runTracker main loop (the setter overwrites each
% scan when usingMSC=true) and across runTracker invocations within a
% MATLAB session (the first setter call inside any new run overwrites
% any stale value before the first step(tam,...)). Defaults to [0;0;0]
% on first getter call if no setter has run yet — distance values would
% be off by the observer offset but the System object wouldn't crash;
% in practice setter always runs first in the gated codepath.
    persistent obsPos
    if nargin >= 1
        obsPos = newPos(:);
        return;
    end
    if isempty(obsPos); obsPos = [0; 0; 0]; end
    p = obsPos;
end