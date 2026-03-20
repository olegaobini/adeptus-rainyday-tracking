function recordDemoVideo(scenarioName, trackerType, filterModel, outputFile)
%recordDemoVideo  Record a polished MP4 demo of a scenario run.
%
% Creates a high-quality video showing:
%   - Sensor coverage rings/wedges on the ground plane
%   - Green detection dots accumulating over time
%   - Colored track markers with trail lines
%   - HUD overlay: scenario name, sensor list, elapsed time, track count
%
% USAGE (from adeptus-rainyday-tracking root, or anywhere with addpath)
%   addpath("scripts");
%   recordDemoVideo                                 % demo_first_run, GNN+IMM
%   recordDemoVideo("demo_tuned_performance")       % different run file
%   recordDemoVideo("demo_first_run","JPDA","IMM")  % different tracker
%   recordDemoVideo("demo_first_run","GNN","IMM","results/my_demo.mp4")
%
% INPUTS
%   scenarioName : string — run file name from config/runs/ (default: "demo_first_run")
%   trackerType  : 'GNN' | 'TOMHT' | 'JPDA' (default: 'GNN')
%   filterModel  : 'CV' | 'IMM' (default: 'IMM')
%   outputFile   : path for output MP4 (default: results/<scenarioName>_demo.mp4)
%
% REQUIREMENTS
%   - Sensor Fusion and Tracking Toolbox
%   - addpath("scripts") then call recordDemoVideo (auto-resolves project root)
%
% See also: trackbench.config.loadRunFile, trackbench.tracking.runTracker

arguments
    scenarioName (1,1) string  = "demo_first_run"
    trackerType  (1,1) string  = "GNN"
    filterModel  (1,1) string  = "IMM"
    outputFile   (1,1) string  = ""
end

%% Resolve project root from this script's location (scripts/ -> parent)
root = resolveRoot();
cd(root);
addpath(genpath(fullfile(root, 'src')));

if outputFile == ""
    resultsDir = fullfile(root, "results");
    if ~isfolder(resultsDir); mkdir(resultsDir); end
    outputFile = fullfile(resultsDir, scenarioName + "_demo.mp4");
end

fprintf('\n========================================\n');
fprintf('  DEMO VIDEO RECORDER\n');
fprintf('  Scenario : %s\n', scenarioName);
fprintf('  Tracker  : %s + %s\n', trackerType, filterModel);
fprintf('  Output   : %s\n', outputFile);
fprintf('========================================\n\n');

%% ========================================================================
%  1. LOAD SCENARIO & GENERATE DETECTIONS
% =========================================================================
[scenario, config, sensors, metas] = trackbench.config.loadRunFile(scenarioName);
dataLog = trackbench.detections.runDetections(scenario, config.degradation.enabled, metas, config.environment);
if isfield(config, 'terrainGrid') && ~isempty(config.terrainGrid)
    dataLog.TerrainGrid = config.terrainGrid;
end

%% ========================================================================
%  2. BUILD TRACKER
% =========================================================================
params     = config.active_params;
% Count total sensors across all platforms
platformNames = fieldnames(sensors);
numSensors = 0;
sensorNameList = {};
for p = 1:numel(platformNames)
    pSensors = sensors.(platformNames{p});
    numSensors = numSensors + numel(pSensors);
    for s = 1:numel(pSensors)
        if s <= numel(metas.(platformNames{p})) && isfield(metas.(platformNames{p}){s}, 'name')
            sensorNameList{end+1} = metas.(platformNames{p}){s}.name; %#ok<AGROW>
        else
            sensorNameList{end+1} = sprintf('Sensor %d', numel(sensorNameList)+1); %#ok<AGROW>
        end
    end
end

tracker = trackbench.tracking.buildTracker(trackerType, filterModel, params, ...
    config.tracker_global, config.filter_params, params.pd, numSensors);

fprintf('[VIDEO] %d sensors loaded: %s\n', numSensors, strjoin(sensorNameList, ', '));

%% ========================================================================
%  3. SETUP FIGURE — Dark theme, high resolution
% =========================================================================
figW = 1920; figH = 1080;  % 1080p
fig = figure('Name', sprintf('Demo: %s', scenarioName), ...
    'Color', [0.05 0.05 0.08], ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Position', [50 50 figW figH], ...
    'Renderer', 'opengl');

% Main 3D axes (80% width, full height with padding)
ax = axes(fig, 'Units', 'normalized', 'Position', [0.05 0.08 0.72 0.84], ...
    'Color', [0.08 0.08 0.12], ...
    'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], 'ZColor', [0.7 0.7 0.7], ...
    'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.4);
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
view(ax, -37.5, 30);  % Nice 3D viewing angle
xlabel(ax, 'X (km)', 'Color', 'w');
ylabel(ax, 'Y (km)', 'Color', 'w');
zlabel(ax, 'Alt (km)', 'Color', 'w');
ax.FontSize = 10;

% Scale factor (meters -> km)
sf = 1/1000;

%% ========================================================================
%  4. COMPUTE AXIS LIMITS FROM TRUTH DATA
% =========================================================================
allTruthPos = [];
T = dataLog.Truth;
[nTgts, nTimes] = size(T);
for ti = 1:nTgts
    for k = 1:nTimes
        if isfield(T(ti,k), 'Position') && ~isempty(T(ti,k).Position)
            allTruthPos = [allTruthPos; T(ti,k).Position(:)' * sf]; %#ok<AGROW>
        end
    end
end
if ~isempty(allTruthPos)
    pad = 15;  % km padding
    xlim(ax, [min(allTruthPos(:,1))-pad, max(allTruthPos(:,1))+pad]);
    ylim(ax, [min(allTruthPos(:,2))-pad, max(allTruthPos(:,2))+pad]);
    zRange = [min(allTruthPos(:,3)), max(allTruthPos(:,3))];
    zlim(ax, [min(zRange(1)-3, -12), max(zRange(2)+3, 1)]);
end

%% ========================================================================
%  5. DRAW SENSOR COVERAGE RINGS (if available)
% =========================================================================
% Build a simple coverage struct from sensor metadata for visual reference
try
    drawSensorCoverageFromMetas(ax, sensors, metas, sf);
catch ME
    fprintf('[VIDEO] Sensor coverage drawing skipped: %s\n', ME.message);
end

%% ========================================================================
%  6. DRAW TRUTH TRAJECTORIES (faint dashed lines for reference)
% =========================================================================
truthColors = [
    0.3 0.5 1.0;   % blue
    1.0 0.3 0.3;   % red
    1.0 0.8 0.2;   % gold
    0.3 0.9 0.5;   % green
    0.8 0.3 0.9;   % purple
    0.2 0.8 0.8;   % cyan
];
for ti = 1:nTgts
    ci = mod(ti-1, size(truthColors,1)) + 1;
    tPos = [];
    for k = 1:nTimes
        if isfield(T(ti,k), 'Position') && ~isempty(T(ti,k).Position)
            tPos = [tPos; T(ti,k).Position(:)' * sf]; %#ok<AGROW>
        end
    end
    if ~isempty(tPos)
        plot3(ax, tPos(:,1), tPos(:,2), tPos(:,3), '--', ...
            'Color', [truthColors(ci,:) 0.25], 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
    end
end

%% ========================================================================
%  7. HUD — Text overlay panel on the right side
% =========================================================================
% Scenario title
annotation(fig, 'textbox', [0.78 0.88 0.21 0.08], ...
    'String', upper(strrep(scenarioName, '_', ' ')), ...
    'Color', [0.3 0.8 1.0], 'FontSize', 16, 'FontWeight', 'bold', ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');

% Description
descText = config.scenario.num_targets + " targets | " + ...
    config.scenario.duration_s + "s";
if config.degradation.enabled
    descText = descText + " | DEGRADED";
else
    descText = descText + " | CLEAR";
end
annotation(fig, 'textbox', [0.78 0.83 0.21 0.04], ...
    'String', descText, ...
    'Color', [0.6 0.6 0.6], 'FontSize', 10, ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center');

% Tracker info
annotation(fig, 'textbox', [0.78 0.78 0.21 0.04], ...
    'String', sprintf('Tracker: %s + %s', trackerType, filterModel), ...
    'Color', [0.9 0.6 0.2], 'FontSize', 11, 'FontWeight', 'bold', ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center');

% Sensor list
sensorListStr = "";
for s = 1:numel(sensorNameList)
    sensorListStr = sensorListStr + sprintf("  %d. %s\n", s, sensorNameList{s});
end
annotation(fig, 'textbox', [0.78 0.48 0.21 0.28], ...
    'String', sprintf("SENSORS (%d)\n%s", numSensors, sensorListStr), ...
    'Color', [0.7 0.7 0.7], 'FontSize', 9, ...
    'FontName', 'Consolas', 'EdgeColor', [0.3 0.3 0.3], ...
    'BackgroundColor', [0.06 0.06 0.1], 'FitBoxToText', 'off', ...
    'Margin', 6);

% Dynamic HUD elements (updated each frame)
hudTime = annotation(fig, 'textbox', [0.78 0.38 0.21 0.06], ...
    'String', 'T = 0.0 s', ...
    'Color', [0.2 1.0 0.4], 'FontSize', 14, 'FontWeight', 'bold', ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center');

hudTracks = annotation(fig, 'textbox', [0.78 0.32 0.21 0.05], ...
    'String', 'Active Tracks: 0', ...
    'Color', [1.0 0.8 0.2], 'FontSize', 12, ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center');

hudDets = annotation(fig, 'textbox', [0.78 0.27 0.21 0.05], ...
    'String', 'Detections: 0', ...
    'Color', [0 0.8 0], 'FontSize', 11, ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center');

% Boeing / project label at bottom
annotation(fig, 'textbox', [0.78 0.02 0.21 0.04], ...
    'String', 'Adeptus Rainy Day Tracking', ...
    'Color', [0.4 0.4 0.4], 'FontSize', 9, ...
    'FontName', 'Consolas', 'EdgeColor', 'none', ...
    'BackgroundColor', 'none', 'FitBoxToText', 'off', ...
    'HorizontalAlignment', 'center', 'FontAngle', 'italic');

%% ========================================================================
%  8. INITIALIZE VIDEO WRITER + ANIMATION OBJECTS
% =========================================================================
vw = VideoWriter(outputFile, 'MPEG-4');
vw.FrameRate = 24;
vw.Quality   = 95;
open(vw);

% Detection accumulator (green dots)
detLine = animatedline(ax, 'Color', [0 0.8 0], 'Marker', '.', ...
    'LineStyle', 'none', 'MarkerSize', 6, ...
    'DisplayName', 'Detections', 'MaximumNumPoints', 1e6);

% Track plotters + trail lines
trackColors = [
    1.0 0.4 0.1;   % orange
    0.2 0.6 1.0;   % blue
    1.0 0.2 0.6;   % magenta
    0.1 0.9 0.4;   % green
    0.9 0.8 0.1;   % yellow
    0.5 0.2 1.0;   % purple
    0.1 0.8 0.8;   % teal
    1.0 0.5 0.5;   % salmon
];
trackTrails = containers.Map('KeyType', 'double', 'ValueType', 'any');
trackMarkers = containers.Map('KeyType', 'double', 'ValueType', 'any');

totalDetsCount = 0;

% Detectable-track-ID support
useDetectableIDs = false;
try
    if isprop(tracker, 'HasDetectableTrackIDsInput') && tracker.HasDetectableTrackIDsInput
        useDetectableIDs = true;
    end
catch
end
hasRotator = isfield(dataLog, 'HasRotator') && dataLog.HasRotator;
if useDetectableIDs && hasRotator
    useDetectableIDs = false;
end
hasSensorConfigs = isfield(dataLog, 'SensorConfig') && ~isempty(dataLog.SensorConfig);
if useDetectableIDs && ~hasSensorConfigs
    useDetectableIDs = false;
end
allTracks = objectTrack.empty(0,1);

%% ========================================================================
%  9. WRITE TITLE CARD — 2 seconds of static intro
% =========================================================================
fprintf('[VIDEO] Writing title card...\n');
drawnow;
titleFrame = getframe(fig);
for f = 1:round(vw.FrameRate * 2)
    writeVideo(vw, titleFrame);
end

%% ========================================================================
%  10. MAIN ANIMATION LOOP — Run tracker, capture every frame
% =========================================================================
numSteps = numel(dataLog.Time);
fprintf('[VIDEO] Recording %d scan steps...\n', numSteps);

for i = 1:numSteps
    simTime    = dataLog.Time(i);
    scanBuffer = dataLog.Detections{i};
    
    % Normalize detections to cell array
    if isempty(scanBuffer)
        scanCells = {};
    elseif iscell(scanBuffer)
        scanCells = scanBuffer(:);
    else
        scanCells = num2cell(scanBuffer(:));
    end
    
    % Compute detectable IDs for GNN/JPDA
    if useDetectableIDs && isLocked(tracker) && ~isempty(allTracks)
        try
            predictedTracks = predictTracksToTime(tracker, 'all', simTime);
            scanConfigs = dataLog.SensorConfig{i};
            if ~iscell(scanConfigs); scanConfigs = {scanConfigs}; end
            detectIDs = computeDetectableIDs(predictedTracks, scanConfigs);
        catch
            detectIDs = uint32([allTracks.TrackID]');
        end
    elseif isLocked(tracker) && ~isempty(allTracks)
        detectIDs = uint32([allTracks.TrackID]');
    else
        detectIDs = uint32([]);
    end
    
    % Update tracker
    trackerWants3Args = isprop(tracker, 'HasDetectableTrackIDsInput') && ...
                        tracker.HasDetectableTrackIDsInput;
    if trackerWants3Args
        [tracks, ~, allTracks] = tracker(scanCells, simTime, detectIDs);
    else
        tracks = tracker(scanCells, simTime);
    end
    
    % --- PLOT DETECTIONS ---
    for d = 1:numel(scanCells)
        det = scanCells{d};
        if isprop(det, 'Measurement') || isfield(det, 'Measurement')
            m = det.Measurement(:);
            if numel(m) >= 3
                addpoints(detLine, m(1)*sf, m(2)*sf, m(3)*sf);
            elseif numel(m) == 2
                addpoints(detLine, m(1)*sf, m(2)*sf, 0);
            end
            totalDetsCount = totalDetsCount + 1;
        end
    end
    
    % --- PLOT TRACKS ---
    [pos, ~] = getTrackPositions(tracks, ...
        [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);
    
    for tIdx = 1:numel(tracks)
        tID = double(tracks(tIdx).TrackID);
        ci = mod(tID - 1, size(trackColors, 1)) + 1;
        tCol = trackColors(ci, :);
        tPos = pos(tIdx, :) * sf;
        
        % Trail line (accumulating)
        if ~isKey(trackTrails, tID)
            hLine = animatedline(ax, 'Color', tCol, 'LineWidth', 2.0, ...
                'MaximumNumPoints', 500);
            hLine.Annotation.LegendInformation.IconDisplayStyle = 'off';
            trackTrails(tID) = hLine;
        end
        addpoints(trackTrails(tID), tPos(1), tPos(2), tPos(3));
        
        % Current position marker
        if ~isKey(trackMarkers, tID)
            hMark = plot3(ax, tPos(1), tPos(2), tPos(3), 's', ...
                'MarkerSize', 10, 'MarkerFaceColor', tCol, ...
                'MarkerEdgeColor', 'w', 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Track %d', tID));
            trackMarkers(tID) = hMark;
        else
            set(trackMarkers(tID), 'XData', tPos(1), 'YData', tPos(2), 'ZData', tPos(3));
        end
        
        % Track ID label
        text(ax, tPos(1)+0.5, tPos(2)+0.5, tPos(3)+0.3, ...
            sprintf('T%d', tID), 'Color', tCol, 'FontSize', 9, ...
            'FontWeight', 'bold', 'FontName', 'Consolas', ...
            'Tag', sprintf('trkLabel%d', tID));
        % Remove old labels for this track
        oldLabels = findobj(ax, 'Tag', sprintf('trkLabel%d', tID));
        if numel(oldLabels) > 1
            delete(oldLabels(2:end));
        end
    end
    
    % --- UPDATE HUD ---
    hudTime.String   = sprintf('T = %.1f s', simTime);
    hudTracks.String = sprintf('Active Tracks: %d', numel(tracks));
    hudDets.String   = sprintf('Detections: %d', totalDetsCount);
    
    % --- CAPTURE FRAME ---
    drawnow;
    frame = getframe(fig);
    writeVideo(vw, frame);
    
    % Progress every 10 steps
    if mod(i, 10) == 0
        fprintf('  [%3d / %3d] t=%.1fs | %d tracks | %d dets\n', ...
            i, numSteps, simTime, numel(tracks), totalDetsCount);
    end
end

%% ========================================================================
%  11. HOLD FINAL FRAME — 3 seconds
% =========================================================================
fprintf('[VIDEO] Writing closing hold...\n');
finalFrame = getframe(fig);
for f = 1:round(vw.FrameRate * 3)
    writeVideo(vw, finalFrame);
end

%% ========================================================================
%  12. FINALIZE
% =========================================================================
close(vw);
fprintf('\n========================================\n');
fprintf('  VIDEO SAVED: %s\n', outputFile);
fprintf('  Duration : ~%.1f seconds\n', (numSteps + 5*vw.FrameRate) / vw.FrameRate);
fprintf('  Resolution: %dx%d @ %d fps\n', figW, figH, vw.FrameRate);
fprintf('========================================\n\n');

% Leave figure open for inspection
end

%% ========================================================================
%                       LOCAL HELPER FUNCTIONS
%% ========================================================================

function drawSensorCoverageFromMetas(ax, sensors, metas, sf)
%drawSensorCoverageFromMetas  Draw range rings from sensor metadata.
%  Reads max range and type from metadata structs to draw coverage indicators.

    platformNames = fieldnames(sensors);
    sensorIdx = 0;
    
    % Colors by sensor category
    radarColor  = [0.2 0.5 1.0];
    mssrColor   = [0.9 0.5 0.1];
    irColor     = [0.9 0.2 0.7];
    fcColor     = [1.0 0.3 0.3];
    otherColor  = [0.2 0.8 0.8];
    
    for p = 1:numel(platformNames)
        pName = platformNames{p};
        for s = 1:numel(sensors.(pName))
            sensorIdx = sensorIdx + 1;
            sensor = sensors.(pName){s};
            
            % Get range limit
            maxRange = 60000;  % default 60km
            try
                if isprop(sensor, 'RangeLimits') && numel(sensor.RangeLimits) >= 2
                    maxRange = sensor.RangeLimits(2);
                elseif isprop(sensor, 'ReferenceRange')
                    maxRange = sensor.ReferenceRange;
                end
            catch
            end
            
            % Get sensor type from metadata
            sType = "UNKNOWN";
            sLabel = sprintf("Sensor %d", sensorIdx);
            if s <= numel(metas.(pName))
                meta = metas.(pName){s};
                if isfield(meta, 'type'); sType = upper(string(meta.type)); end
                if isfield(meta, 'name'); sLabel = string(meta.name); end
            end
            
            % Choose color
            if contains(sType, ["PSR","ASR","ARSR","TWS","AESA","PAR"])
                col = radarColor;
            elseif contains(sType, ["SSR","MSSR","IFF"])
                col = mssrColor;
            elseif contains(sType, ["IR","IRST","FLIR"])
                col = irColor;
            elseif contains(sType, "FIRE")
                col = fcColor;
            else
                col = otherColor;
            end
            
            % Check if it's a sector sensor (narrow FOV)
            isFullRotator = true;
            fovAz = 360;
            try
                if isprop(sensor, 'FieldOfView')
                    fovAz = sensor.FieldOfView(1);
                    if fovAz < 180
                        isFullRotator = false;
                    end
                end
            catch
            end
            
            R = maxRange * sf;  % km
            gz = 0.05;  % slight Z offset
            nPts = 120;
            
            if isFullRotator
                % 360 deg range ring
                theta = linspace(0, 360, nPts);
                xR = R * cosd(theta);
                yR = R * sind(theta);
                zR = gz * ones(1, nPts);
                
                plot3(ax, xR, yR, zR, '--', 'Color', [col 0.5], 'LineWidth', 1.2, ...
                    'HandleVisibility', 'off');
                fill3(ax, xR, yR, zR, col, 'FaceAlpha', 0.04, 'EdgeColor', 'none', ...
                    'HandleVisibility', 'off');
                
                % Legend entry
                rangeNm = maxRange / 1852;
                plot3(ax, NaN, NaN, NaN, '--', 'Color', col, 'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s (%.0f nm)', sLabel, rangeNm));
                
                % Label
                text(ax, 0, R * 0.88, gz, sprintf('%.0f nm', rangeNm), ...
                    'Color', [col 0.7], 'FontSize', 8, 'FontName', 'Consolas', ...
                    'HorizontalAlignment', 'center', 'HandleVisibility', 'off');
            else
                % Sector wedge
                halfFov = fovAz / 2;
                theta = linspace(-halfFov, halfFov, max(30, round(fovAz)));
                xW = [0, R * cosd(theta), 0];
                yW = [0, R * sind(theta), 0];
                zW = gz * ones(1, numel(xW));
                
                fill3(ax, xW, yW, zW, col, 'FaceAlpha', 0.12, ...
                    'EdgeColor', col, 'EdgeAlpha', 0.6, 'LineWidth', 1.2, ...
                    'HandleVisibility', 'off');
                
                rangeNm = maxRange / 1852;
                plot3(ax, NaN, NaN, NaN, '-', 'Color', col, 'LineWidth', 2, ...
                    'DisplayName', sprintf('%s (%.0f nm / %.0f deg)', sLabel, rangeNm, fovAz));
            end
            
            % Sensor position dot
            plot3(ax, 0, 0, gz, 'p', 'MarkerSize', 8, ...
                'MarkerFaceColor', col, 'MarkerEdgeColor', 'w', ...
                'HandleVisibility', 'off');
        end
    end
    
    % Add legend
    leg = legend(ax, 'Location', 'northwest', 'TextColor', [0.8 0.8 0.8], ...
        'Color', [0.1 0.1 0.15], 'EdgeColor', [0.3 0.3 0.3], ...
        'FontSize', 8, 'FontName', 'Consolas');
    leg.AutoUpdate = 'off';
end

function detectableIDs = computeDetectableIDs(predictedTracks, configs)
%computeDetectableIDs  Return IDs of tracks in at least one sensor FOV.
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
            if isfield(cfg, 'IsValidTime') && ~cfg.IsValidTime; continue; end
            if isfield(cfg, 'FieldOfView') && numel(cfg.FieldOfView) >= 2
                inFOV = true;  % Simplified: assume in FOV for recording
                break;
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

function root = resolveRoot()
%resolveRoot  Derive project root from this script's location (scripts/ -> parent).
    thisFile = mfilename('fullpath');
    root = fileparts(fileparts(thisFile));
end
