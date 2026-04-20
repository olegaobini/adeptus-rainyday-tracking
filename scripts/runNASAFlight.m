%% runNASAFlight — Track real aircraft using simulated radar
%
%  NASA DASHlink provides Flight Data Recorder (FDR) telemetry: the
%  aircraft's own GPS/INS position, speed, altitude, heading, etc.
%  This is TRUTH data — the real path the plane actually flew.
%
%  This script takes that real trajectory and asks: "How well can a
%  simulated radar track this real flight?" It:
%    1. Loads the FDR .mat file and extracts the airborne trajectory
%    2. Builds a trackingScenario with the flight as a target platform
%    3. Places a simulated PSR (Primary Search Radar) at the NED origin
%    4. Generates synthetic radar detections of the real flight path
%    5. Runs multiple trackers (GNN, JPDA) on those detections
%    6. Compares tracker performance with metrics
%    7. Optionally applies rain degradation to test robustness
%
%  The radar and tracker are simulated; only the flight path is real.
%
%  DATA SOURCE: https://c3.ndc.nasa.gov/dashlink/resources/664/
%  Each .mat file contains ARINC 717 decoded FDR parameters at 1–16 Hz.
%  Key fields used: LATP (lat), LONP (lon), ALT (altitude), GS (ground
%  speed), TRK (track angle).

clear; clc; close all;

%% ====================== USER CONFIG =================================

% --- Flight data ---
% Path is resolved relative to the project root, so the same script works
% on any machine that has the Tail_687_1 folder as a sibling of the project.
% Override flightFile with an absolute path if your data lives elsewhere.
rnfThisDir  = fileparts(mfilename('fullpath'));      % .../adeptus-rainyday-tracking/scripts
rnfProjRoot = fileparts(rnfThisDir);                 % .../adeptus-rainyday-tracking
rnfParent   = fileparts(rnfProjRoot);                % .../Adding Flight Data
flightFile  = fullfile(rnfParent, "Tail_687_1", "687200104121330.mat");
clear rnfThisDir rnfProjRoot rnfParent
maxDuration_s   = 600;    % seconds of flight to use (Inf = all)
waypointInterval_s = 10;  % waypoint spacing

% --- Radar ---
rcs_dbsm    = 10;         % target RCS (10 ≈ Boeing 737)
psrRange_m  = 120e3;      % PSR max range (120 km ≈ 65 nm)

% --- Trackers to compare ---
trackersToRun = ["GNN", "JPDA"];   % any combo of "GNN", "JPDA"

% --- Weather degradation ---
rainEnabled   = false;     % set true to degrade detections
rainRate_mmhr = 16;        % mm/hr (4=light, 16=moderate, 50=heavy)

%% ====================== LOAD FLIGHT DATA ============================
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║          NASA DASHlink — Real Flight Tracker            ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

fd = trackbench.flightdata.loadNASAFlight(flightFile, ...
    'MaxDuration', maxDuration_s, ...
    'WaypointInterval', waypointInterval_s);

wp  = fd.waypoints;
toa = fd.timeOfArrival;
vel = fd.velocities;
scenarioDuration = toa(end) + 1;

%% ====================== BUILD SCENARIO ==============================
scenario = trackingScenario('UpdateRate', 54, 'StopTime', scenarioDuration);

% Sensor tower at NED origin
tower = platform(scenario, 'Trajectory', kinematicTrajectory( ...
    'Position', [0 0 0], 'Velocity', [0 0 0]));

% PSR sensor
psr = fusionRadarSensor(1, 'Rotator', ...
    'MountingLocation', [0 0 -15], ...
    'FieldOfView', [3 20], ...
    'RangeLimits', [0 psrRange_m], ...
    'RangeResolution', 150, ...
    'AzimuthResolution', 1.4, ...
    'HasElevation', true, ...
    'HasRangeRate', false, ...
    'DetectionProbability', 0.9, ...
    'FalseAlarmRate', 1e-6, ...
    'ReferenceRange', psrRange_m * 0.6, ...
    'CenterFrequency', 2.8e9);
tower.Sensors = {psr};

% Target aircraft from real flight data
traj = waypointTrajectory('Waypoints', wp, 'TimeOfArrival', toa, 'Velocities', vel);
aircraft = platform(scenario, 'Trajectory', traj, ...
    'Signatures', {rcsSignature('Pattern', rcs_dbsm)});

fprintf('\n  Scenario: %.0fs | PSR %.0f km | RCS %d dBsm | Rain: %s\n', ...
    scenarioDuration, psrRange_m/1e3, rcs_dbsm, ...
    iif(rainEnabled, sprintf('%d mm/hr', rainRate_mmhr), 'OFF'));

%% ====================== GENERATE DETECTIONS =========================
fprintf('\nGenerating detections...\n');

allDets = {};
allTimes = [];
scanIdx = 0;
totalTargetDets = 0;
totalClutter = 0;
lastReport = -60;

while advance(scenario)
    time = scenario.SimulationTime;
    if time - lastReport >= 60
        fprintf('  t=%.0f/%.0fs...\n', time, scenarioDuration);
        lastReport = time;
    end

    txEmiss = emit(scenario);
    rxSigs  = propagate(scenario, txEmiss);
    dets    = detect(scenario, rxSigs);

    numDets = numel(dets);
    if numDets > 0
        scanIdx = scanIdx + 1;
        allDets{scanIdx} = dets;         %#ok<SAGROW>
        allTimes(scanIdx) = time;         %#ok<SAGROW>
        nTgt = sum(cellfun(@(d) d.ObjectAttributes{1}.TargetIndex > 0, dets));
        totalTargetDets = totalTargetDets + nTgt;
        totalClutter = totalClutter + (numDets - nTgt);
    end
end

fprintf('  Done: %d scans | %d target hits | %d clutter\n', ...
    scanIdx, totalTargetDets, totalClutter);

%% ====================== RAIN DEGRADATION ============================
if rainEnabled && scanIdx > 0
    fprintf('\nApplying rain degradation (%.0f mm/hr)...\n', rainRate_mmhr);
    droppedDets = 0;
    for k = 1:scanIdx
        dets = allDets{k};
        keepMask = true(numel(dets), 1);
        for jj = 1:numel(dets)
            d = dets{jj};
            range_m = norm(d.Measurement(1:3));
            % ITU-R P.838-3: rain attenuation at S-band (~2.8 GHz)
            % Specific attenuation ≈ 0.01 * R^0.95 dB/km one-way at S-band
            specAtten = 0.01 * rainRate_mmhr^0.95;   % dB/km one-way
            twoWayLoss_dB = 2 * specAtten * (range_m / 1000);
            % Convert to Pd reduction
            dropProb = 1 - 10^(-twoWayLoss_dB / 20);
            if rand() < dropProb
                keepMask(jj) = false;
                droppedDets = droppedDets + 1;
            else
                % Add noise proportional to rain
                noiseFactor = 1 + twoWayLoss_dB / 10;
                d.MeasurementNoise = d.MeasurementNoise * noiseFactor;
                dets{jj} = d;
            end
        end
        allDets{k} = dets(keepMask);
    end
    % Remove empty scans
    emptyMask = cellfun(@isempty, allDets(1:scanIdx));
    allDets = allDets(~emptyMask);
    allTimes = allTimes(~emptyMask);
    scanIdx = numel(allDets);
    fprintf('  Rain dropped %d detections | %d scans remain\n', droppedDets, scanIdx);
end

%% ====================== NORMALIZE DETECTIONS ========================
% Strip to 3D position-only for all trackers
for k = 1:scanIdx
    dets = allDets{k};
    for jj = 1:numel(dets)
        d = dets{jj};
        if numel(d.Measurement) > 3
            d.Measurement = d.Measurement(1:3);
            R = d.MeasurementNoise;
            if size(R,1) >= 3
                d.MeasurementNoise = R(1:3, 1:3);
            end
            mp = d.MeasurementParameters;
            if isstruct(mp)
                for m = 1:numel(mp)
                    if isfield(mp(m), 'HasVelocity')
                        mp(m).HasVelocity = false;
                    end
                end
                d.MeasurementParameters = mp;
            end
            dets{jj} = d;
        end
    end
    allDets{k} = dets;
end

%% ====================== RUN TRACKERS ================================
results = struct();

for tIdx = 1:numel(trackersToRun)
    tName = trackersToRun(tIdx);
    fprintf('\n============ %s Tracker ============\n', tName);

    if tName == "GNN"
        tracker = trackerGNN( ...
            'FilterInitializationFcn', @initcvekf, ...
            'AssignmentThreshold', [30 200], ...
            'ConfirmationThreshold', [2 3], ...
            'DeletionThreshold', [5 5], ...
            'MaxNumTracks', 20);
    else
        tracker = trackerJPDA( ...
            'FilterInitializationFcn', @initcvekf, ...
            'AssignmentThreshold', [30 200], ...
            'ConfirmationThreshold', [2 3], ...
            'DeletionThreshold', [5 5], ...
            'MaxNumTracks', 20);
    end

    trackLog = {};
    trackTimesLog = [];
    allPositions = [];
    allErrors = [];

    for k = 1:scanIdx
        dets = allDets{k};
        t = allTimes(k);

        if isempty(dets) && ~isLocked(tracker)
            continue;
        end

        tracks = tracker(dets, t);
        if ~isempty(tracks)
            trackLog{end+1} = tracks;           %#ok<SAGROW>
            trackTimesLog(end+1) = t;            %#ok<SAGROW>

            % Compute position error vs truth
            for jj = 1:numel(tracks)
                trkPos = tracks(jj).State([1 3 5])';
                % Find closest truth position at this time
                [~, wpI] = min(abs(toa - t));
                truthPos = wp(wpI, :);
                err = norm(trkPos - truthPos);
                allPositions = [allPositions; trkPos];   %#ok<AGROW>
                allErrors = [allErrors; err];             %#ok<AGROW>
            end
        end
    end

    % Store results
    results.(tName).trackLog = trackLog;
    results.(tName).trackTimes = trackTimesLog;
    results.(tName).positions = allPositions;
    results.(tName).errors = allErrors;

    nScansWithTracks = numel(trackLog);
    if ~isempty(allErrors)
        fprintf('  Confirmed track scans: %d\n', nScansWithTracks);
        fprintf('  Position RMS error: %.0f m\n', sqrt(mean(allErrors.^2)));
        fprintf('  Position max error: %.0f m\n', max(allErrors));
        fprintf('  Position mean error: %.0f m\n', mean(allErrors));
    else
        fprintf('  No confirmed tracks.\n');
    end
end

%% ====================== COMPARISON PLOTS ============================
fprintf('\nPlotting...\n');

trackerColors = struct('GNN', [0.8 0.2 0.8], 'JPDA', [1.0 0.5 0.0]);

% --- Figure 1: 3D + Top-Down ---
figure('Name', 'NASA Flight — Tracker Comparison', 'Position', [50 50 1400 700]);

% 3D view
subplot(1,2,1);
hold on; grid on; view(3);
title('3D View — Real Flight + Radar Tracking');
xlabel('North (m)'); ylabel('East (m)'); zlabel('Alt (m)');

% Truth
plot3(wp(:,1), wp(:,2), -wp(:,3), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Truth (FDR)');

% Detections
for k = 1:scanIdx
    for jj = 1:numel(allDets{k})
        d = allDets{k}{jj};
        m = d.Measurement(1:3);
        if d.ObjectAttributes{1}.TargetIndex > 0
            plot3(m(1), m(2), -m(3), 'g.', 'MarkerSize', 6, 'HandleVisibility', 'off');
        end
    end
end
plot3(NaN, NaN, NaN, 'g.', 'MarkerSize', 12, 'DisplayName', 'Detections');

% Tracks per tracker
for tIdx = 1:numel(trackersToRun)
    tName = trackersToRun(tIdx);
    r = results.(tName);
    if ~isempty(r.positions)
        clr = trackerColors.(tName);
        plot3(r.positions(:,1), r.positions(:,2), -r.positions(:,3), ...
            's', 'MarkerSize', 5, 'MarkerFaceColor', clr, 'MarkerEdgeColor', 'k', ...
            'DisplayName', sprintf('%s tracks', tName));
    end
end

plot3(0, 0, 0, 'k^', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'DisplayName', 'Radar');
legend('Location', 'best');

% Top-down view
subplot(1,2,2);
hold on; grid on;
title('Top-Down View');
xlabel('North (m)'); ylabel('East (m)');

plot(wp(:,1), wp(:,2), 'b-', 'LineWidth', 2.5, 'DisplayName', 'Truth (FDR)');

for k = 1:scanIdx
    for jj = 1:numel(allDets{k})
        d = allDets{k}{jj};
        m = d.Measurement(1:3);
        if d.ObjectAttributes{1}.TargetIndex > 0
            plot(m(1), m(2), 'g.', 'MarkerSize', 6, 'HandleVisibility', 'off');
        end
    end
end

for tIdx = 1:numel(trackersToRun)
    tName = trackersToRun(tIdx);
    r = results.(tName);
    if ~isempty(r.positions)
        clr = trackerColors.(tName);
        plot(r.positions(:,1), r.positions(:,2), ...
            's', 'MarkerSize', 5, 'MarkerFaceColor', clr, 'MarkerEdgeColor', 'k', ...
            'DisplayName', sprintf('%s tracks', tName));
    end
end

theta = linspace(0, 2*pi, 100);
plot(psrRange_m*cos(theta), psrRange_m*sin(theta), 'k--', 'DisplayName', 'PSR range');
plot(0, 0, 'k^', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'DisplayName', 'Radar');
legend('Location', 'best');
axis equal;

% --- Figure 2: Error comparison (if tracks exist) ---
hasAnyTracks = false;
for tIdx = 1:numel(trackersToRun)
    if ~isempty(results.(trackersToRun(tIdx)).errors)
        hasAnyTracks = true;
        break;
    end
end

if hasAnyTracks
    figure('Name', 'Tracker Error Comparison', 'Position', [100 100 800 500]);

    % Error bar chart
    subplot(1,2,1);
    hold on;
    names = {};
    rmsVals = [];
    maxVals = [];
    for tIdx = 1:numel(trackersToRun)
        tName = trackersToRun(tIdx);
        r = results.(tName);
        if ~isempty(r.errors)
            names{end+1} = char(tName);             %#ok<SAGROW>
            rmsVals(end+1) = sqrt(mean(r.errors.^2)); %#ok<SAGROW>
            maxVals(end+1) = max(r.errors);          %#ok<SAGROW>
        end
    end
    if ~isempty(names)
        bar(categorical(names), [rmsVals; maxVals]');
        legend('RMS Error', 'Max Error');
        ylabel('Position Error (m)');
        title('Tracker Accuracy');
        grid on;
    end

    % Track continuity
    subplot(1,2,2);
    hold on;
    for tIdx = 1:numel(trackersToRun)
        tName = trackersToRun(tIdx);
        r = results.(tName);
        if ~isempty(r.trackTimes) && ~isempty(r.errors)
            clr = trackerColors.(tName);
            % Plot error over time (one point per track update)
            nPts = min(numel(r.trackTimes), numel(r.errors));
            plot(r.trackTimes(1:nPts), r.errors(1:nPts), '-o', ...
                'Color', clr, 'MarkerSize', 4, 'MarkerFaceColor', clr, ...
                'DisplayName', sprintf('%s', tName));
        end
    end
    xlabel('Time (s)'); ylabel('Position Error (m)');
    title('Error Over Time');
    legend('Location', 'best');
    grid on;
end

%% ====================== SUMMARY ====================================
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║                       SUMMARY                          ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Flight: %s\n', fd.sourceFile);
fprintf('║  Ref origin: (%.4f°N, %.4f°W)\n', fd.refLat, abs(fd.refLon));
fprintf('║  Duration: %.0f s (%.1f min) | Waypoints: %d\n', fd.duration_s, fd.duration_s/60, fd.numWaypoints);
fprintf('║  Detections: %d target | %d clutter | Rain: %s\n', ...
    totalTargetDets, totalClutter, iif(rainEnabled, sprintf('%d mm/hr', rainRate_mmhr), 'OFF'));
fprintf('╠══════════════════════════════════════════════════════════╣\n');
for tIdx = 1:numel(trackersToRun)
    tName = trackersToRun(tIdx);
    r = results.(tName);
    if ~isempty(r.errors)
        fprintf('║  %s: RMS=%.0fm | Max=%.0fm | Scans=%d\n', ...
            tName, sqrt(mean(r.errors.^2)), max(r.errors), numel(r.trackLog));
    else
        fprintf('║  %s: No confirmed tracks\n', tName);
    end
end
fprintf('╚══════════════════════════════════════════════════════════╝\n');

%% ====================== HELPER =====================================
function out = iif(cond, a, b)
    if cond; out = a; else; out = b; end
end
