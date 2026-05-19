%% runNASAFlightGlobe — Track real aircraft on an Earth-centered globe
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Earth-centered version of runNASAFlight. Uses geoTrajectory and
%  trackingGlobeViewer to visualize real flight recorder data on a globe,
%  with simulated ARSR-4 long-range surveillance radars along the route.
%
%  Based on MathWorks example:
%  https://www.mathworks.com/help/fusion/ug/simulate-and-track-en-route-air-traffic-with-earth-centered-scenarios.html
%
%  DATA SOURCE: https://c3.ndc.nasa.gov/dashlink/resources/664/
%
%  See also: trackingGlobeViewer, geoTrajectory, trackingScenario

clear; clc; close all;

%% ====================== USER CONFIG =================================

% Path to the DASHlink .mat file
% Resolved relative to the project root so this script is portable.
% Override with an absolute path if your data lives elsewhere.
rnfgThisDir  = fileparts(mfilename('fullpath'));      % .../adeptus-rainyday-tracking/scripts
rnfgProjRoot = fileparts(rnfgThisDir);                % .../adeptus-rainyday-tracking
rnfgParent   = fileparts(rnfgProjRoot);               % .../Adding Flight Data
flightFile   = fullfile(rnfgParent, "Tail_687_1", "687200104121330.mat");
clear rnfgThisDir rnfgProjRoot rnfgParent

% How much of the flight to use (seconds)
maxDuration_s = 600;    % 10 minutes to start (increase once working)

% Waypoint spacing (seconds)
waypointInterval_s = 10;

% RCS of the target (dBsm). 10 ≈ Boeing 737 class
rcs_dbsm = 10;

% Tracker: 'GNN' or 'JPDA'
trackerType = "GNN";

%% ====================== LOAD FLIGHT DATA ============================
fprintf('\n===== NASA DASHlink Globe Tracker =====\n\n');

fd = trackbench.flightdata.loadNASAFlight(flightFile, ...
    'MaxDuration', maxDuration_s, ...
    'WaypointInterval', waypointInterval_s);

% We need lat/lon/alt directly for geoTrajectory — reload raw data
raw = load(flightFile);

% Extract fields at 1 Hz
lat = extractFDRParam(raw, 'LATP');
lon = extractFDRParam(raw, 'LONP');
alt4 = extractFDRParam(raw, 'ALT');   % 4 Hz, feet
gs4  = extractFDRParam(raw, 'GS');    % 4 Hz, knots

% Downsample 4 Hz to 1 Hz
nLat = numel(lat);
alt = alt4(1:4:end); alt = alt(1:min(end,nLat));
gs  = gs4(1:4:end);  gs  = gs(1:min(end,nLat));

% Find airborne window
validMask = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
validIdx = find(validMask);
startIdx = validIdx(1);
endIdx = min(validIdx(end), startIdx + maxDuration_s - 1);

latAir = lat(startIdx:endIdx);
lonAir = lon(startIdx:endIdx);
altAir_m = alt(startIdx:endIdx) * 0.3048;  % feet → meters

% Subsample to waypoint interval
nSamples = numel(latAir);
step = max(1, round(waypointInterval_s));
wpIdx = 1:step:nSamples;
if wpIdx(end) ~= nSamples
    wpIdx(end+1) = nSamples;
end

lla = [latAir(wpIdx), lonAir(wpIdx), altAir_m(wpIdx)];
toa = (0:numel(wpIdx)-1)' * step;
toa(end) = nSamples - 1;  % exact end time

scenarioDuration = toa(end) + 1;

fprintf('  Flight: %.0f s (%.1f min), %d waypoints\n', scenarioDuration, scenarioDuration/60, size(lla,1));
fprintf('  Lat: [%.4f, %.4f], Lon: [%.4f, %.4f]\n', min(lla(:,1)), max(lla(:,1)), min(lla(:,2)), max(lla(:,2)));
fprintf('  Alt: [%.0f, %.0f] m\n', min(lla(:,3)), max(lla(:,3)));

%% ====================== BUILD EARTH-CENTERED SCENARIO ===============
fprintf('\nBuilding earth-centered scenario...\n');

scene = trackingScenario('IsEarthCentered', true, 'UpdateRate', 1, ...
    'StopTime', scenarioDuration);

% --- Aircraft target (real flight path) ---
flightRoute = geoTrajectory(lla, toa);
airplane = platform(scene, 'Trajectory', flightRoute, ...
    'Signatures', {rcsSignature('Pattern', rcs_dbsm)});

% --- Place ARSR-4 radars along the flight route ---
% Pick 2-3 positions spread along the flight path
nRadars = min(3, floor(size(lla,1)/2));
radarIdxs = round(linspace(1, size(lla,1), nRadars + 2));
radarIdxs = radarIdxs(2:end-1);  % skip first and last (too close to endpoints)

radarLLA = [lla(radarIdxs, 1:2), zeros(nRadars, 1)];  % ground level

fprintf('  Radar sites:\n');
for k = 1:nRadars
    fprintf('    Radar %d: (%.4f, %.4f)\n', k, radarLLA(k,1), radarLLA(k,2));
end

% Model ARSR-4 long-range surveillance radar
for k = 1:nRadars
    radar = fusionRadarSensor(k, ...
        'UpdateRate', 1/12, ...                    % 12-sec rotation
        'FieldOfView', [360; 30], ...
        'HasElevation', true, ...
        'HasINS', true, ...
        'HasRangeRate', true, ...
        'HasFalseAlarms', false, ...
        'ReferenceRange', 463000, ...              % 250 nm
        'ReferenceRCS', 0, ...                     % 1 m^2 target
        'RangeLimits', [0 463000], ...
        'AzimuthResolution', 1.4, ...
        'RangeResolution', 323, ...
        'DetectionCoordinates', 'Scenario');        % Output in ECEF

    platform(scene, 'Position', radarLLA(k,:), ...
        'Signatures', {rcsSignature('Pattern', -50)}, ...
        'Sensors', {radar});
end

%% ====================== VISUALIZE ON GLOBE ==========================
fprintf('\nLaunching globe viewer...\n');

viewer = trackingGlobeViewer('Basemap', 'streets-dark', ...
    'TrackLabelScale', 1.3, 'TrackHistoryDepth', 4000);

% Show flight route
plotTrajectory(viewer, flightRoute);

% Show radar platforms and coverage
plotPlatform(viewer, scene.Platforms(2:end));
covcon = coverageConfig(scene);
plotCoverage(viewer, covcon, 'ECEF', 'Alpha', 0.1);

% Camera: zoom to flight midpoint
midLLA = lla(round(end/2), :);
campos(viewer, midLLA(1), midLLA(2), 500000);  % 500 km altitude view

drawnow;
fprintf('  Globe viewer ready.\n');

%% ====================== GENERATE DETECTIONS & TRACK =================
fprintf('\nRunning simulation (%.0f s at %d Hz)...\n', scenarioDuration, 1);

% Tracker
if trackerType == "GNN"
    tracker = trackerGNN('TrackerIndex', 1, ...
        'MaxNumTracks', 50, ...
        'FilterInitializationFcn', @initcvekf, ...
        'ConfirmationThreshold', [3 5], ...
        'DeletionThreshold', [5 5], ...
        'AssignmentThreshold', [250 2000]);
else
    tracker = trackerJPDA('TrackerIndex', 1, ...
        'MaxNumTracks', 50, ...
        'FilterInitializationFcn', @initcvekf, ...
        'ConfirmationThreshold', [3 5], ...
        'DeletionThreshold', [5 5], ...
        'AssignmentThreshold', [250 2000]);
end

totalDets = 0;
totalTracks = 0;
lastReportTime = -60;

while advance(scene)
    time = scene.SimulationTime;

    % Progress
    if time - lastReportTime >= 60
        fprintf('  t=%.0f/%.0fs...\n', time, scenarioDuration);
        lastReportTime = time;
    end

    % Emit → propagate → detect
    txEmiss = emit(scene);
    rxSigs  = propagate(scene, txEmiss);
    dets    = detect(scene, rxSigs);

    % Normalize detections to 3D position
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

    if ~isempty(dets)
        totalDets = totalDets + numel(dets);
    end

    % Track — first call must have detections
    if ~isempty(dets) || isLocked(tracker)
        tracks = tracker(dets, time);
    else
        tracks = [];
    end
    if ~isempty(tracks)
        totalTracks = totalTracks + 1;
    end

    % Update globe visualization
    if ~isempty(dets)
        plotDetection(viewer, dets);
    end
    if ~isempty(tracks)
        plotTrack(viewer, tracks);
    end
end

fprintf('\n===== Done =====\n');
fprintf('  Duration: %.0f s (%.1f min)\n', scenarioDuration, scenarioDuration/60);
fprintf('  Total detections: %d\n', totalDets);
fprintf('  Scans with confirmed tracks: %d\n', totalTracks);
fprintf('  Ref: Lat [%.2f, %.2f], Lon [%.2f, %.2f]\n', ...
    min(lla(:,1)), max(lla(:,1)), min(lla(:,2)), max(lla(:,2)));

%% ====================== LOCAL HELPERS ===============================
function data = extractFDRParam(raw, fieldName)
    s = raw.(fieldName);
    while numel(s) == 1 && isstruct(s)
        if isfield(s, 'data')
            data = double(s.data(:));
            return;
        end
        s = s(1);
    end
    data = double(s(:));
end
