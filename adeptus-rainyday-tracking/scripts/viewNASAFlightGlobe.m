%% viewNASAFlightGlobe — Visualize saved NASA flight results on globe
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Loads cached detections and results from a previous runSingleScenario
%  run and displays them on a trackingGlobeViewer. No simulation needed.
%
%  USAGE:
%    viewNASAFlightGlobe("nasa_multi_target")   % or "nasa_flight_demo"

function viewNASAFlightGlobe(runName)

if nargin < 1
    runName = "nasa_multi_target";
end

fprintf('\n===== NASA Flight Globe Viewer =====\n\n');

%% Load cached detections
cacheFile = fullfile("cache", runName + ".mat");
if ~isfile(cacheFile)
    error('Cache file not found: %s\nRun the scenario first with runSingleScenario("%s")', cacheFile, runName);
end
fprintf('Loading cached detections from %s...\n', cacheFile);
cached = load(cacheFile);
if isfield(cached, 'detections')
    dataLog = cached.detections;
elseif isfield(cached, 'dataLog')
    dataLog = cached.dataLog;
else
    error('Cache file has no detections or dataLog field.');
end

%% Load run config to get flight file paths
configDir = fullfile(fileparts(mfilename('fullpath')), '..', 'config');
runFile = fullfile(configDir, 'runs', runName + ".json");
runDef = jsondecode(fileread(runFile));

% Load target config
tPath = char(runDef.targets);
if ~endsWith(tPath, '.json'); tPath = [tPath '.json']; end
targetFile = fullfile(configDir, 'targets', tPath);
targetDef = jsondecode(fileread(targetFile));

%% Extract reference origin and flight data
if iscell(targetDef.targets)
    targets = targetDef.targets;
else
    targets = num2cell(targetDef.targets);
end

numTargets = numel(targets);

% Get ref origin — check target config first, fall back to first flight midpoint
refLat = NaN; refLon = NaN;
if isfield(targets{1}, 'ref_lat'); refLat = targets{1}.ref_lat; end
if isfield(targets{1}, 'ref_lon'); refLon = targets{1}.ref_lon; end

fprintf('  Reference origin: (%.4f, %.4f)\n', refLat, refLon);
fprintf('  Targets: %d\n', numTargets);

%% Build truth trajectories in geodetic coords
flightRoutes = cell(1, numTargets);
rcsLabels = cell(1, numTargets);

for k = 1:numTargets
    tDef = targets{k};
    flightFile = string(tDef.flight_data_file);
    
    % Load raw FDR data
    raw = load(flightFile);
    lat = double(raw.LATP.data(:));
    lon = double(raw.LONP.data(:));
    alt4 = double(raw.ALT.data(:));
    gs4 = double(raw.GS.data(:));
    
    nLat = numel(lat);
    alt = alt4(1:4:end); alt = alt(1:min(end,nLat));
    gs = gs4(1:4:end); gs = gs(1:min(end,nLat));
    
    valid = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
    idx = find(valid);
    
    maxDur = tDef.max_duration_s;
    endIdx = min(idx(end), idx(1) + maxDur - 1);
    
    latAir = lat(idx(1):endIdx);
    lonAir = lon(idx(1):endIdx);
    altAir_m = alt(idx(1):endIdx) * 0.3048;
    
    % Subsample
    step = max(1, round(tDef.waypoint_interval_s));
    wpIdx = 1:step:numel(latAir);
    if wpIdx(end) ~= numel(latAir); wpIdx(end+1) = numel(latAir); end
    
    lla = [latAir(wpIdx), lonAir(wpIdx), altAir_m(wpIdx)];
    toa = (0:numel(wpIdx)-1)' * step;
    toa(end) = numel(latAir) - 1;
    
    flightRoutes{k} = geoTrajectory(lla, toa);
    
    label = "Target";
    if isfield(tDef, 'label'); label = string(tDef.label); end
    rcsStr = "";
    if isfield(tDef, 'rcs_dbsm'); rcsStr = sprintf(" (%d dBsm)", tDef.rcs_dbsm); end
    rcsLabels{k} = label + rcsStr;
    
    fprintf('  Flight %d: %s — %d waypoints, %.0fs\n', k, rcsLabels{k}, size(lla,1), toa(end));
    
    % Use first flight midpoint as fallback ref origin
    if isnan(refLat)
        refLat = lla(round(end/2), 1);
        refLon = lla(round(end/2), 2);
        fprintf('  (Auto ref origin from flight midpoint: %.4f, %.4f)\n', refLat, refLon);
    end
end

%% Launch globe viewer
fprintf('\nLaunching globe viewer...\n');

viewer = trackingGlobeViewer('Basemap', 'streets-dark', ...
    'TrackHistoryDepth', 5000);

% Plot truth trajectories
colors = [0 0.7 1; 1 0.5 0; 0.8 0 0.8; 0 0.8 0; 1 1 0];
for k = 1:numTargets
    plotTrajectory(viewer, flightRoutes{k}, 'Color', colors(mod(k-1,5)+1,:), ...
        'LineWidth', 3);
end

% Plot radar tower location marker
fprintf('Plotting radar location...\n');
try
    plotPlatform(viewer, struct('PlatformID', 100, 'Position', [refLat refLon 15], ...
        'Dimensions', struct('Length',30,'Width',30,'Height',15,'OriginOffset',[0 0 0])));
catch
    % Fallback: just note the radar position
    fprintf('  (Radar at %.4f, %.4f — not plottable as platform on globe)\n', refLat, refLon);
end

% Camera: zoom to flight area
campos(viewer, refLat, refLon, 200000);  % 200 km altitude
drawnow; pause(1);

fprintf('\n===== Globe viewer ready =====\n');
fprintf('  Run: %s\n', runName);
fprintf('  Targets: %d flights on globe\n', numTargets);
fprintf('  Radar: (%.4f, %.4f)\n', refLat, refLon);
fprintf('  Rotate the globe to explore the flight paths!\n');

end

function [lat, lon, alt] = ned2geodetic(xN, yE, zD, refLat, refLon, refAlt)
    RE = 6378137;
    f = 1/298.257223563;
    latRad = deg2rad(refLat);
    Rn = RE * (1 - f)^2 / (1 - (2*f - f^2) * sin(latRad)^2)^1.5;
    Re = RE / sqrt(1 - (2*f - f^2) * sin(latRad)^2);
    lat = refLat + rad2deg(xN / (Rn + refAlt));
    lon = refLon + rad2deg(yE / ((Re + refAlt) * cos(latRad)));
    alt = -zD + refAlt;
end
