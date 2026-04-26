%% scanNASAFlights — Profile all DASHlink .mat files and find multi-target combos
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Scans every .mat file in a folder, extracts airborne lat/lon/alt/speed,
%  and reports which flights overlap geographically (good for multi-target).

clear; clc;

% Resolved relative to project root so this script is portable.
% Override dataFolder with an absolute path if your data lives elsewhere.
snfThisDir  = fileparts(mfilename('fullpath'));      % .../adeptus-rainyday-tracking/scripts
snfProjRoot = fileparts(snfThisDir);                 % .../adeptus-rainyday-tracking
snfParent   = fileparts(snfProjRoot);                % .../Adding Flight Data
dataFolder  = fullfile(snfParent, "Tail_687_1");
clear snfThisDir snfProjRoot snfParent
files = dir(fullfile(dataFolder, '*.mat'));
fprintf('Found %d .mat files in %s\n\n', numel(files), dataFolder);

%% Profile each flight
results = [];
fprintf('Scanning flights...\n');
for i = 1:numel(files)
    fpath = fullfile(files(i).folder, files(i).name);
    try
        raw = load(fpath);
        if ~isfield(raw, 'LATP') || ~isfield(raw, 'LONP') || ~isfield(raw, 'ALT') || ~isfield(raw, 'GS')
            continue;
        end

        lat = double(raw.LATP.data(:));
        lon = double(raw.LONP.data(:));
        alt4 = double(raw.ALT.data(:));
        gs4  = double(raw.GS.data(:));

        nLat = numel(lat);
        alt = alt4(1:4:end); alt = alt(1:min(end,nLat));
        gs  = gs4(1:4:end);  gs  = gs(1:min(end,nLat));

        valid = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
        idx = find(valid);
        if numel(idx) < 60  % need at least 60s airborne
            continue;
        end

        r = struct();
        r.file = files(i).name;
        r.duration_s = idx(end) - idx(1);
        r.startLat = lat(idx(1));
        r.startLon = lon(idx(1));
        r.endLat   = lat(idx(end));
        r.endLon   = lon(idx(end));
        r.minLat   = min(lat(idx));
        r.maxLat   = max(lat(idx));
        r.minLon   = min(lon(idx));
        r.maxLon   = max(lon(idx));
        r.maxAlt_ft = max(alt(idx));
        r.maxGS_kts = max(gs(idx));
        r.numSamples = numel(idx);

        % Heading change metric (maneuverability)
        latV = lat(idx); lonV = lon(idx);
        dLat = diff(latV); dLon = diff(lonV);
        hdg = atan2d(dLon, dLat);
        hdgChange = abs(diff(hdg));
        hdgChange(hdgChange > 180) = 360 - hdgChange(hdgChange > 180);
        r.maxTurn_deg = max(hdgChange);
        r.totalTurn_deg = sum(hdgChange);

        results = [results; r]; %#ok<AGROW>
    catch
        % Skip files that can't be loaded
    end

    if mod(i, 50) == 0
        fprintf('  %d/%d...\n', i, numel(files));
    end
end

fprintf('\nProfiled %d valid flights out of %d files.\n\n', numel(results), numel(files));

%% Sort by duration (longest flights first)
[~, sortIdx] = sort([results.duration_s], 'descend');
results = results(sortIdx);

%% Display top flights
fprintf('=== TOP 20 LONGEST FLIGHTS ===\n');
fprintf('%-30s %7s %8s %10s %10s  %-20s → %-20s  %s\n', ...
    'File', 'Dur(s)', 'Alt(ft)', 'GS(kts)', 'Turn(deg)', 'Start', 'End', 'Route');
fprintf('%s\n', repmat('-', 1, 140));

for i = 1:min(20, numel(results))
    r = results(i);
    startStr = sprintf('(%.2f,%.2f)', r.startLat, r.startLon);
    endStr   = sprintf('(%.2f,%.2f)', r.endLat, r.endLon);
    
    % Estimate route distance
    distKm = sqrt((r.endLat-r.startLat)^2 + (r.endLon-r.startLon)^2) * 111;
    routeStr = sprintf('%.0f km', distKm);
    
    fprintf('%-30s %7.0f %8.0f %10.0f %10.1f  %-20s → %-20s  %s\n', ...
        r.file, r.duration_s, r.maxAlt_ft, r.maxGS_kts, r.totalTurn_deg, ...
        startStr, endStr, routeStr);
end

%% Find flights with most maneuvering
fprintf('\n=== TOP 10 MOST MANEUVERING (by total heading change) ===\n');
[~, turnIdx] = sort([results.totalTurn_deg], 'descend');
for i = 1:min(10, numel(results))
    r = results(turnIdx(i));
    fprintf('%-30s  dur=%5.0fs  maxTurn=%.0f°  totalTurn=%.0f°  alt=%.0fft\n', ...
        r.file, r.duration_s, r.maxTurn_deg, r.totalTurn_deg, r.maxAlt_ft);
end

%% Find geographic clusters (flights that overlap)
fprintf('\n=== GEOGRAPHIC OVERLAP SEARCH ===\n');
fprintf('Looking for flights that share airspace (lat/lon bounding box overlap)...\n\n');

nR = numel(results);
overlapPairs = {};
for i = 1:nR
    for j = i+1:nR
        ri = results(i); rj = results(j);
        % Check lat/lon bounding box overlap
        latOverlap = ri.minLat <= rj.maxLat && ri.maxLat >= rj.minLat;
        lonOverlap = ri.minLon <= rj.maxLon && ri.maxLon >= rj.minLon;
        if latOverlap && lonOverlap
            % Calculate overlap area
            oLatMin = max(ri.minLat, rj.minLat);
            oLatMax = min(ri.maxLat, rj.maxLat);
            oLonMin = max(ri.minLon, rj.minLon);
            oLonMax = min(ri.maxLon, rj.maxLon);
            overlapArea = (oLatMax - oLatMin) * (oLonMax - oLonMin);
            
            if overlapArea > 0.1  % meaningful overlap (> ~0.1 deg^2 ≈ 10km x 10km)
                overlapPairs{end+1} = struct('i', i, 'j', j, 'area', overlapArea); %#ok<SAGROW>
            end
        end
    end
end

% Sort by overlap area
if ~isempty(overlapPairs)
    areas = cellfun(@(p) p.area, overlapPairs);
    [~, aIdx] = sort(areas, 'descend');
    
    fprintf('Found %d pairs with significant geographic overlap.\n', numel(overlapPairs));
    fprintf('Top 15 overlapping pairs:\n\n');
    
    shown = 0;
    for k = 1:min(15, numel(aIdx))
        p = overlapPairs{aIdx(k)};
        ri = results(p.i); rj = results(p.j);
        % Skip if both are very short
        if ri.duration_s < 120 || rj.duration_s < 120; continue; end
        shown = shown + 1;
        fprintf('  Pair %d: overlap=%.2f deg²\n', shown, p.area);
        fprintf('    A: %-28s  dur=%5.0fs  alt=%5.0fft  (%6.2f,%7.2f)→(%6.2f,%7.2f)\n', ...
            ri.file, ri.duration_s, ri.maxAlt_ft, ri.startLat, ri.startLon, ri.endLat, ri.endLon);
        fprintf('    B: %-28s  dur=%5.0fs  alt=%5.0fft  (%6.2f,%7.2f)→(%6.2f,%7.2f)\n\n', ...
            rj.file, rj.duration_s, rj.maxAlt_ft, rj.startLat, rj.startLon, rj.endLat, rj.endLon);
    end
else
    fprintf('No significant geographic overlaps found.\n');
end

fprintf('\n=== SCAN COMPLETE ===\n');
