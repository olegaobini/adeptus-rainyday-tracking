function flights = scanFlightFolder(folderPath, opts)
% scanFlightFolder  Profile NASA DASHlink .mat files in a folder.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   flights = scanFlightFolder(folderPath)
%   flights = scanFlightFolder(folderPath, opts)
%
%   Profiles every .mat file in `folderPath`, extracts the airborne window,
%   and returns a struct array — one row per valid flight. Files without
%   LATP/LONP/ALT/GS fields, or with fewer than MinAirborneSamples valid
%   samples, are silently skipped.
%
%   This is a callable refactor of scripts/scanNASAFlights.m (which is a
%   CLI tool that prints to the console). No console output, no display
%   logic — purely returns data. Intended for use by flightDataManagerGUI
%   and any other consumer that needs flight summaries.
%
%   OPTIONS (name-value):
%     MinGroundSpeed     - knots threshold for "airborne" (default: 50)
%     MinAirborneSamples - minimum samples to include (default: 60)
%     SortBy             - 'duration' (default), 'file', 'maxAlt', 'totalTurn'
%
%   OUTPUT struct fields per row:
%     file           - .mat file name (basename only)
%     fullPath       - absolute path
%     duration_s     - airborne duration in seconds
%     startLat, startLon, endLat, endLon  - start/end coordinates
%     minLat, maxLat, minLon, maxLon      - bounding box
%     maxAlt_ft      - peak altitude in feet
%     maxGS_kts      - peak ground speed in knots
%     maxTurn_deg    - largest single-sample heading change
%     totalTurn_deg  - cumulative heading change (maneuvering metric)
%     numSamples     - airborne sample count
%
%  See also: trackbench.flightdata.loadNASAFlight, scanNASAFlights

    arguments
        folderPath (1,:) char
        opts.MinGroundSpeed (1,1) double = 50
        opts.MinAirborneSamples (1,1) double = 60
        opts.SortBy (1,:) char {mustBeMember(opts.SortBy, ...
            {'duration', 'file', 'maxAlt', 'totalTurn'})} = 'duration'
    end

    flights = struct.empty(0, 1);
    if ~isfolder(folderPath)
        warning('trackbench:flightdata:notAFolder', ...
            'Folder does not exist: %s', folderPath);
        return;
    end

    files = dir(fullfile(folderPath, '*.mat'));
    nFiles = numel(files);
    if nFiles == 0
        return;
    end

    out = repmat(emptyRow(), nFiles, 1);
    count = 0;

    for i = 1:nFiles
        fpath = fullfile(files(i).folder, files(i).name);
        try
            raw = load(fpath);
        catch
            continue;  % unreadable .mat — skip
        end

        if ~isfield(raw, 'LATP') || ~isfield(raw, 'LONP') ...
                || ~isfield(raw, 'ALT') || ~isfield(raw, 'GS')
            continue;  % not DASHlink schema
        end

        try
            lat = double(raw.LATP.data(:));
            lon = double(raw.LONP.data(:));
            alt4 = double(raw.ALT.data(:));
            gs4  = double(raw.GS.data(:));
        catch
            continue;  % unexpected nested struct shape
        end

        nLat = numel(lat);
        alt = alt4(1:4:end); alt = alt(1:min(end, nLat));
        gs  = gs4(1:4:end);  gs  = gs(1:min(end, nLat));

        valid = (lat ~= 0) & (abs(lon) > 1) & (gs > opts.MinGroundSpeed);
        idx = find(valid);
        if numel(idx) < opts.MinAirborneSamples
            continue;
        end

        % Heading change metric (maneuverability)
        latV = lat(idx); lonV = lon(idx);
        dLat = diff(latV); dLon = diff(lonV);
        hdg = atan2d(dLon, dLat);
        hdgChange = abs(diff(hdg));
        hdgChange(hdgChange > 180) = 360 - hdgChange(hdgChange > 180);

        r = emptyRow();
        r.file           = files(i).name;
        r.fullPath       = fpath;
        r.duration_s     = idx(end) - idx(1);
        r.startLat       = lat(idx(1));
        r.startLon       = lon(idx(1));
        r.endLat         = lat(idx(end));
        r.endLon         = lon(idx(end));
        r.minLat         = min(lat(idx));
        r.maxLat         = max(lat(idx));
        r.minLon         = min(lon(idx));
        r.maxLon         = max(lon(idx));
        r.maxAlt_ft      = max(alt(idx));
        r.maxGS_kts      = max(gs(idx));
        if isempty(hdgChange)
            r.maxTurn_deg   = 0;
            r.totalTurn_deg = 0;
        else
            r.maxTurn_deg   = max(hdgChange);
            r.totalTurn_deg = sum(hdgChange);
        end
        r.numSamples     = numel(idx);

        count = count + 1;
        out(count) = r;
    end

    if count == 0
        flights = struct.empty(0, 1);
        return;
    end
    flights = out(1:count);

    % Sort
    switch opts.SortBy
        case 'duration'
            [~, sIdx] = sort([flights.duration_s], 'descend');
        case 'file'
            [~, sIdx] = sort({flights.file});
        case 'maxAlt'
            [~, sIdx] = sort([flights.maxAlt_ft], 'descend');
        case 'totalTurn'
            [~, sIdx] = sort([flights.totalTurn_deg], 'descend');
    end
    flights = flights(sIdx);
end


function r = emptyRow()
    r = struct( ...
        'file', '', ...
        'fullPath', '', ...
        'duration_s', 0, ...
        'startLat', 0, 'startLon', 0, ...
        'endLat', 0, 'endLon', 0, ...
        'minLat', 0, 'maxLat', 0, ...
        'minLon', 0, 'maxLon', 0, ...
        'maxAlt_ft', 0, ...
        'maxGS_kts', 0, ...
        'maxTurn_deg', 0, ...
        'totalTurn_deg', 0, ...
        'numSamples', 0);
end
