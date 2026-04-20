function flightData = loadNASAFlight(matFilePath, opts)
% loadNASAFlight  Load a NASA DASHlink FDR .mat file and produce NED waypoints.
%
%   flightData = loadNASAFlight(matFilePath)
%   flightData = loadNASAFlight(matFilePath, opts)
%
%   Reads a DASHlink Tail_XXX .mat file containing LATP, LONP, ALT, GS,
%   TRK fields (ARINC 717 decoded), extracts the airborne portion, converts
%   geodetic coordinates to NED relative to a reference origin, and returns
%   a struct ready for waypointTrajectory.
%
%   OPTIONS (name-value):
%     WaypointInterval - seconds between waypoints (default: 30)
%     MinGroundSpeed   - knots threshold for "airborne" (default: 50)
%     RefLat           - reference latitude (default: auto from midpoint)
%     RefLon           - reference longitude (default: auto from midpoint)
%     MaxDuration      - max seconds to use (default: Inf = all)
%
%   OUTPUT struct fields:
%     flightData.waypoints     - [N x 3] NED positions in meters [x_N, y_E, z_D]
%     flightData.timeOfArrival - [N x 1] time in seconds from start
%     flightData.velocities    - [N x 3] NED velocities in m/s
%     flightData.refLat        - reference latitude (deg)
%     flightData.refLon        - reference longitude (deg)
%     flightData.sourceFile    - path to the original .mat file
%     flightData.numWaypoints  - number of waypoints produced
%     flightData.duration_s    - total duration in seconds
%
%   See also: waypointTrajectory, trackbench.scenario.addTargetFromDef

    arguments
        matFilePath (1,1) string
        opts.MinGroundSpeed (1,1) double = 50
        opts.WaypointInterval (1,1) double = 30
        opts.RefLat (1,1) double = NaN
        opts.RefLon (1,1) double = NaN
        opts.MaxDuration (1,1) double = Inf
    end

    %% Load .mat file
    if ~isfile(matFilePath)
        error('trackbench:flightdata:fileNotFound', ...
            'Flight data file not found: %s', matFilePath);
    end
    raw = load(matFilePath);

    %% Extract fields — DASHlink stores each param as struct with .data, .Rate, etc.
    lat  = extractParam(raw, 'LATP');   % 1 Hz, degrees
    lon  = extractParam(raw, 'LONP');   % 1 Hz, degrees
    alt4 = extractParam(raw, 'ALT');    % 4 Hz, feet
    gs4  = extractParam(raw, 'GS');     % 4 Hz, knots
    trk4 = extractParam(raw, 'TRK');    % 4 Hz, degrees

    % Downsample 4 Hz fields to 1 Hz to match lat/lon
    nLat = numel(lat);
    alt = downsampleTo1Hz(alt4, nLat);
    gs  = downsampleTo1Hz(gs4, nLat);
    trk = downsampleTo1Hz(trk4, nLat);

    %% Identify airborne window
    validMask = (lat ~= 0) & (abs(lon) > 1) & (gs > opts.MinGroundSpeed);
    validIdx = find(validMask);

    if isempty(validIdx)
        error('trackbench:flightdata:noAirborneData', ...
            'No airborne data found in %s (no samples with GS > %.0f kts and valid lat/lon).', ...
            matFilePath, opts.MinGroundSpeed);
    end

    startIdx = validIdx(1);
    endIdx   = validIdx(end);

    % Apply max duration
    maxSamples = floor(opts.MaxDuration);
    if (endIdx - startIdx + 1) > maxSamples
        endIdx = startIdx + maxSamples - 1;
    end

    latAir = lat(startIdx:endIdx);
    lonAir = lon(startIdx:endIdx);
    altAir = alt(startIdx:endIdx);

    %% Convert units: feet -> meters
    altAir_m = altAir * 0.3048;

    %% Reference point for NED conversion
    if isnan(opts.RefLat)
        refLat = latAir(round(end/2));
    else
        refLat = opts.RefLat;
    end
    if isnan(opts.RefLon)
        refLon = lonAir(round(end/2));
    else
        refLon = opts.RefLon;
    end

    %% Convert geodetic to NED (flat-Earth, accurate < 200 km)
    [xN, yE, zD] = geodetic2nedLocal(latAir, lonAir, altAir_m, refLat, refLon, 0);

    %% Subsample to waypoint interval
    nSamples = numel(latAir);
    timeVec = (0:nSamples-1)';
    step = max(1, round(opts.WaypointInterval));
    wpIdx = 1:step:nSamples;
    if wpIdx(end) ~= nSamples
        wpIdx(end+1) = nSamples;
    end

    waypoints = [xN(wpIdx), yE(wpIdx), zD(wpIdx)];
    toa = timeVec(wpIdx);

    %% Compute velocities from waypoint differences
    nWP = size(waypoints, 1);
    vel = zeros(nWP, 3);
    for k = 1:nWP-1
        dt = toa(k+1) - toa(k);
        if dt > 0
            vel(k,:) = (waypoints(k+1,:) - waypoints(k,:)) / dt;
        end
    end
    vel(end,:) = vel(max(1,end-1),:);

    %% Package output
    flightData.waypoints     = waypoints;
    flightData.timeOfArrival = toa;
    flightData.velocities    = vel;
    flightData.refLat        = refLat;
    flightData.refLon        = refLon;
    flightData.sourceFile    = matFilePath;
    flightData.numWaypoints  = nWP;
    flightData.duration_s    = toa(end) - toa(1);

    fprintf('  [loadNASAFlight] Loaded %s\n', matFilePath);
    fprintf('    Airborne window: %d s (%.1f min)\n', flightData.duration_s, flightData.duration_s/60);
    fprintf('    Waypoints: %d at %d-s interval\n', nWP, step);
    fprintf('    Ref origin: (%.4f, %.4f)\n', refLat, refLon);
    fprintf('    NED range: X=[%.0f, %.0f] Y=[%.0f, %.0f] Z=[%.0f, %.0f] m\n', ...
        min(waypoints(:,1)), max(waypoints(:,1)), ...
        min(waypoints(:,2)), max(waypoints(:,2)), ...
        min(waypoints(:,3)), max(waypoints(:,3)));
end

%% ---- Local helpers ----

function data = extractParam(raw, fieldName)
    if ~isfield(raw, fieldName)
        error('trackbench:flightdata:missingField', ...
            'Required field "%s" not found in .mat file.', fieldName);
    end
    s = raw.(fieldName);
    % Unwrap nested (1,1) struct wrapper used by DASHlink .mat format
    while numel(s) == 1 && isstruct(s)
        if isfield(s, 'data')
            data = double(s.data(:));
            return;
        end
        s = s(1);
    end
    data = double(s(:));
end

function out = downsampleTo1Hz(data4hz, nTarget)
    out = data4hz(1:4:end);
    if numel(out) > nTarget
        out = out(1:nTarget);
    elseif numel(out) < nTarget
        out(end+1:nTarget) = out(end);
    end
end

function [xN, yE, zD] = geodetic2nedLocal(lat, lon, alt, refLat, refLon, refAlt)
    RE = 6378137;
    f  = 1/298.257223563;
    dLat = deg2rad(lat - refLat);
    dLon = deg2rad(lon - refLon);
    latRad = deg2rad(refLat);
    Rn = RE * (1 - f)^2 / (1 - (2*f - f^2) * sin(latRad)^2)^1.5;
    Re = RE / sqrt(1 - (2*f - f^2) * sin(latRad)^2);
    xN = dLat .* (Rn + refAlt);
    yE = dLon .* (Re + refAlt) .* cos(latRad);
    zD = -(alt - refAlt);
end
