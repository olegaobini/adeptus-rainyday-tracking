function exportMateoV5FromDataLog(dataLog, outFile, varargin)
%exportMateoV5FromDataLog  Export MATLAB detection logs to a MateoV5-style text file
%                          compatible with the previous capstone's Python Kalman filter.
%
% WHY THIS EXISTS (context for the team)
%   A previous capstone team built a Python Kalman Filter (KF) pipeline that expects
%   a simple text file with one scalar measurement per time step:
%
%       0.26s - "m":   20229.623030
%       0.26s - "mps": 0.000000
%
%   Our MATLAB radar scenario produces *multiple detections per scan* (and sometimes
%   none). This helper reduces each scan to a single "best" scalar measurement so the
%   legacy Python KF can run without modification.
%
% WHAT IT WRITES
%   For each selected scan time t:
%     - "m"   : scalar measurement (range or downrange)
%     - "mps" : scalar speed estimate (meters per second)
%
%   NOTE: In the current version we explicitly FORCE mps = 0 at write-time to support
%         "position-only experiments" (we preserve the finite-difference computation
%         in code for future use).
%
% INPUTS
%   dataLog : struct from helperRunDetections with at least:
%               dataLog.Time        : Nx1 or 1xN scan times (seconds)
%               dataLog.Detections  : 1xN cell, each cell contains scan detections
%                                     (cell array of objectDetection or objectDetection array)
%   outFile : output filename (e.g., "MateoV5.txt")
%
% NAME-VALUE OPTIONS (how you control the reduction to 1 measurement/scan)
%   'Mode'          : "range" (default) or "downrange_y"
%                     "range"       => m = sqrt(x^2 + y^2 + z^2)
%                     "downrange_y" => m = abs(y)
%
%   'PickRule'      : "closestToPrev" (default) or "smallest"
%                     "closestToPrev" => pick the candidate closest to last picked m
%                                         (helps keep continuity when multiple detections exist)
%                     "smallest"       => always pick the smallest m (simple, but can jump targets)
%
%   'MinRange'      : minimum allowed m (meters) (default 0)
%   'MaxRange'      : maximum allowed m (meters) (default inf)
%
%   'WriteMissing'  : "skip" (default) or "hold"
%                     "skip" => if a scan has zero valid detections, omit that time step entirely
%                     "hold" => if missing, repeat last m and set mps=0
%                               (keeps uniform time series for the Python KF)
%
%   'MaxJumpMeters' : max allowed |m(k) - m(k-1)| before treating the pick as clutter/wrong target
%                     (default 200). This acts like a 1D continuity gate.
%
% IMPORTANT LIMITATION (team read)
%   - This exporter intentionally collapses a *multi-target, multi-detection* scan into
%     a *single scalar*. That is NOT a multi-target solution. It is a compatibility bridge
%     for an older single-measurement Python KF demo pipeline.
%   - If you want multi-target export, we should write one file per target/track, or
%     export all detections as arrays with IDs.
%
% EXAMPLE
%   exportMateoV5FromDataLog(dataLog, "MateoV5.txt", ...
%       'Mode',"downrange_y", 'PickRule',"closestToPrev", ...
%       'WriteMissing',"hold", 'MaxJumpMeters',250);
% -------------------------------------------------------------------------

% -------- Parse options (robust, self-documenting configuration)
p = inputParser;
p.addParameter('Mode', "range");
p.addParameter('PickRule', "closestToPrev");
p.addParameter('MinRange', 0);
p.addParameter('MaxRange', inf);
p.addParameter('WriteMissing', "skip");
p.addParameter('MaxJumpMeters', 200);
p.parse(varargin{:});
opt = p.Results;

% -------- Validate minimal fields exist
if ~isfield(dataLog,'Time') || ~isfield(dataLog,'Detections')
    error('dataLog must contain fields Time and Detections.');
end

% Force time vector to column for consistent indexing
tVec = dataLog.Time(:);
detsCell = dataLog.Detections;

if numel(detsCell) ~= numel(tVec)
    error('dataLog.Time and dataLog.Detections must have the same length.');
end

% Open output file for writing
fid = fopen(outFile, 'w');
if fid < 0
    error('Could not open output file: %s', outFile);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU> % ensures file closes even on error

% Track last accepted measurement to support continuity-based picking
prevT = NaN;
prevM = NaN;
havePrev = false;

for k = 1:numel(tVec)
    t = tVec(k);
    dets = detsCell{k};

    % -------- Build candidate scalar measurements (mCandidates) for this scan
    % We may have multiple detections per scan. We convert each detection's
    % 3D measurement [x;y;z] into a scalar "m" depending on opt.Mode.
    mCandidates = [];

    if ~isempty(dets)
        % dataLog may store dets as either a cell array or an object array.
        if iscell(dets)
            detArray = [dets{:}];
        else
            detArray = dets;
        end

        for i = 1:numel(detArray)
            % Extract measurement vector (x,y,z) in meters
            z = detArray(i).Measurement(:);

            % Some sensors produce 2D measurements; pad z=0 so range math works.
            if numel(z) == 2
                z = [z; 0];
            elseif numel(z) < 2
                continue; % invalid measurement
            end

            % Convert measurement to scalar m depending on export mode
            switch string(opt.Mode)
                case "range"
                    mVal = sqrt(z(1)^2 + z(2)^2 + z(3)^2);
                case "downrange_y"
                    mVal = abs(z(2));
                otherwise
                    error('Unknown Mode: %s. Use "range" or "downrange_y".', opt.Mode);
            end

            % Apply min/max range filtering to reject nonsense or out-of-interest clutter
            if isfinite(mVal) && mVal >= opt.MinRange && mVal <= opt.MaxRange
                mCandidates(end+1,1) = mVal; %#ok<AGROW>
            end
        end
    end

    % -------- Handle scans with no valid candidates (missing measurement)
    if isempty(mCandidates)
        switch string(opt.WriteMissing)
            case "skip"
                % Skip writing anything for this time step.
                % Result: output file has non-uniform time spacing.
                continue;

            case "hold"
                % Repeat the last known good measurement if we have one.
                if ~havePrev
                    continue; % nothing to hold yet
                end
                mPicked = prevM;

                % Write measurement and forced zero speed
                fprintf(fid, '%.2fs - "m": %.6f\n', t, mPicked);
                mps = 0;   % position-only compatibility mode
                fprintf(fid, '%.2fs - "mps": %.6f\n', t, mps);

                % Advance time; keep prevM unchanged
                prevT = t;
                continue;

            otherwise
                error('Unknown WriteMissing: %s. Use "skip" or "hold".', opt.WriteMissing);
        end
    end

    % -------- Pick ONE candidate detection for this scan
    % This is where we reduce multi-detection scans to one scalar value.
    switch string(opt.PickRule)
        case "smallest"
            % Simple: always pick the closest (smallest range) detection.
            mPicked = min(mCandidates);

        case "closestToPrev"
            % Continuity-based: pick candidate closest to previous m.
            % Helps avoid jumping between targets/clutter when multiple candidates exist.
            if havePrev
                [~, idx] = min(abs(mCandidates - prevM));
                mPicked = mCandidates(idx);
            else
                mPicked = min(mCandidates);
            end

        otherwise
            error('Unknown PickRule: %s. Use "closestToPrev" or "smallest".', opt.PickRule);
    end

    % -------- Continuity gate: reject sudden jumps as clutter/wrong target
    % If the chosen measurement jumps too far from the previous one, treat it
    % as suspicious. Behavior depends on WriteMissing option:
    %   - skip: drop this scan
    %   - hold: output prevM instead
    if havePrev && isfinite(opt.MaxJumpMeters) && opt.MaxJumpMeters > 0
        if abs(mPicked - prevM) > opt.MaxJumpMeters
            switch string(opt.WriteMissing)
                case "skip"
                    continue;
                case "hold"
                    mPicked = prevM;
                otherwise
                    continue;
            end
        end
    end

    % -------- Compute mps by finite difference (kept for future use)
    % Even though we compute it, we currently FORCE mps=0 before writing.
    if ~havePrev
        mps = 0;
        havePrev = true;
    else
        dt = t - prevT;
        if ~(isfinite(dt) && dt > 0)
            mps = 0;
        else
            mps = (mPicked - prevM) / dt;
        end
    end

    % -------- Write output in the exact expected legacy format
    fprintf(fid, '%.2fs - "m": %.6f\n', t, mPicked);

    % Legacy compatibility mode: overwrite computed mps with 0 so the Python KF
    % is driven by a position-only measurement sequence.
    mps = 0;   % <<< Force speed to zero (position-only experiment)
    fprintf(fid, '%.2fs - "mps": %.6f\n', t, mps);

    % Update history for continuity picking and finite differencing
    prevT = t;
    prevM = mPicked;
end

end
