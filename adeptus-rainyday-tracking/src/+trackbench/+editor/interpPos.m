function pos = interpPos(waypoints, time_s, t)
%INTERPPOS  Linear interpolation of waypoint position at a query time.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   POS = interpPos(WAYPOINTS, TIME_S, T) returns the interpolated
%   position along the waypoint polyline at time T (seconds).
%
%   This is the ground-truth interpolation used by the M3.4 preview.
%   It deliberately matches the linear interpolation in
%   trackbench.scenario.addTargetFromDef so what the operator sees in
%   the preview equals what the simulator will actually do when the
%   scenario runs.  Splines are M4 — do not introduce them here.
%
%   Inputs
%     WAYPOINTS : Nx2 or Nx3 matrix (xy or xyz in metres)
%     TIME_S    : Nx1 vector of timestamps (seconds, monotone increasing)
%     T         : scalar query time (seconds).  Clamped to the path's
%                 [time_s(1), time_s(end)] endpoints — returning a
%                 waypoint exactly when T matches an entry in TIME_S is
%                 a first-class test case (see testPathEditor_M3_preview).
%
%   Output
%     POS : 1xD row vector (D = 2 or 3) matching the column count of
%           WAYPOINTS.
%
%   This function is pure / stateless and does not touch any handles;
%   it is safe to call from tests without a uifigure open.

    arguments
        waypoints (:,:) double
        time_s    (:,1) double
        t         (1,1) double
    end

    if size(waypoints, 1) ~= numel(time_s)
        error('interpPos:sizeMismatch', ...
            'waypoints has %d rows but time_s has %d entries.', ...
            size(waypoints, 1), numel(time_s));
    end
    if size(waypoints, 1) < 1
        error('interpPos:emptyPath', 'waypoints must be non-empty.');
    end

    % single waypoint -> always return it
    if size(waypoints, 1) == 1
        pos = waypoints(1, :);
        return;
    end

    % clamp outside the defined timeline
    if t <= time_s(1)
        pos = waypoints(1, :);
        return;
    end
    if t >= time_s(end)
        pos = waypoints(end, :);
        return;
    end

    % locate the segment containing t.  histc-style index via find is fine
    % here — N is small (typical path is 2..50 waypoints).
    idx = find(time_s <= t, 1, 'last');
    if idx >= numel(time_s)
        pos = waypoints(end, :);
        return;
    end

    dt = time_s(idx + 1) - time_s(idx);
    if dt <= 0
        % degenerate segment (shouldn't happen with monotone time_s, but
        % be defensive — fall back to the earlier waypoint).
        pos = waypoints(idx, :);
        return;
    end
    alpha = (t - time_s(idx)) / dt;
    pos = waypoints(idx, :) + alpha * (waypoints(idx + 1, :) - waypoints(idx, :));
end
