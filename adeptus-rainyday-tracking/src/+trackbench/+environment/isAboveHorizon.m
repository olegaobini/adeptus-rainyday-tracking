function visible = isAboveHorizon(sensorPos, targetPos, refractionFactor)
%isAboveHorizon  Check if a target is above the radar horizon.
%
% Uses the standard 4/3 effective Earth radius model for atmospheric
% refraction. A target is visible if the ground-range between sensor and
% target is less than the sum of both horizon distances.
%
% INPUTS
%   sensorPos : [x, y, z] in NED meters (z negative = altitude)
%   targetPos : [x, y, z] in NED meters, or Nx3 matrix for N targets
%   refractionFactor : (optional) atmospheric refraction multiplier.
%                      Default 4/3 (standard). Set to 1 for no refraction,
%                      >4/3 for ducting conditions, <4/3 for sub-refraction.
%
% OUTPUT
%   visible : logical (or Nx1 logical for multiple targets)
%             true = target is above radar horizon, false = blocked
%
% NOTES
%   Coordinates are NED: Z is negative for altitude above ground.
%   This function uses MATLAB Radar Toolbox 'horizonrange' internally.
%
% EXAMPLES
%   % Tower at 15m, target at 3km altitude, 60nm away
%   vis = isAboveHorizon([0 0 -15], [50000 -90000 -3000]);
%
%   % Same tower, target at 50m descending on approach at 30nm
%   vis = isAboveHorizon([0 0 -15], [0 -55560 -50]);
%
% See also: horizonrange, effearthradius

    if nargin < 3
        refractionFactor = 4/3;
    end

    % Earth radius with refraction
    Re = 6371000;  % mean Earth radius (m)
    Reff = refractionFactor * Re;

    % Extract altitudes from NED (negative Z = altitude above ground)
    sensorAlt = -sensorPos(3);
    sensorAlt = max(sensorAlt, 0);  % clamp to ground level

    % Handle single or multiple targets
    if size(targetPos, 1) == 1
        targetPos = targetPos(:)';
    end
    nTargets = size(targetPos, 1);
    visible = true(nTargets, 1);

    % Sensor horizon distance
    Rh_sensor = horizonrange(sensorAlt, Reff);

    for k = 1:nTargets
        targetAlt = -targetPos(k, 3);
        targetAlt = max(targetAlt, 0);

        % Target horizon distance
        Rh_target = horizonrange(targetAlt, Reff);

        % Maximum two-way detection range due to horizon
        maxRange = Rh_sensor + Rh_target;

        % Actual ground range (horizontal distance, ignoring altitude)
        dx = targetPos(k, 1) - sensorPos(1);
        dy = targetPos(k, 2) - sensorPos(2);
        groundRange = sqrt(dx^2 + dy^2);

        visible(k) = groundRange <= maxRange;
    end
end
