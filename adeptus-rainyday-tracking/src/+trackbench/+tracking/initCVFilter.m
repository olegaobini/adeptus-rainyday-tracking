function filter = initCVFilter(detection, filterParams)
%initCVFilter  Initialize a constant velocity EKF from a detection.
%
%  Creates a 6-state [x vx y vy z vz] constant velocity Extended Kalman
%  Filter using MATLAB's initcvekf, then overrides the process noise and
%  initial covariance with values from the JSON tracker config.
%
%  REFERENCES
%    [1] MathWorks, initcvekf:
%        https://www.mathworks.com/help/fusion/ref/initcvekf.html
%
%  See also: initIMMFilter, initcvekf, trackingEKF

if nargin < 2
    filterParams.init_speed_kmh = 900;
    filterParams.scale_accel_horz = 30;
    filterParams.scale_accel_vert = 20;
end

if ~isfield(filterParams, 'scale_accel_vert')
    filterParams.scale_accel_vert = 20;
end

filter = initcvekf(detection);
classToUse = class(filter.StateCovariance);

% Initial velocity covariance — all 3 axes
spd = filterParams.init_speed_kmh * 1e3 / 3600;
velCov = cast(spd^2, classToUse);
cov = filter.StateCovariance;
cov(2,2) = velCov;  % vx
cov(4,4) = velCov;  % vy
cov(6,6) = velCov;  % vz (was left at default — caused climb tracking issues)
filter.StateCovariance = cov;

% Process noise: [ax, ay, az] acceleration variances
scaleH = cast(filterParams.scale_accel_horz, classToUse);
scaleV = cast(filterParams.scale_accel_vert, classToUse);
Qh = scaleH^2;
Qv = scaleV^2;
filter.ProcessNoise = diag([Qh, Qh, Qv]);

end