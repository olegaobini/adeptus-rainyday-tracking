function filter = initCVFilter(detection, filterParams)
% initCVFilter - Initialize a constant velocity filter
% Modified to accept filterParams from the JSON configuration

% Fallback to defaults if no config parameters are provided
if nargin < 2
    filterParams.init_speed_kmh = 900;
    filterParams.scale_accel_horz = 30;
end

filter = initcvekf(detection);
classToUse = class(filter.StateCovariance);

% Use configured initial speed
spd = filterParams.init_speed_kmh * 1e3 / 3600; % convert km/h to m/s
velCov = cast(spd^2, classToUse);
cov = filter.StateCovariance;
cov(2,2) = velCov;
cov(4,4) = velCov;
filter.StateCovariance = cov;

% Use configured horizontal acceleration scale
scaleAccelHorz = cast(filterParams.scale_accel_horz, classToUse);
Gh = scaleAccelHorz;
Qh = Gh*Gh';
Q = blkdiag(Qh, Qh, 1);
filter.ProcessNoise = Q;
end
