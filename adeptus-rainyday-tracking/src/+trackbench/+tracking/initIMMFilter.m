function filter = initIMMFilter(detection, filterParams)
%initIMMFilter  Initialize an IMM filter with constant velocity and constant turn models.
%
%  PROCESS NOISE
%    scale_accel_horz : horizontal acceleration (X,Y). 30 m/s² default.
%    scale_accel_vert : vertical acceleration (Z). 20 m/s² default.
%    scale_omega_dot  : turn rate change for CT model. 30 deg/s².
%
%  REFERENCES
%    [1] MathWorks, trackingIMM:
%        https://www.mathworks.com/help/fusion/ref/trackingimm.html
%    [2] MathWorks, "Tracking Maneuvering Targets":
%        https://www.mathworks.com/help/fusion/ug/tracking-maneuvering-targets.html
%
%  See also: trackingIMM, initcvekf, initctekf, initCVFilter

if nargin < 2
    filterParams.init_speed_kmh = 900;
    filterParams.imm_transition_prob = 0.97;
    filterParams.scale_accel_horz = 30;
    filterParams.scale_accel_vert = 20;
    filterParams.scale_omega_dot = 30;
end

if ~isfield(filterParams, 'scale_accel_vert')
    filterParams.scale_accel_vert = 20;
end

% Create individual filters
filter1 = initcvekf(detection);
filter2 = initctekf(detection);

classToUse = class(filter1.StateCovariance);

% Process noise scales
spd = filterParams.init_speed_kmh * 1e3 / 3600;
velCov = cast(spd^2, classToUse);
scaleH = cast(filterParams.scale_accel_horz, classToUse);
scaleV = cast(filterParams.scale_accel_vert, classToUse);
scaleO = cast(filterParams.scale_omega_dot, classToUse);
Qh = scaleH^2;
Qv = scaleV^2;
Qo = scaleO^2;

% ── Modify CV filter (state = [x vx y vy z vz], 6 states) ──
% Velocity covariance: allow high initial speed uncertainty on all axes
filter1.StateCovariance(2,2) = velCov;  % vx
filter1.StateCovariance(4,4) = velCov;  % vy
filter1.StateCovariance(6,6) = velCov;  % vz
% Process noise: [ax, ay, az]
filter1.ProcessNoise = diag([Qh, Qh, Qv]);

% ── Modify CT filter (state = [x vx y vy omega z vz], 7 states) ──
% Velocity covariance
filter2.StateCovariance(2,2) = velCov;  % vx
filter2.StateCovariance(4,4) = velCov;  % vy
filter2.StateCovariance(7,7) = velCov;  % vz
% Process noise: [ax, ay, alpha, az]
filter2.ProcessNoise = diag([Qh, Qh, Qo, Qv]);

% ── Create IMM from the already-configured filters ──
% trackingIMM.TrackingFilters is READ-ONLY after construction,
% so all filter modifications must happen BEFORE this line.
filter = trackingIMM({filter1, filter2}, ...
    'TransitionProbabilities', filterParams.imm_transition_prob);

end