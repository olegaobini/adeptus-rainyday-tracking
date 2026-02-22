function filter = initIMMFilter(detection, filterParams)
% initIMMFilter - Initialize an IMM filter with constant velocity and constant turn models
% Modified to accept filterParams from the JSON configuration

% Fallback to defaults if no config parameters are provided
if nargin < 2
    filterParams.init_speed_kmh = 900;
    filterParams.imm_transition_prob = 0.97;
    filterParams.scale_accel_horz = 10;
    filterParams.scale_omega_dot = 30;
end

filter1 = initcvekf(detection);
filter2 = initctekf(detection);

% Use configured transition probability
filter = trackingIMM({filter1,filter2}, 'TransitionProbabilities', filterParams.imm_transition_prob);
classToUse = class(filter.StateCovariance);

% Use configured initial speed
spd = filterParams.init_speed_kmh * 1e3 / 3600; % convert km/h to m/s
velCov = cast(spd^2, classToUse);

% Use configured acceleration scales
scaleAccelHorz = cast(filterParams.scale_accel_horz, classToUse); 
scaleOmegaDot = cast(filterParams.scale_omega_dot, classToUse);  

for i = 1:numel(filter.TrackingFilters)
    filter.TrackingFilters{i}.StateCovariance(2:2:4,2:2:4) = blkdiag(velCov,velCov);
    Gh = scaleAccelHorz;
    Qh = Gh*Gh';
    filter.TrackingFilters{i}.ProcessNoise(1:2,1:2) = blkdiag(Qh, Qh);
    
    if strcmpi(func2str(filter.TrackingFilters{i}.StateTransitionFcn),'constturn')
        Qo = scaleOmegaDot^2;
        Q(1:2,1:2) = blkdiag(Qh, Qh);
        Q(3,3) = Qo;
    end
end
end