function scenario = createScenario(varargin)
%createScenario3D  DASR-configuration tracking scenario.
%
% DASR SENSOR ARCHITECTURE (ASR-11 / AN/GPN-30)
%   Sensor 1  : PSR  - Primary Search Radar
%               60 nm range, 12.5 RPM, 1.4 deg az beamwidth, 5 deg el
%               360-degree Rotator, S-band, passive reflection
%
%   Sensor 2  : MSSR - Monopulse Secondary Surveillance Radar (optional)
%               120 nm range, 12.5 RPM, 1.4 deg az beamwidth, 10 deg el
%               Co-mounted on PSR antenna, co-rotating at same RPM
%               Transponder interrogation -> identity-tagged detections
%               Pd=0.99, FAR=1e-7 (no weather clutter on SSR)
%
% NAME-VALUE PARAMETERS
%   NumTargets     : number of targets. Default 2.
%   SceneDuration  : seconds. Default 50.
%   EnableIFF      : add MSSR as sensor 2. Default false.
%
%   % PSR overrides (DASR defaults already set)
%   RadarRPM       : rev/min. Default 12.5.
%   RadarFOV       : [az; el] deg. Default [1.4; 5].
%   RadarTilt      : elevation tilt deg. Default 2.
%   RadarPd        : probability of detection. Default 0.9.
%   RadarFAR       : false alarm rate. Default 1e-6.
%   RadarRangeLim  : [min max] meters. Default [0 111120] (60 nm).
%   RadarRangeRes  : meters. Default 93 (~303 ft, DASR spec).
%
%   % MSSR overrides (only used if EnableIFF=true)
%   IFFUpdateRate  : ignored - MSSR uses same RPM as PSR.
%   IFFRangeSigma  : range noise (m). Default 100.
%   IFFAzSigma     : az noise (deg). Default 0.1.
%   IFFPd          : transponder Pd. Default 0.99.

p = inputParser;
addParameter(p, "NumTargets",    2,     @(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p, "SceneDuration", 50,    @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, "NumSensors",    1,     @(x)isnumeric(x)&&isscalar(x)&&x>=1);
addParameter(p, "EnableIFF",     false, @(x)islogical(x)||isnumeric(x));

% PSR parameters — DASR (ASR-11) defaults
addParameter(p, "RadarRPM",      12.5,           @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, "RadarFOV",      [1.4; 30],      @(x)isnumeric(x)&&numel(x)==2);
addParameter(p, "RadarSector",   [0 360],        @(x)isnumeric(x)&&numel(x)==2);
addParameter(p, "RadarTilt",     2,              @(x)isnumeric(x)&&isscalar(x));
addParameter(p, "RadarPd",       0.9,            @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
addParameter(p, "RadarFAR",      1e-6,           @(x)isnumeric(x)&&isscalar(x)&&x>=0);
addParameter(p, "RadarRangeLim", [0 111120],     @(x)isnumeric(x)&&numel(x)==2);  % 60 nm
addParameter(p, "RadarRangeRes", 93,             @(x)isnumeric(x)&&isscalar(x)&&x>0);  % 303 ft

% MSSR parameters (IFF)
addParameter(p, "IFFUpdateRate", 12.5,  @(x)isnumeric(x)&&isscalar(x)&&x>0);  % kept for API compat, ignored
addParameter(p, "IFFRangeSigma", 100,   @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, "IFFAzSigma",    0.1,   @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, "IFFPd",         0.99,  @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);

parse(p, varargin{:});
R = p.Results;

numTargets    = R.NumTargets;
sceneDuration = R.SceneDuration;
enableIFF     = logical(R.EnableIFF);

% With DASR config, NumSensors=1 means PSR only.
% EnableIFF=true adds the MSSR as sensor 2.
% We ignore NumSensors>1 for simplicity — DASR is a single PSR+MSSR unit.
numPrimary = 1;  % DASR has one PSR

%% Create scenario
scenario = trackingScenario;

%% PSR — Primary Search Radar (DASR ASR-11 spec)
fprintf('[PSR] Configuring DASR primary radar: %.1f RPM | Az=%.1f deg | Range=%.0f nm\n', ...
    R.RadarRPM, R.RadarFOV(1), R.RadarRangeLim(2)/1852);

[psr, psrMeta] = trackbench.sensors.buildCustomFusionRadarSensor(1, ...
    'rpm',         R.RadarRPM, ...
    'fov',         R.RadarFOV, ...
    'sector',      R.RadarSector, ...
    'tilt',        R.RadarTilt, ...
    'pd',          R.RadarPd, ...
    'far',         R.RadarFAR, ...
    'rangeLimits', R.RadarRangeLim, ...
    'rangeRes',    R.RadarRangeRes, ...
    'detCoords',   'Scenario', ...
    'hasINS',      true);

allSensors = {psr};
allMeta    = {psrMeta};

%% MSSR — Monopulse Secondary Surveillance Radar (co-mounted on PSR)
if enableIFF
    mssrIndex = 2;   % always sensor index 2 in DASR config
    fprintf('[MSSR] Configuring DASR secondary radar: %.1f RPM | Range=%.0f nm\n', ...
        R.RadarRPM, 222240/1852);

    [mssr, mssrMeta] = trackbench.sensors.buildIFFSensor(mssrIndex, ...
        'rpm',         R.RadarRPM, ...   % co-rotating with PSR
        'fov',         [R.RadarFOV(1); 10], ... % same az beam, wider el
        'tilt',        R.RadarTilt, ...
        'pd',          R.IFFPd, ...
        'rangeLimits', [0 222240], ...   % 120 nm
        'rangeRes',    R.IFFRangeSigma);

    allSensors{end+1} = mssr;
    allMeta{end+1}    = mssrMeta;
    fprintf('[Scenario] MSSR added as sensor index %d\n', mssrIndex);
else
    fprintf('[Scenario] MSSR disabled (EnableIFF=false)\n');
end

%% Tower platform
tower = platform(scenario, 'Sensors', allSensors); %#ok<NASGU>

% Scenario update rate driven by PSR
scenario.UpdateRate = psr.UpdateRate;

% Stash metadata
for k = 1:numel(allSensors)
    try
        if k <= numel(allMeta)
            allSensors{k}.UserData = allMeta{k};
        end
    catch
    end
end

%% Aircraft targets
switch numTargets
    case 1
        addAircraft(scenario, sceneDuration, 1);
    case 2
        addAircraft(scenario, sceneDuration, 1);
        addAircraft(scenario, sceneDuration, 2);
    otherwise
        for i = 1:numTargets
            addAircraft(scenario, sceneDuration, i);
        end
end

fprintf('[Scenario] DASR config: PSR(60nm) + MSSR(%s) | %d targets | %.0fs\n', ...
    ternary(enableIFF, '120nm', 'OFF'), numTargets, sceneDuration);
end

% =========================================================================
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function addAircraft(scenario, T, idx)
    switch idx
        case 1
            tNorm = [0 0.3 0.5 0.8 1.0];
            t = tNorm * T;
            wp = [ ...
                -2000    -20000   -3000;
                 0       -20000   -3000;
                 800     -19800   -3300;
                 1200    -19000   -3600;
                 1500    -18000   -4000];
            vel = zeros(numel(t), 3);
            for k = 1:numel(t)-1
                dt = t(k+1) - t(k);
                vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt;
            end
            vel(end,:) = vel(end-1,:);

        case 2
            tNorm = [0 0.25 0.5 0.75 1.0];
            t = tNorm * T;
            wp = [ ...
                2000     -19000   -3500;
                800      -19000   -3500;
                200      -19500   -3400;
                -600     -20000   -3300;
                -1200    -20500   -3200];
            vel = zeros(numel(t), 3);
            for k = 1:numel(t)-1
                dt = t(k+1) - t(k);
                if dt > 0
                    vel(k,:) = (wp(k+1,:) - wp(k,:)) / dt;
                else
                    vel(k,:) = vel(k-1,:);
                end
            end
            vel(end,:) = vel(end-1,:);

        otherwise
            t = [0 T]';
            wp = [ ...
                -1500 + 3000*rand,  -20500 + 3000*rand,  -2500 - 1500*rand;
                 1500 - 3000*rand,  -20500 + 3000*rand,  -2500 - 1500*rand];
            vel = repmat((wp(2,:) - wp(1,:)) / T, numel(t), 1);
    end

    tgt = platform(scenario);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', wp, ...
        'TimeOfArrival', t, ...
        'Velocities', vel);
end
