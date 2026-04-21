function [isValid, info] = validateScanCoverage(scenario, duration, minScans)
%validateScanCoverage  Check that scenario duration produces enough scans.
%
% Inspects all sensors attached to the scenario, computes the expected
% number of complete scans, and warns/errors if insufficient for tracking.
%
% USAGE
%   [ok, info] = validateScanCoverage(scenario, duration);
%   [ok, info] = validateScanCoverage(scenario, duration, 8);
%
% INPUTS
%   scenario : trackingScenario with platforms and sensors attached
%   duration : scenario duration in seconds
%   minScans : minimum scans required (default 5)
%
% OUTPUTS
%   isValid  : true if all sensors will get >= minScans
%   info     : struct with per-sensor diagnostics:
%     .sensors     — table of sensor name, type, scanPeriod, expectedScans
%     .slowest     — name of the bottleneck sensor
%     .minDuration — minimum duration needed for minScans
%     .message     — human-readable summary
%
% The function prints a clear warning if the sim won't produce enough
% scans. It does NOT auto-change the config — the caller decides.
%
% TRACKER REQUIREMENTS
%   - Track initiation typically requires 3 detections (M-of-N logic)
%   - Track confirmation (score-based) needs 3-5 scans
%   - Meaningful metrics need 5+ scans minimum, 10+ preferred
%   - With degradation, missed detections mean you need even more scans
%
% See also: createScenario, loadScenario, buildSensor

arguments
    scenario
    duration (1,1) double {mustBePositive}
    minScans (1,1) double {mustBePositive} = 5
end

%% Collect sensor info from all platforms
sensorNames  = {};
sensorTypes  = {};
scanPeriods  = [];
updateRates  = [];
scanModes    = {};

plats = scenario.Platforms;
for p = 1:numel(plats)
    if iscell(plats)
        plat = plats{p};
    else
        plat = plats(p);
    end
    
    if ~isprop(plat, 'Sensors') || isempty(plat.Sensors)
        continue;
    end
    
    sensors = plat.Sensors;
    for s = 1:numel(sensors)
        if iscell(sensors)
            sen = sensors{s};
        else
            sen = sensors(s);
        end
        
        % Get sensor class name
        className = class(sen);
        shortName = regexp(className, '\.?(\w+)$', 'tokens', 'once');
        if ~isempty(shortName); shortName = shortName{1}; else; shortName = className; end
        
        % Get sensor index
        idx = NaN;
        if isprop(sen, 'SensorIndex'); idx = sen.SensorIndex; end
        
        % Determine scan period based on sensor type and scan mode
        scanMode = 'Unknown';
        scanPeriod = NaN;
        ur = NaN;
        
        if isprop(sen, 'UpdateRate')
            ur = sen.UpdateRate;
        end
        
        % Check scan mode
        if isprop(sen, 'ScanMode')
            scanMode = char(sen.ScanMode);
        end
        
        % Compute scan period depending on scan mode
        switch lower(scanMode)
            case {'mechanical', 'rotator'}
                % Rotating sensor: one full rotation or sector sweep
                if isprop(sen, 'MaxAzimuthScanRate') && isprop(sen, 'FieldOfView')
                    azScanRate = sen.MaxAzimuthScanRate;  % deg/s
                    fov = sen.FieldOfView;
                    
                    % Check if it's a sector or full-rotation scan
                    sectorSpan = 360;  % default full rotation
                    if isprop(sen, 'MechanicalAzimuthLimits')
                        limits = sen.MechanicalAzimuthLimits;
                        span = abs(diff(limits));
                        if span < 359
                            sectorSpan = span;
                        end
                    end
                    
                    if azScanRate > 0
                        scanPeriod = sectorSpan / azScanRate;
                    end
                end
                
            case 'sector'
                % Sector-scanning sensor (e.g. PAR, phased array)
                if isprop(sen, 'MaxAzimuthScanRate') && isprop(sen, 'FieldOfView')
                    azScanRate = sen.MaxAzimuthScanRate;
                    
                    sectorSpan = 40;  % default
                    if isprop(sen, 'MechanicalAzimuthLimits')
                        limits = sen.MechanicalAzimuthLimits;
                        sectorSpan = abs(diff(limits));
                    end
                    
                    if azScanRate > 0
                        % Sector scan: forward + back sweep
                        scanPeriod = 2 * sectorSpan / azScanRate;
                    end
                end
                
            case {'raster'}
                % Raster scan — treat like rotator for period
                if isprop(sen, 'MaxAzimuthScanRate') && sen.MaxAzimuthScanRate > 0
                    scanPeriod = 360 / sen.MaxAzimuthScanRate;
                end
                
            case {'no scanning', 'electronic', 'none'}
                % Staring or electronic-scan sensor — each update is a "scan"
                if ~isnan(ur) && ur > 0
                    scanPeriod = 1 / ur;
                end
                
            otherwise
                % Fallback: use update rate
                if ~isnan(ur) && ur > 0
                    scanPeriod = 1 / ur;
                end
        end
        
        sensorNames{end+1}  = sprintf('Sensor %d (%s)', idx, shortName); %#ok<AGROW>
        sensorTypes{end+1}  = scanMode; %#ok<AGROW>
        scanPeriods(end+1)   = scanPeriod; %#ok<AGROW>
        updateRates(end+1)   = ur; %#ok<AGROW>
        scanModes{end+1}     = scanMode; %#ok<AGROW>
    end
end

%% Compute expected scans and find bottleneck
expectedScans = floor(duration ./ scanPeriods);
expectedScans(isnan(scanPeriods)) = NaN;

% Find the slowest sensor (longest scan period = fewest scans)
[worstScans, worstIdx] = min(expectedScans);
slowestPeriod = scanPeriods(worstIdx);
slowestName   = sensorNames{worstIdx};

% Minimum duration to get minScans from the slowest sensor
minDuration = ceil(minScans * slowestPeriod);

%% Build output
isValid = worstScans >= minScans;

info = struct();
info.sensorNames   = sensorNames;
info.sensorTypes   = sensorTypes;
info.scanPeriods   = scanPeriods;
info.expectedScans = expectedScans;
info.slowest       = slowestName;
info.slowestPeriod = slowestPeriod;
info.worstScans    = worstScans;
info.minDuration   = minDuration;
info.minScans      = minScans;

%% Print diagnostics
fprintf('\n[SCAN CHECK] Duration=%.1fs | Minimum scans needed=%d\n', duration, minScans);
fprintf('  %-35s %-15s %10s %10s\n', 'Sensor', 'ScanMode', 'Period(s)', 'Scans');
fprintf('  %s\n', repmat('-', 1, 75));

for i = 1:numel(sensorNames)
    flag = '';
    if expectedScans(i) < minScans
        flag = ' *** INSUFFICIENT';
    end
    fprintf('  %-35s %-15s %10.2f %10d%s\n', ...
        sensorNames{i}, scanModes{i}, scanPeriods(i), expectedScans(i), flag);
end

if isValid
    info.message = sprintf('OK: All sensors get >= %d scans in %.1fs.', minScans, duration);
    fprintf('[SCAN CHECK] %s\n\n', info.message);
else
    info.message = sprintf( ...
        'WARNING: "%s" only gets %d scans in %.1fs (need %d). Minimum duration: %.0fs.', ...
        slowestName, worstScans, duration, minScans, minDuration);
    fprintf('\n');
    fprintf('  ************************************************************\n');
    fprintf('  * WARNING: %s\n', slowestName);
    fprintf('  * Only %d scan(s) in %.1fs — trackers need at least %d.\n', worstScans, duration, minScans);
    fprintf('  * MINIMUM DURATION: %.0f seconds\n', minDuration);
    fprintf('  * Recommended:      %.0f seconds (for robust metrics)\n', minDuration * 2);
    fprintf('  ************************************************************\n\n');
end

end
