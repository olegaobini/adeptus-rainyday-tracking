function [w, profile, dataLog] = getWeather(t, cfg, dataLog)
%getWeather  Compute weather severity at time t and log to dataLog.
%
% INPUTS
%   t       - current simulation time (seconds)
%   cfg     - full config struct; reads cfg.degradation.*
%   dataLog - detection log struct; appends to dataLog.WeatherSeverity
%
% OUTPUTS
%   w       - scalar severity in [0, 1]. 0 = clear, 1 = full storm.
%   profile - struct with storm window metadata (for diagnostics)
%   dataLog - updated with WeatherSeverity appended

%% 1. Extract degradation config with safe defaults
d = cfg.degradation;

storm_start = getOrDefault(d, 'storm_start_s', 5);
storm_end   = getOrDefault(d, 'storm_end_s',   45);
active_type = getOrDefault(d, 'active_type',   'step');

%% 2. Validate storm window
duration_s = cfg.scenario.duration_s;

if storm_start < 0
    error('getWeather:invalidWindow', ...
        'storm_start_s (%.1f) cannot be negative.', storm_start);
end

if storm_end <= storm_start
    error('getWeather:invalidWindow', ...
        'storm_end_s (%.1f) must be greater than storm_start_s (%.1f).', ...
        storm_end, storm_start);
end

if storm_end > duration_s
    error('getWeather:invalidWindow', ...
        'storm_end_s (%.1f) exceeds scenario duration (%.1f s).', ...
        storm_end, duration_s);
end

%% 3. Compute w based on active_type
switch lower(active_type)

    case 'step'
        % Binary: 0 outside storm window, 1 inside
        if t >= storm_start && t <= storm_end
            w = 1.0;
        else
            w = 0.0;
        end

    case 'ramp'
        % Linear fade in and out over the storm window
        storm_duration = storm_end - storm_start;
        if t < storm_start || t > storm_end
            w = 0.0;
        elseif t <= (storm_start + storm_duration/2)
            w = (t - storm_start) / (storm_duration/2);
        else
            w = (storm_end - t) / (storm_duration/2);
        end

    case 'pulse'
        % Full severity for first 20% of window, then clears
        pulse_end = storm_start + 0.2 * (storm_end - storm_start);
        if t >= storm_start && t <= pulse_end
            w = 1.0;
        else
            w = 0.0;
        end

    otherwise
        error('getWeather:unknownType', ...
            'Unknown active_type "%s". Expected step, ramp, or pulse.', ...
            active_type);
end

%% 4. Build profile struct
profile.storm_start = storm_start;
profile.storm_end   = storm_end;
profile.active_type = active_type;
profile.w           = w;
profile.in_storm    = (w > 0);

%% 5. Append to dataLog.WeatherSeverity (wScan caching pattern)
if ~isfield(dataLog, 'WeatherSeverity')
    dataLog.WeatherSeverity = [];
end

% Only log once per scan, not once per update
% Caller is responsible for only calling getWeather once per scan
dataLog.WeatherSeverity(end+1) = w;

end

%% Local helpers
function val = getOrDefault(s, field, default)
    if isstruct(s) && isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end
