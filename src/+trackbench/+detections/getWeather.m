function [w, profile, dataLog] = getWeather(t, cfg, dataLog)
%getWeather  Compute weather severity at time t and log to dataLog.
%
%  Returns w ∈ [0, 1] representing storm intensity at time t.
%  Supports step (binary), ramp (gradual), and pulse (sharp burst) profiles.
%
% INPUTS
%   t       - current simulation time (seconds)
%   cfg     - config struct with cfg.degradation.storm_start_s, storm_end_s,
%             active_type, and cfg.scenario.duration_s
%   dataLog - detection log struct; appends to dataLog.WeatherSeverity
%
% OUTPUTS
%   w       - scalar severity in [0, 1]. 0 = clear, 1 = full storm.
%   profile - struct with storm window metadata
%   dataLog - updated with WeatherSeverity appended
%
% STORM PROFILES
%   'step'  - Binary on/off within the storm window
%   'ramp'  - Linear fade in during first half, fade out during second half
%   'pulse' - Full severity for first 20% of window, then clears
%
% Author: Daniel (Module 5 — Weather Degradation)

%% 1. Extract degradation config with safe defaults
d = cfg.degradation;

storm_start = getOrDefault(d, 'storm_start_s', 5);
storm_end   = getOrDefault(d, 'storm_end_s',   45);
active_type = getOrDefault(d, 'active_type',   'step');

%% 2. Validate storm window
duration_s = cfg.scenario.duration_s;

if storm_start < 0
    warning('getWeather:invalidWindow', ...
        'storm_start_s (%.1f) is negative, clamping to 0.', storm_start);
    storm_start = 0;
end

if storm_end <= storm_start
    warning('getWeather:invalidWindow', ...
        'storm_end_s (%.1f) <= storm_start_s (%.1f), disabling storm.', ...
        storm_end, storm_start);
    w = 0; profile = struct('storm_start',storm_start,'storm_end',storm_end,...
        'active_type',active_type,'w',0,'in_storm',false);
    return;
end

if storm_end > duration_s
    warning('getWeather:windowClamped', ...
        'storm_end_s (%.1f) exceeds duration (%.1f s), clamping.', ...
        storm_end, duration_s);
    storm_end = duration_s;
end

%% 3. Compute w based on active_type
switch lower(active_type)

    case 'step'
        if t >= storm_start && t <= storm_end
            w = 1.0;
        else
            w = 0.0;
        end

    case 'ramp'
        storm_duration = storm_end - storm_start;
        if t < storm_start || t > storm_end
            w = 0.0;
        elseif t <= (storm_start + storm_duration/2)
            w = (t - storm_start) / (storm_duration/2);
        else
            w = (storm_end - t) / (storm_duration/2);
        end

    case 'pulse'
        pulse_end = storm_start + 0.2 * (storm_end - storm_start);
        if t >= storm_start && t <= pulse_end
            w = 1.0;
        else
            w = 0.0;
        end

    otherwise
        warning('getWeather:unknownType', ...
            'Unknown active_type "%s". Defaulting to step.', active_type);
        w = double(t >= storm_start && t <= storm_end);
end

w = max(0, min(1, w));  % clamp

%% 4. Build profile struct
profile.storm_start = storm_start;
profile.storm_end   = storm_end;
profile.active_type = active_type;
profile.w           = w;
profile.in_storm    = (w > 0);

%% 5. Append to dataLog.WeatherSeverity
if ~isfield(dataLog, 'WeatherSeverity')
    dataLog.WeatherSeverity = [];
end
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
