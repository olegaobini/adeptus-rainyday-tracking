function config = loadConfig(configName, varargin)
% LOAD_CONFIG Load and merge configuration files with override support
%
% Usage:
%   config = load_config("default")                    % Load base config
%   config = load_config("scenarios/heavy_rain")       % Load scenario with overrides
%   config = load_config("default", "enableDegradation", true)  % Runtime override
%
% INPUTS
%   configName : string, name of config file (without .json) or full path
%   varargin   : optional Name-Value pairs for runtime overrides
%
% OUTPUT
%   cfg : struct with validated and merged configuration
%
% FILE STRUCTURE
%   config/
%   ├── default.json          % All defaults
%   ├── scenarios/
%   │   ├── heavy_rain.json       % Weather scenario overrides
%   │   └── urban_canyon.json     % Multi-target scenario overrides
%   └── custom/
%       └── boeing_test_*.json    % User custom configs

    arguments
        configName (1,1) string
    end
    arguments (Repeating)
        varargin
    end
    
    %%  Load Base Configuration
    basePath = fullfile(pwd, "config", "default.json");
    if ~isfile(basePath)
        error("Default config not found: %s", basePath);
    end
    
    fprintf("[CONFIG] Loading base: %s\n", basePath);
    config = jsondecode(fileread(basePath)); 

    %% Load Scenario Config if not "default"
    if ~ismember(configName, ["default", "base_config"])
        % Build Scenario Path
        if ~endsWith(configName, ".json")
            configName = configName + ".json";
        end

        % Try in scenarios/ subdirectory first
        scenarioPath = fullfile(pwd, "config", "scenarios", configName);
        if ~isfile(scenarioPath)
            % Try direct path
            scenarioPath = fullfile(pwd, "config", configName);
        end

        if isfile(scenarioPath)
            fprintf("[CONFIG] Loading scenario: %s\n", scenarioPath);
            scenario = jsondecode(fileread(scenarioPath));

            % Apply overrides if present
            if isfield(scenario, "overrides")
                config = apply_overrides(config, scenario.overrides);
                fprintf("[CONFIG] Applied %d override(s)\n", numel(fieldnames(scenario.overrides)));
            else
                config = merge_structs(config, scenario);
            end
        else
            warning("Scenario file not found: %s. Using base config only.", scenarioPath);
        end
    end

    %% Apply runtime overrides from varargin
    if ~isempty(varargin)
        p = inputParser;
        p.KeepUnmatched = true;
        parse(p, varargin{:});

        overrides = p.Unmatched;
        if ~isempty(fieldnames(overrides))
            config = merge_structs(config, overrides);
            fprintf("[CONFIG] Applied %d runtime override(s)\n", numel(fieldnames(overrides)));
        end
    end

    %% Validate Configuration
    config = validate_config(config);

    %% Select Degradation Specific Parameters
    % This is the key addition - automatically select the right tracker params
    % based on degradation.enabled flag

    if config.degradation.enabled
        activeParams = config.tracker_params.degraded;
        activePd = config.tracker_global.detection_probability.degraded;
        fprintf("[CONFIG] Using DEGRADED tracker parameters\n");
    else
        activeParams = config.tracker_params.ideal;
        activePd = config.tracker_global.detection_probability.ideal;
        fprintf("[CONFIG] Using IDEAL tracker parameters\n");
    end

    % Flatten active params into cfg for easy access in main script
    config.active_params = activeParams;
    config.active_params.pd = activePd;

    fprintf("[CONFIG] Configuration Loaded Successfully.\n")
end

%% ========================================================================
%                           HELPER FUNCTIONS
%% ========================================================================

function config = apply_overrides(config, overrides)
    % Apply dot-notation overrides to nested struct
    % Example: overrides.("scenario.num_targets") = 12
    %          sets config.scenario.num_targets = 12

    fields = fieldnames(overrides);

    for i = 1:length(fields)
        path = fields{i};
        value = overrides.(fields{i});
        config = set_nested_field(config, path, value);
    end
end

function s = set_nested_field(s, path, value)
    % Set a nested field using dot notation
    % path: "scenario.num_targets"
    % value: 12
    
    parts = split(path, '.');
    
    % Navigate to parent
    current = s;
    for i = 1:(length(parts)-1)
        field = parts{i};
        if ~isfield(current, field)
            current.(field) = struct();
        end
        current = current.(field);
    end
    
    % Set the final field
    finalField = parts{end};
    current.(finalField) = value;
    
    % Reconstruct the struct (MATLAB quirk)
    % This is simplified - for production we'll need recursive assignment
    if isscalar(parts)
        s.(parts{1}) = value;
    elseif length(parts) == 2
        s.(parts{1}).(parts{2}) = value;
    elseif length(parts) == 3
        s.(parts{1}).(parts{2}).(parts{3}) = value;
    else
        error("Nested path too deep (max 3 levels). Got: %s", path);
    end
end

function s1 = merge_structs(s1, s2)
    % Merge s2 into s1 (s2 takes precedence)
    % Recursively merges nested structs
    
    fields = fieldnames(s2);
    for i = 1:length(fields)
        field = fields{i};
        
        if isfield(s1, field) && isstruct(s1.(field)) && isstruct(s2.(field))
            % Both are structs - merge recursively
            s1.(field) = merge_structs(s1.(field), s2.(field));
        else
            % Overwrite with s2's value
            s1.(field) = s2.(field);
        end
    end
end

function config = validate_config(config)
    % Validate required structure and apply type conversions

    %% Scenario validation
    if ~isfield(config, "scenario")
        error("Config missing required top-level key: 'scenario'");
    end
    
    s = config.scenario;
    required = ["mode", "num_targets", "duration_s"];
    for k = required
        if ~isfield(s, k)
            error("Config missing required key: scenario.%s", k);
        end
    end
    
    % Type normalization
    config.scenario.mode = string(config.scenario.mode);
    config.scenario.num_targets = double(config.scenario.num_targets);
    config.scenario.duration_s = double(config.scenario.duration_s);
    
    % Value checks
    if ~(config.scenario.mode == "3D" || config.scenario.mode == "2D")
        error("scenario.mode must be '3D' or '2D'. Got: %s", config.scenario.mode);
    end
    if config.scenario.num_targets < 1 || mod(config.scenario.num_targets, 1) ~= 0
        error("scenario.num_targets must be a positive integer. Got: %g", config.scenario.num_targets);
    end
    if config.scenario.duration_s <= 0
        error("scenario.duration_s must be > 0. Got: %g", config.scenario.duration_s);
    end
    
    %% Degradation validation
    if isfield(config, "degradation")
        if ~isfield(config.degradation, "enabled")
            config.degradation.enabled = false;
        end
        config.degradation.enabled = logical(config.degradation.enabled);
    else
        config.degradation.enabled = false;
    end
    
    %% Tracker params validation
    if ~isfield(config, "tracker_params")
        error("Config missing required key: tracker_params");
    end
    if ~isfield(config.tracker_params, "ideal") || ~isfield(config.tracker_params, "degraded")
        error("tracker_params must contain 'ideal' and 'degraded' sub-configs");
    end
    
    fprintf("[CONFIG] Validation passed.\n");
end
