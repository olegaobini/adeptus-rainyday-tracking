function config = normalizeConfig(config)
%normalizeConfig Stage 3: canonical normalization.

if ~isfield(config.scenario, 'frame')
    config.scenario.frame = 'NED';
end

config = normalizeUnits(config);
config = resolveRelativePaths(config);

if isfield(config.scenario, 'duration_min')
    config.scenario.duration_s = double(config.scenario.duration_min) * 60;
end

% Runtime compatibility projection for current simulation stack.
if isfield(config, 'trackers')
    config.tracker_global = config.trackers.global;
    config.tracker_params = config.trackers.params;
    config.trackers_to_run = config.trackers.run;
end

if ~isfield(config, 'degradation')
    config.degradation = struct('enabled', false, 'type', 'none');
end
if ~isfield(config.degradation, 'enabled')
    config.degradation.enabled = false;
end

if isfield(config, 'tracker_params') && isfield(config, 'tracker_global') && ...
        isfield(config.tracker_global, 'detection_probability')
    if config.degradation.enabled
        config.active_params = config.tracker_params.degraded;
        config.active_params.pd = config.tracker_global.detection_probability.degraded;
    else
        config.active_params = config.tracker_params.ideal;
        config.active_params.pd = config.tracker_global.detection_probability.ideal;
    end
end
end

function config = normalizeUnits(config)
conv = struct(...
    'km_to_m', 1000, ...
    'nmi_to_m', 1852, ...
    'ft_to_m', 0.3048, ...
    'kts_to_mps', 0.514444, ...
    'deg_to_rad', pi/180);

config = walkUnit(config, conv);
end

function node = walkUnit(node, conv)
if isstruct(node)
    if numel(node) > 1
        for n = 1:numel(node)
            node(n) = walkUnit(node(n), conv);
        end
        return;
    end
    fn = fieldnames(node);
    for i = 1:numel(fn)
        key = fn{i};
        val = node.(key);

        if isstruct(val)
            node.(key) = walkUnit(val, conv);
        elseif iscell(val)
            for j = 1:numel(val)
                val{j} = walkUnit(val{j}, conv);
            end
            node.(key) = val;
        else
            [newKey, newVal, converted] = convertField(key, val, conv);
            if converted
                node = rmfield(node, key);
                node.(newKey) = newVal;
            end
        end
    end
elseif iscell(node)
    for i = 1:numel(node)
        node{i} = walkUnit(node{i}, conv);
    end
end
end

function [newKey, newVal, converted] = convertField(key, val, conv)
newKey = key;
newVal = val;
converted = false;

if ~(isnumeric(val) && isscalar(val))
    return;
end

if endsWith(key, '_km')
    newKey = extractBefore(key, strlength(key)-2) + "_m";
    newVal = val * conv.km_to_m;
    converted = true;
elseif endsWith(key, '_nmi')
    newKey = extractBefore(key, strlength(key)-3) + "_m";
    newVal = val * conv.nmi_to_m;
    converted = true;
elseif endsWith(key, '_ft')
    newKey = extractBefore(key, strlength(key)-2) + "_m";
    newVal = val * conv.ft_to_m;
    converted = true;
elseif endsWith(key, '_kts')
    newKey = extractBefore(key, strlength(key)-3) + "_mps";
    newVal = val * conv.kts_to_mps;
    converted = true;
elseif endsWith(key, '_deg') && ~strcmp(key, 'heading_deg')
    newKey = extractBefore(key, strlength(key)-3) + "_rad";
    newVal = val * conv.deg_to_rad;
    converted = true;
end
end

function config = resolveRelativePaths(config)
root = trackbench.util.rootDir();
if isempty(root)
    root = pwd;
end

config = walkPath(config, string(root));
end

function node = walkPath(node, root)
if isstruct(node)
    if numel(node) > 1
        for n = 1:numel(node)
            node(n) = walkPath(node(n), root);
        end
        return;
    end
    fn = fieldnames(node);
    for i = 1:numel(fn)
        key = fn{i};
        val = node.(key);
        if isstruct(val)
            node.(key) = walkPath(val, root);
        elseif iscell(val)
            for j = 1:numel(val)
                val{j} = walkPath(val{j}, root);
            end
            node.(key) = val;
        elseif (ischar(val) || isstring(val)) && isPathField(key)
            p = string(val);
            if ~isAbsolutePath(p)
                node.(key) = char(fullfile(root, p));
            end
        end
    end
elseif iscell(node)
    for i = 1:numel(node)
        node{i} = walkPath(node{i}, root);
    end
end
end

function tf = isPathField(key)
keys = {'path', 'file', 'csv', 'datalog_file', 'results_directory'};
tf = any(strcmpi(key, keys)) || endsWith(lower(key), '_path') || endsWith(lower(key), '_file');
end

function tf = isAbsolutePath(p)
p = char(p);
if ispc
    tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once'));
else
    tf = startsWith(p, '/');
end
end
