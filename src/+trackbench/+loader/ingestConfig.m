function config = ingestConfig(configName)
%ingestConfig Stage 1: read files, resolve refs, merge hierarchy.

arguments
    configName (1,1) string
end

[pathType, relPath] = classifyConfigName(configName);
configPath = trackbench.util.pathFromRoot("config", relPath);
if ~isfile(configPath)
    error('ingestConfig:fileNotFound', ...
        'Config file not found: %s\nRequested: %s', configPath, configName);
end

raw = jsondecode(fileread(configPath));

if pathType == "sweep"
    if ~isfield(raw, 'sweep') || ~isstruct(raw.sweep)
        error('ingestConfig:invalidSweep', 'Sweep file %s missing "sweep" object.', configPath);
    end
    if ~isfield(raw, 'base_scenario')
        error('ingestConfig:invalidSweep', 'Sweep file %s missing "base_scenario".', configPath);
    end

    baseConfig = trackbench.loader.ingestConfig(string(raw.base_scenario));
    baseConfig.sweep = raw.sweep;

    if isfield(raw, 'sweep_name')
        baseConfig.sweep_name = string(raw.sweep_name);
    else
        [~, nm] = fileparts(char(configPath));
        baseConfig.sweep_name = string(nm);
    end
    baseConfig.config_source = configName;
    baseConfig.timestamp = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    config = baseConfig;
    return;
end

if isfield(raw, 'template') && strlength(string(raw.template)) > 0
    templatePath = trackbench.util.pathFromRoot("config", "templates", string(raw.template) + ".json");
    if ~isfile(templatePath)
        error('ingestConfig:templateNotFound', ...
            'Template not found: %s (from template="%s")', templatePath, string(raw.template));
    end
    templateCfg = jsondecode(fileread(templatePath));

    rawCopy = raw;
    if isfield(rawCopy, 'template'); rawCopy = rmfield(rawCopy, 'template'); end
    if isfield(rawCopy, 'overrides'); rawCopy = rmfield(rawCopy, 'overrides'); end

    config = trackbench.loader.mergeStructs(templateCfg, rawCopy);
    if isfield(raw, 'overrides') && isstruct(raw.overrides)
        config = applyDotOverrides(config, raw.overrides);
    end
else
    config = raw;
    if isfield(config, 'overrides') && isstruct(config.overrides)
        overrideSrc = config.overrides;
        config = rmfield(config, 'overrides');
        config = applyDotOverrides(config, overrideSrc);
    end
end

config = trackbench.loader.resolveComponentRefs(config);

defaultPath = trackbench.util.pathFromRoot("config", "default.json");
defaults = jsondecode(fileread(defaultPath));
config = trackbench.loader.mergeStructs(defaults, config);

config.config_source = configName;
config.timestamp = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
end

function out = applyDotOverrides(base, overrides)
out = base;
keys = fieldnames(overrides);
for i = 1:numel(keys)
    rawKey = string(keys{i});
    resolvedKey = resolveOverridePath(out, rawKey);
    out = trackbench.loader.setNestedField(out, resolvedKey, overrides.(keys{i}));
end
end

function key = resolveOverridePath(targetConfig, rawKey)
key = rawKey;
if contains(key, ".")
    return;
end

if localParameterExists(targetConfig, key)
    return;
end

resolved = resolveMangledPath(targetConfig, key);
if resolved ~= ""
    key = resolved;
    return;
end

if contains(key, "_")
    dottedFallback = replace(key, "_", ".");
    if localParameterExists(targetConfig, dottedFallback)
        key = dottedFallback;
    else
        key = dottedFallback;
    end
end
end

function exists = localParameterExists(config, dotPath)
parts = strsplit(char(dotPath), '.');
current = config;

for i = 1:numel(parts)
    [field, idx, hasIdx] = parsePart(parts{i});
    if ~isstruct(current) || ~isfield(current, field)
        exists = false;
        return;
    end
    current = current.(field);
    if hasIdx
        if ~iscell(current) || idx < 1 || idx > numel(current)
            exists = false;
            return;
        end
        current = current{idx};
    end
end
exists = true;
end

function [field, idx, hasIdx] = parsePart(part)
expr = regexp(part, '^(\w+)\[(\d+)\]$', 'tokens', 'once');
if isempty(expr)
    field = part;
    idx = 0;
    hasIdx = false;
else
    field = expr{1};
    idx = str2double(expr{2}) + 1;
    hasIdx = true;
end
end

function resolved = resolveMangledPath(config, rawKey)
resolved = "";
paths = collectDotPaths(config, "");
for i = 1:numel(paths)
    p = string(paths{i});
    if replace(p, ".", "_") == rawKey
        resolved = p;
        return;
    end
end
end

function out = collectDotPaths(node, prefix)
out = {};
if ~isstruct(node)
    return;
end

if numel(node) > 1
    for k = 1:numel(node)
        out = [out, collectDotPaths(node(k), prefix)]; %#ok<AGROW>
    end
    return;
end

fields = fieldnames(node);
for i = 1:numel(fields)
    f = fields{i};
    if prefix == ""
        cur = string(f);
    else
        cur = prefix + "." + string(f);
    end
    out{end+1} = char(cur); %#ok<AGROW>
    v = node.(f);
    if isstruct(v)
        out = [out, collectDotPaths(v, cur)]; %#ok<AGROW>
    elseif iscell(v)
        for j = 1:numel(v)
            if isstruct(v{j})
                idxPrefix = sprintf('%s[%d]', cur, j-1);
                out = [out, collectDotPaths(v{j}, string(idxPrefix))]; %#ok<AGROW>
            end
        end
    end
end
end

function [kind, relPath] = classifyConfigName(configName)
name = string(configName);
name = erase(name, "\\");

if endsWith(name, ".json")
    baseName = name;
else
    baseName = name + ".json";
end

if startsWith(baseName, "sweeps/")
    kind = "sweep";
    relPath = baseName;
    return;
end
if startsWith(baseName, "scenarios/")
    kind = "scenario";
    relPath = baseName;
    return;
end
if startsWith(baseName, "templates/")
    kind = "template";
    relPath = baseName;
    return;
end
if baseName == "default.json"
    kind = "scenario";
    relPath = baseName;
    return;
end

kind = "scenario";
relPath = "scenarios/" + baseName;
end
