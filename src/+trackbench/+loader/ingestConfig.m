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

    overrides = struct();
    if isfield(raw, 'overrides') && isstruct(raw.overrides)
        overrides = applyDotOverrides(struct(), raw.overrides);
    end

    rawCopy = raw;
    if isfield(rawCopy, 'template'); rawCopy = rmfield(rawCopy, 'template'); end
    if isfield(rawCopy, 'overrides'); rawCopy = rmfield(rawCopy, 'overrides'); end

    config = trackbench.loader.mergeStructs(templateCfg, rawCopy);
    config = trackbench.loader.mergeStructs(config, overrides);
else
    config = raw;
    if isfield(config, 'overrides') && isstruct(config.overrides)
        o = applyDotOverrides(struct(), config.overrides);
        config = rmfield(config, 'overrides');
        config = trackbench.loader.mergeStructs(config, o);
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
    out = trackbench.loader.setNestedField(out, string(keys{i}), overrides.(keys{i}));
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
