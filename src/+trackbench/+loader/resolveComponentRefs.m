function config = resolveComponentRefs(config)
%resolveComponentRefs Recursively resolve component references.

if isstruct(config) && numel(config) > 1
    for n = 1:numel(config)
        config(n) = trackbench.loader.resolveComponentRefs(config(n));
    end
    return;
end

if isstruct(config)
    fn = fieldnames(config);
    for i = 1:numel(fn)
        key = fn{i};
        val = config.(key);

        if isstruct(val)
            if isfield(val, 'ref')
                compPath = trackbench.loader.findComponent(key, string(val.ref));
                if compPath == ""
                    error('resolveComponentRefs:notFound', ...
                        'Component not found for field "%s", ref "%s".', key, string(val.ref));
                end
                comp = jsondecode(fileread(compPath));
                if isfield(val, 'overrides') && isstruct(val.overrides)
                    over = applyDotOverrides(struct(), val.overrides);
                    comp = trackbench.loader.mergeStructs(comp, over);
                end
                config.(key) = trackbench.loader.resolveComponentRefs(comp);
            else
                config.(key) = trackbench.loader.resolveComponentRefs(val);
            end
        elseif iscell(val)
            for j = 1:numel(val)
                v = val{j};
                if isstruct(v)
                    val{j} = trackbench.loader.resolveComponentRefs(v);
                elseif ischar(v) || isstring(v)
                    compPath = trackbench.loader.findComponent(key, string(v));
                    if compPath ~= ""
                        val{j} = jsondecode(fileread(compPath));
                    end
                end
            end
            config.(key) = val;
        elseif ischar(val) || isstring(val)
            compPath = trackbench.loader.findComponent(key, string(val));
            if compPath ~= ""
                config.(key) = jsondecode(fileread(compPath));
            end
        end
    end
end
end

function out = applyDotOverrides(base, overrides)
out = base;
keys = fieldnames(overrides);
for i = 1:numel(keys)
    out = trackbench.loader.setNestedField(out, string(keys{i}), overrides.(keys{i}));
end
end
