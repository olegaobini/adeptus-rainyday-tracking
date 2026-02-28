function config = normalizeUnits(config)
%normalizeUnits Convert known unit-suffixed fields to canonical units.

conv = struct(...
    'km_to_m', 1000, ...
    'nmi_to_m', 1852, ...
    'ft_to_m', 0.3048, ...
    'kts_to_mps', 0.514444, ...
    'deg_to_rad', pi/180);

config = walk(config, conv);
end

function node = walk(node, conv)
if isstruct(node)
    if numel(node) > 1
        for n = 1:numel(node)
            node(n) = walk(node(n), conv);
        end
        return;
    end
    fn = fieldnames(node);
    for i = 1:numel(fn)
        key = fn{i};
        val = node.(key);

        if isstruct(val)
            node.(key) = walk(val, conv);
        elseif iscell(val)
            for j = 1:numel(val)
                val{j} = walk(val{j}, conv);
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
        node{i} = walk(node{i}, conv);
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
