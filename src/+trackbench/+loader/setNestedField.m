function out = setNestedField(in, dotPath, value)
%setNestedField Set nested struct field using dot-path and optional [idx].

arguments
    in (1,1) struct
    dotPath (1,1) string
    value
end

parts = strsplit(char(dotPath), '.');
out = setPart(in, parts, value);
end

function s = setPart(s, parts, value)
part = parts{1};
[field, idx, hasIdx] = parsePart(part);

if hasIdx
    if ~isfield(s, field)
        s.(field) = {};
    end
    arr = s.(field);
    if ~iscell(arr)
        error('setNestedField:typeMismatch', 'Field "%s" must be a cell for indexed assignment.', field);
    end
    while numel(arr) < idx
        arr{end+1} = struct(); %#ok<AGROW>
    end

    if numel(parts) == 1
        arr{idx} = value;
    else
        child = arr{idx};
        if ~isstruct(child)
            child = struct();
        end
        arr{idx} = setPart(child, parts(2:end), value);
    end
    s.(field) = arr;
    return;
end

if numel(parts) == 1
    s.(field) = value;
    return;
end

if ~isfield(s, field) || ~isstruct(s.(field))
    s.(field) = struct();
end
s.(field) = setPart(s.(field), parts(2:end), value);
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
