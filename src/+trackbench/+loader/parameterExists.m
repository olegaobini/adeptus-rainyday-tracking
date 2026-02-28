function exists = parameterExists(config, dotPath)
%parameterExists True when dot-path points to an existing value.

arguments
    config (1,1) struct
    dotPath (1,1) string
end

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
