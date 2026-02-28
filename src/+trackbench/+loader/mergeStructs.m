function merged = mergeStructs(base, override)
%mergeStructs Deep merge of structs. Override wins conflicts.
% Arrays/cells/scalars are replaced, not merged element-wise.

if ~isstruct(base)
    merged = override;
    return;
end

if ~isstruct(override)
    merged = override;
    return;
end

merged = base;
fields = fieldnames(override);
for i = 1:numel(fields)
    key = fields{i};
    val = override.(key);

    if isfield(base, key) && isstruct(base.(key)) && isstruct(val)
        merged.(key) = trackbench.loader.mergeStructs(base.(key), val);
    else
        merged.(key) = val;
    end
end
end
