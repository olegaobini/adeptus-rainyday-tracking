function config = resolveRelativePaths(config)
%resolveRelativePaths Convert relative file paths to absolute repo-root paths.

root = trackbench.util.rootDir();
if isempty(root)
    root = pwd;
end

config = walk(config, string(root));
end

function node = walk(node, root)
if isstruct(node)
    if numel(node) > 1
        for n = 1:numel(node)
            node(n) = walk(node(n), root);
        end
        return;
    end
    fn = fieldnames(node);
    for i = 1:numel(fn)
        key = fn{i};
        val = node.(key);
        if isstruct(val)
            node.(key) = walk(val, root);
        elseif iscell(val)
            for j = 1:numel(val)
                val{j} = walk(val{j}, root);
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
        node{i} = walk(node{i}, root);
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
