function pathOut = findComponent(fieldName, componentName)
%findComponent Locate component JSON path from field + component name.

pathOut = "";
if ~(ischar(componentName) || isstring(componentName))
    return;
end

name = string(componentName);
if name == ""
    return;
end
if endsWith(name, ".json")
    name = erase(name, ".json");
end

folderMap = struct( ...
    'sensors', 'sensors', ...
    'weather', 'weather', ...
    'paths', 'paths', ...
    'trackers', 'trackers');

f = matlab.lang.makeValidName(char(fieldName));
if ~isfield(folderMap, f)
    return;
end

folder = string(folderMap.(f));
candidate = trackbench.util.pathFromRoot("config", "components", folder, name + ".json");
if isfile(candidate)
    pathOut = string(candidate);
end
end
