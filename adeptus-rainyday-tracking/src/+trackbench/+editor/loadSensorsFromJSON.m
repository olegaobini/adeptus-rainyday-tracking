function nLoaded = loadSensorsFromJSON(state, jsonPath, mode)
%loadSensorsFromJSON  Hydrate EditorState.sensors from a JSON file (M6 §3.4).
%
%  Auto-detects three file shapes:
%    1. Single-sensor file — has top-level "type" and "params" fields.
%         { "name": "...", "type": "PSR", "platform": "tower",
%           "frequency_hz": ..., "params": {...} }
%    2. Multi-sensor bundle — has top-level "sensors":[ {...}, {...} ]
%         where each element is a single-sensor struct as in (1).
%    3. Run file — has top-level "sensors":["TYPE/stem", ...]
%         where each entry is a string path relative to config/sensors/.
%         Each referenced file is then loaded (recursively) as (1).
%
%  Shapes (1) and (2) both ignore unsupported platforms by storing the
%  verbatim parsed struct into sr.originalDef and marking sr.readOnly.
%  Shape (3) recurses through this same function so run files compose
%  naturally.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    jsonPath : absolute path, or path relative to project root, to the
%               file being loaded. For run files the "sensors" entries
%               inside are resolved against config/sensors/.
%    mode     : "append"  — concat to state.sensors; activeSensorIdx is
%                            left at its current value unless no sensor
%                            was active, in which case it points at the
%                            first newly-loaded sensor.
%               "replace" — wipe state.sensors first, then append. Active
%                            sensor idx becomes 1.
%               Default: "append".
%
%  OUTPUT
%    nLoaded  : number of sensors appended.
%
%  ERROR CONVENTIONS
%    Throws trackbench:editor:loadSensorsFromJSON:* on missing file,
%    malformed schema, or if the file contains no sensors. UI callers
%    surface the message via uialert. Individual UNKNOWN passthroughs do
%    NOT throw — they load with sr.readOnly=true so the user can see
%    them, just not edit them.
%
%  See also: trackbench.editor.exportSensorsToJSON,
%            trackbench.editor.openScenarioFromJSON,
%            trackbench.editor.SensorRecord,
%            trackbench.editor.EditorState

    arguments
        state    (1,1) trackbench.editor.EditorState
        jsonPath (1,1) string
        mode     (1,1) string ...
            {mustBeMember(mode, ["append","replace"])} = "append"
    end

    resolved = resolveSensorPath(state, jsonPath);
    if ~isfile(resolved)
        error('trackbench:editor:loadSensorsFromJSON:notFound', ...
            'File not found: %s', resolved);
    end
    raw = fileread(resolved);
    try
        decoded = jsondecode(raw);
    catch ME
        error('trackbench:editor:loadSensorsFromJSON:badJSON', ...
            'Could not parse %s: %s', resolved, ME.message);
    end

    % ── Shape-detect ─────────────────────────────────────────────────
    %  Detect in this order:
    %    run-file form  → "sensors" is an array of STRINGS
    %    bundle form    → "sensors" is an array of STRUCTS (each a sensor)
    %    single-sensor  → top-level "type" + "params"
    records = trackbench.editor.SensorRecord.empty;

    if isstruct(decoded) && isfield(decoded, 'sensors')
        sensorsField = decoded.sensors;
        if isStringArray(sensorsField)
            % Run-file form: recurse for each entry.
            pathCells = toCellString(sensorsField);
            for k = 1:numel(pathCells)
                records = [records, ...
                    parseOneSensorFromPath(state, pathCells{k})]; %#ok<AGROW>
            end
        else
            % Bundle form: struct array or cell array of sensor structs.
            items = normalizeToCell(sensorsField);
            for k = 1:numel(items)
                records(end+1) = ...
                    parseOneSensorStruct(items{k}, resolved); %#ok<AGROW>
            end
        end
    elseif isstruct(decoded) && isfield(decoded, 'type') && isfield(decoded, 'params')
        % Single-sensor form.
        records(end+1) = parseOneSensorStruct(decoded, resolved);
    else
        error('trackbench:editor:loadSensorsFromJSON:badShape', ...
            ['Unrecognized sensor file shape at %s. ' ...
             'Expected either a single-sensor file (top-level type/params), ' ...
             'a bundle ({"sensors":[{...}]}), or a run file ' ...
             '({"sensors":["TYPE/stem"]}).'], resolved);
    end

    if isempty(records)
        error('trackbench:editor:loadSensorsFromJSON:noSensors', ...
            'No sensors found in %s.', resolved);
    end

    % ── Apply mode ───────────────────────────────────────────────────
    state.pushUndo();
    switch mode
        case "replace"
            state.sensors = records;
            state.activeSensorIdx = 1;
        case "append"
            nBefore = numel(state.sensors);
            for k = 1:numel(records)
                state.sensors(end+1) = records(k);
            end
            if nBefore == 0
                state.activeSensorIdx = 1;
            end
    end
    state.anyDirty = false;
    state.sensorsDirty = false;   % freshly loaded sensors == clean
    nLoaded = numel(records);
end


%% ========================================================================
%  Per-sensor parser
%% ========================================================================
function sr = parseOneSensorStruct(tDef, sourceFile)
%parseOneSensorStruct  Convert one decoded sensor-def struct into a
%                      SensorRecord. Pure transform — no state writes.
%
%  UNKNOWN / non-tower fallback:
%    - platform != "tower"                  → passthrough (readOnly=true)
%    - sensorType not in supported list     → passthrough (readOnly=true)
%    In both cases sr.originalDef is set verbatim so re-export round
%    trips the file. The record's supported fields are still populated
%    as best-effort (so the user at least sees a rough position marker).
    sr = trackbench.editor.SensorRecord();
    sr.sourceFile = string(sourceFile);

    if ~isstruct(tDef)
        error('trackbench:editor:loadSensorsFromJSON:badSensor', ...
            'Expected sensor definition struct; got %s.', class(tDef));
    end

    supportedTypes = ["PSR","SSR","ASR","ARSR","PAR", ...
                      "MARITIME","WEATHER","TWS"];
    typeStr = "PSR";
    if isfield(tDef, 'type') && ~isempty(tDef.type)
        typeStr = upper(string(tDef.type));
    end
    platStr = "tower";
    if isfield(tDef, 'platform') && ~isempty(tDef.platform)
        platStr = lower(string(tDef.platform));
    end
    isPassthrough = (platStr ~= "tower") || ~any(typeStr == supportedTypes);

    if isPassthrough
        % Mark readOnly and stash the original for round-trip.
        sr.readOnly    = true;
        sr.originalDef = tDef;
        sr.sensorType  = typeStr;
        sr.platform    = platStr;
        if isfield(tDef, 'name') && ~isempty(tDef.name)
            sr.sensorName = sanitizeName(string(tDef.name));
        else
            sr.sensorName = sprintf("passthrough_%s", typeStr);
        end
        % Best-effort position decode so the sensor at least shows up
        % on the map (useful for visual audit even when not editable).
        if isfield(tDef, 'params') && isfield(tDef.params, 'mountingLoc')
            mL = double(tDef.params.mountingLoc(:)');
            if numel(mL) >= 3
                sr.positionEastM  = mL(1);
                sr.positionNorthM = mL(2);
                sr.mountingLoc    = [0 0 mL(3)];
            end
        end
        return;
    end

    % ── Supported-type path ───────────────────────────────────────────
    sr.sensorType = typeStr;
    sr.platform   = platStr;
    if isfield(tDef, 'name') && ~isempty(tDef.name)
        sr.sensorName = sanitizeName(string(tDef.name));
    else
        sr.sensorName = typeStr + "_loaded";
    end
    if isfield(tDef, 'frequency_hz') && isnumeric(tDef.frequency_hz)
        sr.frequencyHz = double(tDef.frequency_hz);
    end

    % Params block — be forgiving: any missing field keeps the
    % SensorRecord property default, which was seeded by the class
    % definition and is already a sane starting point.
    if isfield(tDef, 'params') && isstruct(tDef.params)
        p = tDef.params;
        if isfield(p, 'rpm');         sr.rpm        = double(p.rpm); end
        if isfield(p, 'fov');         sr.fov        = asRow2(p.fov, sr.fov); end
        if isfield(p, 'tilt');        sr.tilt       = double(p.tilt); end
        if isfield(p, 'sector');      sr.sectorDeg  = asRow2(p.sector, sr.sectorDeg); end
        if isfield(p, 'pd');          sr.pd         = double(p.pd); end
        if isfield(p, 'far');         sr.far        = double(p.far); end
        if isfield(p, 'rangeLimits'); sr.rangeLimits = asRow2(p.rangeLimits, sr.rangeLimits); end
        if isfield(p, 'rangeRes');    sr.rangeResM  = double(p.rangeRes); end
        if isfield(p, 'mountingLoc')
            mL = double(p.mountingLoc(:)');
            if numel(mL) >= 3
                sr.positionEastM  = mL(1);
                sr.positionNorthM = mL(2);
                sr.mountingLoc    = [0 0 mL(3)];
            end
        end
        % Display color round-trip: accept both _display_color (on disk)
        % and x_display_color (the struct-field-safe spelling jsonencode
        % produces if we round-tripped through MATLAB directly).
        color = [];
        if isfield(p, 'x_display_color')
            color = double(p.x_display_color(:)');
        elseif isfield(p, '_display_color')
            color = double(p.('_display_color'));
            color = color(:)';
        end
        if ~isempty(color) && numel(color) == 3 && all(isfinite(color))
            sr.displayColor = max(min(color, 1), 0);
        end
    end

    sr.readOnly = false;
end


%% ========================================================================
%  Run-file recursion
%% ========================================================================
function rec = parseOneSensorFromPath(state, relPath)
%parseOneSensorFromPath  Given a "TYPE/stem" string from a run file's
%                         sensors array, resolve it under config/sensors/
%                         and parse that file as a single sensor.
%
%  Mirrors loadRunFile.m §2 path resolution: if relPath doesn't end in
%  .json, append it; look under <projectRoot>/config/sensors/.
    rel = char(relPath);
    if ~endsWith(lower(rel), ".json")
        rel = [rel '.json'];
    end
    root = state.projectRoot;
    if root == ""; root = string(pwd); end
    full = fullfile(root, "config", "sensors", rel);
    if ~isfile(full)
        error('trackbench:editor:loadSensorsFromJSON:runSensorNotFound', ...
            'Run-file references missing sensor config: %s', full);
    end
    rawInner = fileread(full);
    try
        inner = jsondecode(rawInner);
    catch ME
        error('trackbench:editor:loadSensorsFromJSON:runSensorBadJSON', ...
            'Could not parse %s: %s', full, ME.message);
    end
    rec = parseOneSensorStruct(inner, full);
end


%% ========================================================================
%  Helpers
%% ========================================================================
function full = resolveSensorPath(state, jsonPath)
%resolveSensorPath  Best-effort path resolution for the caller-provided
%                   path. If absolute, return as-is; if relative, try
%                   projectRoot/<rel> then pwd/<rel>.
    jp = char(jsonPath);
    if isAbsolute(jp)
        full = jp;
        return;
    end
    candidates = strings(0);
    if state.projectRoot ~= ""
        candidates(end+1) = fullfile(state.projectRoot, jp);
    end
    candidates(end+1) = string(fullfile(pwd, jp));
    candidates(end+1) = string(jp);
    for i = 1:numel(candidates)
        if isfile(candidates(i))
            full = char(candidates(i));
            return;
        end
    end
    full = char(candidates(1));
end


function yes = isAbsolute(p)
%isAbsolute  Cross-platform absolute path test. Mirrors loadFromJSON.m.
    if ispc
        yes = numel(p) >= 2 && (p(2) == ':' || startsWith(p, '\\'));
    else
        yes = ~isempty(p) && p(1) == '/';
    end
end


function tf = isStringArray(v)
%isStringArray  True when v is a flat collection of strings/chars (as
%               jsondecode yields for a JSON array of quoted paths).
%               Rejects struct arrays and numeric arrays.
    if ischar(v); tf = true; return; end
    if isstring(v); tf = true; return; end
    if iscell(v)
        tf = all(cellfun(@(x) ischar(x) || (isstring(x) && isscalar(x)), v));
        return;
    end
    tf = false;
end


function c = toCellString(v)
%toCellString  Normalize a string/char/string-array/cellstr into a cell
%              array of char vectors.
    if ischar(v)
        c = {v};
    elseif isstring(v)
        c = cellstr(v);
    elseif iscell(v)
        c = cellfun(@char, v, 'UniformOutput', false);
    else
        c = {};
    end
end


function items = normalizeToCell(s)
%normalizeToCell  jsondecode returns a struct array when every element
%                  has the same fields and a cell array otherwise. Flatten
%                  to a cell so the caller can iterate uniformly.
    if iscell(s)
        items = s;
    elseif isstruct(s)
        items = num2cell(s);
    else
        items = {};
    end
end


function row = asRow2(v, fallback)
%asRow2  Coerce a 2-element JSON array to a 1x2 row vector. jsondecode
%        yields column vectors for [a,b] arrays; buildSensor and the
%        editor panel expect rows.
    vv = double(v(:)');
    if numel(vv) < 2
        row = fallback;
    else
        row = vv(1:min(numel(vv), 2));
    end
end


function out = sanitizeName(name)
%sanitizeName  Conservative name cleanup — drop path separators and
%              whitespace. Keeps underscores and hyphens. Mirrors the
%              name handling in EditorState.uniquifySensorName so
%              loaded names don't surprise the dropdown's ItemsData.
    s = char(name);
    s = regexprep(s, '[\\/:*?"<>|]', '_');
    s = regexprep(s, '\s+', '_');
    if isempty(s); s = 'sensor'; end
    out = string(s);
end
