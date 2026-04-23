function wr = loadWeatherFromJSON(projectRoot, relPath)
%loadWeatherFromJSON  Parse a weather JSON file into a WeatherRecord
%                     (M7 §3.4). Mirrors loadTerrainFromJSON.
%
%  Pure function — no state writes. EditorState.loadWeatherFromFile
%  calls this and assigns the returned record into state.weather.
%  Always returns a 1x1 WeatherRecord (never empty — callers wanting
%  to clear weather use setWeatherType("none") which writes the empty
%  sentinel).
%
%  SUPPORTED / UNKNOWN
%    Built-in types resolve field-for-field from the on-disk schema:
%      description       ← description
%      weatherType       ← type             (lowercased)
%      rainRateMmhr      ← rain_rate_mmhr
%      stormStartS       ← storm_start_s
%      stormEndS         ← storm_end_s
%      activeType        ← active_type
%      pdFloor           ← pd_floor
%      clutterMultiplier ← clutter_multiplier   (optional — omitted for
%                                                fog/icing on disk)
%    Any type NOT in {rain, snow, fog, icing} loads as UNKNOWN
%    passthrough: readOnly=true, originalDef holds verbatim def.
%
%  CLUTTER_MULTIPLIER KEY
%    Per handoff §3.4, fog and icing library files OMIT this key. We
%    store clutterMultiplier=1.0 as a fallback so the UI field always
%    has a value, and WeatherRecord.emitsClutterField() lets
%    saveScenarioToJSON drop the key on export for fog/icing.
%
%  See also: trackbench.editor.WeatherRecord,
%            trackbench.editor.weatherDefaults,
%            trackbench.editor.EditorState.loadWeatherFromFile,
%            trackbench.editor.loadTerrainFromJSON

    arguments
        projectRoot (1,1) string
        relPath     (1,1) string
    end

    full = resolveWeatherPath(projectRoot, relPath);
    if ~isfile(full)
        error('trackbench:editor:loadWeatherFromJSON:notFound', ...
            'Weather file not found: %s', full);
    end
    raw = fileread(full);
    try
        def = jsondecode(raw);
    catch ME
        error('trackbench:editor:loadWeatherFromJSON:badJSON', ...
            'Could not parse %s: %s', full, ME.message);
    end
    if ~isstruct(def)
        error('trackbench:editor:loadWeatherFromJSON:badShape', ...
            'Weather file root must be a JSON object: %s', full);
    end

    wr = trackbench.editor.WeatherRecord();
    wr.sourceFile  = string(full);
    wr.originalDef = def;

    rawType = "";
    if isfield(def, 'type') && ~isempty(def.type)
        rawType = lower(string(def.type));
    end
    supported = ["rain", "snow", "fog", "icing"];
    isSupported = any(rawType == supported);

    if isSupported
        wr.weatherType = rawType;
        wr.readOnly    = false;
        wr.description       = pickString(def, 'description', wr.description);
        wr.rainRateMmhr      = pickNumber(def, 'rain_rate_mmhr',     wr.rainRateMmhr);
        wr.stormStartS       = pickNumber(def, 'storm_start_s',      wr.stormStartS);
        wr.stormEndS         = pickNumber(def, 'storm_end_s',        wr.stormEndS);
        wr.activeType        = pickString(def, 'active_type',        wr.activeType);
        wr.pdFloor           = pickNumber(def, 'pd_floor',           wr.pdFloor);
        % clutter_multiplier is optional on disk for fog/icing — fall
        % back to 1.0 so the UI field has a well-defined default.
        wr.clutterMultiplier = pickNumber(def, 'clutter_multiplier', 1.0);
    else
        % UNKNOWN passthrough.
        if rawType == ""
            wr.weatherType = "unknown";
        else
            wr.weatherType = rawType;
        end
        wr.readOnly = true;
        wr.description       = pickString(def, 'description', "UNKNOWN weather — see source file");
        wr.rainRateMmhr      = pickNumber(def, 'rain_rate_mmhr',     wr.rainRateMmhr);
        wr.stormStartS       = pickNumber(def, 'storm_start_s',      wr.stormStartS);
        wr.stormEndS         = pickNumber(def, 'storm_end_s',        wr.stormEndS);
        wr.activeType        = pickString(def, 'active_type',        wr.activeType);
        wr.pdFloor           = pickNumber(def, 'pd_floor',           wr.pdFloor);
        wr.clutterMultiplier = pickNumber(def, 'clutter_multiplier', 1.0);
    end
end


function full = resolveWeatherPath(projectRoot, relPath)
%resolveWeatherPath  Order: absolute → projectRoot/<rel> →
%                     projectRoot/config/weather/<rel>.
    p = char(relPath);
    if isAbsolute(p)
        full = p;
        return;
    end
    root = projectRoot;
    if root == ""; root = string(pwd); end
    if ~endsWith(lower(p), ".json")
        pExt = [p '.json'];
    else
        pExt = p;
    end
    candidates = strings(0);
    candidates(end+1) = fullfile(root, pExt);
    candidates(end+1) = fullfile(root, "config", "weather", pExt);
    candidates(end+1) = string(pExt);
    for i = 1:numel(candidates)
        if isfile(candidates(i))
            full = char(candidates(i));
            return;
        end
    end
    full = char(candidates(1));
end


function yes = isAbsolute(p)
    if ispc
        yes = numel(p) >= 2 && (p(2) == ':' || startsWith(p, '\\'));
    else
        yes = ~isempty(p) && p(1) == '/';
    end
end


function v = pickString(def, field, fallback)
    if isfield(def, field) && ~isempty(def.(field))
        v = string(def.(field));
    else
        v = fallback;
    end
end


function v = pickNumber(def, field, fallback)
    if isfield(def, field) && isnumeric(def.(field)) && ~isempty(def.(field))
        v = double(def.(field));
    else
        v = fallback;
    end
end
