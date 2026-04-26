function outPath = exportWeatherToJSON(state, fullPath)
%exportWeatherToJSON  Write state.weather to a single weather config
%                     JSON file (v3.5 step 4c).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Pure function — takes a complete output path. The caller (buildUI's
%  onWeatherSave callback) shows the uiputfile dialog and resolves the
%  user's chosen path. Same separation pattern as exportTerrainToJSON.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance. state.weather
%               must be non-empty — there's no save-as-(none) path
%               (the (none) state IS no-file; clear it via the type
%               dropdown if that's what you want).
%    fullPath : absolute path of the .json file to write. Missing
%               ".json" extension is appended automatically.
%
%  OUTPUT
%    outPath  : the actual path written, with extension normalized.
%
%  SCHEMA (matches config/weather/<TYPE>/default_<TYPE>.json)
%    {
%      "type":               "rain" | "snow" | "fog" | "icing",
%      "description":        "<text>",
%      "rain_rate_mmhr":     <double>,
%      "storm_start_s":      <double>,
%      "storm_end_s":        <double>,
%      "active_type":        "step" | "ramp" | "pulse",
%      "pd_floor":           <double>,
%      "clutter_multiplier": <double>     (rain/snow only — see below)
%    }
%
%  CLUTTER_MULTIPLIER OMISSION
%    The library files for fog and icing omit clutter_multiplier
%    entirely (those weather types don't generate volume clutter).
%    This writer honors that via WeatherRecord.emitsClutterField()
%    so a load → save round-trip preserves the on-disk file shape.
%
%  UNKNOWN PASSTHROUGH
%    When state.weather.readOnly is true (UNKNOWN passthrough loaded
%    from a file with a type the editor doesn't natively support),
%    the verbatim originalDef struct is emitted instead. Same
%    rationale as exportTerrainToJSON.
%
%  See also: trackbench.editor.WeatherRecord,
%            trackbench.editor.loadWeatherFromJSON,
%            trackbench.editor.exportSensorsToJSON

    arguments
        state    (1,1) trackbench.editor.EditorState
        fullPath (1,1) string
    end

    if isempty(state.weather)
        error('trackbench:editor:exportWeatherToJSON:noWeather', ...
            ['No weather configured to save. Pick a weather type ' ...
             '(rain/snow/fog/icing) from the Weather panel first, ' ...
             'or use Save Scenario which writes "weather: none" when ' ...
             'the scenario has no weather.']);
    end
    if strlength(fullPath) == 0
        error('trackbench:editor:exportWeatherToJSON:emptyPath', ...
            'Output path must be non-empty.');
    end

    fullPath = char(fullPath);
    if ~endsWith(lower(fullPath), '.json')
        fullPath = [fullPath '.json'];
    end

    wr = state.weather;
    if wr.readOnly && ~isempty(fieldnames(wr.originalDef))
        def = wr.originalDef;
    else
        def = buildWeatherStruct(wr);
    end

    jsonStr = jsonencode(def, 'PrettyPrint', true);

    parent = fileparts(fullPath);
    if ~isempty(parent) && ~exist(parent, 'dir')
        mkdir(parent);
    end

    fid = fopen(fullPath, 'w');
    if fid < 0
        error('trackbench:editor:exportWeatherToJSON:openFailed', ...
            'Could not open %s for writing.', fullPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    % Mirror exportTerrainToJSON's bookkeeping: clear the env-dirty
    % flag and stamp sourceFile so the next reload "Load Weather…"
    % starts at this file's directory.
    state.environmentDirty = false;
    wr.sourceFile = string(fullPath);
    state.weather = wr;

    outPath = string(fullPath);
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function def = buildWeatherStruct(wr)
%buildWeatherStruct  WeatherRecord → on-disk schema struct.
%
%  Mirrors exportSensorsToJSON's same-named local helper (kept private
%  there). clutter_multiplier is OMITTED for fog/icing per
%  WeatherRecord.emitsClutterField().
    def = struct();
    def.type           = char(wr.weatherType);
    def.description    = char(wr.description);
    def.rain_rate_mmhr = wr.rainRateMmhr;
    def.storm_start_s  = wr.stormStartS;
    def.storm_end_s    = wr.stormEndS;
    def.active_type    = char(wr.activeType);
    def.pd_floor       = wr.pdFloor;
    if wr.emitsClutterField()
        def.clutter_multiplier = wr.clutterMultiplier;
    end
end
