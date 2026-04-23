classdef WeatherRecord
%WeatherRecord  Per-scenario weather state for the editor (M7 §3.1).
%
%  A plain VALUE class. Weather is OPTIONAL — EditorState.weather is
%  [] when no weather is active, or a single WeatherRecord when one
%  is. This is the "(none)" vs configured distinction the Weather
%  sub-panel surfaces with its dropdown's first entry.
%
%  Do NOT represent "no weather" as a WeatherRecord with a sentinel
%  weatherType. The empty-[] sentinel is the contract every reader
%  must isempty()-guard on; mixing a sentinel record with the
%  dropdown's (none) option gets tangled fast.
%
%  Mutators read-mutate-writeback:
%      wr = state.weather;
%      wr.rainRateMmhr = 40;
%      state.weather = wr;
%
%  WHY VALUE AND NOT HANDLE
%    Same reasoning as SensorRecord and TerrainRecord. Undo/redo
%    snapshots need an independent copy.
%
%  SUPPORTED TYPES (M7 editing)
%    rain, snow, fog, icing. Anything else loads as UNKNOWN
%    passthrough (readOnly=true; originalDef holds verbatim JSON).
%
%  ON-DISK FIELD NAMES (match config/weather/<TYPE>/<file>.json)
%    description       <-> description
%    weatherType       <-> type
%    rainRateMmhr      <-> rain_rate_mmhr
%    stormStartS       <-> storm_start_s
%    stormEndS         <-> storm_end_s
%    activeType        <-> active_type        ("step"|"ramp"|"pulse")
%    pdFloor           <-> pd_floor
%    clutterMultiplier <-> clutter_multiplier (OMITTED on export for
%                                              fog/icing — matches
%                                              default_fog/icing.json)
%
%  RAIN-RATE FIELD NAME NOTE
%    applyWeatherDegradation reinterprets rain_rate_mmhr per type:
%    rain = actual mm/hr, snow = equivalent mm/hr (scaled 0.25
%    internally per Gunn & East), fog = density proxy (5/15/30 =
%    light/moderate/dense), icing = severity (5/15/30 = light/
%    moderate/severe dB loss). The field NAME stays constant on
%    disk so the sim's dispatcher works; only the UI label and
%    Limits change per type (see refreshWeatherPanel in buildUI).
%
%  See also: trackbench.editor.EditorState,
%            trackbench.editor.weatherDefaults,
%            trackbench.editor.loadWeatherFromJSON

    properties
        % ── On-disk fields ────────────────────────────────────────
        description       (1,1) string = "Moderate rain — 16 mm/hr, step profile"
        weatherType       (1,1) string = "rain"   % rain|snow|fog|icing
        rainRateMmhr      (1,1) double = 16
        stormStartS       (1,1) double = 50
        stormEndS         (1,1) double = 130
        activeType        (1,1) string = "step"   % step|ramp|pulse
        pdFloor           (1,1) double = 0.15
        clutterMultiplier (1,1) double = 1.0

        % ── Editor-local / passthrough ─────────────────────────────
        readOnly     (1,1) logical = false
        sourceFile   (1,1) string  = ""
        originalDef  (1,1) struct  = struct()
    end

    methods
        function tf = isSupportedType(obj)
            %isSupportedType  True iff weatherType is one of the four
            %                 editor-editable types. False on UNKNOWN
            %                 passthrough — UI disables fields below
            %                 the type dropdown.
            supported = ["rain","snow","fog","icing"];
            tf = any(strcmpi(obj.weatherType, supported));
        end

        function tf = emitsClutterField(obj)
            %emitsClutterField  True iff clutter_multiplier should be
            %                    written on export. Rain and snow
            %                    generate weather clutter; fog and
            %                    icing do not (default_fog/icing.json
            %                    omit the key entirely). The in-editor
            %                    field stays at 1.0 by default but is
            %                    dropped on export for the dry types
            %                    so we match the on-disk library.
            t = lower(obj.weatherType);
            tf = t == "rain" || t == "snow";
        end
    end
end
