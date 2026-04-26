function wr = weatherDefaults(typeStr)
%weatherDefaults  Fresh WeatherRecord seeded with the on-disk
%                 config/weather/<TYPE>/default_<TYPE>.json values
%                 for the given type.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Mirrors the sensorDefaults / terrainDefaults pattern. Single source
%  of truth for the four editor-supported weather-type seed values.
%  Used by:
%    * EditorState.setWeatherType (cascade reset on UI type change)
%    * openScenarioFromJSON (fallback when run file says a weather
%      type but no library entry resolves)
%
%  NOTE ON "NO WEATHER"
%    The caller is responsible for the (none) case. This function is
%    never called for "none" — EditorState.weather = [] is the empty
%    sentinel. If the dropdown is switched from (none) to e.g. "rain",
%    the caller invokes weatherDefaults("rain") to populate a fresh
%    record. See EditorState.setWeatherType.
%
%  VERIFIED AGAINST DISK 2026-04-22 (handoff §7 step 4):
%    The numeric values below match the four default_<TYPE>.json files
%    under config/weather/ field-for-field. Where the handoff table
%    and disk differed, this function aligns to disk:
%      * snow rain_rate_mmhr is 15 (disk) not 12 (handoff table).
%      * fog  storm window is 0/180 (disk) not 0/600 (handoff table).
%      * icing storm window is 0/180 (disk) not 0/600 (handoff table).
%      * All four pd_floor values are 0.15 on disk, not the varied
%        0.15/0.20/0.30/0.25 shown in the handoff draft.
%    Descriptions are taken verbatim from disk.
%
%  CLUTTER MULTIPLIER
%    rain and snow ship clutter_multiplier=1.0 on disk. fog and icing
%    OMIT the key entirely. We still store clutterMultiplier=1.0 on
%    every record so the editor field always has a well-defined
%    default, but WeatherRecord.emitsClutterField() returns false for
%    fog/icing and saveScenarioToJSON uses that to decide whether to
%    write the key. This keeps round-trips symmetrical with the disk
%    library.
%
%  SUPPORTED TYPES
%    rain, snow, fog, icing. Other inputs fall through to rain-shaped
%    defaults — the Type dropdown restricts input, so the fall-through
%    is belt-and-braces.
%
%  See also: trackbench.editor.WeatherRecord,
%            trackbench.editor.EditorState,
%            trackbench.editor.loadWeatherFromJSON

    wr = trackbench.editor.WeatherRecord();
    wr.weatherType       = lower(string(typeStr));
    wr.activeType        = "step";
    wr.pdFloor           = 0.15;
    wr.clutterMultiplier = 1.0;
    wr.readOnly          = false;
    wr.sourceFile        = "";
    wr.originalDef       = struct();

    switch lower(string(typeStr))
        case "rain"
            wr.description  = "Moderate rain — 16 mm/hr, step profile";
            wr.rainRateMmhr = 16;
            wr.stormStartS  = 50;
            wr.stormEndS    = 130;

        case "snow"
            wr.description  = "Moderate snowfall — 15 mm/hr equivalent precipitation, step profile";
            wr.rainRateMmhr = 15;
            wr.stormStartS  = 50;
            wr.stormEndS    = 130;

        case "fog"
            wr.description  = "Moderate fog — visibility ~300m. Primarily degrades IR/optical sensors. Negligible RF effect below 10 GHz.";
            wr.rainRateMmhr = 15;
            wr.stormStartS  = 0;
            wr.stormEndS    = 180;

        case "icing"
            wr.description  = "Moderate antenna icing — ~4 dB gain loss. Range-independent hardware degradation.";
            wr.rainRateMmhr = 15;
            wr.stormStartS  = 0;
            wr.stormEndS    = 180;

        otherwise
            % Unknown type → rain-shaped guard values. The Type
            % dropdown restricts user input to the supported four,
            % so this branch only fires if a caller invokes with a
            % typo-y string.
            wr.weatherType  = "rain";
            wr.description  = "Moderate rain — 16 mm/hr, step profile";
            wr.rainRateMmhr = 16;
            wr.stormStartS  = 50;
            wr.stormEndS    = 130;
    end
end
