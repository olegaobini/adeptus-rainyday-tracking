function tr = terrainDefaults(typeStr)
%terrainDefaults  Fresh TerrainRecord seeded with the on-disk
%                 config/terrain/<TYPE>/default_<TYPE>.json values
%                 for the given type.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Mirrors the sensorDefaults pattern. Single source of truth for
%  the five editor-supported terrain-type seed values. Used by:
%    * EditorState.setTerrainType (cascade reset on UI type change)
%    * openScenarioFromJSON (fallback when run file has no terrain)
%
%  VERIFIED AGAINST DISK 2026-04-22 (handoff §7 step 4):
%    The numeric values in the table below match the five
%    default_<TYPE>.json files under config/terrain/ field-for-field.
%    Descriptions are taken verbatim from disk — the handoff table
%    drafts differed on rural and urban so those two are aligned to
%    the on-disk files per the "align to disk, not to the tables"
%    rule.
%
%  SUPPORTED TYPES
%    none, water, rural, urban, mountain, desert. Other inputs
%    fall through to rural-shaped defaults — the Type dropdown
%    restricts input, so the fall-through is belt-and-braces.
%
%  See also: trackbench.editor.TerrainRecord,
%            trackbench.editor.EditorState,
%            trackbench.editor.loadTerrainFromJSON

    tr = trackbench.editor.TerrainRecord();
    tr.terrainType      = lower(string(typeStr));
    tr.terrainScale     = 1;
    tr.refractionFactor = 1.333;
    tr.readOnly         = false;
    tr.sourceFile       = "";
    tr.originalDef      = struct();

    switch lower(string(typeStr))
        case "none"
            tr.description    = "No terrain — flat ground at z=0. No occlusion, no clutter.";
            tr.clutterDensity = 0;

        case "water"
            tr.description    = "Flat sea level. No effects. Open ocean / coastal.";
            tr.clutterDensity = 0;

        case "rural"
            tr.description    = "Rolling farmland, 80m peaks, light clutter.";
            tr.clutterDensity = 0.3;

        case "urban"
            tr.description    = "City terrain, building clusters 150m, moderate clutter.";
            tr.clutterDensity = 0.6;

        case "mountain"
            tr.description    = "Mountain ridges and peaks 2000m, heavy occlusion.";
            tr.clutterDensity = 0.5;

        case "desert"
            tr.description    = "Gentle dunes 40m, light clutter.";
            tr.clutterDensity = 0.2;

        otherwise
            % Unknown type → rural-shaped guard values. The Type
            % dropdown restricts user input to the supported five,
            % so this branch only fires if a caller invokes with a
            % typo-y string.
            tr.terrainType    = "rural";
            tr.description    = "Rolling farmland, 80m peaks, light clutter.";
            tr.clutterDensity = 0.3;
    end
end
