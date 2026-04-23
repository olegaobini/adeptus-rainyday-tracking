classdef TerrainRecord
%TerrainRecord  Per-scenario terrain state for the editor (M7 §3.1).
%
%  A plain VALUE class. Every scenario owns exactly one TerrainRecord
%  at EditorState.terrain — unlike targets and sensors, terrain is
%  not a collection. Mutators read-mutate-writeback:
%      tr = state.terrain;
%      tr.terrainType = "mountain";
%      state.terrain = tr;
%
%  WHY VALUE AND NOT HANDLE
%    Same reasoning as TargetRecord and SensorRecord. Undo/redo
%    snapshots the record. With value classes the snapshot is an
%    independent copy; with handle classes Ctrl+Z would do nothing
%    because the snapshot and the live record would be the same
%    object. Do not change this.
%
%  SUPPORTED TYPES (M7 editing)
%    water, rural, urban, mountain, desert. Anything else loads as
%    UNKNOWN passthrough: readOnly=true, originalDef holds the
%    verbatim parsed JSON so re-export round-trips it. The editor
%    disables Terrain-panel fields in passthrough and renders the
%    map with a neutral gray tint.
%
%  ON-DISK FIELD NAMES (match config/terrain/<TYPE>/<file>.json)
%    description      <-> description
%    terrainType      <-> terrain_type
%    terrainScale     <-> terrain_scale
%    clutterDensity   <-> clutter_density
%    refractionFactor <-> refraction_factor
%
%  REFRACTION FACTOR NOTE
%    The sim uses 4/3 ≈ 1.333 as the standard 4/3 Earth radius
%    refraction model. Stored as 1.333 in JSON (tolerance >> 0.001);
%    generateTerrain treats scale 1.0 as the nominal elevation scale.
%
%  See also: trackbench.editor.EditorState,
%            trackbench.editor.terrainDefaults,
%            trackbench.editor.loadTerrainFromJSON

    properties
        % ── On-disk fields ────────────────────────────────────────
        description      (1,1) string = "Rolling farmland, 80m peaks, light clutter."
        terrainType      (1,1) string = "rural"    % water|rural|urban|mountain|desert
        terrainScale     (1,1) double = 1
        clutterDensity   (1,1) double = 0.3
        refractionFactor (1,1) double = 1.333

        % ── Editor-local / passthrough ─────────────────────────────
        %  readOnly=true for UNKNOWN passthrough (unsupported
        %  terrain_type loaded from file). sourceFile is the relative
        %  path the record was loaded from ("mountain/my_terrain") or
        %  "" for editor-authored-from-scratch. originalDef carries
        %  the verbatim parsed JSON struct for passthrough re-export.
        readOnly     (1,1) logical = false
        sourceFile   (1,1) string  = ""
        originalDef  (1,1) struct  = struct()
    end

    methods
        function tf = isFlat(obj)
            %isFlat  True when the terrain has no elevation features —
            %        only water qualifies. Used by the 2D map tint
            %        renderer to decide whether to skip drawing the
            %        tint at all (the sim's generateTerrain returns a
            %        zero heightmap for water).
            tf = strcmpi(obj.terrainType, "water");
        end

        function tf = isSupportedType(obj)
            %isSupportedType  True iff terrainType is one of the five
            %                 editor-editable types. False on UNKNOWN
            %                 passthrough — UI should disable fields.
            supported = ["water","rural","urban","mountain","desert"];
            tf = any(strcmpi(obj.terrainType, supported));
        end
    end
end
