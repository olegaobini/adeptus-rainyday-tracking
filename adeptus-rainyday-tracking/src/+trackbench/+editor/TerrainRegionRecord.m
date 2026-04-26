classdef TerrainRegionRecord
%TerrainRegionRecord  One terrain region with a polygon footprint (v3.5 §5a).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  A plain VALUE class. EditorState.terrainRegions is a 1xN array of
%  these. Each region pairs an XY polygon (in scenario NED meters)
%  with a TerrainRecord that describes the terrain inside that polygon.
%
%  A scenario therefore has TWO terrain layers:
%    1. EditorState.terrain        — the scenario-wide fallback
%    2. EditorState.terrainRegions — zero or more specific regions
%
%  The resolver (trackbench.environment.resolveTerrainAt) returns
%  the first region whose polygon contains the query point, or the
%  fallback if none match.
%
%  WHY VALUE AND NOT HANDLE
%    Same reasoning as TerrainRecord, SensorRecord, TargetRecord.
%    Undo/redo snapshots a value-class array cleanly — each snap is
%    an independent copy. With handles, Ctrl+Z would do nothing
%    because the snapshot and the live record would share memory.
%
%  Mutators read-mutate-writeback (matches the existing collection
%  pattern for targets/sensors):
%      tr = state.terrainRegions;
%      rec = tr(idx);
%      rec.polygonXY = newPoly;
%      tr(idx) = rec;
%      state.terrainRegions = tr;
%
%  ON-DISK FIELD NAMES (match config/runs/<name>.json
%                      terrain.regions[] entries)
%    name        <-> name              (optional editor label)
%    configPath  <-> config            (e.g. "mountain/sharp_peaks")
%    polygonXY   <-> polygon_xy        (Nx2 array, NED meters)
%
%  The inner `terrain` (TerrainRecord) is the resolved record loaded
%  from `configPath` at JSON load time. It is NOT serialized back to
%  the run file — only `name`, `config`, and `polygon_xy` are. The
%  inner record is a load-time cache so the editor and sim engine
%  don't have to re-read the referenced file on every query.
%
%  POLYGON CONVENTION
%    Nx2 array of [x_north, y_east] pairs in scenario NED meters
%    (matches the coordinate system used by addTargetFromDef and
%    drawMap). The polygon is implicitly closed — first point
%    auto-connects to last; do not duplicate the first point at
%    the end. inpolygon() handles the closure.
%
%  See also: trackbench.editor.TerrainRecord,
%            trackbench.editor.EditorState,
%            trackbench.environment.resolveTerrainAt,
%            trackbench.editor.WeatherRegionRecord

    properties
        % ── On-disk fields ─────────────────────────────────────────
        name        (1,1) string = ""              % editor label
        configPath  (1,1) string = ""              % "<TYPE>/<file>" no extension
        polygonXY   (:,2) double = zeros(0,2)      % Nx2 NED meters

        % ── Resolved record (cache; not persisted) ─────────────────
        terrain     (1,1) trackbench.editor.TerrainRecord = ...
                          trackbench.editor.TerrainRecord

        % ── Editor-local ───────────────────────────────────────────
        %  readOnly mirrors the TerrainRecord/SensorRecord pattern —
        %  set true for UNKNOWN-passthrough regions (configPath points
        %  to an unsupported terrain type that loaded as passthrough).
        %  The editor disables polygon drag handles for read-only
        %  regions; mutators short-circuit.
        readOnly    (1,1) logical = false
    end

    methods
        function tf = isValidPolygon(obj)
            %isValidPolygon  True iff polygonXY can enclose any area.
            %
            %  Requires at least 3 distinct points. Two-point or
            %  single-point regions cannot enclose area, so inpolygon()
            %  at resolve time would always return false — flag them
            %  here so the resolver can `continue` past them cheaply.
            %
            %  Also rejects degenerate polygons where every point
            %  coincides (would otherwise log a polyshape warning).
            n = size(obj.polygonXY, 1);
            if n < 3
                tf = false;
                return;
            end
            uniquePts = unique(obj.polygonXY, 'rows');
            tf = size(uniquePts, 1) >= 3;
        end

        function [xs, ys] = xy(obj)
            %xy  Convenience accessor returning x and y as column
            %    vectors — matches inpolygon's calling convention so
            %    callers can write inpolygon(qx, qy, r.xy()).
            xs = obj.polygonXY(:, 1);
            ys = obj.polygonXY(:, 2);
        end
    end
end
