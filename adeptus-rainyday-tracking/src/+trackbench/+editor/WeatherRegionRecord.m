classdef WeatherRegionRecord
%WeatherRegionRecord  One weather region with a polygon footprint (v3.5 §5a).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  A plain VALUE class. EditorState.weatherRegions is a 1xN array of
%  these. Each region pairs an XY polygon (in scenario NED meters)
%  with a WeatherRecord that describes weather inside that polygon.
%
%  An empty/no-weather region is NOT a thing — if you don't want
%  weather in some area, simply don't draw a polygon there. The
%  scenario-wide fallback (EditorState.weather) handles "no weather
%  by default" when it is itself empty (state.weather = []).
%
%  A scenario therefore has TWO weather layers:
%    1. EditorState.weather        — the scenario-wide fallback
%                                    ([] = clear sky, or 1x1 record)
%    2. EditorState.weatherRegions — zero or more specific storm cells
%
%  The resolver (trackbench.environment.resolveWeatherAt) returns the
%  first region whose polygon contains the query point, or the
%  fallback (which may itself be empty) if none match. A common
%  usage is fallback=empty + one region = "single storm cell over a
%  defined area, clear skies elsewhere."
%
%  WHY VALUE AND NOT HANDLE
%    Same reasoning as WeatherRecord. Undo/redo snapshots are
%    independent value-class copies; handle classes would defeat
%    Ctrl+Z.
%
%  Mutators read-mutate-writeback:
%      wr = state.weatherRegions;
%      rec = wr(idx);
%      rec.polygonXY = newPoly;
%      wr(idx) = rec;
%      state.weatherRegions = wr;
%
%  ON-DISK FIELD NAMES (match config/runs/<n>.json
%                      degradation.weather.regions[] entries)
%    name        <-> name              (optional editor label)
%    configPath  <-> config            (e.g. "rain/heavy_x_band")
%    polygonXY   <-> polygon_xy        (Nx2 array, NED meters)
%
%  The inner `weather` (WeatherRecord) is the resolved record loaded
%  from `configPath` at JSON load time. It is NOT serialized back to
%  the run file — only `name`, `config`, and `polygon_xy` are.
%
%  POLYGON CONVENTION
%    Same as TerrainRegionRecord: Nx2 [x_north, y_east] in NED meters,
%    implicitly closed.
%
%  See also: trackbench.editor.WeatherRecord,
%            trackbench.editor.EditorState,
%            trackbench.environment.resolveWeatherAt,
%            trackbench.editor.TerrainRegionRecord

    properties
        % ── On-disk fields ─────────────────────────────────────────
        name        (1,1) string = ""              % editor label
        configPath  (1,1) string = ""              % "<TYPE>/<file>" no extension
        polygonXY   (:,2) double = zeros(0,2)      % Nx2 NED meters

        % ── Resolved record (cache; not persisted) ─────────────────
        weather     (1,1) trackbench.editor.WeatherRecord = ...
                          trackbench.editor.WeatherRecord

        % ── Editor-local ───────────────────────────────────────────
        %  readOnly mirrors the WeatherRecord pattern — set true for
        %  UNKNOWN-passthrough regions. Editor disables drag handles;
        %  mutators short-circuit.
        readOnly    (1,1) logical = false
    end

    methods
        function tf = isValidPolygon(obj)
            %isValidPolygon  True iff polygonXY can enclose any area.
            %                See TerrainRegionRecord.isValidPolygon for
            %                rationale (≥3 distinct points required).
            n = size(obj.polygonXY, 1);
            if n < 3
                tf = false;
                return;
            end
            uniquePts = unique(obj.polygonXY, 'rows');
            tf = size(uniquePts, 1) >= 3;
        end

        function [xs, ys] = xy(obj)
            %xy  x and y as column vectors for inpolygon-style calls.
            xs = obj.polygonXY(:, 1);
            ys = obj.polygonXY(:, 2);
        end
    end
end
