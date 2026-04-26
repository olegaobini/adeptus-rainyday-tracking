function record = resolveTerrainAt(x, y, fallback, regions)
%resolveTerrainAt  Look up the effective terrain at scenario coord (x,y).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  v3.5 §5a — multi-region terrain resolver.
%
%  Returns the TerrainRecord that applies at the given scenario NED
%  coordinate. First-listed region whose polygon contains (x, y) wins;
%  if no region matches, returns the scenario-wide fallback.
%
%  The (a) + (c) rules from §5a, recap:
%    (a) Region overlap → first-listed in the JSON wins. The order in
%        regions(:) is the order in the run file's regions array.
%    (c) No-region fallback → returns `fallback` directly. Callers
%        must always pass a valid fallback record (terrain has no
%        equivalent of weather's "none" — every point on the map
%        must resolve to some terrain to drive the heightmap).
%
%  ARGS
%    x, y      (scalar, NED meters)
%    fallback  trackbench.editor.TerrainRecord — the scenario default
%    regions   1xN trackbench.editor.TerrainRegionRecord array (may be
%              empty: zeros(0,1) or .empty). Empty array is the legacy/
%              single-terrain case and resolves to fallback for every
%              query.
%
%  RETURNS
%    record    trackbench.editor.TerrainRecord — never empty
%
%  PERFORMANCE NOTE
%    Designed to be called per-detection in runDetections.m (5b). For
%    the typical scenario (≤ 5 regions, ≤ 20 polygon vertices each,
%    ~100s of detections per scan) the cost is negligible — inpolygon
%    on small polygons is fast and the early-return on first match
%    keeps the average region count touched << numel(regions).
%
%    For high-region-count scenarios, callers should batch queries
%    and switch to a vectorized inpolygon (see resolveTerrainAtMany,
%    introduced in 5b if profiling shows it's needed).
%
%  See also: trackbench.editor.TerrainRegionRecord,
%            trackbench.environment.resolveWeatherAt
arguments
    x         (1,1) double
    y         (1,1) double
    fallback  (1,1) trackbench.editor.TerrainRecord
    regions   (1,:) trackbench.editor.TerrainRegionRecord = ...
              trackbench.editor.TerrainRegionRecord.empty
end

% First-listed wins. Cheapest possible loop — region counts are
% small (typically < 10) and isValidPolygon short-circuits degenerate
% entries before paying for inpolygon.
for i = 1:numel(regions)
    r = regions(i);
    if ~r.isValidPolygon
        continue;   % degenerate region — skip, never matches
    end
    if inpolygon(x, y, r.polygonXY(:,1), r.polygonXY(:,2))
        record = r.terrain;
        return;
    end
end

record = fallback;
end
