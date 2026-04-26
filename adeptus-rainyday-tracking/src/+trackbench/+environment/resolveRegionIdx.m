function idx = resolveRegionIdx(x, y, regions)
%resolveRegionIdx  Find which (cell-array) region contains (x, y).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  v3.5 §5b — sim-engine companion to the editor-side
%  resolveTerrainAt / resolveWeatherAt (which work on typed
%  TerrainRegionRecord / WeatherRegionRecord arrays).
%
%  Returns the 1-based index of the first region whose polygon contains
%  (x, y), or 0 if no region matches. The 0/1..N return shape is chosen
%  so callers can:
%    - look up the per-region terrain/weather def via regions{idx}.def
%    - cache pre-computed effects (e.g. pdMult function handles) per
%      region in a 1×(N+1) cell array indexed by [idx + 1] (with cell 1
%      being the fallback's cached effects)
%
%  This index-based shape is what runDetections.m uses to avoid re-
%  computing weather effects per detection — it pre-computes once per
%  region per scan and indexes by resolveRegionIdx output.
%
%  Operates on the cell-array-of-structs shape that loadRunFile produces
%  (each entry has .polygon_xy, .config_path, .name, .def).
%
%  ARGS
%    x, y     scalar NED meters
%    regions  cell array of region structs (may be empty)
%
%  RETURNS
%    idx      0 (no match → fallback) or 1..numel(regions) (first match)
%
%  See also: trackbench.environment.resolveTerrainAt,
%            trackbench.environment.resolveWeatherAt,
%            trackbench.environment.composeHeightmap
arguments
    x       (1,1) double
    y       (1,1) double
    regions cell
end

idx = 0;
for i = 1:numel(regions)
    r = regions{i};
    if ~isfield(r, 'polygon_xy') || size(r.polygon_xy, 1) < 3
        continue;
    end
    if inpolygon(x, y, r.polygon_xy(:,1), r.polygon_xy(:,2))
        idx = i;
        return;
    end
end
end
