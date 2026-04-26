function Z = composeHeightmap(fallbackZ, regions, Xg, Yg, scenBounds)
%composeHeightmap  Compose a multi-region heightmap from fallback + regions.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  v3.5 §5b — multi-region terrain heightmap composition (option (i) from
%  the §5a/§5b plan: composite heightmap, MATLAB groundSurface sees one
%  grid, occlusion runs through the existing radar pipeline).
%
%  Stamps each region's terrain into the fallback heightmap. Regions are
%  resolved first-listed-wins (matching the §5a (a) rule): once a grid
%  cell is claimed by an earlier region, later overlapping regions do
%  not overwrite it.
%
%  POLYGON EDGES
%    Polygon edges produce a hard step in elevation. This is acceptable
%    for radar sim — terrain types already differ by 100s to 1000s of
%    meters, so a sharp boundary at a polygon edge is visually
%    consistent with the existing terrain step at the radar hilltop
%    clearing (see generateTerrain). Smooth blending could be added as
%    a follow-up; not needed for the May 1 Boeing demo.
%
%  ARGS
%    fallbackZ   NxN heightmap from generateTerrain(fallbackType, ...)
%                — used as the base layer for cells outside all regions
%    regions     cell array of region structs from loadRunFile, each
%                with fields {.config_path, .name, .polygon_xy, .def}
%                where .def is the inner terrainDef (terrain_type,
%                terrain_scale, ...). Empty cell → returns fallbackZ
%                unchanged (this is the legacy single-terrain path).
%    Xg, Yg      NxN meshgrids matching fallbackZ (NED meters)
%    scenBounds  [xMin xMax; yMin yMax] passed through to generateTerrain
%                so each region's heightmap uses identical grid resolution
%                — required for the mask-based stamping to work cell-wise.
%
%  RETURNS
%    Z           NxN composed heightmap, same dimensions as fallbackZ
%
%  PERFORMANCE
%    Called ONCE per scenario load (from loadRunFile step 9). Cost is
%    O(numRegions × generateTerrain cost). Each generateTerrain call is
%    ~50 ms for the default 200×200 grid, so 5 regions adds ~250 ms to
%    load. Negligible vs the seconds runDetections takes to run.
%
%  See also: trackbench.environment.generateTerrain,
%            trackbench.environment.resolveRegionIdx,
%            trackbench.config.loadRunFile (step 9)
arguments
    fallbackZ   double
    regions     cell
    Xg          double
    Yg          double
    scenBounds  (2,2) double
end

Z = fallbackZ;
if isempty(regions)
    return;   % legacy single-terrain path — no-op
end

% First-listed-wins: track which cells have already been claimed by an
% earlier region. Later regions can stamp only into still-unclaimed
% cells. Same rule as resolveRegionIdx (early return on first match).
claimed = false(size(Z));

for i = 1:numel(regions)
    r = regions{i};
    if ~isfield(r, 'polygon_xy') || size(r.polygon_xy, 1) < 3
        continue;   % degenerate polygon — skip
    end

    % Cells inside this region AND not yet claimed.
    inside = inpolygon(Xg, Yg, r.polygon_xy(:,1), r.polygon_xy(:,2));
    mask = inside & ~claimed;
    nCells = sum(mask, 'all');
    if nCells == 0
        continue;   % fully shadowed by earlier region(s)
    end

    % Build this region's heightmap. Same scenBounds as fallback, so
    % grid dims match and we can index with `mask` directly. `def` is
    % the parsed terrain JSON (terrain_type, terrain_scale, ...).
    rType = 'water';
    rScale = 1.0;
    if isfield(r.def, 'terrain_type');  rType  = r.def.terrain_type;  end
    if isfield(r.def, 'terrain_scale'); rScale = r.def.terrain_scale; end
    [regionZ, ~, ~, ~] = trackbench.environment.generateTerrain( ...
        rType, scenBounds, rScale);

    Z(mask) = regionZ(mask);
    claimed(mask) = true;

    rName = '';
    if isfield(r, 'name'); rName = char(r.name); end
    fprintf('[Heightmap] Region %d: "%s" (%s) -> %d / %d cells\n', ...
        i, rName, rType, nCells, numel(Z));
end
end
