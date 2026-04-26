function record = resolveWeatherAt(x, y, fallback, regions)
%resolveWeatherAt  Look up the effective weather at scenario coord (x,y).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  v3.5 §5a — multi-region weather resolver.
%
%  Returns the WeatherRecord that applies at the given scenario NED
%  coordinate, or empty (1x0) when there's no weather at that location.
%
%  First-listed region whose polygon contains (x, y) wins; if no region
%  matches, returns the scenario-wide fallback (which may itself be
%  empty, meaning "clear sky outside all regions").
%
%  The (a) + (c) rules from §5a, recap:
%    (a) Region overlap → first-listed in the JSON wins.
%    (c) No-region fallback → returns `fallback` directly. Unlike
%        terrain, fallback is permitted to be empty — that's the
%        common "storm cells over here, clear elsewhere" pattern.
%
%  ARGS
%    x, y      (scalar, NED meters)
%    fallback  trackbench.editor.WeatherRecord OR empty (1x0). Empty
%              means "no global weather" — the run file's
%              degradation.weather was "none" (legacy) or
%              {"fallback": "none", ...} (multi-region).
%    regions   1xN trackbench.editor.WeatherRegionRecord array (may be
%              empty)
%
%  RETURNS
%    record    1x1 trackbench.editor.WeatherRecord (configured weather)
%              OR 1x0 empty (no weather at this location)
%
%  Callers MUST `isempty()`-guard the return — same contract as
%  EditorState.weather. Mirrors the single-weather convention so 5b's
%  runDetections can branch identically whether or not regions are in
%  play.
%
%  See also: trackbench.editor.WeatherRegionRecord,
%            trackbench.environment.resolveTerrainAt
arguments
    x         (1,1) double
    y         (1,1) double
    fallback  (1,:) trackbench.editor.WeatherRecord = ...
              trackbench.editor.WeatherRecord.empty
    regions   (1,:) trackbench.editor.WeatherRegionRecord = ...
              trackbench.editor.WeatherRegionRecord.empty
end

% First-listed wins.
for i = 1:numel(regions)
    r = regions(i);
    if ~r.isValidPolygon
        continue;
    end
    if inpolygon(x, y, r.polygonXY(:,1), r.polygonXY(:,2))
        record = r.weather;
        return;
    end
end

record = fallback;
end
