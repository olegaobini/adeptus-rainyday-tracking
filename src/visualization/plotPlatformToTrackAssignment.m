function plotPlatformToTrackAssignment(ax, assignLog, plotTitle)
%plotPlatformToTrackAssignment  Timeline view like the multiplatform example.
%
% assignLog must contain columns:
%   Time (seconds), PlatformID, TrackID, TruthID

if nargin < 3
    plotTitle = "Platform to Track Assignment";
end

cla(ax); hold(ax,'on'); grid(ax,'on');
title(ax, plotTitle);
xlabel(ax, 'Simulation time (s)');

if isempty(assignLog) || height(assignLog) == 0
    ylabel(ax, '');
    text(ax, 0.5, 0.5, "No assignments logged", 'Units','normalized', ...
        'HorizontalAlignment','center');
    return;
end

% Defensive: ensure required vars exist
req = ["Time","PlatformID","TrackID","TruthID"];
for k = 1:numel(req)
    if ~any(strcmp(assignLog.Properties.VariableNames, req(k)))
        error("assignLog is missing required column: %s", req(k));
    end
end

t = assignLog.Time(:);
plat = assignLog.PlatformID(:);
trk  = assignLog.TrackID(:);

uPlat = unique(plat(~isnan(plat)));
uTrk  = unique(trk(~isnan(trk)));

% If only 1 platform exists, plotting y=PlatformID would overlap all tracks.
% So: auto-switch to y=TrackID for single-platform runs.
usePlatformYAxis = numel(uPlat) > 1;

if usePlatformYAxis
    ylabel(ax, 'Platform ID');
    yVal = plat;
else
    ylabel(ax, 'Track ID (single-platform view)');
    yVal = trk;
end

% color palette
cmap = lines(max(10, numel(uTrk)));

for i = 1:numel(uTrk)
    tid = uTrk(i);
    idx = (trk == tid);

    if ~any(idx); continue; end

    ci = cmap(mod(i-1, size(cmap,1))+1, :);

    % draw as a thick line of points (reads like the example)
    plot(ax, t(idx), yVal(idx), '-', 'LineWidth', 3, 'Color', ci);

    % label near first sample
    i0 = find(idx,1,'first');
    text(ax, t(i0), yVal(i0), sprintf(" T%02d", tid), ...
        'FontWeight','bold', 'Color', ci, 'VerticalAlignment','bottom');
end

% nicer limits
xlim(ax, [min(t)-0.5, max(t)+0.5]);
if usePlatformYAxis
    ylim(ax, [min(uPlat)-0.5, max(uPlat)+0.5]);
else
    ylim(ax, [min(uTrk)-0.5, max(uTrk)+0.5]);
end

end