function plotPlatformToTrackAssignment(ax, assignLog, plotTitle, swapReport)
%plotPlatformToTrackAssignment  Timeline view of track-to-truth assignments.
%
% Shows each confirmed track as a colored line over time. Each line is
% labeled with both the Track ID and the Truth ID it's assigned to, so
% you can immediately see which track is following which truth and when
% assignments change (swaps).
%
% assignLog must contain columns:
%   Time (seconds), PlatformID, TrackID, TruthID
%
% swapReport (optional): output from analyzeTrackSwaps. If provided,
%   swap events are marked on the timeline with red X markers and
%   annotated with from->to truth labels.

if nargin < 3; plotTitle = "Platform to Track Assignment"; end
if nargin < 4; swapReport = []; end

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

t    = assignLog.Time(:);
plat = assignLog.PlatformID(:);
trk  = assignLog.TrackID(:);
tru  = assignLog.TruthID(:);

uPlat = unique(plat(~isnan(plat)));
uTrk  = unique(trk(~isnan(trk)));
uTru  = unique(tru(~isnan(tru)));

% If only 1 platform exists, plot y=TrackID instead
usePlatformYAxis = numel(uPlat) > 1;

if usePlatformYAxis
    ylabel(ax, 'Platform ID');
    yVal = plat;
else
    ylabel(ax, 'Track ID (single-platform view)');
    yVal = trk;
end

% Color palette — one color per TRUTH so all tracks on the same truth
% share a color. This makes it visually obvious which truth is being tracked.
truthColors = lines(max(10, numel(uTru)));
truthColorMap = containers.Map('KeyType','double','ValueType','any');
for i = 1:numel(uTru)
    truthColorMap(uTru(i)) = truthColors(i, :);
end

for i = 1:numel(uTrk)
    tid = uTrk(i);
    idx = (trk == tid);
    if ~any(idx); continue; end

    % Find the dominant truth for this track (mode of TruthID assignments)
    truthsForTrack = tru(idx);
    truthsForTrack = truthsForTrack(~isnan(truthsForTrack));
    if isempty(truthsForTrack)
        ci = [0.5 0.5 0.5];  % grey for unassigned
        dominantTruth = NaN;
    else
        dominantTruth = mode(truthsForTrack);
        if isKey(truthColorMap, dominantTruth)
            ci = truthColorMap(dominantTruth);
        else
            ci = [0.5 0.5 0.5];
        end
    end

    % Draw as a thick line
    plot(ax, t(idx), yVal(idx), '-', 'LineWidth', 3, 'Color', ci, ...
        'HandleVisibility', 'off');

    % Label near first sample: "T01 → Truth 2"
    i0 = find(idx, 1, 'first');
    if isnan(dominantTruth)
        labelStr = sprintf(' T%02d (unassigned)', tid);
    else
        labelStr = sprintf(' T%02d \\rightarrow Truth %d', tid, dominantTruth);
    end
    text(ax, t(i0), yVal(i0), labelStr, ...
        'FontWeight', 'bold', 'Color', ci, 'FontSize', 9, ...
        'VerticalAlignment', 'bottom', 'Interpreter', 'tex');
end

%% Add truth color legend
for i = 1:numel(uTru)
    ci = truthColorMap(uTru(i));
    plot(ax, NaN, NaN, '-', 'LineWidth', 3, 'Color', ci, ...
        'DisplayName', sprintf('Truth %d', uTru(i)));
end

%% Overlay swap events
if ~isempty(swapReport) && isstruct(swapReport) && isfield(swapReport,'totalSwaps') && swapReport.totalSwaps > 0
    evts = swapReport.swapEvents;
    for s = 1:height(evts)
        swapTime = evts.Time(s);
        swapTrack = evts.TrackID(s);
        fromTruth = evts.FromTruthID(s);
        toTruth   = evts.ToTruthID(s);
        
        % Y position for this track
        if usePlatformYAxis
            swapRows = assignLog(assignLog.Time == swapTime & assignLog.TrackID == swapTrack, :);
            if height(swapRows) > 0
                ySwap = swapRows.PlatformID(1);
            else
                continue;
            end
        else
            ySwap = swapTrack;
        end
        
        % Red X marker at swap point
        plot(ax, swapTime, ySwap, 'rx', 'MarkerSize', 14, 'LineWidth', 3, ...
            'HandleVisibility', 'off');
        
        % Annotation: "T1→T2" label
        label = sprintf('Truth %d\\rightarrow%d', fromTruth, toTruth);
        text(ax, swapTime, ySwap + 0.15, label, ...
            'Color', 'r', 'FontWeight', 'bold', 'FontSize', 9, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
            'Interpreter', 'tex');
    end
    
    % Legend entry for swaps
    plot(ax, NaN, NaN, 'rx', 'MarkerSize', 10, 'LineWidth', 2, ...
        'DisplayName', sprintf('Swap Events (%d)', swapReport.totalSwaps));
    
    % Swap summary in subtitle
    summaryStr = sprintf('SWAPS: %d | Max consecutive wrong: %d scans', ...
        swapReport.totalSwaps, swapReport.maxConsecutive);
    if ~isempty(swapReport.swapEvents) && any(~isnan(swapReport.swapEvents.SeparationAtSwap_m))
        meanSep = mean(swapReport.swapEvents.SeparationAtSwap_m, 'omitnan');
        summaryStr = sprintf('%s | Mean sep at swap: %.0f m', summaryStr, meanSep);
    end
    subtitle(ax, summaryStr, 'Color', [1 0.3 0.3], 'FontSize', 9);
end

legend(ax, 'show', 'Location', 'best', 'TextColor', 'w', 'Color', [0.15 0.15 0.15]);

% Nicer limits
xlim(ax, [min(t)-0.5, max(t)+0.5]);
if usePlatformYAxis
    ylim(ax, [min(uPlat)-0.5, max(uPlat)+0.5]);
else
    ylim(ax, [min(uTrk)-0.5, max(uTrk)+0.5]);
end

end
