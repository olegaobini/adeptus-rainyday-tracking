function plotPlatformToTrackAssignment(ax, assignLog, plotTitle, swapReport)
%plotPlatformToTrackAssignment  Timeline view of track-to-truth assignments.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
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
    idx = find(trk == tid);
    if isempty(idx); continue; end

    % Sort this track's rows by time so segment boundaries follow scan
    % order, not row order in the table.
    [~, ord] = sort(t(idx));
    idx = idx(ord);
    tTrack         = t(idx);
    truthsForTrack = tru(idx);
    yValTrack      = yVal(idx);

    % Dominant truth (mode of valid assignments) supplies the LABEL color.
    % The line itself is drawn in segments colored by the truth at each
    % scan so swaps appear as a color change in the line, not just a red
    % X overlay — see swap discussion in analyzeTrackSwaps.m.
    validTruths = truthsForTrack(~isnan(truthsForTrack) & truthsForTrack > 0);
    if isempty(validTruths)
        dominantTruth = NaN;
        labelColor = [0.7 0.7 0.7];
    else
        dominantTruth = mode(validTruths);
        if isKey(truthColorMap, dominantTruth)
            labelColor = truthColorMap(dominantTruth);
        else
            labelColor = [0.7 0.7 0.7];
        end
    end

    % Walk the scan sequence and emit one plot() per run of unchanged
    % truth. isequaln treats two NaNs as equal so "no assignment"
    % stretches don't spuriously break into single-point segments.
    nScans   = numel(tTrack);
    segStart = 1;
    for k = 2:nScans+1
        atEnd = (k > nScans);
        if atEnd
            truthChanged = true;
        else
            truthChanged = ~isequaln(truthsForTrack(k-1), truthsForTrack(k));
        end
        if ~truthChanged; continue; end

        segIdx   = segStart:(k-1);
        segTruth = truthsForTrack(segStart);
        if isnan(segTruth) || segTruth <= 0
            segColor = [0.6 0.6 0.6];  % grey for unassigned stretches
        elseif isKey(truthColorMap, segTruth)
            segColor = truthColorMap(segTruth);
        else
            segColor = [0.6 0.6 0.6];
        end

        tSeg = tTrack(segIdx);
        ySeg = yValTrack(segIdx);
        if numel(tSeg) == 1
            % Single-scan segment — render as a marker so it stays visible
            plot(ax, tSeg, ySeg, 'o', ...
                'MarkerFaceColor', segColor, 'MarkerEdgeColor', segColor, ...
                'MarkerSize', 7, 'HandleVisibility', 'off');
        else
            plot(ax, tSeg, ySeg, '-', ...
                'LineWidth', 3, 'Color', segColor, ...
                'HandleVisibility', 'off');
        end

        segStart = k;
    end

    % Label near first sample. If the track was associated with more than
    % one truth during its lifetime (i.e. a swap occurred on this track),
    % list them in order of first appearance: "T01 -> Truth 2->1".
    uniqueTruths = unique(validTruths, 'stable');
    if isempty(uniqueTruths)
        labelStr = sprintf(' T%02d (unassigned)', tid);
    elseif numel(uniqueTruths) == 1
        labelStr = sprintf(' T%02d \\rightarrow Truth %d', tid, uniqueTruths(1));
    else
        truthChain = sprintf('%d', uniqueTruths(1));
        for u = 2:numel(uniqueTruths)
            truthChain = sprintf('%s\\rightarrow%d', truthChain, uniqueTruths(u));
        end
        labelStr = sprintf(' T%02d \\rightarrow Truth %s', tid, truthChain);
    end
    text(ax, tTrack(1), yValTrack(1), labelStr, ...
        'FontWeight', 'bold', 'Color', labelColor, 'FontSize', 9, ...
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
