function plotTrackSwapAnalysis(swapReport, assignLog, figParent)
%plotTrackSwapAnalysis  Visualize track swap events and assignment stability.
%
% Creates a multi-panel figure showing:
%   1. Assignment timeline with swap events highlighted
%   2. Inter-target separation at swap points (if available)
%   3. Summary scorecard
%
% INPUTS
%   swapReport : struct from analyzeTrackSwaps
%   assignLog  : table from helperRunTracker
%   figParent  : (optional) axes or figure handle. If omitted, creates new figure.

if nargin < 3
    figParent = [];
end

%% Create figure
if isempty(figParent)
    fig = figure('Name', 'Track Swap Analysis', 'Color', 'k', ...
        'NumberTitle', 'off', 'Position', [100 100 1000 700]);
else
    fig = figParent;
end

%% Panel 1: Assignment timeline with swap markers
if swapReport.swapFree
    nPanels = 1;
else
    nPanels = 2;
end

ax1 = subplot(nPanels, 1, 1, 'Parent', fig);
set(ax1, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
    'GridColor', 'w', 'GridAlpha', 0.3);
hold(ax1, 'on'); grid(ax1, 'on');
title(ax1, 'Track-to-Truth Assignment Timeline', 'Color', 'w', 'FontSize', 12);
xlabel(ax1, 'Simulation Time (s)');
ylabel(ax1, 'Truth ID');

if ~isempty(assignLog) && height(assignLog) > 0
    trackIDs = unique(assignLog.TrackID);
    colors = lines(numel(trackIDs));

    for ti = 1:numel(trackIDs)
        tID = trackIDs(ti);
        rows = assignLog(assignLog.TrackID == tID, :);
        rows = sortrows(rows, 'Time');

        plot(ax1, rows.Time, rows.TruthID, '-o', ...
            'Color', colors(ti,:), 'LineWidth', 2, ...
            'MarkerSize', 5, 'MarkerFaceColor', colors(ti,:), ...
            'DisplayName', sprintf('Track %d', tID));
    end

    % Overlay swap events as red X markers
    if ~swapReport.swapFree
        swaps = swapReport.swapEvents;
        plot(ax1, swaps.Time, swaps.ToTruthID, 'rx', ...
            'MarkerSize', 14, 'LineWidth', 3, ...
            'DisplayName', sprintf('SWAP EVENT (n=%d)', swapReport.totalSwaps));
    end

    legend(ax1, 'Location', 'best', 'TextColor', 'w', 'Color', [0.2 0.2 0.2]);
end

% Set integer Y ticks for truth IDs
allTruths = unique(assignLog.TruthID(assignLog.TruthID > 0));
if ~isempty(allTruths)
    yticks(ax1, allTruths);
    ylim(ax1, [min(allTruths)-0.5, max(allTruths)+0.5]);
end

%% Panel 2: Separation at swap time (only if swaps exist)
if ~swapReport.swapFree && nPanels > 1
    ax2 = subplot(nPanels, 1, 2, 'Parent', fig);
    set(ax2, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
        'GridColor', 'w', 'GridAlpha', 0.3);
    hold(ax2, 'on'); grid(ax2, 'on');
    title(ax2, 'Target Separation at Swap Events', 'Color', 'w', 'FontSize', 12);
    xlabel(ax2, 'Swap Time (s)');
    ylabel(ax2, 'Inter-Target Separation (m)');

    swaps = swapReport.swapEvents;
    hasSep = ~isnan(swaps.SeparationAtSwap_m);

    if any(hasSep)
        bar(ax2, swaps.Time(hasSep), swaps.SeparationAtSwap_m(hasSep), ...
            'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'w', 'BarWidth', 0.6);

        % Add text labels
        for k = find(hasSep)'
            text(ax2, swaps.Time(k), swaps.SeparationAtSwap_m(k) + 50, ...
                sprintf('%.0f m', swaps.SeparationAtSwap_m(k)), ...
                'Color', 'w', 'FontSize', 9, 'HorizontalAlignment', 'center');
        end
    else
        text(ax2, 0.5, 0.5, 'Separation data not available', ...
            'Units', 'normalized', 'Color', [0.5 0.5 0.5], ...
            'FontSize', 12, 'HorizontalAlignment', 'center');
    end
end

%% Add scorecard annotation
scorecardStr = buildScorecard(swapReport);
annotation(fig, 'textbox', [0.01, 0.01, 0.98, 0.06], ...
    'String', scorecardStr, ...
    'Color', 'w', 'BackgroundColor', [0.15 0.15 0.15], ...
    'EdgeColor', [0.3 0.3 0.3], ...
    'FontSize', 10, 'FontName', 'Consolas', ...
    'HorizontalAlignment', 'center', ...
    'FitBoxToText', 'off', ...
    'Interpreter', 'none');

end

%% ========================================================================
function str = buildScorecard(sr)
    if sr.swapFree
        str = sprintf('SWAP STATUS: CLEAN | 0 swaps detected | All tracks maintained correct identity throughout');
    else
        str = sprintf('SWAP STATUS: %d SWAP(S) DETECTED | Max consecutive wrong scans: %d', ...
            sr.totalSwaps, sr.maxConsecutive);

        if ~isempty(sr.swapEvents) && any(~isnan(sr.swapEvents.SeparationAtSwap_m))
            meanSep = nanmean(sr.swapEvents.SeparationAtSwap_m);
            minSep  = nanmin(sr.swapEvents.SeparationAtSwap_m);
            str = sprintf('%s | Mean separation at swap: %.0f m | Min: %.0f m', ...
                str, meanSep, minSep);
        end
    end
end
