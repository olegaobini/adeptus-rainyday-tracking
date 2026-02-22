%% Results_Runner.m — Load and display saved results with swap analysis
%
% USAGE: Update the filename below to point to your results .mat file,
% then run this script.

resultsFile = 'results_default_reRunDetections_20260221_152450.mat';

S = load(fullfile('results', resultsFile));
results = S.results;
config  = S.config;
runNames = fieldnames(results);

fprintf('\n========================================\n');
fprintf(' RESULTS REPORT: %s\n', resultsFile);
fprintf('========================================\n');

for k = 1:numel(runNames)
    rn = runNames{k};
    R  = results.(rn);

    fprintf("\n=== %s ===\n", upper(rn));

    % Track summary
    if istable(R.trackSummary)
        disp(R.trackSummary);
    end

    % Track metrics with Quality score
    if istable(R.trackMetrics) || istimetable(R.trackMetrics)
        disp(R.trackMetrics);
    end

    % ---- Swap Analysis ----
    if isfield(R, 'swapReport') && isstruct(R.swapReport)
        sr = R.swapReport;
        fprintf('\n--- Track Swap Analysis ---\n');
        if sr.swapFree
            fprintf('  SWAP STATUS: CLEAN | 0 swaps | All tracks held correct identity\n');
        else
            fprintf('  SWAP STATUS: %d SWAP(S) DETECTED\n', sr.totalSwaps);
            fprintf('  Max consecutive wrong scans: %d\n', sr.maxConsecutive);
            fprintf('\n  Swap Events:\n');
            disp(sr.swapEvents);
            fprintf('  Per-Track Breakdown:\n');
            disp(sr.perTrack);
            fprintf('  Per-Truth Impact:\n');
            disp(sr.perTruth);
        end
        fprintf('----------------------------\n');
    else
        fprintf('\n[NOTE] No swap analysis in this results file.\n');
        fprintf('       Re-run trackingWithWeather() to generate swap data.\n');
    end

    % Plot assignment timeline with swap overlay
    if isfield(R, 'assignLog') && ~isempty(R.assignLog)
        figure('Name', sprintf('Assignment | %s', rn), 'Color', 'k', ...
            'NumberTitle', 'off');
        ax = axes('Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
            'GridColor', 'w', 'GridAlpha', 0.3);

        if isfield(R, 'swapReport')
            plotPlatformToTrackAssignment(ax, R.assignLog, ...
                sprintf('Assignment Timeline | %s', upper(rn)), R.swapReport);
        else
            plotPlatformToTrackAssignment(ax, R.assignLog, ...
                sprintf('Assignment Timeline | %s', upper(rn)));
        end
    end

    % Standalone swap figure (only when swaps occurred)
    if isfield(R, 'swapReport') && isstruct(R.swapReport) && ~R.swapReport.swapFree
        plotTrackSwapAnalysis(R.swapReport, R.assignLog);
    end
end

%% Export to CSV
outDir = fullfile(pwd, 'recovered_exports');
if ~exist(outDir, 'dir'), mkdir(outDir); end

for k = 1:numel(runNames)
    rn = runNames{k};
    R  = results.(rn);

    writetable(R.trackSummary, fullfile(outDir, rn + "_trackSummary.csv"));
    writetable(R.truthSummary, fullfile(outDir, rn + "_truthSummary.csv"));

    if istimetable(R.trackMetrics)
        writetimetable(R.trackMetrics, fullfile(outDir, rn + "_trackMetrics.csv"));
    else
        writetable(R.trackMetrics, fullfile(outDir, rn + "_trackMetrics.csv"));
    end

    if istimetable(R.truthMetrics)
        writetimetable(R.truthMetrics, fullfile(outDir, rn + "_truthMetrics.csv"));
    else
        writetable(R.truthMetrics, fullfile(outDir, rn + "_truthMetrics.csv"));
    end

    writetable(R.assignLog, fullfile(outDir, rn + "_assignLog.csv"));

    % Export swap events if present
    if isfield(R, 'swapReport') && isstruct(R.swapReport)
        if ~R.swapReport.swapFree && height(R.swapReport.swapEvents) > 0
            writetable(R.swapReport.swapEvents, fullfile(outDir, rn + "_swapEvents.csv"));
        end
        writetable(R.swapReport.perTrack, fullfile(outDir, rn + "_swapPerTrack.csv"));
    end
end

fprintf("\nExported CSVs to: %s\n", outDir);
