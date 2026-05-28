function viewSavedResults(matPath)
%viewSavedResults  Re-display a previously-saved Rainy Day results .mat.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  USAGE
%      viewSavedResults                          % opens file picker
%      viewSavedResults("path/to/results.mat")
%
%  WHAT IT DOES
%   The base "load <file>" command pulls the results/config structs into
%   the workspace but doesn't display anything useful — you end up
%   staring at two struct variables. This function reads the same file
%   and re-renders, for each tracker combo it finds:
%     * the trackSummary and truthSummary tables (incl. TrackedPct)
%     * the per-tracker summary box (swaps, RMS, quality, est failure)
%     * the platform-to-track assignment plot (in its own figure)
%     * the swap-analysis plot, if any swaps were detected
%
%   This is the same output runSingleScenario produces at end of run,
%   but reconstructed from the saved .mat so old runs are still useful.
%
%  See also: runSingleScenario, plotPlatformToTrackAssignment, plotTrackSwapAnalysis

    if nargin < 1 || isempty(matPath)
        [file, dirPath] = uigetfile('*.mat', 'Select a saved results .mat file');
        if isequal(file, 0)
            fprintf('[viewSavedResults] No file selected.\n');
            return;
        end
        matPath = fullfile(dirPath, file);
    end
    matPath = char(matPath);

    if ~isfile(matPath)
        error('viewSavedResults:notFound', 'File not found: %s', matPath);
    end

    fprintf('\n[viewSavedResults] Loading: %s\n', matPath);
    S = load(matPath);

    if ~isfield(S, 'results')
        error('viewSavedResults:missingResults', ...
            'File does not contain a "results" struct. Got fields: %s', ...
            strjoin(fieldnames(S), ', '));
    end
    results = S.results;

    % Pull scenario label from config if available
    scenarioLabel = 'unknown';
    if isfield(S, 'config')
        cfg = S.config;
        if isfield(cfg, 'run_name') && ~isempty(cfg.run_name)
            scenarioLabel = char(cfg.run_name);
        end
    elseif isfield(results, 'run_id')
        scenarioLabel = char(results.run_id);
    end

    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════╗\n');
    fprintf('║              VIEWING SAVED RESULTS                       ║\n');
    fprintf('╠══════════════════════════════════════════════════════════╣\n');
    fprintf('║  Scenario : %-45s║\n', scenarioLabel);
    fprintf('║  Source   : %-45s║\n', shortenPath(matPath, 45));
    fprintf('╚══════════════════════════════════════════════════════════╝\n');

    % Walk standard tracker combos in display order
    comboOrder = { ...
        'gnn_cv',    'GNN',   'CV'; ...
        'gnn_imm',   'GNN',   'IMM'; ...
        'tomht_cv',  'TOMHT', 'CV'; ...
        'tomht_imm', 'TOMHT', 'IMM'; ...
        'jpda_cv',   'JPDA',  'CV'; ...
        'jpda_imm',  'JPDA',  'IMM' };

    summaryRows = {};

    for c = 1:size(comboOrder, 1)
        comboName = comboOrder{c, 1};
        if ~isfield(results, comboName); continue; end
        R      = results.(comboName);
        tType  = comboOrder{c, 2};
        fModel = comboOrder{c, 3};

        fprintf('\n============ %s + %s ============\n', tType, fModel);

        if isfield(R, 'trackSummary') && istable(R.trackSummary)
            trackbench.reporting.printCompactTrackSummary(R.trackSummary);
        end
        fprintf('\n');
        if isfield(R, 'truthSummary') && istable(R.truthSummary)
            trackbench.reporting.printCompactTruthSummary(R.truthSummary);
        end

        % Summary box: same shape as runSingleScenario's per-tracker box
        swapStr = 'CLEAN';
        totalSwaps = 0;
        if isfield(R, 'swapReport') && isstruct(R.swapReport)
            if isfield(R.swapReport, 'totalSwaps'); totalSwaps = R.swapReport.totalSwaps; end
            if totalSwaps > 0
                swapStr = sprintf('%d SWAP(S)', totalSwaps);
            end
        end

        posRMS_vals = [];
        velRMS_vals = [];
        if isfield(R, 'trackMetrics') && istable(R.trackMetrics)
            if ismember('posRMS', R.trackMetrics.Properties.VariableNames)
                posRMS_vals = R.trackMetrics.posRMS;
            end
            if ismember('velRMS', R.trackMetrics.Properties.VariableNames)
                velRMS_vals = R.trackMetrics.velRMS;
            end
        end

        fprintf('\n  ┌─────────────────────────────────────────────\n');
        fprintf('  │ %s + %s  SUMMARY (from saved .mat)\n', tType, fModel);
        fprintf('  ├─────────────────────────────────────────────\n');
        fprintf('  │ Track Swaps : %s\n', swapStr);
        if ~isempty(posRMS_vals)
            fprintf('  │ Position RMS: ');
            for ti = 1:numel(posRMS_vals); fprintf('T%d=%.1fm  ', ti, posRMS_vals(ti)); end
            fprintf('\n');
        end
        if ~isempty(velRMS_vals)
            fprintf('  │ Velocity RMS: ');
            for ti = 1:numel(velRMS_vals); fprintf('T%d=%.1fm/s  ', ti, velRMS_vals(ti)); end
            fprintf('\n');
        end
        if isfield(R, 'trackMetrics') && istable(R.trackMetrics) ...
                && ismember('Quality', R.trackMetrics.Properties.VariableNames)
            fprintf('  │ Quality     : ');
            for ti = 1:height(R.trackMetrics)
                fprintf('T%d=%s  ', ti, string(R.trackMetrics.Quality(ti)));
            end
            fprintf('\n');
        end

        % Establishment-failure flag (matches runSingleScenario's check)
        if isfield(R, 'truthSummary') && istable(R.truthSummary) ...
                && height(R.truthSummary) > 0 ...
                && all(ismember({'TruthID','TotalLength','EstablishmentLength'}, ...
                    R.truthSummary.Properties.VariableNames))
            estLateMsgs = strings(0, 1);
            for ti = 1:height(R.truthSummary)
                tl = R.truthSummary.TotalLength(ti);
                el = R.truthSummary.EstablishmentLength(ti);
                if tl > 0 && (el / tl) > 0.25
                    estLateMsgs(end+1) = sprintf('Truth%d est@%d/%d (%.0f%%)', ...
                        R.truthSummary.TruthID(ti), el, tl, 100 * el / tl); %#ok<AGROW>
                end
            end
            if ~isempty(estLateMsgs)
                fprintf('  │ Est failure : %s\n', strjoin(estLateMsgs, ', '));
            end
        end
        fprintf('  └─────────────────────────────────────────────\n');

        avgPos = ternary(~isempty(posRMS_vals), sprintf('%.1f', mean(posRMS_vals)), 'N/A');
        summaryRows{end+1} = {tType, fModel, swapStr, avgPos}; %#ok<AGROW>

        % Re-render assignment plot (one figure per tracker for clarity)
        if isfield(R, 'assignLog') && istable(R.assignLog) && height(R.assignLog) > 0
            try
                figAssign = figure('Name', ...
                    sprintf('%s — %s+%s Assignment', scenarioLabel, tType, fModel), ...
                    'NumberTitle', 'off', 'Color', 'k');
                axAssign = axes(figAssign);
                set(axAssign, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
                swapReportArg = [];
                if isfield(R, 'swapReport'); swapReportArg = R.swapReport; end
                trackbench.reporting.plotPlatformToTrackAssignment(axAssign, ...
                    R.assignLog, ...
                    sprintf('%s + %s — Platform to Track Assignment', tType, fModel), ...
                    swapReportArg);
            catch ME
                fprintf('  [WARN] Could not redraw assignment plot: %s\n', ME.message);
            end
        end

        % Re-render swap analysis if swaps occurred
        if totalSwaps > 0 && isfield(R, 'assignLog') && istable(R.assignLog)
            try
                trackbench.reporting.plotTrackSwapAnalysis(R.swapReport, R.assignLog, [], ...
                    sprintf('%s + %s', tType, fModel));
            catch ME
                fprintf('  [WARN] Could not redraw swap analysis: %s\n', ME.message);
            end
        end
    end

    % Cross-tracker summary table (matches runSingleScenario's final box)
    if ~isempty(summaryRows)
        fprintf('\n');
        fprintf('╔═══════════════════════════════════════════════════════╗\n');
        fprintf('║           %s — SAVED RESULTS                ║\n', upper(scenarioLabel));
        fprintf('╠═══════════════════════════════════════════════════════╣\n');
        fprintf('║  %-8s %-6s %-12s %-24s║\n', 'Tracker', 'Filter', 'Swaps', 'Avg posRMS (m)');
        fprintf('╠═══════════════════════════════════════════════════════╣\n');
        for ri = 1:numel(summaryRows)
            r = summaryRows{ri};
            fprintf('║  %-8s %-6s %-12s %-24s║\n', r{1}, r{2}, r{3}, r{4});
        end
        fprintf('╚═══════════════════════════════════════════════════════╝\n');
    end

    fprintf('\n[viewSavedResults] Done. Figures: %d.\n', numel(findall(0, 'Type', 'figure')));
end

%% =====================================================================
%  Local helpers
%% =====================================================================
function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end

function s = shortenPath(p, maxLen)
    % If a path is longer than maxLen, replace the middle with ... so the
    % end (filename) stays visible. Keeps the header box neat.
    if numel(p) <= maxLen
        s = p;
        return;
    end
    keepEnd = maxLen - 4;  % " ..." = 4 chars
    s = ['...', p(end-keepEnd+1:end)];
end
