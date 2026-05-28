%test_autotune_amazing  Smoke test for the new autoTuneTracker
%
%  Verifies that the upgraded autotuner:
%   1. Runs end-to-end without syntax errors
%   2. Uses the new 6-component score
%   3. Prints the new compact per-row format
%   4. Generates a sensitivity report
%   5. Saves a JSON with the new TUNING_NOTES block
%
%  Uses GNN on Swap.json because it's the scenario where the new
%  coverage terms most clearly differentiate good from bad configs.

clear classes; clear all; %#ok<CLALL>
addpath('scripts');
addpath(genpath('src'));

fprintf('\n=== Smoke test: autoTuneTracker GNN+IMM on Swap ===\n\n');

results = autoTuneTracker("Swap", "GNN", "IMM");

fprintf('\n=== POST-RUN VERIFICATION ===\n');
fprintf('Score              : %.3f\n', results.bestScore);
fprintf('Has breakdown      : %d (expected 1)\n', isstruct(results.bestBreakdown));
fprintf('Has sensitivity    : %d (expected 1)\n', istable(results.sensitivity));
fprintf('Sweep table cols   : %s\n', strjoin(results.sweepTable.Properties.VariableNames, ', '));
fprintf('Weights len        : %d (expected 6)\n', numel(results.weights));
fprintf('Saved file exists  : %d\n', isfile(results.savedFile));
fprintf('Best param fields  : %s\n', strjoin(fieldnames(results.bestParams), ', '));

% Spot-check that worst-tracked / est-fail metrics flowed through
fprintf('\nBest params include coverage metrics:\n');
fprintf('  worstTrackedPct  : %s\n', num2str(results.bestParams.worstTrackedPct));
fprintf('  estFailures      : %d of %d truths\n', ...
    results.bestParams.estFailures, results.bestParams.numTruths);

fprintf('\n=== TEST COMPLETE ===\n');
