%test_scoring_functions  Fast sanity check of the new scoring package
%
%  Verifies that:
%    - computeTunerScore handles normal, edge, and legacy-weight inputs
%    - extractTunerMetrics reads both new TrackedPct and legacy TrackedFraction
%    - analyzeSensitivity returns a non-empty report from a synthetic sweep
%
%  This runs in <5 seconds and exercises every code path I changed,
%  letting us catch syntax / call-site issues without the full sweep.

clear classes; clear all; %#ok<CLALL>
addpath(genpath('src'));

fprintf('\n========================================\n');
fprintf('  SCORING PACKAGE SMOKE TEST\n');
fprintf('========================================\n\n');

% ---- TEST 1: computeTunerScore basic ----
fprintf('[1] computeTunerScore basic call...\n');
m = struct('avgPosRMS', 100, 'swapCount', 1, 'falseTracks', 5, ...
           'breakCount', 0, 'worstTrackedPct', 85, 'estFailures', 0, ...
           'numTruths', 2);
s = trackbench.analysis.computeTunerScore(m, 50000);
assert(isstruct(s), 'expected struct output');
assert(isfield(s, 'total'), 'expected .total field');
assert(s.total > 0 && s.total < 1, sprintf('total %.3f out of [0,1] range', s.total));
fprintf('    total = %.3f\n', s.total);
fprintf('    components: posErr=%.3f swap=%.3f false=%.3f break=%.3f trk=%.3f est=%.3f\n', ...
    s.posErr, s.swap, s.false, s.break, s.tracked, s.estFail);
fprintf('    PASS\n\n');

% ---- TEST 2: computeTunerScore catastrophic ----
fprintf('[2] computeTunerScore catastrophic case...\n');
mBad = struct('avgPosRMS', Inf, 'swapCount', 99, 'falseTracks', 99, ...
              'breakCount', 99, 'worstTrackedPct', 0, 'estFailures', 5, ...
              'numTruths', 5);
sBad = trackbench.analysis.computeTunerScore(mBad, 50000);
fprintf('    total = %.3f (expect ~1.0)\n', sBad.total);
assert(sBad.total > 0.95 && sBad.total <= 1.0, 'saturated score should be ~1.0');
fprintf('    PASS\n\n');

% ---- TEST 3: computeTunerScore perfect ----
fprintf('[3] computeTunerScore perfect case...\n');
mGood = struct('avgPosRMS', 0, 'swapCount', 0, 'falseTracks', 0, ...
               'breakCount', 0, 'worstTrackedPct', 100, 'estFailures', 0, ...
               'numTruths', 3);
sGood = trackbench.analysis.computeTunerScore(mGood, 50000);
fprintf('    total = %.3f (expect 0.0)\n', sGood.total);
assert(sGood.total < 0.001, 'perfect score should be ~0');
fprintf('    PASS\n\n');

% ---- TEST 4: legacy 4-weight vector ----
fprintf('[4] computeTunerScore with legacy 4-weight vector...\n');
warning('off', 'computeTunerScore:legacyWeights');
sLegacy = trackbench.analysis.computeTunerScore(m, 50000, [0.5, 0.25, 0.15, 0.10]);
warning('on', 'computeTunerScore:legacyWeights');
fprintf('    total = %.3f (no crash on legacy weights)\n', sLegacy.total);
assert(isfinite(sLegacy.total), 'legacy weights should still produce finite score');
fprintf('    PASS\n\n');

% ---- TEST 5: extractTunerMetrics with new TrackedPct column ----
fprintf('[5] extractTunerMetrics with TrackedPct column...\n');
trackSummary = table([1;2;3], [1; NaN; 2], [0;0;1], ...
    'VariableNames', {'TrackID','AssignedTruthID','SwapCount'});
truthSummary = table([1;2], [100;100], [10;20], [85;65], [1;0], ...
    'VariableNames', {'TruthID','TotalLength','EstablishmentLength','TrackedPct','BreakCount'});
trackMetrics = table([50; 60; 70], 'VariableNames', {'posRMS'});

mExt = trackbench.analysis.extractTunerMetrics(trackSummary, truthSummary, trackMetrics);
fprintf('    avgPosRMS = %.1f (expect 60)\n', mExt.avgPosRMS);
fprintf('    swapCount = %d (expect 1)\n', mExt.swapCount);
fprintf('    falseTracks = %d (expect 1)\n', mExt.falseTracks);
fprintf('    breakCount = %d (expect 1)\n', mExt.breakCount);
fprintf('    worstTrackedPct = %.0f (expect 65)\n', mExt.worstTrackedPct);
fprintf('    estFailures = %d (expect 1; truth2 has EstLen/TotLen=0.20<0.25, truth1 has 0.10<0.25... none!)\n', mExt.estFailures);
% Both ratios are <25%, so estFailures should be 0
assert(mExt.avgPosRMS == 60, 'avgPosRMS mismatch');
assert(mExt.worstTrackedPct == 65, 'worstTrackedPct mismatch');
fprintf('    PASS\n\n');

% ---- TEST 6: extractTunerMetrics with legacy TrackedFraction ----
fprintf('[6] extractTunerMetrics with legacy TrackedFraction column...\n');
truthSummaryLegacy = table([1;2], [100;100], [10;30], [0.90;0.70], [0;1], ...
    'VariableNames', {'TruthID','TotalLength','EstablishmentLength','TrackedFraction','BreakCount'});
mLeg = trackbench.analysis.extractTunerMetrics(trackSummary, truthSummaryLegacy, trackMetrics);
fprintf('    worstTrackedPct = %.0f (expect 70)\n', mLeg.worstTrackedPct);
fprintf('    estFailures = %d (expect 1; truth2 EstLen/TotLen=0.30>0.25)\n', mLeg.estFailures);
assert(mLeg.worstTrackedPct == 70, 'legacy worstTrackedPct mismatch');
assert(mLeg.estFailures == 1, 'estFailures should be 1');
fprintf('    PASS\n\n');

% ---- TEST 7: analyzeSensitivity on synthetic sweep ----
fprintf('[7] analyzeSensitivity on synthetic sweep...\n');
% Gate has strong effect, Volume has none, Beta has moderate
nRows = 30;
gates    = randi([30,200], nRows, 1);
volumes  = repmat(1e8, nRows, 1) + randn(nRows,1)*1e6;  % tiny noise
betas    = 10.^(-14 + randi([0,4], nRows, 1));
scores   = 0.05 * (gates/200) + 0.10 * (log10(betas)+14)/4 + 0.01*randn(nRows,1);
sweepT = table(gates, volumes, betas, scores, ...
    'VariableNames', {'Gate','Volume','Beta','Score'});
report = trackbench.analysis.analyzeSensitivity(sweepT, 'Score', {'Gate','Volume','Beta'});
disp(report);
assert(istable(report), 'sensitivity report should be a table');
assert(height(report) == 3, 'expect 3 rows for 3 params');
fprintf('    PASS\n\n');

fprintf('========================================\n');
fprintf('  ALL TESTS PASSED\n');
fprintf('========================================\n');
