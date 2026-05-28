function results = autoTuneTracker(runName, trackerType, filterModel, opts)
%autoTuneTracker  Sweep tracker parameters against cached detections.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Loads cached detections from a previous run, then sweeps key tracker
%  parameters to find the combination that minimizes a 6-component
%  composite quality score. Detection generation is skipped entirely —
%  only the tracker algorithm runs, making each iteration fast.
%
%  USAGE
%    results = autoTuneTracker("range_rcs_test", "GNN")           % defaults to IMM
%    results = autoTuneTracker("range_rcs_test", "GNN", "IMM")
%    results = autoTuneTracker("range_rcs_test", "GNN", "CV")
%    results = autoTuneTracker("range_rcs_test", "JPDA", "IMM")
%    results = autoTuneTracker("range_rcs_test", "TOMHT", "CV")
%
%  INPUTS
%    runName     : name of run file (must have cached detections in cache/)
%    trackerType : 'GNN', 'JPDA', or 'TOMHT'
%    filterModel : 'IMM' (default) or 'CV'
%    opts        : (optional) struct with fields:
%                    .weights       — 6-element vector for the composite
%                                     score (see HOW SCORING WORKS). Must
%                                     sum to ~1.0. Default:
%                                       [0.25 0.15 0.10 0.10 0.25 0.15]
%                                     A 4-element legacy vector is padded
%                                     with default tracked/estFail weights
%                                     and a warning is issued.
%                    .saveBest      — save best config to JSON (default true)
%                    .compareModels — run best config with alternate filter model (default false)
%                    .filterModel   — LEGACY: still honored for back-compat if filterModel arg omitted
%
%  OUTPUTS
%    results : struct with fields:
%                .bestParams    — best parameter combination
%                .bestScore     — composite score (lower = better, 0-1 range)
%                .bestBreakdown — per-component score breakdown for the winner
%                .sweepTable    — table of ALL combinations with all metrics
%                .sensitivity   — per-parameter influence on score
%                .savedFile     — path to saved JSON config (if saveBest=true)
%                .modelCompare  — struct with CV vs IMM comparison (if compareModels=true)
%
%  HOW SCORING WORKS
%    Each combination is evaluated on 6 metrics, normalized to [0,1] and
%    weighted to produce a composite score in [0,1] (lower = better):
%
%      posErr   = avgPosRMS / maxRange         (accuracy)
%      swap     = swapCount / 5                (identity preservation)
%      false    = log1p(falseTracks)/log1p(20) (clutter discrimination)
%      break    = breakCount / 5               (track continuity)
%      tracked  = (100 - worstTrackedPct)/100  (worst-target coverage)
%      estFail  = estFailures / numTruths      (slow-establishment count)
%
%    Each weight is interpretable as "fraction of the total score that
%    this component contributes when it saturates". The default weights
%    sum to 1.0, so a perfect tracker gets 0 and a catastrophic one gets 1.
%
%    The tracked and estFail terms (new in v3.5) close the blind spot in
%    the old 4-term formula: a tracker that drops one of N targets used to
%    score *better* than a tracker that imperfectly held all N. Now it
%    can't — coverage is 25% of the default weight.
%
%  PASSES
%    Pass 1: Sweep tracker-specific parameters (gate, volume, beta, etc.)
%    Pass 2: Sweep filter parameters using best tracker params from Pass 1.
%            Adapts to filter model — CV skips IMM-only params (omega, transition prob).
%    Pass 3: (optional) Compare best result against alternate filter model (CV↔IMM).
%
%    After all passes, a sensitivity report ranks parameters by how much
%    they actually moved the score during the sweep.
%
%  See also: runSingleScenario, compareTrackers, buildTracker, runTracker,
%            trackbench.analysis.computeTunerScore,
%            trackbench.analysis.extractTunerMetrics,
%            trackbench.analysis.analyzeSensitivity

% --- Argument handling (backward-compatible) ---
% Old signature: autoTuneTracker(run, type, opts) where opts.filterModel is the model.
% New signature: autoTuneTracker(run, type, filterModel, opts).
% If the 3rd arg is a struct, treat it as the old opts form.
if nargin < 3
    filterModel = 'IMM';
    opts = struct();
elseif nargin == 3 && isstruct(filterModel)
    % Legacy call: autoTuneTracker(run, type, opts)
    opts = filterModel;
    if isfield(opts, 'filterModel')
        filterModel = opts.filterModel;
    else
        filterModel = 'IMM';
    end
elseif nargin == 3
    opts = struct();
end

if ~isfield(opts, 'weights');       opts.weights = [0.25, 0.15, 0.10, 0.10, 0.25, 0.15]; end
if ~isfield(opts, 'saveBest');      opts.saveBest = true; end
if ~isfield(opts, 'compareModels'); opts.compareModels = false; end

trackerType = upper(string(trackerType));
trackerType = char(trackerType);
fModel = upper(string(filterModel));
fModel = char(fModel);

if ~ismember(fModel, {'IMM','CV'})
    error('autoTuneTracker:badFilterModel', ...
        'filterModel must be ''IMM'' or ''CV'', got: %s', fModel);
end
if ~ismember(trackerType, {'GNN','JPDA','TOMHT'})
    error('autoTuneTracker:badTrackerType', ...
        'trackerType must be ''GNN'', ''JPDA'', or ''TOMHT'', got: %s', trackerType);
end

fprintf('\n');
fprintf('================================================================\n');
fprintf('  AUTO-TUNE TRACKER: %s + %s filter\n', trackerType, fModel);
fprintf('  Run: %s\n', runName);
fprintf('  %s\n', char(datetime('now','Format','yyyy-MM-dd HH:mm')));
fprintf('================================================================\n\n');

%% 1. Load cached detections
%  Deployed-safe path resolution: in compiled mode mfilename('fullpath')
%  points into the read-only CTF cache, not the writable per-user data
%  dir that mainMenu seeded and cd'd into. resolveRootFromThisFile falls
%  back to pwd when isdeployed (mainMenu has set pwd to
%  %LOCALAPPDATA%\RainyDay). The +trackbench package is baked into the
%  CTF, so addpath is a dev-mode-only call.
root = resolveRootFromThisFile();
if ~isdeployed
    addpath(genpath(fullfile(root, 'src')));
end

cacheFile = fullfile(root, 'cache', runName + ".mat");
if ~isfile(cacheFile)
    error('autoTuneTracker:noCachedData', ...
        'No cached detections found at %s.\nRun the scenario first: runSingleScenario("%s")', ...
        cacheFile, runName);
end

fprintf('[TUNE] Loading cached detections from %s...\n', cacheFile);
loaded = load(cacheFile);
if isfield(loaded, 'detections'); dataLog = loaded.detections;
elseif isfield(loaded, 'dataLog'); dataLog = loaded.dataLog;
else; error('Cache file has no detections field.');
end
fprintf('[TUNE] Loaded: %d scans, %d total detections\n', ...
    numel(dataLog.Time), sum(cellfun(@numel, dataLog.Detections)));

%% 2. Load config for filter params and globals
[~, config, sensors, ~] = trackbench.config.loadRunFile(runName);
trkGlobal = config.tracker_global;
trkFilter = config.filter_params;
pd = config.active_params.pd;
nSensors = countSensors(sensors);
% Derive maxRange from truth positions (matches runTracker's definition:
% max distance any target travels from the radar origin). Replaces the
% old hardcoded 60nm constant, which over-normalized short-range scenarios.
maxRange = computeMaxRangeFromTruth(dataLog);

fprintf('[TUNE] Filter: %s | Pd: %.2f | Sensors: %d | maxRange: %.0fm\n', ...
    fModel, pd, nSensors, maxRange);

%% 3. Define parameter sweep grid
grid = buildSweepGrid(trackerType);
nCombos = size(grid, 1);
fprintf('[TUNE] Sweep grid: %d combinations for %s\n\n', nCombos, trackerType);

%% 4. PASS 1 — Tracker parameter sweep
sweepResults = cell(nCombos, 1);
scoreObjs    = cell(nCombos, 1);
scores       = inf(nCombos, 1);
w = opts.weights;

fprintf('  PASS 1: TRACKER PARAMETER SWEEP\n');
fprintf('  ─────────────────────────────────────────────────────────────────────────────────────\n');
fprintf('   #  | Gate  Volume   Beta     Extra              |  RMS    Sw  FT  Brk Trkd EstF | Score\n');
fprintf('  ────+----------------------------------------------+-------------------------------+-------\n');

for i = 1:nCombos
    combo = grid(i, :);

    try
        % Build modified params
        [params, globalMod] = applyCombo(trackerType, combo, trkGlobal, config);

        % Build tracker
        tracker = trackbench.tracking.buildTracker(trackerType, fModel, params, ...
            globalMod, trkFilter, pd, nSensors);

        % Run tracker (no visuals, no truth plot)
        [trackSummary, truthSummary, trackMetrics, ~] = ...
            trackbench.tracking.runTracker(dataLog, tracker, false, false, false);

        % Extract metrics + compute score (new 6-component formula)
        m = trackbench.analysis.extractTunerMetrics(trackSummary, truthSummary, trackMetrics);
        s = trackbench.analysis.computeTunerScore(m, maxRange, w);

        scores(i)       = s.total;
        m.score         = s.total;
        m.combo         = combo;
        sweepResults{i} = m;
        scoreObjs{i}    = s;

        % Print row
        printSweepRow(i, combo, trackerType, m, s.total, ...
            (i > 1 && s.total <= min(scores(1:i-1))));

    catch ME
        fprintf('  %3d | %4.0f  %.0e  %.0e  %-17s | FAILED: %s\n', ...
            i, combo(1), combo(2), combo(3), formatExtra(trackerType, combo), ...
            ME.message);
        sweepResults{i} = makeFailedMetric(combo);
        scoreObjs{i}    = makeFailedScore(w);
    end

    % Reset tracker for next iteration
    clear tracker;
end

%% 5. Find best from Pass 1
[bestScore, bestIdx] = min(scores);
best     = sweepResults{bestIdx};
bestSObj = scoreObjs{bestIdx};

fprintf('\n');
fprintf('================================================================\n');
fprintf('  PASS 1 WINNER (#%d of %d)\n', bestIdx, nCombos);
fprintf('================================================================\n');
printScoreBreakdown(bestSObj);
fprintf('  Raw metrics:\n');
fprintf('    posRMS         : %.1f m\n', best.avgPosRMS);
fprintf('    Swaps          : %d\n', best.swapCount);
fprintf('    False tracks   : %d\n', best.falseTracks);
fprintf('    Breaks         : %d\n', best.breakCount);
fprintf('    Worst-tracked  : %s\n', fmtPct(best.worstTrackedPct));
fprintf('    Est failures   : %d of %d truths\n', best.estFailures, best.numTruths);
fprintf('  Parameters:\n');
fprintf('    Gate           : %.0f\n', best.combo(1));
fprintf('    Volume         : %.0e\n', best.combo(2));
fprintf('    Beta           : %.0e\n', best.combo(3));
printExtraParams(trackerType, best.combo);

%% 5b. PASS 2 — Filter parameter sweep (using best tracker params)
fprintf('\n');
fprintf('================================================================\n');
fprintf('  PASS 2: FILTER PARAMETER SWEEP (%s)\n', fModel);
fprintf('  (using best tracker params from Pass 1)\n');
if strcmpi(fModel, 'CV')
    fprintf('  NOTE: CV mode — OmegaDot and TransProb are N/A (IMM-only)\n');
end
fprintf('================================================================\n\n');

filterGrid     = buildFilterGrid(trkFilter, fModel);
nFilterCombos  = size(filterGrid, 1);
filterScores   = inf(nFilterCombos, 1);
filterResults  = cell(nFilterCombos, 1);
filterSObjs    = cell(nFilterCombos, 1);
bestFilterParams = trkFilter;  % default: keep original

if strcmpi(fModel, 'IMM')
    fprintf('   #  | AccH AccV  OmgD  TrnP  Spd |  RMS    Sw  FT  Brk Trkd EstF | Score\n');
    fprintf('  ────+----------------------------+-------------------------------+-------\n');
else
    fprintf('   #  | AccH AccV  Spd |  RMS    Sw  FT  Brk Trkd EstF | Score\n');
    fprintf('  ────+----------------+-------------------------------+-------\n');
end

for i = 1:nFilterCombos
    fCombo = filterGrid(i,:);
    modFilter = trkFilter;
    modFilter.scale_accel_horz   = fCombo(1);
    modFilter.scale_accel_vert   = fCombo(2);
    modFilter.scale_omega_dot    = fCombo(3);
    modFilter.imm_transition_prob = fCombo(4);
    modFilter.init_speed_kmh     = fCombo(5);

    try
        [params2, globalMod2] = applyCombo(trackerType, best.combo, trkGlobal, config);
        tracker2 = trackbench.tracking.buildTracker(trackerType, fModel, params2, ...
            globalMod2, modFilter, pd, nSensors);
        [ts2, trs2, tm2, ~] = trackbench.tracking.runTracker(dataLog, tracker2, false, false, false);

        m2 = trackbench.analysis.extractTunerMetrics(ts2, trs2, tm2);
        s2 = trackbench.analysis.computeTunerScore(m2, maxRange, w);

        filterScores(i)   = s2.total;
        m2.filterCombo    = fCombo;
        m2.score          = s2.total;
        filterResults{i}  = m2;
        filterSObjs{i}    = s2;

        printFilterRow(i, fCombo, fModel, m2, s2.total, ...
            (i > 1 && s2.total <= min(filterScores(1:i-1))));

        clear tracker2;
    catch ME
        if strcmpi(fModel, 'IMM')
            fprintf('  %3d | %4.0f %4.0f %5.0f %.3f %4.0f | FAILED: %s\n', ...
                i, fCombo(1), fCombo(2), fCombo(3), fCombo(4), fCombo(5), ME.message);
        else
            fprintf('  %3d | %4.0f %4.0f %4.0f | FAILED: %s\n', ...
                i, fCombo(1), fCombo(2), fCombo(5), ME.message);
        end
        filterResults{i} = makeFailedMetric([]);
        filterResults{i}.filterCombo = fCombo;
        filterSObjs{i}   = makeFailedScore(w);
    end
end

[bestFilterScore, bestFilterIdx] = min(filterScores);
if bestFilterScore < bestScore
    bestFilter   = filterResults{bestFilterIdx};
    bestFilterS  = filterSObjs{bestFilterIdx};
    bestFilterParams = trkFilter;
    bestFilterParams.scale_accel_horz    = bestFilter.filterCombo(1);
    bestFilterParams.scale_accel_vert    = bestFilter.filterCombo(2);
    bestFilterParams.scale_omega_dot     = bestFilter.filterCombo(3);
    bestFilterParams.imm_transition_prob = bestFilter.filterCombo(4);
    bestFilterParams.init_speed_kmh      = bestFilter.filterCombo(5);

    fprintf('\n  PASS 2 IMPROVED: %.3f → %.3f (posRMS: %.0fm → %.0fm)\n', ...
        bestScore, bestFilterScore, best.avgPosRMS, bestFilter.avgPosRMS);
    if strcmpi(fModel, 'IMM')
        fprintf('  Best filter: accelH=%.0f accelV=%.0f omega=%.0f trans=%.3f speed=%.0f\n', ...
            bestFilter.filterCombo(1), bestFilter.filterCombo(2), bestFilter.filterCombo(3), ...
            bestFilter.filterCombo(4), bestFilter.filterCombo(5));
    else
        fprintf('  Best filter: accelH=%.0f accelV=%.0f speed=%.0f\n', ...
            bestFilter.filterCombo(1), bestFilter.filterCombo(2), bestFilter.filterCombo(5));
    end

    % Carry forward all best metrics including the new coverage terms
    best.avgPosRMS       = bestFilter.avgPosRMS;
    best.swapCount       = bestFilter.swapCount;
    best.falseTracks     = bestFilter.falseTracks;
    best.breakCount      = bestFilter.breakCount;
    best.worstTrackedPct = bestFilter.worstTrackedPct;
    best.avgTrackedPct   = bestFilter.avgTrackedPct;
    best.estFailures     = bestFilter.estFailures;
    best.numTruths       = bestFilter.numTruths;
    best.score           = bestFilterScore;
    best.filterCombo     = bestFilter.filterCombo;
    bestScore            = bestFilterScore;
    bestSObj             = bestFilterS;
else
    fprintf('\n  PASS 2: No improvement over Pass 1 (best filter score=%.3f vs tracker score=%.3f)\n', ...
        bestFilterScore, bestScore);
    best.filterCombo = [trkFilter.scale_accel_horz, trkFilter.scale_accel_vert, ...
        trkFilter.scale_omega_dot, trkFilter.imm_transition_prob, trkFilter.init_speed_kmh];
end

%% 5c. PASS 3 — CV vs IMM comparison (optional)
modelCompare = struct();
if opts.compareModels
    altModel = ternary(strcmpi(fModel, 'IMM'), 'CV', 'IMM');

    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('  PASS 3: MODEL COMPARISON (%s vs %s)\n', fModel, altModel);
    fprintf('  (best tracker + filter params, alternate motion model)\n');
    fprintf('================================================================\n\n');

    try
        [params3, globalMod3] = applyCombo(trackerType, best.combo, trkGlobal, config);
        tracker3 = trackbench.tracking.buildTracker(trackerType, altModel, params3, ...
            globalMod3, bestFilterParams, pd, nSensors);
        [ts3, trs3, tm3, ~] = trackbench.tracking.runTracker(dataLog, tracker3, false, false, false);

        m3 = trackbench.analysis.extractTunerMetrics(ts3, trs3, tm3);
        s3 = trackbench.analysis.computeTunerScore(m3, maxRange, w);
        altScore = s3.total;

        modelCompare.primary.model       = fModel;
        modelCompare.primary.posRMS      = best.avgPosRMS;
        modelCompare.primary.swaps       = best.swapCount;
        modelCompare.primary.falseTracks = best.falseTracks;
        modelCompare.primary.breaks      = best.breakCount;
        modelCompare.primary.worstTrackedPct = best.worstTrackedPct;
        modelCompare.primary.estFailures = best.estFailures;
        modelCompare.primary.numTruths   = best.numTruths;
        modelCompare.primary.score       = bestScore;

        modelCompare.alternate.model       = altModel;
        modelCompare.alternate.posRMS      = m3.avgPosRMS;
        modelCompare.alternate.swaps       = m3.swapCount;
        modelCompare.alternate.falseTracks = m3.falseTracks;
        modelCompare.alternate.breaks      = m3.breakCount;
        modelCompare.alternate.worstTrackedPct = m3.worstTrackedPct;
        modelCompare.alternate.estFailures = m3.estFailures;
        modelCompare.alternate.numTruths   = m3.numTruths;
        modelCompare.alternate.score       = altScore;

        if altScore < bestScore
            modelCompare.winner          = altModel;
            modelCompare.improvement_pct = (bestScore - altScore) / max(bestScore, eps) * 100;
        else
            modelCompare.winner          = fModel;
            modelCompare.improvement_pct = 0;
        end

        fprintf('  %-6s | RMS:%7.0fm  Sw:%3d  FT:%3d  Brk:%3d  Trkd:%s  EstF:%d/%d  Score:%.3f\n', ...
            fModel, best.avgPosRMS, best.swapCount, best.falseTracks, best.breakCount, ...
            fmtPct(best.worstTrackedPct), best.estFailures, best.numTruths, bestScore);
        fprintf('  %-6s | RMS:%7.0fm  Sw:%3d  FT:%3d  Brk:%3d  Trkd:%s  EstF:%d/%d  Score:%.3f\n', ...
            altModel, m3.avgPosRMS, m3.swapCount, m3.falseTracks, m3.breakCount, ...
            fmtPct(m3.worstTrackedPct), m3.estFailures, m3.numTruths, altScore);
        fprintf('  ──────\n');

        if altScore < bestScore
            fprintf('  ⚡ %s WINS (%.1f%% better). Consider re-tuning with filterModel=''%s''\n', ...
                altModel, modelCompare.improvement_pct, altModel);
        elseif altScore == bestScore
            fprintf('  ≈ TIE — both models perform equally on this scenario.\n');
        else
            fprintf('  ✓ %s is the better model for this scenario.\n', fModel);
        end

        clear tracker3;
    catch ME
        fprintf('  %s comparison FAILED: %s\n', altModel, ME.message);
        fprintf('  (This can happen if filter params tuned for %s are incompatible with %s)\n', ...
            fModel, altModel);
        modelCompare.error = ME.message;
    end
else
    fprintf('\n  (Model comparison skipped — set opts.compareModels=true to enable)\n');
end

%% 6. Build results table (now with all 6 metric columns)
sweepTable = buildSweepTable(sweepResults, trackerType);

%% 6b. Sensitivity analysis — which params actually moved the score?
paramCols = sweepTableParamCols(trackerType);
try
    sensitivity = trackbench.analysis.analyzeSensitivity(sweepTable, 'Score', paramCols);
catch ME
    warning('autoTuneTracker:sensitivityFailed', ...
        'Sensitivity analysis failed: %s', ME.message);
    sensitivity = table();
end

fprintf('\n');
fprintf('================================================================\n');
fprintf('  SENSITIVITY ANALYSIS\n');
fprintf('  (which parameters actually moved the score during the sweep)\n');
fprintf('================================================================\n');
if isempty(sensitivity) || height(sensitivity) == 0
    fprintf('  (insufficient data — need at least 4 valid sweep rows)\n');
else
    fprintf('  %-12s %12s %12s %10s   %s\n', ...
        'Param', 'Correlation', 'Influence', 'Levels', 'Verdict');
    fprintf('  %-12s %12s %12s %10s   %s\n', ...
        '─────', '───────────', '─────────', '──────', '───────');
    for k = 1:height(sensitivity)
        corr = sensitivity.Correlation(k);
        infl = sensitivity.Influence(k);
        corrStr = ternary(isnan(corr), '   --', sprintf('%+.3f', corr));
        inflStr = ternary(isnan(infl), '   --', sprintf('%.3f', infl));
        fprintf('  %-12s %12s %12s %10d   %s\n', ...
            char(sensitivity.Param(k)), corrStr, inflStr, ...
            sensitivity.NumLevels(k), char(sensitivity.Verdict(k)));
    end
end

%% 7. Save best config
savedFile = '';
if opts.saveBest
    savedFile = saveBestConfig(root, trackerType, fModel, best, bestSObj, ...
        runName, config, bestFilterParams);
    fprintf('\n[TUNE] Best config saved to: %s\n', savedFile);
end

%% 8. Package output
results.bestParams     = best;
results.bestScore      = bestScore;
results.bestBreakdown  = bestSObj;
results.bestIdx        = bestIdx;
results.sweepTable     = sweepTable;
results.sensitivity    = sensitivity;
results.savedFile      = savedFile;
results.runName        = runName;
results.trackerType    = trackerType;
results.filterModel    = fModel;
results.modelCompare   = modelCompare;
results.weights        = w;

fprintf('\n  Top 5 configurations:\n');
disp(sweepTable(1:min(5,height(sweepTable)), :));

fprintf('================================================================\n');
fprintf('  AUTO-TUNE COMPLETE: %s + %s on %s\n', trackerType, fModel, runName);
fprintf('  Final score: %.3f (lower is better, 0=perfect, 1=catastrophic)\n', bestScore);
fprintf('================================================================\n\n');

end


%% ========================================================================
%  SWEEP GRID BUILDERS
%% ========================================================================
function grid = buildSweepGrid(trackerType)
%buildSweepGrid  Create parameter combinations to test.
%  Each row = [gate, volume, beta, extra1, extra2, ...]
%  Grid is kept manageable (~30-40 combinations).

    % Shared parameter ranges
    volumes = [1e7, 1e8, 1e9];
    betas   = [1e-14, 1e-12, 1e-10];

    switch upper(trackerType)
        case 'GNN'
            gates = [30, 60, 100, 200];
            % Extra: confirm_threshold, delete_threshold
            confirms = [8, 15, 25];
            deletes  = [-3, -5, -7];
            grid = combineParams(gates, volumes, betas, confirms, deletes);

        case 'JPDA'
            gates = [30, 60, 100, 200];
            % Extra: jpda_confirm_prob, jpda_delete_prob
            confirms = [0.90, 0.95, 0.99];
            deletes  = [0.02, 0.05, 0.10];
            grid = combineParams(gates, volumes, betas, confirms, deletes);

        case 'TOMHT'
            % TOMHT uses tomht_threshold_multiplier = [0.2, 1, 1] on the gate,
            % so gate=30 becomes [6, 30, 30] which is too tight to initialize
            % tracks. Start at 50 to ensure the inner threshold (0.2*gate=10)
            % is viable.
            gates = [50, 75, 100, 200];
            % Extra: max_branches, confirm_threshold
            branches = [3, 5, 8];
            confirms = [8, 15, 25];
            grid = combineParams(gates, volumes, betas, branches, confirms);

        otherwise
            error('Unknown tracker type: %s', trackerType);
    end
end

function grid = combineParams(p1, p2, p3, p4, p5)
%combineParams  Smart grid: sweep key combos, not full Cartesian product.
%  Strategy: vary 2 params at a time with others at baseline.

    % Baseline: middle of each range
    b1 = p1(ceil(end/2));
    b2 = p2(ceil(end/2));
    b3 = p3(ceil(end/2));
    b4 = p4(ceil(end/2));
    b5 = p5(ceil(end/2));

    grid = [];

    % Sweep gate × volume (most impactful pair)
    for g = p1
        for v = p2
            grid(end+1,:) = [g, v, b3, b4, b5]; %#ok<AGROW>
        end
    end

    % Sweep beta × extra params
    for b = p3
        for e4 = p4
            grid(end+1,:) = [b1, b2, b, e4, b5]; %#ok<AGROW>
        end
    end

    % Sweep extra1 × extra2
    for e4 = p4
        for e5 = p5
            grid(end+1,:) = [b1, b2, b3, e4, e5]; %#ok<AGROW>
        end
    end

    % Full corners (extremes)
    grid(end+1,:) = [p1(1),   p2(1),   p3(1),   p4(1),   p5(1)];
    grid(end+1,:) = [p1(end), p2(end), p3(end), p4(end), p5(end)];
    grid(end+1,:) = [p1(1),   p2(end), p3(1),   p4(end), p5(1)];
    grid(end+1,:) = [p1(end), p2(1),   p3(end), p4(1),   p5(end)];

    % Deduplicate
    grid = unique(grid, 'rows');
end


function grid = buildFilterGrid(baseFilter, filterModel)
%buildFilterGrid  Create filter parameter combinations to sweep.
%  Each row = [accelH, accelV, omegaDot, transProb, initSpeed]
%  Centered around the baseline values with variations above and below.
%
%  When filterModel='CV', omegaDot and transProb are fixed at baseline
%  (they are IMM-only parameters and have no effect on CV filters).
%  This reduces the grid size and avoids wasting sweep iterations.

    if nargin < 2; filterModel = 'IMM'; end

    % Baseline values from current config
    bH = baseFilter.scale_accel_horz;
    bV = baseFilter.scale_accel_vert;
    bO = baseFilter.scale_omega_dot;
    bT = baseFilter.imm_transition_prob;
    bS = baseFilter.init_speed_kmh;

    % Ranges: 0.5x, 1x, 2x, 4x of baseline for accel params
    accH  = unique([max(5, bH*0.5), bH, bH*2, bH*4]);
    accV  = unique([max(3, bV*0.5), bV, bV*2, bV*4]);
    speed = unique([max(200, bS*0.5), bS, bS*1.5]);

    if strcmpi(filterModel, 'IMM')
        % IMM mode: sweep all 5 parameters
        omega = unique([max(5, bO*0.5), bO, bO*2, bO*4]);
        trans = [0.90, 0.95, 0.97, 0.99];
    else
        % CV mode: omegaDot and transProb have no effect — fix at baseline
        omega = bO;
        trans = bT;
    end

    grid = [];

    % Baseline first
    grid(end+1,:) = [bH, bV, bO, bT, bS];

    % Sweep accelH (dominant for maneuvering targets)
    for h = accH
        grid(end+1,:) = [h, bV, bO, bT, bS]; %#ok<AGROW>
    end

    % Sweep accelV
    for v = accV
        grid(end+1,:) = [bH, v, bO, bT, bS]; %#ok<AGROW>
    end

    % Sweep accelH x accelV together (process noise)
    for h = accH
        for v = accV
            grid(end+1,:) = [h, v, bO, bT, bS]; %#ok<AGROW>
        end
    end

    % Sweep omega (turn rate noise) — only has effect for IMM
    for o = omega
        grid(end+1,:) = [bH, bV, o, bT, bS]; %#ok<AGROW>
    end

    % Sweep IMM transition prob — only has effect for IMM
    for t = trans
        grid(end+1,:) = [bH, bV, bO, t, bS]; %#ok<AGROW>
    end

    % Sweep init speed
    for s = speed
        grid(end+1,:) = [bH, bV, bO, bT, s]; %#ok<AGROW>
    end

    % Corners: high noise + fast switching vs low noise + slow switching
    grid(end+1,:) = [accH(end), accV(end), omega(end), trans(1),   speed(end)];
    grid(end+1,:) = [accH(1),   accV(1),   omega(1),   trans(end), speed(1)];

    grid = unique(grid, 'rows');
end


%% ========================================================================
%  PARAMETER APPLICATION
%% ========================================================================
function [params, globalMod] = applyCombo(trackerType, combo, globalBase, config)
%applyCombo  Map a combo row into buildTracker's params/globalParams format.

    gate = combo(1);
    vol  = combo(2);
    bet  = combo(3);

    globalMod = globalBase;
    globalMod.volume = vol;
    globalMod.beta = bet;

    % Start with base params from config
    params = config.active_params;

    switch upper(trackerType)
        case 'GNN'
            params.gate = gate;
            params.confirm_threshold = combo(4);
            params.delete_threshold = combo(5);
            params.far_gnn = 1e-6;

        case 'JPDA'
            params.gate_jpda = gate;
            params.jpda_confirm_prob = combo(4);
            params.jpda_delete_prob = combo(5);
            params.far_jpda = 1e-6;
            params.beta_jpda = bet;
            params.time_tolerance_jpda = 0.05;
            params.num_tracks_jpda = globalBase.max_num_tracks;

        case 'TOMHT'
            params.gate = gate;
            params.max_branches = combo(4);
            params.confirm_threshold = combo(5);
            params.delete_threshold = -5;
            params.far_mht = 1e-6;
            params.tomht_threshold_multiplier = [0.2, 1, 1];
    end
end


%% ========================================================================
%  DISPLAY HELPERS
%% ========================================================================
function printSweepRow(idx, combo, trackerType, m, score, isNewBest)
%printSweepRow  Pass-1 row with all 6 metrics visible.
    extraStr = formatExtra(trackerType, combo);
    star     = ternary(isNewBest, ' *', '');

    rmsStr   = fmtMeters(m.avgPosRMS);
    trkStr   = fmtPctNarrow(m.worstTrackedPct);
    estStr   = fmtEstFail(m.estFailures, m.numTruths);

    fprintf('  %3d | %4.0f  %.0e  %.0e  %-17s | %5s %3d %3d %3d %4s %4s | %.3f%s\n', ...
        idx, combo(1), combo(2), combo(3), extraStr, ...
        rmsStr, m.swapCount, m.falseTracks, m.breakCount, ...
        trkStr, estStr, score, star);
end

function printFilterRow(idx, fCombo, fModel, m, score, isNewBest)
%printFilterRow  Pass-2 row with all 6 metrics visible.
    star   = ternary(isNewBest, ' *', '');
    rmsStr = fmtMeters(m.avgPosRMS);
    trkStr = fmtPctNarrow(m.worstTrackedPct);
    estStr = fmtEstFail(m.estFailures, m.numTruths);

    if strcmpi(fModel, 'IMM')
        fprintf('  %3d | %4.0f %4.0f %5.0f %.3f %4.0f | %5s %3d %3d %3d %4s %4s | %.3f%s\n', ...
            idx, fCombo(1), fCombo(2), fCombo(3), fCombo(4), fCombo(5), ...
            rmsStr, m.swapCount, m.falseTracks, m.breakCount, ...
            trkStr, estStr, score, star);
    else
        fprintf('  %3d | %4.0f %4.0f %4.0f | %5s %3d %3d %3d %4s %4s | %.3f%s\n', ...
            idx, fCombo(1), fCombo(2), fCombo(5), ...
            rmsStr, m.swapCount, m.falseTracks, m.breakCount, ...
            trkStr, estStr, score, star);
    end
end

function s = fmtMeters(r)
%fmtMeters  Format posRMS for fixed-width column. Inf → "  Inf".
    if ~isfinite(r)
        s = '  Inf';
    elseif r >= 99999
        s = '99999';
    else
        s = sprintf('%5.0f', r);
    end
end

function s = fmtPctNarrow(pct)
%fmtPctNarrow  Worst-tracked percent in 4 chars, e.g. " 92%" or " --%".
    if isnan(pct)
        s = ' --%';
    else
        s = sprintf('%3.0f%%', max(0, min(100, pct)));
    end
end

function s = fmtPct(pct)
%fmtPct  Worst-tracked percent in friendly format, no fixed width.
    if isnan(pct)
        s = 'N/A';
    else
        s = sprintf('%.0f%%', max(0, min(100, pct)));
    end
end

function s = fmtEstFail(est, total)
%fmtEstFail  Establishment-failure ratio in 4 chars, e.g. "0/2" or "--".
    if total <= 0
        s = '  --';
    else
        s = sprintf('%d/%d', est, total);
    end
end

function s = formatExtra(trackerType, combo)
    switch upper(trackerType)
        case 'GNN'
            s = sprintf('conf=%d del=%d', combo(4), combo(5));
        case 'JPDA'
            s = sprintf('cP=%.2f dP=%.2f', combo(4), combo(5));
        case 'TOMHT'
            s = sprintf('br=%d conf=%d', combo(4), combo(5));
        otherwise
            s = '';
    end
end

function printExtraParams(trackerType, combo)
    switch upper(trackerType)
        case 'GNN'
            fprintf('    Confirm        : %d\n', combo(4));
            fprintf('    Delete         : %d\n', combo(5));
        case 'JPDA'
            fprintf('    Confirm prob   : %.2f\n', combo(4));
            fprintf('    Delete prob    : %.2f\n', combo(5));
        case 'TOMHT'
            fprintf('    Max branches   : %d\n', combo(4));
            fprintf('    Confirm        : %d\n', combo(5));
    end
end

function printScoreBreakdown(s)
%printScoreBreakdown  Show how each component contributed to the total.
    fprintf('  Score: %.3f  (lower is better, 0 = perfect)\n', s.total);
    fprintf('  Component contributions (raw × weight = weighted):\n');
    labels = {'posErr ', 'swap   ', 'false  ', 'break  ', 'tracked', 'estFail'};
    raws   = [s.posErr, s.swap, s.false, s.break, s.tracked, s.estFail];
    for k = 1:numel(labels)
        bar  = repmat('█', 1, max(0, round(s.weighted(k) * 40)));
        fprintf('    %s  %.3f × %.2f = %.3f  %s\n', ...
            labels{k}, raws(k), s.weights(k), s.weighted(k), bar);
    end
end

function out = ternary(cond, a, b)
    if cond; out = a; else; out = b; end
end


%% ========================================================================
%  RESULTS TABLE
%% ========================================================================
function T = buildSweepTable(sweepResults, trackerType)
%buildSweepTable  Aggregate per-combo metrics into a sorted table.
%  Includes the new tracked-coverage and estFail columns. Column names
%  for the two tracker-specific Extra slots are tracker-aware so the
%  sensitivity report below names real parameters, not 'Extra1/Extra2'.
    n = numel(sweepResults);
    data = zeros(n, 10);
    for i = 1:n
        r = sweepResults{i};
        data(i,:) = [r.combo(1), r.combo(2), r.combo(3), r.combo(4), r.combo(5), ...
                     r.avgPosRMS, r.swapCount, r.falseTracks, r.breakCount, r.score];
        % Coverage stats stay in the per-row struct (sweepResults) for
        % callers who want them. The Score column already reflects
        % coverage via the new scoring formula.
    end
    [extra1Name, extra2Name] = extraColNames(trackerType);
    T = array2table(data, 'VariableNames', ...
        {'Gate','Volume','Beta', extra1Name, extra2Name, ...
         'posRMS','Swaps','FalseTracks','Breaks','Score'});
    T = sortrows(T, 'Score');
end

function cols = sweepTableParamCols(trackerType)
%sweepTableParamCols  Column names eligible for sensitivity analysis.
%  Returns the same tracker-specific names buildSweepTable used so
%  analyzeSensitivity can look them up correctly.
    [extra1Name, extra2Name] = extraColNames(trackerType);
    cols = {'Gate','Volume','Beta', extra1Name, extra2Name};
end

function [n1, n2] = extraColNames(trackerType)
%extraColNames  Map abstract Extra1/Extra2 to tracker-specific column names.
    switch upper(trackerType)
        case 'GNN'
            n1 = 'ConfThresh';  n2 = 'DelThresh';
        case 'JPDA'
            n1 = 'ConfProb';    n2 = 'DelProb';
        case 'TOMHT'
            n1 = 'MaxBranches'; n2 = 'ConfThresh';
        otherwise
            n1 = 'Extra1';      n2 = 'Extra2';
    end
end


%% ========================================================================
%  FAILURE PLACEHOLDERS
%% ========================================================================
function m = makeFailedMetric(combo)
    m.avgPosRMS       = Inf;
    m.swapCount       = 99;
    m.falseTracks     = 99;
    m.breakCount      = 99;
    m.worstTrackedPct = 0;
    m.avgTrackedPct   = 0;
    m.estFailures     = 99;
    m.numTruths       = 1;
    m.score           = Inf;
    m.combo           = combo;
end

function s = makeFailedScore(w)
    s.posErr   = 1;
    s.swap     = 1;
    s.false    = 1;
    s.break    = 1;
    s.tracked  = 1;
    s.estFail  = 1;
    s.weighted = w(:)';
    s.weights  = w(:)';
    s.total    = Inf;
end


%% ========================================================================
%  SAVE BEST CONFIG
%% ========================================================================
function savedFile = saveBestConfig(root, trackerType, filterModel, best, bestSObj, runName, config, filterParams)
%saveBestConfig  Write the best params as a tracker JSON config file.

    combo = best.combo;

    % Filter override block (included in all tracker types)
    filterOverride = struct();
    filterOverride.init_speed_kmh       = filterParams.init_speed_kmh;
    filterOverride.imm_transition_prob  = filterParams.imm_transition_prob;
    filterOverride.scale_accel_horz     = filterParams.scale_accel_horz;
    filterOverride.scale_accel_vert     = filterParams.scale_accel_vert;
    filterOverride.scale_omega_dot      = filterParams.scale_omega_dot;

    % Common TUNING_NOTES with the new 6-component breakdown
    tuningNotes = struct( ...
        'method', 'autoTuneTracker 2-pass sweep + sensitivity (v3.5)', ...
        'filter_model', filterModel, ...
        'run', char(runName), ...
        'score', best.score, ...
        'score_components', struct( ...
            'posErr',   bestSObj.posErr, ...
            'swap',     bestSObj.swap, ...
            'false',    bestSObj.false, ...
            'break',    bestSObj.break, ...
            'tracked',  bestSObj.tracked, ...
            'estFail',  bestSObj.estFail), ...
        'weights', bestSObj.weights, ...
        'raw_metrics', struct( ...
            'posRMS',          best.avgPosRMS, ...
            'swaps',           best.swapCount, ...
            'falseTracks',     best.falseTracks, ...
            'breaks',          best.breakCount, ...
            'worstTrackedPct', best.worstTrackedPct, ...
            'avgTrackedPct',   best.avgTrackedPct, ...
            'estFailures',     best.estFailures, ...
            'numTruths',       best.numTruths), ...
        'date', char(datetime('now','Format','yyyy-MM-dd HH:mm')));

    switch upper(trackerType)
        case 'GNN'
            cfg = struct();
            cfg.description = sprintf('AUTO-TUNED GNN+%s for %s (score=%.3f, posRMS=%.0fm, trkd=%s)', ...
                filterModel, runName, best.score, best.avgPosRMS, fmtPct(best.worstTrackedPct));
            cfg.tracker_type = 'GNN';
            cfg.filter_model = filterModel;
            cfg.volume = combo(2);
            cfg.beta = combo(3);
            cfg.params = struct();
            cfg.params.gate = combo(1);
            cfg.params.far_gnn = 1e-6;
            cfg.params.confirm_threshold = combo(4);
            cfg.params.delete_threshold = combo(5);
            % Include JPDA/TOMHT defaults so file works for any tracker
            cfg.params.gate_jpda = combo(1);
            cfg.params.far_jpda = 1e-6;
            cfg.params.far_mht = 1e-6;
            cfg.params.beta_jpda = combo(3);
            cfg.params.time_tolerance_jpda = 0.05;
            cfg.params.num_tracks_jpda = 500;
            cfg.params.tomht_threshold_multiplier = [0.2, 1, 1];
            cfg.params.max_branches = 5;
            cfg.filter = filterOverride;
            cfg.TUNING_NOTES = tuningNotes;

        case 'JPDA'
            cfg = struct();
            cfg.description = sprintf('AUTO-TUNED JPDA+%s for %s (score=%.3f, posRMS=%.0fm, trkd=%s)', ...
                filterModel, runName, best.score, best.avgPosRMS, fmtPct(best.worstTrackedPct));
            cfg.tracker_type = 'JPDA';
            cfg.filter_model = filterModel;
            cfg.volume = combo(2);
            cfg.beta = combo(3);
            cfg.params = struct();
            cfg.params.gate = combo(1);
            cfg.params.gate_jpda = combo(1);
            cfg.params.far_gnn = 1e-6;
            cfg.params.far_jpda = 1e-6;
            cfg.params.far_mht = 1e-6;
            cfg.params.beta_jpda = combo(3);
            cfg.params.jpda_confirm_prob = combo(4);
            cfg.params.jpda_delete_prob = combo(5);
            cfg.params.time_tolerance_jpda = 0.05;
            cfg.params.num_tracks_jpda = 500;
            cfg.params.confirm_threshold = 20;
            cfg.params.delete_threshold = -5;
            cfg.params.tomht_threshold_multiplier = [0.2, 1, 1];
            cfg.params.max_branches = 5;
            cfg.filter = filterOverride;
            cfg.TUNING_NOTES = tuningNotes;

        case 'TOMHT'
            cfg = struct();
            cfg.description = sprintf('AUTO-TUNED TOMHT+%s for %s (score=%.3f, posRMS=%.0fm, trkd=%s)', ...
                filterModel, runName, best.score, best.avgPosRMS, fmtPct(best.worstTrackedPct));
            cfg.tracker_type = 'TOMHT';
            cfg.filter_model = filterModel;
            cfg.volume = combo(2);
            cfg.beta = combo(3);
            cfg.params = struct();
            cfg.params.gate = combo(1);
            cfg.params.gate_jpda = combo(1);
            cfg.params.far_gnn = 1e-6;
            cfg.params.far_jpda = 1e-6;
            cfg.params.far_mht = 1e-6;
            cfg.params.beta_jpda = combo(3);
            cfg.params.time_tolerance_jpda = 0.05;
            cfg.params.num_tracks_jpda = 500;
            cfg.params.max_branches = combo(4);
            cfg.params.confirm_threshold = combo(5);
            cfg.params.delete_threshold = -5;
            cfg.params.tomht_threshold_multiplier = [0.2, 1, 1];
            cfg.filter = filterOverride;
            cfg.TUNING_NOTES = tuningNotes;
    end

    % Save to config/trackers/autotuned/<run_name>/<TYPE>_<MODEL>.json
    runFolder = strrep(char(runName), '/', '_');
    outDir = fullfile(root, 'config', 'trackers', 'autotuned', runFolder);
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    savedFile = fullfile(outDir, sprintf('%s_%s.json', upper(trackerType), filterModel));

    jsonStr = jsonencode(cfg);
    % Pretty-print JSON
    jsonStr = strrep(jsonStr, ',', sprintf(',\n    '));
    jsonStr = strrep(jsonStr, '{', sprintf('{\n    '));
    jsonStr = strrep(jsonStr, '}', sprintf('\n}'));

    fid = fopen(savedFile, 'w');
    fprintf(fid, '%s', jsonStr);
    fclose(fid);
end


%% ========================================================================
function n = countSensors(sensors)
    n = 0;
    if isempty(sensors) || ~isstruct(sensors); n = 1; return; end
    pNames = fieldnames(sensors);
    for i = 1:numel(pNames)
        n = n + numel(sensors.(pNames{i}));
    end
    n = max(n, 1);
end


%% ========================================================================
function r = computeMaxRangeFromTruth(dataLog)
%computeMaxRangeFromTruth  Match runTracker's maxRange definition.
%  Returns the maximum distance any target travels from the origin (the
%  radar reference frame), matching the maxRange computed inside
%  runTracker for its adaptive assignment/divergence thresholds. Falls
%  back to 60nm PSR range when truth data is missing.
    allPos = [];
    if isfield(dataLog, 'Truth') && ~isempty(dataLog.Truth)
        for ss = 1:size(dataLog.Truth, 2)
            tgts = dataLog.Truth(:, ss);
            for kk = 1:numel(tgts)
                allPos = [allPos; tgts(kk).Position(:)']; %#ok<AGROW>
            end
        end
    end
    if isempty(allPos)
        r = 111120;
    else
        r = max(vecnorm(allPos, 2, 2));
    end
end


%% ========================================================================
function root = resolveRootFromThisFile()
%resolveRootFromThisFile  Deployed-safe project root resolution.
%  In compiled mode (isdeployed), mfilename('fullpath') points into the
%  CTF cache (read-only), not the per-user data dir that mainMenu seeded
%  and cd'd into. pwd is the source of truth in that case. In dev mode,
%  we resolve from this file's location (scripts/ folder, two levels up).
    if isdeployed
        root = pwd;
    else
        thisFile = mfilename('fullpath');
        root = fileparts(fileparts(thisFile));
    end
end
