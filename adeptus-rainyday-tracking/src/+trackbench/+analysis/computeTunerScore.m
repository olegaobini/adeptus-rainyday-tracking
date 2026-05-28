function s = computeTunerScore(m, maxRange, w)
%computeTunerScore  6-component normalized tracker quality score.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   USAGE
%     s = trackbench.analysis.computeTunerScore(m, maxRange)
%     s = trackbench.analysis.computeTunerScore(m, maxRange, weights)
%
%   INPUTS
%     m         struct with fields produced by extractTunerMetrics:
%                 .avgPosRMS, .swapCount, .falseTracks, .breakCount,
%                 .worstTrackedPct, .estFailures, .numTruths
%     maxRange  max truth-to-origin distance for the scenario (m). Used
%               to normalize posRMS to a dimensionless [0,1] range.
%     weights   (optional) 6-element vector summing to ~1.0. Defaults to
%               [0.25 0.15 0.10 0.10 0.25 0.15] which balances accuracy
%               (posRMS), identity (swaps), discrimination (falseTracks,
%               breaks), and coverage (trackedPct, estFailures).
%
%   OUTPUT
%     s  struct with fields:
%          .posErr, .swap, .false, .break, .tracked, .estFail
%            — each in [0, 1], representing how bad that component is
%              (0 = perfect, 1 = saturated/catastrophic)
%          .total
%            — weighted sum, also in [0, 1]. Lower = better.
%          .weighted
%            — 6-element vector of (weight × component), so the manual
%              caller can see which terms dominated the total.
%          .weights
%            — copy of the weights used (for reproducibility).
%
%   SCORING PHILOSOPHY
%     Every component is normalized to [0,1] before weighting:
%       posErr   = avgPosRMS / maxRange,         capped at 1.0
%       swap     = swapCount / 5,                capped at 1.0
%       false    = log1p(falseTracks) / log1p(20)
%       break    = breakCount / 5,               capped at 1.0
%       tracked  = (100 - worstTrackedPct) / 100    [NEW]
%       estFail  = estFailures / numTruths           [NEW]
%
%     This means weights are interpretable as "fraction of the score
%     this component contributes when it saturates". Old formula's
%     weights were nominal-only — the swapCount term swamped everything
%     in practice because it wasn't normalized. The new formula fixes
%     that.
%
%   THE TWO NEW TERMS
%     The tracked and estFail terms together close the blind spot
%     where a tracker can score well by ignoring half the targets:
%       tracked   penalizes the worst-tracked single truth
%       estFail   penalizes the *count* of poorly-established truths
%     Both are needed: tracked alone might let a tracker with 1 great
%     truth and 1 dropped truth tie a balanced tracker; estFail alone
%     wouldn't distinguish "barely tracked" from "never tracked". The
%     combination penalizes both extremes.
%
%   See also: trackbench.analysis.extractTunerMetrics, autoTuneTracker,
%             compareTrackers

    if nargin < 3 || isempty(w)
        w = [0.25, 0.15, 0.10, 0.10, 0.25, 0.15];  % sums to 1.0
    end

    % Defensive: legacy callers may pass a 4-element weight vector from
    % the old [posRMS, swaps, falseTracks, breaks] schema. Pad to 6 with
    % default tracked/estFail weights and renormalize so the result is
    % still well-conditioned.
    if numel(w) == 4
        w = [w(:)', 0.25, 0.15];
        w = w / sum(w);
        warning('computeTunerScore:legacyWeights', ...
            ['Got a 4-element weights vector (legacy format). Padded ' ...
             'with default tracked/estFail weights and renormalized. ' ...
             'Pass a 6-element vector to silence this warning.']);
    end

    if numel(w) ~= 6
        error('computeTunerScore:badWeights', ...
            'weights must be a 6-element vector, got %d elements.', numel(w));
    end

    % ─── Normalize each component to [0, 1] ───
    %  Each is "how bad is this metric, 0=perfect, 1=saturated"

    if ~isfinite(m.avgPosRMS) || maxRange <= 0
        posErr = 1.0;
    else
        posErr = min(m.avgPosRMS / maxRange, 1.0);
    end

    swapTerm  = min(m.swapCount / 5, 1.0);
    falseTerm = min(log1p(m.falseTracks) / log1p(20), 1.0);
    breakTerm = min(m.breakCount / 5, 1.0);

    % Tracked coverage: 100% → 0 penalty, 0% → 1 penalty.
    % If the metric is unavailable (NaN — e.g. old saved data without
    % TrackedPct or TrackedFraction columns), assume neutral 0.5 so the
    % comparison isn't biased toward configs that happen to have it.
    if isnan(m.worstTrackedPct)
        trackedTerm = 0.5;
    else
        trackedTerm = max(0, min((100 - m.worstTrackedPct) / 100, 1.0));
    end

    % Establishment-failure count: fraction of truths that took >25% of
    % their lifetime to be tracked. If no truths exist (shouldn't happen
    % but defensive), no penalty.
    if m.numTruths > 0
        estFailTerm = min(m.estFailures / m.numTruths, 1.0);
    else
        estFailTerm = 0;
    end

    % ─── Compose ───
    components = [posErr, swapTerm, falseTerm, breakTerm, trackedTerm, estFailTerm];
    weighted   = w(:)' .* components;

    s.posErr   = posErr;
    s.swap     = swapTerm;
    s.false    = falseTerm;
    s.break    = breakTerm;
    s.tracked  = trackedTerm;
    s.estFail  = estFailTerm;
    s.weighted = weighted;
    s.weights  = w(:)';
    s.total    = sum(weighted);
end
