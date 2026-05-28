function m = extractTunerMetrics(trackSummary, truthSummary, trackMetrics)
%extractTunerMetrics  Pull scoring-relevant numbers from tracker output.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   USAGE
%     m = trackbench.analysis.extractTunerMetrics(trackSummary, truthSummary, trackMetrics)
%
%   INPUTS
%     trackSummary  table from runTracker (one row per track)
%     truthSummary  table from runTracker (one row per truth)
%     trackMetrics  table from runTracker (per-track quality metrics)
%
%   OUTPUT
%     m  struct with the fields computeTunerScore needs, plus extras
%        useful for display and reporting:
%          .avgPosRMS       — average position RMS across all tracks (m)
%          .swapCount       — total track identity swaps
%          .falseTracks     — count of tracks never associated with truth
%          .breakCount      — total truth breaks (lost then re-acquired)
%          .worstTrackedPct — minimum TrackedPct across truths (0-100)
%          .avgTrackedPct   — mean TrackedPct across truths (0-100)
%          .estFailures     — count of truths with EstLen/TotLen > 0.25
%          .numTruths       — height of truthSummary (for normalization)
%
%   BACKWARD COMPATIBILITY
%     Reads either the new TrackedPct column or the legacy
%     TrackedFraction column, so this works with saved .mat files from
%     before the v3.5 rename without re-running the simulation.
%
%   See also: trackbench.analysis.computeTunerScore, runTracker

    % ─── Defaults ───
    m.avgPosRMS       = Inf;
    m.swapCount       = 0;
    m.falseTracks     = 0;
    m.breakCount      = 0;
    m.worstTrackedPct = NaN;
    m.avgTrackedPct   = NaN;
    m.estFailures     = 0;
    m.numTruths       = 0;

    % ─── posRMS from trackMetrics ───
    if istable(trackMetrics) && ~isempty(trackMetrics) && ...
            ismember('posRMS', trackMetrics.Properties.VariableNames)
        validRMS = trackMetrics.posRMS(isfinite(trackMetrics.posRMS));
        if ~isempty(validRMS)
            m.avgPosRMS = mean(validRMS);
        end
    end

    % ─── Track-derived metrics ───
    if istable(trackSummary) && ~isempty(trackSummary)
        if ismember('SwapCount', trackSummary.Properties.VariableNames)
            m.swapCount = sum(trackSummary.SwapCount);
        end
        if ismember('AssignedTruthID', trackSummary.Properties.VariableNames)
            m.falseTracks = sum(isnan(trackSummary.AssignedTruthID));
        end
    end

    % ─── Truth-derived metrics (incl. NEW coverage) ───
    if istable(truthSummary) && ~isempty(truthSummary)
        m.numTruths = height(truthSummary);

        if ismember('BreakCount', truthSummary.Properties.VariableNames)
            m.breakCount = sum(truthSummary.BreakCount);
        end

        % NEW: tracked coverage (handles both TrackedPct and legacy
        % TrackedFraction column names so old .mat files still work).
        if ismember('TrackedPct', truthSummary.Properties.VariableNames)
            pct = truthSummary.TrackedPct;
            pct = pct(~isnan(pct));
            if ~isempty(pct)
                m.worstTrackedPct = min(pct);
                m.avgTrackedPct   = mean(pct);
            end
        elseif ismember('TrackedFraction', truthSummary.Properties.VariableNames)
            frac = truthSummary.TrackedFraction;
            frac = frac(~isnan(frac));
            if ~isempty(frac)
                m.worstTrackedPct = round(100 * min(frac));
                m.avgTrackedPct   = round(100 * mean(frac));
            end
        end

        % NEW: establishment-failure count. Matches the >25% threshold
        % used in the per-tracker summary box from runSingleScenario.
        if all(ismember({'TotalLength','EstablishmentLength'}, ...
                truthSummary.Properties.VariableNames))
            tl = truthSummary.TotalLength;
            el = truthSummary.EstablishmentLength;
            safe = tl > 0;
            if any(safe)
                m.estFailures = sum( (el(safe) ./ tl(safe)) > 0.25 );
            end
        end
    end
end
