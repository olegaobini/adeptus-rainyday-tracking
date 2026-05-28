function report = analyzeSensitivity(sweepTable, scoreCol, paramCols)
%analyzeSensitivity  Rank parameters by how much they moved the score.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   For a sweep table where rows are parameter combinations and a score
%   column captures the result of each combination, report which
%   parameters actually mattered. Two complementary measures:
%
%     1. Spearman rank correlation between each parameter value and the
%        score. Captures monotonic relationships (bigger gate → lower
%        score). Sign tells direction.
%     2. Normalized "score range" — std-dev of mean-score within each
%        unique parameter value, divided by overall std-dev. Captures
%        non-monotonic relationships too (some middle gate is best).
%
%   USAGE
%     report = trackbench.analysis.analyzeSensitivity(sweepTable, ...
%         'Score', {'Gate', 'Volume', 'Beta'})
%
%   INPUTS
%     sweepTable  table with one row per evaluated config
%     scoreCol    name of the score column (string)
%     paramCols   cell array of parameter column names to analyze
%
%   OUTPUT
%     report  table with columns:
%               .Param         — parameter name
%               .Correlation   — Spearman rho with score (-1 to 1)
%               .Influence     — fraction of score variance explained
%                                by this param's groupwise means (0-1)
%               .NumLevels     — distinct values tested
%               .Verdict       — human-readable summary
%
%   INTERPRETATION
%     |Correlation| > 0.5 → strong monotonic effect, sign tells direction
%     Influence > 0.3     → param materially moved the score (any shape)
%     Influence < 0.05    → param probably wasn't doing anything useful
%
%   See also: autoTuneTracker, compareTrackers

    if ~istable(sweepTable) || height(sweepTable) < 4
        report = table();
        return;
    end

    if ~ismember(scoreCol, sweepTable.Properties.VariableNames)
        error('analyzeSensitivity:noScoreCol', ...
            'Score column "%s" not in sweepTable', scoreCol);
    end

    scores = sweepTable.(scoreCol);
    valid  = isfinite(scores);
    if sum(valid) < 4
        report = table();
        return;
    end
    scores = scores(valid);
    overallStd = std(scores);

    n = numel(paramCols);
    paramNames   = strings(n, 1);
    correlations = nan(n, 1);
    influences   = nan(n, 1);
    levels       = nan(n, 1);
    verdicts     = strings(n, 1);

    for i = 1:n
        pname = paramCols{i};
        paramNames(i) = string(pname);

        if ~ismember(pname, sweepTable.Properties.VariableNames)
            verdicts(i) = "(column not found)";
            continue;
        end

        vals = sweepTable.(pname);
        vals = vals(valid);

        % Cast everything to double for correlation. Skip non-numeric.
        try
            vals = double(vals);
        catch
            verdicts(i) = "(non-numeric)";
            continue;
        end

        uniq = unique(vals(~isnan(vals)));
        levels(i) = numel(uniq);

        if levels(i) < 2
            verdicts(i) = "(only one value tested)";
            continue;
        end

        % Spearman rank correlation. Hand-rolled to avoid Statistics
        % Toolbox dependency.
        rho = spearmanRho(vals, scores);
        correlations(i) = rho;

        % Influence: standard deviation of per-level mean score, scaled
        % by the overall score range. If grouping by this param
        % accounts for most of the score variation, influence ≈ 1.
        groupMeans = zeros(numel(uniq), 1);
        for k = 1:numel(uniq)
            mask = (vals == uniq(k));
            if any(mask)
                groupMeans(k) = mean(scores(mask));
            end
        end
        if overallStd > 0 && numel(groupMeans) > 1
            influences(i) = std(groupMeans) / overallStd;
        else
            influences(i) = 0;
        end

        % Verdict
        if abs(rho) >= 0.5
            dir = ternaryLocal(rho > 0, "↑ score (higher = worse)", "↓ score (higher = better)");
            verdicts(i) = sprintf("strong monotonic effect: %s", dir);
        elseif influences(i) >= 0.3
            verdicts(i) = "moderate non-monotonic effect";
        elseif influences(i) >= 0.1
            verdicts(i) = "minor effect";
        else
            verdicts(i) = "negligible — consider removing from sweep";
        end
    end

    report = table(paramNames, correlations, influences, levels, verdicts, ...
        'VariableNames', {'Param','Correlation','Influence','NumLevels','Verdict'});

    % Sort by influence descending so the most impactful params lead.
    [~, order] = sort(report.Influence, 'descend', 'MissingPlacement', 'last');
    report = report(order, :);
end

function rho = spearmanRho(x, y)
%spearmanRho  Spearman rank correlation, no toolbox dependency.
%  Identical to corr(x,y,'type','Spearman') from Statistics Toolbox but
%  computed from first principles so this works in a plain MATLAB
%  install. NaN-tolerant via pairwise drop.
    keep = ~(isnan(x) | isnan(y));
    if sum(keep) < 3
        rho = NaN;
        return;
    end
    rx = tiedRank(x(keep));
    ry = tiedRank(y(keep));
    n  = numel(rx);
    rho = (sum(rx .* ry) - n * mean(rx) * mean(ry)) / ...
          ((n - 1) * std(rx) * std(ry));
end

function r = tiedRank(v)
%tiedRank  Average ranks for tied values (Spearman convention).
    [~, idx] = sort(v);
    r = zeros(size(v));
    r(idx) = 1:numel(v);
    [vSorted, sIdx] = sort(v);
    rSorted = r(sIdx);
    i = 1;
    while i <= numel(vSorted)
        j = i;
        while j < numel(vSorted) && vSorted(j+1) == vSorted(i)
            j = j + 1;
        end
        if j > i
            avg = mean(rSorted(i:j));
            rSorted(i:j) = avg;
        end
        i = j + 1;
    end
    r(sIdx) = rSorted;
end

function out = ternaryLocal(cond, a, b)
    if cond; out = a; else; out = b; end
end
