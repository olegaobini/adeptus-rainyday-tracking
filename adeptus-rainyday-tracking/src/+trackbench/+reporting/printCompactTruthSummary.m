function printCompactTruthSummary(T)
%printCompactTruthSummary  Compact fixed-width display of a truthSummary.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
% Replaces MATLAB's default disp() for truthSummary tables, which on a
% narrow terminal (deployed app cmd window) wraps the 9-column display
% across multiple lines and breaks column headers. Same data, shorter
% column labels, fits in about 80 characters.
%
% Also bridges the legacy TrackedFraction column (5-decimal float,
% written by older runs) to the new TrackedPct (integer percent), so
% saved .mat files from before that rename still display cleanly.
%
% Columns shown:
%   TruID    : TruthID
%   AssocTrk : AssociatedTrackID — the track that spent the most
%              cumulative time associated with this truth, NOT
%              necessarily the currently-assigned track.
%   TotLen   : TotalLength (truth lifetime in scenario steps)
%   EstLen   : EstablishmentLength (steps until first association)
%   Tracked% : Estimated tracked fraction as integer percent. This is
%              an UPPER BOUND — assumes no track breaks after
%              establishment (which is what BreakCount actually tracks).
%   Breaks   : BreakCount
%   MeanTBR  : MeanTimeBetweenReports (seconds; NaN allowed)
%   MaxTBR   : MaxTimeBetweenReports (seconds; NaN allowed)
%   Reported : Inverted UnreportedStatus — true unreported means
%              Reported = no
%
% See also: trackbench.reporting.printCompactTrackSummary, runTracker

    if isempty(T) || ~istable(T) || height(T) == 0
        fprintf('  (no truth summary)\n');
        return;
    end

    fprintf('  TruID  AssocTrk  TotLen  EstLen  Tracked%%  Breaks  MeanTBR   MaxTBR  Reported\n');
    fprintf('  -----  --------  ------  ------  --------  ------  -------   ------  --------\n');

    hasPct  = ismember('TrackedPct',      T.Properties.VariableNames);
    hasFrac = ismember('TrackedFraction', T.Properties.VariableNames);

    for i = 1:height(T)
        tid  = T.TruthID(i);

        atid = T.AssociatedTrackID(i);
        if isnan(atid)
            atidStr = '     NaN';
        else
            atidStr = sprintf('%8d', atid);
        end

        tl = T.TotalLength(i);
        el = T.EstablishmentLength(i);
        bc = T.BreakCount(i);

        if hasPct
            pctVal = T.TrackedPct(i);
        elseif hasFrac
            pctVal = round(100 * T.TrackedFraction(i));
        else
            pctVal = NaN;
        end
        if isnan(pctVal)
            pctStr = '     NaN';
        else
            pctStr = sprintf('%7d%%', pctVal);
        end

        meanStr = formatTBR(T.MeanTimeBetweenReports(i));
        maxStr  = formatTBR(T.MaxTimeBetweenReports(i));

        % UnreportedStatus = true means there WAS an unreported period.
        % Flip to "Reported = no" so the column reads positively.
        if logical(T.UnreportedStatus(i))
            reportedStr = '      no';
        else
            reportedStr = '     yes';
        end

        fprintf('  %5d  %s  %6d  %6d  %s  %6d  %s  %s  %s\n', ...
            tid, atidStr, tl, el, pctStr, bc, meanStr, maxStr, reportedStr);
    end
end

function s = formatTBR(v)
    % Format Mean/MaxTimeBetweenReports values with a fixed 7-char width.
    % NaN is common when a track had only one report (no "between" gap),
    % so format it explicitly rather than letting sprintf print "NaN".
    if isnan(v)
        s = '    NaN';
    else
        s = sprintf('%7.2f', v);
    end
end
