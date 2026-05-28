function printCompactTrackSummary(T)
%printCompactTrackSummary  Compact fixed-width display of a trackSummary.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
% Replaces MATLAB's default disp() for trackSummary tables, which on a
% narrow terminal (deployed app cmd window) wraps the 9-column display
% across multiple lines and breaks column headers. Same data, shorter
% column labels, fits in about 70 characters.
%
% Columns shown:
%   TrkID    : TrackID
%   Truth    : AssignedTruthID (NaN displayed as "NaN")
%   Surv     : Surviving (yes/no)
%   Len      : TotalLength
%   Reported : Inverted UnreportedStatus — true unreported means
%              Reported = no
%   MeanTBR  : MeanTimeBetweenReports (seconds; NaN allowed)
%   MaxTBR   : MaxTimeBetweenReports (seconds; NaN allowed)
%   Diverged : DivergenceStatus (yes/no)
%   Swaps    : SwapCount
%
% See also: trackbench.reporting.printCompactTruthSummary, runTracker

    if isempty(T) || ~istable(T) || height(T) == 0
        fprintf('  (no track summary)\n');
        return;
    end

    fprintf('  TrkID  Truth  Surv  Len  Reported  MeanTBR   MaxTBR  Diverged  Swaps\n');
    fprintf('  -----  -----  ----  ---  --------  -------   ------  --------  -----\n');

    for i = 1:height(T)
        tid = T.TrackID(i);

        atid = T.AssignedTruthID(i);
        if isnan(atid)
            truthStr = '  NaN';
        else
            truthStr = sprintf('%5d', atid);
        end

        if logical(T.Surviving(i))
            survStr = ' yes';
        else
            survStr = '  no';
        end

        tl = T.TotalLength(i);

        % UnreportedStatus = true means there WAS an unreported period.
        % Flip to "Reported = no" so the column reads positively.
        if logical(T.UnreportedStatus(i))
            reportedStr = '      no';
        else
            reportedStr = '     yes';
        end

        meanStr = formatTBR(T.MeanTimeBetweenReports(i));
        maxStr  = formatTBR(T.MaxTimeBetweenReports(i));

        if logical(T.DivergenceStatus(i))
            divStr = '     yes';
        else
            divStr = '      no';
        end

        swaps = T.SwapCount(i);

        fprintf('  %5d  %s  %s  %3d  %s  %s  %s  %s  %5d\n', ...
            tid, truthStr, survStr, tl, reportedStr, ...
            meanStr, maxStr, divStr, swaps);
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
