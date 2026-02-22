function swapReport = analyzeTrackSwaps(assignLog, truthData, scanTimes)
%analyzeTrackSwaps  Detect and characterize track-to-truth identity swaps.
%
% WHAT IS A TRACK SWAP?
%   A track swap occurs when a tracker reassigns a track from one truth
%   target to another. In a friend-or-foe context, this means a "friend"
%   track could suddenly be associated with a "foe" truth (or vice versa).
%   This is a critical failure mode for air traffic control and IFF systems.
%
% HOW IT WORKS
%   For each TrackID, we look at the sequence of TruthIDs it was assigned
%   to over time. A swap is logged whenever the assigned TruthID changes
%   between consecutive scans.
%
% INPUTS
%   assignLog  : table with columns {Time, PlatformID, TrackID, TruthID, SensorsUsed}
%                (produced by helperRunTracker)
%   truthData  : (optional) dataLog.Truth array for computing inter-target
%                separation at swap time. Pass [] to skip proximity analysis.
%   scanTimes  : (optional) full vector of scan times from dataLog.Time.
%                Used to compute separation at swap time. Pass [] to skip.
%
% OUTPUT
%   swapReport : struct with fields:
%     .totalSwaps      : total number of swap events across all tracks
%     .swapEvents      : table with one row per swap event:
%                        {Time, TrackID, FromTruthID, ToTruthID, SeparationAtSwap_m}
%     .perTrack        : table with per-track swap counts:
%                        {TrackID, SwapCount, FirstSwapTime, LastSwapTime}
%     .perTruth        : table with per-truth "was swapped away" counts:
%                        {TruthID, TimesLost, TimesGained}
%     .swapFree        : logical — true if zero swaps occurred
%     .maxConsecutive   : max consecutive scans any track held wrong assignment
%                        (0 if no swaps)
%
% USAGE
%   swapReport = analyzeTrackSwaps(assignLog);
%   swapReport = analyzeTrackSwaps(assignLog, dataLog.Truth, dataLog.Time);

if nargin < 2; truthData = []; end
if nargin < 3; scanTimes = []; end

%% Initialize output
swapReport = struct();
swapReport.totalSwaps = 0;
swapReport.swapEvents = table([], [], [], [], [], ...
    'VariableNames', {'Time','TrackID','FromTruthID','ToTruthID','SeparationAtSwap_m'});
swapReport.perTrack = table([], [], [], [], ...
    'VariableNames', {'TrackID','SwapCount','FirstSwapTime','LastSwapTime'});
swapReport.perTruth = table([], [], [], ...
    'VariableNames', {'TruthID','TimesLost','TimesGained'});
swapReport.swapFree = true;
swapReport.maxConsecutive = 0;

%% Validate input
if isempty(assignLog) || height(assignLog) == 0
    return;
end

%% Build per-track assignment history
trackIDs = unique(assignLog.TrackID);
allSwapEvents = [];  % will become struct array

for ti = 1:numel(trackIDs)
    tID = trackIDs(ti);
    rows = assignLog(assignLog.TrackID == tID, :);
    rows = sortrows(rows, 'Time');

    times    = rows.Time;
    truthSeq = rows.TruthID;

    % Detect changes in truth assignment
    for k = 2:numel(truthSeq)
        if truthSeq(k) ~= truthSeq(k-1) && truthSeq(k) > 0 && truthSeq(k-1) > 0
            ev = struct();
            ev.Time        = times(k);
            ev.TrackID     = tID;
            ev.FromTruthID = truthSeq(k-1);
            ev.ToTruthID   = truthSeq(k);
            ev.SeparationAtSwap_m = NaN;

            % Compute inter-target separation at swap time if truth data available
            if ~isempty(truthData) && ~isempty(scanTimes)
                ev.SeparationAtSwap_m = getTargetSeparation(...
                    truthData, scanTimes, times(k), truthSeq(k-1), truthSeq(k));
            end

            allSwapEvents = [allSwapEvents; ev]; %#ok<AGROW>
        end
    end
end

%% Build swap events table
if isempty(allSwapEvents)
    swapReport.swapFree = true;
    swapReport.totalSwaps = 0;
    return;
end

swapReport.swapFree = false;
swapReport.totalSwaps = numel(allSwapEvents);

swapReport.swapEvents = struct2table(allSwapEvents);
swapReport.swapEvents = sortrows(swapReport.swapEvents, 'Time');

%% Per-track summary
perTrackData = [];
for ti = 1:numel(trackIDs)
    tID = trackIDs(ti);
    tSwaps = swapReport.swapEvents(swapReport.swapEvents.TrackID == tID, :);
    nSwaps = height(tSwaps);

    pt = struct();
    pt.TrackID = tID;
    pt.SwapCount = nSwaps;
    if nSwaps > 0
        pt.FirstSwapTime = min(tSwaps.Time);
        pt.LastSwapTime  = max(tSwaps.Time);
    else
        pt.FirstSwapTime = NaN;
        pt.LastSwapTime  = NaN;
    end
    perTrackData = [perTrackData; pt]; %#ok<AGROW>
end
swapReport.perTrack = struct2table(perTrackData);

%% Per-truth summary (how often each truth was lost/gained)
allTruthIDs = unique([assignLog.TruthID(assignLog.TruthID > 0)]);
perTruthData = [];
for gi = 1:numel(allTruthIDs)
    gID = allTruthIDs(gi);
    pt = struct();
    pt.TruthID = gID;
    % "Lost" = truth was the FromTruthID (track left this truth)
    pt.TimesLost = sum(swapReport.swapEvents.FromTruthID == gID);
    % "Gained" = truth was the ToTruthID (track arrived at this truth)
    pt.TimesGained = sum(swapReport.swapEvents.ToTruthID == gID);
    perTruthData = [perTruthData; pt]; %#ok<AGROW>
end
if ~isempty(perTruthData)
    swapReport.perTruth = struct2table(perTruthData);
end

%% Max consecutive scans with wrong assignment
% After a swap, count how many scans pass before the track returns to its
% original truth (or the scenario ends)
maxConsec = 0;
for ti = 1:numel(trackIDs)
    tID = trackIDs(ti);
    rows = assignLog(assignLog.TrackID == tID, :);
    rows = sortrows(rows, 'Time');
    truthSeq = rows.TruthID;

    if numel(truthSeq) < 2; continue; end

    % Find the "home" truth (first assigned)
    homeTruth = truthSeq(1);
    consecWrong = 0;
    for k = 2:numel(truthSeq)
        if truthSeq(k) ~= homeTruth && truthSeq(k) > 0
            consecWrong = consecWrong + 1;
            maxConsec = max(maxConsec, consecWrong);
        else
            consecWrong = 0;
        end
    end
end
swapReport.maxConsecutive = maxConsec;

end

%% ========================================================================
%                         LOCAL HELPER FUNCTIONS
%% ========================================================================

function sep = getTargetSeparation(truthData, scanTimes, swapTime, truthA, truthB)
%getTargetSeparation  Compute Euclidean distance between two truth targets
% at the scan time closest to swapTime.
%
% truthData is the [nTargets x nTimes] struct array from dataLog.Truth
% with field 'Position'.

    sep = NaN;

    % Find nearest scan index
    [~, idx] = min(abs(scanTimes - swapTime));

    try
        [nTgts, ~] = size(truthData);

        posA = [];
        posB = [];

        for t = 1:nTgts
            tp = truthData(t, idx);
            if isfield(tp, 'PlatformID')
                if tp.PlatformID == truthA
                    posA = tp.Position(:);
                elseif tp.PlatformID == truthB
                    posB = tp.Position(:);
                end
            end
        end

        if ~isempty(posA) && ~isempty(posB)
            sep = norm(posA - posB);
        end
    catch
        sep = NaN;
    end
end
