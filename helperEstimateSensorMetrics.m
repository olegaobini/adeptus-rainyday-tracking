function sensorM = helperEstimateSensorMetrics(dataLog, maxScans)
%helperEstimateSensorMetrics  Estimate simple sensor metrics PER SCAN.
%
% Assumes dataLog.Detections{k} is one completed scan batch
% (true for your helperRunDetections because it logs only at IsScanDone).

if nargin < 2
    maxScans = inf;
end

numScans = min(numel(dataLog.Detections), maxScans);
detsPerScan = cellfun(@numel, dataLog.Detections(1:numScans));

% Truth count per scan (assumed constant)
% NOTE: dataLog.Truth is typically a struct array sized [numScans x numTruth].
% Using numel(dataLog.Truth(1)) returns 1 (first element), so infer truth count from dimensions.
if iscell(dataLog.Truth)
    % Cell per scan: each cell contains an array of truths for that scan
    nTruth = numel(dataLog.Truth{1});
elseif isstruct(dataLog.Truth)
    szT = size(dataLog.Truth);
    if ismatrix(dataLog.Truth) && szT(1) == numScans
        % Common case: rows are scans, columns are truths
        nTruth = szT(2);
    elseif ismatrix(dataLog.Truth) && szT(2) == numScans
        % Less common: columns are scans, rows are truths
        nTruth = szT(1);
    else
        % Fallback: try to estimate from first scan slice if possible
        try
            nTruth = size(dataLog.Truth(1,:), 2);
        catch
            nTruth = numel(dataLog.Truth);
        end
    end
else
    % Unknown truth container: fall back conservatively
    nTruth = 1;
end

% Scan period (sec)
if numel(dataLog.Time) >= 2
    scanPeriod = mean(diff(dataLog.Time(1:min(numel(dataLog.Time),numScans))));
else
    scanPeriod = NaN;
end

% Detections per scan stats
sensorM.MinDetectionsPerScan  = min(detsPerScan);
sensorM.MeanDetectionsPerScan = mean(detsPerScan);
sensorM.MaxDetectionsPerScan  = max(detsPerScan);

% "Soft Pd": average fraction of truths reported per scan
sensorM.Pd_soft = mean(detsPerScan / max(nTruth,1));
sensorM.Pd_soft = min(max(sensorM.Pd_soft, 0), 1);

% "Hard Pd": fraction of scans where we got at least one detection per truth
sensorM.Pd_est = mean(detsPerScan >= nTruth);

% False alarms per scan (very rough; assumes 1 det per truth when present)
sensorM.FalseAlarmsPerScan = mean(max(detsPerScan - nTruth, 0));

sensorM.NumTruth = nTruth;
sensorM.ScanPeriod = scanPeriod;
end
