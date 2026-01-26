function dataLog = helperRunDetections(scenario, enableDegradation)
%helperRunDetections  Run the scenario and generate a detection log for tracking.
%
% OVERVIEW
%   This helper advances a radarScenario forward in time and produces a
%   scan-by-scan detection log suitable for multi-target tracking evaluation.
%   It optionally injects controlled "RAINY" degradation at the detection
%   level to simulate weather/environmental effects.
%
% HIGH-LEVEL FLOW
%   1) Advance the scenario one update at a time.
%   2) Call the radar sensor to obtain detections at each update.
%   3) Buffer detections until a full scan completes.
%   4) (Optional) Apply degradation:
%        - Random detection dropouts (effective Pd reduction)
%        - Measurement noise inflation (larger R)
%        - Poisson-distributed false detections ONCE PER SCAN
%   5) Force all detections in a scan to share the same timestamp.
%   6) Apply a simple region-of-interest (ROI) gate.
%   7) Log detections and truth for downstream tracker evaluation.
%
% WHY BUFFER BY SCAN?
%   Rotating radars produce detections over multiple updates per scan, but
%   trackers typically assume a batch of detections at a single time step.
%   Snapping all detections to the scan end time avoids time-gating issues
%   (especially for JPDA).
%
% INPUTS
%   scenario          : radarScenario object (from helperCreateScenario*)
%   enableDegradation : (optional) boolean
%                       false -> IDEAL detections
%                       true  -> RAINY detections (dropouts/noise/clutter)
%
% OUTPUT
%   dataLog struct with fields:
%     - Time        : 1xN array of scan times (seconds)
%     - Truth       : 1xN array/cell of ground-truth target states
%     - Detections  : 1xN cell, each entry is a cell array of objectDetections
%
% IMPORTANT DISTINCTION
%   Degradation here is TRACKER-INPUT degradation only. We do NOT modify
%   radar non-tunable properties. Kalman filter tuning (Q/R) is handled
%   inside initCVFilter / initIMMFilter if desired.
% -------------------------------------------------------------------------

if nargin < 2
    enableDegradation = false;   % Default to IDEAL
end
fprintf("enableDegradation = %d\n", enableDegradation);

% In this scenario layout:
%   scenario.Platforms{1} -> radar tower platform
%   tower.Sensors{1}     -> radar sensor
tower = scenario.Platforms{1};
radar = tower.Sensors{1};

% Restart scenario to ensure repeatable behavior
restart(scenario);

% Buffer detections until a scan completes
scanBuffer = {};

% Initialize output log
dataLog.Time = [];
dataLog.Truth = [];
dataLog.Detections = {};

% Fix random seed so degradation and clutter are repeatable
s = rng;
rng(2018);
disp('Please wait. Generating detections for scenario .....')

while advance(scenario)

    % Current simulation time
    simTime = scenario.SimulationTime;

    % Ground-truth target poses relative to radar platform
    targets = targetPoses(tower);

    % Radar ego pose
    ins = pose(tower, 'true');

    % Get detections for this update
    [dets,~,config] = radar(targets,ins,simTime);
    dets = dets(:);     % enforce column cell array
    nFalse = 0;         % number of false detections added this scan

    % -------- Level-1 Weather / Environment Degradation (per update) --------
    % This layer acts AFTER the radar call. Radar model itself is unchanged.
    if enableDegradation
        w = weatherSeverity(simTime);   % 0 (clear) or 1 (storm)

        % Effective detection survival probability
        pdEff = (1-w)*0.95 + w*0.70;    % tune as needed

        % Measurement noise inflation factor
        Rmult = 1 + 2*w;               % 1x clear, 3x storm

        % 1) Randomly drop detections (missed detections)
        if ~isempty(dets)
            keep = rand(numel(dets),1) < pdEff;
            dets = dets(keep);
        end

        % 2) Inflate measurement noise covariance R
        for i = 1:numel(dets)
            dets{i}.MeasurementNoise = dets{i}.MeasurementNoise * Rmult;
        end
    end
    % ----------------------------------------------------------------------

    % Append detections from this update into scan buffer
    if ~isempty(dets)
        scanBuffer = [scanBuffer; dets]; %#ok<AGROW>
    end

    % When a full scan completes, process the batch
    if config.IsScanDone
        nBefore = numel(scanBuffer);

        % Debug: timestamp spread before snapping
        if ~isempty(scanBuffer)
            times = cellfun(@(d)d.Time, scanBuffer);
            fprintf("Scan buffer time span (pre-snap) = %.6f s\n", ...
                max(times) - min(times));
        else
            fprintf("Scan buffer is empty at scan end.\n");
        end

        % -------- Inject false detections ONCE PER SCAN --------
        if enableDegradation
            wScan = weatherSeverity(simTime);

            % Poisson mean false detections per scan
            lambdaFalsePerScan = (1-wScan)*0.0 + wScan*3.0;
            nFalse = poissrnd(lambdaFalsePerScan);

            % Reuse measurement parameters if possible
            haveMP = ~isempty(scanBuffer) && ...
                     isprop(scanBuffer{1}, 'MeasurementParameters');
            if haveMP
                mp = scanBuffer{1}.MeasurementParameters;
            end

            % Clutter measurement noise (larger than real detections)
            if wScan > 0
                clutterSigma = 150;   % meters (storm)
            else
                clutterSigma = 100;   % meters
            end
            Rclutter = eye(3) * clutterSigma^2;

            for i = 1:nFalse
                meas = falseMeasInSurveillanceVolume();

                if haveMP
                    scanBuffer{end+1,1} = objectDetection(simTime, meas, ...
                        'MeasurementNoise', Rclutter, ...
                        'SensorIndex', 1, ...
                        'MeasurementParameters', mp);
                else
                    scanBuffer{end+1,1} = objectDetection(simTime, meas, ...
                        'MeasurementNoise', Rclutter, ...
                        'SensorIndex', 1);
                end
            end
        end

        % -------- Force single timestamp for entire scan --------
        % This avoids time-gating issues in downstream trackers.
        for k = 1:numel(scanBuffer)
            scanBuffer{k}.Time = simTime;
        end

        % -------- Simple ROI gate to suppress obvious clutter --------
        scanBuffer = gateDetectionsROI(scanBuffer);

        fprintf("t=%.2f: dets=%d (before=%d, clutterAdded=%d)\n", ...
            simTime, numel(scanBuffer), nBefore, nFalse);

        % Log results
        dataLog.Time = [dataLog.Time, simTime];
        dataLog.Truth = [dataLog.Truth, targets];
        dataLog.Detections = [dataLog.Detections(:)', {scanBuffer}];

        % Reset for next scan
        scanBuffer = {};
    end
end

% Restore RNG state
rng(s);
disp('Detections generation complete.')

    function w = weatherSeverity(t)
        %weatherSeverity  Simple on/off storm model
        stormStart = 15;   % seconds
        stormEnd   = 30;   % seconds
        w = double(t >= stormStart && t <= stormEnd);
    end

    function meas = falseMeasInSurveillanceVolume()
        %falseMeasInSurveillanceVolume  Generate plausible clutter measurement
        x = (-1.5e3) + (3.0e3)*rand;     % [-1.5 km, +1.5 km]
        y = (-20.5e3) + (2.0e3)*rand;    % [-20.5 km, -18.5 km]
        z = -3e3 + 700*randn;            % around -3 km
        meas = [x; y; z];
    end
end

function detsOut = gateDetectionsROI(detsIn)
%gateDetectionsROI  Hard region-of-interest (ROI) gate.
%
% Keeps only detections inside a fixed 3D bounding box. This is a pragmatic
% clutter limiter, not a statistical (Mahalanobis) gate.

    if isempty(detsIn)
        detsOut = detsIn;
        return;
    end

    % ROI bounds (meters)
    xMin = -8000;  xMax =  8000;
    yMin = -26000; yMax = -16000;
    zMin = -8000;  zMax =  500;

    keep = false(numel(detsIn),1);
    for ii = 1:numel(detsIn)
        z = detsIn{ii}.Measurement(:);
        if numel(z) == 2
            z = [z; 0];   % pad 2D detections
        end

        keep(ii) = (z(1) >= xMin && z(1) <= xMax) && ...
                   (z(2) >= yMin && z(2) <= yMax) && ...
                   (z(3) >= zMin && z(3) <= zMax);
    end
    detsOut = detsIn(keep);
end
