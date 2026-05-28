function filter = trackbenchFilterInit(detection, baseInitFcn)
%TRACKBENCHFILTERINIT  Detection-aware filter initialization router.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   Branches on detection.MeasurementParameters to choose an
%   angle-only filter (initMSCRPEKF → trackingGSF of trackingMSCEKF
%   bank) for IR-style 2-vec [az;el] detections, or falls through to
%   baseInitFcn for the conventional 3-vec position detection path
%   (radar, sonar, lidar).
%
%   v3.7.3 pivot: initrpekf (Cartesian RPEKF) replaced by initMSCRPEKF
%   (modified-spherical-coordinate RPEKF). The Cartesian filter cannot
%   collapse range hypotheses for a MOVING ownship because state and
%   range are coupled in Cartesian; MSC decouples observable bearings
%   from unobservable range, requiring per-scan ObserverInput
%   propagation on each bank filter (runTracker.m owns that side).
%   Phase 3 empirical for v3.6.16 airborne_IRST confirmed the Cartesian
%   filter holds bearings correctly but never collapses range → 0%
%   Tracked%. MSC-RPEKF is the canonical fix.
%
%   Wired as the tracker's FilterInitializationFcn via a closure that
%   bakes in baseInitFcn (the CV/IMM lambda from buildTracker.m).
%
%   REFERENCES
%     [1] MathWorks, "Passive Ranging Using a Single Maneuvering
%         Sensor" example (R2025b) — canonical MSC-RPEKF pattern:
%         https://www.mathworks.com/help/fusion/ug/passive-ranging-using-a-single-maneuvering-sensor.html
%     [2] MathWorks, initcvmscekf documentation (R2025b):
%         https://www.mathworks.com/help/fusion/ref/initcvmscekf.html
%     [3] MathWorks, trackingMSCEKF documentation (R2025b):
%         https://www.mathworks.com/help/fusion/ref/trackingmscekf.html
%     [4] MathWorks, objectDetection MeasurementParameters:
%         https://www.mathworks.com/help/fusion/ref/objectdetection.html

    % Detect angle-only [az; el] detection via MeasurementParameters.
    % MeasurementParameters can be: empty (default), a struct, a cell
    % array of structs, or a struct array (irSensor emits 2x1: outer
    % spherical frame + inner rectangular). Defend all four shapes.
    mp = detection.MeasurementParameters;
    isAngleOnly = false;
    if ~isempty(mp)
        if iscell(mp); mp = mp{1}; end
        mp = mp(1);   % struct-array case (irSensor returns 2x1): take sensor-side (first) frame
        isAngleOnly = isfield(mp, 'Frame') && strcmpi(mp.Frame, 'Spherical') && ...
                      isfield(mp, 'HasRange') && ~mp.HasRange;
    end

    if isAngleOnly
        % v3.7.3 — MSC-RPEKF (range-parameterized modified-spherical-
        % coordinate EKF bank). Replaces v3.7.0's initrpekf for moving-
        % ownship passive ranging. Requires runTracker.m to propagate
        % ObserverInput per-scan on each bank filter; without that the
        % bank cannot collapse range and Tracked% stays at 0% (the
        % v3.6.16 Phase 3 empirical failure mode). See initMSCRPEKF
        % local function below — near-verbatim from canonical example,
        % rMin/rMax tuned to test_IRST_airborne's ~5 km target range.
        filter = initMSCRPEKF(detection);
    else
        % Fall through to the configured CV/IMM init from buildTracker.
        filter = baseInitFcn(detection);
    end
end

function filter = initMSCRPEKF(detection)
% Range-parameterized MSC-EKF for moving-sensor angle-only passive
% ranging. Verbatim from MathWorks "Passive Ranging Using a Single
% Maneuvering Sensor" example, EXCEPT rMin/rMax adjusted from canonical
% [3e4, 8e4] (long-range tactical geometry) to [1e3, 2e4] for
% test_IRST_airborne's ~5 km target range. ObserverInput propagation
% per-scan is required downstream (runTracker.m); without it the
% range hypotheses cannot collapse and the bank stays uncertain in
% range — see v3.6.16 Phase 3 empirical for the empirical confirmation.

    % Range-parametrization constants
    rMin = 1e3;       % 1 km — minimum range hypothesis (scenario-tuned)
    rMax = 2e4;       % 20 km — maximum range hypothesis (scenario-tuned)
    numFilters = 10;  % canonical
    rho = (rMax/rMin)^(1/numFilters);
    Cr = 2*(rho - 1)/(rho + 1)/sqrt(12);
    indFilters = cell(numFilters, 1);
    for i = 1:numFilters
        range = rMin/2 * (rho^i + rho^(i-1));
        rangeSigma = Cr*range;
        % Use initcvmscekf to create a trackingMSCEKF with provided
        % range estimate. R2025b 3D state: [az; azRate; el; elRate; 1/r; rDot/r].
        indFilters{i} = initcvmscekf(detection, [range rangeSigma]);
        % Velocity covariance scaling (canonical 400×, allows ≤200 m/s
        % standard-deviation velocity — captures typical fighter / IR
        % target dynamics).
        indFilters{i}.StateCovariance(2:2:end, 2:2:end) = ...
            400 * indFilters{i}.StateCovariance(2:2:end, 2:2:end);
    end
    filter = trackingGSF(indFilters);
end
