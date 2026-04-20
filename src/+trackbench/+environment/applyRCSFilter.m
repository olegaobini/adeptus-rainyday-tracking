function [dets, nFiltered] = applyRCSFilter(dets, sPos, sensorObj, targets, allPlatforms)
%applyRCSFilter  Probabilistically drop detections based on RCS and range.
%
%  Implements a probabilistic form of the radar equation:
%      SNR_factor = (sigma_target / sigma_ref) * (refRange / R_actual)^4
%      Pd_scale   = max(pd_floor, min(1, SNR_factor))
%  Each detection is kept with probability Pd_scale. For RCS at or above
%  reference at ranges inside refRange, SNR_factor >> 1 and the detection
%  is always kept. For low-RCS targets beyond the effective range,
%  SNR_factor << 1 and most detections drop, with a small floor that
%  accounts for sidelobe leakage and Swerling fluctuation.
%
%  This produces an observable RCS-vs-range Pd differential that the
%  fusionRadarSensor's native statistical detection model leaves too
%  subtle to see at typical scan counts (24-100 scans).
%
%  Restored in v3.4.2 as an OPT-IN filter, gated by envConfig.rcs_range_filter.
%  Default remains OFF so user scenarios still rely on the sensor-native
%  model; TC-05 (RCS verification) turns it ON to expose the R^4 dependence.
%
%  INPUTS
%    dets         : cell array of objectDetection structs (one scan)
%    sPos         : 1x3 sensor position in scenario frame (m)
%    sensorObj    : fusionRadarSensor object — provides ReferenceRange,
%                   ReferenceRCS, CenterFrequency
%    targets      : array of platform poses (output of targetPoses) for
%                   targets visible at this scan time
%    allPlatforms : cell array of scenario.Platforms (for Signature lookup
%                   keyed by PlatformID)
%
%  OUTPUTS
%    dets         : filtered detection cell array
%    nFiltered    : number of detections dropped
%
%  See also: buildRCSProfile, rcsSignature, fusionRadarSensor,
%            trackbench.detections.runDetections

nFiltered = 0;
if isempty(dets) || isempty(targets); return; end

% Get sensor reference parameters (with fallbacks)
refRange    = 111120;
refRCS_dBsm = 0;
radarFreq   = 2.8e9;  % default S-band
try refRange    = sensorObj.ReferenceRange;  catch; end
try refRCS_dBsm = sensorObj.ReferenceRCS;    catch; end
try radarFreq   = sensorObj.CenterFrequency; catch; end
refRCS_lin = 10^(refRCS_dBsm / 10);

% Floor for sidelobe leakage + Swerling fluctuation (consistent with
% applyDopplerFade which uses 0.05 for the same physical reasons).
pdFloor = 0.05;

% Build target position matrix for fast matching
nTgt = numel(targets);
tgtPositions = zeros(nTgt, 3);
for tt = 1:nTgt
    tgtPositions(tt,:) = targets(tt).Position(:)';
end

% Build PlatformID list from targets
tgtPlatIDs = zeros(nTgt, 1);
for tt = 1:nTgt
    try tgtPlatIDs(tt) = targets(tt).PlatformID; catch; tgtPlatIDs(tt) = 0; end
end

keepRCS = true(numel(dets), 1);

for dd = 1:numel(dets)
    detPos = dets{dd}.Measurement(1:3);
    detPos = detPos(:)';

    % Match detection to nearest target by position
    dists = vecnorm(tgtPositions - detPos, 2, 2);
    [minDist, matchIdx] = min(dists);

    if minDist > 10000
        continue;  % no target within 10 km — keep (likely clutter)
    end

    % Look up the matched target's platform for RCS signature
    platID = tgtPlatIDs(matchIdx);
    if platID <= 0 || platID > numel(allPlatforms)
        continue;
    end

    tgtPlat     = allPlatforms{platID};
    tgtRCS_dBsm = refRCS_dBsm;

    try
        if ~isempty(tgtPlat.Signatures)
            sig = tgtPlat.Signatures{1};
            if isa(sig, 'rcsSignature')
                patVal = sig.Pattern;
                if isscalar(patVal)
                    tgtRCS_dBsm = patVal;
                else
                    % Aspect-dependent: compute viewing angle
                    tgtP = tgtPositions(matchIdx,:);
                    dx = tgtP(1) - sPos(1);
                    dy = tgtP(2) - sPos(2);
                    dz = tgtP(3) - sPos(3);
                    bearAz = atan2d(dy, dx);
                    bearEl = atan2d(-dz, sqrt(dx^2 + dy^2));

                    tgtHdg = 0;
                    if isfield(targets(matchIdx), 'Velocity')
                        vv = targets(matchIdx).Velocity(:);
                        if norm(vv(1:2)) > 1
                            tgtHdg = atan2d(vv(2), vv(1));
                        end
                    end

                    % Aspect: 0=nose-on, ±90=broadside, 180=tail
                    aspAz = wrapTo180(bearAz - tgtHdg + 180);
                    aspEl = max(-90, min(90, bearEl));

                    % rcsSignature.value requires (az, el, freq)
                    tgtRCS_dBsm = sig.value(aspAz, aspEl, radarFreq);
                end
                tgtRCS_lin = 10^(tgtRCS_dBsm / 10);
            else
                continue;
            end
        else
            continue;
        end
    catch
        continue;
    end

    if tgtRCS_lin <= 0
        keepRCS(dd) = false;
        continue;
    end

    % SNR factor from the radar equation, relative to reference conditions.
    % >> 1 means target is well above threshold (saturated Pd).
    % << 1 means target is below threshold (Pd drops linearly to floor).
    R_actual   = norm(detPos - sPos);
    SNR_factor = (tgtRCS_lin / refRCS_lin) * (refRange / max(R_actual, 1))^4;

    % Map SNR factor to Pd scaling, clamped to [pdFloor, 1].
    pdScale = max(pdFloor, min(1.0, SNR_factor));

    % Probabilistic drop
    if rand() > pdScale
        keepRCS(dd) = false;
    end
end

nFiltered = sum(~keepRCS);
if nFiltered > 0
    dets = dets(keepRCS);
end

end
