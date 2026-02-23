function [keepMask, elevAngles, vcpRanges] = applyVCPMask(dets, sensorPos, vcpEntry)
%applyVCPMask  Filter detections using pre-computed vertical coverage pattern.
%
%  For each detection, computes the elevation angle from the sensor and
%  checks whether the target's slant range falls within the VCP maximum
%  detection range at that elevation. Detections beyond the VCP range are
%  in a multipath null and are masked out (not detectable).
%
%  A soft transition zone is applied near the VCP boundary: detections
%  within 90-100% of the VCP range have a probabilistic chance of being
%  masked, modelling the gradual fade at lobe edges rather than a hard
%  cutoff.
%
%  INPUTS
%    dets      : cell array of objectDetection objects
%    sensorPos : [x y z] sensor position in NED (m)
%    vcpEntry  : struct from computeVerticalCoverage with:
%                  .angles     — elevation angles (deg)
%                  .maxRange_m — max detection range (m) at each angle
%                  .enabled    — logical; if false, all detections pass
%
%  OUTPUTS
%    keepMask   : logical Nx1 — true = detection survives VCP check
%    elevAngles : double Nx1 — computed elevation angles (degrees)
%    vcpRanges  : double Nx1 — VCP max range (m) at each detection's elevation
%
% See also: trackbench.environment.computeVerticalCoverage

    nDets = numel(dets);
    keepMask   = true(nDets, 1);
    elevAngles = zeros(nDets, 1);
    vcpRanges  = zeros(nDets, 1);
    
    % If VCP is disabled for this sensor, pass everything through
    if ~vcpEntry.enabled
        return;
    end
    
    vcpAng = vcpEntry.angles;
    vcpRng = vcpEntry.maxRange_m;
    
    % Clamp VCP lookup range (avoid NaN from interp1 outside bounds)
    minAng = min(vcpAng);
    maxAng = max(vcpAng);
    
    for i = 1:nDets
        det = dets{i};
        meas = det.Measurement(:);
        
        if numel(meas) < 3
            continue;  % angle-only, skip VCP check
        end
        
        % Use only position elements (first 3); ignore range-rate/velocity
        pos = meas(1:3)';
        
        % Relative position: detection → sensor
        relPos = pos - sensorPos;
        slantRange = norm(relPos);
        
        if slantRange < 500
            continue;  % too close for multipath model, always detectable
        end
        
        % Elevation angle (NED: z-negative = up, so height = -relPos(3))
        height = -relPos(3);
        groundRange = norm(relPos(1:2));
        elevDeg = atand(height / max(groundRange, 1));
        elevAngles(i) = elevDeg;
        
        % Look up VCP max range at this elevation
        elevClamped = max(minAng, min(maxAng, elevDeg));
        vcpMaxRange = interp1(vcpAng, vcpRng, elevClamped, 'linear', 0);
        vcpRanges(i) = vcpMaxRange;
        
        % Decision: is the target within the VCP coverage?
        if vcpMaxRange <= 0
            % Complete null at this elevation — mask detection
            keepMask(i) = false;
        elseif slantRange > vcpMaxRange
            % Beyond VCP range — masked (in a coverage hole)
            keepMask(i) = false;
        elseif slantRange > 0.90 * vcpMaxRange
            % Soft transition zone (90-100% of VCP range):
            % Linear probability fade from 1.0 at 90% to 0.0 at 100%
            frac = (slantRange - 0.90 * vcpMaxRange) / (0.10 * vcpMaxRange);
            pKeep = 1.0 - frac;
            keepMask(i) = rand() < pKeep;
        end
        % else: within 90% of VCP range — fully detectable (keepMask stays true)
    end
end
