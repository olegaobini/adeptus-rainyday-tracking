function [keptDets, diagInfo] = applyDopplerFade(dets, sensorPos, targets, sensorInfo, dopplerConfig)
%applyDopplerFade  Reduce detection probability for targets with low radial velocity.
%
%  Real pulse-Doppler radars use MTI (Moving Target Indication) or Doppler
%  processing to separate targets from ground clutter. Targets moving
%  tangentially (across the beam) have near-zero radial velocity and fall
%  into the clutter notch — they become invisible even at close range.
%
%  This function models that effect by computing each target's radial
%  velocity relative to the radar and reducing Pd when it's below the
%  Minimum Detectable Velocity (MDV).
%
%  PHYSICS
%    v_radial = dot(v_target, unit_vector_from_radar_to_target)
%    If |v_radial| < MDV → target is in the clutter notch
%    Pd scales linearly from 0 at v_radial=0 to 1.0 at v_radial=MDV
%
%    MDV depends on radar wavelength and PRF. For a typical S-band PSR
%    (2.8 GHz, PRF ~1000 Hz): MDV ≈ 30-50 m/s
%
%  REFERENCES
%    [1] MathWorks, "Generate Clutter and Target Returns for MTI Radar":
%        https://www.mathworks.com/help/radar/ug/generate-clutter-and-target-returns-for-mti-radar.html
%    [2] MathWorks, "Doppler Estimation":
%        https://www.mathworks.com/help/radar/ug/doppler-estimation.html
%        "Since the third target is moving along the tangential direction,
%        there is no velocity component in the radial direction. Therefore,
%        the radar cannot detect the Doppler shift of the third target."
%    [3] Skolnik, M., "Introduction to Radar Systems," 3rd ed., McGraw-Hill.
%
%  INPUTS
%    dets          : cell array of objectDetection
%    sensorPos     : [x,y,z] radar position (meters, NED)
%    targets       : struct array with Position and Velocity fields
%    sensorInfo    : sensor metadata (for frequency → wavelength → MDV)
%    dopplerConfig : struct with optional fields:
%                      mdv_ms     : minimum detectable velocity (m/s), default=40
%                      enabled    : boolean, default=true
%
%  OUTPUTS
%    keptDets : cell array of surviving detections (tangential targets dropped)
%    diagInfo : struct with nDropped, radialVelocities
%
%  See also: applyRainDegradation, runDetections

    diagInfo = struct('nDropped', 0, 'radialVelocities', []);

    if isempty(dets) || isempty(targets)
        keptDets = dets;
        return;
    end

    % Parse config
    mdv = 40;  % default MDV: 40 m/s (typical S-band PSR)
    if nargin >= 5 && isstruct(dopplerConfig)
        if isfield(dopplerConfig, 'mdv_ms'); mdv = dopplerConfig.mdv_ms; end
    end

    % Compute MDV from frequency if not explicitly set
    % MDV ≈ wavelength * PRF / 4 (for 2-pulse canceller)
    % For S-band (2.8 GHz), λ ≈ 0.107m, PRF ≈ 1000 Hz → MDV ≈ 27 m/s
    % For X-band (9 GHz), λ ≈ 0.033m, PRF ≈ 1000 Hz → MDV ≈ 8 m/s
    if nargin >= 4 && isfield(sensorInfo, 'radarFreq') && sensorInfo.radarFreq > 0
        freq = sensorInfo.radarFreq;
        lambda = physconst('LightSpeed') / freq;
        prf_assumed = 1000;  % typical PSR PRF
        mdv_computed = lambda * prf_assumed / 4;
        % Use the more conservative (larger) of computed vs config
        mdv = max(mdv, mdv_computed);
    end

    % Build a map of target positions and velocities for lookup
    nTargets = numel(targets);
    tgtPos = zeros(nTargets, 3);
    tgtVel = zeros(nTargets, 3);
    for k = 1:nTargets
        tgtPos(k,:) = reshape(targets(k).Position, 1, []);
        if isfield(targets(k), 'Velocity') || isprop(targets(k), 'Velocity')
            tgtVel(k,:) = reshape(targets(k).Velocity, 1, []);
        end
    end

    keepMask = true(numel(dets), 1);
    radVels = zeros(numel(dets), 1);

    for ii = 1:numel(dets)
        detPos = dets{ii}.Measurement(1:3)';

        % Find closest target to this detection
        dists = vecnorm(tgtPos - detPos, 2, 2);
        [~, closestIdx] = min(dists);

        vel = tgtVel(closestIdx, :);
        pos = tgtPos(closestIdx, :);

        % Radial velocity: projection of velocity onto radar-to-target direction
        toTarget = pos - sensorPos(:)';
        rangeToTarget = norm(toTarget);
        if rangeToTarget < 1; continue; end  % skip if on top of radar

        unitRadial = toTarget / rangeToTarget;
        vRadial = dot(vel, unitRadial);
        radVels(ii) = vRadial;

        % Pd model: linear ramp from 0 at v_radial=0 to 1.0 at v_radial=MDV
        % Targets with |v_radial| >= MDV: full Pd (no effect)
        % Targets with |v_radial| < MDV: reduced Pd proportional to |v_radial|/MDV
        absVr = abs(vRadial);
        if absVr < mdv
            pdFade = absVr / mdv;
            pdFade = max(pdFade, 0.05);  % never completely zero (sidelobe leakage)
            if rand() > pdFade
                keepMask(ii) = false;
            end
        end
    end

    diagInfo.nDropped = sum(~keepMask);
    diagInfo.radialVelocities = radVels;
    keptDets = dets(keepMask);
end