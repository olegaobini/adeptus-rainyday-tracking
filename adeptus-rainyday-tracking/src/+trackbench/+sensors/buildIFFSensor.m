function [mssr, meta] = buildIFFSensor(sensorIndex, varargin)
%buildIFFSensor  Simulate the DASR Monopulse Secondary Surveillance Radar (MSSR).
%
% DASR ARCHITECTURE (ASR-11 / AN/GPN-30)
%   The real DASR has two co-mounted, co-rotating antennas on the same tower:
%     PSR  - Primary Search Radar:      60 nm, S-band, passive reflection
%     MSSR - Monopulse SSR (this file): 120 nm, interrogates transponders
%
%   Key distinction from the old "No scanning / 360 FOV" IFF model:
%     The MSSR antenna is physically mounted on TOP of the PSR antenna and
%     rotates WITH it at the same 12.5 RPM. It does NOT interrogate
%     omnidirectionally — it sweeps a narrow beam just like the PSR, but
%     at twice the range and with identity-tagged transponder replies.
%
%   This means:
%     - Same RPM and update rate as PSR (12.5 RPM)
%     - Same 360-degree rotator scan pattern
%     - Narrow 1.4 deg azimuth beam (monopulse for accurate az)
%     - Wider elevation coverage (~10 deg) to catch aircraft at all
%       altitudes out to 120 nm
%     - Each transponder reply tagged with squawk identity (ObjectClassID)
%     - Very high Pd (0.99) - transponders actively reply
%     - Minimum FAR (1e-7, MATLAB floor) - no weather clutter on SSR
%
% REAL MSSR PARAMETERS (ASR-11)
%   Rotation:       12.5 RPM (co-mounted with PSR)
%   Az beamwidth:   ~1.4 deg (monopulse)
%   El coverage:    ~10 deg
%   Range:          120 nm = 222,240 m
%   Range res:      ~100 m (transponder reply timing)
%   Pd:             ~0.99
%   FAR:            ~0 (clamped to MATLAB min 1e-7)
%
% USAGE
%   [mssr, meta] = buildIFFSensor(3);
%   [mssr, meta] = buildIFFSensor(3, 'rpm', 12.5, 'pd', 0.99);
%
% NAME-VALUE PARAMETERS
%   rpm          : rotation rate (rev/min). Default 12.5.
%   fov          : [az; el] beamwidth (deg). Default [1.4; 10].
%   tilt         : elevation tilt offset (deg). Default 2.
%   pd           : transponder reply probability. Default 0.99.
%   rangeLimits  : [min max] meters. Default [0 222240] (120 nm).
%   rangeRes     : range resolution (m). Default 100.
%   refRange     : reference range (m). Default 222240.
%
% OUTPUTS
%   mssr : fusionRadarSensor configured as DASR MSSR
%   meta : struct of derived parameters for logging

p = inputParser;
addParameter(p, 'rpm',         12.5,       @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, 'fov',         [1.4; 10],  @(x)isnumeric(x)&&numel(x)==2);
addParameter(p, 'tilt',        2,          @(x)isnumeric(x)&&isscalar(x));
addParameter(p, 'pd',          0.99,       @(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
addParameter(p, 'rangeLimits', [0 222240], @(x)isnumeric(x)&&numel(x)==2);
addParameter(p, 'rangeRes',    100,        @(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p, 'refRange',    222240,     @(x)isnumeric(x)&&isscalar(x)&&x>0);
parse(p, varargin{:});
S = p.Results;

fov        = S.fov(:);
rpm        = S.rpm;
scanrate   = rpm * 360 / 60;    % deg/s  (12.5 rpm -> 75 deg/s)
updaterate = scanrate / fov(1); % Hz     (75 / 1.4 -> ~53.6 Hz)

% Rotator - MSSR co-rotates with PSR, identical scan geometry
mssr = fusionRadarSensor(sensorIndex, 'Rotator');

% Rotation rate (must match PSR exactly so scans are synchronised)
if isprop(mssr, 'UpdateRate');         mssr.UpdateRate         = updaterate; end
if isprop(mssr, 'MaxAzimuthScanRate'); mssr.MaxAzimuthScanRate = scanrate;   end
if isprop(mssr, 'AzimuthResolution');  mssr.AzimuthResolution  = fov(1);     end

% Full 360-degree sweep (no sector restriction, unlike PSR in previous config)
if isprop(mssr, 'MechanicalAzimuthLimits')
    mssr.MechanicalAzimuthLimits = [0 360];
end

% FOV
if isprop(mssr, 'FieldOfView'); mssr.FieldOfView = fov; end

% Elevation: wider than PSR to capture high-altitude targets at 120 nm
if isprop(mssr, 'HasElevation'); mssr.HasElevation = true; end
if isprop(mssr, 'MechanicalElevationLimits')
    mssr.MechanicalElevationLimits = [-fov(2) 0] - S.tilt;
end
if isprop(mssr, 'FieldOfView')
    mssr.FieldOfView(2) = fov(2) + 1e-3;
end

% Range: 120 nm
if isprop(mssr, 'RangeLimits');     mssr.RangeLimits     = S.rangeLimits; end
if isprop(mssr, 'RangeResolution'); mssr.RangeResolution = S.rangeRes;    end
if isprop(mssr, 'ReferenceRange');  mssr.ReferenceRange  = S.refRange;    end

% Transponder Pd and minimum FAR (MATLAB clamps at 1e-7)
if isprop(mssr, 'DetectionProbability'); mssr.DetectionProbability = S.pd;  end
if isprop(mssr, 'FalseAlarmRate');       mssr.FalseAlarmRate       = 1e-7;  end

% Scenario coordinates + INS (same as PSR)
if isprop(mssr, 'DetectionCoordinates'); mssr.DetectionCoordinates = 'Scenario'; end
if isprop(mssr, 'HasINS');               mssr.HasINS               = true;       end

% Co-mounted at same height as PSR
if isprop(mssr, 'MountingLocation'); mssr.MountingLocation = [0 0 -15]; end

% High reference RCS so MSSR range equation doesn't limit detections
% (real MSSR uses transponder SNR, not target RCS)
if isprop(mssr, 'ReferenceRCS'); mssr.ReferenceRCS = 20; end  % dBsm

% Metadata
meta = struct();
meta.sensorIndex    = sensorIndex;
meta.type           = 'MSSR';
meta.rpm            = rpm;
meta.fov            = fov;
meta.scanrate_degps = scanrate;
meta.updaterate_hz  = updaterate;
meta.rangeLimits_m  = S.rangeLimits;
meta.rangeRes_m     = S.rangeRes;
meta.pd             = S.pd;
meta.far            = 1e-7;

fprintf('[MSSR] Sensor %d: %.1f RPM | Az=%.1f deg | El=%.1f deg | Range=%.0f nm | Pd=%.2f | UpdateRate=%.1f Hz\n', ...
    sensorIndex, rpm, fov(1), fov(2), S.rangeLimits(2)/1852, S.pd, updaterate);
end
