function [radar, meta] = buildCustomFusionRadarSensor(sensorIndex, varargin)
%buildCustomFusionRadarSensor  Configure a fusionRadarSensor with a single, consistent spec.
%
% This is NOT a new radar model. It is a convenience wrapper around
% fusionRadarSensor so the project has ONE place to define radar settings.
%
% USAGE
%   radar = buildCustomFusionRadarSensor(1);
%   radar = buildCustomFusionRadarSensor(1,'rpm',25,'fov',[1.5;10],'pd',0.9);
%
% Name-Value parameters (project-facing)
%   rpm           : mechanical rotation rate (rev/min). Default 25.
%   fov           : [az; el] degrees. Default [1.5;10].
%   sector        : mechanical azimuth limits [min max] deg. Default [250 290].
%   tilt          : elevation tilt offset (deg). Default 2.
%   pd            : detection probability. Default 0.9.
%   far           : false alarm rate (per m^3). Default 1e-6.
%   rangeLimits   : [min max] meters. Default [0 111e3].
%   rangeRes      : range resolution (m). Default 135.
%   refRange      : reference range (m). Default 111e3.
%   refRCS        : reference RCS (dBsm). Default 0.
%   hasINS        : whether INS is enabled. Default true.
%   detCoords     : detection coordinates. Default 'Scenario'.
%
% OUTPUTS
%   radar : fusionRadarSensor
%   meta  : struct of derived values (scanrate, updaterate, etc.)

arguments
    sensorIndex (1,1) double {mustBeFinite,mustBePositive}
end
arguments (Repeating)
    varargin
end

p = inputParser;
addParameter(p,'rpm',25,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'fov',[1.4;30],@(x)isnumeric(x)&&numel(x)==2);
addParameter(p,'sector',[250 290],@(x)isnumeric(x)&&numel(x)==2);
addParameter(p,'tilt',2,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'pd',0.9,@(x)isnumeric(x)&&isscalar(x)&&x>=0&&x<=1);
addParameter(p,'far',1e-6,@(x)isnumeric(x)&&isscalar(x)&&x>=0);
addParameter(p,'rangeLimits',[0 111e3],@(x)isnumeric(x)&&numel(x)==2);
addParameter(p,'rangeRes',135,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'refRange',111e3,@(x)isnumeric(x)&&isscalar(x)&&x>0);
addParameter(p,'refRCS',0,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'hasINS',true,@(x)islogical(x)&&isscalar(x));
addParameter(p,'detCoords','Scenario',@(x)ischar(x)||isstring(x));
parse(p,varargin{:});
S = p.Results;

fov = S.fov(:);
rpm = S.rpm;
scanrate   = rpm*360/60;      % deg/s
updaterate = scanrate/fov(1); % Hz (updates per azimuth resolution cell)

% Create sensor
radar = fusionRadarSensor(sensorIndex,'Rotator');

% --- Set properties with version-robust guards ---
if isprop(radar,'UpdateRate'); radar.UpdateRate = updaterate; end

% FOV
if isprop(radar,'FieldOfView')
    radar.FieldOfView = fov;
elseif isprop(radar,'FieldOfViewAngles')
    radar.FieldOfViewAngles = fov;
end

% Scan rate / sector
if isprop(radar,'MaxAzimuthScanRate'); radar.MaxAzimuthScanRate = scanrate; end
if isprop(radar,'AzimuthResolution');  radar.AzimuthResolution  = fov(1); end
% Only restrict azimuth for sectors narrower than 360 deg.
% Setting [0 360] explicitly on a Rotator breaks the scan scheduler (zero detections).
sectorSpan = abs(diff(S.sector));
if isprop(radar,'MechanicalAzimuthLimits') && sectorSpan < 359
    radar.MechanicalAzimuthLimits = S.sector;
end

% Range
if isprop(radar,'RangeLimits')
    radar.RangeLimits = S.rangeLimits;
elseif isprop(radar,'MaxRange')
    radar.MaxRange = S.rangeLimits(2);
elseif isprop(radar,'MaxUnambiguousRange')
    radar.MaxUnambiguousRange = S.rangeLimits(2);
end

% Range resolution / reference values
if isprop(radar,'RangeResolution'); radar.RangeResolution = S.rangeRes; end
if isprop(radar,'ReferenceRange');  radar.ReferenceRange  = S.refRange; end
if isprop(radar,'ReferenceRCS');    radar.ReferenceRCS    = S.refRCS; end

% INS + coordinates
if isprop(radar,'HasINS'); radar.HasINS = S.hasINS; end
if isprop(radar,'DetectionCoordinates'); radar.DetectionCoordinates = char(S.detCoords); end

% Probabilities
if isprop(radar,'DetectionProbability'); radar.DetectionProbability = S.pd; end
if isprop(radar,'FalseAlarmRate');       radar.FalseAlarmRate       = S.far; end

% Elevation
if isprop(radar,'HasElevation'); radar.HasElevation = true; end
if isprop(radar,'MechanicalElevationLimits')
    % Relative to mount; match your current pattern: [-el 0] minus a small tilt
    radar.MechanicalElevationLimits = [-fov(2) 0] - S.tilt;
end
if isprop(radar,'FieldOfView')
    radar.FieldOfView(2) = fov(2) + 1e-3; % epsilon prevents edge issues
end

% Mounting location: keep consistent with your existing file
if isprop(radar,'MountingLocation')
    radar.MountingLocation = [0 0 -15];
end

% Metadata (handy for logs/sweeps)
meta = struct();
meta.sensorIndex = sensorIndex;
meta.rpm = rpm;
meta.fov = fov;
meta.scanrate_degps = scanrate;
meta.updaterate_hz = updaterate;
meta.sector_deg = S.sector;
meta.rangeLimits_m = S.rangeLimits;
meta.rangeResolution_m = S.rangeRes;
meta.pd = S.pd;
meta.far = S.far;
meta.detCoords = char(S.detCoords);
end