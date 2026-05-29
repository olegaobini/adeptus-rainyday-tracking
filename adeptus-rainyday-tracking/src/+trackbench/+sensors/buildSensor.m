function [sensor, meta] = buildSensor(sensorIndex, sensorType, varargin)
%buildSensor  Universal sensor factory for the tracking simulation.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
% PURPOSE
%   Single entry point to create ANY sensor supported by the MATLAB Sensor
%   Fusion and Tracking Toolbox (R2025b). Returns a configured sensor object
%   and a metadata struct for logging. Each sensor type has realistic
%   defaults that can be overridden via name-value pairs.
%
% SUPPORTED SENSOR TYPES (sensorType string)
% ───────────────────────────────────────────────────────────────────────
%   RADAR FAMILY (fusionRadarSensor / radarDataGenerator)
%     'PSR'          Primary Search Radar (rotating, passive)
%     'SSR'          Secondary Surveillance Radar / MSSR (transponder)
%     'ASR'          Airport Surveillance Radar (combined PSR defaults)
%     'ARSR'         Air Route Surveillance Radar (long range)
%     'PAR'          Precision Approach Radar (sector scan)
%     'TWS'          Track-While-Scan phased array (no scanning)
%     'AESA'         Active Electronically Scanned Array (sector)
%     'FIRE_CONTROL' Fire control radar (narrow beam, no scanning)
%     'WEATHER'      Weather radar (high RPM, raster)
%     'MARITIME'     Maritime / surface search radar (X-band rotator)
%     'CUSTOM_RADAR' Bare fusionRadarSensor with user-specified params
%
%   INFRARED FAMILY (irSensor)
%     'IRST'         Infrared Search & Track (rotator)
%     'IR_STARING'   Staring IR sensor (no scanning)
%     'FLIR'         Forward-Looking Infrared (sector)
%     'CUSTOM_IR'    Bare irSensor with user-specified params
%
%   SONAR FAMILY (sonarSensor)
%     'ACTIVE_SONAR'  Active sonar (rotator)
%     'PASSIVE_SONAR' Passive sonar (no scanning)
%     'TOWED_ARRAY'   Towed array sonar (passive, sector)
%     'CUSTOM_SONAR'  Bare sonarSensor with user-specified params
%
%   LIDAR (monostaticLidarSensor)
%     'LIDAR'        Monostatic lidar point cloud generator
%     'CUSTOM_LIDAR' Bare monostaticLidarSensor with user-specified params
%
%   ADS-B (adsbTransponder + adsbReceiver)
%     'ADSB_TX'      ADS-B transponder (attach to aircraft platform)
%     'ADSB_RX'      ADS-B receiver (attach to ground station)
%
%   CUSTOM / TEMPLATE
%     'CUSTOM'       Returns a template struct — see "Custom Sensors" below
%
% See also: fusionRadarSensor, radarDataGenerator, irSensor, sonarSensor,
%           monostaticLidarSensor, adsbTransponder, adsbReceiver

arguments
    sensorIndex (1,1) double {mustBeFinite, mustBePositive}
    sensorType  (1,1) string
end
arguments (Repeating)
    varargin
end

sensorType = upper(sensorType);

switch sensorType
    case 'PSR'
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            getDefaults('PSR'), varargin{:});
        meta.type = 'PSR';
        meta.description = 'Primary Search Radar (rotating, passive reflection)';

    case {'SSR', 'MSSR', 'IFF'}
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            getDefaults('SSR'), varargin{:});
        meta.type = 'SSR/MSSR';
        meta.description = 'Secondary Surveillance Radar (transponder interrogation)';

    case 'ASR'
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            getDefaults('ASR'), varargin{:});
        meta.type = 'ASR';
        meta.description = 'Airport Surveillance Radar (S-band, 60nm)';

    case 'ARSR'
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            getDefaults('ARSR'), varargin{:});
        meta.type = 'ARSR';
        meta.description = 'Air Route Surveillance Radar (L-band, 250nm)';

    case 'PAR'
        [sensor, meta] = buildRadar(sensorIndex, 'Sector', ...
            getDefaults('PAR'), varargin{:});
        meta.type = 'PAR';
        meta.description = 'Precision Approach Radar (sector scan)';

    case 'TWS'
        [sensor, meta] = buildRadar(sensorIndex, 'No scanning', ...
            getDefaults('TWS'), varargin{:});
        meta.type = 'TWS';
        meta.description = 'Track-While-Scan phased array';

    case 'AESA'
        [sensor, meta] = buildRadar(sensorIndex, 'No scanning', ...
            getDefaults('AESA'), varargin{:});
        meta.type = 'AESA';
        meta.description = 'Active Electronically Scanned Array';

    case 'FIRE_CONTROL'
        [sensor, meta] = buildRadar(sensorIndex, 'No scanning', ...
            getDefaults('FIRE_CONTROL'), varargin{:});
        meta.type = 'FIRE_CONTROL';
        meta.description = 'Fire control radar (narrow beam, dedicated track)';

    case 'WEATHER'
        [sensor, meta] = buildRadar(sensorIndex, 'Raster', ...
            getDefaults('WEATHER'), varargin{:});
        meta.type = 'WEATHER';
        meta.description = 'Weather surveillance radar (high RPM, raster scan)';

    case 'MARITIME'
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            getDefaults('MARITIME'), varargin{:});
        meta.type = 'MARITIME';
        meta.description = 'Maritime / surface search radar (X-band)';

    case 'CUSTOM_RADAR'
        [sensor, meta] = buildRadar(sensorIndex, 'Rotator', ...
            struct(), varargin{:});
        meta.type = 'CUSTOM_RADAR';
        meta.description = 'User-configured fusionRadarSensor';

    case 'IRST'
        [sensor, meta] = buildIR(sensorIndex, 'Rotator', ...
            getDefaults('IRST'), varargin{:});
        meta.type = 'IRST';
        meta.description = 'Infrared Search & Track (passive, rotating)';

    case 'IR_STARING'
        [sensor, meta] = buildIR(sensorIndex, 'No scanning', ...
            getDefaults('IR_STARING'), varargin{:});
        meta.type = 'IR_STARING';
        meta.description = 'Staring IR sensor (fixed wide FOV)';

    case 'FLIR'
        [sensor, meta] = buildIR(sensorIndex, 'Sector', ...
            getDefaults('FLIR'), varargin{:});
        meta.type = 'FLIR';
        meta.description = 'Forward-Looking Infrared (sector scan)';

    case 'CUSTOM_IR'
        [sensor, meta] = buildIR(sensorIndex, 'No scanning', ...
            struct(), varargin{:});
        meta.type = 'CUSTOM_IR';
        meta.description = 'User-configured irSensor';

    case 'ACTIVE_SONAR'
        [sensor, meta] = buildSonarSensor(sensorIndex, 'No scanning', ...
            getDefaults('ACTIVE_SONAR'), varargin{:});
        meta.type = 'ACTIVE_SONAR';
        meta.description = 'Active sonar (ping + listen)';

    case 'PASSIVE_SONAR'
        [sensor, meta] = buildSonarSensor(sensorIndex, 'No scanning', ...
            getDefaults('PASSIVE_SONAR'), varargin{:});
        meta.type = 'PASSIVE_SONAR';
        meta.description = 'Passive sonar (listen only)';

    case 'TOWED_ARRAY'
        [sensor, meta] = buildSonarSensor(sensorIndex, 'Sector', ...
            getDefaults('TOWED_ARRAY'), varargin{:});
        meta.type = 'TOWED_ARRAY';
        meta.description = 'Towed array sonar (passive, sector)';

    case 'CUSTOM_SONAR'
        [sensor, meta] = buildSonarSensor(sensorIndex, 'No scanning', ...
            struct(), varargin{:});
        meta.type = 'CUSTOM_SONAR';
        meta.description = 'User-configured sonarSensor';

    case 'LIDAR'
        [sensor, meta] = buildLidar(sensorIndex, getDefaults('LIDAR'), varargin{:});
        meta.type = 'LIDAR';
        meta.description = 'Monostatic lidar point cloud generator';

    case 'CUSTOM_LIDAR'
        [sensor, meta] = buildLidar(sensorIndex, struct(), varargin{:});
        meta.type = 'CUSTOM_LIDAR';
        meta.description = 'User-configured monostaticLidarSensor';

    case 'ADSB_TX'
        [sensor, meta] = buildADSBTx(sensorIndex, varargin{:});

    case 'ADSB_RX'
        [sensor, meta] = buildADSBRx(sensorIndex, varargin{:});

    case 'CUSTOM'
        [sensor, meta] = getCustomTemplate(sensorIndex);

    otherwise
        error('buildSensor:unknownType', ...
            ['Unknown sensor type: "%s".\n' ...
             'Supported types: PSR, SSR, ASR, ARSR, PAR, TWS, AESA, FIRE_CONTROL,\n' ...
             'WEATHER, MARITIME, CUSTOM_RADAR, IRST, IR_STARING, FLIR, CUSTOM_IR,\n' ...
             'ACTIVE_SONAR, PASSIVE_SONAR, TOWED_ARRAY, CUSTOM_SONAR,\n' ...
             'LIDAR, CUSTOM_LIDAR, ADSB_TX, ADSB_RX, CUSTOM'], sensorType);
end

meta.sensorIndex = sensorIndex;
fprintf('[buildSensor] Created %s (index=%d) | %s\n', ...
    meta.type, sensorIndex, meta.description);
end


%% ========================================================================
%                         DEFAULT PARAMETER SETS
%% ========================================================================
function D = getDefaults(sensorType)
    switch upper(sensorType)
        case 'PSR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 30];
            D.tilt         = 2;
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 111120];
            D.rangeRes     = 93;
            D.refRange     = 111120;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.centerFreq   = 2.8e9;
        case 'SSR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 10];
            D.tilt         = 2;
            D.pd           = 0.99;
            D.far          = 1e-7;
            D.rangeLimits  = [0 222240];
            D.rangeRes     = 100;
            D.refRange     = 222240;
            D.refRCS       = 20;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.centerFreq   = 1.06e9;
        case 'ASR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 5];
            D.tilt         = 2;
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 111120];
            D.rangeRes     = 93;
            D.refRange     = 111120;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;
        case 'ARSR'
            D.rpm          = 5;
            D.fov          = [1.5; 20];
            D.tilt         = 0;
            D.pd           = 0.85;
            D.far          = 1e-6;
            D.rangeLimits  = [0 463000];
            D.rangeRes     = 250;
            D.refRange     = 463000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;
        case 'PAR'
            D.rpm          = 0;
            D.fov          = [1.0; 1.0];
            D.sector       = [170 190];
            D.tilt         = -3;
            D.pd           = 0.95;
            D.far          = 1e-7;
            D.rangeLimits  = [0 37040];
            D.rangeRes     = 15;
            D.refRange     = 37040;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.updateRate   = 60;
        case 'TWS'
            D.fov          = [60; 30];
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 200000];
            D.rangeRes     = 150;
            D.refRange     = 200000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 2;
        case 'AESA'
            D.fov          = [3; 3];
            D.sector       = [-60 60];
            D.pd           = 0.95;
            D.far          = 1e-7;
            D.rangeLimits  = [0 300000];
            D.rangeRes     = 30;
            D.refRange     = 300000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 20;
        case 'FIRE_CONTROL'
            D.fov          = [2; 2];
            D.pd           = 0.98;
            D.far          = 1e-7;
            D.rangeLimits  = [0 150000];
            D.rangeRes     = 10;
            D.refRange     = 150000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 50;
        case 'WEATHER'
            D.rpm          = 25;
            D.fov          = [1.0; 1.0];
            D.tilt         = 0;
            D.pd           = 0.7;
            D.far          = 1e-3;
            D.rangeLimits  = [0 463000];
            D.rangeRes     = 250;
            D.refRange     = 463000;
            D.refRCS       = -10;
            D.hasElevation = true;
            D.hasRangeRate = true;
        case 'MARITIME'
            D.rpm          = 24;
            D.fov          = [1.2; 25];
            D.tilt         = 0;
            D.pd           = 0.85;
            D.far          = 1e-5;
            D.rangeLimits  = [0 74080];
            D.rangeRes     = 25;
            D.refRange     = 74080;
            D.refRCS       = -5;
            D.hasElevation = false;
            D.hasRangeRate = false;
            D.centerFreq   = 9.4e9;
        case 'IRST'
            D.rpm          = 60;
            D.fov          = [2; 5];
            D.pd           = 0.8;
            D.far          = 1e-6;
            D.rangeLimits  = [0 100000];
            D.rangeRes     = 500;
            D.hasElevation = true;
            D.hasRangeRate = false;
        case 'IR_STARING'
            D.fov          = [90; 60];
            D.pd           = 0.7;
            D.far          = 1e-5;
            D.rangeLimits  = [0 50000];
            D.rangeRes     = 1000;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.updateRate   = 30;
        case 'FLIR'
            D.fov          = [10; 8];
            D.sector       = [-20 20];
            D.pd           = 0.85;
            D.far          = 1e-6;
            D.rangeLimits  = [0 30000];
            D.rangeRes     = 200;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.updateRate   = 30;
        case 'ACTIVE_SONAR'
            D.rpm          = 0;
            D.fov          = [360; 90];
            D.far          = 1e-4;
            D.rangeLimits  = [0 20000];
            D.rangeRes     = 50;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.detectionMode = 'monostatic';
        case 'PASSIVE_SONAR'
            D.fov          = [360; 20];
            D.pd           = 0.5;
            D.far          = 1e-4;
            D.rangeLimits  = [0 50000];
            D.rangeRes     = 500;
            D.hasElevation = false;
            D.hasRangeRate = false;
            D.updateRate   = 1;
        case 'TOWED_ARRAY'
            D.fov          = [120; 10];
            D.sector       = [120 240];
            D.pd           = 0.6;
            D.far          = 1e-4;
            D.rangeLimits  = [0 80000];
            D.rangeRes     = 300;
            D.hasElevation = false;
            D.hasRangeRate = false;
            D.updateRate   = 0.5;
        case 'LIDAR'
            D.maxRange          = 200;
            D.rangeAccuracy     = 0.02;
            D.azResolution      = 0.2;
            D.elResolution      = 0.4;
            D.azLimits          = [-180 180];
            D.elLimits          = [-25 15];
            D.updateRate        = 10;
        otherwise
            D = struct();
    end
end


%% ========================================================================
%                         RADAR BUILDER
%% ========================================================================
function [radar, meta] = buildRadar(idx, scanConfig, defaults, varargin)
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'rpm',          getOr(defaults,'rpm',12.5));
    addParameter(p, 'fov',          getOr(defaults,'fov',[1.4;10]));
    addParameter(p, 'sector',       getOr(defaults,'sector',[0 360]));
    addParameter(p, 'tilt',         getOr(defaults,'tilt',0));
    addParameter(p, 'pd',           getOr(defaults,'pd',0.9));
    addParameter(p, 'far',          getOr(defaults,'far',1e-6));
    addParameter(p, 'rangeLimits',  getOr(defaults,'rangeLimits',[0 111120]));
    addParameter(p, 'rangeRes',     getOr(defaults,'rangeRes',100));
    addParameter(p, 'refRange',     getOr(defaults,'refRange',111120));
    addParameter(p, 'refRCS',       getOr(defaults,'refRCS',0));
    addParameter(p, 'hasElevation', getOr(defaults,'hasElevation',true));
    addParameter(p, 'hasRangeRate', getOr(defaults,'hasRangeRate',false));
    addParameter(p, 'hasINS',       true);
    addParameter(p, 'detCoords',    'Scenario');
    addParameter(p, 'mountingLoc',  [0 0 -15]);
    addParameter(p, 'updateRate',   getOr(defaults,'updateRate',[]));
    addParameter(p, 'centerFreq',   getOr(defaults,'centerFreq',0));
    parse(p, varargin{:});
    S = p.Results;

    fov = S.fov(:);
    rpm = S.rpm;

    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;
        updaterate = scanrate / fov(1);
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 1;
        scanrate   = fov(1);
    end

    radar = fusionRadarSensor(idx, scanConfig);

    safeSet(radar, 'UpdateRate',            updaterate);
    safeSet(radar, 'FieldOfView',           fov);
    safeSet(radar, 'MaxAzimuthScanRate',    scanrate);
    safeSet(radar, 'AzimuthResolution',     fov(1));
    safeSet(radar, 'HasElevation',          S.hasElevation);
    safeSet(radar, 'HasRangeRate',          S.hasRangeRate);

    % Azimuth + elevation pointing.
    %  Rotator/raster: scan a wide swath; tilt offsets the elevation band
    %  (UNCHANGED - the PosterDemo canary depends on this exact behavior).
    %  Sector: in R2025b a mechanical sector that crosses +/-180 deg azimuth,
    %  or whose mechanical-elevation band is offset off boresight, returns NO
    %  detections. So steer the sector via MountingAngles ([z-yaw, y-pitch,
    %  x-roll]): yaw = sector center, pitch = -tilt (tilt<0 points the beam
    %  up), and keep the mechanical scan limits SYMMETRIC about boresight.
    %  Verified empirically + R2025b fusionRadarSensor MountingAngles/ScanMode
    %  docs. drawSensorCoverage and computeSensorCoverageVolume re-add the
    %  mounting yaw, so the drawn wedge still matches the configured sector.
    if strcmpi(scanConfig, 'Sector')
        azCenter = mean(S.sector);
        azHalf   = abs(diff(S.sector)) / 2;
        safeSet(radar, 'MountingAngles', [azCenter, -S.tilt, 0]);
        if azHalf > 0 && azHalf < 180
            safeSet(radar, 'MechanicalAzimuthLimits', [-azHalf  azHalf]);
        end
        if S.hasElevation
            safeSet(radar, 'MechanicalElevationLimits', [-fov(2)/2  fov(2)/2]);
        end
    else
        sectorSpan = abs(diff(S.sector));
        if sectorSpan < 359 && sectorSpan > 0
            safeSet(radar, 'MechanicalAzimuthLimits', S.sector);
        end
        % Elevation limits - tilt offsets the band. Rotators tolerate the
        % asymmetry; this preserves existing PSR/SSR detection behavior.
        if S.hasElevation
            safeSet(radar, 'MechanicalElevationLimits', [-fov(2)/2  fov(2)/2] - S.tilt);
        end
    end

    % FOV epsilon — prevents edge-case missed detections at FOV boundary
    safeSet(radar, 'FieldOfView', [fov(1); fov(2) + 1e-3]);

    safeSet(radar, 'RangeLimits',       S.rangeLimits);
    safeSet(radar, 'RangeResolution',   S.rangeRes);
    safeSet(radar, 'ReferenceRange',    S.refRange);
    safeSet(radar, 'ReferenceRCS',      S.refRCS);

    if S.centerFreq > 0
        safeSet(radar, 'CenterFrequency', S.centerFreq);
    end

    safeSet(radar, 'DetectionProbability', S.pd);
    safeSet(radar, 'FalseAlarmRate',       S.far);
    safeSet(radar, 'HasRCSSignature',      true);
    safeSet(radar, 'HasINS',               S.hasINS);
    safeSet(radar, 'DetectionCoordinates',  char(S.detCoords));
    safeSet(radar, 'MountingLocation',      S.mountingLoc);

    applyUnmatched(radar, p.Unmatched);

    meta = struct();
    meta.rpm            = rpm;
    meta.fov            = fov;
    meta.scanrate_degps = scanrate;
    meta.updaterate_hz  = updaterate;
    meta.sector_deg     = S.sector;
    meta.rangeLimits_m  = S.rangeLimits;
    meta.rangeRes_m     = S.rangeRes;
    meta.refRange_m     = S.refRange;
    meta.refRCS_dBsm    = S.refRCS;
    meta.pd             = S.pd;
    meta.far            = S.far;
    meta.hasElevation   = S.hasElevation;
    meta.hasRangeRate   = S.hasRangeRate;
    meta.mountingLoc    = S.mountingLoc;
    if S.centerFreq > 0
        meta.centerFreq_Hz = S.centerFreq;
        meta.radarFreq     = S.centerFreq;
    end
end


%% ========================================================================
%                         IR SENSOR BUILDER
%% ========================================================================
function [ir, meta] = buildIR(idx, scanConfig, defaults, varargin)
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'rpm',          getOr(defaults,'rpm',60));
    addParameter(p, 'fov',          getOr(defaults,'fov',[2;5]));
    addParameter(p, 'sector',       getOr(defaults,'sector',[0 360]));
    addParameter(p, 'pd',           getOr(defaults,'pd',0.8));
    addParameter(p, 'far',          getOr(defaults,'far',1e-6));
    addParameter(p, 'rangeLimits',  getOr(defaults,'rangeLimits',[0 100000]));
    addParameter(p, 'rangeRes',     getOr(defaults,'rangeRes',500));
    addParameter(p, 'hasElevation', getOr(defaults,'hasElevation',true));
    addParameter(p, 'hasRangeRate', getOr(defaults,'hasRangeRate',false));
    addParameter(p, 'hasINS',       true);
    addParameter(p, 'mountingLoc',  [0 0 -15]);
    addParameter(p, 'updateRate',   getOr(defaults,'updateRate',[]));
    addParameter(p, 'focalLength',  getOr(defaults,'focalLength',800));
    addParameter(p, 'lensDiameter', getOr(defaults,'lensDiameter',0.08));
    addParameter(p, 'numDetectors', getOr(defaults,'numDetectors',[]));
    parse(p, varargin{:});
    S = p.Results;

    fov = S.fov(:); rpm = S.rpm;

    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;
        updaterate = scanrate / max(fov(1), 0.01);
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 30; scanrate = 0;
    end

    ir = irSensor(idx, scanConfig);
    safeSet(ir, 'UpdateRate',    updaterate);
    safeSet(ir, 'HasElevation',  S.hasElevation);
    safeSet(ir, 'HasINS',        S.hasINS);
    safeSet(ir, 'MountingLocation', S.mountingLoc);
    safeSet(ir, 'FocalLength',   S.focalLength);
    safeSet(ir, 'LensDiameter',  S.lensDiameter);

    if isempty(S.numDetectors)
        nAz = max(10, round(deg2rad(fov(1)) * S.focalLength));
        nEl = max(10, round(deg2rad(fov(2)) * S.focalLength));
        numDet = [nEl, nAz];
    else
        numDet = S.numDetectors;
    end
    safeSet(ir, 'NumDetectors', numDet);

    if scanrate > 0
        safeSet(ir, 'MaxMechanicalScanRate', [scanrate; scanrate]);
    end

    sectorSpan = abs(diff(S.sector));
    if sectorSpan < 359 && sectorSpan > 0
        safeSet(ir, 'MechanicalScanLimits', [S.sector; -fov(2)/2 fov(2)/2]);
    end

    safeSet(ir, 'FalseAlarmRate', S.far);
    applyUnmatched(ir, p.Unmatched);

    actualFov = [0; 0];
    try actualFov = ir.FieldOfView; catch; end

    meta = struct();
    meta.rpm            = rpm;
    meta.requestedFov   = fov;
    meta.actualFov      = actualFov;
    meta.numDetectors   = numDet;
    meta.focalLength    = S.focalLength;
    meta.lensDiameter   = S.lensDiameter;
    meta.scanrate_degps = scanrate;
    meta.updaterate_hz  = updaterate;
    meta.far            = S.far;
    meta.mountingLoc    = S.mountingLoc;
end


%% ========================================================================
%                         SONAR BUILDER
%% ========================================================================
function [snr, meta] = buildSonarSensor(idx, scanConfig, defaults, varargin)
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'rpm',            getOr(defaults,'rpm',6));
    addParameter(p, 'fov',            getOr(defaults,'fov',[5;10]));
    addParameter(p, 'sector',         getOr(defaults,'sector',[0 360]));
    addParameter(p, 'far',            getOr(defaults,'far',1e-4));
    addParameter(p, 'rangeLimits',    getOr(defaults,'rangeLimits',[0 20000]));
    addParameter(p, 'rangeRes',       getOr(defaults,'rangeRes',50));
    addParameter(p, 'hasElevation',   getOr(defaults,'hasElevation',true));
    addParameter(p, 'hasRangeRate',   getOr(defaults,'hasRangeRate',false));
    addParameter(p, 'hasINS',         true);
    addParameter(p, 'detCoords',      'Sensor spherical');
    addParameter(p, 'mountingLoc',    [0 0 0]);
    addParameter(p, 'updateRate',     getOr(defaults,'updateRate',[]));
    addParameter(p, 'detectionMode',  getOr(defaults,'detectionMode','passive'));
    addParameter(p, 'centerFreq',     getOr(defaults,'centerFreq',20000));
    addParameter(p, 'bandwidth',      getOr(defaults,'bandwidth',2000));
    parse(p, varargin{:});
    S = p.Results;

    fov = S.fov(:); rpm = S.rpm;

    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;
        updaterate = scanrate / max(fov(1), 0.01);
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 1; scanrate = 0;
    end

    fov2 = fov(:);
    if numel(fov2) < 2; fov2 = [fov2; fov2]; end

    w = warning('off', 'all');
    cleanupW = onCleanup(@() warning(w));
    constructed = false;

    try
        nvFull = {'DetectionMode', S.detectionMode, 'HasElevation', S.hasElevation, 'FieldOfView', fov2};
        if strcmpi(S.detectionMode, 'monostatic'); nvFull = [nvFull, {'EmitterIndex', idx}]; end
        snr = sonarSensor(idx, scanConfig, nvFull{:});
        constructed = true;
    catch; end

    if ~constructed
        try
            nvMin = {'DetectionMode', S.detectionMode};
            if strcmpi(S.detectionMode, 'monostatic'); nvMin = [nvMin, {'EmitterIndex', idx}]; end
            snr = sonarSensor(idx, scanConfig, nvMin{:});
            safeSet(snr, 'HasElevation', S.hasElevation);
            safeSet(snr, 'FieldOfView',  fov2);
            constructed = true;
        catch; end
    end

    if ~constructed
        snr = sonarSensor(idx, scanConfig);
        safeSet(snr, 'DetectionMode', S.detectionMode);
        safeSet(snr, 'HasElevation',  S.hasElevation);
        safeSet(snr, 'FieldOfView',   fov2);
    end

    safeSet(snr, 'UpdateRate',       updaterate);
    safeSet(snr, 'HasINS',           S.hasINS);
    safeSet(snr, 'MountingLocation', S.mountingLoc);
    safeSet(snr, 'CenterFrequency',  S.centerFreq);
    safeSet(snr, 'Bandwidth',        S.bandwidth);
    safeSet(snr, 'FalseAlarmRate',   S.far);
    safeSet(snr, 'RangeResolution',  S.rangeRes);
    safeSet(snr, 'DetectionCoordinates', char(S.detCoords));

    if scanrate > 0
        if S.hasElevation; safeSet(snr, 'MaxMechanicalScanRate', [scanrate; scanrate]);
        else; safeSet(snr, 'MaxMechanicalScanRate', scanrate); end
    end

    sectorSpan = abs(diff(S.sector));
    if sectorSpan < 359 && sectorSpan > 0
        if S.hasElevation; safeSet(snr, 'MechanicalScanLimits', [S.sector; -fov(2)/2 fov(2)/2]);
        else; safeSet(snr, 'MechanicalScanLimits', S.sector); end
    end

    applyUnmatched(snr, p.Unmatched);

    actualFov = [0; 0];
    try actualFov = snr.FieldOfView; catch; end

    meta = struct();
    meta.rpm            = rpm;
    meta.fov            = actualFov;
    meta.scanrate_degps = scanrate;
    meta.updaterate_hz  = updaterate;
    meta.rangeLimits_m  = S.rangeLimits;
    meta.far            = S.far;
    meta.detectionMode  = S.detectionMode;
    meta.centerFreq_Hz  = S.centerFreq;
    meta.bandwidth_Hz   = S.bandwidth;
    meta.mountingLoc    = S.mountingLoc;
end


%% ========================================================================
%                         LIDAR BUILDER
%% ========================================================================
function [lid, meta] = buildLidar(idx, defaults, varargin)
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'maxRange',      getOr(defaults,'maxRange',200));
    addParameter(p, 'rangeAccuracy', getOr(defaults,'rangeAccuracy',0.02));
    addParameter(p, 'azResolution',  getOr(defaults,'azResolution',0.2));
    addParameter(p, 'elResolution',  getOr(defaults,'elResolution',0.4));
    addParameter(p, 'azLimits',      getOr(defaults,'azLimits',[-180 180]));
    addParameter(p, 'elLimits',      getOr(defaults,'elLimits',[-25 15]));
    addParameter(p, 'updateRate',    getOr(defaults,'updateRate',10));
    addParameter(p, 'hasINS',        true);
    addParameter(p, 'detCoords',     'Scenario');
    addParameter(p, 'mountingLoc',   [0 0 -2]);
    parse(p, varargin{:});
    S = p.Results;

    lid = monostaticLidarSensor(idx);
    safeSet(lid, 'MaxRange',              S.maxRange);
    safeSet(lid, 'RangeAccuracy',         S.rangeAccuracy);
    safeSet(lid, 'AzimuthResolution',     S.azResolution);
    safeSet(lid, 'ElevationResolution',   S.elResolution);
    safeSet(lid, 'AzimuthLimits',         S.azLimits);
    safeSet(lid, 'ElevationLimits',       S.elLimits);
    safeSet(lid, 'UpdateRate',            S.updateRate);
    safeSet(lid, 'HasINS',               S.hasINS);
    safeSet(lid, 'DetectionCoordinates',  char(S.detCoords));
    safeSet(lid, 'MountingLocation',      S.mountingLoc);
    applyUnmatched(lid, p.Unmatched);

    meta = struct();
    meta.maxRange_m     = S.maxRange;
    meta.rangeAccuracy  = S.rangeAccuracy;
    meta.azRes_deg      = S.azResolution;
    meta.elRes_deg      = S.elResolution;
    meta.azLimits_deg   = S.azLimits;
    meta.elLimits_deg   = S.elLimits;
    meta.updaterate_hz  = S.updateRate;
    meta.mountingLoc    = S.mountingLoc;
end


%% ========================================================================
%                         ADS-B BUILDERS
%% ========================================================================
function [tx, meta] = buildADSBTx(idx, varargin)
    p = inputParser;
    addParameter(p, 'ICAO',       sprintf('A%05X', idx),  @(x)ischar(x)||isstring(x));
    addParameter(p, 'Category',   'Heavy',                @(x)ischar(x)||isstring(x));
    addParameter(p, 'Callsign',   sprintf('TGT%03d', idx),@(x)ischar(x)||isstring(x));
    addParameter(p, 'UpdateRate', 2,                       @(x)isnumeric(x)&&isscalar(x));
    parse(p, varargin{:});
    S = p.Results;

    tx = adsbTransponder(char(S.ICAO));
    if isprop(tx,'Category');   tx.Category   = adsbCategory(char(S.Category)); end
    if isprop(tx,'Callsign');   tx.Callsign   = char(S.Callsign); end
    if isprop(tx,'UpdateRate'); tx.UpdateRate  = S.UpdateRate; end

    meta = struct();
    meta.type        = 'ADSB_TX';
    meta.description = 'ADS-B Transponder';
    meta.ICAO        = char(S.ICAO);
    meta.Category    = char(S.Category);
    meta.Callsign    = char(S.Callsign);
    meta.updateRate  = S.UpdateRate;
end

function [rx, meta] = buildADSBRx(idx, varargin)
    p = inputParser;
    addParameter(p, 'MaxNumTracks', 200, @(x)isnumeric(x)&&isscalar(x));
    parse(p, varargin{:});
    S = p.Results;

    rx = adsbReceiver;
    if isprop(rx,'ReceiverIndex'); rx.ReceiverIndex = idx; end
    if isprop(rx,'MaxNumTracks');  rx.MaxNumTracks  = S.MaxNumTracks; end

    meta = struct();
    meta.type         = 'ADSB_RX';
    meta.description  = 'ADS-B Receiver';
    meta.maxNumTracks = S.MaxNumTracks;
end


%% ========================================================================
%                         CUSTOM SENSOR TEMPLATE
%% ========================================================================
function [template, meta] = getCustomTemplate(idx)
    template = struct();
    template.NOTE = 'This is a template, not a real sensor. See customSensorTemplate.m';
    template.requiredProperties = { ...
        'SensorIndex','Unique integer ID'; 'UpdateRate','Hz';
        'MountingLocation','[x y z] meters'; 'MountingAngles','[yaw pitch roll] degrees';
        'HasINS','true/false'; 'DetectionCoordinates','''Scenario'' | ''Body'' | ''Sensor spherical'''};
    template.requiredMethods = { ...
        'stepImpl(obj, targets, time)','Return cell array of objectDetection';
        'coverageConfig(obj)','Return struct with FieldOfView, ScanLimits, etc.'};
    meta = struct();
    meta.type = 'CUSTOM';
    meta.description = 'Template struct — see customSensorTemplate.m for implementation';
    meta.sensorIndex = idx;
end


%% ========================================================================
%                         UTILITY FUNCTIONS
%% ========================================================================
function val = getOr(S, field, default)
    if isstruct(S) && isfield(S, field); val = S.(field); else; val = default; end
end

function safeSet(obj, propName, value)
    if isprop(obj, propName)
        w = warning('off', 'MATLAB:system:nonRelevantProperty');
        cleanupObj = onCleanup(@() warning(w));
        obj.(propName) = value;
    end
end

function applyUnmatched(obj, unmatched)
    fields = fieldnames(unmatched);
    for i = 1:numel(fields); safeSet(obj, fields{i}, unmatched.(fields{i})); end
end
