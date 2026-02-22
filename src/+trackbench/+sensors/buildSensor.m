function [sensor, meta] = buildSensor(sensorIndex, sensorType, varargin)
%buildSensor  Universal sensor factory for the tracking simulation.
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
% USAGE
%   % Quick defaults
%   [psr, meta]  = buildSensor(1, 'PSR');
%   [ssr, meta]  = buildSensor(2, 'SSR');
%   [irst, meta] = buildSensor(3, 'IRST');
%
%   % Override specific parameters
%   [psr, meta] = buildSensor(1, 'PSR', 'rpm', 15, 'pd', 0.85);
%   [par, meta] = buildSensor(4, 'PAR', 'sector', [170 190], 'rangeLimits', [0 37000]);
%
%   % ADS-B (different interface)
%   [tx, meta] = buildSensor(1, 'ADSB_TX', 'ICAO', 'A1B2C3');
%   [rx, meta] = buildSensor(1, 'ADSB_RX');
%
%   % Custom sensor template
%   [template, meta] = buildSensor(1, 'CUSTOM');
%
% OUTPUTS
%   sensor : configured sensor object (type depends on sensorType)
%   meta   : struct with all derived parameters for logging/diagnostics
%
% INTEGRATION WITH createScenario3D
%   To add a sensor built here to your scenario:
%     scenario = trackingScenario;
%     [psr, ~] = buildSensor(1, 'PSR');
%     [ssr, ~] = buildSensor(2, 'SSR');
%     tower = platform(scenario, 'Sensors', {psr, ssr});
%
% ─────────────────────────────────────────────────────────────────────
% CUSTOM SENSORS — HOW TO ADD YOUR OWN
% ─────────────────────────────────────────────────────────────────────
% If MATLAB doesn't have a built-in sensor for your use case, you have
% three options:
%
%   OPTION A: Use 'CUSTOM_RADAR' / 'CUSTOM_IR' / 'CUSTOM_SONAR' / 'CUSTOM_LIDAR'
%     These create a bare sensor object with minimal defaults. Pass all
%     your parameters as name-value pairs. The factory will set every
%     property that exists on the sensor object.
%
%     Example:
%       [s, m] = buildSensor(5, 'CUSTOM_RADAR', ...
%           'ScanMode', 'Raster', ...
%           'FieldOfView', [2; 20], ...
%           'RangeLimits', [0 500000], ...
%           'UpdateRate', 10);
%
%   OPTION B: Use 'CUSTOM' to get a template struct
%     Returns a struct showing all fields you need to implement a custom
%     sensor class. Use this as a starting point for writing your own
%     System object that implements the same interface as fusionRadarSensor.
%
%     Required interface for trackingScenario compatibility:
%       - Inherit from matlab.System
%       - Implement stepImpl(obj, targets, time) -> {objectDetection, ...}
%       - Properties: SensorIndex, UpdateRate, MountingLocation,
%         MountingAngles, HasINS, DetectionCoordinates
%       - Method: coverageConfig(obj) -> struct with FieldOfView, etc.
%
%   OPTION C: Wrap an existing non-standard sensor
%     Write a thin wrapper class that translates your sensor's output
%     into objectDetection format. See customSensorTemplate.m for a
%     full example with comments.
%
% See also: fusionRadarSensor, radarDataGenerator, irSensor, sonarSensor,
%           monostaticLidarSensor, adsbTransponder, adsbReceiver,
%           buildCustomFusionRadarSensor, buildIFFSensor

arguments
    sensorIndex (1,1) double {mustBeFinite, mustBePositive}
    sensorType  (1,1) string
end
arguments (Repeating)
    varargin
end

sensorType = upper(sensorType);

switch sensorType
    % =====================================================================
    %  RADAR FAMILY
    % =====================================================================
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

    % =====================================================================
    %  INFRARED FAMILY
    % =====================================================================
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

    % =====================================================================
    %  SONAR FAMILY
    % =====================================================================
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

    % =====================================================================
    %  LIDAR
    % =====================================================================
    case 'LIDAR'
        [sensor, meta] = buildLidar(sensorIndex, getDefaults('LIDAR'), varargin{:});
        meta.type = 'LIDAR';
        meta.description = 'Monostatic lidar point cloud generator';

    case 'CUSTOM_LIDAR'
        [sensor, meta] = buildLidar(sensorIndex, struct(), varargin{:});
        meta.type = 'CUSTOM_LIDAR';
        meta.description = 'User-configured monostaticLidarSensor';

    % =====================================================================
    %  ADS-B
    % =====================================================================
    case 'ADSB_TX'
        [sensor, meta] = buildADSBTx(sensorIndex, varargin{:});

    case 'ADSB_RX'
        [sensor, meta] = buildADSBRx(sensorIndex, varargin{:});

    % =====================================================================
    %  CUSTOM TEMPLATE
    % =====================================================================
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

% Stamp common metadata
meta.sensorIndex = sensorIndex;

fprintf('[buildSensor] Created %s (index=%d) | %s\n', ...
    meta.type, sensorIndex, meta.description);
end


%% ========================================================================
%                         DEFAULT PARAMETER SETS
%% ========================================================================
function D = getDefaults(sensorType)
%getDefaults  Return a struct of realistic defaults for each sensor type.
%
% SOURCES FOR DEFAULTS
%   PSR/ASR  : DASR ASR-11 / AN/GPN-30 specifications
%   SSR      : MSSR Mode S, ICAO Annex 10 Vol IV
%   ARSR     : ARSR-4 / FPS-130 specifications
%   PAR      : AN/GPN-22, AN/FPN-63 specifications
%   AESA     : Generic AESA fighter radar (APG-81 class)
%   TWS      : Generic phased array TWS
%   WEATHER  : WSR-88D NEXRAD specifications
%   MARITIME : Generic X-band nav radar
%   IRST     : Generic naval IRST (e.g. IRST21)
%   FLIR     : Generic targeting FLIR pod
%   Sonar    : Generic hull-mounted / towed array sonar

    switch upper(sensorType)
        % --- RADAR ---
        case 'PSR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 30];
            D.tilt         = 2;
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 111120];      % 60 nm
            D.rangeRes     = 93;              % ~303 ft
            D.refRange     = 111120;
            D.refRCS       = 0;               % dBsm
            D.hasElevation = true;
            D.hasRangeRate = false;

        case 'SSR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 10];
            D.tilt         = 2;
            D.pd           = 0.99;
            D.far          = 1e-7;            % MATLAB floor
            D.rangeLimits  = [0 222240];      % 120 nm
            D.rangeRes     = 100;
            D.refRange     = 222240;
            D.refRCS       = 20;              % high — transponder reply
            D.hasElevation = true;
            D.hasRangeRate = false;

        case 'ASR'
            D.rpm          = 12.5;
            D.fov          = [1.4; 5];
            D.tilt         = 2;
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 111120];      % 60 nm
            D.rangeRes     = 93;
            D.refRange     = 111120;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;

        case 'ARSR'
            D.rpm          = 5;               % slower rotation, longer range
            D.fov          = [1.5; 20];
            D.tilt         = 0;
            D.pd           = 0.85;
            D.far          = 1e-6;
            D.rangeLimits  = [0 463000];      % 250 nm
            D.rangeRes     = 250;
            D.refRange     = 463000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;

        case 'PAR'
            D.rpm          = 0;               % sector scan, not rotating
            D.fov          = [1.0; 1.0];
            D.sector       = [170 190];       % 20-deg sector centered on approach
            D.tilt         = -3;              % slight downtilt for approach path
            D.pd           = 0.95;
            D.far          = 1e-7;
            D.rangeLimits  = [0 37040];       % 20 nm
            D.rangeRes     = 15;              % fine resolution
            D.refRange     = 37040;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.updateRate   = 60;              % high update rate

        case 'TWS'
            D.fov          = [60; 30];        % wide staring FOV
            D.pd           = 0.9;
            D.far          = 1e-6;
            D.rangeLimits  = [0 200000];
            D.rangeRes     = 150;
            D.refRange     = 200000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 2;               % Hz

        case 'AESA'
            D.fov          = [3; 3];          % pencil beam
            D.sector       = [-60 60];        % ±60 deg electronic scan
            D.pd           = 0.95;
            D.far          = 1e-7;
            D.rangeLimits  = [0 300000];
            D.rangeRes     = 30;
            D.refRange     = 300000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 20;              % fast electronic scan

        case 'FIRE_CONTROL'
            D.fov          = [2; 2];          % narrow beam
            D.pd           = 0.98;
            D.far          = 1e-7;            % MATLAB floor
            D.rangeLimits  = [0 150000];
            D.rangeRes     = 10;
            D.refRange     = 150000;
            D.refRCS       = 0;
            D.hasElevation = true;
            D.hasRangeRate = true;
            D.updateRate   = 50;              % very high update rate

        case 'WEATHER'
            D.rpm          = 25;
            D.fov          = [1.0; 1.0];
            D.tilt         = 0;
            D.pd           = 0.7;             % weather returns are diffuse
            D.far          = 1e-3;            % lots of clutter by design
            D.rangeLimits  = [0 463000];      % 250 nm
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
            D.far          = 1e-5;            % sea clutter
            D.rangeLimits  = [0 74080];       % 40 nm
            D.rangeRes     = 25;
            D.refRange     = 74080;
            D.refRCS       = -5;
            D.hasElevation = false;
            D.hasRangeRate = false;

        % --- INFRARED ---
        case 'IRST'
            D.rpm          = 60;              % fast scan
            D.fov          = [2; 5];
            D.pd           = 0.8;
            D.far          = 1e-6;
            D.rangeLimits  = [0 100000];
            D.rangeRes     = 500;             % coarse range
            D.hasElevation = true;
            D.hasRangeRate = false;

        case 'IR_STARING'
            D.fov          = [90; 60];        % wide staring FOV
            D.pd           = 0.7;
            D.far          = 1e-5;
            D.rangeLimits  = [0 50000];
            D.rangeRes     = 1000;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.updateRate   = 30;              % video rate

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

        % --- SONAR ---
        case 'ACTIVE_SONAR'
            D.rpm          = 0;
            D.fov          = [360; 90];       % omni for ping+listen
            D.far          = 1e-4;
            D.rangeLimits  = [0 20000];
            D.rangeRes     = 50;
            D.hasElevation = true;
            D.hasRangeRate = false;
            D.detectionMode = 'monostatic';

        case 'PASSIVE_SONAR'
            D.fov          = [360; 20];       % omni-directional listen
            D.pd           = 0.5;
            D.far          = 1e-4;
            D.rangeLimits  = [0 50000];
            D.rangeRes     = 500;
            D.hasElevation = false;
            D.hasRangeRate = false;
            D.updateRate   = 1;

        case 'TOWED_ARRAY'
            D.fov          = [120; 10];       % wide sector behind ship
            D.sector       = [120 240];
            D.pd           = 0.6;
            D.far          = 1e-4;
            D.rangeLimits  = [0 80000];
            D.rangeRes     = 300;
            D.hasElevation = false;
            D.hasRangeRate = false;
            D.updateRate   = 0.5;

        % --- LIDAR ---
        case 'LIDAR'
            D.maxRange          = 200;        % meters
            D.rangeAccuracy     = 0.02;       % 2 cm
            D.azResolution      = 0.2;        % deg
            D.elResolution      = 0.4;        % deg
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
%buildRadar  Create and configure a fusionRadarSensor.

    % Parse user overrides on top of defaults
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
    parse(p, varargin{:});
    S = p.Results;

    fov = S.fov(:);
    rpm = S.rpm;

    % Compute update rate from RPM if not explicitly given
    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;    % deg/s
        updaterate = scanrate / fov(1);  % Hz
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 1;
        scanrate   = fov(1);
    end

    % Create sensor
    radar = fusionRadarSensor(idx, scanConfig);

    % Apply properties (guard each with isprop for version safety)
    safeSet(radar, 'UpdateRate',            updaterate);
    safeSet(radar, 'FieldOfView',           fov);
    safeSet(radar, 'MaxAzimuthScanRate',    scanrate);
    safeSet(radar, 'AzimuthResolution',     fov(1));
    safeSet(radar, 'HasElevation',          S.hasElevation);
    safeSet(radar, 'HasRangeRate',          S.hasRangeRate);

    % Sector limits (only for non-360 sectors)
    sectorSpan = abs(diff(S.sector));
    if sectorSpan < 359 && sectorSpan > 0
        safeSet(radar, 'MechanicalAzimuthLimits', S.sector);
    end

    % Elevation
    if S.hasElevation
        safeSet(radar, 'MechanicalElevationLimits', [-fov(2) 0] - S.tilt);
    end

    % FOV epsilon — prevents edge-case missed detections at FOV boundary
    safeSet(radar, 'FieldOfView', [fov(1); fov(2) + 1e-3]);

    % Range
    safeSet(radar, 'RangeLimits',       S.rangeLimits);
    safeSet(radar, 'RangeResolution',   S.rangeRes);
    safeSet(radar, 'ReferenceRange',    S.refRange);
    safeSet(radar, 'ReferenceRCS',      S.refRCS);

    % Detection performance
    safeSet(radar, 'DetectionProbability', S.pd);
    safeSet(radar, 'FalseAlarmRate',       S.far);

    % Platform integration
    safeSet(radar, 'HasINS',               S.hasINS);
    safeSet(radar, 'DetectionCoordinates',  char(S.detCoords));
    safeSet(radar, 'MountingLocation',      S.mountingLoc);

    % Apply any unmatched name-value pairs directly
    applyUnmatched(radar, p.Unmatched);

    % Metadata
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
end


%% ========================================================================
%                         IR SENSOR BUILDER
%% ========================================================================
function [ir, meta] = buildIR(idx, scanConfig, defaults, varargin)
%buildIR  Create and configure an irSensor.
%
% NOTE: irSensor.FieldOfView is READ-ONLY. FOV is determined by the
% optical parameters: FocalLength, NumDetectors, LensDiameter, DetectorArea.
% The 'fov' parameter here is used to derive NumDetectors that approximate
% the requested FOV given the default FocalLength.
%
% irSensor uses MaxMechanicalScanRate and MechanicalScanLimits
% (not MaxAzimuthScanRate / MechanicalAzimuthLimits like fusionRadarSensor).

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
    % irSensor optical parameters (override for fine control)
    addParameter(p, 'focalLength',  getOr(defaults,'focalLength',800));
    addParameter(p, 'lensDiameter', getOr(defaults,'lensDiameter',0.08));
    addParameter(p, 'numDetectors', getOr(defaults,'numDetectors',[]));
    parse(p, varargin{:});
    S = p.Results;

    fov = S.fov(:);
    rpm = S.rpm;

    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;
        updaterate = scanrate / max(fov(1), 0.01);
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 30;
        scanrate   = 0;
    end

    ir = irSensor(idx, scanConfig);

    safeSet(ir, 'UpdateRate',    updaterate);
    safeSet(ir, 'HasElevation',  S.hasElevation);
    safeSet(ir, 'HasINS',        S.hasINS);
    safeSet(ir, 'MountingLocation', S.mountingLoc);

    % Optical parameters that determine FOV
    safeSet(ir, 'FocalLength',   S.focalLength);
    safeSet(ir, 'LensDiameter',  S.lensDiameter);

    % Derive NumDetectors from desired FOV if not explicitly provided
    % FOV ≈ NumDetectors / FocalLength (in radians, approx for small angles)
    if isempty(S.numDetectors)
        nAz = max(10, round(deg2rad(fov(1)) * S.focalLength));
        nEl = max(10, round(deg2rad(fov(2)) * S.focalLength));
        numDet = [nEl, nAz];  % [rows=elevation, cols=azimuth]
    else
        numDet = S.numDetectors;
    end
    safeSet(ir, 'NumDetectors', numDet);

    % Scan parameters (irSensor uses MaxMechanicalScanRate, not MaxAzimuthScanRate)
    if scanrate > 0
        safeSet(ir, 'MaxMechanicalScanRate', [scanrate; scanrate]);
    end

    sectorSpan = abs(diff(S.sector));
    if sectorSpan < 359 && sectorSpan > 0
        safeSet(ir, 'MechanicalScanLimits', [S.sector; -fov(2)/2 fov(2)/2]);
    end

    safeSet(ir, 'FalseAlarmRate', S.far);

    applyUnmatched(ir, p.Unmatched);

    % Read back actual FOV for metadata
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
%buildSonarSensor  Create and configure a sonarSensor.
%
% NOTE: sonarSensor uses a different property interface than fusionRadarSensor:
%   - FieldOfView is settable (unlike irSensor) but via name-value at construction
%   - Uses MaxMechanicalScanRate (2-element) not MaxAzimuthScanRate
%   - Uses MechanicalScanLimits (2x2) not MechanicalAzimuthLimits
%   - DetectionCoordinates only relevant for monostatic mode
%   - No DetectionProbability property — detection is SNR-based
%   - sonarSensor step() takes sonarEmission objects, not target structs
%     (trackingScenario handles this automatically when using sonarEmitter)

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

    fov = S.fov(:);
    rpm = S.rpm;

    if isempty(S.updateRate) && rpm > 0
        scanrate   = rpm * 360 / 60;
        updaterate = scanrate / max(fov(1), 0.01);
    elseif ~isempty(S.updateRate)
        updaterate = S.updateRate;
        scanrate   = updaterate * fov(1);
    else
        updaterate = 1;
        scanrate   = 0;
    end

    % sonarSensor: Nontunable properties must be passed at construction.
    % The exact set of valid construction NV-pairs depends on DetectionMode
    % and ScanMode, so we try the full set first and fall back if needed.
    fov2 = fov(:);
    if numel(fov2) < 2; fov2 = [fov2; fov2]; end  % always 2-element

    % Suppress warnings during construction
    w = warning('off', 'all');
    cleanupW = onCleanup(@() warning(w));

    constructed = false;

    % Attempt 1: full NV pairs
    try
        nvFull = {'DetectionMode', S.detectionMode, ...
                  'HasElevation',  S.hasElevation, ...
                  'FieldOfView',   fov2};
        if strcmpi(S.detectionMode, 'monostatic')
            nvFull = [nvFull, {'EmitterIndex', idx}];
        end
        snr = sonarSensor(idx, scanConfig, nvFull{:});
        constructed = true;
    catch
    end

    % Attempt 2: minimal — just DetectionMode
    if ~constructed
        try
            nvMin = {'DetectionMode', S.detectionMode};
            if strcmpi(S.detectionMode, 'monostatic')
                nvMin = [nvMin, {'EmitterIndex', idx}];
            end
            snr = sonarSensor(idx, scanConfig, nvMin{:});
            safeSet(snr, 'HasElevation', S.hasElevation);
            safeSet(snr, 'FieldOfView',  fov2);
            constructed = true;
        catch
        end
    end

    % Attempt 3: bare construction + post-set everything
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

    % Scan parameters — only set az component if HasElevation is false
    if scanrate > 0
        if S.hasElevation
            safeSet(snr, 'MaxMechanicalScanRate', [scanrate; scanrate]);
        else
            safeSet(snr, 'MaxMechanicalScanRate', scanrate);
        end
    end

    sectorSpan = abs(diff(S.sector));
    if sectorSpan < 359 && sectorSpan > 0
        if S.hasElevation
            safeSet(snr, 'MechanicalScanLimits', [S.sector; -fov(2)/2 fov(2)/2]);
        else
            safeSet(snr, 'MechanicalScanLimits', S.sector);
        end
    end

    applyUnmatched(snr, p.Unmatched);

    % Read back actual FOV
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
%buildLidar  Create and configure a monostaticLidarSensor.

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
%buildADSBTx  Create an ADS-B transponder.

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
%buildADSBRx  Create an ADS-B receiver.

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
%getCustomTemplate  Return a struct template showing how to build a custom sensor.
%
% This is NOT a real sensor — it's documentation in struct form.
% To create a real custom sensor, implement a class that:
%   1. Inherits from matlab.System
%   2. Has properties: SensorIndex, UpdateRate, MountingLocation, etc.
%   3. Implements stepImpl(obj, targets, time) returning {objectDetection}
%   4. Optionally implements coverageConfig(obj) for scenario visualization
%
% See customSensorTemplate.m for a full working example.

    template = struct();
    template.NOTE = 'This is a template, not a real sensor. See customSensorTemplate.m';

    template.requiredProperties = { ...
        'SensorIndex',          'Unique integer ID';
        'UpdateRate',           'Hz — how often sensor produces detections';
        'MountingLocation',     '[x y z] meters on platform';
        'MountingAngles',       '[yaw pitch roll] degrees';
        'HasINS',               'true/false — enable INS input';
        'DetectionCoordinates', '''Scenario'' | ''Body'' | ''Sensor spherical'''};

    template.requiredMethods = { ...
        'stepImpl(obj, targets, time)', 'Return cell array of objectDetection';
        'coverageConfig(obj)',          'Return struct with FieldOfView, ScanLimits, etc.'};

    template.objectDetectionFields = { ...
        'Time',                 'Detection timestamp (s)';
        'Measurement',          '[x;y;z] or [az;el;r] depending on coordinates';
        'MeasurementNoise',     'Covariance matrix';
        'SensorIndex',          'Must match this sensor''s SensorIndex';
        'ObjectClassID',        'Integer class label (0=unknown)';
        'ObjectAttributes',     'struct with TargetIndex, SNR, etc.';
        'MeasurementParameters','Struct array for coordinate transforms'};

    template.exampleUsage = [ ...
        "% 1. Copy customSensorTemplate.m and rename your class\n" ...
        "% 2. Set properties for your sensor physics\n" ...
        "% 3. Implement stepImpl to generate objectDetection objects\n" ...
        "% 4. Add to scenario:\n" ...
        "%    mySensor = myCustomSensor(5);\n" ...
        "%    platform(scenario, 'Sensors', {psr, mySensor});\n"];

    meta = struct();
    meta.type = 'CUSTOM';
    meta.description = 'Template struct — see customSensorTemplate.m for implementation';
    meta.sensorIndex = idx;
end


%% ========================================================================
%                         UTILITY FUNCTIONS
%% ========================================================================
function val = getOr(S, field, default)
%getOr  Get field from struct or return default if missing.
    if isstruct(S) && isfield(S, field)
        val = S.(field);
    else
        val = default;
    end
end

function safeSet(obj, propName, value)
%safeSet  Set a property only if it exists on the object.
%  Suppresses warnings for properties that are irrelevant in the current
%  scan mode configuration (e.g. MaxAzimuthScanRate on 'No scanning').
    if isprop(obj, propName)
        w = warning('off', 'MATLAB:system:nonRelevantProperty');
        cleanupObj = onCleanup(@() warning(w));
        obj.(propName) = value;
    end
end

function applyUnmatched(obj, unmatched)
%applyUnmatched  Apply any extra name-value pairs directly to the sensor object.
%  This allows users to set ANY sensor property without the factory
%  needing to know about it explicitly. Silently skips non-existent props.
    fields = fieldnames(unmatched);
    for i = 1:numel(fields)
        safeSet(obj, fields{i}, unmatched.(fields{i}));
    end
end
