classdef customSensorTemplate < matlab.System
%customSensorTemplate  Template for creating a custom sensor compatible
%                      with trackingScenario and the V2 simulation.
%
% HOW TO USE THIS TEMPLATE
%   1. Copy this file and rename the class to match your sensor
%   2. Modify the properties to match your sensor's physical parameters
%   3. Implement stepImpl() to generate objectDetection objects
%   4. Optionally implement coverageConfig() for visualization
%   5. Add to scenario via: platform(scenario, 'Sensors', {yourSensor})
%
% REQUIREMENTS FOR trackingScenario COMPATIBILITY
%   - Must inherit from matlab.System
%   - Must have SensorIndex, UpdateRate properties
%   - stepImpl must accept (targets, time) and return {objectDetection}
%   - Detections must have valid MeasurementParameters for coordinate
%     transforms if using 'Scenario' coordinates
%
% REQUIREMENTS FOR V2 SIMULATION COMPATIBILITY
%   - SensorIndex must be unique across all sensors in the scenario
%   - DetectionCoordinates should be 'Scenario' for consistency
%   - HasINS should be true so platform pose is available
%   - Returned objectDetection.SensorIndex must match this sensor's index
%
% EXAMPLE — CREATING A SIMPLE ACOUSTIC SENSOR
%   classdef acousticSensor < customSensorTemplate
%       properties
%           FrequencyRange = [20 20000]; % Hz
%           SoundSpeedMps  = 343;        % m/s in air
%       end
%       methods (Access = protected)
%           function dets = stepImpl(obj, targets, time)
%               dets = {};
%               for i = 1:numel(targets)
%                   pos = targets(i).Position(:);
%                   rng = norm(pos - obj.MountingLocation(:));
%                   if rng <= obj.MaxRange && rand() < obj.Pd
%                       det = objectDetection(time, pos, ...
%                           'SensorIndex', obj.SensorIndex, ...
%                           'MeasurementNoise', eye(3) * obj.NoiseStd^2);
%                       dets{end+1} = det;
%                   end
%               end
%           end
%       end
%   end
%
% See also: buildSensor, fusionRadarSensor, irSensor, sonarSensor

    % =====================================================================
    %  REQUIRED PROPERTIES (must exist for trackingScenario)
    % =====================================================================
    properties
        SensorIndex (1,1) double = 1
        UpdateRate  (1,1) double = 1            % Hz
    end

    % =====================================================================
    %  PLATFORM MOUNTING (standard across all MATLAB sensors)
    % =====================================================================
    properties
        MountingLocation (1,3) double = [0 0 0]   % [x y z] meters
        MountingAngles   (1,3) double = [0 0 0]   % [yaw pitch roll] degrees
    end

    % =====================================================================
    %  SENSOR CONFIGURATION
    % =====================================================================
    properties
        HasINS               (1,1) logical = true
        DetectionCoordinates (1,:) char     = 'Scenario'
        HasElevation         (1,1) logical = true
        HasNoise             (1,1) logical = true
        HasFalseAlarms       (1,1) logical = true
    end

    % =====================================================================
    %  YOUR SENSOR-SPECIFIC PROPERTIES (modify these)
    % =====================================================================
    properties
        % Detection performance
        Pd       (1,1) double = 0.9     % Probability of detection
        FAR      (1,1) double = 1e-6    % False alarm rate
        NoiseStd (1,1) double = 50      % Measurement noise std dev (meters)

        % Coverage
        MaxRange (1,1) double = 100000  % meters
        FieldOfView (2,1) double = [360; 90]  % [az; el] degrees

        % Your custom parameters go here
        % e.g.:
        % Wavelength_m     = 0.03;     % 10 GHz X-band
        % TransmitPower_W  = 1000;
        % AntennaGain_dBi  = 30;
    end

    % =====================================================================
    %  INTERNAL STATE
    % =====================================================================
    properties (Access = private)
        pLastUpdateTime = -inf
    end

    methods
        function obj = customSensorTemplate(sensorIndex)
            %customSensorTemplate  Constructor.
            if nargin >= 1
                obj.SensorIndex = sensorIndex;
            end
        end
    end

    % =====================================================================
    %  CORE DETECTION GENERATION (implement your physics here)
    % =====================================================================
    methods (Access = protected)
        function dets = stepImpl(obj, targets, time)
            %stepImpl  Generate detections from target truth data.
            %
            % INPUTS
            %   targets : struct array with fields:
            %     .PlatformID  - unique target identifier
            %     .ClassID     - target class (optional)
            %     .Position    - [x y z] in scenario frame (meters)
            %     .Velocity    - [vx vy vz] in scenario frame (m/s)
            %     .Orientation - 3x3 rotation matrix or quaternion
            %     .Dimensions  - struct with Length, Width, Height, etc.
            %     .Signatures  - cell array of signature objects (optional)
            %
            % OUTPUTS
            %   dets : cell array of objectDetection objects

            dets = {};

            % Rate gate: only produce detections at UpdateRate intervals
            dt = 1 / obj.UpdateRate;
            if (time - obj.pLastUpdateTime) < dt - 1e-6
                return;
            end
            obj.pLastUpdateTime = time;

            for i = 1:numel(targets)
                tgt = targets(i);
                tgtPos = tgt.Position(:);

                % ---- YOUR DETECTION LOGIC HERE ----
                % Example: simple range + probability check

                % 1. Compute range to target
                sensorPos = obj.MountingLocation(:);
                relPos = tgtPos - sensorPos;
                rng = norm(relPos);

                % 2. Check if target is in range
                if rng > obj.MaxRange
                    continue;
                end

                % 3. Check if target is in FOV (azimuth + elevation)
                az = atan2d(relPos(2), relPos(1));
                el = atan2d(-relPos(3), norm(relPos(1:2)));
                if abs(az) > obj.FieldOfView(1)/2 || abs(el) > obj.FieldOfView(2)/2
                    continue;
                end

                % 4. Probability of detection (can add range-dependent Pd)
                if rand() > obj.Pd
                    continue;
                end

                % 5. Generate noisy measurement
                if obj.HasNoise
                    noise = obj.NoiseStd * randn(3,1);
                else
                    noise = zeros(3,1);
                end
                measPos = tgtPos + noise;

                % 6. Build measurement noise covariance
                measCov = eye(3) * obj.NoiseStd^2;

                % 7. Create objectDetection
                det = objectDetection(time, measPos, ...
                    'SensorIndex',       obj.SensorIndex, ...
                    'MeasurementNoise',  measCov, ...
                    'ObjectClassID',     0, ...
                    'ObjectAttributes',  struct( ...
                        'TargetIndex', tgt.PlatformID, ...
                        'SNR',         10 * log10(obj.MaxRange / max(rng, 1))));

                dets{end+1} = det; %#ok<AGROW>
            end

            % ---- FALSE ALARMS ----
            if obj.HasFalseAlarms
                % Simple Poisson false alarm model
                % Expected # of false alarms per scan
                volume = (4/3) * pi * obj.MaxRange^3;
                nFA = poissrnd(obj.FAR * volume);
                nFA = min(nFA, 5);  % cap for sanity

                for j = 1:nFA
                    % Random position in sensor volume
                    faRange = obj.MaxRange * rand()^(1/3);
                    faAz    = 360 * rand() - 180;
                    faEl    = 180 * rand() - 90;
                    [fx, fy, fz] = sph2cart(deg2rad(faAz), deg2rad(faEl), faRange);
                    faPos = obj.MountingLocation(:) + [fx; fy; fz];

                    det = objectDetection(time, faPos, ...
                        'SensorIndex',       obj.SensorIndex, ...
                        'MeasurementNoise',  eye(3) * obj.NoiseStd^2, ...
                        'ObjectClassID',     0, ...
                        'ObjectAttributes',  struct('TargetIndex', -j, 'SNR', 0));
                    dets{end+1} = det; %#ok<AGROW>
                end
            end
        end
    end

    % =====================================================================
    %  COVERAGE CONFIG (optional — used by scenario visualization)
    % =====================================================================
    methods
        function config = coverageConfig(obj)
            %coverageConfig  Report sensor coverage for visualization.
            config = struct();
            config.SensorIndex      = obj.SensorIndex;
            config.IsValidTime      = true;
            config.FieldOfView      = obj.FieldOfView;
            config.ScanLimits       = [-obj.FieldOfView(1)/2 obj.FieldOfView(1)/2; ...
                                       -obj.FieldOfView(2)/2 obj.FieldOfView(2)/2];
            config.MountingLocation = obj.MountingLocation;
            config.MountingAngles   = obj.MountingAngles;
            config.RangeLimits      = [0 obj.MaxRange];
        end
    end
end
