classdef SensorRecord
%SensorRecord  Per-sensor state for the multi-sensor editor.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker - UW Senior Capstone, Boeing-sponsored
%
%  A plain VALUE class. Owning EditorState keeps a 1xN array of these in
%  state.sensors and the active one at state.sensors(state.activeSensorIdx).
%  Mutate via read-mutate-writeback:
%      sr = state.activeSensor();  sr.foo = bar;  state.setActiveSensor(sr);
%
%  VALUE not handle: undo/redo snapshots the sensors array, so value-class
%  copies are required for Ctrl+Z to work. Do not change this.
%
%  SUPPORTED TYPES (editing)
%    Radar: PSR, SSR, ASR, ARSR, PAR, MARITIME, WEATHER, TWS.
%    Sonar: ACTIVE_SONAR, PASSIVE_SONAR, TOWED_ARRAY.
%    IR:    IRST, IR_STARING, FLIR.
%    Everything else loads as UNKNOWN passthrough.
%
%  POSITION ENCODING: the editor stores world position on positionEastM /
%  positionNorthM; mountingLoc carries only z (NED, negative = above
%  ground). On export the E/N are baked into mountingLoc[0:1] because the
%  sim's tower platform sits at the origin and MountingLocation IS the
%  world offset.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.sensorDefaults

    properties
        % --- Identity ---
        sensorName (1,1) string = "sensor_1"
        sensorType (1,1) string = "PSR"       % radar|sonar|ir type or UNKNOWN
        platform   (1,1) string = "tower"

        % --- World placement (editor-only; baked into mountingLoc on export) ---
        positionEastM  (1,1) double = 0
        positionNorthM (1,1) double = 0

        % --- Common sensor parameters (units match buildSensor.m) ---
        frequencyHz  (1,1) double = 2.8e9
        rangeLimits  (1,2) double = [0 111120]   % [minRange_m, maxRange_m]

        % --- Rotator params (used when isRotator() is true) ---
        rpm          (1,1) double = 12.5
        fov          (1,2) double = [1.4 30]     % [az_deg, el_deg]
        tilt         (1,1) double = 2            % elevation center offset (deg)

        % --- Sector params (isSector / isNoScan) ---
        sectorDeg    (1,2) double = [0 360]      % [startAz, endAz] deg

        % --- Detection quality ---
        pd           (1,1) double = 0.9
        far          (1,1) double = 1e-6
        rangeResM    (1,1) double = 93

        % --- Mounting (stores only z; x/y baked from position on export) ---
        mountingLoc  (1,3) double = [0 0 -15]

        % --- Display / editor-local ---
        readOnly     (1,1) logical = false
        sourceFile   (1,1) string  = ""
        displayColor (1,3) double  = [0.85 0.55 0.20]

        % --- Unknown-type passthrough (verbatim re-export) ---
        originalDef  (1,1) struct = struct()

        % --- Modality-specific (sonar / IR), unused by radar types ---
        detectionMode (1,1) string  = "monostatic"  % sonar: monostatic | passive
        updateRate    (1,1) double  = 0              % sonar/IR explicit rate (0 = derive)
        hasElevation  (1,1) logical = true           % elevation channel present
    end

    methods
        function tf = isRotator(obj)
            %isRotator  Full 360-degree sweep at rpm > 0. Visual: range ring.
            tf = obj.rpm > 0 && abs(diff(obj.sectorDeg)) >= 359;
        end

        function tf = isSector(obj)
            %isSector  Azimuth sector (0 < span < 360), regardless of rpm.
            %  PAR (rpm=0, electronic sector) hits this too. Dispatch
            %  priority is isRotator -> isNoScan -> isSector -> else.
            span = abs(diff(obj.sectorDeg));
            tf = span < 359 && span > 0;
        end

        function tf = isNoScan(obj)
            %isNoScan  No-scan / staring radar (rpm=0): TWS/AESA/FIRE_CONTROL.
            tf = obj.rpm <= 0 && ( ...
                strcmpi(obj.sensorType, "TWS") || ...
                strcmpi(obj.sensorType, "AESA") || ...
                strcmpi(obj.sensorType, "FIRE_CONTROL"));
        end

        function c = sensorClass(obj)
            %sensorClass  "radar" | "sonar" | "ir" from sensorType.
            t = upper(obj.sensorType);
            if any(t == ["ACTIVE_SONAR","PASSIVE_SONAR","TOWED_ARRAY","CUSTOM_SONAR"])
                c = "sonar";
            elseif any(t == ["IRST","IR_STARING","FLIR","CUSTOM_IR"])
                c = "ir";
            else
                c = "radar";
            end
        end

        function tf = isSonar(obj); tf = obj.sensorClass() == "sonar"; end
        function tf = isIR(obj);    tf = obj.sensorClass() == "ir";    end
    end
end
