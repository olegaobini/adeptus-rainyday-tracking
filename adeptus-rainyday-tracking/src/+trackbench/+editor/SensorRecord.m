classdef SensorRecord
%SensorRecord  Per-sensor state for the multi-sensor editor (M6 §3.1).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  A plain VALUE class. Owning EditorState keeps a 1xN array of these in
%  state.sensors and the active one at state.sensors(state.activeSensorIdx).
%  Mutators read-mutate-writeback:
%      sr = state.activeSensor();
%      sr.foo = bar;
%      state.setActiveSensor(sr);
%
%  WHY VALUE AND NOT HANDLE
%    Undo/redo snapshots the sensors array. With value classes the
%    snapshot is an independent copy of every record. With handle
%    classes the snapshot would hold references to the same objects
%    future edits mutate, and Ctrl+Z would do nothing. Same reason
%    TargetRecord is a value class — do not change this.
%
%  PLATFORM SCOPE FOR M6
%    Only "tower" platforms are authored by the editor. Non-tower
%    platforms in loaded files (e.g. "aircraft" for AESA/FLIR) fall
%    through to UNKNOWN passthrough (see originalDef below): loaded
%    read-only, preserved verbatim on re-export, not exposed to edit.
%
%  SUPPORTED TYPES (M6 editing)
%    PSR, SSR, ASR, ARSR, PAR, MARITIME, WEATHER, TWS.
%    Everything else loads as UNKNOWN passthrough.
%
%  POSITION ENCODING (see handoff §3.4 and EDITOR-side convention note)
%    The editor stores world position on positionEastM/positionNorthM.
%    mountingLoc in the SensorRecord carries ONLY the z component
%    (altitude, NED: negative = above ground) plus two zeros in x/y.
%    On export, mountingLoc[0:1] are baked with positionEastM/North to
%    match buildSensor's expectation that the tower platform is at
%    origin (0,0,0) and the mountingLoc IS the world offset.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.TargetRecord

    properties
        % ── Identity ───────────────────────────────────────────────
        sensorName (1,1) string = "sensor_1"
        sensorType (1,1) string = "PSR"       % PSR|SSR|ASR|ARSR|PAR|MARITIME|WEATHER|TWS|UNKNOWN
        platform   (1,1) string = "tower"     % only "tower" supported in M6

        % ── World placement (NOT in the on-disk JSON — editor-only) ─
        %  Sensors live at a world position. On export, positionEastM
        %  and positionNorthM are baked into mountingLoc[0:2] because
        %  that's what the sim consumes (tower platform origin = 0,0,0,
        %  so MountingLocation IS the world offset).
        positionEastM  (1,1) double = 0
        positionNorthM (1,1) double = 0

        % ── Common sensor parameters (units match buildSensor.m) ────
        frequencyHz  (1,1) double = 2.8e9
        rangeLimits  (1,2) double = [0 111120]   % [minRange_m, maxRange_m]

        % ── Rotator params (used when isRotator() is true) ─────────
        rpm          (1,1) double = 12.5
        fov          (1,2) double = [1.4 30]     % [az_deg, el_deg]
        tilt         (1,1) double = 2            % elevation center offset (deg)

        % ── Sector params (used when isSector() or isNoScan() is true)
        sectorDeg    (1,2) double = [0 360]      % [startAz, endAz] in deg

        % ── Detection quality ──────────────────────────────────────
        pd           (1,1) double = 0.9
        far          (1,1) double = 1e-6
        rangeResM    (1,1) double = 93

        % ── Mounting (body-frame offset; M6 stores only z, x/y=0) ──
        %  On export we bake [positionEastM, positionNorthM, mountingLoc(3)]
        %  into the JSON; the stored in-editor value keeps x/y = 0 and
        %  z as the altitude offset (NED, so -15 = 15 m above ground).
        mountingLoc  (1,3) double = [0 0 -15]

        % ── Display / editor-local ─────────────────────────────────
        readOnly     (1,1) logical = false       % true for UNKNOWN passthrough
        sourceFile   (1,1) string  = ""
        displayColor (1,3) double  = [0.85 0.55 0.20]   % warm orange, distinct from target blue

        % ── Unknown-type passthrough ───────────────────────────────
        %  When sensorType == "UNKNOWN", originalDef holds the verbatim
        %  parsed JSON struct so re-export can round-trip the file.
        %  Default is an empty struct — populated only by the load path
        %  when encountering an unsupported type/platform.
        originalDef  (1,1) struct = struct()
    end

    methods
        function tf = isRotator(obj)
            %isRotator  True when the sensor sweeps a full 360° circle
            %           at rpm > 0. Visual: range ring.
            tf = obj.rpm > 0 && abs(diff(obj.sectorDeg)) >= 359;
        end

        function tf = isSector(obj)
            %isSector  True when the sensor covers an azimuth sector
            %          (0° < span < 360°) regardless of rpm. Two kinds
            %          of sensor hit this branch:
            %            * Mechanical sector scanners (rpm > 0) that
            %              sweep back-and-forth within a sector.
            %            * Phased-array sector scanners (rpm = 0) where
            %              the beam is steered electronically — PAR is
            %              the canonical example: buildSensor classifies
            %              it as 'Sector' scan config with rpm=0 and
            %              sector=[170 190]. The 20° electronic sweep
            %              IS the scan; no mechanical motion.
            %
            %  DISPATCH PRIORITY (drawSensor2D + refreshSensorParamsPanel)
            %    isRotator → isNoScan → isSector → else
            %    isNoScan wins for TWS / AESA / FIRE_CONTROL because
            %    those render as "beam cones" (distinctive staring
            %    visual) even though their sectorDeg also satisfies
            %    isSector. Checking isNoScan BEFORE isSector preserves
            %    that semantic; PAR hits isSector because PAR is not
            %    in the isNoScan type list.
            %
            %  PRE-M6-CHECKIN2 BUG  isSector originally required rpm>0,
            %  which excluded PAR. PAR then fell through to the
            %  else / anchor-ring branch and rendered as a gray circle
            %  instead of a wedge. Dropping the rpm>0 requirement fixes
            %  the render without affecting any other type.
            span = abs(diff(obj.sectorDeg));
            tf = span < 359 && span > 0;
        end

        function tf = isNoScan(obj)
            %isNoScan  True when the sensor is a no-scan / staring type
            %          (rpm=0) like TWS. Visual: sector wedge (static).
            tf = obj.rpm <= 0 && ( ...
                strcmpi(obj.sensorType, "TWS") || ...
                strcmpi(obj.sensorType, "AESA") || ...
                strcmpi(obj.sensorType, "FIRE_CONTROL"));
        end
    end
end
