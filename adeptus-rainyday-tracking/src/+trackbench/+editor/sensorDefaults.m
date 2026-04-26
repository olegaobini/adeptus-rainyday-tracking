function sr = sensorDefaults(typeStr)
%sensorDefaults  Fresh SensorRecord seeded with buildSensor-canonical
%                 defaults for the given type.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  This is the ONE place the 8 editor-supported sensor-type defaults
%  live. Both EditorState.addNewSensor (cold-add path) and
%  buildUI.onSensorTypeChanged (user changes type on an existing
%  sensor) call this — the pre-§3.6C code had two parallel copies of
%  this switch (one private to EditorState, one private to buildUI)
%  that had to be kept in sync by comment alone. They're now one.
%
%  SUPPORTED TYPES (editor-editable)
%    PSR, SSR, ASR, ARSR, PAR, MARITIME, WEATHER, TWS.
%    Anything else falls through to PSR-shaped defaults — the Add-modal
%    and type-DD are restricted, so fall-through is guard only.
%
%  SOURCE OF TRUTH
%    trackbench.sensors.buildSensor.getDefaults is the canonical
%    reference the simulator consumes. If buildSensor defaults change,
%    update THIS FUNCTION — not the now-deleted duplicates.
%
%  MountingLoc note: x/y components are zero because the editor stores
%  world placement on positionEastM/positionNorthM. The z component is
%  NED (negative = above ground); -15 = standard tower height.
%
%  See also: trackbench.editor.SensorRecord, trackbench.sensors.buildSensor

    sr = trackbench.editor.SensorRecord();
    sr.sensorType = upper(string(typeStr));
    sr.platform   = "tower";
    switch sr.sensorType
        case "PSR"
            sr.frequencyHz = 2.8e9;
            sr.rangeLimits = [0 111120];
            sr.rpm         = 12.5;
            sr.fov         = [1.4 30];
            sr.tilt        = 2;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 93;
        case "SSR"
            % Note: handoff text listed 1.03e9; buildSensor.getDefaults
            % uses 1.06e9 (SSR is the transponder uplink at 1.03 GHz and
            % downlink at 1.09 GHz — buildSensor chose the mid-band).
            % We match buildSensor.
            sr.frequencyHz = 1.06e9;
            sr.rangeLimits = [0 222240];
            sr.rpm         = 12.5;
            sr.fov         = [1.4 10];
            sr.tilt        = 2;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.99;
            sr.far         = 1e-7;
            sr.rangeResM   = 100;
        case "ASR"
            sr.frequencyHz = 2.8e9;   % S-band ASR-9/ASR-11
            sr.rangeLimits = [0 111120];
            sr.rpm         = 12.5;
            sr.fov         = [1.4 5];
            sr.tilt        = 2;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 93;
        case "ARSR"
            sr.frequencyHz = 1.3e9;   % L-band long range
            sr.rangeLimits = [0 463000];
            sr.rpm         = 5;
            sr.fov         = [1.5 20];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.85;
            sr.far         = 1e-6;
            sr.rangeResM   = 250;
        case "PAR"
            sr.frequencyHz = 9.0e9;   % X-band precision approach
            sr.rangeLimits = [0 37040];
            sr.rpm         = 0;       % sector scan, no continuous rotation
            sr.fov         = [1.0 1.0];
            sr.tilt        = -3;
            sr.sectorDeg   = [170 190];   % 20° azimuth sector
            sr.pd          = 0.95;
            sr.far         = 1e-7;
            sr.rangeResM   = 15;
        case "MARITIME"
            sr.frequencyHz = 9.4e9;   % X-band marine radar
            sr.rangeLimits = [0 74080];
            sr.rpm         = 24;
            sr.fov         = [1.2 25];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.85;
            sr.far         = 1e-5;
            sr.rangeResM   = 25;
        case "WEATHER"
            sr.frequencyHz = 2.8e9;   % S-band (NEXRAD convention)
            sr.rangeLimits = [0 463000];
            sr.rpm         = 25;
            sr.fov         = [1.0 1.0];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.7;
            sr.far         = 1e-3;
            sr.rangeResM   = 250;
        case "TWS"
            % 9.0e9 matches loadRunFile.getFreqForType('TWS'); handoff's
            % 10e9 was a stub value. sectorDeg = [-60 60] is the staring
            % phased-array electronic scan region (120° forward wedge),
            % not a full 360° — drawBeamCone2D needs the wedge angles
            % so TWS renders distinct from a mechanical rotator.
            sr.frequencyHz = 9.0e9;   % X-band fighter TWS
            sr.rangeLimits = [0 200000];
            sr.rpm         = 0;       % staring / no-scan
            sr.fov         = [60 30];
            sr.tilt        = 0;
            sr.sectorDeg   = [-60 60];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 150;
        otherwise
            % Unknown type → PSR-shaped guard values. Dropdown+modal
            % restrict input to the supported set, so this branch is
            % belt-and-braces only.
            sr.frequencyHz = 2.8e9;
            sr.rangeLimits = [0 111120];
            sr.rpm         = 12.5;
            sr.fov         = [1.4 30];
            sr.tilt        = 2;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 93;
    end
    sr.mountingLoc = [0 0 -15];   % NED: 15 m above ground (standard tower)
end
