function sr = sensorDefaults(typeStr)
%sensorDefaults  Fresh SensorRecord seeded with buildSensor-canonical
%                 defaults for the given type.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker - UW Senior Capstone, Boeing-sponsored
%
%  The ONE place the editor-supported sensor-type defaults live. Both
%  EditorState.addNewSensor and buildUI.onSensorTypeChanged call this.
%
%  SUPPORTED TYPES (editor-editable)
%    Radar: PSR, SSR, ASR, ARSR, PAR, MARITIME, WEATHER, TWS.
%    Sonar: ACTIVE_SONAR, PASSIVE_SONAR, TOWED_ARRAY.
%    IR:    IRST, IR_STARING, FLIR.
%    Anything else falls through to PSR-shaped guard values.
%
%  SOURCE OF TRUTH: trackbench.sensors.buildSensor.getDefaults. If those
%  change, update THIS function to match.
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
            sr.frequencyHz = 2.8e9;
            sr.rangeLimits = [0 111120];
            sr.rpm         = 12.5;
            sr.fov         = [1.4 5];
            sr.tilt        = 2;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 93;
        case "ARSR"
            sr.frequencyHz = 1.3e9;
            sr.rangeLimits = [0 463000];
            sr.rpm         = 5;
            sr.fov         = [1.5 20];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.85;
            sr.far         = 1e-6;
            sr.rangeResM   = 250;
        case "PAR"
            sr.frequencyHz = 9.0e9;
            sr.rangeLimits = [0 37040];
            sr.rpm         = 0;
            sr.fov         = [1.0 1.0];
            sr.tilt        = -3;
            sr.sectorDeg   = [170 190];
            sr.pd          = 0.95;
            sr.far         = 1e-7;
            sr.rangeResM   = 15;
        case "MARITIME"
            sr.frequencyHz = 9.4e9;
            sr.rangeLimits = [0 74080];
            sr.rpm         = 24;
            sr.fov         = [1.2 25];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.85;
            sr.far         = 1e-5;
            sr.rangeResM   = 25;
        case "WEATHER"
            sr.frequencyHz = 2.8e9;
            sr.rangeLimits = [0 463000];
            sr.rpm         = 25;
            sr.fov         = [1.0 1.0];
            sr.tilt        = 0;
            sr.sectorDeg   = [0 360];
            sr.pd          = 0.7;
            sr.far         = 1e-3;
            sr.rangeResM   = 250;
        case "TWS"
            sr.frequencyHz = 9.0e9;
            sr.rangeLimits = [0 200000];
            sr.rpm         = 0;
            sr.fov         = [60 30];
            sr.tilt        = 0;
            sr.sectorDeg   = [-60 60];
            sr.pd          = 0.9;
            sr.far         = 1e-6;
            sr.rangeResM   = 150;
        case "ACTIVE_SONAR"
            sr.frequencyHz   = 20000;        % acoustic center freq (Hz)
            sr.rangeLimits   = [0 20000];
            sr.rpm           = 0;
            sr.fov           = [360 90];
            sr.tilt          = 0;
            sr.sectorDeg     = [0 360];
            sr.pd            = 0.85;
            sr.far           = 1e-4;
            sr.rangeResM     = 50;
            sr.detectionMode = "monostatic";
            sr.updateRate    = 1;
            sr.hasElevation  = true;
        case "PASSIVE_SONAR"
            sr.frequencyHz   = 20000;
            sr.rangeLimits   = [0 50000];
            sr.rpm           = 0;
            sr.fov           = [360 20];
            sr.tilt          = 0;
            sr.sectorDeg     = [0 360];
            sr.pd            = 0.5;
            sr.far           = 1e-4;
            sr.rangeResM     = 500;
            sr.detectionMode = "passive";
            sr.updateRate    = 1;
            sr.hasElevation  = false;
        case "TOWED_ARRAY"
            sr.frequencyHz   = 20000;
            sr.rangeLimits   = [0 80000];
            sr.rpm           = 0;
            sr.fov           = [120 10];
            sr.tilt          = 0;
            sr.sectorDeg     = [120 240];
            sr.pd            = 0.6;
            sr.far           = 1e-4;
            sr.rangeResM     = 300;
            sr.detectionMode = "passive";
            sr.updateRate    = 0.5;
            sr.hasElevation  = false;
        case "IRST"
            sr.frequencyHz   = 0;            % IR: no RF frequency
            sr.rangeLimits   = [0 100000];
            sr.rpm           = 60;
            sr.fov           = [2 5];
            sr.tilt          = 0;
            sr.sectorDeg     = [0 360];
            sr.pd            = 0.8;
            sr.far           = 1e-6;
            sr.rangeResM     = 500;
            sr.updateRate    = 0;
            sr.hasElevation  = true;
        case "IR_STARING"
            sr.frequencyHz   = 0;
            sr.rangeLimits   = [0 50000];
            sr.rpm           = 0;
            sr.fov           = [90 60];
            sr.tilt          = 0;
            sr.sectorDeg     = [0 360];
            sr.pd            = 0.7;
            sr.far           = 1e-5;
            sr.rangeResM     = 1000;
            sr.updateRate    = 30;
            sr.hasElevation  = true;
        case "FLIR"
            sr.frequencyHz   = 0;
            sr.rangeLimits   = [0 30000];
            sr.rpm           = 0;
            sr.fov           = [10 8];
            sr.tilt          = 0;
            sr.sectorDeg     = [-20 20];
            sr.pd            = 0.85;
            sr.far           = 1e-6;
            sr.rangeResM     = 200;
            sr.updateRate    = 30;
            sr.hasElevation  = true;
        otherwise
            % Unknown type -> PSR-shaped guard values.
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
    if sr.sensorClass() == "sonar"
        sr.mountingLoc = [0 0 0];     % sonar: hydrophone at the surface (NED)
    else
        sr.mountingLoc = [0 0 -15];   % NED: 15 m above ground (standard tower)
    end
end
