function outPath = exportSingleSensorToJSON(state, fullPath)
%exportSingleSensorToJSON  Write the ACTIVE sensor (state.activeSensor)
%                          to a single sensor config JSON file.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker - UW Senior Capstone, Boeing-sponsored
%
%  Pure function - takes a complete output path. The caller (buildUI's
%  onSensorsSave callback) shows the uiputfile dialog and resolves the path.
%
%  SCHEMA (radar)  : { name, type, platform, frequency_hz, params:{...} }
%  SCHEMA (sonar)  : { name, type, platform, params:{ fov, sector, far,
%                      rangeLimits, rangeRes, detectionMode, updateRate,
%                      hasElevation, mountingLoc } }   (no frequency_hz)
%  SCHEMA (ir)     : { name, type, platform, params:{ rpm?, fov, sector,
%                      pd, far, rangeLimits, rangeRes, updateRate,
%                      hasElevation, mountingLoc } }   (no frequency_hz)
%
%  UNKNOWN PASSTHROUGH: readOnly sensors emit sr.originalDef verbatim.
%
%  See also: trackbench.editor.SensorRecord, trackbench.editor.sensorClass,
%            trackbench.editor.loadSensorsFromJSON,
%            trackbench.editor.exportSensorsToJSON

    arguments
        state    (1,1) trackbench.editor.EditorState
        fullPath (1,1) string
    end

    if ~state.hasActiveSensor()
        error('trackbench:editor:exportSingleSensorToJSON:noActiveSensor', ...
            ['No active sensor to save. Add a sensor first via the ' ...
             'Sensors panel.']);
    end
    if strlength(fullPath) == 0
        error('trackbench:editor:exportSingleSensorToJSON:emptyPath', ...
            'Output path must be non-empty.');
    end

    fullPath = char(fullPath);
    if ~endsWith(lower(fullPath), '.json')
        fullPath = [fullPath '.json'];
    end

    sr = state.activeSensor();
    if sr.readOnly && ~isempty(fieldnames(sr.originalDef))
        sDef = sr.originalDef;
    else
        sDef = buildSensorStruct(sr);
    end

    jsonStr = jsonencode(sDef, 'PrettyPrint', true);
    % Rename editor-only x_display_color -> _display_color (config convention).
    jsonStr = strrep(jsonStr, '"x_display_color"', '"_display_color"');

    parent = fileparts(fullPath);
    if ~isempty(parent) && ~exist(parent, 'dir')
        mkdir(parent);
    end

    fid = fopen(fullPath, 'w');
    if fid < 0
        error('trackbench:editor:exportSingleSensorToJSON:openFailed', ...
            'Could not open %s for writing.', fullPath);
    end
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fwrite(fid, jsonStr, 'char');

    sr.sourceFile = string(fullPath);
    state.setActiveSensor(sr);

    outPath = string(fullPath);
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function sDef = buildSensorStruct(sr)
%buildSensorStruct  SensorRecord -> on-disk schema struct. Mirrors
%  exportSensorsToJSON's same-named local helper; keep both in sync.
    sDef = struct();
    sDef.name     = char(sr.sensorName);
    sDef.type     = char(sr.sensorType);
    sDef.platform = char(sr.platform);

    p   = struct();
    cls = sr.sensorClass();

    if cls == "sonar"
        % buildSonarSensor params (no RF frequency_hz; acoustic chain).
        p.fov           = sr.fov(:)';
        p.sector        = sr.sectorDeg(:)';
        p.far           = sr.far;
        p.rangeLimits   = sr.rangeLimits(:)';
        if isfinite(sr.rangeResM) && sr.rangeResM > 0; p.rangeRes = sr.rangeResM; end
        p.detectionMode = char(sr.detectionMode);
        if sr.updateRate > 0; p.updateRate = sr.updateRate; end
        p.hasElevation  = logical(sr.hasElevation);
        sDef.params     = finishSensorParams(sr, p);
        return;
    elseif cls == "ir"
        % buildIR params (no RF frequency_hz; passive angle-only).
        if sr.rpm > 0; p.rpm = sr.rpm; end
        p.fov           = sr.fov(:)';
        p.sector        = sr.sectorDeg(:)';
        p.pd            = sr.pd;
        p.far           = sr.far;
        p.rangeLimits   = sr.rangeLimits(:)';
        if isfinite(sr.rangeResM) && sr.rangeResM > 0; p.rangeRes = sr.rangeResM; end
        if sr.updateRate > 0; p.updateRate = sr.updateRate; end
        p.hasElevation  = logical(sr.hasElevation);
        sDef.params     = finishSensorParams(sr, p);
        return;
    end

    % --- radar (behavior unchanged from the original) ---
    sDef.frequency_hz = sr.frequencyHz;

    isRot = sr.isRotator();
    isSec = sr.isSector();
    if isRot || isSec
        p.rpm    = sr.rpm;
        p.fov    = sr.fov(:)';
        p.tilt   = sr.tilt;
        p.sector = sr.sectorDeg(:)';
    else
        p.sector = sr.sectorDeg(:)';
        if all(isfinite(sr.fov)) && any(sr.fov > 0)
            p.fov = sr.fov(:)';
        end
        if isfinite(sr.tilt) && sr.tilt ~= 0
            p.tilt = sr.tilt;
        end
    end

    p.pd          = sr.pd;
    p.far         = sr.far;
    p.rangeLimits = sr.rangeLimits(:)';
    if isfinite(sr.rangeResM) && sr.rangeResM > 0
        p.rangeRes = sr.rangeResM;
    end

    sDef.params = finishSensorParams(sr, p);
end


function p = finishSensorParams(sr, p)
%finishSensorParams  Bake world position into mountingLoc + carry display
%  color (shared by all modalities; sim tower platform sits at origin so
%  mountingLoc IS the world offset).
    mZ = sr.mountingLoc(3);
    if ~isfinite(mZ); mZ = -15; end
    p.mountingLoc = [sr.positionEastM, sr.positionNorthM, mZ];
    if all(isfinite(sr.displayColor)) && numel(sr.displayColor) == 3
        p.x_display_color = sr.displayColor(:)';
    end
end
