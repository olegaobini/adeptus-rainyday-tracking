function outPath = exportSingleSensorToJSON(state, fullPath)
%exportSingleSensorToJSON  Write the ACTIVE sensor (state.activeSensor)
%                          to a single sensor config JSON file (v3.5
%                          step 4c).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Pure function — takes a complete output path. The caller (buildUI's
%  onSensorsSave callback) shows the uiputfile dialog and resolves the
%  user's chosen path. Same separation pattern as exportTerrainToJSON.
%
%  This is the single-sensor counterpart to exportSensorsToJSON, which
%  writes the WHOLE sensors collection (and a run file + targets +
%  environment) as a scenario bundle. Use this when the user wants
%  just one sensor written to disk, e.g. to share a tuned PSR config
%  with a teammate or to save a custom AESA without dragging targets
%  into it.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance. Must have an
%               active sensor (state.hasActiveSensor() == true).
%    fullPath : absolute path of the .json file to write. Missing
%               ".json" extension is appended automatically.
%
%  OUTPUT
%    outPath  : the actual path written.
%
%  SCHEMA (matches config/sensors/<TYPE>/<stem>.json)
%    {
%      "name":         "<sensorName>",
%      "type":         "<sensorType>",
%      "platform":     "tower" | "aircraft" | "ship",
%      "frequency_hz": <double>,
%      "params": { rpm, fov, tilt, sector, pd, far, rangeLimits,
%                  rangeRes (optional), mountingLoc, _display_color }
%    }
%
%  UNKNOWN PASSTHROUGH
%    For readOnly sensors the editor stores the original parsed struct
%    on sr.originalDef. We emit that verbatim instead of re-synthesizing
%    from the limited SensorRecord properties — keeps unsupported types
%    loadable after a Save → Reload round-trip.
%
%  DISPLAY COLOR ROUND-TRIP
%    The per-sensor display color is saved as "_display_color" (the
%    underscore prefix tells the sim pipeline to ignore the key, but
%    the editor's loader picks it back up so colors persist across a
%    Save → Reload). Same trick exportSensorsToJSON uses.
%
%  See also: trackbench.editor.SensorRecord,
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
    % Rename editor-only x_display_color → _display_color (matches the
    % underscore-prefix convention used in every config/sensors file).
    % Same trick exportSensorsToJSON uses; safe because no sensor param
    % is legitimately named "x_display_color".
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
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    % Stamp sourceFile so the Sensor Params panel "Load Sensor…" picker
    % defaults to this directory next time. sensorsDirty stays true if
    % OTHER sensors are unsaved — only the active sensor was saved here.
    sr.sourceFile = string(fullPath);
    state.setActiveSensor(sr);

    outPath = string(fullPath);
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function sDef = buildSensorStruct(sr)
%buildSensorStruct  SensorRecord → on-disk schema struct.
%
%  Mirrors exportSensorsToJSON's same-named local helper. If the schema
%  diverges, update both — they're intentionally redundant rather than
%  coupled.
    sDef = struct();
    sDef.name         = char(sr.sensorName);
    sDef.type         = char(sr.sensorType);
    sDef.platform     = char(sr.platform);
    sDef.frequency_hz = sr.frequencyHz;

    p = struct();

    % ── Scan-kind dispatch ────────────────────────────────────────────
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

    % World position baked into mountingLoc[0:1] (sim's tower platform
    % sits at origin, so mountingLoc IS the world offset).
    mZ = sr.mountingLoc(3);
    if ~isfinite(mZ); mZ = -15; end
    p.mountingLoc = [sr.positionEastM, sr.positionNorthM, mZ];

    if all(isfinite(sr.displayColor)) && numel(sr.displayColor) == 3
        p.x_display_color = sr.displayColor(:)';
    end

    sDef.params = p;
end
