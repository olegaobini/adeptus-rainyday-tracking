function [sensorPaths, runPath, targetsPath, excludedRefCount, envPaths] = ...
        exportSensorsToJSON(state, scenarioName)
%exportSensorsToJSON  Write the full multi-sensor scenario bundle (M6 §3.4).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Emits THREE groups of files in one operation:
%    1. One JSON per writable sensor  → config/sensors/<TYPE>/<sensorName>.json
%    2. One multi-target waypoints JSON (delegated to exportToJSON)
%                                     → config/targets/waypoints/<scenarioName>.json
%    3. One run file referencing both → config/runs/<scenarioName>.json
%
%  This is the milestone-6 replacement for the single-target-only Export
%  JSON button. The existing single-target exporter (exportToJSON.m) still
%  lives on and is invoked from here for the targets file — no JSON-shape
%  duplication. The sensor half is the new work: walking state.sensors and
%  laying out one file per sensor so loadRunFile's "sensors":[...] array of
%  string paths resolves cleanly.
%
%  INPUTS
%    state         : trackbench.editor.EditorState instance
%    scenarioName  : base filename stem (without extension) used for the
%                    targets JSON and the run JSON. Sensors each use their
%                    OWN sensorName regardless — this is just the bundle tag.
%
%  OUTPUTS
%    sensorPaths      : string array, one absolute path per exported sensor
%    runPath          : string, absolute path to the run file
%    targetsPath      : string, absolute path to the targets file
%    excludedRefCount : how many reference targets were filtered out (int),
%                        forwarded from exportToJSON for the success dialog.
%    envPaths         : struct with optional fields .terrain and .weather
%                        holding absolute paths when M7 environment files
%                        were emitted (M7 §3.4). Missing field = not
%                        written (weather=none case).
%
%  REFERENCE / READ-ONLY FILTERING
%    Writable targets are exported as before (delegated). Writable sensors
%    are exported here. Reference targets (readOnly) are filtered. There
%    is NO reference-sensor concept in M6 — "readOnly" on a sensor means
%    UNKNOWN passthrough (loaded, not editable, round-trips via originalDef
%    on re-export). Those are emitted verbatim so users don't lose data
%    when they Open Scenario → Export Scenario a file that happened to
%    contain an AESA/FLIR/etc.
%
%  POSITION ENCODING
%    Editor stores sensor world position on positionEastM/positionNorthM,
%    and mountingLoc(1:2) is kept zero in-memory (only mountingLoc(3) — the
%    altitude offset — is authored by the editor). On export we BAKE the
%    world position into mountingLoc[0:1] because the sim's tower platform
%    sits at origin (0,0,0) and mountingLoc IS the world offset that
%    loadRunFile hands to buildSensor. See SensorRecord header block.
%
%  DISPLAY COLOR ROUND-TRIP
%    An underscore-prefixed params key "_display_color" carries the
%    editor-only display color through to disk and back. The sim ignores
%    underscore-prefixed keys (buildSensor filters via nvPairs), so this
%    adds zero behavior at runtime but lets Open Scenario restore the
%    per-sensor color the user picked.
%
%  See also: trackbench.editor.exportToJSON,
%            trackbench.editor.loadSensorsFromJSON,
%            trackbench.editor.openScenarioFromJSON,
%            trackbench.config.loadRunFile

    arguments
        state        (1,1) trackbench.editor.EditorState
        scenarioName (1,1) string
    end

    if strlength(scenarioName) == 0
        error('trackbench:editor:exportSensorsToJSON:badName', ...
            'Scenario name must be non-empty.');
    end
    scenarioName = sanitizeStem(scenarioName);

    % ── Partition sensors (UNKNOWN passthrough still exports) ─────────
    %  Unlike targets, readOnly sensors ARE emitted — see header. What
    %  we filter for is "has something to write": every sensor currently
    %  in state.sensors goes to disk. Count the passthrough ones for the
    %  caller so the status line can note them separately.
    nSensors = numel(state.sensors);
    if nSensors == 0
        error('trackbench:editor:exportSensorsToJSON:noSensors', ...
            'No sensors to export. Add at least one sensor first.');
    end

    % ── Resolve the project root (used for all sub-path resolution) ───
    root = resolveProjectRoot(state);

    % ── Write each sensor's JSON ──────────────────────────────────────
    sensorPaths   = strings(1, nSensors);
    sensorRefs    = strings(1, nSensors);   % run-file "sensors" entries
    for k = 1:nSensors
        sr = state.sensors(k);
        [sensorPaths(k), sensorRefs(k)] = writeOneSensor(sr, root);
    end

    % ── Delegate targets export to the existing single-responsibility
    %    function. It already handles reference filtering, Nx5 → NED
    %    flip, curve-mode round-trip, and the duration_s rollup. We just
    %    need to override the output filename.
    [targetsPath, excludedRefCount] = ...
        trackbench.editor.exportToJSON(state, scenarioName);

    % ── M7 §3.4 — write environment files (terrain + optional weather)
    %   alongside the sensors+targets bundle. Returns a struct with
    %   .terrainRef (always) and .weatherRef ("none" when empty, else
    %   "<type>/<stem>") plus the absolute paths we wrote (for caller).
    envPaths = struct();
    [terrainRef, terrainFullPath] = writeTerrainFile( ...
        root, scenarioName, state);
    envPaths.terrain = terrainFullPath;
    [weatherRef, weatherFullPath] = writeWeatherFile( ...
        root, scenarioName, state);
    if weatherFullPath ~= ""
        envPaths.weather = weatherFullPath;
    end

    % ── Build the run file referencing sensors + targets + environment
    runPath = writeRunFile(root, scenarioName, sensorRefs, ...
        targetsPath, terrainRef, weatherRef, state);

    % Aggregate dirty bit — everything is now on disk.
    state.anyDirty = false;
    state.sensorsDirty = false;
    state.environmentDirty = false;
end


%% ========================================================================
%  Sensor writer
%% ========================================================================
function [outPath, runRef] = writeOneSensor(sr, root)
%writeOneSensor  Convert one SensorRecord → on-disk JSON matching
%                config/sensors/<TYPE>/<name>.json schema.
%
%  Returns the absolute path written AND the run-file reference shape
%  ("<TYPE>/<stem>") that the run file's "sensors":[] array should carry.
%
%  UNKNOWN PASSTHROUGH
%    For readOnly sensors the editor stores the original parsed struct on
%    sr.originalDef. We emit that verbatim (preserving any fields the
%    editor doesn't understand) instead of re-synthesizing from the
%    limited SensorRecord properties. This keeps AESA/FLIR/etc. loadable
%    after an Open Scenario → Export Scenario round trip.
    if sr.readOnly && ~isempty(fieldnames(sr.originalDef))
        sDef = sr.originalDef;
    else
        sDef = buildSensorStruct(sr);
    end

    typeDir = char(sr.sensorType);
    if isempty(typeDir)
        typeDir = 'UNKNOWN';
    end
    outDir = fullfile(root, "config", "sensors", typeDir);
    if ~exist(outDir, "dir")
        mkdir(outDir);
    end

    stem   = char(sanitizeStem(sr.sensorName));
    if isempty(stem); stem = 'sensor'; end
    outPath = string(fullfile(outDir, stem + ".json"));

    jsonStr = jsonencode(sDef, 'PrettyPrint', true);
    % Rename the editor-only x_display_color field to the underscore-
    % prefixed convention used everywhere else in the config tree
    % (_README, _frequency_hz, etc.). Quoted-key form is unambiguous so
    % a straight string replace is safe — there is no sensor param
    % legitimately named "x_display_color" in the sim pipeline.
    jsonStr = strrep(jsonStr, '"x_display_color"', '"_display_color"');
    fid = fopen(outPath, 'w');
    if fid < 0
        error('trackbench:editor:exportSensorsToJSON:openFailed', ...
            'Could not open %s for writing.', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    % Run-file reference: "<TYPE>/<stem>" (no extension — matches the
    % convention of every file in config/runs/*.json).
    runRef = string(typeDir) + "/" + string(stem);
end


function sDef = buildSensorStruct(sr)
%buildSensorStruct  Materialize a SensorRecord into the disk-schema
%                   struct. Field selection follows scan kind:
%                     rotator (rpm>0, 360°) — rpm/fov/tilt/sector
%                                              (sector auto = [0 360])
%                     sector  (rpm>0, <360°) — rpm/fov/tilt/sector
%                     no-scan (rpm<=0)       — sector only, plus fov when
%                                              non-default (staring arrays)
%
%  This follows the live on-disk examples: PSR has rpm/fov/tilt,
%  TWS/PAR/AESA have sector but omit rpm, MARITIME has rpm/fov/tilt
%  plus the full sector = [0 360] implicitly.
%
%  pd/far/rangeLimits/mountingLoc are ALWAYS emitted because
%  loadRunFile/buildSensor consume them unconditionally. rangeRes is
%  emitted when finite and positive (matches every existing default file).
    sDef = struct();
    sDef.name         = char(sr.sensorName);
    sDef.type         = char(sr.sensorType);
    sDef.platform     = char(sr.platform);
    sDef.frequency_hz = sr.frequencyHz;

    p = struct();

    % ── Scan-kind dispatch ────────────────────────────────────────────
    isRot    = sr.isRotator();
    isSec    = sr.isSector();
    if isRot || isSec
        p.rpm    = sr.rpm;
        p.fov    = sr.fov(:)';    % row vector, matches existing files
        p.tilt   = sr.tilt;
        p.sector = sr.sectorDeg(:)';
    else
        % No-scan / staring / PAR-style: only sector matters. fov is
        % still useful when the sensor is an electronically steered
        % array (TWS/AESA); emitting it lets buildSensor configure
        % beamwidth. For pure PAR-style the default fov=[1 1] is fine
        % to include.
        p.sector = sr.sectorDeg(:)';
        if all(isfinite(sr.fov)) && any(sr.fov > 0)
            p.fov = sr.fov(:)';
        end
        % tilt is still meaningful for no-scan (beam center offset).
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

    % Position encoding: bake world position into mountingLoc[0:1].
    mZ = sr.mountingLoc(3);
    if ~isfinite(mZ); mZ = -15; end
    p.mountingLoc = [sr.positionEastM, sr.positionNorthM, mZ];

    % Editor-only round-trip: carry the per-sensor display color through
    % to disk so Open Scenario → Export Scenario keeps user color picks.
    % MATLAB struct fieldnames can't begin with "_", so we use the
    % "x_display_color" spelling on the struct. writeOneSensor renames
    % the JSON key to "_display_color" after jsonencode so the on-disk
    % file uses the conventional underscore prefix that the sim pipeline
    % ignores (buildSensor filters unknown keys out of its name-value
    % varargin). loadSensorsFromJSON accepts both spellings on read.
    if all(isfinite(sr.displayColor)) && numel(sr.displayColor) == 3
        p.x_display_color = sr.displayColor(:)';
    end

    sDef.params = p;
end


%% ========================================================================
%  Run-file writer
%% ========================================================================
function outPath = writeRunFile(root, scenarioName, sensorRefs, targetsPath, terrainRef, weatherRef, state)
%writeRunFile  Author config/runs/<scenarioName>.json that references
%              the sensors, targets, terrain, and weather we just wrote.
%              Follows the run_template.json schema.
%
%  Targets reference is relative-to-config/targets: "waypoints/<stem>".
%  Sensors reference is relative-to-config/sensors: "<TYPE>/<stem>".
%  Terrain reference is relative-to-config/terrain: "<TYPE>/<stem>".
%  Weather reference is relative-to-config/weather: "<TYPE>/<stem>",
%                     or the literal string "none" when no weather.
%
%  M7 §3.4 — the degradation block is authored from state.degradation
%  (four booleans) + derived weather key. The optional
%  degradationExtras struct (e.g. rcs_range_filter passthrough from
%  an older run file opened earlier) is merged in so round-trips don't
%  strip unknown keys.
    runDir = fullfile(root, "config", "runs");
    if ~exist(runDir, "dir")
        mkdir(runDir);
    end

    runFile = struct();
    if state.description ~= ""
        runFile.description = char(state.description);
    else
        runFile.description = sprintf('Editor-exported scenario "%s" (%s)', ...
            scenarioName, char(datetime("now", "Format", "yyyy-MM-dd")));
    end

    % Sensors array — emit as a CELL array so jsonencode keeps it as a
    % JSON array even if there's only one sensor. Same trick used in
    % exportToJSON for single-target files.
    n = numel(sensorRefs);
    sCell = cell(1, n);
    for k = 1:n
        sCell{k} = char(sensorRefs(k));
    end
    runFile.sensors = sCell;

    % Targets reference: derive "waypoints/<stem>" from the absolute
    % path exportToJSON just returned. Robust across OS path separators.
    [~, tStem, ~] = fileparts(char(targetsPath));
    runFile.targets = sprintf('waypoints/%s', tStem);

    % M7 §3.4 — real terrain reference from state.
    runFile.terrain  = char(terrainRef);
    runFile.trackers = {'GNN/default_GNN'};

    % M7 §3.4 — authored degradation block (four booleans + derived
    % weather) plus any verbatim extras captured on Open Scenario.
    deg = struct();
    deg.terrain_occlusion = logical(state.degradation.terrain_occlusion);
    deg.horizon_masking   = logical(state.degradation.horizon_masking);
    deg.ground_clutter    = logical(state.degradation.ground_clutter);
    deg.doppler_fade      = logical(state.degradation.doppler_fade);
    deg.weather           = char(weatherRef);
    % Merge in verbatim extras (e.g. rcs_range_filter) captured by
    % openScenarioFromJSON into degradationExtras. We skip any keys we
    % already set above so the UI-authored values win.
    if ~isempty(state.degradationExtras) && isstruct(state.degradationExtras)
        extraKeys = fieldnames(state.degradationExtras);
        for k = 1:numel(extraKeys)
            key = extraKeys{k};
            if ~isfield(deg, key)
                deg.(key) = state.degradationExtras.(key);
            end
        end
    end
    runFile.degradation = deg;

    runFile.cache     = struct('use_cached_detections', false, ...
                               'save_detections',       true);
    runFile.platforms = struct();
    runFile.output    = struct('show_visuals',   true, ...
                               'animate_visuals', true, ...
                               'save_results',    true, ...
                               'globe_view',     false);

    jsonStr = jsonencode(runFile, 'PrettyPrint', true);

    outPath = string(fullfile(runDir, scenarioName + ".json"));
    fid = fopen(outPath, 'w');
    if fid < 0
        error('trackbench:editor:exportSensorsToJSON:openFailedRun', ...
            'Could not open %s for writing.', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');
end


%% ========================================================================
%  M7 §3.4 — Environment writers (terrain + weather)
%% ========================================================================

function [ref, outPath] = writeTerrainFile(root, scenarioName, state)
%writeTerrainFile  Emit config/terrain/<type>/<scenarioName>_terrain.json
%                   from state.terrain. Returns the run-file reference
%                   (e.g. "mountain/demo_checklist_terrain") + the
%                   absolute path written.
%
%  UNKNOWN PASSTHROUGH
%    readOnly=true terrain emits state.terrain.originalDef verbatim so
%    unfamiliar fields round-trip. Library files are never overwritten
%    because we always write to "<scenarioName>_terrain.json" (never
%    "default_*.json"). See handoff §3.4 naming convention.
    tr = state.terrain;
    typeDir = char(tr.terrainType);
    if isempty(typeDir); typeDir = 'unknown'; end

    outDir = fullfile(root, "config", "terrain", typeDir);
    if ~exist(outDir, "dir"); mkdir(outDir); end

    stem = char(sanitizeStem(scenarioName) + "_terrain");
    outPath = string(fullfile(outDir, stem + ".json"));

    if tr.readOnly && ~isempty(fieldnames(tr.originalDef))
        def = tr.originalDef;
    else
        def = buildTerrainStruct(tr);
    end

    jsonStr = jsonencode(def, 'PrettyPrint', true);
    fid = fopen(outPath, 'w');
    if fid < 0
        error('trackbench:editor:exportSensorsToJSON:openFailedTerrain', ...
            'Could not open %s for writing.', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    ref = string(typeDir) + "/" + string(stem);
end


function def = buildTerrainStruct(tr)
%buildTerrainStruct  TerrainRecord → on-disk schema struct. Mirrors the
%                     config/terrain/<TYPE>/default_<TYPE>.json layout.
    def = struct();
    def.description       = char(tr.description);
    def.terrain_type      = char(tr.terrainType);
    def.terrain_scale     = tr.terrainScale;
    def.clutter_density   = tr.clutterDensity;
    def.refraction_factor = tr.refractionFactor;
end


function [ref, outPath] = writeWeatherFile(root, scenarioName, state)
%writeWeatherFile  Emit config/weather/<type>/<scenarioName>_weather.json
%                   from state.weather. Returns the run-file reference
%                   ("<type>/<stem>" when configured, "none" when
%                   state.weather is empty) + the absolute path written
%                   (empty string when no file was written).
%
%  CLUTTER_MULTIPLIER OMISSION
%    fog and icing library files omit the clutter_multiplier key (see
%    default_fog.json, default_icing.json). WeatherRecord.
%    emitsClutterField() returns false for those types; this writer
%    honors it so round-trips with the library are symmetrical.
%
%  UNKNOWN PASSTHROUGH
%    readOnly=true weather emits state.weather.originalDef verbatim.
    if isempty(state.weather)
        ref = "none";
        outPath = "";
        return;
    end
    wr = state.weather;

    typeDir = char(wr.weatherType);
    if isempty(typeDir); typeDir = 'unknown'; end

    outDir = fullfile(root, "config", "weather", typeDir);
    if ~exist(outDir, "dir"); mkdir(outDir); end

    stem = char(sanitizeStem(scenarioName) + "_weather");
    outPath = string(fullfile(outDir, stem + ".json"));

    if wr.readOnly && ~isempty(fieldnames(wr.originalDef))
        def = wr.originalDef;
    else
        def = buildWeatherStruct(wr);
    end

    jsonStr = jsonencode(def, 'PrettyPrint', true);
    fid = fopen(outPath, 'w');
    if fid < 0
        error('trackbench:editor:exportSensorsToJSON:openFailedWeather', ...
            'Could not open %s for writing.', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    ref = string(typeDir) + "/" + string(stem);
end


function def = buildWeatherStruct(wr)
%buildWeatherStruct  WeatherRecord → on-disk schema struct. Mirrors
%                     config/weather/<TYPE>/default_<TYPE>.json layout.
%                     clutter_multiplier is OMITTED for types that
%                     don't emit it on disk (fog, icing).
    def = struct();
    def.type           = char(wr.weatherType);
    def.description    = char(wr.description);
    def.rain_rate_mmhr = wr.rainRateMmhr;
    def.storm_start_s  = wr.stormStartS;
    def.storm_end_s    = wr.stormEndS;
    def.active_type    = char(wr.activeType);
    def.pd_floor       = wr.pdFloor;
    if wr.emitsClutterField()
        def.clutter_multiplier = wr.clutterMultiplier;
    end
end


%% ========================================================================
%  Path helpers
%% ========================================================================
function root = resolveProjectRoot(state)
%resolveProjectRoot  Pick the project root that all config/ paths hang
%                    off of. state.projectRoot is authoritative when set
%                    (EditorState ctor populates it). Fall back to pwd
%                    so a detached script-launched editor still works.
    if state.projectRoot ~= "" && isfolder(state.projectRoot)
        root = state.projectRoot;
    else
        root = string(pwd);
    end
end


function stem = sanitizeStem(name)
%sanitizeStem  Drop path separators and unsafe filename chars. Matches
%              the editor's sanitizeName for sensor/target names except
%              we also strip any trailing ".json" so callers can pass
%              either form.
    s = char(name);
    s = regexprep(s, '\.json$', '', 'ignorecase');
    s = regexprep(s, '[\\/:*?"<>|]', '_');
    s = regexprep(s, '\s+', '_');
    stem = string(s);
end
