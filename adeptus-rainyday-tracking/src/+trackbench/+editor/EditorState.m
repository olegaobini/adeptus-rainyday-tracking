classdef EditorState < handle
%EditorState  Mutable state container for the multi-target path editor (M5).
%
%  M5 §3.1 RESHAPE
%    Pre-M5 (M1-M4): EditorState held one target. All scenario fields
%    (waypoints, targetName, rcs*, defaultSpeed*, defaultAltitude*,
%    curveMode, curveTensionAlpha, curveDensityPerSeg, durationS, isDirty)
%    sat directly on EditorState.
%
%    M5 §3.1: EditorState now owns a 1xN array of trackbench.editor.TargetRecord
%    in `targets` and a single active index in `activeIdx`. Per-target
%    scenario fields moved onto TargetRecord (see TargetRecord.m).
%
%    To keep the rest of the editor package and its callers working
%    without rewriting ~100 read/write sites, the per-target fields are
%    re-exposed here as DEPENDENT PROPERTIES that proxy to
%    activeTarget(). Reading state.waypoints returns the active target's
%    waypoints; writing state.waypoints = wp writes them back via the
%    value-class setActiveTarget pattern. drawMap.m is the one place
%    that needs to know about the underlying collection, because it
%    renders every target (active last); everywhere else continues to
%    operate on "the current target" through these dependent properties.
%
%    Internal mutators (addWaypoint, insertAfter, etc.) follow the
%    explicit read-mutate-writeback pattern the M5 handoff calls for —
%    they're the load-bearing path and benefit from being unambiguous.
%
%  WAYPOINT MATRIX FORMAT (unchanged from M4)
%    state.targets(k).waypoints is Nx5:
%      col 1 : x_m, col 2 : y_m, col 3 : altitude_m (positive up),
%      col 4 : time_s, col 5 : leg_speed_kmh.
%
%  See also: trackbench.editor.TargetRecord, trackbench.editor.buildUI,
%            trackbench.editor.drawMap, trackbench.editor.exportToJSON,
%            trackbench.editor.loadFromJSON, pathEditor
%
%  MATLAB NAMESPACE CACHE NOTE
%    After editing this file (or TargetRecord.m), run
%    `clear classes; clear all` before relaunching pathEditor.

    properties
        % ── Multi-target collection (M5 §3.1) ───────────────────────
        targets (1,:) trackbench.editor.TargetRecord = trackbench.editor.TargetRecord.empty
        activeIdx (1,1) double = 0          % 0 ⇒ no active target
        anyDirty  (1,1) logical = false     % aggregate dirty flag (any target unsaved)

        % ── Multi-sensor collection (M6 §3.1) ───────────────────────
        %  Parallel to the targets collection. Deliberately NO
        %  dependent-property proxies (unlike targets): no pre-M6 caller
        %  reads a "current sensor field" directly off state — all access
        %  goes through activeSensor()/setActiveSensor() read-mutate-
        %  writeback. Keeps the proxy layer from bloating further.
        sensors (1,:) trackbench.editor.SensorRecord = trackbench.editor.SensorRecord.empty
        activeSensorIdx (1,1) double = 0    % 0 ⇒ no active sensor
        editMode (1,1) string = "targets"    % "targets" | "sensors" | "environment"

        % ── Environment state (M7 §3.1) ─────────────────────────────
        %  Every scenario owns exactly one TerrainRecord (terrain is
        %  never empty; water is the "no terrain effect" baseline, not
        %  absence). Weather is optional — empty array when the
        %  scenario has no weather configured, or a single WeatherRecord
        %  when it does. Readers MUST isempty()-guard on `weather`.
        %
        %  `degradation` is a struct of four boolean toggles that map
        %  1:1 to the run file's degradation block keys:
        %    terrain_occlusion, horizon_masking, ground_clutter,
        %    doppler_fade.
        %  The fifth run-file degradation key — "weather" — is derived
        %  on export from isempty(weather) ? "none" : weather.weatherType,
        %  so it does NOT live in this struct.
        %
        %  `degradationExtras` carries verbatim unknown keys parsed from
        %  a loaded run file's degradation block (e.g. rcs_range_filter
        %  when present in a legacy file). The editor does not surface
        %  these in the UI but writes them back on Save Scenario so
        %  round-trips through Open→Save don't silently drop data. When
        %  empty struct, nothing extra is emitted.
        %
        %  `environmentDirty` parallels sensorsDirty so the onClose
        %  unsaved-changes prompt can detect environment-only edits
        %  (the terrain always exists and is equality-comparable, but
        %  tracking a dedicated flag matches the sensor pattern and
        %  keeps the comparison cheap).
        terrain (1,1) trackbench.editor.TerrainRecord = trackbench.editor.TerrainRecord()
        weather (1,:) trackbench.editor.WeatherRecord = trackbench.editor.WeatherRecord.empty   % 1x0 (none) or 1x1
        degradation (1,1) struct = struct( ...
            'terrain_occlusion', true, ...
            'horizon_masking',   true, ...
            'ground_clutter',    true, ...
            'doppler_fade',      true)
        degradationExtras (1,1) struct = struct()
        environmentDirty  (1,1) logical = false

        % ── File-level / editor-wide scenario fields ─────────────────
        description      (1,1) string = ""
        timingMode       (1,1) string = "auto"     % "auto" | "manual" (deferred)

        % ── View options (M3) ────────────────────────────────────────
        viewMode         (1,1) string = "2d"        % "2d" | "3d"
        gridSpacingKm    (1,1) double = 5
        splineMode       (1,1) logical = false
        colorByAltitude  (1,1) logical = false
        radarEastM       (1,1) double = 0
        radarNorthM      (1,1) double = 0
        has3DViewState   (1,1) logical = false

        % ── Selection + drag state ───────────────────────────────────
        selectedIndex    (1,1) double = 0          % 0 = none, else 1..N within active target
        hoverIndex       (1,1) double = 0
        dragActive       (1,1) logical = false
        dragStartWP      double = []
        selectHitRadiusM (1,1) double = 500

        % ── Middle-click pan state (2D only) ─────────────────────────
        panActive        (1,1) logical = false
        panStartFigPt    double = []
        panStartXLim     double = []
        panStartYLim     double = []

        % ── Sensor interaction state (M6 §3.5) ───────────────────────
        %  sensorPlacePending  — set by Place-on-map button; next axes
        %                        click in sensors mode teleports the active
        %                        sensor to the click point, then returns
        %                        to normal mode.
        %  sensorsDirty        — true iff sensors have been mutated since
        %                        the last load/export. Needed because the
        %                        onClose guard used to check only
        %                        count() > 0 (active-target waypoint
        %                        count), which missed sensor-only edits.
        %  sensorDragActive    — parallel to dragActive; drag state for the
        %                        active sensor. Kept separate so onMouseMove
        %                        can dispatch without mode-checking.
        %  sensorDragStart     — [x y] click point captured at mousedown so
        %                        downstream pushUndo/restore can roll back
        %                        a drag that was aborted mid-motion.
        sensorPlacePending (1,1) logical = false
        sensorsDirty       (1,1) logical = false
        sensorDragActive   (1,1) logical = false
        sensorDragStart    double = []

        % ── Undo/redo ────────────────────────────────────────────────
        undoStack        cell = {}
        redoStack        cell = {}
        maxUndoLevels    (1,1) double = 50

        % ── UI handles (populated by buildUI) ────────────────────────
        fig              = gobjects(1)
        ax               = gobjects(1)
        statusLabel      = gobjects(1)
        waypointCountLbl = gobjects(1)
        nameField        = gobjects(1)
        selectedPanel    = gobjects(1)
        selLabelIndex    = gobjects(1)
        selFieldX        = gobjects(1)
        selFieldY        = gobjects(1)
        selFieldAlt      = gobjects(1)
        selFieldSpeed    = gobjects(1)
        selFieldTime     = gobjects(1)
        selBtnDelete     = gobjects(1)
        selBtnInsertAfter= gobjects(1)

        % Scenario panel field handles — populated by buildUI so that
        % refreshScenarioPanel() (M5) can re-sync them when the active
        % target changes.
        speedField       = gobjects(1)
        altField         = gobjects(1)
        rcsField         = gobjects(1)
        rcsProfileDD     = gobjects(1)

        % ── M3 UI handles ────────────────────────────────────────────
        colorByAltCheckbox = gobjects(1)
        gridSpacingDD      = gobjects(1)
        viewModeBtn        = gobjects(1)
        radarEastField     = gobjects(1)
        radarNorthField    = gobjects(1)
        previewBtn         = gobjects(1)
        previewFig         = gobjects(1)

        % ── M4 UI handles ────────────────────────────────────────────
        curveModeBtn       = gobjects(1)
        curveTensionDD     = gobjects(1)

        % ── M5 UI handles (Targets sub-panel) ────────────────────────
        targetsDD          = gobjects(1)     % dropdown of targets
        targetsBtnNew      = gobjects(1)
        targetsBtnDuplicate= gobjects(1)
        targetsBtnDelete   = gobjects(1)
        targetsBtnRename   = gobjects(1)

        % v3.5 step 4b — NASA Flight import button (full-width row 3 of
        % the Targets sub-panel). Lives on its own row outside the
        % 4-button strip; see buildTargetsPanel for layout rationale.
        targetsBtnNasaFlight = gobjects(1)

        % v3.5 step 4c — file-I/O / overlay / destructive buttons moved
        % from the now-defunct File panel into the Targets sub-panel.
        % Each lives in its own panel-row group so the visual hierarchy
        % matches the operation class (file I/O / reference overlay /
        % destructive). See buildTargetsPanel for the full layout.
        targetsBtnLoad        = gobjects(1)   % "Load Target…" — was loadBtn in File panel
        targetsBtnSave        = gobjects(1)   % "Save Target…" — was exportBtn (the blue one)
        targetsBtnLoadRef     = gobjects(1)   % "Load as Reference…"
        targetsBtnUnloadRefs  = gobjects(1)   % "Unload References"
        targetsBtnClear       = gobjects(1)   % "Clear all waypoints"

        % ── M5 §3.2 — Scenario-panel handle ──────────────────────────
        %  buildUI captures the uipanel so refreshScenarioPanel can swap
        %  the Title text between "Scenario" and a read-only banner
        %  ("Scenario — REFERENCE (read-only; duplicate to edit)") when
        %  the active target is a reference. This is simpler than
        %  inserting a dedicated banner uilabel inside the already-tight
        %  Scenario grid layout.
        scenarioPanel      = gobjects(1)

        % ── v3.5 step 4a: panel handles + grid for mode-specific show/hide ──
        %  Pre-v3.5, applyEditMode only grayed out non-active panels via
        %  the Enable property. v3.5 step 4a hides them outright by
        %  flipping panel.Visible AND collapsing the parent grid's row
        %  height to 0. That requires capturing both the panel handles
        %  themselves AND the inner-grid handle (so RowHeight can be
        %  rewritten dynamically). The original heights are saved on
        %  load so applyEditMode can restore them when a panel becomes
        %  visible again — there's no other source of truth for them.
        %
        %  scenarioPanel / sensorParamsPanel / terrainPanel /
        %  weatherPanel / selectedPanel were already captured above for
        %  unrelated reasons (title swaps, refresh handles); the new
        %  fields below close the gap so every mode-gated panel has a
        %  handle.
        sensorsPanel       = gobjects(1)     % Sensors dropdown panel (row 2)
        targetsPanel       = gobjects(1)     % Targets dropdown panel (row 4)
        editorInnerGrid    = gobjects(1)     % parent uigridlayout for sub-panels
        editorInnerOriginalRowHeights cell = {}   % original RowHeight cell, captured at build time

        % ── M6 UI handles (Mode-toggle sub-panel) ────────────────────
        modeTargetsBtn     = gobjects(1)     % uibutton('state') — Targets mode
        modeSensorsBtn     = gobjects(1)     % uibutton('state') — Sensors mode

        % ── M6 UI handles (Sensors sub-panel) ────────────────────────
        sensorsDD          = gobjects(1)     % dropdown of sensors
        sensorsBtnAdd      = gobjects(1)
        sensorsBtnDuplicate= gobjects(1)
        sensorsBtnDelete   = gobjects(1)
        sensorsBtnRename   = gobjects(1)

        % v3.5 step 4c — sensor file-I/O buttons moved from the now-
        % defunct File panel into the Sensors sub-panel. Load APPENDS
        % a sensor to the collection (existing onLoadSensors semantics);
        % Save writes the ACTIVE sensor only (single-sensor counterpart
        % to the scenario-bundle Export, via exportSingleSensorToJSON).
        sensorsBtnLoad     = gobjects(1)   % "Load Sensor…"
        sensorsBtnSave     = gobjects(1)   % "Save Sensor…"

        % ── M6 UI handles (Sensor Parameters panel) ──────────────────
        %  These are populated by buildUI's buildSensorParamsPanel. All
        %  default to gobjects(1) so setIfGraphics() short-circuits when
        %  the panel hasn't been built yet (e.g., snapshot restore that
        %  runs before buildUI finishes).
        sensorParamsPanel   = gobjects(1)
        sensorNameField     = gobjects(1)
        sensorTypeDD        = gobjects(1)
        sensorEastField     = gobjects(1)
        sensorNorthField    = gobjects(1)
        sensorAltField      = gobjects(1)
        sensorFreqField     = gobjects(1)
        sensorMaxRangeField = gobjects(1)
        sensorRangeResField = gobjects(1)
        sensorAzFovField    = gobjects(1)
        sensorElFovField    = gobjects(1)
        sensorTiltField     = gobjects(1)
        sensorScanModeLbl   = gobjects(1)
        sensorRpmField      = gobjects(1)
        sensorRpmLabel      = gobjects(1)
        sensorSectorLoField = gobjects(1)
        sensorSectorLoLabel = gobjects(1)
        sensorSectorHiField = gobjects(1)
        sensorSectorHiLabel = gobjects(1)
        sensorPdField       = gobjects(1)
        sensorFarField      = gobjects(1)
        sensorPlaceOnMapBtn = gobjects(1)

        % ── M7 UI handles (Mode-toggle — Environment button) ────────
        modeEnvironmentBtn    = gobjects(1)     % uibutton('state')

        % ── M7 UI handles (Terrain sub-panel) ─────────────────────────
        %  All terrain-panel widgets are captured so refreshTerrainPanel
        %  can rebind values when the active scenario's terrain changes
        %  (file load, undo, type cascade). The overlay toggle drives
        %  drawMap's tint rendering; the degradation checkboxes write
        %  into state.degradation.
        terrainPanel          = gobjects(1)
        terrainTypeDD         = gobjects(1)
        terrainDescField      = gobjects(1)
        terrainScaleField     = gobjects(1)
        terrainClutterField   = gobjects(1)
        terrainRefractionField= gobjects(1)
        terrainLoadBtn        = gobjects(1)
        terrainSaveBtn        = gobjects(1)   % v3.5 step 4c
        terrainOverlayCB      = gobjects(1)
        degTerrainOcclusionCB = gobjects(1)
        degHorizonMaskingCB   = gobjects(1)
        degGroundClutterCB    = gobjects(1)
        degDopplerFadeCB      = gobjects(1)

        % ── M7 UI handles (Weather sub-panel) ─────────────────────────
        %  weatherRateLabel's text changes per weather type (see
        %  refreshWeatherPanel in buildUI) because rain_rate_mmhr means
        %  different things per type. weatherStormSparkline is a small
        %  uiaxes that renders the three storm profiles as a 120x36px
        %  glyph strip over the storm_start..storm_end interval.
        weatherPanel          = gobjects(1)
        weatherTypeDD         = gobjects(1)
        weatherDescField      = gobjects(1)
        weatherRateField      = gobjects(1)
        weatherRateLabel      = gobjects(1)
        weatherStormStartField= gobjects(1)
        weatherStormEndField  = gobjects(1)
        weatherProfileDD      = gobjects(1)
        weatherPdFloorField   = gobjects(1)
        weatherClutterMultField = gobjects(1)
        weatherClutterMultLabel = gobjects(1)
        weatherStormSparkline = gobjects(1)
        weatherLoadBtn        = gobjects(1)
        weatherSaveBtn        = gobjects(1)   % v3.5 step 4c

        % ── M7 §3.3 — storm-window timeline axes (main map column) ──
        %  A second uiaxes placed below the main map axes and above the
        %  status bar, showing scenario duration as a horizontal bar with
        %  the storm window highlighted. Always present in the layout;
        %  Visible flips off when state.weather is empty.
        weatherStormTimelineAx = gobjects(1)

        % ── Callback stashes ─────────────────────────────────────────
        axesClickFcn       function_handle = function_handle.empty

        % ── Paths ────────────────────────────────────────────────────
        projectRoot      (1,1) string = ""
        outputDir        (1,1) string = ""
        loadedFrom       (1,1) string = ""
    end


    %% ── Dependent properties: legacy per-target field proxies ───────
    %  These keep state.waypoints / state.targetName / state.curveMode /
    %  etc. working in every M1-M4 caller. Reads forward to the active
    %  TargetRecord; writes do the read-mutate-writeback dance. If
    %  there's no active target a write becomes a no-op (better than
    %  erroring on UI callbacks that fire during transient empty state).
    properties (Dependent)
        waypoints
        targetName
        rcsDbsm
        rcsProfile
        defaultSpeedKmh
        defaultAltitudeM
        curveMode
        curveTensionAlpha
        curveDensityPerSeg
        durationS
        isDirty
    end


    methods
        function obj = EditorState(projectRoot)
            %EditorState  Construct with the project root for path resolution.
            %             Always seeds with one default editable target so
            %             buildUI's first read of state.targetName etc.
            %             succeeds (matches the M1-M4 launch behavior of
            %             "editor opens with target_1 ready to receive clicks").
            if nargin >= 1 && ~isempty(projectRoot)
                obj.projectRoot = string(projectRoot);
                obj.outputDir = fullfile(obj.projectRoot, ...
                    "config", "targets", "waypoints");
            end
            seed = trackbench.editor.TargetRecord();
            seed.targetName = "m1_test";
            obj.targets   = seed;
            obj.activeIdx = 1;
        end

        %% ── Active-target accessors (M5) ───────────────────────────
        function tr = activeTarget(obj)
            %activeTarget  Return the currently active TargetRecord (value).
            if obj.activeIdx < 1 || obj.activeIdx > numel(obj.targets)
                error('trackbench:editor:EditorState:noActiveTarget', ...
                    'No active target. Create one first.');
            end
            tr = obj.targets(obj.activeIdx);
        end

        function setActiveTarget(obj, tr)
            %setActiveTarget  Write a value-class TargetRecord back into
            %                 the targets array at activeIdx. Bumps anyDirty.
            if obj.activeIdx < 1 || obj.activeIdx > numel(obj.targets)
                error('trackbench:editor:EditorState:noActiveTarget', ...
                    'No active target. Create one first.');
            end
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function tf = hasActiveTarget(obj)
            tf = obj.activeIdx >= 1 && obj.activeIdx <= numel(obj.targets);
        end

        function tf = activeIsReadOnly(obj)
            %activeIsReadOnly  True iff the active target exists AND is
            %                  marked readOnly (M5 §3.2 reference target).
            %                  Mutators short-circuit on this so a reference
            %                  target can never be edited in place — the
            %                  user must Duplicate first to get a writable
            %                  copy. The UI also disables Scenario fields
            %                  when this returns true (defense in depth).
            tf = obj.hasActiveTarget() && obj.targets(obj.activeIdx).readOnly;
        end

        %% ── Active-sensor accessors (M6 §3.1) ─────────────────────
        function sr = activeSensor(obj)
            %activeSensor  Return the currently active SensorRecord (value).
            if obj.activeSensorIdx < 1 || obj.activeSensorIdx > numel(obj.sensors)
                error('trackbench:editor:EditorState:noActiveSensor', ...
                    'No active sensor. Add one first.');
            end
            sr = obj.sensors(obj.activeSensorIdx);
        end

        function setActiveSensor(obj, sr)
            %setActiveSensor  Write a value-class SensorRecord back into
            %                 the sensors array at activeSensorIdx. Bumps
            %                 anyDirty (sensors participate in the aggregate
            %                 save-state for the run file).
            if obj.activeSensorIdx < 1 || obj.activeSensorIdx > numel(obj.sensors)
                error('trackbench:editor:EditorState:noActiveSensor', ...
                    'No active sensor. Add one first.');
            end
            obj.sensors(obj.activeSensorIdx) = sr;
            obj.anyDirty = true;
            obj.sensorsDirty = true;
        end

        function tf = hasActiveSensor(obj)
            tf = obj.activeSensorIdx >= 1 && obj.activeSensorIdx <= numel(obj.sensors);
        end

        function tf = activeSensorIsReadOnly(obj)
            %activeSensorIsReadOnly  True iff the active sensor exists AND
            %                         is marked readOnly. Used both for
            %                         UNKNOWN passthrough (unsupported
            %                         type/platform loaded from file) and
            %                         for reference sensors in later work.
            %                         Mutators short-circuit on this.
            tf = obj.hasActiveSensor() && obj.sensors(obj.activeSensorIdx).readOnly;
        end

        %% ── Sensors-collection mutators (M6 §3.1) ─────────────────
        function newIdx = addNewSensor(obj, typeStr)
            %addNewSensor  Append a fresh SensorRecord of the given type,
            %              seeded with trackbench.editor.sensorDefaults
            %              values matching buildSensor's getDefaults.
            %              Auto-names "sensor_<n+1>" and uniquifies if that
            %              collides. Defaults to PSR if typeStr is omitted.
            %
            %  M6 §3.6C  The defaults live in a package-level function now
            %  (trackbench.editor.sensorDefaults) so buildUI's type-change
            %  path and this cold-add path share one switch.
            if nargin < 2 || strlength(string(typeStr)) == 0
                typeStr = "PSR";
            end
            obj.pushUndo();
            sr = trackbench.editor.sensorDefaults(typeStr);
            sr.sensorName = uniquifySensorName(obj, ...
                sprintf("sensor_%d", numel(obj.sensors) + 1));
            obj.sensors(end+1) = sr;
            obj.activeSensorIdx = numel(obj.sensors);
            obj.anyDirty = true;
            obj.sensorsDirty = true;
            newIdx = obj.activeSensorIdx;
        end

        function newIdx = duplicateActiveSensor(obj)
            %duplicateActiveSensor  Copy the active SensorRecord, suffix
            %                       "_copy" on the name, offset +2 km east
            %                       so the copy doesn't render on top of
            %                       the original. The copy is always
            %                       editable (readOnly cleared); any
            %                       UNKNOWN-passthrough originalDef is
            %                       discarded because the editor only
            %                       owns supported-type sensors going
            %                       forward from a duplicate.
            if ~obj.hasActiveSensor()
                error('trackbench:editor:EditorState:noActiveSensor', ...
                    'No sensor to duplicate.');
            end
            obj.pushUndo();
            src = obj.activeSensor();
            cp = src;                              % value-class copy
            cp.sensorName    = uniquifySensorName(obj, src.sensorName + "_copy");
            cp.readOnly      = false;
            cp.sourceFile    = "";
            cp.positionEastM = src.positionEastM + 2000;   % +2 km east
            cp.originalDef   = struct();
            obj.sensors(end+1) = cp;
            obj.activeSensorIdx = numel(obj.sensors);
            obj.anyDirty = true;
            obj.sensorsDirty = true;
            newIdx = obj.activeSensorIdx;
        end

        function deleteActiveSensor(obj)
            %deleteActiveSensor  Remove the active SensorRecord. If this
            %                    was the last sensor, activeSensorIdx → 0
            %                    (editor enters the "no sensors" state —
            %                    map still renders targets, panel disables).
            if ~obj.hasActiveSensor()
                return;
            end
            obj.pushUndo();
            obj.sensors(obj.activeSensorIdx) = [];
            if isempty(obj.sensors)
                obj.activeSensorIdx = 0;
            else
                obj.activeSensorIdx = max(1, min(obj.activeSensorIdx, numel(obj.sensors)));
            end
            obj.anyDirty = true;
            obj.sensorsDirty = true;
        end

        function renameActiveSensor(obj, newName)
            %renameActiveSensor  Rename the active sensor. Validates
            %                    non-empty, alphanumeric, and uniqueness.
            %                    Read-only sensors (UNKNOWN passthrough)
            %                    cannot be renamed — duplicate first.
            if ~obj.hasActiveSensor()
                return;
            end
            if obj.activeSensorIsReadOnly()
                error('trackbench:editor:EditorState:readOnlySensor', ...
                    'Cannot rename a read-only sensor. Duplicate it first.');
            end
            cleaned = sanitizeName(newName);
            if strlength(cleaned) == 0
                error('trackbench:editor:EditorState:badName', ...
                    'Sensor name cannot be empty.');
            end
            current = obj.activeSensor().sensorName;
            if cleaned == current
                return;   % no-op rename — don't burn an undo step
            end
            if sensorNameExists(obj, cleaned)
                error('trackbench:editor:EditorState:duplicateName', ...
                    'A sensor named "%s" already exists.', cleaned);
            end
            obj.pushUndo();
            sr = obj.activeSensor();
            sr.sensorName = cleaned;
            obj.setActiveSensor(sr);
        end

        function setActiveSensorIdx(obj, newIdx)
            %setActiveSensorIdx  Switch the active sensor. Does not push
            %                    undo (view state, like setActiveIdx).
            if isempty(obj.sensors)
                obj.activeSensorIdx = 0;
                return;
            end
            newIdx = max(1, min(numel(obj.sensors), round(newIdx)));
            if newIdx == obj.activeSensorIdx
                return;
            end
            obj.activeSensorIdx = newIdx;
        end

        function setEditMode(obj, mode)
            %setEditMode  Switch between "targets", "sensors", and
            %             "environment" editing modes. View state only —
            %             does not push undo. Unrecognized modes are
            %             ignored (defense against unexpected toggle
            %             states).
            mode = lower(string(mode));
            if mode ~= "targets" && mode ~= "sensors" && mode ~= "environment"
                return;
            end
            obj.editMode = mode;
        end

        function idx = findSensorAt(obj, x, y, maxDistM)
            %findSensorAt  Hit-test sensor world positions. Returns the
            %              index of the nearest sensor within maxDistM of
            %              (x, y), or 0 if no sensor is in range. Used by
            %              sensor-mode click routing (§3.5).
            if nargin < 4 || isempty(maxDistM); maxDistM = obj.hitRadiusM(); end
            idx = 0;
            if isempty(obj.sensors); return; end
            n = numel(obj.sensors);
            dx = zeros(1, n);
            dy = zeros(1, n);
            for k = 1:n
                dx(k) = obj.sensors(k).positionEastM - x;
                dy(k) = obj.sensors(k).positionNorthM - y;
            end
            d = sqrt(dx.*dx + dy.*dy);
            [minD, k] = min(d);
            if minD <= maxDistM
                idx = k;
            end
        end

        function moveActiveSensorTo(obj, x, y, commit)
            %moveActiveSensorTo  Re-point the active sensor's
            %                    (positionEastM, positionNorthM) to (x, y).
            %                    commit=true pushes undo (intended for the
            %                    start of a drag or a teleport from
            %                    Place-on-map); commit=false is the
            %                    live-drag motion update that should not
            %                    burn an undo slot per move. Mirrors
            %                    moveSelectedTo for waypoints.
            if ~obj.hasActiveSensor(); return; end
            if obj.activeSensorIsReadOnly(); return; end
            if nargin < 4; commit = true; end
            if commit; obj.pushUndo(); end
            sr = obj.sensors(obj.activeSensorIdx);
            sr.positionEastM  = x;
            sr.positionNorthM = y;
            obj.sensors(obj.activeSensorIdx) = sr;
            obj.anyDirty = true;
            obj.sensorsDirty = true;
        end

        function nudgeActiveSensor(obj, dx, dy)
            %nudgeActiveSensor  Keyboard-nudge the active sensor by
            %                   (dx, dy) metres. Pushes undo once per
            %                   nudge (each arrow-keypress is a discrete
            %                   user action, so undo-per-press is the
            %                   intuitive behavior). Mirrors
            %                   nudgeSelectedXY's pattern.
            if ~obj.hasActiveSensor(); return; end
            if obj.activeSensorIsReadOnly(); return; end
            obj.pushUndo();
            sr = obj.sensors(obj.activeSensorIdx);
            sr.positionEastM  = sr.positionEastM  + dx;
            sr.positionNorthM = sr.positionNorthM + dy;
            obj.sensors(obj.activeSensorIdx) = sr;
            obj.anyDirty = true;
            obj.sensorsDirty = true;
        end

        function didAbort = abortSensorDrag(obj)
            %abortSensorDrag  Revert an in-progress sensor drag to the
            %                  position captured at mousedown, then clear
            %                  the drag state and pop the undo snapshot
            %                  that was pushed at drag start. Returns
            %                  true if a drag was actually aborted, false
            %                  if there was nothing to abort (so the
            %                  caller can fall through to the default
            %                  Escape behavior).
            %
            %  WHY POP UNDO  We pushUndo'd at drag start expecting the
            %  drag to commit on mouseup. An aborted drag should be a
            %  no-op on history — trim the top of undoStack so Ctrl+Z
            %  doesn't step back into a state that's identical to the
            %  current one.
            didAbort = false;
            if ~obj.sensorDragActive; return; end
            if ~isempty(obj.sensorDragStart) && obj.hasActiveSensor() ...
                    && ~obj.activeSensorIsReadOnly()
                sr = obj.sensors(obj.activeSensorIdx);
                sr.positionEastM  = obj.sensorDragStart(1);
                sr.positionNorthM = obj.sensorDragStart(2);
                obj.sensors(obj.activeSensorIdx) = sr;
            end
            if ~isempty(obj.undoStack)
                obj.undoStack(end) = [];
            end
            obj.sensorDragActive = false;
            obj.sensorDragStart  = [];
            didAbort = true;
        end

        %% ── Environment mutators (M7 §3.1) ─────────────────────────
        function setTerrainType(obj, typeStr)
            %setTerrainType  Cascade-reset the active scenario's terrain
            %                to the disk defaults for a new type. Fresh
            %                TerrainRecord via terrainDefaults — wipes
            %                any prior edits to the terrain fields. The
            %                UI-side type-change callback is expected to
            %                have already confirmed the reset via uiconfirm.
            %
            %  This pushes undo (type change is a discrete user action).
            %  readOnly terrain (UNKNOWN passthrough loaded from file)
            %  cannot be retyped in place — the scenario-level Load
            %  Terrain flow replaces it wholesale.
            if obj.terrain.readOnly
                error('trackbench:editor:EditorState:readOnlyTerrain', ...
                    'Cannot change type of a read-only terrain. Load a different terrain file instead.');
            end
            obj.pushUndo();
            obj.terrain = trackbench.editor.terrainDefaults(typeStr);
            obj.anyDirty = true;
            obj.environmentDirty = true;
        end

        function setTerrainField(obj, fieldName, value)
            %setTerrainField  Field-level mutator for the Terrain
            %                 sub-panel's scalar widgets. fieldName is
            %                 one of: description, terrainScale,
            %                 clutterDensity, refractionFactor. Pushes
            %                 undo on each call so Ctrl+Z steps back
            %                 past a field edit. Callers MUST NOT
            %                 pushUndo themselves or each edit will
            %                 consume two undo slots.
            %                 No-ops on readOnly terrain.
            if obj.terrain.readOnly; return; end
            obj.pushUndo();
            tr = obj.terrain;
            switch string(fieldName)
                case "description";      tr.description      = string(value);
                case "terrainScale";     tr.terrainScale     = double(value);
                case "clutterDensity";   tr.clutterDensity   = double(value);
                case "refractionFactor"; tr.refractionFactor = double(value);
                otherwise
                    error('trackbench:editor:EditorState:badTerrainField', ...
                        'Unknown terrain field "%s".', string(fieldName));
            end
            obj.terrain = tr;
            obj.anyDirty = true;
            obj.environmentDirty = true;
        end

        function setWeatherType(obj, typeStr)
            %setWeatherType  Switch the scenario's weather. typeStr of
            %                "none" (case-insensitive) clears the weather
            %                record entirely (state.weather = []); any
            %                supported type value creates a fresh
            %                WeatherRecord seeded by weatherDefaults.
            %                The UI-side callback is expected to have
            %                already confirmed the reset via uiconfirm
            %                when a configured weather exists.
            %
            %  readOnly weather (UNKNOWN passthrough) cannot be retyped
            %  in place — the scenario-level Load Weather flow replaces
            %  it wholesale. "none" IS allowed even when the current
            %  weather is readOnly — clearing to (none) is not an in-
            %  place edit.
            typeStr = lower(string(typeStr));
            obj.pushUndo();
            if typeStr == "none"
                obj.weather = trackbench.editor.WeatherRecord.empty;
            else
                if ~isempty(obj.weather) && obj.weather.readOnly
                    error('trackbench:editor:EditorState:readOnlyWeather', ...
                        'Cannot change type of a read-only weather. Load a different weather file or switch to (none) first.');
                end
                obj.weather = trackbench.editor.weatherDefaults(typeStr);
            end
            obj.anyDirty = true;
            obj.environmentDirty = true;
        end

        function setWeatherField(obj, fieldName, value)
            %setWeatherField  Field-level mutator for the Weather
            %                 sub-panel. No-op when state.weather is
            %                 empty (the UI disables fields under the
            %                 dropdown when type=(none), but defense
            %                 in depth). Pushes undo on each call.
            %
            %  fieldName is one of: description, rainRateMmhr,
            %  stormStartS, stormEndS, activeType, pdFloor,
            %  clutterMultiplier.
            if isempty(obj.weather); return; end
            if obj.weather.readOnly; return; end
            obj.pushUndo();
            wr = obj.weather;
            switch string(fieldName)
                case "description";       wr.description       = string(value);
                case "rainRateMmhr";      wr.rainRateMmhr      = double(value);
                case "stormStartS";       wr.stormStartS       = double(value);
                case "stormEndS";         wr.stormEndS         = double(value);
                case "activeType";        wr.activeType        = string(value);
                case "pdFloor";           wr.pdFloor           = double(value);
                case "clutterMultiplier"; wr.clutterMultiplier = double(value);
                otherwise
                    error('trackbench:editor:EditorState:badWeatherField', ...
                        'Unknown weather field "%s".', string(fieldName));
            end
            obj.weather = wr;
            obj.anyDirty = true;
            obj.environmentDirty = true;
        end

        function setDegradationToggle(obj, keyName, tfVal)
            %setDegradationToggle  Flip one of the four degradation
            %                      booleans. keyName is one of the run
            %                      file's key strings: terrain_occlusion,
            %                      horizon_masking, ground_clutter,
            %                      doppler_fade. Pushes undo.
            %
            %  The fifth run-file degradation field — "weather" — is NOT
            %  a boolean and is NOT routed here. It's derived on export
            %  from isempty(state.weather).
            keyName = lower(string(keyName));
            valid = ["terrain_occlusion", "horizon_masking", ...
                     "ground_clutter",    "doppler_fade"];
            if ~any(keyName == valid)
                error('trackbench:editor:EditorState:badDegradationKey', ...
                    'Unknown degradation key "%s".', keyName);
            end
            obj.pushUndo();
            obj.degradation.(keyName) = logical(tfVal);
            obj.anyDirty = true;
            obj.environmentDirty = true;
        end

        function loadTerrainFromFile(obj, relPath)
            %loadTerrainFromFile  Replace the active scenario's terrain
            %                     with one parsed from a file on disk.
            %                     relPath is either a full path or one
            %                     relative to config/terrain/ (e.g.
            %                     "mountain/my_terrain"). Delegates the
            %                     parse to loadTerrainFromJSON (created
            %                     in §3.4) and pushes undo. On parse
            %                     error, leaves state.terrain untouched
            %                     and rethrows.
            obj.pushUndo();
            try
                obj.terrain = trackbench.editor.loadTerrainFromJSON( ...
                    obj.projectRoot, relPath);
                obj.anyDirty = true;
                obj.environmentDirty = true;
            catch ME
                % Roll back the undo push on parse failure — no state
                % change means no history entry.
                if ~isempty(obj.undoStack)
                    obj.undoStack(end) = [];
                end
                rethrow(ME);
            end
        end

        function loadWeatherFromFile(obj, relPath)
            %loadWeatherFromFile  Replace the active scenario's weather
            %                     with one parsed from a file on disk.
            %                     Same semantics as loadTerrainFromFile.
            %                     Always results in a 1x1 weather record
            %                     (never empty — callers wanting to
            %                     clear weather should use
            %                     setWeatherType("none")).
            obj.pushUndo();
            try
                obj.weather = trackbench.editor.loadWeatherFromJSON( ...
                    obj.projectRoot, relPath);
                obj.anyDirty = true;
                obj.environmentDirty = true;
            catch ME
                if ~isempty(obj.undoStack)
                    obj.undoStack(end) = [];
                end
                rethrow(ME);
            end
        end

        %% ── Targets-collection mutators (M5) ──────────────────────
        function newIdx = addNewTarget(obj, name)
            %addNewTarget  Append a fresh TargetRecord with default fields.
            %              Auto-names "target_<n+1>" if name is omitted.
            obj.pushUndo();
            tr = trackbench.editor.TargetRecord();
            if nargin < 2 || strlength(string(name)) == 0
                tr.targetName = sprintf("target_%d", numel(obj.targets) + 1);
            else
                tr.targetName = sanitizeName(name);
            end
            tr.targetName = uniquifyName(obj, tr.targetName);
            tr.displayColor = nextDisplayColor(numel(obj.targets) + 1);
            obj.targets(end+1) = tr;
            obj.activeIdx = numel(obj.targets);
            obj.selectedIndex = 0;
            obj.anyDirty = true;
            newIdx = obj.activeIdx;
        end

        function newIdx = duplicateActiveTarget(obj)
            %duplicateActiveTarget  Copy the active TargetRecord, append
            %                       "_copy" to its name, make the copy
            %                       active. The copy is always editable
            %                       (readOnly cleared) so users can riff
            %                       on a reference path.
            if ~obj.hasActiveTarget()
                error('trackbench:editor:EditorState:noActiveTarget', ...
                    'No target to duplicate.');
            end
            obj.pushUndo();
            src = obj.activeTarget();
            cp = src;                              % value-class copy
            cp.targetName = uniquifyName(obj, src.targetName + "_copy");
            cp.readOnly   = false;
            cp.sourceFile = "";
            cp.displayColor = nextDisplayColor(numel(obj.targets) + 1);
            obj.targets(end+1) = cp;
            obj.activeIdx = numel(obj.targets);
            obj.selectedIndex = 0;
            obj.anyDirty = true;
            newIdx = obj.activeIdx;
        end

        function newIdx = addNasaFlightTarget(obj, flightData, name)
            %addNasaFlightTarget  Append a TargetRecord whose waypoints
            %                     come from a NASA DASHlink FDR .mat file
            %                     loaded via trackbench.flightdata.loadNASAFlight.
            %
            %  v3.5 step 4b — NASA flight as a target type. Mirrors
            %  addNewTarget's pattern (auto-name, undo, displayColor,
            %  activate) but populates waypoints from the loaded flight
            %  data and stamps sourceFile so step 4c's "Save as
            %  recorded_flight target config" knows the origin.
            %
            %  ARGS
            %    flightData  struct returned by loadNASAFlight (must have
            %                .waypoints Nx3 [x_N, y_E, z_D in meters],
            %                .timeOfArrival Nx1 seconds, .velocities Nx3
            %                m/s, .sourceFile path string).
            %    name        optional. If omitted, derived from the .mat
            %                filename (e.g. "Tail_687_1.mat" →
            %                "nasa_Tail_687_1"). Always uniquified.
            %
            %  WAYPOINT FORMAT CONVERSION
            %    loadNASAFlight returns NED [x_N, y_E, z_D] with z_D
            %    positive-down (per the NED convention). TargetRecord's
            %    waypoints are [x_m, y_m, alt_m, time_s, leg_speed_kmh]
            %    with alt positive-up. Conversion:
            %      col 1 = flightData.waypoints(:,1)             (north)
            %      col 2 = flightData.waypoints(:,2)             (east)
            %      col 3 = -flightData.waypoints(:,3)            (alt = -z_D)
            %      col 4 = flightData.timeOfArrival
            %      col 5 = ||flightData.velocities(k,:)|| * 3.6   (km/h)
            %
            %  CURVE MODE
            %    NASA flights are dense (~30 s waypoint interval) and
            %    already represent a real path; spline interpolation
            %    would invent maneuvers that didn't happen. Force
            %    curveMode="straight" so the editor renders straight
            %    legs between consecutive recorded points. Users can
            %    flip to spline manually if they want a smoothed curve.
            arguments
                obj
                flightData (1,1) struct
                name (1,1) string = ""
            end

            % Validate the flightData struct has the fields we need.
            % Better to fail loudly here than silently produce a broken
            % target with NaN waypoints.
            requiredFields = {'waypoints', 'timeOfArrival', ...
                              'velocities', 'sourceFile'};
            for k = 1:numel(requiredFields)
                if ~isfield(flightData, requiredFields{k})
                    error('trackbench:editor:EditorState:badFlightData', ...
                        'flightData struct missing required field "%s". Was it produced by trackbench.flightdata.loadNASAFlight?', ...
                        requiredFields{k});
                end
            end
            ned = flightData.waypoints;
            toa = flightData.timeOfArrival;
            vel = flightData.velocities;
            n = size(ned, 1);
            if n < 2
                error('trackbench:editor:EditorState:notEnoughWaypoints', ...
                    'Flight data has only %d waypoint(s); need at least 2 to form a path.', n);
            end

            % Build the Nx5 waypoint matrix.
            wpMatrix = zeros(n, 5);
            wpMatrix(:, 1) = ned(:, 1);            % north → x_m
            wpMatrix(:, 2) = ned(:, 2);            % east  → y_m
            wpMatrix(:, 3) = -ned(:, 3);           % -z_D  → alt_m
            wpMatrix(:, 4) = toa(:);               % seconds
            % Per-leg speed in km/h: norm of NED velocity at the START
            % of each leg, converted m/s → km/h. Last leg has no "next"
            % so reuse the previous speed (matches loadNASAFlight's own
            % vel(end,:) = vel(end-1,:) trailing-edge handling).
            speedsMs = sqrt(sum(vel .^ 2, 2));
            wpMatrix(:, 5) = speedsMs * 3.6;

            % Auto-name from filename if caller didn't supply one.
            if strlength(name) == 0
                [~, stem] = fileparts(char(flightData.sourceFile));
                if isempty(stem)
                    stem = sprintf("flight_%d", numel(obj.targets) + 1);
                end
                name = "nasa_" + string(stem);
            end

            obj.pushUndo();
            tr = trackbench.editor.TargetRecord();
            tr.targetName        = uniquifyName(obj, sanitizeName(name));
            tr.waypoints         = wpMatrix;
            tr.durationS         = ceil(toa(end) - toa(1));
            % Cruise speed = mean over middle 80% of legs (trim ends to
            % avoid takeoff/landing skewing the value). Falls back to
            % overall mean if there aren't enough waypoints to trim.
            if n >= 5
                trim = max(1, round(0.1 * n));
                tr.defaultSpeedKmh = mean(wpMatrix(trim:end-trim, 5));
            else
                tr.defaultSpeedKmh = mean(wpMatrix(:, 5));
            end
            tr.defaultAltitudeM  = mean(wpMatrix(:, 3));
            tr.curveMode         = "straight";   % see header note
            tr.sourceFile        = string(flightData.sourceFile);
            tr.readOnly          = false;
            tr.displayColor      = nextDisplayColor(numel(obj.targets) + 1);

            obj.targets(end+1) = tr;
            obj.activeIdx     = numel(obj.targets);
            obj.selectedIndex = 0;
            obj.anyDirty      = true;
            newIdx            = obj.activeIdx;
        end

        function deleteActiveTarget(obj)
            %deleteActiveTarget  Remove the active TargetRecord. If this
            %                    was the last target, activeIdx → 0 and
            %                    the editor enters the "no targets" state.
            if ~obj.hasActiveTarget()
                return;
            end
            obj.pushUndo();
            obj.targets(obj.activeIdx) = [];
            if isempty(obj.targets)
                obj.activeIdx = 0;
            else
                obj.activeIdx = max(1, min(obj.activeIdx, numel(obj.targets)));
            end
            obj.selectedIndex = 0;
            obj.anyDirty = true;
        end

        function renameActiveTarget(obj, newName)
            %renameActiveTarget  Rename the active target. Pushes undo.
            %                    Validates non-empty + alphanumeric + uniqueness.
            if ~obj.hasActiveTarget()
                return;
            end
            if obj.activeIsReadOnly()
                error('trackbench:editor:EditorState:readOnlyTarget', ...
                    'Cannot rename a reference target. Duplicate it first.');
            end
            cleaned = sanitizeName(newName);
            if strlength(cleaned) == 0
                error('trackbench:editor:EditorState:badName', ...
                    'Target name cannot be empty.');
            end
            current = obj.activeTarget().targetName;
            if cleaned == current
                return;   % no-op rename — don't burn an undo step
            end
            if nameExists(obj, cleaned)
                error('trackbench:editor:EditorState:duplicateName', ...
                    'A target named "%s" already exists.', cleaned);
            end
            obj.pushUndo();
            tr = obj.activeTarget();
            tr.targetName = cleaned;
            obj.setActiveTarget(tr);
        end

        function nRemoved = unloadAllReferences(obj)
            %unloadAllReferences  Strip every readOnly target from the
            %                     collection. Writable targets keep their
            %                     positions in-order. Pushes undo so the
            %                     user can back out.
            %
            %  Returns the count removed so the UI can post a status
            %  message ("Unloaded N reference target(s)").
            nRemoved = 0;
            if isempty(obj.targets); return; end
            refMask = false(1, numel(obj.targets));
            for k = 1:numel(obj.targets)
                refMask(k) = obj.targets(k).readOnly;
            end
            if ~any(refMask); return; end
            obj.pushUndo();
            obj.targets(refMask) = [];
            nRemoved = sum(refMask);
            if isempty(obj.targets)
                obj.activeIdx = 0;
            else
                % activeIdx was pointing at some writable slot; that slot
                % may have shifted left if references before it were removed.
                % Find the new index by counting non-refs up to the old activeIdx.
                if obj.activeIdx >= 1 && obj.activeIdx <= numel(refMask) ...
                        && ~refMask(obj.activeIdx)
                    obj.activeIdx = sum(~refMask(1:obj.activeIdx));
                else
                    % The active was itself a reference (unusual — user
                    % selected a reference row and then hit Unload-All).
                    % Fall back to first remaining writable.
                    obj.activeIdx = 1;
                end
                obj.activeIdx = max(1, min(numel(obj.targets), obj.activeIdx));
            end
            obj.selectedIndex = 0;
            % anyDirty unchanged — removing references doesn't dirty
            % the writable working set.
        end

        function setActiveIdx(obj, newIdx)
            %setActiveIdx  Switch the active target. Clears selection
            %              (per-target) so the sidebar starts fresh.
            if isempty(obj.targets)
                obj.activeIdx = 0;
                obj.selectedIndex = 0;
                return;
            end
            newIdx = max(1, min(numel(obj.targets), round(newIdx)));
            if newIdx == obj.activeIdx
                return;
            end
            obj.activeIdx = newIdx;
            obj.selectedIndex = 0;
        end

        %% ── Per-target waypoint mutators (read-mutate-writeback) ───
        function addWaypoint(obj, x, y)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            obj.pushUndo();
            tr = obj.activeTarget();
            alt = tr.defaultAltitudeM;
            spd = tr.defaultSpeedKmh;
            tr.waypoints(end+1, :) = [x, y, alt, 0, spd];
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.selectedIndex = size(tr.waypoints, 1);
            obj.anyDirty = true;
        end

        function insertAfter(obj, idx, x, y)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            tr = obj.activeTarget();
            n = size(tr.waypoints, 1);
            if idx < 0 || idx > n
                error('trackbench:editor:insertAfter:badIndex', ...
                    'idx=%d out of range [0..%d]', idx, n);
            end
            obj.pushUndo();
            alt = tr.defaultAltitudeM;
            if idx >= 1 && idx <= n
                alt = tr.waypoints(idx, 3);
            end
            spd = tr.defaultSpeedKmh;
            newRow = [x, y, alt, 0, spd];
            tr.waypoints = [tr.waypoints(1:idx, :); newRow; tr.waypoints(idx+1:end, :)];
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.selectedIndex = idx + 1;
            obj.anyDirty = true;
        end

        function removeSelected(obj)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            tr = obj.activeTarget();
            if obj.selectedIndex < 1 || obj.selectedIndex > size(tr.waypoints, 1)
                return;
            end
            obj.pushUndo();
            idx = obj.selectedIndex;
            tr.waypoints(idx, :) = [];
            n = size(tr.waypoints, 1);
            if n == 0
                obj.selectedIndex = 0;
            else
                obj.selectedIndex = max(1, min(idx, n));
            end
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function clear(obj)
            %clear  Wipe just the active target's waypoints (keeps the
            %       target itself + scenario fields). Matches the M4
            %       semantic of the "Clear all waypoints" button.
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            tr = obj.activeTarget();
            if size(tr.waypoints, 1) == 0 && obj.loadedFrom == ""
                return;
            end
            obj.pushUndo();
            tr.waypoints = zeros(0, 5);
            tr.durationS = 0;
            obj.selectedIndex = 0;
            obj.loadedFrom = "";
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function setWaypointProperty(obj, idx, field, value)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            tr = obj.activeTarget();
            if idx < 1 || idx > size(tr.waypoints, 1); return; end
            obj.pushUndo();
            switch lower(string(field))
                case "x";         tr.waypoints(idx, 1) = value;
                case "y";         tr.waypoints(idx, 2) = value;
                case "altitude";  tr.waypoints(idx, 3) = max(0, value);
                case "speed";     tr.waypoints(idx, 5) = max(1, value);
                otherwise
                    error('trackbench:editor:unknownField', ...
                        'Unknown waypoint field: %s', field);
            end
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function moveSelectedTo(obj, x, y, commit)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            if obj.selectedIndex < 1; return; end
            if nargin < 4; commit = true; end
            if commit; obj.pushUndo(); end
            tr = obj.activeTarget();
            tr.waypoints(obj.selectedIndex, 1) = x;
            tr.waypoints(obj.selectedIndex, 2) = y;
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function applyDefaultAltitudeToAll(obj)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: no edits
            tr = obj.activeTarget();
            if size(tr.waypoints, 1) == 0; return; end
            obj.pushUndo();
            tr.waypoints(:, 3) = tr.defaultAltitudeM;
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        %% ── Selection / hit testing (active target only) ───────────
        function r = hitRadiusM(obj)
            r = obj.selectHitRadiusM;
            ax = obj.ax;
            if ~isgraphics(ax); return; end
            try
                xSpan = ax.XLim(2) - ax.XLim(1);
                ySpan = ax.YLim(2) - ax.YLim(1);
                span = max(xSpan, ySpan);
                r = max(500, min(5000, 0.05 * span));
            catch
            end
        end

        function idx = findWaypointAt(obj, x, y, maxDistM)
            %findWaypointAt  Hit-test ONLY the active target. Reference
            %                / inactive targets never participate in
            %                click routing (M5 spec §2.4 / §3.1).
            if nargin < 4 || isempty(maxDistM); maxDistM = obj.hitRadiusM(); end
            idx = 0;
            if ~obj.hasActiveTarget(); return; end
            wp = obj.activeTarget().waypoints;
            if size(wp, 1) == 0; return; end
            dx = wp(:,1) - x;
            dy = wp(:,2) - y;
            d  = sqrt(dx.*dx + dy.*dy);
            [minD, k] = min(d);
            if minD <= maxDistM
                idx = k;
            end
        end

        function [segIdx, projXY] = findSegmentAt(obj, x, y, maxDistM)
            if nargin < 4 || isempty(maxDistM); maxDistM = obj.hitRadiusM(); end
            segIdx = 0;
            projXY = [x, y];
            if ~obj.hasActiveTarget(); return; end
            wp = obj.activeTarget().waypoints;
            n = size(wp, 1);
            if n < 2; return; end
            bestDist = maxDistM;
            for k = 1:n-1
                p1 = wp(k,   1:2);
                p2 = wp(k+1, 1:2);
                [d, proj, t] = pointToSegmentDistance([x y], p1, p2);
                if d <= bestDist && t > 0.02 && t < 0.98
                    bestDist = d;
                    segIdx = k;
                    projXY = proj;
                end
            end
        end

        %% ── Undo / redo ────────────────────────────────────────────
        function pushUndo(obj)
            snap = obj.snapshot();
            obj.undoStack{end+1} = snap;
            if numel(obj.undoStack) > obj.maxUndoLevels
                obj.undoStack(1:end-obj.maxUndoLevels) = [];
            end
            obj.redoStack = {};
        end

        function ok = undo(obj)
            if isempty(obj.undoStack)
                ok = false;
                return;
            end
            obj.redoStack{end+1} = obj.snapshot();
            snap = obj.undoStack{end};
            obj.undoStack(end) = [];
            obj.restore(snap);
            ok = true;
        end

        function ok = redo(obj)
            if isempty(obj.redoStack)
                ok = false;
                return;
            end
            obj.undoStack{end+1} = obj.snapshot();
            snap = obj.redoStack{end};
            obj.redoStack(end) = [];
            obj.restore(snap);
            ok = true;
        end

        function snap = snapshot(obj)
            %snapshot  Capture the multi-target collection (value-class
            %          copy), active index, selection, and file-level
            %          fields. Pre-M5 callers don't exist in this code
            %          base anymore, but restore() still tolerates a
            %          legacy shape (waypoints + targetName at the top
            %          level) per the M5 §2.6 backward-compat rule.
            snap.targets        = obj.targets;
            snap.activeIdx      = obj.activeIdx;
            snap.selectedIndex  = obj.selectedIndex;
            snap.description    = obj.description;
            snap.loadedFrom     = obj.loadedFrom;
            % M6 additions — sensors participate in undo/redo alongside
            % targets. editMode is view state but a user may undo across
            % a mode switch, so we capture it too.
            snap.sensors         = obj.sensors;
            snap.activeSensorIdx = obj.activeSensorIdx;
            snap.editMode        = obj.editMode;
            % M7 additions — terrain/weather/degradation participate in
            % undo/redo. Value-class records copy by assignment so the
            % snapshot is independent of live state (same pattern as
            % targets/sensors). degradationExtras is captured too so
            % passthrough-only extras survive an undo across a file load.
            snap.terrain           = obj.terrain;
            snap.weather           = obj.weather;
            snap.degradation       = obj.degradation;
            snap.degradationExtras = obj.degradationExtras;
        end

        function restore(obj, snap)
            %restore  Reinstate from a snapshot. Backward-compatible
            %         with pre-M5 snapshots that store waypoints +
            %         per-target fields directly on the snap struct
            %         (wraps them into a one-element targets array).
            if isfield(snap, 'targets') && ~isempty(snap.targets)
                obj.targets = snap.targets;
            elseif isfield(snap, 'waypoints')
                % Pre-M5 shape — wrap into one TargetRecord.
                obj.targets = legacyToRecord(snap);
            else
                obj.targets = trackbench.editor.TargetRecord.empty;
            end
            if isfield(snap, 'activeIdx') && snap.activeIdx >= 1 && ...
                    snap.activeIdx <= numel(obj.targets)
                obj.activeIdx = snap.activeIdx;
            elseif ~isempty(obj.targets)
                obj.activeIdx = 1;
            else
                obj.activeIdx = 0;
            end
            if isfield(snap, 'selectedIndex')
                obj.selectedIndex = min(snap.selectedIndex, obj.count());
            else
                obj.selectedIndex = 0;
            end
            if isfield(snap, 'description')
                obj.description = snap.description;
            end
            if isfield(snap, 'loadedFrom')
                obj.loadedFrom = snap.loadedFrom;
            end
            % M6 additions — backward compatible with pre-M6 snapshots
            % (missing fields → empty sensors, idx 0, targets mode).
            if isfield(snap, 'sensors')
                obj.sensors = snap.sensors;
            else
                obj.sensors = trackbench.editor.SensorRecord.empty;
            end
            if isfield(snap, 'activeSensorIdx') ...
                    && snap.activeSensorIdx >= 1 ...
                    && snap.activeSensorIdx <= numel(obj.sensors)
                obj.activeSensorIdx = snap.activeSensorIdx;
            elseif ~isempty(obj.sensors)
                obj.activeSensorIdx = 1;
            else
                obj.activeSensorIdx = 0;
            end
            if isfield(snap, 'editMode') && ...
                    (snap.editMode == "targets" || ...
                     snap.editMode == "sensors" || ...
                     snap.editMode == "environment")
                obj.editMode = snap.editMode;
            end
            % M7 additions — backward compatible with pre-M7 snapshots.
            % A pre-M7 snapshot is missing all four fields; we fall
            % back to fresh defaults (rural terrain, no weather, all
            % degradation toggles on, no extras). This matches the
            % "legacy scenario opened cold" behavior of openScenario.
            if isfield(snap, 'terrain') && ~isempty(snap.terrain)
                obj.terrain = snap.terrain;
            else
                obj.terrain = trackbench.editor.TerrainRecord();
            end
            if isfield(snap, 'weather')
                obj.weather = snap.weather;
            else
                obj.weather = trackbench.editor.WeatherRecord.empty;
            end
            if isfield(snap, 'degradation') && isstruct(snap.degradation)
                obj.degradation = mergeDegradationDefaults(snap.degradation);
            else
                obj.degradation = defaultDegradation();
            end
            if isfield(snap, 'degradationExtras') && isstruct(snap.degradationExtras)
                obj.degradationExtras = snap.degradationExtras;
            else
                obj.degradationExtras = struct();
            end
            obj.anyDirty = true;
        end

        %% ── Timing (delegates to the active target) ───────────────
        function recomputeTimes(obj)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end   % reference: times are frozen as loaded
            tr = obj.activeTarget();
            tr = recomputeTimesOnRecord(tr);
            obj.targets(obj.activeIdx) = tr;
        end

        function n = count(obj)
            %count  Waypoint count of the ACTIVE target (0 if none).
            if ~obj.hasActiveTarget()
                n = 0;
            else
                n = size(obj.targets(obj.activeIdx).waypoints, 1);
            end
        end

        %% ── Dependent-property accessors (legacy proxies) ─────────
        function v = get.waypoints(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).waypoints;
            else
                v = zeros(0, 5);
            end
        end
        function set.waypoints(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.waypoints = v;
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.targetName(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).targetName;
            else
                v = "";
            end
        end
        function set.targetName(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.targetName = string(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.rcsDbsm(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).rcsDbsm;
            else
                v = 10;
            end
        end
        function set.rcsDbsm(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.rcsDbsm = double(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.rcsProfile(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).rcsProfile;
            else
                v = "airliner";
            end
        end
        function set.rcsProfile(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.rcsProfile = string(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.defaultSpeedKmh(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).defaultSpeedKmh;
            else
                v = 900;
            end
        end
        function set.defaultSpeedKmh(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.defaultSpeedKmh = double(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.defaultAltitudeM(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).defaultAltitudeM;
            else
                v = 3000;
            end
        end
        function set.defaultAltitudeM(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.defaultAltitudeM = double(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.curveMode(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).curveMode;
            else
                v = "straight";
            end
        end
        function set.curveMode(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.curveMode = string(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.curveTensionAlpha(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).curveTensionAlpha;
            else
                v = 0.5;
            end
        end
        function set.curveTensionAlpha(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.curveTensionAlpha = double(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.curveDensityPerSeg(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).curveDensityPerSeg;
            else
                v = 50;
            end
        end
        function set.curveDensityPerSeg(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.curveDensityPerSeg = double(v);
            obj.targets(obj.activeIdx) = tr;
            obj.anyDirty = true;
        end

        function v = get.durationS(obj)
            if obj.hasActiveTarget()
                v = obj.targets(obj.activeIdx).durationS;
            else
                v = 0;
            end
        end
        function set.durationS(obj, v)
            if ~obj.hasActiveTarget(); return; end
            if obj.activeIsReadOnly(); return; end
            tr = obj.targets(obj.activeIdx);
            tr.durationS = double(v);
            obj.targets(obj.activeIdx) = tr;
        end

        function v = get.isDirty(obj)
            %isDirty (legacy)  True if the AGGREGATE state is dirty.
            v = obj.anyDirty;
        end
        function set.isDirty(obj, v)
            obj.anyDirty = logical(v);
        end
    end
end


%% ========================================================================
%  Local helpers (file-scope)
%% ========================================================================
function tr = recomputeTimesOnRecord(tr)
%recomputeTimesOnRecord  Auto-derive time_s from cumulative leg distance
%                        and per-leg speed (col 5) on a single TargetRecord.
%                        Returns the mutated TargetRecord (value-class).
    n = size(tr.waypoints, 1);
    if n == 0
        tr.durationS = 0;
        return;
    end
    tr.waypoints(1, 4) = 0;
    if n < 2
        tr.durationS = 0;
        return;
    end
    for k = 2:n
        legKmh = tr.waypoints(k, 5);
        if ~isfinite(legKmh) || legKmh <= 0
            legKmh = tr.defaultSpeedKmh;
        end
        legMs = legKmh * 1000 / 3600;
        dx = tr.waypoints(k,1) - tr.waypoints(k-1,1);
        dy = tr.waypoints(k,2) - tr.waypoints(k-1,2);
        dz = tr.waypoints(k,3) - tr.waypoints(k-1,3);
        dist = sqrt(dx*dx + dy*dy + dz*dz);
        dt = max(dist / legMs, 1e-3);
        tr.waypoints(k, 4) = tr.waypoints(k-1, 4) + dt;
    end
    tr.durationS = ceil(tr.waypoints(end, 4));
end


function [d, proj, t] = pointToSegmentDistance(p, a, b)
    ab = b - a;
    ap = p - a;
    denom = dot(ab, ab);
    if denom <= 0
        t = 0;
    else
        t = dot(ap, ab) / denom;
    end
    tClamped = max(0, min(1, t));
    proj = a + tClamped * ab;
    d = norm(p - proj);
end


function s = sanitizeName(raw)
%sanitizeName  Strip whitespace, replace disallowed chars with '_'.
    s = strtrim(string(raw));
    s = regexprep(s, '[^A-Za-z0-9_\-]', '_');
end


function tf = nameExists(obj, name)
%nameExists  True if any target other than the active one already has this name.
    tf = false;
    for k = 1:numel(obj.targets)
        if k == obj.activeIdx; continue; end
        if obj.targets(k).targetName == name
            tf = true;
            return;
        end
    end
end


function unique = uniquifyName(obj, base)
%uniquifyName  If base collides with an existing target's name, append _2, _3, …
    unique = sanitizeName(base);
    if strlength(unique) == 0
        unique = "target";
    end
    if ~anyTargetHasName(obj, unique)
        return;
    end
    k = 2;
    candidate = sprintf("%s_%d", unique, k);
    while anyTargetHasName(obj, candidate)
        k = k + 1;
        candidate = sprintf("%s_%d", unique, k);
    end
    unique = candidate;
end


function tf = anyTargetHasName(obj, name)
    tf = false;
    for k = 1:numel(obj.targets)
        if obj.targets(k).targetName == name
            tf = true;
            return;
        end
    end
end


function c = nextDisplayColor(idx)
%nextDisplayColor  Deterministic per-target render color. Active target
%                  always overrides to the canonical blue in drawMap, so
%                  this only shows for inactive targets. Cycles through
%                  a small palette so adding/removing keeps colors stable
%                  per position in the list (per M5 §8 guidance).
    palette = [
        0.20 0.45 0.85;   % blue (also used for the active override)
        0.85 0.40 0.20;   % orange
        0.30 0.65 0.30;   % green
        0.65 0.30 0.65;   % magenta
        0.40 0.55 0.55;   % teal
        0.85 0.65 0.20];  % gold
    row = mod(idx - 1, size(palette, 1)) + 1;
    c = palette(row, :);
end


% M6 §3.6C — defaultsForType was moved to trackbench.editor.sensorDefaults
% (package-level) so it can be shared with buildUI.onSensorTypeChanged.
% See: src/+trackbench/+editor/sensorDefaults.m


function tf = sensorNameExists(obj, name)
%sensorNameExists  True if any sensor other than the active one already
%                   has this name. Mirrors nameExists() for targets.
    tf = false;
    for k = 1:numel(obj.sensors)
        if k == obj.activeSensorIdx; continue; end
        if obj.sensors(k).sensorName == name
            tf = true;
            return;
        end
    end
end


function unique = uniquifySensorName(obj, base)
%uniquifySensorName  If base collides with an existing sensor's name,
%                     append _2, _3, … Mirrors uniquifyName() for targets.
    unique = sanitizeName(base);
    if strlength(unique) == 0
        unique = "sensor";
    end
    if ~anySensorHasName(obj, unique)
        return;
    end
    k = 2;
    candidate = sprintf("%s_%d", unique, k);
    while anySensorHasName(obj, candidate)
        k = k + 1;
        candidate = sprintf("%s_%d", unique, k);
    end
    unique = candidate;
end


function tf = anySensorHasName(obj, name)
    tf = false;
    for k = 1:numel(obj.sensors)
        if obj.sensors(k).sensorName == name
            tf = true;
            return;
        end
    end
end


function targets = legacyToRecord(snap)
%legacyToRecord  Wrap a pre-M5 snapshot (per-target fields directly on
%                the snap struct) into a single-element TargetRecord
%                array. Implements the M5 §2.6 backward-compat rule.
    tr = trackbench.editor.TargetRecord();
    if isfield(snap, 'waypoints');         tr.waypoints         = snap.waypoints; end
    if isfield(snap, 'targetName');        tr.targetName        = string(snap.targetName); end
    if isfield(snap, 'rcsDbsm');           tr.rcsDbsm           = snap.rcsDbsm; end
    if isfield(snap, 'rcsProfile');        tr.rcsProfile        = string(snap.rcsProfile); end
    if isfield(snap, 'defaultSpeedKmh');   tr.defaultSpeedKmh   = snap.defaultSpeedKmh; end
    if isfield(snap, 'defaultAltitudeM');  tr.defaultAltitudeM  = snap.defaultAltitudeM; end
    if isfield(snap, 'curveMode') && ~isempty(snap.curveMode)
        tr.curveMode = string(snap.curveMode);
    end
    if isfield(snap, 'curveTensionAlpha') && ~isempty(snap.curveTensionAlpha)
        tr.curveTensionAlpha = snap.curveTensionAlpha;
    end
    targets = tr;
end

function d = defaultDegradation()
%defaultDegradation  All-on default for the run file's degradation
%                    boolean block. Matches loadRunFile's defaults
%                    for a scenario that doesn't specify overrides
%                    (all four physics effects active). The fifth
%                    run-file degradation field — "weather" — is not
%                    a boolean and is derived on export, so it's not
%                    part of this struct.
    d = struct( ...
        'terrain_occlusion', true, ...
        'horizon_masking',   true, ...
        'ground_clutter',    true, ...
        'doppler_fade',      true);
end

function d = mergeDegradationDefaults(partial)
%mergeDegradationDefaults  Fill missing keys on a partial degradation
%                          struct with defaults. Used by restore() to
%                          tolerate pre-M7 snapshots that have no
%                          degradation field at all, and by
%                          openScenarioFromJSON to tolerate run files
%                          that specify only a subset of the four
%                          toggles (older run files).
    d = defaultDegradation();
    keys = ["terrain_occlusion", "horizon_masking", ...
            "ground_clutter",    "doppler_fade"];
    for k = 1:numel(keys)
        key = char(keys(k));
        if isfield(partial, key)
            d.(key) = logical(partial.(key));
        end
    end
end
