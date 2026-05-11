function [nSensors, nTargets] = openScenarioFromJSON(state, runPath)
%openScenarioFromJSON  Load a full scenario run file into EditorState (M6 §3.4).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Replaces BOTH state.sensors and state.targets with the content the run
%  file references. Matches the loadRunFile.m convention used by the
%  simulator side:
%      {
%        "description": "...",
%        "sensors": ["PSR/default_PSR", ...],
%        "targets": "waypoints/<stem>",   (or other pattern paths)
%        ...
%      }
%
%  This is the inverse of exportSensorsToJSON's run-file writer, and
%  the editor UI binds it to the "Open Scenario…" button. Load Sensors…
%  is the append-only alternative when the user just wants to merge in
%  additional sensors without replacing their targets.
%
%  MODE
%    Always REPLACE. An append version isn't meaningful for full
%    scenarios — if the user wanted to layer content, they'd use Load
%    Sensors… (append) and Load as Reference… (targets overlay).
%
%  INPUTS
%    state   : trackbench.editor.EditorState instance
%    runPath : absolute path or project-relative path to the run JSON.
%              If relative, it's resolved against projectRoot/config/runs.
%
%  OUTPUTS
%    nSensors : count of sensors installed (incl. UNKNOWN passthrough)
%    nTargets : count of targets installed (writable only — refs are
%               not auto-created from a scenario open)
%
%  NOTES ON UNDO
%    Because this function calls loadSensorsFromJSON and loadFromJSON
%    under the hood, and each pushes its own undo snapshot, opening a
%    scenario consumes TWO undo slots: one to get back to the state after
%    sensors were installed (targets still pre-open), one to get all the
%    way back. In practice users never need partial-undo of an Open
%    Scenario, but Ctrl+Z will walk it back in two steps. Documenting
%    the behavior here so it's not a surprise during QA.
%
%  See also: trackbench.editor.exportSensorsToJSON,
%            trackbench.editor.loadSensorsFromJSON,
%            trackbench.editor.loadFromJSON,
%            trackbench.config.loadRunFile

    arguments
        state   (1,1) trackbench.editor.EditorState
        runPath (1,1) string
    end

    resolved = resolveRunPath(state, runPath);
    if ~isfile(resolved)
        error('trackbench:editor:openScenarioFromJSON:notFound', ...
            'Run file not found: %s', resolved);
    end

    raw = fileread(resolved);
    try
        runDef = jsondecode(raw);
    catch ME
        error('trackbench:editor:openScenarioFromJSON:badJSON', ...
            'Could not parse %s: %s', resolved, ME.message);
    end

    if ~isstruct(runDef)
        error('trackbench:editor:openScenarioFromJSON:badShape', ...
            'Run file root must be a JSON object: %s', resolved);
    end

    % ── Load sensors ─────────────────────────────────────────────────
    %  Delegate to loadSensorsFromJSON in "replace" mode. That function
    %  already handles the run-file shape ("sensors" array of strings)
    %  via its shape-detect path, so we can just hand it the run file
    %  itself. If a run file omits "sensors", that's an error — an open
    %  that left the editor with no sensors would be confusing.
    if ~isfield(runDef, 'sensors') || isempty(runDef.sensors)
        error('trackbench:editor:openScenarioFromJSON:noSensors', ...
            'Run file has no "sensors" field: %s', resolved);
    end
    nSensors = trackbench.editor.loadSensorsFromJSON( ...
        state, resolved, "replace");

    % ── Load targets ─────────────────────────────────────────────────
    %  Run files point at a target config by relative path under
    %  config/targets/. loadFromJSON's resolvePath looks under
    %  state.outputDir (config/targets/waypoints) and state.projectRoot,
    %  but run-file target paths like "crossing_pair/default_crossing_pair"
    %  live under config/targets/<pattern>/ — a DIFFERENT subdirectory.
    %  Resolve the absolute path here so loadFromJSON is fed an absolute.
    nTargets = 0;
    if isfield(runDef, 'targets') && ~isempty(runDef.targets)
        tgtAbs = resolveTargetsPathFromRun(state, runDef.targets);
        if isfile(tgtAbs)
            trackbench.editor.loadFromJSON(state, tgtAbs, "replace");
            nTargets = countWritable(state.targets);
        else
            warning('trackbench:editor:openScenarioFromJSON:targetsNotFound', ...
                ['Run file references missing target config:\n  %s\n' ...
                 'Sensors were loaded; targets are unchanged.'], tgtAbs);
        end
    end

    % ── Top-level description ────────────────────────────────────────
    if isfield(runDef, 'description') && ~isempty(runDef.description)
        state.description = string(runDef.description);
    end

    % ── M7 §3.4 — environment (terrain + weather + degradation) ──────
    %   Terrain is REQUIRED by the run schema — if the run file omits
    %   it we fall back to terrainDefaults("rural"). Weather may be
    %   "none" (clears state.weather) or "<type>/<stem>" (loads that
    %   file). The degradation block is a four-booleans + "weather"
    %   struct; "weather" drives the weather load above, the four
    %   booleans go into state.degradation, and ANY other keys (e.g.
    %   rcs_range_filter from older run files) are captured verbatim
    %   into state.degradationExtras for round-trip on re-export.
    loadEnvironmentFromRun(state, runDef);

    state.loadedFrom = string(resolved);
    state.anyDirty   = false;
    state.sensorsDirty = false;
    state.environmentDirty = false;
end


function loadEnvironmentFromRun(state, runDef)
%loadEnvironmentFromRun  Parse the M7 environment fields out of a run
%                         file and write them into state. Idempotent
%                         — safe to re-call (it fully resets terrain,
%                         weather, degradation, extras, regions).
%
%  Resilient to missing / partial fields: an older run file that
%  predates M7 lands as rural terrain + no weather + all-on degradation
%  toggles, which matches EditorState's defaults. The only fatal case
%  is a weather reference like "rain/my_storm" that can't be resolved —
%  we warn and leave state.weather empty so the user sees the gap
%  rather than mysteriously getting default rain.
%
%  v3.5 §5c.5 — terrain and weather are now polymorphic. Each accepts
%  either a legacy string scalar (single component, no regions) or a
%  struct with {fallback, regions[]} (v3.5 multi-region shape). Regions
%  are parsed into TerrainRegionRecord / WeatherRegionRecord arrays via
%  parseTerrainFieldEditor / parseWeatherFieldEditor below. Active
%  region indices are clamped to the new collections at the end —
%  defaults to 1 if any regions exist, 0 otherwise. envSubMode (the
%  Fallback/Regions sub-panel toggle) is editor view state and is
%  intentionally NOT reset by Open Scenario.

    % v3.5 §5c.5 — reset region collections up-front so any partial-
    % failure mid-parse leaves the editor in a consistent state.
    state.terrainRegions = trackbench.editor.TerrainRegionRecord.empty;
    state.weatherRegions = trackbench.editor.WeatherRegionRecord.empty;

    % Terrain — polymorphic dispatch (v3.5 §5c.5). Accept either a
    % legacy string scalar (single component, no regions) or a struct
    % with {fallback, regions[]} (v3.5 multi-region shape).
    if isfield(runDef, 'terrain') && ~isempty(runDef.terrain)
        [state.terrain, state.terrainRegions] = parseTerrainFieldEditor( ...
            runDef.terrain, state);
    else
        state.terrain = trackbench.editor.terrainDefaults("rural");
    end

    % Weather + degradation come from the "degradation" block.
    state.weather = trackbench.editor.WeatherRecord.empty;
    state.degradation = defaultDegradationLocal();
    state.degradationExtras = struct();

    if isfield(runDef, 'degradation') && isstruct(runDef.degradation)
        deg = runDef.degradation;
        knownKeys = ["terrain_occlusion", "horizon_masking", ...
                     "ground_clutter",    "doppler_fade",    ...
                     "weather"];
        % Booleans — only accept true/false/1/0.
        boolKeys = knownKeys(1:4);
        for k = 1:numel(boolKeys)
            key = char(boolKeys(k));
            if isfield(deg, key)
                state.degradation.(key) = logical(deg.(key));
            end
        end
        % Weather reference — polymorphic dispatch (v3.5 §5c.5). Same
        % shapes as terrain except the fallback may be the literal
        % "none" (no global weather, only storm-cell regions).
        if isfield(deg, 'weather') && ~isempty(deg.weather)
            [state.weather, state.weatherRegions] = parseWeatherFieldEditor( ...
                deg.weather, state);
        end
        % Unknown keys → degradationExtras (verbatim passthrough).
        allKeys = fieldnames(deg);
        for k = 1:numel(allKeys)
            key = allKeys{k};
            if ~any(string(key) == knownKeys)
                state.degradationExtras.(key) = deg.(key);
            end
        end
    end

    % v3.5 §5c.5 — clamp active region indices to the new collections.
    %   Default to first region if any exist (matches the active-target
    %   / active-sensor convention); otherwise 0. envSubMode is editor
    %   view state and intentionally NOT reset — the user's panel
    %   context carries across an Open Scenario.
    if isempty(state.terrainRegions)
        state.activeTerrainRegionIdx = 0;
    else
        state.activeTerrainRegionIdx = 1;
    end
    if isempty(state.weatherRegions)
        state.activeWeatherRegionIdx = 0;
    else
        state.activeWeatherRegionIdx = 1;
    end
end


function d = defaultDegradationLocal()
%defaultDegradationLocal  Mirrors EditorState's private
%                          defaultDegradation() helper — duplicated here
%                          to avoid exposing a package-private function.
    d = struct( ...
        'terrain_occlusion', true, ...
        'horizon_masking',   true, ...
        'ground_clutter',    true, ...
        'doppler_fade',      true);
end


%% ========================================================================
%  v3.5 §5c.5 — Multi-region run-file parsers (editor-side mirrors of
%                trackbench.config.loadRunFile.parseTerrainField /
%                parseWeatherField). Build typed value-class arrays
%                (TerrainRegionRecord / WeatherRegionRecord) instead of
%                the sim-engine cell-of-structs intermediate.
%% ========================================================================

function [fallback, regions] = parseTerrainFieldEditor(field, state)
%parseTerrainFieldEditor  Editor-side terrain field parser. Returns:
%    fallback   a TerrainRecord (always populated; falls back to
%               terrainDefaults("rural") if the input is unparseable
%               or fails to load).
%    regions    a TerrainRegionRecord 1xN array (empty for legacy
%               string-scalar input or when regions[] is missing/empty).
    fallback = trackbench.editor.terrainDefaults("rural");
    regions = trackbench.editor.TerrainRegionRecord.empty;

    if ischar(field) || isstring(field)
        % Legacy string scalar.
        ref = string(field);
        if strlength(ref) > 0
            try
                fallback = trackbench.editor.loadTerrainFromJSON( ...
                    state.projectRoot, ref);
            catch ME
                warning('trackbench:editor:openScenarioFromJSON:terrainNotFound', ...
                    ['Run file references missing/bad terrain "%s":\n  %s\n' ...
                     'Falling back to rural default.'], ref, ME.message);
            end
        end
        return;
    end

    if isstruct(field)
        % v3.5 §5a multi-region shape: {fallback, regions: [...]}.
        if isfield(field, 'fallback') && ~isempty(field.fallback)
            fbRef = string(field.fallback);
            try
                fallback = trackbench.editor.loadTerrainFromJSON( ...
                    state.projectRoot, fbRef);
            catch ME
                warning('trackbench:editor:openScenarioFromJSON:terrainNotFound', ...
                    ['Run file references missing/bad terrain fallback "%s":\n  %s\n' ...
                     'Falling back to rural default.'], fbRef, ME.message);
            end
        end
        if isfield(field, 'regions') && ~isempty(field.regions)
            regions = parseRegionsToRecords( ...
                field.regions, 'terrain', state);
        end
        return;
    end

    warning('trackbench:editor:openScenarioFromJSON:badTerrainField', ...
        'Unrecognized terrain field type "%s" — using rural default.', ...
        class(field));
end


function [fallback, regions] = parseWeatherFieldEditor(field, state)
%parseWeatherFieldEditor  Editor-side weather field parser. Returns:
%    fallback   a WeatherRecord (1x1) OR an empty WeatherRecord array.
%               Empty means "no global weather" — corresponds to a
%               legacy "none" string or fallback="none" inside a
%               struct shape.
%    regions    a WeatherRegionRecord 1xN array (empty for legacy
%               string-scalar input or when regions[] is missing/empty).
    fallback = trackbench.editor.WeatherRecord.empty;
    regions = trackbench.editor.WeatherRegionRecord.empty;

    if ischar(field) || isstring(field)
        % Legacy string scalar.
        ref = string(field);
        if strlength(ref) == 0 || strcmpi(ref, "none")
            return;
        end
        try
            fallback = trackbench.editor.loadWeatherFromJSON( ...
                state.projectRoot, ref);
        catch ME
            warning('trackbench:editor:openScenarioFromJSON:weatherNotFound', ...
                ['Run file references missing/bad weather "%s":\n  %s\n' ...
                 'Weather left empty.'], ref, ME.message);
        end
        return;
    end

    if isstruct(field)
        % v3.5 §5a multi-region shape.
        if isfield(field, 'fallback') && ~isempty(field.fallback)
            fbRef = string(field.fallback);
            if strlength(fbRef) > 0 && ~strcmpi(fbRef, "none")
                try
                    fallback = trackbench.editor.loadWeatherFromJSON( ...
                        state.projectRoot, fbRef);
                catch ME
                    warning('trackbench:editor:openScenarioFromJSON:weatherNotFound', ...
                        ['Run file references missing/bad weather fallback "%s":\n  %s\n' ...
                         'Weather fallback left empty.'], fbRef, ME.message);
                end
            end
        end
        if isfield(field, 'regions') && ~isempty(field.regions)
            regions = parseRegionsToRecords( ...
                field.regions, 'weather', state);
        end
        return;
    end

    warning('trackbench:editor:openScenarioFromJSON:badWeatherField', ...
        'Unrecognized weather field type "%s" — treating as "none".', ...
        class(field));
end


function records = parseRegionsToRecords(regs, kind, state)
%parseRegionsToRecords  Common region-array parser. Normalizes the
%                        struct-array-vs-cell-array quirk of jsondecode
%                        (homogeneous JSON object arrays come back as
%                        struct arrays; mixed shapes come back as cell
%                        arrays of structs). Iterates and builds a
%                        typed value-class array.
%
%  Per-region failures (missing config, bad polygon shape, unresolvable
%  config path) warn-and-skip rather than aborting the whole open. The
%  alternative — erroring — would lose the user's other regions on a
%  single bad entry. Tradeoff: the editor is more forgiving than the
%  sim engine's parseRegion (which errors on missing config).
    switch kind
        case 'terrain'
            empty = trackbench.editor.TerrainRegionRecord.empty;
        case 'weather'
            empty = trackbench.editor.WeatherRegionRecord.empty;
        otherwise
            error('parseRegionsToRecords:badKind', ...
                'Unknown region kind: %s', kind);
    end

    if isstruct(regs); regs = num2cell(regs); end
    n = numel(regs);
    if n == 0
        records = empty;
        return;
    end

    % Pre-allocate; trim with `keep` mask after the loop.
    switch kind
        case 'terrain'
            tmp = repmat(trackbench.editor.TerrainRegionRecord, 1, n);
        case 'weather'
            tmp = repmat(trackbench.editor.WeatherRegionRecord, 1, n);
    end
    keep = false(1, n);

    for k = 1:n
        rDef = regs{k};
        if ~isstruct(rDef)
            warning('trackbench:editor:openScenarioFromJSON:badRegion', ...
                'Region #%d is not a JSON object (%s) — skipping.', ...
                k, class(rDef));
            continue;
        end
        rec = tmp(k);
        if isfield(rDef, 'name')
            rec.name = string(rDef.name);
        end
        if isfield(rDef, 'config') && ~isempty(rDef.config)
            rec.configPath = string(rDef.config);
        else
            warning('trackbench:editor:openScenarioFromJSON:missingRegionConfig', ...
                'Region #%d (%s) missing required "config" field — skipping.', ...
                k, char(rec.name));
            continue;
        end
        if isfield(rDef, 'polygon_xy')
            rec.polygonXY = normalizePolygonXY(rDef.polygon_xy, rec.name);
        end
        % Resolve the inner record from configPath. Failure is non-fatal
        % — keep the region with whatever inner-record default the value
        % class provides, warn so the user can fix the path. Keeps the
        % polygon and name visible in the editor for repair.
        try
            switch kind
                case 'terrain'
                    rec.terrain = trackbench.editor.loadTerrainFromJSON( ...
                        state.projectRoot, rec.configPath);
                case 'weather'
                    rec.weather = trackbench.editor.loadWeatherFromJSON( ...
                        state.projectRoot, rec.configPath);
            end
        catch ME
            warning('trackbench:editor:openScenarioFromJSON:regionConfigNotFound', ...
                ['Region "%s" config "%s" could not be loaded:\n  %s\n' ...
                 'Region added with default inner record — fix Change Config.'], ...
                char(rec.name), char(rec.configPath), ME.message);
        end
        tmp(k) = rec;
        keep(k) = true;
    end

    records = tmp(keep);
end


function poly = normalizePolygonXY(raw, regionName)
%normalizePolygonXY  Coerce a jsondecode'd polygon_xy value into the
%                     Nx2 double matrix that TerrainRegionRecord /
%                     WeatherRegionRecord expect.
%
%  jsondecode quirks handled here:
%    * Empty array "[]"      → 0x0  → reshape to zeros(0,2)
%    * Single pair [x,y]     → 2x1 column or 1x2 row → reshape to 1x2
%    * Normal Nx2 (N>=2)     → already correct
%    * Transposed 2xN (N>=3) → flip to Nx2
%    * Anything else         → warn and return zeros(0,2)
%
%  Mirrors trackbench.config.loadRunFile.parseRegion's polygon
%  validation, with the addition of empty-array tolerance (the sim
%  engine errors on bad shapes; the editor wants to display+repair
%  scenarios with zero or partial polygon data).
    if ~isnumeric(raw)
        warning('trackbench:editor:openScenarioFromJSON:badPolygon', ...
            'Region "%s" polygon_xy must be numeric (got %s) — using empty.', ...
            char(regionName), class(raw));
        poly = zeros(0, 2);
        return;
    end
    if isempty(raw)
        poly = zeros(0, 2);
        return;
    end
    if isvector(raw) && numel(raw) == 2
        poly = raw(:).';   % normalize to 1x2 row
        return;
    end
    if size(raw, 2) == 2
        poly = raw;
        return;
    end
    if size(raw, 1) == 2 && size(raw, 2) > 2
        poly = raw';   % flip transposed to Nx2
        return;
    end
    warning('trackbench:editor:openScenarioFromJSON:badPolygon', ...
        'Region "%s" polygon_xy has unexpected shape %dx%d — using empty.', ...
        char(regionName), size(raw, 1), size(raw, 2));
    poly = zeros(0, 2);
end


%% ========================================================================
%  Path resolvers
%% ========================================================================
function full = resolveRunPath(state, runPath)
%resolveRunPath  Let the user give us either an absolute path, a path
%                  relative to project root, or a bare "name" that lives
%                  under config/runs/. Mirrors loadRunFile's entrypoint
%                  behavior so the UI picker and programmatic usage
%                  both work.
    p = char(runPath);
    if isAbsolute(p)
        full = p;
        return;
    end
    root = state.projectRoot;
    if root == ""; root = string(pwd); end
    candidates = strings(0);
    candidates(end+1) = fullfile(root, p);
    if ~endsWith(lower(p), ".json")
        candidates(end+1) = fullfile(root, p + ".json");
        candidates(end+1) = fullfile(root, "config", "runs", p + ".json");
    else
        candidates(end+1) = fullfile(root, "config", "runs", p);
    end
    for i = 1:numel(candidates)
        if isfile(candidates(i))
            full = char(candidates(i));
            return;
        end
    end
    full = char(candidates(1));
end


function full = resolveTargetsPathFromRun(state, targetsField)
%resolveTargetsPathFromRun  Run files reference targets by a pattern
%                            path like "waypoints/m1_test" or
%                            "crossing_pair/default_crossing_pair". Prepend
%                            <projectRoot>/config/targets/ and .json.
    rel = char(targetsField);
    if ~endsWith(lower(rel), ".json")
        rel = [rel '.json'];
    end
    root = state.projectRoot;
    if root == ""; root = string(pwd); end
    full = char(fullfile(root, "config", "targets", rel));
end


function yes = isAbsolute(p)
%isAbsolute  Cross-platform absolute path test. Mirrors loadFromJSON.m
%            and loadSensorsFromJSON.m — duplicated here because each
%            file is self-contained and value-coupling to a shared
%            private helper isn't worth the package churn.
    if ispc
        yes = numel(p) >= 2 && (p(2) == ':' || startsWith(p, '\\'));
    else
        yes = ~isempty(p) && p(1) == '/';
    end
end


function n = countWritable(targets)
%countWritable  Tally non-readOnly targets.
    n = 0;
    for k = 1:numel(targets)
        if ~targets(k).readOnly
            n = n + 1;
        end
    end
end
