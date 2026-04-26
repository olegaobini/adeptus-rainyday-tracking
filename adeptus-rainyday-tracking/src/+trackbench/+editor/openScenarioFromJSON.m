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
%                         weather, degradation, extras).
%
%  Resilient to missing / partial fields: an older run file that
%  predates M7 lands as rural terrain + no weather + all-on degradation
%  toggles, which matches EditorState's defaults. The only fatal case
%  is a weather reference like "rain/my_storm" that can't be resolved —
%  we warn and leave state.weather empty so the user sees the gap
%  rather than mysteriously getting default rain.

    % Terrain — required by schema but tolerated if missing.
    if isfield(runDef, 'terrain') && ~isempty(runDef.terrain)
        terrainRef = string(runDef.terrain);
        try
            state.terrain = trackbench.editor.loadTerrainFromJSON( ...
                state.projectRoot, terrainRef);
        catch ME
            warning('trackbench:editor:openScenarioFromJSON:terrainNotFound', ...
                ['Run file references missing/bad terrain "%s":\n  %s\n' ...
                 'Falling back to rural default.'], terrainRef, ME.message);
            state.terrain = trackbench.editor.terrainDefaults("rural");
        end
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
        % Weather reference.
        if isfield(deg, 'weather') && ~isempty(deg.weather)
            wRef = string(deg.weather);
            if wRef ~= "" && lower(wRef) ~= "none"
                try
                    state.weather = trackbench.editor.loadWeatherFromJSON( ...
                        state.projectRoot, wRef);
                catch ME
                    warning('trackbench:editor:openScenarioFromJSON:weatherNotFound', ...
                        ['Run file references missing/bad weather "%s":\n  %s\n' ...
                         'Weather left empty.'], wRef, ME.message);
                end
            end
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
