function loadFromJSON(state, jsonPath, mode)
%loadFromJSON  Hydrate an EditorState from a waypoints-behavior target file.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  M5 §3.2 — MULTI-TARGET LOAD WITH REFERENCE MODE
%    The loader now reads EVERY waypoints-behavior target in the file and
%    materializes each one as a TargetRecord. The two operating modes share
%    a single pure-transform helper, parseTargetsAsRecords, so reference
%    overlay never diverges from default load.
%
%    mode = "replace" (default)
%       Behaves like the M4 loader extended for multi-target. pushUndo,
%       wipe state.targets, install records, activeIdx → 1, pull
%       description + duration_s onto the EditorState. If the file holds
%       N targets, the editor opens with N writable targets.
%
%    mode = "reference"
%       Append each parsed record to state.targets with readOnly = true
%       and sourceFile = resolved-path. Active target is NOT changed.
%       Top-level fields (description, duration_s, loadedFrom) are NOT
%       overwritten — the current editing context is preserved. References
%       are filtered out by exportToJSON, so loading-as-reference is a
%       view-only overlay.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    jsonPath : absolute or project-relative path to the JSON file
%    mode     : (optional) "replace" (default) | "reference"
%
%  CONVENTIONS
%    - Forgiving on parse (per spec §4.10): accepts integer-valued pos,
%      missing rcs_profile, missing description, either a struct-array or
%      cell-array waypoints field, and both {"targets": [...]} and
%      {"targets": {...}} shapes (jsondecode flip-flops based on whether
%      the single-target case has uniform fields).
%    - Flips negative Z (NED) back to POSITIVE altitude for UI storage.
%    - Computes per-leg speed from distance / time deltas so the sidebar
%      shows meaningful values after load. Row 1's leg_speed_kmh is set to
%      the default so there's no NaN leaking through.
%    - Backward compatible with M4 single-target files: a one-target file
%      simply yields a one-element TargetRecord array.
%
%  CURVE-MODE ROUND-TRIP (M4.3.4)
%    If a target has both "curve_mode": "curved" AND a "control_waypoints"
%    array, the CONTROL POINTS (not the dense simulator waypoints) populate
%    the editor. curveMode is set to "curved" and curve_tension_alpha (if
%    present) is restored so the rendered curve matches what was exported.
%    M3-era files — missing both fields, or with curve_mode="straight" and
%    no control_waypoints — load as straight mode with the default tension
%    (alpha=0.5). This keeps existing JSON files working.
%
%  ERRORS
%    Throws trackbench:editor:loadFromJSON:* on missing file, malformed
%    schema, or too few waypoints (in any one of the file's targets). UI
%    callers should wrap in try/catch and surface the message via uialert.
%
%  See also: trackbench.editor.exportToJSON,
%            trackbench.editor.EditorState,
%            trackbench.scenario.addTargetFromDef

    arguments
        state    (1,1) trackbench.editor.EditorState
        jsonPath (1,1) string
        mode     (1,1) string {mustBeMember(mode, ["replace","reference"])} = "replace"
    end

    % ── Resolve path ─────────────────────────────────────────────────
    resolved = resolvePath(state, jsonPath);
    if ~isfile(resolved)
        error('trackbench:editor:loadFromJSON:notFound', ...
            'File not found: %s', resolved);
    end

    raw = fileread(resolved);
    try
        decoded = jsondecode(raw);
    catch ME
        error('trackbench:editor:loadFromJSON:badJSON', ...
            'Could not parse %s: %s', resolved, ME.message);
    end

    % ── Pure transform: JSON → TargetRecord array ───────────────────
    %  Pull the seed default speed off the EditorState so NaN/missing
    %  speed_kmh fields backfill to whatever the user has configured
    %  globally — same fallback behavior as the M4 loader.
    records = parseTargetsAsRecords(decoded, resolved, state.defaultSpeedKmh);

    if isempty(records)
        error('trackbench:editor:loadFromJSON:noWaypointsTarget', ...
            ['No target with behavior="waypoints" found in %s. ' ...
             'loadFromJSON only supports waypoint-behavior files.'], resolved);
    end

    % ── Apply per mode ──────────────────────────────────────────────
    state.pushUndo();
    switch mode
        case "replace"
            applyReplace(state, records, decoded, resolved);
        case "reference"
            applyReference(state, records, resolved);
    end
end


%% ========================================================================
%  Mode appliers
%% ========================================================================
function applyReplace(state, records, decoded, resolved)
%applyReplace  Wipe state.targets and install the parsed records as the
%              new editing population. All records are writable (readOnly
%              defaults to false on TargetRecord). Top-level metadata
%              (description, duration_s, loadedFrom) is pulled from the
%              file onto the EditorState.
    state.targets   = records;
    state.activeIdx = 1;

    % Top-level scenario metadata.
    if isstruct(decoded) && isfield(decoded, 'description') ...
            && ~isempty(decoded.description)
        state.description = string(decoded.description);
    else
        state.description = "";
    end

    state.selectedIndex = 0;
    state.loadedFrom    = string(resolved);

    % Roll the active target's durationS up against the file-level
    % duration_s if present. Inactive writables keep the per-record
    % durationS already populated by parseTargetsAsRecords.
    activeTr = state.activeTarget();
    if isstruct(decoded) && isfield(decoded, 'duration_s') ...
            && isnumeric(decoded.duration_s) && isfinite(decoded.duration_s)
        activeTr.durationS = max(activeTr.durationS, ...
                                 ceil(double(decoded.duration_s)));
        state.setActiveTarget(activeTr);
    end

    state.anyDirty = false;   % freshly loaded == no unsaved changes
end


function applyReference(state, records, resolved)
%applyReference  Mark each parsed record readOnly + sourceFile and append
%                to state.targets without disturbing the current active
%                target. Top-level fields are intentionally NOT touched —
%                a reference overlay must not nuke the user's working
%                description or replace loadedFrom.
    for k = 1:numel(records)
        r = records(k);
        r.readOnly   = true;
        r.sourceFile = string(resolved);
        % Give reference targets a distinguishing color so they read as
        % overlay rather than competing with writable inactive targets.
        % Cycle through a small palette by current count to avoid every
        % reference rendering as the same color.
        r.displayColor = nextReferenceColor(numel(state.targets) + 1);
        state.targets(end+1) = r;
    end
    % Do NOT change activeIdx — the user is overlaying a reference on
    % top of whatever they were editing.
    % anyDirty is left as-is: loading a reference doesn't make the
    % writable working set dirty by itself.
end


%% ========================================================================
%  Pure transform: decoded JSON → TargetRecord array
%% ========================================================================
function records = parseTargetsAsRecords(decoded, sourceFile, defaultSpeedKmh)
%parseTargetsAsRecords  Convert a parsed JSON payload into a TargetRecord
%                       array. Pure function — no state writes, no UI
%                       calls. Both "replace" and "reference" modes feed
%                       through here so behavior never diverges.
%
%  Iterates EVERY waypoints-behavior target in the file. M4 single-target
%  files yield a one-element array, which is the round-trip baseline.
%
%  INPUTS
%    decoded         : output of jsondecode(fileread(...))
%    sourceFile      : absolute path the JSON was read from. Stored on
%                      each record's sourceFile property regardless of
%                      mode (the mode applier may overwrite for reference,
%                      but having the provenance set here keeps the helper
%                      a true pure transform).
%    defaultSpeedKmh : fallback per-leg speed (km/h) when the file omits
%                      speed_kmh on a waypoint. Comes from the editor's
%                      global default, not from the file.
%
%  OUTPUT
%    records : 1xN trackbench.editor.TargetRecord array. Empty if the
%              file has no waypoints-behavior targets — caller decides
%              whether that's an error.
    targetList = allWaypointsTargets(decoded);
    n = numel(targetList);
    records = trackbench.editor.TargetRecord.empty;
    for k = 1:n
        records(end+1) = buildOneTargetRecord( ...
            targetList{k}, sourceFile, defaultSpeedKmh); %#ok<AGROW>
    end
end


function tr = buildOneTargetRecord(tgt, sourceFile, defaultSpeedKmh)
%buildOneTargetRecord  Convert a single decoded target struct into a
%                      TargetRecord. Mirrors the M4 single-target loader
%                      logic verbatim, just retargeted onto a TargetRecord
%                      instead of EditorState's old direct fields.
    tr = trackbench.editor.TargetRecord();
    tr.sourceFile      = string(sourceFile);
    tr.defaultSpeedKmh = defaultSpeedKmh;

    % ── Curve-mode detection (M4.3.4) ───────────────────────────────
    [wpRaw, curveModeDetected, tensionAlpha] = pickEditorWaypoints(tgt);
    n = numel(wpRaw);
    if n < 2
        nameForMsg = "<unnamed>";
        if isfield(tgt, 'name') && ~isempty(tgt.name)
            nameForMsg = string(tgt.name);
        elseif isfield(tgt, 'label') && ~isempty(tgt.label)
            nameForMsg = string(tgt.label);
        end
        error('trackbench:editor:loadFromJSON:tooFewWaypoints', ...
            'Target "%s" has %d waypoints; need at least 2.', ...
            char(nameForMsg), n);
    end

    % ── Build the Nx5 matrix (x, y, alt, time_s, leg_speed_kmh) ─────
    wp = zeros(n, 5);
    speedProvided = false(n, 1);
    for k = 1:n
        if iscell(wpRaw)
            w = wpRaw{k};
        else
            w = wpRaw(k);
        end
        pos = double(w.pos(:)');                     % row vector
        if numel(pos) < 3
            error('trackbench:editor:loadFromJSON:badPos', ...
                'Waypoint %d: pos must have 3 elements, got %d.', k, numel(pos));
        end
        wp(k, 1) = pos(1);                            % x (east)
        wp(k, 2) = pos(2);                            % y (north)
        wp(k, 3) = abs(pos(3));                       % altitude (UI stores positive)
        if isfield(w, 'time_s')
            wp(k, 4) = double(w.time_s);
        else
            wp(k, 4) = NaN;   % backfilled below
        end
        % Optional metadata from exportToJSON — if present, use it directly.
        if isfield(w, 'speed_kmh') && isnumeric(w.speed_kmh) && isfinite(w.speed_kmh) ...
                && w.speed_kmh > 0
            wp(k, 5) = double(w.speed_kmh);
            speedProvided(k) = true;
        end
    end

    % Enforce strictly increasing time (patch any NaN/bad values).
    if any(~isfinite(wp(:,4))) || any(diff(wp(:,4)) <= 0)
        warning('trackbench:editor:loadFromJSON:timeBackfill', ...
            'Source time_s non-monotonic or missing — rebuilding from default speed.');
        wp(:,4) = NaN;
    end

    % ── Back-compute per-leg speeds for rows where speed_kmh was absent
    if ~speedProvided(1)
        wp(1, 5) = defaultSpeedKmh;
    end
    if all(isfinite(wp(:,4)))
        for k = 2:n
            if speedProvided(k)
                continue;
            end
            dx = wp(k,1) - wp(k-1,1);
            dy = wp(k,2) - wp(k-1,2);
            dz = wp(k,3) - wp(k-1,3);
            dist = sqrt(dx*dx + dy*dy + dz*dz);
            dt   = wp(k,4) - wp(k-1,4);
            if dt > 0 && dist > 0
                legKmh = (dist / dt) * 3.6;
            else
                legKmh = defaultSpeedKmh;
            end
            wp(k, 5) = legKmh;
        end
    else
        wp(~speedProvided, 5) = defaultSpeedKmh;
    end

    % ── Self-contained time rebuild ─────────────────────────────────
    %  If any times are still NaN (hand-edited / non-monotonic source),
    %  rebuild them here from per-leg speed + distance. Mirrors the
    %  math in EditorState.recomputeTimesOnRecord so the result is
    %  indistinguishable from what the editor would produce on load.
    %  Without this, an inactive record (e.g. a reference) would carry
    %  NaN times the renderer can't draw — state.recomputeTimes only
    %  touches the active target.
    if any(~isfinite(wp(:,4)))
        wp(1, 4) = 0;
        for k = 2:n
            legKmh = wp(k, 5);
            if ~isfinite(legKmh) || legKmh <= 0
                legKmh = defaultSpeedKmh;
            end
            legMs = legKmh * 1000 / 3600;
            dx = wp(k,1) - wp(k-1,1);
            dy = wp(k,2) - wp(k-1,2);
            dz = wp(k,3) - wp(k-1,3);
            dist = sqrt(dx*dx + dy*dy + dz*dz);
            dt   = max(dist / legMs, 1e-3);
            wp(k, 4) = wp(k-1, 4) + dt;
        end
    end

    tr.waypoints         = wp;
    tr.curveMode         = curveModeDetected;
    tr.curveTensionAlpha = tensionAlpha;

    % Scenario fields — be defensive against missing keys.
    if isfield(tgt, 'name') && ~isempty(tgt.name)
        tr.targetName = string(tgt.name);
    elseif isfield(tgt, 'label') && ~isempty(tgt.label)
        tr.targetName = string(tgt.label);
    end
    if isfield(tgt, 'rcs_dbsm') && isnumeric(tgt.rcs_dbsm)
        tr.rcsDbsm = double(tgt.rcs_dbsm);
    end
    if isfield(tgt, 'rcs_profile') && ~isempty(tgt.rcs_profile)
        tr.rcsProfile = string(tgt.rcs_profile);
    else
        tr.rcsProfile = "none";
    end

    % Derived: per-target durationS — ceil of last waypoint's time, with
    % the time-backfill case handled by the EditorState recomputeTimes
    % path on the active target only. Inactive targets carry their
    % parsed durations directly.
    if all(isfinite(wp(:,4)))
        tr.durationS = ceil(wp(end, 4));
    else
        tr.durationS = 0;   % EditorState will recompute when this becomes active
    end
end


%% ========================================================================
%  Helpers (file-scope)
%% ========================================================================
function full = resolvePath(state, jsonPath)
%resolvePath  If jsonPath is not absolute, look first in state.outputDir
%             (config/targets/waypoints/), then in projectRoot, then as-is.
    jp = char(jsonPath);
    if isAbsolute(jp)
        full = jp;
        return;
    end
    candidates = strings(0);
    if state.outputDir ~= ""
        candidates(end+1) = fullfile(state.outputDir, jp);
        % Also tolerate the caller giving "m1_test" with no extension.
        if ~endsWith(lower(jp), ".json")
            candidates(end+1) = fullfile(state.outputDir, jp + ".json");
        end
    end
    if state.projectRoot ~= ""
        candidates(end+1) = fullfile(state.projectRoot, jp);
    end
    candidates(end+1) = string(jp);
    for i = 1:numel(candidates)
        if isfile(candidates(i))
            full = char(candidates(i));
            return;
        end
    end
    % Fall through to the first candidate so the caller sees a helpful
    % "not found" error with the most likely location.
    full = char(candidates(1));
end


function yes = isAbsolute(p)
%isAbsolute  Cross-platform absolute path test.
    if ispc
        yes = numel(p) >= 2 && (p(2) == ':' || startsWith(p, '\\'));
    else
        yes = ~isempty(p) && p(1) == '/';
    end
end


function list = allWaypointsTargets(decoded)
%allWaypointsTargets  Return a cell array of every behavior=="waypoints"
%                     target in the decoded payload. Tolerates the same
%                     shape variations the old firstWaypointsTarget did:
%                     {targets: [..]}, {targets: bare-struct},
%                     bare-target file, top-level cell array.
    list = {};
    if isstruct(decoded) && isfield(decoded, 'targets')
        targets = decoded.targets;
    elseif iscell(decoded)
        targets = decoded;
    elseif isstruct(decoded) && isfield(decoded, 'behavior')
        % Bare single-target file (no wrapper).
        targets = decoded;
    else
        return;
    end

    % Normalize to a cell-array we can iterate over.
    if iscell(targets)
        items = targets;
    elseif isstruct(targets)
        items = num2cell(targets);
    else
        return;
    end

    for i = 1:numel(items)
        t = items{i};
        if ~isstruct(t); continue; end
        if isfield(t, 'behavior') && strcmpi(string(t.behavior), "waypoints") ...
                && isfield(t, 'waypoints') && ~isempty(t.waypoints)
            list{end+1} = t; %#ok<AGROW>
        end
    end
end


function [wpRaw, curveMode, tensionAlpha] = pickEditorWaypoints(tgt)
%pickEditorWaypoints  Decide which list of waypoints the editor should
%                     show, and whether to enter curved mode.
%
%  Returns:
%    wpRaw        : struct-array or cell-array of waypoint structs that
%                   the outer loop will unpack into the Nx5 matrix.
%    curveMode    : "straight" | "curved" (string)
%    tensionAlpha : double in [0,1]; 0.5 (centripetal) default.
%
%  Logic:
%    1. Default to straight + alpha=0.5 (M3-era files, legacy imports).
%    2. If tgt has curve_mode="curved" AND control_waypoints is a non-
%       empty array, load the control_waypoints as the editor list and
%       flip to curved mode.
%    3. Otherwise, load the simulator-facing waypoints as editor list
%       and stay in straight mode — even if curve_mode="curved" was set
%       but the control list is missing/empty (defensive fallback; the
%       user gets visible points rather than an error).
%    4. tension alpha is pulled from curve_tension_alpha when curved
%       mode was detected; else stays at the default.
    curveMode    = "straight";
    tensionAlpha = 0.5;

    isCurvedFlag = isfield(tgt, 'curve_mode') && ~isempty(tgt.curve_mode) ...
        && strcmpi(string(tgt.curve_mode), "curved");
    hasControls = isfield(tgt, 'control_waypoints') && ~isempty(tgt.control_waypoints);

    if isCurvedFlag && hasControls
        wpRaw     = tgt.control_waypoints;
        curveMode = "curved";
        if isfield(tgt, 'curve_tension_alpha') ...
                && isnumeric(tgt.curve_tension_alpha) ...
                && isfinite(tgt.curve_tension_alpha)
            a = double(tgt.curve_tension_alpha);
            if a >= 0 && a <= 1
                tensionAlpha = a;
            end
        end
    else
        wpRaw = tgt.waypoints;
    end
end


function rgb = nextReferenceColor(idx)
%nextReferenceColor  Cycle a small palette so multiple references don't
%                    all render the same color. Tuned to be distinct from
%                    the active-target canonical blue and from each other.
    palette = [ ...
        0.85 0.50 0.20;   % orange
        0.50 0.20 0.65;   % purple
        0.20 0.65 0.40;   % green
        0.85 0.20 0.45;   % magenta
        0.55 0.55 0.20];  % olive
    rgb = palette(mod(idx-1, size(palette,1)) + 1, :);
end
