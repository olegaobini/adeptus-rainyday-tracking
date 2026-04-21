function loadFromJSON(state, jsonPath)
%loadFromJSON  Hydrate an EditorState from a waypoints-behavior target file.
%
%  Reads a JSON target definition of the schema written by exportToJSON
%  (and by hand-authored files under config/targets/waypoints/) and
%  populates `state` in place. Existing waypoints and scenario fields
%  are REPLACED. An undo snapshot is pushed first so the user can back out.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    jsonPath : absolute or project-relative path to the JSON file
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
%
%  ERRORS
%    Throws trackbench:editor:loadFromJSON:* on missing file, malformed
%    schema, or too few waypoints. UI callers should wrap in try/catch and
%    surface the message via uialert.
%
%  See also: trackbench.editor.exportToJSON,
%            trackbench.editor.EditorState,
%            trackbench.scenario.addTargetFromDef

    arguments
        state    (1,1) trackbench.editor.EditorState
        jsonPath (1,1) string
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

    % ── Locate the first "waypoints" target ─────────────────────────
    tgt = firstWaypointsTarget(decoded);
    if isempty(tgt)
        error('trackbench:editor:loadFromJSON:noWaypointsTarget', ...
            ['No target with behavior="waypoints" found in %s. ' ...
             'loadFromJSON only supports waypoint-behavior files.'], resolved);
    end

    wpRaw = tgt.waypoints;
    n = numel(wpRaw);
    if n < 2
        error('trackbench:editor:loadFromJSON:tooFewWaypoints', ...
            'File has %d waypoints; need at least 2.', n);
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
    %    (col 5 already filled for provided rows in the loop above)
    if ~speedProvided(1)
        wp(1, 5) = state.defaultSpeedKmh;
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
                legKmh = state.defaultSpeedKmh;
            end
            wp(k, 5) = legKmh;
        end
    else
        wp(~speedProvided, 5) = state.defaultSpeedKmh;
    end

    % ── Apply to state (pushUndo first so Ctrl+Z backs out a load) ──
    state.pushUndo();
    state.waypoints = wp;

    % Scenario fields — be defensive against missing keys.
    if isfield(tgt, 'name') && ~isempty(tgt.name)
        state.targetName = string(tgt.name);
    elseif isfield(tgt, 'label') && ~isempty(tgt.label)
        state.targetName = string(tgt.label);
    end
    if isfield(tgt, 'rcs_dbsm')
        state.rcsDbsm = double(tgt.rcs_dbsm);
    end
    if isfield(tgt, 'rcs_profile') && ~isempty(tgt.rcs_profile)
        state.rcsProfile = string(tgt.rcs_profile);
    else
        state.rcsProfile = "none";
    end
    if isstruct(decoded) && isfield(decoded, 'description') && ~isempty(decoded.description)
        state.description = string(decoded.description);
    else
        state.description = "";
    end

    state.selectedIndex = 0;
    state.loadedFrom    = string(resolved);

    % Recompute times so durations reflect current state (which will also
    % re-derive from col 5 speeds if we had to backfill).
    if any(~isfinite(wp(:,4)))
        state.recomputeTimes();
    else
        state.durationS = ceil(wp(end, 4));
        if isfield(decoded, 'duration_s') && isnumeric(decoded.duration_s)
            state.durationS = max(state.durationS, ceil(double(decoded.duration_s)));
        end
    end

    state.isDirty = false;   % freshly loaded == no unsaved changes
end


%% ========================================================================
%  Local helpers (file-scope)
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


function tgt = firstWaypointsTarget(decoded)
%firstWaypointsTarget  Return the first target with behavior="waypoints",
%                      tolerating cell/struct/array decodings and both
%                      top-level shapes ({targets: [..]} or bare target).
    tgt = [];
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

    % Normalize to a list we can iterate over.
    if iscell(targets)
        list = targets;
    elseif isstruct(targets)
        list = num2cell(targets);
    else
        return;
    end

    for i = 1:numel(list)
        t = list{i};
        if ~isstruct(t); continue; end
        if isfield(t, 'behavior') && strcmpi(string(t.behavior), "waypoints")
            if isfield(t, 'waypoints') && ~isempty(t.waypoints)
                tgt = t;
                return;
            end
        end
    end
end
