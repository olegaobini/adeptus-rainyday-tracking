function outPath = exportToJSON(state, filename)
%exportToJSON  Write a waypoints-behavior JSON file consumable by
%              trackbench.scenario.addTargetFromDef ("waypoints" case).
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    filename : (optional) filename without extension. Defaults to
%               state.targetName. Writes to
%                   <state.outputDir>/<filename>.json
%
%  OUTPUT
%    outPath  : absolute path of the written file (string)
%
%  SCHEMA (must match buildFromWaypoints in addTargetFromDef.m)
%    {
%      "description": "<auto>",
%      "duration_s": <scalar>,
%      "targets": [
%        {
%          "name":        "<targetName>",
%          "label":       "<targetName>",
%          "behavior":    "waypoints",
%          "rcs_dbsm":    <scalar>,
%          "rcs_profile": "<profile or omitted if 'none'>",
%          "waypoints": [
%            { "pos": [x, y, z], "time_s": <t> },
%            ...
%          ]
%        }
%      ]
%    }
%
%  NED CONVENTION
%    The editor stores altitude as POSITIVE-UP in state.waypoints(:,3).
%    addTargetFromDef expects Z as NEGATIVE-DOWN (NED). We flip the sign
%    here on export. Do NOT flip it twice — the simulator will refuse
%    positive-Z (altitude below ground) with a confusing trajectory.
%
%  VALIDATION
%    - At least 2 waypoints (addTargetFromDef errors otherwise).
%    - time_s strictly increasing (EditorState.recomputeTimes enforces a
%      1 ms floor, but we double-check here to catch manual-entry bugs
%      added in later milestones).
%
%  See also: trackbench.scenario.addTargetFromDef,
%            trackbench.editor.EditorState

    arguments
        state (1,1) trackbench.editor.EditorState
        filename (1,1) string = ""
    end

    if filename == ""
        filename = state.targetName;
    end
    if filename == ""
        filename = "m1_test";
    end

    % ── Preconditions ────────────────────────────────────────────────
    n = state.count();
    if n < 2
        error('trackbench:editor:exportToJSON:tooFewWaypoints', ...
            ['Need at least 2 waypoints to export (got %d). ' ...
             'addTargetFromDef:buildFromWaypoints requires nPts >= 2.'], n);
    end

    % Refresh timing in case the caller mutated waypoints without
    % going through addWaypoint()
    state.recomputeTimes();

    t = state.waypoints(:, 4);
    if any(diff(t) <= 0)
        error('trackbench:editor:exportToJSON:nonMonotonicTime', ...
            ['Waypoint time_s must be strictly increasing. ' ...
             'Check default speed > 0 and that duplicate points were not added.']);
    end

    % ── Build per-waypoint struct array ──────────────────────────────
    %  Order matters for JSON field order; build with explicit fields.
    %  speed_kmh is optional metadata — addTargetFromDef ignores it, but
    %  loadFromJSON uses it to avoid recomputing per-leg speed from time
    %  deltas (which can be lossy if the file was edited by hand).
    wpStructs = struct('pos', cell(1, n), 'time_s', cell(1, n), ...
                       'speed_kmh', cell(1, n));
    for k = 1:n
        x = state.waypoints(k, 1);
        y = state.waypoints(k, 2);
        altPos = state.waypoints(k, 3);   % positive-up in UI
        z = -abs(altPos);                  % NED: negative-down on disk
        wpStructs(k).pos = [x, y, z];
        wpStructs(k).time_s = t(k);
        legKmh = state.waypoints(k, 5);
        if ~isfinite(legKmh) || legKmh <= 0
            legKmh = state.defaultSpeedKmh;
        end
        wpStructs(k).speed_kmh = legKmh;
    end

    % ── Build the single-target struct ───────────────────────────────
    target = struct();
    target.name        = char(state.targetName);
    target.label       = char(state.targetName);
    target.behavior    = 'waypoints';
    target.rcs_dbsm    = state.rcsDbsm;
    if state.rcsProfile ~= "" && state.rcsProfile ~= "none"
        target.rcs_profile = char(state.rcsProfile);
    end
    target.waypoints = wpStructs;

    % ── Top-level wrapper (matches existing target config files) ─────
    dateStr = char(datetime("now", "Format", "yyyy-MM-dd"));
    if state.description ~= ""
        desc = char(state.description);
    else
        desc = sprintf('Custom path drawn interactively on %s', dateStr);
    end

    root = struct();
    root.description = desc;
    root.duration_s  = max(state.durationS, ceil(t(end)));
    % Wrap in a 1x1 cell so jsonencode produces a JSON array [ {...} ]
    % matching the existing target config convention. A plain struct(…)
    % with "targets", target works too (MATLAB emits it as an array when
    % there's one target, and as an object when there's one); the cell
    % form is unambiguous.
    root.targets = {target};

    jsonStr = jsonencode(root, 'PrettyPrint', true);

    % ── Write file ───────────────────────────────────────────────────
    outDir = state.outputDir;
    if outDir == ""
        % Caller constructed state without a project root — fall back
        % to current directory so the export at least produces a file.
        outDir = fullfile(pwd, "config", "targets", "waypoints");
    end
    if ~exist(outDir, "dir")
        mkdir(outDir);
    end

    outPath = fullfile(outDir, filename + ".json");
    fid = fopen(outPath, 'w');
    if fid < 0
        error('trackbench:editor:exportToJSON:openFailed', ...
            'Could not open %s for writing.', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));  % destroyed on function exit, closes fid
    fwrite(fid, jsonStr, 'char');

    state.isDirty = false;
end
