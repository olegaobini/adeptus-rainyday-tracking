function [outPath, excludedRefCount] = exportToJSON(state, filename)
%exportToJSON  Write a waypoints-behavior JSON file consumable by
%              trackbench.scenario.addTargetFromDef ("waypoints" case).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  M5 §3.2 — MULTI-TARGET WITH REFERENCE FILTERING
%    Writes EVERY non-reference (writable) target in state.targets to the
%    same root.targets array. Reference targets (readOnly == true) are
%    NEVER exported — they came from disk and stay on disk untouched. If
%    the user wants them in the exported file, they should Duplicate the
%    reference first; the duplicate is a writable copy that will export.
%
%    Sim-side contract is unchanged: addTargetFromDef.m has always
%    iterated root.targets, so this milestone is just finally letting
%    the editor emit what the simulator has always accepted.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    filename : (optional) filename without extension. Default rules:
%               - 0 writable targets → error (nothing to export)
%               - 1 writable target  → that target's targetName
%               - 2+ writable        → "multi_target"
%
%  OUTPUTS
%    outPath          : absolute path of the written file (string)
%    excludedRefCount : how many reference targets were filtered out (int).
%                       The UI uses this to post the "(M reference target(s)
%                       were not included.)" message in the success dialog.
%
%  SCHEMA (per-target struct, unchanged from M4)
%    {
%      "name":        "<targetName>",
%      "label":       "<targetName>",
%      "behavior":    "waypoints",
%      "rcs_dbsm":    <scalar>,
%      "rcs_profile": "<profile or omitted if 'none'>",
%      "curve_mode":  "straight" | "curved",
%      "curve_tension_alpha": <0..1>,         (only when curved)
%      "control_waypoints": [ ... ],           (only when curved)
%      "waypoints": [ ... ]
%    }
%
%  TOP LEVEL
%    {
%      "description": "<auto>",
%      "duration_s": <max across writable targets>,
%      "targets": [ {target1}, {target2}, ... ]
%    }
%
%  NED CONVENTION
%    UI stores altitude POSITIVE-UP in waypoints(:,3). On disk Z is
%    NEGATIVE-DOWN (NED). The flip happens in buildWaypointStructs. Do
%    NOT flip twice — the simulator will refuse positive Z.
%
%  VALIDATION (per writable target)
%    - At least 2 waypoints (addTargetFromDef errors otherwise).
%    - time_s strictly increasing (recomputeTimes enforces a 1 ms floor
%      but we double-check here against hand-edited matrices).
%
%  See also: trackbench.scenario.addTargetFromDef,
%            trackbench.editor.EditorState,
%            trackbench.editor.TargetRecord,
%            trackbench.editor.loadFromJSON

    arguments
        state (1,1) trackbench.editor.EditorState
        filename (1,1) string = ""
    end

    % ── Partition targets into writable vs reference ─────────────────
    %  state.targets is a 1xN TargetRecord array. Use a numeric loop
    %  rather than logical indexing on a property because MATLAB syntax
    %  for property-based filtering on object arrays is finicky and
    %  fragile across releases.
    nTargets   = numel(state.targets);
    writable   = trackbench.editor.TargetRecord.empty;
    nRefs      = 0;
    for k = 1:nTargets
        if state.targets(k).readOnly
            nRefs = nRefs + 1;
        else
            writable(end+1) = state.targets(k); %#ok<AGROW>
        end
    end
    excludedRefCount = nRefs;

    nWritable = numel(writable);
    if nWritable == 0
        error('trackbench:editor:exportToJSON:noWritableTargets', ...
            ['No writable targets to export. ' ...
             'All %d target(s) are reference (read-only). ' ...
             'Duplicate a reference target to create an editable copy first.'], ...
             nTargets);
    end

    % ── Refresh active-target timing in case the caller mutated waypoints
    %    via the dependent setter without recomputing. Inactive writables
    %    only get mutated through setActiveIdx → mutator pattern, which
    %    already ensures monotonic times.
    if state.hasActiveTarget() && ~state.activeTarget().readOnly
        state.recomputeTimes();
        % The recompute happens on the active target — pick it back up
        % out of the writable list so the active row uses the refreshed
        % copy. If the active target IS one of the writables (it usually
        % is), we need to refresh its mirror in `writable`.
        for k = 1:nWritable
            if writable(k).targetName == state.activeTarget().targetName
                writable(k) = state.activeTarget();
            end
        end
    end

    % ── Decide filename ──────────────────────────────────────────────
    if filename == ""
        if nWritable >= 2
            filename = "multi_target";
        else
            filename = writable(1).targetName;
        end
    end
    if filename == ""
        filename = "m1_test";
    end

    % ── Build per-target structs ─────────────────────────────────────
    targetStructs = cell(1, nWritable);
    durations     = zeros(1, nWritable);
    for k = 1:nWritable
        tr = writable(k);
        validateTargetForExport(tr);
        targetStructs{k} = buildTargetStruct(tr);
        durations(k) = max(tr.durationS, ceil(tr.waypoints(end, 4)));
    end

    % ── Top-level wrapper ────────────────────────────────────────────
    dateStr = char(datetime("now", "Format", "yyyy-MM-dd"));
    if state.description ~= ""
        desc = char(state.description);
    elseif nWritable >= 2
        desc = sprintf( ...
            'Multi-target scenario (%d targets) exported on %s.', ...
            nWritable, dateStr);
    else
        desc = sprintf('Custom path drawn interactively on %s', dateStr);
    end

    root = struct();
    root.description = desc;
    root.duration_s  = max(durations);
    % Wrap as a cell so jsonencode emits an array even when nWritable==1.
    root.targets = targetStructs;

    jsonStr = jsonencode(root, 'PrettyPrint', true);

    % ── Write file ───────────────────────────────────────────────────
    outDir = state.outputDir;
    if outDir == ""
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
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    % Aggregate dirty bit clears on a successful export — every writable
    % target's contents are now on disk. Reference targets are unchanged
    % (they were already on disk).
    state.anyDirty = false;
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function validateTargetForExport(tr)
%validateTargetForExport  Per-target preconditions for export.
%
%  Errors with the offending target's name in the message so the user can
%  fix the right target when there are several in the editor.
    n = size(tr.waypoints, 1);
    if n < 2
        error('trackbench:editor:exportToJSON:tooFewWaypoints', ...
            ['Target "%s" has %d waypoints; need at least 2 to export ' ...
             '(addTargetFromDef:buildFromWaypoints requires nPts >= 2).'], ...
             char(tr.targetName), n);
    end
    t = tr.waypoints(:, 4);
    if any(diff(t) <= 0)
        error('trackbench:editor:exportToJSON:nonMonotonicTime', ...
            ['Target "%s" has non-monotonic time_s on its waypoints. ' ...
             'Check default speed > 0 and that duplicate points were not added.'], ...
             char(tr.targetName));
    end
end


function s = buildTargetStruct(tr)
%buildTargetStruct  Convert one writable TargetRecord into the JSON
%                   per-target struct. Pure transform — no state writes.
%
%  Curve-mode rules (mirrored from M4 single-target export):
%    - In straight mode, "waypoints" is the control list itself.
%    - In curved mode, "control_waypoints" is the editable points and
%      "waypoints" is the densified Nx5 the simulator linearly interpolates
%      between. addTargetFromDef does not need to know about curves.
    controlStructs = buildWaypointStructs( ...
        tr.waypoints(:, 1:5), tr.defaultSpeedKmh);

    isCurved = (tr.curveMode == "curved");
    if isCurved
        control5 = tr.waypoints(:, 1:5);
        [densePts, denseT] = trackbench.editor.catmullRomCurve( ...
            control5, tr.curveDensityPerSeg, tr.curveTensionAlpha);
        dense5 = buildDenseNx5(densePts, denseT, tr.defaultSpeedKmh);
        simStructs = buildWaypointStructs(dense5, tr.defaultSpeedKmh);
    else
        simStructs = controlStructs;
    end

    s = struct();
    s.name        = char(tr.targetName);
    s.label       = char(tr.targetName);
    s.behavior    = 'waypoints';
    s.rcs_dbsm    = tr.rcsDbsm;
    if tr.rcsProfile ~= "" && tr.rcsProfile ~= "none"
        s.rcs_profile = char(tr.rcsProfile);
    end
    s.curve_mode = char(tr.curveMode);
    if isCurved
        s.curve_tension_alpha = tr.curveTensionAlpha;
        s.control_waypoints   = controlStructs;
    end
    s.waypoints = simStructs;
end


function wpStructs = buildWaypointStructs(wp5, defaultSpeedKmh)
%buildWaypointStructs  Convert an Nx5 [x y alt_pos time_s speed_kmh]
%                      matrix into the per-waypoint struct array consumed
%                      by jsonencode. Positive-up altitude flipped to
%                      NED (negative-down) z on the way out.
    n = size(wp5, 1);
    wpStructs = struct('pos', cell(1, n), 'time_s', cell(1, n), ...
                       'speed_kmh', cell(1, n));
    for k = 1:n
        x = wp5(k, 1);
        y = wp5(k, 2);
        altPos = wp5(k, 3);
        z = -abs(altPos);
        wpStructs(k).pos    = [x, y, z];
        wpStructs(k).time_s = wp5(k, 4);
        legKmh = wp5(k, 5);
        if ~isfinite(legKmh) || legKmh <= 0
            legKmh = defaultSpeedKmh;
        end
        wpStructs(k).speed_kmh = legKmh;
    end
end


function dense5 = buildDenseNx5(densePts, denseT, defaultSpeedKmh)
%buildDenseNx5  Assemble an Nx5 dense-waypoint matrix from the output of
%               catmullRomCurve and a default leg speed.
%
%  Per-dense-point speed is back-computed from distance/dt between
%  consecutive dense points. Row 1 inherits the default (no predecessor
%  to measure against). If dt==0 or distance==0 on any row, the default
%  is substituted — catmullRomCurve nudges coincident knots by 1e-9 so
%  this rarely fires, but the guard keeps export robust against
%  hand-edited control files.
    M = size(densePts, 1);
    dense5 = zeros(M, 5);
    dense5(:, 1:3) = densePts;
    dense5(:, 4)   = denseT;
    dense5(1, 5)   = defaultSpeedKmh;
    for k = 2:M
        dx = densePts(k,1) - densePts(k-1,1);
        dy = densePts(k,2) - densePts(k-1,2);
        dz = densePts(k,3) - densePts(k-1,3);
        dist = sqrt(dx*dx + dy*dy + dz*dz);
        dt   = denseT(k) - denseT(k-1);
        if dt > 0 && dist > 0
            dense5(k, 5) = (dist / dt) * 3.6;
        else
            dense5(k, 5) = defaultSpeedKmh;
        end
    end
end
