classdef EditorState < handle
%EditorState  Mutable state container for the interactive path editor.
%
%  Holds the in-progress waypoint list, scenario-level settings, UI
%  handles, selection, and undo/redo stacks. Passed by reference between
%  buildUI, drawMap, and exportToJSON so that UI callbacks can mutate a
%  single shared state object.
%
%  MILESTONE COVERAGE
%    M1 : click-to-add, auto-timing, export
%    M2 : selection, drag, delete, insert, undo/redo, load from JSON
%         (per-waypoint leg speeds stored in column 5)
%    M3+: 3D view, spline, overlays — wired up later.
%
%  WAYPOINT MATRIX FORMAT
%    state.waypoints is Nx5:
%      col 1 : x_m           (NED East, meters)
%      col 2 : y_m           (NED North, meters)
%      col 3 : altitude_m    (POSITIVE meters above ground; flipped to
%                             negative Z on export per NED convention)
%      col 4 : time_s        (seconds from start; auto-recomputed when
%                             in timing mode "auto")
%      col 5 : leg_speed_kmh (speed on the leg ARRIVING at this waypoint;
%                             ignored for row 1)
%
%  See also: trackbench.editor.buildUI, trackbench.editor.drawMap,
%            trackbench.editor.exportToJSON, trackbench.editor.loadFromJSON,
%            pathEditor
%
%  MATLAB NAMESPACE CACHE NOTE
%    After editing this file, run `clear classes; clear all` before
%    relaunching pathEditor.

    properties
        % ── Waypoint list (see FORMAT comment above) ─────────────────
        waypoints (:,5) double = zeros(0,5)

        % ── Scenario-level defaults (spec §4.3) ──────────────────────
        targetName       (1,1) string = "m1_test"
        description      (1,1) string = ""
        rcsDbsm          (1,1) double = 10
        rcsProfile       (1,1) string = "airliner"
        defaultSpeedKmh  (1,1) double = 900
        defaultAltitudeM (1,1) double = 3000
        timingMode       (1,1) string = "auto"     % "auto" | "manual" (deferred)
        durationS        (1,1) double = 0

        % ── View options (M3) ────────────────────────────────────────
        viewMode         (1,1) string = "2d"        % "2d" | "3d" (M3.3)
        gridSpacingKm    (1,1) double = 5
        splineMode       (1,1) logical = false
        colorByAltitude  (1,1) logical = false      % M3.1: colormap waypoints by alt
        radarEastM       (1,1) double = 0           % M3.5: editable radar position
        radarNorthM      (1,1) double = 0           % M3.5: editable radar position
        has3DViewState   (1,1) logical = false      % M3.3: true once 3D camera has been auto-fitted; reset on mode switch so next 3D entry autofits again

        % ── Selection + drag state (M2) ──────────────────────────────
        selectedIndex    (1,1) double = 0          % 0 = none, else 1..N
        hoverIndex       (1,1) double = 0          % used for hit-test viz
        dragActive       (1,1) logical = false
        dragStartWP      double = []               % snapshot for undo granularity
        selectHitRadiusM (1,1) double = 500        % spec §4.1 — click-within-500m

        % ── Middle-click pan state (shortcuts pass, 2D only) ─────────
        % Populated in onAxesClick when evt.Button == 2, consumed by
        % onMouseMove's pan branch, and cleared by onMouseUp. In 3D the
        % figure's built-in interactivity handles pan, so these fields
        % stay empty.
        panActive        (1,1) logical = false
        panStartFigPt    double = []               % fig.CurrentPoint at pan start (pixels)
        panStartXLim     double = []               % ax.XLim snapshot at pan start
        panStartYLim     double = []               % ax.YLim snapshot at pan start

        % ── Undo/redo (M2) ───────────────────────────────────────────
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

        % ── M3 UI handles ────────────────────────────────────────────
        colorByAltCheckbox = gobjects(1)     % M3.1
        gridSpacingDD      = gobjects(1)     % M3.2
        viewModeBtn        = gobjects(1)     % M3.3 (toggle button)
        radarEastField     = gobjects(1)     % M3.5
        radarNorthField    = gobjects(1)     % M3.5
        previewBtn         = gobjects(1)     % M3.4
        previewFig         = gobjects(1)     % M3.4: handle to the open preview figure (if any)

        % ── Callback stashes ─────────────────────────────────────────
        % Patch C: drawMap calls cla(ax,'reset') which wipes
        % ax.ButtonDownFcn to ''. buildUI stashes the installed handle
        % here so drawMap can re-install it after the reset.
        axesClickFcn       function_handle = function_handle.empty

        % ── Paths ────────────────────────────────────────────────────
        projectRoot      (1,1) string = ""
        outputDir        (1,1) string = ""
        loadedFrom       (1,1) string = ""

        % ── Misc ─────────────────────────────────────────────────────
        isDirty          (1,1) logical = false
    end

    methods
        function obj = EditorState(projectRoot)
            %EditorState  Construct with the project root for path resolution.
            if nargin >= 1 && ~isempty(projectRoot)
                obj.projectRoot = string(projectRoot);
                obj.outputDir = fullfile(obj.projectRoot, ...
                    "config", "targets", "waypoints");
            end
        end

        %% ── Mutation (auto-pushes undo) ──────────────────────────────
        function addWaypoint(obj, x, y)
            %addWaypoint  Append a waypoint at (x,y) with the current
            %             default altitude and default leg speed.
            obj.pushUndo();
            alt = obj.defaultAltitudeM;
            spd = obj.defaultSpeedKmh;
            obj.waypoints(end+1, :) = [x, y, alt, 0, spd];
            obj.recomputeTimes();
            obj.selectedIndex = size(obj.waypoints, 1);
            obj.isDirty = true;
        end

        function insertAfter(obj, idx, x, y)
            %insertAfter  Insert a new waypoint between idx and idx+1.
            %             If idx == 0, insert at the start (becomes #1).
            %             If idx == count(), appends.
            n = obj.count();
            if idx < 0 || idx > n
                error('trackbench:editor:insertAfter:badIndex', ...
                    'idx=%d out of range [0..%d]', idx, n);
            end
            obj.pushUndo();
            alt = obj.defaultAltitudeM;
            if idx >= 1 && idx <= n
                alt = obj.waypoints(idx, 3);   % inherit from neighbor
            end
            spd = obj.defaultSpeedKmh;
            newRow = [x, y, alt, 0, spd];
            obj.waypoints = [obj.waypoints(1:idx, :); newRow; obj.waypoints(idx+1:end, :)];
            obj.recomputeTimes();
            obj.selectedIndex = idx + 1;
            obj.isDirty = true;
        end

        function removeSelected(obj)
            if obj.selectedIndex < 1 || obj.selectedIndex > obj.count()
                return;
            end
            obj.pushUndo();
            idx = obj.selectedIndex;
            obj.waypoints(idx, :) = [];
            if obj.count() == 0
                obj.selectedIndex = 0;
            else
                obj.selectedIndex = max(1, min(idx, obj.count()));
            end
            obj.recomputeTimes();
            obj.isDirty = true;
        end

        function clear(obj)
            if obj.count() == 0 && obj.loadedFrom == ""
                return;  % nothing to undo
            end
            obj.pushUndo();
            obj.waypoints = zeros(0, 5);
            obj.durationS = 0;
            obj.selectedIndex = 0;
            obj.loadedFrom = "";
            obj.isDirty = true;
        end

        function setWaypointProperty(obj, idx, field, value)
            %setWaypointProperty  Edit x/y/altitude/leg_speed for a single
            %                     waypoint. Called from the sidebar fields.
            if idx < 1 || idx > obj.count(); return; end
            obj.pushUndo();
            switch lower(string(field))
                case "x";         obj.waypoints(idx, 1) = value;
                case "y";         obj.waypoints(idx, 2) = value;
                case "altitude";  obj.waypoints(idx, 3) = max(0, value);
                case "speed";     obj.waypoints(idx, 5) = max(1, value);
                otherwise
                    error('trackbench:editor:unknownField', ...
                        'Unknown waypoint field: %s', field);
            end
            obj.recomputeTimes();
            obj.isDirty = true;
        end

        function moveSelectedTo(obj, x, y, commit)
            %moveSelectedTo  Used during drag. With commit=false we don't
            %                push undo (per-frame move is noisy); the
            %                caller pushes undo once on drag start.
            if obj.selectedIndex < 1; return; end
            if nargin < 4; commit = true; end
            if commit; obj.pushUndo(); end
            obj.waypoints(obj.selectedIndex, 1) = x;
            obj.waypoints(obj.selectedIndex, 2) = y;
            obj.recomputeTimes();
            obj.isDirty = true;
        end

        function applyDefaultAltitudeToAll(obj)
            %applyDefaultAltitudeToAll  Bulk-set altitude on every existing
            %                           waypoint to the current default.
            %                           Per user feedback on M1.
            if obj.count() == 0; return; end
            obj.pushUndo();
            obj.waypoints(:, 3) = obj.defaultAltitudeM;
            obj.recomputeTimes();
            obj.isDirty = true;
        end

        %% ── Selection / hit testing ─────────────────────────────────
        function r = hitRadiusM(obj)
            %hitRadiusM  Zoom-aware click tolerance in world meters.
            %
            %  A fixed 500 m radius feels tight at default zoom (40 km wide)
            %  and far too loose when zoomed in. We compute the radius as
            %  ~2.5% of the current axes width/height (whichever is larger),
            %  which maps to roughly the same ~15 screen pixels at any zoom.
            %  Clamped to [250 m, 2500 m] so very extreme zooms don't make
            %  hit-testing useless or absurdly permissive.
            r = obj.selectHitRadiusM;
            ax = obj.ax;
            if ~isgraphics(ax); return; end
            try
                xSpan = ax.XLim(2) - ax.XLim(1);
                ySpan = ax.YLim(2) - ax.YLim(1);
                span = max(xSpan, ySpan);
                r = max(500, min(5000, 0.05 * span));
            catch
                % If the axes limits aren't set yet, stick with the
                % property default.
            end
        end

        function idx = findWaypointAt(obj, x, y, maxDistM)
            %findWaypointAt  Return the index of the waypoint whose 2D
            %                position is closest to (x,y) within maxDistM
            %                meters. Returns 0 if nothing is within range.
            if nargin < 4 || isempty(maxDistM); maxDistM = obj.hitRadiusM(); end
            if obj.count() == 0; idx = 0; return; end
            dx = obj.waypoints(:,1) - x;
            dy = obj.waypoints(:,2) - y;
            d  = sqrt(dx.*dx + dy.*dy);
            [minD, k] = min(d);
            if minD <= maxDistM
                idx = k;
            else
                idx = 0;
            end
        end

        function [segIdx, projXY] = findSegmentAt(obj, x, y, maxDistM)
            %findSegmentAt  If (x,y) lies within maxDistM of a path segment
            %               (but NOT within the waypoint hit radius of
            %               either endpoint), return the index of the
            %               segment-start waypoint and the perpendicular
            %               projection of (x,y) onto that segment. Used
            %               by Shift+click insert.
            if nargin < 4 || isempty(maxDistM); maxDistM = obj.hitRadiusM(); end
            segIdx = 0;
            projXY = [x, y];
            n = obj.count();
            if n < 2; return; end
            bestDist = maxDistM;
            for k = 1:n-1
                p1 = obj.waypoints(k,   1:2);
                p2 = obj.waypoints(k+1, 1:2);
                [d, proj, t] = pointToSegmentDistance([x y], p1, p2);
                % Require hit to be between the endpoints (not at an
                % endpoint — those are handled by findWaypointAt).
                if d <= bestDist && t > 0.02 && t < 0.98
                    bestDist = d;
                    segIdx = k;
                    projXY = proj;
                end
            end
        end

        %% ── Undo / redo ─────────────────────────────────────────────
        function pushUndo(obj)
            %pushUndo  Snapshot current state (waypoints + scenario props
            %          + selection) onto the undo stack. Called before any
            %          mutating operation.
            snap = obj.snapshot();
            obj.undoStack{end+1} = snap;
            if numel(obj.undoStack) > obj.maxUndoLevels
                obj.undoStack(1:end-obj.maxUndoLevels) = [];
            end
            % A new edit invalidates the redo future.
            obj.redoStack = {};
        end

        function ok = undo(obj)
            %undo  Pop the newest undo snapshot and restore it. Pushes
            %      current state onto redoStack so a subsequent redo
            %      round-trips.
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
            snap.waypoints        = obj.waypoints;
            snap.targetName       = obj.targetName;
            snap.description      = obj.description;
            snap.rcsDbsm          = obj.rcsDbsm;
            snap.rcsProfile       = obj.rcsProfile;
            snap.defaultSpeedKmh  = obj.defaultSpeedKmh;
            snap.defaultAltitudeM = obj.defaultAltitudeM;
            snap.selectedIndex    = obj.selectedIndex;
            snap.loadedFrom       = obj.loadedFrom;
        end

        function restore(obj, snap)
            obj.waypoints        = snap.waypoints;
            obj.targetName       = snap.targetName;
            obj.description      = snap.description;
            obj.rcsDbsm          = snap.rcsDbsm;
            obj.rcsProfile       = snap.rcsProfile;
            obj.defaultSpeedKmh  = snap.defaultSpeedKmh;
            obj.defaultAltitudeM = snap.defaultAltitudeM;
            obj.selectedIndex    = min(snap.selectedIndex, obj.count());
            obj.loadedFrom       = snap.loadedFrom;
            obj.recomputeTimes();
            obj.isDirty = true;
        end

        %% ── Timing ──────────────────────────────────────────────────
        function recomputeTimes(obj)
            %recomputeTimes  Auto-derive time_s from cumulative leg
            %                distance and per-leg speed (col 5).
            n = obj.count();
            if n == 0
                obj.durationS = 0;
                return;
            end
            obj.waypoints(1, 4) = 0;
            if n < 2
                obj.durationS = 0;
                return;
            end
            for k = 2:n
                legKmh = obj.waypoints(k, 5);
                if ~isfinite(legKmh) || legKmh <= 0
                    legKmh = obj.defaultSpeedKmh;
                end
                legMs = legKmh * 1000 / 3600;
                dx = obj.waypoints(k,1) - obj.waypoints(k-1,1);
                dy = obj.waypoints(k,2) - obj.waypoints(k-1,2);
                dz = obj.waypoints(k,3) - obj.waypoints(k-1,3);
                dist = sqrt(dx*dx + dy*dy + dz*dz);
                dt = max(dist / legMs, 1e-3);   % addTargetFromDef needs strictly increasing t
                obj.waypoints(k, 4) = obj.waypoints(k-1, 4) + dt;
            end
            obj.durationS = ceil(obj.waypoints(end, 4));
        end

        function n = count(obj)
            n = size(obj.waypoints, 1);
        end
    end
end


%% ========================================================================
%  Local helper (file-scope)
%% ========================================================================
function [d, proj, t] = pointToSegmentDistance(p, a, b)
%pointToSegmentDistance  Perpendicular distance from point p to segment a-b,
%                        along with the projection point and parameter t
%                        (0 at a, 1 at b).
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
