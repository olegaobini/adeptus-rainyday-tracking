classdef TargetRecord
%TargetRecord  Per-target state for the multi-target path editor (M5 §3.1).
%
%  A plain VALUE class. Owning EditorState keeps a 1xN array of these in
%  state.targets and the active one at state.targets(state.activeIdx).
%  Mutators read-mutate-writeback:
%      tr = state.activeTarget();
%      tr.foo = bar;
%      state.setActiveTarget(tr);
%
%  WHY VALUE AND NOT HANDLE
%    Undo/redo snapshots the targets array. With value classes the snapshot
%    is an independent copy of every record. With handle classes the
%    snapshot would hold references to the same objects future edits
%    mutate, and Ctrl+Z would do nothing. Do not change this to a handle.
%
%  WHAT MOVED OFF EditorState
%    waypoints, targetName, rcsDbsm, rcsProfile, defaultSpeedKmh,
%    defaultAltitudeM, curveMode, curveTensionAlpha, curveDensityPerSeg,
%    durationS, isDirty (per-target dirty bit; EditorState now keeps an
%    aggregate anyDirty flag).
%
%  WHAT IS NEW IN M5
%    readOnly      true ⇒ reference-target overlay (§3.2 will start
%                  setting this; §3.1 leaves it false everywhere).
%    sourceFile    where this target was loaded from (used in §3.2 for
%                  status messages and re-save UX). "" when freshly
%                  created in the editor.
%    displayColor  per-target render color used by the inactive-rendering
%                  path. Active target always overrides to the canonical
%                  blue regardless of this value (see drawMap.m).
%
%  See also: trackbench.editor.EditorState, trackbench.editor.drawMap

    properties
        % ── Path data ──────────────────────────────────────────────
        % Same Nx5 format the M4 editor used: [x_m, y_m, alt_m, time_s, leg_speed_kmh].
        waypoints (:,5) double = zeros(0,5)

        % ── Scenario metadata (was on EditorState in M4) ───────────
        targetName       (1,1) string  = "target_1"
        rcsDbsm          (1,1) double  = 10
        rcsProfile       (1,1) string  = "airliner"
        defaultSpeedKmh  (1,1) double  = 900
        defaultAltitudeM (1,1) double  = 3000

        % ── Curve options (was on EditorState in M4) ───────────────
        curveMode          (1,1) string  = "straight"
        curveTensionAlpha  (1,1) double  = 0.5
        curveDensityPerSeg (1,1) double  = 50

        % ── Derived ────────────────────────────────────────────────
        durationS (1,1) double = 0

        % ── M5 additions ───────────────────────────────────────────
        readOnly     (1,1) logical = false
        sourceFile   (1,1) string  = ""
        displayColor (1,3) double  = [0.20 0.45 0.85]
    end

    methods
        function n = count(obj)
            n = size(obj.waypoints, 1);
        end
    end
end
