function fig = pathEditor()
%pathEditor  Launch the interactive target-path editor.
%
%  Draw a target flight path by left-clicking on a map, set a few scenario
%  defaults, and export to a waypoints-behavior JSON that
%  trackbench.scenario.addTargetFromDef can consume directly.
%
%  USAGE
%      addpath("scripts");
%      pathEditor;
%
%  OUTPUT
%      fig  : (optional) handle to the uifigure, useful for scripted tests.
%
%  Milestone 1 — minimum viable click-and-export:
%    - Left-click on map → appends a waypoint at default altitude/speed
%    - "Export JSON"    → writes config/targets/waypoints/<name>.json
%    - "Clear all"      → removes all waypoints
%
%  Future milestones add selection, drag, delete, undo, 3D view, spline
%  preview, kinematic warnings, NASA flight overlay, and load/save.
%  See COWORK_HANDOFF.md for the full build order.
%
%  MATLAB NAMESPACE CACHE NOTE
%    The editor lives in the +trackbench.+editor package. After editing
%    any file in that package, run
%          clear classes; clear all
%    before relaunching — otherwise MATLAB may silently use cached code.
%
%  See also: trackbench.editor.EditorState, trackbench.editor.buildUI,
%            trackbench.scenario.addTargetFromDef, runSingleScenario

    % ── Resolve project root ─────────────────────────────────────────
    %  This file lives at <root>/scripts/pathEditor.m, so the root is
    %  two directories above mfilename.
    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));

    % Make sure the +trackbench package is on the path. If the user
    % launched from outside the project, addpath of src/ covers it.
    srcDir = fullfile(projectRoot, 'src');
    if isfolder(srcDir)
        addpath(genpath(srcDir));
    end

    % ── Build state + UI ─────────────────────────────────────────────
    state = trackbench.editor.EditorState(projectRoot);
    trackbench.editor.buildUI(state);

    if nargout > 0
        fig = state.fig;
    end
end
