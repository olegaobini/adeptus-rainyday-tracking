function fig = pathEditor(projectRoot, domainKey)
%pathEditor  Launch the interactive target-path editor.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Draw a target flight path by left-clicking on a map, set a few scenario
%  defaults, and export to a waypoints-behavior JSON that
%  trackbench.scenario.addTargetFromDef can consume directly.
%
%  USAGE
%      addpath("scripts");
%      pathEditor;                 % standalone, resolves root from script location
%      pathEditor(projectRoot);    % caller (e.g. runScenarioGUI) passes the root
%
%  OUTPUT
%      fig  : (optional) handle to the uifigure, useful for scripted tests.
%
%  PROJECT ROOT RESOLUTION (deployed-safe, in priority order)
%    1. Explicit projectRoot argument from caller (most reliable; used when
%       launched from runScenarioGUI so the GUI's already-resolved root is
%       handed straight through).
%    2. isdeployed → pwd. The compiled exe's launcher cd's into the project
%       dir before invoking the GUI, so pwd is the real project root.
%    3. mfilename('fullpath') traversal — works for `addpath('scripts');
%       pathEditor;` from the MATLAB console.
%
%  Why this matters: in deployed (.exe) mode mfilename('fullpath') returns
%  a path inside the MCR's temporary CTF cache, not the project. Without
%  the deployed-mode branch, exports landed in
%  <temp>/mcrCache/.../scripts/pathEditor.m → walks up two → an MCR temp
%  folder, NOT the project's config/runs.
%
%  Milestone 1 — minimum viable click-and-export:
%    - Left-click on map → appends a waypoint at default altitude/speed
%    - "Export JSON"    → writes config/targets/waypoints/<n>.json
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

    % ── Resolve project root (deployed-safe) ─────────────────────────
    if nargin >= 1 && ~isempty(projectRoot)
        % Caller-supplied root wins — most reliable path.
        projectRoot = char(projectRoot);
    elseif isdeployed
        % In a compiled exe, mfilename returns a CTF-cache path, not the
        % project. Our launcher cd's into the project dir before starting,
        % so pwd is correct.
        projectRoot = pwd;
    else
        % Console / IDE: this file lives at <root>/scripts/pathEditor.m,
        % so the root is two directories above mfilename.
        thisFile = mfilename('fullpath');
        projectRoot = fileparts(fileparts(thisFile));
    end

    % Make sure the +trackbench package is on the path. Skip in deployed
    % mode — the namespace is baked into the CTF and addpath would no-op.
    if ~isdeployed
        srcDir = fullfile(projectRoot, 'src');
        if isfolder(srcDir)
            addpath(genpath(srcDir));
        end
    end

    % ── Sanity check the resolved root before handing it to EditorState
    %    so the user gets a clear message instead of a confusing "no such
    %    sensor type" deep inside an export call later.
    if ~isfolder(fullfile(projectRoot, 'config'))
        error('trackbench:pathEditor:badRoot', ...
            ['pathEditor could not locate the project root.\n' ...
             '   Tried: %s\n' ...
             '   Expected to find: %s\n' ...
             'When launching from the compiled exe, make sure the\n' ...
             'launcher (trackbench.bat / trackbench.vbs) sets the working\n' ...
             'directory to the project folder before starting the GUI.'], ...
            projectRoot, fullfile(projectRoot, 'config'));
    end

    % ── Build state + UI ─────────────────────────────────────────────
    if nargin < 2 || isempty(domainKey); domainKey = "radar"; end
    state = trackbench.editor.EditorState(projectRoot, domainKey);
    trackbench.editor.buildUI(state);

    if nargout > 0
        fig = state.fig;
    end
end
