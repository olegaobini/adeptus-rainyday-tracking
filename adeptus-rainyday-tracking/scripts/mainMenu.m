function varargout = mainMenu()
%mainMenu  Top-level launcher for Rainy Day Tracker.
%
%  Provides the master menu window with three entry points. Each opens
%  in its own uifigure; sub-windows are independent (closing the menu
%  does not close any open sub-windows, and vice versa).
%
%  Levels:
%    1. Path Editor / Scenario Builder    (build sensors, targets,
%                                          terrain, weather, run files)
%    2. Run Simulation                    (load run file, edit, execute)
%    3. Validation & Documentation        (test suite + project docs)
%
%  USAGE
%      mainMenu;            % standalone or compiled-exe entry point
%      fig = mainMenu();    % capture the menu uifigure handle
%
%  PROJECT ROOT RESOLUTION (deployed-safe):
%    Tries multiple candidates in priority order and uses the first one
%    that contains a `config/` folder. This survives both launch styles
%    of a deployed build:
%      a) Start Menu / desktop shortcut — Windows runs the .exe with
%         pwd = the install dir (`C:\Program Files\<App>\application\`).
%         AdditionalFiles in the installer puts config/ here, so pwd
%         resolves correctly.
%      b) trackbench.bat / trackbench.vbs in dev mode — the .bat does
%         `cd /d "%~dp0"`, so pwd is the project root from git.
%      c) Bare invocation that didn't cd — fall back to the CTF cache
%         (ctfroot), where build_executable's `-a config` bundle was
%         extracted at runtime. This is the safety net when an end user
%         has somehow deleted the install-dir config/ folder.
%    For dev mode (not deployed), uses mfilename traversal as before.
%
%  ARCHITECTURE NOTES (3-option layout, post-Scenario-Runner cleanup):
%    The original 4-button menu separated "build a run file by composing
%    existing JSON configs" (Scenario Runner) from "build a run file by
%    drawing a new flight path" (Path Editor). With the v3.5 path editor
%    expansion (config-savers for sensors / terrain / weather, scenario
%    bundle export), the path editor subsumes the Scenario Runner's role
%    completely, and the Run Simulation window already covers tracker
%    selection / cache / degradation overrides for any saved run file.
%    Keeping the Scenario Runner as a 4th option duplicated workflows
%    and confused which window to use, so it was removed.
%
%  See also: pathEditor, runSimGUI, validationDocsGUI

    % ── Resolve project root (deployed-safe) ─────────────────────────
    if isdeployed
        % Try multiple candidates in priority order. First one that
        % contains a config/ subfolder wins.
        candidates = { ...
            pwd, ...                                        % install dir / launcher cd target
            ctfroot, ...                                    % CTF cache extraction
            fullfile(ctfroot, 'adeptus-rainyday-tracking') }; % deeper CTF nest, just in case
        projectRoot = '';
        for cIdx = 1:numel(candidates)
            cand = char(candidates{cIdx});
            if isfolder(fullfile(cand, 'config'))
                projectRoot = cand;
                break;
            end
        end
        if isempty(projectRoot)
            triedList = strjoin(candidates, sprintf('\n      '));
            error('trackbench:mainMenu:badRoot', ...
                ['Could not locate the project root in deployed mode.\n' ...
                 '   Tried (in order):\n      %s\n' ...
                 'Each candidate was checked for a config/ subfolder.\n' ...
                 'Reinstall the app, or contact the developer if the\n' ...
                 'installer is missing config files.'], triedList);
        end
    else
        thisFile = mfilename('fullpath');
        projectRoot = fileparts(fileparts(thisFile));
    end
    projectRoot = char(projectRoot);

    % Make sure the +trackbench package is on the path. Skip in deployed
    % mode — the namespace is baked into the CTF.
    if ~isdeployed
        srcDir = fullfile(projectRoot, 'src');
        if isfolder(srcDir)
            addpath(genpath(srcDir));
        end
    end

    % Final sanity check — in deployed mode this is redundant (we already
    % verified above), but cheap. In dev mode it's the primary check.
    if ~isfolder(fullfile(projectRoot, 'config'))
        error('trackbench:mainMenu:badRoot', ...
            ['Could not locate the project root.\n' ...
             '   Tried: %s\n' ...
             '   Expected to find: %s\n' ...
             'When launching from the compiled exe, make sure the\n' ...
             'installer placed config/ alongside mainMenu.exe (use\n' ...
             'AdditionalFiles in build_installer.m).'], ...
            projectRoot, fullfile(projectRoot, 'config'));
    end

    % ── Build menu UI ─────────────────────────────────────────────────
    % Window is small and fixed-size; sub-windows open at their own
    % default positions and the user can arrange.
    fig = uifigure('Name', 'Rainy Day Tracker', ...
                   'Position', [120 200 460 460], ...
                   'Resize', 'off');

    gl = uigridlayout(fig, [8, 1]);
    %               Title  Subtitle  spacer  btn1  btn2  btn3  flex  Exit
    gl.RowHeight = {38,    20,       12,     58,   58,   58,   '1x', 32};
    gl.ColumnWidth = {'1x'};
    gl.RowSpacing = 8;
    gl.Padding = [28 18 28 18];

    % Title
    lblTitle = uilabel(gl, 'Text', 'RAINY DAY TRACKER', ...
        'FontSize', 22, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
    lblTitle.Layout.Row = 1;

    % Subtitle
    lblSubtitle = uilabel(gl, ...
        'Text', 'Advanced Radar Tracking in Degraded Weather', ...
        'FontSize', 11, 'FontColor', [0.4 0.4 0.4], ...
        'HorizontalAlignment', 'center');
    lblSubtitle.Layout.Row = 2;

    % Three main option buttons
    btn1 = uibutton(gl, 'Text', '1.  Path Editor / Scenario Builder', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @(~,~) launchPathEditor());
    btn1.Layout.Row = 4;

    btn2 = uibutton(gl, 'Text', '2.  Run Simulation', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @(~,~) launchRunSim());
    btn2.Layout.Row = 5;

    btn3 = uibutton(gl, 'Text', '3.  Validation & Documentation', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @(~,~) launchValidationDocs());
    btn3.Layout.Row = 6;

    % Exit button at the bottom — closes the menu only. Open sub-windows
    % stay open (they're independent uifigures).
    btnExit = uibutton(gl, 'Text', 'Exit', ...
        'FontSize', 12, ...
        'ButtonPushedFcn', @(~,~) close(fig));
    btnExit.Layout.Row = 8;

    % ── Sub-window launchers ──────────────────────────────────────────
    % All wrapped in try/catch + uialert so the menu stays responsive
    % even if a sub-window fails to construct. projectRoot is captured
    % from the outer scope so each sub-window inherits the resolved root
    % rather than recomputing it (avoids the CTF-cache pitfall in deployed
    % mode).

    function launchPathEditor()
        try
            pathEditor(projectRoot);
        catch ME
            uialert(fig, ME.message, 'Path Editor Error');
        end
    end

    function launchRunSim()
        try
            runSimGUI(projectRoot);
        catch ME
            uialert(fig, ME.message, 'Run Simulation Error');
        end
    end

    function launchValidationDocs()
        % Calls validationDocsGUI when it exists; placeholder otherwise
        % during the v3.5 GUI rebuild.
        if exist('validationDocsGUI', 'file') == 2
            try
                validationDocsGUI(projectRoot);
            catch ME
                uialert(fig, ME.message, 'Validation & Docs Error');
            end
        else
            msg = ['Validation & Documentation window is coming next.' ...
                   newline newline ...
                   'For now, run validation tests by calling ' ...
                   'runTestPlan from the MATLAB console, or open ' ...
                   'README.md / handout / glossary directly from the ' ...
                   'project folder.'];
            uialert(fig, msg, 'Coming Soon', 'Icon', 'info');
        end
    end

    if nargout > 0
        varargout{1} = fig;
    end
end
