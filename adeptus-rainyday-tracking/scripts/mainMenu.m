function varargout = mainMenu()
%mainMenu  Top-level launcher for Rainy Day Tracker.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
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
%    Dev mode (running mainMenu from MATLAB):
%      projectRoot = the source folder, resolved by mfilename traversal.
%      No copying happens; the source tree is the working tree.
%
%    Deployed mode (compiled exe):
%      The install dir (typically C:\Program Files\Rainy Day Tracker\
%      application\) is read-only for standard Windows users, so cache,
%      results, and run-file edits cannot land there. To make the app
%      work as a non-admin standard install, on first launch we:
%        1. Find the install dir by scanning pwd, ctfroot, and the
%           nested ctfroot/adeptus-rainyday-tracking. Whichever has a
%           config/ subfolder wins (this is `findDeployedInstallRoot`).
%        2. Resolve a per-user data dir at %LOCALAPPDATA%\RainyDay
%           (with USERPROFILE / tempdir fallbacks).
%        3. Seed it once: copy config/, docs/, README.md, CHECKPOINT.md
%           from the install dir; create empty cache/ and results/.
%        4. cd into the user data dir and set projectRoot = userRoot.
%      All sub-windows (pathEditor, runSimGUI, validationDocsGUI) and
%      runSingleScenario then operate against the writable user data
%      dir without further changes.
%
%      Reset to factory defaults: delete %LOCALAPPDATA%\RainyDay and
%      relaunch. The next launch re-seeds from the current install.
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
    %  In dev mode (running from MATLAB), projectRoot = the source folder.
    %
    %  In deployed mode (compiled exe installed to Program Files via the
    %  Web/Offline installer), the install dir is READ-ONLY for standard
    %  Windows users — cache writes, results writes, and run-file edits
    %  via runSimGUI all fail there. To fix this we seed a per-user data
    %  dir at %LOCALAPPDATA%\RainyDay on first launch (copying config/,
    %  docs/, README, etc. from the read-only install dir), then cd into
    %  it so any downstream pwd-based root resolution (e.g.
    %  runSingleScenario's resolveRootFromThisFile) automatically picks
    %  up the writable copy.
    %
    %  Reset to factory defaults: delete %LOCALAPPDATA%\RainyDay and
    %  relaunch. The next launch re-seeds from the install dir.
    if isdeployed
        installRoot = findDeployedInstallRoot();
        userRoot    = resolveUserDataRoot();
        seedUserDataRoot(userRoot, installRoot);
        try
            cd(userRoot);
        catch ME
            error('trackbench:mainMenu:cannotCdUserRoot', ...
                ['Could not change directory to the per-user data dir:\n' ...
                 '   %s\n' ...
                 '   Reason: %s\n' ...
                 'Check that the folder is writable, or delete it and\n' ...
                 'relaunch to re-seed.'], userRoot, ME.message);
        end
        projectRoot = userRoot;
        fprintf('[mainMenu] Install root  : %s\n', installRoot);
        fprintf('[mainMenu] User data dir : %s\n', userRoot);
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


%% ========================================================================
%  DEPLOYED-MODE HELPERS — per-user data dir resolution & seeding.
%
%  These are no-ops in dev mode (the isdeployed branch never executes).
%  See the projectRoot block in mainMenu for rationale.
%% ========================================================================

function installRoot = findDeployedInstallRoot()
%findDeployedInstallRoot  Locate the read-only install dir containing config/.
%   Tries pwd, ctfroot, and ctfroot/adeptus-rainyday-tracking in order.
%   Errors clearly if none have a config/ subfolder.
    candidates = { ...
        pwd, ...                                        % install dir / launcher cd target
        ctfroot, ...                                    % CTF cache extraction
        fullfile(ctfroot, 'adeptus-rainyday-tracking') }; % deeper CTF nest, just in case
    installRoot = '';
    for cIdx = 1:numel(candidates)
        cand = char(candidates{cIdx});
        if isfolder(fullfile(cand, 'config'))
            installRoot = cand;
            return;
        end
    end
    triedList = strjoin(candidates, sprintf('\n      '));
    error('trackbench:mainMenu:badInstallRoot', ...
        ['Could not locate the install dir containing config/.\n' ...
         '   Tried (in order):\n      %s\n' ...
         'Reinstall the app, or contact the developer if the\n' ...
         'installer is missing config files (AdditionalFiles).'], triedList);
end


function userRoot = resolveUserDataRoot()
%resolveUserDataRoot  Per-user, writable data dir for cache/results/configs.
%   Returns %LOCALAPPDATA%\RainyDay, falling back to
%   USERPROFILE\AppData\Local\RainyDay, then tempdir\RainyDay if neither
%   env var is set. Does NOT create the folder — seedUserDataRoot's job.
    appData = getenv('LOCALAPPDATA');
    if isempty(appData)
        userProfile = getenv('USERPROFILE');
        if ~isempty(userProfile)
            appData = fullfile(userProfile, 'AppData', 'Local');
        end
    end
    if isempty(appData)
        appData = tempdir();
        warning('trackbench:mainMenu:noAppData', ...
            ['Neither LOCALAPPDATA nor USERPROFILE is set. Falling back\n' ...
             'to temp dir: %s\nUser data may not persist across reboots.'], ...
            appData);
    end
    userRoot = fullfile(appData, 'RainyDay');
end


function seedUserDataRoot(userRoot, installRoot)
%seedUserDataRoot  First-launch population of the per-user data dir.
%   Idempotent: only copies items that don't already exist in userRoot,
%   so user edits to run files (etc.) survive subsequent launches.
%   Always ensures cache/ and results/ exist (even on later launches).
%
%   Items copied from installRoot (skipped if already present):
%     config/         — sensors, targets, terrain, trackers, weather, runs.
%                       User edits land here.
%     docs/           — PDFs, handouts, glossary (read by validationDocsGUI).
%     README.md       — project overview.
%     CHECKPOINT.md   — current state notes.
%
%   Items always created (writable scratch):
%     cache/    — detection cache (.mat files keyed by run name).
%     results/  — saved tracker results (timestamped .mat files).
    if ~isfolder(userRoot)
        mkdir(userRoot);
    end

    % Always-present writable scratch dirs.
    scratchDirs = {'cache', 'results'};
    for k = 1:numel(scratchDirs)
        target = fullfile(userRoot, scratchDirs{k});
        if ~isfolder(target)
            mkdir(target);
        end
    end

    % Seed read-mostly content from the install dir on first launch only.
    % Subsequent launches preserve any edits the user made.
    seedItems = {'config', 'docs', 'README.md', 'CHECKPOINT.md'};
    seededAny = false;
    for k = 1:numel(seedItems)
        src = fullfile(installRoot, seedItems{k});
        dst = fullfile(userRoot,    seedItems{k});
        if isfile(dst) || isfolder(dst)
            continue;  % already seeded; preserve user edits
        end
        if ~isfile(src) && ~isfolder(src)
            continue;  % installer didn't ship this; skip silently
        end
        try
            copyfile(src, dst, 'f');
            seededAny = true;
        catch ME
            warning('trackbench:mainMenu:seedFailed', ...
                'Failed to copy %s -> %s: %s', src, dst, ME.message);
        end
    end

    % Drop a marker so devs (and future-us) can tell at a glance this is
    % a seeded user data dir, not a cloned source repo.
    markerPath = fullfile(userRoot, '.rainyday_userdata');
    if seededAny || ~isfile(markerPath)
        try
            fid = fopen(markerPath, 'w');
            if fid ~= -1
                fprintf(fid, ['Rainy Day Tracker - per-user data dir\n' ...
                              'Seeded: %s\n' ...
                              'From  : %s\n' ...
                              'Delete this folder and relaunch the app\n' ...
                              'to re-seed from the install dir.\n'], ...
                    char(datetime("now")), installRoot);
                fclose(fid);
            end
        catch
            % Marker is decorative; never block startup on this.
        end
    end
end
