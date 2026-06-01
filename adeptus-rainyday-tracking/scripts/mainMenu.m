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

    % == Deployment dependency manifest (read by mcc at build time) ===
    %  These toolbox entry points are reached only through runtime
    %  function handles or System-object internals, which mcc static
    %  dependency analysis can miss. Listing them forces build_executable
    %  to bundle the experimental sonar / IR / passive-ranging code into
    %  the compiled EXE. Pure compiler directive - inert at run time.
    %#function sonarSensor sonarEmitter tsSignature
    %#function irSensor
    %#function trackingGSF trackingMSCEKF initcvmscekf cvmeasmsc cvmeasmscjac
    %#function monostaticLidarSensor adsbTransponder adsbReceiver

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
                   'Position', [120 200 460 526], ...
                   'Resize', 'off');

    gl = uigridlayout(fig, [9, 1]);
    %               Title  Subtitle  spacer  btn1  btn2  btn3  btn4  flex  Exit
    gl.RowHeight = {38,    20,       12,     58,   58,   58,   58,   '1x', 32};
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

    btn4 = uibutton(gl, 'Text', '4.  Flight Data Manager', ...
        'FontSize', 14, ...
        'ButtonPushedFcn', @(~,~) launchFlightDataMgr());
    btn4.Layout.Row = 7;

    % Exit button at the bottom — closes the menu only. Open sub-windows
    % stay open (they're independent uifigures).
    btnExit = uibutton(gl, 'Text', 'Exit', ...
        'FontSize', 12, ...
        'ButtonPushedFcn', @(~,~) close(fig));
    btnExit.Layout.Row = 9;

    % ── Sub-window launchers ──────────────────────────────────────────
    % All wrapped in try/catch + uialert so the menu stays responsive
    % even if a sub-window fails to construct. projectRoot is captured
    % from the outer scope so each sub-window inherits the resolved root
    % rather than recomputing it (avoids the CTF-cache pitfall in deployed
    % mode).

    function launchPathEditor()
        try
            dk = trackbench.editor.selectDomain();
            if strlength(dk) == 0; return; end   % user closed the domain picker
            pathEditor(projectRoot, dk);
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

    function launchFlightDataMgr()
        % Calls flightDataManagerGUI when it exists; placeholder otherwise.
        % Browses NASA DASHlink FDR (.mat) files, visualizes on a globe,
        % and composes multi-flight batches for Path Editor import.
        if exist('flightDataManagerGUI', 'file') == 2
            try
                flightDataManagerGUI(projectRoot);
            catch ME
                uialert(fig, ME.message, 'Flight Data Manager Error');
            end
        else
            msg = ['Flight Data Manager is coming soon.' newline newline ...
                   'This will provide browsing, globe visualization, ' ...
                   'and multi-flight scenario import for NASA DASHlink ' ...
                   'flight data recorder (.mat) files.'];
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
%   Always ensures cache/, results/, and flight_data/ exist.
%
%   Items copied from installRoot (skipped if already present):
%     config/                — sensors, targets, terrain, trackers,
%                              weather, runs. User edits land here.
%     docs/                  — PDFs, handouts, glossary (read by
%                              validationDocsGUI).
%     README.md              — project overview.
%     CHECKPOINT.md          — current state notes.
%     flight_data/Tail_687_1 — NASA DASHlink sample flights (3 .mat
%                              files, ~7.8 MB). FDM defaults to scanning
%                              this folder; users can add more .mat
%                              files here or in sibling subfolders.
%
%   Items always created (writable scratch):
%     cache/        — detection cache (.mat files keyed by run name).
%     results/      — saved tracker results (timestamped .mat files).
%     flight_data/  — user-writable home for FDR .mat files.
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

    % Seed / refresh read-mostly content from the install dir. v3.7.x:
    % previously copied each item ONLY if absent, so a reinstall with
    % updated content never reached an already-seeded user dir (stale
    % configs/docs persisted forever). Now we stamp a signature of the
    % installed shipped content; a newer install (different signature)
    % REFRESHES the shipped defaults - copyfile merges into the existing
    % folder, overwriting same-named files (bundled scenarios, docs) and
    % adding new ones while leaving user-CREATED files untouched (it never
    % deletes). First seed still copies everything. A user's edit to a
    % SHIPPED scenario is replaced on update; user-created scenarios survive.
    seedItems  = {'config', 'docs', 'README.md', 'CHECKPOINT.md'};
    installSig = localSeedSignature(installRoot, seedItems);
    sigPath    = fullfile(userRoot, '.seed_signature');
    prevSig    = '';
    if isfile(sigPath)
        try prevSig = strtrim(fileread(sigPath)); catch; end
    end
    refreshShipped = ~isempty(installSig) && ~strcmp(installSig, prevSig);
    seededAny = false;
    for k = 1:numel(seedItems)
        src = fullfile(installRoot, seedItems{k});
        dst = fullfile(userRoot,    seedItems{k});
        if ~isfile(src) && ~isfolder(src)
            continue;  % installer didn't ship this; skip silently
        end
        if (isfile(dst) || isfolder(dst)) && ~refreshShipped
            continue;  % already current; preserve user edits
        end
        try
            copyfile(src, dst, 'f');   % first seed, or refresh-merge on update
            seededAny = true;
        catch ME
            warning('trackbench:mainMenu:seedFailed', ...
                'Failed to copy %s -> %s: %s', src, dst, ME.message);
        end
    end
    if refreshShipped || ~isfile(sigPath)
        try
            fid = fopen(sigPath, 'w');
            if fid ~= -1; fprintf(fid, '%s', installSig); fclose(fid); end
        catch
            % Signature stamp is best-effort; never block startup on it.
        end
    end

    % Seed flight data folder (v3.6.0+). Two-step:
    %   (a) Always ensure <userRoot>/flight_data/ exists — this is the
    %       user-writable home for FDR .mat files. Users add their own
    %       flights here (or in subfolders) and the Flight Data Manager
    %       defaults to scanning it.
    %   (b) On first launch only, copy the installer-bundled
    %       <installRoot>/Tail_687_1/ samples into
    %       <userRoot>/flight_data/Tail_687_1/ so the FDM has something
    %       to scan out of the box.
    flightDataDir = fullfile(userRoot, 'flight_data');
    if ~isfolder(flightDataDir)
        mkdir(flightDataDir);
    end
    sampleSrc = fullfile(installRoot, 'Tail_687_1');
    sampleDst = fullfile(flightDataDir, 'Tail_687_1');
    if isfolder(sampleSrc) && ~isfolder(sampleDst)
        try
            copyfile(sampleSrc, sampleDst, 'f');
            seededAny = true;
        catch ME
            warning('trackbench:mainMenu:flightSeedFailed', ...
                'Failed to copy %s -> %s: %s', sampleSrc, sampleDst, ME.message);
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


function sig = localSeedSignature(rootDir, items)
%localSeedSignature  Compact signature of the installed shipped content
%  (file count + total bytes + latest mtime across the items). A reinstall
%  changes it, which triggers a one-time refresh of the shipped defaults.
%  Returns '' on failure, in which case the caller falls back to the old
%  copy-if-absent behavior.
    sig = '';
    try
        nFiles = 0; totalBytes = 0; maxMtime = 0;
        for k = 1:numel(items)
            p = fullfile(rootDir, items{k});
            if isfile(p)
                d = dir(p); nFiles = nFiles + 1;
                totalBytes = totalBytes + d.bytes;
                maxMtime   = max(maxMtime, d.datenum);
            elseif isfolder(p)
                d = dir(fullfile(p, '**', '*'));
                d = d(~[d.isdir]);
                nFiles = nFiles + numel(d);
                if ~isempty(d)
                    totalBytes = totalBytes + sum([d.bytes]);
                    maxMtime   = max(maxMtime, max([d.datenum]));
                end
            end
        end
        sig = sprintf('n=%d|b=%d|t=%.6f', nFiles, totalBytes, maxMtime);
    catch
        sig = '';
    end
end
