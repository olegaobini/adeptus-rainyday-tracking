function build_installer(flavor, opts)
%BUILD_INSTALLER  Wrap mainMenu.exe in a self-installing Windows package.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   build_installer()                 % default: 'web' (small, downloads MCR)
%   build_installer('offline')        % bundle MCR into the installer (~4.5 GB)
%   build_installer('web')            % installer fetches MCR at install time (~3 MB)
%   build_installer('both')           % build both flavors back-to-back
%   build_installer(flavor, opts)     % advanced overrides (see below)
%
%   This wraps the already-built mainMenu.exe in a Windows installer
%   package using compiler.package.installer. End users get a single
%   .exe (or zip) installer that:
%     - copies the app to Program Files,
%     - installs the MATLAB Runtime if missing (offline) or downloads
%       it during install (web),
%     - registers the app in Apps & Features so it can be uninstalled
%       cleanly,
%     - creates a Start Menu shortcut.
%
%   No MATLAB or toolbox license is needed on the target machine.
%   Just the matching MATLAB Runtime, which the installer handles.
%
%   PREREQS
%     - mainMenu.exe must already exist at the project root
%       (run scripts/build_executable.m first if it doesn't).
%     - MATLAB Compiler license on the build machine (the same one
%       that built mainMenu.exe).
%
%   FLAVORS
%     'web'     ~3 MB installer. Needs internet during INSTALL only.
%               Best for distribution via GitHub Releases.
%     'offline' ~4.5 GB installer. Fully self-contained. Best for
%               air-gapped or demo machines that may not have
%               internet at install time.
%     'both'    Builds web first, then offline. Total time can be
%               20+ minutes because offline embeds the entire MCR.
%
%   OPTS (name-value)
%     AppName        char  Display name           default 'Rainy Day Tracker'
%     Version        char  4-part version         default '3.5.0.0'
%     AuthorCompany  char  Vendor                 default 'Team Adeptus, UW'
%     AuthorName     char  Lead dev / contact     default 'Team Adeptus'
%     Summary        char  One-line description   default see below
%     Description    char  Long description       default see below
%     OutputDir      char  Where to write the     default <project>/installer/
%                          installer files
%
%   OUTPUT
%     <projectRoot>/installer/web/RainyDayTrackerInstaller_web.exe
%     <projectRoot>/installer/offline/RainyDayTrackerInstaller_mcr.exe
%
%   See also: build_executable, compiler.package.installer

    arguments
        flavor (1,:) char {mustBeMember(flavor, {'web', 'offline', 'both'})} = 'web'
        opts.AppName       (1,:) char = 'Rainy Day Tracker'
        opts.Version       (1,:) char = '3.5.1.0'
        opts.AuthorCompany (1,:) char = 'Team Adeptus, University of Washington'
        opts.AuthorName    (1,:) char = 'Team Adeptus'
        opts.Summary       (1,:) char = ...
            'Modular MATLAB radar tracking simulator (Boeing capstone, UW).'
        opts.Description   (1,:) char = [ ...
            'Rainy Day is a modular MATLAB framework for evaluating radar ' ...
            'target tracking performance under degraded weather conditions. ' ...
            'Compose custom scenarios with sensors, targets, terrain, ' ...
            'trackers, and weather via JSON configs - no MATLAB code ' ...
            'required. Boeing-sponsored senior capstone, University of ' ...
            'Washington.']
        opts.OutputDir     (1,:) char = ''
    end

    %% ----- Locate project root (same logic as build_executable) -------
    thisDir = fileparts(mfilename('fullpath'));
    if exist(fullfile(thisDir, 'adeptus-rainyday-tracking', 'config'), 'dir')
        projectRoot = fullfile(thisDir, 'adeptus-rainyday-tracking');
    elseif exist(fullfile(thisDir, 'config'), 'dir')
        projectRoot = thisDir;
    elseif exist(fullfile(fileparts(thisDir), 'config'), 'dir')
        projectRoot = fileparts(thisDir);
    else
        error('build_installer:noProjectRoot', ...
            'Could not locate the project root containing the "config" folder.');
    end
    fprintf('Project root: %s\n', projectRoot);

    %% ----- Locate the EXE to wrap ------------------------------------
    exeRoot       = fullfile(projectRoot, 'mainMenu.exe');
    exeTrackbench = fullfile(projectRoot, 'trackbench', 'mainMenu.exe');
    if isfile(exeTrackbench)
        appExe = exeTrackbench;
    elseif isfile(exeRoot)
        appExe = exeRoot;
        warning('build_installer:noTrackbenchExe', [ ...
            'Using project-root mainMenu.exe (%s) but the matching\n' ...
            'buildresult.json is in trackbench/ and may be missing.\n' ...
            'If the build fails, run >> build_executable to regenerate.'], ...
            exeRoot);
    else
        error('build_installer:noExe', [ ...
            'mainMenu.exe not found at %s or %s.\n\n' ...
            'Run build_executable first:\n    >> build_executable'], ...
            exeRoot, exeTrackbench);
    end
    fprintf('Wrapping app exe: %s\n', appExe);

    % Locate the matching buildresult.json (preferred since R2025a) or
    % fall back to requiredMCRProducts.txt for older compiler builds.
    exeDir = fileparts(appExe);
    buildJson = fullfile(exeDir, 'buildresult.json');
    legacyTxt = fullfile(exeDir, 'requiredMCRProducts.txt');
    if isfile(buildJson)
        productsFile = buildJson;
        fprintf('Products file: %s\n', productsFile);
    elseif isfile(legacyTxt)
        productsFile = legacyTxt;
        fprintf('Products file (legacy): %s\n', productsFile);
        warning('build_installer:legacyProductsFile', [ ...
            'Using requiredMCRProducts.txt (legacy). Re-run\n' ...
            'build_executable on R2025a+ to generate buildresult.json.']);
    else
        error('build_installer:noProductsFile', [ ...
            'Neither buildresult.json nor requiredMCRProducts.txt found in %s.\n\n' ...
            'Re-run >> build_executable to regenerate the build artifacts.'], ...
            exeDir);
    end

    %% ----- Locate runtime resources to ship next to the .exe --------
    % build_executable.m bundles config/ INTO the CTF (compressed component
    % archive embedded in the .exe). That makes them readable by code that
    % calls jsondecode/fileread on a path resolved via ctfroot, but it
    % does NOT put them on disk next to the installed .exe. End users
    % expect to be able to:
    %   - Edit run files / sensor configs / weather configs by hand
    %   - Open docs/Rainy Day.pdf from the file explorer or from the
    %     validation/docs window (which calls `winopen` on a relative
    %     path — ctfroot doesn't help there).
    % To make both work, we ship config/ + docs/ + the small markdown
    % docs as AdditionalFiles so they land in the install dir alongside
    % mainMenu.exe.
    additionalFiles = {};
    candidatePaths = { ...
        fullfile(projectRoot, 'config'), ...
        fullfile(projectRoot, 'docs'), ...
        fullfile(projectRoot, 'README.md'), ...
        fullfile(projectRoot, 'CHECKPOINT.md') };
    for k = 1:numel(candidatePaths)
        if isfile(candidatePaths{k}) || isfolder(candidatePaths{k})
            additionalFiles{end+1} = candidatePaths{k}; %#ok<AGROW>
        end
    end
    fprintf('AdditionalFiles to ship alongside the EXE:\n');
    for k = 1:numel(additionalFiles)
        fprintf('  - %s\n', additionalFiles{k});
    end

    %% ----- Output dir -------------------------------------------------
    if isempty(opts.OutputDir)
        opts.OutputDir = fullfile(projectRoot, 'installer');
    end
    if ~exist(opts.OutputDir, 'dir'); mkdir(opts.OutputDir); end
    fprintf('Installer output: %s\n', opts.OutputDir);

    %% ----- Optional splash / icon -----------------------------------
    splashPath = '';
    iconPath   = '';
    assetsDir  = fullfile(projectRoot, 'docs', 'installer_assets');
    if isfile(fullfile(assetsDir, 'splash.png'))
        splashPath = fullfile(assetsDir, 'splash.png');
        fprintf('Using custom splash: %s\n', splashPath);
    end
    if isfile(fullfile(assetsDir, 'app.ico'))
        iconPath = fullfile(assetsDir, 'app.ico');
        fprintf('Using custom icon: %s\n', iconPath);
    end

    %% ----- Dispatch -------------------------------------------------
    if any(strcmp(flavor, {'offline', 'both'}))
        ensureRuntimeDownloaded();
    end

    switch flavor
        case 'web'
            buildOne('web', appExe, productsFile, additionalFiles, opts, splashPath, iconPath);
        case 'offline'
            buildOne('offline', appExe, productsFile, additionalFiles, opts, splashPath, iconPath);
        case 'both'
            buildOne('web', appExe, productsFile, additionalFiles, opts, splashPath, iconPath);
            fprintf('\n[both] Web flavor done. Starting offline flavor...\n\n');
            buildOne('offline', appExe, productsFile, additionalFiles, opts, splashPath, iconPath);
    end

    fprintf('\n=== build_installer: ALL DONE ===\n');
end


function ensureRuntimeDownloaded()
%ensureRuntimeDownloaded  Make sure the MCR install image is cached locally.
    fprintf('\nChecking MATLAB Runtime cache (required for offline installer)...\n');
    fprintf('First run downloads ~2 GB from MathWorks. Subsequent runs are instant.\n');
    t0 = tic;
    try
        compiler.runtime.download();
    catch ME
        error('build_installer:runtimeDownloadFailed', [ ...
            'compiler.runtime.download failed: %s\n\n' ...
            'Check your internet connection and try again. If the\n' ...
            'machine is air-gapped, you can pre-download the runtime\n' ...
            'on a connected machine and copy the MathWorks Runtime\n' ...
            'cache folder over.'], ME.message);
    end
    fprintf('Runtime ready (%.1f s).\n', toc(t0));
end


function buildOne(flavor, appExe, productsFile, additionalFiles, opts, splashPath, iconPath)
%buildOne  Build one installer flavor.
    fprintf('\n--- Building %s installer ---\n', upper(flavor));

    flavorDir = fullfile(opts.OutputDir, flavor);
    if ~exist(flavorDir, 'dir'); mkdir(flavorDir); end

    appNameNoSpaces = strrep(opts.AppName, ' ', '');
    if strcmp(flavor, 'web')
        installerName = sprintf('%sInstaller_web', appNameNoSpaces);
        runtimeMode   = 'web';
    else
        installerName = sprintf('%sInstaller_mcr', appNameNoSpaces);
        runtimeMode   = 'installer';
    end
    fprintf('Installer name: %s.exe\n', installerName);
    fprintf('Runtime delivery: %s\n', runtimeMode);

    nv = { ...
        'InstallerName',    installerName, ...
        'AuthorCompany',    opts.AuthorCompany, ...
        'AuthorName',       opts.AuthorName, ...
        'Description',      opts.Description, ...
        'Summary',          opts.Summary, ...
        'ApplicationName',  opts.AppName, ...
        'Version',          opts.Version, ...
        'Shortcut',         appExe, ...
        'RuntimeDelivery',  runtimeMode, ...
        'AdditionalFiles',  additionalFiles, ...
        'OutputDir',        flavorDir };
    if ~isempty(splashPath)
        nv = [nv, {'InstallerSplash', splashPath}];
    end
    if ~isempty(iconPath)
        nv = [nv, {'InstallerIcon', iconPath}];
    end

    fprintf('Calling compiler.package.installer...\n');
    if strcmp(flavor, 'offline')
        fprintf('(Offline flavor takes 5-15 minutes; please be patient.)\n');
    else
        fprintf('(Web flavor takes about 30 seconds.)\n');
    end
    t0 = tic;
    compiler.package.installer(appExe, productsFile, nv{:});
    fprintf('Done in %.1f s.\n', toc(t0));

    files = dir(fullfile(flavorDir, '*.exe'));
    if isempty(files)
        warning('build_installer:noOutput', ...
            'compiler.package.installer reported success but no .exe was found in %s', ...
            flavorDir);
    else
        for k = 1:numel(files)
            sizeMB = files(k).bytes / 1e6;
            fprintf('  -> %s  (%.1f MB)\n', ...
                fullfile(files(k).folder, files(k).name), sizeMB);
        end
    end
end
