function build_executable()
% BUILD_EXECUTABLE Automates the compilation of the Rainy Day Tracker.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%   Compiles scripts/mainMenu.m into trackbench/mainMenu.exe via mcc.
%   The launcher (trackbench.bat) prefers this exe when it exists and
%   falls back to a MATLAB session otherwise.
%
%   v3.5: switched from runScenarioGUI.m to mainMenu.m as the entry
%   point. mainMenu.m is the 3-button top-level launcher that opens
%   pathEditor / runSimGUI / validationDocsGUI as separate uifigures;
%   mcc's dependency analyzer follows those calls automatically and
%   bundles all transitive dependencies (the +trackbench package, the
%   +trackbench.editor sub-package, etc.).
%
%   USAGE
%       cd <projectRoot>
%       build_executable
%
%   Compile time is typically 3–7 minutes depending on toolbox count.
%   The MCR runtime download for end users is separate and not
%   produced here.

    disp('Starting compilation process...');

    % Robustly find the inner project directory containing the config folder
    thisDir = fileparts(mfilename('fullpath'));
    if exist(fullfile(thisDir, 'adeptus-rainyday-tracking', 'config'), 'dir')
        projectRoot = fullfile(thisDir, 'adeptus-rainyday-tracking');
    elseif exist(fullfile(thisDir, 'config'), 'dir')
        projectRoot = thisDir;
    elseif exist(fullfile(fileparts(thisDir), 'config'), 'dir')
        projectRoot = fileparts(thisDir);
    else
        error('Could not locate the project root containing the "config" folder.');
    end

    % Define input files and folders
    configDir  = fullfile(projectRoot, 'config');
    srcDir     = fullfile(projectRoot, 'src');
    scriptsDir = fullfile(projectRoot, 'scripts');
    outputDir  = fullfile(projectRoot, 'trackbench');

    % Create the build directory if it doesn't exist
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    % v3.5: clean up the previous-entry-point exe so the trackbench
    % folder doesn't accumulate stale builds. Only deletes the OLD
    % runScenarioGUI.exe — mcc itself overwrites buildresult.json,
    % includedSupportPackages.txt, etc.
    oldExe = fullfile(outputDir, 'runScenarioGUI.exe');
    if exist(oldExe, 'file')
        fprintf('Removing stale old entry-point exe: %s\n', oldExe);
        delete(oldExe);
    end

    % Ensure the compiler can find scripts and the +trackbench package
    addpath(scriptsDir);
    addpath(genpath(srcDir));

    % Main GUI entry point — the 3-button top-level launcher.
    % v3.5: switched from runScenarioGUI.m to mainMenu.m. mainMenu's
    % isdeployed branch sets projectRoot=pwd, so the launcher (.bat)
    % must cd into the project folder before starting the exe — which
    % it does (cd /d "%~dp0").
    mainScript = fullfile(scriptsDir, 'mainMenu.m');

    % Run the MATLAB compiler (mcc)
    %   -m : build standalone executable
    %   -a : add files/folders to the package (config JSONs, src package,
    %        scripts dir for the sibling GUI .m files mainMenu launches)
    %   -d : output directory
    %
    % NOTE: mcc dependency analysis follows function calls statically,
    % so pathEditor / runSimGUI / validationDocsGUI are picked up from
    % mainMenu.m's launchPathEditor / launchRunSim / launchValidationDocs
    % nested functions automatically. We still pass scriptsDir as -a so
    % any runtime-only access (e.g. exist('validationDocsGUI','file'))
    % succeeds inside the CTF.
    disp(['Running mcc on ', mainScript, '...']);
    disp('This may take a few minutes.')
    mcc('-m', mainScript, ...
        '-a', configDir, ...
        '-a', srcDir, ...
        '-a', scriptsDir, ...
        '-d', outputDir);

    % Copy the config folder next to the .exe so users can edit JSON files
    % without rebuilding.
    disp('Copying config folder to the output directory...');
    copyfile(configDir, fullfile(outputDir, 'config'));

    disp('Compilation complete! Check the "trackbench" folder.');
    disp('Entry point: trackbench/mainMenu.exe');
end