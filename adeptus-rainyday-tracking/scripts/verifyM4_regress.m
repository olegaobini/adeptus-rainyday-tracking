function verifyM4_regress()
%verifyM4_regress  Post-M5 sanity check: prove the runDetections truth-shape
%                  fix didn't regress single-target behavior.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Runs runSingleScenario("m4_curved_demo") under -batch and saves the
%  3D scenario plot as
%    ../PROGRESS_M5_screenshots/m4_regress_01_Scenario_Truth_and_Detections_3D.png
%  so Michael can diff it against the pre-M5 artifact at
%    ../PROGRESS_M4_screenshots/m4_curved_02_Scenario_Truth_and_Detections_3D.png
%
%  Writes ../PROGRESS_M5_screenshots/_REGRESS_DONE.marker on clean exit so
%  the caller can poll a filesystem signal rather than parsing logs.

    here       = fileparts(mfilename('fullpath'));
    projRoot   = fileparts(here);          % adeptus-rainyday-tracking/
    parentRoot = fileparts(projRoot);      % After Presentation/
    screenshotsDir = fullfile(parentRoot, 'PROGRESS_M5_screenshots');
    if ~exist(screenshotsDir, 'dir'); mkdir(screenshotsDir); end

    fprintf('==== verifyM4_regress ====\n');
    fprintf('Project root  : %s\n', projRoot);
    fprintf('Screenshots   : %s\n', screenshotsDir);

    % Run the M4 single-target scenario through the same simulator path
    % whose runDetections.m was edited for multi-target truth preservation.
    cd(projRoot);
    addpath(genpath(fullfile(projRoot, 'src')));
    addpath(fullfile(projRoot, 'scripts'));

    fprintf('\n---- runSingleScenario("m4_curved_demo") ----\n\n');
    runSingleScenario('m4_curved_demo');

    % Save any open figure that looks like the 3D scenario plot.
    figs = findobj(0, 'Type', 'figure');
    fprintf('\nFigures to save : %d\n', numel(figs));
    savedCount = 0;
    for i = 1:numel(figs)
        f = figs(i);
        name = get(f, 'Name');
        if isempty(name); name = sprintf('fig_%d', i); end
        safeName = regexprep(name, '[^a-zA-Z0-9_\- ]', '_');
        safeName = regexprep(safeName, '\s+', '_');
        outFile = fullfile(screenshotsDir, ...
            sprintf('m4_regress_%02d_%s.png', i, safeName));
        try
            exportgraphics(f, outFile, 'Resolution', 150);
            fprintf('  [save] %s\n', outFile);
            savedCount = savedCount + 1;
        catch ME
            warning('Could not save fig %d (%s): %s', i, name, ME.message);
        end
    end

    % Completion marker for independent exit detection.
    markerPath = fullfile(screenshotsDir, '_REGRESS_DONE.marker');
    fid = fopen(markerPath, 'w');
    fprintf(fid, 'completed %s\nfigs=%d saved=%d\n', ...
        char(datetime('now')), numel(figs), savedCount);
    fclose(fid);

    fprintf('\nREGRESS DONE. Saved %d of %d figures.\n', savedCount, numel(figs));
end
