function cleanup_showcase()
%cleanup_showcase  Update showcase run files to v3.4 format and remove dead code.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Run once after v3.4.0 changes:
%    addpath("scripts"); cleanup_showcase
%

root = fileparts(fileparts(mfilename('fullpath')));
showcaseDir = fullfile(root, 'config', 'runs', 'showcase');

fprintf('\n=== TrackBench v3.4.0 Cleanup ===\n\n');

%% -- 1. Update showcase run files to new degradation format --
fprintf('--- Updating showcase run files ---\n');

showcaseFiles = dir(fullfile(showcaseDir, '*.json'));
for i = 1:numel(showcaseFiles)
    fpath = fullfile(showcaseDir, showcaseFiles(i).name);
    
    try
        raw = jsondecode(fileread(fpath));
    catch
        fprintf('  SKIP (parse error): %s\n', showcaseFiles(i).name);
        continue;
    end
    
    if ~isfield(raw, 'degradation'); continue; end
    
    % Check if already in new format
    if isfield(raw.degradation, 'terrain_occlusion')
        % Already new format — just clean propagation_model if present
        if isfield(raw.degradation, 'propagation_model')
            raw.degradation = rmfield(raw.degradation, 'propagation_model');
            jsonStr = jsonencode(raw, 'PrettyPrint', true);
            fid = fopen(fpath, 'w'); fprintf(fid, '%s\n', jsonStr); fclose(fid);
            fprintf('  CLEANED propagation_model: %s\n', showcaseFiles(i).name);
        else
            fprintf('  OK (already new format): %s\n', showcaseFiles(i).name);
        end
        continue;
    end
    
    % Determine weather from old format
    weatherRef = 'none';
    if isfield(raw.degradation, 'enabled') && raw.degradation.enabled
        if isfield(raw.degradation, 'type')
            oldType = char(raw.degradation.type);
            switch lower(oldType)
                case 'rain';       weatherRef = 'rain/default_rain';
                case 'heavy_rain'; weatherRef = 'rain/heavy_rain';
                otherwise;         weatherRef = sprintf('rain/%s', oldType);
            end
        else
            weatherRef = 'rain/default_rain';
        end
    end
    
    % Determine terrain effects from terrain reference
    terrainRef = '';
    if isfield(raw, 'terrain'); terrainRef = char(raw.terrain); end
    isWater = contains(lower(terrainRef), 'water');
    
    % Build new degradation block
    newDeg = struct();
    newDeg.terrain_occlusion = ~isWater;
    newDeg.horizon_masking   = ~isWater;
    newDeg.ground_clutter    = ~isWater;
    newDeg.doppler_fade      = true;
    newDeg.rcs_range_filter  = false;
    newDeg.weather           = weatherRef;
    
    raw.degradation = newDeg;
    
    % Write back
    jsonStr = jsonencode(raw, 'PrettyPrint', true);
    fid = fopen(fpath, 'w');
    fprintf(fid, '%s\n', jsonStr);
    fclose(fid);
    fprintf('  UPDATED: %s (weather=%s)\n', showcaseFiles(i).name, weatherRef);
end

%% -- 2. Delete dead VCP scripts --
fprintf('\n--- Removing dead VCP scripts ---\n');
deadScripts = {
    fullfile(root, 'scripts', 'designVCPDemo.m')
    fullfile(root, 'scripts', 'plotVCP.m')
    fullfile(root, 'scripts', 'plotVCPOverlay3D.m')
};
for i = 1:numel(deadScripts)
    if isfile(deadScripts{i})
        delete(deadScripts{i});
        [~, fn, ext] = fileparts(deadScripts{i});
        fprintf('  DELETED: scripts/%s%s\n', fn, ext);
    end
end

%% -- 3. Delete dead VCP source files --
fprintf('\n--- Removing dead VCP source files ---\n');
deadSrc = {
    fullfile(root, 'src', '+trackbench', '+environment', 'computePropFactor.m')
    fullfile(root, 'src', '+trackbench', '+environment', 'computeVerticalCoverage.m')
    fullfile(root, 'src', '+trackbench', '+environment', 'applyVCPMask.m')
};
for i = 1:numel(deadSrc)
    if isfile(deadSrc{i})
        delete(deadSrc{i});
        [~, fn, ext] = fileparts(deadSrc{i});
        fprintf('  DELETED: +environment/%s%s\n', fn, ext);
    end
end

%% -- 4. Delete VCP demo run file --
fprintf('\n--- Removing VCP demo run file ---\n');
vpcRun = fullfile(root, 'config', 'runs', 'demo_vcp_propagation.json');
if isfile(vpcRun)
    delete(vpcRun);
    fprintf('  DELETED: demo_vcp_propagation.json\n');
end

%% -- 5. Delete _cleanup_backup if present --
fprintf('\n--- Cleaning up backup folder ---\n');
backupDir = fullfile(root, 'scripts', '_cleanup_backup');
if isfolder(backupDir)
    rmdir(backupDir, 's');
    fprintf('  DELETED: scripts/_cleanup_backup/\n');
end

%% -- 6. Update VCP comment in runDetections.m --
fprintf('\n--- Updating runDetections VCP comment ---\n');
rdPath = fullfile(root, 'src', '+trackbench', '+detections', 'runDetections.m');
if isfile(rdPath)
    txt = fileread(rdPath);
    oldComment = 'see plotVCP.m for visualization';
    if contains(txt, oldComment)
        txt = strrep(txt, oldComment, 'VCP files removed in v3.4.0 cleanup');
        fid = fopen(rdPath, 'w'); fprintf(fid, '%s', txt); fclose(fid);
        fprintf('  UPDATED: runDetections.m VCP comment\n');
    end
end

%% -- 7. Delete old cleanup scripts (already ran) --
fprintf('\n--- Removing spent cleanup scripts ---\n');
oldCleanup = fullfile(root, 'scripts', 'cleanup_v340.m');
if isfile(oldCleanup)
    delete(oldCleanup);
    fprintf('  DELETED: cleanup_v340.m\n');
end

%% -- Summary --
fprintf('\n=== Cleanup Complete ===\n');
fprintf('Removed: 3 VCP scripts, 3 VCP source files, 1 VCP run file\n');
fprintf('Updated: %d showcase files to v3.4 degradation format\n', numel(showcaseFiles));
fprintf('\nNext steps:\n');
fprintf('  1. clear classes; clear all\n');
fprintf('  2. addpath("scripts"); addpath("src");\n');
fprintf('  3. runTestPlan  %% verify nothing broke\n');
fprintf('  4. runSingleScenario("range_rcs_test")  %% quick smoke test\n\n');
end
