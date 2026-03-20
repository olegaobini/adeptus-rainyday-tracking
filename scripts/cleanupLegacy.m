function cleanupLegacy()
%cleanupLegacy  Remove legacy/catalog files that are not used by the modular path.
%
%  This script deletes files and folders that were part of the V2 catalog
%  system or one-off patch scripts. The modular path (loadRunFile) does not
%  depend on any of them.
%
%  SAFE TO RUN: Only touches files listed below. Does NOT touch:
%    - config/runs/, config/sensors/<TYPE>/, config/targets/, config/terrain/,
%      config/trackers/ (modular configs)
%    - src/+trackbench/+config/loadRunFile.m (modular loader)
%    - src/+trackbench/+scenario/addTargetFromDef.m (modular target builder)
%    - Any file in the active modular dependency chain
%
%  USAGE
%    cd to adeptus-rainyday-tracking root, then:
%    addpath('scripts');
%    cleanupLegacy

root = fileparts(fileparts(mfilename('fullpath')));
fprintf('\n[CLEANUP] Root: %s\n\n', root);

deleted = 0;
skipped = 0;

%% ====================================================================
%  1. LEGACY SCRIPTS (patch/debug/catalog-only)
%% ====================================================================
legacyScripts = {
    'scripts/addDemoScenarios.m'
    'scripts/applyTerrainFix.m'
    'scripts/convertCatalogToModular.m'
    'scripts/debugFireControl.m'
    'scripts/fixCrossingAltitude.m'
    'scripts/fixRunTracker.m'
    'scripts/fixShowcaseTrackers.m'
    'scripts/fixTerrainCoverage.m'
    'scripts/fixTerrainVisuals.m'
    'scripts/patchLayeredDefense.m'
    'scripts/patchRunTracker.m'
    'scripts/runSingleExperiment.m'
    'scripts/quickBatch.m'
    'scripts/runTrackingWithWeather.m'
    'scripts/updateDemoScenarios.m'
    'scripts/runAllScenarios.m'
};

fprintf('[CLEANUP] Removing legacy scripts...\n');
for i = 1:numel(legacyScripts)
    [deleted, skipped] = tryDelete(root, legacyScripts{i}, deleted, skipped);
end

%% ====================================================================
%  2. LEGACY SRC FILES (catalog/compat shims)
%% ====================================================================
legacySrc = {
    'src/+trackbench/runScenario.m'
    'src/+trackbench/+config/loadConfig.m'
    'src/+trackbench/+loader/loadConfig.m'
    'src/+trackbench/+results/ResultsSchema.m'
    'src/+trackbench/+batch/runAllScenarios.m'
    'src/+trackbench/+scenario/createScenario.m'
    'src/+trackbench/+scenario/loadScenario.m'
    'src/+trackbench/+scenario/loadScenarioCatalog.m'
    'src/+trackbench/+sensors/loadSensors.m'
    'src/+trackbench/+sensors/buildCustomFusionRadarSensor.m'
    'src/+trackbench/+sensors/buildIFFSensor.m'
    'src/+trackbench/+sensors/customSensorTemplate.m'
    'src/+trackbench/+reporting/plotScenarioAndDetections.m'
    'src/+trackbench/+environment/computePropFactor.m'
};

fprintf('[CLEANUP] Removing legacy src files...\n');
for i = 1:numel(legacySrc)
    [deleted, skipped] = tryDelete(root, legacySrc{i}, deleted, skipped);
end

%% ====================================================================
%  3. REMOVE EMPTY PACKAGE FOLDERS left behind
%% ====================================================================
emptyPkgs = {
    'src/+trackbench/+loader'
    'src/+trackbench/+results'
    'src/+trackbench/+batch'
};

fprintf('[CLEANUP] Removing empty package folders...\n');
for i = 1:numel(emptyPkgs)
    d = fullfile(root, emptyPkgs{i});
    if exist(d, 'dir')
        contents = dir(d);
        contents = contents(~ismember({contents.name}, {'.', '..'}));
        if isempty(contents)
            rmdir(d);
            fprintf('  DELETED folder: %s\n', emptyPkgs{i});
            deleted = deleted + 1;
        else
            fprintf('  SKIP (not empty): %s\n', emptyPkgs{i});
            skipped = skipped + 1;
        end
    end
end

%% ====================================================================
%  4. LEGACY CONFIG FILES (catalog system, flat sensor configs)
%% ====================================================================
legacyConfig = {
    'config/default.json'
    'config/default_reRunDetections.json'
    'config/sensors/sensors.json'
    'config/sensors/sensors_dasr.json'
    'config/sensors/sensors_demo.json'
    'config/sensors/sensors_demo_dasr.json'
    'config/sensors/sensors_demo_psr.json'
    'config/sensors/sensors_fighter.json'
    'config/sensors/sensors_fire_control.json'
    'config/sensors/sensors_ir_fusion.json'
    'config/sensors/sensors_layered_defense.json'
    'config/sensors/sensors_long_range.json'
    'config/sensors/sensors_maritime.json'
    'config/sensors/sensors_phased_array.json'
    'config/sensors/sensors_approach.json'
};

fprintf('[CLEANUP] Removing legacy config files...\n');
for i = 1:numel(legacyConfig)
    [deleted, skipped] = tryDelete(root, legacyConfig{i}, deleted, skipped);
end

%% ====================================================================
%  5. LEGACY CONFIG FOLDERS
%% ====================================================================
legacyDirs = {
    'config/scenarios'
    'config/templates'
};

fprintf('[CLEANUP] Removing legacy config folders...\n');
for i = 1:numel(legacyDirs)
    d = fullfile(root, legacyDirs{i});
    if exist(d, 'dir')
        rmdir(d, 's');
        fprintf('  DELETED folder: %s\n', legacyDirs{i});
        deleted = deleted + 1;
    else
        fprintf('  SKIP (not found): %s\n', legacyDirs{i});
        skipped = skipped + 1;
    end
end

%% ====================================================================
%  6. LEGACY ROOT FILES
%% ====================================================================
legacyRoot = {
    'README'
};

fprintf('[CLEANUP] Removing legacy root files...\n');
for i = 1:numel(legacyRoot)
    [deleted, skipped] = tryDelete(root, legacyRoot{i}, deleted, skipped);
end

%% ====================================================================
%  7. LEGACY TEST FILES (reference catalog functions)
%% ====================================================================
legacyTests = {
    'tests/testLoadScenario.m'
    'tests/testLoadSensors.m'
};

fprintf('[CLEANUP] Removing legacy test files...\n');
for i = 1:numel(legacyTests)
    [deleted, skipped] = tryDelete(root, legacyTests{i}, deleted, skipped);
end

%% ====================================================================
%  SUMMARY
%% ====================================================================
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║              LEGACY CLEANUP COMPLETE                    ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Deleted : %-3d items                                    ║\n', deleted);
fprintf('║  Skipped : %-3d items (not found or not empty)          ║\n', skipped);
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  REMAINING (modular path):                              ║\n');
fprintf('║    scripts/runSingleScenario.m                          ║\n');
fprintf('║    scripts/buildModularConfig.m                         ║\n');
fprintf('║    scripts/recordDemoVideo.m                            ║\n');
fprintf('║    scripts/cleanupLegacy.m  (this script, delete when   ║\n');
fprintf('║                              done)                      ║\n');
fprintf('║    src/+trackbench/  (modular pipeline only)            ║\n');
fprintf('║    config/runs/      (run files)                        ║\n');
fprintf('║    config/sensors/<TYPE>/  (per-sensor configs)         ║\n');
fprintf('║    config/targets/<PATTERN>/  (target definitions)      ║\n');
fprintf('║    config/terrain/<TYPE>/  (terrain + environment)      ║\n');
fprintf('║    config/trackers/<TYPE>/  (tracker configs)           ║\n');
fprintf('║    tests/testBuildSensor.m  (still valid)               ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  TEST:  runSingleScenario("my_run")                     ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

end

function [deleted, skipped] = tryDelete(root, relPath, deleted, skipped)
    f = fullfile(root, relPath);
    if exist(f, 'file')
        delete(f);
        fprintf('  DELETED: %s\n', relPath);
        deleted = deleted + 1;
    else
        fprintf('  SKIP (not found): %s\n', relPath);
        skipped = skipped + 1;
    end
end
