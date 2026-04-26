%% runAllPresentationPrep.m — Run everything needed for Boeing presentation
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
% Run with: addpath("scripts"); runAllPresentationPrep
% Leave MATLAB open and let it finish — saves results to docs/presentation_data.mat

clear classes; clear all; clc; close all;

root = fileparts(mfilename('fullpath'));
if isempty(root); root = pwd; end
addpath(fullfile(root, "scripts"));
addpath(genpath(fullfile(root, "src")));

resultsLog = {};
logFile = fullfile(root, 'docs', 'presentation_prep_log.txt');
fid = fopen(logFile, 'w');

fprintf(fid, '=== PRESENTATION PREP LOG ===\n');
fprintf(fid, 'Started: %s\n\n', char(datetime('now')));

%% 1. Auto-Tune GNN for heavy rain
fprintf('\n========== STEP 1: Auto-Tuner ==========\n');
fprintf(fid, '=== STEP 1: Auto-Tuner (demo_multi_5_heavy_rain, GNN) ===\n');
try
    tunerResults = autoTuneTracker("demo_multi_5_heavy_rain", "GNN");
    fprintf(fid, 'Auto-tuner COMPLETED.\n');
    fprintf(fid, 'Best score: %.4f\n', tunerResults.bestScore);
    if isfield(tunerResults, 'bestParams')
        fprintf(fid, 'Best params: %s\n', jsonencode(tunerResults.bestParams));
    end
    fprintf(fid, '\n');
catch ME
    fprintf(fid, 'Auto-tuner FAILED: %s\n\n', ME.message);
end

%% 2. Run demo progression — capture results
demoRuns = {
    "demo_multi_1_water",       "Baseline (water, clear)";
    "demo_multi_8_icing",       "Icing (mountain, antenna ice)";
    "demo_multi_5_heavy_rain",  "Heavy Rain (mountain, 40mm/hr)";
};

for i = 1:size(demoRuns, 1)
    runName = demoRuns{i, 1};
    desc = demoRuns{i, 2};
    fprintf('\n========== STEP %d: %s ==========\n', i+1, desc);
    fprintf(fid, '=== STEP %d: %s (%s) ===\n', i+1, desc, runName);
    try
        runSingleScenario(runName);
        fprintf(fid, 'Run COMPLETED.\n\n');
    catch ME
        fprintf(fid, 'Run FAILED: %s\n\n', ME.message);
    end
end

%% 3. Run tuned version (if auto-tuner succeeded)
fprintf('\n========== STEP 5: Heavy Rain TUNED ==========\n');
fprintf(fid, '=== STEP 5: Heavy Rain with auto-tuned GNN ===\n');
tunedCfg = fullfile(root, 'config', 'trackers', 'GNN', 'autotuned_GNN_demo_multi_5_heavy_rain.json');
if isfile(tunedCfg)
    try
        runSingleScenario("demo_multi_5_heavy_rain_tuned");
        fprintf(fid, 'Tuned run COMPLETED.\n\n');
    catch ME
        fprintf(fid, 'Tuned run FAILED: %s\n\n', ME.message);
    end
else
    fprintf(fid, 'Tuned config not found — skipping.\n\n');
end

%% 4. Run RCS test
fprintf('\n========== STEP 6: RCS Test ==========\n');
fprintf(fid, '=== STEP 6: RCS Test (range_rcs_test) ===\n');
try
    runSingleScenario("range_rcs_test");
    fprintf(fid, 'RCS test COMPLETED.\n\n');
catch ME
    fprintf(fid, 'RCS test FAILED: %s\n\n', ME.message);
end

%% Done
fprintf(fid, '\n=== ALL DONE ===\n');
fprintf(fid, 'Finished: %s\n', char(datetime('now')));
fclose(fid);

fprintf('\n\n======================================\n');
fprintf('  PRESENTATION PREP COMPLETE\n');
fprintf('  Log saved to: %s\n', logFile);
fprintf('======================================\n');
