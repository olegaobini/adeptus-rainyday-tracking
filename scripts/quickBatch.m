%% quickBatch.m — Run all 18 scenarios, report summary
cd('C:\Users\Admin\Documents\RAINY DAY GIT COPY\adeptus-rainyday-tracking');
addpath(genpath(fullfile(pwd, 'src')));

results = trackbench.batch.runAllScenarios("default");
