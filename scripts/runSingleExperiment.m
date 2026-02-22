% runSingleExperiment  User-facing entry point for a single experiment run.
%
% USAGE
%   1. cd to the adeptus-rainyday-tracking root directory
%   2. addpath('src')
%   3. runSingleExperiment              % uses default_reRunDetections config
%   4. runSingleExperiment("default")   % uses default config
%
% This script delegates to trackbench.runExperiment, which reads the JSON
% config, builds the scenario, generates detections, runs all enabled
% trackers, and saves results.
%
% See also: trackbench.runExperiment, trackbench.batch.runAllScenarios

function runSingleExperiment(configName)
arguments
    configName (1,1) string = "default_reRunDetections"
end

trackbench.batch.runExperiment(configName);
end
