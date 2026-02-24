% runAllScenarios  User-facing entry point for batch scenario runs.
%
% USAGE
%   1. cd to the adeptus-rainyday-tracking root directory
%   2. addpath('scripts')
%
%   % SHOWCASE MODE — each scenario uses its dedicated sensors:
%   runAllScenarios
%
%   % MY SENSORS MODE — all scenarios use YOUR sensors.json toggles:
%   runAllScenarios(true)
%
% CONFIGURE
%   config/default.json → scenarios_to_run   (toggle scenarios on/off)
%   config/default.json → trackers_to_run    (toggle trackers on/off)
%   config/sensors/sensors.json              (toggle sensors — used in MY SENSORS mode)
%
% See also: runSingleScenario, trackbench.batch.runAllScenarios

function allResults = runAllScenarios(useMySensors)
arguments
    useMySensors (1,1) logical = false
end

% Ensure package is on the path
srcDir = fullfile(fileparts(mfilename('fullpath')), '..', 'src');
addpath(genpath(srcDir));

allResults = trackbench.batch.runAllScenarios("default", useMySensors);
end
