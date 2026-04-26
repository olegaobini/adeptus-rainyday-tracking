%diagBeamLimits  Query the sensor's actual coverage config to find real beam limits.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  USAGE: clear classes; clear all; addpath("scripts"); diagBeamLimits

clc; close all;

root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(fullfile(root, "src")));

[scenario, ~, ~, ~] = trackbench.config.loadRunFile("range_rcs_test");

tower = scenario.Platforms{1};
psr = tower.Sensors{1};

fprintf('\n--- PSR Properties ---\n');
fprintf('  FieldOfView: [%.2f, %.2f]\n', psr.FieldOfView(1), psr.FieldOfView(2));
fprintf('  MechanicalElevationLimits: [%.2f, %.2f]\n', psr.MechanicalElevationLimits(1), psr.MechanicalElevationLimits(2));
fprintf('  MechanicalAzimuthLimits: [%.0f, %.0f]\n', psr.MechanicalAzimuthLimits(1), psr.MechanicalAzimuthLimits(2));

fprintf('\n--- coverageConfig() output ---\n');
cc = coverageConfig(psr);
disp(cc);

% Print all fields
fn = fieldnames(cc);
for i = 1:numel(fn)
    val = cc.(fn{i});
    if isnumeric(val) && numel(val) <= 6
        fprintf('  %s: %s\n', fn{i}, mat2str(val, 4));
    else
        fprintf('  %s: [%s]\n', fn{i}, class(val));
    end
end

% The key field is ScanLimits — this tells us exactly where the beam scans
if isfield(cc, 'ScanLimits')
    sl = cc.ScanLimits;
    fprintf('\n--- ACTUAL SCAN LIMITS ---\n');
    if size(sl, 1) >= 2
        fprintf('  Azimuth scan:   [%.1f° to %.1f°]\n', sl(1,1), sl(1,2));
        fprintf('  Elevation scan: [%.1f° to %.1f°]\n', sl(2,1), sl(2,2));
        fprintf('\n  THIS is where the beam actually scans.\n');
        fprintf('  The beam FOV (%.1f° el) is centered within these limits.\n', psr.FieldOfView(2));
        
        % Compute actual detection zone
        elCenter = mean(sl(2,:));
        halfFov = psr.FieldOfView(2) / 2;
        fprintf('\n  Beam center estimate: %.1f°\n', elCenter);
        fprintf('  Detection zone: [%.1f° to %.1f°]\n', sl(2,1) - halfFov, sl(2,2) + halfFov);
    else
        fprintf('  ScanLimits has %d rows (expected 2)\n', size(sl,1));
        disp(sl);
    end
end
