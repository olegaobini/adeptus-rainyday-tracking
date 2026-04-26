function verifyM4_renderTruth()
%verifyM4_renderTruth  Render 2D + control-vs-dense views of the exported
%                      m4_curved_demo target, for the §3.5 check-in.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Reads config/targets/waypoints/m4_curved_demo.json (written by
%  verifyM4_endToEnd.m) and plots:
%    Fig A — 2D bird's-eye view: control points + dense curve + linear
%            (piecewise) connection, so the smoothing is obvious.
%    Fig B — Altitude vs. time, again with controls overlaid on the dense
%            trace, to prove the altitude arch is interpolated smoothly.
%
%  Both figures are saved as PNGs alongside the 3D capture under
%  After Presentation/PROGRESS_M4_screenshots/. No sim run is required —
%  this is a pure rendering of what's already on disk, so the script
%  finishes in a second or two (vs. the minute-plus the full sim takes).

    thisFile = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));

    screenshotsDir = fullfile(fileparts(projectRoot), 'PROGRESS_M4_screenshots');
    if ~exist(screenshotsDir, 'dir'); mkdir(screenshotsDir); end

    jsonPath = fullfile(projectRoot, 'config', 'targets', 'waypoints', ...
        'm4_curved_demo.json');
    assert(isfile(jsonPath), 'Missing %s — run verifyM4_endToEnd first.', jsonPath);

    raw = jsondecode(fileread(jsonPath));
    if iscell(raw.targets)
        tgt = raw.targets{1};
    else
        tgt = raw.targets(1);
    end

    ctrl  = stackPos(tgt.control_waypoints);
    dense = stackPos(tgt.waypoints);

    % ── Fig A: bird's-eye view ───────────────────────────────────────
    fA = figure('Name', 'M4 curved demo — birds eye', ...
                'Color', 'w', 'Position', [100 100 820 640]);
    hold on; grid on; axis equal;
    plot(ctrl(:,1)/1000, ctrl(:,2)/1000, 'k--', 'LineWidth', 1.0, ...
        'DisplayName', 'Linear (controls only)');
    plot(dense(:,1)/1000, dense(:,2)/1000, 'b-', 'LineWidth', 1.8, ...
        'DisplayName', 'Catmull-Rom dense (\alpha=0.5)');
    plot(ctrl(:,1)/1000, ctrl(:,2)/1000, 'ro', 'MarkerFaceColor', 'r', ...
        'MarkerSize', 7, 'DisplayName', 'Control waypoints');
    % Radar origin
    plot(0, 0, 'pk', 'MarkerFaceColor', 'y', 'MarkerSize', 12, ...
        'DisplayName', 'Radar site (0,0)');
    xlabel('East (km)'); ylabel('North (km)');
    title(sprintf('M4.3 exported target: %s — bird''s-eye view', tgt.name), ...
        'Interpreter', 'none');
    legend('Location', 'best');
    pathA = fullfile(screenshotsDir, 'm4_curved_03_BirdsEye_ControlsVsDense.png');
    exportgraphics(fA, pathA, 'Resolution', 150);
    fprintf('saved: %s\n', pathA);

    % ── Fig B: altitude vs time ──────────────────────────────────────
    fB = figure('Name', 'M4 curved demo — altitude over time', ...
                'Color', 'w', 'Position', [120 120 820 460]);
    hold on; grid on;
    ctrlT  = stackTime(tgt.control_waypoints);
    denseT = stackTime(tgt.waypoints);
    plot(ctrlT, abs(ctrl(:,3)), 'k--', 'LineWidth', 1.0, ...
        'DisplayName', 'Linear (controls only)');
    plot(denseT, abs(dense(:,3)), 'b-', 'LineWidth', 1.8, ...
        'DisplayName', 'Catmull-Rom dense');
    plot(ctrlT, abs(ctrl(:,3)), 'ro', 'MarkerFaceColor', 'r', ...
        'MarkerSize', 7, 'DisplayName', 'Control waypoints');
    xlabel('time (s)'); ylabel('altitude (m, positive up)');
    title(sprintf('M4.3 exported target: %s — altitude profile', tgt.name), ...
        'Interpreter', 'none');
    legend('Location', 'best');
    pathB = fullfile(screenshotsDir, 'm4_curved_04_AltitudeProfile.png');
    exportgraphics(fB, pathB, 'Resolution', 150);
    fprintf('saved: %s\n', pathB);

    fprintf('\nRender done. Two static views written to:\n  %s\n', screenshotsDir);
end


function P = stackPos(wps)
    n = numel(wps);
    P = zeros(n, 3);
    for k = 1:n
        if iscell(wps); w = wps{k}; else; w = wps(k); end
        p = double(w.pos(:)');
        P(k, :) = p(1:3);
    end
end

function T = stackTime(wps)
    n = numel(wps);
    T = zeros(n, 1);
    for k = 1:n
        if iscell(wps); w = wps{k}; else; w = wps(k); end
        T(k) = double(w.time_s);
    end
end
