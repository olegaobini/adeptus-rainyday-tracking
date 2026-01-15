function plotInitialScenario(dataLog, opts)
%plotInitialScenario  Plot truth trajectories and detections from your dataLog
%
% USAGE:
%   plotInitialScenario(dataLog)
%   plotInitialScenario(dataLog, struct("Units","km"))
%   plotInitialScenario(dataLog, struct("Units","m","ShowLastScanOnly",true))
%
% Assumes (your current logging style):
%   dataLog.Time       : 1xN numeric
%   dataLog.Truth      : Ntgt x N struct array with field Position (3x1)
%   dataLog.Detections : 1xN cell, each cell is cell array of objectDetection

arguments
    dataLog
    opts.Units (1,1) string {mustBeMember(opts.Units,["km","m"])} = "km"
    opts.Title (1,1) string = "Scenario Truth and Detections"
    opts.ShowHistory (1,1) logical = true          % show all past detections
    opts.ShowLastScanOnly (1,1) logical = false    % if true, only plot final scan detections
    opts.EqualAxis (1,1) logical = true
end

figure; clf; hold on; grid on;
title(opts.Title);

% ---- unit scaling ----
if opts.Units == "km"
    s = 1/1000; xlabel("X (km)"); ylabel("Y (km)");
else
    s = 1;      xlabel("X (m)");  ylabel("Y (m)");
end

% =========================
% Plot TRUTH trajectories
% =========================
T = dataLog.Truth;

if ~isempty(T)
    [nTgts, nTimes] = size(T);

    for ti = 1:nTgts
        X = nan(1,nTimes);
        Y = nan(1,nTimes);

        for k = 1:nTimes
            if isfield(T(ti,k),'Position') && ~isempty(T(ti,k).Position)
                p = T(ti,k).Position(:);
                if numel(p) >= 2
                    X(k) = p(1);
                    Y(k) = p(2);
                end
            end
        end

        % Trajectory
        plot(X*s, Y*s, 'k--', 'LineWidth', 1.2);

        % Current truth position marker (triangle)
        lastValid = find(~isnan(X) & ~isnan(Y), 1, 'last');
        if ~isempty(lastValid)
            plot(X(lastValid)*s, Y(lastValid)*s, 'k^', ...
                'MarkerFaceColor','k', 'MarkerSize',6);
        end
    end
end

% =========================
% Plot DETECTIONS
% =========================
D = dataLog.Detections;

if isempty(D)
    legend({'Trajectory','Targets'}, 'Location','best');
    if opts.EqualAxis, axis equal; end
    return;
end

% Helper: extract [x,y] from a scan (cell array of objectDetection)
    function [x, y] = scanXY(scan)
        if isempty(scan)
            x = []; y = []; return;
        end
        scan = scan(:);
        x = nan(numel(scan),1);
        y = nan(numel(scan),1);
        for j = 1:numel(scan)
            z = scan{j}.Measurement(:);
            if numel(z) >= 2
                x(j) = z(1);
                y(j) = z(2);
            end
        end
        good = ~isnan(x) & ~isnan(y);
        x = x(good); y = y(good);
    end

% Plot either history or last scan only
if opts.ShowLastScanOnly
    [xNow, yNow] = scanXY(D{end});
    if ~isempty(xNow)
        plot(xNow*s, yNow*s, 'o', 'MarkerSize',5, 'LineWidth',1.0);
    end
else
    if opts.ShowHistory
        allX = []; allY = [];
        for scanIdx = 1:numel(D)
            [x, y] = scanXY(D{scanIdx});
            allX = [allX; x]; %#ok<AGROW>
            allY = [allY; y]; %#ok<AGROW>
        end

        if ~isempty(allX)
            hHist = scatter(allX*s, allY*s, 10, 'filled');
            try
                hHist.MarkerFaceAlpha = 0.25;
                hHist.MarkerEdgeAlpha = 0.25;
            catch
            end
        end
    end

    % Last scan on top
    [xNow, yNow] = scanXY(D{end});
    if ~isempty(xNow)
        plot(xNow*s, yNow*s, 'o', 'MarkerSize',5, 'LineWidth',1.0);
    end
end

legend({'Trajectory','Targets','Detections (history)','Detections (last scan)'}, 'Location','best');
if opts.EqualAxis, axis equal; end
end
