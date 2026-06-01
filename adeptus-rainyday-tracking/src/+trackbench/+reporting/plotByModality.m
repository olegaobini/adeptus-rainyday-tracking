function fig = plotByModality(dataLog, modality)
%plotByModality  Sensor-modality-appropriate results view.
%   fig = trackbench.reporting.plotByModality(dataLog, modality)
%
%   Author:  Team Adeptus - UW Senior Capstone, Boeing-sponsored
%   Since:   v3.7.x (modality-view exploration)
%
%   The built-in theaterPlot is a Cartesian / altitude-up view tuned for
%   radar. It is unhelpful for the other two modalities, so this renders a
%   view matched to the scenario's PRIMARY sensor instead:
%
%     'sonar' : depth-down 3D - surface station, submerged contact, sonar
%               detections, and a representative GNN/CV track. (z = depth,
%               positive down.)
%     'ir'    : azimuth-elevation "seeker" plane - the angle-only world an
%               IR / IRST sensor actually measures (no range). Shows the
%               truth bearing track and the IR detection bearings inside the
%               sensor field of regard.
%
%   Radar / mixed scenarios keep the standard plots (this returns []).
%
%   See also: plotInitialScenario, runTracker, runSingleScenario

    if nargin < 2 || isempty(modality); modality = 'sonar'; end
    switch lower(string(modality))
        case "sonar"; fig = plotSonarDepth(dataLog);
        case "ir";    fig = plotIRSeeker(dataLog);
        otherwise
            fig = [];
            fprintf('[plotByModality] "%s" uses the standard radar plots.\n', char(modality));
    end
end


% ======================================================================
function fig = plotSonarDepth(dataLog)
% Depth-down 3D: surface station at origin, submerged contact, detections,
% and a representative track from an internal GNN/CV tracker.
    [truth, det, trk] = gatherTruthDetTrack(dataLog);

    fig = figure('Name','Sonar - depth view','Color','k','NumberTitle','off');
    ax = axes(fig,'Color',[0.04 0.07 0.12],'XColor','w','YColor','w', ...
        'ZColor','w','GridColor',[0.3 0.4 0.5]);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on'); view(ax,[-37 22]);
    set(ax,'ZDir','reverse');   % depth increases downward

    allp = [truth; det; trk];
    allp(any(isnan(allp),2),:) = [];
    if isempty(allp); allp = [0 0 0]; end
    xl = [min(allp(:,1))-200, max(allp(:,1))+200];
    yl = [min(allp(:,2))-200, max(allp(:,2))+200];
    [sx,sy] = meshgrid(linspace(xl(1),xl(2),2), linspace(yl(1),yl(2),2));
    surf(ax, sx, sy, zeros(size(sx)), 'FaceColor',[0.1 0.35 0.55], ...
        'FaceAlpha',0.15, 'EdgeColor','none');   % sea surface at z=0

    h = gobjects(0); lbl = {};
    h(end+1) = plot3(ax,0,0,0,'p','MarkerSize',20,'MarkerFaceColor',[0.2 0.85 1], ...
        'MarkerEdgeColor','w','LineWidth',1); lbl{end+1} = 'Sonar station (surface)';
    if ~isempty(truth)
        h(end+1) = plot3(ax,truth(:,1),truth(:,2),truth(:,3),'--', ...
            'Color',[1 0.85 0.2],'LineWidth',2.2); lbl{end+1} = 'Truth (contact)';
    end
    if ~isempty(det)
        h(end+1) = plot3(ax,det(:,1),det(:,2),det(:,3),'o', ...
            'Color',[0.25 1 0.45],'MarkerSize',7,'LineWidth',1.5); lbl{end+1} = 'Sonar detections';
    end
    if ~isempty(trk)
        h(end+1) = plot3(ax,trk(:,1),trk(:,2),trk(:,3),'-', ...
            'Color',[1 0.35 0.85],'LineWidth',2.2); lbl{end+1} = 'Track';
    end
    xlabel(ax,'North (m)','Color','w'); ylabel(ax,'East (m)','Color','w');
    zlabel(ax,'Depth (m, +down)','Color','w');
    title(ax,'Sonar view: submerged contact (depth vs. range)','Color','w','Interpreter','none');
    legend(ax,h,lbl,'TextColor','w','Color',[0.1 0.12 0.18], ...
        'EdgeColor',[0.3 0.4 0.5],'Location','northeastoutside');
end


% ======================================================================
function fig = plotIRSeeker(dataLog)
% Azimuth-elevation "seeker" plane - the angle-only view an IR sensor sees.
% IR is bearings-only (no range); detections are [az; el] in degrees.
    [sPos, ~, ~, elLim, yaw] = irSensorGeom(dataLog);
    nScan = numel(dataLog.Time);
    if isempty(elLim); elLim = [-45 45]; end
    elLo = min(elLim); elHi = max(elLim);
    elPadLo = elLo - 15; elPadHi = elHi + 15;   % view band = scan band + margin

    % Truth bearings in the sensor frame (world bearing minus mounting yaw).
    tAz = []; tEl = [];
    if isfield(dataLog,'Truth') && ~isempty(dataLog.Truth)
        for s = 1:nScan
            col = dataLog.Truth(:,s);
            for t = 1:numel(col)
                p  = col(t).Position(:)' - sPos;
                rg = hypot(p(1), p(2));
                tAz(end+1) = wrap180(atan2d(p(2), p(1)) - yaw);   %#ok<AGROW>
                tEl(end+1) = atan2d(-p(3), rg);                   %#ok<AGROW>
            end
        end
    end

    % IR detection bearings [az; el] in degrees. Drop non-physical / far
    % out-of-band false alarms (e.g. el=360) so the in-FOV picture survives.
    dAz = []; dEl = []; dCol = [];
    for s = 1:nScan
        D = dataLog.Detections{s};
        for i = 1:numel(D)
            m = D{i}.Measurement;
            if numel(m) ~= 2; continue; end
            e = m(2);
            if e < elPadLo || e > elPadHi; continue; end
            dAz(end+1) = wrap180(m(1)); dEl(end+1) = e; dCol(end+1) = dataLog.Time(s); %#ok<AGROW>
        end
    end

    fig = figure('Name','IR - seeker (az/el) view','Color','k','NumberTitle','off');
    ax = axes(fig,'Color',[0.05 0.05 0.07],'XColor','w','YColor','w','GridColor',[0.35 0.35 0.4]);
    hold(ax,'on'); grid(ax,'on'); box(ax,'on');

    azAll = [tAz dAz]; if isempty(azAll); azAll = [-1 1]; end
    axLo = min(azAll) - 10; axHi = max(azAll) + 10;

    % Elevation scan band the sensor actually sweeps.
    patch(ax,[axLo axHi axHi axLo],[elLo elLo elHi elHi],[0.2 0.3 0.45], ...
        'FaceAlpha',0.12,'EdgeColor',[0.4 0.5 0.7],'LineStyle','--');

    h = gobjects(0); lbl = {};
    if ~isempty(dAz)
        h(end+1) = scatter(ax,dAz,dEl,26,dCol,'o','LineWidth',1.2);
        lbl{end+1} = 'IR detections (time-colored)';
    end
    if ~isempty(tAz)
        h(end+1) = plot(ax,tAz,tEl,'-','Color',[1 0.85 0.2],'LineWidth',2.4);
        lbl{end+1} = 'Truth bearing';
    end
    plot(ax,0,0,'+','Color',[0.6 0.7 0.9],'MarkerSize',12,'LineWidth',1);  % boresight
    if ~isempty(dCol)
        c = colorbar(ax); c.Color='w'; c.Label.String='time (s)'; c.Label.Color='w'; colormap(ax,parula);
    end
    xlabel(ax,'Azimuth (deg, sensor frame)','Color','w');
    ylabel(ax,'Elevation (deg)','Color','w');
    title(ax,'IR view: target bearing + detections in the seeker scan band','Color','w','Interpreter','none');
    xlim(ax,[axLo axHi]); ylim(ax,[elPadLo elPadHi]);
    if ~isempty(h); legend(ax,h,lbl,'TextColor','w','Color',[0.12 0.12 0.16], ...
        'EdgeColor',[0.35 0.35 0.4],'Location','northeastoutside'); end
end


% ======================================================================
function [truth, det, trk] = gatherTruthDetTrack(dataLog)
% Truth track (per target, NaN-separated), Cartesian detection positions,
% and a representative track from an internal GNN/CV tracker.
    nScan = numel(dataLog.Time);
    truth = [];
    if isfield(dataLog,'Truth') && ~isempty(dataLog.Truth)
        for t = 1:size(dataLog.Truth,1)
            P = zeros(nScan,3);
            for s = 1:nScan; P(s,:) = dataLog.Truth(t,s).Position(:)'; end
            if ~isempty(truth); truth(end+1,:) = [NaN NaN NaN]; end %#ok<AGROW>
            truth = [truth; P]; %#ok<AGROW>
        end
    end

    det = [];
    for s = 1:nScan
        D = dataLog.Detections{s};
        for i = 1:numel(D)
            m = D{i}.Measurement;
            if numel(m) >= 3; det(end+1,:) = m(1:3)'; end %#ok<AGROW>
        end
    end

    trk = [];
    try
        tracker = trackerGNN('FilterInitializationFcn',@initekfimm, ...
            'AssignmentThreshold',2000,'ConfirmationThreshold',[2 3],'DeletionThreshold',[5 5]);
        ids = []; hist = {};   % accumulate each track's path separately
        for s = 1:nScan
            D = dataLog.Detections{s}; if isempty(D); continue; end
            Dc = {};
            for i = 1:numel(D)
                if numel(D{i}.Measurement) >= 3
                    D{i}.Time = dataLog.Time(s); Dc{end+1} = D{i}; %#ok<AGROW>
                end
            end
            if isempty(Dc); continue; end
            tr = tracker(Dc(:)', dataLog.Time(s));
            for k = 1:numel(tr)
                st = tr(k).State; p = [st(1) st(3) st(5)];
                idx = find(ids == tr(k).TrackID, 1);
                if isempty(idx)
                    ids(end+1) = tr(k).TrackID; hist{end+1} = p; %#ok<AGROW>
                else
                    hist{idx}(end+1,:) = p; %#ok<AGROW>
                end
            end
        end
        % One NaN-separated polyline per track, so distinct contacts are not
        % joined into a zigzag (mirrors the per-target truth handling above).
        for j = 1:numel(hist)
            if ~isempty(trk); trk(end+1,:) = [NaN NaN NaN]; end %#ok<AGROW>
            trk = [trk; hist{j}]; %#ok<AGROW>
        end
    catch
        trk = [];
    end
end


% ======================================================================
function [sPos, fov, azLim, elLim, yaw] = irSensorGeom(dataLog)
% Pull the IR sensor's world position, FOV and scan limits from coverage.
    sPos = [0 0 0]; fov = []; azLim = []; elLim = []; yaw = 0;
    if ~isfield(dataLog,'SensorCoverage') || isempty(dataLog.SensorCoverage); return; end
    cov = dataLog.SensorCoverage;
    for i = 1:numel(cov)
        isIR = isfield(cov(i),'isIR') && ~isempty(cov(i).isIR) && cov(i).isIR;
        if ~isIR; continue; end
        if isfield(cov(i),'position');  sPos = cov(i).position(:)'; end
        if isfield(cov(i),'fov');       fov  = cov(i).fov; end
        if isfield(cov(i),'mountingYaw'); yaw = cov(i).mountingYaw; end
        if isfield(cov(i),'azLimits');  azLim = cov(i).azLimits; end
        if isfield(cov(i),'scanElLimits'); elLim = cov(i).scanElLimits;
        elseif isfield(cov(i),'elLimits'); elLim = cov(i).elLimits; end
        return;
    end
end


function a = wrap180(a)
    a = mod(a + 180, 360) - 180;
end
