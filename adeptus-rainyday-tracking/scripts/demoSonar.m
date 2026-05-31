function demoSonar()
%demoSonar  Standalone demo proving the SONAR modality works end-to-end:
%           build -> emit -> propagate -> detect -> track -> plot. Completes
%           the trio alongside radar (fusionRadarSensor) and IR (irSensor).
%
%   Author:  Team Adeptus - exploratory (v3.7.x sonar branch)
%   Project: Rainy Day Tracker - UW Senior Capstone, Boeing-sponsored
%
%   WHY THIS IS SEPARATE FROM runDetections
%     Sonar uses a fundamentally different path than the radar/IR pipeline:
%       * A monostatic (active) sonar needs a paired sonarEmitter (the ping).
%       * Detections come from the scenario emit -> propagate ->
%         detect(scene, signals, configs) chain, NOT the 3-arg
%         sensor(targets, ins, time) call that radar/IR use.
%       * Targets must carry an acoustic target-strength signature
%         (tsSignature), not RCS/IR - the air-tracking scenarios' aircraft
%         are acoustically silent, so sonar belongs in its own scenario.
%
%   SCENARIO
%     A station at the origin emits a loud omni ping and listens with a
%     monostatic sonarSensor. A submerged contact circles at ~1.2 km range
%     and 80 m depth carrying a target strength. The sonar detects it on
%     nearly every ping; a GNN/CV tracker (fed Cartesian-converted
%     detections) follows it.
%
%   USAGE
%     >> cd <project root>;  demoSonar

    %% --- Monostatic active sonar + its ping emitter --------------------
    son = trackbench.sensors.buildSensor(1, 'ACTIVE_SONAR');   % monostatic
    try son.HasFalseAlarms = false; catch; end                 % clean demo

    scene  = trackingScenario('UpdateRate', son.UpdateRate, 'StopTime', 60);
    own    = platform(scene);                                   % station at origin
    pinger = sonarEmitter(1, 'UpdateRate', son.UpdateRate);
    try pinger.FieldOfView = [360; 180]; catch; end            % omnidirectional ping
    pinger.SourceLevel = 215;                                   % dB - reaches ~km range
    own.Emitters = {pinger};
    own.Sensors  = {son};

    %% --- Submerged contact: circle at ~1.2 km, 80 m deep ---------------
    wp  = [ 1200 0 80; 850 850 80; 0 1200 80; -850 850 80; -1200 0 80; ...
           -850 -850 80; 0 -1200 80; 850 -850 80; 1200 0 80];
    toa = linspace(0, 60, size(wp,1))';
    tgt = platform(scene, 'Trajectory', waypointTrajectory(wp, toa));
    tgt.Signatures = {tsSignature};

    %% --- Tracker (GNN + CV) on Cartesian-converted detections ----------
    tracker = trackerGNN('FilterInitializationFcn', @initcvekf, ...
        'AssignmentThreshold', 500, 'ConfirmationThreshold', [2 3], ...
        'DeletionThreshold', [6 6]);

    %% --- Figure -------------------------------------------------------
    fig = figure('Name','Sonar Demo - monostatic active sonar', ...
                 'Color','k','NumberTitle','off');
    ax = axes(fig,'Color',[0.08 0.08 0.12],'XColor','w','YColor','w', ...
              'ZColor','w','GridColor',[0.4 0.4 0.5]);
    hold(ax,'on'); grid(ax,'on'); view(ax,3); set(ax,'ZDir','reverse');
    xlabel(ax,'North (m)'); ylabel(ax,'East (m)'); zlabel(ax,'Depth (m, +down)');
    title(ax,'Sonar (monostatic) - contact detection + track','Color','w');
    plot3(ax,0,0,0,'p','MarkerSize',16,'MarkerFaceColor',[0.2 0.8 1], ...
        'MarkerEdgeColor','w','DisplayName','Sonar station');

    truthXYZ = zeros(0,3); detXYZ = zeros(0,3); trkXYZ = zeros(0,3);
    nDet = 0; nTrkScans = 0;

    %% --- Run -----------------------------------------------------------
    while advance(scene)
        t = scene.SimulationTime;
        p = pose(tgt,'true'); truthXYZ(end+1,:) = p.Position; %#ok<AGROW>

        [emtx, cfgs] = emit(scene);
        sig  = propagate(scene, emtx);
        dets = detect(scene, sig, cfgs);

        % Convert each sonar detection (az,el,range; sensor spherical, sensor
        % at origin/identity) to a Cartesian NED position objectDetection.
        cartDets = {};
        for i = 1:numel(dets)
            m = dets{i}.Measurement;
            if numel(m) ~= 3; continue; end
            az = m(1); el = m(2); r = m(3);
            x = r*cosd(el)*cosd(az);   % North
            y = r*cosd(el)*sind(az);   % East
            z = -r*sind(el);           % Down (NED)
            detXYZ(end+1,:) = [x y z]; nDet = nDet + 1; %#ok<AGROW>
            cartDets{end+1} = objectDetection(t, [x;y;z], ...
                'MeasurementNoise', diag([60 60 60].^2), ...
                'SensorIndex', son.SensorIndex); %#ok<AGROW>
        end

        if ~isempty(cartDets)
            try
                tracks = tracker(cartDets, t);
                for k = 1:numel(tracks)
                    st = tracks(k).State;             % [x vx y vy z vz]
                    trkXYZ(end+1,:) = [st(1) st(3) st(5)]; %#ok<AGROW>
                end
                nTrkScans = nTrkScans + 1;
            catch ME
                fprintf('[demoSonar] tracker step skipped: %s\n', ME.message);
            end
        end
    end

    %% --- Plot results --------------------------------------------------
    plot3(ax, truthXYZ(:,1), truthXYZ(:,2), truthXYZ(:,3), '--', ...
        'Color',[1 0.85 0.2], 'LineWidth',1.5, 'DisplayName','Truth (contact)');
    if ~isempty(detXYZ)
        plot3(ax, detXYZ(:,1), detXYZ(:,2), detXYZ(:,3), '.', ...
            'Color',[0.2 1 0.3], 'MarkerSize',10, 'DisplayName','Sonar detections');
    end
    if ~isempty(trkXYZ)
        plot3(ax, trkXYZ(:,1), trkXYZ(:,2), trkXYZ(:,3), '-', ...
            'Color',[1 0.3 0.8], 'LineWidth',1.4, 'DisplayName','Track');
    end
    legend(ax,'show','TextColor','w','Color',[0.15 0.15 0.2]);
    axis(ax,'equal');

    fprintf('[demoSonar] %d sonar detections; %d scans produced a track update.\n', ...
        nDet, nTrkScans);
end
