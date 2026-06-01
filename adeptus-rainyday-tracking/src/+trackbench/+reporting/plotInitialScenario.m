function plotInitialScenario(dataLog, animateFlag)
% PLOTINITIALSCENARIO Visualize Truth and Detections (3D Animated)
%
%   Author:  Daniel Trofimchik (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
% NED CONVENTION: In the scenario, Z-negative = altitude above ground.
% For display, we NEGATE Z so altitude plots UPWARD (intuitive).
% The Z-axis label reads "Altitude (km)" instead of "Z (km)".
%
% Colors: Truth tracks are distinct (Blue, Red, etc.). Detections are Green.
% Beam envelope cones show sensor elevation coverage (green=upper, red=lower).
    
    if nargin < 2
        animateFlag = true;
    end
    
    % 1. SETUP FIGURE
    figure('Name', 'Scenario Truth and Detections (3D)', 'Color', 'k', ...
           'NumberTitle', 'off');
    
    ax = axes('Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
              'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.3);
    
    hold(ax, 'on');
    grid(ax, 'on');
    view(3);
    
    xlabel(ax, 'X (km)');
    ylabel(ax, 'Y (km)');
    zlabel(ax, 'Altitude (km)');
    
    % Scale factor (meters to km)
    s = 1/1000; 

    %% 1B. DRAW SENSOR COVERAGE (background layer)
    if isfield(dataLog, 'SensorCoverage') && ~isempty(dataLog.SensorCoverage)
        % Ground rings only for STATIONARY sensors; moving sensors get an
        % animated cone + marker (a static ground ring would sit at the origin
        % while the platform flies away).
        covStatic = dataLog.SensorCoverage;
        try
            isMov = arrayfun(@(c) isfield(c,'isMoving') && c.isMoving, covStatic);
            covStatic = covStatic(~isMov);
        catch
        end
        if ~isempty(covStatic)
            trackbench.reporting.drawSensorCoverage(ax, covStatic, true);
        end
    end

    %% 1C. DRAW SENSOR COVERAGE VOLUMES (v3.7.5 — replaces drawBeamEnvelope cones)
    % Faithful swept-volume rendering of each sensor's actual scan envelope:
    % parametric (az, el, r) shell + cones + (sector only) side walls,
    % patched in altitude-up display coordinates. Colors mirror
    % drawSensorCoverage.m v3.6.15 conventions so each volume aligns with
    % its existing ground ring. drawBeamEnvelope.m retained as fallback.
    movingCov = struct('patch', {}, 'star', {}, 'baseV', {});
    if isfield(dataLog, 'SensorCoverage') && ~isempty(dataLog.SensorCoverage)
        try
            sf = 1/1000;  % NED meters -> display km
            for kSensor = 1:numel(dataLog.SensorCoverage)
                cov = dataLog.SensorCoverage(kSensor);
                if ~cov.isRadar && ~cov.isIR; continue; end
                % v3.7.8 — MSSR/SSR now renders its swept coverage volume
                % (same path as PSR). The volume is built from this sensor's
                % own coverageConfig, so it tracks the SSR JSON config
                % dynamically (range, FOV, tilt, az span).

                % Faithful range: computeSensorCoverageVolume caps at 120 km
                % to tame fusionRadarSensor's default [0 Inf]. Pass this
                % sensor's real finite range so the SSR volume reaches its ring.
                volOpts = struct();
                if isfinite(cov.maxRange) && cov.maxRange > 0
                    volOpts.rMaxCap = cov.maxRange;
                end
                [V, F] = trackbench.reporting.computeSensorCoverageVolume(cov, volOpts);

                % NED -> display: scale m->km, negate Z so altitude points up
                Vdisp = V * sf;
                Vdisp(:, 3) = -Vdisp(:, 3);

                % Per-type color matches drawSensorCoverage.m (FROZEN, v3.6.15)
                if cov.isMSSR
                    col = [0.9 0.5 0.1];                 % orange  — MSSR/SSR
                elseif cov.isIR
                    col = [0.9 0.2 0.8];                 % magenta — IR
                elseif ~cov.isRotator && cov.isRadar
                    col = [0.1 0.9 0.3];                 % green   — sector radar
                else
                    col = [0.2 0.6 1.0];                 % blue    — PSR / rotator
                end

                isMovingCov = isfield(cov,'isMoving') && cov.isMoving && ...
                    isfield(dataLog,'OwnshipPose') && ~isempty(dataLog.OwnshipPose);
                if isMovingCov
                    % Moving sensor: keep a handle + base vertices so the loop
                    % can fly the cone with the ownship (see OwnshipPose update).
                    hMovP = patch(ax, 'Vertices', Vdisp, 'Faces', F, ...
                        'FaceColor', col, 'FaceAlpha', 0.18, ...
                        'EdgeColor', col, 'EdgeAlpha', 0.10, ...
                        'LineWidth', 0.3, 'HandleVisibility', 'off');
                    hMovS = line(ax, nan, nan, nan, 'Marker', 'pentagram', ...
                        'MarkerSize', 16, 'MarkerFaceColor', col, ...
                        'MarkerEdgeColor', 'w', 'LineStyle', 'none', ...
                        'HandleVisibility', 'off');
                    movingCov(end+1) = struct('patch', hMovP, ...
                        'star', hMovS, 'baseV', Vdisp); %#ok<AGROW>
                else
                    patch(ax, 'Vertices', Vdisp, 'Faces', F, ...
                        'FaceColor', col, 'FaceAlpha', 0.18, ...
                        'EdgeColor', col, 'EdgeAlpha', 0.10, ...
                        'LineWidth', 0.3, ...
                        'HandleVisibility', 'off');
                end
            end
        catch ME
            fprintf('[WARN] Coverage volume skipped: %s\n', ME.message);
        end
    end

    %% 2. INITIALIZE ANIMATION OBJECTS
    T = dataLog.Truth;
    D = dataLog.Detections;
    
    hTrails = gobjects(0); 
    hMarkers = gobjects(0);
    
    if ~isempty(T)
        [nTgts, nTimes] = size(T);
        colors = lines(nTgts); 
        
        for ti = 1:nTgts
            hTrails(ti) = animatedline('Color', colors(ti,:), ...
                'LineWidth', 1.5, 'LineStyle', '--', ...
                'DisplayName', sprintf('Truth %d', ti));
            
            hMarkers(ti) = line(nan, nan, nan, 'Marker', '^', ...
                'MarkerFaceColor', colors(ti,:), 'MarkerEdgeColor', 'k', ...
                'MarkerSize', 6, 'LineStyle', 'none');
            
            hMarkers(ti).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    else
        nTimes = numel(D);
        nTgts = 0;
    end

    hDetect = animatedline('Color', [0 0.8 0], 'Marker', '.', ...
        'LineStyle', 'none', 'MarkerSize', 5, ...
        'DisplayName', 'Detections');

    %% 3. RUN ANIMATION LOOP
    if animateFlag
        fprintf('Animating 3D scenario... (Press Ctrl+C to stop)\n');
    else
        fprintf('Generating static 3D scenario plot...\n');
    end
    
    legend(ax, 'show', 'Location', 'best', 'TextColor', 'w', 'Color', [0.15 0.15 0.15]);

    for k = 1:nTimes
        
        % --- UPDATE TRUTH ---
        for ti = 1:nTgts
            if isfield(T(ti,k),'Position') && ~isempty(T(ti,k).Position)
                p = T(ti,k).Position(:);
                if numel(p) >= 3
                    xVal = p(1) * s; 
                    yVal = p(2) * s;
                    zVal = -p(3) * s;
                    
                    addpoints(hTrails(ti), xVal, yVal, zVal);
                    set(hMarkers(ti), 'XData', xVal, 'YData', yVal, 'ZData', zVal);
                end
            end
        end
        
        % --- UPDATE MOVING SENSOR COVERAGE (flies with the ownship) ---
        if ~isempty(movingCov) && isfield(dataLog,'OwnshipPose') && ...
                k <= numel(dataLog.OwnshipPose)
            op = dataLog.OwnshipPose(k).Position(:)';
            if numel(op) >= 3
                offs = [op(1)*s, op(2)*s, -op(3)*s];
                for mc = 1:numel(movingCov)
                    set(movingCov(mc).patch, 'Vertices', movingCov(mc).baseV + offs);
                    set(movingCov(mc).star, 'XData', offs(1), ...
                        'YData', offs(2), 'ZData', offs(3));
                end
            end
        end

        % --- UPDATE DETECTIONS ---
        if k <= numel(D)
            scanDets = D{k}; 
            if ~isempty(scanDets)
                if iscell(scanDets)
                    for j = 1:numel(scanDets)
                        det = scanDets{j};
                        if isprop(det, 'Measurement') || isfield(det, 'Measurement')
                            meas = det.Measurement(:);
                            if numel(meas) >= 3
                                % v3.7.5: filter HasINS=false body-frame visualization artifact
                                % (display altitude > -1m kept; root cause investigation deferred
                                % to v3.7.4; this is viz-only mitigation).
                                if meas(3) <= 1
                                    addpoints(hDetect, meas(1)*s, meas(2)*s, -meas(3)*s);
                                end
                            elseif numel(meas) == 2
                                addpoints(hDetect, meas(1)*s, meas(2)*s, 0);
                            end
                        end
                    end
                elseif isstruct(scanDets) && isfield(scanDets, 'Measurement')
                     for j = 1:numel(scanDets)
                         meas = scanDets(j).Measurement(:);
                         if numel(meas) >= 3
                             % v3.7.5: see cell-array branch above for filter rationale.
                             if meas(3) <= 1
                                 addpoints(hDetect, meas(1)*s, meas(2)*s, -meas(3)*s);
                             end
                         else
                             addpoints(hDetect, meas(1)*s, meas(2)*s, 0);
                         end
                     end
                end
            end
            
            if animateFlag && mod(k, 2) == 0
                drawnow limitrate;
            end
        end
        
        if animateFlag
            drawnow;
        end
    end
    
    %% 4. AUTO-FIT AXES + GROUND PLANE
    if nTgts > 0
        allPos = [];
        for ti = 1:nTgts
            for kk = 1:nTimes
                if isfield(T(ti,kk),'Position') && ~isempty(T(ti,kk).Position)
                    p = T(ti,kk).Position(:);
                    allPos(:,end+1) = [p(1)*s; p(2)*s; -p(3)*s]; %#ok<AGROW>
                end
            end
        end
        if ~isempty(allPos)
            pad = 3;
            xLims = [min(allPos(1,:))-pad, max(allPos(1,:))+pad];
            yLims = [min(allPos(2,:))-pad, max(allPos(2,:))+pad];
            zLims = [min(allPos(3,:))-0.5, max(allPos(3,:))+pad];
            zLims(1) = min(zLims(1), -0.2);
            xlim(ax, xLims);
            ylim(ax, yLims);
            zlim(ax, zLims);
            
            if isfield(dataLog, 'TerrainGrid') && ~isempty(dataLog.TerrainGrid)
                tg = dataLog.TerrainGrid;
                Xterr = tg.X * s;
                Yterr = tg.Y * s;
                Zterr = -tg.Z * s;
                
                xMask = Xterr(1,:) >= xLims(1)-2 & Xterr(1,:) <= xLims(2)+2;
                yMask = Yterr(:,1) >= yLims(1)-2 & Yterr(:,1) <= yLims(2)+2;
                Xsub = Xterr(yMask, xMask);
                Ysub = Yterr(yMask, xMask);
                Zsub = Zterr(yMask, xMask);
                
                if numel(Xsub) > 4
                    surf(ax, Xsub, Ysub, Zsub, ...
                        'FaceColor', 'interp', 'EdgeColor', 'none', ...
                        'FaceAlpha', 0.5, ...
                        'HandleVisibility', 'off');
                    colormap(ax, [0.15 0.12 0.08; 0.25 0.20 0.12; ...
                                  0.35 0.30 0.15; 0.45 0.40 0.20; ...
                                  0.55 0.50 0.30; 0.65 0.60 0.40]);
                    
                    maxTerrElev = max(Zsub(:));
                    if maxTerrElev > 0.05
                        text(ax, xLims(2)-2, yLims(1)+1, maxTerrElev+0.1, ...
                            sprintf('Terrain (peak %.0fm)', maxTerrElev*1000), ...
                            'Color', [0.7 0.6 0.4], 'FontSize', 8, ...
                            'FontWeight', 'bold', 'FontAngle', 'italic');
                    end
                end
            else
                gx = [xLims(1) xLims(2) xLims(2) xLims(1)];
                gy = [yLims(1) yLims(1) yLims(2) yLims(2)];
                gz = [0 0 0 0];
                fill3(ax, gx, gy, gz, [0.25 0.20 0.15], ...
                    'FaceAlpha', 0.3, 'EdgeColor', [0.4 0.35 0.3], ...
                    'EdgeAlpha', 0.5, 'LineWidth', 0.5, ...
                    'HandleVisibility', 'off');
                
                text(ax, xLims(2)-1, yLims(1)+0.5, 0.02, 'Ground', ...
                    'Color', [0.6 0.5 0.4], 'FontSize', 9, 'FontWeight', 'bold', ...
                    'FontAngle', 'italic');
            end
        end
    end

    drawnow;
    
    if animateFlag
        fprintf('3D Animation complete.\n');
    else
        fprintf('3D Static plot complete.\n');
    end
end
