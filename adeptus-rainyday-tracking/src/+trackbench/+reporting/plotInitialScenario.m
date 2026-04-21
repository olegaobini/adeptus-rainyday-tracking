function plotInitialScenario(dataLog, animateFlag)
% PLOTINITIALSCENARIO Visualize Truth and Detections (3D Animated)
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
        trackbench.reporting.drawSensorCoverage(ax, dataLog.SensorCoverage, true);
    end

    %% 1C. DRAW BEAM ELEVATION ENVELOPE (3D cone showing beam limits)
    if isfield(dataLog, 'SensorCoverage') && ~isempty(dataLog.SensorCoverage)
        try
            trackbench.reporting.drawBeamEnvelope(ax, dataLog.SensorCoverage);
        catch ME
            fprintf('[WARN] Beam envelope skipped: %s\n', ME.message);
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
                                addpoints(hDetect, meas(1)*s, meas(2)*s, -meas(3)*s);
                            elseif numel(meas) == 2
                                addpoints(hDetect, meas(1)*s, meas(2)*s, 0);
                            end
                        end
                    end
                elseif isstruct(scanDets) && isfield(scanDets, 'Measurement')
                     for j = 1:numel(scanDets)
                         meas = scanDets(j).Measurement(:);
                         if numel(meas) >= 3
                             addpoints(hDetect, meas(1)*s, meas(2)*s, -meas(3)*s);
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
