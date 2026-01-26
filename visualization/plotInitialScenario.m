function plotInitialScenario(dataLog)
% PLOTINITIALSCENARIO Visualize Truth and Detections (3D Animated)
% Colors: Truth tracks are distinct (Blue, Red, etc.). Detections are Green.

    % 1. SETUP FIGURE
    figure('Name', 'Scenario Truth and Detections (3D)', 'Color', 'k', ...
           'NumberTitle', 'off');
    
    % Use 3D view
    ax = axes('Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w', ...
              'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.3);
    
    hold(ax, 'on');
    grid(ax, 'on');
    axis equal;
    view(3); % Switch camera to 3D default view
    
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)'); % New Z-axis label
    
    % Scale factor (meters to km)
    s = 1/1000; 

    %% 2. INITIALIZE ANIMATION OBJECTS
    T = dataLog.Truth;
    D = dataLog.Detections;
    
    % Initialize handles
    hTrails = gobjects(0); 
    hMarkers = gobjects(0);
    
    if ~isempty(T)
        [nTgts, nTimes] = size(T);
        colors = lines(nTgts); 
        
        for ti = 1:nTgts
            % The Trail (3D Line)
            hTrails(ti) = animatedline('Color', colors(ti,:), ...
                'LineWidth', 1.5, 'LineStyle', '--', ...
                'DisplayName', sprintf('Truth %d', ti));
            
            % The Marker (3D Triangle)
            % Note: 'line' supports X, Y, Z inputs
            hMarkers(ti) = line(nan, nan, nan, 'Marker', '^', ...
                'MarkerFaceColor', colors(ti,:), 'MarkerEdgeColor', 'k', ...
                'MarkerSize', 6, 'LineStyle', 'none');
            
            hMarkers(ti).Annotation.LegendInformation.IconDisplayStyle = 'off';
        end
    else
        nTimes = numel(D);
        nTgts = 0;
    end

    % B. Create Detection Handle (Green Dots in 3D)
    hDetect = animatedline('Color', [0 0.8 0], 'Marker', '.', ...
        'LineStyle', 'none', 'MarkerSize', 5, ...
        'DisplayName', 'Detections');

    %% 3. RUN ANIMATION LOOP
    fprintf('Animating 3D scenario... (Press Ctrl+C to stop)\n');
    
    % Setup Legend
    validHandles = [hTrails(:); hDetect];
    if ~isempty(validHandles)
        legend(validHandles, 'Location', 'best', 'TextColor', 'w', 'Color', 'none');
    end

    for k = 1:nTimes
        
        % --- UPDATE TRUTH ---
        for ti = 1:nTgts
            if isfield(T(ti,k),'Position') && ~isempty(T(ti,k).Position)
                p = T(ti,k).Position(:);
                % Check if we have 3 dimensions (X, Y, Z)
                if numel(p) >= 3
                    xVal = p(1) * s; 
                    yVal = p(2) * s;
                    zVal = p(3) * s; % Extract Altitude
                    
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
                            % Check for 3D measurement
                            if numel(meas) >= 3
                                addpoints(hDetect, meas(1)*s, meas(2)*s, meas(3)*s);
                            elseif numel(meas) == 2
                                % Fallback if sensor is 2D (assume Z=0)
                                addpoints(hDetect, meas(1)*s, meas(2)*s, 0);
                            end
                        end
                    end
                elseif isstruct(scanDets) && isfield(scanDets, 'Measurement')
                     for j = 1:numel(scanDets)
                         meas = scanDets(j).Measurement(:);
                         if numel(meas) >= 3
                             addpoints(hDetect, meas(1)*s, meas(2)*s, meas(3)*s);
                         else
                             addpoints(hDetect, meas(1)*s, meas(2)*s, 0);
                         end
                     end
                end
            end
            
            if mod(k, 2) == 0
             drawnow limitrate;
            end
        end
        
        % --- RENDER FRAME ---
        drawnow;
        % Removed pause for speed, or add pause(0.01) if too fast
    end
    
    fprintf('3D Animation complete.\n');
end
