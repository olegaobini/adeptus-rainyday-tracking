function plotInitialScenario(dataLog, animateFlag)
% PLOTINITIALSCENARIO Visualize Truth and Detections (3D Animated)
% Colors: Truth tracks are distinct (Blue, Red, etc.). Detections are Green.
    
    if nargin < 2
        animateFlag = true; % Default behavior
    end
    
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
    if animateFlag
        fprintf('Animating 3D scenario... (Press Ctrl+C to stop)\n');
    else
        fprintf('Generating static 3D scenario plot...\n');
    end
    
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
                if numel(p) >= 3
                    xVal = p(1) * s; 
                    yVal = p(2) * s;
                    zVal = p(3) * s; 
                    
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
                                addpoints(hDetect, meas(1)*s, meas(2)*s, meas(3)*s);
                            elseif numel(meas) == 2
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
            
            % Only limit frame rate if we are animating
            if animateFlag && mod(k, 2) == 0
                drawnow limitrate;
            end
        end
        
        % --- RENDER FRAME ---
        if animateFlag
            drawnow;
        end
    end
    
    % Force one final render at the very end to show the static plot
    drawnow;
    
    if animateFlag
        fprintf('3D Animation complete.\n');
    else
        fprintf('3D Static plot complete.\n');
    end
end
