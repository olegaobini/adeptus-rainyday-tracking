function drawSensorCoverage(ax, sensorCoverage, scaleKm)
%drawSensorCoverage  Draw sensor coverage areas on a 3D plot.
%
% Draws range rings (360° sensors) or wedge arcs (sector sensors) on the
% ground plane (Z=0) to show each sensor's coverage area.
%
% INPUTS
%   ax              : axes handle to draw on
%   sensorCoverage  : struct array from dataLog.SensorCoverage
%   scaleKm         : if true, positions/ranges are in meters → draw in km
%
% Coverage types:
%   360° rotator  : dashed circle at max range + translucent fill
%   Sector sensor : wedge arc with filled area
%
% Azimuth convention (MATLAB NED):
%   0° = North (+X), 90° = East (+Y), measured clockwise

if nargin < 3; scaleKm = true; end

if isempty(sensorCoverage); return; end

sf = 1;
if scaleKm; sf = 1/1000; end  % meters to km

% Color scheme per sensor type
nSensors = numel(sensorCoverage);
baseColors = [
    0.2  0.6  1.0;   % blue  — PSR/radar
    0.9  0.5  0.1;   % orange — MSSR
    0.1  0.9  0.3;   % green  — PAR / sector
    0.9  0.2  0.8;   % magenta — IR
    0.2  0.9  0.9;   % cyan   — other
    1.0  0.8  0.2;   % yellow
    0.6  0.3  0.9;   % purple
];

hold(ax, 'on');

for k = 1:nSensors
    cov = sensorCoverage(k);
    
    % Choose color based on sensor type
    if cov.isMSSR
        col = baseColors(2, :);   % orange
    elseif cov.isIR
        col = baseColors(4, :);   % magenta
    elseif ~cov.isRotator && cov.isRadar
        col = baseColors(3, :);   % green (sector radar like PAR)
    elseif cov.isRadar
        col = baseColors(1, :);   % blue (PSR)
    else
        ci = mod(k-1, size(baseColors,1)) + 1;
        col = baseColors(ci, :);
    end
    
    % Sensor position on ground plane
    cx = cov.position(1) * sf;
    cy = cov.position(2) * sf;
    R  = cov.maxRange * sf;
    
    % Ground plane Z (slight offset below 0 so it doesn't z-fight with data)
    gz = 0.05;
    
    azMin = cov.azLimits(1) + cov.mountingYaw;
    azMax = cov.azLimits(2) + cov.mountingYaw;
    azSpan = azMax - azMin;
    
    if azSpan >= 350
        % ---- 360° rotator: draw range ring ----
        nPts = 120;
        theta = linspace(0, 360, nPts);
        xRing = cx + R * cosd(theta);
        yRing = cy + R * sind(theta);
        zRing = gz * ones(1, nPts);
        
        % Dashed range ring
        plot3(ax, xRing, yRing, zRing, '--', 'Color', [col 0.6], ...
            'LineWidth', 1.5, 'HandleVisibility', 'off');
        
        % Translucent fill (very subtle)
        fill3(ax, xRing, yRing, zRing, col, ...
            'FaceAlpha', 0.05, 'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
        
        % Label at top of ring
        rangeStr = sprintf('%.0f nm', cov.maxRange / 1852);
        text(ax, cx, cy + R * 0.92, gz, ...
            sprintf('%s (%s)', cov.label, rangeStr), ...
            'Color', col, 'FontSize', 8, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        
        % Legend entry
        plot3(ax, NaN, NaN, NaN, '--', 'Color', col, 'LineWidth', 1.5, ...
            'DisplayName', sprintf('%s (%.0f nm)', cov.label, cov.maxRange/1852));
        
    else
        % ---- Sector sensor: draw wedge ----
        nPts = max(30, round(azSpan));
        theta = linspace(azMin, azMax, nPts);
        
        % Wedge outline: center → arc → center
        xWedge = [cx, cx + R * cosd(theta), cx];
        yWedge = [cy, cy + R * sind(theta), cy];
        zWedge = gz * ones(1, numel(xWedge));
        
        % Filled wedge
        fill3(ax, xWedge, yWedge, zWedge, col, ...
            'FaceAlpha', 0.15, 'EdgeColor', col, 'EdgeAlpha', 0.7, ...
            'LineWidth', 1.5, 'HandleVisibility', 'off');
        
        % Range arc (bold outer edge)
        xArc = cx + R * cosd(theta);
        yArc = cy + R * sind(theta);
        zArc = gz * ones(1, nPts);
        plot3(ax, xArc, yArc, zArc, '-', 'Color', col, ...
            'LineWidth', 2, 'HandleVisibility', 'off');
        
        % Radial lines from center to edges of sector
        plot3(ax, [cx, cx + R*cosd(azMin)], [cy, cy + R*sind(azMin)], [gz gz], ...
            '-', 'Color', [col 0.7], 'LineWidth', 1, 'HandleVisibility', 'off');
        plot3(ax, [cx, cx + R*cosd(azMax)], [cy, cy + R*sind(azMax)], [gz gz], ...
            '-', 'Color', [col 0.7], 'LineWidth', 1, 'HandleVisibility', 'off');
        
        % Label at midpoint of arc
        midAz = (azMin + azMax) / 2;
        labelR = R * 0.75;
        rangeStr = sprintf('%.0f nm', cov.maxRange / 1852);
        text(ax, cx + labelR*cosd(midAz), cy + labelR*sind(midAz), gz, ...
            sprintf('%s\n%s / %.0f°', cov.label, rangeStr, azSpan), ...
            'Color', col, 'FontSize', 8, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
        
        % Legend entry
        plot3(ax, NaN, NaN, NaN, '-', 'Color', col, 'LineWidth', 2, ...
            'DisplayName', sprintf('%s (%.0f nm / %.0f°)', cov.label, cov.maxRange/1852, azSpan));
    end
    
    % Sensor position marker
    plot3(ax, cx, cy, gz, 'p', 'MarkerSize', 10, ...
        'MarkerFaceColor', col, 'MarkerEdgeColor', 'w', ...
        'HandleVisibility', 'off');
end

end
