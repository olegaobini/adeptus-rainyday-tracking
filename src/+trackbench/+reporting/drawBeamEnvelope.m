function drawBeamEnvelope(ax, sensorCoverage, ~)
%drawBeamEnvelope  Draw 3D beam elevation coverage cones on scenario plot.
%
%  Uses the ACTUAL scan limits from coverageConfig() (stored in coverage
%  metadata as scanElLimits). These are the true beam edges — if a target
%  is between the cones, it WILL be detected (subject to Pd < 1 misses).
%
%  Green cone = upper beam edge (sky boundary)
%  Red cone   = lower beam edge (ground boundary)
%
%  See also: drawSensorCoverage, plotInitialScenario, runDetections

if isempty(sensorCoverage); return; end

s = 1/1000;  % meters → km

for k = 1:numel(sensorCoverage)
    cov = sensorCoverage(k);
    if ~cov.isRadar && ~cov.isIR; continue; end
    if cov.isMSSR; continue; end
    
    % Sensor position (NED → altitude-up)
    pos = cov.position;
    sensorX = pos(1) * s;
    sensorY = pos(2) * s;
    sensorZ = -pos(3) * s;
    
    % Use actual scan elevation limits from coverageConfig()
    % These are the TRUE beam edges as reported by MATLAB.
    if isfield(cov, 'scanElLimits') && numel(cov.scanElLimits) == 2
        beamLowerEdge = cov.scanElLimits(1);
        beamUpperEdge = cov.scanElLimits(2);
    elseif isfield(cov, 'elLimits') && numel(cov.elLimits) == 2
        % Fallback: use MechanicalElevationLimits directly
        beamLowerEdge = cov.elLimits(1);
        beamUpperEdge = cov.elLimits(2);
    else
        fov = cov.fov;
        beamLowerEdge = -fov(2)/2;
        beamUpperEdge = fov(2)/2;
    end
    
    % Display range
    maxRange = min(cov.maxRange, 120000);
    displayRange = min(maxRange * 0.5, 60000) * s;
    
    % Azimuth
    azLimits = cov.azLimits;
    isRotator = cov.isRotator || (abs(diff(azLimits)) >= 350);
    
    if isRotator
        nAz = 72;
        azAngles = linspace(0, 360, nAz+1);
    else
        nAz = max(12, round(abs(diff(azLimits))/3));
        azAngles = linspace(azLimits(1), azLimits(2), nAz+1);
    end
    
    % Draw upper beam edge (green = sky boundary)
    drawConeSurface(ax, sensorX, sensorY, sensorZ, ...
        displayRange, beamUpperEdge, azAngles, [0.2 0.8 0.2], 0.08);
    
    % Draw lower beam edge (red = ground boundary)
    drawConeSurface(ax, sensorX, sensorY, sensorZ, ...
        displayRange, beamLowerEdge, azAngles, [0.8 0.2 0.2], 0.08);
    
    % Label
    labelR = displayRange * 0.35;
    midEl = (beamUpperEdge + beamLowerEdge) / 2;
    labelZ = sensorZ + labelR * sind(midEl);
    labelY = sensorY - labelR * cosd(midEl);
    
    labelStr = sprintf('%s EL:[%.0f° to +%.0f°]', ...
        string(cov.label), beamLowerEdge, beamUpperEdge);
    text(ax, sensorX, labelY, labelZ, labelStr, ...
        'Color', [0.5 1 0.5], 'FontSize', 8, 'FontWeight', 'bold');
end

end


function drawConeSurface(ax, cx, cy, cz, rangeKm, elAngleDeg, azAngles, color, alpha)
    nAz = numel(azAngles);
    X = zeros(2, nAz);
    Y = zeros(2, nAz);
    Z = zeros(2, nAz);
    
    for ri = 1:2
        r = [0, rangeKm];
        groundR = r(ri) * cosd(elAngleDeg);
        alt = r(ri) * sind(elAngleDeg);
        for ai = 1:nAz
            az = azAngles(ai);
            X(ri, ai) = cx + groundR * cosd(az);
            Y(ri, ai) = cy + groundR * sind(az);
            Z(ri, ai) = cz + alt;
        end
    end
    
    surf(ax, X, Y, Z, ...
        'FaceColor', color, 'FaceAlpha', alpha, ...
        'EdgeColor', color, 'EdgeAlpha', 0.12, ...
        'LineWidth', 0.3, ...
        'HandleVisibility', 'off');
end
