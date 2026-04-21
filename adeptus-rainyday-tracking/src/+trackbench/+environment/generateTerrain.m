function [Zterrain, boundary, Xg, Yg] = generateTerrain(terrainType, scenarioBounds, elevScale)
%generateTerrain  Procedural terrain heightmap for occlusion modelling.
%
%  Generates a terrain elevation grid in NED coordinates (z-negative = up)
%  based on terrain type. The heightmap is attached to a trackingScenario
%  via groundSurface() to enable SurfaceManager.occlusion() line-of-sight
%  checks that replace the simpler 4/3 Earth horizon model.
%
%  TERRAIN PROFILES:
%    water    — Flat sea level (z=0). No terrain occlusion.
%    rural    — Gentle rolling hills, 20-80m peaks. Minimal occlusion.
%    urban    — Mostly flat with building-cluster bumps (50-150m).
%    mountain — Ridge lines and peaks, 500-2000m. Significant occlusion.
%    desert   — Flat with gentle dunes, 10-40m. Negligible occlusion.
%
%  INPUTS
%    terrainType    : 'water'|'rural'|'urban'|'mountain'|'desert' (string)
%    scenarioBounds : [xMin xMax; yMin yMax] bounding box (m), or empty
%    elevScale      : elevation multiplier (default 1.0)
%
%  OUTPUTS
%    Zterrain  : NxN terrain height grid (NED: negative = above ground)
%    boundary  : [xMin xMax; yMin yMax] for groundSurface 'Boundary'
%    Xg, Yg    : meshgrid coordinates (m) for optional plotting

    if nargin < 2 || isempty(scenarioBounds)
        scenarioBounds = [-80000 80000; -80000 80000];
    end
    if nargin < 3 || isempty(elevScale)
        elevScale = 1.0;
    end
    
    terrainType = lower(string(terrainType));
    boundary = scenarioBounds;
    
    nPts = 200;
    xVec = linspace(boundary(1,1), boundary(1,2), nPts);
    yVec = linspace(boundary(2,1), boundary(2,2), nPts);
    [Xg, Yg] = meshgrid(xVec, yVec);
    
    rngState = rng;
    rng(42, 'twister');
    
    switch terrainType
        case "water"
            Zterrain = zeros(nPts);
            
        case "rural"
            % Gentle rolling farmland — periodic so it tiles across any grid
            Z1 = 30 * sin(Xg / 8000) .* cos(Yg / 6000);
            Z2 = 20 * sin(Xg / 4000 + 1.2) .* sin(Yg / 9000 + 0.7);
            Z3 = 15 * sin(Xg / 12000 + 0.3) .* cos(Yg / 5000 + 2.1);
            Z4 = 10 * cos(Xg / 15000 + 1.8) .* sin(Yg / 7000 + 0.5);
            Zterrain = -(Z1 + Z2 + Z3 + Z4);
            
        case "urban"
            Zbase = 10 * sin(Xg / 12000) .* cos(Yg / 10000);
            Zbuildings = zeros(nPts);
            clusterCenters = [
                5000, -8000, 120;
                -3000, -15000, 80;
                12000, -5000, 100;
                -10000, -20000, 60;
                8000, -25000, 90;
                0, -3000, 150;
            ];
            for c = 1:size(clusterCenters, 1)
                cx = clusterCenters(c, 1);
                cy = clusterCenters(c, 2);
                ch = clusterCenters(c, 3);
                cw = 1500 + 1000 * rand;
                Zbuildings = Zbuildings + ch * exp(-((Xg-cx).^2 + (Yg-cy).^2) / (cw^2));
            end
            Zterrain = -(Zbase + Zbuildings);
            
        case "mountain"
            % Mountain terrain using PERIODIC functions so features tile
            % across the entire grid regardless of size.
            
            % Long-wavelength ridges (run roughly E-W, repeat every ~50km)
            ridge1 = 600 * sin(Yg / 25000 * pi) .* (0.7 + 0.3 * sin(Xg / 15000));
            ridge2 = 400 * sin((Yg + 12000) / 20000 * pi) .* (0.6 + 0.4 * cos(Xg / 18000));
            
            % Diagonal ridge (NW-SE orientation)
            diagDist = (Xg * 0.6 + Yg * 0.8);
            ridge3 = 500 * sin(diagDist / 22000 * pi) .* (0.5 + 0.5 * sin(Xg / 30000));
            
            % Medium-wavelength peaks (repeat every ~30km)
            peaks1 = 350 * (sin(Xg / 14000 * pi).^2) .* (cos(Yg / 16000 * pi).^2);
            peaks2 = 250 * (cos(Xg / 18000 * pi + 1).^2) .* (sin(Yg / 12000 * pi + 0.7).^2);
            
            % Short-wavelength roughness
            rough = 80 * sin(Xg / 3000) .* cos(Yg / 4000) + ...
                    60 * cos(Xg / 5000 + 2) .* sin(Yg / 3500 + 1);
            
            % Valley floors (lower-frequency undulation)
            valley = 100 * sin(Xg / 40000) .* cos(Yg / 35000);
            
            Zterrain = -(ridge1 + ridge2 + ridge3 + peaks1 + peaks2 + rough + valley);
            
        case "desert"
            Z1 = 15 * sin(Xg / 3000 + 0.5) .* cos(Yg / 4000);
            Z2 = 10 * sin(Xg / 6000) .* sin(Yg / 2000 + 1.0);
            Z3 = 8 * cos(Xg / 8000 + 1.5) .* sin(Yg / 5000);
            Zterrain = -(Z1 + Z2 + Z3);
            
        otherwise
            warning('generateTerrain:unknownType', ...
                'Unknown terrain type "%s", using rural.', terrainType);
            [Zterrain, boundary, Xg, Yg] = trackbench.environment.generateTerrain( ...
                'rural', scenarioBounds, elevScale);
            rng(rngState);
            return;
    end
    
    % Apply elevation scale
    if elevScale ~= 1.0
        Zterrain = Zterrain * elevScale;
    end
    
    % Ensure terrain doesn't go above z=0 (no underground terrain in NED)
    Zterrain = min(Zterrain, 0);
    
    % ---- RADAR HILLTOP CLEARING ----
    % The radar platform sits at the origin (0,0,0). Without this fix,
    % the terrain at origin can be hundreds of meters above z=0, which
    % buries the radar underground. This creates a smooth cleared hilltop:
    % the radar sits at the terrain's local peak, and the surrounding
    % terrain blends naturally from the hilltop down to the full terrain.
    %
    % The clearing radius (5km) creates a realistic radar installation
    % site — a cleared hilltop with good line-of-sight in all directions.
    radarElev = Zterrain(round(nPts/2), round(nPts/2));  % terrain height at origin
    if radarElev < -10  % only if terrain is significantly above sea level
        dist = sqrt(Xg.^2 + Yg.^2);
        clearRadius = 5000;  % 5km smooth clearing around radar
        % Gaussian blend: at origin terrain → 0, far away → unchanged
        % The radar platform will sit at z=0 (ground level at origin)
        blend = 1 - exp(-(dist / clearRadius).^2);
        Zterrain = Zterrain .* blend;
        
        % Now RAISE the radar to sit on a small hilltop above surroundings.
        % This gives the radar realistic elevated line-of-sight.
        radarHillHeight = 50;  % 50m hilltop — realistic for a radar site
        radarHill = -radarHillHeight * exp(-(dist / 2000).^2);
        Zterrain = Zterrain + radarHill;
        Zterrain = min(Zterrain, 0);  % re-clamp
    end
    
    rng(rngState);
    
    maxElev = -min(Zterrain(:));
    fprintf('[Terrain] %s: grid=%dx%d, maxElev=%.0fm, scale=%.1f\n', ...
        terrainType, nPts, nPts, maxElev, elevScale);
end