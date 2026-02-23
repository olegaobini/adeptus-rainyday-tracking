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
%    rural    — Gentle rolling hills, 20-80m peaks. Minimal occlusion
%               except for very low-altitude targets at long range.
%    urban    — Mostly flat with building-cluster bumps (50-150m).
%               Creates intermittent occlusion at close/medium range.
%    mountain — Ridge lines and peaks, 500-2000m. Significant occlusion
%               for low-altitude targets. Valleys create coverage shadows.
%    desert   — Flat with gentle dunes, 10-40m. Negligible occlusion.
%
%  INPUTS
%    terrainType    : 'water'|'rural'|'urban'|'mountain'|'desert' (string)
%    scenarioBounds : [xMin xMax; yMin yMax] bounding box (m), or empty
%                     for default [-80000 80000; -80000 80000]
%    elevScale      : elevation multiplier (default 1.0). 0 = flat, 2 = double height.
%
%  OUTPUTS
%    Zterrain  : NxN terrain height grid (NED: negative = above ground)
%    boundary  : [xMin xMax; yMin yMax] for groundSurface 'Boundary'
%    Xg, Yg    : meshgrid coordinates (m) for optional plotting
%
%  USAGE
%    [Z, bnd] = generateTerrain('mountain', [-50000 50000; -50000 50000]);
%    groundSurface(scenario, 'Terrain', Z, 'Boundary', bnd);
%
% See also: groundSurface, trackingScenario, SurfaceManager

    if nargin < 2 || isempty(scenarioBounds)
        scenarioBounds = [-80000 80000; -80000 80000];
    end
    if nargin < 3 || isempty(elevScale)
        elevScale = 1.0;
    end
    
    terrainType = lower(string(terrainType));
    boundary = scenarioBounds;
    
    % Grid resolution: 200x200 gives ~800m cells for 160km span
    % Fine enough for LOS checks, coarse enough for fast generation
    nPts = 200;
    xVec = linspace(boundary(1,1), boundary(1,2), nPts);
    yVec = linspace(boundary(2,1), boundary(2,2), nPts);
    [Xg, Yg] = meshgrid(xVec, yVec);
    
    % Seed RNG for reproducibility (terrain should be the same each run)
    rngState = rng;
    rng(42, 'twister');
    
    switch terrainType
        case "water"
            % Flat ocean — no terrain features
            Zterrain = zeros(nPts);
            
        case "rural"
            % Gentle rolling farmland: low-frequency sinusoidal hills
            % Max elevation ~50-80m
            Z1 = 30 * sin(Xg / 8000) .* cos(Yg / 6000);
            Z2 = 20 * sin(Xg / 4000 + 1.2) .* sin(Yg / 9000 + 0.7);
            % A few isolated hills
            Z3 = 50 * exp(-((Xg - 15000).^2 + (Yg + 12000).^2) / (5000^2));
            Z4 = 40 * exp(-((Xg + 8000).^2 + (Yg + 25000).^2) / (6000^2));
            Zterrain = -(Z1 + Z2 + Z3 + Z4);  % NED: negate for altitude
            
        case "urban"
            % Mostly flat with building-cluster bumps
            % Baseline: flat with gentle undulation
            Zbase = 10 * sin(Xg / 12000) .* cos(Yg / 10000);
            % Building clusters: gaussian bumps, 50-150m
            Zbuildings = zeros(nPts);
            clusterCenters = [
                5000, -8000, 120;   % tall cluster
                -3000, -15000, 80;
                12000, -5000, 100;
                -10000, -20000, 60;
                8000, -25000, 90;
                0, -3000, 150;      % near sensor, tall buildings
            ];
            for c = 1:size(clusterCenters, 1)
                cx = clusterCenters(c, 1);
                cy = clusterCenters(c, 2);
                ch = clusterCenters(c, 3);
                cw = 1500 + 1000 * rand;  % cluster width
                Zbuildings = Zbuildings + ch * exp(-((Xg-cx).^2 + (Yg-cy).^2) / (cw^2));
            end
            Zterrain = -(Zbase + Zbuildings);
            
        case "mountain"
            % Ridge lines and peaks: creates significant LOS shadows
            % Primary ridge running roughly E-W at Y = -15000 to -20000
            ridge1 = 1200 * exp(-(Yg + 18000).^2 / (3000^2)) .* ...
                     (0.7 + 0.3 * sin(Xg / 6000));
            % Secondary ridge at different angle
            ridge2 = 800 * exp(-((Yg + 10000) - 0.3*(Xg-5000)).^2 / (2500^2));
            % Isolated peaks
            peak1 = 1500 * exp(-((Xg - 12000).^2 + (Yg + 22000).^2) / (4000^2));
            peak2 = 1000 * exp(-((Xg + 10000).^2 + (Yg + 30000).^2) / (5000^2));
            % Valley floor
            valley = 200 * sin(Xg / 10000) .* cos(Yg / 8000);
            Zterrain = -(ridge1 + ridge2 + peak1 + peak2 + valley);
            
        case "desert"
            % Flat with gentle dunes
            Z1 = 15 * sin(Xg / 3000 + 0.5) .* cos(Yg / 4000);
            Z2 = 10 * sin(Xg / 6000) .* sin(Yg / 2000 + 1.0);
            % Occasional mesa
            Z3 = 40 * exp(-((Xg + 5000).^2 + (Yg + 20000).^2) / (3000^2));
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
    
    rng(rngState);
    
    % Summary
    maxElev = -min(Zterrain(:));
    fprintf('[Terrain] %s: grid=%dx%d, maxElev=%.0fm, scale=%.1f\n', ...
        terrainType, nPts, nPts, maxElev, elevScale);
end
