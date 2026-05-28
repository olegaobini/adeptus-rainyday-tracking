function [V, F] = computeSensorCoverageVolume(cov, opts)
%COMPUTESENSORCOVERAGEVOLUME  Patch-ready vertices/faces for the 3D swept
%coverage volume of a sensor.
%
%   [V, F] = trackbench.reporting.computeSensorCoverageVolume(cov)
%   [V, F] = trackbench.reporting.computeSensorCoverageVolume(cov, opts)
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker - UW Senior Capstone, Boeing-sponsored
%   Since:   v3.7.5
%
%   Returns a closed boundary mesh (vertices V, faces F) describing the
%   3D region a sensor's mechanical scan envelope sweeps out, ready for
%       patch(ax, 'Vertices', V, 'Faces', F, 'FaceColor', ..., 'FaceAlpha', ...).
%   Output is in WORLD-frame NED meters; the caller is responsible for any
%   m->km scaling and NED->altitude-up Z-negation needed for display
%   (see plotInitialScenario.m for the canonical conversion).
%
%   This function supersedes drawBeamEnvelope.m: instead of two open cones
%   marking the upper/lower beam edges, it returns the full enclosed swept
%   volume so the audience can see what the sensor actually covers.
%   drawBeamEnvelope is retained as a fallback only and is no longer called
%   from the primary plotting path.
%
%   INPUTS
%     cov  : one element of dataLog.SensorCoverage (NOT the full array).
%            Required fields:
%              .position     1x3 NED meters, world frame (already composed
%                            as platform.Trajectory.Position +
%                            sensor.MountingLocation per
%                            runDetections.m:327).
%              .azLimits     1x2 deg, sensor-frame azimuth scan limits.
%                            mountingYaw is added inside this function to
%                            obtain the world-frame azimuth, mirroring
%                            drawSensorCoverage.m:68 (frozen) so the new
%                            volume aligns with the existing ground ring.
%              .mountingYaw  scalar deg, sensor mounting yaw.
%              .maxRange     scalar m. Capped against opts.rMaxCap to
%                            handle Inf RangeLimits (fusionRadarSensor
%                            default is [0 Inf], which the runDetections
%                            try/catch does not catch since no exception
%                            is thrown for Inf).
%            One of:
%              .scanElLimits 1x2 deg, from coverageConfig(sensor) - the
%                            true beam edges (preferred).
%              .elLimits     1x2 deg, fallback (MechanicalElevationLimits).
%            Optional:
%              .isRotator    bool. If true (or if azSpan>=350), the volume
%                            closes around 360 deg with no side walls.
%
%     opts : optional struct. Defaults used for absent fields:
%              .nAz      30      azimuth samples on outer shell
%              .nEl      10      elevation samples on outer shell
%              .rMaxCap  120000  m. Caps Inf RangeLimits per project
%                                convention (drawBeamEnvelope.m:47).
%
%   OUTPUT
%     V    nV-by-3 vertex matrix [x y z] in WORLD-frame NED meters.
%     F    nF-by-4 quad face index matrix into V.
%
%   GEOMETRY
%     For a 360 deg rotator (azSpan >= 350): the volume is bounded by
%       (1) outer spherical cap at r=rMax over an (az, el) grid,
%       (2) top cone at el=elMax over an (az, r) grid,
%       (3) bottom cone at el=elMin over an (az, r) grid.
%     For a sector sensor (azSpan < 350) the volume is additionally bounded
%     by two flat side walls:
%       (4) left wall at az=azMin over an (el, r) grid,
%       (5) right wall at az=azMax over an (el, r) grid.
%
%   ROTATION
%     Mounting yaw is applied via the cov.azLimits + cov.mountingYaw
%     convention from drawSensorCoverage.m (frozen). Mounting pitch and
%     roll are not propagated into cov by runDetections.m today; this
%     function treats them as 0. Per the R2025b fusionRadarSensor doc
%     (Sensor Mounting section), MountingAngles is [z-yaw, y-pitch,
%     x-roll]; a yaw-only rotation is correct for the project's current
%     scenarios (tower-mounted radars, roughly-upright aircraft). When
%     non-trivial pitch/roll lands - e.g., a forward-tilted IRST on an
%     aircraft body - either extend cov to carry the full triple, or add
%     a sensor-object overload of this function and apply MountingAngles
%     as a ZYX rotation matrix before world-frame translation.
%
%   PERFORMANCE
%     Default 30 x 10 sampling on the outer shell yields ~300 vertices and
%     ~330 quads per rotator (closed) or ~420 vertices and ~490 quads per
%     sector. A typical PosterDemo scene with two sensors costs <1000
%     vertices and <1000 quads total; static rendering is well below the
%     interactive-frame budget. If profiling later shows a hit, drop to
%     opts.nAz=20, opts.nEl=6.
%
%   See also: drawSensorCoverage, drawBeamEnvelope, plotInitialScenario,
%             trackbench.detections.runDetections, coverageConfig

% --- input handling -----------------------------------------------------
if nargin < 2 || isempty(opts); opts = struct; end
if ~isfield(opts, 'nAz');     opts.nAz     = 30;    end
if ~isfield(opts, 'nEl');     opts.nEl     = 10;    end
if ~isfield(opts, 'rMaxCap'); opts.rMaxCap = 120e3; end

% --- extract geometry primitives from cov -------------------------------
% Azimuth: world frame. Matches drawSensorCoverage.m:68 (frozen) so the
% new coverage volume lines up with the ground ring drawn by that file.
azMinDeg = cov.azLimits(1) + cov.mountingYaw;
azMaxDeg = cov.azLimits(2) + cov.mountingYaw;
azSpan   = azMaxDeg - azMinDeg;

% Elevation: prefer scanElLimits (true beam edges from coverageConfig).
if isfield(cov, 'scanElLimits') && numel(cov.scanElLimits) == 2
    elMinDeg = cov.scanElLimits(1);
    elMaxDeg = cov.scanElLimits(2);
elseif isfield(cov, 'elLimits') && numel(cov.elLimits) == 2
    elMinDeg = cov.elLimits(1);
    elMaxDeg = cov.elLimits(2);
elseif isfield(cov, 'fov') && numel(cov.fov) >= 2
    elMinDeg = -cov.fov(2)/2;
    elMaxDeg =  cov.fov(2)/2;
else
    error('trackbench:reporting:computeSensorCoverageVolume:missingElLimits', ...
        'cov must carry scanElLimits, elLimits, or fov.');
end

% Range: cap Inf per project convention.
rMax = cov.maxRange;
if ~isfinite(rMax) || rMax <= 0
    rMax = opts.rMaxCap;
end
rMax = min(rMax, opts.rMaxCap);

% Rotator detection: explicit flag wins; else infer from azSpan.
isRotator = false;
if isfield(cov, 'isRotator') && ~isempty(cov.isRotator)
    isRotator = logical(cov.isRotator);
end
if ~isRotator && abs(azSpan) >= 350
    isRotator = true;
end

% Sensor origin in world frame (NED meters).
pos = cov.position(:)';

% --- parametric grids ---------------------------------------------------
nAz = opts.nAz;
nEl = opts.nEl;

if isRotator
    % Closed 360 deg - use exactly 0..360 to align with drawSensorCoverage.
    azGrid  = linspace(0, 360, nAz + 1);
    azGrid(end) = [];                   % drop duplicate at 360 (wraps)
    closeAz = true;
else
    azGrid  = linspace(azMinDeg, azMaxDeg, nAz);
    closeAz = false;
end
elGrid = linspace(elMinDeg, elMaxDeg, nEl);
rGrid  = [0, rMax];

% --- helper: spherical sensor frame -> world NED meters -----------------
% elevation positive = above horizon. NED z+ = down, so the world-z of a
% point at range r and elevation el above the sensor is pos(3) - r*sin(el).
ned = @(r, az, el) [ ...
    pos(1) + r .* cosd(el) .* cosd(az), ...
    pos(2) + r .* cosd(el) .* sind(az), ...
    pos(3) - r .* sind(el) ];

% --- assemble surface vertices + faces ----------------------------------
V = zeros(0, 3);
F = zeros(0, 4);

% (1) Outer shell at r = rMax over (az, el)
[AZ, EL] = ndgrid(azGrid, elGrid);
shellV = ned(rMax, AZ(:), EL(:));
shellF = quadFaces(numel(azGrid), numel(elGrid), closeAz);
[V, F] = appendMesh(V, F, shellV, shellF);

% (2) Top cone at el = elMax over (az, r)
[AZ, R] = ndgrid(azGrid, rGrid);
topV = ned(R(:), AZ(:), elMaxDeg .* ones(numel(R), 1));
topF = quadFaces(numel(azGrid), numel(rGrid), closeAz);
[V, F] = appendMesh(V, F, topV, topF);

% (3) Bottom cone at el = elMin over (az, r)
botV = ned(R(:), AZ(:), elMinDeg .* ones(numel(R), 1));
botF = quadFaces(numel(azGrid), numel(rGrid), closeAz);
[V, F] = appendMesh(V, F, botV, botF);

% (4)+(5) Side walls (sector sensors only)
if ~closeAz
    [EL, R] = ndgrid(elGrid, rGrid);
    leftV  = ned(R(:), azMinDeg .* ones(numel(R), 1), EL(:));
    rightV = ned(R(:), azMaxDeg .* ones(numel(R), 1), EL(:));
    sideF = quadFaces(numel(elGrid), numel(rGrid), false);
    [V, F] = appendMesh(V, F, leftV,  sideF);
    [V, F] = appendMesh(V, F, rightV, sideF);
end

end


function F = quadFaces(nU, nV, closeU)
%QUADFACES  Quad face indices into a u-by-v ndgrid vertex linearization.
%   If closeU is true the u dimension wraps (last column connects to first),
%   so a full row produces nU faces; otherwise (nU - 1) faces per row.
%   Vertex linear index for (i, j) in ndgrid(u, v) is (j-1)*nU + i.

nRows = (nU - double(~closeU)) * (nV - 1);
F = zeros(nRows, 4);
row = 0;
for j = 1:(nV - 1)
    for i = 1:nU
        iNext = i + 1;
        if iNext > nU
            if closeU
                iNext = 1;
            else
                continue
            end
        end
        row = row + 1;
        a = (j-1) * nU + i;
        b = (j-1) * nU + iNext;
        c =  j    * nU + iNext;
        d =  j    * nU + i;
        F(row, :) = [a b c d];
    end
end
F = F(1:row, :);

end


function [V, F] = appendMesh(V, F, addV, addF)
%APPENDMESH  Append sub-mesh (addV, addF) to (V, F), re-indexing addF.
offset = size(V, 1);
V = [V; addV];
F = [F; addF + offset];

end
