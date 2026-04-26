function centerDeg = sectorCenterDeg(sectorDeg)
%sectorCenterDeg  Wrap-aware azimuth midpoint of a [start end] sector.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  centerDeg = trackbench.editor.sectorCenterDeg([a1 a2])
%
%  Returns the angular mean of the two sector edges, handling the case
%  where the sector wraps across 0°/360°. Result is folded into the
%  canonical [0, 360) range.
%
%  Why this exists
%    The naive `(a1 + a2)/2` gives 180° for a [350, 10] sector when the
%    user intent is 0° (the sector crosses north). M6 §3.6 rendering
%    (drawBeamConeVolume3D) centers a cone along that midpoint, and
%    wrap-unaware math aimed no-scan TWS-style sensors at the wrong
%    bearing when the user placed the forward wedge across north.
%
%  Algorithm
%    Project each edge onto the unit circle (cosd/sind), average the
%    components, then atan2d the result. For a [350, 10] sector the
%    averaged vector points at 0° exactly. For a non-wrapping [170, 190]
%    it degenerates to the arithmetic mean (180°).
%
%  Edge cases
%    * Empty / single-element input → returns 0 (guard only; callers
%      should pass two-element vectors from SensorRecord.sectorDeg).
%    * Diametrically opposed edges (e.g. [0, 180]) — atan2d of the zero
%      vector returns 0, which matches the "take the first edge" tie
%      break the old arithmetic path would have given for [0, 180].
%
%  See also: trackbench.editor.drawMap

    if numel(sectorDeg) < 2
        centerDeg = 0;
        return;
    end
    a1 = sectorDeg(1);
    a2 = sectorDeg(2);
    x  = cosd(a1) + cosd(a2);
    y  = sind(a1) + sind(a2);
    if abs(x) < 1e-12 && abs(y) < 1e-12
        % Edges are exactly opposite; no well-defined midpoint.
        centerDeg = 0;
        return;
    end
    centerDeg = atan2d(y, x);
    if centerDeg < 0
        centerDeg = centerDeg + 360;
    end
end
