function [newXLim, newYLim] = zoomAroundPoint(xLim, yLim, cx, cy, scrollCount, factorPerTick)
%zoomAroundPoint  Compute new axis limits for a mousewheel zoom that keeps
%                 the world point (cx, cy) stationary under the cursor.
%
%  INPUT
%    xLim           [xMin xMax] current axes X limits
%    yLim           [yMin yMax] current axes Y limits
%    cx, cy         Cursor world coordinates (the "anchor" for the zoom)
%    scrollCount    Wheel ticks — positive = zoom out, negative = zoom in.
%                   Uses the same convention as uifigure's
%                   WindowScrollWheelFcn evt.VerticalScrollCount.
%    factorPerTick  (optional) Zoom factor per tick. Default 1.2.
%
%  OUTPUT
%    newXLim, newYLim  New limits that would replace ax.XLim / ax.YLim.
%
%  MATH
%    factor = factorPerTick ^ scrollCount   (so +tick → factor > 1 → wider)
%    newXLim(1) = cx - (cx - xLim(1)) * factor
%    newXLim(2) = cx + (xLim(2) - cx) * factor
%    Same for Y.
%
%    This keeps the world point (cx, cy) at the same relative position in
%    the viewport after the zoom — i.e. the cursor stays pinned to the
%    same piece of the map, which is the standard "zoom where you point"
%    UX that feels natural in all modern mapping tools.
%
%  DEGENERATE CASES
%    - Zero-width span: returns the input unchanged (avoids NaN).
%    - Non-finite inputs: returns the input unchanged.
%    - Zero scroll count: returns the input unchanged.
%
%  DESIGN NOTES
%    - Pure function: no UI side effects, no dependency on a figure or
%      axes handle. Called from buildUI/onScrollWheel, unit-tested in
%      testPathEditor_shortcuts.m.
%    - 3D zoom is NOT handled here — in 3D the scroll wheel modifies
%      ax.CameraViewAngle instead of XLim/YLim, which doesn't benefit
%      from the anchor-point math (the camera orbits the target).
%
%  See also: trackbench.editor.computeScaleBar, trackbench.editor.buildUI

    arguments
        xLim          (1,2) double
        yLim          (1,2) double
        cx            (1,1) double
        cy            (1,1) double
        scrollCount   (1,1) double
        factorPerTick (1,1) double = 1.2
    end

    newXLim = xLim;
    newYLim = yLim;

    if scrollCount == 0
        return;
    end
    if ~all(isfinite([xLim yLim cx cy factorPerTick]))
        return;
    end
    if xLim(2) <= xLim(1) || yLim(2) <= yLim(1)
        return;
    end

    factor = factorPerTick ^ scrollCount;

    newXLim = [cx - (cx - xLim(1)) * factor, ...
               cx + (xLim(2) - cx) * factor];
    newYLim = [cy - (cy - yLim(1)) * factor, ...
               cy + (yLim(2) - cy) * factor];
end
