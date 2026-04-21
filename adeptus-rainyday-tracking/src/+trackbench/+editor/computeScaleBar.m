function [lengthM, labelStr] = computeScaleBar(xSpanM)
%computeScaleBar  Pick a "nice round" scale-bar length for a given axes span.
%
%  INPUT
%    xSpanM   Visible axes width in world meters (positive, finite).
%
%  OUTPUT
%    lengthM   Scale-bar length in meters. Always one of:
%                10, 20, 50, 100, 200, 500, 1000, 2000, 5000,
%                10000, 20000, 50000, 100000, 200000, 500000
%              Chosen as the largest such value that is <= 15% of xSpanM,
%              with a 10 m floor so tiny zooms still show a scale bar.
%
%    labelStr  Human-friendly string, e.g. "5 km", "500 m".
%
%  DESIGN NOTES
%    - The "1/2/5 decade" set is standard for mapping scale bars — users
%      read "5 km" more easily than a computed 4.7 km.
%    - Aiming for 15% of span gives a visibly useful bar without
%      cluttering the map.
%    - Called from drawMap. Pure function (no UI side effects) so it's
%      unit-testable in testPathEditor_M3.m.

    arguments
        xSpanM (1,1) double {mustBeFinite, mustBePositive}
    end

    target = 0.15 * xSpanM;
    nice = [10, 20, 50, ...
            100, 200, 500, ...
            1000, 2000, 5000, ...
            10000, 20000, 50000, ...
            100000, 200000, 500000];

    pick = nice(find(nice <= target, 1, 'last'));
    if isempty(pick)
        pick = nice(1);   % floor at 10 m
    end
    lengthM = pick;

    if lengthM >= 1000
        labelStr = sprintf('%g km', lengthM / 1000);
    else
        labelStr = sprintf('%g m', lengthM);
    end
end
