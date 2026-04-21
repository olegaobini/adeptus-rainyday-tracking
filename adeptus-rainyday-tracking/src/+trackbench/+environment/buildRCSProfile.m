function sig = buildRCSProfile(profileName, baseRCS_dBsm)
%buildRCSProfile  Create an rcsSignature with a realistic aspect-dependent pattern.
%
%  Instead of a single isotropic RCS value, this creates a full azimuth ×
%  elevation pattern that varies with viewing angle — just like real aircraft.
%
%  MATLAB's fusionRadarSensor uses this natively: when a platform has an
%  rcsSignature with a Pattern matrix, the sensor automatically looks up
%  the RCS for the current aspect angle between the radar and target.
%
%  AVAILABLE PROFILES
%    'stealth'    : Very low frontal RCS, moderate side/rear. Models B-2/F-117.
%    'fighter'    : Low frontal, moderate side, high rear (engine nozzles).
%    'airliner'   : Large uniform RCS, slightly higher broadside.
%    'drone'      : Small, relatively uniform.
%    'missile'    : Very small frontal, moderate rear (exhaust plume).
%
%  USAGE
%    sig = buildRCSProfile('stealth', -10);  % base RCS = -10 dBsm
%    tgt.Signatures = {sig};
%
%  REFERENCES
%    [1] MathWorks, rcsSignature documentation:
%        https://www.mathworks.com/help/fusion/ref/rcssignature.html
%    [2] MathWorks, rcsSignature.value examples (Boeing 737 RCS data):
%        https://www.mathworks.com/help/radar/ref/rcssignature.value.html
%    [3] Skolnik, M., "Introduction to Radar Systems," 3rd ed.
%        Table of typical aircraft RCS values by aspect angle.
%    [4] Knott, E., Shaeffer, J., Tuley, M., "Radar Cross Section," 2nd ed.
%
%  See also: rcsSignature, addTargetFromDef

    if nargin < 2
        baseRCS_dBsm = 0;
    end

    az = -180:10:180;   % azimuth angles (degrees)
    el = -90:10:90;     % elevation angles (degrees)
    nAz = numel(az);
    nEl = numel(el);

    switch lower(profileName)
        case 'stealth'
            % Stealth aircraft (B-2, F-117 type)
            % Extreme frontal reduction, moderate side lobes, larger rear
            % Front (0°): baseRCS (very low)
            % Side (±90°): baseRCS + 15-20 dB
            % Rear (180°): baseRCS + 10-15 dB
            azPattern = zeros(1, nAz);
            for i = 1:nAz
                a = abs(az(i));
                if a <= 30
                    azPattern(i) = baseRCS_dBsm;  % frontal sector: stealth
                elseif a <= 60
                    azPattern(i) = baseRCS_dBsm + 10 * ((a - 30) / 30);  % transition
                elseif a <= 120
                    azPattern(i) = baseRCS_dBsm + 10 + 8 * sin(pi * (a - 60) / 60);  % side lobes
                elseif a <= 150
                    azPattern(i) = baseRCS_dBsm + 12;  % rear quarter
                else
                    azPattern(i) = baseRCS_dBsm + 10;  % dead rear (slightly less than broadside)
                end
            end

        case 'fighter'
            % Conventional fighter (F-15, F-16 type)
            % Moderate frontal, large broadside, very large rear (engines)
            azPattern = zeros(1, nAz);
            for i = 1:nAz
                a = abs(az(i));
                if a <= 20
                    azPattern(i) = baseRCS_dBsm;  % nose-on
                elseif a <= 45
                    azPattern(i) = baseRCS_dBsm + 5 * ((a - 20) / 25);
                elseif a <= 135
                    azPattern(i) = baseRCS_dBsm + 5 + 10 * sin(pi * (a - 45) / 90);  % broadside peak
                elseif a <= 160
                    azPattern(i) = baseRCS_dBsm + 12;  % rear quarter
                else
                    azPattern(i) = baseRCS_dBsm + 15;  % tail-on (engine cavities)
                end
            end

        case 'airliner'
            % Large commercial aircraft (737/747/A320 type)
            % Large RCS from all angles, broadside peak from fuselage
            azPattern = zeros(1, nAz);
            for i = 1:nAz
                a = abs(az(i));
                if a <= 30
                    azPattern(i) = baseRCS_dBsm - 3;  % nose-on (slightly less)
                elseif a <= 60
                    azPattern(i) = baseRCS_dBsm;
                elseif a <= 120
                    azPattern(i) = baseRCS_dBsm + 5 * sin(pi * (a - 60) / 60);  % broadside peak
                else
                    azPattern(i) = baseRCS_dBsm - 2;  % tail
                end
            end

        case 'drone'
            % Small UAV — relatively uniform, small
            azPattern = baseRCS_dBsm + 3 * sin(pi * abs(az) / 180);  % mild variation

        case 'missile'
            % Cruise missile — very small frontal, moderate rear (exhaust)
            azPattern = zeros(1, nAz);
            for i = 1:nAz
                a = abs(az(i));
                if a <= 15
                    azPattern(i) = baseRCS_dBsm;  % nose-on: tiny
                elseif a <= 90
                    azPattern(i) = baseRCS_dBsm + 8 * ((a - 15) / 75);
                else
                    azPattern(i) = baseRCS_dBsm + 8 + 5 * ((a - 90) / 90);  % rear: exhaust
                end
            end

        otherwise
            % Isotropic fallback
            azPattern = baseRCS_dBsm * ones(1, nAz);
    end

    % Expand to 2D: modest elevation variation (±3 dB)
    % Belly-up and top-down give slightly higher RCS than horizon-level
    pattern = zeros(nEl, nAz);
    for j = 1:nEl
        elFactor = 2 * abs(cosd(el(j)));  % max at horizon, reduced overhead
        pattern(j, :) = azPattern - (3 - elFactor);
    end

    % Create MATLAB rcsSignature object
    % Ref: https://www.mathworks.com/help/fusion/ref/rcssignature.html
    sig = rcsSignature('Pattern', pattern, 'Azimuth', az, 'Elevation', el);

    fprintf('    [RCS] Profile "%s": base=%.0f dBsm, front=%.0f, side=%.0f, rear=%.0f dBsm\n', ...
        profileName, baseRCS_dBsm, pattern(10, find(az==0)), ...
        max(pattern(10, find(az==90)), pattern(10, find(az==-90))), ...
        pattern(10, find(az==180, 1)));
end