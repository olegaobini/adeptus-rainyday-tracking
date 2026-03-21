function [pdMultiplier, noiseMultiplier, weatherClutter] = applyRainDegradation( ...
    simTime, rainConfig, sensorInfo, sensorPos, sensorParams)
%applyRainDegradation  Physics-based rain degradation for radar detections.
%
%  Computes three effects of rain on radar performance:
%    1. Detection probability reduction (range-dependent, frequency-dependent)
%    2. Measurement noise increase
%    3. Weather clutter generation within the sensor's coverage volume
%
%  PHYSICS MODEL
%    Rain attenuation uses the MATLAB Phased Array System Toolbox function
%    rainpl(), which implements the ITU-R P.838-3 specific attenuation model:
%      gamma_R = k * R^alpha  (dB/km, one-way)
%    where R is rain rate (mm/hr) and k, alpha are frequency-dependent
%    coefficients from ITU-R P.838-3 Table 1.
%
%    Two-way path loss at slant range d:
%      L_twoway = 2 * rainpl(d, freq, rainRate)
%
%    Key insight: S-band (2.8 GHz) barely affected by rain, X-band (9 GHz)
%    severely degraded. This is why real ATC uses S-band PSR.
%
%  REFERENCES
%    [1] ITU-R P.838-3: "Specific attenuation model for rain for use in
%        prediction methods," International Telecommunication Union, 2005.
%    [2] MathWorks, rainpl() documentation:
%        https://www.mathworks.com/help/phased/ref/rainpl.html
%    [3] MathWorks, "Modeling the Propagation of Radar Signals" example:
%        https://www.mathworks.com/help/radar/ug/modeling-the-propagation-of-rf-signals.html
%        "Rain can be a major limiting factor for radar systems, especially
%        when operating above 5 GHz."
%    [4] MathWorks, rainreflectivity() for weather clutter volume returns:
%        https://www.mathworks.com/help/radar/ref/rainreflectivity.html
%    [5] Seybold, J., "Introduction to RF Propagation," Wiley, 2005.
%
%  IMPLEMENTATION NOTES
%    - rainpl() is from the Phased Array System Toolbox (valid 1-1000 GHz)
%    - Weather clutter density scales with freq^2 (Rayleigh scattering)
%    - Noise model includes wet radome loss (~0.5-1.5 dB, per [5])
%    - IR visibility model uses exponential decay (Marshall-Palmer)
%    - SSR/MSSR uses mild link-budget degradation (active transponder)
%
%  OUTPUTS
%    pdMultiplier    : function handle @(slantRange) -> scalar 0-1
%    noiseMultiplier : scalar >= 1.0 (multiply MeasurementNoise by this)
%    weatherClutter  : cell array of objectDetection weather clutter returns
%
%  RAIN RATE GUIDELINES (for run file config)
%    1-4   : light rain / drizzle
%    4-16  : moderate rain (default = 16)
%    16-50 : heavy rain
%    50+   : severe / tropical downpour
%
%  See also: rainpl, rainreflectivity, clutterVolumeRCS, tropopl

%% Parse rain config with defaults
rainRate    = getField(rainConfig, 'rain_rate_mmhr', 16);
pdFloor     = getField(rainConfig, 'pd_floor', 0.15);
clutterMult = getField(rainConfig, 'clutter_multiplier', 1.0);

%% Get sensor properties
freq = 2.8e9;  % default S-band
if isfield(sensorInfo, 'radarFreq') && sensorInfo.radarFreq > 0
    freq = sensorInfo.radarFreq;
end
sIdx = 1;
if isfield(sensorInfo, 'sensorIndex')
    sIdx = sensorInfo.sensorIndex;
end

isIR   = isfield(sensorInfo, 'isIR') && sensorInfo.isIR;
isMSSR = isfield(sensorInfo, 'isMSSR') && sensorInfo.isMSSR;

%% Sensor geometry
rMax = 111120;
if isfield(sensorParams, 'rangeLimits')
    rMax = sensorParams.rangeLimits(2);
end

%% ════════════════════════════════════════════════════════════════════
%  LAYER 1: RANGE-DEPENDENT Pd REDUCTION (function handle)
%  ────────────────────────────────────────────────────────────────────
%  Uses rainpl() from MATLAB Phased Array System Toolbox.
%  rainpl implements ITU-R P.838-3 [1] and returns one-way attenuation
%  in dB for a given range, frequency, and rain rate.
%
%  Detection model:
%    SNR_rain = SNR_freespace - L_twoway
%    L_twoway = 2 * rainpl(range, freq, rainRate)   (dB, two-way)
%    Pd_rain  = Pd_ideal * 10^(-L_twoway / SNR_margin)
%
%  SNR_margin (12 dB) represents the typical detection threshold above
%  the noise floor for a radar operating at Pd=0.9, Pfa=1e-6 [5].
%% ════════════════════════════════════════════════════════════════════

snrMargin_dB = 12;  % typical radar detection threshold [5]

if isMSSR
    % SSR transponders are active — rain affects the interrogation link
    % but far less than passive radar returns. Mild degradation only.
    % Ref: SSR operates at 1030/1090 MHz (L-band), very low rain loss.
    pdMultiplier = @(slantRange) max(pdFloor, 1.0 - 0.05 * (rainRate / 16));
    
elseif isIR
    % IR sensors degraded by rain visibility, not RF attenuation.
    % Visibility model: exponential decay with rain rate.
    % Heavy rain (50 mm/hr) -> visibility ~1-2 km (Marshall-Palmer model)
    visibilityKm = 30 / (1 + 0.6 * rainRate^0.7);
    pdMultiplier = @(slantRange) max(pdFloor, exp(-slantRange / (visibilityKm * 1000)));
    
else
    % Radar: use rainpl() for ITU-R P.838-3 frequency-dependent attenuation
    % rainpl(range_m, freq_Hz, rainRate_mmhr) -> one-way loss in dB
    %
    % Capture freq, rainRate, snrMargin_dB, pdFloor in closure
    capturedFreq = freq;
    capturedRainRate = rainRate;
    capturedSnrMargin = snrMargin_dB;
    capturedPdFloor = pdFloor;
    
    pdMultiplier = @(slantRange) computeRadarPd( ...
        slantRange, capturedFreq, capturedRainRate, capturedSnrMargin, capturedPdFloor);
end

%% ════════════════════════════════════════════════════════════════════
%  LAYER 2: MEASUREMENT NOISE INCREASE
%  ────────────────────────────────────────────────────────────────────
%  Rain adds noise through:
%    - Reduced SNR -> worse range/angle estimates
%    - Atmospheric scintillation
%    - Wet radome loss (adds ~0.5-2 dB) [5]
%
%  Model: noise multiplier scales with rain rate.
%  Light rain (4): 1.2x | Moderate (16): 1.8x | Heavy (50): 3.0x
%% ════════════════════════════════════════════════════════════════════

if isIR
    noiseMultiplier = 1.0 + 0.5 * (rainRate / 16);
elseif isMSSR
    noiseMultiplier = 1.0 + 0.1 * (rainRate / 16);
else
    noiseMultiplier = 1.0 + 0.8 * log1p(rainRate / 8);
    % Wet radome contribution: 0.5-1.5 dB -> noise factor 1.1-1.4
    radomeLoss_dB = 0.5 + min(rainRate / 50, 1.0);
    noiseMultiplier = noiseMultiplier * 10^(radomeLoss_dB / 10);
end

%% ════════════════════════════════════════════════════════════════════
%  LAYER 3: WEATHER CLUTTER (generated ONCE per scan at flush time)
%  ────────────────────────────────────────────────────────────────────
%  Rain produces volume clutter returns within the radar beam.
%  Clutter density scales with rain rate and frequency (Rayleigh
%  scattering: reflectivity ~ freq^4, but illuminated volume also
%  increases, net effect ~ freq^2 for clutter count) [4].
%
%  Generated within the sensor's ACTUAL coverage volume (range, FOV).
%% ════════════════════════════════════════════════════════════════════

weatherClutter = {};

if isMSSR || isIR
    return;
end

freqGHz = freq / 1e9;
freqFactor = (freqGHz / 2.8)^2;   % S-band baseline, quadratic with freq
rangeFactor = (rMax / 111120);      % normalize to 60nm baseline
rateFactor  = (rainRate / 16)^0.8;  % sub-linear with rain rate (saturation)

baseClutterRate = 3.0;  % returns per scan at baseline conditions
clutterLambda = baseClutterRate * freqFactor * rangeFactor * rateFactor * clutterMult;
clutterLambda = min(clutterLambda, 30);

nClutter = poissrnd(clutterLambda);
if nClutter == 0; return; end

% Rain clutter noise
clutterSigma = 200 + 50 * (rainRate / 16);
Rclutter = eye(3) * clutterSigma^2;

% Rain height extent (meteorological model)
rainHeightM = max(2500, 5000 - 20 * rainRate);

for ii = 1:nClutter
    az = rand() * 360;
    rMin_c = max(sensorParams.rangeLimits(1), 2000);
    rMax_c = rMax * 0.7;
    rng_c = rMin_c + (rMax_c - rMin_c) * rand()^0.6;
    h_c = rand() * rainHeightM;
    
    xC = sensorPos(1) + rng_c * sind(az);
    yC = sensorPos(2) + rng_c * cosd(az);
    zC = -h_c;
    
    weatherClutter{end+1, 1} = objectDetection(simTime, [xC; yC; zC], ...
        'MeasurementNoise', Rclutter, ...
        'SensorIndex', sIdx); %#ok<AGROW>
end

end  % main function


%% ========================================================================
%  RADAR Pd COMPUTATION USING rainpl()
%% ========================================================================
function pd = computeRadarPd(slantRange, freq, rainRate, snrMargin, pdFloor)
%computeRadarPd  Compute detection probability at a given range under rain.
%
%  Uses MATLAB's rainpl() [ITU-R P.838-3] for one-way attenuation,
%  then converts to two-way Pd reduction via SNR margin model.
%
%  If rainpl() is unavailable (missing Phased Array Toolbox), falls back
%  to a built-in ITU coefficient table.

    if slantRange < 500
        pd = 1.0;  % close range: negligible rain loss
        return;
    end
    
    try
        % rainpl(range_m, freq_Hz, rainRate_mmhr) -> one-way loss in dB
        % Ref: https://www.mathworks.com/help/phased/ref/rainpl.html
        L_oneway_dB = rainpl(slantRange, freq, rainRate);
        L_twoway_dB = 2 * L_oneway_dB;
    catch
        % Fallback: hand-coded ITU-R P.838-3 coefficients
        % (in case Phased Array Toolbox is not installed)
        [k_coeff, alpha_coeff] = getITU838Fallback(freq);
        gamma_dBperkm = k_coeff * rainRate^alpha_coeff;
        L_twoway_dB = 2 * gamma_dBperkm * slantRange / 1000;
    end
    
    pd = max(pdFloor, 10^(-L_twoway_dB / snrMargin));
end


%% ========================================================================
%  FALLBACK ITU-R P.838-3 COEFFICIENTS (used if rainpl unavailable)
%% ========================================================================
function [k, alpha] = getITU838Fallback(freq)
%getITU838Fallback  Frequency-dependent rain attenuation coefficients.
%
%  Fallback for environments without Phased Array System Toolbox.
%  Coefficients from ITU-R P.838-3 Table 1 (horizontal polarization).
%  Ref: https://www.itu.int/rec/R-REC-P.838-3-200503-I/en

    freqGHz = freq / 1e9;
    refTable = [
        1.0,   0.0000387, 0.912;
        2.0,   0.000154,  0.963;
        3.0,   0.000650,  1.121;
        5.0,   0.00175,   1.332;
        7.0,   0.00265,   1.312;
        9.0,   0.00680,   1.310;
        10.0,  0.0101,    1.276;
        15.0,  0.0367,    1.154;
        20.0,  0.0751,    1.099;
        30.0,  0.187,     1.021;
    ];
    freqClamped = max(refTable(1,1), min(refTable(end,1), freqGHz));
    k     = exp(interp1(log(refTable(:,1)), log(refTable(:,2)), log(freqClamped), 'linear'));
    alpha = interp1(refTable(:,1), refTable(:,3), freqClamped, 'linear');
end


%% ========================================================================
function val = getField(s, field, default)
    if isstruct(s) && isfield(s, field); val = s.(field); else; val = default; end
end