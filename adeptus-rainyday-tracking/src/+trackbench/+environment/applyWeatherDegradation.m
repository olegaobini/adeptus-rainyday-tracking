function [pdMultiplier, noiseMultiplier, weatherClutter] = applyWeatherDegradation( ...
    simTime, weatherConfig, sensorInfo, sensorPos, sensorParams, weatherType)
%applyWeatherDegradation  Dispatch weather degradation by type.
%
%   Author:  James Gallegos (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Unified interface for all weather effects. Routes to the appropriate
%  physics model based on weatherType, returning the same 3 outputs.
%
%  SUPPORTED TYPES
%    'rain'  — ITU-R P.838-3 via rainpl(). Frequency-dependent RF path loss.
%              Heavy impact on X-band, mild on S-band. Generates volume clutter.
%    'snow'  — Uses rainpl() at 25% of equivalent precipitation rate.
%              Snow particles are less dense than rain drops (Gunn & East, 1954).
%              Reduced clutter vs rain (lower reflectivity).
%    'fog'   — Visibility-based degradation. Primarily affects IR/optical sensors.
%              Negligible RF effect below 10 GHz. Uses fogpl() for X-band+.
%              Wet radome effect on all radar. No volume clutter.
%    'icing' — Antenna hardware degradation (ice buildup reduces gain).
%              Flat Pd reduction across all ranges. Not a path effect.
%              No weather clutter. Noise increase from degraded receiver.
%
%  OUTPUTS (same interface as applyRainDegradation)
%    pdMultiplier    : function handle @(slantRange) -> scalar 0-1
%    noiseMultiplier : scalar >= 1.0
%    weatherClutter  : cell array of objectDetection weather returns
%
%  See also: applyRainDegradation, rainpl, fogpl

if nargin < 6 || isempty(weatherType)
    weatherType = 'rain';
end

switch lower(weatherType)
    case 'rain'
        [pdMultiplier, noiseMultiplier, weatherClutter] = ...
            trackbench.environment.applyRainDegradation( ...
                simTime, weatherConfig, sensorInfo, sensorPos, sensorParams);
    
    case 'snow'
        [pdMultiplier, noiseMultiplier, weatherClutter] = ...
            applySnowDegradation(simTime, weatherConfig, sensorInfo, sensorPos, sensorParams);
    
    case 'fog'
        [pdMultiplier, noiseMultiplier, weatherClutter] = ...
            applyFogDegradation(simTime, weatherConfig, sensorInfo, sensorPos, sensorParams);
    
    case 'icing'
        [pdMultiplier, noiseMultiplier, weatherClutter] = ...
            applyIcingDegradation(simTime, weatherConfig, sensorInfo, sensorPos, sensorParams);
    
    otherwise
        warning('applyWeatherDegradation:unknownType', ...
            'Unknown weather type "%s", defaulting to rain.', weatherType);
        [pdMultiplier, noiseMultiplier, weatherClutter] = ...
            trackbench.environment.applyRainDegradation( ...
                simTime, weatherConfig, sensorInfo, sensorPos, sensorParams);
end

end


%% ========================================================================
%  SNOW — rainpl() at reduced rate (Gunn & East, 1954)
%% ========================================================================
function [pdMult, noiseMult, clutter] = applySnowDegradation(simTime, config, si, sPos, sParams)
%applySnowDegradation  Snow attenuation via scaled rain model.
%
%  Snow particles are less dense and have different dielectric properties
%  than rain drops. Empirically, snow attenuation is approximately 20-30%
%  of the equivalent liquid precipitation rate for RF frequencies.
%  Ref: Gunn & East (1954), Battan (1973), "Radar Meteorology"
%
%  Snow also produces less volume clutter than rain because snow has
%  lower radar reflectivity (Z-R relationship differs).

    % Scale rain rate: snow ≈ 25% of equivalent rain rate for RF attenuation
    snowConfig = config;
    snowConfig.rain_rate_mmhr = config.rain_rate_mmhr * 0.25;
    
    % Use rain model with reduced rate
    [pdMult, noiseMult, clutter] = trackbench.environment.applyRainDegradation( ...
        simTime, snowConfig, si, sPos, sParams);
    
    % Reduce clutter density further (snow reflectivity lower than rain)
    % Keep only ~40% of rain-equivalent clutter returns
    if ~isempty(clutter)
        keepMask = rand(numel(clutter), 1) < 0.4;
        clutter = clutter(keepMask);
    end
    
    % Snow adds mild noise from wet antenna + scattering
    noiseMult = max(1.0, noiseMult * 0.8);  % less noise than rain
end


%% ========================================================================
%  FOG — Visibility-based, primarily affects IR/optical
%% ========================================================================
function [pdMult, noiseMult, clutter] = applyFogDegradation(simTime, config, si, sPos, sParams)
%applyFogDegradation  Fog/low-visibility degradation.
%
%  Fog is liquid water droplets suspended in air at ground level.
%  RF effect is negligible below 10 GHz (S-band, L-band unaffected).
%  Above 10 GHz, uses MATLAB fogpl() [ITU-R P.840] if available.
%
%  Primary effect: drastically reduces IR/optical sensor visibility.
%  Secondary effect: wet radome loss on radar antennas.
%
%  PARAMETERS
%    config.rain_rate_mmhr — reinterpreted as fog density proxy:
%      5  = light fog (visibility ~1 km)
%      15 = moderate fog (visibility ~300 m)
%      30 = dense fog (visibility ~100 m)
%
%  Ref: ITU-R P.840-6, Koschmieder visibility law

    fogDensity = getField(config, 'rain_rate_mmhr', 15);  % reuse rain_rate as density proxy
    pdFloor = getField(config, 'pd_floor', 0.15);
    
    freq = 2.8e9;
    if isfield(si, 'radarFreq') && si.radarFreq > 0; freq = si.radarFreq; end
    isIR = isfield(si, 'isIR') && si.isIR;
    isMSSR = isfield(si, 'isMSSR') && si.isMSSR;
    
    rMax = 111120;
    if isfield(sParams, 'rangeLimits'); rMax = sParams.rangeLimits(2); end
    
    % Visibility in km (Koschmieder law approximation)
    % fogDensity 5 -> ~1.5 km, 15 -> ~0.5 km, 30 -> ~0.25 km
    visibilityKm = 8.0 / (1 + 0.5 * fogDensity);
    
    if isIR
        % IR sensors severely affected — exponential decay with range
        % At visibility distance, Pd drops to ~5% (Lambert-Beer law)
        capturedVis = visibilityKm;
        capturedFloor = pdFloor;
        pdMult = @(r) max(capturedFloor, exp(-3.0 * r / (capturedVis * 1000)));
        noiseMult = 1.0 + 1.5 * (fogDensity / 15);  % significant noise increase
    
    elseif isMSSR
        % SSR transponders barely affected by fog
        pdMult = @(r) 1.0;
        noiseMult = 1.0;
    
    else
        % Radar — fog has negligible RF effect below 10 GHz
        freqGHz = freq / 1e9;
        if freqGHz >= 10
            % Above 10 GHz: use fogpl if available, else mild approximation
            % fogpl(range, freq, temperature, liquidWaterDensity)
            % Typical liquid water density for fog: 0.05-0.5 g/m³
            lwc = min(0.5, 0.02 * fogDensity);  % liquid water content (g/m³)
            capturedFreq = freq;
            capturedLWC = lwc;
            capturedFloor = pdFloor;
            pdMult = @(r) computeFogPdRadar(r, capturedFreq, capturedLWC, capturedFloor);
        else
            % Below 10 GHz: negligible RF attenuation from fog
            % Only wet radome effect
            pdMult = @(r) max(pdFloor, 1.0 - 0.02 * (fogDensity / 15));
        end
        
        % Wet radome loss (condensation on antenna)
        radomeLoss = 0.3 + 0.5 * min(fogDensity / 30, 1.0);  % 0.3-0.8 dB
        noiseMult = 1.0 + 0.3 * (fogDensity / 15) + 10^(radomeLoss / 10) - 1;
    end
    
    % No weather clutter from fog (no volume returns)
    clutter = {};
end


%% ========================================================================
%  ICING — Antenna hardware degradation
%% ========================================================================
function [pdMult, noiseMult, clutter] = applyIcingDegradation(simTime, config, si, sPos, sParams)
%applyIcingDegradation  Antenna icing degradation.
%
%  Ice accumulation on radar antennas causes:
%    - Increased sidelobe levels (distorted aperture distribution)
%    - Reduced main beam gain (2-6 dB typical for moderate icing)
%    - Increased system noise temperature (lossy ice layer)
%    - Mechanical issues with rotating antennas (weight, drag)
%
%  This is NOT a path propagation effect — it's hardware degradation.
%  Effect is range-independent (unlike rain/fog).
%
%  PARAMETERS
%    config.rain_rate_mmhr — reinterpreted as icing severity:
%      5  = light rime ice (1-2 dB gain loss)
%      15 = moderate glaze ice (3-4 dB gain loss)
%      30 = severe ice accumulation (5-6 dB gain loss)
%
%  Ref: Skolnik, "Introduction to Radar Systems", Ch. 12
%       FAA Order 6560.20B (radar maintenance in icing conditions)

    icingSeverity = getField(config, 'rain_rate_mmhr', 15);  % reuse as severity
    pdFloor = getField(config, 'pd_floor', 0.15);
    
    isIR = isfield(si, 'isIR') && si.isIR;
    isMSSR = isfield(si, 'isMSSR') && si.isMSSR;
    
    if isIR
        % IR windows can ice over — moderate effect
        iceFactor = min(0.5, 0.02 * icingSeverity);
        pdMult = @(r) max(pdFloor, 1.0 - iceFactor);
        noiseMult = 1.0 + 0.3 * (icingSeverity / 15);
    
    elseif isMSSR
        % SSR antennas affected by icing but less than PSR
        gainLoss_dB = 0.5 + 1.0 * min(icingSeverity / 30, 1.0);
        pdReduction = 1.0 - (1.0 - 10^(-gainLoss_dB / 10));
        pdMult = @(r) max(pdFloor, pdReduction);
        noiseMult = 1.0 + 0.1 * (icingSeverity / 15);
    
    else
        % Radar: flat Pd reduction from antenna gain loss (range-independent)
        % Light (5): ~2 dB loss, Moderate (15): ~4 dB loss, Severe (30): ~6 dB
        gainLoss_dB = 1.0 + 5.0 * min(icingSeverity / 30, 1.0);
        snrMargin = 12;  % same as rain model
        
        % Flat Pd across all ranges (hardware, not path)
        pdReduction = max(pdFloor, 10^(-gainLoss_dB / snrMargin));
        capturedPd = pdReduction;
        capturedFloor = pdFloor;
        pdMult = @(r) max(capturedFloor, capturedPd);
        
        % Ice on radome increases system noise temperature
        noiseMult = 1.0 + 0.6 * (icingSeverity / 15);
    end
    
    % No weather clutter from icing (no atmospheric scatterers)
    clutter = {};
end


%% ========================================================================
%  HELPER: Fog Pd for radar above 10 GHz
%% ========================================================================
function pd = computeFogPdRadar(slantRange, freq, lwc, pdFloor)
    if slantRange < 500; pd = 1.0; return; end
    try
        % fogpl(range, freq, temperature, liquidWaterDensity)
        L_oneway = fogpl(slantRange, freq, 15, lwc);  % 15°C typical
        L_twoway = 2 * L_oneway;
        pd = max(pdFloor, 10^(-L_twoway / 12));
    catch
        % Fallback if fogpl not available
        % Approximate: fog attenuation ~0.01-0.1 dB/km at 10-30 GHz
        freqGHz = freq / 1e9;
        gamma = 0.005 * lwc * freqGHz^1.5;  % dB/km approximate
        L_twoway = 2 * gamma * slantRange / 1000;
        pd = max(pdFloor, 10^(-L_twoway / 12));
    end
end


%% ========================================================================
function val = getField(s, field, default)
    if isstruct(s) && isfield(s, field); val = s.(field); else; val = default; end
end
