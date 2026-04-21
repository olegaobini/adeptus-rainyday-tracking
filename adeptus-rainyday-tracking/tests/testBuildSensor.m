%% testBuildSensor.m — Verify all buildSensor types create without error
% Run from V2 root: >> testBuildSensor

clc; close all;
addpath(genpath(fullfile(pwd, 'src', 'helpers')));

fprintf('=== buildSensor Factory Test ===\n\n');

results = {};
passed = 0;
failed = 0;

%% ---- RADAR FAMILY ----
radarTypes = {'PSR', 'SSR', 'ASR', 'ARSR', 'PAR', 'TWS', 'AESA', ...
              'FIRE_CONTROL', 'WEATHER', 'MARITIME', 'CUSTOM_RADAR'};

for i = 1:numel(radarTypes)
    sType = radarTypes{i};
    try
        [s, m] = buildSensor(i, sType);
        assert(isa(s, 'fusionRadarSensor'), 'Wrong class');
        assert(s.SensorIndex == i, 'SensorIndex mismatch');
        assert(~isempty(m.type), 'Empty meta type');
        fprintf('  [PASS] %-15s | class=%-20s | idx=%d | range=[%.0f, %.0f]m\n', ...
            sType, class(s), s.SensorIndex, s.RangeLimits(1), s.RangeLimits(2));
        passed = passed + 1;
        results{end+1} = {sType, 'PASS', ''}; %#ok<SAGROW>
    catch ME
        fprintf('  [FAIL] %-15s | %s\n', sType, ME.message);
        failed = failed + 1;
        results{end+1} = {sType, 'FAIL', ME.message}; %#ok<SAGROW>
    end
end

%% ---- IR FAMILY ----
irTypes = {'IRST', 'IR_STARING', 'FLIR', 'CUSTOM_IR'};

for i = 1:numel(irTypes)
    sType = irTypes{i};
    idx = 20 + i;
    try
        [s, m] = buildSensor(idx, sType);
        assert(isa(s, 'irSensor'), 'Wrong class: %s', class(s));
        assert(s.SensorIndex == idx, 'SensorIndex mismatch');
        fprintf('  [PASS] %-15s | class=%-20s | idx=%d\n', sType, class(s), s.SensorIndex);
        passed = passed + 1;
        results{end+1} = {sType, 'PASS', ''}; %#ok<SAGROW>
    catch ME
        fprintf('  [FAIL] %-15s | %s\n', sType, ME.message);
        failed = failed + 1;
        results{end+1} = {sType, 'FAIL', ME.message}; %#ok<SAGROW>
    end
end

%% ---- SONAR FAMILY ----
sonarTypes = {'ACTIVE_SONAR', 'PASSIVE_SONAR', 'TOWED_ARRAY', 'CUSTOM_SONAR'};

for i = 1:numel(sonarTypes)
    sType = sonarTypes{i};
    idx = 30 + i;
    try
        [s, m] = buildSensor(idx, sType);
        assert(isa(s, 'sonarSensor'), 'Wrong class: %s', class(s));
        assert(s.SensorIndex == idx, 'SensorIndex mismatch');
        fprintf('  [PASS] %-15s | class=%-20s | idx=%d\n', sType, class(s), s.SensorIndex);
        passed = passed + 1;
        results{end+1} = {sType, 'PASS', ''}; %#ok<SAGROW>
    catch ME
        fprintf('  [FAIL] %-15s | %s\n', sType, ME.message);
        failed = failed + 1;
        results{end+1} = {sType, 'FAIL', ME.message}; %#ok<SAGROW>
    end
end

%% ---- LIDAR ----
lidarTypes = {'LIDAR', 'CUSTOM_LIDAR'};

for i = 1:numel(lidarTypes)
    sType = lidarTypes{i};
    idx = 40 + i;
    try
        [s, m] = buildSensor(idx, sType);
        assert(isa(s, 'monostaticLidarSensor'), 'Wrong class: %s', class(s));
        assert(s.SensorIndex == idx, 'SensorIndex mismatch');
        fprintf('  [PASS] %-15s | class=%-20s | idx=%d\n', sType, class(s), s.SensorIndex);
        passed = passed + 1;
        results{end+1} = {sType, 'PASS', ''}; %#ok<SAGROW>
    catch ME
        fprintf('  [FAIL] %-15s | %s\n', sType, ME.message);
        failed = failed + 1;
        results{end+1} = {sType, 'FAIL', ME.message}; %#ok<SAGROW>
    end
end

%% ---- ADS-B ----
try
    [tx, m] = buildSensor(50, 'ADSB_TX', 'ICAO', 'A1B2C3');
    assert(isa(tx, 'adsbTransponder'), 'Wrong class: %s', class(tx));
    fprintf('  [PASS] %-15s | class=%-20s\n', 'ADSB_TX', class(tx));
    passed = passed + 1;
    results{end+1} = {'ADSB_TX', 'PASS', ''}; %#ok<SAGROW>
catch ME
    fprintf('  [FAIL] %-15s | %s\n', 'ADSB_TX', ME.message);
    failed = failed + 1;
    results{end+1} = {'ADSB_TX', 'FAIL', ME.message}; %#ok<SAGROW>
end

try
    [rx, m] = buildSensor(51, 'ADSB_RX');
    assert(isa(rx, 'adsbReceiver'), 'Wrong class: %s', class(rx));
    fprintf('  [PASS] %-15s | class=%-20s\n', 'ADSB_RX', class(rx));
    passed = passed + 1;
    results{end+1} = {'ADSB_RX', 'PASS', ''}; %#ok<SAGROW>
catch ME
    fprintf('  [FAIL] %-15s | %s\n', 'ADSB_RX', ME.message);
    failed = failed + 1;
    results{end+1} = {'ADSB_RX', 'FAIL', ME.message}; %#ok<SAGROW>
end

%% ---- CUSTOM TEMPLATE ----
try
    [tmpl, m] = buildSensor(99, 'CUSTOM');
    assert(isstruct(tmpl), 'Expected struct template');
    assert(isfield(tmpl, 'requiredProperties'), 'Missing requiredProperties');
    fprintf('  [PASS] %-15s | struct template returned\n', 'CUSTOM');
    passed = passed + 1;
    results{end+1} = {'CUSTOM', 'PASS', ''}; %#ok<SAGROW>
catch ME
    fprintf('  [FAIL] %-15s | %s\n', 'CUSTOM', ME.message);
    failed = failed + 1;
    results{end+1} = {'CUSTOM', 'FAIL', ME.message}; %#ok<SAGROW>
end

%% ---- PARAMETER OVERRIDE TEST ----
fprintf('\n--- Parameter Override Tests ---\n');

try
    [s, m] = buildSensor(60, 'PSR', 'rpm', 25, 'pd', 0.75, 'rangeLimits', [0 200000]);
    assert(abs(m.rpm - 25) < 0.01, 'RPM override failed');
    assert(abs(m.pd - 0.75) < 0.01, 'Pd override failed');
    assert(abs(m.rangeLimits_m(2) - 200000) < 1, 'Range override failed');
    fprintf('  [PASS] PSR with overrides (rpm=25, pd=0.75, range=200km)\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] PSR overrides | %s\n', ME.message);
    failed = failed + 1;
end

try
    [s, m] = buildSensor(61, 'SSR', 'pd', 0.95, 'rangeLimits', [0 300000]);
    assert(abs(m.pd - 0.95) < 0.01, 'SSR Pd override failed');
    fprintf('  [PASS] SSR with overrides (pd=0.95, range=300km)\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] SSR overrides | %s\n', ME.message);
    failed = failed + 1;
end

%% ---- UNMATCHED PASSTHROUGH TEST ----
try
    [s, m] = buildSensor(62, 'PSR', 'HasRangeRate', true);
    assert(s.HasRangeRate == true, 'Unmatched passthrough failed');
    fprintf('  [PASS] Unmatched passthrough (HasRangeRate=true)\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] Unmatched passthrough | %s\n', ME.message);
    failed = failed + 1;
end

%% ---- SCENARIO INTEGRATION TEST ----
fprintf('\n--- Scenario Integration Test ---\n');

try
    [psr, ~] = buildSensor(1, 'PSR');
    [ssr, ~] = buildSensor(2, 'SSR');

    scen = trackingScenario;
    scen.UpdateRate = psr.UpdateRate;
    tower = platform(scen, 'Sensors', {psr, ssr});

    % Add a simple target
    tgt = platform(scen);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', [0 -20000 -3000; 1000 -20000 -3000], ...
        'TimeOfArrival', [0 50], ...
        'Velocities', [20 0 0; 20 0 0]);

    % Run one step
    advance(scen);
    fprintf('  [PASS] PSR+SSR added to trackingScenario, advanced 1 step\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] Scenario integration | %s\n', ME.message);
    failed = failed + 1;
end

%% ---- SUMMARY ----
fprintf('\n===================================\n');
fprintf(' RESULTS: %d passed, %d failed, %d total\n', passed, failed, passed+failed);
fprintf('===================================\n');

if failed > 0
    fprintf('\nFailed tests:\n');
    for i = 1:numel(results)
        if strcmp(results{i}{2}, 'FAIL')
            fprintf('  %s: %s\n', results{i}{1}, results{i}{3});
        end
    end
end
