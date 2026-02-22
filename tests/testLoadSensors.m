%% testLoadSensors.m — Verify loadSensors reads JSON and builds sensors
% Run from V2 root: >> testLoadSensors

clc; close all;
addpath(genpath(fullfile(pwd, 'src', 'helpers')));

fprintf('=== loadSensors Test ===\n\n');

passed = 0;
failed = 0;

%% Test 1: Default catalog loads (PSR + SSR enabled)
fprintf('--- Test 1: Default catalog ---\n');
try
    [sensors, metas, catalog] = loadSensors();
    
    assert(isfield(sensors, 'tower'), 'No tower platform group');
    assert(numel(sensors.tower) == 2, 'Expected 2 tower sensors, got %d', numel(sensors.tower));
    
    % Check sensor indices are sequential
    assert(sensors.tower{1}.SensorIndex == 1, 'First sensor index should be 1');
    assert(sensors.tower{2}.SensorIndex == 2, 'Second sensor index should be 2');
    
    % Check types
    assert(isa(sensors.tower{1}, 'fusionRadarSensor'), 'Sensor 1 wrong class');
    assert(isa(sensors.tower{2}, 'fusionRadarSensor'), 'Sensor 2 wrong class');
    
    % Check metadata
    assert(strcmp(metas.tower{1}.catalogName, 'DASR Primary Search Radar'), 'Wrong name for sensor 1');
    assert(strcmp(metas.tower{2}.catalogName, 'DASR Secondary Surveillance Radar (MSSR/IFF)'), 'Wrong name for sensor 2');
    
    fprintf('  [PASS] Default catalog: 2 tower sensors built\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 2: Scenario integration — attach to trackingScenario
fprintf('\n--- Test 2: Scenario integration ---\n');
try
    [sensors, ~] = loadSensors();
    
    scen = trackingScenario;
    scen.UpdateRate = sensors.tower{1}.UpdateRate;
    tower = platform(scen, 'Sensors', sensors.tower);
    
    % Add target
    tgt = platform(scen);
    tgt.Trajectory = waypointTrajectory( ...
        'Waypoints', [0 -20000 -3000; 1000 -20000 -3000], ...
        'TimeOfArrival', [0 50], ...
        'Velocities', [20 0 0; 20 0 0]);
    
    advance(scen);
    fprintf('  [PASS] Sensors attached to scenario, advanced 1 step\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 3: All sensors enabled — build everything
fprintf('\n--- Test 3: All sensors enabled (temp catalog) ---\n');
try
    % Read the catalog, enable everything, write to temp file
    raw = jsondecode(fileread(fullfile(pwd, 'config', 'sensors.json')));
    for i = 1:numel(raw.sensors)
        raw.sensors(i).enabled = true;
    end
    tempPath = fullfile(pwd, 'config', 'sensors_all_test.json');
    fid = fopen(tempPath, 'w');
    fprintf(fid, '%s', jsonencode(raw));
    fclose(fid);
    
    [sensors, metas] = loadSensors('sensors_all_test');
    
    % Count total sensors across all platforms
    platforms = fieldnames(sensors);
    totalSensors = 0;
    for p = 1:numel(platforms)
        totalSensors = totalSensors + numel(sensors.(platforms{p}));
    end
    
    % We have 19 entries in the catalog
    % ADS-B TX/RX are different (not fusionRadarSensor), but should still build
    fprintf('  Built %d sensors across %d platforms\n', totalSensors, numel(platforms));
    assert(totalSensors >= 17, 'Expected at least 17 sensors, got %d', totalSensors);
    
    % Verify sequential indices
    allIndices = [];
    for p = 1:numel(platforms)
        for s = 1:numel(sensors.(platforms{p}))
            sObj = sensors.(platforms{p}){s};
            if isprop(sObj, 'SensorIndex')
                allIndices(end+1) = sObj.SensorIndex; %#ok<AGROW>
            end
        end
    end
    sortedIdx = sort(allIndices);
    expectedIdx = 1:numel(sortedIdx);
    assert(isequal(sortedIdx, expectedIdx), ...
        'Sensor indices not sequential. Got: [%s]', num2str(sortedIdx));
    
    fprintf('  [PASS] All sensors built, indices sequential [1..%d]\n', totalSensors);
    passed = passed + 1;
    
    % Cleanup temp file
    delete(tempPath);
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
    try delete(tempPath); catch; end
end

%% Test 4: No sensors enabled
fprintf('\n--- Test 4: No sensors enabled ---\n');
try
    raw = jsondecode(fileread(fullfile(pwd, 'config', 'sensors.json')));
    for i = 1:numel(raw.sensors)
        raw.sensors(i).enabled = false;
    end
    tempPath = fullfile(pwd, 'config', 'sensors_none_test.json');
    fid = fopen(tempPath, 'w');
    fprintf(fid, '%s', jsonencode(raw));
    fclose(fid);
    
    [sensors, ~] = loadSensors('sensors_none_test');
    
    platforms = fieldnames(sensors);
    assert(isempty(platforms), 'Expected no platforms, got %d', numel(platforms));
    
    fprintf('  [PASS] No sensors enabled, empty result\n');
    passed = passed + 1;
    
    delete(tempPath);
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
    try delete(tempPath); catch; end
end

%% Test 5: Parameter overrides from JSON
fprintf('\n--- Test 5: Parameter overrides ---\n');
try
    raw = jsondecode(fileread(fullfile(pwd, 'config', 'sensors.json')));
    % Disable all, enable only PSR with custom params
    for i = 1:numel(raw.sensors)
        raw.sensors(i).enabled = false;
    end
    raw.sensors(1).enabled = true;
    raw.sensors(1).params.rpm = 25;
    raw.sensors(1).params.pd = 0.75;
    raw.sensors(1).params.rangeLimits = [0; 200000];
    
    tempPath = fullfile(pwd, 'config', 'sensors_override_test.json');
    fid = fopen(tempPath, 'w');
    fprintf(fid, '%s', jsonencode(raw));
    fclose(fid);
    
    [sensors, metas] = loadSensors('sensors_override_test');
    
    psr = sensors.tower{1};
    meta = metas.tower{1};
    
    assert(abs(meta.rpm - 25) < 0.01, 'RPM override failed: got %.1f', meta.rpm);
    assert(abs(meta.pd - 0.75) < 0.01, 'Pd override failed: got %.2f', meta.pd);
    assert(abs(psr.RangeLimits(2) - 200000) < 1, 'Range override failed');
    
    fprintf('  [PASS] Overrides applied (rpm=25, pd=0.75, range=200km)\n');
    passed = passed + 1;
    
    delete(tempPath);
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
    try delete(tempPath); catch; end
end

%% Test 6: Multi-platform grouping
fprintf('\n--- Test 6: Multi-platform grouping ---\n');
try
    raw = jsondecode(fileread(fullfile(pwd, 'config', 'sensors.json')));
    % Enable PSR (tower), AESA (aircraft), MARITIME (ship)
    for i = 1:numel(raw.sensors)
        raw.sensors(i).enabled = false;
    end
    % PSR = index 1 (tower)
    raw.sensors(1).enabled = true;
    % AESA = index 7 (aircraft)
    raw.sensors(7).enabled = true;
    % MARITIME = index 10 (ship)
    raw.sensors(10).enabled = true;
    
    tempPath = fullfile(pwd, 'config', 'sensors_multi_test.json');
    fid = fopen(tempPath, 'w');
    fprintf(fid, '%s', jsonencode(raw));
    fclose(fid);
    
    [sensors, metas] = loadSensors('sensors_multi_test');
    
    platforms = fieldnames(sensors);
    assert(numel(platforms) == 3, 'Expected 3 platforms, got %d', numel(platforms));
    assert(isfield(sensors, 'tower'), 'Missing tower platform');
    assert(isfield(sensors, 'aircraft'), 'Missing aircraft platform');
    assert(isfield(sensors, 'ship'), 'Missing ship platform');
    
    fprintf('  [PASS] 3 platforms: tower, aircraft, ship\n');
    passed = passed + 1;
    
    delete(tempPath);
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
    try delete(tempPath); catch; end
end

%% Summary
fprintf('\n===================================\n');
fprintf(' RESULTS: %d passed, %d failed, %d total\n', passed, failed, passed+failed);
fprintf('===================================\n');
