%% testLoadScenario.m — Verify scenario catalog + loader
% Run from project root: >> run('tests/testLoadScenario.m')

clc; close all;
addpath(fullfile(pwd, 'src'));

fprintf('=== loadScenario Test ===\n\n');

passed = 0;
failed = 0;

%% Test 1: List scenarios
fprintf('--- Test 1: Catalog listing ---\n');
try
    scenarioList = trackbench.scenario.loadScenarioCatalog();
    assert(height(scenarioList) >= 8, 'Expected at least 8 scenarios, got %d', height(scenarioList));
    fprintf('  [PASS] Catalog has %d scenarios\n', height(scenarioList));
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 2: Load dasr_ideal
fprintf('\n--- Test 2: dasr_ideal ---\n');
try
    [scen, cfg, sens, metas] = trackbench.scenario.loadScenario("dasr_ideal");

    assert(isa(scen, 'trackingScenario'), 'Not a trackingScenario');
    assert(cfg.scenario.num_targets == 2, 'Wrong target count');
    assert(cfg.degradation.enabled == false, 'Should not be degraded');
    assert(isfield(sens, 'tower'), 'Missing tower platform');

    for i = 1:5; advance(scen); end
    fprintf('  [PASS] dasr_ideal: 2 targets, ideal, advanced 5 steps\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 3: Load dasr_degraded
fprintf('\n--- Test 3: dasr_degraded ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("dasr_degraded");

    assert(cfg.degradation.enabled == true, 'Should be degraded');
    assert(strcmp(cfg.degradation.type, 'rain'), 'Wrong degradation type');
    assert(cfg.active_params.pd < 0.9, 'Pd should be degraded (< 0.9)');

    advance(scen);
    fprintf('  [PASS] dasr_degraded: degraded mode, pd=%.2f\n', cfg.active_params.pd);
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 4: Load crossing_targets
fprintf('\n--- Test 4: crossing_targets ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("crossing_targets");

    assert(cfg.scenario.num_targets == 2, 'Wrong target count');
    assert(cfg.scenario.duration_s == 60, 'Wrong duration');

    advance(scen);
    fprintf('  [PASS] crossing_targets: 2 targets, 60s\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 5: Load high_density
fprintf('\n--- Test 5: high_density (5 targets) ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("high_density");

    assert(cfg.scenario.num_targets == 5, 'Wrong target count');

    advance(scen);
    fprintf('  [PASS] high_density: 5 targets\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 6: Load maneuvering_evasive
fprintf('\n--- Test 6: maneuvering_evasive (S-maneuver) ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("maneuvering_evasive");

    assert(cfg.scenario.num_targets == 1, 'Wrong target count');

    for i = 1:5; advance(scen); end
    fprintf('  [PASS] maneuvering_evasive: 1 target, S-maneuver\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 7: Load storm_window
fprintf('\n--- Test 7: storm_window ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("storm_window");

    assert(cfg.degradation.enabled == true, 'Should be degraded');
    assert(cfg.active_params.pd <= 0.5, 'Pd should be 0.5 for heavy rain');
    assert(cfg.tracker_params.degraded.gate >= 100, 'Gate should be >= 100');

    advance(scen);
    fprintf('  [PASS] storm_window: heavy rain, pd=%.2f, gate=%d\n', ...
        cfg.active_params.pd, cfg.tracker_params.degraded.gate);
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 8: Load head_on
fprintf('\n--- Test 8: head_on ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("head_on");

    assert(cfg.scenario.num_targets == 2, 'Wrong target count');

    advance(scen);
    fprintf('  [PASS] head_on: 2 targets converging\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 9: Load approach_pattern
fprintf('\n--- Test 9: approach_pattern ---\n');
try
    [scen, cfg, ~, ~] = trackbench.scenario.loadScenario("approach_pattern");

    assert(cfg.scenario.num_targets == 2, 'Wrong target count');
    assert(cfg.scenario.duration_s == 90, 'Wrong duration');

    advance(scen);
    fprintf('  [PASS] approach_pattern: 2 targets on approach, 90s\n');
    passed = passed + 1;
catch ME
    fprintf('  [FAIL] %s\n', ME.message);
    failed = failed + 1;
end

%% Test 10: Invalid scenario name
fprintf('\n--- Test 10: Invalid scenario name ---\n');
try
    trackbench.scenario.loadScenario("nonexistent_scenario");
    fprintf('  [FAIL] Should have thrown error\n');
    failed = failed + 1;
catch ME
    if contains(ME.identifier, 'scenarioNotFound')
        fprintf('  [PASS] Correctly rejected invalid scenario name\n');
        passed = passed + 1;
    else
        fprintf('  [FAIL] Wrong error: %s\n', ME.message);
        failed = failed + 1;
    end
end

%% Summary
fprintf('\n===================================\n');
fprintf(' RESULTS: %d passed, %d failed, %d total\n', passed, failed, passed+failed);
fprintf('===================================\n');
