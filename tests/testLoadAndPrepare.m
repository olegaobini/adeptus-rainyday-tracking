repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'scripts'));
setupTrackbench(repoRoot);

fprintf('=== loadAndPrepare Integration Test ===\n');
pass = 0; fail = 0;

try
    plan = trackbench.loader.loadAndPrepare("scenarios/my_test");
    assert(numel(plan) == 1);
    cfg = plan(1);
    assert(isfield(cfg, 'scenario'));
    assert(isfield(cfg, 'run_id'));
    fprintf('  [PASS] scenario path integration\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] scenario integration: %s\n', ME.message); fail = fail + 1;
end

try
    plan = trackbench.loader.loadAndPrepare("sweeps/weather_study");
    assert(numel(plan) == 3);
    fprintf('  [PASS] sweep path integration\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] sweep integration: %s\n', ME.message); fail = fail + 1;
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoadAndPrepare failed'); end
