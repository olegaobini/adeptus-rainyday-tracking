repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'scripts'));
setupTrackbench(repoRoot);

fprintf('=== Loader Expand Test ===\n');
pass = 0; fail = 0;

try
    cfg = trackbench.loader.ingestConfig("scenarios/my_test");
    cfg = trackbench.loader.normalizeConfig(cfg);
    plan = trackbench.loader.expandRunPlan(cfg);
    assert(numel(plan) == 1);
    assert(isfield(plan, 'run_id'));
    fprintf('  [PASS] non-sweep expands to single run\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] single expansion: %s\n', ME.message); fail = fail + 1;
end

try
    cfg = trackbench.loader.ingestConfig("sweeps/weather_study");
    trackbench.loader.validateConfig(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    plan = trackbench.loader.expandRunPlan(cfg);
    assert(numel(plan) == 3);
    fprintf('  [PASS] sweep expanded to 3 runs\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] sweep expansion: %s\n', ME.message); fail = fail + 1;
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoaderExpand failed'); end
