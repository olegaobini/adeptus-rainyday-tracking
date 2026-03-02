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

try
    tmpSweepPath = fullfile(repoRoot, 'config', 'sweeps', 'tmp_grid_key_test.json');
    fid = fopen(tmpSweepPath, 'w');
    fprintf(fid, ['{', ...
        '"sweep_name":"tmp_grid",', ...
        '"base_scenario":"scenarios/my_test",', ...
        '"sweep":{"mode":"grid","parameters":{"trackers.params.ideal.gate":[35,45]}}', ...
        '}']);
    fclose(fid);

    cfg = trackbench.loader.ingestConfig("sweeps/tmp_grid_key_test");
    trackbench.loader.validateConfig(cfg);
    cfg = trackbench.loader.normalizeConfig(cfg);
    plan = trackbench.loader.expandRunPlan(cfg);
    assert(numel(plan) == 2);
    assert(plan(1).trackers.params.ideal.gate == 35);
    assert(plan(2).trackers.params.ideal.gate == 45);
    fprintf('  [PASS] dotted grid keys survive jsondecode mangling\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] dotted grid keys: %s\n', ME.message); fail = fail + 1;
end

if exist('tmpSweepPath', 'var') && isfile(tmpSweepPath)
    delete(tmpSweepPath);
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoaderExpand failed'); end
