repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'scripts'));
setupTrackbench(repoRoot);

fprintf('=== Loader Ingest Test ===\n');
pass = 0; fail = 0;

try
    cfg = trackbench.loader.ingestConfig("scenarios/my_test");
    assert(isfield(cfg, 'scenario'));
    assert(isfield(cfg, 'sensors'));
    assert(isfield(cfg, 'trackers'));
    assert(cfg.scenario.duration_s == 30);
    fprintf('  [PASS] scenario ingest merged defaults/template\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] scenario ingest: %s\n', ME.message); fail = fail + 1;
end

try
    cfg = trackbench.loader.ingestConfig("sweeps/weather_study");
    assert(isfield(cfg, 'sweep'));
    assert(strcmp(string(cfg.sweep.mode), "single"));
    fprintf('  [PASS] sweep ingest preserved sweep definition\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] sweep ingest: %s\n', ME.message); fail = fail + 1;
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoaderIngest failed'); end
