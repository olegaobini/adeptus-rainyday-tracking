repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'scripts'));
setupTrackbench(repoRoot);

fprintf('=== Loader Validate Test ===\n');
pass = 0; fail = 0;

try
    cfg = trackbench.loader.ingestConfig("scenarios/my_test");
    trackbench.loader.validateConfig(cfg);
    fprintf('  [PASS] valid config passes\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] valid config: %s\n', ME.message); fail = fail + 1;
end

try
    bad = struct();
    trackbench.loader.validateConfig(bad);
    fprintf('  [FAIL] missing fields should fail\n'); fail = fail + 1;
catch
    fprintf('  [PASS] missing fields caught\n'); pass = pass + 1;
end

try
    cfg = trackbench.loader.ingestConfig("sweeps/weather_study");
    trackbench.loader.validateConfig(cfg);
    fprintf('  [PASS] sweep structure validated\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] sweep validate: %s\n', ME.message); fail = fail + 1;
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoaderValidate failed'); end
