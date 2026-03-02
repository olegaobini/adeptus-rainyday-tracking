repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'scripts'));
setupTrackbench(repoRoot);

fprintf('=== Loader Normalize Test ===\n');
pass = 0; fail = 0;

try
    cfg = trackbench.loader.ingestConfig("scenarios/my_test");
    cfg.scenario.duration_min = 2;
    cfg = trackbench.loader.normalizeConfig(cfg);
    assert(cfg.scenario.duration_s == 120);
    assert(isfield(cfg, 'active_params'));
    assert(isfield(cfg, 'tracker_global'));
    fprintf('  [PASS] duration + runtime projection normalized\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] normalize basic: %s\n', ME.message); fail = fail + 1;
end

try
    c2 = struct();
    c2.scenario = struct('frame', 'NED', 'duration_s', 1);
    c2.truth = struct('targets', {{struct('path', 'data/truths/crossing_pair.csv')}});
    c2 = trackbench.loader.normalizeConfig(c2);
    p = string(c2.truth.targets{1}.path);
    assert(startsWith(p, '/'));
    fprintf('  [PASS] relative paths converted to absolute\n'); pass = pass + 1;
catch ME
    fprintf('  [FAIL] path normalize: %s\n', ME.message); fail = fail + 1;
end

fprintf('RESULT: %d passed, %d failed\n', pass, fail);
if fail > 0, error('testLoaderNormalize failed'); end
