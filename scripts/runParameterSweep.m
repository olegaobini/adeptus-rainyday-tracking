function allResults = runParameterSweep(sweepName)
%runParameterSweep Expand and execute a sweep config.
%
% Usage:
%   runParameterSweep("sweeps/weather_study")

arguments
    sweepName (1,1) string = "sweeps/weather_study"
end

clc; close all;
ctx = setupTrackbench();
plan = trackbench.loader.loadAndPrepare(sweepName);

allResults = struct();
for i = 1:numel(plan)
    cfg = plan(i);
    fprintf('\n[%d/%d] %s\n', i, numel(plan), cfg.run_id);

    [results, detections] = trackbench.runScenario(cfg, string(cfg.run_id));

    outDir = fullfile(ctx.root, getOr(cfg, 'output.results_directory', 'outputs'));
    if ~exist(outDir, 'dir'); mkdir(outDir); end
    ts = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    outFile = fullfile(outDir, sprintf('results_%s_%s.mat', cfg.run_id, ts));
    config = cfg; %#ok<NASGU>
    save(outFile, 'results', 'config', 'detections', '-v7.3');
    fprintf('[INFO] Saved %s\n', outFile);

    key = matlab.lang.makeValidName(char(cfg.run_id));
    allResults.(key) = results;
end
end

function v = getOr(s, dotPath, fallback)
parts = strsplit(dotPath, '.');
cur = s;
for i = 1:numel(parts)
    if ~isstruct(cur) || ~isfield(cur, parts{i})
        v = fallback;
        return;
    end
    cur = cur.(parts{i});
end
v = cur;
end
