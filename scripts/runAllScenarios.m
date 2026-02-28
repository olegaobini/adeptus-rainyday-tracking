function allResults = runAllScenarios()
%runAllScenarios Run every scenario JSON in config/scenarios.

clc; close all;
ctx = setupTrackbench();

scenDir = trackbench.util.pathFromRoot("config", "scenarios");
files = dir(fullfile(scenDir, '*.json'));

names = {};
for i = 1:numel(files)
    nm = files(i).name;
    [~, b] = fileparts(nm);
    names{end+1} = b; %#ok<AGROW>
end

if isempty(names)
    fprintf('[runAll] No scenario files found in %s\n', scenDir);
    allResults = struct();
    return;
end

allResults = struct();
for i = 1:numel(names)
    cfgName = "scenarios/" + string(names{i});
    fprintf('\n[%d/%d] %s\n', i, numel(names), cfgName);

    plan = trackbench.loader.loadAndPrepare(cfgName);
    for j = 1:numel(plan)
        cfg = plan(j);
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
