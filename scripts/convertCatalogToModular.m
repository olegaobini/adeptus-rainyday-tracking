function convertCatalogToModular()
%convertCatalogToModular  Convert all catalog scenarios to modular run files.
%
%  Reads scenario_catalog.json and creates:
%    config/runs/showcase/<name>.json  — run file for each scenario
%    config/targets/<name>/            — target file for each unique target set
%
%  Sensor mapping: flat sensor configs → individual modular sensor references
%  Terrain mapping: catalog overrides → modular terrain files
%
%  USAGE
%    cd to adeptus-rainyday-tracking root, then:
%    addpath('scripts');
%    convertCatalogToModular

root = fileparts(fileparts(mfilename('fullpath')));
configDir = fullfile(root, 'config');
showcaseDir = fullfile(configDir, 'runs', 'showcase');
targetDir = fullfile(configDir, 'targets');

if ~exist(showcaseDir, 'dir'); mkdir(showcaseDir); end

%% Load catalog
catPath = fullfile(configDir, 'scenarios', 'scenario_catalog.json');
catalog = jsondecode(fileread(catPath));
scenNames = fieldnames(catalog.scenarios);

fprintf('\n[CONVERT] Converting %d catalog scenarios to modular run files\n\n', numel(scenNames));

%% Sensor mapping: flat config name → modular sensor references
sensorMap = struct();
sensorMap.sensors             = {{'PSR/default_PSR', 'SSR/default_SSR'}};
sensorMap.sensors_dasr        = {{'PSR/default_PSR', 'SSR/default_SSR'}};
sensorMap.sensors_demo_psr    = {{'PSR/default_PSR'}};
sensorMap.sensors_demo_dasr   = {{'PSR/default_PSR', 'SSR/default_SSR'}};
sensorMap.sensors_demo        = {{'PSR/default_PSR'}};
sensorMap.sensors_fighter     = {{'AESA/default_AESA', 'FLIR/default_FLIR'}};
sensorMap.sensors_approach    = {{'PSR/default_PSR', 'SSR/default_SSR', 'PAR/default_PAR'}};
sensorMap.sensors_long_range  = {{'ARSR/default_ARSR'}};
sensorMap.sensors_phased_array = {{'TWS/default_TWS', 'AESA/default_AESA'}};
sensorMap.sensors_ir_fusion   = {{'PSR/default_PSR', 'IRST/default_IRST'}};
sensorMap.sensors_maritime    = {{'MARITIME/default_MARITIME'}};
sensorMap.sensors_fire_control = {{'PSR/default_PSR', 'FIRE_CONTROL/default_FIRE_CONTROL'}};
sensorMap.sensors_layered_defense = {{'PSR/default_PSR', 'IRST/default_IRST', 'FIRE_CONTROL/default_FIRE_CONTROL', 'TWS/default_TWS'}};
sensorMap.sensors_default_wedge = {{'PSR/default_PSR'}};  % wedge was a special PSR config

%% Process each scenario
for i = 1:numel(scenNames)
    name = scenNames{i};
    if strcmp(name, 'default'); continue; end  % skip the "default" entry
    
    scen = catalog.scenarios.(name);
    fprintf('[CONVERT] %s\n', name);
    
    %% 1. Map sensors
    sensorCfg = 'sensors';
    if isfield(scen, 'sensor_config')
        sensorCfg = char(scen.sensor_config);
    end
    
    sensorCfgClean = strrep(sensorCfg, '-', '_');
    if isfield(sensorMap, sensorCfgClean)
        sensorRefs = sensorMap.(sensorCfgClean){1};
    else
        % Fallback: PSR+SSR
        sensorRefs = {'PSR/default_PSR', 'SSR/default_SSR'};
        fprintf('  [WARN] Unknown sensor config "%s", defaulting to PSR+SSR\n', sensorCfg);
    end
    
    %% 2. Create target file
    tgtFolderName = name;
    tgtFolder = fullfile(targetDir, tgtFolderName);
    tgtFile = fullfile(tgtFolder, sprintf('default_%s.json', name));
    
    if ~exist(tgtFolder, 'dir'); mkdir(tgtFolder); end
    
    if ~exist(tgtFile, 'file')
        % Extract duration from overrides
        duration = 50;
        if isfield(scen, 'overrides') && isstruct(scen.overrides)
            ov = scen.overrides;
            flds = fieldnames(ov);
            for f = 1:numel(flds)
                if contains(flds{f}, 'duration_s')
                    duration = ov.(flds{f});
                end
            end
        end
        
        % Build target struct
        tgtStruct = struct();
        tgtStruct.description = scen.description;
        tgtStruct.duration_s = duration;
        tgtStruct.targets = scen.targets;
        
        fid = fopen(tgtFile, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', jsonencode(tgtStruct, 'PrettyPrint', true));
        fclose(fid);
        fprintf('  Created targets: %s\n', tgtFile);
    end
    
    %% 3. Determine terrain
    terrainType = 'rural';
    if isfield(scen, 'overrides') && isstruct(scen.overrides)
        ov = scen.overrides;
        flds = fieldnames(ov);
        for f = 1:numel(flds)
            if contains(flds{f}, 'terrain_type')
                terrainType = char(ov.(flds{f}));
            end
        end
    end
    
    % Map to modular terrain file
    terrainRef = sprintf('%s/default_%s', terrainType, terrainType);
    
    %% 4. Determine degradation
    degradEnabled = false;
    degradType = 'rain';
    if isfield(scen, 'overrides') && isstruct(scen.overrides)
        ov = scen.overrides;
        flds = fieldnames(ov);
        for f = 1:numel(flds)
            if contains(flds{f}, 'degradation') && contains(flds{f}, 'enabled')
                degradEnabled = logical(ov.(flds{f}));
            end
            if contains(flds{f}, 'degradation') && contains(flds{f}, 'type')
                degradType = char(ov.(flds{f}));
            end
        end
    end
    
    %% 5. Determine platforms
    hasPlatforms = isfield(scen, 'platforms') && isstruct(scen.platforms);
    
    %% 6. Build run file
    run = struct();
    run.description = scen.description;
    run.sensors = sensorRefs;
    run.targets = sprintf('%s/default_%s', name, name);
    run.terrain = terrainRef;
    run.trackers = {'GNN/default_GNN', 'JPDA/default_JPDA'};
    run.degradation = struct('enabled', degradEnabled, 'type', degradType);
    
    if hasPlatforms
        run.platforms = scen.platforms;
    else
        run.platforms = struct('placeholder', 0);
    end
    
    run.cache = struct('use_cached_detections', true, 'save_detections', true);
    run.output = struct('show_visuals', true, 'animate_visuals', true, ...
        'save_results', true, 'print_diagnostics', true, 'results_directory', 'results');
    
    % Write run file
    runFile = fullfile(showcaseDir, sprintf('%s.json', name));
    if ~exist(runFile, 'file')
        txt = jsonencode(run, 'PrettyPrint', true);
        % Clean up placeholder
        txt = regexprep(txt, '\{\s*"placeholder":\s*0\s*\}', '{}');
        fid = fopen(runFile, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', txt);
        fclose(fid);
        fprintf('  Created run: %s\n', runFile);
    end
end

%% Write README for showcase folder
readmePath = fullfile(showcaseDir, 'README.md');
if ~exist(readmePath, 'file')
    fid = fopen(readmePath, 'w', 'n', 'UTF-8');
    fprintf(fid, '# Showcase Scenarios\n\n');
    fprintf(fid, 'These are the original catalog scenarios converted to modular run files.\n');
    fprintf(fid, 'Each is a proven, tested scenario from the Boeing demo.\n\n');
    fprintf(fid, '```matlab\n');
    fprintf(fid, 'runSingleScenario("showcase/dasr_ideal")        %%%% DASR baseline\n');
    fprintf(fid, 'runSingleScenario("showcase/crossing_targets")  %%%% Track swap test\n');
    fprintf(fid, 'runSingleScenario("showcase/demo_ideal")        %%%% Boeing demo (5 targets, mountain)\n');
    fprintf(fid, 'runSingleScenario("showcase/fighter_intercept")  %%%% AESA+FLIR airborne\n');
    fprintf(fid, '```\n\n');
    fprintf(fid, 'These use the same sensors/targets/terrain as the catalog originals\n');
    fprintf(fid, 'but through the modular system. Each component can be swapped independently.\n');
    fclose(fid);
end

%% Summary
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║         CATALOG → MODULAR CONVERSION COMPLETE           ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  Showcase runs: config/runs/showcase/                   ║\n');
fprintf('║  Target files:  config/targets/<scenario>/              ║\n');
fprintf('╠══════════════════════════════════════════════════════════╣\n');
fprintf('║  RUN:  runSingleScenario("showcase/dasr_ideal")         ║\n');
fprintf('║  RUN:  runSingleScenario("showcase/demo_ideal")         ║\n');
fprintf('║  RUN:  runSingleScenario("showcase/fighter_intercept")  ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

end