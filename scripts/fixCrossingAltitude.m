%% fixCrossingAltitude.m — Fix crossing targets to same altitude for swap demo
% Run from adeptus-rainyday-tracking root:
%   run('scripts/fixCrossingAltitude.m')
%
% WHAT: Changes layered_defense target 2 from altitude 4200 to 4000 (matches target 1),
%       and target 4 orbit altitude from 3800 to 3500 (matches target 3 s_maneuver).
%       Also widens gate from 60 to 80 for more association ambiguity.
%
% WHY: 200m altitude separation lets the tracker disambiguate crossing targets
%      trivially. Same altitude forces the tracker to rely on velocity alone,
%      which is much harder with a 4.8s scan period — causing real track swaps.

root = fileparts(fileparts(mfilename('fullpath')));
catPath = fullfile(root, 'config', 'scenarios', 'scenario_catalog.json');
fprintf('Patching: %s\n', catPath);
txt = fileread(catPath);

changed = false;

%% Fix 1: Target 2 altitude 4200 → 4000 (start_pos Z)
old1 = '"start_pos": [0, -50000, -4200]';
new1 = '"start_pos": [0, -50000, -4000]';
if contains(txt, old1)
    txt = strrep(txt, old1, new1);
    fprintf('  Fixed: target 2 start_pos Z: -4200 → -4000\n');
    changed = true;
end

%% Fix 2: Target 2 altitude 4200 → 4000 (end_pos Z)
old2 = '"end_pos": [0, -20000, -4200]';
new2 = '"end_pos": [0, -20000, -4000]';
if contains(txt, old2)
    txt = strrep(txt, old2, new2);
    fprintf('  Fixed: target 2 end_pos Z: -4200 → -4000\n');
    changed = true;
end

%% Fix 3: Target 2 altitude_m 4200 → 4000
% Only replace in the layered_defense section (after "CROSSING PAIR A")
old3 = ['"altitude_m": 4200,' newline '          "_note": "CROSSING PAIR A'];
new3 = ['"altitude_m": 4000,' newline '          "_note": "CROSSING PAIR — south to north, SAME ALTITUDE as target 1, co-located at t=40s'];
if contains(txt, old3)
    txt = strrep(txt, old3, new3);
    fprintf('  Fixed: target 2 altitude_m: 4200 → 4000\n');
    changed = true;
else
    % Try alternate phrasing (already partially patched)
    old3b = '"altitude_m": 4200';
    if contains(txt, old3b)
        % Only the first occurrence in layered_defense context
        txt = strrep(txt, old3b, '"altitude_m": 4000');
        fprintf('  Fixed: altitude_m 4200 → 4000\n');
        changed = true;
    end
end

%% Fix 4: Target 4 orbit altitude 3800 → 3500 (match target 3)
old4 = '"start_pos": [5000, -28000, -3800]';
new4 = '"start_pos": [0, -28000, -3500]';
if contains(txt, old4)
    txt = strrep(txt, old4, new4);
    fprintf('  Fixed: target 4 start_pos: [5000,-28000,-3800] → [0,-28000,-3500]\n');
    changed = true;
end

old4b = '"altitude_m": 3800';
if contains(txt, old4b)
    txt = strrep(txt, old4b, '"altitude_m": 3500');
    fprintf('  Fixed: target 4 altitude_m: 3800 → 3500\n');
    changed = true;
end

%% Fix 5: Target 3 s_maneuver start closer to orbit center
old5 = '"start_pos": [-10000, -25000, -3500]';
new5 = '"start_pos": [-5000, -28000, -3500]';
if contains(txt, old5)
    txt = strrep(txt, old5, new5);
    fprintf('  Fixed: target 3 start_pos closer to orbit center\n');
    changed = true;
end

%% Fix 6: Widen gate 60 → 80 in layered_defense
% Only change the one in layered_defense (after the datalog_file line)
old6 = ['"data_logging.datalog_file": "cache/layered_defense.mat",' newline '        "tracker_params.ideal.gate": 60'];
new6 = ['"data_logging.datalog_file": "cache/layered_defense.mat",' newline '        "tracker_params.ideal.gate": 80'];
if contains(txt, old6)
    txt = strrep(txt, old6, new6);
    fprintf('  Fixed: layered_defense gate: 60 → 80\n');
    changed = true;
end

%% Write back
if changed
    fid = fopen(catPath, 'w');
    fprintf(fid, '%s', txt);
    fclose(fid);
    fprintf('\nPATCHED. Changes:\n');
    fprintf('  - Crossing targets 1+2: SAME altitude (4000m) → tracker cannot disambiguate by Z\n');
    fprintf('  - S-maneuver + orbit (3+4): SAME altitude (3500m), orbit center 5km from maneuver start\n');
    fprintf('  - Gate widened 60→80 for more association ambiguity\n');
    fprintf('  → Track swaps expected at crossing point t≈40s\n\n');
else
    fprintf('\nNo changes needed (already patched or targets not found).\n');
end

% Clear stale cache
cacheFile = fullfile(root, 'cache', 'layered_defense.mat');
if isfile(cacheFile)
    delete(cacheFile);
    fprintf('Deleted stale cache.\n');
end

fprintf('\nRun:  runSingleScenario("layered_defense")\n');
