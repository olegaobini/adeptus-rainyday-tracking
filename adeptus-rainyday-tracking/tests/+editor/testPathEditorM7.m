function testPathEditorM7()
%testPathEditorM7  Programmatic tests for M7 §3.6 milestone close.
%
%  Exercises the eight end-to-end contracts for the M7 Environment mode:
%    TC-M7-01 — Environment-mode state (TerrainRecord, WeatherRecord,
%               degradation) defaults are coherent.
%    TC-M7-02 — terrainDefaults + weatherDefaults match the on-disk
%               default_<TYPE>.json library files field-for-field.
%    TC-M7-03 — Rain-over-mountain scenario round-trips export → open
%               with every terrain/weather/degradation field preserved.
%    TC-M7-04 — Fog weather OMITS clutter_multiplier on disk (matches
%               default_fog.json / default_icing.json); round-trip
%               preserves the omission.
%    TC-M7-05 — UNKNOWN-passthrough terrain + weather round-trips with
%               readOnly=true and originalDef intact.
%    TC-M7-06 — rcs_range_filter (extra degradation key) round-trips
%               via state.degradationExtras verbatim passthrough — the
%               handoff promise that Open Scenario → Export Scenario
%               doesn't strip unknown degradation keys.
%    TC-M7-07 — Environment callbacks push exactly ONE undo slot per
%               edit (double-pushUndo regression from check-in #1).
%    TC-M7-08 — Storm-window validation: setting stormEnd < stormStart
%               clamps the end to start+1 (§3.5 interaction polish).
%
%  USAGE
%    addpath("tests");
%    editor.testPathEditorM7
%
%  Tests are programmatic — most cases stand up a fresh EditorState and
%  exercise the unit under test directly. TC-M7-07 and TC-M7-08 need
%  a uifigure to exercise the widget callbacks; those cases build a
%  hidden figure and close it on the way out.
%
%  See also: editor.testPathEditorM6,
%            trackbench.editor.EditorState,
%            trackbench.editor.exportSensorsToJSON,
%            trackbench.editor.openScenarioFromJSON,
%            trackbench.editor.loadTerrainFromJSON,
%            trackbench.editor.loadWeatherFromJSON

    thisFile    = mfilename('fullpath');
    projectRoot = fileparts(fileparts(fileparts(thisFile)));
    addpath(genpath(fullfile(projectRoot, 'src')));

    fprintf('\n==== testPathEditorM7 (§3.6 milestone close) ====\n');
    nPass = 0; nFail = 0;

    % Ephemeral sandbox so we never touch the real config/ tree.
    tmpRoot = tempname;
    mkdir(tmpRoot);
    mkdir(fullfile(tmpRoot, 'config', 'sensors'));
    mkdir(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
    mkdir(fullfile(tmpRoot, 'config', 'runs'));
    mkdir(fullfile(tmpRoot, 'config', 'terrain'));
    mkdir(fullfile(tmpRoot, 'config', 'weather'));
    tmpCleaner = onCleanup(@() rmdirSafe(tmpRoot));

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-01 — Environment-mode state defaults are coherent
    %% ────────────────────────────────────────────────────────────────
    try
        s = trackbench.editor.EditorState(string(tmpRoot));
        assert(~isempty(s.terrain), ...
            'state.terrain should NEVER be empty (1x1 record always)');
        assert(s.terrain.terrainType == "rural", ...
            sprintf('default terrainType should be rural, got %s', s.terrain.terrainType));
        assert(isempty(s.weather), ...
            '(none) is the default — state.weather starts 1x0 empty');
        assert(all([s.degradation.terrain_occlusion, s.degradation.horizon_masking, ...
                    s.degradation.ground_clutter,    s.degradation.doppler_fade]), ...
            'all four degradation toggles default to true');
        assert(isempty(fieldnames(s.degradationExtras)), ...
            'degradationExtras should start empty');
        % Setting weather then clearing returns empty (sentinel contract).
        s.setWeatherType("rain");
        assert(~isempty(s.weather));
        s.setWeatherType("none");
        assert(isempty(s.weather), ...
            'setWeatherType("none") must restore the empty sentinel');
        ok(1);
    catch err
        fail(1, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-02 — Defaults match disk library files
    %% ────────────────────────────────────────────────────────────────
    try
        terrainTypes = ["water","rural","urban","mountain","desert"];
        for tt = terrainTypes
            diskDef = jsondecode(fileread(fullfile(projectRoot, ...
                'config', 'terrain', char(tt), sprintf('default_%s.json', tt))));
            fresh = trackbench.editor.terrainDefaults(tt);
            assertApprox(fresh.terrainScale,     diskDef.terrain_scale,     0, ...
                sprintf('%s terrain_scale', tt));
            assertApprox(fresh.clutterDensity,   diskDef.clutter_density,   0, ...
                sprintf('%s clutter_density', tt));
            assertApprox(fresh.refractionFactor, diskDef.refraction_factor, 0, ...
                sprintf('%s refraction_factor', tt));
            assert(string(fresh.description) == string(diskDef.description), ...
                sprintf('%s description drift: "%s" vs "%s"', tt, ...
                    fresh.description, string(diskDef.description)));
        end
        weatherTypes = ["rain","snow","fog","icing"];
        for wt = weatherTypes
            diskDef = jsondecode(fileread(fullfile(projectRoot, ...
                'config', 'weather', char(wt), sprintf('default_%s.json', wt))));
            fresh = trackbench.editor.weatherDefaults(wt);
            assertApprox(fresh.rainRateMmhr, diskDef.rain_rate_mmhr, 0, ...
                sprintf('%s rain_rate_mmhr', wt));
            assertApprox(fresh.stormStartS,  diskDef.storm_start_s,  0, ...
                sprintf('%s storm_start_s', wt));
            assertApprox(fresh.stormEndS,    diskDef.storm_end_s,    0, ...
                sprintf('%s storm_end_s', wt));
            assertApprox(fresh.pdFloor,      diskDef.pd_floor,       0, ...
                sprintf('%s pd_floor', wt));
        end
        ok(2);
    catch err
        fail(2, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-03 — Rain-over-mountain round-trip
    %% ────────────────────────────────────────────────────────────────
    try
        s = buildSandboxState(tmpRoot);
        % Configure a rain-over-mountain scenario, edited from defaults.
        s.setTerrainType("mountain");
        tr = s.terrain;
        tr.terrainScale = 1.5; tr.clutterDensity = 0.6; s.terrain = tr;
        s.setWeatherType("rain");
        wr = s.weather;
        wr.rainRateMmhr = 40; wr.stormStartS = 50; wr.stormEndS = 400;
        wr.activeType = "ramp"; s.weather = wr;
        s.setDegradationToggle("doppler_fade", false);
        [~, runP, ~, ~, ~] = trackbench.editor.exportSensorsToJSON(s, "tc_m7_03");
        % Load back into a fresh state.
        s2 = trackbench.editor.EditorState(string(tmpRoot));
        s2.projectRoot = string(tmpRoot);
        trackbench.editor.openScenarioFromJSON(s2, char(runP));
        assert(s2.terrain.terrainType == "mountain",  'terrain type drift');
        assertApprox(s2.terrain.terrainScale, 1.5, 0, 'terrain scale drift');
        assertApprox(s2.terrain.clutterDensity, 0.6, 0, 'clutter density drift');
        assert(~isempty(s2.weather),                     'weather lost on reload');
        assert(s2.weather.weatherType == "rain",         'weather type drift');
        assertApprox(s2.weather.rainRateMmhr, 40, 0,    'rain rate drift');
        assertApprox(s2.weather.stormStartS,  50, 0,    'storm start drift');
        assertApprox(s2.weather.stormEndS,    400, 0,   'storm end drift');
        assert(s2.weather.activeType == "ramp",          'active_type drift');
        assert(~s2.degradation.doppler_fade,             'doppler_fade drift');
        assert(s2.degradation.terrain_occlusion,         'terrain_occlusion drift');
        ok(3);
    catch err
        fail(3, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-04 — Fog omits clutter_multiplier on disk
    %% ────────────────────────────────────────────────────────────────
    try
        s = buildSandboxState(tmpRoot);
        s.setWeatherType("fog");
        assert(~s.weather.emitsClutterField(), ...
            'Fog weather should emitsClutterField()==false');
        [~, runP, ~, ~, envP] = ...
            trackbench.editor.exportSensorsToJSON(s, "tc_m7_04_fog");
        fogJson = jsondecode(fileread(char(envP.weather)));
        assert(~isfield(fogJson, 'clutter_multiplier'), ...
            'Fog weather JSON must omit clutter_multiplier');
        % Round-trip should still succeed and still omit on re-export.
        s2 = trackbench.editor.EditorState(string(tmpRoot));
        s2.projectRoot = string(tmpRoot);
        trackbench.editor.openScenarioFromJSON(s2, char(runP));
        [~, ~, ~, ~, envP2] = ...
            trackbench.editor.exportSensorsToJSON(s2, "tc_m7_04_fog2");
        fogJson2 = jsondecode(fileread(char(envP2.weather)));
        assert(~isfield(fogJson2, 'clutter_multiplier'), ...
            'Fog round-trip re-export must still omit clutter_multiplier');
        ok(4);
    catch err
        fail(4, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-05 — UNKNOWN passthrough round-trip (terrain + weather)
    %% ────────────────────────────────────────────────────────────────
    try
        % Write an unsupported-type terrain + weather file into the
        % sandbox and load them.
        unkTerrainDir = fullfile(tmpRoot, 'config', 'terrain', 'plasma');
        unkWeatherDir = fullfile(tmpRoot, 'config', 'weather', 'hurricane');
        mkdir(unkTerrainDir); mkdir(unkWeatherDir);
        unkTerrainPath = fullfile(unkTerrainDir, 'exotic.json');
        unkWeatherPath = fullfile(unkWeatherDir, 'cat5.json');
        fid = fopen(unkTerrainPath, 'w');
        fwrite(fid, '{"terrain_type":"plasma","description":"Exotic","terrain_scale":2,"clutter_density":0.7,"refraction_factor":1.5,"_custom":"preserved"}');
        fclose(fid);
        fid = fopen(unkWeatherPath, 'w');
        fwrite(fid, '{"type":"hurricane","description":"cat 5","rain_rate_mmhr":200,"storm_start_s":0,"storm_end_s":3600,"active_type":"ramp","pd_floor":0.02,"_wind_kts":140}');
        fclose(fid);
        % Load each into a TerrainRecord / WeatherRecord.
        trUnk = trackbench.editor.loadTerrainFromJSON(string(tmpRoot), ...
            "plasma/exotic");
        wrUnk = trackbench.editor.loadWeatherFromJSON(string(tmpRoot), ...
            "hurricane/cat5");
        assert(trUnk.readOnly,  'UNK terrain must load readOnly=true');
        assert(wrUnk.readOnly,  'UNK weather must load readOnly=true');
        assert(isfield(trUnk.originalDef, 'x_custom'),  ...
            'UNK terrain originalDef should preserve _custom field');
        assert(isfield(wrUnk.originalDef, 'x_wind_kts'), ...
            'UNK weather originalDef should preserve _wind_kts field');
        ok(5);
    catch err
        fail(5, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-06 — rcs_range_filter extras passthrough
    %% ────────────────────────────────────────────────────────────────
    try
        s = buildSandboxState(tmpRoot);
        s.setTerrainType("rural");
        s.degradationExtras.rcs_range_filter = 0.07;
        [~, runP, ~, ~, ~] = ...
            trackbench.editor.exportSensorsToJSON(s, "tc_m7_06");
        % Decode run file — degradation block should carry the extra.
        runDef = jsondecode(fileread(char(runP)));
        assertApprox(runDef.degradation.rcs_range_filter, 0.07, 0, ...
            'rcs_range_filter extras key did not survive export');
        % Round-trip open should carry it into state.degradationExtras.
        s2 = trackbench.editor.EditorState(string(tmpRoot));
        s2.projectRoot = string(tmpRoot);
        trackbench.editor.openScenarioFromJSON(s2, char(runP));
        assert(isfield(s2.degradationExtras, 'rcs_range_filter'), ...
            'rcs_range_filter must round-trip into degradationExtras');
        assertApprox(s2.degradationExtras.rcs_range_filter, 0.07, 0, ...
            'rcs_range_filter value drift');
        ok(6);
    catch err
        fail(6, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-07 — Double-pushUndo regression (one slot per edit)
    %% ────────────────────────────────────────────────────────────────
    %  Hunt for the bug caught at check-in #1: setters in EditorState
    %  push undo, and callbacks in buildUI MUST NOT push again.
    try
        s = trackbench.editor.EditorState(string(tmpRoot));
        s.projectRoot = string(tmpRoot);
        s.addNewTarget('probe');
        fig = uifigure('Visible','off','Position',[100 100 1600 1000]);
        s.fig = fig;
        figCleaner = onCleanup(@() close(fig));
        trackbench.editor.buildUI(s);
        s.modeEnvironmentBtn.Value = true;
        s.modeEnvironmentBtn.ValueChangedFcn(s.modeEnvironmentBtn, struct());

        n0 = numel(s.undoStack);
        % Terrain type (fresh rural → desert, no divergence → no uiconfirm).
        s.terrainTypeDD.Value = 'desert';
        s.terrainTypeDD.ValueChangedFcn(s.terrainTypeDD, struct());
        assert(numel(s.undoStack) - n0 == 1, ...
            sprintf('terrain type push delta = %d (expect 1)', ...
                numel(s.undoStack) - n0));
        n0 = numel(s.undoStack);
        % Terrain field
        s.terrainScaleField.Value = 1.75;
        s.terrainScaleField.ValueChangedFcn(s.terrainScaleField, struct());
        assert(numel(s.undoStack) - n0 == 1, 'terrain field push != 1');
        n0 = numel(s.undoStack);
        % Degradation toggle
        s.degTerrainOcclusionCB.Value = false;
        s.degTerrainOcclusionCB.ValueChangedFcn(s.degTerrainOcclusionCB, struct());
        assert(numel(s.undoStack) - n0 == 1, 'deg toggle push != 1');
        n0 = numel(s.undoStack);
        % Weather type (from (none) → rain, no prior weather, no prompt).
        s.weatherTypeDD.Value = 'rain';
        s.weatherTypeDD.ValueChangedFcn(s.weatherTypeDD, struct());
        assert(numel(s.undoStack) - n0 == 1, 'weather type push != 1');
        n0 = numel(s.undoStack);
        % Weather field
        s.weatherRateField.Value = 33;
        s.weatherRateField.ValueChangedFcn(s.weatherRateField, struct());
        assert(numel(s.undoStack) - n0 == 1, 'weather field push != 1');
        % Single undo round-trip: Ctrl+Z one edit must revert it.
        pre = s.weather.rainRateMmhr;
        s.weatherRateField.Value = 99;
        s.weatherRateField.ValueChangedFcn(s.weatherRateField, struct());
        s.undo();
        assert(s.weather.rainRateMmhr == pre, ...
            'single undo did not revert weather field — double-push likely');
        ok(7);
    catch err
        fail(7, err);
    end

    %% ────────────────────────────────────────────────────────────────
    %  TC-M7-08 — Storm-window clamp (§3.5)
    %% ────────────────────────────────────────────────────────────────
    try
        s = trackbench.editor.EditorState(string(tmpRoot));
        s.projectRoot = string(tmpRoot);
        s.addNewTarget('probe');
        fig = uifigure('Visible','off','Position',[100 100 1600 1000]);
        s.fig = fig;
        figCleaner = onCleanup(@() close(fig));
        trackbench.editor.buildUI(s);
        s.modeEnvironmentBtn.Value = true;
        s.modeEnvironmentBtn.ValueChangedFcn(s.modeEnvironmentBtn, struct());

        s.setWeatherType("rain");
        % Rain defaults: start=50, end=130. Try setting end=10 — must clamp.
        s.weatherStormEndField.Value = 10;
        s.weatherStormEndField.ValueChangedFcn(s.weatherStormEndField, struct());
        assert(s.weather.stormEndS == 51, ...
            sprintf('end-before-start clamp failed: got %g (expect 51)', ...
                s.weather.stormEndS));
        % Bump start past end — end should auto-follow.
        s.weatherStormStartField.Value = 100;
        s.weatherStormStartField.ValueChangedFcn(s.weatherStormStartField, struct());
        assert(s.weather.stormEndS == 101, ...
            sprintf('start-past-end bump failed: end=%g (expect 101)', ...
                s.weather.stormEndS));
        ok(8);
    catch err
        fail(8, err);
    end

    %% ────────────────────────────────────────────────────────────────
    fprintf('\n---- Summary: %d pass, %d fail ----\n', nPass, nFail);
    if nFail > 0
        error('testPathEditorM7:failures', ...
            'testPathEditorM7: %d test(s) failed', nFail);
    end

    function ok(k)
        nPass = nPass + 1;
        fprintf('  [TC-M7-%02d] PASS\n', k);
    end
    function fail(k, err)
        nFail = nFail + 1;
        fprintf('  [TC-M7-%02d] FAIL  %s\n', k, err.message);
        for ii = 1:min(numel(err.stack), 3)
            fprintf('            at %s:%d\n', err.stack(ii).name, err.stack(ii).line);
        end
    end
end


%% ========================================================================
%  File-scope helpers
%% ========================================================================
function s = buildSandboxState(tmpRoot)
%buildSandboxState  Shared setup for the round-trip tests. Stands up an
%                    EditorState rooted at the sandbox with one target
%                    + one sensor — the minimum needed for
%                    exportSensorsToJSON to succeed.
    s = trackbench.editor.EditorState(string(tmpRoot));
    s.projectRoot = string(tmpRoot);
    s.outputDir   = string(fullfile(tmpRoot, 'config', 'targets', 'waypoints'));
    % Remove the auto-created blank target, then add a real one.
    while numel(s.targets) > 0
        s.deleteActiveTarget();
        if numel(s.targets) == 0; break; end
    end
    s.addNewTarget('airliner');
    s.addWaypoint(0, 0);
    s.addWaypoint(50000, 20000);
    s.addNewSensor('PSR');
    s.activeSensorIdx = numel(s.sensors);
    sr = s.activeSensor();
    sr.sensorName = "tc_m7_psr";
    s.setActiveSensor(sr);
end


function assertApprox(actual, expected, tol, msg)
    if tol == 0
        okFlag = actual == expected;
    else
        okFlag = abs(actual - expected) <= tol;
    end
    if ~okFlag
        error('testPathEditorM7:approx', ...
            '%s: actual=%.12g expected=%.12g tol=%.3g', ...
            msg, actual, expected, tol);
    end
end


function rmdirSafe(path)
    try
        if exist(path, 'dir')
            rmdir(path, 's');
        end
    catch
    end
end
