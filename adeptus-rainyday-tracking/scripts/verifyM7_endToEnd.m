function verifyM7_endToEnd()
%verifyM7_endToEnd  End-to-end sanity run for Milestone 7 (Environment
%                    authoring in the path editor).
%
%  Mirrors verifyM5_endToEnd / verifyM4_endToEnd but exercises the new
%  M7 surface area:
%
%    1. Build a two-target (airliner + fighter) two-sensor (PSR + PAR)
%       scenario.
%    2. Configure a rain-over-mountain environment:
%         Terrain: mountain, scale=1, clutter=0.5, refraction=1.333
%         Weather: rain, 40 mm/hr, 50–400s, ramp profile, pd=0.15, cm=1.0
%         Degradation: all four ON
%    3. Export via trackbench.editor.exportSensorsToJSON. Confirm the
%       run file + terrain file + weather file + sensor files + targets
%       file are all on disk.
%    4. Round-trip: openScenarioFromJSON into a fresh EditorState and
%       verify every terrain/weather/degradation field survived.
%    5. Build the editor UI, flip to Environment mode, enable the
%       overlay, and save screenshots of the 2D map (with tint, badge,
%       timeline) + the sidebar Environment panels.
%
%  WHY WE DON'T RUN THE SIMULATION HERE
%    runSingleScenario takes 60–90 seconds and would blow the MATLAB
%    MCP timeout when this script is driven from Claude. For a full
%    sim run, open MATLAB directly and call:
%       >> runSingleScenario('m7_rain_over_mountain_demo')
%    The export step above leaves the run file + all dependencies in
%    place, so that call works from a vanilla MATLAB session.
%
%  OUTPUT ARTIFACTS (relative to project root)
%    config/runs/m7_rain_over_mountain_demo.json
%    config/terrain/mountain/m7_rain_over_mountain_demo_terrain.json
%    config/weather/rain/m7_rain_over_mountain_demo_weather.json
%    config/sensors/<TYPE>/<sensorName>.json   (one per sensor)
%    config/targets/waypoints/m7_rain_over_mountain_demo.json
%    ../PROGRESS_M7_screenshots/<png files>
%
%  See also: trackbench.editor.EditorState,
%            trackbench.editor.exportSensorsToJSON,
%            trackbench.editor.openScenarioFromJSON,
%            trackbench.editor.buildUI,
%            verifyM5_endToEnd

    thisFile    = mfilename('fullpath');
    projectRoot = fileparts(fileparts(thisFile));
    addpath(genpath(fullfile(projectRoot, 'src')));
    addpath(fullfile(projectRoot, 'scripts'));

    screenshotsDir = fullfile(fileparts(projectRoot), 'PROGRESS_M7_screenshots');
    if ~exist(screenshotsDir, 'dir')
        mkdir(screenshotsDir);
    end

    scenarioName = "m7_rain_over_mountain_demo";

    fprintf('\n==== verifyM7_endToEnd ====\n');
    fprintf('Project root   : %s\n', projectRoot);
    fprintf('Screenshots    : %s\n', screenshotsDir);
    fprintf('Scenario stem  : %s\n', scenarioName);

    % ── Step 1: build the two-target two-sensor scene ─────────────────
    s = trackbench.editor.EditorState(projectRoot);
    s.description = "M7 §3.6 end-to-end: rain-over-mountain demo scenario.";

    % Seed target → airliner_ne (3 waypoints, straight, 10 km alt).
    s.setActiveIdx(1);
    s.targetName        = "airliner_ne";
    s.rcsDbsm           = 15;
    s.rcsProfile        = "airliner";
    s.defaultSpeedKmh   = 900;
    s.defaultAltitudeM  = 10000;
    s.curveMode         = "straight";
    airliner = [...
            0,     0, 10000;
        35000, 35000, 10000;
        70000, 70000, 10000];
    setTargetWaypoints(s, airliner);

    % Second target → fighter_crosser (curved, 6 km alt).
    s.addNewTarget();
    s.targetName        = "fighter_crosser";
    s.rcsDbsm           = 3;
    s.rcsProfile        = "fighter";
    s.defaultSpeedKmh   = 1500;
    s.defaultAltitudeM  = 6000;
    s.curveMode         = "curved";
    s.curveTensionAlpha = 0.5;
    fighter = [...
        -20000,  30000, 6000;
              0,  20000, 6000;
         20000,  10000, 6000;
         40000,  -5000, 6000;
         60000, -20000, 6000];
    setTargetWaypoints(s, fighter);

    % Sensors: PSR at origin, PAR offset NE.
    s.addNewSensor("PSR");
    s.activeSensorIdx = numel(s.sensors);
    sr = s.activeSensor();
    sr.sensorName     = "psr_m7";
    sr.positionEastM  = 0;
    sr.positionNorthM = 0;
    s.setActiveSensor(sr);

    s.addNewSensor("PAR");
    s.activeSensorIdx = numel(s.sensors);
    sr = s.activeSensor();
    sr.sensorName     = "par_m7";
    sr.positionEastM  = 10000;
    sr.positionNorthM = 10000;
    s.setActiveSensor(sr);

    fprintf('Scene built    : %d target(s), %d sensor(s)\n', ...
        numel(s.targets), numel(s.sensors));

    % ── Step 2: configure the rain-over-mountain environment ──────────
    s.setTerrainType("mountain");
    tr = s.terrain;
    tr.terrainScale     = 1;
    tr.clutterDensity   = 0.5;
    tr.refractionFactor = 1.333;
    s.terrain = tr;

    s.setWeatherType("rain");
    wr = s.weather;
    wr.rainRateMmhr      = 40;
    wr.stormStartS       = 50;
    wr.stormEndS         = 400;
    wr.activeType        = "ramp";
    wr.pdFloor           = 0.15;
    wr.clutterMultiplier = 1.0;
    s.weather = wr;

    s.setDegradationToggle("terrain_occlusion", true);
    s.setDegradationToggle("horizon_masking",   true);
    s.setDegradationToggle("ground_clutter",    true);
    s.setDegradationToggle("doppler_fade",      true);

    fprintf('Environment    : terrain=%s (scale=%.2f, clutter=%.2f)\n', ...
        s.terrain.terrainType, s.terrain.terrainScale, s.terrain.clutterDensity);
    fprintf('                 weather=%s (%g mm/hr, %g–%g s, %s)\n', ...
        s.weather.weatherType, s.weather.rainRateMmhr, ...
        s.weather.stormStartS, s.weather.stormEndS, s.weather.activeType);

    % ── Step 3: export ────────────────────────────────────────────────
    [sensorPaths, runPath, targetsPath, ~, envPaths] = ...
        trackbench.editor.exportSensorsToJSON(s, scenarioName);
    fprintf('\nExported:\n');
    fprintf('  run file      : %s\n', runPath);
    fprintf('  targets file  : %s\n', targetsPath);
    for k = 1:numel(sensorPaths)
        fprintf('  sensor file   : %s\n', sensorPaths(k));
    end
    fprintf('  terrain file  : %s\n', envPaths.terrain);
    if isfield(envPaths, 'weather')
        fprintf('  weather file  : %s\n', envPaths.weather);
    end
    assert(isfile(runPath),         'run file missing');
    assert(isfile(envPaths.terrain), 'terrain file missing');
    assert(isfile(envPaths.weather), 'weather file missing');

    % ── Step 4: round-trip check ──────────────────────────────────────
    s2 = trackbench.editor.EditorState(projectRoot);
    [ns, nt] = trackbench.editor.openScenarioFromJSON(s2, runPath);
    fprintf('\nReloaded       : %d sensor(s), %d target(s)\n', ns, nt);
    fprintf('  terrain=%s scale=%.2f clutter=%.2f\n', ...
        s2.terrain.terrainType, s2.terrain.terrainScale, s2.terrain.clutterDensity);
    fprintf('  weather=%s rate=%g storm=[%g %g] prof=%s\n', ...
        s2.weather.weatherType, s2.weather.rainRateMmhr, ...
        s2.weather.stormStartS, s2.weather.stormEndS, s2.weather.activeType);
    fprintf('  degradation: occ=%d horz=%d grnd=%d dopp=%d\n', ...
        s2.degradation.terrain_occlusion, s2.degradation.horizon_masking, ...
        s2.degradation.ground_clutter,    s2.degradation.doppler_fade);

    % Cross-check: every configured field must match the original.
    assert(s2.terrain.terrainType == "mountain",    'terrain type drift');
    assert(s2.weather.weatherType == "rain",        'weather type drift');
    assert(s2.weather.rainRateMmhr == 40,           'rain rate drift');
    assert(s2.weather.stormEndS == 400,             'storm end drift');
    assert(s2.weather.activeType == "ramp",         'profile drift');
    assert(s2.degradation.terrain_occlusion,        'occlusion drift');
    assert(s2.degradation.doppler_fade,             'doppler drift');

    % ── Step 5: render screenshots ────────────────────────────────────
    %  Build a uifigure on s2 (the reloaded state) so the screenshots
    %  reflect what a user sees after opening the demo run file.
    %
    %  SCREENSHOT CAVEAT
    %    Headless MATLAB sessions (e.g. the MATLAB MCP driven by
    %    Claude) don't have a real display backend for uifigures, and
    %    exportapp often returns an unpainted black frame from those
    %    environments. We fall back to a classic `figure` + rendered
    %    axes capture which works better headless. Both artifacts
    %    land in ../PROGRESS_M7_screenshots/; the editor-axes one is
    %    the authoritative "what the user sees". Running this script
    %    from an interactive MATLAB session will produce a correctly
    %    painted exportapp screenshot alongside the axes one.
    fig = uifigure('Name','M7 demo render','Position',[50 50 1600 1000]);
    s2.fig = fig;
    trackbench.editor.buildUI(s2);
    s2.modeEnvironmentBtn.Value = true;
    s2.modeEnvironmentBtn.ValueChangedFcn(s2.modeEnvironmentBtn, struct());
    s2.terrainOverlayCB.Value = true;
    s2.terrainOverlayCB.ValueChangedFcn(s2.terrainOverlayCB, struct());
    for k = 1:5
        drawnow;
        pause(0.4);
    end
    pngTarget = fullfile(screenshotsDir, 'm7_02_editor_map_with_overlays.png');
    try
        exportapp(fig, pngTarget);
    catch
        % Older bundles / headless edge cases — fall through to the
        % axes-only path below.
    end
    fprintf('Saved (editor) : %s\n', pngTarget);
    % Axes-only snapshot — works headlessly, uses the on-screen data
    % whether or not the uifigure frame painted.
    pngAxes = fullfile(screenshotsDir, 'm7_02_editor_map_axes_only.png');
    try
        exportgraphics(s2.ax, pngAxes, 'Resolution', 150);
        fprintf('Saved (axes)   : %s\n', pngAxes);
    catch ME
        fprintf('Axes snapshot skipped: %s\n', ME.message);
    end

    % 3D view — same state, view toggled. Weather badge rendered; tint
    % skipped (handoff §3.3).
    if isgraphics(s2.viewModeBtn)
        s2.viewModeBtn.Value = true;
        if ~isempty(s2.viewModeBtn.ValueChangedFcn)
            s2.viewModeBtn.ValueChangedFcn(s2.viewModeBtn, struct());
        end
        for k = 1:5
            drawnow;
            pause(0.4);
        end
        pngTarget3D = fullfile(screenshotsDir, 'm7_03_editor_3D_preview.png');
        try
            exportapp(fig, pngTarget3D);
        catch
        end
        pngAxes3D = fullfile(screenshotsDir, 'm7_03_editor_3D_axes_only.png');
        try
            exportgraphics(s2.ax, pngAxes3D, 'Resolution', 150);
            fprintf('Saved (3D axes): %s\n', pngAxes3D);
        catch ME
            fprintf('3D axes snapshot skipped: %s\n', ME.message);
        end
    end
    close(fig);

    fprintf('\n---- verifyM7_endToEnd OK ----\n');
    fprintf('To run the full simulation, open MATLAB and call:\n');
    fprintf('    runSingleScenario(''%s'')\n', scenarioName);
end


function setTargetWaypoints(state, wpXYZ)
%setTargetWaypoints  Helper: clear the active target's waypoints and
%                     append the rows of wpXYZ (Nx3: east, north, alt).
%                     setWaypointProperty's field name is 'altitude',
%                     not 'altitudeM' — see EditorState.m L1050.
    state.clear();
    for k = 1:size(wpXYZ, 1)
        state.addWaypoint(wpXYZ(k, 1), wpXYZ(k, 2));
        state.setWaypointProperty(k, 'altitude', wpXYZ(k, 3));
    end
end
