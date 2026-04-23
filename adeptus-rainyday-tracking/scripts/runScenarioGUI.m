function runScenarioGUI()
    % RUNSCENARIOGUI Launches an interactive app to configure and run a simulation.
    % Scans the config directories, populates dropdowns, builds the run JSON,
    % and immediately executes runSingleScenario.

    % Create the main UI figure
    fig = uifigure('Name', 'Rainy Day — Scenario Runner', 'Position', [100 100 800 600]);
    gl = uigridlayout(fig, [6, 3]);
    gl.RowHeight = {'1x', 30, 30, '1x', 30, 30};
    gl.ColumnWidth = {130, '1x', 250};

    % Helper function to scan subdirectories for valid .json configs
    function items = getAvailableConfigs(subDir)
        basePath = fullfile(pwd, 'config', subDir);
        files = dir(fullfile(basePath, '**', '*.json'));
        items = {};
        for i = 1:length(files)
            % Skip template and global configuration files
            if contains(files(i).name, 'template') || contains(files(i).name, 'globals')
                continue;
            end
            
            % Construct the relative path string (e.g., "PSR/default_PSR")
            folderPath = strrep(files(i).folder, basePath, '');
            if startsWith(folderPath, filesep)
                folderPath = folderPath(2:end);
            end
            folderPath = strrep(folderPath, '\', '/'); % Normalize slashes
            nameStr = strrep(files(i).name, '.json', '');
            
            if isempty(folderPath)
                items{end+1} = nameStr; %#ok<AGROW>
            else
                items{end+1} = [folderPath, '/', nameStr]; %#ok<AGROW>
            end
        end
        if isempty(items)
            items = {'default'};
        end
    end

    % Create UI components and Layout
    lbl1 = uilabel(gl, 'Text', 'Select Sensor(s):', 'VerticalAlignment', 'top');
    lbl1.Layout.Row = 1; lbl1.Layout.Column = 1;
    lbSensor = uilistbox(gl, 'Items', getAvailableConfigs('sensors'), 'Multiselect', 'on');
    lbSensor.Layout.Row = 1; lbSensor.Layout.Column = 2;
    lastSensor = cellstr(getpref('RainyDayGUI', 'Sensors', {}));
    validSensors = lastSensor(ismember(lastSensor, lbSensor.Items));
    if ~isempty(validSensors); lbSensor.Value = validSensors; end

    lbl2 = uilabel(gl, 'Text', 'Select Target/Path:');
    lbl2.Layout.Row = 2; lbl2.Layout.Column = 1;
    ddTarget = uidropdown(gl, 'Items', getAvailableConfigs('targets'));
    ddTarget.Layout.Row = 2; ddTarget.Layout.Column = 2;
    lastTarget = getpref('RainyDayGUI', 'Target', '');
    if ismember(lastTarget, ddTarget.Items); ddTarget.Value = lastTarget; end

    lbl3 = uilabel(gl, 'Text', 'Select Terrain:');
    lbl3.Layout.Row = 3; lbl3.Layout.Column = 1;
    ddTerrain = uidropdown(gl, 'Items', getAvailableConfigs('terrain'));
    ddTerrain.Layout.Row = 3; ddTerrain.Layout.Column = 2;
    lastTerrain = getpref('RainyDayGUI', 'Terrain', '');
    if ismember(lastTerrain, ddTerrain.Items); ddTerrain.Value = lastTerrain; end

    lbl4 = uilabel(gl, 'Text', 'Select Tracker(s):', 'VerticalAlignment', 'top');
    lbl4.Layout.Row = 4; lbl4.Layout.Column = 1;
    lbTracker = uilistbox(gl, 'Items', getAvailableConfigs('trackers'), 'Multiselect', 'on');
    lbTracker.Layout.Row = 4; lbTracker.Layout.Column = 2;
    lastTracker = cellstr(getpref('RainyDayGUI', 'Trackers', {}));
    validTrackers = lastTracker(ismember(lastTracker, lbTracker.Items));
    if ~isempty(validTrackers); lbTracker.Value = validTrackers; end

    lbl5 = uilabel(gl, 'Text', 'Select Weather:');
    lbl5.Layout.Row = 5; lbl5.Layout.Column = 1;
    ddWeather = uidropdown(gl, 'Items', getAvailableConfigs('weather'));
    ddWeather.Layout.Row = 5; ddWeather.Layout.Column = 2;
    lastWeather = getpref('RainyDayGUI', 'Weather', '');
    if ismember(lastWeather, ddWeather.Items); ddWeather.Value = lastWeather; end

    lbl6 = uilabel(gl, 'Text', 'Run File Name:');
    lbl6.Layout.Row = 6; lbl6.Layout.Column = 1;
    txtRunName = uieditfield(gl, 'text', 'Value', getpref('RainyDayGUI', 'RunName', 'gui_generated_run'));
    txtRunName.Layout.Row = 6; txtRunName.Layout.Column = 2;

    % Options Panel spanning the right column
    pnlOpts = uipanel(gl, 'Title', 'Scenario Options', 'FontWeight', 'bold', 'FontSize', 14, 'Scrollable', 'on');
    pnlOpts.Layout.Row = [1 6]; pnlOpts.Layout.Column = 3;
    
    glOpts = uigridlayout(pnlOpts, [17, 1]);
    glOpts.RowHeight = {22, 22, 22, 22, 22, 22, 5, 22, 22, '1x', 24, 24, 24, 24, 24, 5, 32};
    
    lblPhys = uilabel(glOpts, 'Text', 'Environment Physics:', 'FontWeight', 'bold');
    lblPhys.Layout.Row = 1;
    
    chkOcclusion = uicheckbox(glOpts, 'Text', 'Terrain Occlusion', 'Value', getpref('RainyDayGUI', 'Occlusion', true)); chkOcclusion.Layout.Row = 2;
    chkMasking   = uicheckbox(glOpts, 'Text', 'Horizon Masking', 'Value', getpref('RainyDayGUI', 'Masking', true));   chkMasking.Layout.Row = 3;
    chkClutter   = uicheckbox(glOpts, 'Text', 'Ground Clutter', 'Value', getpref('RainyDayGUI', 'Clutter', true));    chkClutter.Layout.Row = 4;
    chkDoppler   = uicheckbox(glOpts, 'Text', 'Doppler Fade', 'Value', getpref('RainyDayGUI', 'Doppler', true));      chkDoppler.Layout.Row = 5;
    chkRCS       = uicheckbox(glOpts, 'Text', 'RCS Range Filter (opt-in)', 'Value', getpref('RainyDayGUI', 'RCS', false)); chkRCS.Layout.Row = 6;
    
    lblPerf = uilabel(glOpts, 'Text', 'Performance:', 'FontWeight', 'bold');
    lblPerf.Layout.Row = 8;
    
    chkCache = uicheckbox(glOpts, 'Text', 'Use Cached Detections', 'Value', getpref('RainyDayGUI', 'Cache', false)); chkCache.Layout.Row = 9;
    
    btnRefresh = uibutton(glOpts, 'Text', 'Refresh Config Lists', ...
                          'ButtonPushedFcn', @(~,~) refreshLists());
    btnRefresh.Layout.Row = 11;
    
    btnPathEd = uibutton(glOpts, 'Text', 'Open Path Editor', ...
                         'ButtonPushedFcn', @(~,~) launchEditor());
    btnPathEd.Layout.Row = 12;

    btnCompare = uibutton(glOpts, 'Text', 'Compare Trackers', ...
                          'ButtonPushedFcn', @(~,~) compareTrackersBtnPushed());
    btnCompare.Layout.Row = 13;

    btnTune = uibutton(glOpts, 'Text', 'Auto-Tune Tracker', ...
                       'ButtonPushedFcn', @(~,~) tuneTrackerBtnPushed());
    btnTune.Layout.Row = 14;

    btnRunTests = uibutton(glOpts, 'Text', 'Run Validation Tests', ...
                       'ButtonPushedFcn', @(~,~) runTestsBtnPushed());
    btnRunTests.Layout.Row = 15;

    btnRun = uibutton(glOpts, 'Text', 'Run Simulation', ...
                      'FontWeight', 'bold', ...
                      'ButtonPushedFcn', @runSimulationBtnPushed);
    btnRun.Layout.Row = 17;

    function refreshLists()
        lbSensor.Items = getAvailableConfigs('sensors');
        ddTarget.Items = getAvailableConfigs('targets');
        ddTerrain.Items = getAvailableConfigs('terrain');
        lbTracker.Items = getAvailableConfigs('trackers');
        ddWeather.Items = getAvailableConfigs('weather');
    end

    function launchEditor()
        try pathEditor(); catch ME; uialert(fig, ME.message, 'Error Launching Editor'); end
    end

    function runName = saveRunFile()
        % Helper function to build and save the JSON for all 3 run actions.
        % Errors on fopen failure — callers wrap in try/catch and surface
        % the message via uialert. Mirrors the fopen-error pattern used
        % across the editor's export helpers (exportSensorsToJSON).
        runName = txtRunName.Value;
        runFile = fullfile(pwd, 'config', 'runs', [runName, '.json']);

        % Save GUI preferences for the next session
        setpref('RainyDayGUI', 'RunName', runName);
        setpref('RainyDayGUI', 'Target', ddTarget.Value);
        setpref('RainyDayGUI', 'Terrain', ddTerrain.Value);
        setpref('RainyDayGUI', 'Weather', ddWeather.Value);
        setpref('RainyDayGUI', 'Occlusion', chkOcclusion.Value);
        setpref('RainyDayGUI', 'Masking', chkMasking.Value);
        setpref('RainyDayGUI', 'Clutter', chkClutter.Value);
        setpref('RainyDayGUI', 'Doppler', chkDoppler.Value);
        setpref('RainyDayGUI', 'RCS', chkRCS.Value);
        setpref('RainyDayGUI', 'Cache', chkCache.Value);

        % Construct the config struct based on v3.4.2 spec
        runConfig = struct();
        runConfig.description = 'Auto-generated run from GUI';
        
        sensors = cellstr(lbSensor.Value);
        if isempty(sensors); sensors = {}; end
        setpref('RainyDayGUI', 'Sensors', sensors);
        runConfig.sensors = sensors;
        
        runConfig.targets = ddTarget.Value;
        runConfig.terrain = ddTerrain.Value;
        
        trackers = cellstr(lbTracker.Value);
        if isempty(trackers); trackers = {}; end
        setpref('RainyDayGUI', 'Trackers', trackers);
        runConfig.trackers = trackers;
        runConfig.compare_trackers = trackers; % Needed for Compare Trackers
        
        % Add physical degradation toggles and weather setup
        runConfig.degradation = struct('terrain_occlusion', chkOcclusion.Value, 'horizon_masking', chkMasking.Value, ...
            'ground_clutter', chkClutter.Value, 'doppler_fade', chkDoppler.Value, 'rcs_range_filter', chkRCS.Value, 'weather', ddWeather.Value);
        runConfig.cache = struct('use_cached_detections', chkCache.Value, 'save_detections', true);
        runConfig.platforms = struct();
        runConfig.output = struct('show_visuals', true, 'animate_visuals', true, 'save_results', true);

        % Save the constructed JSON file. Check fopen explicitly so a
        % locked file or missing config/runs/ directory surfaces a clear
        % error instead of a cryptic "Invalid file identifier" from the
        % subsequent fprintf. onCleanup guarantees the handle closes on
        % error paths too.
        runDir = fileparts(runFile);
        if ~exist(runDir, 'dir')
            error('runScenarioGUI:saveRunFile:noRunsDir', ...
                'config/runs/ directory not found: %s', runDir);
        end
        fid = fopen(runFile, 'w');
        if fid < 0
            error('runScenarioGUI:saveRunFile:openFailed', ...
                ['Could not open %s for writing.\n' ...
                 'The file may be locked by another program, or the ' ...
                 'config/runs/ directory may not be writable.'], runFile);
        end
        closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
        fprintf(fid, '%s', jsonencode(runConfig, 'PrettyPrint', true));
    end

    function runSimulationBtnPushed(~, ~)
        try
            runName = saveRunFile();
        catch ME
            uialert(fig, ME.message, 'Error Saving Run File');
            return;
        end
        
        % Suppress the chained-run console pause prompt for GUI execution
        setappdata(0, 'trackbench_suppressPause', true);
        try
            runSingleScenario(runName);
        catch ME
            setappdata(0, 'trackbench_suppressPause', false);
            rethrow(ME);
        end
        setappdata(0, 'trackbench_suppressPause', false);
    end

    function compareTrackersBtnPushed(~, ~)
        try
            runName = saveRunFile();
        catch ME
            uialert(fig, ME.message, 'Error Saving Run File');
            return;
        end
        if length(cellstr(lbTracker.Value)) < 2
            uialert(fig, 'Please select at least two trackers from the list (using Ctrl+Click) to compare.', 'Not Enough Trackers');
            return;
        end
        
        if ~isfile(fullfile(pwd, 'cache', [runName, '.mat']))
            uialert(fig, 'No cached detections found. Please click "Run Simulation" once to generate the scenario environment before comparing.', 'Cache Required');
            return;
        end
        
        try compareTrackers(runName);
        catch ME; uialert(fig, ME.message, 'Error Comparing Trackers'); end
    end

    function tuneTrackerBtnPushed(~, ~)
        try
            runName = saveRunFile();
        catch ME
            uialert(fig, ME.message, 'Error Saving Run File');
            return;
        end
        trackers = cellstr(lbTracker.Value);
        if isempty(trackers)
            uialert(fig, 'Please select a tracker to tune.', 'No Tracker Selected');
            return;
        end
        
        if ~isfile(fullfile(pwd, 'cache', [runName, '.mat']))
            uialert(fig, 'No cached detections found. Please click "Run Simulation" once to generate the scenario environment before tuning.', 'Cache Required');
            return;
        end
        
        % Extract the base algorithm name (e.g., "GNN/default_GNN" -> "GNN")
        parts = split(trackers{1}, '/');
        trackerType = parts{1};
        
        try autoTuneTracker(runName, trackerType);
        catch ME; uialert(fig, ME.message, 'Error Auto-Tuning Tracker'); end
    end

    function runTestsBtnPushed()
        try runTestPlan();
        catch ME; uialert(fig, ME.message, 'Error Running Tests'); end
    end
end
