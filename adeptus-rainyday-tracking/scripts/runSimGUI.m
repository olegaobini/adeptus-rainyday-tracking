function varargout = runSimGUI(projectRoot)
%runSimGUI  Level 3 of mainMenu — load, edit, and run an existing run file.
%
%  Provides a focused interface to:
%    - Pick an existing config/runs/<runName>.json from a dropdown
%    - Edit its trackers, degradation toggles, weather, cache flag,
%      and description
%    - Save those changes back to the run file
%    - Run / Compare / Auto-Tune the loaded scenario
%
%  DIRTY-TRACKING SEMANTICS
%    A snapshot of the loaded run file is kept in originalConfig. Any UI
%    change is compared to that snapshot. If they differ, isDirty=true and
%    the "Save Changes to Run File" button enables. Saving rewrites the
%    file in place and re-snapshots, returning the editor to clean.
%
%    Run / Compare / Auto-Tune always commit pending edits before
%    executing — the on-disk file matches what was actually run, so the
%    detection cache (keyed by run name) stays deterministic across
%    sessions.
%
%    Switching run files (via dropdown), Reload, Refresh, or closing the
%    window with unsaved changes triggers a Save / Discard / Cancel confirm.
%
%  SMART CACHE BEHAVIOR
%    After a successful Run, the cache exists for the configuration that
%    was just executed. The GUI auto-checks "Use Cached Detections" and
%    persists ONLY that flag to disk, so subsequent Run/Compare/Tune
%    calls hit the cache for free without the user having to remember.
%
%    The cache is invalidated by any change to the 5 degradation toggles
%    or the weather dropdown — those affect detection generation. When
%    the user toggles one of these while "Use Cached Detections" is
%    active, a confirm dialog appears: agree and the cache flag flips
%    off automatically; cancel and the widget reverts to its prior value.
%
%    Tracker selection and description do NOT invalidate the cache (they
%    don't affect detection generation), so editing them never prompts.
%
%  PRESERVATION OF UNTOUCHED FIELDS
%    The GUI exposes a subset of run-file fields. Anything outside that set
%    (sensors, targets, terrain, output, platforms, custom keys) is
%    preserved verbatim from originalConfig on save. buildCurrentConfig
%    overlays the GUI-controlled fields onto a copy of originalConfig.
%
%  USAGE
%      runSimGUI;                  % standalone — resolves projectRoot
%      runSimGUI(projectRoot);     % caller (mainMenu) supplies the root
%
%  See also: runSingleScenario, compareTrackers, autoTuneTracker, mainMenu

    % ── Resolve projectRoot (deployed-safe) ─────────────────────────
    if nargin < 1 || isempty(projectRoot)
        if isdeployed
            projectRoot = pwd;
        else
            projectRoot = fileparts(fileparts(mfilename('fullpath')));
        end
    end
    projectRoot = char(projectRoot);

    % ── Singleton: raise existing window instead of opening another ──
    %  Without this, every click of the Main Menu's "Run Simulation"
    %  button (or every console call) would spawn a fresh uifigure,
    %  fragmenting state and leaving orphans.
    existing = findall(groot, 'Type', 'figure', 'Name', 'Rainy Day — Run Simulation');
    if ~isempty(existing)
        existing = existing(arrayfun(@isvalid, existing));
    end
    if ~isempty(existing)
        figure(existing(1));
        if nargout > 0, varargout{1} = existing(1); end
        return;
    end

    % ── Closure-shared state ────────────────────────────────────────
    currentRunName = '';
    originalConfig = struct();
    isDirty        = false;

    % ── Build figure ────────────────────────────────────────────────
    fig = uifigure('Name', 'Rainy Day — Run Simulation', ...
                   'Position', [200 150 760 580]);
    fig.CloseRequestFcn = @(~,~) onCloseRequest();

    gl = uigridlayout(fig, [5 1]);
    gl.RowHeight   = {40, 40, '1x', 36, 40};
    gl.ColumnWidth = {'1x'};
    gl.RowSpacing  = 8;
    gl.Padding     = [12 12 12 12];

    % ── Row 1: Run file picker + Refresh + Reload ──────────────────
    glRow1 = uigridlayout(gl, [1 4]);
    glRow1.Layout.Row    = 1;
    glRow1.ColumnWidth   = {90, '1x', 90, 90};
    glRow1.ColumnSpacing = 8;
    glRow1.Padding       = [0 0 0 0];

    uilabel(glRow1, 'Text', 'Run File:', ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    ddRunFile = uidropdown(glRow1, 'Items', listRunFiles(), ...
        'ValueChangedFcn', @(~,~) onRunFileSelected());
    uibutton(glRow1, 'Text', 'Refresh', ...
        'Tooltip', 'Re-scan config/runs/ for new files', ...
        'ButtonPushedFcn', @(~,~) refreshOptions());
    btnReload = uibutton(glRow1, 'Text', 'Reload', ...
        'Tooltip', 'Re-read the selected run file from disk (discards changes)', ...
        'ButtonPushedFcn', @(~,~) reloadCurrent());

    % ── Row 2: Description ──────────────────────────────────────────
    glRow2 = uigridlayout(gl, [1 2]);
    glRow2.Layout.Row    = 2;
    glRow2.ColumnWidth   = {90, '1x'};
    glRow2.ColumnSpacing = 8;
    glRow2.Padding       = [0 0 0 0];

    uilabel(glRow2, 'Text', 'Description:', ...
        'HorizontalAlignment', 'right', 'FontWeight', 'bold');
    editDescription = uieditfield(glRow2, 'text', ...
        'ValueChangedFcn', @(~,~) updateDirty());

    % ── Row 3: Trackers + Degradations side by side ────────────────
    glRow3 = uigridlayout(gl, [1 2]);
    glRow3.Layout.Row    = 3;
    glRow3.ColumnWidth   = {'1x', '1x'};
    glRow3.ColumnSpacing = 12;
    glRow3.Padding       = [0 0 0 0];

    % --- Left: Trackers panel ---
    pnlTrackers = uipanel(glRow3, 'Title', 'Trackers (multi-select)', ...
        'FontWeight', 'bold');
    glTrack = uigridlayout(pnlTrackers, [1 1]);
    listTrackers = uilistbox(glTrack, 'Multiselect', 'on', ...
        'Items', {'(no run loaded)'}, ...
        'ValueChangedFcn', @(~,~) updateDirty());

    % --- Right: Degradation panel ---
    %  Layout: 5 toggles, spacer, weather row (label+dropdown sub-grid),
    %  spacer, cache toggle. The weather row uses a horizontal sub-grid
    %  rather than a separate label-row + dropdown-row because bold
    %  uilabels in stand-alone narrow rows of a scrollable uipanel get
    %  clipped vertically (the bottom of the descenders cuts off).
    %  Side-by-side avoids the issue and is more compact.
    pnlDeg = uipanel(glRow3, 'Title', 'Degradations & Performance', ...
        'FontWeight', 'bold', 'Scrollable', 'on');
    glDeg = uigridlayout(pnlDeg, [9 1]);
    %                ckOc ckMa ckCl ckDp ckRC sp wRow sp ckCa
    glDeg.RowHeight  = {26,  26,  26,  26,  26,  8,  30,  8,  26};
    glDeg.RowSpacing = 4;

    chkOcclusion = uicheckbox(glDeg, 'Text', 'Terrain Occlusion', ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));
    chkMasking   = uicheckbox(glDeg, 'Text', 'Horizon Masking', ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));
    chkClutter   = uicheckbox(glDeg, 'Text', 'Ground Clutter', ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));
    chkDoppler   = uicheckbox(glDeg, 'Text', 'Doppler Fade', ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));
    chkRCS       = uicheckbox(glDeg, 'Text', 'RCS Range Filter (opt-in)', ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));

    % Weather sub-grid: label + dropdown side-by-side, both vertically
    % centered in a single 30 px row. Explicit Layout.Row=7 places it
    % AFTER the spacer (row 6) instead of falling into row 6 itself.
    glWeather = uigridlayout(glDeg, [1 2]);
    glWeather.Layout.Row    = 7;
    glWeather.Layout.Column = 1;
    glWeather.ColumnWidth   = {70, '1x'};
    glWeather.ColumnSpacing = 6;
    glWeather.Padding       = [0 0 0 0];
    uilabel(glWeather, 'Text', 'Weather:', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left', 'VerticalAlignment', 'center');
    ddWeather = uidropdown(glWeather, 'Items', listWeatherOptions(), ...
        'ValueChangedFcn', @(src, evt) onDegradationChanged(src, evt));

    % Cache toggle — explicit Layout.Row=9 because the sub-grid above
    % occupied row 7 and MATLAB's auto-placement would otherwise put
    % chkCache into row 6 (the spacer).
    chkCache = uicheckbox(glDeg, 'Text', 'Use Cached Detections', ...
        'ValueChangedFcn', @(~,~) updateDirty());
    chkCache.Layout.Row = 9;

    % ── Row 4: Save Changes button ──────────────────────────────────
    btnSave = uibutton(gl, 'Text', 'Save Changes to Run File', ...
        'FontWeight', 'bold', ...
        'Enable', 'off', ...
        'ButtonPushedFcn', @(~,~) saveCurrent());
    btnSave.Layout.Row = 4;

    % ── Row 5: Run / Compare / Auto-Tune buttons ────────────────────
    glRow5 = uigridlayout(gl, [1 3]);
    glRow5.Layout.Row    = 5;
    glRow5.ColumnWidth   = {'1x', '1x', '1x'};
    glRow5.ColumnSpacing = 8;
    glRow5.Padding       = [0 0 0 0];

    btnRun = uibutton(glRow5, 'Text', 'Run Simulation', ...
        'FontWeight', 'bold', 'FontSize', 13, ...
        'ButtonPushedFcn', @(~,~) runSimulation());
    btnCompare = uibutton(glRow5, 'Text', 'Compare Trackers', ...
        'ButtonPushedFcn', @(~,~) compareTrackersAction());
    btnTune = uibutton(glRow5, 'Text', 'Auto-Tune Tracker', ...
        'ButtonPushedFcn', @(~,~) autoTuneAction());

    % ── Initial load ────────────────────────────────────────────────
    if ~isempty(ddRunFile.Items)
        loadRunFile(ddRunFile.Value);
    else
        setControlsEnabled(false);
        uialert(fig, ...
            ['No run files found in config/runs/. Use the Compose Run File ' ...
             'or Path Editor windows to create one first.'], ...
            'No Run Files', 'Icon', 'info');
    end

    % Return fig only when the caller captures it. Critically, do NOT
    % `clear fig` here — nested-function closures look up `fig` by name
    % in this workspace at call time, so clearing it breaks every
    % uialert/uiconfirm/delete(fig) call from every callback below.
    if nargout > 0, varargout{1} = fig; end


%% ========================================================================
%  Config dir scanning
%% ========================================================================

    function items = listRunFiles()
        % Top-level config/runs/*.json only — subdirs like validation/
        % are intentionally hidden so the dropdown stays focused on
        % user-facing runs.
        runDir = fullfile(projectRoot, 'config', 'runs');
        files = dir(fullfile(runDir, '*.json'));
        items = {};
        for k = 1:numel(files)
            name = files(k).name;
            if contains(name, 'template') || startsWith(name, '_')
                continue;  % hide templates and hidden files
            end
            items{end+1} = strrep(name, '.json', ''); %#ok<AGROW>
        end
    end

    function items = listConfigDir(subDir)
        % Mirrors runScenarioGUI's getAvailableConfigs — returns
        % "<TYPE>/<stem>" strings for each non-template/non-globals JSON
        % under config/<subDir>/**/*.json.
        basePath = fullfile(projectRoot, 'config', subDir);
        files = dir(fullfile(basePath, '**', '*.json'));
        items = {};
        for k = 1:numel(files)
            if contains(files(k).name, 'template') || ...
               contains(files(k).name, 'globals')
                continue;
            end
            folderPath = strrep(files(k).folder, basePath, '');
            if startsWith(folderPath, filesep)
                folderPath = folderPath(2:end);
            end
            folderPath = strrep(folderPath, '\', '/');
            stem = strrep(files(k).name, '.json', '');
            if isempty(folderPath)
                items{end+1} = stem; %#ok<AGROW>
            else
                items{end+1} = [folderPath '/' stem]; %#ok<AGROW>
            end
        end
    end

    function items = listWeatherOptions()
        % Weather dropdown always includes "none" (clear sky) as the
        % first option. Run files with degradation.weather=="none" or no
        % weather field both map here.
        items = [{'none'}, listConfigDir('weather')];
    end


%% ========================================================================
%  Load & save
%% ========================================================================

    function loadRunFile(name)
        % Read config/runs/<name>.json and populate every widget. Stores
        % the full struct on originalConfig so dirty-tracking has a
        % baseline and saveCurrent can preserve unedited fields verbatim.
        %
        % Programmatic widget value sets do NOT fire ValueChangedFcn in
        % uifigure widgets (R2018b+), so onDegradationChanged/updateDirty
        % do not run during load. We explicitly clear isDirty + Save
        % button at the end.
        runPath = fullfile(projectRoot, 'config', 'runs', [char(name) '.json']);
        if ~isfile(runPath)
            uialert(fig, sprintf('Run file not found: %s', runPath), ...
                'Load Failed');
            setControlsEnabled(false);
            return;
        end

        try
            jsonStr = fileread(runPath);
            cfg = jsondecode(jsonStr);
        catch ME
            uialert(fig, sprintf('Failed to parse %s:\n%s', runPath, ME.message), ...
                'Load Failed');
            setControlsEnabled(false);
            return;
        end

        currentRunName = char(name);
        originalConfig = cfg;

        % Description
        if isfield(cfg, 'description')
            editDescription.Value = char(string(cfg.description));
        else
            editDescription.Value = '';
        end

        % Trackers — re-scan available list, pre-select what's in the run
        listTrackers.Items = listConfigDir('trackers');
        if isempty(listTrackers.Items)
            listTrackers.Items = {'(none available)'};
        end
        runTrackers = {};
        if isfield(cfg, 'trackers')
            runTrackers = cellstr(string(cfg.trackers));
        end
        validTrackers = intersect(runTrackers, listTrackers.Items, 'stable');
        if ~isempty(validTrackers)
            listTrackers.Value = validTrackers;
        else
            listTrackers.Value = {};
        end

        % Degradation block — handle missing fields and the legacy
        % "enabled":true shorthand by defaulting unspecified toggles to
        % true (all-on, RCS off — matches loadRunFile.m).
        deg = struct();
        if isfield(cfg, 'degradation') && isstruct(cfg.degradation)
            deg = cfg.degradation;
        end
        chkOcclusion.Value = logical(getFieldOrDefault(deg, 'terrain_occlusion', true));
        chkMasking.Value   = logical(getFieldOrDefault(deg, 'horizon_masking',   true));
        chkClutter.Value   = logical(getFieldOrDefault(deg, 'ground_clutter',    true));
        chkDoppler.Value   = logical(getFieldOrDefault(deg, 'doppler_fade',      true));
        chkRCS.Value       = logical(getFieldOrDefault(deg, 'rcs_range_filter',  false));

        % Weather string normalization — empty/missing → "none".
        ddWeather.Items = listWeatherOptions();
        weatherVal = char(string(getFieldOrDefault(deg, 'weather', 'none')));
        if isempty(weatherVal); weatherVal = 'none'; end
        if ismember(weatherVal, ddWeather.Items)
            ddWeather.Value = weatherVal;
        else
            % Run references a weather file no longer on disk — surface a
            % note and fall back to 'none' so the GUI is consistent.
            ddWeather.Value = 'none';
            uialert(fig, sprintf( ...
                ['Weather config "%s" referenced by this run no longer ' ...
                 'exists in config/weather/. Falling back to "none". ' ...
                 'Click Save Changes to commit, or Reload to revert.'], ...
                weatherVal), ...
                'Missing Weather Config', 'Icon', 'warning');
        end

        % Cache flag
        cacheUse = false;
        if isfield(cfg, 'cache') && isstruct(cfg.cache) && ...
                isfield(cfg.cache, 'use_cached_detections')
            cacheUse = logical(cfg.cache.use_cached_detections);
        end
        chkCache.Value = cacheUse;

        % Reset dirty state and re-enable controls
        isDirty        = false;
        btnSave.Enable = 'off';
        setControlsEnabled(true);
    end

    function reloadCurrent()
        % Re-read the currently selected run file from disk. Confirms
        % first if there are unsaved changes.
        if isempty(currentRunName); return; end
        if isDirty
            choice = uiconfirm(fig, sprintf( ...
                'Discard unsaved changes to "%s" and reload from disk?', ...
                currentRunName), 'Unsaved Changes', ...
                'Options', {'Discard and Reload', 'Cancel'}, ...
                'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
                'Icon', 'warning');
            if ~strcmp(choice, 'Discard and Reload')
                return;
            end
        end
        loadRunFile(currentRunName);
    end

    function refreshOptions()
        % Re-scan config/runs/ and the weather/tracker dropdowns. If the
        % previously selected run still exists, reload it. If unsaved
        % changes exist, confirm first.
        if isDirty
            choice = uiconfirm(fig, sprintf( ...
                ['You have unsaved changes to "%s". Refresh will reload ' ...
                 'the file list and reload the selected run, discarding ' ...
                 'unsaved edits.'], currentRunName), 'Unsaved Changes', ...
                'Options', {'Save and Refresh', 'Discard and Refresh', 'Cancel'}, ...
                'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
                'Icon', 'warning');
            switch choice
                case 'Save and Refresh'
                    saveCurrent();
                case 'Cancel'
                    return;
                % 'Discard and Refresh' falls through
            end
        end
        prevSel = ddRunFile.Value;
        ddRunFile.Items = listRunFiles();
        if ~isempty(ddRunFile.Items) && ismember(prevSel, ddRunFile.Items)
            ddRunFile.Value = prevSel;
            loadRunFile(prevSel);
        elseif ~isempty(ddRunFile.Items)
            loadRunFile(ddRunFile.Value);
        else
            currentRunName = '';
            setControlsEnabled(false);
        end
    end

    function saveCurrent()
        % Write the current widget state back to
        % config/runs/<currentRunName>.json, preserving unexposed fields
        % verbatim from originalConfig (sensors, targets, terrain,
        % output, platforms, etc.).
        if isempty(currentRunName); return; end
        cfg = buildCurrentConfig();
        runPath = fullfile(projectRoot, 'config', 'runs', [currentRunName '.json']);
        try
            writeJsonFile(runPath, cfg);
        catch ME
            uialert(fig, sprintf('Failed to save:\n%s', ME.message), ...
                'Save Failed');
            return;
        end
        originalConfig = cfg;
        isDirty        = false;
        btnSave.Enable = 'off';
    end

    function commitCacheFlag(value)
        % Persist ONLY the cache.use_cached_detections flag to disk,
        % leaving every other field — including dirty widget edits not
        % yet saved by the user — untouched. Used after a successful Run
        % so subsequent runs hit the cache without forcing the user to
        % commit unrelated pending edits.
        if isempty(currentRunName); return; end
        if ~isfield(originalConfig, 'cache') || ~isstruct(originalConfig.cache)
            originalConfig.cache = struct('save_detections', true);
        end
        originalConfig.cache.use_cached_detections = logical(value);
        runPath = fullfile(projectRoot, 'config', 'runs', [currentRunName '.json']);
        try
            writeJsonFile(runPath, originalConfig);
        catch
            % Save failed silently — non-critical for the run flow,
            % the cache flag in the widget is still correct in memory.
        end
    end


%% ========================================================================
%  Config building & dirty tracking
%% ========================================================================

    function cfg = buildCurrentConfig()
        % Overlay current widget values onto originalConfig and return
        % the merged struct. Fields not exposed by the GUI are preserved.
        cfg = originalConfig;

        cfg.description = char(editDescription.Value);

        trackers = {};
        if ~isempty(listTrackers.Value)
            trackers = cellstr(listTrackers.Value);
        end
        cfg.trackers         = trackers;
        cfg.compare_trackers = trackers;  % match runScenarioGUI convention

        if ~isfield(cfg, 'degradation') || ~isstruct(cfg.degradation)
            cfg.degradation = struct();
        end
        cfg.degradation.terrain_occlusion = logical(chkOcclusion.Value);
        cfg.degradation.horizon_masking   = logical(chkMasking.Value);
        cfg.degradation.ground_clutter    = logical(chkClutter.Value);
        cfg.degradation.doppler_fade      = logical(chkDoppler.Value);
        cfg.degradation.rcs_range_filter  = logical(chkRCS.Value);
        cfg.degradation.weather           = char(ddWeather.Value);

        if ~isfield(cfg, 'cache') || ~isstruct(cfg.cache)
            cfg.cache = struct('save_detections', true);
        end
        cfg.cache.use_cached_detections = logical(chkCache.Value);
    end

    function updateDirty()
        % Compare current widget values against originalConfig. Any diff
        % flips isDirty and enables the Save button. Called from every
        % editable widget's ValueChangedFcn.
        if isempty(currentRunName); return; end
        candidate = buildCurrentConfig();
        isDirty = ~configsEqual(candidate, originalConfig);
        if isDirty
            btnSave.Enable = 'on';
        else
            btnSave.Enable = 'off';
        end
    end

    function onDegradationChanged(src, evt)
        % A cache-invalidating widget (5 degradation toggles or weather
        % dropdown) changed. If the cache is currently active for this
        % run (chkCache=true AND cache file exists), warn the user that
        % the change will invalidate the cache and let them confirm or
        % cancel.
        %
        % On Continue: chkCache is auto-unchecked (the cache is now
        % stale and will be regenerated on next Run).
        % On Cancel: the widget's PreviousValue from the event arg is
        % restored — no change takes effect.
        if chkCache.Value && cacheExists()
            choice = uiconfirm(fig, sprintf( ...
                ['Changing this will invalidate the cached detections ' ...
                 'for "%s". The next run will regenerate detections ' ...
                 'from scratch (slower).' newline newline ...
                 'Continue?'], currentRunName), ...
                'Invalidate Cache?', ...
                'Options', {'Continue', 'Cancel'}, ...
                'DefaultOption', 'Continue', 'CancelOption', 'Cancel', ...
                'Icon', 'warning');
            if strcmp(choice, 'Cancel')
                src.Value = evt.PreviousValue;
                return;
            end
            chkCache.Value = false;
        end
        updateDirty();
    end

    function tf = cacheExists()
        % Does cache/<currentRunName>.mat exist on disk?
        if isempty(currentRunName); tf = false; return; end
        cachePath = fullfile(projectRoot, 'cache', [currentRunName '.mat']);
        tf = isfile(cachePath);
    end


%% ========================================================================
%  Run / Compare / Auto-Tune
%% ========================================================================
%
%  All three actions follow the same pattern:
%    1. If dirty, save first so the on-disk file reflects what's about
%       to run. The cache key is the run name; saving keeps cache hits
%       deterministic. (saveCurrent's uialert handles fopen failures.)
%    2. Validate prerequisites (tracker selection, cache for compare/tune)
%    3. Invoke runSingleScenario / compareTrackers / autoTuneTracker
%    4. (Run only) On success, auto-check chkCache and persist that
%       flag to disk via commitCacheFlag.

    function runSimulation()
        if isempty(currentRunName)
            uialert(fig, 'No run file selected.', 'Cannot Run'); return;
        end
        if isDirty
            saveCurrent();
            if isDirty
                return;  % saveCurrent already showed the error
            end
        end

        setappdata(0, 'trackbench_suppressPause', true);
        try
            runSingleScenario(currentRunName);
        catch ME
            setappdata(0, 'trackbench_suppressPause', false);
            uialert(fig, ME.message, 'Run Simulation Error');
            return;
        end
        setappdata(0, 'trackbench_suppressPause', false);

        % Smart cache: after a successful run the cache is fresh for the
        % current configuration, so enable "Use Cached Detections" and
        % persist that single flag to the run file.
        if ~chkCache.Value
            chkCache.Value = true;
            commitCacheFlag(true);
            updateDirty();  % chkCache may now match originalConfig again
        end
    end

    function compareTrackersAction()
        if isempty(currentRunName)
            uialert(fig, 'No run file selected.', 'Cannot Compare'); return;
        end
        if numel(cellstr(listTrackers.Value)) < 2
            uialert(fig, ...
                'Select at least two trackers (Ctrl+Click) to compare.', ...
                'Need More Trackers'); return;
        end
        if isDirty
            saveCurrent();
            if isDirty, return; end
        end
        cachePath = fullfile(projectRoot, 'cache', [currentRunName '.mat']);
        if ~isfile(cachePath)
            uialert(fig, sprintf( ...
                ['No cached detections for "%s". Click Run Simulation ' ...
                 'once first to generate the cache.'], currentRunName), ...
                'Cache Required');
            return;
        end
        try
            compareTrackers(currentRunName);
        catch ME
            uialert(fig, ME.message, 'Compare Trackers Error');
        end
    end

    function autoTuneAction()
        if isempty(currentRunName)
            uialert(fig, 'No run file selected.', 'Cannot Tune'); return;
        end
        sel = cellstr(listTrackers.Value);
        if isempty(sel)
            uialert(fig, 'Select a tracker to tune.', 'No Tracker Selected');
            return;
        end
        if isDirty
            saveCurrent();
            if isDirty, return; end
        end
        cachePath = fullfile(projectRoot, 'cache', [currentRunName '.mat']);
        if ~isfile(cachePath)
            uialert(fig, sprintf( ...
                ['No cached detections for "%s". Click Run Simulation ' ...
                 'once first to generate the cache.'], currentRunName), ...
                'Cache Required');
            return;
        end
        % Use the first selected tracker's algorithm class
        % ("GNN/default_GNN" → "GNN") matching runScenarioGUI.
        parts = split(sel{1}, '/');
        trackerType = parts{1};
        try
            autoTuneTracker(currentRunName, trackerType);
        catch ME
            uialert(fig, ME.message, 'Auto-Tune Error');
        end
    end


%% ========================================================================
%  Change-confirmation flow (switch / close)
%% ========================================================================

    function onRunFileSelected()
        % User picked a different run file. Confirm before discarding
        % unsaved changes; revert dropdown selection on cancel.
        target = ddRunFile.Value;
        if strcmp(target, currentRunName)
            return;
        end
        if isDirty
            choice = uiconfirm(fig, sprintf( ...
                'Save changes to "%s" before switching?', currentRunName), ...
                'Unsaved Changes', ...
                'Options', {'Save', 'Discard', 'Cancel'}, ...
                'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
                'Icon', 'warning');
            switch choice
                case 'Save'
                    saveCurrent();
                case 'Cancel'
                    ddRunFile.Value = currentRunName;
                    return;
                % 'Discard' falls through
            end
        end
        loadRunFile(target);
    end

    function onCloseRequest()
        % Confirm before closing if there are unsaved changes.
        if isDirty
            choice = uiconfirm(fig, sprintf( ...
                'Save changes to "%s" before closing?', currentRunName), ...
                'Unsaved Changes', ...
                'Options', {'Save', 'Discard', 'Cancel'}, ...
                'DefaultOption', 'Cancel', 'CancelOption', 'Cancel', ...
                'Icon', 'warning');
            switch choice
                case 'Save'
                    saveCurrent();
                case 'Cancel'
                    return;
                % 'Discard' falls through
            end
        end
        delete(fig);
    end


%% ========================================================================
%  UI enable/disable
%% ========================================================================

    function setControlsEnabled(tf)
        sv = 'off';
        if tf, sv = 'on'; end
        editDescription.Enable = sv;
        listTrackers.Enable    = sv;
        chkOcclusion.Enable    = sv;
        chkMasking.Enable      = sv;
        chkClutter.Enable      = sv;
        chkDoppler.Enable      = sv;
        chkRCS.Enable          = sv;
        ddWeather.Enable       = sv;
        chkCache.Enable        = sv;
        btnRun.Enable          = sv;
        btnCompare.Enable      = sv;
        btnTune.Enable         = sv;
        btnReload.Enable       = sv;
        % btnSave is governed by isDirty; force off when no run loaded.
        if ~tf, btnSave.Enable = 'off'; end
    end
end


%% ========================================================================
%  Module-level pure helpers (NOT nested)
%% ========================================================================

function v = getFieldOrDefault(s, name, default)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default;
    end
end

function v = getNested(s, outer, inner, default)
    v = default;
    if ~isstruct(s) || ~isfield(s, outer); return; end
    inner_struct = s.(outer);
    if ~isstruct(inner_struct) || ~isfield(inner_struct, inner); return; end
    v = inner_struct.(inner);
end

function tf = configsEqual(a, b)
    % Compare only the GUI-editable fields. Other fields are preserved
    % verbatim by buildCurrentConfig and don't need to enter dirty checks.
    tf = ...
        isequal(char(getFieldOrDefault(a, 'description', '')), ...
                char(getFieldOrDefault(b, 'description', ''))) && ...
        isequal(sort(cellstr(getFieldOrDefault(a, 'trackers', {}))), ...
                sort(cellstr(getFieldOrDefault(b, 'trackers', {})))) && ...
        degEqual(getFieldOrDefault(a, 'degradation', struct()), ...
                 getFieldOrDefault(b, 'degradation', struct())) && ...
        isequal(logical(getNested(a, 'cache', 'use_cached_detections', false)), ...
                logical(getNested(b, 'cache', 'use_cached_detections', false)));
end

function tf = degEqual(a, b)
    % Defaults match buildCurrentConfig's emitted shape: all toggles
    % default true except rcs_range_filter (false), and weather defaults
    % to 'none'. A run file with NO degradation block compares equal to
    % a freshly populated default block.
    tf = isequal(logical(getFieldOrDefault(a, 'terrain_occlusion', true)), ...
                 logical(getFieldOrDefault(b, 'terrain_occlusion', true))) && ...
         isequal(logical(getFieldOrDefault(a, 'horizon_masking',   true)), ...
                 logical(getFieldOrDefault(b, 'horizon_masking',   true))) && ...
         isequal(logical(getFieldOrDefault(a, 'ground_clutter',    true)), ...
                 logical(getFieldOrDefault(b, 'ground_clutter',    true))) && ...
         isequal(logical(getFieldOrDefault(a, 'doppler_fade',      true)), ...
                 logical(getFieldOrDefault(b, 'doppler_fade',      true))) && ...
         isequal(logical(getFieldOrDefault(a, 'rcs_range_filter',  false)), ...
                 logical(getFieldOrDefault(b, 'rcs_range_filter',  false))) && ...
         isequal(char(string(getFieldOrDefault(a, 'weather', 'none'))), ...
                 char(string(getFieldOrDefault(b, 'weather', 'none'))));
end

function writeJsonFile(path, cfg)
    fid = fopen(path, 'w');
    if fid < 0
        error('runSimGUI:writeJsonFile:openFailed', ...
            'Could not open %s for writing.', path);
    end
    closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s', jsonencode(cfg, 'PrettyPrint', true));
end
