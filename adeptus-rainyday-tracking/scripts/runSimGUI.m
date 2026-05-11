function varargout = runSimGUI(projectRoot)
%runSimGUI  Level 3 of mainMenu — load, edit, and run an existing run file.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
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
    %  v3.5 §Step2 — layout extended from [1 1] (list-only) to [2 1] so a
    %  button strip can live below the list. The list is the growing
    %  row; the button strip is fixed height.
    pnlTrackers = uipanel(glRow3, 'Title', 'Trackers (multi-select)', ...
        'FontWeight', 'bold');
    glTrack = uigridlayout(pnlTrackers, [2 1]);
    glTrack.RowHeight  = {'1x', 32};
    glTrack.RowSpacing = 6;
    listTrackers = uilistbox(glTrack, 'Multiselect', 'on', ...
        'Items', {'(no run loaded)'}, ...
        'ValueChangedFcn', @(~,~) onTrackerSelectionChanged());

    % Button strip: "Edit Tracker Params..." + "Globals...". Edit is
    % disabled until exactly one tracker is selected (the dropdown
    % below uses Multiselect so we need single-selection for editing).
    glTrackBtns = uigridlayout(glTrack, [1 2]);
    glTrackBtns.Layout.Row = 2;
    glTrackBtns.ColumnWidth = {'1x', '1x'};
    glTrackBtns.ColumnSpacing = 6;
    glTrackBtns.Padding = [0 0 0 0];
    btnEditTracker = uibutton(glTrackBtns, ...
        'Text', 'Edit Tracker Params…', ...
        'Enable', 'off', ...
        'Tooltip', ['Edit the selected tracker''s parameters. ' ...
                    'Disabled when zero or multiple trackers are selected.'], ...
        'ButtonPushedFcn', @(~,~) onEditTracker());
    btnEditGlobals = uibutton(glTrackBtns, ...
        'Text', 'Globals…', ...
        'Tooltip', ['Edit shared parameters (max tracks, Pd, filter ' ...
                    'initialization). Affects ALL trackers.'], ...
        'ButtonPushedFcn', @(~,~) onEditGlobals()); %#ok<NASGU>

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
        % v3.5 §5c.5 — polymorphic weather field. The on-disk shape may
        % be either a legacy string scalar OR a {fallback, regions[]}
        % struct. The dropdown shows the FALLBACK (or 'none' if missing);
        % any regions are preserved through the originalConfig copy and
        % round-tripped on Save (see saveChanges below). Editing weather
        % regions themselves still requires the Path Editor.
        ddWeather.Items = listWeatherOptions();
        weatherField = getFieldOrDefault(deg, 'weather', 'none');
        if isstruct(weatherField)
            if isfield(weatherField, 'fallback') && ~isempty(weatherField.fallback)
                weatherVal = char(string(weatherField.fallback));
            else
                weatherVal = 'none';
            end
        elseif ischar(weatherField) || isstring(weatherField)
            weatherVal = char(string(weatherField));
        else
            weatherVal = 'none';
        end
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
        % v3.5 §5c.5 — polymorphic weather write. If the original cfg
        % carried a {fallback, regions[]} struct, keep the regions and
        % only update the fallback from the dropdown. Otherwise emit a
        % legacy string scalar (matches editor exporter behavior).
        chosenWeather = char(ddWeather.Value);
        existingWeather = struct();
        if isfield(cfg.degradation, 'weather')
            existingWeather = cfg.degradation.weather;
        end
        if isstruct(existingWeather) && isfield(existingWeather, 'regions') ...
                && ~isempty(existingWeather.regions)
            cfg.degradation.weather = struct( ...
                'fallback', chosenWeather, ...
                'regions',  existingWeather.regions);
        else
            cfg.degradation.weather = chosenWeather;
        end

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
        % v3.5 §Step2 — Edit/Globals buttons. Globals follows the master
        % enable (it doesn't need a tracker selected to operate). Edit
        % goes off here; onTrackerSelectionChanged re-enables it when
        % exactly one tracker is selected.
        btnEditGlobals.Enable = sv;
        btnEditTracker.Enable = 'off';
        if tf
            onTrackerSelectionChanged();
        end
        % btnSave is governed by isDirty; force off when no run loaded.
        if ~tf, btnSave.Enable = 'off'; end
    end

    function onTrackerSelectionChanged()
        %onTrackerSelectionChanged  v3.5 §Step2 — listbox callback.
        %   Updates the Edit Tracker button enable state (1 selected
        %   only) and forwards to updateDirty so the existing dirty-
        %   tracking logic still fires.
        val = listTrackers.Value;
        if iscell(val)
            nSel = numel(val);
        elseif ischar(val) || isstring(val)
            nSel = 1;
        else
            nSel = 0;
        end
        % "(no run loaded)" placeholder counts as zero real trackers.
        if nSel == 1
            firstVal = val;
            if iscell(firstVal); firstVal = firstVal{1}; end
            if strcmp(char(firstVal), '(no run loaded)')
                nSel = 0;
            end
        end
        if nSel == 1 && strcmp(listTrackers.Enable, 'on')
            btnEditTracker.Enable = 'on';
        else
            btnEditTracker.Enable = 'off';
        end
        updateDirty();
    end

    function onEditTracker()
        %onEditTracker  v3.5 §Step2 — open the per-tracker editor modal.
        %
        %  STEP 2.2 (this commit): full editor for GNN — Identity +
        %  Algorithm Parameters, real Save logic with name validation,
        %  list refresh on save. JPDA / TOMHT show Identity only with a
        %  "coming in 2.3" placeholder and a disabled Save button.
        %
        %  Save-As semantics:
        %   * Default name pre-fills with "my_<TYPE>" or auto-suffixed
        %     to first unused "my_<TYPE>_N".
        %   * Names starting with "default_" are rejected (Q2 design).
        %   * Existing files trigger an Overwrite confirm.
        %   * Saved file lands at config/trackers/<TYPE>/<name>.json.
        %   * Untouched fields in the source JSON (e.g. JPDA-specific
        %     params inside a GNN file) are PRESERVED so editing GNN
        %     never silently strips JPDA/TOMHT-relevant settings.
        val = listTrackers.Value;
        if iscell(val); val = val{1}; end
        trackerRef = char(val);
        parts = split(string(trackerRef), '/');
        if numel(parts) ~= 2
            uialert(fig, sprintf('Invalid tracker reference "%s".', trackerRef), ...
                'Edit Tracker'); return;
        end
        trackerType = char(parts(1));
        stem        = char(parts(2));

        jsonPath = fullfile(projectRoot, 'config', 'trackers', trackerType, [stem '.json']);
        if ~exist(jsonPath, 'file')
            uialert(fig, sprintf('Tracker file not found:\n%s', jsonPath), ...
                'Edit Tracker'); return;
        end
        try
            cfg = jsondecode(fileread(jsonPath));
        catch err
            uialert(fig, sprintf('Failed to read JSON:\n%s', err.message), ...
                'Edit Tracker'); return;
        end

        % Current values with defensive fallbacks. paramsStruct may be
        % missing entirely on a hand-edited JSON; we still want sensible
        % values to seed the form.
        curDesc      = char(string(getFieldOrDefault(cfg, 'description', '')));
        curFilter    = char(string(getFieldOrDefault(cfg, 'filter_model', 'IMM')));
        curVolume    = getFieldOrDefault(cfg, 'volume', 1e9);
        curBeta      = getFieldOrDefault(cfg, 'beta',   1e-14);
        paramsStruct = getFieldOrDefault(cfg, 'params', struct());
        curGate      = getFieldOrDefault(paramsStruct, 'gate',              45);
        curConfirm   = getFieldOrDefault(paramsStruct, 'confirm_threshold', 20);
        curDelete    = getFieldOrDefault(paramsStruct, 'delete_threshold', -5);
        curFarGnn    = getFieldOrDefault(paramsStruct, 'far_gnn',           1e-6);
        % v3.5 §Step2.3 — JPDA + TOMHT-specific fields.
        curGateJpda  = getFieldOrDefault(paramsStruct, 'gate_jpda',          45);
        curFarJpda   = getFieldOrDefault(paramsStruct, 'far_jpda',           1e-6);
        curBetaJpda  = getFieldOrDefault(paramsStruct, 'beta_jpda',          1e-14);
        curTimeTol   = getFieldOrDefault(paramsStruct, 'time_tolerance_jpda', 0.05);
        curNumTracksJ = getFieldOrDefault(paramsStruct, 'num_tracks_jpda',   500);
        curJpdaConf  = getFieldOrDefault(paramsStruct, 'jpda_confirm_prob',  0.95);
        curJpdaDel   = getFieldOrDefault(paramsStruct, 'jpda_delete_prob',   0.05);
        curFarMht    = getFieldOrDefault(paramsStruct, 'far_mht',            1e-6);
        curMaxBranch = getFieldOrDefault(paramsStruct, 'max_branches',       5);
        % TOMHT threshold multiplier is an array [confirm, history, delete]
        % multiplied by gate inside buildTracker. Normalize length: pad
        % with defaults if shorter, truncate if longer.
        rawMult = getFieldOrDefault(paramsStruct, 'tomht_threshold_multiplier', [0.2, 1, 1]);
        rawMult = rawMult(:).';  % force row
        defMult = [0.2, 1, 1];
        for kPad = 1:3
            if numel(rawMult) < kPad; rawMult(kPad) = defMult(kPad); end
        end
        curMult1 = rawMult(1);  curMult2 = rawMult(2);  curMult3 = rawMult(3);

        % Pre-fill Save As name. Refuse-default-name guard (Q2) means
        % default_<TYPE> auto-suggests my_<TYPE>; any other name
        % auto-suggests the same name (user can edit before Save).
        if startsWith(stem, 'default_')
            suggestedName = suggestNewTrackerName(projectRoot, trackerType, 'my');
        else
            suggestedName = stem;
        end

        % ---- Build modal -------------------------------------------------
        editorFig = uifigure('Name', ['Edit Tracker — ' trackerRef], ...
            'Position', [200 120 560 600], ...
            'WindowStyle', 'modal');
        gl = uigridlayout(editorFig, [4 1]);
        gl.RowHeight  = {130, '1x', 40, 40};
        gl.RowSpacing = 10;
        gl.Padding    = [14 14 14 14];

        % ---- Identity section --------------------------------------------
        pnlId = uipanel(gl, 'Title', 'Identity', 'FontWeight', 'bold');
        glId = uigridlayout(pnlId, [3 2]);
        glId.ColumnWidth = {140, '1x'};
        glId.RowHeight   = {26, 26, 26};
        glId.RowSpacing  = 4;
        uilabel(glId, 'Text', 'Description:');
        editDesc = uieditfield(glId, 'text', 'Value', curDesc);
        uilabel(glId, 'Text', 'Tracker type:');
        uilabel(glId, 'Text', trackerType, 'FontWeight', 'bold');
        uilabel(glId, 'Text', 'Filter model:');
        filterDD = uidropdown(glId, 'Items', {'IMM', 'CV'}, ...
            'Value', curFilter, ...
            'Tooltip', ['IMM = Interacting Multiple Model (CV + Coordinated Turn). ' ...
                        'CV = Constant Velocity only.']);

        % ---- Algorithm Parameters section --------------------------------
        pnlAlg = uipanel(gl, 'Title', 'Algorithm Parameters', 'FontWeight', 'bold', ...
            'Scrollable', 'on');
        pnlAlg.Layout.Row = 2;
        % v3.5 §Step2.3 — row count sized for TOMHT (the widest at 9 rows).
        % Smaller types under-fill but the panel is Scrollable so excess
        % rows don't push the buttons off-screen.
        glAlg = uigridlayout(pnlAlg, [9 2]);
        glAlg.ColumnWidth = {220, '1x'};
        glAlg.RowHeight   = repmat({26}, 1, 9);
        glAlg.RowSpacing  = 4;

        % Initialize all per-type field handles to [] so onSave's branch
        % can safely test isempty(). Without this, accessing an undefined
        % editGate inside JPDA's branch would error.
        editGate = []; editConfirm = []; editDelete = []; editFar = [];
        editVolume = []; editBeta = [];
        editGateJ = []; editFarJ = []; editBetaJ = []; editTimeTol = [];
        editNumTrJ = []; editJpdaConf = []; editJpdaDel = [];
        editFarMht = []; editMaxBr = [];
        editMult1 = []; editMult2 = []; editMult3 = [];

        switch upper(trackerType)
            case 'GNN'
                uilabel(glAlg, 'Text', 'Gate (Mahalanobis):');
                editGate = uieditfield(glAlg, 'numeric', 'Value', curGate, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Confirmation threshold:');
                editConfirm = uieditfield(glAlg, 'numeric', 'Value', curConfirm);
                uilabel(glAlg, 'Text', 'Deletion threshold:');
                editDelete = uieditfield(glAlg, 'numeric', 'Value', curDelete);
                uilabel(glAlg, 'Text', 'False alarm rate (FAR):');
                editFar = uieditfield(glAlg, 'numeric', 'Value', curFarGnn, ...
                    'Limits', [0 1], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Volume:');
                editVolume = uieditfield(glAlg, 'numeric', 'Value', curVolume, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Beta (new-target density):');
                editBeta = uieditfield(glAlg, 'numeric', 'Value', curBeta, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                saveEnabled = true;
            case 'JPDA'
                % JPDA uses _jpda-suffixed gate/FAR/beta because it has
                % Integrated track logic, not Score, so the value scales
                % are different than GNN/TOMHT.
                uilabel(glAlg, 'Text', 'Gate (JPDA, Mahalanobis):');
                editGateJ = uieditfield(glAlg, 'numeric', 'Value', curGateJpda, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'False alarm rate (JPDA):');
                editFarJ = uieditfield(glAlg, 'numeric', 'Value', curFarJpda, ...
                    'Limits', [0 1], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Beta (JPDA new-target density):');
                editBetaJ = uieditfield(glAlg, 'numeric', 'Value', curBetaJpda, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Time tolerance (s):');
                editTimeTol = uieditfield(glAlg, 'numeric', 'Value', curTimeTol, ...
                    'Limits', [0 Inf]);
                uilabel(glAlg, 'Text', 'Max num tracks (JPDA):');
                editNumTrJ = uieditfield(glAlg, 'numeric', 'Value', curNumTracksJ, ...
                    'Limits', [1 Inf], 'RoundFractionalValues', 'on');
                uilabel(glAlg, 'Text', 'Confirm probability:');
                editJpdaConf = uieditfield(glAlg, 'numeric', 'Value', curJpdaConf, ...
                    'Limits', [0 1]);
                uilabel(glAlg, 'Text', 'Delete probability:');
                editJpdaDel = uieditfield(glAlg, 'numeric', 'Value', curJpdaDel, ...
                    'Limits', [0 1]);
                saveEnabled = true;
            case 'TOMHT'
                uilabel(glAlg, 'Text', 'Gate (Mahalanobis):');
                editGate = uieditfield(glAlg, 'numeric', 'Value', curGate, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Confirmation threshold:');
                editConfirm = uieditfield(glAlg, 'numeric', 'Value', curConfirm);
                uilabel(glAlg, 'Text', 'Deletion threshold:');
                editDelete = uieditfield(glAlg, 'numeric', 'Value', curDelete);
                uilabel(glAlg, 'Text', 'False alarm rate (MHT):');
                editFarMht = uieditfield(glAlg, 'numeric', 'Value', curFarMht, ...
                    'Limits', [0 1], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Volume:');
                editVolume = uieditfield(glAlg, 'numeric', 'Value', curVolume, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Beta (new-target density):');
                editBeta = uieditfield(glAlg, 'numeric', 'Value', curBeta, ...
                    'Limits', [0 Inf], 'LowerLimitInclusive', false);
                uilabel(glAlg, 'Text', 'Max track branches:');
                editMaxBr = uieditfield(glAlg, 'numeric', 'Value', curMaxBranch, ...
                    'Limits', [1 Inf], 'RoundFractionalValues', 'on');
                % Threshold multiplier triple. Each is multiplied by gate
                % inside buildTracker (line: tomhtThresh = mult * gate)
                % to form trackerTOMHT's AssignmentThreshold array:
                % [Confirm, History, Deletion]. We label as such.
                uilabel(glAlg, 'Text', 'Threshold mult (confirm):');
                editMult1 = uieditfield(glAlg, 'numeric', 'Value', curMult1, ...
                    'Limits', [0 Inf]);
                uilabel(glAlg, 'Text', 'Threshold mult (history):');
                editMult2 = uieditfield(glAlg, 'numeric', 'Value', curMult2, ...
                    'Limits', [0 Inf]);
                uilabel(glAlg, 'Text', 'Threshold mult (deletion):');
                editMult3 = uieditfield(glAlg, 'numeric', 'Value', curMult3, ...
                    'Limits', [0 Inf]);
                saveEnabled = true;
            otherwise
                uilabel(glAlg, ...
                    'Text', sprintf(['(Unknown tracker type "%s". ' ...
                                     'No editor available.)'], trackerType), ...
                    'WordWrap', 'on');
                saveEnabled = false;
        end

        % ---- Save-as row -------------------------------------------------
        glSaveAs = uigridlayout(gl, [1 2]);
        glSaveAs.Layout.Row = 3;
        glSaveAs.ColumnWidth = {140, '1x'};
        glSaveAs.Padding = [0 0 0 0];
        uilabel(glSaveAs, 'Text', 'Save as:');
        editName = uieditfield(glSaveAs, 'text', 'Value', suggestedName, ...
            'Tooltip', sprintf( ...
                'Writes to config/trackers/%s/<name>.json. "default_" names refused.', ...
                trackerType));

        % ---- Buttons row -------------------------------------------------
        glBtns = uigridlayout(gl, [1 3]);
        glBtns.Layout.Row = 4;
        glBtns.ColumnWidth = {'1x', 100, 100};
        glBtns.ColumnSpacing = 8;
        glBtns.Padding = [0 0 0 0];
        uilabel(glBtns);  % spacer
        uibutton(glBtns, 'Text', 'Cancel', ...
            'ButtonPushedFcn', @(~,~) delete(editorFig));
        btnSaveModal = uibutton(glBtns, 'Text', 'Save', ...
            'FontWeight', 'bold', ...
            'ButtonPushedFcn', @(~,~) onSave());
        if ~saveEnabled
            btnSaveModal.Enable = 'off';
            btnSaveModal.Tooltip = sprintf( ...
                'Save for %s tracker editor lands in step 2.3.', trackerType); %#ok<NASGU>
        end

        uiwait(editorFig);

        % ---- Nested: Save callback ---------------------------------------
        function onSave()
            newName = strtrim(char(editName.Value));
            [okName, msg] = validateSaveAsName(newName);
            if ~okName
                uialert(editorFig, msg, 'Invalid name'); return;
            end
            outPath = fullfile(projectRoot, 'config', 'trackers', trackerType, ...
                [newName '.json']);
            if exist(outPath, 'file')
                choice = uiconfirm(editorFig, ...
                    sprintf('Overwrite existing %s/%s.json?', trackerType, newName), ...
                    'File exists', ...
                    'Options', {'Overwrite', 'Cancel'}, ...
                    'DefaultOption', 'Cancel', ...
                    'CancelOption',  'Cancel', ...
                    'Icon', 'warning');
                if ~strcmp(choice, 'Overwrite'); return; end
            end
            % Build output — preserve all unedited fields (notably the
            % JPDA/TOMHT-specific params that this GNN file may carry).
            outCfg = cfg;
            outCfg.description  = char(editDesc.Value);
            outCfg.tracker_type = trackerType;
            outCfg.filter_model = char(filterDD.Value);
            if ~isstruct(outCfg.params); outCfg.params = struct(); end
            % v3.5 §Step2.3 — write only the fields used by this tracker
            % type. Cross-type fields are preserved verbatim from cfg.
            switch upper(trackerType)
                case 'GNN'
                    outCfg.volume = editVolume.Value;
                    outCfg.beta   = editBeta.Value;
                    outCfg.params.gate              = editGate.Value;
                    outCfg.params.confirm_threshold = editConfirm.Value;
                    outCfg.params.delete_threshold  = editDelete.Value;
                    outCfg.params.far_gnn           = editFar.Value;
                case 'JPDA'
                    outCfg.params.gate_jpda            = editGateJ.Value;
                    outCfg.params.far_jpda             = editFarJ.Value;
                    outCfg.params.beta_jpda            = editBetaJ.Value;
                    outCfg.params.time_tolerance_jpda  = editTimeTol.Value;
                    outCfg.params.num_tracks_jpda      = editNumTrJ.Value;
                    outCfg.params.jpda_confirm_prob    = editJpdaConf.Value;
                    outCfg.params.jpda_delete_prob     = editJpdaDel.Value;
                case 'TOMHT'
                    outCfg.volume = editVolume.Value;
                    outCfg.beta   = editBeta.Value;
                    outCfg.params.gate              = editGate.Value;
                    outCfg.params.confirm_threshold = editConfirm.Value;
                    outCfg.params.delete_threshold  = editDelete.Value;
                    outCfg.params.far_mht           = editFarMht.Value;
                    outCfg.params.max_branches      = editMaxBr.Value;
                    outCfg.params.tomht_threshold_multiplier = ...
                        [editMult1.Value, editMult2.Value, editMult3.Value];
            end
            try
                writeJsonFile(outPath, outCfg);
            catch err
                uialert(editorFig, sprintf('Write failed:\n%s', err.message), ...
                    'Save failed', 'Icon', 'error'); return;
            end
            % Refresh tracker list + select the new entry.
            newRef = [trackerType '/' newName];
            refreshOptions();
            if any(strcmp(listTrackers.Items, newRef))
                listTrackers.Value = {newRef};
                onTrackerSelectionChanged();
            end
            delete(editorFig);
        end
    end

    function [ok, msg] = validateSaveAsName(name)
        %validateSaveAsName  v3.5 §Step2 — enforce filename rules for
        %  tracker Save As. Forbidden:
        %    * Empty / whitespace-only
        %    * Names starting with "default_" (Q2 design — default_*
        %      configs are read-only baselines)
        %    * Characters outside [A-Za-z0-9_-]
        ok = false; msg = '';
        if isempty(name)
            msg = 'Save name cannot be empty.'; return;
        end
        if startsWith(name, 'default_') || strcmp(name, 'default')
            msg = ['Names starting with "default_" are reserved for ' ...
                   'read-only baseline configs. Use a different name (e.g. "my_GNN").'];
            return;
        end
        if ~all(isstrprop(name, 'alphanum') | (name == '_') | (name == '-'))
            msg = 'Name may only contain letters, digits, underscores, and dashes.';
            return;
        end
        ok = true;
    end

    function onEditGlobals()
        %onEditGlobals  v3.5 §Step2 — open the tracker_globals.json editor.
        %   STEP 2.1 (this commit): skeleton modal only — real fields
        %   in 2.4.
        editorFig = uifigure('Name', 'Edit Tracker Globals', ...
            'Position', [220 220 480 380], ...
            'WindowStyle', 'modal');
        gl = uigridlayout(editorFig, [2 1]);
        gl.RowHeight = {'1x', 40};
        gl.Padding = [16 16 16 16];
        gl.RowSpacing = 12;
        uilabel(gl, ...
            'Text', ['Globals editor (tracker_globals.json).' newline newline ...
                     '(fields land in step 2.4: max tracks, Pd ' ...
                     'ideal/degraded, filter init params)'], ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment',   'top');
        glBtns = uigridlayout(gl, [1 3]);
        glBtns.Layout.Row = 2;
        glBtns.ColumnWidth = {'1x', 100, 100};
        glBtns.ColumnSpacing = 8;
        glBtns.Padding = [0 0 0 0];
        uilabel(glBtns);  % spacer
        uibutton(glBtns, 'Text', 'Cancel', ...
            'ButtonPushedFcn', @(~,~) delete(editorFig));
        btnSaveModal = uibutton(glBtns, 'Text', 'Save', ...
            'FontWeight', 'bold', ...
            'Enable', 'off', ...
            'Tooltip', 'Save lights up in step 2.4 once fields exist.', ...
            'ButtonPushedFcn', @(~,~) delete(editorFig)); %#ok<NASGU>
        uiwait(editorFig);
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

function name = suggestNewTrackerName(projectRoot, trackerType, prefix)
%suggestNewTrackerName  v3.5 §Step2 helper — build a fresh tracker
%  filename stem under config/trackers/<TYPE>/. Tries "<prefix>_<TYPE>"
%  first; if taken, appends _2, _3, ... until an unused name is found.
    base = sprintf('%s_%s', prefix, trackerType);
    name = base;
    n = 2;
    while exist(fullfile(projectRoot, 'config', 'trackers', trackerType, ...
            [name '.json']), 'file')
        name = sprintf('%s_%d', base, n);
        n = n + 1;
        if n > 999  % defensive: don't loop forever on a bug
            break;
        end
    end
end
