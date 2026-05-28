function varargout = flightDataManagerGUI(projectRoot)
%flightDataManagerGUI  Browse NASA flight data and build multi-flight batches.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Browses NASA DASHlink Flight Data Recorder (.mat) files, visualizes
%  trajectories on a globe (true geodetic positions, no simulation),
%  and composes multi-flight batches with per-flight start offsets for
%  import into the Path Editor as recorded_flight target scenarios.
%
%  USAGE
%      flightDataManagerGUI;            % uses pwd as projectRoot
%      flightDataManagerGUI(rootPath);  % from mainMenu
%      fig = flightDataManagerGUI(...); % capture figure handle
%
%  LAYOUT (3-pane workspace)
%     ┌────────────────────────────────────────────────────────────────┐
%     │ Folder: [<path>........] [Browse…] [Rescan]                    │
%     ├──────────────┬──────────────────────────┬──────────────────────┤
%     │ FLIGHT LIST  │ PREVIEW                  │ BATCH BUILDER        │
%     │              │                          │                      │
%     │ uitable of   │ Single-flight top-down   │ Per-flight controls  │
%     │ .mat files   │ lat/lon plot + summary,  │ (rcs, max_duration,  │
%     │ + checkboxes │ or merged NED layout for │ waypoint_interval,   │
%     │              │ the batch preview.       │ start_offset_s).     │
%     │              │                          │ + global ref origin  │
%     │              │                          │ + batch_name.        │
%     ├──────────────┴──────────────────────────┴──────────────────────┤
%     │ [View on Globe] [Preview NED] [Export to Path Editor]  [Close] │
%     └────────────────────────────────────────────────────────────────┘
%
%  SUBSTEP COVERAGE
%     5f.1  Skeleton — menu entry, layout, folder picker, all placeholders
%     5f.2  Left pane wired: scanFlightFolder + uitable + single-flight preview
%     5f.3  "View on Globe" wired: trackingGlobeViewer + geoTrajectory
%     5f.4  Right pane wired: per-flight controls + "Preview NED" layout
%     5f.5  "Export to Path Editor" wired: write recorded_flight JSON + open PE
%
%  See also: mainMenu, pathEditor, runNASAFlight,
%            trackbench.flightdata.loadNASAFlight, scanNASAFlights

    arguments
        projectRoot (1,:) char = pwd
    end

    % Default flight folder (v3.6.0+ branches on deployment mode):
    %   - Deployed: <projectRoot>/flight_data/Tail_687_1/. In deployed
    %     mode mainMenu sets projectRoot = userRoot (writable per-user
    %     data dir), and seedUserDataRoot ships the 3 bundled sample
    %     flights into <userRoot>/flight_data/Tail_687_1/ on first
    %     launch. Users can add more .mat files in that subfolder or in
    %     sibling subfolders of <userRoot>/flight_data/.
    %   - Dev mode: sibling of project root (matches runNASAFlight /
    %     scanNASAFlights convention — Tail_687_1/ lives next to
    %     adeptus-rainyday-tracking/).
    if isdeployed
        defaultFlightFolder = fullfile(projectRoot, 'flight_data', 'Tail_687_1');
    else
        defaultFlightFolder = fullfile(fileparts(projectRoot), 'Tail_687_1');
    end

    % Shared state — populated by onRescan, read by table/preview callbacks.
    % NOTE: row order in 'flights' matches row order in tblFlights.Data so
    % onTableSelection can use the row index directly.
    flights = struct.empty(0, 1);

    % Per-flight batch controls — one struct per row in the batch builder
    % table, indexed parallel to the rows in tblBatch. Order is determined
    % by the order flights were checked (1st checked → row 1). `flightIdx`
    % points back to the row in `flights` so the export step can find
    % `fullPath` etc.
    batchEntries = struct('name', {}, 'label', {}, 'rcs_dbsm', {}, ...
                          'max_duration_s', {}, 'waypoint_interval_s', {}, ...
                          'start_offset_s', {}, 'flightIdx', {});

    % ── Figure ────────────────────────────────────────────────────────
    fig = uifigure('Name', 'Flight Data Manager — NASA DASHlink Browser', ...
                   'Position', [80 100 1200 700]);

    % Root grid: toolbar / 3-pane workspace / action row
    rootGL = uigridlayout(fig, [3, 1]);
    rootGL.RowHeight   = {44, '1x', 50};
    rootGL.ColumnWidth = {'1x'};
    rootGL.RowSpacing  = 6;
    rootGL.Padding     = [10 10 10 10];

    % ── Top toolbar: folder picker (FULLY WIRED in 5f.1) ──────────────
    topGL = uigridlayout(rootGL, [1, 5]);
    topGL.Layout.Row    = 1;
    topGL.ColumnWidth   = {90, '1x', 100, 100, 50};
    topGL.ColumnSpacing = 6;
    topGL.Padding       = [0 0 0 0];

    uilabel(topGL, 'Text', 'Flight folder:', 'HorizontalAlignment', 'right');

    edtFolder = uieditfield(topGL, 'text', 'Value', defaultFlightFolder, ...
        'Tooltip', 'Folder containing NASA DASHlink .mat files');

    uibutton(topGL, 'Text', 'Browse…', ...
        'ButtonPushedFcn', @(~,~) onBrowse());

    uibutton(topGL, 'Text', 'Rescan', ...
        'ButtonPushedFcn', @(~,~) onRescan());

    uibutton(topGL, 'Text', '?', ...
        'FontSize', 14, 'FontWeight', 'bold', ...
        'Tooltip', 'Show Flight Data Manager workflow help', ...
        'ButtonPushedFcn', @(~,~) onHelp());

    % ── Middle: 3-pane workspace ──────────────────────────────────────
    midGL = uigridlayout(rootGL, [1, 3]);
    midGL.Layout.Row     = 2;
    midGL.ColumnWidth    = {460, '1x', 340};
    midGL.ColumnSpacing  = 8;
    midGL.Padding        = [0 0 0 0];

    % Left pane — Flight list (uitable)
    pnlList = uipanel(midGL, 'Title', 'Flights', 'FontWeight', 'bold');
    pnlList.Layout.Column = 1;
    pnlListGL = uigridlayout(pnlList, [2, 1]);
    pnlListGL.RowHeight   = {'1x', 22};
    pnlListGL.ColumnWidth = {'1x'};
    pnlListGL.RowSpacing  = 4;
    pnlListGL.Padding     = [6 6 6 6];

    tblFlights = uitable(pnlListGL, ...
        'ColumnName',     {'Use', 'File', 'Dur (s)', 'Max Alt (ft)', ...
                           'Max GS (kts)', 'Turn (°)'}, ...
        'ColumnFormat',   {'logical', 'char', 'numeric', 'numeric', ...
                           'numeric', 'numeric'}, ...
        'ColumnEditable', [true false false false false false], ...
        'ColumnWidth',    {36, 138, 50, 70, 70, 50}, ...
        'RowName',        {}, ...
        'CellSelectionCallback', @onTableSelection, ...
        'CellEditCallback',      @onFlightUseToggle);
    tblFlights.Layout.Row = 1;

    lblListStatus = uilabel(pnlListGL, ...
        'Text', '(empty — click Rescan to load flights)', ...
        'FontSize', 10, 'FontColor', [0.5 0.5 0.5]);
    lblListStatus.Layout.Row = 2;

    % Center pane — Preview (single-flight plot in 5f.2, multi-flight in 5f.4)
    pnlPreview = uipanel(midGL, 'Title', 'Preview', 'FontWeight', 'bold');
    pnlPreview.Layout.Column = 2;
    pnlPreviewGL = uigridlayout(pnlPreview, [2, 1]);
    pnlPreviewGL.RowHeight   = {'1x', 110};
    pnlPreviewGL.ColumnWidth = {'1x'};
    pnlPreviewGL.RowSpacing  = 4;
    pnlPreviewGL.Padding     = [6 6 6 6];

    axPreview = uiaxes(pnlPreviewGL);
    axPreview.Layout.Row = 1;
    title(axPreview, '(no flight selected)');
    xlabel(axPreview, 'Longitude (°)');
    ylabel(axPreview, 'Latitude (°)');
    grid(axPreview, 'on');
    axis(axPreview, 'equal');

    lblPreviewSummary = uilabel(pnlPreviewGL, ...
        'Text', 'Click a row in the flight list to preview that flight.', ...
        'FontName', 'Consolas', 'FontSize', 11, ...
        'WordWrap', 'on', ...
        'VerticalAlignment', 'top', ...
        'HorizontalAlignment', 'left');
    lblPreviewSummary.Layout.Row = 2;

    % Right pane — Batch builder (wired in 5f.4)
    pnlBatch = uipanel(midGL, 'Title', 'Batch Builder', 'FontWeight', 'bold');
    pnlBatch.Layout.Column = 3;
    pnlBatchGL = uigridlayout(pnlBatch, [4, 1]);
    pnlBatchGL.RowHeight   = {'1x', 28, 28, 32};
    pnlBatchGL.ColumnWidth = {'1x'};
    pnlBatchGL.RowSpacing  = 4;
    pnlBatchGL.Padding     = [6 6 6 6];

    tblBatch = uitable(pnlBatchGL, ...
        'ColumnName',     {'Name', 'Label', 'RCS', 'MaxDur', 'WpInt', 'Offset'}, ...
        'ColumnFormat',   {'char', 'char', 'numeric', 'numeric', 'numeric', 'numeric'}, ...
        'ColumnEditable', [true true true true true true], ...
        'ColumnWidth',    {90, 80, 40, 50, 40, 50}, ...
        'RowName',        {}, ...
        'CellEditCallback', @onBatchCellEdit);
    tblBatch.Layout.Row = 1;

    % Batch name row
    batchNameGL = uigridlayout(pnlBatchGL, [1, 2]);
    batchNameGL.Layout.Row    = 2;
    batchNameGL.ColumnWidth   = {78, '1x'};
    batchNameGL.ColumnSpacing = 4;
    batchNameGL.Padding       = [0 0 0 0];
    uilabel(batchNameGL, 'Text', 'Batch name:', 'HorizontalAlignment', 'right');
    edtBatchName = uieditfield(batchNameGL, 'text', ...
        'Value', defaultBatchName(), ...
        'Tooltip', 'Used as filename: config/targets/recorded_flight/<name>.json');

    % Ref origin row
    refGL = uigridlayout(pnlBatchGL, [1, 5]);
    refGL.Layout.Row     = 3;
    refGL.ColumnWidth    = {78, 32, '1x', 32, '1x'};
    refGL.ColumnSpacing  = 4;
    refGL.Padding        = [0 0 0 0];
    chkAutoRef = uicheckbox(refGL, 'Text', 'Auto ref', 'Value', true, ...
        'ValueChangedFcn', @(~,~) onAutoRefToggle());
    uilabel(refGL, 'Text', 'Lat:', 'HorizontalAlignment', 'right');
    edtRefLat = uieditfield(refGL, 'numeric', 'Value', 0, 'Enable', 'off');
    uilabel(refGL, 'Text', 'Lon:', 'HorizontalAlignment', 'right');
    edtRefLon = uieditfield(refGL, 'numeric', 'Value', 0, 'Enable', 'off');

    % Refresh button
    btnRefreshBatch = uibutton(pnlBatchGL, ...
        'Text', 'Refresh from checks', ...
        'ButtonPushedFcn', @(~,~) refreshBatchFromChecks());
    btnRefreshBatch.Layout.Row = 4;

    % ── Bottom: action buttons ────────────────────────────────────────
    botGL = uigridlayout(rootGL, [1, 5]);
    botGL.Layout.Row     = 3;
    botGL.ColumnWidth    = {160, 160, 200, '1x', 100};
    botGL.ColumnSpacing  = 8;
    botGL.Padding        = [0 4 0 4];

    btnViewGlobe = uibutton(botGL, 'Text', 'View on Globe', ...
        'FontSize', 12, 'Enable', 'off', ...
        'Tooltip', 'Visualize selected flights on a globe (wired in 5f.3)', ...
        'ButtonPushedFcn', @(~,~) onViewGlobe());

    btnPreviewNED = uibutton(botGL, 'Text', 'Preview NED Layout', ...
        'FontSize', 12, 'Enable', 'off', ...
        'Tooltip', 'Preview multi-flight scenario in NED coords (wired in 5f.4)', ...
        'ButtonPushedFcn', @(~,~) onPreviewNED());

    btnExport = uibutton(botGL, 'Text', 'Export to Path Editor', ...
        'FontSize', 12, 'Enable', 'off', ...
        'Tooltip', 'Write the batch as a recorded_flight target JSON and launch Path Editor', ...
        'ButtonPushedFcn', @(~,~) onExport());

    uilabel(botGL, 'Text', '');  % spacer

    uibutton(botGL, 'Text', 'Close', 'FontSize', 12, ...
        'ButtonPushedFcn', @(~,~) close(fig));

    % ── Callbacks ─────────────────────────────────────────────────────

    function onBrowse()
        startDir = edtFolder.Value;
        if ~isfolder(startDir); startDir = pwd; end
        sel = uigetdir(startDir, 'Select NASA flight data folder');
        if ischar(sel) && isfolder(sel)
            edtFolder.Value = sel;
        end
        figure(fig);  % regain focus from the modal picker
    end

    function onRescan()
        folder = edtFolder.Value;
        if ~isfolder(folder)
            uialert(fig, sprintf('Folder does not exist:\n%s', folder), ...
                'Rescan Failed');
            return;
        end
        lblListStatus.Text = 'Scanning…';
        drawnow;
        try
            flights = trackbench.flightdata.scanFlightFolder(folder);
        catch ME
            uialert(fig, ME.message, 'Rescan Failed');
            lblListStatus.Text = '(scan error — see alert)';
            return;
        end

        nFlights = numel(flights);
        if nFlights == 0
            tblFlights.Data = {};
            lblListStatus.Text = '(no DASHlink .mat files found in this folder)';
            btnViewGlobe.Enable = 'off';
            clearPreview();
            refreshBatchFromChecks();  % clears batch table + disables Preview NED
            return;
        end

        % Build table data: {Use, File, Dur, MaxAlt, MaxGS, Turn}.
        % Row order matches `flights` so onTableSelection can index directly.
        data = cell(nFlights, 6);
        for k = 1:nFlights
            f = flights(k);
            data{k, 1} = false;
            data{k, 2} = f.file;
            data{k, 3} = round(f.duration_s);
            data{k, 4} = round(f.maxAlt_ft);
            data{k, 5} = round(f.maxGS_kts);
            data{k, 6} = round(f.totalTurn_deg);
        end
        tblFlights.Data = data;

        lblListStatus.Text = sprintf( ...
            '%d flight(s) — check rows to include in a batch.', nFlights);
        btnViewGlobe.Enable = 'on';
        clearPreview();
        refreshBatchFromChecks();  % rebuild batch table (any prior checks cleared)
    end

    function onViewGlobe()
        % Identify checked flights from column 1 of tblFlights.Data
        data = tblFlights.Data;
        if isempty(data); return; end
        checkedMask = false(size(data, 1), 1);
        for k = 1:size(data, 1)
            v = data{k, 1};
            if islogical(v)
                checkedMask(k) = v;
            elseif isnumeric(v)
                checkedMask(k) = v ~= 0;
            end
        end

        checkedIdx = find(checkedMask);
        if isempty(checkedIdx)
            uialert(fig, ...
                'Check at least one flight in the "Use" column before opening the globe view.', ...
                'No flights selected');
            return;
        end

        % Build geoTrajectory per checked flight + collect lat/lon for framing.
        nSel    = numel(checkedIdx);
        routes  = cell(nSel, 1);
        labels  = cell(nSel, 1);
        allLats = [];
        allLons = [];
        failed  = {};

        for j = 1:nSel
            f = flights(checkedIdx(j));
            try
                raw = load(f.fullPath);
                lat  = double(raw.LATP.data(:));
                lon  = double(raw.LONP.data(:));
                alt4 = double(raw.ALT.data(:));
                gs4  = double(raw.GS.data(:));

                nLat = numel(lat);
                alt = alt4(1:4:end); alt = alt(1:min(end, nLat));
                gs  = gs4(1:4:end);  gs  = gs(1:min(end, nLat));

                validIdxMask = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
                idx = find(validIdxMask);
                if isempty(idx); failed{end+1} = f.file; continue; end %#ok<AGROW>

                % Subsample at 10 s intervals for smooth rendering without overload.
                step = 10;
                wpIdx = idx(1):step:idx(end);
                if wpIdx(end) ~= idx(end); wpIdx(end+1) = idx(end); end %#ok<AGROW>

                lla = [lat(wpIdx), lon(wpIdx), alt(wpIdx) * 0.3048];  % ft → m
                toa = (0:numel(wpIdx)-1)' * step;
                toa(end) = wpIdx(end) - wpIdx(1);

                routes{j}  = geoTrajectory(lla, toa);
                labels{j}  = f.file;
                allLats    = [allLats; lla(:,1)]; %#ok<AGROW>
                allLons    = [allLons; lla(:,2)]; %#ok<AGROW>
            catch ME
                failed{end+1} = sprintf('%s (%s)', f.file, ME.message); %#ok<AGROW>
            end
        end

        keep = ~cellfun(@isempty, routes);
        routes = routes(keep);
        labels = labels(keep); %#ok<NASGU>

        if isempty(routes)
            uialert(fig, ...
                sprintf('No flights could be loaded for globe view.\nFailed: %s', ...
                    strjoin(failed, '; ')), ...
                'Globe view failed');
            return;
        end

        % Launch globe viewer in its own figure (don't try to embed in FDM).
        viewer = trackingGlobeViewer('Basemap', 'streets-dark', ...
            'TrackHistoryDepth', 5000);

        palette = [0.20 0.70 1.00;   % cyan-blue
                   1.00 0.55 0.10;   % orange
                   0.85 0.20 0.85;   % magenta
                   0.20 0.85 0.30;   % green
                   1.00 0.90 0.20];  % yellow

        for j = 1:numel(routes)
            clr = palette(mod(j-1, size(palette,1)) + 1, :);
            plotTrajectory(viewer, routes{j}, 'Color', clr, 'LineWidth', 3);
        end

        % Camera: union midpoint, altitude scaled to extent.
        midLat = (min(allLats) + max(allLats)) / 2;
        midLon = (min(allLons) + max(allLons)) / 2;
        latSpan = max(allLats) - min(allLats);
        lonSpan = max(allLons) - min(allLons);
        % 111 km per degree of lat; ~85 km/deg lon at mid-US latitudes.
        extentKm = max(latSpan * 111, lonSpan * 85);
        camAlt = max(50e3, extentKm * 1.4 * 1000);  % 1.4x to give margin
        campos(viewer, midLat, midLon, camAlt);
        drawnow;

        if ~isempty(failed)
            % Non-blocking notice; globe still rendered for whatever loaded.
            uialert(fig, ...
                sprintf('Globe rendered, but %d flight(s) failed:\n%s', ...
                    numel(failed), strjoin(failed, newline)), ...
                'Partial render', 'Icon', 'warning');
        end
    end

    function onPreviewNED()
        if isempty(batchEntries)
            uialert(fig, 'Check at least one flight first.', 'Empty batch');
            return;
        end

        % If auto-ref is on (or the user never set a value), make sure the
        % displayed lat/lon are fresh before we use them.
        if chkAutoRef.Value || (edtRefLat.Value == 0 && edtRefLon.Value == 0)
            updateAutoRefDisplay();
        end
        refLat = edtRefLat.Value;
        refLon = edtRefLon.Value;

        cla(axPreview);
        hold(axPreview, 'on');

        palette = [0.20 0.70 1.00; 1.00 0.55 0.10; 0.85 0.20 0.85; ...
                   0.20 0.85 0.30; 1.00 0.90 0.20];

        sumLines = {};
        for j = 1:numel(batchEntries)
            e = batchEntries(j);
            f = flights(e.flightIdx);
            try
                fd = trackbench.flightdata.loadNASAFlight( ...
                    string(f.fullPath), ...
                    'MaxDuration',      e.max_duration_s, ...
                    'WaypointInterval', e.waypoint_interval_s, ...
                    'RefLat',           refLat, ...
                    'RefLon',           refLon);
                wp = fd.waypoints;
                % start_offset_s is displayed as a label only; at the
                % geometry level NED waypoints don't move with offset —
                % that's a runtime concern handled by the scenario builder.
                clr = palette(mod(j-1, size(palette,1)) + 1, :);
                plot(axPreview, wp(:,2)/1000, wp(:,1)/1000, '-', ...
                    'Color', clr, 'LineWidth', 1.8, ...
                    'DisplayName', sprintf('%s (+%ds)', e.label, e.start_offset_s));
                plot(axPreview, wp(1,2)/1000, wp(1,1)/1000, 'o', ...
                    'Color', clr, 'MarkerSize', 7, 'MarkerFaceColor', clr, ...
                    'HandleVisibility', 'off');
                plot(axPreview, wp(end,2)/1000, wp(end,1)/1000, 's', ...
                    'Color', clr, 'MarkerSize', 7, 'MarkerFaceColor', clr, ...
                    'HandleVisibility', 'off');
                sumLines{end+1} = sprintf('%s: %d wp, offset %ds', ...
                    e.label, fd.numWaypoints, e.start_offset_s); %#ok<AGROW>
            catch ME
                sumLines{end+1} = sprintf('%s: FAILED (%s)', e.label, ME.message); %#ok<AGROW>
            end
        end

        plot(axPreview, 0, 0, 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'y', ...
            'DisplayName', 'Radar (origin)');
        hold(axPreview, 'off');
        grid(axPreview, 'on');
        axis(axPreview, 'equal');
        title(axPreview, sprintf('NED preview — ref (%.3f°, %.3f°)', refLat, refLon));
        xlabel(axPreview, 'East (km)');
        ylabel(axPreview, 'North (km)');
        legend(axPreview, 'Location', 'best');

        if isempty(sumLines)
            lblPreviewSummary.Text = '(no batch entries to plot)';
        else
            lblPreviewSummary.Text = strjoin(sumLines, newline);
        end
    end

    function onExport()
        % Write batchEntries as a recorded_flight target JSON under
        % config/targets/recorded_flight/<batch_name>.json. Optionally
        % launch Path Editor afterward (no auto-load — the user
        % navigates to the file from Path Editor's Open dialog).
        if isempty(batchEntries)
            uialert(fig, 'Check at least one flight first.', 'Empty batch');
            return;
        end

        % Validate batch name and strip any extension the user typed.
        rawName = strtrim(edtBatchName.Value);
        if isempty(rawName)
            uialert(fig, 'Batch name is required.', 'Export Failed');
            return;
        end
        [~, batchName] = fileparts(rawName);
        if isempty(batchName)
            uialert(fig, 'Batch name is required.', 'Export Failed');
            return;
        end

        % Refresh ref lat/lon in auto mode (or both fields at 0).
        if chkAutoRef.Value || (edtRefLat.Value == 0 && edtRefLon.Value == 0)
            updateAutoRefDisplay();
        end
        refLat = edtRefLat.Value;
        refLon = edtRefLon.Value;

        % Confirm overwrite if the file already exists.
        outPath = fullfile(projectRoot, 'config', 'targets', ...
            'recorded_flight', [batchName '.json']);
        if isfile(outPath)
            choice = uiconfirm(fig, sprintf( ...
                ['A target file already exists at:\n%s\n\n' ...
                 'Overwrite?'], outPath), ...
                'Overwrite?', ...
                'Options',       {'Overwrite', 'Cancel'}, ...
                'DefaultOption', 'Cancel', ...
                'CancelOption',  'Cancel', ...
                'Icon',          'warning');
            if strcmp(choice, 'Cancel'); return; end
        end

        % Build + write.
        try
            writtenPath = trackbench.flightdata.buildBatchTargetJSON( ...
                batchEntries, flights, ...
                'ProjectRoot', projectRoot, ...
                'BatchName',   batchName, ...
                'RefLat',      refLat, ...
                'RefLon',      refLon, ...
                'Overwrite',   true);
        catch ME
            uialert(fig, ME.message, 'Export Failed');
            return;
        end

        % Offer to launch Path Editor (no auto-load; user opens manually).
        msg = sprintf([ ...
            'Target file written:\n%s\n\n' ...
            'Open Path Editor now?  In the editor, use Open Scenario ' ...
            'or Import Targets to load this file.'], writtenPath);
        choice = uiconfirm(fig, msg, 'Export complete', ...
            'Options',       {'Open Path Editor', 'Stay here'}, ...
            'DefaultOption', 'Stay here', ...
            'CancelOption',  'Stay here', ...
            'Icon',          'success');
        if strcmp(choice, 'Open Path Editor')
            try
                pathEditor(projectRoot);
            catch ME
                uialert(fig, ME.message, 'Could not launch Path Editor');
            end
        end
    end

    function onHelp()
        % Multi-paragraph workflow help. Displayed via uialert (modal).
        msg = sprintf([ ...
            'FLIGHT DATA MANAGER — workflow\n\n' ...
            '1. Pick a folder of NASA DASHlink .mat files and click Rescan.\n' ...
            '2. Click a row to preview a flight in lat/lon. Toggle Use to\n' ...
            '   include it in a batch.\n' ...
            '3. View on Globe shows all checked flights as geoTrajectory\n' ...
            '   lines on a 3D globe (no simulation).\n' ...
            '4. Batch Builder (right pane) — edit per-flight Name, Label,\n' ...
            '   RCS, MaxDur, WpInt, and Offset for each checked flight.\n' ...
            '5. Auto ref origin = mean midpoint of checked flights; uncheck\n' ...
            '   to type lat/lon manually.\n' ...
            '6. Preview NED Layout plots the merged scenario in km E/N.\n' ...
            '7. Export to Path Editor writes a recorded_flight target JSON\n' ...
            '   to config/targets/recorded_flight/<batch_name>.json.\n\n' ...
            'START OFFSET CAVEAT — a flight with start_offset_s > 0 SITS\n' ...
            'at its first waypoint from scenario t=0 to t=offset, then\n' ...
            'begins moving. If that waypoint is in radar range you will\n' ...
            'see a stationary detection during the pre-offset window.']);
        uialert(fig, msg, 'Flight Data Manager — Help', 'Icon', 'info');
    end

    function onTableSelection(~, event)
        % Single-row click → preview that flight in the center pane.
        % The checkbox column is also editable, and clicking it ALSO
        % fires this callback (cell selection happens before cell edit),
        % which is the intended UX: clicking 'Use' previews the flight.
        if isempty(event.Indices); return; end
        rowIdx = event.Indices(1, 1);
        if rowIdx < 1 || rowIdx > numel(flights); return; end
        previewFlight(flights(rowIdx));
    end

    function previewFlight(f)
        % Lat/lon top-down plot of one flight's airborne sub-track.
        % Re-loads the .mat file to get the full sample stream
        % (scanFlightFolder stores summary stats only). Cheap (~50 ms
        % per flight, single load).
        cla(axPreview);
        try
            raw = load(f.fullPath);
            lat = double(raw.LATP.data(:));
            lon = double(raw.LONP.data(:));
            gs4 = double(raw.GS.data(:));
            gs  = gs4(1:4:end); gs = gs(1:min(end, numel(lat)));
            valid = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
            idx = find(valid);
            if isempty(idx)
                title(axPreview, sprintf('%s — no airborne window', ...
                    f.file), 'Interpreter', 'none');
                lblPreviewSummary.Text = '(no airborne samples > 50 kts)';
                return;
            end
            plot(axPreview, lon(idx), lat(idx), 'b-', 'LineWidth', 1.6);
            hold(axPreview, 'on');
            plot(axPreview, lon(idx(1)),   lat(idx(1)),   'go', ...
                'MarkerSize', 8, 'MarkerFaceColor', 'g');  % start
            plot(axPreview, lon(idx(end)), lat(idx(end)), 'rs', ...
                'MarkerSize', 8, 'MarkerFaceColor', 'r');  % end
            hold(axPreview, 'off');
            grid(axPreview, 'on');
            axis(axPreview, 'equal');
            title(axPreview, f.file, 'Interpreter', 'none');
            xlabel(axPreview, 'Longitude (°)');
            ylabel(axPreview, 'Latitude (°)');
        catch ME
            title(axPreview, sprintf('%s — preview error', f.file), ...
                'Interpreter', 'none');
            lblPreviewSummary.Text = sprintf('Preview failed: %s', ME.message);
            return;
        end

        % Approximate route distance (km) via flat-Earth lat/lon delta.
        distKm = sqrt((f.endLat - f.startLat)^2 + ...
                      (f.endLon - f.startLon)^2) * 111;

        lblPreviewSummary.Text = sprintf([ ...
            'File         : %s\n' ...
            'Duration     : %d s   (%.1f min)\n' ...
            'Start → End  : (%7.3f, %8.3f) → (%7.3f, %8.3f)  ≈ %.0f km\n' ...
            'Max altitude : %d ft   |   Max ground speed: %d kts\n' ...
            'Maneuvering  : max turn %.1f°   total turn %.0f°'], ...
            f.file, round(f.duration_s), f.duration_s/60, ...
            f.startLat, f.startLon, f.endLat, f.endLon, distKm, ...
            round(f.maxAlt_ft), round(f.maxGS_kts), ...
            f.maxTurn_deg, f.totalTurn_deg);
    end

    function clearPreview()
        cla(axPreview);
        title(axPreview, '(no flight selected)');
        xlabel(axPreview, 'Longitude (°)');
        ylabel(axPreview, 'Latitude (°)');
        grid(axPreview, 'on');
        lblPreviewSummary.Text = ...
            'Click a row in the flight list to preview that flight.';
    end

    % ── Batch builder helpers (5f.4) ──────────────────────────────────

    function onFlightUseToggle(~, event)
        % Fires when a checkbox in the 'Use' column is toggled. Rebuild
        % the batch table from the current set of checks.
        if isempty(event.Indices); return; end
        if event.Indices(2) ~= 1; return; end  % only the Use column
        refreshBatchFromChecks();
    end

    function refreshBatchFromChecks()
        % Rebuild batchEntries from whatever rows are currently checked,
        % preserving prior per-flight edits where possible.
        data = tblFlights.Data;
        if isempty(data)
            batchEntries = emptyBatchArray();
            tblBatch.Data = {};
            updateAutoRefDisplay();
            btnPreviewNED.Enable = 'off';
            btnExport.Enable     = 'off';
            return;
        end

        checkedIdx = [];
        for k = 1:size(data, 1)
            v = data{k, 1};
            if (islogical(v) && v) || (isnumeric(v) && v ~= 0)
                checkedIdx(end+1) = k; %#ok<AGROW>
            end
        end

        if isempty(checkedIdx)
            batchEntries = emptyBatchArray();
            tblBatch.Data = {};
            updateAutoRefDisplay();
            btnPreviewNED.Enable = 'off';
            btnExport.Enable     = 'off';
            return;
        end

        % Preserve existing edits when possible (match by flightIdx).
        newEntries = repmat(emptyBatchEntry(), 1, numel(checkedIdx));
        for j = 1:numel(checkedIdx)
            flightIdx = checkedIdx(j);
            f = flights(flightIdx);
            existing = findExistingByFlightIdx(batchEntries, flightIdx);
            if ~isempty(existing)
                newEntries(j)       = existing;
                newEntries(j).label = sprintf('Target_%d', j);  % renumber labels
            else
                [~, baseName] = fileparts(f.file);
                e = emptyBatchEntry();
                e.name                = baseName;
                e.label               = sprintf('Target_%d', j);
                e.rcs_dbsm            = 10;
                e.max_duration_s      = min(600, round(f.duration_s));
                e.waypoint_interval_s = 10;
                e.start_offset_s      = 0;
                e.flightIdx           = flightIdx;
                newEntries(j) = e;
            end
        end
        batchEntries = newEntries;

        % Push to table.
        nB = numel(batchEntries);
        tableData = cell(nB, 6);
        for j = 1:nB
            e = batchEntries(j);
            tableData{j,1} = e.name;
            tableData{j,2} = e.label;
            tableData{j,3} = e.rcs_dbsm;
            tableData{j,4} = e.max_duration_s;
            tableData{j,5} = e.waypoint_interval_s;
            tableData{j,6} = e.start_offset_s;
        end
        tblBatch.Data = tableData;

        updateAutoRefDisplay();
        btnPreviewNED.Enable = 'on';
        btnExport.Enable     = 'on';
    end

    function onBatchCellEdit(~, event)
        % Sync edits from tblBatch back into the batchEntries struct array.
        if isempty(event.Indices); return; end
        r = event.Indices(1); c = event.Indices(2);
        if r < 1 || r > numel(batchEntries); return; end
        v = event.NewData;
        switch c
            case 1, batchEntries(r).name                = char(v);
            case 2, batchEntries(r).label               = char(v);
            case 3, batchEntries(r).rcs_dbsm            = double(v);
            case 4, batchEntries(r).max_duration_s      = double(v);
            case 5, batchEntries(r).waypoint_interval_s = double(v);
            case 6, batchEntries(r).start_offset_s      = double(v);
        end
    end

    function onAutoRefToggle()
        % Toggle the ref lat/lon fields between auto (read-only, fed by
        % flight union midpoint) and manual (user-editable).
        if chkAutoRef.Value
            edtRefLat.Enable = 'off';
            edtRefLon.Enable = 'off';
            updateAutoRefDisplay();
        else
            edtRefLat.Enable = 'on';
            edtRefLon.Enable = 'on';
        end
    end

    function updateAutoRefDisplay()
        % In auto mode, refresh the displayed ref lat/lon from the union
        % midpoint of the currently checked flights. In manual mode, do
        % nothing — preserve the user's typed value.
        if ~chkAutoRef.Value; return; end
        if isempty(batchEntries)
            edtRefLat.Value = 0;
            edtRefLon.Value = 0;
            return;
        end
        idxList = [batchEntries.flightIdx];
        latsSum = [flights(idxList).startLat] + [flights(idxList).endLat];
        lonsSum = [flights(idxList).startLon] + [flights(idxList).endLon];
        % Each flight contributes (start + end); dividing by 2 gives the
        % per-flight midpoint, then mean() averages across flights.
        edtRefLat.Value = mean(latsSum) / 2;
        edtRefLon.Value = mean(lonsSum) / 2;
    end

    function e = emptyBatchEntry()
        e = struct('name', '', 'label', '', 'rcs_dbsm', 10, ...
                   'max_duration_s', 600, 'waypoint_interval_s', 10, ...
                   'start_offset_s', 0, 'flightIdx', 0);
    end

    function arr = emptyBatchArray()
        arr = struct('name', {}, 'label', {}, 'rcs_dbsm', {}, ...
                     'max_duration_s', {}, 'waypoint_interval_s', {}, ...
                     'start_offset_s', {}, 'flightIdx', {});
    end

    function out = findExistingByFlightIdx(entries, flightIdx)
        out = [];
        for k = 1:numel(entries)
            if entries(k).flightIdx == flightIdx
                out = entries(k);
                return;
            end
        end
    end

    function name = defaultBatchName()
        name = char(datetime("now", "Format", "'nasa_batch_'yyyyMMdd_HHmm"));
    end

    if nargout > 0
        varargout{1} = fig;
    end
end
