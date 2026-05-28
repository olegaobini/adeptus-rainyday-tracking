# HANDOFF — 5f.3 (Globe view) + 5f.4 (Batch builder)

**Audience:** Cowork (Opus) session
**Originator:** Claude (Sonnet) chat, 2026-05-25
**Project root:** `C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking`
**Active branch:** `Michael---Working_on_pathEditor`
**MATLAB version:** R2025b, Windows
**When done:** Hand back to Michael for end-to-end verification; he passes 5f.5 + 5f.6 back to Claude.

---

## Mission

Wire up two pieces of the new Flight Data Manager GUI (`scripts/flightDataManagerGUI.m`):
1. **5f.3** — "View on Globe" button: launches `trackingGlobeViewer` with checked flights as `geoTrajectory` lines (true geodetic positions, NO simulation).
2. **5f.4** — Right-pane batch builder + "Preview NED Layout" button: per-flight controls (rcs, max_duration, waypoint_interval, start_offset_s) keyed to which rows are checked in the left pane, plus a merged-scenario NED preview in the center pane.

Both substeps modify `flightDataManagerGUI.m` only — no other file edits required for these two substeps. (5f.5 will touch the schema; that's Claude's job after you're done.)

---

## Prerequisites — read first

Read these files in this order to load context:

1. `CHECKPOINT.md` — entire file, particularly the "What's NEXT" section to understand the 5f series.
2. `scripts/flightDataManagerGUI.m` — current state after 5f.1 + 5f.2. The skeleton + file browser + single-flight preview are done.
3. `scripts/legacy/runNASAFlightGlobe.m` — REFERENCE for the globe + `geoTrajectory` pattern. Note: it builds a trackingScenario and runs detections. **For 5f.3 we want only the visualization portion** (lines ~80–115: `trackingGlobeViewer` setup, `plotTrajectory`, `campos`). Skip the scenario / detection / tracker code.
4. `scripts/viewNASAFlightGlobe.m` — REFERENCE for the post-run globe viewer pattern. Useful for the `geoTrajectory` build, especially `flightRoutes{k} = geoTrajectory(lla, toa);`.
5. `src/+trackbench/+flightdata/loadNASAFlight.m` — useful loader. NOTE: returns NED waypoints, not lat/lon. For 5f.3 we need the **raw lat/lon stream**, so read from `raw.LATP.data` / `raw.LONP.data` / `raw.ALT.data` / `raw.GS.data` directly (see `previewFlight` in `flightDataManagerGUI.m` lines ~230–250 for the pattern).
6. `src/+trackbench/+flightdata/scanFlightFolder.m` — new in 5f.2. Returns the struct array consumed by the GUI. Field reference, especially `fullPath`.
7. `config/targets/recorded_flight/nasa_multi_target.json` — current JSON schema for batch flights. 5f.4 batch builder should produce the same shape (5f.5 actually writes the file, but 5f.4's data model needs to match).

---

## 5f.3 — Globe view

### Goal

When the user has flights loaded and at least one box checked in the "Use" column, the "View on Globe" button:
1. Opens a `trackingGlobeViewer` window in a new uifigure (NOT inside the FDM window — globe viewers want a full figure).
2. Plots each checked flight as a `geoTrajectory` colored uniquely (cycle through a 5-color palette).
3. Frames the camera over the union midpoint of all flights at an altitude that makes them all visible (~200–500 km altitude depending on extent).
4. Does NOT simulate anything. No sensors, no detections, no tracker. Pure visualization.

### Code changes required in `flightDataManagerGUI.m`

**Change 1 — Store button handles.** The current code creates the bottom-row buttons without retaining references:

```matlab
uibutton(botGL, 'Text', 'View on Globe', ...
    'FontSize', 12, 'Enable', 'off', ...
    ...);
```

Change all 3 action buttons (`View on Globe`, `Preview NED Layout`, `Export to Path Editor`) to assign references:

```matlab
btnViewGlobe = uibutton(botGL, 'Text', 'View on Globe', ...);
btnPreviewNED = uibutton(botGL, 'Text', 'Preview NED Layout', ...);
btnExport = uibutton(botGL, 'Text', 'Export to Path Editor', ...);
```

These references will be needed in 5f.3 (enable globe button), 5f.4 (enable NED button), and 5f.5 (enable export button — Claude's territory).

**Change 2 — Enable the globe button when flights are loaded.** At the end of `onRescan` (after the `nFlights > 0` branch populates the table), add:

```matlab
btnViewGlobe.Enable = 'on';
```

In the empty-folder branch (`nFlights == 0`), keep it disabled:

```matlab
btnViewGlobe.Enable = 'off';
```

**Change 3 — Implement `onViewGlobe`.** Replace the placeholder body:

```matlab
function onViewGlobe()
    % Identify checked flights from column 1 of tblFlights.Data
    data = tblFlights.Data;
    if isempty(data); return; end
    checkedMask = false(size(data, 1), 1);
    for k = 1:size(data, 1)
        v = data{k, 1};
        if islogical(v); checkedMask(k) = v;
        elseif isnumeric(v); checkedMask(k) = v ~= 0;
        end
    end

    checkedIdx = find(checkedMask);
    if isempty(checkedIdx)
        uialert(fig, ...
            'Check at least one flight in the "Use" column before opening the globe view.', ...
            'No flights selected');
        return;
    end

    % Build geoTrajectory per checked flight + union midpoint
    nSel = numel(checkedIdx);
    routes  = cell(nSel, 1);
    labels  = cell(nSel, 1);
    allLats = [];
    allLons = [];
    failed  = {};

    for j = 1:nSel
        f = flights(checkedIdx(j));
        try
            raw = load(f.fullPath);
            lat = double(raw.LATP.data(:));
            lon = double(raw.LONP.data(:));
            alt4 = double(raw.ALT.data(:));
            gs4  = double(raw.GS.data(:));

            nLat = numel(lat);
            alt = alt4(1:4:end); alt = alt(1:min(end, nLat));
            gs  = gs4(1:4:end);  gs  = gs(1:min(end, nLat));

            valid = (lat ~= 0) & (abs(lon) > 1) & (gs > 50);
            idx = find(valid);
            if isempty(idx); failed{end+1} = f.file; continue; end %#ok<AGROW>

            % Subsample at 10 s intervals for smooth rendering without overload
            step = 10;
            wpIdx = idx(1):step:idx(end);
            if wpIdx(end) ~= idx(end); wpIdx(end+1) = idx(end); end

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

    valid = ~cellfun(@isempty, routes);
    routes = routes(valid);
    labels = labels(valid);

    if isempty(routes)
        uialert(fig, ...
            sprintf('No flights could be loaded for globe view.\nFailed: %s', ...
                strjoin(failed, '; ')), ...
            'Globe view failed');
        return;
    end

    % Launch globe viewer in its own figure
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

    % Camera: union midpoint, altitude scaled to extent
    midLat = (min(allLats) + max(allLats)) / 2;
    midLon = (min(allLons) + max(allLons)) / 2;
    latSpan = max(allLats) - min(allLats);
    lonSpan = max(allLons) - min(allLons);
    % 111 km per degree of lat; ~85 km/deg lon at mid-US latitudes
    extentKm = max(latSpan * 111, lonSpan * 85);
    camAlt = max(50e3, extentKm * 1.4 * 1000);  % 1.4x to give margin
    campos(viewer, midLat, midLon, camAlt);
    drawnow;

    if ~isempty(failed)
        % Non-blocking notice; globe still rendered for whatever loaded
        uialert(fig, ...
            sprintf('Globe rendered, but %d flight(s) failed:\n%s', ...
                numel(failed), strjoin(failed, newline)), ...
            'Partial render', 'Icon', 'warning');
    end
end
```

### 5f.3 verification (Cowork should self-verify before handing back)

In MATLAB:

```matlab
clear classes; clear all
cd 'C:\Users\Admin\OneDrive - UW\2026\MATLAB\Capstone\Git\adeptus-rainyday-tracking\scripts'
mainMenu
```

Then in FDM:
1. Click Rescan — table populates with 3 flights. **View on Globe button becomes enabled.**
2. Click View on Globe with no boxes checked → alert "Check at least one flight…"
3. Check 1 box → click View on Globe → globe opens with single colored trajectory, camera framed.
4. Check 2–3 boxes → click View on Globe → multiple colored trajectories visible, camera frames the union.
5. Close globe → can re-open it. Each click opens a fresh globe (no leftover state).
6. Click Rescan again after pointing at an empty folder → button disables again.

---

## 5f.4 — Batch builder + NED preview

### Goal

Replace the right-pane placeholder with a per-flight controls panel + global controls. Wire up `onPreviewNED` to show the merged scenario in the center pane (replacing the single-flight preview).

### Data model

Per-flight controls (one row per CHECKED flight in the left pane):

| Field | Type | Default |
|---|---|---|
| name | char | `<basename_no_ext>` (e.g. `687200104121330`) |
| label | char | `Target_<k>` where k is 1-indexed in the batch |
| rcs_dbsm | numeric | 10 |
| max_duration_s | numeric | min(600, flight duration) |
| waypoint_interval_s | numeric | 10 |
| start_offset_s | numeric | 0 |

Global controls:
- `batch_name` (text edit, default `nasa_batch_<timestamp>`, format `nasa_batch_YYYYMMDD_HHMM`)
- `ref_lat` (numeric edit) + `ref_lon` (numeric edit) with an "Auto" checkbox
  - When Auto is checked: fields are disabled and read-only, display the auto-computed union midpoint of checked flights
  - When unchecked: fields are editable, user can override

Add a third shared state variable to the top of `flightDataManagerGUI.m`:

```matlab
% Per-flight batch controls — one struct per row in the batch builder
% table, indexed parallel to the rows in tblBatch. Order is determined
% by the order flights were checked (1st checked → row 1).
batchEntries = struct('name', {}, 'label', {}, 'rcs_dbsm', {}, ...
                      'max_duration_s', {}, 'waypoint_interval_s', {}, ...
                      'start_offset_s', {}, 'flightIdx', {});
```

`flightIdx` points back to the row in `flights` (so the export step can find `fullPath` etc.).

### Code changes required in `flightDataManagerGUI.m`

**Change 1 — Replace the right-pane placeholder.**

The current Batch Builder pane is just a placeholder uilabel. Replace it with a panel containing:
- A `uitable` (`tblBatch`) for per-flight controls — 6 editable columns from the data model above
- Below it: a `uigridlayout` with global controls:
  - Row 1: `[Label "Batch name:"] [uieditfield batch_name]`
  - Row 2: `[Checkbox "Auto ref origin"] [Label "Lat:"] [uieditfield_numeric ref_lat] [Label "Lon:"] [uieditfield_numeric ref_lon]`
  - Row 3: `[Button "Refresh from checks"]`

Suggested layout (right pane = 340 px wide, plenty of room):

```matlab
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
```

**Change 2 — Hook checkbox toggles in the flight list.**

Add a `CellEditCallback` to `tblFlights`:

```matlab
tblFlights = uitable(pnlListGL, ...
    ...
    'CellSelectionCallback', @onTableSelection, ...
    'CellEditCallback',      @onFlightUseToggle);
```

```matlab
function onFlightUseToggle(~, event)
    if isempty(event.Indices); return; end
    if event.Indices(2) ~= 1; return; end  % only the Use column
    refreshBatchFromChecks();
end
```

**Change 3 — Implement `refreshBatchFromChecks`.**

```matlab
function refreshBatchFromChecks()
    data = tblFlights.Data;
    if isempty(data)
        batchEntries = struct('name', {}, 'label', {}, 'rcs_dbsm', {}, ...
                              'max_duration_s', {}, 'waypoint_interval_s', {}, ...
                              'start_offset_s', {}, 'flightIdx', {});
        tblBatch.Data = {};
        updateAutoRefDisplay();
        btnPreviewNED.Enable = 'off';
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
        batchEntries = struct('name', {}, 'label', {}, 'rcs_dbsm', {}, ...
                              'max_duration_s', {}, 'waypoint_interval_s', {}, ...
                              'start_offset_s', {}, 'flightIdx', {});
        tblBatch.Data = {};
        updateAutoRefDisplay();
        btnPreviewNED.Enable = 'off';
        return;
    end

    % Preserve existing edits when possible (match by flightIdx)
    newEntries = repmat(emptyBatchEntry(), 1, numel(checkedIdx));
    for j = 1:numel(checkedIdx)
        flightIdx = checkedIdx(j);
        f = flights(flightIdx);
        existing = findExistingByFlightIdx(batchEntries, flightIdx);
        if ~isempty(existing)
            newEntries(j) = existing;
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

    % Push to table
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
end

function e = emptyBatchEntry()
    e = struct('name', '', 'label', '', 'rcs_dbsm', 10, ...
               'max_duration_s', 600, 'waypoint_interval_s', 10, ...
               'start_offset_s', 0, 'flightIdx', 0);
end

function out = findExistingByFlightIdx(entries, flightIdx)
    out = [];
    for k = 1:numel(entries)
        if entries(k).flightIdx == flightIdx; out = entries(k); return; end
    end
end
```

**Change 4 — Implement `onBatchCellEdit`.**

```matlab
function onBatchCellEdit(~, event)
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
```

**Change 5 — Auto ref origin.**

```matlab
function onAutoRefToggle()
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
    if ~chkAutoRef.Value; return; end  % manual mode — don't overwrite
    if isempty(batchEntries)
        edtRefLat.Value = 0; edtRefLon.Value = 0;
        return;
    end
    allLats = [flights([batchEntries.flightIdx]).startLat] + ...
              [flights([batchEntries.flightIdx]).endLat];
    allLons = [flights([batchEntries.flightIdx]).startLon] + ...
              [flights([batchEntries.flightIdx]).endLon];
    edtRefLat.Value = mean(allLats) / 2;
    edtRefLon.Value = mean(allLons) / 2;
end
```

(Note: the `/2` is because each flight contributes start + end, so dividing the sum by `2 × nFlights` gives a midpoint average. Cleaner alternative: collect min/max per flight and use the overall midpoint. Cowork can choose.)

**Change 6 — Default batch name helper.**

```matlab
function name = defaultBatchName()
    name = char(datetime("now", "Format", "'nasa_batch_'yyyyMMdd_HHmm"));
end
```

**Change 7 — Implement `onPreviewNED`.**

This computes the merged NED layout that Path Editor will receive, and plots it in the center pane (replacing the single-flight preview).

```matlab
function onPreviewNED()
    if isempty(batchEntries)
        uialert(fig, 'Check at least one flight first.', 'Empty batch');
        return;
    end

    refLat = edtRefLat.Value;
    refLon = edtRefLon.Value;
    if chkAutoRef.Value || refLat == 0
        updateAutoRefDisplay();
        refLat = edtRefLat.Value;
        refLon = edtRefLon.Value;
    end

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
                'MaxDuration', e.max_duration_s, ...
                'WaypointInterval', e.waypoint_interval_s, ...
                'RefLat', refLat, ...
                'RefLon', refLon);
            wp = fd.waypoints;
            % Offset is plotted as a label only; the actual time-zero shift
            % is a runtime concern — at the geometry level NED waypoints
            % don't move.
            clr = palette(mod(j-1, size(palette,1)) + 1, :);
            plot(axPreview, wp(:,2)/1000, wp(:,1)/1000, '-', ...
                'Color', clr, 'LineWidth', 1.8, ...
                'DisplayName', sprintf('%s (+%ds)', e.label, e.start_offset_s));
            plot(axPreview, wp(1,2)/1000, wp(1,1)/1000, 'o', ...
                'Color', clr, 'MarkerSize', 7, 'MarkerFaceColor', clr);
            plot(axPreview, wp(end,2)/1000, wp(end,1)/1000, 's', ...
                'Color', clr, 'MarkerSize', 7, 'MarkerFaceColor', clr);
            sumLines{end+1} = sprintf('%s: %d wp, offset %ds', ...
                e.label, fd.numWaypoints, e.start_offset_s); %#ok<AGROW>
        catch ME
            sumLines{end+1} = sprintf('%s: FAILED (%s)', e.label, ME.message); %#ok<AGROW>
        end
    end

    plot(axPreview, 0, 0, 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'y');
    hold(axPreview, 'off');
    grid(axPreview, 'on');
    axis(axPreview, 'equal');
    title(axPreview, sprintf('NED preview — ref (%.3f°, %.3f°)', refLat, refLon));
    xlabel(axPreview, 'East (km)');
    ylabel(axPreview, 'North (km)');
    legend(axPreview, 'Location', 'best');

    lblPreviewSummary.Text = strjoin(sumLines, newline);
end
```

### 5f.4 verification

1. Click Rescan → empty Batch Builder table, Preview NED button disabled.
2. Check a flight in the Use column → batch table populates with 1 row, Preview NED enables, Auto Lat/Lon updates to that flight's midpoint area.
3. Edit "RCS" → 5 in the batch row → field accepts, value persists.
4. Edit "Offset" → 30 → persists.
5. Uncheck the flight → batch row disappears, Preview NED disables.
6. Check 3 flights → 3 rows, labels auto-renumbered Target_1 / Target_2 / Target_3, edits preserved when reordering happens.
7. Toggle Auto ref off → Lat/Lon fields enable, can type values. Toggle on → fields disable + auto-fill again.
8. Click Preview NED → center pane plots 3 NED trajectories from the chosen ref origin, legend labels include offsets, radar at origin marked. Summary text below lists per-flight waypoint counts.

---

## Architectural constraints

1. **No simulation.** 5f.3 and 5f.4 must NOT instantiate `trackingScenario`, NOT call `emit`/`propagate`/`detect`, NOT use any tracker. The whole point of FDM is to compose data BEFORE handing to Path Editor → Run Simulation.

2. **No new files outside `scripts/flightDataManagerGUI.m`.** Both substeps should be GUI-only changes. The `scanFlightFolder` helper is already there from 5f.2; the `loadNASAFlight` helper is already there from project history.

3. **Preserve the existing 5f.2 functionality.** Single-flight preview (clicking a row in the left table → lat/lon plot in center) must keep working. NED preview REPLACES the center pane content when triggered, but clicking a row in the left table goes back to single-flight preview.

4. **Line endings.** `Filesystem:edit_file` on Windows converts CRLF → LF on touched files. `flightDataManagerGUI.m` is already LF (created via `write_file` in 5f.1). If you use `Filesystem:edit_file`, the file stays LF — no new churn. Don't restore CRLF; the project tolerates LF and the CRLF-converted-files list in `CHECKPOINT.md` already covers this case.

5. **Anchor edits on unique single-line statements.** `flightDataManagerGUI.m` has comment headers with Unicode box-drawing characters (`╓`, `├`, etc.) that have caused anchor mismatches in the past. Anchor on real code lines (variable assignments, function declarations), not header comments.

6. **MATLAB R2025b uitable quirks:**
   - When a column is `'logical'` with `ColumnEditable=true`, clicking the cell toggles the checkbox AND fires both `CellSelectionCallback` and `CellEditCallback`. Selection fires first.
   - `CellEditCallback`'s `event.NewData` may come back as logical or as 0/1 depending on the platform. The code in `refreshBatchFromChecks` handles both.
   - After setting `tblBatch.Data = {}`, the column headers persist (good). Don't try to clear them.

7. **`geoTrajectory` requires Sensor Fusion and Tracking Toolbox.** It's installed (per the project's toolbox list in `CHECKPOINT.md`). The function takes `[lat, lon, alt_m]` matrices and time-of-arrival vectors.

8. **`trackingGlobeViewer` opens its own window.** Don't try to embed it inside the FDM uifigure — it manages its own figure handle.

---

## Hand back to Michael

When done, post to chat:

1. **Files modified:** list (should be just `scripts/flightDataManagerGUI.m`)
2. **Line count change:** before/after
3. **Self-verification result:** which of the 5f.3 and 5f.4 verification steps passed
4. **Known issues / deferred:** anything you couldn't make work, things you decided to do differently
5. **Architecture notes:** any decisions you made that weren't in this doc (e.g. different layout proportions, additional buttons)
6. **Line-ending status:** confirm `flightDataManagerGUI.m` stayed LF

Michael will verify in MATLAB end-to-end. If verification passes, this doc can be deleted. If it doesn't, the next session iterates from here.

Once both verifications pass, control returns to Claude (Sonnet) chat for **5f.5** (schema update + Path Editor export) and **5f.6** (docs polish).
