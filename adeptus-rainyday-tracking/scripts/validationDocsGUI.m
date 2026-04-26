function varargout = validationDocsGUI(projectRoot)
%validationDocsGUI  Level 3 of mainMenu — validation suite + documentation hub.
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Three panels in one window:
%    1. Validation Test Plan — runs runTestPlan, shows inline pass/fail
%       table grouped by test case. Console output is captured and
%       displayed in a scrollable text area below the table.
%    2. Diagnostic Suite — runs verifySimulation, shows captured console
%       output in a text area (this script is silent / console-only by
%       design, no structured results).
%    3. Documentation — buttons that open project docs and folders in
%       the OS default app via winopen / open.
%
%  USAGE
%      validationDocsGUI;                  % standalone
%      validationDocsGUI(projectRoot);     % caller (mainMenu) supplies root
%
%  HOW THE INLINE TEST RESULTS WORK
%    runTestPlan logs each individual assertion via logResult, building a
%    .details cell array of structs {tc, check, passed, detail}. We
%    capture stdout via evalc, then re-load the .mat file the script
%    writes to results/ (test_plan_results_<timestamp>.mat) to get the
%    structured details. Each check becomes one row in the uitable, with
%    a green PASS or red FAIL badge.
%
%    The .mat file is the source of truth for the table because the
%    console output is human-formatted; parsing it would be brittle.
%    runTestPlan's own save call gives us a clean structured dump.
%
%  See also: runTestPlan, verifySimulation, mainMenu

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
    existing = findall(groot, 'Type', 'figure', ...
                       'Name', 'Rainy Day — Validation & Documentation');
    if ~isempty(existing)
        existing = existing(arrayfun(@isvalid, existing));
    end
    if ~isempty(existing)
        figure(existing(1));
        if nargout > 0, varargout{1} = existing(1); end
        return;
    end

    % ── Build figure ────────────────────────────────────────────────
    fig = uifigure('Name', 'Rainy Day — Validation & Documentation', ...
                   'Position', [240 100 900 680]);

    gl = uigridlayout(fig, [1 1]);
    gl.Padding = [10 10 10 10];

    tg = uitabgroup(gl);
    tg.Layout.Row = 1; tg.Layout.Column = 1;

    buildTestPlanTab(tg, projectRoot, fig);
    buildDiagnosticsTab(tg, projectRoot, fig);
    buildDocsTab(tg, projectRoot, fig);

    if nargout > 0, varargout{1} = fig; end
end


%% ========================================================================
%  Tab 1: Validation Test Plan
%% ========================================================================
function buildTestPlanTab(tg, projectRoot, ~)
    tab = uitab(tg, 'Title', 'Validation Test Plan');

    gl = uigridlayout(tab, [4 1]);
    gl.RowHeight   = {44, 32, 240, '1x'};
    gl.ColumnWidth = {'1x'};
    gl.RowSpacing  = 8;
    gl.Padding     = [12 12 12 12];

    % Row 1: Run button + status label
    glRow1 = uigridlayout(gl, [1 2]);
    glRow1.Layout.Row    = 1;
    glRow1.ColumnWidth   = {200, '1x'};
    glRow1.ColumnSpacing = 12;
    glRow1.Padding       = [0 0 0 0];

    btnRun = uibutton(glRow1, 'Text', 'Run Validation Tests', ...
        'FontWeight', 'bold', 'FontSize', 13, ...
        'ButtonPushedFcn', @(~,~) runTests()); %#ok<NASGU>
    lblStatus = uilabel(glRow1, ...
        'Text', 'Click "Run Validation Tests" to execute the test plan (~30–60 seconds).', ...
        'FontColor', [0.4 0.4 0.4], ...
        'VerticalAlignment', 'center');

    % Row 2: Summary line (PASS / FAIL counts)
    lblSummary = uilabel(gl, 'Text', '', ...
        'FontWeight', 'bold', 'FontSize', 14, ...
        'HorizontalAlignment', 'center');
    lblSummary.Layout.Row = 2;

    % Row 3: Inline pass/fail table
    %   Columns: TC | Result | Check | Detail
    %   One row per assertion (runTestPlan logs ~27 across 9 cases).
    %   The 'Result' column is colored via uistyle/addStyle below.
    tbl = uitable(gl, ...
        'ColumnName', {'TC', 'Result', 'Check', 'Detail'}, ...
        'ColumnEditable', [false false false false], ...
        'ColumnWidth', {60, 80, 380, 'auto'}, ...
        'Data', cell(0, 4), ...
        'RowStriping', 'on');
    tbl.Layout.Row = 3;

    % Row 4: Captured console output
    txtConsole = uitextarea(gl, 'Editable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 11, ...
        'Value', {'(console output will appear here after running)'});
    txtConsole.Layout.Row = 4;


    function runTests()
        % Run runTestPlan with output captured via evalc, then reload
        % the .mat file it saves to populate the structured table.
        lblStatus.Text = 'Running tests... (this can take 30–60 seconds, the GUI will be unresponsive)';
        lblStatus.FontColor = [0.7 0.4 0]; drawnow;

        % Snapshot which result files exist before so we can identify
        % the freshly-written one. runTestPlan names with a timestamp,
        % so the newest .mat is what we want — but using a snapshot
        % avoids accidentally picking up an older file if the timestamp
        % collides (rare, but safer).
        resultsDir = fullfile(projectRoot, 'results');
        beforeFiles = dir(fullfile(resultsDir, 'test_plan_results_*.mat'));

        runError = [];
        try
            consoleOutput = evalc('runTestPlan');
        catch ME
            runError = ME;
            consoleOutput = sprintf('ERROR during runTestPlan:\n%s\n%s', ...
                ME.message, getReport(ME, 'extended'));
        end

        % Show captured console output verbatim (split on newlines for
        % uitextarea, which expects a cell array of lines).
        txtConsole.Value = strsplit(consoleOutput, newline);

        % Find the newly-written results file.
        afterFiles = dir(fullfile(resultsDir, 'test_plan_results_*.mat'));
        newFile = '';
        if numel(afterFiles) > numel(beforeFiles)
            % At least one new file. Pick the newest by datenum.
            [~, idx] = max([afterFiles.datenum]);
            newFile = fullfile(afterFiles(idx).folder, afterFiles(idx).name);
        end

        if isempty(newFile)
            lblSummary.Text = 'Tests did not produce a result file';
            lblSummary.FontColor = [0.7 0 0];
            tbl.Data = cell(0, 4);
            if ~isempty(runError)
                lblStatus.Text = sprintf('Run failed: %s', runError.message);
            else
                lblStatus.Text = 'Run completed but no .mat file was written';
            end
            lblStatus.FontColor = [0.7 0 0];
            return;
        end

        try
            S = load(newFile, 'testResults');
            R = S.testResults;
        catch ME
            lblSummary.Text = sprintf('Could not load results: %s', ME.message);
            lblSummary.FontColor = [0.7 0 0];
            return;
        end

        % Populate table from R.details
        rows = cell(numel(R.details), 4);
        for i = 1:numel(R.details)
            d = R.details{i};
            if d.passed, status = 'PASS'; else, status = 'FAIL'; end
            detailStr = char(d.detail);
            rows{i, 1} = char(d.tc);
            rows{i, 2} = status;
            rows{i, 3} = char(d.check);
            rows{i, 4} = detailStr;
        end
        tbl.Data = rows;

        % Color the Result column: green for PASS rows, red for FAIL.
        % uistyle/addStyle is the supported way to color uitable cells.
        % Clear any prior styling before re-applying so a second test
        % run doesn't accumulate stale styles on now-passing rows.
        removeStyle(tbl);
        passRows = find(strcmp(rows(:, 2), 'PASS'));
        failRows = find(strcmp(rows(:, 2), 'FAIL'));
        if ~isempty(passRows)
            addStyle(tbl, uistyle('BackgroundColor', [0.85 0.95 0.85]), ...
                'cell', [passRows, 2 * ones(numel(passRows), 1)]);
        end
        if ~isempty(failRows)
            addStyle(tbl, uistyle('BackgroundColor', [0.98 0.82 0.82], ...
                'FontWeight', 'bold'), ...
                'cell', [failRows, 2 * ones(numel(failRows), 1)]);
            % Make whole failing rows visually obvious — light tint
            % across all 4 columns so they stand out when scanning.
            for col = [1 3 4]
                addStyle(tbl, uistyle('BackgroundColor', [1 0.94 0.94]), ...
                    'cell', [failRows, col * ones(numel(failRows), 1)]);
            end
        end

        % Summary line
        total = R.pass + R.fail;
        if R.fail == 0
            lblSummary.Text = sprintf('✅ ALL %d CHECKS PASSED', total);
            lblSummary.FontColor = [0 0.5 0];
        else
            lblSummary.Text = sprintf('%d/%d passed — %d FAILED', ...
                R.pass, total, R.fail);
            lblSummary.FontColor = [0.8 0 0];
        end

        lblStatus.Text = sprintf('Results saved: %s', newFile);
        lblStatus.FontColor = [0.4 0.4 0.4];
    end
end


%% ========================================================================
%  Tab 2: Diagnostic Suite (verifySimulation)
%% ========================================================================
function buildDiagnosticsTab(tg, ~, ~)
    tab = uitab(tg, 'Title', 'Diagnostic Suite');

    gl = uigridlayout(tab, [3 1]);
    gl.RowHeight   = {44, 32, '1x'};
    gl.ColumnWidth = {'1x'};
    gl.RowSpacing  = 8;
    gl.Padding     = [12 12 12 12];

    % Row 1: Run button + status
    glRow1 = uigridlayout(gl, [1 2]);
    glRow1.Layout.Row    = 1;
    glRow1.ColumnWidth   = {200, '1x'};
    glRow1.ColumnSpacing = 12;
    glRow1.Padding       = [0 0 0 0];

    btnRun = uibutton(glRow1, 'Text', 'Run Diagnostic Suite', ...
        'FontWeight', 'bold', 'FontSize', 13, ...
        'ButtonPushedFcn', @(~,~) runDiag()); %#ok<NASGU>
    lblStatus = uilabel(glRow1, ...
        'Text', 'Runs verifySimulation — 40+ pipeline checks across 8 phases.', ...
        'FontColor', [0.4 0.4 0.4], ...
        'VerticalAlignment', 'center');

    lblHint = uilabel(gl, ...
        'Text', ['Diagnostic output is console-only (no structured ' ...
                 'pass/fail file). Scan for ✗ FAIL or ⚠ WARN lines.'], ...
        'FontColor', [0.4 0.4 0.4]);
    lblHint.Layout.Row = 2;

    txtConsole = uitextarea(gl, 'Editable', 'off', ...
        'FontName', 'Consolas', 'FontSize', 11, ...
        'Value', {'(console output will appear here after running)'});
    txtConsole.Layout.Row = 3;

    function runDiag()
        lblStatus.Text = 'Running diagnostics... (typically 5–15 seconds)';
        lblStatus.FontColor = [0.7 0.4 0]; drawnow;

        try
            output = evalc('verifySimulation');
        catch ME
            output = sprintf('ERROR during verifySimulation:\n%s\n%s', ...
                ME.message, getReport(ME, 'extended'));
            lblStatus.Text = sprintf('Run failed: %s', ME.message);
            lblStatus.FontColor = [0.8 0 0];
            txtConsole.Value = strsplit(output, newline);
            return;
        end

        txtConsole.Value = strsplit(output, newline);

        % Quick-scan the captured output for fail/warn counts so the
        % user gets a status line without having to read all of it.
        % verifySimulation prints "✓ PASS", "✗ FAIL", "⚠ WARN" tokens — NOT
        % bracketed [PASS]/[FAIL]/[WARN]. We anchor the regex to those
        % literal Unicode markers (escaped). The 'all' flag captures every
        % occurrence rather than just the first.
        nFail = numel(regexp(output, '✗\s*FAIL', 'match'));
        nWarn = numel(regexp(output, '⚠\s*WARN', 'match'));
        nPass = numel(regexp(output, '✓\s*PASS', 'match'));

        if nFail == 0 && nWarn == 0
            lblStatus.Text = sprintf('✅ %d checks passed (0 failed, 0 warnings)', nPass);
            lblStatus.FontColor = [0 0.5 0];
        elseif nFail == 0
            lblStatus.Text = sprintf('%d passed, %d warnings', nPass, nWarn);
            lblStatus.FontColor = [0.7 0.4 0];
        else
            lblStatus.Text = sprintf('%d passed, %d FAILED, %d warnings', ...
                nPass, nFail, nWarn);
            lblStatus.FontColor = [0.8 0 0];
        end
    end
end


%% ========================================================================
%  Tab 3: Documentation
%% ========================================================================
function buildDocsTab(tg, projectRoot, parentFig)
    tab = uitab(tg, 'Title', 'Documentation');

    % Layout: 2 doc buttons + spacer + 3 folder buttons + spacer + 2 link buttons
    %  rows:  1=hdr, 2=PDF, 3=README, 4=spacer,
    %         5=hdr2, 6=logs, 7=results, 8=cache, 9=spacer, 10=hdr3, 11=links
    gl = uigridlayout(tab, [11 1]);
    gl.RowHeight = {30, 36, 36, 18, 30, 36, 36, 36, 18, 30, 36};
    gl.ColumnWidth = {'1x'};
    gl.RowSpacing = 6;
    gl.Padding = [16 16 16 16];

    % ─── Section 1: Project documentation files ───
    %  IMPORTANT: each button is created on its own line, with the handle
    %  saved to a variable, THEN .Layout.Row set on the next line. The
    %  chained "uibutton(...).Layout.Row = N" syntax does NOT work —
    %  MATLAB's parser misreads it and throws "Unable to use a value of
    %  type matlab.ui.container.GridLayout as an index" during construction.
    %  The %#ok<NASGU> suppresses "variable might be unused" lint since we
    %  don't reference these handles after creation.

    lbl1 = uilabel(gl, 'Text', 'Project Documentation', ...
        'FontWeight', 'bold', 'FontSize', 14);
    lbl1.Layout.Row = 1;

    % Rainy Day overview PDF — the project pitch / rundown document.
    btnPDF = uibutton(gl, 'Text', 'Open Rainy Day Overview (.pdf)', ...
        'ButtonPushedFcn', @(~,~) openDoc(parentFig, ...
            findFirstExisting({ ...
                fullfile(projectRoot, 'docs', 'Rainy Day.pdf'), ...
                fullfile(projectRoot, 'docs', 'RainyDay.pdf')})));
    btnPDF.Layout.Row = 2;

    % README — may live inside the project or one level up.
    % Boeing Handout and Glossary buttons were removed in step 3b: their
    % content is fully covered by the Rainy Day Overview PDF above, so
    % three near-identical doc buttons added clutter without value.
    btnREADME = uibutton(gl, 'Text', 'Open README.md', ...
        'ButtonPushedFcn', @(~,~) openDoc(parentFig, ...
            findFirstExisting({ ...
                fullfile(projectRoot, 'README.md'), ...
                fullfile(fileparts(projectRoot), 'README.md')})));
    btnREADME.Layout.Row = 3;

    % Spacer row 4

    % ─── Section 2: Project folders (Explorer) ───
    lbl2 = uilabel(gl, 'Text', 'Project Folders', ...
        'FontWeight', 'bold', 'FontSize', 14);
    lbl2.Layout.Row = 5;

    btnLogs = uibutton(gl, 'Text', 'Open logs/ folder', ...
        'ButtonPushedFcn', @(~,~) openDoc(parentFig, ...
            ensureExists(fullfile(projectRoot, 'logs'), parentFig)));
    btnLogs.Layout.Row = 6;

    btnResults = uibutton(gl, 'Text', 'Open results/ folder', ...
        'ButtonPushedFcn', @(~,~) openDoc(parentFig, ...
            ensureExists(fullfile(projectRoot, 'results'), parentFig)));
    btnResults.Layout.Row = 7;

    btnCache = uibutton(gl, 'Text', 'Open cache/ folder', ...
        'ButtonPushedFcn', @(~,~) openDoc(parentFig, ...
            ensureExists(fullfile(projectRoot, 'cache'), parentFig)));
    btnCache.Layout.Row = 8;

    % Spacer row 9

    % ─── Section 3: External references ───
    lbl3 = uilabel(gl, 'Text', 'External References', ...
        'FontWeight', 'bold', 'FontSize', 14);
    lbl3.Layout.Row = 10;

    glExt = uigridlayout(gl, [1 2]);
    glExt.Layout.Row    = 11;
    glExt.ColumnWidth   = {'1x', '1x'};
    glExt.ColumnSpacing = 8;
    glExt.RowHeight     = {36};
    glExt.Padding       = [0 0 0 0];

    btnMatlab = uibutton(glExt, 'Text', 'MATLAB Tracking Examples', ...
        'ButtonPushedFcn', @(~,~) web( ...
            'https://www.mathworks.com/help/fusion/examples.html', ...
            '-browser')); %#ok<NASGU>
    btnNASA = uibutton(glExt, 'Text', 'NASA DASHlink Flight Data', ...
        'ButtonPushedFcn', @(~,~) web( ...
            'https://c3.ndc.nasa.gov/dashlink/resources/664/', ...
            '-browser')); %#ok<NASGU>
end


%% ========================================================================
%  Helpers (module-level)
%% ========================================================================
function path = findFirstExisting(candidates)
    % Return the first path in candidates that exists on disk, else
    % return the first candidate (so the openDoc helper can show a
    % "doesn't exist" message rather than silently doing nothing).
    path = candidates{1};
    for k = 1:numel(candidates)
        if isfile(candidates{k}) || isfolder(candidates{k})
            path = candidates{k};
            return;
        end
    end
end

function path = ensureExists(path, parentFig)
    % For folders we want to create-on-demand: cache/, results/, logs/
    % may not exist yet on a fresh checkout, but they're standard project
    % locations so it's reasonable to mkdir before opening.
    if ~isfolder(path)
        try
            mkdir(path);
        catch ME
            uialert(parentFig, sprintf('Could not create %s:\n%s', ...
                path, ME.message), 'Folder Create Failed');
        end
    end
end

function openDoc(parentFig, path)
    % Open a file or folder in the OS default app (Windows Explorer for
    % folders, Word for .docx, web browser for .md depending on OS
    % association). Surfaces a clean uialert if the path is missing.
    if ~isfile(path) && ~isfolder(path)
        uialert(parentFig, sprintf( ...
            ['Could not find:\n%s\n\nThe file may not be present in this ' ...
             'project copy. Check the docs/ folder of the source project, ' ...
             'or pull the latest version from git.'], path), ...
            'File Not Found', 'Icon', 'warning');
        return;
    end
    try
        if ispc
            winopen(path);
        else
            % macOS/Linux fallback. Less common for Rainy Day users but
            % keeps the GUI cross-platform.
            if ismac
                system(sprintf('open "%s" &', path));
            else
                system(sprintf('xdg-open "%s" &', path));
            end
        end
    catch ME
        uialert(parentFig, sprintf('Could not open:\n%s\n\n%s', ...
            path, ME.message), 'Open Failed');
    end
end
