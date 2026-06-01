function key = selectDomain()
%selectDomain  Modal "choose sensor domain" picker for the Path Editor.
%
%   key = trackbench.editor.selectDomain()
%
%   Author:  Team Adeptus - UW Senior Capstone, Boeing-sponsored
%   Since:   v3.7.x (modality-domain Phase 1; design/ADR-001-modality-domains.md)
%
%   Returns "radar" | "sonar" | "ir", or "" if the dialog is closed without
%   a choice. mainMenu's launchPathEditor calls this before opening the
%   editor, then hands the key to pathEditor(projectRoot, key).

    key = "";

    fig = uifigure('Name', 'New Scenario - Choose Sensor Domain', ...
        'Position', [100 100 560 300], 'Color', [0.10 0.12 0.16], ...
        'WindowStyle', 'modal');

    gl = uigridlayout(fig, [3 3], ...
        'RowHeight', {44, '1x', 34}, 'ColumnWidth', {'1x','1x','1x'}, ...
        'Padding', [18 18 18 18], 'RowSpacing', 14, 'ColumnSpacing', 14, ...
        'BackgroundColor', [0.10 0.12 0.16]);

    ttl = uilabel(gl, 'Text', 'Choose the sensor domain for this scenario', ...
        'FontSize', 16, 'FontWeight', 'bold', 'FontColor', 'w', ...
        'HorizontalAlignment', 'center');
    ttl.Layout.Row = 1; ttl.Layout.Column = [1 3];

    doms   = {'radar','sonar','ir'};
    colors = {[0.20 0.30 0.45], [0.10 0.34 0.50], [0.34 0.22 0.40]};
    for i = 1:3
        d = trackbench.editor.sensorDomain(doms{i});
        b = uibutton(gl, 'Text', d.label, 'FontSize', 15, 'FontWeight', 'bold', ...
            'FontColor', 'w', 'BackgroundColor', colors{i}, ...
            'ButtonPushedFcn', @(~,~) onPick(doms{i}));
        b.Layout.Row = 2; b.Layout.Column = i;
    end

    hint = uilabel(gl, 'Text', ...
        'Radar = ATC / air (altitude)   |   Sonar = underwater (depth)   |   IR = passive angle-only', ...
        'FontColor', [0.70 0.72 0.78], 'HorizontalAlignment', 'center');
    hint.Layout.Row = 3; hint.Layout.Column = [1 3];

    uiwait(fig);

    function onPick(k)
        key = string(k);
        uiresume(fig);
        if isvalid(fig); delete(fig); end
    end
end
