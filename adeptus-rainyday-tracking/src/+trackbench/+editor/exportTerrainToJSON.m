function outPath = exportTerrainToJSON(state, fullPath)
%exportTerrainToJSON  Write state.terrain to a single terrain config
%                     JSON file (v3.5 step 4c).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Pure function — takes a complete output path. The caller (buildUI's
%  onTerrainSave callback) is responsible for showing the uiputfile
%  dialog and resolving the user's chosen path. This separation keeps
%  the file I/O testable without GUI state.
%
%  INPUTS
%    state    : trackbench.editor.EditorState instance
%    fullPath : absolute path of the .json file to write. Missing
%               ".json" extension is appended automatically.
%
%  OUTPUT
%    outPath  : the actual path written (same as fullPath, with
%               extension normalized).
%
%  SCHEMA (matches config/terrain/<TYPE>/default_<TYPE>.json)
%    {
%      "description":       "<text>",
%      "terrain_type":      "rural" | "water" | "urban" | "mountain" | "desert",
%      "terrain_scale":     <double>,
%      "clutter_density":   <double>,
%      "refraction_factor": <double>
%    }
%
%  UNKNOWN PASSTHROUGH
%    When state.terrain.readOnly is true (UNKNOWN passthrough loaded
%    from a file with a terrain_type the editor doesn't natively
%    support), the verbatim originalDef struct is emitted instead.
%    This preserves any extra fields the editor wouldn't otherwise
%    know about, so a load → save round-trip is faithful.
%
%  NOT WRITTEN ELSEWHERE
%    This is the standalone counterpart to exportSensorsToJSON's
%    inner writeTerrainFile helper. The exporter is split out so the
%    Terrain sub-panel's "Save Terrain…" button can write a single
%    file without dragging the whole scenario-bundle pipeline in.
%
%  See also: trackbench.editor.TerrainRecord,
%            trackbench.editor.loadTerrainFromJSON,
%            trackbench.editor.exportSensorsToJSON

    arguments
        state    (1,1) trackbench.editor.EditorState
        fullPath (1,1) string
    end

    if strlength(fullPath) == 0
        error('trackbench:editor:exportTerrainToJSON:emptyPath', ...
            'Output path must be non-empty.');
    end

    % Normalize extension. uiputfile appends the chosen filter's
    % extension, but defensive normalization keeps the function safe
    % when called with hand-built paths.
    fullPath = char(fullPath);
    if ~endsWith(lower(fullPath), '.json')
        fullPath = [fullPath '.json'];
    end

    tr = state.terrain;
    if tr.readOnly && ~isempty(fieldnames(tr.originalDef))
        % UNKNOWN passthrough: round-trip the verbatim parsed struct.
        def = tr.originalDef;
    else
        def = buildTerrainStruct(tr);
    end

    jsonStr = jsonencode(def, 'PrettyPrint', true);

    % Ensure the parent directory exists (uiputfile would normally
    % choose an existing folder, but allow programmatic callers to
    % point at a freshly-named subfolder).
    parent = fileparts(fullPath);
    if ~isempty(parent) && ~exist(parent, 'dir')
        mkdir(parent);
    end

    fid = fopen(fullPath, 'w');
    if fid < 0
        error('trackbench:editor:exportTerrainToJSON:openFailed', ...
            'Could not open %s for writing.', fullPath);
    end
    cleaner = onCleanup(@() fclose(fid));
    fwrite(fid, jsonStr, 'char');

    % Save successful — environment is no longer dirty for the
    % terrain side. Note: weather may still be dirty independently;
    % anyDirty stays at its current value because targets/sensors
    % might still have pending edits. environmentDirty is the
    % conservative flag the onClose prompt actually checks for env
    % changes, so flipping it here is correct.
    %
    % VALUE-CLASS ASSIGNMENT NOTE
    %   We use the explicit read-mutate-writeback pattern (tr = state.terrain;
    %   tr.field = ...; state.terrain = tr;) rather than the chained
    %   form (state.terrain.field = ...). MATLAB usually auto-handles
    %   the chain for handle-class containers, but the explicit form
    %   is what every other mutator in EditorState uses, so we match
    %   for consistency and to avoid edge-case surprises.
    state.environmentDirty = false;
    tr = state.terrain;
    tr.sourceFile = string(fullPath);
    state.terrain = tr;

    outPath = string(fullPath);
end


%% ========================================================================
%  Local helpers
%% ========================================================================
function def = buildTerrainStruct(tr)
%buildTerrainStruct  TerrainRecord → on-disk schema struct.
%
%  Mirrors exportSensorsToJSON's same-named local helper (kept private
%  there to avoid a public dependency). If the schema diverges, update
%  both places — they're intentionally redundant rather than coupled.
    def = struct();
    def.description       = char(tr.description);
    def.terrain_type      = char(tr.terrainType);
    def.terrain_scale     = tr.terrainScale;
    def.clutter_density   = tr.clutterDensity;
    def.refraction_factor = tr.refractionFactor;
end
