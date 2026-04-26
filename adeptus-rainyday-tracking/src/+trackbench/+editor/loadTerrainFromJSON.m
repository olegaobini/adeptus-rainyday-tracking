function tr = loadTerrainFromJSON(projectRoot, relPath)
%loadTerrainFromJSON  Parse a terrain JSON file into a TerrainRecord
%                     (M7 §3.4).
%
%   Author:  Michael Harding (Team Adeptus)
%   Project: Rainy Day Tracker — UW Senior Capstone, Boeing-sponsored
%
%  Pure function — no state writes. EditorState.loadTerrainFromFile
%  calls this and assigns the returned record into state.terrain.
%
%  INPUTS
%    projectRoot : sandbox root directory. Used to resolve relative
%                  paths like "mountain/my_terrain" under
%                  <projectRoot>/config/terrain/<rel>.json.
%    relPath     : absolute path, a "TYPE/stem" reference, or a bare
%                  stem. Missing ".json" extension is appended.
%
%  OUTPUT
%    tr : trackbench.editor.TerrainRecord
%
%  SUPPORTED / UNKNOWN
%    Built-in types resolve field-for-field from the on-disk schema:
%      description       ← description
%      terrainType       ← terrain_type    (lowercased)
%      terrainScale      ← terrain_scale
%      clutterDensity    ← clutter_density
%      refractionFactor  ← refraction_factor
%    Any terrain_type NOT in {water, rural, urban, mountain, desert}
%    loads as UNKNOWN passthrough: readOnly=true, the TerrainRecord
%    type field is left at the verbatim decoded type (string), and
%    originalDef carries the full parsed struct so saveScenarioToJSON
%    can round-trip the file without losing fields the editor doesn't
%    understand.
%
%  ERROR CONVENTIONS
%    Throws trackbench:editor:loadTerrainFromJSON:* on missing file or
%    malformed shape. Callers (EditorState.loadTerrainFromFile) surface
%    the message via uialert.
%
%  See also: trackbench.editor.TerrainRecord,
%            trackbench.editor.terrainDefaults,
%            trackbench.editor.EditorState.loadTerrainFromFile,
%            trackbench.editor.loadWeatherFromJSON

    arguments
        projectRoot (1,1) string
        relPath     (1,1) string
    end

    full = resolveTerrainPath(projectRoot, relPath);
    if ~isfile(full)
        error('trackbench:editor:loadTerrainFromJSON:notFound', ...
            'Terrain file not found: %s', full);
    end
    raw = fileread(full);
    try
        def = jsondecode(raw);
    catch ME
        error('trackbench:editor:loadTerrainFromJSON:badJSON', ...
            'Could not parse %s: %s', full, ME.message);
    end
    if ~isstruct(def)
        error('trackbench:editor:loadTerrainFromJSON:badShape', ...
            'Terrain file root must be a JSON object: %s', full);
    end

    tr = trackbench.editor.TerrainRecord();
    tr.sourceFile  = string(full);
    tr.originalDef = def;

    % Type first — it drives the supported / UNKNOWN branch.
    rawType = "";
    if isfield(def, 'terrain_type') && ~isempty(def.terrain_type)
        rawType = lower(string(def.terrain_type));
    end
    supported = ["water", "rural", "urban", "mountain", "desert"];
    isSupported = any(rawType == supported);

    if isSupported
        tr.terrainType = rawType;
        tr.readOnly    = false;
        tr.description      = pickString(def, 'description', tr.description);
        tr.terrainScale     = pickNumber(def, 'terrain_scale',     tr.terrainScale);
        tr.clutterDensity   = pickNumber(def, 'clutter_density',   tr.clutterDensity);
        tr.refractionFactor = pickNumber(def, 'refraction_factor', tr.refractionFactor);
    else
        % UNKNOWN passthrough — keep the verbatim type string so
        % saveScenarioToJSON can round-trip it. readOnly gates the UI.
        if rawType == ""
            tr.terrainType = "unknown";
        else
            tr.terrainType = rawType;
        end
        tr.readOnly = true;
        % Populate the scalar fields best-effort so the overlay has
        % something to render; fields are disabled in the UI anyway.
        tr.description      = pickString(def, 'description', "UNKNOWN terrain — see source file");
        tr.terrainScale     = pickNumber(def, 'terrain_scale',     tr.terrainScale);
        tr.clutterDensity   = pickNumber(def, 'clutter_density',   tr.clutterDensity);
        tr.refractionFactor = pickNumber(def, 'refraction_factor', tr.refractionFactor);
    end
end


function full = resolveTerrainPath(projectRoot, relPath)
%resolveTerrainPath  Resolve the user-provided path the same way
%                     loadSensorsFromJSON resolves sensor refs. Order:
%                       1. absolute path → use verbatim
%                       2. relative → projectRoot/<rel>
%                       3. relative → projectRoot/config/terrain/<rel>
%                     Missing ".json" is appended in each candidate.
    p = char(relPath);
    if isAbsolute(p)
        full = p;
        return;
    end
    root = projectRoot;
    if root == ""; root = string(pwd); end
    if ~endsWith(lower(p), ".json")
        pExt = [p '.json'];
    else
        pExt = p;
    end
    candidates = strings(0);
    candidates(end+1) = fullfile(root, pExt);
    candidates(end+1) = fullfile(root, "config", "terrain", pExt);
    candidates(end+1) = string(pExt);
    for i = 1:numel(candidates)
        if isfile(candidates(i))
            full = char(candidates(i));
            return;
        end
    end
    full = char(candidates(1));
end


function yes = isAbsolute(p)
    if ispc
        yes = numel(p) >= 2 && (p(2) == ':' || startsWith(p, '\\'));
    else
        yes = ~isempty(p) && p(1) == '/';
    end
end


function v = pickString(def, field, fallback)
    if isfield(def, field) && ~isempty(def.(field))
        v = string(def.(field));
    else
        v = fallback;
    end
end


function v = pickNumber(def, field, fallback)
    if isfield(def, field) && isnumeric(def.(field)) && ~isempty(def.(field))
        v = double(def.(field));
    else
        v = fallback;
    end
end
