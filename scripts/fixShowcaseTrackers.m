function fixShowcaseTrackers()
%fixShowcaseTrackers  Fix nested tracker arrays in showcase run files.
%  Fixes: "trackers": [["a","b"]] → "trackers": ["a","b"]

root = fileparts(fileparts(mfilename('fullpath')));
showcaseDir = fullfile(root, 'config', 'runs', 'showcase');

files = dir(fullfile(showcaseDir, '*.json'));
fixed = 0;

for i = 1:numel(files)
    fpath = fullfile(files(i).folder, files(i).name);
    txt = fileread(fpath);
    
    % The pretty-printed nested array looks like:
    %   "trackers": [
    %     [
    %       "GNN/default_GNN",
    %       "JPDA/default_JPDA"
    %     ]
    %   ],
    %
    % We need to remove the inner [ ] pair, producing:
    %   "trackers": [
    %     "GNN/default_GNN",
    %     "JPDA/default_JPDA"
    %   ],
    
    % Match the pattern with flexible whitespace
    pattern = '("trackers"\s*:\s*\[)\s*\[([^\]]*)\]\s*(\])';
    newTxt = regexprep(txt, pattern, '$1$2$3');
    
    if ~strcmp(txt, newTxt)
        fid = fopen(fpath, 'w', 'n', 'UTF-8');
        fprintf(fid, '%s', newTxt);
        fclose(fid);
        fprintf('Fixed: %s\n', files(i).name);
        fixed = fixed + 1;
    end
end

fprintf('\nFixed %d / %d files.\n', fixed, numel(files));
end