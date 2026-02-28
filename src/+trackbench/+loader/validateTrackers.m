function validateTrackers(trackers)
%validateTrackers Validate trackers section.

if ~isstruct(trackers)
    error('validateTrackers:type', 'trackers must be a struct.');
end

req = {'global', 'params', 'run'};
for i = 1:numel(req)
    if ~isfield(trackers, req{i})
        error('validateTrackers:missingField', 'trackers.%s is required.', req{i});
    end
end

if ~isfield(trackers.params, 'ideal') || ~isfield(trackers.params, 'degraded')
    error('validateTrackers:missingParams', 'trackers.params must include ideal and degraded.');
end
end
