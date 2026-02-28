function validateTruth(truth)
%validateTruth Validate truth section.

if ~isstruct(truth)
    error('validateTruth:type', 'truth must be a struct.');
end

if ~isfield(truth, 'targets')
    error('validateTruth:missingTargets', 'truth.targets is required.');
end

targets = truth.targets;
if ~(iscell(targets) || isstruct(targets))
    error('validateTruth:typeTargets', 'truth.targets must be a cell array or struct array.');
end
end
