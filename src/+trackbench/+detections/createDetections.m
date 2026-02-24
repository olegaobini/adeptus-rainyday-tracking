function dataLog = createDetections(scenario, enableDegradation)
% CREATEDETECTIONS  Backward-compatible wrapper.
%   Main branch expects trackbench.detections.createDetections(scenario, flag).
%   Real implementation lives in trackbench.detections.runDetections()
%   which now supports multi-sensor and environment configs.
%
% See also: trackbench.detections.runDetections

    if nargin < 2
        enableDegradation = false;
    end

    % Call the expanded version with default (empty) sensor/env args
    dataLog = trackbench.detections.runDetections(scenario, enableDegradation, [], struct());
end
