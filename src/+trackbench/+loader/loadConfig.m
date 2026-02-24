function config = loadConfig(configName, varargin)
% LOADCONFIG  Backward-compatible wrapper.
%   Main branch expects trackbench.loader.loadConfig().
%   Real implementation lives in trackbench.config.loadConfig().
%
% See also: trackbench.config.loadConfig

    config = trackbench.config.loadConfig(configName, varargin{:});
end
