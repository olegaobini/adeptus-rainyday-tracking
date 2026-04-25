function launch_trackbench()
% LAUNCH_MAIN_GUI is the main entry point for the compiled application.
% It ensures paths are set correctly and launches the primary GUI.

% The 'isdeployed' function returns true only when running as a compiled app.
if ~isdeployed
    % When running in the MATLAB IDE, add project paths. When deployed,
    % the compiler handles this automatically.
    disp('Running in development mode. Setting up project paths...');
    
    % Get the project root directory relative to this script's location
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    
    addpath(fullfile(projectRoot, 'scripts'));
    addpath(genpath(fullfile(projectRoot, 'src')));
end

% Launch the main GUI, wrapped in a try-catch block to show errors.
try
    fprintf('Launching trackbench...\n');
    runScenarioGUI;
catch ME
    % In a deployed app, errors are silent. We must show them in a dialog.
    errorMessage = sprintf('A critical error occurred:\n\n%s', getReport(ME, 'basic'));
    errordlg(errorMessage, 'Rainy Day Tracker Error');
    
    % Also print to the console, which can be helpful for debugging.
    fprintf(2, '%s\n', errorMessage);
end

end
