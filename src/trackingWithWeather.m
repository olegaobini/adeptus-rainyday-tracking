% -------------------------------------------------------------------------
% Tracking Closely Spaced Targets Under Ambiguity
%
% PURPOSE
%   Run a consistent simulation + tracking pipeline to compare:
%     - Trackers: GNN, TOMHT, JPDA
%     - Motion models: CV vs IMM
%   across two scenarios:
%     - IDEAL  : clean detections, low clutter assumptions
%     - RAINY  : degraded detections, higher ambiguity + clutter assumptions
%
% BASELINE SOURCE
%   Adapted from MathWorks example:
%   https://www.mathworks.com/help/fusion/ug/tracking-closely-spaced-targets-under-ambiguity.html
%
% PROJECT EXTENSIONS 
%   - Scenario toggle (IDEAL vs RAINY) that changes "knobs" consistently
%   - Automated tracker runs for all combinations (tracker + motion model)
%   - Automated JPDA tracker tuning for ideal/nonideal scenarios
%   - Reporting: track-level and truth-level performance metrics + timing
%
% NOTES FOR TEAM
%   - This is the top-level driver script. All "real work" happens in helpers:
%       helperCreateScenario3D   -> scenario definition (truth + radar)
%       helperRunDetections      -> generates detection logs
%       helperRunTracker         -> runs a tracker and produces metrics
%       initCVFilter/initIMMFilter -> defines the filter used per track
%   - If something looks "off", most debugging starts in helperRunDetections
%     (time stamps, measurement noise, clutter injection, etc.).
%
% -------------------------------------------------------------------------

clc; clear; close all;

%% ---------- Scenario Toggle ----------
% enableDegradation:
%   false -> IDEAL scenario (baseline MathWorks-like conditions)
%   true  -> RAINY scenario  (degraded detection conditions)
%
% This single flag should be the ONLY thing you need to flip when running
% "ideal vs rainy day" experiments. All scenario-dependent thresholds are
% derived from it below.
enableDegradation = true;
% ------------------------------------

fprintf("\n==============================\n");
fprintf(" RUN START | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");

%% Create scenario + detections
%------------------- Default Scenario------------------------------
% To demonstrate a case where sensor reports are ambiguously assigned to
% tracks, you create a simple scenario. In this scenario, a single radar
% object, located at the origin (not shown), scans a small region about 20
% km from the radar. Initially, the radar reports about two detections per
% scan. When the detections are coming from a region around the X = 0, Y =
% -20 km position, the radar reports a single detection per scan for a
% while, followed by two radar detections reported from around Y = -19.5km
% and toward the sensor (up).
% ------------------------------------------------------------------
% 
% Two ways to define the scenario:
%   helperCreateScenario      -> original reference scenario
%   helperCreateScenario3D    -> our configurable scenario builder
%
% dataLog is the key output:
%   dataLog.Time        -> time stamps per scan/update
%   dataLog.Truth       -> ground truth platform states
%   dataLog.Detections  -> detections per scan (cell array of objectDetections)
%
% IMPORTANT:
%   helperRunDetections is where "RAINY" degradation is injected:
%     - detection dropouts (effective Pd)
%     - inflated measurement noise
%     - added clutter (false alarms)
%
% scenario = helperCreateScenario;
% dataLog  = helperRunDetections(scenario, enableDegradation);

%scenario = helperCreateScenario3D("NumTargets",4,"SceneDuration",60);
%dataLog = helperRunDetections(scenario, enableDegradation);

%% Create scenario + detections
% ================= DETECTION SOURCE =================
useSavedDataLog = true;                 % true = load, false = generate+save
dataLogFile = "cache/myRun1.mat";
% ====================================================

% ================= SCENARIO SELECT ==================
scenarioMode = "3D";   % "2D" uses helperCreateScenario
                        % "3D" uses helperCreateScenario3D
NumTargets    = 4;     % only used for 3D
SceneDuration = 60;    % only used for 3D
% ====================================================

if useSavedDataLog
    load(dataLogFile,"dataLog");
    fprintf("[replay] Loaded dataLog from %s\n", dataLogFile);

else
    % ---- pick scenario builder ----
    if scenarioMode == "3D"
        scenario = helperCreateScenario3D("NumTargets",NumTargets,"SceneDuration",SceneDuration);
    else
        scenario = helperCreateScenario();
    end

    dataLog = helperRunDetections(scenario, enableDegradation);

    if ~exist("cache","dir"), mkdir("cache"); end
    save(dataLogFile,"dataLog","-v7.3");
    fprintf("[record] Saved dataLog to %s\n", dataLogFile);
end

% Visualization: plot of truth + detections.
plotInitialScenario(dataLog);

%% Quick stats on detection count per scan
% Helps confirm degradation is actually happening:
%   IDEAL -> generally stable count close to number of targets
%   RAINY -> increased variability (clutter + dropouts)
nPerScan = cellfun(@numel, dataLog.Detections);
fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
    min(nPerScan), mean(nPerScan), max(nPerScan));


% Export to generate detection file for previous capstone Kalman Filter
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% 
% 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
if enableDegradation
    exportMateoV5FromDataLog(dataLog, "MateoV5_nonideal.txt", ...
        'Mode', "range", ...
        'PickRule', "closestToPrev", ...
        'WriteMissing', "hold", ...
        'MaxJumpMeters', 200);
else 
    exportMateoV5FromDataLog(dataLog, "MateoV5_ideal.txt", ...
    'Mode', "range", ...
    'PickRule', "closestToPrev", ...
    'WriteMissing', "hold", ...
    'MaxJumpMeters', 200);
end 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 


% exportMateoV5FromDataLog(dataLog, "MateoV5_nonideal.txt", ...
%     'WriteMissing', "hold", ...
%     'MaxJumpMeters', 200);
% 
% exportMateoV5FromDataLog(dataLog, "MateoV5_ideal.txt", ...
%     'WriteMissing', "hold", ...
%     'MaxJumpMeters', 200);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% Global tracker knobs (shared baseline assumptions)
% These parameters influence track initiation/maintenance and association.
%
% MaxNumTracks:
%   Upper bound on how many tracks can exist at once (prevents blow-up)
%
% Volume:
%   Assumed surveillance volume (m^3). Used with FAR and Beta.
%   Bigger volume + same FAR implies fewer false alarms per unit volume.
%
% Beta:
%   New track "birth" intensity / expected new target rate (model-dependent).
%   Too low -> tracker refuses to initiate tracks.
%   Too high -> tracker spawns too many tracks (especially under clutter).
%
% pd:
%   DetectionProbability - assumed probability a target is detected each scan.
%   If this is far from reality, track scoring can behave poorly.
numTracks = 500;
vol  = 1e9;
beta = 1e-14;
if (enableDegradation) pd = 0.7;
else pd = 0.9; end

%% Scenario-dependent knobs (gating + clutter assumptions)
% gate:
%   Association threshold. Larger = more permissive; smaller = stricter.
%   Under heavy clutter, a too-large gate increases wrong associations.
%   Under low SNR / noisy measurement, too-small gate loses the target.
%
% farX:
%   FalseAlarmRate assumptions for each tracker type.
%   These are not "truth"; they are the tracker's internal clutter model.
%   If FAR is set too low in a cluttered environment, some trackers can
%   over-trust measurements and diverge or fail initiation logic.
if enableDegradation
    gate = 35;

    % Under degradation, we assume more false alarms than ideal.
    % (We keep these relatively moderate so association isn't totally starved.)
    farGNN  = 1e-2;
    farMHT  = 1e-3;
    farJPDA = 1e-3;
else
    gate = 45;

    % IDEAL = essentially "near-zero clutter assumption"
    farGNN  = 1e-6;
    farMHT  = 1e-6;
    farJPDA = 1e-6;
end

fprintf("Scenario: %s | gate=%.2f | farGNN=%.3g | farMHT=%.3g | farJPDA=%.3g | pd=%.2f | vol=%.3g | beta=%.3g\n\n", ...
    ternary(enableDegradation,"RAINY","IDEAL"), gate, farGNN, farMHT, farJPDA, pd, vol, beta);

%% TOMHT-only thresholds (score-based track management)
% TOMHT uses score logic internally to decide when a hypothesis is "real".
% ConfirmationThreshold:
%   score level needed before a track is considered confirmed
% DeletionThreshold:
%   score level below which tracks are deleted
if enableDegradation
    confirmThresh = 30;
    deleteThresh = -15;
else
    confirmThresh = 20;
    deleteThresh = -5;
end

%% TOMHT scenario-dependent knobs
% tomhtThresh:
%   Three-stage assignment threshold used by TOMHT (varies by dimension/state).
%   We scale relative to gate for simplicity and consistent tuning.
%
% maxBranches:
%   Limits branching factor of hypotheses.
%   Higher = can represent more ambiguity, but costs time and may explode.
if enableDegradation
    tomhtThresh = [0.2, 5, 5] * gate;
    maxBranches = 3;
else
    tomhtThresh = [0.2, 1, 1] * gate;
    maxBranches = 5;
end

%% JPDA scenario-dependent knobs (initiation + anti-spam controls)
% JPDA has different "initiation" behavior than GNN/TOMHT.
% In clutter/degradation, it often fails to start tracks unless
% NewTargetDensity (betaJPDA) is high enough.
%
% NOTE:
%   Raising betaJPDA helps initiation but can create track spam.
%   We control spam primarily by limiting MaxNumTracks for JPDA.
if enableDegradation
    betaJPDA    = 1e-11;        % encourage initiation (tune 1e-12..1e-10)
    gateJPDA    = gate + 10;    % looser gate helps association under clutter
    timeTolJPDA = 0.05;         % detections already snapped to scan times

    numTracksJPDA = 200;        % cap so it doesn't explode
else
    betaJPDA    = beta;
    gateJPDA    = gate;
    timeTolJPDA = 0.05;

    numTracksJPDA = numTracks;
end

%% ============ GNN + CV ============
% GNN: Global Nearest Neighbor (hard assignment)
% CV:  Constant Velocity filter initialization
fprintf("\n============ GNN + CV ============\n");
tracker = trackerGNN( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gate, ...
    'TrackLogic', 'Score', ...
    'DetectionProbability', pd, ...
    'FalseAlarmRate', farGNN, ...
    'Volume', vol, ...
    'Beta', beta);

% helperRunTracker does:
%   - step through dataLog detections
%   - call tracker(detections, time)
%   - compute metrics vs truth
[trackSummary, truthSummary, trackMetrics, truthMetrics, timeGNNCV] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

%% ============ GNN + IMM ============
% IMM: Interacting Multiple Model (handles maneuvering better than CV)
fprintf("\n============ GNN + IMM ============\n");
tracker = trackerGNN( ...
    'FilterInitializationFcn', @initIMMFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gate, ...
    'TrackLogic', 'Score', ...
    'DetectionProbability', pd, ...
    'FalseAlarmRate', farGNN, ...
    'Volume', vol, ...
    'Beta', beta);

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeGNNIMM] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

%% ============ TOMHT + CV ============
% TOMHT: Track-Oriented MHT (multiple hypotheses; good under ambiguity)
fprintf("\n============ TOMHT + CV ============\n");
tracker = trackerTOMHT( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', tomhtThresh, ...
    'DetectionProbability', pd, ...
    'FalseAlarmRate', farMHT, ...
    'Volume', vol, ...
    'Beta', beta, ...
    'ConfirmationThreshold', confirmThresh, ...
    'DeletionThreshold', deleteThresh, ...
    'MaxNumHistoryScans', 10, ...
    'MaxNumTrackBranches', maxBranches, ...
    'NScanPruning', 'Hypothesis', ...
    'OutputRepresentation', 'Tracks');

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeTOMHTCV] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

%% ============ TOMHT + IMM ============
fprintf("\n============ TOMHT + IMM ============\n");
tracker = trackerTOMHT( ...
    'FilterInitializationFcn', @initIMMFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', tomhtThresh, ...
    'DetectionProbability', pd, ...
    'FalseAlarmRate', farMHT, ...
    'Volume', vol, ...
    'Beta', beta, ...
    'ConfirmationThreshold', confirmThresh, ...
    'DeletionThreshold', deleteThresh, ...
    'MaxNumHistoryScans', 10, ...
    'MaxNumTrackBranches', maxBranches, ...
    'NScanPruning', 'Hypothesis', ...
    'OutputRepresentation', 'Tracks');

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeTOMHTIMM] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

%% ============ JPDA + CV ============
% JPDA: Joint Probabilistic Data Association (soft association)
% Integrated logic means it internally manages existence based on association probs.
fprintf("\n============JPDA + CV==================\n");
tracker = trackerJPDA( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracksJPDA, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gateJPDA, ...
    'TrackLogic', 'Integrated', ...
    'DetectionProbability', pd, ...
    'ClutterDensity', farJPDA/vol, ...
    'NewTargetDensity', betaJPDA, ...
    'TimeTolerance', timeTolJPDA);

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeJPDACV] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

%% ============ JPDA + IMM ============
fprintf("\n============JPDA + IMM==================\n");
tracker = trackerJPDA( ...
    'FilterInitializationFcn', @initIMMFilter, ...
    'MaxNumTracks', numTracksJPDA, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gateJPDA, ...
    'TrackLogic', 'Integrated', ...
    'DetectionProbability', pd, ...
    'ClutterDensity', farJPDA/vol, ...
    'NewTargetDensity', betaJPDA, ...
    'TimeTolerance', timeTolJPDA);

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeJPDAIMM] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

fprintf("\n==============================\n");
fprintf(" RUN END | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");

%% -------- Local helper: ternary --------
% Small utility so we can write:
%   ternary(cond, "A", "B")
% instead of MATLAB's  if/else just for printing.
function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end
