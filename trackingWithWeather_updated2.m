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
enableDegradation = true;
enableAutoTune    = true;

% --------- Tracker tuning state (modifiable by auto-tune) ---------
tune.gateGNN      = 30;
tune.gateMHT      = 35;
tune.gateJPDA     = 40;

tune.beta         = 1e-13;     % starting point
tune.farScaleGNN  = 10;        % <-- clutter scale knob (GNN only)

autoTuneOnlyGNN   = true;      % keep it focused; you can expand later
% ------------------------------------

fprintf("\n==============================\n");
fprintf(" RUN START | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");

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
nPerScan = cellfun(@numel, dataLog.Detections);
fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
    min(nPerScan), mean(nPerScan), max(nPerScan));

%% =================== Sensor metrics (for reporting + tuning) ===================
sensorM = helperEstimateSensorMetrics(dataLog, 400);

fprintf("Sensor metrics (per scan): Pd_soft=%.3f | Pd_est=%.3f | FA/scan=%.3f | meanDets/scan=%.3f | scanT=%.3fs\n\n", ...
    sensorM.Pd_soft, sensorM.Pd_est, sensorM.FalseAlarmsPerScan, ...
    sensorM.MeanDetectionsPerScan, sensorM.ScanPeriod);

%% Export to generate detection file for previous capstone Kalman Filter
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

%% Global tracker knobs (shared baseline assumptions)
numTracks = 500;

% IMPORTANT: vol should represent the region you *allow* measurements from.
% If you gate to an ROI box, use that box volume. Otherwise use something conservative.
% ROI volume (must match gateDetectionsROI bounds)
xMin = -8000;  xMax =  8000;
yMin = -26000; yMax = -16000;
zMin = -8000;  zMax =   500;
vol  = (xMax-xMin) * (yMax-yMin) * (zMax-zMin);  % m^3

% --- Sensor-based assumed Pd (count-based; can be inflated by clutter) ---
pd = max(0.05, min(0.99, sensorM.Pd_soft));

% In RAINY, clamp Pd to something realistic so score logic doesn't over-trust
% measurements and spawn tracks aggressively.
if enableDegradation
    pd = min(pd, 0.85);
end

% --- Sensor-based clutter density per scan (false alarms / m^3 / scan) ---
% dataLog is per scan (logged at IsScanDone), so do NOT divide by ScanPeriod here.
if vol > 0
    clutterDensity_from_data = sensorM.FalseAlarmsPerScan / vol;  % 1/m^3 per scan
else
    clutterDensity_from_data = 1e-12; % fallback
end

% sane bounds (still <<1)
clutterDensity_from_data = max(1e-15, min(1e-3, clutterDensity_from_data));

%% Scenario-dependent knobs (gating + initiation/clutter assumptions)
% Separate gates so we can be stricter for GNN (reduces false track spawns)
if enableDegradation
    gateGNN  = 30;   % was 35 (tighter => less track spam)
    gateMHT  = 35;   % keep TOMHT a bit looser to handle ambiguity
    gateJPDA = 40;   % JPDA often wants looser than GNN, but not huge
else
    gateGNN  = 45;
    gateMHT  = 45;
    gateJPDA = 45;
end

% --- Beta controls track birth intensity (most important knob for track spam) ---
% Your run showed lots of NaN tracks => beta is too "encouraging" under clutter.
if enableDegradation
    beta = 1e-13;   % was 1e-14 (two orders lower)
else
    beta = 1e-14;
end

% Scale clutter assumptions per tracker type (still sensor-driven)
if enableDegradation
    farGNN  = clutterDensity_from_data * tune.farScaleGNN;   % <-- FIX: use tune.farScaleGNN
    farMHT  = clutterDensity_from_data * 1;
    farJPDA = clutterDensity_from_data * 1;
else
    farGNN  = clutterDensity_from_data * 1;
    farMHT  = clutterDensity_from_data * 1;
    farJPDA = clutterDensity_from_data * 1;
end

% JPDA expects clutter density (false alarms per m^3)
clutterDensityJPDA = farJPDA;

fprintf("Scenario: %s | gateGNN=%.2f gateMHT=%.2f gateJPDA=%.2f | FAR(GNN)=%.3g 1/m^3 | FAR(MHT)=%.3g 1/m^3 | CD(JPDA)=%.3g 1/m^3 | pd=%.2f | vol=%.3g | beta=%.3g\n\n", ...
    ternary(enableDegradation,"RAINY","IDEAL"), gateGNN, gateMHT, gateJPDA, farGNN, farMHT, clutterDensityJPDA, pd, vol, beta);

%% GNN track-management thresholds (history-based)
% These are counts of detections/misses (not scores). Helps reject clutter tracks,
% and prevents aggressive deletion in degraded (RAINY) conditions.
if enableDegradation
    confirmGNN = 3;   % hits required to confirm
    deleteGNN  = 6;   % misses required to delete (slightly lenient under missed detections)
else
    confirmGNN = 2;
    deleteGNN  = 4;
end

%% TOMHT-only thresholds (score-based track management)
if enableDegradation
    confirmThresh = 35;  % slightly higher => fewer clutter tracks confirm
    deleteThresh  = -10; % slightly less aggressive deletion of real tracks
else
    confirmThresh = 20;
    deleteThresh  = -5;
end

%% TOMHT scenario-dependent knobs
% Scale thresholds relative to TOMHT gate
if enableDegradation
    tomhtThresh = [0.2, 5, 5] * gateMHT;
    maxBranches = 3;
else
    tomhtThresh = [0.2, 1, 1] * gateMHT;
    maxBranches = 5;
end

%% JPDA scenario-dependent knobs (initiation + anti-spam controls)
if enableDegradation
    betaJPDA      = 1e-12;      % was 1e-11 (slightly less spam)
    timeTolJPDA   = 0.05;
    numTracksJPDA = 120;        % cap down from 200
else
    betaJPDA      = beta;
    timeTolJPDA   = 0.05;
    numTracksJPDA = numTracks;
end


%% ============ GNN + CV ============
fprintf("\n============ GNN + CV ============\n");

% Baseline knobs for GNN (KEEP farScaleGNN; do NOT wipe tune with struct())
tune.enableDegradation = enableDegradation;
tune.gateGNN = gateGNN;
tune.beta    = beta;
tune.pd      = pd;
tune.maxNumTracksGNN = 80;  % cap during autotune to prevent spam (set [] to disable)

% ----- Run #1 -----
maxTracks1 = numTracks;
if enableAutoTune && ~isempty(tune.maxNumTracksGNN)
    maxTracks1 = tune.maxNumTracksGNN;
end

tracker = trackerGNN( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', maxTracks1, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', tune.gateGNN, ...
	    'TrackLogic', 'History', ...
	    'ConfirmationThreshold', confirmGNN, ...
	    'DeletionThreshold', deleteGNN, ...
    'ClutterDensity', farGNN);

[trackSummary1, truthSummary1, trackMetrics1, truthMetrics1, timeGNNCV1] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary1); disp(truthSummary1);
disp(trackMetrics1); disp(truthMetrics1);

% FIX: pass farScaleGNN into the report function
trackerAfterActionReport("GNN + CV (baseline)", trackSummary1, truthSummary1, sensorM, tune.gateGNN, tune.beta, tune.pd, tune.farScaleGNN);

% ----- AUTO-TUNE: one iteration -----
% ----- AUTO-TUNE: run until tolerance -----
if enableAutoTune
    fprintf("\n============ GNN + CV (AUTO-TUNE UNTIL TOL) ============\n");

    % --- stopping config ---
    maxIters  = 25;     % hard cap
    patience  = 6;      % stop if no improvement after N iterations

    tol.meanBreaks = 2.5;   % don't over-optimize breaks yet
    tol.shortRate  = 0.40;  % aim to cut spam massively
    tol.estFail    = 0;
    tol.minTracks  = 2*height(truthSummary1);   % guardrail: at least 2x truths

    % --- start from baseline results ---
    [bestJ, bestParts] = scoreGNNRun(trackSummary1, truthSummary1);
    bestTune   = tune;
    bestTS     = trackSummary1;
    bestTR     = truthSummary1;
    noImprove  = 0;

    % IMPORTANT FIX: keep "last run" separate from "best run"
    lastTS = trackSummary1;
    lastTR = truthSummary1;

    fprintf("[AUTO] baseline score J=%.3f | meanBreaks=%.2f shortRate=%.2f estFail=%d tracks=%d\n", ...
        bestJ, bestParts.meanBreaks, bestParts.shortRate, bestParts.estFail, bestParts.numTracks);

    % If already good enough, stop immediately
    if meetsTolerance(bestParts, tol)
        fprintf("[AUTO] Baseline already meets tolerance. Stopping.\n");
    else
        tuneIter = tune;

        for it = 1:maxIters
            % 1) update knobs based on LAST run symptoms (FIXED)
            tuneIter = autoTuneGNNOnce(lastTS, lastTR, sensorM, tuneIter);

            % 2) build tracker with updated knobs
            maxTracksIt = numTracks;
            if ~isempty(tuneIter.maxNumTracksGNN)
                maxTracksIt = tuneIter.maxNumTracksGNN;
            end

            farGNN_it = clutterDensity_from_data * tuneIter.farScaleGNN;

            tracker = trackerGNN( ...
                'FilterInitializationFcn', @initCVFilter, ...
                'MaxNumTracks', maxTracksIt, ...
                'MaxNumSensors', 1, ...
                'AssignmentThreshold', tuneIter.gateGNN, ...
	                'TrackLogic', 'History', ...
	                'ConfirmationThreshold', confirmGNN, ...
	                'DeletionThreshold', deleteGNN, ...
                'ClutterDensity', farGNN_it);

            [ts, tr, tm, trm, tsec] = helperRunTracker(dataLog, tracker, false);

            % FIX: update LAST summaries every iteration
            lastTS = ts;
            lastTR = tr;

            % 3) score + check tolerance
            [J, parts] = scoreGNNRun(ts, tr);

            fprintf("[AUTO it=%02d] J=%.3f | meanBreaks=%.2f shortRate=%.2f estFail=%d tracks=%d | gate=%.1f beta=%.1e pd=%.2f farScale=%.1f\n", ...
                it, J, parts.meanBreaks, parts.shortRate, parts.estFail, parts.numTracks, ...
                tuneIter.gateGNN, tuneIter.beta, tuneIter.pd, tuneIter.farScaleGNN);

            if meetsTolerance(parts, tol)
                fprintf("[AUTO] Tolerance met at iter %d. Stopping.\n", it);
                bestJ = J; bestParts = parts; bestTune = tuneIter; bestTS = ts; bestTR = tr;
                break
            end

            % 4) keep best-so-far + patience stop
            if J < bestJ
                bestJ = J; bestParts = parts; bestTune = tuneIter; bestTS = ts; bestTR = tr;
                noImprove = 0;
            else
                noImprove = noImprove + 1;
                if noImprove >= patience
                    fprintf("[AUTO] No improvement for %d iterations. Stopping.\n", patience);
                    break
                end
            end
        end
    end

    % Report best found
    fprintf("\n[AUTO] Best found: J=%.3f | meanBreaks=%.2f shortRate=%.2f estFail=%d tracks=%d\n", ...
        bestJ, bestParts.meanBreaks, bestParts.shortRate, bestParts.estFail, bestParts.numTracks);

    trackerAfterActionReport("GNN + CV (best autotuned)", bestTS, bestTR, sensorM, ...
        bestTune.gateGNN, bestTune.beta, bestTune.pd, bestTune.farScaleGNN);

    % If you want the rest of the script to use tuned values:
    if ~autoTuneOnlyGNN
        gateGNN = bestTune.gateGNN;
        beta    = bestTune.beta;
        pd      = bestTune.pd;

        tune.farScaleGNN = bestTune.farScaleGNN;
        farGNN = clutterDensity_from_data * tune.farScaleGNN;
    end
end
%% =================== GNN-only tuned knobs for later GNN runs ===================
% Use bestTune only if it exists (i.e., auto-tune actually executed).
if exist("bestTune","var")
    gateGNN_GNNONLY = bestTune.gateGNN;
    beta_GNNONLY    = bestTune.beta;
    pd_GNNONLY      = bestTune.pd;
    farGNN_GNNONLY  = clutterDensity_from_data * bestTune.farScaleGNN;

    maxTracks_GNNONLY = numTracks;
    if isfield(bestTune,"maxNumTracksGNN") && ~isempty(bestTune.maxNumTracksGNN)
        maxTracks_GNNONLY = bestTune.maxNumTracksGNN;
    end
else
    gateGNN_GNNONLY = gateGNN;
    beta_GNNONLY    = beta;
    pd_GNNONLY      = pd;
    farGNN_GNNONLY  = farGNN;
    maxTracks_GNNONLY = numTracks;
end
% ==============================================================================

% ==============================================================================

fprintf("[GNN-ONLY] gate=%.1f beta=%.1e pd=%.2f far=%.3g maxTracks=%d\n", ...
    gateGNN_GNNONLY, beta_GNNONLY, pd_GNNONLY, farGNN_GNNONLY, maxTracks_GNNONLY);



%% ============ GNN + IMM ============
fprintf("\n============ GNN + IMM ============\n");
tracker = trackerGNN( ...
    'FilterInitializationFcn', @initIMMFilter, ...
    'MaxNumTracks', maxTracks_GNNONLY, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gateGNN_GNNONLY, ...
	    'TrackLogic', 'History', ...
	    'ConfirmationThreshold', confirmGNN, ...
	    'DeletionThreshold', deleteGNN, ...
    'ClutterDensity', farGNN_GNNONLY);

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeGNNIMM] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);


%% ============ TOMHT + CV ============
fprintf("\n============ TOMHT + CV ============\n");
tracker = trackerTOMHT( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', tomhtThresh, ...
    'FalseAlarmRate', farMHT, ...
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
    'FalseAlarmRate', farMHT, ...
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
fprintf("\n============JPDA + CV==================\n");
tracker = trackerJPDA( ...
    'FilterInitializationFcn', @initCVFilter, ...
    'MaxNumTracks', numTracksJPDA, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold', gateJPDA, ...
    'TrackLogic', 'Integrated', ...
    'ClutterDensity', clutterDensityJPDA, ...
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
    'ClutterDensity', clutterDensityJPDA, ...
    'NewTargetDensity', betaJPDA, ...
    'TimeTolerance', timeTolJPDA);

[trackSummary, truthSummary, trackMetrics, truthMetrics, timeJPDAIMM] = helperRunTracker(dataLog, tracker, false);
disp(trackSummary); disp(truthSummary);
disp(trackMetrics); disp(truthMetrics);

fprintf("\n==============================\n");
fprintf(" RUN END | Scenario = %s\n", ternary(enableDegradation,"RAINY","IDEAL"));
fprintf("==============================\n\n");

%% -------- Local helper: ternary --------
function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

% ===================== FIXED: added farScaleGNN arg =====================
function trackerAfterActionReport(trackerName, trackSummary, truthSummary, sensorM, gate, beta, pd, farScaleGNN)
fprintf("Assumed clutter scale (GNN): %.1f × sensor\n", farScaleGNN);

fprintf("\n--- After-Action Tracker Report: %s ---\n", trackerName);

% ---------------- Track-level symptoms ----------------
numTracks = height(trackSummary);
shortTracks = sum(trackSummary.TotalLength < 5);
deadTracks  = sum(~trackSummary.Surviving);
meanLen     = mean(trackSummary.TotalLength);

fprintf("Tracks created: %d | Mean length: %.1f | Short tracks (<5): %d\n", ...
    numTracks, meanLen, shortTracks);

% ---------------- Truth-level symptoms ----------------
meanBreaks = mean(truthSummary.BreakCount);
maxBreaks  = max(truthSummary.BreakCount);
estFail = sum(isnan(truthSummary.AssociatedTrackID));

fprintf("Truth breaks: mean=%.2f | max=%d | failed establishment=%d/%d\n", ...
    meanBreaks, maxBreaks, estFail, height(truthSummary));

% ---------------- Sensor context ----------------
fprintf("Sensor context: Pd_est=%.2f | FA/scan=%.2f | scanT=%.2fs\n", ...
    sensorM.Pd_est, sensorM.FalseAlarmsPerScan, sensorM.ScanPeriod);

fprintf("Assumed Pd used by tracker: %.2f\n", pd);

% ================= Recommendations =================
fprintf("\nRecommended tuning actions:\n");

% Track spam
if numTracks > 5*height(truthSummary)
    fprintf(" • TRACK SPAM detected → Reduce Beta (current %.1e)\n", beta);
end

% Short tracks
if shortTracks > height(truthSummary)
    fprintf(" • Many short-lived tracks → Tighten gate (current %.1f) or reduce Beta\n", gate);
end

% Breaks
if meanBreaks > 3
    fprintf(" • Frequent truth breaks → Increase gate by +5 (current %.1f)\n", gate);
end

% Establishment failure
if estFail > 0
    fprintf(" • Track initiation weak → Increase Beta or reduce gate\n");
end

% Pd mismatch checks
if pd > sensorM.Pd_est + 0.1
    fprintf(" • Pd too optimistic → Lower Pd toward %.2f\n", sensorM.Pd_est);
end

if pd < sensorM.Pd_est - 0.2
    fprintf(" • Pd too pessimistic → Increase Pd to avoid track breaks\n");
end

% Healthy case
if meanBreaks <= 1 && shortTracks < height(truthSummary)
    fprintf(" • Tracker behavior is STABLE — no immediate tuning needed\n");
end

fprintf("--- End After-Action Report ---\n\n");
end


function tune = autoTuneGNNOnce(trackSummary, truthSummary, sensorM, tune)
%AUTOTUNEGNNONCE One iteration: adjust beta/gate based on symptoms.

numTruth  = height(truthSummary);
numTracks = height(trackSummary);
shortTracks = sum(trackSummary.TotalLength < 5);
meanBreaks  = mean(truthSummary.BreakCount);
maxBreaks   = max(truthSummary.BreakCount);

% Track spam heuristic
spam = (numTracks > 20*numTruth) || (shortTracks > 10*numTruth);

% Break heuristic
breaky = (meanBreaks >= 3) || (maxBreaks >= 10);

fprintf("\n[AUTO-TUNE] spam=%d | breaky=%d | tracks=%d | short=%d | meanBreaks=%.2f | maxBreaks=%d\n", ...
    spam, breaky, numTracks, shortTracks, meanBreaks, maxBreaks);

% Keep pd clamped in rainy
if tune.enableDegradation
    tune.pd = min(tune.pd, 0.85);
end

% 1) If spam, reduce beta and ALSO increase clutter belief progressively
if spam
    % Always nudge clutter scale up when spam is present (helps a lot in practice)
    oldFS = tune.farScaleGNN;
    tune.farScaleGNN = min(50, max(1, tune.farScaleGNN * 1.5));
    if tune.farScaleGNN ~= oldFS
        fprintf("[AUTO-TUNE] Track spam → farScaleGNN %.1f -> %.1f\n", oldFS, tune.farScaleGNN);
    end

    % Then reduce beta (until floor)
    if tune.beta > 1e-14
        old = tune.beta;
        tune.beta = max(1e-14, tune.beta/10);
        fprintf("[AUTO-TUNE] Track spam → beta %.1e -> %.1e\n", old, tune.beta);
    end
end



% 2) If breaky, increase gate modestly (helps maintain associations)
if breaky
    old = tune.gateGNN;
    tune.gateGNN = min(60, tune.gateGNN + 5);
    fprintf("[AUTO-TUNE] Frequent breaks → gateGNN %.1f -> %.1f\n", old, tune.gateGNN);
end

fprintf("[AUTO-TUNE] New knobs: gateGNN=%.1f | beta=%.1e | pd=%.2f | farScaleGNN=%.1f\n\n", ...
    tune.gateGNN, tune.beta, tune.pd, tune.farScaleGNN);
end


function printDelta(beforeTruth, afterTruth, beforeTracks, afterTracks)
fprintf("\n=== AUTO-TUNE DELTA (Before -> After) ===\n");
fprintf("Tracks created: %d -> %d\n", height(beforeTracks), height(afterTracks));
fprintf("Short tracks (<5): %d -> %d\n", ...
    sum(beforeTracks.TotalLength < 5), sum(afterTracks.TotalLength < 5));
fprintf("Mean BreakCount: %.2f -> %.2f\n", mean(beforeTruth.BreakCount), mean(afterTruth.BreakCount));
fprintf("Max BreakCount : %d -> %d\n", max(beforeTruth.BreakCount), max(afterTruth.BreakCount));
fprintf("========================================\n\n");
end
function [J, parts] = scoreGNNRun(trackSummary, truthSummary)
% Lower is better. Returns scalar score J and useful parts for stopping.

numTruth    = height(truthSummary);
numTracks   = height(trackSummary);
shortTracks = sum(trackSummary.TotalLength < 5);

meanBreaks  = mean(truthSummary.BreakCount);
maxBreaks   = max(truthSummary.BreakCount);
estFail     = sum(isnan(truthSummary.AssociatedTrackID));

shortRate = shortTracks / max(1,numTracks);

parts.meanBreaks = meanBreaks;
parts.maxBreaks  = maxBreaks;
parts.shortRate  = shortRate;
parts.numTracks  = numTracks;
parts.estFail    = estFail;

% Your situation: spam is the main issue, breaks are moderate.
w_break    = 8;     % keep breaks important
w_short    = 25;    % heavily punish spam/short tracks
w_tracks   = 0.10;  % punish huge track counts
w_est      = 40;    % establishment failure is a big deal
w_maxbreak = 1;     % small extra penalty

J = w_break*meanBreaks ...
  + w_maxbreak*maxBreaks ...
  + w_short*shortRate ...
  + w_tracks*(numTracks/max(1,numTruth)) ...
  + w_est*(estFail/max(1,numTruth));
end

function ok = meetsTolerance(parts, tol)
ok = true;
ok = ok && (parts.meanBreaks <= tol.meanBreaks);
ok = ok && (parts.shortRate  <= tol.shortRate);
ok = ok && (parts.estFail    <= tol.estFail);

% Guardrail: don't "win" by producing too few tracks
if isfield(tol,"minTracks")
    ok = ok && (parts.numTracks >= tol.minTracks);
end
end
