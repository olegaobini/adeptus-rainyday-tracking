function [results, detections] = runScenario(config, configName, detections)
% RUNSCENARIO  Backward-compatible wrapper for main-branch API.
%   Main branch expects: trackbench.runScenario(config, configName, detections)
%   This wrapper delegates to the V2 package functions while preserving
%   the original call signature and return format.
%
% INPUTS
%   config      : fully loaded/validated config struct
%   configName  : string identifier for the run (optional, default "")
%   detections  : precomputed detection log (optional, default [])
%
% OUTPUTS
%   results    : standard results struct (ResultsSchema-compatible)
%   detections : detection log struct used by the trackers
%
% See also: trackbench.batch.runExperiment, trackbench.detections.runDetections

    arguments
        config     (1,1) struct
        configName (1,1) string = ""
        detections                = []
    end

    %% Extract parameters from pre-loaded config
    numTargets    = config.scenario.num_targets;
    sceneDuration = config.scenario.duration_s;
    enableDegradation = config.degradation.enabled;
    params = config.active_params;
    pd     = params.pd;

    %% Generate detections if not provided
    if isempty(detections)
        scenario = trackbench.scenario.createScenario( ...
            "NumTargets",    numTargets, ...
            "SceneDuration", sceneDuration);

        % NOTE: runScenario is the main-branch compatible path.
        % We pass an empty envCfg so the basic default run behaves like
        % main's original pipeline (no VCP masking, no ground clutter,
        % no horizon masking). Use runExperiment() for full V2 features.
        envCfg = struct('horizon_masking', false, 'ground_clutter', false, ...
                        'propagation_model', false);

        detections = trackbench.detections.runDetections(scenario, enableDegradation, [], envCfg);
    end

    %% Validate scan coverage before tracking
    numScans = numel(detections.Time);
    minScansNeeded = 5;
    if numScans < minScansNeeded
        avgScanPeriod = sceneDuration / max(numScans, 1);
        suggestedDuration = ceil(minScansNeeded * avgScanPeriod * 1.5);
        warning('runScenario:insufficientScans', ...
            ['Only %d scan(s) in %.1fs — trackers need at least %d to establish tracks.\n' ...
             'Scan period ≈ %.1fs. Increase scenario.duration_s to at least %.0fs in default.json.'], ...
            numScans, sceneDuration, minScansNeeded, avgScanPeriod, suggestedDuration);
    else
        fprintf('[SCAN CHECK] %d scans in %.1fs — OK.\n', numScans, sceneDuration);
    end

    %% Visualization flags
    showVis = true;  animVis = true;
    if isfield(config.output, 'show_visuals');    showVis = config.output.show_visuals;    end
    if isfield(config.output, 'animate_visuals'); animVis = config.output.animate_visuals; end

    if showVis
        trackbench.reporting.plotInitialScenario(detections, animVis);
    end

    %% Detection diagnostics
    if config.output.print_diagnostics
        nPerScan = cellfun(@numel, detections.Detections);
        fprintf("Detections/scan stats: min=%g, mean=%.2f, max=%g\n", ...
            min(nPerScan), mean(nPerScan), max(nPerScan));
    end

    fprintf(" | gate=%.1f | farGNN=%.2e | farMHT=%.2e | farJPDA=%.2e | pd=%.2f | volume=%.2e | beta=%.2e\n", ...
        params.gate, params.far_gnn, params.far_mht, params.far_jpda, pd, ...
        config.tracker_global.volume, config.tracker_global.beta);

    %% Build results struct (ResultsSchema-compatible)
    results = trackbench.results.ResultsSchema.create();
    results.run_id = configName;
    results.config = config;

    %% Run enabled tracker combos
    trackerCombos = {
        {'GNN', 'CV',   config.trackers_to_run.gnn_cv};
        {'GNN', 'IMM',  config.trackers_to_run.gnn_imm};
        {'TOMHT','CV',  config.trackers_to_run.tomht_cv};
        {'TOMHT','IMM', config.trackers_to_run.tomht_imm};
        {'JPDA', 'CV',  config.trackers_to_run.jpda_cv};
        {'JPDA', 'IMM', config.trackers_to_run.jpda_imm}
    };

    for c = 1:length(trackerCombos)
        tType   = trackerCombos{c}{1};
        fModel  = trackerCombos{c}{2};
        enabled = trackerCombos{c}{3};
        if ~enabled; continue; end

        comboName = lower(sprintf('%s_%s', tType, fModel));
        fprintf('\n============ %s + %s ============\n', tType, fModel);

        tracker = trackbench.tracking.buildTracker(tType, fModel, params, ...
            config.tracker_global, config.filter_params, pd);

        [trackSummary, truthSummary, trackMetrics, truthMetrics, time] = ...
            trackbench.tracking.runTracker(detections, tracker, false, showVis, animVis);

        results.tracker_results.(comboName).trackSummary = trackSummary;
        results.tracker_results.(comboName).truthSummary = truthSummary;
        results.tracker_results.(comboName).trackMetrics = trackMetrics;
        results.tracker_results.(comboName).truthMetrics = truthMetrics;
        results.tracker_results.(comboName).time         = time;

        if config.output.print_diagnostics
            disp(trackSummary); disp(truthSummary);
            disp(trackMetrics); disp(truthMetrics);
        end
    end

    fprintf("\n==============================\n");
    fprintf(" RUN END\n");
    fprintf("==============================\n\n");
end
