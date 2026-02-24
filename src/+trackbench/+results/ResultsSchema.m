% src/+trackbench/+results/ResultsSchema.m

classdef ResultsSchema
    properties (Constant)
        REQUIRED_FIELDS = {'run_id', 'timestamp', 'config', 'metrics'};
    end
    
    methods (Static)
        function results = create()
            % Create empty results struct with standard fields
            results = struct();
            results.run_id = '';
            results.timestamp = datetime('now');
            results.config = struct();
            results.tracks = [];
            results.metrics = struct();
            results.execution_time_s = 0;
            results.tracker_diagnostics = struct();
            results.tracker_results = struct();
            results.errors = {};
        end
        
        function isValid = validate(results)
            % Check results struct has required fields
            isValid = all(isfield(results, ResultsSchema.REQUIRED_FIELDS));
        end
    end
end
