function validateSensors(sensors)
%validateSensors Validate sensors section.

if ~isstruct(sensors)
    error('validateSensors:type', 'sensors must be a struct.');
end

if isfield(sensors, 'sensors')
    entries = sensors.sensors;
    if ~isstruct(entries)
        error('validateSensors:shape', 'sensors.sensors must be an array of sensor structs.');
    end
    for i = 1:numel(entries)
        e = entries(i);
        if ~isfield(e, 'type')
            error('validateSensors:missingType', 'sensors.sensors(%d).type is required.', i);
        end
        if ~isfield(e, 'enabled')
            error('validateSensors:missingEnabled', 'sensors.sensors(%d).enabled is required.', i);
        end
    end
end
end
