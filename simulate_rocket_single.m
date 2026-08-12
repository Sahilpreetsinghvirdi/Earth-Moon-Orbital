function trajectory = simulate_rocket_single(config, inputs, launchIndex)
singleInputs = struct;
fields = fieldnames(inputs);
for fieldIndex = 1:numel(fields)
    fieldName = fields{fieldIndex};
    singleInputs.(fieldName) = inputs.(fieldName)(launchIndex);
end
singleConfig = config;
singleConfig.launchCount = 1;
[singleResults, sampledTrajectory] = simulate_rocket_batch(singleConfig, singleInputs, 1);
trajectory = sampledTrajectory;
trajectory.launchIndex = launchIndex;
trajectory.results = singleResults;
validRows = find(~isnan(trajectory.altitude_m(:, 1)), 1, 'last');
if isempty(validRows)
    validRows = 1;
end
trajectory.time_s = trajectory.time_s(1:validRows);
trajectory.downrange_m = trajectory.downrange_m(1:validRows, 1);
trajectory.altitude_m = trajectory.altitude_m(1:validRows, 1);
trajectory.velocity_mps = trajectory.velocity_mps(1:validRows, 1);
trajectory.acceleration_mps2 = trajectory.acceleration_mps2(1:validRows, 1);
trajectory.dynamicPressure_Pa = trajectory.dynamicPressure_Pa(1:validRows, 1);
trajectory.active = trajectory.active(1:validRows, 1);
trajectory = rmfield(trajectory, 'indices');
end
