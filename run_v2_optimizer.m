function output = run_v2_optimizer(options)
baseDirectory = fileparts(mfilename('fullpath'));
if nargin < 1
    options = struct;
end
config = v2_config(options);
if isfield(options, 'resume') && ~options.resume
    previousState = [];
else
    [previousState, ~] = v2_load_mission_state(config);
end
if isfield(options, 'epoch')
    config.epoch = v2_as_datetime(options.epoch);
    config.searchStartEpoch = config.epoch;
end
if isfield(options, 'searchStartEpoch')
    config.searchStartEpoch = v2_as_datetime(options.searchStartEpoch);
end
if isfield(options, 'searchEndEpoch')
    config.searchEndEpoch = v2_as_datetime(options.searchEndEpoch);
end
if config.randomized
    config.randomSeed = randi(2 ^ 31 - 1);
end
if config.optimizer.showProgress
    fprintf('%s\n', config.name)
    fprintf('Branch V2, epoch %s, search through %s\n', char(config.epoch), char(config.searchEndEpoch))
end
optimization = v2_optimize_mission(config, previousState);
if isempty(optimization.bestResult)
    trajectory = empty_trajectory();
else
    trajectory = v2_build_mission_trajectory(optimization.bestResult, config);
end
reportPath = v2_write_report(config, optimization, trajectory);
statePath = v2_save_mission_state(config, optimization, trajectory);
outputDirectory = fullfile(baseDirectory, config.output.directory);
resultPath = fullfile(outputDirectory, config.output.resultFile);
artifacts = struct('resultFile', resultPath, 'reportFile', reportPath, 'stateFile', statePath, 'dashboardFigure', '', 'trajectoryFigure', '');
if ~isempty(optimization.bestResult)
    artifacts = v2_visualize_dashboard(config, optimization, trajectory, artifacts);
end
output = struct('config', config, 'optimization', optimization, 'trajectory', trajectory, 'artifacts', artifacts, 'previousStateLoaded', ~isempty(previousState));
save(resultPath, 'output', '-v7.3')
if config.optimizer.showProgress
    if isempty(optimization.bestResult)
        fprintf('No valid mission candidate found.\n')
    else
        fprintf('Best launch date: %s\n', char(optimization.bestResult.candidate.launchEpoch))
        fprintf('Best total delta-v: %.3f m/s, estimated fuel %.3f kg\n', optimization.bestResult.totalDeltaV_mps, optimization.bestResult.fuel.requiredPropellant_kg)
    end
    fprintf('Result: %s\n', resultPath)
end
end

function trajectory = empty_trajectory()
trajectory = struct('time_s', zeros(0, 1), 'epoch', NaT(0, 1, 'TimeZone', 'UTC'), 'position_m', zeros(0, 3), 'velocity_mps', zeros(0, 3), 'moonPosition_m', zeros(0, 3), 'moonDistance_m', zeros(0, 1), 'earthDistance_m', zeros(0, 1), 'specificEnergyEarth_Jkg', zeros(0, 1), 'specificEnergyMoon_Jkg', zeros(0, 1), 'phase', strings(0, 1), 'valid', false(0, 1), 'completed', false, 'collision', false, 'burnEvents', struct([]));
end
