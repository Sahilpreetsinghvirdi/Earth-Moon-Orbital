function output = v2_rebuild_live_output()
config = v2_config();
config.searchStartEpoch = datetime(2026, 8, 24, 0, 0, 0, 'TimeZone', 'UTC');
candidate = struct('launchEpoch', config.searchStartEpoch, 'outboundFlightTime_days', 4, 'returnFlightTime_days', 2, 'lunarApproachDuration_days', config.mission.lunarApproachDuration_days, 'lunarOrbitDuration_days', config.mission.lunarOrbitDuration_days, 'lunarOrbitAltitude_m', config.physics.moonParkingAltitude_m);
result = v2_evaluate_candidate(candidate, config);
if ~result.valid
    error('V2:RebuildFailure', 'The replacement Earth-Moon mission candidate is not valid.')
end
trajectory = v2_build_mission_trajectory(result, config);
if ~trajectory.completed || trajectory.collision || ~trajectory.lunarSOIEntered || ~trajectory.lunarOrbitValid || ~trajectory.earthArrivalSafe
    error('V2:RebuildFailure', 'The replacement Earth-Moon trajectory did not complete safely.')
end
optimization = struct('bestResult', result, 'bestLaunchDate', result.candidate.launchEpoch, 'bestDeltaV_mps', result.totalDeltaV_mps, 'bestFuel_kg', result.fuel.requiredPropellant_kg, 'candidatesTested', 1, 'successfulCandidates', 1, 'iterations', 1, 'stopReason', 'Rebuilt direct translunar mission');
baseDirectory = fileparts(mfilename('fullpath'));
outputDirectory = fullfile(baseDirectory, config.output.directory);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
resultPath = fullfile(outputDirectory, config.output.resultFile);
artifacts = struct('resultFile', resultPath, 'reportFile', '', 'stateFile', '', 'dashboardFigure', '', 'trajectoryFigure', '');
output = struct('config', config, 'optimization', optimization, 'trajectory', trajectory, 'artifacts', artifacts, 'previousStateLoaded', false);
temporaryPath = [resultPath, '.tmp'];
save(temporaryPath, 'output', '-v7.3')
movefile(temporaryPath, resultPath, 'f')
end
