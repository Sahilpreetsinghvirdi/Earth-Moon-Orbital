function statePath = v2_save_mission_state(config, optimization, trajectory)
outputDirectory = fullfile(fileparts(mfilename('fullpath')), config.output.directory);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
statePath = fullfile(outputDirectory, config.output.stateFile);
currentEpoch = v2_as_datetime(datetime('now', 'TimeZone', 'UTC'));
currentMoonState = v2_get_celestial_state(currentEpoch, 'moon', config);
missionState = struct;
missionState.schemaVersion = '2.0';
missionState.savedAt = currentEpoch;
missionState.lastOptimizationEpoch = config.epoch;
missionState.currentEpoch = currentEpoch;
missionState.currentMoonState = currentMoonState;
missionState.ephemeris = config.ephemeris;
missionState.optimizerConfig = config.optimizer;
missionState.missionConfig = config.mission;
missionState.bestResult = optimization.bestResult;
missionState.bestTrajectory = trajectory;
missionState.bestLaunchDate = optimization.bestLaunchDate;
missionState.bestDeltaV_mps = optimization.bestDeltaV_mps;
missionState.bestFuel_kg = optimization.bestFuel_kg;
missionState.optimizationHistory = optimization.history;
missionState.candidatesTested = optimization.candidatesTested;
missionState.successfulCandidates = optimization.successfulCandidates;
missionState.stopReason = optimization.stopReason;
save(statePath, 'missionState', '-v7.3')
end
