function [state, statePath] = v2_load_mission_state(config)
outputDirectory = fullfile(fileparts(mfilename('fullpath')), config.output.directory);
statePath = fullfile(outputDirectory, config.output.stateFile);
state = [];
if ~exist(statePath, 'file')
    return
end
loaded = load(statePath, 'missionState');
if ~isfield(loaded, 'missionState')
    return
end
state = loaded.missionState;
currentEpoch = v2_as_datetime(datetime('now', 'TimeZone', 'UTC'));
state.currentEpoch = currentEpoch;
state.currentMoonState = v2_get_celestial_state(currentEpoch, 'moon', config);
state.recalculatedFromEphemeris = true;
state.previousBestStillAvailable = isfield(state, 'bestResult') && ~isempty(state.bestResult);
end
