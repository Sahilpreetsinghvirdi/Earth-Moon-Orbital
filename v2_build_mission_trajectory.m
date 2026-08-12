function trajectory = v2_build_mission_trajectory(result, config, callback, stopFunction)
if nargin < 3 || isempty(callback)
    callback = @(~) true;
end
if nargin < 4 || isempty(stopFunction)
    stopFunction = @() false;
end
if isempty(result) || ~result.valid
    error('V2:InvalidBestResult', 'A valid mission result is required to build a trajectory.')
end
launchState = struct('position_m', result.earthParkingPosition_m, 'velocity_mps', result.outboundLambert.velocity1_mps);
outbound = v2_propagate_trajectory(config, result.candidate.launchEpoch, launchState, result.candidate.outboundFlightTime_days * 86400, 'Earth departure and lunar transfer', callback, stopFunction);
outboundFinalState = struct('position_m', outbound.position_m(end, :)', 'velocity_mps', outbound.velocity_mps(end, :)');
orbit = v2_propagate_lunar_orbit(config, result.arrivalEpoch, result.candidate.lunarOrbitDuration_days * 86400, result.candidate.lunarOrbitAltitude_m, callback, stopFunction, outboundFinalState, result.returnLambert.velocity1_mps, result.moonDepartureState.velocity_mps);
returnState = struct('position_m', orbit.position_m(end, :)', 'velocity_mps', orbit.velocity_mps(end, :)');
returnSegment = v2_propagate_trajectory(config, result.departureEpoch, returnState, result.candidate.returnFlightTime_days * 86400, 'Lunar departure and Earth return', callback, stopFunction);
outbound = normalize_segment(outbound, 'Earth departure and lunar transfer');
orbit = normalize_segment(orbit, 'Lunar orbit');
returnSegment = normalize_segment(returnSegment, 'Lunar departure and Earth return');
trajectory = concatenate_segments(outbound, orbit, returnSegment);
trajectory.burnEvents = struct('epoch', {result.candidate.launchEpoch, result.arrivalEpoch, result.departureEpoch, result.returnEpoch}, 'name', {'Earth departure', 'Lunar orbit insertion', 'Lunar departure', 'Earth capture'}, 'deltaV_mps', {result.departureDeltaV_mps, result.lunarOrbitInsertionDeltaV_mps, result.lunarDepartureDeltaV_mps, result.earthCaptureDeltaV_mps});
trajectory.result = result;
outboundCount = numel(outbound.time_s);
orbitCount = numel(orbit.time_s);
returnStartIndex = outboundCount + orbitCount + 1;
trajectory.lunarSOIEntered = any(outbound.moonDistance_m <= config.physics.moonSOI_m);
trajectory.lunarEncounterDistance_m = min(outbound.moonDistance_m);
trajectory.earthArrivalDistance_m = trajectory.earthDistance_m(end);
trajectory.earthArrivalSafe = trajectory.completed && ~trajectory.collision && trajectory.earthArrivalDistance_m <= config.physics.earthSOI_m;
if outboundCount > 0 && orbitCount > 0 && returnStartIndex <= size(trajectory.position_m, 1)
    trajectory.phaseBoundaryJumps_m = [norm(orbit.position_m(1, :) - outbound.position_m(end, :)), norm(returnSegment.position_m(1, :) - orbit.position_m(end, :))];
else
    trajectory.phaseBoundaryJumps_m = [nan, nan];
end
end

function segment = normalize_segment(segment, phaseName)
count = numel(segment.time_s);
if ~isfield(segment, 'acceleration_mps2')
    segment.acceleration_mps2 = nan(count, 3);
end
if ~isfield(segment, 'specificEnergyEarth_Jkg')
    segment.specificEnergyEarth_Jkg = nan(count, 1);
end
if ~isfield(segment, 'specificEnergyMoon_Jkg')
    segment.specificEnergyMoon_Jkg = nan(count, 1);
end
if ~isfield(segment, 'earthDistance_m')
    segment.earthDistance_m = vecnorm(segment.position_m, 2, 2);
end
if ~isfield(segment, 'moonDistance_m')
    segment.moonDistance_m = vecnorm(segment.position_m - segment.moonPosition_m, 2, 2);
end
if ~isfield(segment, 'phase') || numel(segment.phase) ~= count
    segment.phase = repmat(string(phaseName), count, 1);
end
end

function output = concatenate_segments(varargin)
segments = varargin;
output = struct;
fieldNames = {'time_s', 'epoch', 'position_m', 'velocity_mps', 'acceleration_mps2', 'moonPosition_m', 'moonDistance_m', 'earthDistance_m', 'specificEnergyEarth_Jkg', 'specificEnergyMoon_Jkg', 'phase', 'valid'};
for fieldIndex = 1:numel(fieldNames)
    fieldName = fieldNames{fieldIndex};
    values = cell(1, numel(segments));
    for segmentIndex = 1:numel(segments)
        values{segmentIndex} = segments{segmentIndex}.(fieldName);
        if segmentIndex > 1 && ~isempty(values{segmentIndex})
            values{segmentIndex} = values{segmentIndex}(2:end, :);
        end
    end
    if strcmp(fieldName, 'time_s')
        offset = 0;
        for segmentIndex = 1:numel(values)
            values{segmentIndex} = values{segmentIndex} + offset;
            if ~isempty(values{segmentIndex})
                offset = values{segmentIndex}(end);
            end
        end
    end
    output.(fieldName) = vertcat(values{:});
end
output.completed = all(cellfun(@(segment) segment.completed, segments));
output.collision = any(cellfun(@(segment) segment.collision, segments));
end
