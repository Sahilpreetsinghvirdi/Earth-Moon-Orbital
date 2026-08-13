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
launchState = struct('position_m', result.earthParkingPosition_m, 'velocity_mps', result.earthParkingVelocity_mps);
outboundDuration_s = result.candidate.outboundFlightTime_days * 86400;
earthDepartureBurnDuration_s = min(config.mission.earthDepartureBurnDuration_s, outboundDuration_s / 4);
earthDepartureDeltaV_mps = retarget_earth_departure(result, config, launchState, outboundDuration_s, earthDepartureBurnDuration_s);
outbound = v2_propagate_trajectory(config, result.candidate.launchEpoch, launchState, outboundDuration_s, 'Earth departure and lunar transfer', callback, stopFunction, earthDepartureDeltaV_mps, earthDepartureBurnDuration_s, "delta-v");
outboundFinalState = struct('position_m', outbound.position_m(end, :)', 'velocity_mps', outbound.velocity_mps(end, :)');
if ~isfield(result.candidate, 'lunarApproachDuration_days')
    result.candidate.lunarApproachDuration_days = config.mission.lunarApproachDuration_days;
end
if ~isfield(result, 'insertionEpoch')
    result.insertionEpoch = result.arrivalEpoch + days(result.candidate.lunarApproachDuration_days);
end
approach = v2_propagate_lunar_approach(config, result.arrivalEpoch, result.candidate.lunarApproachDuration_days * 86400, outboundFinalState, result.candidate.lunarOrbitAltitude_m, callback, stopFunction);
approachFinalState = struct('position_m', approach.position_m(end, :)', 'velocity_mps', approach.velocity_mps(end, :)');
departureTargetVelocity_mps = result.returnLambert.velocity1_mps;
actualReturnLambert = result.returnLambert;
for targetingIteration = 1:3
    orbit = v2_propagate_lunar_orbit(config, result.insertionEpoch, result.candidate.lunarOrbitDuration_days * 86400, result.candidate.lunarOrbitAltitude_m, callback, stopFunction, approachFinalState, departureTargetVelocity_mps, result.moonDepartureState.velocity_mps);
    returnStartPosition_m = orbit.position_m(end, :)';
    [actualReturnLambert, actualReturnTargetPosition_m] = select_return_lambert(config, orbit.epoch(end), returnStartPosition_m, orbit.velocity_mps(end, :)', orbit.moonPosition_m(end, :)', result.earthArrivalPosition_m, result.candidate.returnFlightTime_days * 86400);
    if ~actualReturnLambert.valid
        break
    end
    departureTargetVelocity_mps = actualReturnLambert.velocity1_mps;
end
returnState = struct('position_m', orbit.position_m(end, :)', 'velocity_mps', orbit.velocity_mps(end, :)');
actualDepartureEpoch = orbit.epoch(end);
returnDuration_s = result.candidate.returnFlightTime_days * 86400;
terminalDuration_s = min(config.mission.earthTerminalCorrectionDuration_s, returnDuration_s / 2);
mainReturnDuration_s = returnDuration_s - terminalDuration_s;
returnSegment = v2_propagate_trajectory(config, actualDepartureEpoch, returnState, mainReturnDuration_s, 'Lunar departure and Earth return', callback, stopFunction);
terminalStart = struct('position_m', returnSegment.position_m(end, :)', 'velocity_mps', returnSegment.velocity_mps(end, :)');
terminalLambert = v2_solve_lambert(terminalStart.position_m, actualReturnTargetPosition_m, terminalDuration_s, config.physics.muEarth_m3ps2, true);
if terminalLambert.valid
    terminalSegment = v2_propagate_trajectory(config, returnSegment.epoch(end), terminalStart, terminalDuration_s, 'Earth terminal approach and capture', callback, stopFunction, terminalLambert.velocity1_mps, config.mission.earthTerminalBurnDuration_s);
    returnSegment = concatenate_segments(normalize_segment(returnSegment, 'Lunar departure and Earth return'), normalize_segment(terminalSegment, 'Earth terminal approach and capture'));
end
outbound = normalize_segment(outbound, 'Earth departure and lunar transfer');
approach = normalize_segment(approach, 'Lunar approach');
orbit = normalize_segment(orbit, 'Lunar orbit');
returnSegment = normalize_segment(returnSegment, 'Lunar departure and Earth return');
trajectory = concatenate_segments(outbound, approach, orbit, returnSegment);
trajectory.actualReturnLambert = actualReturnLambert;
trajectory.returnTargetPosition_m = actualReturnTargetPosition_m;
actualEarthArrivalEpoch = trajectory.epoch(end);
trajectory.burnEvents = struct('epoch', {result.candidate.launchEpoch, result.arrivalEpoch, result.insertionEpoch, actualDepartureEpoch, actualEarthArrivalEpoch}, 'name', {'Earth departure', 'Lunar approach burn', 'Lunar orbit insertion', 'Lunar departure', 'Earth capture'}, 'deltaV_mps', {result.departureDeltaV_mps, norm(approach.targetVelocity_mps - (outboundFinalState.velocity_mps - result.moonArrivalState.velocity_mps)), result.lunarOrbitInsertionDeltaV_mps, result.lunarDepartureDeltaV_mps, result.earthCaptureDeltaV_mps});
trajectory.result = result;
trajectory.actualDepartureEpoch = actualDepartureEpoch;
trajectory.actualEarthArrivalEpoch = actualEarthArrivalEpoch;
outboundCount = numel(outbound.time_s);
approachCount = numel(approach.time_s);
orbitCount = numel(orbit.time_s);
returnStartIndex = outboundCount + approachCount + orbitCount + 1;
trajectory.lunarSOIEntered = any(trajectory.moonDistance_m <= config.physics.moonSOI_m);
trajectory.lunarEncounterDistance_m = min(trajectory.moonDistance_m);
trajectory.lunarOrbitMinimumDistance_m = min(orbit.moonDistance_m);
trajectory.lunarOrbitPeriapsisAltitude_m = trajectory.lunarOrbitMinimumDistance_m - config.physics.moonRadius_m;
orbitCoast = strcmp(orbit.phase, 'Lunar orbit');
trajectory.lunarOrbitBound = any(orbitCoast) && all(orbit.specificEnergyMoon_Jkg(orbitCoast) < 0);
trajectory.lunarOrbitValid = trajectory.lunarOrbitMinimumDistance_m <= config.physics.moonRadius_m + config.mission.maximumLunarPeriapsisAltitude_m && trajectory.lunarOrbitBound;
trajectory.earthArrivalDistance_m = trajectory.earthDistance_m(end);
trajectory.earthArrivalTargetError_m = norm(trajectory.position_m(end, :)' - actualReturnTargetPosition_m);
trajectory.earthArrivalSafe = trajectory.completed && ~trajectory.collision && trajectory.earthArrivalDistance_m <= config.mission.maximumEarthArrivalDistance_m;
if outboundCount > 0 && approachCount > 0 && orbitCount > 0 && returnStartIndex <= size(trajectory.position_m, 1)
    trajectory.phaseBoundaryJumps_m = [norm(approach.position_m(1, :) - outbound.position_m(end, :)), norm(orbit.position_m(1, :) - approach.position_m(end, :)), norm(returnSegment.position_m(1, :) - orbit.position_m(end, :))];
else
    trajectory.phaseBoundaryJumps_m = [nan, nan, nan];
end

function burnDeltaV_mps = retarget_earth_departure(result, config, launchState, outboundDuration_s, burnDuration_s)
burnDeltaV_mps = result.outboundLambert.velocity1_mps - launchState.velocity_mps;
if burnDuration_s <= 0 || outboundDuration_s <= burnDuration_s
    return
end
arrivalMoon = v2_get_celestial_state(result.arrivalEpoch, 'moon', config, 'analytical');
targetDirection_m = -arrivalMoon.position_m / max(norm(arrivalMoon.position_m), 1);
targetPosition_m = arrivalMoon.position_m + targetDirection_m * config.mission.lunarApproachStartDistance_m;
for iteration = 1:8
    burn = v2_propagate_trajectory(config, result.candidate.launchEpoch, launchState, burnDuration_s, 'Earth departure and lunar transfer', [], @() false, burnDeltaV_mps, burnDuration_s, "delta-v");
    if isempty(burn.position_m)
        return
    end
    transfer = v2_solve_lambert(burn.position_m(end, :)', targetPosition_m, outboundDuration_s - burnDuration_s, config.physics.muEarth_m3ps2, true);
    if ~transfer.valid
        return
    end
    correction_mps = transfer.velocity1_mps - burn.velocity_mps(end, :)';
    burnDeltaV_mps = burnDeltaV_mps + correction_mps;
    if norm(correction_mps) < 0.01
        break
    end
end
end

function [solution, targetPosition_m] = select_return_lambert(config, departureEpoch, startPosition_m, startVelocity_mps, moonPosition_m, baseTargetPosition_m, timeOfFlight_s)
solution = struct('valid', false, 'velocity1_mps', nan(3, 1), 'velocity2_mps', nan(3, 1), 'transferAngle_rad', nan, 'iterations', 0, 'residual_s', inf, 'message', 'No outward return branch');
targetPosition_m = baseTargetPosition_m;
relativePosition_m = startPosition_m - moonPosition_m;
moon = v2_get_celestial_state(departureEpoch, 'moon', config, 'analytical');
moonVelocity_mps = moon.velocity_mps;
bestScore = inf;
for angle_deg = -180:5:180
    rotation = [cosd(angle_deg), -sind(angle_deg), 0; sind(angle_deg), cosd(angle_deg), 0; 0, 0, 1];
    candidateTarget_m = rotation * baseTargetPosition_m;
    for prograde = [true, false]
        candidate = v2_solve_lambert(startPosition_m, candidateTarget_m, timeOfFlight_s, config.physics.muEarth_m3ps2, prograde);
        if ~candidate.valid
            continue
        end
        moonRadialVelocity_mps = dot(candidate.velocity1_mps - moonVelocity_mps, relativePosition_m / max(norm(relativePosition_m), 1));
        if moonRadialVelocity_mps <= 0
            continue
        end
        score = norm(candidate.velocity1_mps - startVelocity_mps) + 0.01 * norm(candidate.velocity2_mps);
        if score < bestScore
            bestScore = score;
            solution = candidate;
            targetPosition_m = candidateTarget_m;
        end
    end
end
if ~solution.valid
    solution = v2_solve_lambert(startPosition_m, baseTargetPosition_m, timeOfFlight_s, config.physics.muEarth_m3ps2, true);
end
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
