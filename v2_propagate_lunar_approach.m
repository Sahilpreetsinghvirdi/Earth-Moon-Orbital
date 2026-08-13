function trajectory = v2_propagate_lunar_approach(config, startEpoch, duration_s, initialState, altitude_m, callback, stopFunction)
if nargin < 6 || isempty(callback)
    callback = @(~) true;
end
if nargin < 7 || isempty(stopFunction)
    stopFunction = @() false;
end
step_s = lunar_step(config);
burnDuration_s = min(config.mission.captureBurnDuration_s, duration_s / 12);
stepCount = ceil(duration_s / step_s) + 1;
time_s = nan(stepCount, 1);
epoch = NaT(stepCount, 1, 'TimeZone', 'UTC');
position_m = nan(stepCount, 3);
velocity_mps = nan(stepCount, 3);
acceleration_mps2 = nan(stepCount, 3);
moonPosition_m = nan(stepCount, 3);
moonDistance_m = nan(stepCount, 1);
earthDistance_m = nan(stepCount, 1);
specificEnergyEarth_Jkg = nan(stepCount, 1);
specificEnergyMoon_Jkg = nan(stepCount, 1);
phase = strings(stepCount, 1);
valid = false(stepCount, 1);
moonStart = v2_get_celestial_state(startEpoch, 'moon', config, 'analytical');
relativePosition = initialState.position_m(:) - moonStart.position_m;
relativeVelocity = initialState.velocity_mps(:) - moonStart.velocity_mps;
approachTargetAltitude_m = config.mission.trajectoryLunarPeriapsisAltitude_m + config.mission.lunarApproachSafetyAltitude_m;
targetPosition = target_periapsis_position(relativePosition, relativeVelocity, approachTargetAltitude_m, config);
transfer = v2_solve_lambert(relativePosition, targetPosition, duration_s, config.physics.muMoon_m3ps2, true);
if transfer.valid
    burnDeltaV = transfer.velocity1_mps - relativeVelocity;
    targetVelocity = transfer.velocity2_mps;
    for iteration = 1:3
        [burnPosition, burnVelocity] = propagate_burn(relativePosition, relativeVelocity, burnDeltaV, burnDuration_s);
        remainingDuration_s = duration_s - burnDuration_s;
        correctionTransfer = v2_solve_lambert(burnPosition, targetPosition, remainingDuration_s, config.physics.muMoon_m3ps2, true);
        if ~correctionTransfer.valid
            break
        end
        correction_mps = correctionTransfer.velocity1_mps - burnVelocity;
        burnDeltaV = burnDeltaV + correction_mps;
        targetVelocity = correctionTransfer.velocity2_mps;
        transfer = correctionTransfer;
        if norm(correction_mps) < 0.02
            break
        end
    end
else
    burnDeltaV = inward_transfer_velocity(relativePosition, relativeVelocity, approachTargetAltitude_m, config) - relativeVelocity;
    targetVelocity = relativeVelocity + burnDeltaV;
end
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= duration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    moon = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
    currentAcceleration = relative_acceleration(currentTime_s, relativePosition, burnDeltaV, burnDuration_s, config);
    writeIndex = writeIndex + 1;
    time_s(writeIndex) = currentTime_s;
    epoch(writeIndex) = currentEpoch;
    position_m(writeIndex, :) = (moon.position_m + relativePosition)';
    velocity_mps(writeIndex, :) = (moon.velocity_mps + relativeVelocity)';
    acceleration_mps2(writeIndex, :) = currentAcceleration';
    moonPosition_m(writeIndex, :) = moon.position_m';
    moonDistance_m(writeIndex) = norm(relativePosition);
    earthDistance_m(writeIndex) = norm(moon.position_m + relativePosition);
    specificEnergyEarth_Jkg(writeIndex) = 0.5 * dot(velocity_mps(writeIndex, :)', velocity_mps(writeIndex, :)') - config.physics.muEarth_m3ps2 / max(earthDistance_m(writeIndex), 1);
    specificEnergyMoon_Jkg(writeIndex) = 0.5 * dot(relativeVelocity, relativeVelocity) - config.physics.muMoon_m3ps2 / max(moonDistance_m(writeIndex), 1);
    if currentTime_s < burnDuration_s
        phase(writeIndex) = "Lunar approach burn";
    else
        phase(writeIndex) = "Lunar approach coast";
    end
    valid(writeIndex) = all(isfinite([relativePosition; relativeVelocity; currentAcceleration]));
    state = struct('time_s', currentTime_s, 'epoch', currentEpoch, 'position_m', position_m(writeIndex, :)', 'velocity_mps', velocity_mps(writeIndex, :)', 'acceleration_mps2', currentAcceleration, 'moonPosition_m', moon.position_m, 'moonDistance_m', moonDistance_m(writeIndex), 'earthDistance_m', earthDistance_m(writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(writeIndex), 'phase', phase(writeIndex), 'valid', valid(writeIndex));
    if ~callback(state) || ~valid(writeIndex)
        break
    end
    if moonDistance_m(writeIndex) <= config.physics.moonRadius_m
        break
    end
    dt_s = min(step_s, duration_s - currentTime_s);
    if dt_s <= 0
        break
    end
    [relativePosition, relativeVelocity] = rk4_step(currentTime_s, relativePosition, relativeVelocity, dt_s, burnDeltaV, burnDuration_s, config);
    currentTime_s = currentTime_s + dt_s;
end
trajectory = struct('time_s', time_s(1:writeIndex), 'epoch', epoch(1:writeIndex), 'position_m', position_m(1:writeIndex, :), 'velocity_mps', velocity_mps(1:writeIndex, :), 'acceleration_mps2', acceleration_mps2(1:writeIndex, :), 'moonPosition_m', moonPosition_m(1:writeIndex, :), 'moonDistance_m', moonDistance_m(1:writeIndex), 'earthDistance_m', earthDistance_m(1:writeIndex), 'specificEnergyEarth_Jkg', specificEnergyEarth_Jkg(1:writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(1:writeIndex), 'phase', phase(1:writeIndex), 'valid', valid(1:writeIndex), 'completed', writeIndex >= min(stepCount, ceil(duration_s / step_s)), 'collision', any(moonDistance_m(1:writeIndex) <= config.physics.moonRadius_m), 'targetPosition_m', targetPosition, 'targetVelocity_mps', targetVelocity, 'lambert', transfer);

    function [finalPosition, finalVelocity] = propagate_burn(initialPosition, initialVelocity, deltaV_mps, burnLength_s)
        finalPosition = initialPosition;
        finalVelocity = initialVelocity;
        elapsed_s = 0;
        while elapsed_s < burnLength_s - 1e-9
            dt_s = min(step_s, burnLength_s - elapsed_s);
            [finalPosition, finalVelocity] = rk4_step(elapsed_s, finalPosition, finalVelocity, dt_s, deltaV_mps, burnLength_s, config);
            elapsed_s = elapsed_s + dt_s;
        end
    end
end

function value = lunar_step(config)
if isfield(config.integrator, 'lunarStep_s')
    value = min(config.integrator.highFidelityStep_s, config.integrator.lunarStep_s);
else
    value = config.integrator.highFidelityStep_s;
end
end

function acceleration = relative_acceleration(time_s, position_m, burnDeltaV_mps, burnDuration_s, config)
gravity = -config.physics.muMoon_m3ps2 * position_m / max(norm(position_m), 1) ^ 3;
if time_s >= burnDuration_s || burnDuration_s <= 0
    acceleration = gravity;
    return
end
normalizedTime = min(1, max(0, time_s / burnDuration_s));
burnProfile = 30 * normalizedTime * (1 - normalizedTime) ^ 4 / burnDuration_s;
acceleration = gravity + burnProfile * burnDeltaV_mps;
end

function [nextPosition, nextVelocity] = rk4_step(time_s, position_m, velocity_mps, dt_s, burnDeltaV_mps, burnDuration_s, config)
k1 = relative_acceleration(time_s, position_m, burnDeltaV_mps, burnDuration_s, config);
k2 = relative_acceleration(time_s + dt_s / 2, position_m + velocity_mps * dt_s / 2, burnDeltaV_mps, burnDuration_s, config);
k3 = relative_acceleration(time_s + dt_s / 2, position_m + (velocity_mps + k1 * dt_s / 2) * dt_s / 2, burnDeltaV_mps, burnDuration_s, config);
k4 = relative_acceleration(time_s + dt_s, position_m + (velocity_mps + k2 * dt_s / 2) * dt_s, burnDeltaV_mps, burnDuration_s, config);
nextPosition = position_m + dt_s / 6 * (velocity_mps + 2 * (velocity_mps + k1 * dt_s / 2) + 2 * (velocity_mps + k2 * dt_s / 2) + (velocity_mps + k3 * dt_s));
nextVelocity = velocity_mps + dt_s / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
end

function targetPosition = target_periapsis_position(position_m, velocity_mps, altitude_m, config)
radius_m = norm(position_m);
radial = position_m / max(radius_m, 1);
angularMomentum = cross(position_m, velocity_mps);
if norm(angularMomentum) < eps
    normal = [0; 0; 1];
else
    normal = angularMomentum / norm(angularMomentum);
end
tangential = cross(normal, radial);
tangential = tangential / max(norm(tangential), 1);
if dot(tangential, velocity_mps) < 0
    tangential = -tangential;
end
angle_rad = pi - deg2rad(2);
targetDirection = cos(angle_rad) * radial + sin(angle_rad) * tangential;
targetPosition = (config.physics.moonRadius_m + altitude_m) * targetDirection;
end

function velocity_mps = inward_transfer_velocity(position_m, incomingVelocity_mps, altitude_m, config)
radial = position_m / max(norm(position_m), 1);
angularMomentum = cross(position_m, incomingVelocity_mps);
if norm(angularMomentum) < eps
    normal = [0; 0; 1];
else
    normal = angularMomentum / norm(angularMomentum);
end
tangential = cross(normal, radial);
tangential = tangential / max(norm(tangential), 1);
if dot(tangential, incomingVelocity_mps) < 0
    tangential = -tangential;
end
targetRadius_m = config.physics.moonRadius_m + altitude_m;
speed_mps = sqrt(config.physics.muMoon_m3ps2 * (2 / norm(position_m) - 2 / (norm(position_m) + targetRadius_m)));
velocity_mps = tangential * speed_mps;
end
