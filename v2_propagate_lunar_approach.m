function trajectory = v2_propagate_lunar_approach(config, startEpoch, duration_s, initialState, altitude_m, callback, stopFunction)
if nargin < 6 || isempty(callback)
    callback = @(~) true;
end
if nargin < 7 || isempty(stopFunction)
    stopFunction = @() false;
end
step_s = config.integrator.highFidelityStep_s;
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
approachTargetAltitude_m = altitude_m + config.mission.lunarApproachSafetyAltitude_m;
targetPosition = target_periapsis_position(relativePosition, relativeVelocity, approachTargetAltitude_m, config);
transfer = v2_solve_lambert(relativePosition, targetPosition, duration_s, config.physics.muMoon_m3ps2, true);
if transfer.valid
    burnTargetVelocity = transfer.velocity1_mps;
    targetVelocity = transfer.velocity2_mps;
else
    burnTargetVelocity = inward_transfer_velocity(relativePosition, relativeVelocity, approachTargetAltitude_m, config);
    targetVelocity = burnTargetVelocity;
end
initialVelocity = relativeVelocity;
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= duration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    moon = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
    currentAcceleration = relative_acceleration(currentTime_s, relativePosition, relativeVelocity, currentEpoch);
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
    [relativePosition, relativeVelocity] = rk4_step(currentTime_s, relativePosition, relativeVelocity, currentEpoch, dt_s);
    currentTime_s = currentTime_s + dt_s;
end
trajectory = struct('time_s', time_s(1:writeIndex), 'epoch', epoch(1:writeIndex), 'position_m', position_m(1:writeIndex, :), 'velocity_mps', velocity_mps(1:writeIndex, :), 'acceleration_mps2', acceleration_mps2(1:writeIndex, :), 'moonPosition_m', moonPosition_m(1:writeIndex, :), 'moonDistance_m', moonDistance_m(1:writeIndex), 'earthDistance_m', earthDistance_m(1:writeIndex), 'specificEnergyEarth_Jkg', specificEnergyEarth_Jkg(1:writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(1:writeIndex), 'phase', phase(1:writeIndex), 'valid', valid(1:writeIndex), 'completed', writeIndex >= min(stepCount, ceil(duration_s / step_s)), 'collision', any(moonDistance_m(1:writeIndex) <= config.physics.moonRadius_m), 'targetPosition_m', targetPosition, 'targetVelocity_mps', targetVelocity, 'lambert', transfer);

    function acceleration = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentEpoch)
        gravity = -config.physics.muMoon_m3ps2 * currentPosition / max(norm(currentPosition), 1) ^ 3;
        if timeValue_s < burnDuration_s
            normalizedTime = min(1, max(0, timeValue_s / max(burnDuration_s, eps)));
            desiredDerivative = (6 * normalizedTime - 6 * normalizedTime ^ 2) * (burnTargetVelocity - initialVelocity) / max(burnDuration_s, eps);
            acceleration = desiredDerivative;
        else
            acceleration = gravity;
        end
    end

    function [nextPosition, nextVelocity] = rk4_step(timeValue_s, currentPosition, currentVelocity, currentEpoch, dt)
        k1 = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentEpoch);
        k2 = relative_acceleration(timeValue_s + dt / 2, currentPosition + currentVelocity * dt / 2, currentVelocity + k1 * dt / 2, currentEpoch + seconds(dt / 2));
        k3 = relative_acceleration(timeValue_s + dt / 2, currentPosition + (currentVelocity + k1 * dt / 2) * dt / 2, currentVelocity + k2 * dt / 2, currentEpoch + seconds(dt / 2));
        k4 = relative_acceleration(timeValue_s + dt, currentPosition + (currentVelocity + k2 * dt / 2) * dt, currentVelocity + k3 * dt, currentEpoch + seconds(dt));
        nextPosition = currentPosition + dt / 6 * (currentVelocity + 2 * (currentVelocity + k1 * dt / 2) + 2 * (currentVelocity + k2 * dt / 2) + (currentVelocity + k3 * dt));
        nextVelocity = currentVelocity + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
end

function targetPosition = target_periapsis_position(position, velocity, altitude_m, config)
radius = norm(position);
radial = position / max(radius, 1);
angularMomentum = cross(position, velocity);
if norm(angularMomentum) < eps
    normal = [0; 0; 1];
else
    normal = angularMomentum / norm(angularMomentum);
end
tangential = cross(normal, radial);
tangential = tangential / max(norm(tangential), 1);
if dot(tangential, velocity) < 0
    tangential = -tangential;
end
angle_rad = pi - deg2rad(2);
targetDirection = cos(angle_rad) * radial + sin(angle_rad) * tangential;
targetPosition = (config.physics.moonRadius_m + altitude_m) * targetDirection;
end

function velocity = inward_transfer_velocity(position, incomingVelocity, altitude_m, config)
radial = position / max(norm(position), 1);
angularMomentum = cross(position, incomingVelocity);
if norm(angularMomentum) < eps
    normal = [0; 0; 1];
else
    normal = angularMomentum / norm(angularMomentum);
end
tangential = cross(normal, radial);
tangential = tangential / max(norm(tangential), 1);
if dot(tangential, incomingVelocity) < 0
    tangential = -tangential;
end
targetRadius = config.physics.moonRadius_m + altitude_m;
speed = sqrt(config.physics.muMoon_m3ps2 * (2 / norm(position) - 2 / (norm(position) + targetRadius)));
velocity = tangential * speed;
end
