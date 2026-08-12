function trajectory = v2_propagate_trajectory(config, startEpoch, initialState, duration_s, phaseName, callback, stopFunction, targetVelocity_mps, targetBurnDuration_s)
if nargin < 6 || isempty(callback)
    callback = @(~) true;
end
if nargin < 7 || isempty(stopFunction)
    stopFunction = @() false;
end
if nargin < 8
    targetVelocity_mps = [];
end
if nargin < 9 || isempty(targetBurnDuration_s)
    targetBurnDuration_s = 0;
end
step_s = config.integrator.highFidelityStep_s;
stepCount = min(ceil(duration_s / step_s) + 1, config.integrator.maxSteps);
time_s = nan(stepCount, 1);
epoch = NaT(stepCount, 1, 'TimeZone', 'UTC');
position_m = nan(stepCount, 3);
velocity_mps = nan(stepCount, 3);
acceleration_mps2 = nan(stepCount, 3);
moonPosition_m = nan(stepCount, 3);
moonDistance_m = nan(stepCount, 1);
earthDistance_m = nan(stepCount, 1);
specificEnergyEarth_Jkg = nan(stepCount, 1);
phase = strings(stepCount, 1);
valid = false(stepCount, 1);
position = initialState.position_m(:);
velocity = initialState.velocity_mps(:);
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= duration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    [acceleration, diagnostics] = step_acceleration(currentTime_s, currentEpoch, position, velocity);
    writeIndex = writeIndex + 1;
    time_s(writeIndex) = currentTime_s;
    epoch(writeIndex) = currentEpoch;
    position_m(writeIndex, :) = position';
    velocity_mps(writeIndex, :) = velocity';
    acceleration_mps2(writeIndex, :) = acceleration';
    moonPosition_m(writeIndex, :) = diagnostics.moonPosition_m';
    moonDistance_m(writeIndex) = diagnostics.moonDistance_m;
    earthDistance_m(writeIndex) = norm(position);
    specificEnergyEarth_Jkg(writeIndex) = 0.5 * dot(velocity, velocity) - config.physics.muEarth_m3ps2 / max(norm(position), 1);
    phase(writeIndex) = string(phaseName);
    valid(writeIndex) = all(isfinite([position; velocity; acceleration]));
    state = struct('time_s', currentTime_s, 'epoch', currentEpoch, 'position_m', position, 'velocity_mps', velocity, 'acceleration_mps2', acceleration, 'moonPosition_m', diagnostics.moonPosition_m, 'moonDistance_m', diagnostics.moonDistance_m, 'earthDistance_m', norm(position), 'phase', phaseName, 'valid', valid(writeIndex));
    if ~callback(state)
        break
    end
    if ~valid(writeIndex)
        break
    end
    if diagnostics.earthCollision || diagnostics.moonCollision
        break
    end
    dt_s = min(step_s, duration_s - currentTime_s);
    if dt_s <= 0
        break
    end
    [position, velocity] = rk4_step(currentTime_s, currentEpoch, position, velocity, dt_s);
    currentTime_s = currentTime_s + dt_s;
end
trajectory = struct;
trajectory.time_s = time_s(1:writeIndex);
trajectory.epoch = epoch(1:writeIndex);
trajectory.position_m = position_m(1:writeIndex, :);
trajectory.velocity_mps = velocity_mps(1:writeIndex, :);
trajectory.acceleration_mps2 = acceleration_mps2(1:writeIndex, :);
trajectory.moonPosition_m = moonPosition_m(1:writeIndex, :);
trajectory.moonDistance_m = moonDistance_m(1:writeIndex);
trajectory.earthDistance_m = earthDistance_m(1:writeIndex);
trajectory.specificEnergyEarth_Jkg = specificEnergyEarth_Jkg(1:writeIndex);
trajectory.phase = phase(1:writeIndex);
trajectory.valid = valid(1:writeIndex);
trajectory.completed = writeIndex > 0 && writeIndex >= min(stepCount, ceil(duration_s / step_s));
trajectory.collision = false;
if writeIndex > 0
    trajectory.collision = trajectory.earthDistance_m(end) <= config.physics.earthRadius_m || trajectory.moonDistance_m(end) <= config.physics.moonRadius_m;
end

    function [nextPosition, nextVelocity] = rk4_step(timeValue_s, currentEpoch, currentPosition, currentVelocity, dt)
        [k1, ~] = step_acceleration(timeValue_s, currentEpoch, currentPosition, currentVelocity);
        [k2, ~] = step_acceleration(timeValue_s + dt / 2, currentEpoch + seconds(dt / 2), currentPosition + currentVelocity * dt / 2, currentVelocity + k1 * dt / 2);
        [k3, ~] = step_acceleration(timeValue_s + dt / 2, currentEpoch + seconds(dt / 2), currentPosition + (currentVelocity + k1 * dt / 2) * dt / 2, currentVelocity + k2 * dt / 2);
        [k4, ~] = step_acceleration(timeValue_s + dt, currentEpoch + seconds(dt), currentPosition + (currentVelocity + k2 * dt / 2) * dt, currentVelocity + k3 * dt);
        nextPosition = currentPosition + dt / 6 * (currentVelocity + 2 * (currentVelocity + k1 * dt / 2) + 2 * (currentVelocity + k2 * dt / 2) + (currentVelocity + k3 * dt));
        nextVelocity = currentVelocity + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end

    function [totalAcceleration, diagnostics] = step_acceleration(timeValue_s, currentEpoch, currentPosition, currentVelocity)
        [totalAcceleration, diagnostics] = v2_acceleration(config, currentEpoch, currentPosition, currentVelocity);
        if ~isempty(targetVelocity_mps) && targetBurnDuration_s > 0 && timeValue_s < targetBurnDuration_s
            normalizedTime = min(1, max(0, timeValue_s / targetBurnDuration_s));
            desiredDerivative = (6 * normalizedTime - 6 * normalizedTime ^ 2) * (targetVelocity_mps(:) - initialState.velocity_mps(:)) / targetBurnDuration_s;
            totalAcceleration = desiredDerivative;
        end
    end
end

function [acceleration_mps2, diagnostics] = v2_acceleration(config, epoch, position_m, velocity_mps)
moon = v2_get_celestial_state(epoch, 'moon', config, 'analytical');
earthDistance_m = norm(position_m);
moonRelativePosition_m = position_m - moon.position_m;
moonDistance_m = norm(moonRelativePosition_m);
earthGravity = -config.physics.muEarth_m3ps2 * position_m / max(earthDistance_m, 1) ^ 3;
moonGravity = -config.physics.muMoon_m3ps2 * moonRelativePosition_m / max(moonDistance_m, 1) ^ 3;
sunGravity = zeros(3, 1);
sun = v2_get_celestial_state(epoch, 'sun', config, 'analytical');
if config.integrator.includeSunGravity
    sunRelativePosition_m = sun.position_m - position_m;
    sunGravity = config.physics.muSun_m3ps2 * (sunRelativePosition_m / max(norm(sunRelativePosition_m), 1) ^ 3 - sun.position_m / max(norm(sun.position_m), 1) ^ 3);
end
acceleration_mps2 = earthGravity + moonGravity + sunGravity;
diagnostics = struct('moonPosition_m', moon.position_m, 'moonVelocity_mps', moon.velocity_mps, 'moonDistance_m', moonDistance_m, 'earthCollision', earthDistance_m <= config.physics.earthRadius_m, 'moonCollision', moonDistance_m <= config.physics.moonRadius_m, 'ephemerisSource', moon.source);
end
