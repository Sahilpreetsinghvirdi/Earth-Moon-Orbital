function trajectory = v2_propagate_lunar_orbit(config, startEpoch, duration_s, altitude_m, callback, stopFunction)
if nargin < 5 || isempty(callback)
    callback = @(~) true;
end
if nargin < 6 || isempty(stopFunction)
    stopFunction = @() false;
end
step_s = config.integrator.highFidelityStep_s;
stepCount = ceil(duration_s / step_s) + 1;
time_s = nan(stepCount, 1);
epoch = NaT(stepCount, 1, 'TimeZone', 'UTC');
position_m = nan(stepCount, 3);
velocity_mps = nan(stepCount, 3);
moonPosition_m = nan(stepCount, 3);
moonDistance_m = nan(stepCount, 1);
specificEnergyMoon_Jkg = nan(stepCount, 1);
phase = strings(stepCount, 1);
moonStart = v2_get_celestial_state(startEpoch, 'moon', config, 'analytical');
radial = -moonStart.position_m / norm(moonStart.position_m);
normal = [0; 0; 1];
tangential = cross(normal, radial);
tangential = tangential / norm(tangential);
relativePosition = radial * (config.physics.moonRadius_m + altitude_m);
relativeVelocity = tangential * sqrt(config.physics.muMoon_m3ps2 / norm(relativePosition));
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= duration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    moon = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
    writeIndex = writeIndex + 1;
    time_s(writeIndex) = currentTime_s;
    epoch(writeIndex) = currentEpoch;
    position_m(writeIndex, :) = (moon.position_m + relativePosition)';
    velocity_mps(writeIndex, :) = (moon.velocity_mps + relativeVelocity)';
    moonPosition_m(writeIndex, :) = moon.position_m';
    moonDistance_m(writeIndex) = norm(relativePosition);
    specificEnergyMoon_Jkg(writeIndex) = 0.5 * dot(relativeVelocity, relativeVelocity) - config.physics.muMoon_m3ps2 / max(norm(relativePosition), 1);
    phase(writeIndex) = "Lunar orbit";
    state = struct('time_s', currentTime_s, 'epoch', currentEpoch, 'position_m', position_m(writeIndex, :)', 'velocity_mps', velocity_mps(writeIndex, :)', 'moonPosition_m', moon.position_m, 'moonDistance_m', moonDistance_m(writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(writeIndex), 'phase', 'Lunar orbit', 'valid', true);
    if ~callback(state)
        break
    end
    dt_s = min(step_s, duration_s - currentTime_s);
    if dt_s <= 0
        break
    end
    [relativePosition, relativeVelocity] = rk4_step(relativePosition, relativeVelocity, dt_s);
    currentTime_s = currentTime_s + dt_s;
end
trajectory = struct('time_s', time_s(1:writeIndex), 'epoch', epoch(1:writeIndex), 'position_m', position_m(1:writeIndex, :), 'velocity_mps', velocity_mps(1:writeIndex, :), 'moonPosition_m', moonPosition_m(1:writeIndex, :), 'moonDistance_m', moonDistance_m(1:writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(1:writeIndex), 'phase', phase(1:writeIndex), 'valid', true(writeIndex, 1), 'completed', writeIndex >= min(stepCount, ceil(duration_s / step_s)), 'collision', any(moonDistance_m(1:writeIndex) <= config.physics.moonRadius_m));

    function [nextPosition, nextVelocity] = rk4_step(currentPosition, currentVelocity, dt)
        k1 = moon_acceleration(currentPosition);
        k2 = moon_acceleration(currentPosition + currentVelocity * dt / 2);
        k3 = moon_acceleration(currentPosition + (currentVelocity + k1 * dt / 2) * dt / 2);
        k4 = moon_acceleration(currentPosition + (currentVelocity + k2 * dt / 2) * dt);
        nextPosition = currentPosition + dt / 6 * (currentVelocity + 2 * (currentVelocity + k1 * dt / 2) + 2 * (currentVelocity + k2 * dt / 2) + (currentVelocity + k3 * dt));
        nextVelocity = currentVelocity + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end

    function acceleration = moon_acceleration(currentPosition)
        acceleration = -config.physics.muMoon_m3ps2 * currentPosition / max(norm(currentPosition), 1) ^ 3;
    end
end
