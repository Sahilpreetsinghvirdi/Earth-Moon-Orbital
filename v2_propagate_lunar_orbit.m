function trajectory = v2_propagate_lunar_orbit(config, startEpoch, duration_s, altitude_m, callback, stopFunction, initialState, departureTargetVelocity_mps, departureMoonVelocity_mps)
if nargin < 5 || isempty(callback)
    callback = @(~) true;
end
if nargin < 6 || isempty(stopFunction)
    stopFunction = @() false;
end
if nargin < 7
    initialState = [];
end
if nargin < 8
    departureTargetVelocity_mps = [];
end
if nargin < 9
    departureMoonVelocity_mps = [];
end
step_s = config.integrator.highFidelityStep_s;
captureBurnDuration_s = min(config.mission.captureBurnDuration_s, duration_s / 3);
departureBurnDuration_s = min(config.mission.departureBurnDuration_s, duration_s / 3);
if captureBurnDuration_s + departureBurnDuration_s > duration_s
    departureBurnDuration_s = max(0, duration_s - captureBurnDuration_s);
end
departureBurnStart_s = max(captureBurnDuration_s, duration_s - departureBurnDuration_s);
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
if isempty(initialState)
    radial = -moonStart.position_m / norm(moonStart.position_m);
    normal = [0; 0; 1];
    tangential = cross(normal, radial);
    tangential = tangential / norm(tangential);
    relativePosition = radial * (config.physics.moonRadius_m + altitude_m);
    relativeVelocity = tangential * sqrt(config.physics.muMoon_m3ps2 / norm(relativePosition));
else
    relativePosition = initialState.position_m(:) - moonStart.position_m;
    relativeVelocity = initialState.velocity_mps(:) - moonStart.velocity_mps;
end
captureInitialVelocity = relativeVelocity;
capturePredictedPosition = relativePosition;
for iteration = 1:6
    captureTargetVelocity = captured_orbit_velocity(capturePredictedPosition, relativeVelocity, altitude_m, config);
    capturePredictedPosition = relativePosition + 0.5 * (relativeVelocity + captureTargetVelocity) * captureBurnDuration_s;
end
departureInitialVelocity = nan(3, 1);
departureTargetRelativeVelocity = nan(3, 1);
departureDirection = nan(3, 1);
if ~isempty(departureTargetVelocity_mps)
    targetRelativeDirection = departureTargetVelocity_mps(:) - departureMoonVelocity_mps(:);
    if norm(targetRelativeDirection) > 0
        targetRelativeDirection = targetRelativeDirection / norm(targetRelativeDirection);
    else
        targetRelativeDirection = [];
    end
else
    targetRelativeDirection = [];
end
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= duration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    moon = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
    currentAcceleration = relative_acceleration(currentTime_s, relativePosition, relativeVelocity, currentEpoch, moon);
    writeIndex = writeIndex + 1;
    time_s(writeIndex) = currentTime_s;
    epoch(writeIndex) = currentEpoch;
    position_m(writeIndex, :) = (moon.position_m + relativePosition)';
    velocity_mps(writeIndex, :) = (moon.velocity_mps + relativeVelocity)';
    acceleration_mps2(writeIndex, :) = (currentAcceleration)';
    moonPosition_m(writeIndex, :) = moon.position_m';
    moonDistance_m(writeIndex) = norm(relativePosition);
    earthDistance_m(writeIndex) = norm(moon.position_m + relativePosition);
    specificEnergyEarth_Jkg(writeIndex) = 0.5 * dot(velocity_mps(writeIndex, :)', velocity_mps(writeIndex, :)') - config.physics.muEarth_m3ps2 / max(earthDistance_m(writeIndex), 1);
    specificEnergyMoon_Jkg(writeIndex) = 0.5 * dot(relativeVelocity, relativeVelocity) - config.physics.muMoon_m3ps2 / max(moonDistance_m(writeIndex), 1);
    phase(writeIndex) = phase_name(currentTime_s);
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
trajectory = struct('time_s', time_s(1:writeIndex), 'epoch', epoch(1:writeIndex), 'position_m', position_m(1:writeIndex, :), 'velocity_mps', velocity_mps(1:writeIndex, :), 'acceleration_mps2', acceleration_mps2(1:writeIndex, :), 'moonPosition_m', moonPosition_m(1:writeIndex, :), 'moonDistance_m', moonDistance_m(1:writeIndex), 'earthDistance_m', earthDistance_m(1:writeIndex), 'specificEnergyEarth_Jkg', specificEnergyEarth_Jkg(1:writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(1:writeIndex), 'phase', phase(1:writeIndex), 'valid', valid(1:writeIndex), 'completed', writeIndex >= min(stepCount, ceil(duration_s / step_s)), 'collision', any(moonDistance_m(1:writeIndex) <= config.physics.moonRadius_m));

    function value = phase_name(timeValue_s)
        if timeValue_s < captureBurnDuration_s
            value = "Lunar capture burn";
        elseif timeValue_s >= departureBurnStart_s && departureBurnDuration_s > 0
            value = "Lunar departure burn";
        else
            value = "Lunar orbit";
        end
    end

    function acceleration = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentEpoch, moon)
        gravity = -config.physics.muMoon_m3ps2 * currentPosition / max(norm(currentPosition), 1) ^ 3;
        thrust = zeros(3, 1);
        if timeValue_s < captureBurnDuration_s && captureBurnDuration_s > 0
            thrust = smooth_velocity_derivative(captureInitialVelocity, captureTargetVelocity, timeValue_s, captureBurnDuration_s) - gravity;
        elseif timeValue_s >= departureBurnStart_s && departureBurnDuration_s > 0
            if any(isnan(departureInitialVelocity))
                departureInitialVelocity = currentVelocity;
                departureDirection = targetRelativeDirection;
                if isempty(departureDirection) || dot(departureDirection, currentPosition) <= 0
                    departureDirection = currentPosition / norm(currentPosition);
                end
                departureSpeed = norm(departureTargetVelocity_mps(:) - moon.velocity_mps);
                desiredSpeed = sqrt(departureSpeed ^ 2 + 2 * config.physics.muMoon_m3ps2 / max(norm(currentPosition), 1));
                departureTargetRelativeVelocity = departureDirection * desiredSpeed;
            end
            thrust = smooth_velocity_derivative(departureInitialVelocity, departureTargetRelativeVelocity, timeValue_s - departureBurnStart_s, departureBurnDuration_s) - gravity;
        end
        acceleration = gravity + thrust;
    end

    function derivative = smooth_velocity_derivative(initialVelocity, finalVelocity, elapsed_s, burnDuration_s)
        normalizedTime = min(1, max(0, elapsed_s / max(burnDuration_s, eps)));
        derivative = (6 * normalizedTime - 6 * normalizedTime ^ 2) * (finalVelocity - initialVelocity) / max(burnDuration_s, eps);
    end

    function [nextPosition, nextVelocity] = rk4_step(timeValue_s, currentPosition, currentVelocity, currentEpoch, dt)
        moon1 = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
        k1 = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentEpoch, moon1);
        moon2 = v2_get_celestial_state(currentEpoch + seconds(dt / 2), 'moon', config, 'analytical');
        k2 = relative_acceleration(timeValue_s + dt / 2, currentPosition + currentVelocity * dt / 2, currentVelocity + k1 * dt / 2, currentEpoch + seconds(dt / 2), moon2);
        k3 = relative_acceleration(timeValue_s + dt / 2, currentPosition + (currentVelocity + k1 * dt / 2) * dt / 2, currentVelocity + k2 * dt / 2, currentEpoch + seconds(dt / 2), moon2);
        moon4 = v2_get_celestial_state(currentEpoch + seconds(dt), 'moon', config, 'analytical');
        k4 = relative_acceleration(timeValue_s + dt, currentPosition + (currentVelocity + k2 * dt / 2) * dt, currentVelocity + k3 * dt, currentEpoch + seconds(dt), moon4);
        nextPosition = currentPosition + dt / 6 * (currentVelocity + 2 * (currentVelocity + k1 * dt / 2) + 2 * (currentVelocity + k2 * dt / 2) + (currentVelocity + k3 * dt));
        nextVelocity = currentVelocity + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
end

function velocity = captured_orbit_velocity(position, incomingVelocity, altitude_m, config)
radius = norm(position);
periapsisAltitude_m = max(altitude_m, config.mission.trajectoryLunarPeriapsisAltitude_m);
periapsisRadius = config.physics.moonRadius_m + periapsisAltitude_m;
radial = position / max(radius, 1);
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
if radius > periapsisRadius
    semiMajorAxis = (radius + periapsisRadius) / 2;
    speed = sqrt(config.physics.muMoon_m3ps2 * (2 / radius - 1 / semiMajorAxis));
else
    speed = sqrt(config.physics.muMoon_m3ps2 / max(radius, 1));
end
velocity = tangential * speed;
end
