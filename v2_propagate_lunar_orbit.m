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
step_s = orbit_lunar_step(config);
captureBurnDuration_s = min(config.mission.captureBurnDuration_s, duration_s / 3);
departureBurnDuration_s = min(config.mission.departureBurnDuration_s, duration_s / 3);
if captureBurnDuration_s + departureBurnDuration_s > duration_s
    departureBurnDuration_s = max(0, duration_s - captureBurnDuration_s);
end
nominalDuration_s = duration_s;
maximumDuration_s = duration_s + config.mission.departurePhasingWindow_s;
departureBurnStart_s = nan;
stepCount = ceil(maximumDuration_s / step_s) + 1;
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
    orbitNormal = [0; 0; 1];
    tangential = cross(orbitNormal, radial);
    tangential = tangential / norm(tangential);
    relativePosition = radial * (config.physics.moonRadius_m + altitude_m);
    relativeVelocity = tangential * sqrt(config.physics.muMoon_m3ps2 / norm(relativePosition));
else
    relativePosition = initialState.position_m(:) - moonStart.position_m;
    relativeVelocity = initialState.velocity_mps(:) - moonStart.velocity_mps;
    orbitNormal = cross(relativePosition, relativeVelocity);
    if norm(orbitNormal) < eps
        orbitNormal = [0; 0; 1];
    else
        orbitNormal = orbitNormal / norm(orbitNormal);
    end
end
captureDeltaV_mps = capture_orbit_velocity(relativePosition, relativeVelocity, orbitNormal, config) - relativeVelocity;
for iteration = 1:6
    [capturePosition, captureVelocity] = propagate_finite_burn(relativePosition, relativeVelocity, captureDeltaV_mps, captureBurnDuration_s, step_s, config);
    captureTargetVelocity = capture_orbit_velocity(capturePosition, captureVelocity, orbitNormal, config);
    correction_mps = captureTargetVelocity - captureVelocity;
    captureDeltaV_mps = captureDeltaV_mps + correction_mps;
    if norm(correction_mps) < 0.02
        break
    end
end
departureInitialVelocity = nan(3, 1);
departureDeltaV_mps = nan(3, 1);
departureTargetRelativeVelocity_mps = nan(3, 1);
writeIndex = 0;
currentTime_s = 0;
while currentTime_s <= maximumDuration_s + 1e-6 && writeIndex < stepCount
    if stopFunction()
        break
    end
    currentEpoch = startEpoch + seconds(currentTime_s);
    moon = v2_get_celestial_state(currentEpoch, 'moon', config, 'analytical');
    if isnan(departureBurnStart_s) && currentTime_s >= nominalDuration_s
        earthDirection = -moon.position_m / norm(moon.position_m);
        radialDirection = relativePosition / max(norm(relativePosition), 1);
        if dot(earthDirection, radialDirection) > cosd(20) || currentTime_s >= maximumDuration_s - departureBurnDuration_s
            departureBurnStart_s = currentTime_s;
        end
    end
    currentAcceleration = relative_acceleration(currentTime_s, relativePosition, relativeVelocity, moon.velocity_mps);
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
    phase(writeIndex) = phase_name(currentTime_s);
    valid(writeIndex) = all(isfinite([relativePosition; relativeVelocity; currentAcceleration]));
    state = struct('time_s', currentTime_s, 'epoch', currentEpoch, 'position_m', position_m(writeIndex, :)', 'velocity_mps', velocity_mps(writeIndex, :)', 'acceleration_mps2', currentAcceleration, 'moonPosition_m', moon.position_m, 'moonDistance_m', moonDistance_m(writeIndex), 'earthDistance_m', earthDistance_m(writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(writeIndex), 'phase', phase(writeIndex), 'valid', valid(writeIndex));
    if ~callback(state) || ~valid(writeIndex)
        break
    end
    if moonDistance_m(writeIndex) <= config.physics.moonRadius_m
        break
    end
    if ~isnan(departureBurnStart_s) && currentTime_s >= departureBurnStart_s + departureBurnDuration_s - 1e-6
        break
    end
    dt_s = min(step_s, maximumDuration_s - currentTime_s);
    if dt_s <= 0
        break
    end
    [relativePosition, relativeVelocity] = rk4_step(currentTime_s, relativePosition, relativeVelocity, moon.velocity_mps, dt_s);
    currentTime_s = currentTime_s + dt_s;
end
completed = writeIndex > 0 && ~isnan(departureBurnStart_s) && time_s(writeIndex) >= departureBurnStart_s + departureBurnDuration_s - 1e-6;
trajectory = struct('time_s', time_s(1:writeIndex), 'epoch', epoch(1:writeIndex), 'position_m', position_m(1:writeIndex, :), 'velocity_mps', velocity_mps(1:writeIndex, :), 'acceleration_mps2', acceleration_mps2(1:writeIndex, :), 'moonPosition_m', moonPosition_m(1:writeIndex, :), 'moonDistance_m', moonDistance_m(1:writeIndex), 'earthDistance_m', earthDistance_m(1:writeIndex), 'specificEnergyEarth_Jkg', specificEnergyEarth_Jkg(1:writeIndex), 'specificEnergyMoon_Jkg', specificEnergyMoon_Jkg(1:writeIndex), 'phase', phase(1:writeIndex), 'valid', valid(1:writeIndex), 'completed', completed, 'collision', any(moonDistance_m(1:writeIndex) <= config.physics.moonRadius_m), 'nominalDuration_s', nominalDuration_s, 'departureBurnStart_s', departureBurnStart_s, 'actualDuration_s', time_s(max(writeIndex, 1)), 'captureDeltaV_mps', captureDeltaV_mps, 'departureTargetRelativeVelocity_mps', departureTargetRelativeVelocity_mps);

    function value = phase_name(timeValue_s)
        if timeValue_s < captureBurnDuration_s
            value = "Lunar capture burn";
        elseif ~isnan(departureBurnStart_s) && timeValue_s >= departureBurnStart_s && departureBurnDuration_s > 0
            value = "Lunar departure burn";
        elseif timeValue_s >= nominalDuration_s
            value = "Lunar departure phasing";
        else
            value = "Lunar orbit";
        end
    end

    function acceleration = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentMoonVelocity_mps)
        gravity = moon_gravity(currentPosition, config);
        if timeValue_s < captureBurnDuration_s && captureBurnDuration_s > 0
            acceleration = gravity + burn_profile(timeValue_s, captureBurnDuration_s) * captureDeltaV_mps;
            return
        end
        if ~isnan(departureBurnStart_s) && timeValue_s >= departureBurnStart_s && departureBurnDuration_s > 0
            if any(isnan(departureInitialVelocity))
                departureInitialVelocity = currentVelocity;
                radialDirection = currentPosition / max(norm(currentPosition), 1);
                tangentialDirection = cross(orbitNormal, radialDirection);
                tangentialDirection = tangentialDirection / max(norm(tangentialDirection), 1);
                if dot(tangentialDirection, currentVelocity) < 0
                    tangentialDirection = -tangentialDirection;
                end
                if isempty(departureTargetVelocity_mps)
                    requestedRelativeVelocity_mps = currentVelocity;
                else
                    requestedRelativeVelocity_mps = departureTargetVelocity_mps(:) - currentMoonVelocity_mps;
                end
                if norm(requestedRelativeVelocity_mps) < eps
                    requestedDirection = tangentialDirection;
                else
                    requestedDirection = requestedRelativeVelocity_mps / norm(requestedRelativeVelocity_mps);
                end
                minimumEscapeSpeed_mps = sqrt(2 * config.physics.muMoon_m3ps2 / max(norm(currentPosition), 1)) * 1.06;
                departureTargetRelativeVelocity_mps = requestedDirection * max(norm(requestedRelativeVelocity_mps), minimumEscapeSpeed_mps);
                departureDeltaV_mps = departureTargetRelativeVelocity_mps - departureInitialVelocity;
            end
            acceleration = gravity + burn_profile(timeValue_s - departureBurnStart_s, departureBurnDuration_s) * departureDeltaV_mps;
            return
        end
        acceleration = gravity;
    end

    function [nextPosition, nextVelocity] = rk4_step(timeValue_s, currentPosition, currentVelocity, currentMoonVelocity_mps, dt_s)
        k1 = relative_acceleration(timeValue_s, currentPosition, currentVelocity, currentMoonVelocity_mps);
        k2 = relative_acceleration(timeValue_s + dt_s / 2, currentPosition + currentVelocity * dt_s / 2, currentVelocity + k1 * dt_s / 2, currentMoonVelocity_mps);
        k3 = relative_acceleration(timeValue_s + dt_s / 2, currentPosition + (currentVelocity + k1 * dt_s / 2) * dt_s / 2, currentVelocity + k2 * dt_s / 2, currentMoonVelocity_mps);
        k4 = relative_acceleration(timeValue_s + dt_s, currentPosition + (currentVelocity + k2 * dt_s / 2) * dt_s, currentVelocity + k3 * dt_s, currentMoonVelocity_mps);
        nextPosition = currentPosition + dt_s / 6 * (currentVelocity + 2 * (currentVelocity + k1 * dt_s / 2) + 2 * (currentVelocity + k2 * dt_s / 2) + (currentVelocity + k3 * dt_s));
        nextVelocity = currentVelocity + dt_s / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    end
end

function value = orbit_lunar_step(config)
if isfield(config.integrator, 'lunarStep_s')
    value = min(config.integrator.highFidelityStep_s, config.integrator.lunarStep_s);
else
    value = config.integrator.highFidelityStep_s;
end
end

function [finalPosition_m, finalVelocity_mps] = propagate_finite_burn(position_m, velocity_mps, deltaV_mps, duration_s, step_s, config)
finalPosition_m = position_m;
finalVelocity_mps = velocity_mps;
elapsed_s = 0;
while elapsed_s < duration_s - 1e-9
    dt_s = min(step_s, duration_s - elapsed_s);
    k1 = moon_gravity(finalPosition_m, config) + burn_profile(elapsed_s, duration_s) * deltaV_mps;
    k2 = moon_gravity(finalPosition_m + finalVelocity_mps * dt_s / 2, config) + burn_profile(elapsed_s + dt_s / 2, duration_s) * deltaV_mps;
    k3 = moon_gravity(finalPosition_m + (finalVelocity_mps + k1 * dt_s / 2) * dt_s / 2, config) + burn_profile(elapsed_s + dt_s / 2, duration_s) * deltaV_mps;
    k4 = moon_gravity(finalPosition_m + (finalVelocity_mps + k2 * dt_s / 2) * dt_s, config) + burn_profile(elapsed_s + dt_s, duration_s) * deltaV_mps;
    finalPosition_m = finalPosition_m + dt_s / 6 * (finalVelocity_mps + 2 * (finalVelocity_mps + k1 * dt_s / 2) + 2 * (finalVelocity_mps + k2 * dt_s / 2) + (finalVelocity_mps + k3 * dt_s));
    finalVelocity_mps = finalVelocity_mps + dt_s / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    elapsed_s = elapsed_s + dt_s;
end
end

function acceleration_mps2 = moon_gravity(position_m, config)
acceleration_mps2 = -config.physics.muMoon_m3ps2 * position_m / max(norm(position_m), 1) ^ 3;
end

function value = burn_profile(elapsed_s, duration_s)
if duration_s <= 0
    value = 0;
    return
end
normalizedTime = min(1, max(0, elapsed_s / duration_s));
value = 30 * normalizedTime * (1 - normalizedTime) ^ 4 / duration_s;
end

function velocity_mps = capture_orbit_velocity(position_m, referenceVelocity_mps, orbitNormal, config)
radialDirection = position_m / max(norm(position_m), 1);
tangentialDirection = cross(orbitNormal, radialDirection);
tangentialDirection = tangentialDirection / max(norm(tangentialDirection), 1);
if dot(tangentialDirection, referenceVelocity_mps) < 0
    tangentialDirection = -tangentialDirection;
end
velocity_mps = tangentialDirection * sqrt(config.physics.muMoon_m3ps2 / max(norm(position_m), 1));
end
