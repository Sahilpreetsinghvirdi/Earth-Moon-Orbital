function [result, trajectory, completed] = simulate_earth_moon_trial(config, input, onStep, shouldStop, storeHistory)
if nargin < 3 || isempty(onStep)
    onStep = @(~) true;
end
if nargin < 4 || isempty(shouldStop)
    shouldStop = @() false;
end
if nargin < 5 || isempty(storeHistory)
    storeHistory = false;
end
maximumSteps = ceil(input.burnDuration_s / config.poweredTimeStep_s) + ceil((config.maxSimulationTime_s - input.burnDuration_s) / config.coastTimeStep_s) + 3;
if storeHistory
    capacity = maximumSteps;
else
    capacity = config.trajectoryOverviewPoints + 2;
end
timeHistory_s = nan(capacity, 1);
positionHistory_m = nan(capacity, 2);
velocityHistory_mps = nan(capacity, 2);
moonPositionHistory_m = nan(capacity, 2);
altitudeHistory_m = nan(capacity, 1);
moonDistanceHistory_m = nan(capacity, 1);
moonEnergyHistory_Jkg = nan(capacity, 1);
dynamicPressureHistory_Pa = nan(capacity, 1);
time_s = 0;
position_m = [config.earth.radius_m + 1; 0];
velocity_mps = input.initialSpeed_mps .* [cosd(input.launchAzimuthCorrection_deg); sind(input.launchAzimuthCorrection_deg)];
maximumAltitude_m = 0;
maximumVelocity_mps = norm(velocity_mps);
maximumDynamicPressure_Pa = 0;
minimumMoonDistance_m = inf;
minimumMoonRelativeVelocity_mps = inf;
earthEscape = false;
enteredMoonSOI = false;
lunarCapture = false;
lunarOrbit = false;
earthImpact = false;
lunarImpact = false;
lunarInsertionBurnApplied = false;
lunarInsertionDeltaVApplied_mps = 0;
lunarInsertionTime_s = nan;
lunarInsertionMoonDistance_m = nan;
translunarInjectionApplied = false;
translunarInjectionTime_s = nan;
captureDuration_s = 0;
maximumCaptureDuration_s = 0;
lunarPeriapses = 0;
previousMoonRadialVelocity_mps = nan;
sampleIndex = 0;
nextOverviewTime_s = 0;
completed = true;
state = make_state(time_s, position_m, velocity_mps);
record_state(state)
if ~onStep(state)
    completed = false;
end
while completed && time_s < config.maxSimulationTime_s && ~earthImpact && ~lunarImpact
    if shouldStop()
        completed = false;
        break
    end
    if time_s < input.burnDuration_s
        dt_s = min(config.poweredTimeStep_s, input.burnDuration_s - time_s);
    else
        dt_s = min(config.coastTimeStep_s, config.maxSimulationTime_s - time_s);
    end
    if dt_s <= 0
        break
    end
    previousTime_s = time_s;
    previousPosition_m = position_m;
    [previousMoonPosition_m, ~] = earth_moon_moon_state(config, previousTime_s, input.moonPhase_deg);
    [nextPosition_m, nextVelocity_mps] = rk4_step(time_s, position_m, velocity_mps, dt_s);
    time_s = time_s + dt_s;
    position_m = nextPosition_m;
    velocity_mps = nextVelocity_mps;
    state = make_state(time_s, position_m, velocity_mps);
    earthSegmentImpact = time_s > config.poweredTimeStep_s && segment_intersects_circle(previousPosition_m, position_m, [0; 0], config.earth.radius_m);
    lunarSegmentImpact = segment_intersects_circle(previousPosition_m - previousMoonPosition_m, position_m - state.moonPosition_m, [0; 0], config.moon.radius_m);
    if ~earthSegmentImpact && ~lunarSegmentImpact && ~translunarInjectionApplied && time_s >= input.burnDuration_s && state.altitude_m >= config.earth.atmosphereBoundary_m
        velocity_mps = velocity_mps + input.translunarInjectionDeltaV_mps * velocity_mps / max(norm(velocity_mps), 1);
        translunarInjectionApplied = true;
        translunarInjectionTime_s = time_s;
        state = make_state(time_s, position_m, velocity_mps);
    end
    if ~earthSegmentImpact && ~lunarSegmentImpact && state.moonDistance_m <= config.moon.sphereOfInfluence_m && ~lunarInsertionBurnApplied
        burnDirection = state.moonRelativeVelocity_mps / max(state.moonRelativeSpeed_mps, 1);
        velocity_mps = velocity_mps - input.lunarInsertionDeltaV_mps * burnDirection;
        lunarInsertionBurnApplied = true;
        lunarInsertionDeltaVApplied_mps = input.lunarInsertionDeltaV_mps;
        lunarInsertionTime_s = time_s;
        lunarInsertionMoonDistance_m = state.moonDistance_m;
        state = make_state(time_s, position_m, velocity_mps);
    end
    update_events(state, dt_s)
    if earthSegmentImpact
        earthImpact = true;
    end
    if lunarSegmentImpact
        lunarImpact = true;
    end
    shouldRecord = storeHistory || time_s + 1e-9 >= nextOverviewTime_s || earthImpact || lunarImpact || time_s >= config.maxSimulationTime_s;
    if shouldRecord
        record_state(state)
        while nextOverviewTime_s <= time_s
            nextOverviewTime_s = nextOverviewTime_s + config.maxSimulationTime_s / max(config.trajectoryOverviewPoints - 1, 1);
        end
    end
    if ~onStep(state)
        completed = false;
    end
end
if ~completed
    classification = 'Stopped';
elseif lunarImpact
    classification = 'Lunar impact';
elseif lunarOrbit
    classification = 'Lunar orbit';
elseif lunarCapture
    classification = 'Temporary lunar capture';
elseif enteredMoonSOI
    classification = 'Moon encounter';
elseif earthImpact
    classification = 'Earth impact';
elseif earthEscape
    classification = 'Earth escape';
else
    classification = 'No lunar encounter';
end
trajectory = trim_trajectory();
result = struct;
result.initialMass_kg = input.initialMass_kg;
result.burnDuration_s = input.burnDuration_s;
result.flightTime_s = time_s;
result.maximumAltitude_m = maximumAltitude_m;
result.maximumVelocity_mps = maximumVelocity_mps;
result.maximumDynamicPressure_Pa = maximumDynamicPressure_Pa;
result.minimumMoonDistance_m = minimumMoonDistance_m;
result.minimumMoonRelativeVelocity_mps = minimumMoonRelativeVelocity_mps;
result.earthEscape = earthEscape;
result.enteredMoonSOI = enteredMoonSOI;
result.moonEncounter = enteredMoonSOI;
result.lunarCapture = lunarCapture;
result.lunarOrbit = lunarOrbit;
result.earthImpact = earthImpact;
result.lunarImpact = lunarImpact;
result.lunarInsertionBurnApplied = lunarInsertionBurnApplied;
result.lunarInsertionDeltaVApplied_mps = lunarInsertionDeltaVApplied_mps;
result.lunarInsertionTime_s = lunarInsertionTime_s;
result.lunarInsertionMoonDistance_m = lunarInsertionMoonDistance_m;
result.translunarInjectionApplied = translunarInjectionApplied;
result.translunarInjectionDeltaVApplied_mps = input.translunarInjectionDeltaV_mps * double(translunarInjectionApplied);
result.translunarInjectionTime_s = translunarInjectionTime_s;
result.maximumCaptureDuration_s = maximumCaptureDuration_s;
result.lunarPeriapses = lunarPeriapses;
result.finalAltitude_m = state.altitude_m;
result.finalVelocity_mps = norm(state.velocity_mps);
result.finalMoonDistance_m = state.moonDistance_m;
result.finalMoonSpecificEnergy_Jkg = state.moonSpecificEnergy_Jkg;
result.classification = classification;

    function stateOutput = make_state(currentTime_s, currentPosition_m, currentVelocity_mps)
        [acceleration_mps2, diagnostics] = earth_moon_acceleration(config, currentTime_s, currentPosition_m, currentVelocity_mps, input);
        stateOutput = struct;
        stateOutput.time_s = currentTime_s;
        stateOutput.position_m = currentPosition_m;
        stateOutput.velocity_mps = currentVelocity_mps;
        stateOutput.acceleration_mps2 = acceleration_mps2;
        stateOutput.moonPosition_m = diagnostics.moonPosition_m;
        stateOutput.moonVelocity_mps = diagnostics.moonVelocity_mps;
        stateOutput.altitude_m = diagnostics.altitude_m;
        stateOutput.moonDistance_m = diagnostics.moonDistance_m;
        stateOutput.moonRelativePosition_m = diagnostics.moonRelativePosition_m;
        stateOutput.moonRelativeVelocity_mps = diagnostics.moonRelativeVelocity_mps;
        stateOutput.moonRelativeSpeed_mps = diagnostics.moonRelativeSpeed_mps;
        stateOutput.moonSpecificEnergy_Jkg = diagnostics.moonSpecificEnergy_Jkg;
        stateOutput.earthSpecificEnergy_Jkg = diagnostics.earthSpecificEnergy_Jkg;
        stateOutput.dynamicPressure_Pa = diagnostics.dynamicPressure_Pa;
        stateOutput.fuel_kg = diagnostics.fuel_kg;
        stateOutput.mass_kg = diagnostics.mass_kg;
        stateOutput.pitch_deg = diagnostics.pitch_deg;
    end

    function [nextPosition_m, nextVelocity_mps] = rk4_step(startTime_s, startPosition_m, startVelocity_mps, step_s)
        [k1Velocity_mps2, ~] = earth_moon_acceleration(config, startTime_s, startPosition_m, startVelocity_mps, input);
        [k2Velocity_mps2, ~] = earth_moon_acceleration(config, startTime_s + 0.5 * step_s, startPosition_m + 0.5 * step_s * startVelocity_mps, startVelocity_mps + 0.5 * step_s * k1Velocity_mps2, input);
        [k3Velocity_mps2, ~] = earth_moon_acceleration(config, startTime_s + 0.5 * step_s, startPosition_m + 0.5 * step_s * (startVelocity_mps + 0.5 * step_s * k1Velocity_mps2), startVelocity_mps + 0.5 * step_s * k2Velocity_mps2, input);
        [k4Velocity_mps2, ~] = earth_moon_acceleration(config, startTime_s + step_s, startPosition_m + step_s * (startVelocity_mps + 0.5 * step_s * k2Velocity_mps2), startVelocity_mps + step_s * k3Velocity_mps2, input);
        nextPosition_m = startPosition_m + step_s / 6 * (startVelocity_mps + 2 * (startVelocity_mps + 0.5 * step_s * k1Velocity_mps2) + 2 * (startVelocity_mps + 0.5 * step_s * k2Velocity_mps2) + (startVelocity_mps + step_s * k3Velocity_mps2));
        nextVelocity_mps = startVelocity_mps + step_s / 6 * (k1Velocity_mps2 + 2 * k2Velocity_mps2 + 2 * k3Velocity_mps2 + k4Velocity_mps2);
    end

    function update_events(currentState, elapsed_s)
        maximumAltitude_m = max(maximumAltitude_m, currentState.altitude_m);
        maximumVelocity_mps = max(maximumVelocity_mps, norm(currentState.velocity_mps));
        maximumDynamicPressure_Pa = max(maximumDynamicPressure_Pa, currentState.dynamicPressure_Pa);
        minimumMoonDistance_m = min(minimumMoonDistance_m, currentState.moonDistance_m);
        minimumMoonRelativeVelocity_mps = min(minimumMoonRelativeVelocity_mps, currentState.moonRelativeSpeed_mps);
        if currentState.earthSpecificEnergy_Jkg > 0 && norm(currentState.position_m) >= config.earth.escapeCheckRadius_m
            earthEscape = true;
        end
        if currentState.moonDistance_m <= config.moon.sphereOfInfluence_m
            enteredMoonSOI = true;
            if currentState.moonSpecificEnergy_Jkg < 0
                lunarCapture = true;
                captureDuration_s = captureDuration_s + elapsed_s;
                maximumCaptureDuration_s = max(maximumCaptureDuration_s, captureDuration_s);
                radialVelocity_mps = dot(currentState.moonRelativePosition_m, currentState.moonRelativeVelocity_mps) / max(currentState.moonDistance_m, 1);
                if isfinite(previousMoonRadialVelocity_mps) && previousMoonRadialVelocity_mps < 0 && radialVelocity_mps >= 0
                    lunarPeriapses = lunarPeriapses + 1;
                end
                previousMoonRadialVelocity_mps = radialVelocity_mps;
                if captureDuration_s >= config.mission.minimumLunarOrbitDuration_s && lunarPeriapses >= config.mission.minimumLunarPeriapses
                    lunarOrbit = true;
                end
            else
                captureDuration_s = 0;
                previousMoonRadialVelocity_mps = nan;
            end
        else
            captureDuration_s = 0;
            previousMoonRadialVelocity_mps = nan;
        end
        if currentState.time_s > config.poweredTimeStep_s && norm(currentState.position_m) <= config.earth.radius_m && dot(currentState.position_m, currentState.velocity_mps) < 0
            earthImpact = true;
        end
        if currentState.moonDistance_m <= config.moon.radius_m
            lunarImpact = true;
        end
    end

    function record_state(currentState)
        if storeHistory || sampleIndex == 0 || currentState.time_s + 1e-9 >= nextOverviewTime_s || earthImpact || lunarImpact
            sampleIndex = sampleIndex + 1;
            if sampleIndex > numel(timeHistory_s)
                timeHistory_s(end + 1, 1) = nan;
                positionHistory_m(end + 1, :) = [nan, nan];
                velocityHistory_mps(end + 1, :) = [nan, nan];
                moonPositionHistory_m(end + 1, :) = [nan, nan];
                altitudeHistory_m(end + 1, 1) = nan;
                moonDistanceHistory_m(end + 1, 1) = nan;
                moonEnergyHistory_Jkg(end + 1, 1) = nan;
                dynamicPressureHistory_Pa(end + 1, 1) = nan;
            end
            timeHistory_s(sampleIndex) = currentState.time_s;
            positionHistory_m(sampleIndex, :) = currentState.position_m';
            velocityHistory_mps(sampleIndex, :) = currentState.velocity_mps';
            moonPositionHistory_m(sampleIndex, :) = currentState.moonPosition_m';
            altitudeHistory_m(sampleIndex) = currentState.altitude_m;
            moonDistanceHistory_m(sampleIndex) = currentState.moonDistance_m;
            moonEnergyHistory_Jkg(sampleIndex) = currentState.moonSpecificEnergy_Jkg;
            dynamicPressureHistory_Pa(sampleIndex) = currentState.dynamicPressure_Pa;
        end
    end

    function output = trim_trajectory()
        output = struct;
        output.time_s = timeHistory_s(1:sampleIndex);
        output.position_m = positionHistory_m(1:sampleIndex, :);
        output.velocity_mps = velocityHistory_mps(1:sampleIndex, :);
        output.moonPosition_m = moonPositionHistory_m(1:sampleIndex, :);
        output.altitude_m = altitudeHistory_m(1:sampleIndex);
        output.moonDistance_m = moonDistanceHistory_m(1:sampleIndex);
        output.moonSpecificEnergy_Jkg = moonEnergyHistory_Jkg(1:sampleIndex);
        output.dynamicPressure_Pa = dynamicPressureHistory_Pa(1:sampleIndex);
    end

    function intersects = segment_intersects_circle(startPoint_m, endPoint_m, centerPoint_m, radius_m)
        segment_m = endPoint_m - startPoint_m;
        startRelative_m = startPoint_m - centerPoint_m;
        segmentLengthSquared_m2 = dot(segment_m, segment_m);
        if segmentLengthSquared_m2 <= eps
            closestPoint_m = startPoint_m;
        else
            fraction = min(max(-dot(startRelative_m, segment_m) / segmentLengthSquared_m2, 0), 1);
            closestPoint_m = startPoint_m + fraction * segment_m;
        end
        intersects = norm(closestPoint_m - centerPoint_m) <= radius_m;
    end
end
