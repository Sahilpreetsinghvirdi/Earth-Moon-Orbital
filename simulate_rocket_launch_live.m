function [result, trajectory, completed] = simulate_rocket_launch_live(config, input, onStep, shouldStop)
if nargin < 3 || isempty(onStep)
    onStep = @(~, ~) true;
end
if nargin < 4 || isempty(shouldStop)
    shouldStop = @() false;
end
dt = config.timeStep_s;
maxSteps = ceil(config.maxFlightTime_s / dt);
g0 = config.g0_mps2;
x_m = 0;
y_m = config.initialAltitude_m;
vx_mps = 0;
vy_mps = 0;
fuel_kg = input.propellantMass_kg;
apogee_m = y_m;
maximumVelocity_mps = 0;
maximumDownrange_m = x_m;
maximumAcceleration_mps2 = 0;
maximumDynamicPressure_Pa = 0;
burnoutTime_s = nan;
impactOccurred = false;
numericalFailure = false;
timedOut = false;
impactTime_s = nan;
impactDownrange_m = nan;
impactVelocityX_mps = nan;
impactVelocityY_mps = nan;
impactSpeed_mps = nan;
impactFlightPathAngle_deg = nan;
impactDynamicPressure_Pa = nan;
finalX_m = nan;
finalY_m = nan;
finalVx_mps = nan;
finalVy_mps = nan;
timeHistory_s = nan(maxSteps + 1, 1);
downrangeHistory_m = nan(maxSteps + 1, 1);
altitudeHistory_m = nan(maxSteps + 1, 1);
velocityHistory_mps = nan(maxSteps + 1, 1);
accelerationHistory_mps2 = nan(maxSteps + 1, 1);
dynamicPressureHistory_Pa = nan(maxSteps + 1, 1);
fuelHistory_kg = nan(maxSteps + 1, 1);
massHistory_kg = nan(maxSteps + 1, 1);
flightPathAngleHistory_deg = nan(maxSteps + 1, 1);
statusHistory = strings(maxSteps + 1, 1);
timeHistory_s(1) = 0;
downrangeHistory_m(1) = x_m;
altitudeHistory_m(1) = y_m;
velocityHistory_mps(1) = 0;
accelerationHistory_mps2(1) = 0;
dynamicPressureHistory_Pa(1) = 0;
fuelHistory_kg(1) = fuel_kg;
massHistory_kg(1) = input.dryMass_kg + fuel_kg;
flightPathAngleHistory_deg(1) = input.launchAngle_deg;
statusHistory(1) = "LAUNCH";
initialState = make_state(0, x_m, y_m, vx_mps, vy_mps, 0, 0, 0, fuel_kg, massHistory_kg(1), input.launchAngle_deg, apogee_m, maximumDownrange_m, maximumVelocity_mps, maximumAcceleration_mps2, maximumDynamicPressure_Pa, "LAUNCH", input);
if ~onStep(initialState, "LAUNCH")
    completed = false;
    trajectory = trim_trajectory();
    result = make_result();
    return
end
completed = true;
for step = 1:maxSteps
    if shouldStop()
        completed = false;
        break
    end
    t_s = (step - 1) * dt;
    altitudeForModel_m = max(y_m, 0);
    mass_kg = input.dryMass_kg + fuel_kg;
    rho_kgpm3 = config.referenceDensity_kgpm3 * input.densityMultiplier * (config.nominalTemperature_K / input.temperature_K) * exp(-altitudeForModel_m / input.scaleHeight_m);
    airVx_mps = vx_mps - input.windSpeed_mps;
    airVy_mps = vy_mps;
    airSpeed_mps = hypot(airVx_mps, airVy_mps);
    dynamicPressure_Pa = 0.5 * rho_kgpm3 * airSpeed_mps ^ 2;
    dragFactor_Nspm = dynamicPressure_Pa * input.dragCoefficient * input.referenceArea_m2 / max(airSpeed_mps, 1e-9);
    dragX_N = -dragFactor_Nspm * airVx_mps;
    dragY_N = -dragFactor_Nspm * airVy_mps;
    poweredFraction = double(fuel_kg > 0) * min(1, fuel_kg / (input.burnRate_kgps * dt));
    thrustX_N = input.thrust_N * cosd(input.launchAngle_deg);
    thrustY_N = input.thrust_N * sind(input.launchAngle_deg);
    accelerationX_mps2 = (thrustX_N * poweredFraction + dragX_N) / mass_kg;
    gravity_mps2 = g0 * (config.earthRadius_m / (config.earthRadius_m + altitudeForModel_m)) ^ 2;
    accelerationY_mps2 = (thrustY_N * poweredFraction + dragY_N) / mass_kg - gravity_mps2;
    accelerationMagnitude_mps2 = hypot(accelerationX_mps2, accelerationY_mps2);
    speed_mps = hypot(vx_mps, vy_mps);
    maximumDynamicPressure_Pa = max(maximumDynamicPressure_Pa, dynamicPressure_Pa);
    maximumAcceleration_mps2 = max(maximumAcceleration_mps2, accelerationMagnitude_mps2);
    maximumVelocity_mps = max(maximumVelocity_mps, speed_mps);
    xNext_m = x_m + vx_mps * dt + 0.5 * accelerationX_mps2 * dt ^ 2;
    yNext_m = y_m + vy_mps * dt + 0.5 * accelerationY_mps2 * dt ^ 2;
    vxNext_mps = vx_mps + accelerationX_mps2 * dt;
    vyNext_mps = vy_mps + accelerationY_mps2 * dt;
    fuelNext_kg = max(0, fuel_kg - input.burnRate_kgps * poweredFraction * dt);
    if ~isfinite(xNext_m) || ~isfinite(yNext_m) || ~isfinite(vxNext_mps) || ~isfinite(vyNext_mps) || ~isfinite(fuelNext_kg)
        numericalFailure = true;
        completed = false;
        finalX_m = x_m;
        finalY_m = y_m;
        finalVx_mps = vx_mps;
        finalVy_mps = vy_mps;
        break
    end
    if isnan(burnoutTime_s) && fuel_kg > 0 && fuelNext_kg <= 0
        burnoutTime_s = t_s + dt * fuel_kg / input.burnRate_kgps;
    end
    potentialImpact = y_m > 0 && yNext_m <= 0 && vyNext_mps < 0;
    status = "FLIGHT";
    if potentialImpact
        interpolation = min(max(y_m / (y_m - yNext_m), 0), 1);
        impactOccurred = true;
        impactTime_s = t_s + interpolation * dt;
        impactDownrange_m = x_m + interpolation * (xNext_m - x_m);
        impactVelocityX_mps = vx_mps + interpolation * (vxNext_mps - vx_mps);
        impactVelocityY_mps = vy_mps + interpolation * (vyNext_mps - vy_mps);
        impactSpeed_mps = hypot(impactVelocityX_mps, impactVelocityY_mps);
        impactFlightPathAngle_deg = atan2d(impactVelocityY_mps, impactVelocityX_mps);
        impactAirVx_mps = impactVelocityX_mps - input.windSpeed_mps;
        impactAirSpeed_mps = hypot(impactAirVx_mps, impactVelocityY_mps);
        impactDynamicPressure_Pa = 0.5 * config.referenceDensity_kgpm3 * input.densityMultiplier * (config.nominalTemperature_K / input.temperature_K) * impactAirSpeed_mps ^ 2;
        maximumDynamicPressure_Pa = max(maximumDynamicPressure_Pa, impactDynamicPressure_Pa);
        maximumVelocity_mps = max(maximumVelocity_mps, impactSpeed_mps);
        xNext_m = impactDownrange_m;
        yNext_m = 0;
        vxNext_mps = impactVelocityX_mps;
        vyNext_mps = impactVelocityY_mps;
        status = "IMPACT";
    elseif yNext_m < apogee_m && vyNext_mps < 0
        status = "APOGEE";
    end
    nextTime_s = t_s + dt;
    nextSpeed_mps = hypot(vxNext_mps, vyNext_mps);
    nextFlightPathAngle_deg = atan2d(vyNext_mps, vxNext_mps);
    apogee_m = max(apogee_m, yNext_m);
    maximumDownrange_m = max(maximumDownrange_m, xNext_m);
    timeHistory_s(step + 1) = nextTime_s;
    downrangeHistory_m(step + 1) = xNext_m;
    altitudeHistory_m(step + 1) = yNext_m;
    velocityHistory_mps(step + 1) = nextSpeed_mps;
    accelerationHistory_mps2(step + 1) = accelerationMagnitude_mps2;
    dynamicPressureHistory_Pa(step + 1) = dynamicPressure_Pa;
    fuelHistory_kg(step + 1) = fuelNext_kg;
    massHistory_kg(step + 1) = input.dryMass_kg + fuelNext_kg;
    flightPathAngleHistory_deg(step + 1) = nextFlightPathAngle_deg;
    statusHistory(step + 1) = status;
    currentState = make_state(nextTime_s, xNext_m, yNext_m, vxNext_mps, vyNext_mps, nextSpeed_mps, accelerationMagnitude_mps2, dynamicPressure_Pa, fuelNext_kg, massHistory_kg(step + 1), nextFlightPathAngle_deg, apogee_m, maximumDownrange_m, maximumVelocity_mps, maximumAcceleration_mps2, maximumDynamicPressure_Pa, status, input);
    if ~onStep(currentState, status)
        completed = false;
        break
    end
    x_m = xNext_m;
    y_m = yNext_m;
    vx_mps = vxNext_mps;
    vy_mps = vyNext_mps;
    fuel_kg = fuelNext_kg;
    if impactOccurred
        finalX_m = impactDownrange_m;
        finalY_m = 0;
        finalVx_mps = impactVelocityX_mps;
        finalVy_mps = impactVelocityY_mps;
        break
    end
end
if ~impactOccurred && ~numericalFailure && completed
    timedOut = true;
    finalX_m = x_m;
    finalY_m = y_m;
    finalVx_mps = vx_mps;
    finalVy_mps = vy_mps;
end
trajectory = trim_trajectory();
result = make_result();

    function output = trim_trajectory()
        validRows = find(isfinite(timeHistory_s), 1, 'last');
        if isempty(validRows)
            validRows = 1;
        end
        output = struct;
        output.time_s = timeHistory_s(1:validRows);
        output.downrange_m = downrangeHistory_m(1:validRows);
        output.altitude_m = altitudeHistory_m(1:validRows);
        output.velocity_mps = velocityHistory_mps(1:validRows);
        output.acceleration_mps2 = accelerationHistory_mps2(1:validRows);
        output.dynamicPressure_Pa = dynamicPressureHistory_Pa(1:validRows);
        output.fuel_kg = fuelHistory_kg(1:validRows);
        output.mass_kg = massHistory_kg(1:validRows);
        output.flightPathAngle_deg = flightPathAngleHistory_deg(1:validRows);
        output.status = statusHistory(1:validRows);
    end

    function output = make_result()
        output = struct;
        output.apogee_m = apogee_m;
        output.maximumVelocity_mps = maximumVelocity_mps;
        output.maximumAcceleration_mps2 = maximumAcceleration_mps2;
        output.maximumDynamicPressure_Pa = maximumDynamicPressure_Pa;
        output.fuelConsumed_kg = input.propellantMass_kg - fuel_kg;
        output.remainingFuel_kg = fuel_kg;
        output.burnoutTime_s = burnoutTime_s;
        output.impactOccurred = impactOccurred;
        output.numericalFailure = numericalFailure;
        output.timedOut = timedOut;
        output.flightTime_s = impactTime_s;
        output.downrange_m = impactDownrange_m;
        if impactOccurred
            output.impactAltitude_m = 0;
        else
            output.impactAltitude_m = nan;
        end
        output.impactVelocityX_mps = impactVelocityX_mps;
        output.impactVelocityY_mps = impactVelocityY_mps;
        output.impactSpeed_mps = impactSpeed_mps;
        output.impactFlightPathAngle_deg = impactFlightPathAngle_deg;
        output.impactDynamicPressure_Pa = impactDynamicPressure_Pa;
        output.finalX_m = finalX_m;
        output.finalY_m = finalY_m;
        output.finalVelocity_mps = hypot(finalVx_mps, finalVy_mps);
        output.finalVx_mps = finalVx_mps;
        output.finalVy_mps = finalVy_mps;
    end
end

function state = make_state(time_s, x_m, y_m, vx_mps, vy_mps, speed_mps, acceleration_mps2, dynamicPressure_Pa, fuel_kg, mass_kg, flightPathAngle_deg, apogee_m, maximumDownrange_m, maximumVelocity_mps, maximumAcceleration_mps2, maximumDynamicPressure_Pa, status, input)
state = struct('time_s', time_s, 'downrange_m', x_m, 'altitude_m', y_m, 'velocityX_mps', vx_mps, 'velocityY_mps', vy_mps, 'velocity_mps', speed_mps, 'acceleration_mps2', acceleration_mps2, 'dynamicPressure_Pa', dynamicPressure_Pa, 'fuel_kg', fuel_kg, 'mass_kg', mass_kg, 'flightPathAngle_deg', flightPathAngle_deg, 'apogee_m', apogee_m, 'maximumDownrange_m', maximumDownrange_m, 'maximumVelocity_mps', maximumVelocity_mps, 'maximumAcceleration_mps2', maximumAcceleration_mps2, 'maximumDynamicPressure_Pa', maximumDynamicPressure_Pa, 'status', char(status), 'parameters', input);
end
