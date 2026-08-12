function [acceleration_mps2, diagnostics] = earth_moon_acceleration(config, time_s, position_m, velocity_mps, input)
distanceEarth_m = norm(position_m);
altitude_m = distanceEarth_m - config.earth.radius_m;
[moonPosition_m, moonVelocity_mps] = earth_moon_moon_state(config, time_s, input.moonPhase_deg);
moonRelativePosition_m = position_m - moonPosition_m;
moonDistance_m = norm(moonRelativePosition_m);
moonRelativeVelocity_mps = velocity_mps - moonVelocity_mps;
fuel_kg = max(0, input.propellantMass_kg - input.burnRate_kgps * time_s);
mass_kg = input.dryMass_kg + fuel_kg;
earthGravity_mps2 = -config.earth.mu_m3ps2 .* position_m ./ max(distanceEarth_m, 1) ^ 3;
moonGravity_mps2 = -config.moon.mu_m3ps2 .* moonRelativePosition_m ./ max(moonDistance_m, 1) ^ 3;
if altitude_m <= config.earth.atmosphereBoundary_m
    density_kgpm3 = config.earth.referenceDensity_kgpm3 * input.densityMultiplier * exp(-max(altitude_m, 0) / config.earth.scaleHeight_m);
else
    density_kgpm3 = 0;
end
speed_mps = norm(velocity_mps);
drag_mps2 = -0.5 * input.dragCoefficient * input.referenceArea_m2 * density_kgpm3 * speed_mps .* velocity_mps ./ mass_kg;
thrust_mps2 = [0; 0];
if fuel_kg > 0
    radialDirection = position_m ./ max(distanceEarth_m, 1);
    tangentialDirection = [-radialDirection(2); radialDirection(1)];
    turnProgress = min(max((time_s - input.turnStart_s) / input.turnDuration_s, 0), 1);
    smoothProgress = turnProgress ^ 2 * (3 - 2 * turnProgress);
    pitch_deg = smoothProgress * input.targetPitch_deg + input.launchAzimuthCorrection_deg;
    thrustDirection = cosd(pitch_deg) .* radialDirection + sind(pitch_deg) .* tangentialDirection;
    thrust_mps2 = input.thrust_N .* thrustDirection ./ mass_kg;
else
    pitch_deg = input.targetPitch_deg + input.launchAzimuthCorrection_deg;
end
acceleration_mps2 = earthGravity_mps2 + moonGravity_mps2 + drag_mps2 + thrust_mps2;
diagnostics = struct;
diagnostics.moonPosition_m = moonPosition_m;
diagnostics.moonVelocity_mps = moonVelocity_mps;
diagnostics.altitude_m = altitude_m;
diagnostics.distanceEarth_m = distanceEarth_m;
diagnostics.moonDistance_m = moonDistance_m;
diagnostics.moonRelativePosition_m = moonRelativePosition_m;
diagnostics.moonRelativeVelocity_mps = moonRelativeVelocity_mps;
diagnostics.moonRelativeSpeed_mps = norm(moonRelativeVelocity_mps);
diagnostics.moonSpecificEnergy_Jkg = 0.5 * diagnostics.moonRelativeSpeed_mps ^ 2 - config.moon.mu_m3ps2 / max(moonDistance_m, 1);
diagnostics.earthSpecificEnergy_Jkg = 0.5 * speed_mps ^ 2 - config.earth.mu_m3ps2 / max(distanceEarth_m, 1);
diagnostics.density_kgpm3 = density_kgpm3;
diagnostics.dynamicPressure_Pa = 0.5 * density_kgpm3 * speed_mps ^ 2;
diagnostics.fuel_kg = fuel_kg;
diagnostics.mass_kg = mass_kg;
diagnostics.pitch_deg = pitch_deg;
diagnostics.thrust_mps2 = thrust_mps2;
diagnostics.drag_mps2 = drag_mps2;
end
