function inputs = generate_rocket_inputs(config)
n = config.launchCount;
d = config.distributions;
inputs = struct;
inputs.launchAngle_deg = bounded_normal(n, d.launchAngle_deg);
inputs.initialMass_kg = bounded_normal(n, d.initialMass_kg);
inputs.propellantMass_kg = bounded_normal(n, d.propellantMass_kg);
maximumPropellant = inputs.initialMass_kg - 28.0;
inputs.propellantMass_kg = min(inputs.propellantMass_kg, maximumPropellant);
inputs.propellantMass_kg = max(inputs.propellantMass_kg, 8.0);
inputs.dryMass_kg = inputs.initialMass_kg - inputs.propellantMass_kg;
inputs.burnRate_kgps = bounded_normal(n, d.burnRate_kgps);
inputs.specificImpulse_s = bounded_normal(n, d.specificImpulse_s);
inputs.engineEfficiency = bounded_normal(n, d.engineEfficiency);
inputs.thrust_N = inputs.burnRate_kgps .* inputs.specificImpulse_s .* config.g0_mps2 .* inputs.engineEfficiency;
inputs.dragCoefficient = bounded_normal(n, d.dragCoefficient);
inputs.referenceArea_m2 = bounded_normal(n, d.referenceArea_m2);
inputs.densityMultiplier = bounded_normal(n, d.densityMultiplier);
inputs.scaleHeight_m = bounded_normal(n, d.scaleHeight_m);
inputs.temperature_K = bounded_normal(n, d.temperature_K);
inputs.windSpeed_mps = bounded_normal(n, d.windSpeed_mps);
inputs.burnDuration_s = inputs.propellantMass_kg ./ inputs.burnRate_kgps;
inputs.ballisticCoefficient_kgpm2 = inputs.initialMass_kg ./ (inputs.dragCoefficient .* inputs.referenceArea_m2);
end

function values = bounded_normal(n, specification)
values = specification(1) + specification(2) .* randn(n, 1);
values = min(max(values, specification(3)), specification(4));
end
