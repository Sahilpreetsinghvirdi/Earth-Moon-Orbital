function inputs = generate_earth_moon_inputs(config)
n = config.trialCount;
d = config.distributions;
inputs = struct;
inputs.dryMass_kg = bounded_normal(n, d.dryMass_kg);
inputs.propellantMass_kg = bounded_normal(n, d.propellantMass_kg);
inputs.burnRate_kgps = bounded_normal(n, d.burnRate_kgps);
inputs.specificImpulse_s = bounded_normal(n, d.specificImpulse_s);
inputs.engineEfficiency = bounded_normal(n, d.engineEfficiency);
inputs.dragCoefficient = bounded_normal(n, d.dragCoefficient);
inputs.referenceArea_m2 = bounded_normal(n, d.referenceArea_m2);
inputs.densityMultiplier = bounded_normal(n, d.densityMultiplier);
inputs.initialSpeed_mps = bounded_normal(n, d.initialSpeed_mps);
inputs.turnStart_s = bounded_normal(n, d.turnStart_s);
inputs.turnDuration_s = bounded_normal(n, d.turnDuration_s);
inputs.targetPitch_deg = bounded_normal(n, d.targetPitch_deg);
inputs.launchAzimuthCorrection_deg = bounded_normal(n, d.launchAzimuthCorrection_deg);
inputs.moonPhase_deg = bounded_normal(n, d.moonPhase_deg);
inputs.lunarInsertionDeltaV_mps = bounded_normal(n, d.lunarInsertionDeltaV_mps);
inputs.translunarInjectionDeltaV_mps = bounded_normal(n, d.translunarInjectionDeltaV_mps);
inputs.initialMass_kg = inputs.dryMass_kg + inputs.propellantMass_kg;
inputs.thrust_N = inputs.burnRate_kgps .* inputs.specificImpulse_s .* config.g0_mps2 .* inputs.engineEfficiency;
inputs.burnDuration_s = inputs.propellantMass_kg ./ inputs.burnRate_kgps;
inputs.ballisticCoefficient_kgpm2 = inputs.initialMass_kg ./ (inputs.dragCoefficient .* inputs.referenceArea_m2);
end

function values = bounded_normal(count, specification)
values = specification(1) + specification(2) .* randn(count, 1);
values = min(max(values, specification(3)), specification(4));
end
