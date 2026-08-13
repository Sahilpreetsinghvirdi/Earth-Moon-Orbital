function fuel = v2_calculate_fuel(totalDeltaV_mps, vehicle, config)
g0 = config.physics.g0_mps2;
massRatio = exp(max(totalDeltaV_mps, 0) / (vehicle.engineIsp_s * g0));
requiredPropellant_kg = vehicle.dryMass_kg * (massRatio - 1);
finalMass_kg = vehicle.dryMass_kg + requiredPropellant_kg;
fuel = struct;
fuel.totalDeltaV_mps = totalDeltaV_mps;
fuel.massRatio = massRatio;
fuel.requiredPropellant_kg = requiredPropellant_kg;
fuel.finalMass_kg = finalMass_kg;
fuel.propellantFraction = requiredPropellant_kg / max(finalMass_kg, eps);
fuel.withinVehicleCapacity = requiredPropellant_kg <= vehicle.maxPropellant_kg;
fuel.rocketEquationResidual_mps = vehicle.engineIsp_s * g0 * log(finalMass_kg / vehicle.dryMass_kg) - totalDeltaV_mps;
end
