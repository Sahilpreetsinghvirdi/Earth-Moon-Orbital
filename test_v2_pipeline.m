function validation = test_v2_pipeline()
baseDirectory = fileparts(mfilename('fullpath'));
config = v2_config();
config.searchEndEpoch = config.searchStartEpoch + days(10);
config.mission.maxCandidates = 9;
config.optimizer.maxIterations = 9;
config.mission.maxRuntime_s = 120;
config.optimizer.showProgress = false;
config.optimizer.useParallel = false;
config.output.directory = 'results_v2_test';
moon0 = v2_get_celestial_state(config.epoch, 'moon', config, 'analytical');
moon1 = v2_get_celestial_state(config.epoch + days(1), 'moon', config, 'analytical');
assert(norm(moon0.position_m) > 3.0e8 && norm(moon0.position_m) < 4.2e8)
assert(norm(moon1.position_m - moon0.position_m) > 1.0e6)
assert(norm(moon0.velocity_mps) > 700 && norm(moon0.velocity_mps) < 1400)
lambert = v2_solve_lambert([7000000; 0; 0], [0; 384400000; 0], 4 * 86400, config.physics.muEarth_m3ps2, true);
assert(lambert.valid)
assert(lambert.residual_s < 1)
fuel = v2_calculate_fuel(10000, config.vehicle, config);
assert(abs(fuel.rocketEquationResidual_mps) < 1e-8)
candidate = struct('launchEpoch', config.searchStartEpoch, 'outboundFlightTime_days', 3, 'returnFlightTime_days', 3, 'lunarApproachDuration_days', 1, 'lunarOrbitDuration_days', 1, 'lunarOrbitAltitude_m', 100000);
candidateResult = v2_evaluate_candidate(candidate, config);
assert(candidateResult.valid)
assert(candidateResult.constraints.moonApproachValid)
assert(candidateResult.constraints.moonDepartureOutwardValid)
assert(candidateResult.moonArrivalState.epoch > candidateResult.moonLaunchState.epoch)
assert(candidateResult.insertionEpoch > candidateResult.arrivalEpoch)
assert(candidateResult.returnEpoch > candidateResult.arrivalEpoch)
invalidCandidate = candidate;
invalidCandidate.lunarOrbitAltitude_m = 1000;
invalidResult = v2_evaluate_candidate(invalidCandidate, config);
assert(~invalidResult.valid)
optimization = v2_optimize_mission(config, []);
assert(~isempty(optimization.bestResult))
trajectory = v2_build_mission_trajectory(optimization.bestResult, config);
assert(trajectory.completed)
assert(~trajectory.collision)
assert(trajectory.lunarSOIEntered)
assert(trajectory.earthArrivalSafe)
assert(max(trajectory.phaseBoundaryJumps_m) < 1)
assert(any(strcmp(trajectory.phase, 'Lunar capture burn')))
assert(any(strcmp(trajectory.phase, 'Lunar departure burn')))
assert(trajectory.lunarOrbitValid)
assert(trajectory.lunarOrbitMinimumDistance_m < config.physics.moonRadius_m + 5e5)
orbitMask = strcmp(trajectory.phase, 'Lunar orbit');
assert(max(trajectory.moonDistance_m(orbitMask)) - min(trajectory.moonDistance_m(orbitMask)) < 2e4)
departureIndex = find(strcmp(trajectory.phase, 'Lunar departure burn'), 1, 'first');
departurePosition_m = trajectory.position_m(departureIndex, :)' - trajectory.moonPosition_m(departureIndex, :)';
departureVelocity_mps = trajectory.velocity_mps(departureIndex, :)' - v2_get_celestial_state(trajectory.epoch(departureIndex), 'moon', config, 'analytical').velocity_mps;
assert(abs(dot(departurePosition_m, departureVelocity_mps)) / norm(departurePosition_m) < 100)
assert(any(strcmp(trajectory.phase, 'Earth departure and lunar transfer burn')))
assert(abs(norm(trajectory.velocity_mps(1, :)) - norm(candidateResult.earthParkingVelocity_mps)) < 1e-6)
outboundMask = startsWith(trajectory.phase, "Earth departure") | startsWith(trajectory.phase, "Lunar approach");
assert(max(trajectory.earthDistance_m(outboundMask)) > 3e8)
assert(min(trajectory.moonDistance_m(outboundMask)) < config.physics.moonSOI_m)
statePath = v2_save_mission_state(config, optimization, trajectory);
assert(exist(statePath, 'file') == 2)
[loadedState, ~] = v2_load_mission_state(config);
assert(~isempty(loadedState))
assert(loadedState.recalculatedFromEphemeris)
validation = struct('passed', true, 'moonDistance_km', norm(moon0.position_m) / 1000, 'moonMotion_km', norm(moon1.position_m - moon0.position_m) / 1000, 'lambertResidual_s', lambert.residual_s, 'bestLaunchDate', optimization.bestLaunchDate, 'bestDeltaV_mps', optimization.bestDeltaV_mps, 'bestFuel_kg', optimization.bestFuel_kg, 'trajectoryPoints', numel(trajectory.time_s), 'statePath', statePath, 'baseDirectory', baseDirectory);
fprintf('V2 validation passed: Moon %.3f km, motion %.3f km/day, Lambert residual %.6g s, best %.3f m/s, %.3f kg fuel\n', validation.moonDistance_km, validation.moonMotion_km, validation.lambertResidual_s, validation.bestDeltaV_mps, validation.bestFuel_kg)
end
