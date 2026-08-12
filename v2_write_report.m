function reportPath = v2_write_report(config, optimization, trajectory)
outputDirectory = fullfile(fileparts(mfilename('fullpath')), config.output.directory);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
reportPath = fullfile(outputDirectory, config.output.reportFile);
fileIdentifier = fopen(reportPath, 'w');
if fileIdentifier < 0
    error('V2:ReportWriteFailure', 'Unable to write V2 mission report.')
end
cleanup = onCleanup(@() fclose(fileIdentifier));
fprintf(fileIdentifier, '%s\n', config.name);
fprintf(fileIdentifier, 'Version: %s\n', config.version);
fprintf(fileIdentifier, 'Optimization start: %s\n', char(optimization.startedAt));
fprintf(fileIdentifier, 'Optimization end: %s\n', char(optimization.endedAt));
fprintf(fileIdentifier, 'Stop reason: %s\n', optimization.stopReason);
fprintf(fileIdentifier, 'Ephemeris: %s\n', optimization.ephemerisSource);
fprintf(fileIdentifier, 'Parallel evaluation: %d\n\n', optimization.usedParallel);
fprintf(fileIdentifier, 'Candidates tested: %d\n', optimization.candidatesTested);
fprintf(fileIdentifier, 'Successful candidates: %d\n', optimization.successfulCandidates);
fprintf(fileIdentifier, 'Iterations: %d\n', optimization.iterations);
if isempty(optimization.bestResult)
    fprintf(fileIdentifier, 'No valid mission candidate was found.\n');
    return
end
best = optimization.bestResult;
fprintf(fileIdentifier, '\nBest mission\n');
fprintf(fileIdentifier, 'Launch date: %s\n', char(best.candidate.launchEpoch));
fprintf(fileIdentifier, 'Moon arrival: %s\n', char(best.arrivalEpoch));
fprintf(fileIdentifier, 'Lunar orbit: %s for %.3f days\n', char(best.departureEpoch), best.candidate.lunarOrbitDuration_days);
fprintf(fileIdentifier, 'Earth return: %s\n', char(best.returnEpoch));
fprintf(fileIdentifier, 'Mission score: %.8f\n', best.score);
fprintf(fileIdentifier, 'Total delta-v: %.3f m/s\n', best.totalDeltaV_mps);
fprintf(fileIdentifier, 'Estimated fuel: %.3f kg\n', best.fuel.requiredPropellant_kg);
fprintf(fileIdentifier, 'Propellant fraction: %.6f\n', best.fuel.propellantFraction);
fprintf(fileIdentifier, 'Final mass: %.3f kg\n', best.fuel.finalMass_kg);
fprintf(fileIdentifier, 'Mass ratio: %.6f\n', best.fuel.massRatio);
fprintf(fileIdentifier, 'Departure delta-v: %.3f m/s\n', best.departureDeltaV_mps);
fprintf(fileIdentifier, 'Lunar insertion delta-v: %.3f m/s\n', best.lunarOrbitInsertionDeltaV_mps);
fprintf(fileIdentifier, 'Lunar departure delta-v: %.3f m/s\n', best.lunarDepartureDeltaV_mps);
fprintf(fileIdentifier, 'Earth capture delta-v: %.3f m/s\n', best.earthCaptureDeltaV_mps);
fprintf(fileIdentifier, 'Earth arrival speed: %.3f m/s\n', best.earthArrivalSpeed_mps);
fprintf(fileIdentifier, 'Lunar periapsis altitude: %.3f km\n', best.candidate.lunarOrbitAltitude_m / 1000);
fprintf(fileIdentifier, 'Trajectory points: %d\n', numel(trajectory.time_s));
fprintf(fileIdentifier, 'Trajectory completed: %d\n', trajectory.completed);
fprintf(fileIdentifier, 'Trajectory collision: %d\n', trajectory.collision);
fprintf(fileIdentifier, 'Lunar SOI entered: %d\n', trajectory.lunarSOIEntered);
fprintf(fileIdentifier, 'Minimum lunar distance: %.3f km\n', trajectory.lunarEncounterDistance_m / 1000);
fprintf(fileIdentifier, 'Earth arrival distance: %.3f km\n', trajectory.earthArrivalDistance_m / 1000);
fprintf(fileIdentifier, 'Earth arrival safe within SOI: %d\n', trajectory.earthArrivalSafe);
fprintf(fileIdentifier, 'Phase-boundary position jumps: %.3f km, %.3f km\n', trajectory.phaseBoundaryJumps_m(1) / 1000, trajectory.phaseBoundaryJumps_m(2) / 1000);
fprintf(fileIdentifier, 'Capture burn duration: %.1f s\n', config.mission.captureBurnDuration_s);
fprintf(fileIdentifier, 'Departure burn duration: %.1f s\n', config.mission.departureBurnDuration_s);
end
