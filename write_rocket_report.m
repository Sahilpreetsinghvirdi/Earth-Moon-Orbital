function reportPath = write_rocket_report(outputDirectory, config, inputs, flightResults, analysis, resultsFileName)
reportPath = fullfile(outputDirectory, 'engineering_summary_report.txt');
fileIdentifier = fopen(reportPath, 'w');
if fileIdentifier < 0
    error('RocketMonteCarlo:ReportWriteFailure', 'Unable to create the engineering summary report.')
end
cleanup = onCleanup(@() fclose(fileIdentifier));
fprintf(fileIdentifier, '%s\n', config.simulationName);
fprintf(fileIdentifier, 'Generated: %s\n\n', char(datetime('now')));
fprintf(fileIdentifier, 'Simulation configuration\n');
fprintf(fileIdentifier, 'Launches: %d\n', config.launchCount);
fprintf(fileIdentifier, 'Random seed: %d\n', config.randomSeed);
fprintf(fileIdentifier, 'Integrator time step: %.3f s\n', config.timeStep_s);
fprintf(fileIdentifier, 'Maximum modeled flight time: %.1f s\n', config.maxFlightTime_s);
fprintf(fileIdentifier, 'Trajectory history sample count: %d\n\n', config.trajectorySampleCount);
fprintf(fileIdentifier, 'Mission outcome\n');
fprintf(fileIdentifier, 'Successful launches: %d of %d (%.2f%%)\n', analysis.successCount, analysis.totalLaunches, analysis.successRate_percent);
fprintf(fileIdentifier, 'Terminal impacts: %d of %d (%.2f%%)\n\n', analysis.terminalFlightCount, analysis.totalLaunches, analysis.terminalFlightRate_percent);
fprintf(fileIdentifier, 'Mission limits\n');
fprintf(fileIdentifier, 'Apogee: %.0f to %.0f m\n', config.mission.minApogee_m, config.mission.maxApogee_m);
fprintf(fileIdentifier, 'Downrange: %.0f to %.0f m\n', config.mission.minRange_m, config.mission.maxRange_m);
fprintf(fileIdentifier, 'Maximum dynamic pressure: %.1f kPa\n', config.mission.maxDynamicPressure_Pa / 1000);
fprintf(fileIdentifier, 'Maximum acceleration: %.2f g\n\n', config.mission.maxAcceleration_mps2 / config.g0_mps2);
fprintf(fileIdentifier, 'Terminal-flight performance statistics\n');
fprintf(fileIdentifier, '%-35s %10s %10s %10s %10s %10s\n', 'Metric', 'Mean', 'Std Dev', 'P05', 'P50', 'P95');
for metricIndex = 1:numel(analysis.metricNames)
    metricName = analysis.metricNames{metricIndex};
    statistics = analysis.allFlightStatistics.(metricName);
    fprintf(fileIdentifier, '%-35s %10.3f %10.3f %10.3f %10.3f %10.3f\n', analysis.metricLabels{metricIndex}, statistics.mean, statistics.standardDeviation, statistics.p05, statistics.p50, statistics.p95);
end
fprintf(fileIdentifier, '\nFailure criteria counts\n');
failureNames = fieldnames(analysis.failureCounts);
for failureIndex = 1:numel(failureNames)
    failureName = failureNames{failureIndex};
    failureCount = analysis.failureCounts.(failureName);
    fprintf(fileIdentifier, '%-30s %8d  %6.2f%%\n', failureName, failureCount, 100 * failureCount / analysis.totalLaunches);
end
fprintf(fileIdentifier, '\nTop sensitivity inputs\n');
topCount = min(10, numel(analysis.sensitivity.ranking));
for rankIndex = 1:topCount
    inputIndex = analysis.sensitivity.ranking(rankIndex);
    fprintf(fileIdentifier, '%2d. %-28s maximum |r| = %.4f\n', rankIndex, analysis.sensitivity.inputNames{inputIndex}, analysis.sensitivity.sortedInfluence(rankIndex));
end
fprintf(fileIdentifier, '\nRepresentative trajectories\n');
for trajectoryIndex = 1:numel(analysis.trajectorySelection.indices)
    launchIndex = analysis.trajectorySelection.indices(trajectoryIndex);
    fprintf(fileIdentifier, '%s\n', analysis.trajectorySelection.labels{trajectoryIndex});
    fprintf(fileIdentifier, 'Launch index: %d, score: %.3f, success: %d\n', launchIndex, analysis.trajectorySelection.performanceScores(trajectoryIndex), flightResults.missionSuccess(launchIndex));
    fprintf(fileIdentifier, 'Apogee: %.2f m, range: %.2f m, max velocity: %.2f m/s, max acceleration: %.3f g, max q: %.3f kPa, impact speed: %.2f m/s\n', flightResults.apogee_m(launchIndex), flightResults.downrange_m(launchIndex), flightResults.maximumVelocity_mps(launchIndex), flightResults.maximumAcceleration_mps2(launchIndex) / config.g0_mps2, flightResults.maximumDynamicPressure_Pa(launchIndex) / 1000, flightResults.impactSpeed_mps(launchIndex));
    fprintf(fileIdentifier, 'Angle: %.3f deg, initial mass: %.3f kg, propellant: %.3f kg, thrust: %.2f N, Cd: %.4f, wind: %.3f m/s\n', inputs.launchAngle_deg(launchIndex), inputs.initialMass_kg(launchIndex), inputs.propellantMass_kg(launchIndex), inputs.thrust_N(launchIndex), inputs.dragCoefficient(launchIndex), inputs.windSpeed_mps(launchIndex));
end
fprintf(fileIdentifier, '\nSaved self-contained data file: %s\n', resultsFileName);
fprintf(fileIdentifier, 'The MAT file includes the complete per-launch randomized input data, per-launch result metrics and impact conditions, configuration, equations, statistics, sensitivity analysis, trajectory envelopes, sampled time histories, and detailed representative trajectories.\n');
end
