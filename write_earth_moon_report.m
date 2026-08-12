function reportPath = write_earth_moon_report(outputDirectory, simulationResults)
reportPath = fullfile(outputDirectory, 'earth_moon_summary_report.txt');
fileIdentifier = fopen(reportPath, 'w');
if fileIdentifier < 0
    error('EarthMoonSimulation:ReportWriteFailure', 'Unable to create the Earth-Moon summary report.')
end
cleanup = onCleanup(@() fclose(fileIdentifier));
config = simulationResults.config;
analysis = simulationResults.analysis;
bestResult = simulationResults.trialResults(simulationResults.bestTrialIndex);
bestInput = earth_moon_trial_input(simulationResults.inputs, simulationResults.bestTrialIndex);
fprintf(fileIdentifier, '%s\n', config.simulationName);
fprintf(fileIdentifier, 'Generated: %s\n\n', simulationResults.metadata.endedAt);
fprintf(fileIdentifier, 'Configuration\n');
fprintf(fileIdentifier, 'Trials: %d\n', config.trialCount);
fprintf(fileIdentifier, 'Random seed: %d\n', config.randomSeed);
fprintf(fileIdentifier, 'Maximum simulation time: %.3f days\n', config.maxSimulationTime_s / 86400);
fprintf(fileIdentifier, 'Moon orbital radius: %.0f km\n', config.moon.orbitalRadius_m / 1000);
fprintf(fileIdentifier, 'Moon SOI radius: %.0f km\n\n', config.moon.sphereOfInfluence_m / 1000);
fprintf(fileIdentifier, 'Mission outcomes\n');
fprintf(fileIdentifier, 'Successful lunar orbits: %d of %d (%.2f%%)\n', analysis.successfulTrials, analysis.totalTrials, analysis.successRate_percent);
fprintf(fileIdentifier, 'Moon encounters: %d\n', analysis.moonEncounterCount);
fprintf(fileIdentifier, 'Lunar captures: %d\n', analysis.lunarCaptureCount);
fprintf(fileIdentifier, 'Lunar impacts: %d\n', analysis.lunarImpactCount);
fprintf(fileIdentifier, 'Earth escapes: %d\n', analysis.earthEscapeCount);
fprintf(fileIdentifier, 'Earth impacts: %d\n\n', analysis.earthImpactCount);
fprintf(fileIdentifier, 'Classification counts\n');
for classIndex = 1:numel(analysis.classifications)
    fprintf(fileIdentifier, '%-28s %d\n', analysis.classifications{classIndex}, analysis.classificationCounts(classIndex));
end
fprintf(fileIdentifier, '\nBest trajectory\n');
fprintf(fileIdentifier, 'Trial: %d\n', simulationResults.bestTrialIndex);
fprintf(fileIdentifier, 'Classification: %s\n', bestResult.classification);
fprintf(fileIdentifier, 'Minimum Moon distance: %.3f km\n', bestResult.minimumMoonDistance_m / 1000);
fprintf(fileIdentifier, 'Minimum Moon-relative velocity: %.3f m/s\n', bestResult.minimumMoonRelativeVelocity_mps);
fprintf(fileIdentifier, 'Maximum altitude: %.3f km\n', bestResult.maximumAltitude_m / 1000);
fprintf(fileIdentifier, 'Moon phase: %.3f deg\n', bestInput.moonPhase_deg);
fprintf(fileIdentifier, 'Target pitch: %.3f deg\n', bestInput.targetPitch_deg);
fprintf(fileIdentifier, 'Translunar injection delta-v: %.3f m/s\n', bestInput.translunarInjectionDeltaV_mps);
fprintf(fileIdentifier, 'Lunar insertion delta-v: %.3f m/s\n', bestInput.lunarInsertionDeltaV_mps);
fprintf(fileIdentifier, 'Lunar insertion distance: %.3f km\n', bestResult.lunarInsertionMoonDistance_m / 1000);
fprintf(fileIdentifier, 'Initial mass: %.3f kg\n', bestInput.initialMass_kg);
fprintf(fileIdentifier, 'Thrust: %.3f MN\n\n', bestInput.thrust_N / 1e6);
fprintf(fileIdentifier, 'Metric statistics\n');
fprintf(fileIdentifier, '%-34s %12s %12s %12s %12s\n', 'Metric', 'Mean', 'P05', 'P50', 'P95');
for metricIndex = 1:numel(analysis.metricNames)
    metricName = analysis.metricNames{metricIndex};
    statistics = analysis.metricStatistics.(metricName);
    fprintf(fileIdentifier, '%-34s %12.3f %12.3f %12.3f %12.3f\n', metricName, statistics.mean, statistics.p05, statistics.p50, statistics.p95);
end
end
