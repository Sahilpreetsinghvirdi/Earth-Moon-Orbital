function artifacts = v2_visualize_dashboard(config, optimization, trajectory, artifacts)
outputDirectory = fullfile(fileparts(mfilename('fullpath')), config.output.directory);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
figureHandle = figure('Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1500, 900]);
axesHandle = subplot(2, 2, 1, 'Parent', figureHandle);
hold(axesHandle, 'on')
earthAngle = linspace(0, 2 * pi, 240);
fill(axesHandle, config.physics.earthRadius_m * cos(earthAngle) / 1e6, config.physics.earthRadius_m * sin(earthAngle) / 1e6, [0.15, 0.40, 0.78], 'EdgeColor', [0.03, 0.12, 0.30])
transferMask = startsWith(trajectory.phase, "Earth departure") | startsWith(trajectory.phase, "Lunar approach") | startsWith(trajectory.phase, "Lunar capture");
orbitMask = startsWith(trajectory.phase, "Lunar orbit") | startsWith(trajectory.phase, "Lunar departure phasing");
returnMask = startsWith(trajectory.phase, "Lunar departure burn") | startsWith(trajectory.phase, "Lunar departure and Earth return") | startsWith(trajectory.phase, "Earth terminal");
plot(axesHandle, trajectory.moonPosition_m(:, 1) / 1e6, trajectory.moonPosition_m(:, 2) / 1e6, '--', 'Color', [0.50, 0.50, 0.50], 'LineWidth', 1.0)
plot(axesHandle, trajectory.position_m(transferMask, 1) / 1e6, trajectory.position_m(transferMask, 2) / 1e6, 'Color', [0.92, 0.22, 0.08], 'LineWidth', 1.6)
plot(axesHandle, trajectory.position_m(orbitMask, 1) / 1e6, trajectory.position_m(orbitMask, 2) / 1e6, 'Color', [0.10, 0.62, 0.36], 'LineWidth', 1.6)
plot(axesHandle, trajectory.position_m(returnMask, 1) / 1e6, trajectory.position_m(returnMask, 2) / 1e6, 'Color', [0.10, 0.40, 0.90], 'LineWidth', 1.6)
moonEncounter_m = trajectory.moonPosition_m(find(transferMask, 1, 'last'), :);
fill(axesHandle, (moonEncounter_m(1) + config.physics.moonRadius_m * cos(earthAngle)) / 1e6, (moonEncounter_m(2) + config.physics.moonRadius_m * sin(earthAngle)) / 1e6, [0.65, 0.65, 0.68], 'EdgeColor', [0.25, 0.25, 0.25])
axis(axesHandle, 'equal')
axisLimit_m = max(1.05 * config.physics.moonSOI_m, max(abs([trajectory.position_m(:); trajectory.moonPosition_m(:)])) * 1.05);
xlim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
ylim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
grid(axesHandle, 'on')
xlabel(axesHandle, 'Earth-centered x (10^6 m)')
ylabel(axesHandle, 'Earth-centered y (10^6 m)')
title(axesHandle, 'Earth-Moon mission trajectory')
legend(axesHandle, {'Earth', 'Moon path', 'Transfer and capture', 'Lunar orbit', 'Earth return', 'Moon at arrival'}, 'Location', 'best')
axesHandle = subplot(2, 2, 2, 'Parent', figureHandle);
plot(axesHandle, optimization.history.iteration, optimization.history.bestDeltaV_mps, 'Color', [0.12, 0.45, 0.72], 'LineWidth', 1.5)
grid(axesHandle, 'on')
xlabel(axesHandle, 'Iteration')
ylabel(axesHandle, 'Best total delta-v (m/s)')
title(axesHandle, 'Best delta-v vs iteration')
axesHandle = subplot(2, 2, 3, 'Parent', figureHandle);
launchDates = optimization.history.launchEpoch;
validDates = ~isnat(launchDates);
scatter(axesHandle, launchDates(validDates), optimization.history.bestScore(validDates), 14, optimization.history.bestDeltaV_mps(validDates), 'filled')
grid(axesHandle, 'on')
xlabel(axesHandle, 'Launch date')
ylabel(axesHandle, 'Mission score')
title(axesHandle, 'Mission score vs launch date')
colorbar(axesHandle)
axesHandle = subplot(2, 2, 4, 'Parent', figureHandle);
axis(axesHandle, 'off')
best = optimization.bestResult;
if ~isfield(best.candidate, 'lunarApproachDuration_days')
    best.candidate.lunarApproachDuration_days = config.mission.lunarApproachDuration_days;
end
if ~isfield(best, 'insertionEpoch')
    best.insertionEpoch = best.arrivalEpoch + days(best.candidate.lunarApproachDuration_days);
end
summary = sprintf('V2 OPTIMIZATION SUMMARY\n\nLaunch Date:       %s\nMoon Encounter:    %s\nLunar Orbit:       %s\nEarth Return:      %s\n\nTotal delta-v:     %.2f m/s\nEstimated fuel:    %.2f kg\nMission score:     %.6f\n\nLunar orbit valid: %d\nCandidates tested: %d\nSuccessful:        %d\nIterations:        %d\nStop reason:       %s', char(best.candidate.launchEpoch), char(best.arrivalEpoch), char(best.insertionEpoch), char(best.returnEpoch), best.totalDeltaV_mps, best.fuel.requiredPropellant_kg, best.score, trajectory.lunarOrbitValid, optimization.candidatesTested, optimization.successfulCandidates, optimization.iterations, optimization.stopReason);
text(axesHandle, 0.02, 0.96, summary, 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontName', 'Consolas', 'FontSize', 10)
dashboardPath = fullfile(outputDirectory, config.output.dashboardFigure);
saveas(figureHandle, dashboardPath)
close(figureHandle)
artifacts.dashboardFigure = dashboardPath;
figureHandle = figure('Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1300, 760]);
plot(trajectory.time_s / 86400, trajectory.moonDistance_m / 1000, 'Color', [0.85, 0.20, 0.20], 'LineWidth', 1.4)
hold on
yline(config.physics.moonSOI_m / 1000, '--k', 'Moon SOI')
grid on
xlabel('Mission elapsed time (days)')
ylabel('Moon distance (km)')
title('Best trajectory Moon-distance history')
trajectoryPath = fullfile(outputDirectory, config.output.trajectoryFigure);
saveas(figureHandle, trajectoryPath)
close(figureHandle)
artifacts.trajectoryFigure = trajectoryPath;
end
