function figureFiles = create_earth_moon_figures(config, outputDirectory, simulationResults)
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
figureFiles = struct;
trajectory = simulationResults.bestTrajectory;
bestResult = simulationResults.trialResults(simulationResults.bestTrialIndex);
moonOrbitAngle = linspace(0, 2 * pi, 720);
moonOrbit_m = config.moon.orbitalRadius_m .* [cos(moonOrbitAngle); sin(moonOrbitAngle)];
earthAngle = linspace(0, 2 * pi, 240);
earthOutline_m = config.earth.radius_m .* [cos(earthAngle); sin(earthAngle)];
moonAngle = linspace(0, 2 * pi, 160);
latestMoonPosition_m = trajectory.moonPosition_m(end, :)';
moonOutline_m = latestMoonPosition_m + config.moon.radius_m .* [cos(moonAngle); sin(moonAngle)];
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [80, 80, 1320, 760]);
axesHandle = axes('Parent', figureHandle);
hold(axesHandle, 'on')
plot(axesHandle, moonOrbit_m(1, :) / 1e6, moonOrbit_m(2, :) / 1e6, '--', 'Color', [0.55, 0.55, 0.55], 'LineWidth', 1.1)
fill(axesHandle, earthOutline_m(1, :) / 1e6, earthOutline_m(2, :) / 1e6, [0.15, 0.40, 0.78], 'EdgeColor', [0.05, 0.18, 0.40])
fill(axesHandle, moonOutline_m(1, :) / 1e6, moonOutline_m(2, :) / 1e6, [0.64, 0.64, 0.67], 'EdgeColor', [0.25, 0.25, 0.25])
plot(axesHandle, trajectory.position_m(:, 1) / 1e6, trajectory.position_m(:, 2) / 1e6, 'Color', [0.93, 0.25, 0.08], 'LineWidth', 1.8)
plot(axesHandle, trajectory.position_m(1, 1) / 1e6, trajectory.position_m(1, 2) / 1e6, '^', 'MarkerSize', 8, 'MarkerFaceColor', [0.12, 0.65, 0.30], 'MarkerEdgeColor', 'k')
plot(axesHandle, trajectory.position_m(end, 1) / 1e6, trajectory.position_m(end, 2) / 1e6, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.93, 0.25, 0.08], 'MarkerEdgeColor', 'k')
axis(axesHandle, 'equal')
axisLimit_m = max([1.1 * config.moon.orbitalRadius_m, 1.05 * max(abs(trajectory.position_m(:)))]);
xlim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
ylim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
grid(axesHandle, 'on')
xlabel(axesHandle, 'Earth-centered x (10^6 m)')
ylabel(axesHandle, 'Earth-centered y (10^6 m)')
title(axesHandle, sprintf('Best trajectory: trial %d, %s', simulationResults.bestTrialIndex, bestResult.classification))
legend(axesHandle, {'Moon orbit', 'Earth', 'Moon at final time', 'Rocket trajectory', 'Launch', 'Final state'}, 'Location', 'best')
figureFiles.bestTrajectory = save_figure(figureHandle, outputDirectory, 'earth_moon_best_trajectory.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [90, 90, 1320, 700]);
subplot(1, 2, 1)
bar(simulationResults.analysis.classificationCounts, 'FaceColor', [0.22, 0.55, 0.86])
set(gca, 'XTick', 1:numel(simulationResults.analysis.classifications), 'XTickLabel', simulationResults.analysis.classifications, 'XTickLabelRotation', 35)
ylabel('Trials')
title('Final mission classifications')
grid on
subplot(1, 2, 2)
minimumMoonDistance_km = [simulationResults.trialResults.minimumMoonDistance_m] / 1000;
histogram(minimumMoonDistance_km, min(30, max(8, round(sqrt(numel(minimumMoonDistance_km))))), 'FaceColor', [0.93, 0.60, 0.18], 'EdgeColor', 'none')
hold on
xline(config.moon.sphereOfInfluence_m / 1000, '--r', 'LineWidth', 1.4)
xline(config.moon.radius_m / 1000, '--k', 'LineWidth', 1.4)
grid on
xlabel('Minimum Moon distance (km)')
ylabel('Trials')
title('Moon approach distribution')
legend({'Trials', 'Moon SOI', 'Moon radius'}, 'Location', 'best')
figureFiles.outcomes = save_figure(figureHandle, outputDirectory, 'earth_moon_outcomes.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [100, 100, 1320, 700]);
subplot(2, 1, 1)
plot(trajectory.time_s / 86400, trajectory.altitude_m / 1000, 'Color', [0.12, 0.47, 0.71], 'LineWidth', 1.5)
grid on
xlabel('Simulation time (days)')
ylabel('Altitude (km)')
title('Best trajectory altitude')
subplot(2, 1, 2)
plot(trajectory.time_s / 86400, trajectory.moonDistance_m / 1000, 'Color', [0.86, 0.25, 0.25], 'LineWidth', 1.5)
hold on
yline(config.moon.sphereOfInfluence_m / 1000, '--k', 'LineWidth', 1.2)
grid on
xlabel('Simulation time (days)')
ylabel('Moon distance (km)')
title('Best trajectory Moon distance')
legend({'Rocket-Moon distance', 'Moon SOI'}, 'Location', 'best')
figureFiles.bestMetrics = save_figure(figureHandle, outputDirectory, 'earth_moon_best_metrics.png');
close(figureHandle)
end

function figurePath = save_figure(figureHandle, outputDirectory, fileName)
figurePath = fullfile(outputDirectory, fileName);
saveas(figureHandle, figurePath)
end
