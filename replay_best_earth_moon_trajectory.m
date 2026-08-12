function replay_best_earth_moon_trajectory(source, speedFactor)
if nargin < 1 || isempty(source)
    baseDirectory = fileparts(mfilename('fullpath'));
    source = fullfile(baseDirectory, 'results', 'earth_moon_orbital_results.mat');
end
if nargin < 2 || isempty(speedFactor)
    speedFactor = 100000;
end
if ischar(source) || isstring(source)
    loaded = load(source, 'simulationResults');
    simulationResults = loaded.simulationResults;
else
    simulationResults = source;
end
config = simulationResults.config;
trajectory = simulationResults.bestTrajectory;
figureHandle = figure('Name', 'Best Earth-Moon Trajectory Replay', 'NumberTitle', 'off', 'Color', 'w', 'Position', [80, 80, 1320, 780]);
axesHandle = axes('Parent', figureHandle);
hold(axesHandle, 'on')
grid(axesHandle, 'on')
axis(axesHandle, 'equal')
axisLimit_m = max(1.12 * config.moon.orbitalRadius_m, 1.05 * max(abs(trajectory.position_m(:))));
xlim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
ylim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
xlabel(axesHandle, 'Earth-centered x (10^6 m)')
ylabel(axesHandle, 'Earth-centered y (10^6 m)')
title(axesHandle, sprintf('Best trajectory replay: trial %d', simulationResults.bestTrialIndex))
bodyAngle = linspace(0, 2 * pi, 160);
fill(axesHandle, config.earth.radius_m * cos(bodyAngle) / 1e6, config.earth.radius_m * sin(bodyAngle) / 1e6, [0.15, 0.40, 0.78], 'EdgeColor', [0.04, 0.14, 0.34])
moonHandle = fill(axesHandle, nan, nan, [0.65, 0.65, 0.68], 'EdgeColor', [0.25, 0.25, 0.25]);
pathHandle = plot(axesHandle, nan, nan, '-', 'Color', [0.93, 0.25, 0.08], 'LineWidth', 1.7);
rocketHandle = plot(axesHandle, nan, nan, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.93, 0.25, 0.08], 'MarkerEdgeColor', 'k');
infoHandle = text(axesHandle, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', 'FontName', 'Consolas', 'BackgroundColor', 'w');
for pointIndex = 1:numel(trajectory.time_s)
    if ~isgraphics(figureHandle)
        return
    end
    moonPosition_m = trajectory.moonPosition_m(pointIndex, :)';
    set(moonHandle, 'XData', (moonPosition_m(1) + config.moon.radius_m * cos(bodyAngle)) / 1e6, 'YData', (moonPosition_m(2) + config.moon.radius_m * sin(bodyAngle)) / 1e6)
    set(pathHandle, 'XData', trajectory.position_m(1:pointIndex, 1) / 1e6, 'YData', trajectory.position_m(1:pointIndex, 2) / 1e6)
    set(rocketHandle, 'XData', trajectory.position_m(pointIndex, 1) / 1e6, 'YData', trajectory.position_m(pointIndex, 2) / 1e6)
    set(infoHandle, 'String', sprintf('Time: %.3f days\nAltitude: %.0f km\nMoon distance: %.0f km', trajectory.time_s(pointIndex) / 86400, trajectory.altitude_m(pointIndex) / 1000, trajectory.moonDistance_m(pointIndex) / 1000))
    drawnow
    if pointIndex > 1
        pause(min(0.08, (trajectory.time_s(pointIndex) - trajectory.time_s(pointIndex - 1)) / speedFactor))
    end
end
end
