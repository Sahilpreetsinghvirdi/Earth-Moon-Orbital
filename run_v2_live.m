function run_v2_live(source, speedFactor)
if nargin < 1 || isempty(source)
    baseDirectory = fileparts(mfilename('fullpath'));
    source = fullfile(baseDirectory, 'results', 'v2_optimization_results.mat');
end
if nargin < 2 || isempty(speedFactor)
    speedFactor = 100000;
end
if ischar(source) || isstring(source)
    loaded = load(source, 'output');
    output = loaded.output;
else
    output = source;
end
if isempty(output.trajectory.time_s)
    error('V2:NoTrajectory', 'The optimization result has no trajectory to replay.')
end
config = output.config;
trajectory = output.trajectory;
figureHandle = figure('Name', 'V2 Live Earth-Moon Mission', 'NumberTitle', 'off', 'Color', 'w', 'Position', [60, 60, 1500, 850]);
axesHandle = axes('Parent', figureHandle, 'Position', [0.06, 0.10, 0.62, 0.82]);
hold(axesHandle, 'on')
grid(axesHandle, 'on')
axis(axesHandle, 'equal')
globalLimit_m = max(1.05 * config.physics.moonSOI_m, max(abs([trajectory.position_m(:); trajectory.moonPosition_m(:)])) * 1.05);
xlim(axesHandle, [-globalLimit_m, globalLimit_m] / 1e6)
ylim(axesHandle, [-globalLimit_m, globalLimit_m] / 1e6)
xlabel(axesHandle, 'Earth-centered x (10^6 m)')
ylabel(axesHandle, 'Earth-centered y (10^6 m)')
title(axesHandle, 'V2 calculated trajectory replay')
bodyAngle = linspace(0, 2 * pi, 160);
earthHandle = fill(axesHandle, config.physics.earthRadius_m * cos(bodyAngle) / 1e6, config.physics.earthRadius_m * sin(bodyAngle) / 1e6, [0.15, 0.40, 0.78], 'EdgeColor', [0.03, 0.12, 0.30]);
moonHandle = fill(axesHandle, nan, nan, [0.65, 0.65, 0.68], 'EdgeColor', [0.25, 0.25, 0.25]);
pathHandle = plot(axesHandle, nan, nan, '-', 'Color', [0.92, 0.22, 0.08], 'LineWidth', 1.8);
spacecraftHandle = plot(axesHandle, nan, nan, 'o', 'MarkerFaceColor', [0.92, 0.22, 0.08], 'MarkerEdgeColor', 'k');
infoHandle = uicontrol('Parent', figureHandle, 'Style', 'text', 'Units', 'normalized', 'Position', [0.72, 0.18, 0.25, 0.68], 'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'FontName', 'Consolas', 'FontSize', 10);
frameStride = max(1, round(config.live.frameStride));
frameIndices = unique([1:frameStride:numel(trajectory.time_s), numel(trajectory.time_s)]);
for frameIndex = 1:numel(frameIndices)
    index = frameIndices(frameIndex);
    if ~isgraphics(figureHandle)
        return
    end
    moonPosition_m = trajectory.moonPosition_m(index, :)';
    localView = contains(string(trajectory.phase(index)), 'Lunar') && trajectory.moonDistance_m(index) <= 1.15 * config.physics.moonSOI_m;
    if localView
        localLimit_m = min(70e6, max(5e6, 1.25 * trajectory.moonDistance_m(index)));
        relativePath_m = trajectory.position_m(1:index, :) - trajectory.moonPosition_m(1:index, :);
        set(earthHandle, 'XData', -moonPosition_m(1) / 1e6 + config.physics.earthRadius_m * cos(bodyAngle) / 1e6, 'YData', -moonPosition_m(2) / 1e6 + config.physics.earthRadius_m * sin(bodyAngle) / 1e6)
        set(moonHandle, 'XData', config.physics.moonRadius_m * cos(bodyAngle) / 1e6, 'YData', config.physics.moonRadius_m * sin(bodyAngle) / 1e6)
        set(pathHandle, 'XData', relativePath_m(:, 1) / 1e6, 'YData', relativePath_m(:, 2) / 1e6)
        set(spacecraftHandle, 'XData', relativePath_m(end, 1) / 1e6, 'YData', relativePath_m(end, 2) / 1e6)
        xlim(axesHandle, [-localLimit_m, localLimit_m] / 1e6)
        ylim(axesHandle, [-localLimit_m, localLimit_m] / 1e6)
        xlabel(axesHandle, 'Moon-centered x (10^6 m)')
        ylabel(axesHandle, 'Moon-centered y (10^6 m)')
        title(axesHandle, 'V2 lunar encounter and orbit')
        viewName = 'Moon-centered local view';
    else
        set(earthHandle, 'XData', config.physics.earthRadius_m * cos(bodyAngle) / 1e6, 'YData', config.physics.earthRadius_m * sin(bodyAngle) / 1e6)
        set(moonHandle, 'XData', (moonPosition_m(1) + config.physics.moonRadius_m * cos(bodyAngle)) / 1e6, 'YData', (moonPosition_m(2) + config.physics.moonRadius_m * sin(bodyAngle)) / 1e6)
        set(pathHandle, 'XData', trajectory.position_m(1:index, 1) / 1e6, 'YData', trajectory.position_m(1:index, 2) / 1e6)
        set(spacecraftHandle, 'XData', trajectory.position_m(index, 1) / 1e6, 'YData', trajectory.position_m(index, 2) / 1e6)
        xlim(axesHandle, [-globalLimit_m, globalLimit_m] / 1e6)
        ylim(axesHandle, [-globalLimit_m, globalLimit_m] / 1e6)
        xlabel(axesHandle, 'Earth-centered x (10^6 m)')
        ylabel(axesHandle, 'Earth-centered y (10^6 m)')
        title(axesHandle, 'V2 calculated trajectory replay')
        viewName = 'Earth-centered global view';
    end
    info = sprintf('MISSION DATE\n%s\n\nElapsed: %.3f days\nVelocity: %.2f m/s\nEarth distance: %.2f km\nMoon distance: %.2f km\nCurrent delta-v: %.2f m/s\nPhase: %s\nView: %s', char(trajectory.epoch(index)), trajectory.time_s(index) / 86400, norm(trajectory.velocity_mps(index, :)), trajectory.earthDistance_m(index) / 1000, trajectory.moonDistance_m(index) / 1000, cumulative_delta_v(output.optimization.bestResult, trajectory.time_s(index)), char(trajectory.phase(index)), viewName);
    set(infoHandle, 'String', info)
    drawnow limitrate
    if index > 1
        pause(min(0.08, max(0, (trajectory.time_s(index) - trajectory.time_s(index - 1)) / speedFactor)))
    end
end
end

function value = cumulative_delta_v(result, time_s)
value = 0;
if ~isfield(result, 'candidate') || ~isfield(result, 'departureDeltaV_mps')
    return
end
if time_s >= 0
    value = value + result.departureDeltaV_mps;
end
approachEnd_s = (result.candidate.outboundFlightTime_days + result.candidate.lunarApproachDuration_days) * 86400;
orbitEnd_s = (result.candidate.outboundFlightTime_days + result.candidate.lunarApproachDuration_days + result.candidate.lunarOrbitDuration_days) * 86400;
if time_s >= approachEnd_s
    value = value + result.lunarOrbitInsertionDeltaV_mps;
end
if time_s >= orbitEnd_s
    value = value + result.lunarDepartureDeltaV_mps;
end
if time_s >= result.flightTime_days * 86400
    value = value + result.earthCaptureDeltaV_mps;
end
end
