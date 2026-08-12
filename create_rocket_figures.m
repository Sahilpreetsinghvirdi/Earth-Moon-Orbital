function figureFiles = create_rocket_figures(config, outputDirectory, inputs, flightResults, analysis, trajectorySample, representativeTrajectories)
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
figureFiles = struct;
envelope = analysis.trajectoryEnvelope;
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [80, 80, 1300, 620]);
subplot(1, 2, 1)
plot_envelope(envelope.time_s, envelope.altitude_m, [0.22, 0.55, 0.86]);
hold on
trajectoryNames = fieldnames(representativeTrajectories);
lineColors = [0.10, 0.10, 0.10; 0.78, 0.18, 0.18; 0.10, 0.55, 0.22; 0.55, 0.24, 0.72];
for trajectoryIndex = 1:numel(trajectoryNames)
    trajectory = representativeTrajectories.(trajectoryNames{trajectoryIndex});
    plot(trajectory.time_s, trajectory.altitude_m, 'LineWidth', 1.4, 'Color', lineColors(trajectoryIndex, :));
end
grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
title('Altitude envelope and representative flights')
legend({'5th-95th percentile', 'Median', 'Best', 'Worst', 'Median apogee', 'Median range'}, 'Location', 'best')
subplot(1, 2, 2)
plotCount = min(config.trajectoryPlotCount, size(trajectorySample.altitude_m, 2));
plotColumns = round(linspace(1, size(trajectorySample.altitude_m, 2), plotCount));
hold on
sampleHandle = [];
for columnIndex = 1:numel(plotColumns)
    lineHandle = plot(trajectorySample.downrange_m(:, plotColumns(columnIndex)), trajectorySample.altitude_m(:, plotColumns(columnIndex)), 'Color', [0.78, 0.83, 0.90]);
    if columnIndex == 1
        sampleHandle = lineHandle;
    else
        set(lineHandle, 'HandleVisibility', 'off')
    end
end
representativeHandles = gobjects(numel(trajectoryNames), 1);
for trajectoryIndex = 1:numel(trajectoryNames)
    trajectory = representativeTrajectories.(trajectoryNames{trajectoryIndex});
    representativeHandles(trajectoryIndex) = plot(trajectory.downrange_m, trajectory.altitude_m, 'LineWidth', 1.8, 'Color', lineColors(trajectoryIndex, :));
end
grid on
xlabel('Downrange distance (m)')
ylabel('Altitude (m)')
title('Trajectory spread in the flight plane')
legend([sampleHandle; representativeHandles], {'Sampled trajectories', 'Best', 'Worst', 'Median apogee', 'Median range'}, 'Location', 'best')
figureFiles.trajectoryEnvelope = save_figure(figureHandle, outputDirectory, 'trajectory_envelopes.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [100, 100, 1280, 760]);
subplot(2, 2, 1)
histogram(flightResults.apogee_m(flightResults.impactOccurred), 70, 'FaceColor', [0.22, 0.55, 0.86], 'EdgeColor', 'none')
xline(config.mission.minApogee_m, '--r', 'LineWidth', 1.2)
xline(config.mission.maxApogee_m, '--r', 'LineWidth', 1.2)
grid on
xlabel('Apogee (m)')
ylabel('Launch count')
title('Apogee distribution')
subplot(2, 2, 2)
histogram(flightResults.downrange_m(flightResults.impactOccurred), 70, 'FaceColor', [0.20, 0.67, 0.39], 'EdgeColor', 'none')
xline(config.mission.minRange_m, '--r', 'LineWidth', 1.2)
xline(config.mission.maxRange_m, '--r', 'LineWidth', 1.2)
grid on
xlabel('Downrange distance (m)')
ylabel('Launch count')
title('Range distribution')
subplot(2, 2, 3)
histogram(flightResults.maximumVelocity_mps, 70, 'FaceColor', [0.93, 0.60, 0.18], 'EdgeColor', 'none')
grid on
xlabel('Maximum velocity (m/s)')
ylabel('Launch count')
title('Maximum velocity distribution')
subplot(2, 2, 4)
histogram(flightResults.maximumAcceleration_mps2 ./ config.g0_mps2, 70, 'FaceColor', [0.65, 0.35, 0.72], 'EdgeColor', 'none')
xline(config.mission.maxAcceleration_mps2 / config.g0_mps2, '--r', 'LineWidth', 1.2)
grid on
xlabel('Maximum acceleration (g)')
ylabel('Launch count')
title('Maximum acceleration distribution')
figureFiles.distributions = save_figure(figureHandle, outputDirectory, 'performance_distributions.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [110, 110, 1280, 560]);
valid = flightResults.impactOccurred;
plotIndices = find(valid);
plotIndices = plotIndices(round(linspace(1, numel(plotIndices), min(12000, numel(plotIndices)))));
subplot(1, 2, 1)
failureIndices = plotIndices(~flightResults.missionSuccess(plotIndices));
successIndices = plotIndices(flightResults.missionSuccess(plotIndices));
scatter(flightResults.downrange_m(failureIndices) / 1000, flightResults.apogee_m(failureIndices) / 1000, 7, [0.72, 0.72, 0.72], 'filled')
hold on
scatter(flightResults.downrange_m(successIndices) / 1000, flightResults.apogee_m(successIndices) / 1000, 7, [0.12, 0.55, 0.28], 'filled')
xline(config.mission.minRange_m / 1000, '--r')
xline(config.mission.maxRange_m / 1000, '--r')
yline(config.mission.minApogee_m / 1000, '--r')
yline(config.mission.maxApogee_m / 1000, '--r')
grid on
xlabel('Range (km)')
ylabel('Apogee (km)')
title('Mission success region')
legend({'Failure', 'Success', 'Mission limits'}, 'Location', 'best')
subplot(1, 2, 2)
scatter(flightResults.maximumDynamicPressure_Pa(plotIndices) / 1000, flightResults.maximumAcceleration_mps2(plotIndices) / config.g0_mps2, 8, double(flightResults.missionSuccess(plotIndices)), 'filled')
hold on
xline(config.mission.maxDynamicPressure_Pa / 1000, '--r', 'LineWidth', 1.2)
yline(config.mission.maxAcceleration_mps2 / config.g0_mps2, '--r', 'LineWidth', 1.2)
grid on
xlabel('Maximum dynamic pressure (kPa)')
ylabel('Maximum acceleration (g)')
title('Structural loading performance')
colorbar
figureFiles.successPerformance = save_figure(figureHandle, outputDirectory, 'success_and_loading_performance.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [120, 120, 1350, 620]);
subplot(1, 2, 1)
imagesc(analysis.sensitivity.pearsonCorrelation)
axis tight
caxis([-1, 1])
colormap(parula)
colorbar
set(gca, 'XTick', 1:numel(analysis.sensitivity.outputNames), 'XTickLabel', shorten_labels(analysis.sensitivity.outputNames), 'XTickLabelRotation', 35)
set(gca, 'YTick', 1:numel(analysis.sensitivity.inputNames), 'YTickLabel', shorten_labels(analysis.sensitivity.inputNames))
title('Pearson parameter sensitivity')
subplot(1, 2, 2)
ranking = analysis.sensitivity.ranking;
topCount = min(10, numel(ranking));
barh(analysis.sensitivity.sortedInfluence(1:topCount), 'FaceColor', [0.22, 0.55, 0.86])
set(gca, 'YDir', 'reverse', 'YTick', 1:topCount, 'YTickLabel', shorten_labels(analysis.sensitivity.inputNames(ranking(1:topCount))))
xlim([0, 1])
grid on
xlabel('Largest absolute Pearson correlation')
title('Most influential randomized inputs')
figureFiles.sensitivity = save_figure(figureHandle, outputDirectory, 'parameter_sensitivity.png');
close(figureHandle)
figureHandle = figure('Visible', config.figureVisible, 'Color', 'w', 'Position', [130, 130, 1280, 760]);
subplot(2, 2, 1)
plot_envelope(envelope.time_s, envelope.velocity_mps, [0.93, 0.60, 0.18]);
grid on
xlabel('Time (s)')
ylabel('Velocity (m/s)')
title('Velocity envelope')
subplot(2, 2, 2)
plot_envelope(envelope.time_s, envelope.acceleration_mps2 ./ config.g0_mps2, [0.65, 0.35, 0.72]);
yline(config.mission.maxAcceleration_mps2 / config.g0_mps2, '--r')
grid on
xlabel('Time (s)')
ylabel('Acceleration (g)')
title('Acceleration envelope')
subplot(2, 2, 3)
plot_envelope(envelope.time_s, envelope.dynamicPressure_Pa ./ 1000, [0.18, 0.65, 0.70]);
yline(config.mission.maxDynamicPressure_Pa / 1000, '--r')
grid on
xlabel('Time (s)')
ylabel('Dynamic pressure (kPa)')
title('Dynamic pressure envelope')
subplot(2, 2, 4)
bar([analysis.successRate_percent, 100 - analysis.successRate_percent], 'FaceColor', [0.22, 0.55, 0.86])
set(gca, 'XTick', [1, 2], 'XTickLabel', {'Mission success', 'Mission failure'})
ylim([0, 100])
grid on
ylabel('Launches (%)')
title('Mission outcome rate')
figureFiles.envelopesAndOutcome = save_figure(figureHandle, outputDirectory, 'flight_envelopes_and_outcome.png');
close(figureHandle)
end

function plot_envelope(time_s, percentileValues, colorValue)
valid = isfinite(percentileValues(:, 1)) & isfinite(percentileValues(:, 2)) & isfinite(percentileValues(:, 3));
time_s = time_s(valid);
percentileValues = percentileValues(valid, :);
fill([time_s; flipud(time_s)], [percentileValues(:, 1); flipud(percentileValues(:, 3))], colorValue, 'FaceAlpha', 0.22, 'EdgeColor', 'none')
hold on
plot(time_s, percentileValues(:, 2), 'Color', colorValue, 'LineWidth', 1.8)
end

function labels = shorten_labels(names)
labels = cell(size(names));
for index = 1:numel(names)
    labels{index} = strrep(strrep(names{index}, '_mps2', ''), '_', ' ');
end
end

function figurePath = save_figure(figureHandle, outputDirectory, fileName)
figurePath = fullfile(outputDirectory, fileName);
apply_light_theme(figureHandle)
set(figureHandle, 'PaperPositionMode', 'auto')
print(figureHandle, figurePath, '-dpng', '-r180')
end

function apply_light_theme(figureHandle)
axesHandles = findall(figureHandle, 'Type', 'axes');
for axisIndex = 1:numel(axesHandles)
    axisHandle = axesHandles(axisIndex);
    set(axisHandle, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.65, 0.65, 0.65], 'GridAlpha', 0.35)
    set(get(axisHandle, 'Title'), 'Color', 'k')
    set(get(axisHandle, 'XLabel'), 'Color', 'k')
    set(get(axisHandle, 'YLabel'), 'Color', 'k')
end
legendHandles = findall(figureHandle, 'Type', 'legend');
for legendIndex = 1:numel(legendHandles)
    set(legendHandles(legendIndex), 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.25, 0.25, 0.25])
end
end
