function loadedResults = open_rocket_results(resultsFilePath, figureFiles, openDashboard)
if nargin < 1 || isempty(resultsFilePath)
    baseDirectory = fileparts(mfilename('fullpath'));
    resultsFilePath = fullfile(baseDirectory, 'results', 'rocket_monte_carlo_results.mat');
end
if nargin < 2 || isempty(figureFiles)
    figureFiles = default_figure_files(fileparts(resultsFilePath));
end
if nargin < 3
    openDashboard = true;
end
if ~isfile(resultsFilePath)
    error('RocketMonteCarlo:MissingResults', 'Results file not found: %s', resultsFilePath)
end
loadedFile = load(resultsFilePath, 'simulationResults');
loadedResults = loadedFile.simulationResults;
assignin('base', 'simulationResults', loadedResults)
assignin('base', 'rocketMonteCarloResults', loadedResults)
fprintf('Results loaded into the base workspace as simulationResults and rocketMonteCarloResults.\n')
fprintf('MAT file: %s\n', resultsFilePath)
if openDashboard && usejava('desktop')
    show_figure_dashboard(figureFiles, loadedResults.analysis.successRate_percent);
end
end

function figureFiles = default_figure_files(outputDirectory)
figureFiles = struct;
figureFiles.trajectoryEnvelope = fullfile(outputDirectory, 'trajectory_envelopes.png');
figureFiles.distributions = fullfile(outputDirectory, 'performance_distributions.png');
figureFiles.successPerformance = fullfile(outputDirectory, 'success_and_loading_performance.png');
figureFiles.sensitivity = fullfile(outputDirectory, 'parameter_sensitivity.png');
figureFiles.envelopesAndOutcome = fullfile(outputDirectory, 'flight_envelopes_and_outcome.png');
end

function show_figure_dashboard(figureFiles, successRate_percent)
figureNames = fieldnames(figureFiles);
dashboard = figure('Name', 'Rocket Monte Carlo Results Dashboard', 'NumberTitle', 'off', 'Color', 'w', 'Position', [50, 50, 1600, 950]);
annotation(dashboard, 'textbox', [0.05, 0.935, 0.90, 0.045], 'String', sprintf('Rocket Monte Carlo Results Dashboard | Mission Success: %.2f%%', successRate_percent), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'k', 'FontSize', 20, 'FontWeight', 'bold', 'EdgeColor', 'none')
annotation(dashboard, 'textbox', [0.05, 0.895, 0.90, 0.028], 'String', 'Click any preview to open it in a maximized wide-view window', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', [0.15, 0.15, 0.15], 'FontSize', 13, 'FontWeight', 'normal', 'EdgeColor', 'none')
layout = tiledlayout(dashboard, 2, 3, 'Padding', 'loose', 'TileSpacing', 'loose');
layout.OuterPosition = [0.03, 0.035, 0.94, 0.82];
for figureIndex = 1:numel(figureNames)
    figurePath = figureFiles.(figureNames{figureIndex});
    if isfile(figurePath)
        axisHandle = nexttile(layout);
        imageData = imread(figurePath);
        displayName = format_figure_name(figureNames{figureIndex});
        imageHandle = image(axisHandle, imageData);
        axis(axisHandle, 'image')
        axis(axisHandle, 'off')
        set(axisHandle, 'Color', 'w', 'ButtonDownFcn', @(~, ~) open_full_figure(figurePath, displayName))
        set(imageHandle, 'ButtonDownFcn', @(~, ~) open_full_figure(figurePath, displayName))
        title(axisHandle, displayName, 'Color', 'k', 'FontSize', 13, 'FontWeight', 'bold', 'Interpreter', 'none')
    end
end
drawnow
end

function open_full_figure(figurePath, displayName)
if ~isfile(figurePath)
    return
end
fullFigure = figure('Name', displayName, 'NumberTitle', 'off', 'Color', 'w', 'Position', [40, 40, 1650, 980]);
try
    fullFigure.WindowState = 'maximized';
catch
end
axisHandle = axes('Parent', fullFigure, 'Position', [0.015, 0.025, 0.97, 0.92]);
image(axisHandle, imread(figurePath))
axis(axisHandle, 'image')
axis(axisHandle, 'off')
title(axisHandle, displayName, 'Color', 'k', 'FontSize', 18, 'FontWeight', 'bold', 'Interpreter', 'none')
drawnow
end

function displayName = format_figure_name(figureName)
displayName = regexprep(figureName, '([a-z])([A-Z])', '$1 $2');
displayName = strrep(displayName, '_', ' ');
displayName(1) = upper(displayName(1));
end
