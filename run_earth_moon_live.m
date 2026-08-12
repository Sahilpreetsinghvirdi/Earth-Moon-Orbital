function simulationResults = run_earth_moon_live(trialCount)
baseDirectory = fileparts(mfilename('fullpath'));
if isempty(baseDirectory)
    baseDirectory = pwd;
end
addpath(baseDirectory)
config = earth_moon_orbital_config();
config.resultsFileName = 'earth_moon_live_results.mat';
config.showProgress = false;
if nargin >= 1 && ~isempty(trialCount)
    config.trialCount = max(1, round(trialCount));
else
    config.trialCount = config.live.defaultTrials;
end
rng(config.randomSeed, 'twister')
inputs = generate_earth_moon_inputs(config);
figureHandle = figure('Name', 'Live Earth-Moon Monte Carlo Simulation', 'NumberTitle', 'off', 'Color', 'w', 'Position', [40, 40, 1550, 900], 'CloseRequestFcn', @close_callback);
axesHandle = axes('Parent', figureHandle, 'Position', [0.05, 0.10, 0.62, 0.84]);
hold(axesHandle, 'on')
grid(axesHandle, 'on')
axis(axesHandle, 'equal')
axisLimit_m = 1.12 * config.moon.orbitalRadius_m;
xlim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
ylim(axesHandle, [-axisLimit_m, axisLimit_m] / 1e6)
xlabel(axesHandle, 'Earth-centered x (10^6 m)')
ylabel(axesHandle, 'Earth-centered y (10^6 m)')
title(axesHandle, 'Live 2D Earth-Moon Rocket Trajectories')
orbitAngle = linspace(0, 2 * pi, 720);
plot(axesHandle, config.moon.orbitalRadius_m * cos(orbitAngle) / 1e6, config.moon.orbitalRadius_m * sin(orbitAngle) / 1e6, '--', 'Color', [0.60, 0.60, 0.60])
bodyAngle = linspace(0, 2 * pi, 160);
fill(axesHandle, config.earth.radius_m * cos(bodyAngle) / 1e6, config.earth.radius_m * sin(bodyAngle) / 1e6, [0.15, 0.40, 0.78], 'EdgeColor', [0.04, 0.14, 0.34])
[initialMoonPosition_m, ~] = earth_moon_moon_state(config, 0, inputs.moonPhase_deg(1));
moonHandle = fill(axesHandle, (initialMoonPosition_m(1) + config.moon.radius_m * cos(bodyAngle)) / 1e6, (initialMoonPosition_m(2) + config.moon.radius_m * sin(bodyAngle)) / 1e6, [0.65, 0.65, 0.68], 'EdgeColor', [0.25, 0.25, 0.25]);
completedHandle = plot(axesHandle, nan, nan, '-', 'Color', [0.72, 0.72, 0.72], 'LineWidth', 0.55);
currentPathHandle = plot(axesHandle, nan, nan, '-', 'Color', [0.93, 0.25, 0.08], 'LineWidth', 1.7);
rocketHandle = plot(axesHandle, nan, nan, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.93, 0.25, 0.08], 'MarkerEdgeColor', 'k');
infoPanel = uipanel('Parent', figureHandle, 'Title', 'Live Statistics', 'FontSize', 12, 'Position', [0.71, 0.48, 0.26, 0.45]);
infoHandle = uicontrol('Parent', infoPanel, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04, 0.04, 0.92, 0.92], 'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'FontName', 'Consolas', 'FontSize', 10, 'String', 'Preparing trials...');
controlPanel = uipanel('Parent', figureHandle, 'Title', 'Controls', 'FontSize', 12, 'Position', [0.71, 0.20, 0.26, 0.20]);
uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Stop after current step', 'Units', 'normalized', 'Position', [0.05, 0.57, 0.90, 0.24], 'Callback', @stop_callback);
uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'Animation speed', 'Units', 'normalized', 'Position', [0.05, 0.22, 0.31, 0.18], 'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
speedHandle = uicontrol('Parent', controlPanel, 'Style', 'slider', 'Min', 0.25, 'Max', 20, 'Value', 1, 'Units', 'normalized', 'Position', [0.36, 0.24, 0.59, 0.16], 'BackgroundColor', 'w');
setappdata(figureHandle, 'stopRequested', false)
trialResults = struct([]);
trajectoryOverview = struct([]);
completedTrajectoryCapacity = (config.trajectoryOverviewPoints + 1) * config.trialCount;
completedTrajectoryX = nan(completedTrajectoryCapacity, 1);
completedTrajectoryY = nan(completedTrajectoryCapacity, 1);
completedTrajectoryPointCount = 0;
startedAt = datetime('now');
stopped = false;
for trialIndex = 1:config.trialCount
    if ~isgraphics(figureHandle) || getappdata(figureHandle, 'stopRequested')
        stopped = true;
        break
    end
    input = earth_moon_trial_input(inputs, trialIndex);
    currentPathCapacity = ceil(input.burnDuration_s / config.poweredTimeStep_s) + ceil((config.maxSimulationTime_s - input.burnDuration_s) / config.coastTimeStep_s) + 3;
    currentX = nan(currentPathCapacity, 1);
    currentY = nan(currentPathCapacity, 1);
    currentPointCount = 0;
    stepCount = 0;
    [trialResult, trialTrajectory, completed] = simulate_earth_moon_trial(config, input, @update_live, @should_stop);
    if ~completed
        stopped = true;
        break
    end
    if trialIndex == 1
        trialResults = trialResult;
        trajectoryOverview = trialTrajectory;
    else
        trialResults(trialIndex, 1) = trialResult;
        trajectoryOverview(trialIndex, 1) = trialTrajectory;
    end
    if isgraphics(figureHandle)
        if currentPointCount > 0
            displayPointCount = min(config.trajectoryOverviewPoints, currentPointCount);
            displayIndices = round(linspace(1, currentPointCount, displayPointCount));
            writeStart = completedTrajectoryPointCount + 1;
            writeEnd = writeStart + displayPointCount - 1;
            completedTrajectoryX(writeStart:writeEnd) = currentX(displayIndices);
            completedTrajectoryY(writeStart:writeEnd) = currentY(displayIndices);
            completedTrajectoryPointCount = writeEnd + 1;
            completedTrajectoryX(completedTrajectoryPointCount) = nan;
            completedTrajectoryY(completedTrajectoryPointCount) = nan;
            set(completedHandle, 'XData', completedTrajectoryX(1:completedTrajectoryPointCount), 'YData', completedTrajectoryY(1:completedTrajectoryPointCount))
        end
        set(currentPathHandle, 'XData', nan, 'YData', nan)
        drawnow
    end
end
if stopped
    simulationResults = [];
    return
end
analysis = analyze_earth_moon_results(config, trialResults);
bestTrialIndex = analysis.bestTrialIndex;
bestInput = earth_moon_trial_input(inputs, bestTrialIndex);
[bestTrialResult, bestTrajectory] = simulate_earth_moon_trial(config, bestInput, [], [], true);
trialResults(bestTrialIndex, 1) = bestTrialResult;
endedAt = datetime('now');
outputDirectory = fullfile(baseDirectory, config.outputDirectoryName);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory)
end
simulationResults = struct;
simulationResults.metadata = struct('schemaVersion', '1.0', 'startedAt', char(startedAt), 'endedAt', char(endedAt), 'elapsedSeconds', seconds(endedAt - startedAt), 'matlabVersion', version, 'platform', computer, 'randomGenerator', 'twister');
simulationResults.config = config;
simulationResults.inputs = inputs;
simulationResults.trialResults = trialResults;
simulationResults.trajectoryOverview = trajectoryOverview;
simulationResults.analysis = analysis;
simulationResults.bestTrialIndex = bestTrialIndex;
simulationResults.bestTrajectory = bestTrajectory;
simulationResults.artifacts = struct('outputDirectory', outputDirectory, 'resultsFile', fullfile(outputDirectory, config.resultsFileName));
if config.makeFigures
    simulationResults.artifacts.figureFiles = create_earth_moon_figures(config, outputDirectory, simulationResults);
else
    simulationResults.artifacts.figureFiles = struct;
end
simulationResults.artifacts.summaryReport = write_earth_moon_report(outputDirectory, simulationResults);
save(simulationResults.artifacts.resultsFile, 'simulationResults', '-v7.3')
if isgraphics(figureHandle)
    set(infoHandle, 'String', sprintf('Completed %d trials\n\nMoon encounters: %d\nLunar captures: %d\nLunar orbits: %d\n\nBest trial: %d\n%s\n\nSaved:\n%s', config.trialCount, analysis.moonEncounterCount, analysis.lunarCaptureCount, analysis.lunarOrbitCount, bestTrialIndex, bestTrialResult.classification, simulationResults.artifacts.resultsFile))
end

    function continueSimulation = update_live(state)
        continueSimulation = ~should_stop();
        if ~continueSimulation
            return
        end
        stepCount = stepCount + 1;
        currentPointCount = currentPointCount + 1;
        currentX(currentPointCount) = state.position_m(1) / 1e6;
        currentY(currentPointCount) = state.position_m(2) / 1e6;
        if mod(stepCount, config.live.frameStride) ~= 0 && state.time_s > 0
            return
        end
        moonX = state.moonPosition_m(1) + config.moon.radius_m * cos(bodyAngle);
        moonY = state.moonPosition_m(2) + config.moon.radius_m * sin(bodyAngle);
        set(moonHandle, 'XData', moonX / 1e6, 'YData', moonY / 1e6)
        set(currentPathHandle, 'XData', currentX(1:currentPointCount), 'YData', currentY(1:currentPointCount))
        set(rocketHandle, 'XData', currentX(currentPointCount), 'YData', currentY(currentPointCount))
        completedCount = numel(trialResults);
        if isempty(trialResults)
            encounters = 0;
            captures = 0;
            orbits = 0;
        else
            encounters = nnz([trialResults.moonEncounter]);
            captures = nnz([trialResults.lunarCapture]);
            orbits = nnz([trialResults.lunarOrbit]);
        end
        infoText = sprintf('EARTH-MOON MONTE CARLO\n\nTrial:              %d / %d\nSimulation Time:    %.3f days\nRocket Altitude:    %.0f km\nRocket Velocity:    %.0f m/s\nMoon Distance:      %.0f km\nMoon Relative V:    %.0f m/s\nEarth Escape:       %s\nMoon SOI:           %s\nLunar Capture:      %s\n\nCompleted Trials:   %d\nMoon Encounters:    %d\nLunar Captures:     %d\nLunar Orbits:       %d', trialIndex, config.trialCount, state.time_s / 86400, state.altitude_m / 1000, norm(state.velocity_mps), state.moonDistance_m / 1000, state.moonRelativeSpeed_mps, yes_no(state.earthSpecificEnergy_Jkg > 0 && norm(state.position_m) >= config.earth.escapeCheckRadius_m), yes_no(state.moonDistance_m <= config.moon.sphereOfInfluence_m), yes_no(state.moonSpecificEnergy_Jkg < 0 && state.moonDistance_m <= config.moon.sphereOfInfluence_m), completedCount, encounters, captures, orbits);
        set(infoHandle, 'String', infoText)
        drawnow limitrate
        if isgraphics(speedHandle)
            pause(config.live.pause_s / max(get(speedHandle, 'Value'), 0.25))
        end
    end

    function value = should_stop()
        value = ~isgraphics(figureHandle) || getappdata(figureHandle, 'stopRequested');
    end

    function stop_callback(~, ~)
        if isgraphics(figureHandle)
            setappdata(figureHandle, 'stopRequested', true)
        end
    end

    function close_callback(~, ~)
        setappdata(figureHandle, 'stopRequested', true)
        delete(figureHandle)
    end
end

function output = yes_no(value)
if value
    output = 'YES';
else
    output = 'NO';
end
end
