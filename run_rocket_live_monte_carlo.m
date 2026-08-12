function simulationResults = run_rocket_live_monte_carlo(trialCount, autoStart)
baseDirectory = fileparts(mfilename('fullpath'));
if isempty(baseDirectory)
    baseDirectory = pwd;
end
addpath(baseDirectory)
rehash
config = rocket_monte_carlo_config();
if nargin >= 1 && ~isempty(trialCount)
    config.liveDefaultTrials = min(max(round(trialCount), 1), config.launchCount);
end
if nargin >= 2 && ~isempty(autoStart)
    config.liveAutoStart = logical(autoStart);
end
if ~usejava('desktop')
    error('RocketMonteCarlo:LiveDesktopRequired', 'The live visualization requires the MATLAB desktop graphics environment.')
end
rng(config.randomSeed, 'twister');
inputs = generate_rocket_inputs(config);
requestedTrials = min(config.liveDefaultTrials, config.launchCount);
sampleCount = min(config.trajectorySampleCount, requestedTrials);
sampleIndices = sort(randperm(requestedTrials, sampleCount))';
state = struct;
state.baseDirectory = baseDirectory;
state.config = config;
state.inputs = inputs;
state.startedAt = datetime('now');
state.requestedTrials = requestedTrials;
state.nextTrial = 1;
state.currentTrial = 0;
state.running = false;
state.paused = false;
state.stopRequested = false;
state.resetRequested = false;
state.finished = false;
state.speedFactor = 1;
state.sampleIndices = sampleIndices;
state.sampleMap = zeros(requestedTrials, 1);
state.sampleMap(sampleIndices) = 1:sampleCount;
state.flightResults = initialize_results(requestedTrials);
state.trajectorySample = initialize_trajectory_sample(config, sampleIndices);
state.trajectoryOverview = initialize_trajectory_overview(config, requestedTrials);
[state.completedTrajectoryX, state.completedTrajectoryY] = initialize_completed_trajectory_buffers(config, requestedTrials);
state.completedTrajectoryPointCount = 0;
state.visibleTrajectoryHandles = gobjects(0, 1);
state.visibleTrajectoryIndices = zeros(0, 1);
state.simulationResults = [];
figureHandle = create_live_figure();
state.figure = figureHandle;
guidata(figureHandle, state);
if config.liveAutoStart
    drawnow
    start_callback(figureHandle, [])
end
uiwait(figureHandle)
if isgraphics(figureHandle)
    finalState = guidata(figureHandle);
    simulationResults = finalState.simulationResults;
    if ~finalState.finished
        delete(figureHandle)
    end
else
    simulationResults = [];
end

    function figureHandle = create_live_figure()
        figureHandle = figure('Name', 'Live Monte Carlo Rocket Trajectory', 'NumberTitle', 'off', 'Color', 'w', 'MenuBar', 'none', 'ToolBar', 'figure', 'Position', [40, 40, 1600, 900], 'CloseRequestFcn', @close_callback);
        axesHandle = axes('Parent', figureHandle, 'Position', [0.055, 0.12, 0.45, 0.80], 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'FontSize', 11);
        hold(axesHandle, 'on')
        grid(axesHandle, 'on')
        axesHandle.GridAlpha = 0.25;
        [initialXLimits, initialYLimits] = initial_live_limits();
        set(axesHandle, 'DataAspectRatio', [1, 1, 1], 'PlotBoxAspectRatioMode', 'auto', 'XLim', initialXLimits, 'YLim', initialYLimits)
        xlabel(axesHandle, 'Horizontal distance (m)')
        ylabel(axesHandle, 'Altitude (m)')
        title(axesHandle, 'Live 2D Monte Carlo Rocket Launches', 'Color', 'k', 'FontSize', 18, 'FontWeight', 'bold')
        groundX = linspace(-10000, 10000, 300);
        groundY = -(groundX .^ 2) ./ (2 * config.earthRadius_m);
        fill(axesHandle, [groundX, fliplr(groundX)], [groundY, -3000 * ones(size(groundY))], [0.80, 0.88, 0.97], 'EdgeColor', 'none', 'FaceAlpha', 0.9, 'Tag', 'earthRegion')
        plot(axesHandle, groundX, groundY, 'Color', [0.10, 0.32, 0.55], 'LineWidth', 2, 'Tag', 'earthSurface')
        plot(axesHandle, 0, 0, '^', 'MarkerSize', 10, 'MarkerFaceColor', [0.18, 0.45, 0.80], 'MarkerEdgeColor', 'k', 'Tag', 'launchPad')
        text(axesHandle, 0, -max(100, 0.04 * config.mission.maxApogee_m), 'Earth launch region', 'Color', [0.10, 0.25, 0.45], 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Tag', 'earthLabel')
        stateAxes = struct;
        stateAxes.axes = axesHandle;
        stateAxes.completedTrajectories = plot(axesHandle, nan, nan, '-', 'Color', [0.72, 0.72, 0.72], 'LineWidth', 0.55, 'Tag', 'completedTrajectories');
        stateAxes.launchDirection = plot(axesHandle, [0, 350], [0, 350 * tand(78)], '--', 'Color', [0.30, 0.30, 0.30], 'LineWidth', 1, 'Tag', 'launchDirection');
        stateAxes.launchDirectionLabel = text(axesHandle, 420, 420 * tand(78), 'Launch direction', 'Color', [0.20, 0.20, 0.20], 'Tag', 'launchDirectionLabel');
        stateAxes.currentTrajectory = plot(axesHandle, nan, nan, '-', 'Color', [0.95, 0.25, 0.08], 'LineWidth', 2.2, 'Tag', 'currentTrajectory');
        stateAxes.rocket = plot(axesHandle, nan, nan, 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.95, 0.20, 0.05], 'MarkerEdgeColor', 'k', 'Tag', 'activeRocket');
        stateAxes.currentApogee = plot(axesHandle, nan, nan, 'p', 'MarkerSize', 10, 'MarkerFaceColor', [0.98, 0.75, 0.05], 'MarkerEdgeColor', 'k', 'Tag', 'currentApogee');
        stateAxes.infoPanel = uipanel('Parent', figureHandle, 'Title', 'Live information', 'FontSize', 12, 'ForegroundColor', 'k', 'BackgroundColor', 'w', 'Position', [0.715, 0.49, 0.27, 0.43]);
        stateAxes.infoText = uicontrol('Parent', stateAxes.infoPanel, 'Style', 'text', 'Units', 'normalized', 'Position', [0.03, 0.03, 0.94, 0.94], 'HorizontalAlignment', 'left', 'BackgroundColor', 'w', 'ForegroundColor', 'k', 'FontName', 'Consolas', 'FontSize', 10, 'String', format_info([], 0, requestedTrials, 'READY'));
        stateAxes.controlPanel = uipanel('Parent', figureHandle, 'Title', 'Simulation controls', 'FontSize', 12, 'ForegroundColor', 'k', 'BackgroundColor', 'w', 'Position', [0.715, 0.12, 0.27, 0.31]);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'pushbutton', 'String', 'Start', 'Units', 'normalized', 'Position', [0.04, 0.72, 0.20, 0.16], 'FontSize', 10, 'Callback', @start_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'pushbutton', 'String', 'Pause', 'Units', 'normalized', 'Position', [0.27, 0.72, 0.20, 0.16], 'FontSize', 10, 'Callback', @pause_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'pushbutton', 'String', 'Resume', 'Units', 'normalized', 'Position', [0.50, 0.72, 0.20, 0.16], 'FontSize', 10, 'Callback', @resume_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'pushbutton', 'String', 'Stop', 'Units', 'normalized', 'Position', [0.73, 0.72, 0.20, 0.16], 'FontSize', 10, 'Callback', @stop_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'pushbutton', 'String', 'Reset', 'Units', 'normalized', 'Position', [0.04, 0.52, 0.20, 0.16], 'FontSize', 10, 'Callback', @reset_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'text', 'String', 'Trials', 'Units', 'normalized', 'Position', [0.28, 0.53, 0.20, 0.12], 'BackgroundColor', 'w', 'ForegroundColor', 'k', 'HorizontalAlignment', 'right');
        stateAxes.trialEdit = uicontrol('Parent', stateAxes.controlPanel, 'Style', 'edit', 'String', num2str(requestedTrials), 'Units', 'normalized', 'Position', [0.51, 0.52, 0.42, 0.16], 'BackgroundColor', 'w', 'ForegroundColor', 'k', 'Callback', @trial_callback);
        uicontrol('Parent', stateAxes.controlPanel, 'Style', 'text', 'String', 'Speed', 'Units', 'normalized', 'Position', [0.04, 0.26, 0.20, 0.12], 'BackgroundColor', 'w', 'ForegroundColor', 'k', 'HorizontalAlignment', 'right');
        stateAxes.speedSlider = uicontrol('Parent', stateAxes.controlPanel, 'Style', 'slider', 'Min', 0.25, 'Max', 30, 'Value', 1, 'Units', 'normalized', 'Position', [0.27, 0.30, 0.52, 0.10], 'Callback', @speed_callback);
        stateAxes.speedText = uicontrol('Parent', stateAxes.controlPanel, 'Style', 'text', 'String', '1.00x', 'Units', 'normalized', 'Position', [0.80, 0.24, 0.16, 0.16], 'BackgroundColor', 'w', 'ForegroundColor', 'k', 'HorizontalAlignment', 'left');
        stateAxes.statusText = uicontrol('Parent', figureHandle, 'Style', 'text', 'String', 'Status: READY', 'Units', 'normalized', 'Position', [0.055, 0.035, 0.62, 0.045], 'BackgroundColor', 'w', 'ForegroundColor', [0.15, 0.15, 0.15], 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
        stateAxes.limitListeners = [addlistener(axesHandle, 'XLim', 'PostSet', @(~, ~) update_earth_region(axesHandle)), addlistener(axesHandle, 'YLim', 'PostSet', @(~, ~) update_earth_region(axesHandle))];
        setappdata(figureHandle, 'liveAxes', stateAxes)
        update_earth_region(axesHandle)
    end

    function start_callback(source, ~)
        figureHandle = callback_figure(source);
        if isempty(figureHandle) || ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        if state.running || state.finished
            return
        end
        state.stopRequested = false;
        state.resetRequested = false;
        state.running = true;
        state.paused = false;
        guidata(figureHandle, state)
        run_trials(figureHandle)
    end

    function pause_callback(source, ~)
        figureHandle = callback_figure(source);
        if ~isempty(figureHandle) && isgraphics(figureHandle)
            state = guidata(figureHandle);
            state.paused = true;
            guidata(figureHandle, state)
        end
    end

    function resume_callback(source, ~)
        figureHandle = callback_figure(source);
        if ~isempty(figureHandle) && isgraphics(figureHandle)
            state = guidata(figureHandle);
            state.paused = false;
            guidata(figureHandle, state)
        end
    end

    function stop_callback(source, ~)
        figureHandle = callback_figure(source);
        if ~isempty(figureHandle) && isgraphics(figureHandle)
            state = guidata(figureHandle);
            state.stopRequested = true;
            state.paused = false;
            guidata(figureHandle, state)
        end
    end

    function reset_callback(source, ~)
        figureHandle = callback_figure(source);
        if isempty(figureHandle) || ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        state.stopRequested = true;
        state.resetRequested = true;
        state.paused = false;
        state.nextTrial = 1;
        state.currentTrial = 0;
        state.startedAt = datetime('now');
        state.finished = false;
        state.flightResults = initialize_results(state.requestedTrials);
        state.trajectorySample = initialize_trajectory_sample(state.config, state.sampleIndices);
        state.trajectoryOverview = initialize_trajectory_overview(state.config, state.requestedTrials);
        [state.completedTrajectoryX, state.completedTrajectoryY] = initialize_completed_trajectory_buffers(state.config, state.requestedTrials);
        state.completedTrajectoryPointCount = 0;
        guidata(figureHandle, state)
        clear_live_axes(figureHandle)
        update_live_info(figureHandle, [], 0, 'READY')
    end

    function trial_callback(source, ~)
        figureHandle = callback_figure(source);
        if isempty(figureHandle) || ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        requestedValue = str2double(get(source, 'String'));
        if ~isfinite(requestedValue)
            requestedValue = state.requestedTrials;
        end
        requestedValue = min(max(round(requestedValue), 1), state.config.launchCount);
        if state.running
            requestedValue = max(requestedValue, state.currentTrial);
        end
        state.requestedTrials = requestedValue;
        state.config.liveDefaultTrials = requestedValue;
        if ~state.running
            sampleCount = min(state.config.trajectorySampleCount, requestedValue);
            state.sampleIndices = sort(randperm(requestedValue, sampleCount))';
            state.sampleMap = zeros(requestedValue, 1);
            state.sampleMap(state.sampleIndices) = 1:sampleCount;
            state.flightResults = initialize_results(requestedValue);
            state.trajectorySample = initialize_trajectory_sample(state.config, state.sampleIndices);
            state.trajectoryOverview = initialize_trajectory_overview(state.config, requestedValue);
            [state.completedTrajectoryX, state.completedTrajectoryY] = initialize_completed_trajectory_buffers(state.config, requestedValue);
            state.completedTrajectoryPointCount = 0;
            state.nextTrial = 1;
            state.currentTrial = 0;
            clear_live_axes(figureHandle)
        end
        set(source, 'String', num2str(requestedValue))
        guidata(figureHandle, state)
    end

    function speed_callback(source, ~)
        figureHandle = callback_figure(source);
        if isempty(figureHandle) || ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        state.speedFactor = get(source, 'Value');
        axesState = getappdata(figureHandle, 'liveAxes');
        set(axesState.speedText, 'String', sprintf('%.2fx', state.speedFactor))
        guidata(figureHandle, state)
    end

    function close_callback(source, ~)
        if isgraphics(source)
            state = guidata(source);
            state.stopRequested = true;
            state.paused = false;
            guidata(source, state)
            uiresume(source)
            delete(source)
        end
    end

    function figureHandle = callback_figure(source)
        if isgraphics(source, 'figure')
            figureHandle = source;
        else
            figureHandle = ancestor(source, 'figure');
        end
    end

    function run_trials(figureHandle)
        state = guidata(figureHandle);
        for launchIndex = state.nextTrial:state.requestedTrials
            if ~isgraphics(figureHandle)
                return
            end
            state = guidata(figureHandle);
            if state.stopRequested || state.resetRequested
                break
            end
            state.currentTrial = launchIndex;
            state.nextTrial = launchIndex;
            guidata(figureHandle, state)
            input = scalar_input(state.inputs, launchIndex);
            callback = @(currentState, event) live_step_callback(figureHandle, launchIndex, currentState, event);
            stopCheck = @() live_stop_check(figureHandle);
            [result, trajectory, completed] = simulate_rocket_launch_live(state.config, input, callback, stopCheck);
            if ~isgraphics(figureHandle)
                return
            end
            state = guidata(figureHandle);
            if ~state.stopRequested && ~state.resetRequested && (completed || result.numericalFailure || result.timedOut)
                state.flightResults = assign_result(state.flightResults, result, launchIndex);
                state.trajectoryOverview = archive_trajectory_overview(state.trajectoryOverview, trajectory, launchIndex);
                archiveIndex = state.sampleMap(launchIndex);
                if archiveIndex > 0
                    state.trajectorySample = archive_trajectory(state.trajectorySample, trajectory, archiveIndex);
                end
                if ~isempty(trajectory.downrange_m)
                    draw_completed_trajectory(figureHandle, trajectory, launchIndex)
                end
                state.nextTrial = launchIndex + 1;
            else
                state.nextTrial = launchIndex;
            end
            state.running = false;
            guidata(figureHandle, state)
            if state.stopRequested || state.resetRequested
                break
            end
            state.running = true;
            guidata(figureHandle, state)
        end
        state = guidata(figureHandle);
        state.running = false;
        guidata(figureHandle, state)
        if ~state.stopRequested && ~state.resetRequested && state.nextTrial > state.requestedTrials
            finalize_live_run(figureHandle)
        elseif state.resetRequested
            state.resetRequested = false;
            state.stopRequested = false;
            guidata(figureHandle, state)
        else
            update_live_info(figureHandle, [], state.currentTrial, 'STOPPED')
        end
    end

    function keepGoing = live_step_callback(figureHandle, launchIndex, currentState, event)
        keepGoing = false;
        if ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        if state.stopRequested || state.resetRequested
            return
        end
        update_live_graphics(figureHandle, launchIndex, currentState, event)
        while state.paused && ~state.stopRequested && ~state.resetRequested && isgraphics(figureHandle)
            drawnow
            pause(0.05)
            state = guidata(figureHandle);
        end
        if state.stopRequested || state.resetRequested || ~isgraphics(figureHandle)
            return
        end
        drawnow limitrate
        pause(state.config.liveFrameInterval_s / max(state.speedFactor, 0.01))
        keepGoing = true;
    end

    function stopNow = live_stop_check(figureHandle)
        stopNow = ~isgraphics(figureHandle);
        if ~stopNow
            state = guidata(figureHandle);
            stopNow = state.stopRequested || state.resetRequested;
        end
    end

    function finalize_live_run(figureHandle)
        state = guidata(figureHandle);
        runConfig = state.config;
        runConfig.launchCount = state.requestedTrials;
        runConfig.liveVisualization = true;
        liveInputs = slice_inputs(state.inputs, state.requestedTrials);
        [analysis, flightResults] = analyze_rocket_results(runConfig, liveInputs, state.flightResults, state.trajectorySample);
        representativeTrajectories = struct;
        representativeNames = {'bestOverall', 'worstOverall', 'medianApogee', 'medianRange'};
        representativeConfig = runConfig;
        representativeConfig.showProgress = false;
        for trajectoryIndex = 1:numel(representativeNames)
            representativeTrajectories.(representativeNames{trajectoryIndex}) = simulate_rocket_single(representativeConfig, liveInputs, analysis.trajectorySelection.indices(trajectoryIndex));
        end
        outputDirectory = fullfile(state.baseDirectory, runConfig.outputDirectoryName);
        if ~exist(outputDirectory, 'dir')
            mkdir(outputDirectory)
        end
        artifacts = struct;
        artifacts.outputDirectory = outputDirectory;
        artifacts.resultsFile = fullfile(outputDirectory, runConfig.liveResultsFileName);
        artifacts.summaryReport = fullfile(outputDirectory, 'engineering_summary_report_live.txt');
        if runConfig.makeFigures
            artifacts.figureFiles = create_rocket_figures(runConfig, outputDirectory, liveInputs, flightResults, analysis, state.trajectorySample, representativeTrajectories);
        else
            artifacts.figureFiles = struct;
        end
        endedAt = datetime('now');
        runMetadata = struct('schemaVersion', '1.0-live', 'startedAt', char(state.startedAt), 'endedAt', char(endedAt), 'elapsedSeconds', seconds(endedAt - state.startedAt), 'matlabVersion', version, 'platform', computer, 'randomGenerator', 'twister', 'dataRetention', 'All simulated launches retain randomized input parameters and final result metrics. Full histories are retained for the live trajectory archive sample.');
        simulationResults = struct('metadata', runMetadata, 'config', runConfig, 'inputParameters', liveInputs, 'flightResults', flightResults, 'analysis', analysis, 'trajectorySample', state.trajectorySample, 'trajectoryOverview', state.trajectoryOverview, 'representativeTrajectories', representativeTrajectories, 'artifacts', artifacts);
        write_rocket_report(outputDirectory, runConfig, liveInputs, flightResults, analysis, artifacts.resultsFile);
        save(artifacts.resultsFile, 'simulationResults', '-v7.3')
        state.flightResults = flightResults;
        state.simulationResults = simulationResults;
        state.finished = true;
        state.running = false;
        guidata(figureHandle, state)
        update_live_info(figureHandle, [], state.requestedTrials, 'COMPLETE')
        if runConfig.openResultsOnCompletion
            open_rocket_results(artifacts.resultsFile, artifacts.figureFiles, runConfig.openFigureDashboardOnCompletion)
        end
        uiresume(figureHandle)
    end

    function update_live_graphics(figureHandle, launchIndex, currentState, event)
        axesState = getappdata(figureHandle, 'liveAxes');
        currentTrajectory = axesState.currentTrajectory;
        existingX = get(currentTrajectory, 'XData');
        existingY = get(currentTrajectory, 'YData');
        if isempty(existingX) || all(isnan(existingX))
            existingX = currentState.downrange_m;
            existingY = currentState.altitude_m;
        else
            existingX = [existingX, currentState.downrange_m];
            existingY = [existingY, currentState.altitude_m];
        end
        set(currentTrajectory, 'XData', existingX, 'YData', existingY)
        set(axesState.rocket, 'XData', currentState.downrange_m, 'YData', currentState.altitude_m)
        set(axesState.currentApogee, 'XData', currentState.downrange_m, 'YData', currentState.apogee_m)
        directionLength = 500;
        directionAngle_deg = currentState.parameters.launchAngle_deg;
        set(axesState.launchDirection, 'XData', [0, directionLength * cosd(directionAngle_deg)], 'YData', [0, directionLength * sind(directionAngle_deg)])
        set(axesState.launchDirectionLabel, 'Position', [directionLength * cosd(directionAngle_deg), directionLength * sind(directionAngle_deg), 0], 'String', sprintf('Launch %.1f deg', directionAngle_deg))
        update_view_limits(axesState.axes, currentState.downrange_m, currentState.altitude_m, currentState.apogee_m)
        update_live_info(figureHandle, currentState, launchIndex, event)
    end

    function update_view_limits(axesHandle, currentX_m, ~, currentApogee_m)
        currentXLimit = max(1000, abs(currentX_m) * 1.25 + 500);
        currentYLimit = max(1000, currentApogee_m * 1.20 + 100);
        oldXLimits = get(axesHandle, 'XLim');
        oldYLimits = get(axesHandle, 'YLim');
        xLeft = min([-500, 1.25 * min(currentX_m, 0) - 500, oldXLimits(1)]);
        xRight = max([1000, currentXLimit, oldXLimits(2)]);
        yBottom = min([-100, -0.06 * currentYLimit, oldYLimits(1)]);
        yTop = max([currentYLimit, oldYLimits(2)]);
        axisSpan = max(xRight - xLeft, yTop - yBottom);
        xRight = xLeft + axisSpan;
        yTop = yBottom + axisSpan;
        set(axesHandle, 'XLim', [xLeft, xRight], 'YLim', [yBottom, yTop])
        update_earth_region(axesHandle)
    end

    function update_earth_region(axesHandle)
        if ~isgraphics(axesHandle)
            return
        end
        xLimits = get(axesHandle, 'XLim');
        yLimits = get(axesHandle, 'YLim');
        surfaceX = linspace(xLimits(1), xLimits(2), 500);
        surfaceY = -(surfaceX .^ 2) ./ (2 * config.earthRadius_m);
        surfaceHandle = findobj(axesHandle, 'Tag', 'earthSurface');
        regionHandle = findobj(axesHandle, 'Tag', 'earthRegion');
        labelHandle = findobj(axesHandle, 'Tag', 'earthLabel');
        if isgraphics(surfaceHandle)
            set(surfaceHandle, 'XData', surfaceX, 'YData', surfaceY)
        end
        if isgraphics(regionHandle)
            set(regionHandle, 'XData', [surfaceX, fliplr(surfaceX)], 'YData', [surfaceY, yLimits(1) * ones(size(surfaceY))])
        end
        if isgraphics(labelHandle)
            set(labelHandle, 'Position', [mean(xLimits), yLimits(1) + 0.18 * (yLimits(2) - yLimits(1)), 0])
        end
    end

    function draw_completed_trajectory(figureHandle, trajectory, launchIndex)
        state = guidata(figureHandle);
        axesState = getappdata(figureHandle, 'liveAxes');
        if isempty(trajectory.downrange_m)
            return
        end
        pointCount = min(state.config.liveTrajectoryOverviewPoints, numel(trajectory.downrange_m));
        pointIndices = round(linspace(1, numel(trajectory.downrange_m), pointCount));
        xSegment = single(trajectory.downrange_m(pointIndices));
        ySegment = single(trajectory.altitude_m(pointIndices));
        writeStart = state.completedTrajectoryPointCount + 1;
        writeEnd = writeStart + pointCount - 1;
        if writeEnd + 1 > numel(state.completedTrajectoryX)
            extraCount = max(pointCount + 1, numel(state.completedTrajectoryX));
            state.completedTrajectoryX(end + extraCount, 1) = single(NaN);
            state.completedTrajectoryY(end + extraCount, 1) = single(NaN);
        end
        state.completedTrajectoryX(writeStart:writeEnd) = xSegment;
        state.completedTrajectoryY(writeStart:writeEnd) = ySegment;
        state.completedTrajectoryPointCount = writeEnd + 1;
        state.completedTrajectoryX(state.completedTrajectoryPointCount) = single(NaN);
        state.completedTrajectoryY(state.completedTrajectoryPointCount) = single(NaN);
        refreshInterval = max(1, min(state.config.liveCompletedRefreshInterval, state.requestedTrials));
        if mod(launchIndex, refreshInterval) == 0 || launchIndex == state.requestedTrials
            set(axesState.completedTrajectories, 'XData', state.completedTrajectoryX(1:state.completedTrajectoryPointCount), 'YData', state.completedTrajectoryY(1:state.completedTrajectoryPointCount))
        end
        set(axesState.currentTrajectory, 'XData', nan, 'YData', nan)
        guidata(figureHandle, state)
    end

    function clear_live_axes(figureHandle)
        if ~isgraphics(figureHandle)
            return
        end
        state = guidata(figureHandle);
        axesState = getappdata(figureHandle, 'liveAxes');
        state.visibleTrajectoryHandles = gobjects(0, 1);
        state.visibleTrajectoryIndices = zeros(0, 1);
        [state.completedTrajectoryX, state.completedTrajectoryY] = initialize_completed_trajectory_buffers(state.config, state.requestedTrials);
        state.completedTrajectoryPointCount = 0;
        set(axesState.completedTrajectories, 'XData', nan, 'YData', nan)
        set(axesState.currentTrajectory, 'XData', nan, 'YData', nan)
        set(axesState.rocket, 'XData', nan, 'YData', nan)
        set(axesState.currentApogee, 'XData', nan, 'YData', nan)
        [initialXLimits, initialYLimits] = initial_live_limits();
        set(axesState.axes, 'XLim', initialXLimits, 'YLim', initialYLimits)
        guidata(figureHandle, state)
    end

    function update_live_info(figureHandle, currentState, launchIndex, status)
        if ~isgraphics(figureHandle)
            return
        end
        axesState = getappdata(figureHandle, 'liveAxes');
        state = guidata(figureHandle);
        if isempty(currentState)
            textValue = format_info([], launchIndex, state.requestedTrials, status);
        else
            textValue = format_info(currentState, launchIndex, state.requestedTrials, status);
        end
        set(axesState.infoText, 'String', textValue)
        set(axesState.statusText, 'String', ['Status: ', char(status)])
        drawnow limitrate
    end
end

function results = initialize_results(n)
names = {'apogee_m', 'maximumVelocity_mps', 'maximumAcceleration_mps2', 'maximumDynamicPressure_Pa', 'fuelConsumed_kg', 'remainingFuel_kg', 'burnoutTime_s', 'impactOccurred', 'numericalFailure', 'timedOut', 'flightTime_s', 'downrange_m', 'impactAltitude_m', 'impactVelocityX_mps', 'impactVelocityY_mps', 'impactSpeed_mps', 'impactFlightPathAngle_deg', 'impactDynamicPressure_Pa', 'finalX_m', 'finalY_m', 'finalVelocity_mps', 'finalVx_mps', 'finalVy_mps'};
for index = 1:numel(names)
    if ismember(names{index}, {'impactOccurred', 'numericalFailure', 'timedOut'})
        results.(names{index}) = false(n, 1);
    else
        results.(names{index}) = nan(n, 1);
    end
end
end

function sample = initialize_trajectory_sample(config, sampleIndices)
maxSteps = ceil(config.maxFlightTime_s / config.timeStep_s);
sample = struct('indices', sampleIndices, 'time_s', (0:maxSteps)' .* config.timeStep_s, 'downrange_m', nan(maxSteps + 1, numel(sampleIndices)), 'altitude_m', nan(maxSteps + 1, numel(sampleIndices)), 'velocity_mps', nan(maxSteps + 1, numel(sampleIndices)), 'acceleration_mps2', nan(maxSteps + 1, numel(sampleIndices)), 'dynamicPressure_Pa', nan(maxSteps + 1, numel(sampleIndices)), 'active', false(maxSteps + 1, numel(sampleIndices)));
end

function overview = initialize_trajectory_overview(config, launchCount)
pointCount = config.liveTrajectoryOverviewPoints;
overview = struct('time_s', single(nan(pointCount, launchCount)), 'downrange_m', single(nan(pointCount, launchCount)), 'altitude_m', single(nan(pointCount, launchCount)), 'velocity_mps', single(nan(pointCount, launchCount)), 'dynamicPressure_Pa', single(nan(pointCount, launchCount)));
end

function [xData, yData] = initialize_completed_trajectory_buffers(config, launchCount)
pointCapacity = max(1, (config.liveTrajectoryOverviewPoints + 1) * launchCount);
xData = nan(pointCapacity, 1, 'single');
yData = nan(pointCapacity, 1, 'single');
end

function [xLimits, yLimits] = initial_live_limits()
yLimits = [-100, 2100];
axisSpan = diff(yLimits);
xLimits = [-0.5 * axisSpan, 0.5 * axisSpan];
end

function output = archive_trajectory_overview(overview, trajectory, launchIndex)
pointCount = size(overview.time_s, 1);
trajectoryPointCount = numel(trajectory.time_s);
if trajectoryPointCount == 0
    output = overview;
    return
end
sampleIndices = round(linspace(1, trajectoryPointCount, pointCount));
overview.time_s(:, launchIndex) = single(trajectory.time_s(sampleIndices));
overview.downrange_m(:, launchIndex) = single(trajectory.downrange_m(sampleIndices));
overview.altitude_m(:, launchIndex) = single(trajectory.altitude_m(sampleIndices));
overview.velocity_mps(:, launchIndex) = single(trajectory.velocity_mps(sampleIndices));
overview.dynamicPressure_Pa(:, launchIndex) = single(trajectory.dynamicPressure_Pa(sampleIndices));
output = overview;
end

function output = archive_trajectory(sample, trajectory, archiveIndex)
rowCount = min(numel(trajectory.time_s), size(sample.downrange_m, 1));
sample.downrange_m(1:rowCount, archiveIndex) = trajectory.downrange_m(1:rowCount);
sample.altitude_m(1:rowCount, archiveIndex) = trajectory.altitude_m(1:rowCount);
sample.velocity_mps(1:rowCount, archiveIndex) = trajectory.velocity_mps(1:rowCount);
sample.acceleration_mps2(1:rowCount, archiveIndex) = trajectory.acceleration_mps2(1:rowCount);
sample.dynamicPressure_Pa(1:rowCount, archiveIndex) = trajectory.dynamicPressure_Pa(1:rowCount);
sample.active(1:rowCount, archiveIndex) = trajectory.status(1:rowCount) ~= "IMPACT";
output = sample;
end

function output = assign_result(results, result, launchIndex)
names = fieldnames(result);
for index = 1:numel(names)
    outputValue = result.(names{index});
    if isfield(results, names{index})
        results.(names{index})(launchIndex) = outputValue;
    end
end
output = results;
end

function input = scalar_input(inputs, launchIndex)
names = fieldnames(inputs);
input = struct;
for index = 1:numel(names)
    input.(names{index}) = inputs.(names{index})(launchIndex);
end
end

function output = slice_inputs(inputs, count)
names = fieldnames(inputs);
output = struct;
for index = 1:numel(names)
    output.(names{index}) = inputs.(names{index})(1:count);
end
end

function textValue = format_info(currentState, launchIndex, totalTrials, status)
if isempty(currentState)
    textValue = sprintf('Trial: %d / %d\nSimulation time: --\nAltitude: --\nDistance: --\nVelocity: --\nFlight-path angle: --\nMaximum altitude: --\nMaximum range: --\nMaximum q: --\n\nParameters\nAngle: --\nInitial mass: --\nPropellant: --\nThrust: --\nCd: --\nWind: --\n\nStatus: %s', launchIndex, totalTrials, char(status));
else
    input = currentState.parameters;
    textValue = sprintf('Trial: %d / %d\nSimulation time: %7.2f s\nAltitude: %9.1f m\nDistance: %9.1f m\nVelocity: %8.1f m/s\nFlight-path angle: %7.2f deg\nMaximum altitude: %8.1f m\nMaximum range: %9.1f m\nMaximum q: %8.2f kPa\n\nParameters\nAngle: %7.2f deg\nInitial mass: %7.2f kg\nPropellant: %7.2f kg\nThrust: %8.1f N\nCd: %7.3f\nWind: %7.2f m/s\n\nStatus: %s', launchIndex, totalTrials, currentState.time_s, currentState.altitude_m, currentState.downrange_m, currentState.velocity_mps, currentState.flightPathAngle_deg, currentState.apogee_m, currentState.maximumDownrange_m, currentState.maximumDynamicPressure_Pa / 1000, input.launchAngle_deg, input.initialMass_kg, input.propellantMass_kg, input.thrust_N, input.dragCoefficient, input.windSpeed_mps, char(status));
end
end
