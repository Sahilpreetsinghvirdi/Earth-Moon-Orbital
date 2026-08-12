function simulationResults = run_rocket_monte_carlo(runMode)
if nargin >= 1 && (ischar(runMode) || isstring(runMode)) && strcmpi(string(runMode), 'live')
    simulationResults = run_rocket_live_monte_carlo();
    return
end
baseDirectory = fileparts(mfilename('fullpath'));
if isempty(baseDirectory)
    baseDirectory = pwd;
end
addpath(baseDirectory)
rehash
config = rocket_monte_carlo_config();
if config.liveVisualization
    simulationResults = run_rocket_live_monte_carlo();
    return
end
outputDirectory = fullfile(baseDirectory, config.outputDirectoryName);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
resultsFilePath = fullfile(outputDirectory, 'rocket_monte_carlo_results.mat');
failureLogPath = fullfile(outputDirectory, 'simulation_failure.txt');
startedAt = datetime('now');
try
    rng(config.randomSeed, 'twister');
    fprintf('%s\n', config.simulationName)
    fprintf('Random seed: %d\n', config.randomSeed)
    fprintf('Generating %d randomized launches.\n', config.launchCount)
    inputs = generate_rocket_inputs(config);
    sampleCount = min(config.trajectorySampleCount, config.launchCount);
    sampleIndices = sort(randperm(config.launchCount, sampleCount))';
    fprintf('Integrating trajectories with %.3f s time steps.\n', config.timeStep_s)
    [flightResults, trajectorySample] = simulate_rocket_batch(config, inputs, sampleIndices);
    fprintf('Performing engineering analysis.\n')
    [analysis, flightResults] = analyze_rocket_results(config, inputs, flightResults, trajectorySample);
    representativeTrajectories = struct;
    representativeNames = {'bestOverall', 'worstOverall', 'medianApogee', 'medianRange'};
    representativeConfig = config;
    representativeConfig.showProgress = false;
    for trajectoryIndex = 1:numel(representativeNames)
        representativeTrajectories.(representativeNames{trajectoryIndex}) = simulate_rocket_single(representativeConfig, inputs, analysis.trajectorySelection.indices(trajectoryIndex));
    end
    artifacts = struct;
    artifacts.outputDirectory = outputDirectory;
    artifacts.resultsFile = resultsFilePath;
    artifacts.summaryReport = fullfile(outputDirectory, 'engineering_summary_report.txt');
    if config.makeFigures
        fprintf('Generating figures.\n')
        artifacts.figureFiles = create_rocket_figures(config, outputDirectory, inputs, flightResults, analysis, trajectorySample, representativeTrajectories);
    else
        artifacts.figureFiles = struct;
    end
    endedAt = datetime('now');
    runMetadata = struct;
    runMetadata.schemaVersion = '1.0';
    runMetadata.startedAt = char(startedAt);
    runMetadata.endedAt = char(endedAt);
    runMetadata.elapsedSeconds = seconds(endedAt - startedAt);
    runMetadata.matlabVersion = version;
    runMetadata.platform = computer;
    runMetadata.randomGenerator = 'twister';
    runMetadata.dataRetention = 'All 100,000 per-launch sampled input parameters and calculated output metrics are retained. Full time histories are retained for the sampled trajectory ensemble and four detailed representative flights.';
    simulationResults = struct;
    simulationResults.metadata = runMetadata;
    simulationResults.config = config;
    simulationResults.inputParameters = inputs;
    simulationResults.flightResults = flightResults;
    simulationResults.analysis = analysis;
    simulationResults.trajectorySample = trajectorySample;
    simulationResults.representativeTrajectories = representativeTrajectories;
    simulationResults.artifacts = artifacts;
    write_rocket_report(outputDirectory, config, inputs, flightResults, analysis, resultsFilePath);
    save(resultsFilePath, 'simulationResults', '-v7.3');
    if config.openResultsOnCompletion
        try
            open_rocket_results(resultsFilePath, artifacts.figureFiles, config.openFigureDashboardOnCompletion);
        catch viewerError
            warning('RocketMonteCarlo:ViewerFailure', 'Results were saved, but the automatic viewer could not open: %s', viewerError.message)
        end
    end
    fprintf('Completed in %.1f s.\n', runMetadata.elapsedSeconds)
    fprintf('Results file: %s\n', resultsFilePath)
    fprintf('Summary report: %s\n', artifacts.summaryReport)
catch errorInfo
    fileIdentifier = fopen(failureLogPath, 'w');
    if fileIdentifier >= 0
        fprintf(fileIdentifier, 'Simulation failed at %s\n\n%s\n', char(datetime('now')), getReport(errorInfo, 'extended', 'hyperlinks', 'off'));
        fclose(fileIdentifier);
    end
    rethrow(errorInfo)
end
end
