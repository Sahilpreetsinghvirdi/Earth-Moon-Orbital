function simulationResults = run_earth_moon_monte_carlo(trialCount)
baseDirectory = fileparts(mfilename('fullpath'));
if isempty(baseDirectory)
    baseDirectory = pwd;
end
addpath(baseDirectory)
config = earth_moon_orbital_config();
if nargin >= 1 && ~isempty(trialCount)
    config.trialCount = max(1, round(trialCount));
end
outputDirectory = fullfile(baseDirectory, config.outputDirectoryName);
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
resultsFilePath = fullfile(outputDirectory, config.resultsFileName);
startedAt = datetime('now');
rng(config.randomSeed, 'twister')
inputs = generate_earth_moon_inputs(config);
fprintf('%s\n', config.simulationName)
fprintf('Trials: %d, maximum duration: %.2f days\n', config.trialCount, config.maxSimulationTime_s / 86400)
trialResults = struct([]);
trajectoryOverview = struct([]);
progressInterval = max(1, ceil(config.trialCount / config.progressIntervals));
for trialIndex = 1:config.trialCount
    input = earth_moon_trial_input(inputs, trialIndex);
    [trialResult, trialTrajectory] = simulate_earth_moon_trial(config, input);
    if trialIndex == 1
        trialResults = trialResult;
        trajectoryOverview = trialTrajectory;
    else
        trialResults(trialIndex, 1) = trialResult;
        trajectoryOverview(trialIndex, 1) = trialTrajectory;
    end
    if config.showProgress && (mod(trialIndex, progressInterval) == 0 || trialIndex == config.trialCount)
        fprintf('Completed trial %d / %d\n', trialIndex, config.trialCount)
    end
end
analysis = analyze_earth_moon_results(config, trialResults);
bestTrialIndex = analysis.bestTrialIndex;
bestInput = earth_moon_trial_input(inputs, bestTrialIndex);
[bestTrialResult, bestTrajectory] = simulate_earth_moon_trial(config, bestInput, [], [], true);
trialResults(bestTrialIndex, 1) = bestTrialResult;
endedAt = datetime('now');
artifacts = struct;
artifacts.outputDirectory = outputDirectory;
artifacts.resultsFile = resultsFilePath;
artifacts.summaryReport = fullfile(outputDirectory, 'earth_moon_summary_report.txt');
simulationResults = struct;
simulationResults.metadata = struct('schemaVersion', '1.0', 'startedAt', char(startedAt), 'endedAt', char(endedAt), 'elapsedSeconds', seconds(endedAt - startedAt), 'matlabVersion', version, 'platform', computer, 'randomGenerator', 'twister');
simulationResults.config = config;
simulationResults.inputs = inputs;
simulationResults.trialResults = trialResults;
simulationResults.trajectoryOverview = trajectoryOverview;
simulationResults.analysis = analysis;
simulationResults.bestTrialIndex = bestTrialIndex;
simulationResults.bestTrajectory = bestTrajectory;
simulationResults.artifacts = artifacts;
if config.makeFigures
    simulationResults.artifacts.figureFiles = create_earth_moon_figures(config, outputDirectory, simulationResults);
else
    simulationResults.artifacts.figureFiles = struct;
end
simulationResults.artifacts.summaryReport = write_earth_moon_report(outputDirectory, simulationResults);
save(resultsFilePath, 'simulationResults', '-v7.3')
fprintf('Completed in %.1f s. Best trial: %d (%s)\n', simulationResults.metadata.elapsedSeconds, bestTrialIndex, bestTrialResult.classification)
fprintf('Results file: %s\n', resultsFilePath)
end
