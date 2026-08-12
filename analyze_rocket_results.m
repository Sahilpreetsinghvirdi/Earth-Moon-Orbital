function [analysis, flightResults] = analyze_rocket_results(config, inputs, flightResults, trajectorySample)
n = config.launchCount;
terminalFlight = flightResults.impactOccurred & ~flightResults.numericalFailure & ~flightResults.timedOut;
criteria = struct;
criteria.apogeeTooLow = flightResults.apogee_m < config.mission.minApogee_m;
criteria.apogeeTooHigh = flightResults.apogee_m > config.mission.maxApogee_m;
criteria.rangeTooShort = flightResults.downrange_m < config.mission.minRange_m;
criteria.rangeTooLong = flightResults.downrange_m > config.mission.maxRange_m;
criteria.dynamicPressureExceeded = flightResults.maximumDynamicPressure_Pa > config.mission.maxDynamicPressure_Pa;
criteria.accelerationExceeded = flightResults.maximumAcceleration_mps2 > config.mission.maxAcceleration_mps2;
criteria.noImpact = ~flightResults.impactOccurred;
criteria.numericalFailure = flightResults.numericalFailure;
criteria.timedOut = flightResults.timedOut;
missionSuccess = terminalFlight & ~criteria.apogeeTooLow & ~criteria.apogeeTooHigh & ~criteria.rangeTooShort & ~criteria.rangeTooLong & ~criteria.dynamicPressureExceeded & ~criteria.accelerationExceeded;
flightResults.missionSuccess = missionSuccess;
flightResults.missionFailure = ~missionSuccess;
analysis = struct;
analysis.missionCriteria = config.mission;
analysis.totalLaunches = n;
analysis.successCount = nnz(missionSuccess);
analysis.failureCount = n - analysis.successCount;
analysis.successRate_percent = 100 * analysis.successCount / n;
analysis.terminalFlightCount = nnz(terminalFlight);
analysis.terminalFlightRate_percent = 100 * analysis.terminalFlightCount / n;
analysis.failureCriteria = criteria;
analysis.failureCounts = struct;
criteriaNames = fieldnames(criteria);
for criterionIndex = 1:numel(criteriaNames)
    criterionName = criteriaNames{criterionIndex};
    analysis.failureCounts.(criterionName) = nnz(criteria.(criterionName));
end
metricNames = {'apogee_m', 'downrange_m', 'maximumVelocity_mps', 'maximumAcceleration_mps2', 'maximumDynamicPressure_Pa', 'flightTime_s', 'fuelConsumed_kg', 'impactSpeed_mps'};
metricLabels = {'Apogee (m)', 'Downrange (m)', 'Maximum velocity (m/s)', 'Maximum acceleration (m/s^2)', 'Maximum dynamic pressure (Pa)', 'Flight time (s)', 'Fuel consumed (kg)', 'Impact speed (m/s)'};
analysis.metricNames = metricNames;
analysis.metricLabels = metricLabels;
analysis.allFlightStatistics = struct;
analysis.successFlightStatistics = struct;
for metricIndex = 1:numel(metricNames)
    metricName = metricNames{metricIndex};
    analysis.allFlightStatistics.(metricName) = metric_statistics(flightResults.(metricName), terminalFlight);
    analysis.successFlightStatistics.(metricName) = metric_statistics(flightResults.(metricName), missionSuccess);
end
analysis.inputStatistics = struct;
inputNames = {'launchAngle_deg', 'initialMass_kg', 'propellantMass_kg', 'burnRate_kgps', 'specificImpulse_s', 'thrust_N', 'dragCoefficient', 'referenceArea_m2', 'densityMultiplier', 'scaleHeight_m', 'temperature_K', 'windSpeed_mps', 'ballisticCoefficient_kgpm2'};
for inputIndex = 1:numel(inputNames)
    inputName = inputNames{inputIndex};
    analysis.inputStatistics.(inputName) = metric_statistics(inputs.(inputName), true(n, 1));
end
analysis.sensitivity = sensitivity_analysis(inputs, flightResults, terminalFlight);
analysis.trajectoryEnvelope = make_trajectory_envelope(trajectorySample, missionSuccess);
analysis.performanceScore = performance_score(config, flightResults, missionSuccess, terminalFlight);
analysis.trajectorySelection = select_trajectories(flightResults, missionSuccess, terminalFlight, analysis.performanceScore);
analysis.successIndices = find(missionSuccess);
analysis.failureIndices = find(~missionSuccess);
analysis.terminalFlight = terminalFlight;
end

function statistics = metric_statistics(values, selection)
values = values(selection & isfinite(values));
statistics = struct;
statistics.count = numel(values);
if isempty(values)
    statistics.mean = nan;
    statistics.standardDeviation = nan;
    statistics.minimum = nan;
    statistics.p05 = nan;
    statistics.p50 = nan;
    statistics.p95 = nan;
    statistics.maximum = nan;
    return
end
values = sort(values(:));
statistics.mean = mean(values);
statistics.standardDeviation = std(values, 0);
statistics.minimum = values(1);
statistics.p05 = percentile_from_sorted(values, 5);
statistics.p50 = percentile_from_sorted(values, 50);
statistics.p95 = percentile_from_sorted(values, 95);
statistics.maximum = values(end);
end

function value = percentile_from_sorted(values, percentile)
position = 1 + (numel(values) - 1) * percentile / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);
if lowerIndex == upperIndex
    value = values(lowerIndex);
else
    value = values(lowerIndex) + (position - lowerIndex) * (values(upperIndex) - values(lowerIndex));
end
end

function sensitivity = sensitivity_analysis(inputs, flightResults, terminalFlight)
inputNames = {'launchAngle_deg', 'initialMass_kg', 'propellantMass_kg', 'burnRate_kgps', 'specificImpulse_s', 'thrust_N', 'dragCoefficient', 'referenceArea_m2', 'densityMultiplier', 'scaleHeight_m', 'temperature_K', 'windSpeed_mps', 'ballisticCoefficient_kgpm2'};
outputNames = {'apogee_m', 'downrange_m', 'maximumVelocity_mps', 'maximumAcceleration_mps2', 'maximumDynamicPressure_Pa'};
correlationMatrix = nan(numel(inputNames), numel(outputNames));
for inputIndex = 1:numel(inputNames)
    x = inputs.(inputNames{inputIndex});
    for outputIndex = 1:numel(outputNames)
        y = flightResults.(outputNames{outputIndex});
        valid = terminalFlight & isfinite(x) & isfinite(y);
        correlationMatrix(inputIndex, outputIndex) = pearson_correlation(x(valid), y(valid));
    end
end
maximumAbsoluteCorrelation = max(abs(correlationMatrix), [], 2);
[sortedInfluence, ranking] = sort(maximumAbsoluteCorrelation, 'descend');
sensitivity = struct;
sensitivity.inputNames = inputNames;
sensitivity.outputNames = outputNames;
sensitivity.pearsonCorrelation = correlationMatrix;
sensitivity.maximumAbsoluteCorrelation = maximumAbsoluteCorrelation;
sensitivity.ranking = ranking;
sensitivity.sortedInfluence = sortedInfluence;
end

function coefficient = pearson_correlation(x, y)
if numel(x) < 2
    coefficient = nan;
    return
end
x = x(:);
y = y(:);
x = x - mean(x);
y = y - mean(y);
denominator = sqrt(sum(x .^ 2) * sum(y .^ 2));
if denominator <= eps
    coefficient = nan;
else
    coefficient = sum(x .* y) / denominator;
end
end

function envelope = make_trajectory_envelope(trajectorySample, missionSuccess)
sampleSuccess = missionSuccess(trajectorySample.indices);
if nnz(sampleSuccess) < 20
    sampleSuccess = true(size(sampleSuccess));
end
envelope = struct;
envelope.sourceIndices = trajectorySample.indices(sampleSuccess);
envelope.time_s = trajectorySample.time_s;
envelope.activeCount = sum(~isnan(trajectorySample.altitude_m(:, sampleSuccess)), 2);
envelope.altitude_m = column_percentiles(trajectorySample.altitude_m(:, sampleSuccess));
envelope.downrange_m = column_percentiles(trajectorySample.downrange_m(:, sampleSuccess));
envelope.velocity_mps = column_percentiles(trajectorySample.velocity_mps(:, sampleSuccess));
envelope.acceleration_mps2 = column_percentiles(trajectorySample.acceleration_mps2(:, sampleSuccess));
envelope.dynamicPressure_Pa = column_percentiles(trajectorySample.dynamicPressure_Pa(:, sampleSuccess));
[envelope.altitudeMean_m, envelope.altitudeStandardDeviation_m] = column_mean_standard_deviation(trajectorySample.altitude_m(:, sampleSuccess));
[envelope.downrangeMean_m, envelope.downrangeStandardDeviation_m] = column_mean_standard_deviation(trajectorySample.downrange_m(:, sampleSuccess));
[envelope.velocityMean_mps, envelope.velocityStandardDeviation_mps] = column_mean_standard_deviation(trajectorySample.velocity_mps(:, sampleSuccess));
[envelope.accelerationMean_mps2, envelope.accelerationStandardDeviation_mps2] = column_mean_standard_deviation(trajectorySample.acceleration_mps2(:, sampleSuccess));
[envelope.dynamicPressureMean_Pa, envelope.dynamicPressureStandardDeviation_Pa] = column_mean_standard_deviation(trajectorySample.dynamicPressure_Pa(:, sampleSuccess));
end

function percentiles = column_percentiles(values)
rowCount = size(values, 1);
percentiles = nan(rowCount, 3);
for rowIndex = 1:rowCount
    rowValues = values(rowIndex, :);
    rowValues = sort(rowValues(isfinite(rowValues)));
    if ~isempty(rowValues)
        percentiles(rowIndex, 1) = percentile_from_sorted(rowValues, 5);
        percentiles(rowIndex, 2) = percentile_from_sorted(rowValues, 50);
        percentiles(rowIndex, 3) = percentile_from_sorted(rowValues, 95);
    end
end
end

function [meanValues, standardDeviationValues] = column_mean_standard_deviation(values)
rowCount = size(values, 1);
meanValues = nan(rowCount, 1);
standardDeviationValues = nan(rowCount, 1);
for rowIndex = 1:rowCount
    rowValues = values(rowIndex, :);
    rowValues = rowValues(isfinite(rowValues));
    if ~isempty(rowValues)
        meanValues(rowIndex) = mean(rowValues);
        if numel(rowValues) > 1
            standardDeviationValues(rowIndex) = std(rowValues, 0);
        else
            standardDeviationValues(rowIndex) = 0;
        end
    end
end
end

function score = performance_score(config, flightResults, missionSuccess, terminalFlight)
targetApogee_m = 0.5 * (config.mission.minApogee_m + config.mission.maxApogee_m);
targetRange_m = 0.5 * (config.mission.minRange_m + config.mission.maxRange_m);
apogeeQuality = max(-1, 1 - abs(flightResults.apogee_m - targetApogee_m) / targetApogee_m);
rangeQuality = max(-1, 1 - abs(flightResults.downrange_m - targetRange_m) / targetRange_m);
pressureQuality = max(-1, 1 - flightResults.maximumDynamicPressure_Pa / config.mission.maxDynamicPressure_Pa);
accelerationQuality = max(-1, 1 - flightResults.maximumAcceleration_mps2 / config.mission.maxAcceleration_mps2);
score = 100 * (0.45 * apogeeQuality + 0.35 * rangeQuality + 0.10 * pressureQuality + 0.10 * accelerationQuality);
score(~terminalFlight) = -500;
score(~missionSuccess & terminalFlight) = score(~missionSuccess & terminalFlight) - 100;
end

function selection = select_trajectories(flightResults, missionSuccess, terminalFlight, score)
validForRepresentative = find(missionSuccess);
if isempty(validForRepresentative)
    validForRepresentative = find(terminalFlight);
end
if isempty(validForRepresentative)
    validForRepresentative = 1:numel(score);
end
[~, bestIndex] = max(score);
[~, worstIndex] = min(score);
medianApogee = median(flightResults.apogee_m(validForRepresentative));
[~, localMedianApogee] = min(abs(flightResults.apogee_m(validForRepresentative) - medianApogee));
medianApogeeIndex = validForRepresentative(localMedianApogee);
medianRange = median(flightResults.downrange_m(validForRepresentative));
[~, localMedianRange] = min(abs(flightResults.downrange_m(validForRepresentative) - medianRange));
medianRangeIndex = validForRepresentative(localMedianRange);
selection = struct;
selection.labels = {'Best overall performance', 'Worst overall performance', 'Median apogee representative', 'Median range representative'};
selection.indices = [bestIndex; worstIndex; medianApogeeIndex; medianRangeIndex];
selection.performanceScores = score(selection.indices);
end
