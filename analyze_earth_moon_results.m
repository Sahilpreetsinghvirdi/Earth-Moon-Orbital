function analysis = analyze_earth_moon_results(config, trialResults)
count = numel(trialResults);
analysis = struct;
analysis.totalTrials = count;
analysis.successfulTrials = nnz([trialResults.lunarOrbit]);
analysis.successRate_percent = 100 * analysis.successfulTrials / max(count, 1);
analysis.moonEncounterCount = nnz([trialResults.moonEncounter]);
analysis.lunarCaptureCount = nnz([trialResults.lunarCapture]);
analysis.lunarOrbitCount = nnz([trialResults.lunarOrbit]);
analysis.earthEscapeCount = nnz([trialResults.earthEscape]);
analysis.earthImpactCount = nnz([trialResults.earthImpact]);
analysis.lunarImpactCount = nnz([trialResults.lunarImpact]);
analysis.classifications = {'Lunar orbit', 'Temporary lunar capture', 'Moon encounter', 'Lunar impact', 'Earth impact', 'Earth escape', 'No lunar encounter', 'Stopped'};
analysis.classificationCounts = zeros(numel(analysis.classifications), 1);
for classIndex = 1:numel(analysis.classifications)
    analysis.classificationCounts(classIndex) = nnz(strcmp({trialResults.classification}, analysis.classifications{classIndex}));
end
metricNames = {'maximumAltitude_m', 'maximumVelocity_mps', 'maximumDynamicPressure_Pa', 'minimumMoonDistance_m', 'minimumMoonRelativeVelocity_mps', 'flightTime_s', 'maximumCaptureDuration_s', 'finalMoonSpecificEnergy_Jkg'};
analysis.metricNames = metricNames;
analysis.metricStatistics = struct;
for metricIndex = 1:numel(metricNames)
    metricName = metricNames{metricIndex};
    analysis.metricStatistics.(metricName) = metric_statistics([trialResults.(metricName)]');
end
minimumMoonDistance_m = [trialResults.minimumMoonDistance_m]';
minimumMoonRelativeVelocity_mps = [trialResults.minimumMoonRelativeVelocity_mps]';
score = -minimumMoonDistance_m / 1e6 - minimumMoonRelativeVelocity_mps / 1000;
score([trialResults.earthEscape]) = score([trialResults.earthEscape]) + 200;
score([trialResults.moonEncounter]) = score([trialResults.moonEncounter]) + 1000;
score([trialResults.lunarCapture]) = score([trialResults.lunarCapture]) + 10000;
score([trialResults.lunarOrbit]) = score([trialResults.lunarOrbit]) + 100000;
score([trialResults.lunarImpact]) = score([trialResults.lunarImpact]) + 3000;
[analysis.bestScore, analysis.bestTrialIndex] = max(score);
analysis.performanceScore = score;
analysis.configuredLunarSOI_m = config.moon.sphereOfInfluence_m;
end

function statistics = metric_statistics(values)
values = sort(values(isfinite(values)));
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
