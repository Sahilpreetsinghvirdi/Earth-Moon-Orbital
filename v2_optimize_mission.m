function optimization = v2_optimize_mission(config, previousState)
startedAt = datetime('now', 'TimeZone', 'UTC');
rng(config.randomSeed, 'twister')
optimization = struct;
optimization.schemaVersion = '2.0';
optimization.startedAt = startedAt;
optimization.ephemerisSource = config.ephemeris.source;
optimization.candidateResults = {};
optimization.history = empty_history();
optimization.bestResult = [];
optimization.iterations = 0;
optimization.candidatesTested = 0;
optimization.successfulCandidates = 0;
optimization.converged = false;
optimization.stopReason = 'Not started';
optimization.usedParallel = false;
if nargin >= 2 && ~isempty(previousState) && isfield(previousState, 'bestResult') && ~isempty(previousState.bestResult)
    optimization.bestResult = previousState.bestResult;
end
phaseDefinitions = build_phase_definitions(config);
for phaseIndex = 1:numel(phaseDefinitions)
    phase = phaseDefinitions(phaseIndex);
    candidates = build_candidates(phase, config, optimization.bestResult);
    [phaseResults, usedParallel] = evaluate_batch(candidates, config);
    optimization.usedParallel = optimization.usedParallel || usedParallel;
    optimization.candidateResults = [optimization.candidateResults; phaseResults];
    optimization.candidatesTested = optimization.candidatesTested + numel(phaseResults);
    validFlags = cellfun(@(entry) entry.valid, phaseResults);
    optimization.successfulCandidates = optimization.successfulCandidates + nnz(validFlags);
    for candidateIndex = 1:numel(phaseResults)
        optimization.iterations = optimization.iterations + 1;
        candidateResult = phaseResults{candidateIndex};
        if candidateResult.valid && (isempty(optimization.bestResult) || candidateResult.score > optimization.bestResult.score)
            previousBestDeltaV_mps = inf;
            if ~isempty(optimization.bestResult) && isfield(optimization.bestResult, 'totalDeltaV_mps')
                previousBestDeltaV_mps = optimization.bestResult.totalDeltaV_mps;
            end
            optimization.bestResult = candidateResult;
            if abs(previousBestDeltaV_mps - candidateResult.totalDeltaV_mps) <= config.mission.convergenceTolerance_mps
                optimization.converged = true;
            end
        end
        optimization.history = append_history(optimization.history, optimization.iterations, optimization.candidatesTested, optimization.successfulCandidates, optimization.bestResult, startedAt);
        if config.optimizer.showProgress && (mod(optimization.iterations, max(1, floor(config.mission.maxCandidates / 20))) == 0 || candidateIndex == numel(phaseResults))
            print_progress(phase.name, optimization, startedAt)
        end
        if should_stop(optimization, startedAt, config)
            optimization.stopReason = stop_reason(optimization, startedAt, config);
            optimization.endedAt = datetime('now', 'TimeZone', 'UTC');
            optimization = finalize_optimization(optimization);
            return
        end
    end
    if optimization.converged && phaseIndex == numel(phaseDefinitions)
        optimization.stopReason = 'Convergence tolerance reached';
        break
    end
end
if isempty(optimization.bestResult)
    optimization.stopReason = 'No candidate completed';
elseif isempty(optimization.stopReason) || strcmp(optimization.stopReason, 'Not started')
    optimization.stopReason = 'Search phases completed';
end
optimization.endedAt = datetime('now', 'TimeZone', 'UTC');
optimization = finalize_optimization(optimization);
optimization.elapsedSeconds = seconds(optimization.endedAt - startedAt);
end

function optimization = finalize_optimization(optimization)
optimization.bestLaunchDate = [];
optimization.bestDeltaV_mps = nan;
optimization.bestFuel_kg = nan;
if ~isempty(optimization.bestResult)
    optimization.bestLaunchDate = optimization.bestResult.candidate.launchEpoch;
    optimization.bestDeltaV_mps = optimization.bestResult.totalDeltaV_mps;
    optimization.bestFuel_kg = optimization.bestResult.fuel.requiredPropellant_kg;
end
end

function phases = build_phase_definitions(config)
phases = struct('name', {'coarse', 'refine', 'final'}, 'step_days', {config.mission.coarseStep_days, config.mission.refineStep_hours / 24, config.mission.finalStep_minutes / 1440}, 'radius_days', {0, config.mission.refinementRadius_days, 3});
end

function candidates = build_candidates(phase, config, bestResult)
if phase.radius_days == 0 || isempty(bestResult)
    dates = config.searchStartEpoch:days(phase.step_days):config.searchEndEpoch;
else
    center = bestResult.candidate.launchEpoch;
    startDate = max(config.searchStartEpoch, center - days(phase.radius_days));
    endDate = min(config.searchEndEpoch, center + days(phase.radius_days));
    dates = startDate:days(phase.step_days):endDate;
end
outboundTimes = config.mission.outboundFlightTime_days;
returnTimes = config.mission.returnFlightTime_days;
orbitAltitudes_m = config.mission.minimumLunarPeriapsisAltitude_m + [0, 100000, 250000];
orbitDuration_days = config.mission.lunarOrbitDuration_days;
if strcmp(phase.name, 'coarse')
    outboundTimes = outboundTimes(ceil(numel(outboundTimes) / 2));
    returnTimes = returnTimes(ceil(numel(returnTimes) / 2));
    orbitAltitudes_m = config.physics.moonParkingAltitude_m;
end
count = numel(dates) * numel(outboundTimes) * numel(returnTimes) * numel(orbitAltitudes_m);
phaseCapacity = max(1, floor(config.mission.maxCandidates / 3));
candidates = repmat(struct('launchEpoch', dates(1), 'outboundFlightTime_days', 0, 'returnFlightTime_days', 0, 'lunarOrbitDuration_days', orbitDuration_days, 'lunarOrbitAltitude_m', 0), min(count, phaseCapacity), 1);
dateCapacity = max(1, floor(numel(candidates) / max(numel(outboundTimes) * numel(returnTimes) * numel(orbitAltitudes_m), 1)));
dateIndices = unique(round(linspace(1, numel(dates), min(numel(dates), dateCapacity))));
writeIndex = 0;
for dateIndex = dateIndices
    for outboundIndex = 1:numel(outboundTimes)
        for returnIndex = 1:numel(returnTimes)
            for altitudeIndex = 1:numel(orbitAltitudes_m)
                if writeIndex >= numel(candidates)
                    break
                end
                writeIndex = writeIndex + 1;
                candidates(writeIndex).launchEpoch = dates(dateIndex);
                candidates(writeIndex).outboundFlightTime_days = outboundTimes(outboundIndex);
                candidates(writeIndex).returnFlightTime_days = returnTimes(returnIndex);
                candidates(writeIndex).lunarOrbitDuration_days = orbitDuration_days;
                candidates(writeIndex).lunarOrbitAltitude_m = orbitAltitudes_m(altitudeIndex);
            end
            if writeIndex >= numel(candidates)
                break
            end
        end
        if writeIndex >= numel(candidates)
            break
        end
    end
    if writeIndex >= numel(candidates)
        break
    end
end
candidates = candidates(1:writeIndex);
end

function [results, usedParallel] = evaluate_batch(candidates, config)
results = cell(numel(candidates), 1);
usedParallel = false;
parallelAvailable = config.optimizer.useParallel && license('test', 'Distrib_Computing_Toolbox');
if parallelAvailable
    try
        parfor index = 1:numel(candidates)
            results{index} = v2_evaluate_candidate(candidates(index), config);
        end
        usedParallel = true;
        return
    catch
        usedParallel = false;
    end
end
for index = 1:numel(candidates)
    results{index} = v2_evaluate_candidate(candidates(index), config);
end
end

function history = empty_history()
history = struct('iteration', zeros(0, 1), 'candidatesTested', zeros(0, 1), 'successfulCandidates', zeros(0, 1), 'bestScore', zeros(0, 1), 'bestDeltaV_mps', zeros(0, 1), 'bestFuel_kg', zeros(0, 1), 'launchEpoch', NaT(0, 1, 'TimeZone', 'UTC'), 'elapsedSeconds', zeros(0, 1));
end

function history = append_history(history, iteration, candidatesTested, successfulCandidates, bestResult, startedAt)
history.iteration(end + 1, 1) = iteration;
history.candidatesTested(end + 1, 1) = candidatesTested;
history.successfulCandidates(end + 1, 1) = successfulCandidates;
if isempty(bestResult)
    history.bestScore(end + 1, 1) = -inf;
    history.bestDeltaV_mps(end + 1, 1) = nan;
    history.bestFuel_kg(end + 1, 1) = nan;
    history.launchEpoch(end + 1, 1) = NaT;
else
    history.bestScore(end + 1, 1) = bestResult.score;
    history.bestDeltaV_mps(end + 1, 1) = bestResult.totalDeltaV_mps;
    history.bestFuel_kg(end + 1, 1) = bestResult.fuel.requiredPropellant_kg;
    history.launchEpoch(end + 1, 1) = bestResult.candidate.launchEpoch;
end
history.elapsedSeconds(end + 1, 1) = seconds(datetime('now', 'TimeZone', 'UTC') - startedAt);
end

function value = should_stop(optimization, startedAt, config)
value = optimization.candidatesTested >= config.mission.maxCandidates || seconds(datetime('now', 'TimeZone', 'UTC') - startedAt) >= config.mission.maxRuntime_s || optimization.iterations >= config.optimizer.maxIterations;
end

function reason = stop_reason(optimization, startedAt, config)
if optimization.candidatesTested >= config.mission.maxCandidates
    reason = 'Maximum candidates reached';
elseif optimization.iterations >= config.optimizer.maxIterations
    reason = 'Maximum iterations reached';
elseif seconds(datetime('now', 'TimeZone', 'UTC') - startedAt) >= config.mission.maxRuntime_s
    reason = 'Maximum runtime reached';
else
    reason = 'Search stopped';
end
end

function print_progress(phaseName, optimization, startedAt)
if isempty(optimization.bestResult)
    fprintf('%s phase: %d candidates, no valid mission yet\n', phaseName, optimization.candidatesTested)
else
    fprintf('%s phase: %d candidates, valid %d, best %.1f m/s, %.1f kg fuel, elapsed %.1f s\n', phaseName, optimization.candidatesTested, optimization.successfulCandidates, optimization.bestResult.totalDeltaV_mps, optimization.bestResult.fuel.requiredPropellant_kg, seconds(datetime('now', 'TimeZone', 'UTC') - startedAt))
end
end
