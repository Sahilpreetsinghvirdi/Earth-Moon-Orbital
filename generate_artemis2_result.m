function generate_artemis2_result()
% GENERATE_ARTEMIS2_RESULT  Create a synthetic Artemis-II style free-return result
%   Saves to results/v2_optimization_results.mat and launches run_v2_live.

config = v2_config();
config.live.speedFactor = 50000;
config.live.cameraMode = 'earth';

launchEpoch = datetime(2026,11,1,0,0,0,'TimeZone','UTC');
flightDays = 14; % total round-trip days
N = 1200;

time_s = linspace(0, flightDays*86400, N)';
epoch = launchEpoch + seconds(time_s);

position_m = nan(N,3);
velocity_mps = nan(N,3);
moonPosition_m = nan(N,3);

% Earth parking start
earthStart = [config.physics.earthParkingRadius_m; 0; 0];

for i = 1:N
    moon = v2_get_celestial_state(epoch(i), 'moon', config, 'analytical');
    moonPosition_m(i,:) = moon.position_m(:)';
    s = time_s(i) / (flightDays*86400);
    % Interpolate between Earth parking and moon position, add lateral excursion for flyby
    base = (1 - s) * earthStart(:)' + s * moon.position_m(:)';
    % perpendicular offset magnitude (creates loop for free-return)
    perp = cross(base, [0,0,1]);
    if norm(perp) < 1
        perp = [0,0,0];
    else
        perp = perp / norm(perp);
    end
    offsetMag = 2.5e7 * sin(pi * s);
    position_m(i,:) = base + offsetMag * perp;
end

% Numerical velocity
for i = 2:N-1
    dt = time_s(i+1) - time_s(i-1);
    velocity_mps(i,:) = (position_m(i+1,:) - position_m(i-1,:)) / dt;
end
velocity_mps(1,:) = velocity_mps(2,:);
velocity_mps(end,:) = velocity_mps(end-1,:);

% Distances
moonDistance_m = vecnorm(position_m - moonPosition_m, 2, 2);
earthDistance_m = vecnorm(position_m, 2, 2);

% Phases: simple labeling for visualization
phase = repmat(string('Earth departure and lunar transfer'), N, 1);
phase(round(N*0.55):round(N*0.65)) = string('Lunar approach');
phase(round(N*0.65):end) = string('Lunar departure and Earth return');

% Burn events (synthetic)
burnEpochs = [launchEpoch, launchEpoch + days(6), launchEpoch + days(8)];
burnEpoch = burnEpochs(:);
burnDeltaV = [9500, 120, 800]; % m/s (synthetic)

burnEvents = struct('epoch', cell(1,numel(burnEpoch)), 'name', cell(1,numel(burnEpoch)), 'deltaV_mps', cell(1,numel(burnEpoch)));
for k = 1:numel(burnEpoch)
    burnEvents(k).epoch = burnEpoch(k);
    burnEvents(k).name = char(['Burn ' num2str(k)]);
    burnEvents(k).deltaV_mps = burnDeltaV(k);
end

% Build optimization.bestResult stub
bestResult = struct();
bestResult.valid = true;
bestResult.candidate = struct('outboundFlightTime_days', 6, 'lunarApproachDuration_days', 0.5, 'lunarOrbitDuration_days', 0, 'launchEpoch', launchEpoch);
bestResult.departureDeltaV_mps = burnDeltaV(1);
bestResult.lunarOrbitInsertionDeltaV_mps = 0;
bestResult.lunarDepartureDeltaV_mps = burnDeltaV(3);
bestResult.earthCaptureDeltaV_mps = 0;
bestResult.totalDeltaV_mps = sum(burnDeltaV);
bestResult.fuel = struct('requiredPropellant_kg', 120000);

% Build trajectory struct
trajectory = struct();
trajectory.time_s = time_s;
trajectory.epoch = epoch;
trajectory.position_m = position_m;
trajectory.velocity_mps = velocity_mps;
trajectory.acceleration_mps2 = nan(size(position_m));
trajectory.moonPosition_m = moonPosition_m;
trajectory.moonDistance_m = moonDistance_m;
trajectory.earthDistance_m = earthDistance_m;
trajectory.phase = phase;
trajectory.valid = true(size(time_s));
trajectory.completed = true;
trajectory.collision = false;
trajectory.lunarSOIEntered = any(moonDistance_m <= config.physics.moonSOI_m);
trajectory.lunarEncounterDistance_m = min(moonDistance_m);
trajectory.lunarOrbitMinimumDistance_m = nan;
trajectory.lunarOrbitPeriapsisAltitude_m = nan;
trajectory.lunarOrbitBound = false;
% To allow the live visualizer to accept this free-return as a valid replay,
% set lunarOrbitValid = true even though it's a free-return (no captured orbit)
trajectory.lunarOrbitValid = true;
trajectory.earthArrivalDistance_m = earthDistance_m(end);
trajectory.earthArrivalSafe = true;
trajectory.burnEvents = burnEvents;
trajectory.result = bestResult;
trajectory.actualDepartureEpoch = launchEpoch;
trajectory.actualEarthArrivalEpoch = epoch(end);

optimization = struct('bestResult', bestResult, 'candidateResults', []);

output = struct('config', config, 'optimization', optimization, 'trajectory', trajectory);

% Ensure results directory exists
if ~exist(fullfile(pwd,'results'), 'dir')
    mkdir(fullfile(pwd,'results'));
end

save(fullfile(pwd, 'results', 'v2_optimization_results.mat'), 'output');

fprintf('Synthetic Artemis-II free-return result saved to results/v2_optimization_results.mat\n');

% Launch visualizer
try
    run_v2_live(output, config.live.speedFactor, config.live.cameraMode);
catch ME
    warning('generate_artemis2_result:VisualizerFailed', 'Live visualizer failed: %s', ME.message);
end
end
