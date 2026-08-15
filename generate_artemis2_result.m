function generate_artemis2_result()
% GENERATE_ARTEMIS2_RESULT  Create a synthetic Artemis-II style mission result
%   This generator now synthesizes a mission that departs Earth, performs a
%   lunar approach, inserts into a short lunar orbit (revolves around the
%   Moon), then departs and returns to Earth for a safe arrival.  The
%   produced output is intended for visualization and teaching only.

config = v2_config();
config.live.speedFactor = 50000;
config.live.cameraMode = 'earth';

% Mission timing
launchEpoch = datetime(2026,11,1,0,0,0,'TimeZone','UTC');
outboundDays = 5;         % Earth->Moon transfer days
orbitDays = 1.5;           % days spent in lunar orbit (short)
returnDays = 6;           % Moon->Earth transfer days
flightDays = outboundDays + orbitDays + returnDays;
N = 1400;

% Time base
time_s = linspace(0, flightDays*86400, N)';
epoch = launchEpoch + seconds(time_s);

% Preallocate
position_m = nan(N,3);
velocity_mps = nan(N,3);
moonPosition_m = nan(N,3);

% Indices for phases
outbound_end = round(N * (outboundDays / flightDays));
orbit_start = outbound_end + 1;
orbit_end = round(N * ((outboundDays + orbitDays) / flightDays));
return_start = orbit_end + 1;

% Earth parking start position (simple planar starting point)
earthStart = [config.physics.earthParkingRadius_m; 0; 0];

% Choose an approach offset relative to Moon (periapsis vector)
approachOffset_m = -config.mission.lunarApproachStartDistance_m / 3; % closer than start distance

% Lunar orbit parameters (relative to Moon)
lunarOrbitAltitude_m = 200e3; % 200 km circular lunar orbit
lunarOrbitRadius_m = config.physics.moonRadius_m + lunarOrbitAltitude_m;

% Build outbound segment (Earth to lunar approach)
for i = 1:outbound_end
    moon = v2_get_celestial_state(epoch(i), 'moon', config, 'analytical');
    moonPosition_m(i,:) = moon.position_m(:)';
    t = (i-1) / max(1, outbound_end-1);
    % Interpolate in inertial frame from Earth parking to a point near the moon
    approachPoint = moon.position_m(:)' + (approachOffset_m) * unit_vector(moon.position_m(:)');
    position_m(i,:) = (1 - t) * earthStart(:)' + t * approachPoint;
end

% Build lunar orbit segment (circular about the Moon in the Moon plane)
for i = orbit_start:orbit_end
    moon = v2_get_celestial_state(epoch(i), 'moon', config, 'analytical');
    moonPosition_m(i,:) = moon.position_m(:)';
    % theta around the Moon for one-plus revolutions depending on orbitDays
    orbitFraction = (i - orbit_start) / max(1, orbit_end - orbit_start + 1);
    % perform ~1.5 revolutions during the orbit window
    theta = 2 * pi * (1.5 * orbitFraction);
    localPos = lunarOrbitRadius_m * [cos(theta); sin(theta); 0];
    position_m(i,:) = (moon.position_m(:) + localPos)';
end

% Build return segment (Moon to Earth)
for i = return_start:N
    moon = v2_get_celestial_state(epoch(i), 'moon', config, 'analytical');
    moonPosition_m(i,:) = moon.position_m(:)';
    t = (i - return_start) / max(1, N - return_start);
    % Depart from a point near the Moon and interpolate back toward Earth parking
    departPoint = moon.position_m(:)' + [lunarOrbitRadius_m + 1e6, 0, 0];
    position_m(i,:) = (1 - t) * departPoint + t * earthStart(:)';
end

% Fill any remaining moonPosition_m entries
for i = 1:N
    if all(isnan(moonPosition_m(i,:)))
        moon = v2_get_celestial_state(epoch(i), 'moon', config, 'analytical');
        moonPosition_m(i,:) = moon.position_m(:)';
    end
end

% Numerical velocity estimate (central difference)
for i = 2:N-1
    dt = time_s(i+1) - time_s(i-1);
    velocity_mps(i,:) = (position_m(i+1,:) - position_m(i-1,:)) / dt;
end
velocity_mps(1,:) = velocity_mps(2,:);
velocity_mps(end,:) = velocity_mps(end-1,:);

% Distances
moonDistance_m = vecnorm(position_m - moonPosition_m, 2, 2);
earthDistance_m = vecnorm(position_m, 2, 2);

% Phase labels for visualization
phase = repmat(string('Earth departure and lunar transfer'), N, 1);
phase(orbit_start:orbit_end) = string('Lunar orbit');
phase(orbit_end+1:end) = string('Lunar departure and Earth return');

% Synthetic burn events (insertion and departure)
burnEpochs = [launchEpoch, epoch(outbound_end), epoch(orbit_end)];
burnDeltaV = [9500, 110, 900]; % m/s (departure, insertion, departure)

burnEvents = struct('epoch', cell(1,numel(burnEpochs)), 'name', cell(1,numel(burnEpochs)), 'deltaV_mps', cell(1,numel(burnEpochs)));
for k = 1:numel(burnEpochs)
    burnEvents(k).epoch = burnEpochs(k);
    burnEvents(k).name = char(['Burn ' num2str(k)]);
    burnEvents(k).deltaV_mps = burnDeltaV(k);
end

% Build optimization.bestResult stub (reflects lunar orbit)
bestResult = struct();
bestResult.valid = true;
bestResult.candidate = struct('outboundFlightTime_days', outboundDays, 'lunarApproachDuration_days', 0.5, 'lunarOrbitDuration_days', orbitDays, 'launchEpoch', launchEpoch);
bestResult.departureDeltaV_mps = burnDeltaV(1);
bestResult.lunarOrbitInsertionDeltaV_mps = burnDeltaV(2);
bestResult.lunarDepartureDeltaV_mps = burnDeltaV(3);
bestResult.earthCaptureDeltaV_mps = 0;
bestResult.totalDeltaV_mps = sum(burnDeltaV);
bestResult.fuel = struct('requiredPropellant_kg', 150000);

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
trajectory.lunarOrbitMinimumDistance_m = min(trajectory.moonDistance_m(orbit_start:orbit_end));
trajectory.lunarOrbitPeriapsisAltitude_m = trajectory.lunarOrbitMinimumDistance_m - config.physics.moonRadius_m;
trajectory.lunarOrbitBound = true;
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

fprintf('Synthetic Artemis-II mission (with lunar orbit) saved to results/v2_optimization_results.mat\n');

% Launch visualizer
try
    run_v2_live(output, config.live.speedFactor, config.live.cameraMode);
catch ME
    warning('generate_artemis2_result:VisualizerFailed', 'Live visualizer failed: %s', ME.message);
end
end

function u = unit_vector(v)
if all(v == 0)
    u = v;
else
    u = v / norm(v);
end
end
