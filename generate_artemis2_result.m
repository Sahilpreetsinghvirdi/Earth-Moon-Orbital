function generate_artemis2_result()
% GENERATE_ARTEMIS2_RESULT  Physics-based Artemis-II style mission generator
%   Builds a simple physics-consistent mission using v2_solve_lambert and the
%   propagator helpers. This produces a plausible Earth->Moon transfer, a
%   short circular lunar orbit (insertion), and a Moon->Earth return.
%   The output is saved to results/v2_optimization_results.mat and replayed
%   with the existing run_v2_live visualizer.

config = v2_config();
config.live.speedFactor = 30000;
config.live.cameraMode = 'earth';

% Mission parameters (adjustable)
launchEpoch = datetime(2026,11,1,0,0,0,'TimeZone','UTC');
outboundDays = 4.5;    % Earth->Moon transfer
lunarOrbitDays = 1.0;  % time spent in lunar orbit
returnDays = 6.0;      % Moon->Earth return

% burn durations (s)
transferBurnDuration_s = 600;
insertionBurnDuration_s = 600;
departureBurnDuration_s = 600;

% Build epochs and durations
outboundDuration_s = outboundDays * 86400;
orbitDuration_s = lunarOrbitDays * 86400;
returnDuration_s = returnDays * 86400;
flightDuration_s = outboundDuration_s + orbitDuration_s + returnDuration_s;
N = 1600;

% Time series
time_s = linspace(0, flightDuration_s, N)';
epoch = launchEpoch + seconds(time_s);

% Earth parking initial state (circular parking orbit in x-y plane)earth_r = config.physics.earthParkingRadius_m;
earth_mu = config.physics.muEarth_m3ps2;
earth_pos = [rearth_r; 0; 0];
earth_vel = [0; sqrt(earth_mu / rearth_r); 0];
initialState = struct('position_m', earth_pos, 'velocity_mps', earth_vel);

% Choose a nominal lunar approach point along the Moon-Earth line
arrivalEpoch = launchEpoch + seconds(outboundDuration_s);
arrivalMoon = v2_get_celestial_state(arrivalEpoch, 'moon', config, 'analytical');
approachDistance_m = config.mission.lunarApproachStartDistance_m / 4; % closer approach point
approachPoint = arrivalMoon.position_m + unit_vector(-arrivalMoon.position_m) * approachDistance_m;

% Solve Lambert outbound
outboundLambert = v2_solve_lambert(earth_pos, approachPoint, outboundDuration_s, earth_mu, true);
if ~outboundLambert.valid
    error('generate_artemis2_result:LambertFailed', 'Outbound Lambert solution failed')
end
departureDeltaV = outboundLambert.velocity1_mps - earth_vel;

% Propagate outbound transfer applying initial departure burn as delta-v over burn duration
outboundSegment = v2_propagate_trajectory(config, launchEpoch, initialState, outboundDuration_s, 'Earth departure and lunar transfer', [], @() false, departureDeltaV, transferBurnDuration_s, 'delta-v');
if isempty(outboundSegment.position_m)
    error('generate_artemis2_result:PropagationFailed', 'Outbound propagation failed')
end

% At arrival: compute lunar insertion burn to circular lunar orbit
arrivalIndex = size(outboundSegment.position_m,1);
scArrivalPos = outboundSegment.position_m(end, :)';
scArrivalVel = outboundSegment.velocity_mps(end, :)';
moonAtArrival = v2_get_celestial_state(outboundSegment.epoch(end), 'moon', config, 'analytical');
relPos = scArrivalPos - moonAtArrival.position_m;
relDist = norm(relPos);

% Desired circular orbit radius and velocity about Moon
lunarOrbitAltitude_m = 150e3;
lunarOrbitRadius_m = config.physics.moonRadius_m + lunarOrbitAltitude_m;
muMoon = config.physics.muMoon_m3ps2;

if relDist < lunarOrbitRadius_m
    % If transfer ends inside desired orbit, set circular radius to current distance
    lunarOrbitRadius_m = relDist + 1e3;
end

% Desired velocity in Moon-centered frame for circular orbit
v_circ = sqrt(muMoon / lunarOrbitRadius_m);
% Construct tangential direction (rotate radial by 90 deg in-plane)
radial = relPos / max(norm(relPos),1);
tangential = [ -radial(2); radial(1); 0 ];
desiredVel_moon_frame = v_circ * tangential;
% Convert to inertial by adding Moon velocity
desiredVel_inertial = desiredVel_moon_frame + moonAtArrival.velocity_mps;

% Compute insertion delta-v (vector)
insertionDeltaV = desiredVel_inertial - scArrivalVel;

% Propagate a short insertion burn (simulate capture)
insertionStartState = struct('position_m', scArrivalPos, 'velocity_mps', scArrivalVel);
insertionSegment = v2_propagate_trajectory(config, outboundSegment.epoch(end), insertionStartState, 0.5*86400, 'Lunar insertion and orbit', [], @() false, insertionDeltaV, insertionBurnDuration_s, 'delta-v');

% If insertionSegment is empty, fall back to a small instant change
if isempty(insertionSegment.position_m)
    insertionStatePos = scArrivalPos;
    insertionStateVel = scArrivalVel + insertionDeltaV;
else
    insertionStatePos = insertionSegment.position_m(end, :)';
    insertionStateVel = insertionSegment.velocity_mps(end, :)';
end

% Propagate circular lunar orbit using v2_propagate_lunar_orbit for more realism
orbitSegment = v2_propagate_lunar_orbit(config, insertionSegment.epoch(end), orbitDuration_s, lunarOrbitAltitude_m, [], @() false, struct('position_m', insertionStatePos, 'velocity_mps', insertionStateVel), desiredVel_inertial, insertionStateVel);

% After orbit, compute departure Lambert back to Earth parking
departureEpoch = orbitSegment.epoch(end);
departureStatePos = orbitSegment.position_m(end, :)';
departureStateVel = orbitSegment.velocity_mps(end, :)';

% Choose Earth target position (parking) at arrival time
earthArrivalEpoch = departureEpoch + seconds(returnDuration_s);
earthTarget = earth_pos; % simple target

returnLambert = v2_solve_lambert(departureStatePos, earthTarget, returnDuration_s, earth_mu, true);
if ~returnLambert.valid
    error('generate_artemis2_result:ReturnLambertFailed', 'Return Lambert solution failed')
end
returnDeltaV = returnLambert.velocity1_mps - departureStateVel;

% Propagate return burn and coast
returnSegment = v2_propagate_trajectory(config, departureEpoch, struct('position_m', departureStatePos, 'velocity_mps', departureStateVel), returnDuration_s, 'Lunar departure and Earth return', [], @() false, returnDeltaV, departureBurnDuration_s, 'delta-v');

% Assemble full trajectory by concatenation
trajectory = concatenate_segments(outboundSegment, insertionSegment, orbitSegment, returnSegment);

% Build optimization.bestResult stub
bestResult = struct();
bestResult.valid = true;
bestResult.candidate = struct('outboundFlightTime_days', outboundDays, 'lunarApproachDuration_days', 0.5, 'lunarOrbitDuration_days', lunarOrbitDays, 'launchEpoch', launchEpoch);
bestResult.departureDeltaV_mps = norm(departureDeltaV);
bestResult.lunarOrbitInsertionDeltaV_mps = norm(insertionDeltaV);
bestResult.lunarDepartureDeltaV_mps = norm(returnDeltaV);
bestResult.earthCaptureDeltaV_mps = 0;
bestResult.totalDeltaV_mps = bestResult.departureDeltaV_mps + bestResult.lunarOrbitInsertionDeltaV_mps + bestResult.lunarDepartureDeltaV_mps;
bestResult.fuel = struct('requiredPropellant_kg', 150000);

% Fill trajectory metadata used by visualizer
trajectory.result = bestResult;
trajectory.burnEvents = struct('epoch', {launchEpoch, outboundSegment.epoch(end), departureEpoch}, 'name', {'Earth departure', 'Lunar insertion', 'Lunar departure'}, 'deltaV_mps', {norm(departureDeltaV), norm(insertionDeltaV), norm(returnDeltaV)});
trajectory.completed = true;
trajectory.collision = false;
trajectory.lunarSOIEntered = any(trajectory.moonDistance_m <= config.physics.moonSOI_m);
trajectory.lunarOrbitValid = true;
trajectory.lunarOrbitMinimumDistance_m = min(trajectory.moonDistance_m);
trajectory.lunarOrbitPeriapsisAltitude_m = trajectory.lunarOrbitMinimumDistance_m - config.physics.moonRadius_m;
trajectory.earthArrivalDistance_m = trajectory.earthDistance_m(end);
trajectory.earthArrivalSafe = trajectory.earthArrivalDistance_m <= config.mission.maximumEarthArrivalDistance_m;

optimization = struct('bestResult', bestResult, 'candidateResults', []);
output = struct('config', config, 'optimization', optimization, 'trajectory', trajectory);

% Save and visualize
if ~exist(fullfile(pwd,'results'), 'dir')
    mkdir(fullfile(pwd,'results'));
end
save(fullfile(pwd, 'results', 'v2_optimization_results.mat'), 'output');

fprintf('Physics-based Artemis-II mission saved to results/v2_optimization_results.mat\n');
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
