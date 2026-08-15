function output = run_artemis2(varargin)
% RUN_ARTEMIS2  Configure and run a V2 free-return lunar flyby search
%   OUTPUT = RUN_ARTEMIS2() runs an optimized search using the V2 pipeline
%   configured for a free-return lunar flyby (no captured lunar orbit). The
%   function saves results into the configured results directory and launches
%   the live visualizer when a valid trajectory is found.

if nargin >= 1 && isstruct(varargin{1})
    overrides = varargin{1};
else
    overrides = struct();
end

config = v2_config();
% Configure for a free-return Artemis-II style flyby
config.mission.lunarOrbitDuration_days = 0;         % no sustained lunar orbit
config.mission.lunarApproachDuration_days = 0.5;    % short approach window
% plausible outbound/return windows for a crewed flyby search (days)
config.mission.outboundFlightTime_days = [5, 6, 7, 8];
config.mission.returnFlightTime_days = [4, 5, 6];
config.mission.maxCandidates = 1200;
config.mission.maxRuntime_s = 600;   % limit runtime for interactive runs
config.optimizer.maxIterations = 1200;
config.optimizer.useParallel = false; % safe default on local machine

% apply any user overrides provided
config = v2_config(apply_overrides(config, overrides));

fprintf('Starting Artemis-II style free-return search using V2 pipeline...\n');
optimization = v2_optimize_mission(config, []);

if isempty(optimization.bestResult)
    warning('run_artemis2:NoResult', 'No valid mission candidate found in search.');
    output = struct('config', config, 'optimization', optimization, 'trajectory', []);
    save(fullfile(config.output.directory, config.output.resultFile), 'output');
    return
end

fprintf('Best candidate found: delta-v = %.1f m/s, fuel = %.1f kg\n', optimization.bestResult.totalDeltaV_mps, optimization.bestResult.fuel.requiredPropellant_kg);

try
    trajectory = v2_build_mission_trajectory(optimization.bestResult, config);
catch ME
    warning('run_artemis2:TrajectoryBuildFailed', 'Failed to build trajectory: %s', ME.message);
    trajectory = [];
end

output = struct('config', config, 'optimization', optimization, 'trajectory', trajectory);

% Ensure results directory exists
if ~exist(config.output.directory, 'dir')
    mkdir(config.output.directory);
end

save(fullfile(config.output.directory, config.output.resultFile), 'output');

if ~isempty(trajectory)
    fprintf('Trajectory built. Launching live replay...\n');
    try
        run_v2_live(output, config.live.speedFactor, config.live.cameraMode);
    catch ME2
        warning('run_artemis2:VisualizerFailed', 'Live visualizer failed: %s', ME2.message);
    end
end
end

function out = apply_overrides(base, overrides)
out = base;
if isempty(overrides) || ~isstruct(overrides)
    return
end
names = fieldnames(overrides);
for k = 1:numel(names)
    n = names{k};
    if isstruct(overrides.(n)) && isfield(out, n)
        out.(n) = apply_overrides(out.(n), overrides.(n));
    else
        out.(n) = overrides.(n);
    end
end
end
