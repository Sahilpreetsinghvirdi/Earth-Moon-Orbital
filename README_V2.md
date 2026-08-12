# V2 Earth-Moon Mission and Optimal Trajectory Simulator

V2 is developed on the `v2-earth-moon` branch. The committed `main` branch remains the V1 baseline and is not merged or rewritten by V2 work.

Run an optimization from MATLAB:

```matlab
cd('D:\Visual Studio Files\Earth-Moon Orbital Rocket Simulation')
output = run_v2_optimizer;
```

Use a bounded reproducible test window:

```matlab
options = struct;
options.epoch = datetime('now', 'TimeZone', 'UTC');
options.searchEndEpoch = options.epoch + days(30);
options.resume = false;
options.randomized = false;
options.optimizer = struct('showProgress', true, 'useParallel', false);
options.mission = struct('maxCandidates', 30, 'maxRuntime_s', 120, 'coarseStep_days', 1);
output = run_v2_optimizer(options);
```

Replay the saved calculated trajectory:

```matlab
run_v2_live
```

The V2 pipeline contains explicit mission epochs, hierarchical launch-date search, future Moon-state evaluation, universal-variable Lambert transfers, Earth/Moon/Sun numerical propagation, lunar two-body orbit propagation, burn and rocket-equation fuel accounting, constraint rejection, convergence history, persistent `results/mission_state.mat`, and dashboard artifacts.

The ephemeris adapter first attempts MATLAB Aerospace Toolbox `planetEphemeris` for Moon endpoint states. When that call is unavailable or unsupported, it reports and uses a documented analytical fallback based on mean lunar orbital elements. High-rate RK4 propagation uses the time-dependent analytical fallback to avoid repeatedly loading toolbox ephemeris data; this mode is explicitly recorded in trajectory diagnostics.

The optimizer evaluates coarse dates, refines around the current best launch window, and performs a final sub-day search. It records the best launch date, Moon arrival, lunar orbit, Earth return, total delta-v, fuel estimate, score, candidates, successful candidates, iterations, and stopping reason.

Run focused validation:

```matlab
validation = test_v2_pipeline;
```

V2 output files are separate from the V1 result artifacts:

- `results/v2_optimization_results.mat`
- `results/mission_state.mat`
- `results/v2_mission_report.txt`
- `results/v2_optimization_dashboard.png`
- `results/v2_best_trajectory.png`
