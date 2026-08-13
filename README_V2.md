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

The live replay keeps its main camera fixed for the entire run. The default is a fixed Earth-centered view; choose a fixed Moon-centered view explicitly with `run_v2_live([], [], 'moon')`, or pass `'earth'` for the default. The red leg is Earth departure, lunar approach, and capture; the green leg is the bound Moon-centred orbit; the blue leg is the finite lunar departure burn and Earth return. A fixed Moon-centred inset shows the low lunar orbit at useful scale while the main view shows the Moon moving around Earth. If the saved result was interrupted or damaged, `run_v2_live` rebuilds and replaces it with the validated direct mission automatically.

The V2 pipeline contains explicit mission epochs, hierarchical launch-date search, future Moon-state evaluation, universal-variable Lambert transfers, Earth/Moon/Sun numerical propagation, a one-day lunar approach transfer, near-Moon finite capture and departure burns, lunar two-body orbit propagation, burn and rocket-equation fuel accounting, constraint rejection, convergence history, persistent `results/mission_state.mat`, and dashboard artifacts.

The ephemeris adapter first attempts MATLAB Aerospace Toolbox `planetEphemeris` for Moon endpoint states. When that call is unavailable or unsupported, it reports and uses a documented analytical fallback based on mean lunar orbital elements. High-rate RK4 propagation uses the time-dependent analytical fallback to avoid repeatedly loading toolbox ephemeris data; this mode is explicitly recorded in trajectory diagnostics.

The optimizer evaluates coarse dates, refines around the current best launch window, and performs a final sub-day search. It records the best launch date, Moon arrival, lunar orbit, Earth return, total delta-v, fuel estimate, score, candidates, successful candidates, iterations, and stopping reason.

Lambert candidates are rejected when the spacecraft is moving in the wrong radial direction at lunar arrival or departure. The trajectory builder also reports Earth-arrival safety, lunar sphere-of-influence entry, and phase-boundary position jumps. The current mission model uses patched-conic Lambert handoffs with numerical Earth-Moon-Sun propagation inside each transfer and a numerical lunar two-body segment; reported phase-boundary jumps are an explicit limitation rather than hidden continuous dynamics.

Earth departure, lunar approach, capture, and departure are modeled as finite-duration burns and numerical propagation, not instantaneous state replacements. The spacecraft starts at the 100 km circular Earth parking orbit, executes a smooth 480-second prograde trans-lunar injection, and then coasts under Earth-Moon-Sun gravity toward the moving Moon. The direct transfer begins its lunar approach 15,000 km from the Moon. That approach is iteratively retargeted to the 300 km lunar parking radius, where a continuous capture burn forms a low bound orbit. The return leg leaves that orbit with a finite burn that starts tangent to the lunar orbit and converges toward the Earth-return solution; it then receives a finite terminal Earth-capture correction. The spacecraft state remains continuous through all phase boundaries; the report records the boundary jumps, Earth-arrival target error, and lunar specific-energy history.

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
