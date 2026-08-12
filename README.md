# Earth-Moon Orbital Rocket Simulation

This folder is an independent copy of the original MATLAB Monte Carlo Rocket Trajectory project. The original project remains untouched at `D:\Visual Studio Files\MATLAB Monte Carlo rocket trajectory`.

The new Earth-Moon model is a two-dimensional, Earth-centered Monte Carlo simulation in SI units. Each launch uses powered ascent with randomized vehicle and guidance parameters, an upper-stage translunar injection above the atmosphere, Earth and Moon gravity, a continuously moving circular Moon, exponential atmosphere, quadratic drag, and a randomized retrograde lunar-insertion impulse at first inbound SOI entry. The Moon is evaluated as:

```matlab
rMoon(t) = a * [cos(n * t + phi), sin(n * t + phi)]
n = sqrt(muEarth / a^3)
```

The rocket's acceleration combines Earth gravity, Moon gravity, atmospheric drag, and powered ascent thrust. Trials record Earth escape, Moon SOI entry, closest Moon approach, Moon-relative velocity, Moon-relative specific energy, temporary capture, lunar orbit, and impact states. Every trial retains a compact trajectory overview; the selected best trial is re-simulated and saved at full resolution.

Run a batch Monte Carlo study:

```matlab
cd('D:\Visual Studio Files\Earth-Moon Orbital Rocket Simulation')
simulationResults = run_earth_moon_monte_carlo;
```

For a shorter run, supply the number of trials:

```matlab
simulationResults = run_earth_moon_monte_carlo(20);
```

Run the live 2D visualization:

```matlab
simulationResults = run_earth_moon_live(12);
```

The live window updates the rocket and revolving Moon from simulation time, shows current Earth escape, Moon SOI, capture indicators, and cumulative mission statistics. The Stop control ends the current live run without writing a partial result file.

Replay the best saved trajectory with Moon motion:

```matlab
replay_best_earth_moon_trajectory;
```

The batch run writes these independent artifacts inside this project's `results` folder:

- `earth_moon_orbital_results.mat`
- `earth_moon_summary_report.txt`
- `earth_moon_best_trajectory.png`
- `earth_moon_outcomes.png`
- `earth_moon_best_metrics.png`

The copied original MATLAB files remain in this folder for reference and can still run independently. The new Earth-Moon entry points do not modify or call files in the original project folder.
