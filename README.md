# Trajectory-Induced Self-Calibration for Hidden-Target Localization

Code and data for the paper *"Trajectory-Induced Self-Calibration for
Hidden-Target Localization Through an Unknown-Pose Range-Bearing Relay"*
(submitted to the American Control Conference, ACC 2027; arXiv preprint
forthcoming).

This repository is a dependency-free C++17 simulation harness that proves
and validates the paper's central claim: a vehicle can self-calibrate a
range-bearing relay of unknown position and yaw from two distinct
vehicle-relative observations, making a hidden target's relay-local packet
globally actionable. The estimator core here is shared with the companion
CDC paper's closed-loop controller
([excitation-supervised-closed-loop](https://github.com/yashbagla321/excitation-supervised-closed-loop)),
which builds a supervised excitation-reset controller on top of the same
identifiability result. This repository's `main.cpp` is trimmed to the
open-loop sweeps and tables this paper cites; the companion repository
runs the closed-loop supervision study instead.

## Model

A relay beacon has unknown position `x` and yaw `psi`. At vehicle pose
`q_k` it reports noisy local-frame range-bearing packets to both the
vehicle and a hidden target `p`, related by `q_k = x + R(psi) * l_k^v` and
`p = x + R(psi) * l^t`. Two distinct vehicle-relative observations
(`l_a^v != l_b^v`) recover `x`, `psi`, and `p` in closed form; a single
pose leaves a one-dimensional gauge. The paper's theorems establish this
constructively, with a companion rank/conditioning analysis via the
trajectory-spread margin `S_v`.

## Build

### Windows

```powershell
cmake -S . -B build
cmake --build build --config Release
```

### Linux

```bash
sudo apt update && sudo apt install build-essential cmake
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run

```powershell
.\build\adaptive_localization_sim.exe
```

```bash
./build/adaptive_localization_sim
```

By default the executable loads `config/simulation.ini` (plain text,
heavily commented: output directory, Monte Carlo scenarios, beacon counts,
noise levels, random seeds, and solver parameters) and writes into
`results/`. Pass a different config path as the first argument.

## Results-to-paper map

| File | Paper content |
|---|---|
| `results/minimal_beacon_excitation.csv` | Table I (single-pose gauge vs. two-pose rank recovery vs. full trajectory) |
| `results/trajectory_sweep.csv`, `results/near_degenerate_trajectory_sweep.csv`, `results/information_conditioning.csv` | Fig. 2 (trajectory spread `S_v` vs. conditioning and target RMSE, 9 trajectory types) |
| `results/monte_carlo_summary.csv`, `results/monte_carlo_trials.csv`, `results/expanded_baseline_summary.csv` | Table II (estimator comparison: batch/Huber/multistart Gauss-Newton, sliding-window, two-view/naive EKF, single-target-packet) |
| `results/noise_robustness.csv` | Noise-sweep numbers in Section V prose |
| `results/poor_initialization_sweep.csv`, `results/intermittent_measurement_sweep.csv`, `results/outlier_robustness_sweep.csv`, `results/vehicle_localization_noise_sweep.csv`, `results/geometry_sweep.csv`, `results/initial_pose_robustness.csv` | Table III (robustness under poor initialization, dropouts, outliers, vehicle-pose error, weak trajectories) |

Note: `results/expanded_baseline_summary.csv`'s wall-clock timing column is
host-machine-dependent (the paper's Table II caption says so explicitly);
every error/RMSE/rank column is seed-deterministic and will reproduce
exactly.

## Figures

- `figures/acc_geometry.pdf` — the two-view geometry figure (Fig. 1).
  Regenerate with `python scripts/render_geometry_figure.py` (requires
  `pdflatex`).
- `figures/conditioning_points.dat` — the data table Fig. 2 is drawn from
  natively in pgfplots (matches the paper's fonts/math exactly, rather than
  a separately rendered raster image). Regenerate with
  `python scripts/export_conditioning_data.py` after running the
  simulation binary (needs Pillow, see `requirements.txt`).
- `figures/conditioning_summary.png` — a standalone raster rendering of the
  same Fig. 2 data (two log-log scatter panels plus a shared legend), for
  quick inspection without a LaTeX toolchain. Regenerate with
  `python scripts/render_conditioning_summary.py`.

Python scripts depend only on Pillow; install with
`pip install -r requirements.txt`.

## Citation

```bibtex
@unpublished{bagla2027identifiability,
  author = {Bagla, Yash},
  title  = {Trajectory-Induced Self-Calibration for Hidden-Target
            Localization Through an Unknown-Pose Range-Bearing Relay},
  note   = {Submitted to the American Control Conference (ACC)},
  year   = {2027}
}
```
