# Experiment Analysis Guide

This document describes the data that `experiment_runner_node` records and
how the analyzer / plotting pipeline turns it into the figures used in the
ICROS 2026 paper.

## 1. Experiment protocol

- Start distance: **30 mm** (centre-of-robot to target)
- Start directions: **8** bearings at 45° increments
- Heading offsets: configurable via launch arg `heading_offsets_deg`
  (default `[0.0]`, 3-value grid example: `[0.0, -3.0, 3.0]`)
- Repeats per (direction, heading) combination: configurable via `repeats`
  (default 3)
- Total trials = `#controllers × 8 × #heading × repeats`

Default baseline grid (one heading, three repeats) → **24 trials per
controller**, or **72 trials** across Kinematic / MPPI / DWB.

## 2. Termination criteria

A trial ends as soon as **one** of the following is true:

| Criterion | Threshold | Meaning |
|-----------|-----------|---------|
| Success | 5 consecutive samples (~0.1 s) with `‖err‖ ≤ 3 mm` | Converged |
| Convergence-failure | ≥ 60 s accumulated inside `‖err‖ ≤ 5 mm` without reaching 3 mm | Stuck near goal |
| Hard timeout | 90 s wall-clock | Aborted |

The runner adds a short tail (`post_controller_timeout`) after termination so the
time-series captures settling behaviour, then moves on to the next trial.

## 3. Output layout

```
results/<controller>_<YYYYMMDD_HHMMSS>/
├── summary.csv          # one row per trial
├── timeseries/
│   ├── trial_001.csv    # 50 Hz log for each trial
│   └── ...
├── metrics.csv          # 7 metrics per trial (produced by experiment_analyzer)
├── report.txt           # aggregate statistics
└── plots/
    ├── convergence.png
    ├── direction_bars.png
    ├── polar.png
    ├── heading_bars.png          (only if #heading ≥ 2)
    ├── heatmap_settling.png      (only if #heading ≥ 2)
    └── steering_vs_settling.png
```

### 3.1 `summary.csv` columns

| Column | Unit | Description |
|--------|------|-------------|
| `trial_id` | — | monotonically increasing |
| `controller` | — | `kinematic` / `MPPI` / `DWB` |
| `direction_deg` | deg | bearing of the start offset |
| `heading_offset_deg` | deg | initial heading offset |
| `repeat` | — | repeat index within the (direction, heading) cell |
| `result` | — | `success` / `convergence_fail` / `timeout` |
| `elapsed_s` | s | total wall-time of the trial |
| `start_x`, `start_y`, `start_yaw` | m, m, rad | measured start pose |
| `final_x`, `final_y`, `final_yaw` | m, m, rad | final pose |

### 3.2 `timeseries/trial_XXX.csv` columns

`t, x, y, yaw, vx, vy, wz, j0, j1, j2, j3`

`j0…j3` are the four joint states (two drive + two steering).
Sampling is 50 Hz.

## 4. The 7 metrics

Computed by `experiment_analyzer` and saved into `metrics.csv`.

| Metric | Unit | Definition |
|--------|------|-----------|
| Settling time | s | time from start to the first sample with `‖err‖ ≤ 3 mm` |
| Steady-state error | mm | mean of `‖err‖` over the **last 1 s** of the trial |
| Max overshoot | mm | largest post-goal excursion along the start→goal line |
| Cross-track RMS | mm | RMS of lateral deviation from the nominal straight path |
| Heading error (final) | deg | `|yaw_final − yaw_goal|` normalised to `[0, π]` |
| Control effort | m/s | `∫ |dv/dt| dt` summed over `vx` and `vy` |
| Steering change | rad | `∫ |dq_steer/dt| dt` — quantifies caster-alignment load |

## 5. How each plot is built

| File | What it shows | Suggested use |
|------|---------------|---------------|
| `convergence.png` | `‖err(t)‖` for each trial on one axis | Qualitative shape of convergence |
| `direction_bars.png` | per-direction mean of each metric | Direction-dependent bias |
| `polar.png` | polar overlay of settling-time per direction | Compact summary for the paper |
| `heading_bars.png` | per-heading mean (if ≥ 2 heading offsets) | Heading-offset robustness |
| `heatmap_settling.png` | direction × heading heatmap of settling time | Worst-case / best-case cells |
| `steering_vs_settling.png` | scatter of `Steering change` vs `Settling time` with Pearson ρ | Is the controller trading stability for smoothness? |

## 6. Cross-controller comparison

After you have runs for two or more controllers under the same grid:

```bash
python3 src/icros_test_setup/scripts/plot_compare.py \
  results/kinematic_20260414_... \
  results/mppi_20260415_... \
  results/dwb_20260415_...
```

Output directory `results/comparison_<timestamp>/`:

| File | Content |
|------|---------|
| `boxplots.png` | side-by-side box plots for every metric |
| `radar.png` | normalised radar chart (all metrics scaled to `[0, 1]`) |
| `convergence_compare.png` | stacked convergence curves for representative directions (0/90/180/270°) |
| `success_rate.png` | overall success / convergence-fail / timeout rate per controller |
| `heading_robustness.png` | settling-time per heading (only if ≥ 2 heading cells) |
| `steering_vs_settling.png` | combined scatter with per-controller colour |
| `comparison.csv` | controller × metric table with `mean / std / min / max` |

## 7. Reproducing the paper figures

1. Build the workspace (native or Docker) with the binary plugin in place.
2. Run each controller under the same grid (use the same
   `heading_offsets_deg` and `repeats`):

   ```bash
   for ctrl in kinematic mppi dwb; do
     ros2 launch icros_test_setup experiment_${ctrl}.launch.py \
       heading_offsets_deg:='[0.0,-3.0,3.0]' repeats:=3
   done
   ```
3. Pass all three result directories to `plot_compare.py`.
4. The PNGs under `results/comparison_*/` correspond one-to-one with the
   figures in Sections IV-B and IV-C of the paper.

## 8. Sanity check with sample data

The repository ships with `results/kinematic_sample/`, `results/MPPI_sample/`,
and `results/comparison_sample/` so you can verify the plotting pipeline
without running the full 90-minute benchmark:

```bash
python3 -m icros_test_setup.experiment_analyzer \
  src/icros_test_setup/results/kinematic_sample
python3 src/icros_test_setup/scripts/plot_compare.py \
  src/icros_test_setup/results/kinematic_sample \
  src/icros_test_setup/results/MPPI_sample
```
