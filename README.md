# omni-docking-bench

**Reproducibility package for the ICROS 2026 paper**
*“Kinematic-based Precision Docking Controller for a Double-Steering-Drive Omni AMR.”*

A single ROS 2 workspace that lets reviewers reproduce the paper's
short-range docking benchmark — **3 cm start distance, 3 mm settling criterion,
8-direction × heading-offset × multi-repeat grid** — comparing three controllers:

| Controller | Type | Role |
|------------|------|------|
| **Kinematic** (ours) | PGV-direct feedback (`pgv_guided_controller`) | Proposed method |
| **MPPI** | Sampling-based MPC (Nav2) | Baseline |
| **DWB** | Dynamic Window Approach (Nav2) | Baseline |

Target platform: **ROS 2 Jazzy on Ubuntu 24.04 (x86_64)**.

---

## Repository layout

```
omni-docking-bench/
├── src/
│   ├── dsd_bot_description/         # URDF / xacro for the DSD-bot
│   ├── dsd_control_demo/            # ros2_control hardware + bringup
│   ├── pgv_guided_controller/       # Kinematic proposed controller (PGV feedback)
│   └── icros_test_setup/            # Experiment runner, analyzer, plotter
├── libs/
│   └── double_steering_drive_controller/  # Pre-built .so (binary plugin)
├── docker/
│   ├── Dockerfile.jazzy
│   ├── docker-compose.yml
│   └── entrypoint.sh
├── docs/
│   ├── EXPERIMENT_ANALYSIS.md       # How to interpret the 7 metrics & plots
│   └── RELEASE_PROCESS.md           # How the binary plugin is (re)built
├── results/                         # Sample runs are tracked, the rest is ignored
├── .github/workflows/build.yml      # CI: colcon build + smoke test
├── LICENSE                          # Apache-2.0
├── CITATION.cff
└── README.md                        # ← you are here
```

---

## Quick start

Three equivalent entry points. Pick whichever suits your environment.

### 1) Native ROS 2 Jazzy (Ubuntu 24.04)

```bash
# System deps
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-controller \
  python3-colcon-common-extensions python3-pip

# 1. Clone
git clone https://github.com/mach0312/omni-docking-bench.git ~/omni_docking_bench_ws
cd ~/omni_docking_bench_ws

# 2. Place the pre-built binary plugin
#    See libs/double_steering_drive_controller/README.md for the GitHub Release link.
curl -L -o libs/double_steering_drive_controller/lib/libdouble_steering_drive_controller.so \
  https://github.com/mach0312/omni-docking-bench/releases/download/v0.1.0/libdouble_steering_drive_controller.so

# 3. Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 4. Run an experiment (kinematic controller, 24 trials, ~5 min)
ros2 launch icros_test_setup experiment_kinematic.launch.py \
  heading_offsets_deg:='[0.0]' repeats:=3
```

### 2) Single Docker container

```bash
git clone https://github.com/mach0312/omni-docking-bench.git
cd omni-docking-bench
# Place the binary plugin under libs/double_steering_drive_controller/lib/ (see §1 above)
docker build -f docker/Dockerfile.jazzy -t omni-docking-bench:jazzy .

docker run --rm -it \
  -v $PWD/results:/workspace/omni_docking_bench_ws/src/icros_test_setup/results \
  omni-docking-bench:jazzy \
  ros2 launch icros_test_setup experiment_kinematic.launch.py \
    heading_offsets_deg:='[0.0]' repeats:=3
```

### 3) docker-compose (multi-container)

```bash
cd docker
docker compose build
# Start the ros2_control/hardware + dummy PGV stack (detached)
docker compose up -d robot
# Run an experiment inside the "runner" service (one-shot)
docker compose run --rm runner \
  ros2 launch icros_test_setup experiment_kinematic.launch.py \
    heading_offsets_deg:='[0.0]' repeats:=3
docker compose down
```

Results are written to `results/<controller>_<timestamp>/` on the host.

---

## What gets produced

Every run of `experiment_*_launch.py` writes:

- `summary.csv` — one row per trial
- `timeseries/trial_XXX.csv` — 50 Hz logs of pose / velocity / joint states
- `metrics.csv` — 7 evaluation metrics per trial
- `report.txt` — aggregate statistics
- `plots/*.png` — convergence curves, polar plots, heatmaps, etc.

To analyse across controllers:

```bash
python3 src/icros_test_setup/scripts/plot_compare.py \
  results/kinematic_* results/mppi_* results/dwb_*
```

A sample run is provided under `results/*_sample/` so you can verify the
plotting pipeline without running the full benchmark.

Detailed metric definitions and interpretation guide: see
[`docs/EXPERIMENT_ANALYSIS.md`](docs/EXPERIMENT_ANALYSIS.md).

---

## The binary plugin in `libs/`

The `double_steering_drive_controller` is distributed **as a pre-built
shared library** (`.so`) rather than source. This is sufficient for
reproducibility because `controller_manager` loads controllers via
`pluginlib::dlopen()` — only the class name (`double_steering_drive_controller/DoubleSteeringDriveController`)
and a matching ABI are required.

- Platform: Ubuntu 24.04 / ROS 2 Jazzy / x86_64 / g++-14
- SHA-256 of the official binary: see the GitHub Release notes
- Full details: [`libs/double_steering_drive_controller/README.md`](libs/double_steering_drive_controller/README.md)

Rebuild / release procedure: [`docs/RELEASE_PROCESS.md`](docs/RELEASE_PROCESS.md).

---

## Citation

```bibtex
@inproceedings{son2026omnidocking,
  title   = {Kinematic-based Precision Docking Controller for a Double-Steering-Drive Omni AMR},
  author  = {Son, Jaerak},
  booktitle = {Proc. ICROS Annual Conference},
  year    = {2026}
}
```

See [`CITATION.cff`](CITATION.cff) for the machine-readable form.

---

## License

Apache-2.0 — see [`LICENSE`](LICENSE).

## Contact

손재락 (Jaerak Son) — `sjr9017@khu.ac.kr`
