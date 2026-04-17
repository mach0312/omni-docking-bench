#!/usr/bin/env bash
# omni-docking-bench container entrypoint.
# Sources ROS 2 Jazzy and the workspace overlay, then execs the user command.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "/workspace/omni_docking_bench_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /workspace/omni_docking_bench_ws/install/setup.bash
fi

exec "$@"
