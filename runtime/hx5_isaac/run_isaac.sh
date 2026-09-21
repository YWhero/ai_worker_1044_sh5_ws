#!/usr/bin/env bash
set -euo pipefail
profile_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
profile_isaac="${ISAAC_SIM_DIR:-/home/robotis-ai/isaac_sim/app/6.1.0}"
profile_runtime="$profile_root/simulation/isaac/runtime"
mkdir -p "$profile_runtime"/{cache,config,data,logs,portable,tmp}
export XDG_CACHE_HOME="$profile_runtime/cache" XDG_CONFIG_HOME="$profile_runtime/config" XDG_DATA_HOME="$profile_runtime/data" TMPDIR="$profile_runtime/tmp"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export __GL_SHADER_DISK_CACHE_PATH="$profile_runtime/cache/gl" CUDA_CACHE_PATH="$profile_runtime/cache/cuda"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DOMAIN_ID RMW_IMPLEMENTATION ZENOH_CONFIG_OVERRIDE
export ISAAC_SIM_DIR="$profile_isaac"
profile_script="$1"
shift
exec "$profile_isaac/python.sh" --no-ros-env "$profile_script" "$@" \
  --portable-root "$profile_runtime/portable" \
  --/app/settings/persistent=false --/app/settings/loadUserConfig=false --/app/settings/saveUserConfig=false \
  --/log/file="$profile_runtime/logs/isaac.log" \
  --/app/window/title="Isaac Sim | SH5 HX5 Logistics Cell"
