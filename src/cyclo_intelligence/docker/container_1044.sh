#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

export CYCLO_COMPOSE_PROJECT_NAME=cyclo_intelligence_1044_sh5
export CYCLO_MAIN_CONTAINER_NAME=cyclo_intelligence_1044_sh5
export LEROBOT_CONTAINER_NAME=lerobot_server_1044_sh5
export VITACFORMER_CONTAINER_NAME=vitacformer_server_1044_sh5
export GROOT_CONTAINER_NAME=groot_server_1044_sh5
export CYCLO_NAVIGATION_CONTAINER="${CYCLO_1044_NAVIGATION_CONTAINER:-ai_worker_1044_sh5}"
export CYCLO_AGENT_SOCKETS_DIR=/var/run/robotis/agent_sockets/cyclo_intelligence_1044_sh5

export CYCLO_UI_PORT="${CYCLO_1044_UI_PORT:-7280}"
export CYCLO_ROSBRIDGE_PORT="${CYCLO_1044_ROSBRIDGE_PORT:-7290}"
export CYCLO_VIDEO_SERVER_PORT="${CYCLO_1044_VIDEO_SERVER_PORT:-7282}"
export CYCLO_WEB_VIDEO_SERVER_PORT="${CYCLO_1044_WEB_VIDEO_SERVER_PORT:-7285}"
export CYCLO_SUPERVISOR_API_PORT="${CYCLO_1044_SUPERVISOR_API_PORT:-7300}"

# Override this with the ROS domain configured on AI Worker 1044 when moving
# from isolated development to the real robot.
export ROS_DOMAIN_ID="${CYCLO_1044_ROS_DOMAIN_ID:-104}"

case "$(uname -m)" in
  aarch64|arm64) image_arch=arm64 ;;
  *) image_arch=amd64 ;;
esac
export CYCLO_MAIN_IMAGE="cyclo-1044-sh5/main:local-${image_arch}"
export CYCLO_LEROBOT_IMAGE="cyclo-1044-sh5/lerobot:sh5-c8ce413-${image_arch}"
export CYCLO_LEROBOT_TREX_IMAGE="cyclo-1044-sh5/lerobot-trex:sh5-c8ce413-${image_arch}"
export CYCLO_VITACFORMER_IMAGE="cyclo-1044-sh5/vitacformer:local-${image_arch}"
export CYCLO_GROOT_IMAGE="cyclo-1044-sh5/groot:local-${image_arch}"
export CYCLO_BUILD_LOCAL_IMAGES=1
export CYCLO_BUILD_SOURCE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"

if [ "${1:-}" = check ]; then
  exec python3 "${SCRIPT_DIR}/check_1044.py"
fi

case "${1:-help}" in
  prepare|build|start|start-lerobot|start-vitacformer|start-groot)
    if [ -f "${SCRIPT_DIR}/../.gitmodules" ]; then
      git -C "${SCRIPT_DIR}/.." submodule update --init --recursive
    fi
    if [ "$1" = prepare ]; then
      exit 0
    fi
    if [ "$1" = build ]; then
      exec "${SCRIPT_DIR}/container.sh" "$@"
    fi
    has_build_flag=0
    for argument in "$@"; do
      case "${argument}" in
        --build|-b) has_build_flag=1 ;;
      esac
    done
    if [ "${has_build_flag}" -eq 0 ]; then
      set -- "$@" --build
    fi
    ;;
esac

exec "${SCRIPT_DIR}/container.sh" "$@"
