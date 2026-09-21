#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export HX5_ISAAC_ROOT="$ROOT"
CELL_ROOT="${HX5_ISAAC_CELL_ROOT:-/home/robotis-ai/workspaces/isaac_logistics_cell_ws}"
COMPOSE=(docker compose -f "$ROOT/runtime/hx5_isaac/compose.yaml")
POLICY_COMPOSE=(docker compose -f "$ROOT/runtime/hx5_isaac/policies.yaml" -f "$ROOT/runtime/hx5_isaac/docker-compose.hand-act.yml")
LEADER_COMPOSE=(docker compose -f "$ROOT/runtime/hx5_isaac/leader-compose.yaml")
SIM_CONTAINER=ai_worker_1044_hx5_isaac
CYCLO_CONTAINER=cyclo_intelligence_1044_hx5_isaac
ros_exec() {
  local container="$1" overlay=""; shift
  local interactive=()
  if [ "$container" = "$CYCLO_CONTAINER" ]; then overlay=/workspace/ros_transport/install/local_setup.bash; fi
  if [ -t 0 ] && [ -t 1 ]; then interactive=(-it); fi
  docker exec "${interactive[@]}" "$container" bash --noprofile --norc -c \
    'set -e; source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; if [ -n "$1" ]; then source "$1"; fi; shift; exec "$@"' bash "$overlay" "$@"
}
case "${1:-help}" in
  start)
    python3 "$ROOT/runtime/hx5_isaac/prepare.py" --data-only
    "${COMPOSE[@]}" up -d ai_worker cyclo
    bash "$ROOT/runtime/hx5_isaac/prepare_transport.sh"
    ;;
  prepare)
    "$0" start
    python3 "$ROOT/runtime/hx5_isaac/prepare.py"
    bash "$CELL_ROOT/build.sh"
    ;;
  isaac)
    shift
    "$0" start
    if [ ! -f "$CELL_ROOT/scene_metadata.json" ]; then "$0" prepare; fi
    exec flock -n -o "$ROOT/simulation/isaac/runtime/isaac.lock" \
      "$ROOT/runtime/hx5_isaac/run_isaac.sh" "$ROOT/src/hx5_isaac/simulator.py" \
      --stage "$CELL_ROOT/logistics_cell.usda" --metadata "$CELL_ROOT/scene_metadata.json" "$@"
    ;;
  cyclo)
    bash "$ROOT/runtime/hx5_isaac/prepare_transport.sh"
    docker exec "$CYCLO_CONTAINER" /command/s6-rc -u change orchestrator cyclo_data
    printf '%s\n' 'Isaac Cyclo services started. UI: http://localhost:7880/'
    ;;
  validate)
    shift
    ros_exec "$SIM_CONTAINER" python3 /hx5_isaac_runtime/validate_integration.py "$@"
    ;;
  ui-install)
    shift
    python3 "$ROOT/runtime/hx5_isaac/install_ui.py" "$@"
    ;;
  reset)
    shift
    python3 "$ROOT/runtime/hx5_isaac/reset_simulation.py" "$@"
    ;;
  sim-stop)
    shift
    python3 "$ROOT/runtime/hx5_isaac/reset_simulation.py" --stop "$@"
    ;;
  enter)
    ros_exec "$SIM_CONTAINER" bash --noprofile
    ;;
  leader)
    shift
    leader_check=false; leader_config_args=()
    for option in "$@"; do
      case "$option" in
        --check) leader_check=true ;;
        --mobile) leader_config_args+=(--mobile) ;;
        *) printf 'Unknown leader option: %s\n' "$option" >&2; exit 1 ;;
      esac
    done
    python3 "$ROOT/runtime/hx5_isaac/leader_preflight.py"
    python3 "$ROOT/runtime/hx5_isaac/reset_simulation.py" --check
    ros_exec "$SIM_CONTAINER" python3 /hx5_isaac_runtime/leader_runtime_check.py
    if "$leader_check"; then
      printf '%s\n' 'Leader devices are available and Isaac is idle. No physical hardware was started.'
      exit 0
    fi
    python3 "$ROOT/runtime/hx5_isaac/leader_config.py" "${leader_config_args[@]}"
    "${LEADER_COMPOSE[@]}" up -d leader
    if ! ros_exec "$SIM_CONTAINER" python3 /hx5_isaac_runtime/leader_hardware_check.py; then
      docker logs --tail 60 lg2_leader_1044_hx5_isaac >&2 || true
      "${LEADER_COMPOSE[@]}" down
      printf '%s\n' 'Skeleton Leader failed to become ready and was stopped. See the hardware diagnostics above.' >&2
      exit 1
    fi
    printf '%s\n' 'Physical LG2 Skeleton Leader started on Isaac domain 115; no physical follower devices are exposed.' \
      'Match both arms to the Isaac follower before holding both gripper triggers for 2 seconds.' \
      'Use runtime/hx5_isaac.sh leader-stop before Mission Canvas control or simulation reset.'
    ;;
  leader-stop)
    "${LEADER_COMPOSE[@]}" down
    ;;
  status)
    "${COMPOSE[@]}" ps
    "${LEADER_COMPOSE[@]}" ps
    ;;
  policy-start|policy-stop)
    action="$1"; backend="${2:-vitacformer}"
    case "$backend" in lerobot|vitacformer|groot) ;; *) printf 'Unknown backend: %s\n' "$backend" >&2; exit 1 ;; esac
    if [ "$action" = policy-start ]; then "${POLICY_COMPOSE[@]}" up -d "$backend"; else "${POLICY_COMPOSE[@]}" stop "$backend"; fi
    ;;
  stop)
    "${LEADER_COMPOSE[@]}" down
    "${POLICY_COMPOSE[@]}" down
    "${COMPOSE[@]}" down
    ;;
  *)
    printf '%s\n' 'Usage: runtime/hx5_isaac.sh {start|prepare|isaac [--headless]|cyclo|validate|ui-install BUILD_DIR|leader [--check] [--mobile]|leader-stop|policy-start [vitacformer|lerobot|groot]|policy-stop [backend]|reset [--check]|sim-stop|enter|status|stop}' \
      'Isaac Cyclo UI: http://localhost:7880/  Robot: FFW SH5 Rev1 / HX5  ROS domain: 115' \
      'Gazebo domain 105 / UI 7380 and all Gazebo data are preserved.'
    ;;
esac
