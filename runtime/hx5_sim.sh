#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
export HX5_SIM_ROOT="$ROOT"
COMPOSE=(docker compose -f "$ROOT/runtime/hx5_sim/compose.yaml")
POLICY_COMPOSE=(docker compose -f "$ROOT/runtime/hx5_sim/policies.yaml" -f "$ROOT/runtime/hx5_sim/docker-compose.hand-act.yml")
LEADER_COMPOSE=(docker compose -p cyclo_1044_hx5_leader -f "$ROOT/runtime/hx5_sim/leader-compose.yaml")
SIM_CONTAINER=ai_worker_1044_hx5_sim
CYCLO_CONTAINER=cyclo_intelligence_1044_hx5_sim
ros_exec() {
  local container="$1"
  shift
  local interactive=()
  local transport_setup=""
  if [ "$container" = "$CYCLO_CONTAINER" ]; then
    transport_setup=/workspace/ros_transport/install/local_setup.bash
  fi
  if [ -t 0 ] && [ -t 1 ]; then interactive=(-it); fi
  docker exec "${interactive[@]}" "$container" bash --noprofile --norc -c \
    'set -e; source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; if [ -n "$1" ]; then source "$1"; fi; shift; exec "$@"' bash "$transport_setup" "$@"
}
case "${1:-help}" in
  build)
    shift
    if [ "$#" = 0 ]; then set -- ai_worker cyclo; fi
    "${COMPOSE[@]}" build "$@"
    ;;
  start)
    mkdir -p "$ROOT/simulation/ai_worker" "$ROOT/simulation/cyclo" \
      "$ROOT/simulation/agent_sockets/ai_worker" "$ROOT/simulation/agent_sockets/cyclo"
    "${COMPOSE[@]}" up -d ai_worker cyclo
    bash "$ROOT/runtime/hx5_sim/prepare_transport.sh"
    ;;
  router)
    printf '%s\n' 'The dedicated Zenoh router starts automatically with start (localhost:7455).'
    docker logs --tail 20 "$SIM_CONTAINER"
    ;;
  gazebo)
    shift
    ros_exec "$SIM_CONTAINER" flock -n -o /tmp/hx5-gazebo.lock ros2 launch hx5_simulation workcell.launch.py "$@"
    ;;
  cyclo)
    bash "$ROOT/runtime/hx5_sim/prepare_transport.sh"
    ros_exec "$CYCLO_CONTAINER" flock -n -o /tmp/hx5-cyclo.lock ros2 launch orchestrator cyclo_intelligence_bringup.launch.py
    ;;
  enter)
    ros_exec "$SIM_CONTAINER" bash --noprofile
    ;;
  status)
    "${COMPOSE[@]}" ps
    ;;
  stop)
    "${LEADER_COMPOSE[@]}" down
    "${POLICY_COMPOSE[@]}" down
    "${COMPOSE[@]}" down
    ;;
  policy-start|policy-stop)
    action="$1"
    backend="${2:-lerobot}"
    case "$backend" in lerobot|vitacformer|groot) ;; *) printf 'Unknown backend: %s\n' "$backend" >&2; exit 1 ;; esac
    if [ "$action" = policy-start ]; then
      "${POLICY_COMPOSE[@]}" up -d "$backend"
    else
      "${POLICY_COMPOSE[@]}" stop "$backend"
    fi
    ;;
  model-prepare)
    python3 "$ROOT/runtime/hx5_sim/prepare_model.py"
    ;;
  reset)
    shift
    python3 "$ROOT/runtime/hx5_sim/reset_simulation.py" "$@"
    ;;
  leader)
    for device in "${HX5_SIM_LEFT_LEADER_DEVICE:-/dev/left_leader}" "${HX5_SIM_RIGHT_LEADER_DEVICE:-/dev/right_leader}"; do
      if [ ! -c "$device" ]; then printf 'Leader device not found: %s\n' "$device" >&2; exit 1; fi
      if fuser "$device" >/dev/null 2>&1; then printf 'Leader device already in use: %s\n' "$device" >&2; exit 1; fi
    done
    printf '%s\n' 'Stop any existing leader teleoperation first. This command powers ONLY the physical LG2 leader.'
    "${LEADER_COMPOSE[@]}" up -d leader
    ros_exec lg2_leader_1044_hx5_sim flock -n -o /tmp/hx5-leader.lock ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py
    ;;
  leader-stop)
    "${LEADER_COMPOSE[@]}" down
    ;;
  *)
    printf '%s\n' 'Usage: runtime/hx5_sim.sh {build [ai_worker|cyclo]|start|router|gazebo [gui:=false rviz:=false]|cyclo|leader|leader-stop|policy-start [lerobot|vitacformer|groot]|policy-stop [backend]|model-prepare|reset [--check] [--initial-pose inference|navigation|task]|enter|status|stop}' \
      'Cyclo UI: http://localhost:7380/  Robot: FFW SH5 Rev1  ROS domain: 105' \
      'No real follower device is exposed. Physical leader setup is separate.'
    ;;
esac
