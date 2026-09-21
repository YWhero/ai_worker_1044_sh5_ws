#!/command/with-contenv bash
# Same two-process entry points as the official policy services, without
# interactive .bashrc overriding this profile's dedicated Zenoh endpoint.
set -euo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-105}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_SHM_ENABLED=false
export ZENOH_TRANSPORT_SHM_ENABLED=false
export PYTHONPATH="/app:/policy_runtime:${ZENOH_SDK_PATH:-/zenoh_sdk}:${ROBOT_CLIENT_SDK_PATH:-/robot_client_sdk}:${ACTION_CHUNK_PROCESSING_SDK_PATH:-/action_chunk_processing_sdk}${PYTHONPATH:+:$PYTHONPATH}"
service_name="$(basename "$(dirname "$0")")"
# s6-supervise executes ./run from the service directory, rather than
# passing the absolute path used by this profile's read-only bind mount.
if [ "$service_name" = . ]; then service_name="$(basename "$(pwd -P)")"; fi
case "$service_name" in
  main-runtime) exec python3 -m main_runtime ;;
  engine-process)
    python3 /policy_runtime/hf_token_sync.py || true
    exec python3 -m engine_process
    ;;
  *) printf 'Unsupported policy service path: %s\n' "$0" >&2; exit 1 ;;
esac
