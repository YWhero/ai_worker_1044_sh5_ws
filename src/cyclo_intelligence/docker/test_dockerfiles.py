import os
from pathlib import Path
import subprocess
import tempfile
import tomllib
import re


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_main_dockerfiles_install_compose_v2():
    for dockerfile in (
        REPO_ROOT / "docker" / "Dockerfile.arm64",
        REPO_ROOT / "docker" / "Dockerfile.amd64",
    ):
        contents = dockerfile.read_text()

        assert "docker-compose-v2" in contents, (
            f"{dockerfile.name} must install docker-compose-v2 so "
            "supervisor_api can recreate policy containers with docker compose"
        )


def test_main_compose_mounts_shared_s6_runner():
    contents = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()

    assert (
        "./s6-services/common/ros2_service_run.sh:"
        "/usr/local/lib/s6-services/ros2_service_run.sh:ro"
    ) in contents


def test_interactive_bashrc_includes_simple_ros_zenoh_block():
    dockerfiles = (
        REPO_ROOT / "docker" / "Dockerfile.arm64",
        REPO_ROOT / "docker" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.amd64",
    )
    required = (
        'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"',
        "export RMW_IMPLEMENTATION=rmw_zenoh_cpp",
        "export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true'",
        "# export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=true;mode=\\\"client\\\";connect/endpoints=[\\\"tcp/192.168.60.139:7447\\\"]'",
    )
    removed = (
        "export ZENOH_ROUTER_IP=${ZENOH_ROUTER_IP:-127.0.0.1}",
        "export ZENOH_ROUTER_PORT=${ZENOH_ROUTER_PORT:-7447}",
        "if [ \"${ZENOH_ROUTER_IP}\" = \"127.0.0.1\" ]",
        "export ZENOH_TRANSPORT_SHM_ENABLED=${ZENOH_TRANSPORT_SHM_ENABLED:-true}",
        "export ZENOH_SHM_ENABLED=${ZENOH_SHM_ENABLED:-true}",
    )

    for dockerfile in dockerfiles:
        contents = dockerfile.read_text()
        for expected in required:
            assert expected in contents, f"{dockerfile} is missing {expected}"
        for unexpected in removed:
            assert unexpected not in contents, f"{dockerfile} still has {unexpected}"


def test_dockerfiles_prepend_ros_zenoh_block_before_existing_bashrc():
    dockerfiles = (
        REPO_ROOT / "docker" / "Dockerfile.arm64",
        REPO_ROOT / "docker" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.amd64",
    )

    for dockerfile in dockerfiles:
        contents = dockerfile.read_text()
        write_block_index = contents.index("> /tmp/cyclo_bashrc")
        existing_bashrc_index = contents.index("cat /root/.bashrc >> /tmp/cyclo_bashrc")
        assert write_block_index < existing_bashrc_index, (
            f"{dockerfile} must write Cyclo env before appending the base bashrc"
        )


def test_ros_zenoh_runtime_env_file_is_not_referenced_by_images_or_s6():
    paths = (
        REPO_ROOT / "docker" / "Dockerfile.arm64",
        REPO_ROOT / "docker" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.amd64",
        REPO_ROOT / "docker" / "s6-services" / "common" / "ros2_service_run.sh",
        REPO_ROOT / "cyclo_brain" / "policy" / "common" / "s6-services" / "main-runtime" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "common" / "s6-services" / "engine-process" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "s6-services" / "main-runtime" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "s6-services" / "engine-process" / "run",
    )

    for path in paths:
        contents = path.read_text()
        assert "CYCLO_ROS_ENV_FILE" not in contents, f"{path} references the old env file hook"
        assert "ros_zenoh.env" not in contents, f"{path} references the old runtime env file"


def test_s6_services_run_through_interactive_bashrc_shell():
    paths = (
        REPO_ROOT / "docker" / "s6-services" / "common" / "ros2_service_run.sh",
        REPO_ROOT / "cyclo_brain" / "policy" / "common" / "s6-services" / "main-runtime" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "common" / "s6-services" / "engine-process" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "s6-services" / "main-runtime" / "run",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "s6-services" / "engine-process" / "run",
    )

    for path in paths:
        contents = path.read_text()
        assert "bash -ic" in contents, f"{path} does not run through an interactive bashrc shell"
        assert "source /root/.bashrc" not in contents, f"{path} manually sources /root/.bashrc"
        assert "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}" in contents
        assert (
            "export ZENOH_CONFIG_OVERRIDE=${ZENOH_CONFIG_OVERRIDE:-transport/shared_memory/enabled=true}"
            in contents
        )
        assert "export ZENOH_ROUTER_IP=${ZENOH_ROUTER_IP:-127.0.0.1}" not in contents


def test_compose_does_not_override_ros_zenoh_runtime_env():
    contents = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()

    removed_entries = (
        "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}",
        "RMW_IMPLEMENTATION=rmw_zenoh_cpp",
        "CYCLO_ROS_ENV_FILE=",
        "ZENOH_ROUTER_IP=127.0.0.1",
        "ZENOH_ROUTER_PORT=7447",
        "ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=true",
        "ZENOH_TRANSPORT_SHM_ENABLED=true",
        "ZENOH_SHM_ENABLED=true",
    )
    for entry in removed_entries:
        assert entry not in contents


def test_cyclo_runtime_ports_avoid_physical_ai_defaults():
    compose = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()
    nginx_run = (REPO_ROOT / "docker" / "s6-services" / "nginx" / "run").read_text()
    supervisor_run = (
        REPO_ROOT / "docker" / "s6-services" / "supervisor_api" / "run"
    ).read_text()
    launch = (
        REPO_ROOT / "orchestrator" / "launch" / "orchestrator_bringup.launch.py"
    ).read_text()
    orchestrator = (
        REPO_ROOT / "orchestrator" / "orchestrator" / "orchestrator_node.py"
    ).read_text()
    ui_config = (
        REPO_ROOT / "orchestrator" / "ui" / "public" / "cyclo-config.js"
    ).read_text()

    expected_defaults = (
        "CYCLO_UI_PORT=${CYCLO_UI_PORT:-7080}",
        "CYCLO_ROSBRIDGE_PORT=${CYCLO_ROSBRIDGE_PORT:-7090}",
        "CYCLO_VIDEO_SERVER_PORT=${CYCLO_VIDEO_SERVER_PORT:-7082}",
        "CYCLO_WEB_VIDEO_SERVER_PORT=${CYCLO_WEB_VIDEO_SERVER_PORT:-7085}",
        "CYCLO_SUPERVISOR_API_PORT=${CYCLO_SUPERVISOR_API_PORT:-7100}",
    )
    for expected in expected_defaults:
        assert expected in compose

    assert 'UI_PORT="$(port_or_default ' in nginx_run
    assert "CYCLO_UI_PORT:-}\" 7080" in nginx_run
    assert "CYCLO_ROSBRIDGE_PORT:-}\" 7090" in nginx_run
    assert "CYCLO_VIDEO_SERVER_PORT:-}\" 7082" in nginx_run
    assert "CYCLO_WEB_VIDEO_SERVER_PORT:-}\" 7085" in nginx_run
    assert "CYCLO_SUPERVISOR_API_PORT:-}\" 7100" in nginx_run
    assert 'PORT="${CYCLO_SUPERVISOR_API_PORT:-7100}"' in supervisor_run
    assert "_env_int('CYCLO_ROSBRIDGE_PORT', '7090')" in launch
    assert "_env_int('CYCLO_WEB_VIDEO_SERVER_PORT', '7085')" in launch
    assert "_env_int('CYCLO_VIDEO_SERVER_PORT', '7082')" in orchestrator
    assert "uiPort: 7080" in ui_config
    assert "rosbridgePort: 7090" in ui_config
    assert "videoServerPort: 7082" in ui_config
    assert "webVideoServerPort: 7085" in ui_config
    assert "supervisorApiPort: 7100" in ui_config

    for old_default in (
        "CYCLO_UI_PORT=${CYCLO_UI_PORT:-80}",
        "CYCLO_ROSBRIDGE_PORT=${CYCLO_ROSBRIDGE_PORT:-9090}",
        "CYCLO_VIDEO_SERVER_PORT=${CYCLO_VIDEO_SERVER_PORT:-8082}",
        "CYCLO_WEB_VIDEO_SERVER_PORT=${CYCLO_WEB_VIDEO_SERVER_PORT:-8085}",
        "CYCLO_SUPERVISOR_API_PORT=${CYCLO_SUPERVISOR_API_PORT:-8100}",
    ):
        assert old_default not in compose


def test_nginx_runtime_port_rewrite_keeps_data_api_on_video_server():
    nginx_run = (REPO_ROOT / "docker" / "s6-services" / "nginx" / "run").read_text()
    nginx_conf = (REPO_ROOT / "orchestrator" / "ui" / "nginx.conf").read_text()

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        conf_path = tmp_path / "default.conf"
        config_path = tmp_path / "cyclo-config.js"
        script_path = tmp_path / "run-nginx-test.sh"

        conf_path.write_text(nginx_conf)
        test_script = nginx_run.split('exec nginx -g "daemon off;"')[0]
        test_script = test_script.replace(
            "/etc/nginx/conf.d/default.conf",
            str(conf_path),
        )
        test_script = test_script.replace(
            "/usr/share/nginx/html/cyclo-config.js",
            str(config_path),
        )
        script_path.write_text(test_script)

        subprocess.run(
            ["bash", str(script_path)],
            check=True,
            text=True,
            capture_output=True,
            env={
                **os.environ,
                "CYCLO_UI_PORT": "7080",
                "CYCLO_ROSBRIDGE_PORT": "7090",
                "CYCLO_VIDEO_SERVER_PORT": "7082",
                "CYCLO_WEB_VIDEO_SERVER_PORT": "7085",
                "CYCLO_SUPERVISOR_API_PORT": "7100",
            },
        )

        rewritten = conf_path.read_text()
        assert "listen 7080;" in rewritten
        assert "location /api/" in rewritten
        assert "proxy_pass http://127.0.0.1:7100/;" in rewritten
        assert "location /data-api/" in rewritten
        assert "proxy_pass http://127.0.0.1:7082/;" in rewritten
        assert rewritten.count("proxy_pass http://127.0.0.1:7100/;") == 1
        assert rewritten.count("proxy_pass http://127.0.0.1:7082/;") == 1

        runtime_config = config_path.read_text()
        assert "rosbridgePort: 7090" in runtime_config
        assert "videoServerPort: 7082" in runtime_config


def test_policy_compose_keeps_image_defaults_in_images():
    compose = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()
    duplicated_entries = (
        "ZENOH_SDK_PATH=/zenoh_sdk",
        "ROBOT_CLIENT_SDK_PATH=/robot_client_sdk",
        "ACTION_CHUNK_PROCESSING_SDK_PATH=/action_chunk_processing_sdk",
        "POLICY_BACKEND=lerobot",
        "POLICY_ENGINE_MODULE=lerobot_engine",
        "POLICY_BACKEND=groot",
        "POLICY_ENGINE_MODULE=groot_engine",
        "GROOT_TRT_ENABLED=false",
        "CONTROL_HZ=100",
        "INFERENCE_HZ=15",
        "TARGET_CHUNK_SIZE=none",
        "REFILL_MARGIN_S=0.2",
        "REFILL_LATENCY_WARMUP_SAMPLES=1",
        "REFILL_LATENCY_SAMPLE_MAX_S=2.0",
        "HF_HOME=/root/.cache/huggingface",
        "HUGGINGFACE_HUB_CACHE=/root/.cache/huggingface/hub",
        "TRANSFORMERS_CACHE=/root/.cache/huggingface/hub",
    )
    for entry in duplicated_entries:
        assert entry not in compose

    policy_dockerfiles = (
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.amd64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "vitacformer" / "Dockerfile.amd64",
    )
    for dockerfile in policy_dockerfiles:
        contents = dockerfile.read_text()
        assert "ENV ZENOH_SDK_PATH=/zenoh_sdk" in contents
        assert "ENV ROBOT_CLIENT_SDK_PATH=/robot_client_sdk" in contents
        assert "ENV ACTION_CHUNK_PROCESSING_SDK_PATH=/action_chunk_processing_sdk" in contents


def test_lerobot_trex_flavor_is_isolated_and_pinned():
    default_extras = {
        "Dockerfile.arm64": ".[training,smolvla,xvla,fastwam,hilserl,async,peft]",
        "Dockerfile.amd64": ".[dataset,smolvla,xvla,fastwam,hilserl,async,peft]",
    }
    for filename, normal_install in default_extras.items():
        dockerfile = (
            REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / filename
        )
        contents = dockerfile.read_text()
        assert "ARG LEROBOT_POLICY_FLAVOR=default" in contents
        assert normal_install in contents
        assert 'trex) ' in contents and '".[training,async]"' in contents
        assert '"transformers==4.57.3"' in contents
        assert '"tokenizers==0.22.2"' in contents
        assert '"huggingface-hub==0.36.2"' in contents
        assert "--no-deps" in contents
        assert "from transformers import Qwen3VLForConditionalGeneration" in contents


def test_trex_compose_override_preserves_lerobot_runtime_identity():
    compose = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()
    trex = (REPO_ROOT / "docker" / "docker-compose.trex.yml").read_text()
    supervisor = (REPO_ROOT / "docker" / "supervisor_api" / "app.py").read_text()
    orchestrator = (
        REPO_ROOT / "orchestrator" / "orchestrator" / "orchestrator_node.py"
    ).read_text()
    bt_action = (
        REPO_ROOT
        / "orchestrator"
        / "orchestrator"
        / "bt"
        / "actions"
        / "send_command.py"
    ).read_text()
    model_selector = (
        REPO_ROOT
        / "orchestrator"
        / "ui"
        / "src"
        / "components"
        / "InferenceModelSelector.js"
    ).read_text()

    assert "LEROBOT_POLICY_FLAVOR: ${LEROBOT_POLICY_FLAVOR:-default}" in compose
    assert "CYCLO_LEROBOT_POLICY_FLAVOR=${CYCLO_LEROBOT_POLICY_FLAVOR:-default}" in compose
    assert "robotis/lerobot-trex-zenoh:1.3.2-${ARCH:-arm64}" in trex
    assert "LEROBOT_POLICY_FLAVOR: trex" in trex
    assert "CYCLO_LEROBOT_POLICY_FLAVOR=trex" in trex
    assert "HF_HUB_CACHE=/root/.cache/huggingface/hub" in trex
    assert "HUGGINGFACE_HUB_CACHE=/root/.cache/huggingface/hub" in trex
    assert "TRANSFORMERS_CACHE=/root/.cache/huggingface/hub" in trex
    assert "INFERENCE_HZ=${TREX_RUNTIME_ACTION_HZ:-1.25}" in trex
    assert "REFILL_MARGIN_S=${TREX_REFILL_MARGIN_S:-2.0}" in trex
    assert "TREX_RUNTIME_ACTION_HZ=${TREX_RUNTIME_ACTION_HZ:-1.25}" in trex
    assert "TREX_REFILL_MARGIN_S=${TREX_REFILL_MARGIN_S:-2.0}" in trex
    assert "container_name:" not in trex
    assert '"CYCLO_LEROBOT_POLICY_FLAVOR", "default"' in supervisor
    assert '"docker-compose.trex.yml"' in supervisor
    assert "'smolvla', 'trex', 'xvla'" in orchestrator
    assert "'lerobot:trex': 'lerobot'" in bt_action
    assert "value: 'lerobot:trex'" in model_selector
    assert "'lerobot:vitacformer': 'vitacformer'" in bt_action
    assert "value: 'vitacformer:vitacformer'" in model_selector


def test_vitacformer_is_a_baked_dedicated_backend():
    compose = (REPO_ROOT / "docker" / "docker-compose.yml").read_text()
    supervisor = (REPO_ROOT / "docker" / "supervisor_api" / "app.py").read_text()
    lerobot_engine = REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "lerobot_engine"

    assert "container_name: ${VITACFORMER_CONTAINER_NAME:-vitacformer_server}" in compose
    assert "robotis/vitacformer-zenoh:1.0.0-${ARCH:-arm64}" in compose
    assert "policy/vitacformer/Dockerfile.${ARCH:-arm64}" in compose
    assert "../cyclo_brain/policy/vitacformer" not in compose
    assert '"VITACFORMER_CONTAINER_NAME", "vitacformer_server"' in supervisor
    assert not (lerobot_engine / "vitacformer.py").exists()

    for architecture in ("arm64", "amd64"):
        dockerfile = (
            REPO_ROOT
            / "cyclo_brain"
            / "policy"
            / "vitacformer"
            / f"Dockerfile.{architecture}"
        ).read_text()
        assert "COPY policy/vitacformer/vitacformer_engine/ /app/vitacformer_engine/" in dockerfile
        assert "COPY policy/common/runtime/ /policy_runtime/" in dockerfile
        assert "ENV POLICY_BACKEND=vitacformer" in dockerfile
        assert "ENV POLICY_ENGINE_MODULE=vitacformer_engine" in dockerfile
        assert "ENV INFERENCE_HZ=30" in dockerfile
def test_lerobot_extras_preserve_baseline_with_fastwam_build_backport():
    project = tomllib.loads((REPO_ROOT / "cyclo_brain/policy/lerobot/lerobot/pyproject.toml").read_text())
    available = project["project"]["optional-dependencies"]
    dockerfiles = (
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.arm64",
        REPO_ROOT / "cyclo_brain" / "policy" / "lerobot" / "Dockerfile.amd64",
    )

    for dockerfile in dockerfiles:
        install_lines = [
            line
            for line in dockerfile.read_text().splitlines()
            if "default)" in line and "pip install" in line and '".[' in line
        ]
        assert len(install_lines) == 1, f"Could not identify LeRobot extras in {dockerfile}"
        install_line = install_lines[0]
        extras = re.search(r'\.\[([^]]+)\]', install_line).group(1).split(',')
        assert set(extras) - set(available) <= {"fastwam"}
        assert "git apply --check /tmp/fastwam-c8ce413.patch" in dockerfile.read_text()
        assert {"smolvla", "xvla", "fastwam", "hilserl", "async", "peft"} <= set(extras)


def test_vitacformer_keeps_base_numpy_abi_compatible_with_opencv():
    root = REPO_ROOT / "cyclo_brain/policy/vitacformer"
    for architecture in ("amd64", "arm64"):
        contents = (root / f"Dockerfile.{architecture}").read_text()
        assert '"opencv-python-headless>=4.5,<4.12"' in contents
        if architecture == "amd64":
            assert '"numpy==1.26.4"' in contents


def test_groot_amd64_keeps_numpy_compatible_with_opencv():
    dockerfile = (
        REPO_ROOT / "cyclo_brain" / "policy" / "groot" / "Dockerfile.amd64"
    )
    contents = dockerfile.read_text()

    assert contents.count('"numpy==1.26.4"') >= 3
    assert contents.count('"ml_dtypes==0.5.4"') >= 2
