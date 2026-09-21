import os
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_1044_wrapper_sets_isolated_defaults(tmp_path):
    docker_dir = tmp_path / "docker"
    docker_dir.mkdir()
    shutil.copy2(
        REPO_ROOT / "docker" / "container_1044.sh",
        docker_dir / "container_1044.sh",
    )
    (docker_dir / "container_1044.sh").chmod(0o755)
    (docker_dir / "container.sh").write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$CYCLO_COMPOSE_PROJECT_NAME\" \"$CYCLO_MAIN_CONTAINER_NAME\" "
        "\"$LEROBOT_CONTAINER_NAME\" \"$VITACFORMER_CONTAINER_NAME\" "
        "\"$GROOT_CONTAINER_NAME\" \"$CYCLO_NAVIGATION_CONTAINER\" "
        "\"$CYCLO_AGENT_SOCKETS_DIR\" "
        "\"$CYCLO_UI_PORT\" \"$CYCLO_ROSBRIDGE_PORT\" "
        "\"$CYCLO_SUPERVISOR_API_PORT\" \"$ROS_DOMAIN_ID\" \"$*\"\n"
    )
    (docker_dir / "container.sh").chmod(0o755)

    env = dict(os.environ)
    for name in (
        "CYCLO_COMPOSE_PROJECT_NAME",
        "CYCLO_MAIN_CONTAINER_NAME",
        "LEROBOT_CONTAINER_NAME",
        "VITACFORMER_CONTAINER_NAME",
        "GROOT_CONTAINER_NAME",
        "CYCLO_NAVIGATION_CONTAINER",
        "CYCLO_1044_NAVIGATION_CONTAINER",
        "CYCLO_AGENT_SOCKETS_DIR",
        "CYCLO_UI_PORT",
        "CYCLO_ROSBRIDGE_PORT",
        "CYCLO_SUPERVISOR_API_PORT",
        "CYCLO_1044_ROS_DOMAIN_ID",
        "ROS_DOMAIN_ID",
    ):
        env.pop(name, None)

    result = subprocess.run(
        [str(docker_dir / "container_1044.sh"), "status"],
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    assert result.stdout.splitlines() == [
        "cyclo_intelligence_1044_sh5",
        "cyclo_intelligence_1044_sh5",
        "lerobot_server_1044_sh5",
        "vitacformer_server_1044_sh5",
        "groot_server_1044_sh5",
        "ai_worker_1044_sh5",
        "/var/run/robotis/agent_sockets/cyclo_intelligence_1044_sh5",
        "7280",
        "7290",
        "7300",
        "104",
        "status",
    ]

    built = subprocess.run(
        [str(docker_dir / "container_1044.sh"), "start"],
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    assert built.stdout.splitlines()[-1] == "start --build"

    inherited_env = {
        **env,
        "ROS_DOMAIN_ID": "73",
        "CYCLO_COMPOSE_PROJECT_NAME": "hero_cyclo_intelligence",
        "CYCLO_MAIN_CONTAINER_NAME": "hero_cyclo_intelligence",
        "CYCLO_UI_PORT": "7180",
    }
    isolated = subprocess.run(
        [str(docker_dir / "container_1044.sh"), "start"],
        env=inherited_env,
        check=True,
        text=True,
        capture_output=True,
    )
    assert isolated.stdout.splitlines() == built.stdout.splitlines()

    overridden = subprocess.run(
        [str(docker_dir / "container_1044.sh"), "status"],
        env={**inherited_env, "CYCLO_1044_ROS_DOMAIN_ID": "105"},
        check=True,
        text=True,
        capture_output=True,
    )
    assert overridden.stdout.splitlines()[-2] == "105"


def _copy_container_script(tmp_path):
    docker_dir = tmp_path / "docker"
    docker_dir.mkdir()
    shutil.copy2(REPO_ROOT / "docker" / "container.sh", docker_dir / "container.sh")
    (docker_dir / "container.sh").chmod(0o755)
    return docker_dir


def _write_start_stub(tmp_path):
    log_path = tmp_path / "docker.log"
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    docker_stub = bin_dir / "docker"
    docker_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_STUB_LOG\"\n"
        "case \" $* \" in\n"
        "  *' config --format json '*)\n"
        "    printf '%s\\n' '{\"services\":{\"lerobot\":{\"image\":\"robotis/lerobot-zenoh:1.4.1-arm64\"},\"vitacformer\":{\"image\":\"robotis/vitacformer-zenoh:1.0.0-arm64\"},\"groot\":{\"image\":\"robotis/groot-zenoh:1.3.5-arm64\"}}}'\n"
        "    ;;\n"
        "esac\n"
        "if [ \"$1\" = image ] && [ \"$2\" = inspect ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "fi\n"
        "exit 0\n"
    )
    docker_stub.chmod(0o755)
    return log_path


def _write_enter_stub(tmp_path, running_container):
    log_path = tmp_path / "docker.log"
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    docker_stub = bin_dir / "docker"
    docker_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_STUB_LOG\"\n"
        "if [ \"$1\" = ps ]; then\n"
        f"  printf '%s\\n' '{running_container}'\n"
        "fi\n"
        "exit 0\n"
    )
    docker_stub.chmod(0o755)
    return log_path


def _stub_env(tmp_path, log_path):
    env = {
        **os.environ,
        "PATH": f"{tmp_path / 'bin'}:{os.environ['PATH']}",
        "DOCKER_STUB_LOG": str(log_path),
        "CYCLO_AGENT_SOCKETS_DIR": str(tmp_path / "agent_sockets"),
    }
    env.pop("CYCLO_LEROBOT_POLICY_FLAVOR", None)
    env.pop("CYCLO_LEROBOT_RUNTIME_PROFILE", None)
    return env


def test_start_does_not_create_legacy_runtime_env_file_from_default(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)

    result = subprocess.run(
        [str(docker_dir / "container.sh"), "start"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    legacy_runtime_env = docker_dir / "workspace" / "config" / "ros_zenoh.env"
    assert not legacy_runtime_env.exists()
    assert "Created ROS/Zenoh runtime env from default" not in result.stdout


def test_start_creates_repo_local_mount_directories(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)

    subprocess.run(
        [str(docker_dir / "container.sh"), "start"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    assert (docker_dir / "workspace" / "dataset").is_dir()
    assert (docker_dir / "workspace" / "rosbag2").is_dir()
    assert (docker_dir / "workspace" / "lerobot").is_dir()
    assert (docker_dir / "workspace" / "model" / "lerobot").is_dir()
    assert (docker_dir / "workspace" / "model" / "vitacformer").is_dir()
    assert (docker_dir / "workspace" / "model" / "groot").is_dir()
    assert (docker_dir / "huggingface").is_dir()


def test_start_preserves_existing_legacy_runtime_env_file_without_touching(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    legacy_runtime_env = docker_dir / "workspace" / "config" / "ros_zenoh.env"
    legacy_runtime_env.parent.mkdir(parents=True)
    legacy_runtime_env.write_text("export ZENOH_ROUTER_IP=192.168.60.139\n")
    log_path = _write_start_stub(tmp_path)

    result = subprocess.run(
        [str(docker_dir / "container.sh"), "start"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    assert legacy_runtime_env.read_text() == "export ZENOH_ROUTER_IP=192.168.60.139\n"
    assert "Created ROS/Zenoh runtime env from default" not in result.stdout


def test_enter_lerobot_uses_plain_bash(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_enter_stub(tmp_path, "lerobot_server")

    subprocess.run(
        [str(docker_dir / "container.sh"), "enter-lerobot"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    legacy_runtime_env = docker_dir / "workspace" / "config" / "ros_zenoh.env"
    docker_calls = log_path.read_text().splitlines()
    assert not legacy_runtime_env.exists()
    assert "exec -it lerobot_server bash" in docker_calls
    assert not any("bash -lc" in call for call in docker_calls)


def test_enter_vitacformer_uses_dedicated_container(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_enter_stub(tmp_path, "vitacformer_server")

    subprocess.run(
        [str(docker_dir / "container.sh"), "enter-vitacformer"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    assert "exec -it vitacformer_server bash" in log_path.read_text().splitlines()


def test_start_pulls_main_image_only(tmp_path):
    log_path = tmp_path / "docker.log"
    docker_stub = tmp_path / "docker"
    docker_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_STUB_LOG\"\n"
        "case \" $* \" in\n"
        "  *' config --format json '*)\n"
        "    printf '%s\\n' '{\"services\":{\"lerobot\":{\"image\":\"robotis/lerobot-zenoh:1.4.1-arm64\"},\"vitacformer\":{\"image\":\"robotis/vitacformer-zenoh:1.0.0-arm64\"},\"groot\":{\"image\":\"robotis/groot-zenoh:1.3.5-arm64\"}}}'\n"
        "    ;;\n"
        "esac\n"
        "if [ \"$1\" = image ] && [ \"$2\" = inspect ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "fi\n"
        "exit 0\n"
    )
    docker_stub.chmod(0o755)

    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "DOCKER_STUB_LOG": str(log_path),
        "CYCLO_AGENT_SOCKETS_DIR": str(tmp_path / "agent_sockets"),
    }

    subprocess.run(
        [str(REPO_ROOT / "docker" / "container.sh"), "start"],
        cwd=REPO_ROOT,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    docker_calls = log_path.read_text().splitlines()
    assert any(
        "pull --ignore-pull-failures cyclo_intelligence" in call
        for call in docker_calls
    )
    assert not any("pull --ignore-pull-failures lerobot" in call for call in docker_calls)
    assert not any("pull --ignore-pull-failures groot" in call for call in docker_calls)


def test_start_groot_build_skips_prebuilt_pull(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-groot", "--build"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    docker_calls = log_path.read_text().splitlines()
    assert not any("pull --ignore-pull-failures groot" in call for call in docker_calls)
    assert any("up -d --build groot" in call for call in docker_calls)


def test_start_vitacformer_build_uses_dedicated_service(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-vitacformer", "--build"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    docker_calls = log_path.read_text().splitlines()
    assert not any(
        "pull --ignore-pull-failures vitacformer" in call
        for call in docker_calls
    )
    assert any("up -d --build vitacformer" in call for call in docker_calls)


def test_trex_flavor_from_dotenv_layers_compose_override(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    (docker_dir / "docker-compose.trex.yml").write_text("services: {}\n")
    (docker_dir / ".env").write_text("CYCLO_LEROBOT_POLICY_FLAVOR=trex\n")
    log_path = _write_start_stub(tmp_path)

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env={
            key: value
            for key, value in _stub_env(tmp_path, log_path).items()
            if key != "CYCLO_LEROBOT_POLICY_FLAVOR"
        },
        check=True,
        text=True,
        capture_output=True,
    )

    trex_arg = f"-f {docker_dir / 'docker-compose.trex.yml'}"
    compose_calls = [
        call
        for call in log_path.read_text().splitlines()
        if call.startswith("compose ")
    ]
    assert compose_calls
    assert all(trex_arg in call for call in compose_calls)


def test_explicit_default_flavor_overrides_trex_dotenv(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    (docker_dir / "docker-compose.trex.yml").write_text("services: {}\n")
    (docker_dir / ".env").write_text("CYCLO_LEROBOT_POLICY_FLAVOR=trex\n")
    log_path = _write_start_stub(tmp_path)
    env = _stub_env(tmp_path, log_path)
    env["CYCLO_LEROBOT_POLICY_FLAVOR"] = "default"

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    assert "docker-compose.trex.yml" not in log_path.read_text()


def test_invalid_lerobot_flavor_fails_before_docker(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)
    env = _stub_env(tmp_path, log_path)
    env["CYCLO_LEROBOT_POLICY_FLAVOR"] = "unknown"

    result = subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env=env,
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "must be 'default' or 'trex'" in result.stderr
    assert not log_path.exists() or not log_path.read_text()


def test_hand_act_runtime_profile_is_layered_last(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    (docker_dir / "docker-compose.trex.yml").write_text("services: {}\n")
    (docker_dir / "docker-compose.override.yml").write_text("services: {}\n")
    (docker_dir / "docker-compose.hand-act.yml").write_text("services: {}\n")
    (docker_dir / ".env").write_text(
        "CYCLO_LEROBOT_POLICY_FLAVOR=trex\n"
        "CYCLO_LEROBOT_RUNTIME_PROFILE=hand-act\n"
    )
    log_path = _write_start_stub(tmp_path)

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env=_stub_env(tmp_path, log_path),
        check=True,
        text=True,
        capture_output=True,
    )

    compose_calls = [
        call for call in log_path.read_text().splitlines() if call.startswith("compose ")
    ]
    expected_suffix = (
        f"-f {docker_dir / 'docker-compose.trex.yml'} "
        f"-f {docker_dir / 'docker-compose.override.yml'} "
        f"-f {docker_dir / 'docker-compose.hand-act.yml'}"
    )
    assert compose_calls
    assert all(expected_suffix in call for call in compose_calls)
    assert all(
        call.rsplit(" -f ", 1)[-1].startswith(
            str(docker_dir / "docker-compose.hand-act.yml")
        )
        for call in compose_calls
    )


def test_explicit_default_runtime_profile_overrides_dotenv(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    (docker_dir / "docker-compose.hand-act.yml").write_text("services: {}\n")
    (docker_dir / ".env").write_text(
        "CYCLO_LEROBOT_RUNTIME_PROFILE=hand-act\n"
    )
    log_path = _write_start_stub(tmp_path)
    env = _stub_env(tmp_path, log_path)
    env["CYCLO_LEROBOT_RUNTIME_PROFILE"] = "default"

    subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    assert "docker-compose.hand-act.yml" not in log_path.read_text()


def test_invalid_lerobot_runtime_profile_fails_before_docker(tmp_path):
    docker_dir = _copy_container_script(tmp_path)
    log_path = _write_start_stub(tmp_path)
    env = _stub_env(tmp_path, log_path)
    env["CYCLO_LEROBOT_RUNTIME_PROFILE"] = "unknown"

    result = subprocess.run(
        [str(docker_dir / "container.sh"), "start-lerobot"],
        cwd=tmp_path,
        env=env,
        check=False,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 2
    assert "must be 'default' or 'hand-act'" in result.stderr
    assert not log_path.exists() or not log_path.read_text()


def test_start_does_not_remove_policy_container_with_stale_workspace_mount(tmp_path):
    log_path = tmp_path / "docker.log"
    docker_stub = tmp_path / "docker"
    docker_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_STUB_LOG\"\n"
        "case \" $* \" in\n"
        "  *' config --format json '*)\n"
        "    printf '%s\\n' '{\"services\":{\"lerobot\":{\"image\":\"robotis/lerobot-zenoh:1.4.1-arm64\"},\"groot\":{\"image\":\"robotis/groot-zenoh:1.3.5-arm64\"}}}'\n"
        "    exit 0\n"
        "    ;;\n"
        "esac\n"
        "if [ \"$1\" = image ] && [ \"$2\" = inspect ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"$1\" = inspect ] && [ \"$2\" = -f ] && [ \"$3\" = '{{.Image}}' ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"$1\" = inspect ] && [ \"$2\" = -f ]; then\n"
        "  case \"$3\" in\n"
        "    *'.Destination \"/workspace\"'*)\n"
        "      if [ \"$4\" = lerobot_server ]; then\n"
        "        printf '%s\\n' '/old/workspace'\n"
        "      else\n"
        "        printf '%s\\n' \"$EXPECTED_WORKSPACE_DIR\"\n"
        "      fi\n"
        "      exit 0\n"
        "      ;;\n"
        "  esac\n"
        "fi\n"
        "exit 0\n"
    )
    docker_stub.chmod(0o755)

    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "DOCKER_STUB_LOG": str(log_path),
        "CYCLO_AGENT_SOCKETS_DIR": str(tmp_path / "agent_sockets"),
        "EXPECTED_WORKSPACE_DIR": str(REPO_ROOT / "docker" / "workspace"),
    }

    subprocess.run(
        [str(REPO_ROOT / "docker" / "container.sh"), "start"],
        cwd=REPO_ROOT,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    docker_calls = log_path.read_text().splitlines()
    assert "rm -f lerobot_server" not in docker_calls
    assert "rm -f groot_server" not in docker_calls


def test_start_lerobot_removes_stale_workspace_mount(tmp_path):
    log_path = tmp_path / "docker.log"
    docker_stub = tmp_path / "docker"
    docker_stub.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_STUB_LOG\"\n"
        "case \" $* \" in\n"
        "  *' config --format json '*)\n"
        "    printf '%s\\n' '{\"services\":{\"lerobot\":{\"image\":\"robotis/lerobot-zenoh:1.4.1-arm64\"},\"groot\":{\"image\":\"robotis/groot-zenoh:1.3.5-arm64\"}}}'\n"
        "    exit 0\n"
        "    ;;\n"
        "esac\n"
        "if [ \"$1\" = image ] && [ \"$2\" = inspect ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"$1\" = inspect ] && [ \"$2\" = -f ] && [ \"$3\" = '{{.Image}}' ]; then\n"
        "  printf '%s\\n' 'sha256:current'\n"
        "  exit 0\n"
        "fi\n"
        "if [ \"$1\" = inspect ] && [ \"$2\" = -f ]; then\n"
        "  case \"$3\" in\n"
        "    *'.Destination \"/workspace\"'*)\n"
        "      printf '%s\\n' '/old/workspace'\n"
        "      exit 0\n"
        "      ;;\n"
        "  esac\n"
        "fi\n"
        "exit 0\n"
    )
    docker_stub.chmod(0o755)

    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "DOCKER_STUB_LOG": str(log_path),
        "CYCLO_AGENT_SOCKETS_DIR": str(tmp_path / "agent_sockets"),
    }

    subprocess.run(
        [str(REPO_ROOT / "docker" / "container.sh"), "start-lerobot"],
        cwd=REPO_ROOT,
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )

    docker_calls = log_path.read_text().splitlines()
    assert any(
        "pull --ignore-pull-failures lerobot" in call
        for call in docker_calls
    )
    assert not any("pull --ignore-pull-failures groot" in call for call in docker_calls)
    assert any(
        call == "rm -f lerobot_server"
        for call in docker_calls
    )
