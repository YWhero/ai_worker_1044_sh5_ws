"""Render the real s6 script against temporary files without starting nginx."""

import os
from pathlib import Path
import shutil
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def renderer(tmp_path):
    config = tmp_path / "default.conf"
    runtime = tmp_path / "cyclo-config.js"
    shutil.copy2(REPO_ROOT / "orchestrator/ui/nginx.conf", config)
    script = tmp_path / "run"
    source = (REPO_ROOT / "docker/s6-services/nginx/run").read_text()
    source = source.replace("/etc/nginx/conf.d/default.conf", str(config))
    source = source.replace("/usr/share/nginx/html/cyclo-config.js", str(runtime))
    script.write_text(source)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    nginx = bin_dir / "nginx"
    nginx.write_text("#!/bin/sh\nexit 0\n")
    nginx.chmod(0o755)
    env = {key: value for key, value in os.environ.items() if not key.startswith("CYCLO_")}
    env.update({
        "PATH": f"{bin_dir}:{env['PATH']}",
        "CYCLO_UI_PORT": "7880",
        "CYCLO_ROSBRIDGE_PORT": "7890",
        "CYCLO_SUPERVISOR_API_PORT": "7900",
        "CYCLO_VIDEO_SERVER_PORT": "7882",
    })

    def render(**overrides):
        result = subprocess.run(
            ["bash", str(script)], env={**env, **overrides},
            text=True, capture_output=True, check=False,
        )
        return result, config.read_text(), runtime.read_text() if runtime.exists() else ""

    return render


def test_proxy_forwards_websocket_to_isaac_bridge(renderer):
    result, config, runtime = renderer(CYCLO_ROSBRIDGE_PATH="/rosbridge/")
    assert result.returncode == 0, result.stderr
    assert "location = /rosbridge/ {" in config
    assert "proxy_pass http://127.0.0.1:7890/;" in config
    block = config.split("# CYCLO_ROSBRIDGE_PROXY_BEGIN", 1)[1].split("# CYCLO_ROSBRIDGE_PROXY_END", 1)[0]
    for directive in (
        "proxy_http_version 1.1;",
        "proxy_set_header Upgrade $http_upgrade;",
        'proxy_set_header Connection "upgrade";',
        "proxy_read_timeout 3600s;",
        "proxy_buffering off;",
    ):
        assert directive in block
    assert "rosbridgePath: '/rosbridge/'," in runtime
    assert "rosbridgePort: 7890," in runtime
    assert "listen 7880;" in config
    assert "http://127.0.0.1:7900/navigation/topics/ws;" in config
    assert "http://127.0.0.1:7900/;" in config
    assert "http://127.0.0.1:7882/;" in config


def test_default_profile_keeps_direct_bridge_without_proxy(renderer):
    result, config, runtime = renderer()
    assert result.returncode == 0, result.stderr
    assert "CYCLO_ROSBRIDGE_PROXY_BEGIN" not in config
    assert "location = /rosbridge/" not in config
    assert "rosbridgePath: ''," in runtime


def test_restarts_update_ports_and_do_not_duplicate_proxy(renderer):
    renderer(CYCLO_ROSBRIDGE_PATH="/rosbridge/")
    result, config, runtime = renderer(CYCLO_ROSBRIDGE_PATH="/rosbridge/", CYCLO_ROSBRIDGE_PORT="8890")
    assert result.returncode == 0, result.stderr
    assert config.count("location = /rosbridge/") == 1
    assert config.count("CYCLO_ROSBRIDGE_PROXY_BEGIN") == 1
    assert "http://127.0.0.1:8890/;" in config
    assert "http://127.0.0.1:7890/;" not in config
    assert "rosbridgePort: 8890," in runtime


def test_unsetting_path_removes_previous_proxy(renderer):
    renderer(CYCLO_ROSBRIDGE_PATH="/rosbridge/")
    result, config, runtime = renderer()
    assert result.returncode == 0, result.stderr
    assert "CYCLO_ROSBRIDGE_PROXY_BEGIN" not in config
    assert "rosbridgePath: ''," in runtime


def test_absolute_path_is_normalized(renderer):
    result, config, runtime = renderer(CYCLO_ROSBRIDGE_PATH="/robot/ros_bridge")
    assert result.returncode == 0, result.stderr
    assert "location = /robot/ros_bridge/ {" in config
    assert "rosbridgePath: '/robot/ros_bridge/'," in runtime


@pytest.mark.parametrize("path", ["/", "//remote/bridge", "/bridge;bad", "/bridge'bad", "/bridge?x=1", "/../bridge", "https://remote/bridge"])
def test_invalid_paths_fail_before_mutating_config(renderer, path):
    baseline = (REPO_ROOT / "orchestrator/ui/nginx.conf").read_text()
    result, config, runtime = renderer(CYCLO_ROSBRIDGE_PATH=path)
    assert result.returncode == 1
    assert "CYCLO_ROSBRIDGE_PATH" in result.stderr
    assert config == baseline
    assert runtime == ""
