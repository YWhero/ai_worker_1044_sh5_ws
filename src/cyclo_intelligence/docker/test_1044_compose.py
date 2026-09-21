import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("flavor", ["default", "trex"])
def test_cli_and_supervisor_resolve_identical_isolated_services(tmp_path, flavor):
    wrapper = tmp_path / "container_1044.sh"
    shutil.copy2(ROOT / "docker/container_1044.sh", wrapper)
    inner = tmp_path / "container.sh"
    inner.write_text("#!/bin/sh\nenv -0\n")
    inner.chmod(0o755)
    output = subprocess.check_output(["bash", str(wrapper), "status"], env={
        **os.environ, "ROS_DOMAIN_ID": "73", "CYCLO_LEROBOT_POLICY_FLAVOR": flavor,
    })
    environment = dict(entry.split("=", 1) for entry in output.decode().split("\0") if entry)
    environment["ARCH"] = "amd64"
    command = ["docker", "compose", "-f", str(ROOT / "docker/docker-compose.yml")]
    if flavor == "trex":
        command += ["-f", str(ROOT / "docker/docker-compose.trex.yml")]
    command += ["config", "--format", "json"]
    cli = json.loads(subprocess.check_output(command, env=environment))
    main = cli["services"]["cyclo_intelligence"]
    inside = json.loads(subprocess.check_output(command, env={
        "PATH": os.environ["PATH"], "ARCH": "amd64", **main["environment"],
    }))
    assert cli["name"] == inside["name"] == "cyclo_intelligence_1044_sh5"
    for service in ("cyclo_intelligence", "lerobot", "vitacformer", "groot"):
        outside_service = cli["services"][service]
        inside_service = inside["services"][service]
        assert outside_service["image"] == inside_service["image"]
        assert outside_service["image"].startswith("cyclo-1044-sh5/")
        assert outside_service["container_name"] == inside_service["container_name"]
        assert inside_service["environment"]["ROS_DOMAIN_ID"] == "104"
    build_source = main["environment"]["CYCLO_BUILD_SOURCE_DIR"]
    mount = next(volume for volume in main["volumes"] if volume["target"] == build_source)
    assert mount["source"] == build_source
    assert mount["read_only"] is True
