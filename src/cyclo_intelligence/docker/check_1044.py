import json
import os
from pathlib import Path
import subprocess
import sys
import tomllib


REPO_ROOT = Path(__file__).resolve().parents[1]


def fastwam_backport_available(root):
    backport = root / "cyclo_brain/policy/lerobot/backports"
    required = (
        "fastwam/configuration_fastwam.py", "fastwam/modeling_fastwam.py",
        "fastwam/processor_fastwam.py", "fastwam-c8ce413.patch",
    )
    if not all((backport / name).is_file() for name in required):
        return False
    try:
        result = subprocess.run(
            ["git", "apply", "--check", str(backport / "fastwam-c8ce413.patch")],
            cwd=root / "cyclo_brain/policy/lerobot/lerobot",
            capture_output=True, text=True, timeout=10, check=False,
        )
        return result.returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def source_checks(root):
    backport_ready = fastwam_backport_available(root)
    required = {
        "LeRobot": "cyclo_brain/policy/lerobot/lerobot/pyproject.toml",
        "Zenoh SDK": "cyclo_brain/sdk/zenoh_ros2_sdk/pyproject.toml",
        "GR00T": "cyclo_brain/policy/groot/Isaac-GR00T/pyproject.toml",
        "native T-Rex": "cyclo_brain/policy/lerobot/lerobot/src/lerobot/policies/trex/configuration_trex.py",
        "FastWAM": "cyclo_brain/policy/lerobot/lerobot/src/lerobot/policies/fastwam/configuration_fastwam.py",
    }
    findings = []
    for name, relative in required.items():
        present = (root / relative).is_file()
        if name == "FastWAM" and not present and backport_ready:
            findings.append((True, name, "build-time backport available; policy image validation still required"))
        else:
            findings.append((present, name, relative if not present else "source present"))
    project_path = root / required["LeRobot"]
    if project_path.is_file():
        project = tomllib.loads(project_path.read_text())
        extras = project["project"].get("optional-dependencies", {})
        findings.append(("fastwam" in extras or backport_ready, "FastWAM dependency extra",
                         "build-time backport" if backport_ready else "required by baseline Dockerfile"))
    return findings


def inspect_docker(kind, name):
    result = subprocess.run(
        ["docker", kind, "inspect", name], capture_output=True, text=True,
        timeout=15, check=False,
    )
    if result.returncode:
        return None
    return json.loads(result.stdout)[0]


def runtime_checks():
    findings = []
    for variable in (
        "CYCLO_MAIN_IMAGE", "CYCLO_LEROBOT_IMAGE", "CYCLO_LEROBOT_TREX_IMAGE",
        "CYCLO_VITACFORMER_IMAGE", "CYCLO_GROOT_IMAGE",
    ):
        name = os.environ[variable]
        findings.append((inspect_docker("image", name) is not None, variable, name))
    name = os.environ["CYCLO_NAVIGATION_CONTAINER"]
    container = inspect_docker("container", name)
    findings.append((bool(container and container["State"]["Running"]), "Navigation container", name))
    if container:
        environment = dict(entry.split("=", 1) for entry in container["Config"].get("Env", []) if "=" in entry)
        domain = environment.get("ROS_DOMAIN_ID", "unset")
        findings.append((domain == os.environ["ROS_DOMAIN_ID"], "Navigation ROS domain", domain))
    return findings


def main():
    if os.environ.get("CYCLO_COMPOSE_PROJECT_NAME") != "cyclo_intelligence_1044_sh5":
        print("Run: ./docker/container_1044.sh check", file=sys.stderr)
        return 2
    findings = source_checks(REPO_ROOT)
    try:
        findings.extend(runtime_checks())
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        findings.append((False, "Docker inspection", str(error)))
    for ready, name, detail in findings:
        print(f"{'OK' if ready else 'MISSING'} | {name} | {detail}")
    print("Read-only prerequisite check. Does not start containers or command the robot.")
    print("Sensor data, hand preset services, Nav2 and checkpoint inference still require live validation.")
    return 0 if all(ready for ready, _, _ in findings) else 1


if __name__ == "__main__":
    sys.exit(main())
