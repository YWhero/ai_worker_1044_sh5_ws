import os
from pathlib import Path
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("targets, expected", [
    ([], "cyclo_intelligence"),
    (["main", "vitacformer"], "cyclo_intelligence vitacformer"),
    (["lerobot"], "lerobot"),
])
def test_build_only_never_starts_or_removes_containers(tmp_path, targets, expected):
    directory = tmp_path / "docker"
    directory.mkdir()
    for name in ("container.sh", "container_1044.sh"):
        shutil.copy2(ROOT / "docker" / name, directory / name)
    executable = tmp_path / "docker-bin"
    executable.mkdir()
    log = tmp_path / "calls.log"
    stub = executable / "docker"
    stub.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CALL_LOG"\n')
    stub.chmod(0o755)
    subprocess.run(
        ["bash", str(directory / "container_1044.sh"), "build", *targets],
        env={**os.environ, "PATH": str(executable) + ":" + os.environ["PATH"],
             "CALL_LOG": str(log)},
        check=True, capture_output=True, text=True,
    )
    calls = log.read_text().splitlines()
    assert any(line.endswith(" build " + expected) for line in calls)
    for line in calls:
        assert not set(line.split()) & {"up", "start", "stop", "restart", "rm", "down", "run"}
