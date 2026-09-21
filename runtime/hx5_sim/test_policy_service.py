"""s6 invokes ./run; service dispatch must preserve the simulation router."""
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('service,module', [
    ('main-runtime', 'main_runtime'), ('engine-process', 'engine_process'),
])
@pytest.mark.parametrize('relative', [True, False])
def test_entrypoint_and_endpoint(tmp_path, service, module, relative):
    service_dir = tmp_path / service
    service_dir.mkdir()
    entry = service_dir / 'run'
    entry.write_text(Path(__file__).with_name('policy_service.sh').read_text())
    binary_dir = tmp_path / 'bin'
    binary_dir.mkdir()
    python = binary_dir / 'python3'
    python.write_text('#!/bin/sh\nprintf "%s|%s|%s\\n" "$*" "$ROS_DOMAIN_ID" "$ZENOH_CONFIG_OVERRIDE"\n')
    python.chmod(0o755)
    endpoint = 'mode="client";connect/endpoints=["tcp/127.0.0.1:7455"]'
    env = dict(os.environ, PATH=str(binary_dir) + ':' + os.environ['PATH'],
               ROS_DOMAIN_ID='105', ZENOH_CONFIG_OVERRIDE=endpoint)
    result = subprocess.run(['bash', './run' if relative else str(entry)],
                            cwd=service_dir, env=env, capture_output=True,
                            text=True, check=True)
    assert f'-m {module}|105|{endpoint}' in result.stdout
