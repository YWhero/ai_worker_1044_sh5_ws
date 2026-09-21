import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

SCRIPT = Path(__file__).with_name('ros_service_finish.sh')


@pytest.mark.parametrize('ignore_term', [False, True])
def test_cleanup_finishes_before_s6_default_timeout(tmp_path, ignore_term):
    ready = tmp_path / 'ready'
    child = subprocess.Popen([
        sys.executable, '-c',
        'import signal, time; from pathlib import Path; '
        + ('signal.signal(signal.SIGTERM, signal.SIG_IGN); ' if ignore_term else '')
        + f'Path({str(ready)!r}).touch(); time.sleep(60)',
    ], start_new_session=True)
    pgid_file = tmp_path / 'bt_node.pgid'
    pgid_file.write_text(str(child.pid))
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready.exists()
        result = subprocess.run(
            ['sh', str(SCRIPT)], capture_output=True, text=True, timeout=4,
            env={**os.environ, 'SERVICE_NAME': 'bt_node', 'CYCLO_SERVICE_RUN_DIR': str(tmp_path)},
        )
        assert result.returncode == 0, result.stderr
        assert child.wait(timeout=1) == -(signal.SIGKILL if ignore_term else signal.SIGTERM)
        assert not pgid_file.exists()
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGKILL)
        child.wait()


@pytest.mark.parametrize('pgid', ['1', 'bad', str(os.getpgrp())])
def test_cleanup_rejects_unsafe_process_group(tmp_path, pgid):
    (tmp_path / 'bt_node.pgid').write_text(pgid)
    result = subprocess.run(
        ['sh', str(SCRIPT)], capture_output=True, text=True, timeout=1,
        env={**os.environ, 'SERVICE_NAME': 'bt_node', 'CYCLO_SERVICE_RUN_DIR': str(tmp_path)},
    )
    assert result.returncode != 0
