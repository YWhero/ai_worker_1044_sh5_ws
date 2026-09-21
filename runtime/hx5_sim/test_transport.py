import os
from pathlib import Path
import subprocess


PROFILE = Path(__file__).resolve().parent


def run_prepare(tmp_path, cached=False, build_status=0):
    docker = tmp_path / 'docker'
    docker.write_text(
        '#!/bin/sh\n'
        'printf "%s\\n" "$*" >> "$DOCKER_LOG"\n'
        'if [ "$1" = inspect ]; then printf "sha256:test-image\\n"; exit 0; fi\n'
        'case "$*" in\n'
        '  *"test -f /workspace/ros_transport/install"*) exit "$CACHE_STATUS" ;;\n'
        '  *"cat > /workspace/ros_transport/readiness.patch"*) cat > "$PATCH_COPY" ;;\n'
        '  *"bash --noprofile --norc -s"*) cat > "$BUILD_BODY"; exit "$BUILD_STATUS" ;;\n'
        'esac\n'
    )
    docker.chmod(0o755)
    environment = {
        **os.environ,
        'PATH': f'{tmp_path}:{os.environ["PATH"]}',
        'DOCKER_LOG': str(tmp_path / 'docker.log'),
        'CACHE_STATUS': '0' if cached else '1',
        'BUILD_STATUS': str(build_status),
        'PATCH_COPY': str(tmp_path / 'patch.copy'),
        'BUILD_BODY': str(tmp_path / 'build.sh'),
    }
    return subprocess.run(
        ['bash', str(PROFILE / 'prepare_transport.sh')],
        env=environment, capture_output=True, text=True, check=False,
    )


def test_cached_transport_does_not_download_or_rebuild(tmp_path):
    result = run_prepare(tmp_path, cached=True)
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / 'build.sh').exists()
    assert not (tmp_path / 'patch.copy').exists()


def test_build_is_pinned_and_only_targets_hx5_cyclo(tmp_path):
    result = run_prepare(tmp_path)
    assert result.returncode == 0, result.stderr
    assert (tmp_path / 'patch.copy').read_text() == (PROFILE / 'rmw_zenoh_readiness.patch').read_text()
    commands = (tmp_path / 'docker.log').read_text().splitlines()
    assert all('cyclo_intelligence_1044_hx5_sim' in command for command in commands)
    build = (tmp_path / 'build.sh').read_text()
    assert '41f316772ba1210a79e715d61951ddf50857b640' in build
    assert 'sha256sum -c' in build
    assert '"${VERSION%%-*}" != 0.2.10' in build
    assert build.index('colcon build') < build.index('> build-key')
    assert '--allow-overriding rmw_zenoh_cpp' in build
    assert subprocess.run(['bash', '-n', str(tmp_path / 'build.sh')]).returncode == 0


def test_failed_transport_build_is_not_silently_ignored(tmp_path):
    assert run_prepare(tmp_path, build_status=7).returncode == 7


def test_transport_is_only_sourced_for_the_simulation_cyclo():
    launcher = (PROFILE.parent / 'hx5_sim.sh').read_text()
    assert 'if [ "$container" = "$CYCLO_CONTAINER" ]; then' in launcher
    assert 'transport_setup=/workspace/ros_transport/install/local_setup.bash' in launcher
    assert 'source /workspace/ros_transport/install/local_setup.bash' in (PROFILE / 'ros_service.sh').read_text()
