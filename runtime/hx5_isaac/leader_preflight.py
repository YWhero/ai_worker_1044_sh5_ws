"""Read-only checks before opening the two physical Skeleton LG2 USB ports."""
import json
import os
from pathlib import Path
import stat
import subprocess
import sys

LEADER = 'lg2_leader_1044_hx5_isaac'


def device_identity(path):
    info = Path(path).stat()
    if not stat.S_ISCHR(info.st_mode):
        raise RuntimeError(f'Leader USB device is not a character device: {path}')
    return info.st_rdev


def check_devices(left, right, containers, inspect_device=device_identity, run=subprocess.run):
    identities = []
    for side, path in (('left', left), ('right', right)):
        try:
            identities.append(inspect_device(path))
        except FileNotFoundError as error:
            raise RuntimeError(f'Missing {side} Skeleton Leader USB device: {path}. '
                               'Connect both leaders or set HX5_ISAAC_LEFT_LEADER_DEVICE/HX5_ISAAC_RIGHT_LEADER_DEVICE.') from error
    if identities[0] == identities[1]:
        raise RuntimeError('Left and right Skeleton Leaders must use distinct USB devices')
    for container in containers:
        if not container.get('State', {}).get('Running'):
            continue
        if container.get('Name', '').lstrip('/') == LEADER:
            raise RuntimeError('Isaac Skeleton Leader is already running; use leader-stop before changing modes')
        host = container.get('HostConfig', {})
        devices = host.get('Devices') or []
        for device in devices:
            path = device['PathOnHost']
            try:
                conflicting = inspect_device(path) in identities
            except (FileNotFoundError, RuntimeError):
                conflicting = path in (left, right)
            if conflicting:
                raise RuntimeError(f'Leader USB device is exposed to running container {container["Name"]}: {path}')
        for mount in container.get('Mounts', []):
            source = mount.get('Source', '')
            if source in (left, right):
                raise RuntimeError(f'Leader USB device is mounted in running container {container["Name"]}: {source}')
    for path in (left, right):
        result = run(['fuser', str(path)], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            raise RuntimeError(f'Leader USB device is already in use: {path}; processes: {result.stdout.strip()}')
        if result.returncode != 1 or result.stderr.strip():
            raise RuntimeError(f'Cannot verify leader USB ownership: {path}: {result.stderr.strip()}')
    return {'left': str(left), 'right': str(right), 'domain': 115, 'router': 'tcp/127.0.0.1:7855'}


def main():
    left = os.environ.get('HX5_ISAAC_LEFT_LEADER_DEVICE', '/dev/left_leader')
    right = os.environ.get('HX5_ISAAC_RIGHT_LEADER_DEVICE', '/dev/right_leader')
    # Validate absence and aliasing before even querying Docker.
    for side, path in (('left', left), ('right', right)):
        try:
            device_identity(path)
        except FileNotFoundError as error:
            raise RuntimeError(f'Missing {side} Skeleton Leader USB device: {path}. '
                               'Connect both leaders or set HX5_ISAAC_LEFT_LEADER_DEVICE/HX5_ISAAC_RIGHT_LEADER_DEVICE.') from error
    identifiers = subprocess.check_output(['docker', 'ps', '-q'], text=True, timeout=10).split()
    containers = json.loads(subprocess.check_output(['docker', 'inspect', *identifiers], text=True, timeout=10)) if identifiers else []
    print(json.dumps(check_devices(left, right, containers), indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'Skeleton Leader preflight failed: {error}', file=sys.stderr)
        raise SystemExit(1)
