"""Install a built frontend only into the Isaac profile's persistent UI volume."""
import argparse
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('build', type=Path, help='Frontend build directory containing index.html')
    source = parser.parse_args().build.resolve()
    destination = ROOT / 'simulation/isaac/ui'
    if not (source / 'index.html').is_file() or not (source / 'static').is_dir():
        parser.error('Build directory must contain index.html and static/')
    if source == destination.resolve():
        parser.error('Use a separate frontend build directory')
    destination.mkdir(parents=True, exist_ok=True)
    # This file belongs to the running profile, not the frontend build.
    # Copying the build's default ports would disconnect the Isaac UI.
    shutil.copytree(source, destination, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('cyclo-config.js'))
    container = 'cyclo_intelligence_1044_hx5_isaac'
    for direction in ('-d', '-u'):
        subprocess.run(['docker', 'exec', container, '/command/s6-rc',
                        direction, 'change', 'nginx'], check=True)
    print('Isaac UI installed at http://localhost:7880/; reload the browser.')


if __name__ == '__main__':
    main()
