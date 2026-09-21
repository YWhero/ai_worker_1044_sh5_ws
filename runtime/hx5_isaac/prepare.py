"""Export the official configured SH5 URDF and initialize private Isaac data."""
import argparse
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'simulation/isaac/assets/sh5'
EXPORT = '''
from pathlib import Path
import tempfile, xml.etree.ElementTree as E
from hx5_simulation.model import build_description
with tempfile.TemporaryDirectory() as directory:
    _, root = build_description(Path(directory))
for element in list(root):
    if element.tag in ('gazebo', 'ros2_control'):
        root.remove(element)
print(E.tostring(root, encoding='unicode'))
'''


def prepare_ui():
    """Keep Isaac's built frontend separate from the running Gazebo image."""
    destination = ROOT / 'simulation/isaac/ui'
    if (destination / 'index.html').is_file():
        return
    destination.mkdir(parents=True, exist_ok=True)
    # Seed once from the already prepared image. Subsequent frontend fixes
    # persist in this profile and are not replaced on each simulator start.
    container = subprocess.check_output(['docker', 'create',
        'cyclo-1044-sh5/main:hx5-sim-amd64'], text=True).strip()
    try:
        subprocess.run(['docker', 'cp', container + ':/usr/share/nginx/html/.',
                        str(destination)], check=True)
    finally:
        subprocess.run(['docker', 'rm', container], check=True, stdout=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-only', action='store_true')
    args = parser.parse_args()
    for directory in ('ai_worker', 'ai_worker/maps', 'cyclo', 'agent_sockets/ai_worker', 'agent_sockets/cyclo'):
        (ROOT / 'simulation/isaac' / directory).mkdir(parents=True, exist_ok=True)
    for name in ('bt', 'zenoh_cache', 'ros_transport'):
        source, destination = ROOT / 'simulation/cyclo' / name, ROOT / 'simulation/isaac/cyclo' / name
        if source.exists() and not destination.exists():
            shutil.copytree(source, destination)
    # Different scene geometry requires a new Isaac SLAM map/localization.
    (ROOT / 'simulation/isaac/cyclo/navigation').mkdir(parents=True, exist_ok=True)
    prepare_ui()
    if args.data_only:
        return
    ASSETS.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(['docker', 'exec', '-i', 'ai_worker_1044_hx5_isaac',
        'bash', '--noprofile', '--norc', '-c',
        'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; python3 -'],
        input=EXPORT, text=True, check=True, capture_output=True)
    root = ET.fromstring(result.stdout)
    locations = {'ffw_description': ROOT/'src/ai_worker/ffw_description',
                 'robotis_hand_description': ROOT/'src/robotis_hand/robotis_hand_description'}
    for mesh in root.findall('.//mesh'):
        name = mesh.get('filename', '')
        if name.startswith('package://'):
            package, relative = name[10:].split('/', 1)
            location = locations[package] / relative
        elif name.startswith('file:///root/ros2_ws/install/'):
            package, relative = name.split('/share/', 1)[1].split('/', 1)
            if package in locations:
                location = locations[package] / relative
            else:
                location = ASSETS/'vendor'/package/relative
                location.parent.mkdir(parents=True, exist_ok=True)
                content = subprocess.check_output(['docker', 'exec', 'ai_worker_1044_hx5_isaac', 'cat', name[7:]])
                location.write_bytes(content)
        else:
            location = Path(name.removeprefix('file://'))
        if not location.is_file():
            raise FileNotFoundError(location)
        mesh.set('filename', str(location))
    ET.ElementTree(root).write(ASSETS/'robot.urdf', encoding='unicode', xml_declaration=True)
    print(f'Official SH5/HX5 URDF ready: {ASSETS / "robot.urdf"}')


if __name__ == '__main__':
    main()
