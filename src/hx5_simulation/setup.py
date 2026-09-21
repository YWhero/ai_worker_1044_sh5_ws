from glob import glob
from setuptools import setup

setup(
    name='hx5_simulation', version='0.1.0', packages=['hx5_simulation'],
    package_data={'hx5_simulation': ['initial_poses/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/hx5_simulation']),
        ('share/hx5_simulation', ['package.xml']),
        ('share/hx5_simulation/launch', glob('launch/*.launch.py')),
    ],
    entry_points={'console_scripts': [
        'sim_io = hx5_simulation.sim_io:main',
        'hand_presets = hx5_simulation.hand_presets:main',
    ]},
)
