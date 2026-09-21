"""Persist recording camera choices per robot; an absent choice means all."""

import json
import os
from pathlib import Path
import tempfile


class CameraSelection:
    def __init__(self, path=None):
        self.path = Path(path or os.environ.get(
            'CYCLO_RECORDING_CAMERAS_PATH',
            '/workspace/config/recording_cameras.json',
        ))
        self.robots = {}
        if self.path.exists():
            data = json.loads(self.path.read_text())
            if not isinstance(data, dict) or any(
                not isinstance(names, list)
                or any(not isinstance(name, str) for name in names)
                for names in data.values()
            ):
                raise ValueError(f'Invalid recording camera settings: {self.path}')
            self.robots = data

    def enabled(self, robot_type, available):
        selected = self.robots.get(robot_type, available)
        return [name for name in available if name in selected]

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(dir=self.path.parent, suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(self.robots, stream, indent=2)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
