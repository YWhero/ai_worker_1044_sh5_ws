import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from cyclo_data.recorder.camera_selection import CameraSelection


def test_default_all_and_explicit_none_survive_restart(tmp_path):
    path = tmp_path / 'config' / 'cameras.json'
    store = CameraSelection(path)
    assert store.enabled('sh5', ['left', 'right']) == ['left', 'right']
    store.robots['sh5'] = []
    store.save()
    restored = CameraSelection(path)
    assert restored.enabled('sh5', ['left', 'right']) == []
    assert restored.enabled('other', ['left', 'right']) == ['left', 'right']


def test_removed_cameras_are_ignored_and_config_order_is_preserved(tmp_path):
    store = CameraSelection(tmp_path / 'cameras.json')
    store.robots['sh5'] = ['right', 'removed', 'left']
    assert store.enabled('sh5', ['left', 'right', 'wrist']) == ['left', 'right']


def test_failed_atomic_write_preserves_previous_file(tmp_path, monkeypatch):
    path = tmp_path / 'cameras.json'
    store = CameraSelection(path)
    store.robots['sh5'] = ['left']
    store.save()
    store.robots['sh5'] = []
    def fail(*args):
        raise OSError('disk unavailable')
    monkeypatch.setattr('cyclo_data.recorder.camera_selection.os.replace', fail)
    with pytest.raises(OSError):
        store.save()
    assert json.loads(path.read_text()) == {'sh5': ['left']}
    assert list(tmp_path.glob('*.tmp')) == []
