import json

from test_session_archive import _make_manager, _write_segment
from cyclo_data.recorder.session_manager import _recording_source_metadata


def test_real_recordings_do_not_gain_simulation_labels(monkeypatch):
    monkeypatch.delenv('CYCLO_EPISODE_SCHEMA_VERSION', raising=False)
    monkeypatch.setenv('CYCLO_SIM_TACTILE_SOURCE', 'gazebo_contact_force_projected_grid_v1')
    assert _recording_source_metadata() == {}


def test_simulation_labels_are_explicitly_opt_in(monkeypatch):
    monkeypatch.setenv('CYCLO_EPISODE_SCHEMA_VERSION', 'cyclo_gazebo_mcap')
    monkeypatch.setenv('CYCLO_SIM_TACTILE_SOURCE', 'gazebo_contact_force_projected_grid_v1')
    assert _recording_source_metadata() == {
        'schema_version': 'cyclo_gazebo_mcap',
        'simulation': {
            'tactile_source': 'gazebo_contact_force_projected_grid_v1',
            'hardware_calibrated': False,
        },
    }


def test_full_episode_carries_simulation_labels(monkeypatch, tmp_path):
    monkeypatch.setenv('CYCLO_EPISODE_SCHEMA_VERSION', 'cyclo_gazebo_mcap')
    monkeypatch.setenv('CYCLO_SIM_TACTILE_SOURCE', 'gazebo_contact_force_projected_grid_v1')
    root = tmp_path / 'Task_simulation_MCAP'
    manager = _make_manager(root, subtask_total=1)
    _write_segment(root, full_idx=0, subtask_idx=0, subtask_total=1, with_video=False)
    saved = manager._archive_full_episode(0)
    metadata = json.loads((saved / 'episode_info.json').read_text())
    assert metadata['schema_version'] == 'cyclo_gazebo_mcap'
    assert metadata['simulation']['hardware_calibrated'] is False
