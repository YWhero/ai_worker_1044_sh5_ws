"""Independent checks for odom-frame capture into an Isaac-only world pose."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest
import yaml

MODULE = Path(__file__).with_name('capture_initial_pose.py')
spec = importlib.util.spec_from_file_location('capture_initial_pose_under_test', MODULE)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


@pytest.fixture
def snapshot():
    path = capture.ROOT/'simulation/isaac/ai_worker/diagnostics/requested_initial_pose_20260918.json'
    return json.loads(path.read_text())


@pytest.fixture
def origin():
    # This is the configured bridge origin at the time of the real capture,
    # rather than a later metadata file that the exporter may already update.
    return {'spawn': [.36, -1.25, 0.0, 90.0]}


@pytest.fixture(scope='module')
def description():
    return capture.read_robot_description(capture.ROOT/'simulation/isaac/assets/sh5/robot.urdf')


def test_nonzero_origin_roundtrip_preserves_full_orientation_and_height(snapshot, description):
    metadata = {'spawn': [-.4, .7, .2, 90.0]}
    snapshot.pop('odometry_origin_spawn', None)  # Exercise the legacy metadata path.
    snapshot['base_position'] = [.2, -.6, .1]
    # A non-planar normalized quaternion, rotated by the known +90deg origin.
    snapshot['base_orientation_xyzw'] = [.25, .25, .25, math.sqrt(13)/4]
    pose = capture.snapshot_to_pose(snapshot, metadata, description)
    saved = pose['metadata']
    assert saved['world_position_m'] == pytest.approx([.2, .9, .3])
    assert saved['world_orientation_wxyz'] == pytest.approx([
        (math.sqrt(13)-1)/(4*math.sqrt(2)), 0.0,
        1/(2*math.sqrt(2)), (math.sqrt(13)+1)/(4*math.sqrt(2))])
    # Recovering local coordinates independently gives the original samples.
    x, y, z = saved['world_position_m']
    assert [y-.7, -(x+.4), z-.2] == pytest.approx(snapshot['base_position'])


def test_real_snapshot_saves_all57_measured_axes_and_normalizes_float_quaternion(snapshot, origin, description):
    pose = capture.snapshot_to_pose(snapshot, origin, description)
    saved = pose['metadata']
    assert saved['world_position_m'] == pytest.approx([
        .36021131277084345, -.7280516624450684, 1.4901161193847656e-7])
    assert saved['world_orientation_wxyz'] == pytest.approx([
        .7067136986203583, 7.26431508450977e-8,
        4.65661223366011e-8, .7074996453584453], abs=1e-13)
    assert math.sqrt(sum(q*q for q in saved['world_orientation_wxyz'])) == pytest.approx(1, abs=1e-14)
    assert saved['world_yaw_degrees'] == pytest.approx(90.06368406375177)
    assert set(pose['initial_positions']) == set(capture.POSITION_JOINT_NAMES)
    assert len(pose['initial_positions']) == 57
    assert pose['initial_positions'] == {n: snapshot['joints'][n] for n in capture.POSITION_JOINT_NAMES}
    assert pose['initial_positions']['head_joint1'] == .6863727569580078
    assert pose['initial_positions']['lift_joint'] == -.005598837044090033
    assert 'left_wheel_drive' not in pose['initial_positions']


@pytest.mark.parametrize('key,value', [('odom_frame', 'map'), ('base_frame', 'base_footprint')])
def test_wrong_frame_is_rejected(snapshot, origin, description, key, value):
    snapshot[key] = value
    with pytest.raises(ValueError, match='odom -> base_link'):
        capture.snapshot_to_pose(snapshot, origin, description)


@pytest.mark.parametrize('name', ['finger_r_joint20', 'lift_joint'])
def test_missing_hand_or_lift_axis_is_rejected(snapshot, origin, description, name):
    del snapshot['joints'][name]
    with pytest.raises(ValueError, match='all 57'):
        capture.snapshot_to_pose(snapshot, origin, description)


@pytest.mark.parametrize('joint_stamp', [718.7, float('nan'), -1.0])
def test_unsynchronized_or_invalid_stamp_is_rejected(snapshot, origin, description, joint_stamp):
    snapshot['joint_stamp'] = joint_stamp
    with pytest.raises(ValueError, match='within 0.1'):
        capture.snapshot_to_pose(snapshot, origin, description)


@pytest.mark.parametrize('key,value', [
    ('base_position', [float('nan'), 0.0, 0.0]),
    ('base_position', [0.0, 0.0, float('inf')]),
    ('base_orientation_xyzw', [0.0, float('inf'), 0.0, 1.0]),
    ('base_position', [0.0, 0.0]),
])
def test_nonfinite_or_malformed_base_pose_is_rejected(snapshot, origin, description, key, value):
    snapshot[key] = value
    with pytest.raises(ValueError, match='finite measured'):
        capture.snapshot_to_pose(snapshot, origin, description)


@pytest.mark.parametrize('quaternion', [[0.0]*4, [0.0, 0.0, 0.0, 2.0]])
def test_zero_or_nonunit_quaternion_is_rejected(snapshot, origin, description, quaternion):
    snapshot['base_orientation_xyzw'] = quaternion
    with pytest.raises(ValueError, match='normalized'):
        capture.snapshot_to_pose(snapshot, origin, description)


@pytest.mark.parametrize('name', ['finger_l_joint2', 'head_joint1'])
def test_official_hand_and_head_joint_limits_are_enforced(snapshot, origin, description, name):
    snapshot['joints'][name] = description['joints'][name]['limit']['upper']+.01
    with pytest.raises(ValueError, match='official URDF limit'):
        capture.snapshot_to_pose(snapshot, origin, description)


def test_conversion_does_not_mutate_input_or_write_any_file(snapshot, origin, description, monkeypatch):
    originals = copy.deepcopy((snapshot, origin))
    def reject_write(*args, **kwargs):
        raise AssertionError('Snapshot conversion must not write files')
    with monkeypatch.context() as context:
        context.setattr(Path, 'write_text', reject_write)
        capture.snapshot_to_pose(snapshot, origin, description)
    assert (snapshot, origin) == originals


def test_private_export_updates_only_requested_files_and_preserves_frozen_gazebo_pose(
        tmp_path, snapshot, origin, description, monkeypatch, capsys):
    snapshot.pop('odometry_origin_spawn', None)  # Explicit metadata keeps legacy CLI compatible.
    frozen = (capture.ROOT/'src/hx5_simulation/hx5_simulation/initial_poses/'
              'vitacformer_task519_sync.yaml')
    frozen_before = hashlib.sha256(frozen.read_bytes()).hexdigest()
    snapshot_path = tmp_path/'snapshot.json'
    metadata_path = tmp_path/'metadata.json'
    layout_path = tmp_path/'layout.json'
    output = tmp_path/'isaac_only_initial_pose.yaml'
    snapshot_path.write_text(json.dumps(snapshot))
    metadata_path.write_text(json.dumps(origin))
    layout = {'robot': {'look_at_xy': [.36, -.095]}, 'conveyor': {'top_height': .75}}
    layout_path.write_text(json.dumps(layout))
    monkeypatch.setattr(sys, 'argv', [str(MODULE), '--snapshot', str(snapshot_path),
        '--metadata', str(metadata_path), '--layout', str(layout_path),
        '--urdf', str(capture.ROOT/'simulation/isaac/assets/sh5/robot.urdf'),
        '--output', str(output)])
    capture.main()
    exported = yaml.safe_load(output.read_text())
    revised = json.loads(layout_path.read_text())
    assert exported == capture.snapshot_to_pose(snapshot, origin, description)
    assert revised['robot']['initial_pose_source'] == str(output.resolve())
    assert revised['robot']['position_m'] == exported['metadata']['world_position_m']
    assert revised['robot']['orientation_wxyz'] == exported['metadata']['world_orientation_wxyz']
    assert revised['robot']['look_at_xy'] == layout['robot']['look_at_xy']
    assert revised['conveyor'] == layout['conveyor']
    assert json.loads(metadata_path.read_text()) == origin
    assert json.loads(snapshot_path.read_text()) == snapshot
    assert hashlib.sha256(frozen.read_bytes()).hexdigest() == frozen_before
    assert not list(tmp_path.glob('*.new'))
    assert json.loads(capsys.readouterr().out)['joints'] == 57


def test_embedded_capture_origin_wins_over_changed_scene_metadata(snapshot, origin, description):
    snapshot['odometry_origin_spawn'] = origin['spawn']
    # The scene has since restarted at the pose saved by this very snapshot.
    newer_metadata = {'spawn': [.3602113127708435, -.7280516624450684,
                                1.4901161193847656e-7, 90.06368406375174]}
    pose = capture.snapshot_to_pose(snapshot, newer_metadata, description)
    assert pose == capture.snapshot_to_pose(snapshot, None, description)
    assert pose['metadata']['odometry_origin_spawn'] == origin['spawn']
    assert pose['metadata']['world_position_m'] == pytest.approx([
        .36021131277084345, -.7280516624450684, 1.4901161193847656e-7])


def test_legacy_snapshot_without_original_metadata_is_rejected(snapshot, description):
    snapshot.pop('odometry_origin_spawn', None)
    with pytest.raises(ValueError, match='explicit original --metadata'):
        capture.snapshot_to_pose(snapshot, None, description)


def test_legacy_cli_does_not_guess_current_metadata_or_change_outputs(tmp_path, snapshot, monkeypatch):
    snapshot.pop('odometry_origin_spawn', None)
    source = tmp_path/'snapshot.json'
    source.write_text(json.dumps(snapshot))
    output, layout = tmp_path/'pose.yaml', tmp_path/'layout.json'
    output.write_text('original pose\n'); layout.write_text('original layout\n')
    monkeypatch.setattr(sys, 'argv', [str(MODULE), '--snapshot', str(source),
        '--output', str(output), '--layout', str(layout)])
    with pytest.raises(SystemExit) as error:
        capture.main()
    assert error.value.code == 2
    assert output.read_text() == 'original pose\n'
    assert layout.read_text() == 'original layout\n'
    assert not list(tmp_path.glob('*.new'))


def test_embedded_origin_cli_exports_without_current_metadata(tmp_path, snapshot, origin, monkeypatch):
    snapshot['odometry_origin_spawn'] = origin['spawn']
    source, output, layout = tmp_path/'snapshot.json', tmp_path/'pose.yaml', tmp_path/'layout.json'
    source.write_text(json.dumps(snapshot)); layout.write_text(json.dumps({'robot': {}}))
    monkeypatch.setattr(sys, 'argv', [str(MODULE), '--snapshot', str(source),
        '--metadata', str(tmp_path/'nonexistent_current_metadata.json'),
        '--output', str(output), '--layout', str(layout)])
    capture.main()
    saved = yaml.safe_load(output.read_text())
    assert saved['metadata']['odometry_origin_spawn'] == origin['spawn']
    assert saved['metadata']['world_position_m'][1] == pytest.approx(-.7280516624450684)
    assert not list(tmp_path.glob('*.new'))


def test_layout_staging_failure_preserves_both_originals_and_cleans_temps(tmp_path, monkeypatch):
    output, layout = tmp_path/'pose.yaml', tmp_path/'layout.json'
    output.write_bytes(b'original pose\n'); layout.write_bytes(b'original layout\n')
    write_text = Path.write_text
    def fail_layout_stage(path, *args, **kwargs):
        if path.name.startswith(layout.name+'.') and path.name.endswith('.new'):
            raise OSError('layout staging failed')
        return write_text(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'write_text', fail_layout_stage)
    with pytest.raises(OSError, match='layout staging failed'):
        capture.write_pose_and_layout(output, 'new pose\n', layout, 'new layout\n')
    assert output.read_bytes() == b'original pose\n'
    assert layout.read_bytes() == b'original layout\n'
    assert not list(tmp_path.glob('*.new'))


@pytest.mark.parametrize('existing_pose', [True, False])
def test_failed_second_replace_restores_pose_and_layout_and_cleans_temps(
        tmp_path, monkeypatch, existing_pose):
    output, layout = tmp_path/'pose.yaml', tmp_path/'layout.json'
    if existing_pose:
        output.write_bytes(b'original pose\n')
        output.chmod(0o640)
    layout.write_bytes(b'original layout\n')
    replace = Path.replace
    def fail_layout_replace(path, target):
        if target == layout:
            # Prove this tests rollback after the pose commit, not staging.
            assert output.read_text() == 'new pose\n'
            raise OSError('layout replace failed')
        return replace(path, target)
    monkeypatch.setattr(Path, 'replace', fail_layout_replace)
    with pytest.raises(OSError, match='layout replace failed'):
        capture.write_pose_and_layout(output, 'new pose\n', layout, 'new layout\n')
    if existing_pose:
        assert output.read_bytes() == b'original pose\n'
        assert output.stat().st_mode & 0o777 == 0o640
    else:
        assert not output.exists()
    assert layout.read_bytes() == b'original layout\n'
    assert not list(tmp_path.glob('*.new'))
