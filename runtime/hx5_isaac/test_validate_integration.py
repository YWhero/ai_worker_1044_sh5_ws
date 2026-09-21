"""Meaningful malformed/freshness/frame regressions, without requiring ROS."""

import math
from io import BytesIO
import json
from types import SimpleNamespace as N

import pytest

from validate_integration import (
    AUX, ARMS, HANDS, Samples, has_tf_path, joint_values,
    pose_error, pressure_values, scan_summary, xyzw,
    CAMERA_FRAMES, camera_summary, camera_info_summary, json_scalar, main,
)


def test_fifty_four_policy_joints_and_aux_feedback_are_distinct():
    assert len(ARMS + HANDS) == 54
    assert len(ARMS + HANDS + AUX) == 57
    message = N(name=ARMS + HANDS, position=[0.0] * 54)
    assert len(joint_values(message, ARMS + HANDS)) == 54
    with pytest.raises(ValueError, match='missing joints'):
        joint_values(message, AUX)
    message.position[10] = math.nan
    with pytest.raises(ValueError, match='non-finite'):
        joint_values(message, ARMS + HANDS)


def test_tactile_requires_all_five_fingertips_and_nine_uint8_taxels():
    sensors = [N(sensor_name=f'finger_l_sensor{i}', pressure_values=[i] * 9,
                 pressure_names=[f'Present Pressure {j}' for j in range(1, 10)]) for i in range(1, 6)]
    message = N(sensors=sensors, hand_name='left')
    assert pressure_values(message, 'l')['finger_l_sensor5'] == 45
    with pytest.raises(ValueError, match='missing fingertip'):
        pressure_values(N(sensors=sensors[:-1], hand_name='left'), 'l')
    with pytest.raises(ValueError, match='sensor order'):
        pressure_values(N(sensors=list(reversed(sensors)), hand_name='left'), 'l')
    with pytest.raises(ValueError, match='hand_name'):
        pressure_values(N(sensors=sensors, hand_name='right'), 'l')
    sensors[0].pressure_names.reverse()
    with pytest.raises(ValueError, match='pressure_names'):
        pressure_values(message, 'l')
    sensors[0].pressure_names.reverse()
    sensors[0].pressure_values = [0] * 8
    with pytest.raises(ValueError, match='nine uint8'):
        pressure_values(message, 'l')
    sensors[0].pressure_values = [256] * 9
    with pytest.raises(ValueError, match='nine uint8'):
        pressure_values(message, 'l')


def test_scan_accepts_no_return_infinity_but_rejects_nan_and_out_of_range():
    scan = N(range_min=0.1, range_max=10.0, angle_min=0.0, angle_max=0.2,
             angle_increment=0.1, ranges=[1.0, math.inf, 2.0], header=N(frame_id='isaac_scan'))
    assert scan_summary(scan)['minimum_m'] == 1.0
    scan.header.frame_id = 'base_link'
    with pytest.raises(ValueError, match='isaac_scan'):
        scan_summary(scan)
    scan.header.frame_id = 'isaac_scan'
    scan.ranges[0] = math.nan
    with pytest.raises(ValueError, match='NaN'):
        scan_summary(scan)
    scan.ranges[0] = 11.0
    with pytest.raises(ValueError, match='out-of-range'):
        scan_summary(scan)


def test_stamp_probe_requires_advance_and_rejects_regression_stale_or_future():
    samples = Samples()
    samples.update(1.0)
    assert not samples.report(samples.last_wall, 1.0, 1.0)['passed']
    samples.update(1.1)
    assert samples.report(samples.last_wall, 1.1, 1.0)['passed']
    assert not samples.report(samples.last_wall + 2.0, 1.1, 1.0)['passed']
    assert not samples.report(samples.last_wall, 0.0, 1.0)['passed']
    samples.update(1.05)
    assert 'timestamp regressions' in ' '.join(samples.report(samples.last_wall, 1.1, 1.0)['failures'])


def test_tf_path_rejects_multiple_parents_and_cycles():
    graph = {'laser': {'laser_mount'}, 'laser_mount': {'base_link'}}
    assert has_tf_path(graph, 'laser', 'base_link')
    graph['laser_mount'].add('other_robot')
    assert not has_tf_path(graph, 'laser', 'base_link')
    assert not has_tf_path({'laser': {'loop'}, 'loop': {'laser'}}, 'laser', 'base_link')


def test_later_good_sample_does_not_hide_malformed_messages():
    samples = Samples()
    samples.update(1.0, error='malformed tactile')
    samples.update(1.1, {'valid': True})
    report = samples.report(samples.last_wall, 1.1, 1.0)
    assert report['malformed_count'] == 1
    assert not report['passed']
    assert 'malformed tactile' in report['failures'][0]


def jpeg_message(size=(672, 376)):
    from PIL import Image

    buffer = BytesIO()
    Image.new('RGB', size, (4, 12, 31)).save(buffer, format='JPEG')
    return N(data=buffer.getvalue(), format='jpeg', header=N(frame_id=CAMERA_FRAMES[0]))


def test_jpeg_decodes_expected_rgb_dimensions_and_exact_optical_frame():
    message = jpeg_message()
    assert camera_summary(message, CAMERA_FRAMES[0])['width'] == 672
    message.header.frame_id = 'base_link'
    with pytest.raises(ValueError, match='frame_id'):
        camera_summary(message, CAMERA_FRAMES[0])
    with pytest.raises(ValueError, match='672x376'):
        camera_summary(jpeg_message((188, 336)), CAMERA_FRAMES[0])


def test_jpeg_markers_without_a_decodable_image_are_rejected():
    message = jpeg_message()
    message.data = b'\xff\xd8\xff\xd9'
    with pytest.raises(ValueError, match='cannot be decoded'):
        camera_summary(message, CAMERA_FRAMES[0])


def test_camera_info_matches_fixed_sensor_resolution_and_frame():
    message = N(width=672, height=376, k=[367., 0., 336., 0., 367., 188., 0., 0., 1.],
                header=N(frame_id=CAMERA_FRAMES[0]))
    assert camera_info_summary(message, CAMERA_FRAMES[0])['height'] == 376
    message.header.frame_id = CAMERA_FRAMES[1]
    with pytest.raises(ValueError, match='frame_id'):
        camera_info_summary(message, CAMERA_FRAMES[0])
    message.header.frame_id = CAMERA_FRAMES[0]
    message.width = 0
    with pytest.raises(ValueError, match='672x376'):
        camera_info_summary(message, CAMERA_FRAMES[0])


def test_each_sensor_has_its_own_dimensions_and_hd720_profile_is_supported():
    message = jpeg_message((424, 240))
    message.header.frame_id = CAMERA_FRAMES[2]
    assert camera_summary(message, CAMERA_FRAMES[2])['height'] == 240
    with pytest.raises(ValueError, match='672x376'):
        camera_summary(message, CAMERA_FRAMES[2], (672, 376))
    message = jpeg_message((1280, 720))
    assert camera_summary(message, CAMERA_FRAMES[0], (1280, 720))['width'] == 1280


def test_ros_numpy_scalar_summaries_serialize_as_numeric_json():
    np = pytest.importorskip('numpy')
    content = json.dumps({'sum': np.int64(22), 'value': np.float32(.5)},
                         default=json_scalar, allow_nan=False)
    assert json.loads(content) == {'sum': 22, 'value': .5}


def test_tf_quaternion_sign_is_equivalent_but_non_normalized_is_invalid():
    quaternion = xyzw(N(x=0.0, y=0.0, z=0.0, w=1.0))
    assert pose_error([0, 0, 0], quaternion, [0, 0, 0], [0, 0, 0, -1]) == (0.0, 0.0)
    with pytest.raises(ValueError, match='not normalized'):
        xyzw(N(x=0.0, y=0.0, z=0.0, w=0.5))
    with pytest.raises(ValueError, match='non-finite TF'):
        pose_error([math.nan, 0, 0], quaternion, [0, 0, 0], quaternion)


def test_wrong_existing_domain_is_rejected_before_ros_imports(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '106')
    with pytest.raises(SystemExit) as error:
        main(['--duration', '1'])
    assert error.value.code == 2
    assert 'ROS_DOMAIN_ID=115' in capsys.readouterr().err
