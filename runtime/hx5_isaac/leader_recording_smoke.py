"""Bounded synthetic LG2 → Isaac → Cyclo MCAP/video check; never opens USB devices.

Run inside the private Cyclo container with its ROS overlays sourced. This moves
simulated arms/hands, creates one new scratch episode, then requests Isaac reset.
It does not validate a physical Skeleton Leader, human demonstration, or grasp.
"""
import collections
import copy
import json
import math
import os
from pathlib import Path
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET


def positions(message, names):
    """Extract the configured policy slice from an eight-joint LG2 command."""
    assert len(message.joint_names) == len(set(message.joint_names)), 'duplicate action joints'
    assert message.points, 'empty action trajectory'
    point = message.points[-1]
    assert len(point.positions) == len(message.joint_names), 'action names/positions mismatch'
    values = dict(zip(message.joint_names, map(float, point.positions)))
    assert all(math.isfinite(v) for v in values.values()), 'nonfinite action'
    assert set(names).issubset(values), 'configured action joints missing'
    return [values[name] for name in names]


def excursion(samples):
    return max((max(column) - min(column) for column in zip(*samples)), default=0.)


def main():
    import cv2
    import pandas as pd
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from interfaces.msg import RecordingStatus, InferenceStatus, TaskInfo
    from interfaces.srv import RecordingCommand
    from shared.robot_configs import schema
    import rosbag2_py
    import yaml
    sys.path.insert(0, '/hx5_isaac_runtime')
    from validate_integration import ARMS, HANDS, AUX, joint_values, pressure_values

    assert os.environ.get('ROS_DOMAIN_ID') == '115', 'private Isaac domain115 required'
    root = Path('/workspace/rosbag2/Task_20260918_IsaacSkeletonContract20260918_MCAP')
    output = Path('/workspace/diagnostics/leader-recording-smoke.json')
    assert not root.exists() and not output.exists(), 'scratch task/report already exists; refusing overwrite'
    section = schema.load_robot_section('ffw_sh5_rev1')
    groups = schema.get_recorded_action_groups(section)
    assert sorted(len(g['joint_names']) for g in groups.values()) == [7, 7, 20, 20]
    assert sum((g['joint_names'] for g in groups.values()), []) == ARMS + HANDS
    action_topics = {g['topic']: g['joint_names'] for g in groups.values()}
    topics = list(dict.fromkeys(schema.get_mcap_record_topics(section) + ['/joint_states', '/odom', '/scan', '/tf_static']))
    assert set(action_topics).issubset(topics), 'schema omitted action topics'
    urdf = schema.get_urdf_path(section)
    limits = {j.attrib['name']: (float(j.find('limit').attrib['lower']), float(j.find('limit').attrib['upper']))
              for j in ET.parse(urdf).getroot().findall('joint') if j.find('limit') is not None and 'lower' in j.find('limit').attrib}
    info = TaskInfo(task_num='20260918', task_name='IsaacSkeletonContract20260918', task_type='integration',
                    task_instruction=['Synthetic LG2 arm/gripper transport and Isaac MCAP/video contract check.'],
                    tags=['isaac_sim', 'sh5', 'hx5', 'synthetic_lg2_smoke'], control_hz=30, inference_hz=30,
                    include_robotis_license=False)
    report = {'result': 'failed', 'ros_domain_id': 115, 'task_path': str(root), 'responses': [], 'failures': [],
              'usb_devices_observed_only': [str(p) for p in Path('/dev/serial/by-id').glob('*')],
              'scope': 'Synthetic official LG2 command transport, simulated measured motion, and stored MCAP/video; no physical leader or grasp validation.'}
    rclpy.init()
    node = rclpy.create_node('isaac_synthetic_leader_recording_smoke')
    states, inference, feedback, phases = [], [], [], []
    actions = {topic: [] for topic in action_topics}
    latest, subscriptions, started, original = {}, [], False, None

    def recording(message):
        states.append(message)
        if not phases or phases[-1] != int(message.record_phase): phases.append(int(message.record_phase))

    def observed(message):
        value = joint_values(message, ARMS + HANDS)
        assert set(value) == set(ARMS + HANDS), 'policy feedback must contain exactly54 joints'
        latest.update(message=message, received=time.monotonic())
        feedback.append(value)

    def action(message, topic, names):
        values = positions(message, names)
        assert all(limits[n][0] <= v <= limits[n][1] for n, v in zip(names, values)), 'translated action exceeds official URDF limits'
        actions[topic].append((list(message.joint_names), values))

    subscriptions.append(node.create_subscription(RecordingStatus, '/data/recording/status', recording, 10))
    subscriptions.append(node.create_subscription(InferenceStatus, '/task/inference_status', lambda m: inference.append(m), 10))
    subscriptions.append(node.create_subscription(JointState, '/arm_hand/joint_states', observed, 10))
    for topic, names in action_topics.items():
        subscriptions.append(node.create_subscription(JointTrajectory, topic,
            lambda m, t=topic, n=names: action(m, t, n), 10))
    publishers = {side: node.create_publisher(JointTrajectory, groups[f'arm_{label}']['topic'], 10)
                  for side, label in [('l', 'left'), ('r', 'right')]}
    client = node.create_client(RecordingCommand, '/data/recording')

    def spin(duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=min(.05, max(0., deadline-time.monotonic())))

    def result(client_, request, timeout=35.):
        future = client_.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.05)
        assert future.done(), 'service response timeout'
        response = future.result()
        assert response.success, response.message
        return response

    def command(value, task=info, robot='ffw_sh5_rev1'):
        response = result(client, RecordingCommand.Request(command=value, task_info=task, robot_type=robot, topics=topics, urdf_path=urdf))
        report['responses'].append({'command': int(value), 'success': response.success, 'message': response.message})

    def idle():
        def api(path):
            with urllib.request.urlopen('http://127.0.0.1:7880/api/' + path, timeout=8) as response: return json.load(response)
        assert not api('navigation/status')['is_up'] and api('services/bt_node/status')['state'] == 'down', 'Navigation/Task Engine must be stopped'
        assert states and states[-1].record_phase == RecordingStatus.READY, 'Recorder must be READY'
        assert node.count_publishers('/data/recording/status'), 'recording heartbeat unavailable'
        assert not inference or inference[-1].inference_phase == 0, 'inference must be unloaded/READY'
        # Orchestrator's publisher sends phases once on commands; it has no
        # idle heartbeat. A missing phase is acceptable only with all model
        # backends stopped, as in the existing simulation reset guard.
        for backend in ('vitacformer', 'lerobot', 'groot'):
            assert api(f'backends/{backend}/status')['container_state'] != 'running' or inference, 'backend running without inference heartbeat'

    try:
        assert client.wait_for_service(timeout_sec=5.), 'recording service unavailable'
        spin(3.)
        idle()
        assert latest and time.monotonic() - latest['received'] < 1., 'Isaac joint feedback unavailable/stale'
        assert all(node.count_publishers(groups[f'arm_{label}']['topic']) == 1 for label in ('left', 'right')), 'another leader command publisher is active'
        original = copy.deepcopy(states[-1])
        baseline = dict(feedback[-1])
        targets = []
        for delta, gripper in ((.015, .10), (-.015, .20)):
            target = {name: value + (delta if name.endswith('joint7') else 0.) for name, value in baseline.items() if name in ARMS}
            assert all(limits[name][0] <= value <= limits[name][1] for name, value in target.items()), 'arm target exceeds official SH5 URDF limits'
            assert -6.28 <= gripper <= 6.28, 'LG2 raw gripper exceeds official URDF limits'
            targets.append((target, gripper))
        started = True
        command(RecordingCommand.Request.START)
        spin(.3)
        assert states[-1].record_phase == RecordingStatus.RECORDING, 'recorder did not enter RECORDING'
        for target, gripper in targets:
            deadline, next_tick = time.monotonic() + 2.5, time.monotonic()
            while time.monotonic() < deadline:
                if time.monotonic() >= next_tick:
                    for side in ('l', 'r'):
                        names = [f'arm_{side}_joint{i}' for i in range(1, 8)]
                        message = JointTrajectory(joint_names=names + [f'gripper_{side}_joint1'])
                        # Official LG2 broadcaster leaves header.stamp zero;
                        # the recorder's ROS clock must provide action time.
                        point = JointTrajectoryPoint(positions=[target[n] for n in names] + [gripper])
                        point.time_from_start.nanosec = 100000000
                        message.points = [point]
                        publishers[side].publish(message)
                    next_tick += .05
                rclpy.spin_once(node, timeout_sec=min(.01, max(0., next_tick-time.monotonic())))
        spin(.3)
        command(RecordingCommand.Request.FINISH)
        started = False
        deadline, episodes = time.monotonic() + 40., []
        while time.monotonic() < deadline:
            spin(.1)
            episodes = [p for p in root.rglob('episode_info.json') if 'segments' not in p.parts]
            if episodes and states[-1].record_phase == 0 and json.loads(episodes[0].read_text()).get('transcoding_status') not in ('pending', 'running'): break
        assert states[-1].record_phase == 0 and len(episodes) == 1, 'archive not READY or one episode missing'
        episode = episodes[0].parent
        details = json.loads(episodes[0].read_text())
        assert details['schema_version'] == 'cyclo_isaac_mcap' and details['format_version'] == 'robotis_v2'
        assert details['robot_type'] == 'ffw_sh5_rev1'
        assert details['simulation']['hardware_calibrated'] is False
        assert details['simulation']['tactile_source'] == 'physx_contact_force_projected_grid_v1'
        for side, label in [('l', 'left'), ('r', 'right')]:
            arm, hand = groups[f'arm_{label}']['topic'], groups[f'hand_{label}']['topic']
            assert actions[arm] and all(names == [f'arm_{side}_joint{i}' for i in range(1, 8)] + [f'gripper_{side}_joint1'] for names, _ in actions[arm]), 'raw LG2 eight-joint input missing'
            assert actions[hand] and all(len(names) == 20 for names, _ in actions[hand]), 'translated20 hand commands missing'
            assert excursion([values for _, values in actions[hand]]) > .001 and any(abs(v) > .001 for _, values in actions[hand] for v in values), 'hand commands did not vary'
            assert excursion([[v[f'arm_{side}_joint7']] for v in feedback]) > .008, 'measured arm joint7 did not move'
            assert excursion([[v[f'finger_{side}_joint{i}'] for i in range(1, 21)] for v in feedback]) > .005, 'measured fingers did not move'
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(episode), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        types = {e.name: get_message(e.type) for e in reader.get_all_topics_and_types()}
        counts, stamps, logs, recorded_actions = collections.Counter(), {}, {}, {t: [] for t in action_topics}
        log_header_offsets = {}
        while reader.has_next():
            topic, data, logged = reader.read_next()
            message = deserialize_message(data, types[topic]); counts[topic] += 1
            logs.setdefault(topic, [logged, logged])[1] = logged
            if topic in action_topics:
                values = positions(message, action_topics[topic])
                assert all(limits[n][0] <= v <= limits[n][1] for n, v in zip(action_topics[topic], values)), 'recorded action exceeds official URDF limits'
                recorded_actions[topic].append(values)
                if len(action_topics[topic]) == 7:
                    assert message.header.stamp.sec == message.header.stamp.nanosec == 0, 'synthetic LG2 header must match official zero timestamp'
            if topic == '/arm_hand/joint_states': assert set(joint_values(message, ARMS + HANDS)) == set(ARMS + HANDS)
            if topic == '/joint_states': joint_values(message, AUX)
            if topic in ('/left_hand/finger_pressures', '/right_hand/finger_pressures'): pressure_values(message, 'l' if topic.startswith('/left') else 'r')
            stamp = message.clock if topic == '/clock' else getattr(getattr(message, 'header', None), 'stamp', None)
            if stamp is not None:
                ns = stamp.sec * 1000000000 + stamp.nanosec
                assert topic not in stamps or ns >= stamps[topic][1], f'header stamp regressed: {topic}'
                stamps.setdefault(topic, [ns, ns])[1] = ns
                if topic in ('/clock', '/arm_hand/joint_states', '/left_hand/finger_pressures', '/right_hand/finger_pressures'):
                    log_header_offsets[topic] = max(log_header_offsets.get(topic, 0), abs(logged-ns))
        required = list(action_topics) + ['/arm_hand/joint_states', '/joint_states', '/clock', '/left_hand/finger_pressures', '/right_hand/finger_pressures']
        assert all(counts[t] > 0 for t in required), 'required recorded action/state/tactile topic missing'
        for topic in required[-5:]: assert stamps[topic][1] > stamps[topic][0], f'header stamp did not advance: {topic}'
        assert all(offset <= 250000000 for offset in log_header_offsets.values()), 'MCAP log time differs from simulation sensor time'
        clock_start, clock_end = stamps['/clock']
        assert all(clock_start-250000000 <= logs[t][0] <= logs[t][1] <= clock_end+250000000 for t in action_topics), 'zero-header action MCAP timeline is outside simulation clock range'
        assert all(excursion(samples) > .001 for samples in recorded_actions.values()), 'recorded policy action did not vary'
        videos = {}
        for path in sorted((episode / 'videos').rglob('*.mp4')):
            capture, total, shapes = cv2.VideoCapture(str(path)), 0, set()
            while True:
                ok, frame = capture.read()
                if not ok: break
                total += 1; shapes.add(tuple(frame.shape))
            capture.release()
            expected = (376, 672, 3) if 'head' in path.stem else (424, 240, 3)
            assert total and shapes == {expected}, f'video dimensions/decode failed: {path}: {shapes}'
            timestamps = pd.read_parquet(path.with_name(path.stem + '_timestamps.parquet'))
            assert len(timestamps) == total and list(timestamps.frame_index) == list(range(total)), 'video timestamp/frame count mismatch'
            assert timestamps.header_stamp_ns.is_monotonic_increasing and timestamps.header_stamp_ns.iloc[-1] > timestamps.header_stamp_ns.iloc[0], 'camera timestamps invalid'
            assert timestamps.recv_ns.is_monotonic_increasing, 'camera reception timestamps regressed'
            assert clock_start-500000000 <= int(timestamps.header_stamp_ns.iloc[0]) <= int(timestamps.header_stamp_ns.iloc[-1]) <= clock_end+500000000, 'video timestamps are outside recorded simulation clock range'
            videos[path.name] = {'decoded_frames': total, 'frame_shape': expected, 'timestamp_rows': len(timestamps)}
        assert len(videos) == 4, 'four MP4 streams required'
        cameras = list((episode / 'camera_info').glob('cam_*.yaml'))
        assert len(cameras) == 4, 'four camera calibration snapshots required'
        for path in cameras:
            value = yaml.safe_load(path.read_text()); expected = (672, 376) if 'head' in path.stem else (424, 240)
            assert (int(value['width']), int(value['height'])) == expected, 'native CameraInfo resolution mismatch'
        report.update(result='passed', episode_dir=str(episode), policy_action_dimension=54, policy_state_dimension=54,
                      topic_counts=dict(counts), header_stamp_ranges_ns=stamps, mcap_log_ranges_ns=logs,
                      max_log_header_offsets_ns=log_header_offsets, videos=videos, episode_metadata=details,
                      observed_action_counts={t: len(a) for t, a in actions.items()},
                      measured_joint7_ranges={s: excursion([[v[f'arm_{s}_joint7']] for v in feedback]) for s in ('l', 'r')},
                      measured_finger_ranges={s: excursion([[v[f'finger_{s}_joint{i}'] for i in range(1, 21)] for v in feedback]) for s in ('l', 'r')})
    except Exception as error:
        report['error'] = repr(error)
    finally:
        try:
            if started: command(RecordingCommand.Request.FINISH)
            deadline = time.monotonic() + 40.
            while states and states[-1].record_phase != 0 and time.monotonic() < deadline: spin(.1)
            if original and states[-1].record_phase == 0:
                if original.task_info.task_num and original.task_info.task_name:
                    command(RecordingCommand.Request.SET_TASK_INFO, original.task_info, original.robot_type)
                    report['original_task_info_restored'] = True
                else: report['original_task_info_restored'] = 'original TaskInfo empty; official API has no clear operation'
                idle()
                reset = node.create_client(Trigger, '/simulation/reset')
                assert reset.wait_for_service(timeout_sec=4.), 'Isaac reset unavailable'
                response = result(reset, Trigger.Request(), 20.)
                report['reset'] = {'success': response.success, 'message': response.message}
        except Exception as error:
            report['failures'].append(f'cleanup/reset: {error}'); report['result'] = 'failed'
        report.update(recording_phases=phases, final_recording_phase=int(states[-1].record_phase) if states else None)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
        print(json.dumps(report, indent=2))
        node.destroy_node(); rclpy.shutdown()
    return 0 if report['result'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
