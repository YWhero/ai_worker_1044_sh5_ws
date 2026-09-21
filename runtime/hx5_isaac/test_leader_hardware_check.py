"""Post-launch readiness regressions without ROS, USB access, or robot commands."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).with_name('leader_hardware_check.py')
spec = importlib.util.spec_from_file_location('leader_hardware_check_under_test', SCRIPT)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def controllers(inactive=None):
    return SimpleNamespace(controller=[SimpleNamespace(name=name, type=kind,
        state='inactive' if name == inactive else 'active') for name, kind in check.CONTROLLERS.items()])


def joints(stamp_ns=1000000000):
    return SimpleNamespace(name=list(check.JOINTS), position=[.1]*len(check.JOINTS),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns//1000000000,
                                                    nanosec=stamp_ns%1000000000)))


def test_required_names_match_official_lg2_config_without_loading_ros():
    # The source file has repeated '/**' keys. Reuse its lossless parser.
    config_script = SCRIPT.with_name('leader_config.py')
    config_spec = importlib.util.spec_from_file_location('leader_config_hardware_test', config_script)
    config = importlib.util.module_from_spec(config_spec); config_spec.loader.exec_module(config)
    root = SCRIPT.parents[2]
    source = root/'src/ai_worker/ffw_bringup/config/ffw_lg2_leader/ffw_lg2_leader_ai_hardware_controller.yaml'
    official = config.yaml.load(source.read_text(), Loader=config.ControllerLoader)['/**']
    manager = official['controller_manager']['ros__parameters']
    assert check.CONTROLLERS == {name: manager[name]['type'] for name in check.CONTROLLERS}
    broadcaster = official['joint_trajectory_command_broadcaster']['ros__parameters']
    assert set(check.JOINTS) == set(broadcaster['left_joints'] + broadcaster['right_joints'])
    assert len(check.JOINTS) == 16


@pytest.mark.parametrize('name', list(check.CONTROLLERS))
def test_each_official_controller_must_be_active(name):
    with pytest.raises(RuntimeError, match=name+'=inactive'):
        check.controller_states(controllers(name))


def test_missing_duplicate_and_wrong_type_controllers_are_rejected():
    response = controllers(); response.controller.pop()
    with pytest.raises(RuntimeError, match='joint_state_broadcaster=missing'):
        check.controller_states(response)
    response = controllers(); response.controller.append(response.controller[0])
    with pytest.raises(RuntimeError, match='duplicate controller names'):
        check.controller_states(response)
    response = controllers(); response.controller[0].type = 'unrelated/Controller'
    with pytest.raises(RuntimeError, match='unexpected type'):
        check.controller_states(response)


@pytest.mark.parametrize('name', ['arm_l_joint7', 'arm_r_joint1', 'gripper_l_joint1', 'gripper_r_joint1'])
def test_missing_arm_or_gripper_feedback_is_rejected(name):
    message = joints(); index = message.name.index(name)
    message.name.pop(index); message.position.pop(index)
    with pytest.raises(RuntimeError, match='14 arm and 2 gripper'):
        check.joint_sample(message)


def test_duplicate_joint_cannot_masquerade_as_complete_state():
    message = joints(); message.name.append('gripper_r_joint1'); message.position.append(.2)
    with pytest.raises(RuntimeError, match='duplicate joint'):
        check.joint_sample(message)


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_extra_or_required_position_invalidates_sample(bad_value):
    message = joints(); message.name.append('extra_joint'); message.position.append(bad_value)
    with pytest.raises(RuntimeError, match='nonfinite'):
        check.joint_sample(message)


def test_names_positions_and_timestamp_must_be_valid():
    message = joints(); message.position.pop()
    with pytest.raises(RuntimeError, match='incomplete'):
        check.joint_sample(message)
    with pytest.raises(RuntimeError, match='zero header'):
        check.joint_sample(joints(0))
    message = joints(); message.header.stamp.nanosec = 1000000000
    with pytest.raises(RuntimeError, match='invalid header'):
        check.joint_sample(message)


def test_receipt_and_stamp_must_both_progress_but_joint_motion_is_not_required():
    feedback = check.Feedback(); feedback.receive(joints(), 10.)
    with pytest.raises(RuntimeError, match='advancing'):
        feedback.report(10.)
    feedback.receive(joints(1010000000), 10.01)
    report = feedback.report(10.02)
    assert report['required_joints'] == report['measured_joints'] == 16
    assert report['feedback_stamp_progress_ns'] == 10000000
    assert report['feedback_age_wall_sec'] == .01
    # Freshly received repetitions of an old stamp are not live feedback.
    feedback.receive(joints(1010000000), 11.1)
    with pytest.raises(RuntimeError, match='advancing'):
        feedback.report(11.1)
    with pytest.raises(RuntimeError, match='stale'):
        feedback.report(12.2)


def test_regression_or_invalid_sample_discards_prior_progress_until_valid_samples_resume():
    feedback = check.Feedback(); feedback.receive(joints(), 1.); feedback.receive(joints(1100000000), 1.1)
    feedback.receive(joints(500000000), 1.2)
    with pytest.raises(RuntimeError, match='regressed'):
        feedback.report(1.2)
    feedback.receive(joints(600000000), 1.3)
    assert feedback.report(1.3)['feedback_stamp_progress_ns'] == 100000000
    bad = joints(700000000); bad.position[0] = float('nan'); feedback.receive(bad, 1.4)
    with pytest.raises(RuntimeError, match='nonfinite'):
        feedback.report(1.4)
    feedback.receive(joints(800000000), 1.5)
    with pytest.raises(RuntimeError, match='advancing'):
        feedback.report(1.5)


class Harness:
    """Service discovery/controller activation and joint callbacks on a fake wall clock."""
    def __init__(self, publisher_count=1, service=True, response_delay=.05):
        self.now = 0.0
        self.publisher_count = publisher_count
        self.service = service
        self.response_delay = response_delay
        self.observed = check.Feedback()
        self.requests = []
        self.spin_intervals = []
        self.activate_at = .8
        self.publish_feedback = True

    def count_publishers(self, topic):
        assert topic == '/leader/joint_states'
        return self.publisher_count

    def service_is_ready(self):
        return self.service

    def call_async(self, request):
        self.requests.append(request)
        now, harness = self.now, self
        class Future:
            def done(self):
                return harness.now - now >= harness.response_delay
            def result(self):
                return controllers('joystick_controller' if harness.now < harness.activate_at else None)
        return Future()

    def spin_once(self, node, timeout_sec):
        assert node is self
        self.spin_intervals.append(timeout_sec)
        self.now += timeout_sec
        if self.publish_feedback:
            self.observed.receive(joints(1000000000+round(self.now*1000000000)), self.now)

    def run(self, timeout=2):
        return check.wait_for_readiness(self, self, self.observed, self.spin_once,
                                       lambda: object(), timeout, lambda: self.now)


def test_async_readiness_waits_for_all_controller_activation_and_live_state():
    harness = Harness()
    report = harness.run()
    assert report['ready'] is True and report['domain'] == 115
    assert report['controllers'] == dict.fromkeys(check.CONTROLLERS, 'active')
    assert report['publisher_count'] == 1 and report['feedback_stamp_progress_ns'] > 0
    assert len(harness.requests) >= 2  # Inactive startup is retried, not accepted.
    assert .8 <= harness.now <= 2


@pytest.mark.parametrize('kind', ['no_service', 'no_reply', 'no_publisher', 'no_feedback', 'inactive'])
def test_every_unready_startup_path_obeys_single_wall_deadline(kind):
    harness = Harness()
    if kind == 'no_service': harness.service = False
    if kind == 'no_reply': harness.response_delay = 100
    if kind == 'no_publisher': harness.publisher_count = 0
    if kind == 'no_feedback': harness.publish_feedback = False
    if kind == 'inactive': harness.activate_at = 100
    with pytest.raises(RuntimeError, match='Timed out after 2 wall seconds'):
        harness.run()
    assert harness.now == pytest.approx(2)
    assert all(0 <= duration <= .05 for duration in harness.spin_intervals)


def test_duplicate_publisher_fails_without_requesting_hardware_or_controller_changes():
    harness = Harness(publisher_count=2)
    with pytest.raises(RuntimeError, match='exactly one.*found 2'):
        harness.run()
    assert harness.requests == []


def test_wrong_private_domain_fails_before_ros_import_or_initialization(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    assert check.main([]) == 1
    report = json.loads(capsys.readouterr().out)
    assert report['ready'] is False and 'ROS_DOMAIN_ID=115' in report['error']


def test_runtime_failure_is_json_with_nonzero_exit_and_no_traceback(monkeypatch, capsys):
    def fail(timeout):
        raise RuntimeError('Controller readiness deadline expired')
    monkeypatch.setattr(check, 'check', fail)
    assert check.main([]) == 1
    captured = capsys.readouterr()
    assert captured.err == '' and captured.out.count('\n') == 1
    assert json.loads(captured.out)['error'] == 'Controller readiness deadline expired'


@pytest.mark.parametrize('timeout', ['0', '31', 'nan', 'inf'])
def test_invalid_deadline_has_concise_json_failure_without_running_check(timeout, monkeypatch, capsys):
    monkeypatch.setattr(check, 'check', lambda *args: pytest.fail('invalid timeout must not initialize ROS'))
    assert check.main(['--timeout', timeout]) == 1
    captured = capsys.readouterr()
    assert captured.err == '' and 'Traceback' not in captured.out
    assert json.loads(captured.out)['ready'] is False
