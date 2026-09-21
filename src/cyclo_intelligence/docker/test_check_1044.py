import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location("check_1044", Path(__file__).with_name("check_1044.py"))
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def test_missing_policy_sources_are_not_reported_as_ready(tmp_path):
    findings = dict((name, ready) for ready, name, _ in check.source_checks(tmp_path))
    assert not findings["native T-Rex"]
    assert not findings["FastWAM"]


def test_present_submodule_still_checks_policy_extras(tmp_path):
    project = tmp_path / "cyclo_brain/policy/lerobot/lerobot/pyproject.toml"
    project.parent.mkdir(parents=True)
    project.write_text('[project.optional-dependencies]\nact = []\n')
    findings = dict((name, ready) for ready, name, _ in check.source_checks(tmp_path))
    assert findings["LeRobot"]
    assert not findings["FastWAM dependency extra"]


def test_runtime_inspection_only_targets_wrapper_names(monkeypatch):
    names = {
        "CYCLO_MAIN_IMAGE": "isolated/main",
        "CYCLO_LEROBOT_IMAGE": "isolated/lerobot",
        "CYCLO_LEROBOT_TREX_IMAGE": "isolated/trex",
        "CYCLO_VITACFORMER_IMAGE": "isolated/vitacformer",
        "CYCLO_GROOT_IMAGE": "isolated/groot",
        "CYCLO_NAVIGATION_CONTAINER": "ai_worker_1044_sh5",
        "ROS_DOMAIN_ID": "104",
    }
    for name, value in names.items():
        monkeypatch.setenv(name, value)
    inspected = []

    def inspect(kind, name):
        inspected.append((kind, name))
        return {"State": {"Running": True}, "Config": {"Env": ["ROS_DOMAIN_ID=73"]}}

    monkeypatch.setattr(check, "inspect_docker", inspect)
    findings = dict((name, ready) for ready, name, _ in check.runtime_checks())
    assert findings["Navigation container"]
    assert not findings["Navigation ROS domain"]
    assert inspected[-1] == ("container", "ai_worker_1044_sh5")
