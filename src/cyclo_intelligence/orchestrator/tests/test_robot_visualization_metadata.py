"""The robot-info service must expose measured auxiliary joints to live 3D.

Execute the unchanged service method without ROS initialization so this
metadata contract can be tested with the host Python environment.
"""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "shared"))
from shared.robot_configs import schema as robot_schema  # noqa: E402


def robot_info_callback():
    path = ROOT / "orchestrator/orchestrator/orchestrator_node.py"
    tree = ast.parse(path.read_text())
    node = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
        and node.name == "get_robot_info_callback"
    )
    module = ast.Module(body=[node], type_ignores=[])
    namespace = {"robot_schema": robot_schema}
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    return namespace[node.name]


def test_sh5_robot_info_includes_auxiliary_feedback_and_preserves_policy_commands():
    section = robot_schema.load_robot_section("ffw_sh5_rev1")
    instance = SimpleNamespace(robot_section=section, robot_type="ffw_sh5_rev1")
    response = robot_info_callback()(instance, SimpleNamespace(), SimpleNamespace())

    assert response.success
    assert response.state_joint_topics == ["/arm_hand/joint_states", "/joint_states"]
    assert response.action_topics == robot_schema.get_action_topics(section)
    assert response.action_topic_types == robot_schema.get_action_topic_types(section)
    assert response.urdf_path.endswith("ffw_sh5_follower.urdf")
    assert robot_schema.get_joint_state_topics(section) == ["/arm_hand/joint_states"]
    assert sum(len(group["joint_names"])
               for group in robot_schema.get_state_groups(section).values()) == 54


def test_sg2_robot_info_feedback_is_unchanged():
    section = robot_schema.load_robot_section("ffw_sg2_rev1")
    instance = SimpleNamespace(robot_section=section, robot_type="ffw_sg2_rev1")
    response = robot_info_callback()(instance, SimpleNamespace(), SimpleNamespace())
    assert response.success
    assert response.state_joint_topics == robot_schema.get_joint_state_topics(section)


def test_robot_info_without_selected_robot_does_not_expose_stale_model_metadata():
    instance = SimpleNamespace(robot_section=None, robot_type="")
    response = robot_info_callback()(instance, SimpleNamespace(), SimpleNamespace())
    assert response.success is False
    assert response.urdf_path == ""
    assert response.robot_type == ""
