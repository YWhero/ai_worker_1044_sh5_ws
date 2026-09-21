#!/usr/bin/env python3

from __future__ import annotations

from unittest.mock import Mock

from lerobot_engine.engine import LeRobotEngine


def _loaded_engine() -> LeRobotEngine:
    engine = object.__new__(LeRobotEngine)
    engine._policy = object()
    engine._preprocessor = object()
    engine._postprocessor = object()
    engine._loaded_robot_type = "ffw_sh5_rev1"
    engine._loaded_model_path = "/models/tactile_act"
    engine._action_keys = ["arm_left", "arm_right", "hand_left", "hand_right"]
    engine._teardown_robot = Mock()
    engine._reset_policy_runtime_state = Mock()
    engine._init_robot = Mock()
    engine._infer_image_resize = Mock(return_value={"cam_left_head": (240, 320)})
    engine._image_resize = {}
    return engine


def test_reset_cycle_keeps_weights_and_reinitializes_episode_inputs():
    engine = _loaded_engine()

    result = engine.reset_cycle()

    assert result["success"] is True
    assert result["action_keys"] == engine._action_keys
    engine._teardown_robot.assert_called_once_with()
    engine._reset_policy_runtime_state.assert_called_once_with()
    engine._init_robot.assert_called_once_with("ffw_sh5_rev1")
    engine._infer_image_resize.assert_called_once_with(engine._policy)
    assert engine._loaded_model_path == "/models/tactile_act"
    assert engine._image_resize == {"cam_left_head": (240, 320)}


def test_reset_cycle_failure_tears_down_robot_but_preserves_cached_policy():
    engine = _loaded_engine()
    engine._init_robot.side_effect = RuntimeError("tactile input unavailable")

    result = engine.reset_cycle()

    assert result["success"] is False
    assert "tactile input unavailable" in result["message"]
    assert engine._teardown_robot.call_count == 2
    assert engine._loaded_model_path == "/models/tactile_act"
    assert engine._loaded_robot_type == "ffw_sh5_rev1"


def test_reset_cycle_rejects_when_no_policy_is_loaded():
    engine = _loaded_engine()
    engine._policy = None

    result = engine.reset_cycle()

    assert result == {
        "success": False,
        "message": "No loaded policy to reset",
    }
    engine._teardown_robot.assert_not_called()
