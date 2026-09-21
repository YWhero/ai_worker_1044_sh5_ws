"""Exercise the actual localhost framing contract, without Isaac or ROS."""
import json
import socket
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulator import JsonLineServer,PhysicsRuntime,json_safe


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.server = JsonLineServer(0)
        self.client = socket.create_connection(self.server.listener.getsockname())
        self.client.settimeout(1)
        self.server.poll()

    def tearDown(self):
        self.client.close()
        self.server.close()

    def test_partial_commands_and_two_commands_in_one_read(self):
        self.client.sendall(b'{"kind":"basevelocity","vx":')
        self.assertEqual(self.server.poll(),[])
        self.client.sendall(b'0.2}\n{"kind":"stop"}\n')
        self.assertEqual(self.server.poll(),[{"kind":"basevelocity","vx":.2},{"kind":"stop"}])

    def test_measured_frame_is_strict_json_with_null_lidar_misses(self):
        state = {"kind":"state","positions":[.12],"scan":{"ranges":[None,1.2]}}
        self.server.publish(state)
        self.assertEqual(json.loads(self.client.recv(4096)),state)
        with self.assertRaises(ValueError):
            self.server.encode({"positions":[float("nan")]})

    def test_bad_command_yields_structured_error_and_disconnect_is_observed(self):
        self.client.sendall(b'not json\n')
        self.assertEqual(self.server.poll(),[])
        self.server.flush()
        response=json.loads(self.client.recv(4096))
        self.assertEqual(response["kind"],"command_result")
        self.assertFalse(response["ok"])
        self.client.close()
        self.server.poll()
        self.assertTrue(self.server.disconnected)

    def test_shutdown_holds_measured_pose_and_sends_ack_before_close(self):
        runtime=PhysicsRuntime.__new__(PhysicsRuntime)
        runtime.np=SimpleNamespace(array=lambda values,dtype:tuple(values))
        runtime.robot=Mock()
        runtime.robot.get_joint_positions.return_value=[.1,.2]
        runtime.trajectories={"head":"pending"}
        runtime.stats={"commands":0}
        runtime.base_velocity=(.2,0,0)
        runtime.shutdown_requested=False
        transaction=runtime.command({"kind":"shutdown","request_id":"stop-123"},self.server)
        self.assertTrue(transaction)
        self.assertTrue(runtime.shutdown_requested)
        self.assertEqual(runtime.base_velocity,(0.,0.,0.))
        self.assertEqual(runtime.trajectories,{})
        self.assertEqual(runtime.target_positions,(.1,.2))
        runtime.robot.set_world_pose.assert_not_called()
        runtime.robot.set_joint_positions.assert_not_called()
        self.assertTrue(self.server.finish_responses())
        ack=json.loads(self.client.recv(4096))
        self.assertEqual(ack["kind"],"command_result")
        self.assertEqual(ack["request_id"],"stop-123")
        self.assertTrue(ack["ok"])

    def test_inspect_response_contains_native_values_and_does_not_change_targets(self):
        runtime=PhysicsRuntime.__new__(PhysicsRuntime)
        runtime.robot=Mock()
        runtime.world=SimpleNamespace(current_time=3.0)
        runtime.epoch=2
        runtime.names=["finger_l_joint1"]
        runtime.target_positions=[.85]
        runtime.initial_positions=[.86]
        runtime.base_velocity=(0.,0.,0.)
        runtime.last_base_command=float("-inf")
        runtime.metadata={"joints":{"finger_l_joint1":{"drive":{"stiffness":160}}}}
        runtime.hold_count=1
        runtime.last_hold_reason="sidecar disconnected"
        runtime.trajectories={}
        runtime.robot.get_joint_positions.return_value=[.84]
        runtime.robot.get_joint_velocities.return_value=[0.]
        controller=runtime.robot.get_articulation_controller.return_value
        controller.get_gains.return_value=([160.],[16.])
        controller.get_applied_action.return_value=SimpleNamespace(joint_positions=[.85],
                                                                  joint_velocities=[0.],joint_indices=None)
        self.assertFalse(runtime.command({"kind":"inspect","request_id":"inspect-1"},self.server))
        self.server.finish_responses()
        ack=json.loads(self.client.recv(4096))
        info=json.loads(ack["message"])
        self.assertTrue(ack["ok"])
        self.assertEqual(ack["request_id"],"inspect-1")
        self.assertEqual(info["native_stiffness"],[160.])
        self.assertEqual(info["measured_positions"],[.84])
        self.assertEqual(info["last_base_command_time"],None)
        self.assertEqual(runtime.target_positions,[.85])
        self.assertEqual(runtime.hold_count,1)
        runtime.robot.apply_action.assert_not_called()
        runtime.robot.set_world_pose.assert_not_called()
        self.assertEqual(json_safe([float("nan"),float("inf"),None]),[None,None,None])


if __name__ == "__main__":
    unittest.main()
