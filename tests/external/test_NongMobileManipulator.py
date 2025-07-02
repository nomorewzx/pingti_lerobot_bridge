import unittest
from unittest.mock import MagicMock, patch
import torch
import json
from pingti.common.device.configs import NongBotRobotConfig
from dataclasses import dataclass, field
import typing

from pingti.external.NongMobileManipulator import NongMobileManipulator

@dataclass
class DummyConfig(NongBotRobotConfig):
    ip: str = '127.0.0.1'
    port: int = 5555
    video_port: int = 5556
    calibration_dir: str = '.'
    leader_arms: dict = field(default_factory=dict)
    follower_arms: dict = field(default_factory=dict)
    cameras: dict = field(default_factory=dict)
    teleop_keys: dict = field(default_factory=dict)
    type: str = 'dummy'
    mock: bool = True

class TestNongMobileManipulatorSendAction(unittest.TestCase):
    def setUp(self):
        self.config = DummyConfig()
        # Patch JoyConListener to prevent thread from starting
        patcher = patch('pingti.external.NongMobileManipulator.JoyConListener', autospec=True)
        self.addCleanup(patcher.stop)
        self.mock_joycon = patcher.start()
        self.robot = NongMobileManipulator(self.config)
        self.robot.is_connected = True
        self.robot.cmd_socket = MagicMock()

    def test_send_action_full_tensor(self):
        action = torch.arange(14, dtype=torch.float32)
        returned = self.robot.send_action(action)
        # Check that the sent message is correct
        sent_json = json.loads(self.robot.cmd_socket.send_string.call_args[0][0])  # type: ignore
        self.assertIn('raw_velocity', sent_json)
        self.assertIn('arm_positions', sent_json)
        self.assertEqual(sent_json['raw_velocity']['x_speed'], 12.0)
        self.assertEqual(sent_json['raw_velocity']['steer_angle_speed'], 13.0)
        self.assertEqual(sent_json['arm_positions']['right'], list(range(6)))
        self.assertEqual(sent_json['arm_positions']['left'], list(range(6, 12)))
        self.assertTrue(torch.equal(returned, action))

    def test_send_action_short_tensor(self):
        action = torch.arange(10, dtype=torch.float32)  # Will be padded to 14
        returned = self.robot.send_action(action)
        sent_json = json.loads(self.robot.cmd_socket.send_string.call_args[0][0])  # type: ignore
        self.assertEqual(sent_json['raw_velocity']['x_speed'], 0.0)  # padded
        self.assertEqual(sent_json['raw_velocity']['steer_angle_speed'], 0.0)  # padded
        self.assertEqual(len(sent_json['arm_positions']['right']), 6)
        self.assertEqual(len(sent_json['arm_positions']['left']), 6)
        self.assertTrue(torch.equal(returned, torch.cat([action, torch.zeros(4)])))

    def test_send_action_not_connected(self):
        self.robot.is_connected = False
        with self.assertRaises(Exception) as context:
            self.robot.send_action(torch.zeros(14))
        self.assertIn('Not connected', str(context.exception))

if __name__ == '__main__':
    unittest.main() 