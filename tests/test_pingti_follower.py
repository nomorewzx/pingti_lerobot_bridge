import unittest
from unittest.mock import MagicMock, patch
from types import SimpleNamespace
import tempfile
import shutil
import os
import json
from pathlib import Path
from lerobot.motors import MotorNormMode

# Import the class under test
from pingti.robots.pingti_follower.pingti_follower import PingtiFollower

class DummyConfig:
    id = "dummy_id"
    port = 'dummy_port'
    cameras = {}
    use_degrees = False
    max_relative_target = None
    disable_torque_on_disconnect = True
    calibration_dir = None  # Will be set in setUp

class TestPingtiFollowerSendAction(unittest.TestCase):
    def setUp(self):
        # Create a temp directory for calibration as a Path object
        self.temp_dir = Path(tempfile.mkdtemp())
        DummyConfig.calibration_dir = self.temp_dir
        self.config = DummyConfig()
        # Create a minimal mock calibration file (simulate lerobot's mock_calibration_dir)
        calib_data = {
            "shoulder_lift": {},
            "shoulder_lift_secondary": {},
            "elbow_flex": {},
            "elbow_flex_secondary": {},
            "wrist_flex": {},
        }
        with open(self.temp_dir / "main_follower.json", "w") as f:
            json.dump(calib_data, f)
        self.robot = PingtiFollower(self.config)
        self.robot.bus = MagicMock()
        self.robot.bus.is_connected = True
        self.robot.cameras = {}  # No cameras, so all(cam.is_connected for ...) is True
        self.robot.mirror_joints = ['shoulder_lift', 'elbow_flex']
        self.robot.bus.motors = {
            'shoulder_lift': SimpleNamespace(norm_mode=MotorNormMode.DEGREES),
            'shoulder_lift_secondary': SimpleNamespace(norm_mode=MotorNormMode.DEGREES),
            'elbow_flex': SimpleNamespace(norm_mode=MotorNormMode.RANGE_0_100),
            'elbow_flex_secondary': SimpleNamespace(norm_mode=MotorNormMode.RANGE_0_100),
            'wrist_flex': SimpleNamespace(norm_mode=MotorNormMode.DEGREES),
        }
        self.robot.bus.sync_write = MagicMock()
        self.robot.bus.sync_read = MagicMock(return_value={
            'shoulder_lift': 0,
            'shoulder_lift_secondary': 0,
            'elbow_flex': 0,
            'elbow_flex_secondary': 0,
            'wrist_flex': 0,
        })

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_send_action_mirror_joint_degrees(self):
        action = {'shoulder_lift.pos': 42.0}
        self.robot.bus.motors['shoulder_lift'].norm_mode = MotorNormMode.DEGREES
        self.robot.bus.motors['shoulder_lift_secondary'].norm_mode = MotorNormMode.DEGREES
        sent = self.robot.send_action(action)
        args, kwargs = self.robot.bus.sync_write.call_args
        self.assertEqual(args[0], 'Goal_Position')
        goal_dict = args[1]
        self.assertIn('shoulder_lift', goal_dict)
        self.assertIn('shoulder_lift_secondary', goal_dict)
        self.assertEqual(goal_dict['shoulder_lift'], 42.0)
        self.assertEqual(goal_dict['shoulder_lift_secondary'], -42.0)
        self.assertEqual(sent['shoulder_lift.pos'], 42.0)
        self.assertEqual(sent['shoulder_lift_secondary.pos'], -42.0)

    def test_send_action_mirror_joint_range_0_100(self):
        action = {'elbow_flex.pos': 30.0}
        self.robot.bus.motors['elbow_flex'].norm_mode = MotorNormMode.RANGE_0_100
        self.robot.bus.motors['elbow_flex_secondary'].norm_mode = MotorNormMode.RANGE_0_100
        sent = self.robot.send_action(action)
        args, kwargs = self.robot.bus.sync_write.call_args
        goal_dict = args[1]
        self.assertIn('elbow_flex', goal_dict)
        self.assertIn('elbow_flex_secondary', goal_dict)
        self.assertEqual(goal_dict['elbow_flex'], 30.0)
        self.assertEqual(goal_dict['elbow_flex_secondary'], 70.0)
        self.assertEqual(sent['elbow_flex.pos'], 30.0)
        self.assertEqual(sent['elbow_flex_secondary.pos'], 70.0)

    def test_send_action_non_mirror_joint(self):
        action = {'wrist_flex.pos': 10.0}
        sent = self.robot.send_action(action)
        args, kwargs = self.robot.bus.sync_write.call_args
        goal_dict = args[1]
        self.assertIn('wrist_flex', goal_dict)
        self.assertNotIn('wrist_flex_secondary', goal_dict)
        self.assertEqual(goal_dict['wrist_flex'], 10.0)
        self.assertEqual(sent['wrist_flex.pos'], 10.0)

    def test_send_action_mirror_joint_range_m100_100(self):
        action = {'shoulder_lift.pos': 55.0}
        self.robot.bus.motors['shoulder_lift'].norm_mode = MotorNormMode.RANGE_M100_100
        self.robot.bus.motors['shoulder_lift_secondary'].norm_mode = MotorNormMode.RANGE_M100_100
        sent = self.robot.send_action(action)
        args, kwargs = self.robot.bus.sync_write.call_args
        goal_dict = args[1]
        self.assertIn('shoulder_lift', goal_dict)
        self.assertIn('shoulder_lift_secondary', goal_dict)
        self.assertEqual(goal_dict['shoulder_lift'], 55.0)
        self.assertEqual(goal_dict['shoulder_lift_secondary'], -55.0)
        self.assertEqual(sent['shoulder_lift.pos'], 55.0)
        self.assertEqual(sent['shoulder_lift_secondary.pos'], -55.0)

    def test_get_observation_filters_secondary(self):
        # Simulate bus returning both primary and secondary
        self.robot.bus.sync_read.return_value = {
            'shoulder_lift': 12.3,
            'shoulder_lift_secondary': 99.9,
            'elbow_flex': 45.6,
            'elbow_flex_secondary': 88.8,
            'wrist_flex': 7.8,
        }
        obs = self.robot.get_observation()
        # Only primary joints should be present
        self.assertIn('shoulder_lift.pos', obs)
        self.assertIn('elbow_flex.pos', obs)
        self.assertIn('wrist_flex.pos', obs)
        self.assertNotIn('shoulder_lift_secondary.pos', obs)
        self.assertNotIn('elbow_flex_secondary.pos', obs)
        # Values should match
        self.assertEqual(obs['shoulder_lift.pos'], 12.3)
        self.assertEqual(obs['elbow_flex.pos'], 45.6)
        self.assertEqual(obs['wrist_flex.pos'], 7.8)

if __name__ == '__main__':
    unittest.main()