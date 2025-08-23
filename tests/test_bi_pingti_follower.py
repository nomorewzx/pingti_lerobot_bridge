import unittest
from unittest.mock import MagicMock, patch
from types import SimpleNamespace

from pingti.robots.bi_pingti_follower.bi_pingti_follower import BiPingtiFollower
from pingti.robots.bi_pingti_follower.config_bi_pingti_follower import BiPingtiFollowerConfig


class DummyBiPingtiFollowerConfig:
    id = 'dummy_bi_pingti_id'
    calibration_dir = None
    left_arm_port = 'dummy_left_port'
    right_arm_port = 'dummy_right_port'
    left_arm_disable_torque_on_disconnect = True
    left_arm_max_relative_target = None
    left_arm_use_degrees = False
    right_arm_disable_torque_on_disconnect = True
    right_arm_max_relative_target = None
    right_arm_use_degrees = False
    cameras = {}


class TestBiPingtiFollowerMotorsFt(unittest.TestCase):
    def setUp(self):
        self.config = DummyBiPingtiFollowerConfig()
        
        # Create mock left and right arms with motors including secondary ones
        self.left_arm = MagicMock()
        self.right_arm = MagicMock()
        
        # Mock motors for left arm - some with secondary, some without
        self.left_arm.bus.motors = {
            'shoulder_lift': MagicMock(),
            'shoulder_lift_secondary': MagicMock(),
            'elbow_flex': MagicMock(),
            'elbow_flex_secondary': MagicMock(),
            'wrist_flex': MagicMock(),
            'gripper': MagicMock(),
        }
        
        # Mock motors for right arm - some with secondary, some without
        self.right_arm.bus.motors = {
            'shoulder_lift': MagicMock(),
            'elbow_flex': MagicMock(),
            'elbow_flex_secondary': MagicMock(),
            'wrist_flex': MagicMock(),
            'gripper': MagicMock(),
        }
        
        # Create the BiPingtiFollower instance
        self.robot = BiPingtiFollower(self.config)
        
        # Replace the arms with our mocked ones
        self.robot.left_arm = self.left_arm
        self.robot.right_arm = self.right_arm

    def test_motors_ft_filters_out_secondary_motors(self):
        """Test that _motors_ft property filters out motors ending with 'secondary'"""
        motors_ft = self.robot._motors_ft
        
        # Check that primary motors are included
        self.assertIn('left_shoulder_lift.pos', motors_ft)
        self.assertIn('left_elbow_flex.pos', motors_ft)
        self.assertIn('left_wrist_flex.pos', motors_ft)
        self.assertIn('left_gripper.pos', motors_ft)
        
        self.assertIn('right_shoulder_lift.pos', motors_ft)
        self.assertIn('right_elbow_flex.pos', motors_ft)
        self.assertIn('right_wrist_flex.pos', motors_ft)
        self.assertIn('right_gripper.pos', motors_ft)
        
        # Check that secondary motors are filtered out
        self.assertNotIn('left_shoulder_lift_secondary.pos', motors_ft)
        self.assertNotIn('left_elbow_flex_secondary.pos', motors_ft)
        self.assertNotIn('right_shoulder_lift_secondary.pos', motors_ft)
        self.assertNotIn('right_elbow_flex_secondary.pos', motors_ft)
        
        # Verify the type is correct for all included motors
        for motor_name, motor_type in motors_ft.items():
            self.assertEqual(motor_type, float)
        
        # Verify the total count (should be 8: 4 left + 4 right, excluding secondary motors)
        self.assertEqual(len(motors_ft), 8)

    def test_motors_ft_with_only_primary_motors(self):
        """Test _motors_ft when there are no secondary motors"""
        # Remove secondary motors
        self.left_arm.bus.motors = {
            'shoulder_lift': MagicMock(),
            'elbow_flex': MagicMock(),
            'wrist_flex': MagicMock(),
        }
        
        self.right_arm.bus.motors = {
            'shoulder_lift': MagicMock(),
            'elbow_flex': MagicMock(),
            'wrist_flex': MagicMock(),
        }
        
        motors_ft = self.robot._motors_ft
        
        # All motors should be included
        self.assertIn('left_shoulder_lift.pos', motors_ft)
        self.assertIn('left_elbow_flex.pos', motors_ft)
        self.assertIn('left_wrist_flex.pos', motors_ft)
        self.assertIn('right_shoulder_lift.pos', motors_ft)
        self.assertIn('right_elbow_flex.pos', motors_ft)
        self.assertIn('right_wrist_flex.pos', motors_ft)
        
        # Should have 6 motors total
        self.assertEqual(len(motors_ft), 6)

    def test_motors_ft_with_only_secondary_motors(self):
        """Test _motors_ft when all motors are secondary (should return empty dict)"""
        # Only secondary motors
        self.left_arm.bus.motors = {
            'shoulder_lift_secondary': MagicMock(),
            'elbow_flex_secondary': MagicMock(),
        }
        
        self.right_arm.bus.motors = {
            'shoulder_lift_secondary': MagicMock(),
            'elbow_flex_secondary': MagicMock(),
        }
        
        motors_ft = self.robot._motors_ft
        
        # Should be empty since all motors end with 'secondary'
        self.assertEqual(len(motors_ft), 0)
        self.assertEqual(motors_ft, {})


if __name__ == '__main__':
    unittest.main()
