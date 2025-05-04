import os
import sys
from pathlib import Path
import torch

from lerobot.common.robot_devices.robots.mobile_manipulator import MobileManipulator
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs

from pingti.common.device.configs import LeKiwiPingTiRobotConfig
from pingti.common.device.utils import make_motors_buses_from_configs

PYNPUT_AVAILABLE = True
try:
    # Only import if there's a valid X server or if we're not on a Pi
    if ("DISPLAY" not in os.environ) and ("linux" in sys.platform):
        print("No DISPLAY set. Skipping pynput import.")
        raise ImportError("pynput blocked intentionally due to no display.")

    from pynput import keyboard
except ImportError:
    keyboard = None
    PYNPUT_AVAILABLE = False
except Exception as e:
    keyboard = None
    PYNPUT_AVAILABLE = False
    print(f"Could not import pynput: {e}")

class PingtiMobileManipulator(MobileManipulator):
    def __init__(self, config: LeKiwiPingTiRobotConfig):
        """
        Expected keys in config:
          - ip, port, video_port for the remote connection.
          - calibration_dir, leader_arms, follower_arms, max_relative_target, etc.
        """
        self.robot_type = config.type
        self.config = config
        self.remote_ip = config.ip
        self.remote_port = config.port
        self.remote_port_video = config.video_port
        self.calibration_dir = Path(self.config.calibration_dir)
        self.logs = {}

        self.teleop_keys = self.config.teleop_keys

        # For teleoperation, the leader arm (local) is used to record the desired arm pose.
        self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)

        self.follower_arms = make_motors_buses_from_configs(self.config.follower_arms)

        self.cameras = make_cameras_from_configs(self.config.cameras)

        self.is_connected = False

        self.last_frames = {}
        self.last_present_speed = {}
        self.last_remote_arm_state = torch.zeros(6, dtype=torch.float32)

        # Define three speed levels and a current index
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow

        # ZeroMQ context and sockets.
        self.context = None
        self.cmd_socket = None
        self.video_socket = None

        # Keyboard state for base teleoperation.
        self.running = True
        self.pressed_keys = {
            "forward": False,
            "backward": False,
            "left": False,
            "right": False,
            "rotate_left": False,
            "rotate_right": False,
        }

        if PYNPUT_AVAILABLE:
            print("pynput is available - enabling local keyboard listener.")
            self.listener = keyboard.Listener(
                on_press=self.on_press,
                on_release=self.on_release,
            )
            self.listener.start()
        else:
            print("pynput not available - skipping local keyboard listener.")
            self.listener = None