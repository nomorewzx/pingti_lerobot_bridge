from typing import List
from dataclasses import dataclass, field
from lerobot.common.robot_devices.cameras.configs import CameraConfig, OpenCVCameraConfig
from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig, MotorsBusConfig
from lerobot.common.robot_devices.robots.configs import ManipulatorRobotConfig, RobotConfig

@MotorsBusConfig.register_subclass("feetech_group")
@dataclass
class FeetechMotorGroupsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, List[tuple[int, str]]]
    mock: bool = False

@RobotConfig.register_subclass("pingti")
@dataclass
class PingTiRobotConfig(ManipulatorRobotConfig):
    calibration_dir: str = ".cache/calibration/pingti"
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    leader_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port="/dev/tty.usbmodem58A60699971",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [1, "sts3215"],
                    "shoulder_lift": [2, "sts3215"],
                    "elbow_flex": [3, "sts3215"],
                    "wrist_flex": [4, "sts3215"],
                    "wrist_roll": [5, "sts3215"],
                    "gripper": [6, "sts3215"],
                },
            ),
        }
    )

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorGroupsBusConfig( 
                port="/dev/tty.usbserial-A50285BI",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [(1, "scs_series")],
                    "shoulder_lift": [(2, "scs_series"), (3, "scs_series")],
                    "elbow_flex": [(4, "scs_series"), (5, "scs_series")],
                    "wrist_flex": [(6, "scs_series")],
                    "wrist_roll": [(7, "scs_series")],
                    "gripper": [(8, "scs_series")],
                },
            ),
        }
    )

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "wrist_camera": OpenCVCameraConfig(
                camera_index=0,
                fps=30,
                width=640,
                height=480,
            ),
            "front_camera": OpenCVCameraConfig(
                camera_index=1,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    mock: bool = False


@RobotConfig.register_subclass("lekiwi_pingti")
@dataclass
class LeKiwiPingTiRobotConfig(RobotConfig):
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # Network Configuration
    ip: str = "192.168.31.246"
    port: int = 5555
    video_port: int = 5556

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "front": OpenCVCameraConfig(
                camera_index="/dev/video1", fps=30, width=640, height=480, rotation=180
            ),
            "wrist": OpenCVCameraConfig(
                camera_index="/dev/video0", fps=30, width=640, height=480, rotation=90
            ),
        }
    )

    calibration_dir: str = ".cache/calibration/lekiwi"

    leader_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorsBusConfig(
                port="/dev/tty.usbmodem58A60699971",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [1, "sts3215"],
                    "shoulder_lift": [2, "sts3215"],
                    "elbow_flex": [3, "sts3215"],
                    "wrist_flex": [4, "sts3215"],
                    "wrist_roll": [5, "sts3215"],
                    "gripper": [6, "sts3215"],
                },
            ),
        }
    )

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "main": FeetechMotorGroupsBusConfig( 
                port="/dev/ttyUSB0",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [(1, "scs_series")],
                    "shoulder_lift": [(2, "scs_series"), (3, "scs_series")],
                    "elbow_flex": [(4, "scs_series"), (5, "scs_series")],
                    "wrist_flex": [(6, "scs_series")],
                    "wrist_roll": [(7, "scs_series")],
                    "gripper": [(8, "scs_series")],
                    "left_wheel": [(9, "scs_series")],
                    "back_wheel": [(10, "scs_series")],
                    "right_wheel": [(11, "scs_series")],
                },
            ),
        }
    )

    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # Movement
            "forward": "w",
            "backward": "s",
            "rotate_left": "z",
            "rotate_right": "x",
            # Speed control
            "speed_up": "r",
            "speed_down": "f",
            # quit teleop
            "quit": "q",
        }
    )

    mock: bool = False
@RobotConfig.register_subclass("nong_bot")
@dataclass
class NongBotRobotConfig(RobotConfig):
    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a list that is the same length as
    # the number of motors in your follower arms.
    max_relative_target: int | None = None

    # Network Configuration
    ip: str = "192.168.10.59"
    port: int = 5555
    video_port: int = 5556

    base_serial_port = '/dev/ttyBase'

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "front": OpenCVCameraConfig(
                camera_index="/dev/video1", fps=30, width=640, height=480, rotation=180
            ),
            "back": OpenCVCameraConfig(
               camera_index="/dev/video3", fps=30, width=640, height=480, rotation=180
            ),
            "right_wrist": OpenCVCameraConfig(
               camera_index="/dev/video7", fps=30, width=640, height=480, rotation=-90
            ),
            "left_wrist": OpenCVCameraConfig(
               camera_index="/dev/video5", fps=30, width=640, height=480, rotation=None
            )
        }
    )

    calibration_dir: str = ".cache/calibration/nong_bot"

    leader_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "right": FeetechMotorsBusConfig(
                port="/dev/tty.usbmodem58A60699971",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [1, "scs_series"],
                    "shoulder_lift": [2, "scs_series"],
                    "elbow_flex": [3, "scs_series"],
                    "wrist_flex": [4, "scs_series"],
                    "wrist_roll": [5, "scs_series"],
                    "gripper": [6, "scs_series"],
                },
            ),
            "left": FeetechMotorsBusConfig(
                port="/dev/tty.usbmodem5A680134741",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [1, "scs_series"],
                    "shoulder_lift": [2, "scs_series"],
                    "elbow_flex": [3, "scs_series"],
                    "wrist_flex": [4, "scs_series"],
                    "wrist_roll": [5, "scs_series"],
                    "gripper": [6, "scs_series"],
                },
            )
        }
    )

    follower_arms: dict[str, MotorsBusConfig] = field(
        default_factory=lambda: {
            "right": FeetechMotorGroupsBusConfig( 
                port="/dev/ttyRightArm",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [(1, "scs_series")],
                    "shoulder_lift": [(2, "scs_series"), (3, "scs_series")],
                    "elbow_flex": [(4, "scs_series"), (5, "scs_series")],
                    "wrist_flex": [(6, "scs_series")],
                    "wrist_roll": [(7, "scs_series")],
                    "gripper": [(8, "scs_series")]
                },
            ),
            "left": FeetechMotorGroupsBusConfig( 
                port="/dev/ttyLeftArm",
                motors={
                    # name: (index, model)
                    "shoulder_pan": [(1, "scs_series")],
                    "shoulder_lift": [(2, "scs_series"), (3, "scs_series")],
                    "elbow_flex": [(4, "scs_series"), (5, "scs_series")],
                    "wrist_flex": [(6, "scs_series")],
                    "wrist_roll": [(7, "scs_series")],
                    "gripper": [(8, "scs_series")]
                },
            )
        }
    )

    teleop_keys: dict[str, str] = field(
        # JoyCon button mapping
        default_factory=lambda: {
            # Movement
            "forward": "a",
            "backward": "y",
            "rotate_left": "x",
            "rotate_right": "b",
            "stop": "r",
        }
    )

    mock: bool = False



from lerobot.common.robot_devices.control_configs import ControlConfig
@ControlConfig.register_subclass("remote_nong_bot")
@dataclass
class RemoteNongBotConfig(ControlConfig):
    log_interval: int = 100
    # Display all cameras on screen
    display_data: bool = False
    # Rerun configuration for remote robot (https://ref.rerun.io/docs/python/0.22.1/common/initialization_functions/#rerun.connect_tcp)
    viewer_ip: str | None = None
    viewer_port: str | None = None


