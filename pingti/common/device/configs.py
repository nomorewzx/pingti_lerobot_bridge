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
                port="/dev/tty.usbserial-1140",
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
