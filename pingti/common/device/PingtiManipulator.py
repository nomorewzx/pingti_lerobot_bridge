from dataclasses import dataclass
import json
from typing import List
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.motors.configs import MotorsBusConfig
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.robots.configs import ManipulatorRobotConfig
from lerobot.common.robot_devices.robots.manipulator import ManipulatorRobot
from lerobot.common.robot_devices.robots.utils import get_arm_id
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError
from pathlib import Path


def make_motors_buses_from_configs(motors_bus_configs: dict[str, MotorsBusConfig]) -> list[MotorsBus]:
    motors_buses = {}

    for key, cfg in motors_bus_configs.items():
        if cfg.type == "dynamixel":
            from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus

            motors_buses[key] = DynamixelMotorsBus(cfg)

        elif cfg.type == "feetech":
            from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

            motors_buses[key] = FeetechMotorsBus(cfg)
        elif cfg.type == "feetech_group":
            from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus
            motors_buses[key] = FeetechMotorGroupsBus(cfg)
        else:
            raise ValueError(f"The motor type '{cfg.type}' is not valid.")

    return motors_buses


class PingtiManipulatorRobot(ManipulatorRobot):
    # Init function is inherited from the parent class
    def __init__(self, config: ManipulatorRobotConfig):
        self.config = config
        self.robot_type = self.config.type
        self.calibration_dir = Path(self.config.calibration_dir)
        self.cameras = make_cameras_from_configs(self.config.cameras)
        self.is_connected = False
        self.logs = {}

        self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)
        self.follower_arms = make_motors_buses_from_configs(self.config.follower_arms)
        
    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "ManipulatorRobot is already connected. Do not run `robot.connect()` twice."
            )

        if not self.leader_arms and not self.follower_arms and not self.cameras:
            raise ValueError(
                "ManipulatorRobot doesn't have any device to connect. See example of usage in docstring of the class."
            )

        # Connect the arms
        for name in self.follower_arms:
            print(f"Connecting {name} follower arm.")
            self.follower_arms[name].connect()
        for name in self.leader_arms:
            print(f"Connecting {name} leader arm.")
            self.leader_arms[name].connect()

        if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
            from lerobot.common.robot_devices.motors.dynamixel import TorqueMode
        elif self.robot_type in ["so100", "moss", "pingti"]:
            from lerobot.common.robot_devices.motors.feetech import TorqueMode

        # We assume that at connection time, arms are in a rest position, and torque can
        # be safely disabled to run calibration and/or set robot preset configurations.
        for name in self.follower_arms:
            self.follower_arms[name].write("Torque_Enable", TorqueMode.DISABLED.value)
        for name in self.leader_arms:
            self.leader_arms[name].write("Torque_Enable", TorqueMode.DISABLED.value)

        self.activate_calibration()

        # Set robot preset (e.g. torque in leader gripper for Koch v1.1)
        if self.robot_type in ["koch", "koch_bimanual"]:
            self.set_koch_robot_preset()
        elif self.robot_type == "aloha":
            self.set_aloha_robot_preset()
        elif self.robot_type in ["so100", "moss"]:
            self.set_so100_robot_preset()
        elif self.robot_type in ["pingti"]:
            self.set_pingti_robot_preset()

        # Enable torque on all motors of the follower arms
        for name in self.follower_arms:
            print(f"Activating torque on {name} follower arm.")
            self.follower_arms[name].write("Torque_Enable", 1)

        if self.config.gripper_open_degree is not None:
            if self.robot_type not in ["koch", "koch_bimanual"]:
                raise NotImplementedError(
                    f"{self.robot_type} does not support position AND current control in the handle, which is require to set the gripper open."
                )
            # Set the leader arm in torque mode with the gripper motor set to an angle. This s it possible
            # to squeeze the gripper and have it spring back to an open position on its own.
            for name in self.leader_arms:
                self.leader_arms[name].write("Torque_Enable", 1, "gripper")
                self.leader_arms[name].write("Goal_Position", self.config.gripper_open_degree, "gripper")

        # Check both arms can be read
        for name in self.follower_arms:
            self.follower_arms[name].read("Present_Position")
        for name in self.leader_arms:
            self.leader_arms[name].read("Present_Position")

        # Connect the cameras
        for name in self.cameras:
            self.cameras[name].connect()

        self.is_connected = True
    
    def set_pingti_robot_preset(self):
        for name in self.follower_arms:
            # Mode=0 for Position Control
            self.follower_arms[name].write("Mode", 0)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.follower_arms[name].write("P_Coefficient", 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.follower_arms[name].write("I_Coefficient", 0)
            self.follower_arms[name].write("D_Coefficient", 0)
            # Close the write lock so that Maximum_Acceleration gets written to EPROM address,
            # which is mandatory for Maximum_Acceleration to take effect after rebooting.
            self.follower_arms[name].write("Lock", 0)
            # Set Maximum_Acceleration to 254 to speedup acceleration and deceleration of
            # the motors. Note: this configuration is not in the official STS3215 Memory Table
            self.follower_arms[name].write("Maximum_Acceleration", 125)
            self.follower_arms[name].write("Acceleration", 254)
    
    def activate_calibration(self):
        """After calibration all motors function in human interpretable ranges.
        Rotations are expressed in degrees in nominal range of [-180, 180],
        and linear motions (like gripper of Aloha) in nominal range of [0, 100].
        """

        def load_or_run_calibration_(name, arm, arm_type):
            arm_id = get_arm_id(name, arm_type)
            arm_calib_path = self.calibration_dir / f"{arm_id}.json"

            if arm_calib_path.exists():
                with open(arm_calib_path) as f:
                    calibration = json.load(f)
            else:
                # TODO(rcadene): display a warning in __init__ if calibration file not available
                print(f"Missing calibration file '{arm_calib_path}'")

                if self.robot_type in ["koch", "koch_bimanual", "aloha"]:
                    from lerobot.common.robot_devices.robots.dynamixel_calibration import run_arm_calibration

                    calibration = run_arm_calibration(arm, self.robot_type, name, arm_type)

                elif self.robot_type in ["so100", "moss", "pingti"]:
                    from lerobot.common.robot_devices.robots.feetech_calibration import (
                        run_arm_manual_calibration,
                    )

                    calibration = run_arm_manual_calibration(arm, self.robot_type, name, arm_type)
                print(f"Calibration is done! Saving calibration file '{arm_calib_path}'")
                arm_calib_path.parent.mkdir(parents=True, exist_ok=True)
                with open(arm_calib_path, "w") as f:
                    json.dump(calibration, f)

            return calibration

        for name, arm in self.follower_arms.items():
            calibration = load_or_run_calibration_(name, arm, "follower")
            arm.set_calibration(calibration)
        for name, arm in self.leader_arms.items():
            calibration = load_or_run_calibration_(name, arm, "leader")
            arm.set_calibration(calibration)