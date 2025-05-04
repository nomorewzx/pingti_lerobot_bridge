from lerobot.common.robot_devices.motors.configs import MotorsBusConfig
from lerobot.common.robot_devices.motors.utils import MotorsBus

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
