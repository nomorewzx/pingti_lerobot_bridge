from dataclasses import asdict
import logging
from pprint import pformat
from lerobot.common.robot_devices.control_configs import CalibrateControlConfig, ControlPipelineConfig, RecordControlConfig, ReplayControlConfig, TeleoperateControlConfig
from lerobot.common.robot_devices.robots.configs import ManipulatorRobotConfig, RobotConfig
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser
from lerobot.scripts.control_robot import calibrate, record, replay, teleoperate
from pingti.common.device.configs import PingTiRobotConfig

def make_robot_from_config(config: RobotConfig):
    if isinstance(config, ManipulatorRobotConfig):
        from pingti.common.device.PingtiManipulator import PingtiManipulatorRobot
        return PingtiManipulatorRobot(config)
    else:
        raise ValueError(f"Robot type '{config.robot_type}' is not available.")


@parser.wrap()
def control_pingti_robot(cfg: ControlPipelineConfig):
    print('hello in func')
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)

    if isinstance(cfg.control, CalibrateControlConfig):
        calibrate(robot, cfg.control)
    elif isinstance(cfg.control, TeleoperateControlConfig):
        teleoperate(robot, cfg.control)
    elif isinstance(cfg.control, RecordControlConfig):
        record(robot, cfg.control)
    elif isinstance(cfg.control, ReplayControlConfig):
        replay(robot, cfg.control)

    if robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
        robot.disconnect()

print('hello!')
if __name__ == "__main__":
    print('In control pingti robot')
    control_pingti_robot()
