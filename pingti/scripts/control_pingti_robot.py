from dataclasses import asdict
import logging
from pprint import pformat
from lerobot.common.robot_devices.control_configs import CalibrateControlConfig, ControlPipelineConfig, RecordControlConfig, ReplayControlConfig, TeleoperateControlConfig, RemoteRobotConfig
from lerobot.common.robot_devices.robots.configs import ManipulatorRobotConfig, RobotConfig
from lerobot.scripts.control_robot import _init_rerun
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser
from lerobot.scripts.control_robot import calibrate, record, replay, teleoperate
from pingti.common.device.configs import LeKiwiPingTiRobotConfig
from pingti.common.device.lekiwi_pingti_remote import run_lekiwi_pingti


def make_robot_from_config(config: RobotConfig):
    if isinstance(config, ManipulatorRobotConfig):
        from pingti.common.device.PingtiManipulator import PingtiManipulatorRobot
        return PingtiManipulatorRobot(config)
    elif isinstance(config, LeKiwiPingTiRobotConfig):
        from pingti.common.device.PingtiMobileManipulator import PingtiMobileManipulator
        return PingtiMobileManipulator(config)
    else:
        raise ValueError(f"Robot type '{config.robot_type}' is not available.")


@parser.wrap()
def control_pingti_robot(cfg: ControlPipelineConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    if isinstance(cfg.control, CalibrateControlConfig):
        calibrate(robot, cfg.control)
    elif isinstance(cfg.control, TeleoperateControlConfig):
        _init_rerun(control_config=cfg.control, session_name="lerobot_control_loop_teleop")
        teleoperate(robot, cfg.control)
    elif isinstance(cfg.control, RecordControlConfig):
        _init_rerun(control_config=cfg.control, session_name="lerobot_control_loop_record")
        record(robot, cfg.control)
    elif isinstance(cfg.control, ReplayControlConfig):
        replay(robot, cfg.control)
    elif isinstance(cfg.control, RemoteRobotConfig):
        _init_rerun(control_config=cfg.control, session_name="lerobot_control_loop_remote")
        run_lekiwi_pingti(cfg.robot)

    if robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
        robot.disconnect()

if __name__ == "__main__":
    control_pingti_robot()
