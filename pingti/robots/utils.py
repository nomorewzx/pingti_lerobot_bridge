from lerobot.robots import (
    RobotConfig,
    make_robot_from_config as make_lerobot_robot_from_config,
)
from lerobot.robots.robot import Robot


def make_robot_from_config(config: RobotConfig) -> Robot:
    if config.type == "pingti_follower":
        from pingti.robots.pingti_follower import PingtiFollower

        return PingtiFollower(config)
    elif config.type == "pingti_lite_follower":
        from pingti.robots.pingti_follower import PingtiLiteFollower

        return PingtiLiteFollower(config)
    elif config.type == "bi_pingti_follower":
        from pingti.robots.bi_pingti_follower import BiPingtiFollower

        return BiPingtiFollower(config)
    elif config.type == "mock_robot":
        from tests.mocks.mock_robot import MockRobot

        return MockRobot(config)

    return make_lerobot_robot_from_config(config)
