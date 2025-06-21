from pingti.common.device.configs import NongBotRobotConfig
from pathlib import Path
motor_config = NongBotRobotConfig().leader_arms.get("main")

print('Motor Config below')
print(motor_config)

from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

motor_bus = FeetechMotorsBus(motor_config)

motor_bus.connect()

pos = motor_bus.read_with_motor_ids(['scs_series'],[2],'Present_Position')

print('Motor pos:', pos)


motor_bus.write_with_motor_ids(['scs_series'], [2], 'Goal_Position', [2048])