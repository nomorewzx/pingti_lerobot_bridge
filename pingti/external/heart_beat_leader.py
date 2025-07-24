from pingti.common.device.configs import NongBotRobotConfig
from pathlib import Path
import json
from pingti.external import NongMobileManipulator
motor_config = NongBotRobotConfig().leader_arms.get("right")

print('Motor Config below')
print(motor_config)


from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

motor_bus = FeetechMotorsBus(motor_config)

motor_bus.connect()

calibration = '/Users/zxwang/repos/pingti_lerobot_bridge/.cache/calibration/nong_bot/right_leader.json'
with open(calibration, 'r') as f:
    calibration = json.load(f)

motor_bus.set_calibration(calibration)

group_read_pos = motor_bus.read('Present_Position')
print('Pos:')
print(group_read_pos)

print('Read torque enable')
torque_enabled = motor_bus.read("Torque_Enable")
print(f'Torque Enable: {torque_enabled}')