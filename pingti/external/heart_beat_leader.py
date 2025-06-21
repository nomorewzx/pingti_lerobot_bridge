from pingti.common.device.configs import NongBotRobotConfig
from pathlib import Path
motor_config = NongBotRobotConfig().leader_arms.get("main")

print('Motor Config below')
print(motor_config)

from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

motor_bus = FeetechMotorsBus(motor_config)

motor_bus.connect()

for i in range(6):
    print(f'Read present position of motor {i+1}:')
    pos = motor_bus.read_with_motor_ids(['scs_series'],[i+1],'Present_Position')
    print(pos)

pos = motor_bus.read_with_motor_ids(['scs_series','scs_series','scs_series'],
[2,5,6],'Present_Position')
print('read multiple motor with multiple ids')
print(pos)


group_read_pos = motor_bus.read('Present_Position')
print('Pos:')
print(group_read_pos)

print('Read torque enable')
torque_enabled = motor_bus.read("Torque_Enable")
print(f'Torque Enable: {torque_enabled}')

import json

leader_arm_calib_path = Path(NongBotRobotConfig().calibration_dir) / "main_leader.json"

with open(leader_arm_calib_path) as f:
    calibration = json.load(f)
motor_bus.set_calibration(calibration=calibration)

calib_pos = motor_bus.read('Present_Position')
print(f'calib pos: {calib_pos}')