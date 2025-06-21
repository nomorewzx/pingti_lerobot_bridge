from pingti.common.device.configs import NongBotRobotConfig
motor_config = NongBotRobotConfig().follower_arms.get("main")

print('Motor Config below')
print(motor_config)

from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus

from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus

from lerobot.common.robot_devices.robots.lekiwi_remote import calibrate_follower_arm

motor_bus = FeetechMotorGroupsBus(motor_config)

motor_bus.connect()

calibrate_follower_arm(motor_bus, NongBotRobotConfig().calibration_dir)


for i in range(8):
    print(f'Read present position of motor {i+1}:')
    pos = motor_bus.read_with_motor_ids(['scs_series'],[i+1],'Present_Position')
    print(pos)

group_read_pos = motor_bus.read('Present_Position')
print(group_read_pos)

print('Read torque enable')
torque_enabled = motor_bus.read("Torque_Enable")
print('Torque Enable')


pos = motor_bus.read("Present_Position")
print(f'Pos for arm: {pos}')