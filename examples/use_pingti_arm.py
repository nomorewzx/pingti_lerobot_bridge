from pingti.common.device.configs import FeetechMotorGroupsBusConfig
from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus

port="/dev/ttyUSB0"

motors={
                    # name: (index, model)
                    "shoulder_pan": [(1, "scs_series")],
                    "shoulder_lift": [(2, "scs_series"), (3, "scs_series")],
                    "elbow_flex": [(4, "scs_series"), (5, "scs_series")],
                    "wrist_flex": [(6, "scs_series")],
                    "wrist_roll": [(7, "scs_series")],
                    "gripper": [(8, "scs_series")],
                    "left_wheel": [(9, "scs_series")],
                    "back_wheel": [(10, "scs_series")],
                    "right_wheel": [(11, "scs_series")],
                }

feetech_motor_group_config = FeetechMotorGroupsBusConfig(port=port, motors=motors)

motors_bus = FeetechMotorGroupsBus(feetech_motor_group_config)

motors_bus.connect()

values = motors_bus.read_with_motor_ids(['scs_series'], [4,5], 'Present_Position')


motors_bus.write_with_motor_ids(['scs_series'], [4], 'Goal_Position', [2048])