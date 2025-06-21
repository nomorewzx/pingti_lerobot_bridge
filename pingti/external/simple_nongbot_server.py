import zmq
import serial
import time
import threading
import json
from lerobot.common.robot_devices.robots.mobile_manipulator import LeKiwi
from lerobot.common.robot_devices.robots.lekiwi_remote import calibrate_follower_arm, setup_zmq_sockets, run_camera_capture


def xor_check(data):
    xor = data[0]
    for b in data[1:]:
        xor ^= b
    return xor

def build_control_command(x_speed, steer_angle):
    buf = [0xCD, 0x0A, 0x01]

    # X 轴方向
    buf.append(0x00 if x_speed >= 0 else 0x01)
    x_abs = abs(x_speed)
    buf.append((x_abs >> 8) & 0xFF)
    buf.append(x_abs & 0xFF)

    # Y 轴占位（未使用）
    buf.extend([0x00, 0x00, 0x00])

    # 角速度方向
    buf.append(0x00 if steer_angle >= 0 else 0x01)
    steer_abs = abs(steer_angle)
    buf.append((steer_abs >> 8) & 0xFF)
    buf.append(steer_abs & 0xFF)

    # 校验
    buf.append(xor_check(buf[2:]))
    return bytes(buf)


class NongBase:
    def __init__(self, ser):
        self.ser = ser

    def set_velocity(self, command_speeds):
        cmd = build_control_command(command_speeds[0], command_speeds[1])
        self.ser.write(cmd)
        print(f"[Serial] Sent: {' '.join(hex(b) for b in cmd)}")

    def stop(self):
        cmd = build_control_command(0, 0)
        self.ser.write(cmd)
        print(f"[Serial] Sent: {' '.join(hex(b) for b in cmd)}")

def run_nong_bot(robot_config):
    """
    NongBot consists of a NongBase and a PingTi arm.
    The function will 
    1. Initialize the NongBase and PingTi arm.
    2. Initialize the ZeroMQ sockets for receiving commands and streaming observations.
    3. Process incoming commands (arm and wheel commands) and send back sensor and camera data.
    """
    # Import helper functions and classes
    from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
    from lerobot.common.robot_devices.motors.feetech import TorqueMode
    from pingti.common.device.feetech_motor_group import FeetechMotorGroupsBus

    # Initialize cameras from the robot configuration.
    cameras = make_cameras_from_configs(robot_config.cameras)
    for cam in cameras.values():
        cam.connect()

    # Initialize the motors bus using the follower arm configuration.
    motor_config = robot_config.follower_arms.get("main")
    if motor_config is None:
        print("[ERROR] Follower arm 'main' configuration not found.")
        return
    
    motor_bus = FeetechMotorGroupsBus(motor_config)
    motor_bus.connect()

    # Calibrate the follower arm.
    calibrate_follower_arm(motor_bus, robot_config.calibration_dir)

    # Define the expected arm motor IDs.
    arm_motor_ids = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    # Start the camera capture thread.
    latest_images_dict = {}
    images_lock = threading.Lock()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=run_camera_capture, args=(cameras, images_lock, latest_images_dict, stop_event), daemon=True
    )
    cam_thread.start()

    ser = serial.Serial(robot_config.base_serial_port, 115200, timeout=0.1)
    nong_base = NongBase(ser)

    context, cmd_socket, video_socket = setup_zmq_sockets(robot_config)
    
    last_cmd_time = time.time()
    print("LeKiwi robot server started. Waiting for commands...")

    try:
        while True:
            loop_start_time = time.time()
            raw_command = None
            # Process incoming commands (non-blocking).
            while True:
                try:
                    msg = cmd_socket.recv_string(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    data = json.loads(msg)
                    print('Data Received....')
                    print(data)
                    # Process arm position commands.
                    if "arm_positions" in data:
                        arm_positions = data["arm_positions"]
                        if not isinstance(arm_positions, list):
                            print(f"[ERROR] Invalid arm_positions: {arm_positions}")
                        elif len(arm_positions) < len(arm_motor_ids):
                            print(
                                f"[WARNING] Received {len(arm_positions)} arm positions, expected {len(arm_motor_ids)}"
                            )
                        else:
                            for motor, pos in zip(arm_motor_ids, arm_positions, strict=False):
                                motor_bus.write("Goal_Position", pos, motor)
                    # Process wheel (base) commands.
                    if "raw_velocity" in data:
                        raw_command = data["raw_velocity"]
                        command_speeds = [
                            int(raw_command.get("x_speed", 0)),
                            int(raw_command.get("steer_angle_speed", 0)),
                        ]
                        nong_base.set_velocity(command_speeds)
                        last_cmd_time = time.time()
                except Exception as e:
                    print(f"[ERROR] Parsing message failed: {e}")
            now = time.time()
            if now - last_cmd_time > 0.5:
                nong_base.stop()
                last_cmd_time = now

            # Set current velocity  to 0 if no raw_command received
            if raw_command is None:
                current_velocity = [0,0]
            else:
                current_velocity = raw_command

            # Read the follower arm state from the motors bus.
            follower_arm_state = []
            for motor in arm_motor_ids:
                try:
                    pos = motor_bus.read("Present_Position", motor)
                    # Convert the position to a float (or use as is if already numeric).
                    follower_arm_state.append(float(pos) if not isinstance(pos, (int, float)) else pos)
                except Exception as e:
                    print(f"[ERROR] Reading motor {motor} failed: {e}")

            # Get the latest camera images.
            with images_lock:
                images_dict_copy = dict(latest_images_dict)

            # Build the observation dictionary.
            observation = {
                "images": images_dict_copy,
                "raw_velocity": current_velocity,
                "follower_arm_state": follower_arm_state,
            }
            # Send the observation over the video socket.
            video_socket.send_string(json.dumps(observation))

            # Ensure a short sleep to avoid overloading the CPU.
            elapsed = time.time() - loop_start_time
            time.sleep(
                max(0.033 - elapsed, 0)
            )  # If robot jitters increase the sleep and monitor cpu load with `top` in cmd
    except KeyboardInterrupt:
        print("Shutting down NongBot server")
    finally:
        stop_event.set()
        cam_thread.join()
        nong_base.stop()
        motor_bus.disconnect()
        cmd_socket.close()
        video_socket.close()
        context.term()

if __name__ == "__main__":
    # 初始化串口
    robot_config = load_robot_config("robot_config.yaml")

    run_nong_bot(robot_config)
