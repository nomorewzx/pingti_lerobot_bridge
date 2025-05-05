"""
Copy from https://github.com/huggingface/lerobot/blob/main/lerobot/common/robot_devices/robots/lekiwi_remote.py 
and modify for PingTi arm
"""

import time
import json
import threading
import cv2
import zmq
from lerobot.common.robot_devices.robots.mobile_manipulator import LeKiwi
from lerobot.common.robot_devices.robots.lekiwi_remote import calibrate_follower_arm, setup_zmq_sockets, run_camera_capture
from pathlib import Path


def run_lekiwi_pingti(robot_config):
    """
    Runs the LeKiwi robot:
      - Sets up cameras and connects them.
      - Initializes the follower arm motors.
      - Calibrates the follower arm if necessary.
      - Creates ZeroMQ sockets for receiving commands and streaming observations.
      - Processes incoming commands (arm and wheel commands) and sends back sensor and camera data.
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
    motors_bus = FeetechMotorGroupsBus(motor_config)
    motors_bus.connect()

    # Calibrate the follower arm.
    calibrate_follower_arm(motors_bus, robot_config.calibration_dir)

    # Create the LeKiwi robot instance.
    robot = LeKiwi(motors_bus)

    # Define the expected arm motor IDs.
    arm_motor_ids = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

    # Disable torque for each arm motor.
    for motor in arm_motor_ids:
        motors_bus.write("Torque_Enable", TorqueMode.DISABLED.value, motor)

    # Set up ZeroMQ sockets.
    context, cmd_socket, video_socket = setup_zmq_sockets(robot_config)

    # Start the camera capture thread.
    latest_images_dict = {}
    images_lock = threading.Lock()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=run_camera_capture, args=(cameras, images_lock, latest_images_dict, stop_event), daemon=True
    )
    cam_thread.start()

    last_cmd_time = time.time()
    print("LeKiwi robot server started. Waiting for commands...")

    try:
        while True:
            loop_start_time = time.time()

            # Process incoming commands (non-blocking).
            while True:
                try:
                    msg = cmd_socket.recv_string(zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    data = json.loads(msg)
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
                                motors_bus.write("Goal_Position", pos, motor)
                    # Process wheel (base) commands.
                    if "raw_velocity" in data:
                        raw_command = data["raw_velocity"]
                        # Expect keys: "left_wheel", "back_wheel", "right_wheel".
                        command_speeds = [
                            int(raw_command.get("left_wheel", 0)),
                            int(raw_command.get("back_wheel", 0)),
                            int(raw_command.get("right_wheel", 0)),
                        ]
                        robot.set_velocity(command_speeds)
                        last_cmd_time = time.time()
                except Exception as e:
                    print(f"[ERROR] Parsing message failed: {e}")

            # Watchdog: stop the robot if no command is received for over 0.5 seconds.
            now = time.time()
            if now - last_cmd_time > 0.5:
                robot.stop()
                last_cmd_time = now

            # Read current wheel speeds from the robot.
            current_velocity = robot.read_velocity()

            # Read the follower arm state from the motors bus.
            follower_arm_state = []
            for motor in arm_motor_ids:
                try:
                    pos = motors_bus.read("Present_Position", motor)
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
                "present_speed": current_velocity,
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
        print("Shutting down LeKiwi server.")
    finally:
        stop_event.set()
        cam_thread.join()
        robot.stop()
        motors_bus.disconnect()
        cmd_socket.close()
        video_socket.close()
        context.term()