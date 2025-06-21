import base64
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import zmq

from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.motors.feetech import TorqueMode
from lerobot.common.robot_devices.motors.utils import MotorsBus
from pingti.common.device.utils import make_motors_buses_from_configs
from lerobot.common.robot_devices.robots.configs import LeKiwiRobotConfig
from lerobot.common.robot_devices.robots.feetech_calibration import run_arm_manual_calibration
from lerobot.common.robot_devices.robots.utils import get_arm_id
from lerobot.common.robot_devices.utils import RobotDeviceNotConnectedError
import time
from pingti.common.device.configs import NongBotRobotConfig
from threading import Thread
from pyjoycon import JoyCon, get_R_id

class JoyConListener(Thread):
    def __init__(self, on_press, on_release):
        super().__init__()
        self.running = True
        self.joycon = JoyCon(*get_R_id())  # 连接右手 Joy-Con
        self.on_press = on_press
        self.on_release = on_release

        self.prev_buttons = set()

    def get_pressed_buttons(self, state):
        pressed = set()

        for section in ['left', 'right', 'shared']:
            if section in state['buttons']:
                for btn, val in state['buttons'][section].items():
                    if val:  # 按下时是 1
                        print('++++++++++++')
                        print('Pressed', btn)
                        
                        pressed.add(btn)

        return pressed


    def run(self):
        while self.running:
            state = self.joycon.get_status()
            pressed_buttons = self.get_pressed_buttons(state=state)

            for btn in pressed_buttons - self.prev_buttons:
                print('--------------compare with prev buttons, pressed', btn)
                self.on_press(btn)

            for btn in self.prev_buttons - pressed_buttons:
                print('--------------compare with prev buttons, released', btn)
                self.on_release(btn)

            self.prev_buttons = pressed_buttons
            time.sleep(0.01)

    def stop(self):
        self.running = False


class NongMobileManipulator:
    """
    NongMobileManipulator is a class for connecting to and controlling Nong robot.
    The robot includes a 4WD differential mobile base and a remote follower arm.
    The leader arm is connected locally (on the laptop) and its joint positions are recorded and then
    forwarded to the remote follower arm (after applying a safety clamp).
    In parallel, JoyCon teleoperation is used to generate raw velocity commands for the wheels. The JoyCon is connected to the laptop via USB.
    """

    def __init__(self, config: NongBotRobotConfig):
        """
        Expected keys in config:
          - ip, port, video_port for the remote connection.
          - calibration_dir, leader_arms, follower_arms, max_relative_target, etc.
        """
        self.robot_type = config.type
        self.config = config
        self.remote_ip = config.ip
        self.remote_port = config.port
        self.remote_port_video = config.video_port
        self.calibration_dir = Path(self.config.calibration_dir)
        self.logs = {}

        self.teleop_keys = self.config.teleop_keys

        # For teleoperation, the leader arm (local) is used to record the desired arm pose.
        self.leader_arms = make_motors_buses_from_configs(self.config.leader_arms)

        self.follower_arms = make_motors_buses_from_configs(self.config.follower_arms)

        self.cameras = make_cameras_from_configs(self.config.cameras)

        self.is_connected = False

        self.last_frames = {}
        self.last_present_speed = torch.zeros(2, dtype=torch.float32)
        self.last_remote_arm_state = torch.zeros(6, dtype=torch.float32)

        # Define three speed levels and a current index
        self.speed_levels = [
            {"x": 150, "steer_angle_speed": 200},  # slow
            {"x": 100, "steer_angle_speed": 100},  # medium
            {"x": 200, "steer_angle_speed": 200},  # fast
        ]
        self.speed_index = 0  # Start at slow

        # ZeroMQ context and sockets.
        self.context = None
        self.cmd_socket = None
        self.video_socket = None

        # Keyboard state for base teleoperation.
        self.running = True
        self.pressed_keys = {
            "forward": False,
            "backward": False,
            "rotate_left": False,
            "rotate_right": False,
        }

        self.listener = JoyConListener(
            on_press=self.on_press,
            on_release=self.on_release
        )

        self.listener.start()

    def get_motor_names(self, arms: dict[str, MotorsBus]) -> list:
        return [f"{arm}_{motor}" for arm, bus in arms.items() for motor in bus.motors]

    @property
    def camera_features(self) -> dict:
        cam_ft = {}
        for cam_key, cam in self.cameras.items():
            key = f"observation.images.{cam_key}"
            cam_ft[key] = {
                "shape": (cam.height, cam.width, cam.channels),
                "names": ["height", "width", "channels"],
                "info": None,
            }
        return cam_ft

    @property
    def motor_features(self) -> dict:
        follower_arm_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]
        observations = ["x_speed", "steer_angle_speed"] #Is steer_angle the abs value or steer speed?
        combined_names = follower_arm_names + observations
        return {
            "action": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (len(combined_names),),
                "names": combined_names,
            },
        }

    @property
    def features(self):
        return {**self.motor_features, **self.camera_features}

    @property
    def has_camera(self):
        return len(self.cameras) > 0

    @property
    def num_cameras(self):
        return len(self.cameras)

    @property
    def available_arms(self):
        available = []
        for name in self.leader_arms:
            available.append(get_arm_id(name, "leader"))
        for name in self.follower_arms:
            available.append(get_arm_id(name, "follower"))
        return available

    def on_press(self, key):
        try:
            # Movement
            # JoyCon
            if key == self.teleop_keys["forward"]:
                self.pressed_keys["forward"] = True
            elif key == self.teleop_keys["backward"]:
                self.pressed_keys["backward"] = True
            elif key == self.teleop_keys["rotate_left"]:
                self.pressed_keys["rotate_left"] = True
            elif key == self.teleop_keys["rotate_right"]:
                self.pressed_keys["rotate_right"] = True
            # Quit teleoperation
            elif key == self.teleop_keys["quit"]:
                self.running = False
                return False
            # Speed control
            # elif key.char == self.teleop_keys["speed_up"]:
            #     self.speed_index = min(self.speed_index + 1, 2)
            #     print(f"Speed index increased to {self.speed_index}")
            # elif key.char == self.teleop_keys["speed_down"]:
            #     self.speed_index = max(self.speed_index - 1, 0)
            #     print(f"Speed index decreased to {self.speed_index}")

        except AttributeError as e:
            print('AttributeError:', e)
            # e.g., if key is special like Key.esc
            if key == 'plus':
                self.running = False
                return False

    def on_release(self, key):
        try:
            if key == self.teleop_keys["forward"]:
                self.pressed_keys["forward"] = False
            elif key == self.teleop_keys["backward"]:
                self.pressed_keys["backward"] = False
            elif key == self.teleop_keys["rotate_left"]:
                self.pressed_keys["rotate_left"] = False
            elif key == self.teleop_keys["rotate_right"]:
                self.pressed_keys["rotate_right"] = False
            elif key == self.teleop_keys["stop"]:
                self.pressed_keys["stop"] = False
        except AttributeError as e:
            print(e)

    def connect(self):
        if not self.leader_arms:
            raise ValueError("MobileManipulator has no leader arm to connect.")
        for name in self.leader_arms:
            print(f"Connecting {name} leader arm.")
            self.calibrate_leader()

        # Set up ZeroMQ sockets to communicate with the remote mobile robot.
        self.context = zmq.Context()
        self.cmd_socket = self.context.socket(zmq.PUSH)
        connection_string = f"tcp://{self.remote_ip}:{self.remote_port}"
        self.cmd_socket.connect(connection_string)
        self.cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.video_socket = self.context.socket(zmq.PULL)
        video_connection = f"tcp://{self.remote_ip}:{self.remote_port_video}"
        self.video_socket.connect(video_connection)
        self.video_socket.setsockopt(zmq.CONFLATE, 1)
        print(
            f"[INFO] Connected to remote robot at {connection_string} and video stream at {video_connection}."
        )
        self.is_connected = True

    def load_or_run_calibration_(self, name, arm, arm_type):
        arm_id = get_arm_id(name, arm_type)
        arm_calib_path = self.calibration_dir / f"{arm_id}.json"

        if arm_calib_path.exists():
            with open(arm_calib_path) as f:
                calibration = json.load(f)
        else:
            print(f"Missing calibration file '{arm_calib_path}'")
            calibration = run_arm_manual_calibration(arm, self.robot_type, name, arm_type)
            print(f"Calibration is done! Saving calibration file '{arm_calib_path}'")
            arm_calib_path.parent.mkdir(parents=True, exist_ok=True)
            with open(arm_calib_path, "w") as f:
                json.dump(calibration, f)

        return calibration

    def calibrate_leader(self):
        for name, arm in self.leader_arms.items():
            # Connect the bus
            arm.connect()

            # Disable torque on all motors
            for motor_id in arm.motors:
                arm.write("Torque_Enable", TorqueMode.DISABLED.value, motor_id)

            # Now run calibration
            calibration = self.load_or_run_calibration_(name, arm, "leader")
            arm.set_calibration(calibration)

    def calibrate_follower(self):
        for name, bus in self.follower_arms.items():
            bus.connect()

            # Disable torque on all motors
            for motor_id in bus.motors:
                bus.write("Torque_Enable", 0, motor_id)

            # Then filter out wheels
            arm_only_dict = {k: v for k, v in bus.motors.items() if not k.startswith("wheel_")}
            if not arm_only_dict:
                continue

            original_motors = bus.motors
            bus.motors = arm_only_dict

            calibration = self.load_or_run_calibration_(name, bus, "follower")
            bus.set_calibration(calibration)

            bus.motors = original_motors

    def _get_data(self):
        """
        Polls the video socket for up to 15 ms. If data arrives, decode only
        the *latest* message, returning frames, speed, and arm state. If
        nothing arrives for any field, use the last known values.
        """
        frames = {}
        present_speed = {}
        remote_arm_state_tensor = torch.zeros(6, dtype=torch.float32)

        # Poll up to 15 ms
        poller = zmq.Poller()
        poller.register(self.video_socket, zmq.POLLIN)
        socks = dict(poller.poll(15))
        if self.video_socket not in socks or socks[self.video_socket] != zmq.POLLIN:
            # No new data arrived → reuse ALL old data
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        # Drain all messages, keep only the last
        last_msg = None
        while True:
            try:
                obs_string = self.video_socket.recv_string(zmq.NOBLOCK)
                last_msg = obs_string
            except zmq.Again:
                break

        if not last_msg:
            # No new message → also reuse old
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        # Decode only the final message
        try:
            observation = json.loads(last_msg)
            images_dict = observation.get("images", {})
            new_speed = observation.get("raw_velocity", None)
            new_arm_state = observation.get("follower_arm_state", None)

            # Convert images
            for cam_name, image_b64 in images_dict.items():
                if image_b64:
                    jpg_data = base64.b64decode(image_b64)
                    np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
                    frame_candidate = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    if frame_candidate is not None:
                        frames[cam_name] = frame_candidate

            # If remote_arm_state is None and frames is None there is no message then use the previous message
            if new_arm_state is not None and frames is not None:
                self.last_frames = frames

                remote_arm_state_tensor = torch.tensor(new_arm_state, dtype=torch.float32)
                self.last_remote_arm_state = remote_arm_state_tensor

                present_speed = torch.tensor(new_speed, dtype=torch.float32)
                self.last_present_speed = present_speed
            else:
                frames = self.last_frames

                remote_arm_state_tensor = self.last_remote_arm_state

                present_speed = self.last_present_speed

        except Exception as e:
            print(f"[DEBUG] Error decoding video message: {e}")
            # If decode fails, fall back to old data
            return (self.last_frames, self.last_present_speed, self.last_remote_arm_state)

        return frames, present_speed, remote_arm_state_tensor

    # def _process_present_speed(self, present_speed: dict) -> torch.Tensor:
    #     state_tensor = torch.zeros(3, dtype=torch.int32)
    #     if present_speed:
    #         decoded = {key: MobileManipulator.raw_to_degps(value) for key, value in present_speed.items()}
    #         if "1" in decoded:
    #             state_tensor[0] = decoded["1"]
    #         if "2" in decoded:
    #             state_tensor[1] = decoded["2"]
    #         if "3" in decoded:
    #             state_tensor[2] = decoded["3"]
    #     return state_tensor

    def teleop_step(
        self, record_data: bool = False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("MobileManipulator is not connected. Run `connect()` first.")

        speed_setting = self.speed_levels[self.speed_index]
        x_speed = speed_setting["x"]  # e.g. 0.1, 0.25, or 0.4
        steer_angle_speed = speed_setting["steer_angle_speed"]  # e.g. 30, 60, or 90

        # Prepare to assign the position of the leader to the follower
        arm_positions = []
        for name in self.leader_arms:
            pos = self.leader_arms[name].read("Present_Position")
            pos_tensor = torch.from_numpy(pos).float()
            arm_positions.extend(pos_tensor.tolist())

        x_cmd = 0.0  # m/s forward/backward
        steer_angle_cmd = 0.0  # deg/s rotation
        if self.pressed_keys["forward"]:
            x_cmd += x_speed
        if self.pressed_keys["backward"]:
            x_cmd -= x_speed
        if self.pressed_keys["rotate_left"]:
            steer_angle_cmd += steer_angle_speed
        if self.pressed_keys["rotate_right"]:
            steer_angle_cmd -= steer_angle_speed

        message = {"raw_velocity": {'x_speed': x_cmd, 'steer_angle_speed':steer_angle_cmd}, "arm_positions": arm_positions}
        self.cmd_socket.send_string(json.dumps(message))

        if not record_data:
            return

        obs_dict = self.capture_observation()

        arm_state_tensor = torch.tensor(arm_positions, dtype=torch.float32)

        wheel_tensor = torch.tensor([x_cmd, steer_angle_speed], dtype=torch.float32)
        action_tensor = torch.cat([arm_state_tensor, wheel_tensor])
        action_dict = {"action": action_tensor}

        return obs_dict, action_dict

    def capture_observation(self) -> dict:
        """
        Capture observations from the remote robot: current follower arm positions,
        present wheel speeds (converted to body-frame velocities: x, y, theta),
        and a camera frame.
        """
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

        frames, present_speed, remote_arm_state_tensor = self._get_data()

        combined_state_tensor = torch.cat((remote_arm_state_tensor, present_speed), dim=0)

        obs_dict = {"observation.state": combined_state_tensor}

        # Loop over each configured camera
        for cam_name, cam in self.cameras.items():
            frame = frames.get(cam_name, None)
            if frame is None:
                # Create a black image using the camera's configured width, height, and channels
                frame = np.zeros((cam.height, cam.width, cam.channels), dtype=np.uint8)
            obs_dict[f"observation.images.{cam_name}"] = torch.from_numpy(frame)

        return obs_dict

    def send_action(self, action: torch.Tensor) -> torch.Tensor:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected. Run `connect()` first.")

        # Ensure the action tensor has at least 9 elements:
        #   - First 6: arm positions.
        #   - Last 2: base commands.
        if action.numel() < 8:
            # Pad with zeros if there are not enough elements.
            padded = torch.zeros(8, dtype=action.dtype)
            padded[: action.numel()] = action
            action = padded

        # Extract arm and base actions.
        arm_actions = action[:6].flatten()
        base_actions = action[6:].flatten()

        x_cmd_mm = base_actions[0].item()  # mm/s
        steer_angle_speed = base_actions[1].item()  # deg/s

        # Compute wheel commands from body commands.
        wheel_commands = [x_cmd_mm, steer_angle_speed]

        arm_positions_list = arm_actions.tolist()

        message = {"raw_velocity": wheel_commands, "arm_positions": arm_positions_list}
        self.cmd_socket.send_string(json.dumps(message))

        return action

    def print_logs(self):
        pass

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError("Not connected.")
        if self.cmd_socket:
            stop_cmd = {
                "raw_velocity": {"x_speed": 0, "steer_angle_speed": 0},
                "arm_positions": {},
            }
            self.cmd_socket.send_string(json.dumps(stop_cmd))
            self.cmd_socket.close()
        if self.video_socket:
            self.video_socket.close()
        if self.context:
            self.context.term()
        self.listener.stop()
        self.is_connected = False
        print("[INFO] Disconnected from remote robot.")

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
            self.listener.stop()
