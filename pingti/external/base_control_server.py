"""
Base control server for PingTi mobile base, structured like lekiwi_pingti_remote.py
"""

import time
import json
import threading
import serial
import zmq
import cv2
from pathlib import Path
from lerobot.common.robot_devices.cameras.utils import make_cameras_from_configs
from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig


def xor_check(data):
    """Calculate XOR checksum for serial command"""
    xor = data[0]
    for b in data[1:]:
        xor ^= b
    return xor


def build_control_command(x_speed, steer_angle):
    """Build serial control command for the base"""
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


def send_command(ser, x_speed, steer_angle):
    """Send control command to serial port"""
    cmd = build_control_command(x_speed, steer_angle)
    ser.write(cmd)
    print(f"[Serial] Sent: {' '.join(hex(b) for b in cmd)}")


def setup_zmq_sockets():
    """Set up ZeroMQ sockets for command and video streaming"""
    context = zmq.Context()
    
    # Command socket (REP - receives commands and sends acknowledgments)
    cmd_socket = context.socket(zmq.REP)
    cmd_socket.bind("tcp://*:5555")
    cmd_socket.setsockopt(zmq.RCVTIMEO, 10)  # 10ms timeout
    
    # Video socket (PUB - publishes observations)
    video_socket = context.socket(zmq.PUB)
    video_socket.bind("tcp://*:5556")
    
    return context, cmd_socket, video_socket


def run_camera_capture(cameras, images_lock, latest_images_dict, stop_event):
    """Capture camera frames in a separate thread"""
    while not stop_event.is_set():
        try:
            for name, camera in cameras.items():
                frame = camera.read()
                with images_lock:
                    # Convert frame to list for JSON serialization
                    if frame is not None:
                        # Resize frame to reduce bandwidth
                        frame_resized = cv2.resize(frame, (320, 240))
                        latest_images_dict[name] = frame_resized.tolist()
        except Exception as e:
            print(f"[ERROR] Camera capture failed: {e}")
        
        time.sleep(0.033)  # ~30 FPS


class BaseController:
    """Controller for the mobile base"""
    
    def __init__(self, serial_port="/dev/ttyUSB0", baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None
        self.current_x_speed = 0
        self.current_steer_angle = 0
        
    def connect(self):
        """Connect to the serial port"""
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        print(f"✅ 串口已连接 {self.serial_port}")
        
    def set_velocity(self, command_speeds):
        """
        Set velocity based on wheel speeds
        command_speeds: [left_wheel, back_wheel, right_wheel]
        """
        if len(command_speeds) >= 3:
            left_wheel = command_speeds[0]
            back_wheel = command_speeds[1] 
            right_wheel = command_speeds[2]
            
            # Convert wheel speeds to x_speed and steer_angle
            # This is a simplified conversion - adjust based on your robot's kinematics
            x_speed = int((left_wheel + right_wheel) / 2)
            steer_angle = int((right_wheel - left_wheel) / 2)
            
            self.current_x_speed = x_speed
            self.current_steer_angle = steer_angle
            
            send_command(self.ser, x_speed, steer_angle)
    
    def stop(self):
        """Stop the base"""
        self.current_x_speed = 0
        self.current_steer_angle = 0
        send_command(self.ser, 0, 0)
    
    def read_velocity(self):
        """Read current velocity (simplified - returns last set values)"""
        return [self.current_x_speed, 0, self.current_steer_angle]


def run_base_control():
    """
    Main function that runs the base control server:
      - Sets up cameras and connects them
      - Initializes the base controller
      - Creates ZeroMQ sockets for receiving commands and streaming observations
      - Processes incoming commands and sends back sensor and camera data
    """
    
    # Initialize cameras
    camera_configs = {
        "front": OpenCVCameraConfig(
            camera_index="/dev/video1", fps=30, width=640, height=480, rotation=180
        ),
        "wrist": OpenCVCameraConfig(
            camera_index="/dev/video0", fps=30, width=640, height=480, rotation=90
        ),
    }
    
    cameras = make_cameras_from_configs(camera_configs)
    for cam in cameras.values():
        try:
            cam.connect()
            print(f"✅ Camera connected: {cam}")
        except Exception as e:
            print(f"❌ Camera connection failed: {e}")
    
    # Initialize base controller
    base_controller = BaseController()
    base_controller.connect()
    
    # Set up ZeroMQ sockets
    context, cmd_socket, video_socket = setup_zmq_sockets()
    
    # Start camera capture thread
    latest_images_dict = {}
    images_lock = threading.Lock()
    stop_event = threading.Event()
    cam_thread = threading.Thread(
        target=run_camera_capture, 
        args=(cameras, images_lock, latest_images_dict, stop_event), 
        daemon=True
    )
    cam_thread.start()
    
    last_cmd_time = time.time()
    print("Base control server started. Waiting for commands...")
    
    try:
        while True:
            loop_start_time = time.time()
            
            # Process incoming commands (non-blocking with timeout)
            try:
                msg = cmd_socket.recv_string()
                try:
                    data = json.loads(msg)
                    
                    # Process wheel velocity commands
                    if "raw_velocity" in data:
                        raw_command = data["raw_velocity"]
                        # Expect keys: "left_wheel", "back_wheel", "right_wheel"
                        command_speeds = [
                            int(raw_command.get("left_wheel", 0)),
                            int(raw_command.get("back_wheel", 0)),
                            int(raw_command.get("right_wheel", 0)),
                        ]
                        base_controller.set_velocity(command_speeds)
                        last_cmd_time = time.time()
                    
                    # Process simple x/steer commands (backward compatibility)
                    elif "x" in data or "steer" in data:
                        x = data.get("x", 0)
                        steer = data.get("steer", 0)
                        send_command(base_controller.ser, x, steer)
                        last_cmd_time = time.time()
                    
                    # Send acknowledgment
                    cmd_socket.send_string("OK")
                    
                except json.JSONDecodeError as e:
                    print(f"[ERROR] JSON parsing failed: {e}")
                    cmd_socket.send_string("ERROR")
                    
            except zmq.Again:
                # No message received within timeout
                pass
            except Exception as e:
                print(f"[ERROR] Command processing failed: {e}")
                try:
                    cmd_socket.send_string("ERROR")
                except:
                    pass
            
            # Watchdog: stop the robot if no command is received for over 0.5 seconds
            now = time.time()
            if now - last_cmd_time > 0.5:
                base_controller.stop()
                last_cmd_time = now
            
            # Read current base status
            current_velocity = base_controller.read_velocity()
            
            # Get latest camera images
            with images_lock:
                images_dict_copy = dict(latest_images_dict)
            
            # Build observation dictionary
            observation = {
                "images": images_dict_copy,
                "present_speed": current_velocity,
                "base_state": {
                    "x_speed": base_controller.current_x_speed,
                    "steer_angle": base_controller.current_steer_angle
                }
            }
            
            # Send observation over video socket
            try:
                video_socket.send_string(json.dumps(observation), zmq.NOBLOCK)
            except zmq.Again:
                # Skip if no subscribers
                pass
            
            # Ensure consistent loop timing
            elapsed = time.time() - loop_start_time
            time.sleep(max(0.033 - elapsed, 0))  # ~30 FPS
            
    except KeyboardInterrupt:
        print("Shutting down base control server.")
    finally:
        stop_event.set()
        cam_thread.join()
        base_controller.stop()
        if base_controller.ser:
            base_controller.ser.close()
        cmd_socket.close()
        video_socket.close()
        context.term()


if __name__ == "__main__":
    run_base_control()
