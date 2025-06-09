"""
JoyCon client for PingTi base control
Run on macOS - updated to work with the new base control server structure
"""
from pyjoycon import JoyCon, get_R_id
import zmq
import time
import json
import threading

# 获取右手 JoyCon
joycon = JoyCon(*get_R_id())

# 初始化 ZeroMQ 客户端
ctx = zmq.Context()

# Command socket (REQ - sends commands and receives acknowledgments)
cmd_sock = ctx.socket(zmq.REQ)
cmd_sock.connect("tcp://192.168.31.246:5555")  # TODO: 替换为你的 Ubuntu IP 地址
cmd_sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout

# Video socket (SUB - receives observations)
video_sock = ctx.socket(zmq.SUB)
video_sock.connect("tcp://192.168.31.246:5556")  # TODO: 替换为你的 Ubuntu IP 地址
video_sock.setsockopt_string(zmq.SUBSCRIBE, "")
video_sock.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout


def observation_receiver():
    """Receive and process observations from the server"""
    while True:
        try:
            obs_msg = video_sock.recv_string()
            observation = json.loads(obs_msg)
            
            # Process observation data
            if "base_state" in observation:
                base_state = observation["base_state"]
                print(f"[Base State] x_speed={base_state.get('x_speed', 0)}, steer_angle={base_state.get('steer_angle', 0)}")
            
            if "present_speed" in observation:
                speed = observation["present_speed"]
                print(f"[Speed] {speed}")
                
        except zmq.Again:
            # No observation received - continue
            continue
        except Exception as e:
            print(f"[ERROR] Observation processing failed: {e}")
            break
        
        time.sleep(0.1)


# 按键映射表
def parse_right_joycon_buttons(data):
    right = data['buttons']['right']
    shared = data['buttons']['shared']

    if right['a']:
        return 'FORWARD'
    elif right['b']:
        return 'BACKWARD'
    elif right['x']:
        return 'LEFT'
    elif right['y']:
        return 'RIGHT'
    elif right['zr']:
        return 'STOP'
    elif shared['plus']:
        return 'QUIT'
    else:
        return None


# 控制映射 - updated to use raw_velocity format
command_map = {
    "FORWARD":  {"raw_velocity": {"left_wheel": 200, "back_wheel": 0, "right_wheel": 200}},
    "BACKWARD": {"raw_velocity": {"left_wheel": -200, "back_wheel": 0, "right_wheel": -200}},
    "LEFT":     {"raw_velocity": {"left_wheel": -100, "back_wheel": 0, "right_wheel": 100}},
    "RIGHT":    {"raw_velocity": {"left_wheel": 100, "back_wheel": 0, "right_wheel": -100}},
    "STOP":     {"raw_velocity": {"left_wheel": 0, "back_wheel": 0, "right_wheel": 0}}
}

# Start observation receiver thread
obs_thread = threading.Thread(target=observation_receiver, daemon=True)
obs_thread.start()

# 主循环
print("🎮 JoyCon 控制已启动，按 PLUS 键退出")
last_action = None
command_pending = False

while True:
    try:
        status = joycon.get_status()
        action = parse_right_joycon_buttons(status)

        if action == "QUIT":
            print("退出程序")
            break
        elif action in command_map and action != last_action:
            # Only send command if it's different from the last one
            if not command_pending:
                command = command_map[action]
                cmd_sock.send_string(json.dumps(command))
                command_pending = True
                print(f"[JoyCon] {action} => {command}")
                last_action = action
        
        # Check for acknowledgment from server
        if command_pending:
            try:
                response = cmd_sock.recv_string()
                command_pending = False
                if response != "OK":
                    print(f"[ERROR] Server response: {response}")
            except zmq.Again:
                # No response yet - continue
                pass

    except Exception as e:
        print(f"[ERROR] JoyCon control failed: {e}")
        break
    
    time.sleep(0.1)

# Cleanup
cmd_sock.close()
video_sock.close()
ctx.term()
