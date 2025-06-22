"""
Run on macos
"""
from pyjoycon import JoyCon, get_R_id
import zmq
import time

# 获取右手 JoyCon
joycon = JoyCon(*get_R_id())

# 初始化 ZeroMQ 客户端（PUB 模式）
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.connect("tcp://192.168.31.246:5555")  # TODO: 替换为你的 Ubuntu IP 地址

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

# 控制映射
command_map = {
    "FORWARD":  (200, 0),
    "BACKWARD": (-200, 0),
    "LEFT":     (0, 200),
    "RIGHT":    (0, -200),
    "STOP":     (0, 0)
}

# 主循环
print("🎮 JoyCon 控制已启动，按 PLUS 键退出")
while True:
    status = joycon.get_status()
    action = parse_right_joycon_buttons(status)

    if action == "QUIT":
        print("退出程序")
        break
    elif action in command_map:
        x, steer = command_map[action]
        sock.send_json({"x": x, "steer": steer})
        print(f"[JoyCon] {action} => x={x}, steer={steer}")

    time.sleep(0.1)
