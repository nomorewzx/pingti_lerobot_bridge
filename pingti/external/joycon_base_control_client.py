from threading import Thread
import time

# 假设你用这个库读取 Joy-Con 输入
from pyjoycon import JoyCon, get_R_id  # pip install pyjoycon

def on_press(key):
    print(f"Joy-Con key pressed: {key}")

def on_release(key):
    print(f"Joy-Con key released: {key}")

# 启动 Joy-Con 监听器
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
                        pressed.add(btn)

        return pressed


    def run(self):
        while self.running:
            state = self.joycon.get_status()
            pressed_buttons = self.get_pressed_buttons(state=state)

            # 检测按下事件
            for btn in pressed_buttons - self.prev_buttons:
                self.on_press(btn)

            # 检测释放事件
            for btn in self.prev_buttons - pressed_buttons:
                self.on_release(btn)

            self.prev_buttons = pressed_buttons
            time.sleep(0.05)  # 控制监听频率

    def stop(self):
        self.running = False

# 启动监听器
listener = JoyConListener(on_press=on_press, on_release=on_release)
listener.start()

# 程序运行期间做其他事…
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    listener.stop()
    listener.join()
