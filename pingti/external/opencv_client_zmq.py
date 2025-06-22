import numpy as np
import zmq
import rerun as rr
import cv2

def main():
    # Rerun 初始化
    rr.init("OpenCVCamera Stream", spawn=True)
    rr.log("world/camera", rr.ViewCoordinates.RIGHT_HAND_Z_UP)

    # ZMQ 配置
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.31.125:5555")  # 替换为服务器IP
    socket.setsockopt(zmq.SUBSCRIBE, b"")

    print("Connected to server, waiting for images...")

    try:
        while True:
            # 接收并解码图像
            buffer = socket.recv()
            img = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            # 转换 BGR→RGB（如果服务器未转换）
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # 发送到 Rerun
            rr.log("world/camera/image", rr.Image(img_rgb))

    except KeyboardInterrupt:
        print("Stopping client...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()