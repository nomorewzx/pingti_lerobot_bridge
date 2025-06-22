import zmq
import time
from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
from lerobot.common.robot_devices.cameras.configs import OpenCVCameraConfig

def main():
    zmq_port = "5555"
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{zmq_port}")
    print(f"Server streaming on port {zmq_port}...")

    # 初始化你的 OpenCVCamera
    config = OpenCVCameraConfig(
                camera_index="/dev/video3", fps=30, width=640, height=480, rotation=180
            )
    camera = OpenCVCamera(config)
    camera.connect()

    config2 = OpenCVCameraConfig(
                camera_index="/dev/video1", fps=30, width=640, height=480, rotation=180
            )
    camera2 = OpenCVCamera(config2)
    camera2.connect()


    try:
        while True:
            frame = camera.async_read()
            frame2 = camera2.async_read()
            
            _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            socket.send(buffer.tobytes())

            time.sleep(1 / camera.fps)  

    except KeyboardInterrupt:
        print("Stopping server...")
    finally:
        camera.disconnect()
        socket.close()
        context.term()

if __name__ == "__main__":
    import cv2
    main()
