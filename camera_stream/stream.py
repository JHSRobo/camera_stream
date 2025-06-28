import rclpy 
from rclpy.node import Node 
import glob 
import cv2 
import time

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_stream')

        self.log = self.get_logger()

        self.cameras = {}

        self.find_cameras()
        self.log.info(str(self.cameras))

    def find_cameras(self):
        for path in glob.glob("/dev/video*"):
            i = int(path.strip("/dev/video/"))
            try:
                video = cv2.VideoCapture(i, cv2.CAP_V4L2) 
                time.sleep(0.1)

                if video is None or not video.isOpened():
                    continue 
                ret, frame = video.read()
                video.release()

                if ret:
                    self.cameras[i] = path

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamerNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
