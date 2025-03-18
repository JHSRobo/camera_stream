import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import socket 

# w W
class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        self.log = self.get_logger()
        
        self.vid_capture = cv2.VideoCapture(0)
        if not vid_capture.isOpened():
            self.log.info("Camera Not Found")
            exit()

        # Used the ip at the end of the topic name because it seemed like the most obvious identifier
        # for the camera that was guranteed to be unique.
        self.ip = self.get_ip()
        self.cam_name = "camera" + self.ip.split(".")[3]
        self.feed_publisher = self.create_publisher(Image, self.cam_name, 10)
        framerate = 1.0 / 50.0
        self.create_timer(framerate, self.pub_feed)

    def pub_feed(self):
        success, frame = self.vid_capture.read()
        if success:
            img = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            self.feed_publisher.publish(img)


    def get_ip(self):
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        return ip

def main(args=None):
    rclpy.init(args=args)
    streamer_node = CameraStreamer()
    rclpy.spin(streamer_node)
    streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
