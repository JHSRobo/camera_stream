import rclpy 
from rclpy.node import Node 
import cv2 
import subprocess
from launch import LaunchService
from launch_ros.actions import Node as LaunchNode 
from launch import LaunchDescription 

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_stream')

        self.log = self.get_logger()

        # Find cameras 
        self.cameras = []
        self.find_cameras()
        self.log.info(f"Acquired cameras at indexes {self.cameras}")  

        launch_service = LaunchService()
        launch_service.include_launch_description(self.generate_launch_description())
        launch_service.run()

    # This code is kind of jank but I'm pretty sure it'll work with any usb cams. 
    # The v4l2-ctl --list-devices command lists out all camera devices followed by a list of their corresponding /dev files.
    # The first /dev file following the camera should always be where the camera feed of the preceding device is published to 
    # so the following code just remembers which /dev files come immediately over a device connected over usb (which should always be a camera).
    def find_cameras(self):
        cmd = ["v4l2-ctl", "--list-devices"]
        output = subprocess.check_output(cmd, text=True)
        output = list(output.split("\n"))
        output = list(line.strip("\t") for line in output)

        for i, line in enumerate(output):
            if "usb" in line:
                cam_index = int(output[i+1].strip("/dev/video"))
                self.cameras.append(cam_index)


    def generate_launch_description(self):
        launch_nodes = []
        for camera in self.cameras:
            path = "/dev/video" + str(camera)
            topic_name = "cam" + str(camera)
            launch_nodes.append(
                LaunchNode(
                    package="usb_cam",
                    executable="usb_cam_node_exe",
                    namespace=topic_name,
                    parameters=[{
                        "video_device": path,
                        "image_width": 1920,
                        "image_height": 1080,
                        "framerate": 30.0,
                        "pixel_format": "raw_mjpeg"
                    }]
                )
            )
        return LaunchDescription(launch_nodes)

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamerNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
