import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import pyudev
import yaml

class CameraManagerNode(Node):
    def __init__(self):
        self.log = self.get_logger()

        self.load_config()
        self.get_devices()


        for stream in self.camera_streams:
            stream.start()

        # Create service for restarting streams
        self.restart_stream_srv = self.create_srv

        # Create client for changing streaming mode

        # Create camera parameters
        self.brightness = self.create_parameter()
        self.contrast = self.create_parameter()

    def load_config(self):

        self.cam_config = yaml.load(cam_config_path)

    def get_devices(self):
        context = pyudev.Context()
        for device in context.list_devices(subsystem="video4linux"):
            print(device.device_node)
            print(device.properties)

        if True:
            self.log.info(f"Camera {camera} not connected.")

        # Helpful tools: monitor the bitrate of each USB device, CPU usage, RAM usage, ethernet bitrate, latency?
