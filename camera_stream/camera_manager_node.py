import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import yaml
from pathlib import Path

from lib.camera import Camera
from lib.data_classes import StreamConfig, CameraSettings

class CameraManagerNode(Node):
    def __init__(self):
        self.log = self.get_logger()

        self.config_dir = Path(get_package_share_directory("camera_stream")) / "config"

        # Create service for restarting streams
        self.restart_stream_srv = self.create_srv

        # Create client for changing streaming mode

        # Create camera parameters
        self.brightness = self.create_parameter()
        self.contrast = self.create_parameter()

    def is_active(camera):
        path = Path()
        if path.exists()

    def init_cams(self):
        pass

    def load_config(self):
        pass


def main():
    pass

if __name__ == "__main__":
    main()

