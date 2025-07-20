import rclpy 
from rclpy.node import Node 
import socket 
import subprocess

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_stream')

        self.log = self.get_logger()

        # Some code to quickly grab the current RPi's ip address
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        # ip = "192.168.88.111"

        # Camera Streaming Command
        self.ustreamer_cmd = ["ustreamer", "--host=" + ip, "--format=MJPEG", "--encoder=HW", "--resolution=1920x1080", "--desired-fps=60", "--buffers=2"]

        # Camera Streaming Port starts at 5000 and increments by 1
        self.port = 5000

        # Look through the available usb devices and start http streams for new camera devices 
        self.find_cameras()


    # This code is kind of jank but I couldn't come up with anything better. If you do, change it.
    # The v4l2-ctl --list-devices command lists out all camera devices followed by a list of their corresponding /dev files.
    def find_cameras(self):
        # Runs v4l2-ctl --list-devices and stores the output in the variable output 
        cmd = ["v4l2-ctl", "--list-devices"]
        output = subprocess.check_output(cmd, text=True)

        # Split the output into a list and remove the tab characters 
        output = list(output.split("\n"))
        output = list(line.strip("\t") for line in output)

        # Read through these lines. When a usb device is a camera, read the next line for that camera's /dev/video file 
        for i, line in enumerate(output):
            if "camera" in line.lower():
                dev = output[i+1] 

                device_flag = "--device=" + dev
                port_flag = "--port=" + str(self.port) 

                # Create a new process that streams this cameras and disables the logging of that process 
                # If you're having issues, remove the stdout and stderr flags
                subprocess.Popen([*self.ustreamer_cmd, device_flag, port_flag], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                self.port += 1

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamerNode()
    rclpy.spin(camera_stream_node)
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
