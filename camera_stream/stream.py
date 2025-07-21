import rclpy 
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
import socket 
import subprocess

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_stream')

        self.log = self.get_logger()

        # Some code to quickly grab the current RPi's ip address
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        ip = "192.168.88.111"

        # Camera Settings as Parameters 
        self.camera_params = {}

        self.add_bounded_parameter("brightness", 20, -64, 64, 1)
        self.add_bounded_parameter("contrast", 0, 0, 95, 1)
        self.add_bounded_parameter("saturation", 70, 0, 255, 1)
        self.add_bounded_parameter("hue", 0, -2000, 2000, 1)
        self.add_bounded_parameter("gamma", 110, 64, 300, 1)
        self.add_bounded_parameter("gain", 110, 0, 255, 1)
        self.add_bounded_parameter("sharpness", 2, 0, 7, 1)
        self.add_bounded_parameter("backlight_compensation", 80, 0, 100, 1)


        # Camera Streaming Command
        self.ustreamer_cmd = ["ustreamer", "--host=" + ip, "--format=MJPEG", "--encoder=HW", "--resolution=1920x1080", "--desired-fps=60", "--buffers=2"]

        # Camera streaming port (which is added later on to the ustreamer command) starts at 5000 and increments by 1
        self.port = 5000

        # Look through the available usb devices and start http streams for new camera devices 
        self.cameras = []
        self.find_cameras()

        self.add_on_set_parameters_callback(self.update_parameters)


    def add_bounded_parameter(self, name, cur_val, from_val, to_val, step):
        bounds = IntegerRange()
        bounds.from_value = from_val 
        bounds.to_value = to_val 
        bounds.step = step 
        descriptor = ParameterDescriptor(integer_range = [bounds])
        self.camera_params[name] = cur_val
        self.declare_parameter(name, self.camera_params[name], descriptor)


    def update_parameters(self, params):
        for param in params:
            self.camera_params[param.name] = param.value 
            for dev in self.cameras:
                cmd = ["v4l2-ctl", f"--device={dev}", "--set-ctrl", f"{param.name}={self.camera_params[param.name]}"] 
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        return SetParametersResult(successful=True)


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
                self.cameras.append(dev)

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
