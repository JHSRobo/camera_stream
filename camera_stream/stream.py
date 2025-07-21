import rclpy 
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
import socket 
import toml
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
        with open("/home/jhsrobo/corews/src/camera_stream/settings.toml", "r") as f:
            self.settings = toml.load(f)
            self.log.info(f"Loaded Camera Settings: {self.settings}")

        # Parameter to determine whether to save new camera settings to settings.toml at the end of the program
        self.save_changes_on_shutdown = False 
        self.declare_parameter("save_changes_on_shutdown", self.save_changes_on_shutdown)

        # Camera Settings Parameters
        self.add_bounded_parameter("brightness", self.settings["brightness"], -64, 64, 1)
        self.add_bounded_parameter("contrast", self.settings["contrast"], 0, 95, 1)
        self.add_bounded_parameter("saturation", self.settings["saturation"], 0, 255, 1)
        self.add_bounded_parameter("hue", self.settings["hue"], -2000, 2000, 1)
        self.add_bounded_parameter("gamma", self.settings["gamma"], 64, 300, 1)
        self.add_bounded_parameter("gain", self.settings["gain"], 0, 255, 1)
        self.add_bounded_parameter("sharpness", self.settings["sharpness"], 0, 7, 1)
        self.add_bounded_parameter("backlight_compensation", self.settings["backlight_compensation"], 0, 100, 1)


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
        self.settings[name] = cur_val
        self.declare_parameter(name, self.settings[name], descriptor)


    # params is a list of param objects that contain a name, value, and data type
    def update_parameters(self, params):
        for param in params:
            if param.name == "save_changes_on_shutdown":
                self.save_changes_on_shutdown = param.value 
            else:
                self.settings[param.name] = param.value 
                for dev in self.cameras:
                    cmd = ["v4l2-ctl", f"--device={dev}", "--set-ctrl", f"{param.name}={self.settings[param.name]}"] 
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
                self.log.info(f"Streaming camera at device {dev} to port {self.port}")

                self.port += 1

    def write_to_config(self):
        if self.save_changes_on_shutdown:
            with open("settings.toml", "w") as f:
                toml.dump(self.settings, f)

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamerNode()
    try: rclpy.spin(camera_stream_node)
    except KeyboardInterrupt: camera_stream_node.write_to_config()
    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
