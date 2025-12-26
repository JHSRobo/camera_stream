import rclpy 
from rclpy.node import Node 
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from std_msgs.msg import Int32
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

        # Load saved camera settings from settings.toml
        self.config_path = "/home/jhsrobo/corews/src/camera_stream/settings.toml"
        with open(self.config_path, "r") as f:
            self.settings = toml.load(f)
            self.log.info(f"Loaded Camera Settings: {self.settings}")

        # Parameter to determine whether to save new camera settings to settings.toml at the end of program execution
        self.save_changes_on_shutdown = False 
        self.declare_parameter("save_changes_on_shutdown", self.save_changes_on_shutdown)

        # Make ros parameters for each camera setting
        self.add_bounded_parameter("brightness", self.settings["brightness"], -64, 64, 1)
        self.add_bounded_parameter("contrast", self.settings["contrast"], 0, 95, 1)
        self.add_bounded_parameter("saturation", self.settings["saturation"], 0, 255, 1)
        self.add_bounded_parameter("hue", self.settings["hue"], -2000, 2000, 1)
        self.add_bounded_parameter("gamma", self.settings["gamma"], 64, 300, 1)
        self.add_bounded_parameter("gain", self.settings["gain"], 0, 255, 1)
        self.add_bounded_parameter("sharpness", self.settings["sharpness"], 0, 7, 1)
        self.add_bounded_parameter("backlight_compensation", self.settings["backlight_compensation"], 0, 100, 1)


        # Camera Streaming Command
        self.ustreamer_cmd = ["ustreamer", "--host=" + ip, "--format=MJPEG", "--encoder=HW", "--resolution=1920x1080", "--desired-fps=30", "--buffers=4", "--workers=4"]
        self.log.info(str(self.ustreamer_cmd))

        # Camera streaming port (which is added later on to the ustreamer command) starts at 5000 and increments by 1
        self.port = 5000
        self.stereo_port = 1111

        # Look through the available usb devices and start http streams for new camera devices 
        self.cameras = []
        self.find_cameras()

        self.camera_count_publisher = self.create_publisher(Int32, 'camera_count', 10)

        self.create_timer(1, self.send_camera_count)

        # This callback is only ran when a parameter is changed
        self.add_on_set_parameters_callback(self.update_parameters)

    def send_camera_count(self):
        msg = Int32()
        msg.data = len(self.cameras)
        self.camera_count_publisher.publish(msg)

    # Function to quickly declare a camera setting as a ros parameter and store the setting in self.settings
    def add_bounded_parameter(self, name, cur_val, from_val, to_val, step):
        self.settings[name] = cur_val

        bounds = IntegerRange()

        bounds.from_value = from_val 
        bounds.to_value = to_val 
        bounds.step = step 

        descriptor = ParameterDescriptor(integer_range = [bounds])
        self.declare_parameter(name, self.settings[name], descriptor)

    # Interface with a given camera to change a single setting
    def set_setting(self, dev, name):
        cmd = ["v4l2-ctl", f"--device={dev}", "--set-ctrl", f"{name}={self.settings[name]}"] 
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Interface with a given camera to change all settings
    def set_all_settings(self, dev):
        cmd = ["v4l2-ctl", f"--device={dev}", "--set-ctrl"] 

        # settings_changes is a string that contains all of the settings changes separated by commas
        settings_changes = ""
        for key in self.settings.keys():
            settings_changes += f"{key}={self.settings[key]},"

        settings_changes = settings_changes[:-1] # remove the last unnecesary comma

        cmd.append(settings_changes)
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # params is a list of param objects that contain a name, value, and data type
    # params only contains param objects of parameters that have been changed
    def update_parameters(self, params):
        for param in params:
            if param.name == "save_changes_on_shutdown":
                self.save_changes_on_shutdown = param.value 
            else:
                self.settings[param.name] = param.value 
                for dev in self.cameras:
                    self.set_setting(dev, param.name)

        return SetParametersResult(successful=True)


    # This code is kind of jank but I couldn't come up with anything better. If you do, change it.
    # The v4l2-ctl --list-devices command lists out all camera devices followed by a list of their corresponding /dev files.
    def find_cameras(self):
        # Runs v4l2-ctl --list-devices and stores the output in the variable output 
        cmd = ["v4l2-ctl", "--list-devices"]
        output = subprocess.check_output(cmd, text=True)

        # Splits the output into a list and remove the tab characters 
        output = list(output.split("\n"))
        output = list(line.strip("\t") for line in output)

        # Reads through the lines of output. When a usb device is a camera, read the next line for that camera's /dev/video file 
        for i, line in enumerate(output):
            line = line.lower()
            if "cam" in line:
                if "3d" in line:
                    self.log.info("check")
                    self.add_camera(output[i+1], True)
                else:
                    self.add_camera(output[i+1], False)

        # Configures all cameras with saved settings after initializing the camera feeds
        for dev in self.cameras:
            self.set_all_settings(dev)

    # Save new settings to the toml file is selected
    def write_to_config(self):
        if self.save_changes_on_shutdown:
            with open(self.config_path, "w") as f:
                toml.dump(self.settings, f)

    def add_camera(self, dev, stereo):
        device_flag = "--device=" + dev 
        if stereo:
            port_flag = "--port=" + str(self.stereo_port) 
            # Create a new process that streams this cameras and disables the logging of that process 
            # If you're having issues, remove the stdout and stderr flags
            subprocess.Popen([*self.ustreamer_cmd, device_flag, port_flag])
            self.log.info(f"Streaming stereographic camera {self.stereo_port - 1000} at device {dev} to port {self.stereo_port}")

            self.stereo_port += 1
        else:
            port_flag = "--port=" + str(self.port) 

            # Create a new process that streams this cameras and disables the logging of that process 
            # If you're having issues, remove the stdout and stderr flags
            subprocess.Popen([*self.ustreamer_cmd, device_flag, port_flag])
            self.log.info(f"Streaming camera {self.port - 4999} at device {dev} to port {self.port}")

            self.port += 1 

def main(args=None):
    rclpy.init(args=args)
    camera_stream_node = CameraStreamerNode()

    try: rclpy.spin(camera_stream_node)
    except KeyboardInterrupt: camera_stream_node.write_to_config() # save changes to the toml on shutdown

    camera_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
