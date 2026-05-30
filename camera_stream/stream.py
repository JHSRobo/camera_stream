import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
import socket
import toml
import subprocess

class CameraStreamerNode(Node):
    def __init__(self):
        super().__init__('camera_stream')
        self.log = self.get_logger()

        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()

        self.config_path = "/home/jhsrobo/corews/src/camera_stream/settings.toml"
        with open(self.config_path, "r") as f:
            self.settings = toml.load(f)
            self.log.info(f"Loaded Camera Settings: {self.settings}")

        self.save_changes_on_shutdown = False
        self.declare_parameter("save_changes_on_shutdown", self.save_changes_on_shutdown)

        self.add_bounded_parameter("brightness", self.settings["brightness"], -64, 64, 1)
        self.add_bounded_parameter("contrast", self.settings["contrast"], 0, 95, 1)
        self.add_bounded_parameter("saturation", self.settings["saturation"], 0, 255, 1)
        self.add_bounded_parameter("hue", self.settings["hue"], -2000, 2000, 1)
        self.add_bounded_parameter("gamma", self.settings["gamma"], 64, 300, 1)
        self.add_bounded_parameter("gain", self.settings["gain"], 0, 255, 1)
        self.add_bounded_parameter("sharpness", self.settings["sharpness"], 0, 7, 1)
        self.add_bounded_parameter("backlight_compensation", self.settings["backlight_compensation"], 0, 100, 1)

        self.ustreamer_cmd = [
            "ustreamer",
            "--host=" + ip,
            "--format=MJPEG",
            "--encoder=HW",
            "--resolution=1920x1080",
            "--desired-fps=60",
            "--buffers=4",
            "--workers=4",
            "--port=5000",
        ]

        self.cameras = []
        self.active_index = 0
        self.active_process = None
        self.cached_button_input = [0, 0, 0, 0, 0]

        self.find_cameras()

        # Reorder cameras according to camera_order in settings.toml
        if "camera_order" in self.settings:
            order = self.settings["camera_order"]
            try:
                self.cameras = [self.cameras[i - 1] for i in order]
                self.log.info(f"Camera order remapped to: {self.cameras}")
            except IndexError:
                self.log.warn("camera_order contains an out of range index, using default order")
        else:
            self.log.info("No camera_order found, using default order")

        if self.cameras:
            self.start_stream(0)

        self.camera_count_publisher = self.create_publisher(Int32, 'camera_count', 10)
        self.create_timer(1, self.send_camera_count)

        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.add_on_set_parameters_callback(self.update_parameters)

    # ── Joystick ─────────────────────────────────────────────────────────────

    def joy_callback(self, joy: Joy):
        b = joy.buttons
        c = self.cached_button_input

        if len(b) <= 8:
            return

        desired = None
        change = False

        if b[4] or b[5] or b[6] or b[7] or b[8]:
            if b[4]:
                if b[5] and not c[1]:
                    desired = "2"; change = True
                if b[6] and not c[2]:
                    desired = "3"; change = True
                if b[7] and not c[3]:
                    desired = "4"; change = True
                if b[8] and not c[4]:
                    desired = "5"; change = True
                if not c[0]:
                    desired = "1"; change = True
            else:
                if b[5] and not c[1]:
                    desired = "2"; change = True
                if b[6] and not c[2]:
                    desired = "3"; change = True
                if b[7] and not c[3]:
                    desired = "4"; change = True
                if b[8] and not c[4]:
                    desired = "5"; change = True

        if change and desired is not None:
            idx = int(desired) - 1
            if idx < len(self.cameras):
                if idx != self.active_index:
                    self.active_index = idx
                    self.stop_stream()
                    self.start_stream(idx)
            else:
                self.log.warn(f"No camera mapped to button (requested cam {desired}, only {len(self.cameras)} available)")

        self.cached_button_input = [b[4], b[5], b[6], b[7], b[8]]

    # ── Stream management ────────────────────────────────────────────────────

    def start_stream(self, index: int):
        dev = self.cameras[index]
        self.set_all_settings(dev)
        cmd = [*self.ustreamer_cmd, f"--device={dev}"]
        self.active_process = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.log.info(f"Streaming camera {index + 1} ({dev}) on port 5000")

    def stop_stream(self):
        if self.active_process and self.active_process.poll() is None:
            self.active_process.terminate()
            try:
                self.active_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.active_process.kill()
        self.active_process = None

    def send_camera_count(self):
        msg = Int32()
        msg.data = len(self.cameras)
        self.camera_count_publisher.publish(msg)

    # ── Camera settings ──────────────────────────────────────────────────────

    def add_bounded_parameter(self, name, cur_val, from_val, to_val, step):
        self.settings[name] = cur_val
        bounds = IntegerRange()
        bounds.from_value = from_val
        bounds.to_value = to_val
        bounds.step = step
        self.declare_parameter(name, self.settings[name], ParameterDescriptor(integer_range=[bounds]))

    def set_setting(self, dev, name):
        subprocess.run(
            ["v4l2-ctl", f"--device={dev}", "--set-ctrl", f"{name}={self.settings[name]}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def set_all_settings(self, dev):
        settings_changes = ",".join(f"{k}={v}" for k, v in self.settings.items())
        cmd = ["v4l2-ctl", f"--device={dev}", "--set-ctrl", settings_changes]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def update_parameters(self, params):
        for param in params:
            if param.name == "save_changes_on_shutdown":
                self.save_changes_on_shutdown = param.value
            else:
                self.settings[param.name] = param.value
                if self.cameras:
                    self.set_setting(self.cameras[self.active_index], param.name)
        return SetParametersResult(successful=True)

    # ── Camera discovery ─────────────────────────────────────────────────────

    def find_cameras(self):
        output = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True)
        lines = [line.strip("\t") for line in output.split("\n")]
        for i, line in enumerate(lines):
            if "camera" in line.lower():
                dev = lines[i + 1]
                self.cameras.append(dev)
                self.log.info(f"Found camera at {dev}")

    def write_to_config(self):
        if self.save_changes_on_shutdown:
            with open(self.config_path, "w") as f:
                toml.dump(self.settings, f)


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_to_config()
    finally:
        node.stop_stream()
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
