import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from rov_interfaces.srv import RestartStream

from lib.camera import Camera
from lib.data_classes import StreamConfig, CameraSettings
from lib.util import load_streams_config

class CameraManagerNode(Node):
    def __init__(self):
        self.log = self.get_logger()

        self.CONTROLS = {
            "brightness": 0,
            "contrast": 128,
            "saturation": 128,
            "hue": 0,
            "gamma": 200,
            "sharpness"": 1024,
            "backlight_compensation": 0,

            "white_balance_automatic": False,
            "white_balance_temperature": 5000,

            "auto_exposure": 0,
            "exposure_time_absolute": 156,
            "gain": 1,

            "zoom_absolute": 0,

            "power_line_frequency": 1
        }

        # Initialize cameras
        config_dir = Path(get_package_share_directory("camera_stream")) / "config"

        self.streams_config = load_streams_config(config_dir)
        self.cameras: dict[str, Camera] = {}

        self.initialize_cameras()


        # Image parameters
        self._add_bounded_parameter("brightness", 0, -128, 127, 1)
        self._add_bounded_parameter("contrast", 128, 0, 255, 1)
        self._add_bounded_parameter("saturation", 128, 0, 255, 1)
        self._add_bounded_parameter("hue", 0, -9000, 9000, 100)
        self._add_bounded_parameter("gamma", 200, 100, 300, 1)
        self._add_bounded_parameter("sharpness", 1024, 0, 2048, 1)
        self._add_bounded_parameter("backlight_compensation", 0, 0, 9, 1)

        # White balance parameters
        self.declare_parameter("white_balance_automatic", False)
        self._add_bounded_parameter("white_balance_temperature", 5000, 2000, 7500, 1)

        # Exposure parameters
        self._add_bounded_parameter("auto_exposure", 0, 0, 3, 1)
        self._add_bounded_parameter("exposure_time_absolute", 156, 1, 10000, 1)
        self._add_bounded_parameter("gain", 1, 1, 54, 1)

        # Zoom parameters
        self._add_bounded_parameter("zoom_absolute", 0, 0, 100, 1)

        # Power parameters
        self._add_bounded_parameter("power_line_frequency", 1, 0, 2, 1)

        self.add_on_set_parameters_callback(self._parameter_callback)


        # ROS service for restarting camera streams
        self.restart_stream_srv = self.create_srv(RestartStream, "restart_stream", self._restart_stream_callback)


    # -------------------- Public methods --------------------

    def start_all_streams(self):
        for cam_name in self.cameras:
            self.start_stream(cam_name)

    def stop_all_streams(self):
        for cam_name in self.cameras:
            self.stop_stream(cam_name)

    # -------------------- Private methods --------------------

    def _start_stream(self, cam_name):
        self._init_camera(cam_name)

        camera = self.cameras[cam_name]

        if camera is not None:
            camera.start_stream()

    def _init_camera(self, cam_name):
        stream_config = self.streams_config[cam_name]

        try:
            camera = Camera(stream_config)
        except RuntimeError:
            camera = None
            self.log.warn(f"Failed to initialize camera '{cam_name}'")

        self.cameras[cam_name] = camera

    def _stop_stream(self, cam_name):
        camera = self.cameras[name]

        if camera is not None:
            camera.stop_stream()

    def _set_camera_control(control_name, value):
        for cam_name in self.cameras:
            camera = self.cameras[cam_name]
            camera.set_control(control_name, value)

    def _parameter_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True

        for parameter in parameters:
            if self.CONTROLS[parameter.name] != parameter.value:
                try:
                    self._set_camera_control(parameter.name, parameter.value)
                except:
                    self.log.warn(f"Failed to set parameter '{parameter.name}' to value: {parameter.value}")
                    result.successful = False
                    return result

                self.CONTROLS[parameter.name] = parameter.value

        return result

    def _restart_stream_callback(self, request, response):
        camera_name = request.camera_name

        try:
            self._stop_stream(name)
            self._start_stream(camera_name)
        except:
            response.success = False
            return response

        response.success = True
        return response

    def _add_bounded_parameter(self, parameter_name, default_value, from_value, to_value, step):
        range = IntegerRange(
            from_value=from_value,
            to_value=to_value,
            step=step
        )
        parameter_descriptor = ParameterDescriptor(integer_range=range)

        self.declare_parameter(parameter_name, default_value, parameter_descriptor)

def main():
    rclpy.init(args=args)

    camera_manager_node = CameraMangerNode()
    camera_manager_node.start_all_streams()

    rclpy.spin(camera_manager_node)

    camera_manager_node.stop_all_streams()

    camera_manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

