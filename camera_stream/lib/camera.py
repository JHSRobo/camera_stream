from v4l2py.device import device

from data_classes import StreamConfig, CameraSettings
from streams import H264Stream, MJPEGStream

STREAM_FORMATS = {
    "h264": H264Stream,
    "mjpeg": MJPEGStream
}

class Camera:
    def __init__(self, stream_config: StreamConfig):
        self.stream_config = stream_config

        try:
            stream_format = STREAM_FORMATS[stream_config.format]
        except KeyError:
            raise ValueError(f"Unsupported stream format: {stream_config.format}")

        self.stream = stream_format(stream_config)
        self.device = device(stream_config.source)

    def start_stream(self):
        self.stream.start()

    def stop_stream(self):
        self.stream.stop()

    def load_parameters(self, camera_settings: CameraSettings):
        self.device.open()

        for parameter, value in vars(camera_settings).items():
            self.set_parameter(parameter, value)

        self.device.close()

    def set_parameter(self, parameter: str, value):
        self.device.open()

        try:
            self.device.controls[parameter].value = value

        except AttributeError:
            raise RuntimeError(f"Camera does not support control: {parameter}")

        except ValueError:
            raise RuntimeError(f"Invalid value: {value}")

        self.device.close()

    def open(self):
        self.device.open()

    def close(self):
        self.stream.stop()
        self.device.close()
