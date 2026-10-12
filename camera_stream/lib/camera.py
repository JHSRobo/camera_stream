from v4l2py.device import device

from data_classes import StreamConfig, CameraSettings
from streams import H264Stream, MJPEGStream

STREAM_FORMATS = {
    "h264": H264Stream,
    "mjpeg": MJPEGStream
}

class Camera:
    def __init__(self, stream_config: StreamConfig):

        # Verify that camera is accessible
        source = Path(stream_config.source)

        if not source.exists():
            raise RuntimeError(
                f"Camera '{stream_config.name}' source not found: {source}."
                f"Check that camera is connected and source path is correct.")

        # Use correct stream class based on format
        try:
            stream_format = STREAM_FORMATS[stream_config.format]
        except KeyError:
            raise ValueError(f"Unsupported stream format: {stream_config.format}")

        self.stream = stream_format(stream_config)
        self.device = device(stream_config.source)

    # -------------------- Public methods --------------------

    def start_stream(self):
        self.stream.start()

    def stop_stream(self):
        self.stream.stop()

    def set_control(control_name, value):
        self._open_device()

        if self.CONTROLS[control_name] != value:
            try:
                self.device.controls[control_name].value = value
            except AttributeError:
                raise RuntimeError(f"Camera does not support control: {control_name}")
            except ValueError:
                raise RuntimeError(f"Invalid value: {value}")

            self.CONTROLS[control_name] = value

        self._close_device()

    # -------------------- Protected methods --------------------

    def _open_device(self):
        self.device.open()

    def _close_device(self):
        self.device.close()
