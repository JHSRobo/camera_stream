import yaml
from v4l2py.device import Device

from streams import H264Stream, MJPEGStream

class Camera:
    def __init__(self, name, config_dir):
        self.name = name

        camera_config = f"{config_dir}/cameras.yaml"

        with open(camera_config, "r") as file:
            config = yaml.safe_load(file)[self.name]

            self.source = config["source"]

            self.frame_rate = config["frame_rate"]
            self.width = config["width"]
            self.height = config["height"]
            self.format = config["format"]

            self.host = config["host"]
            self.port = config["port"]

            self.profile = config["profile"]
            self.mode = config["mode"]

        if self.format == "MJPEG":
            self.stream = MJPEGStream(
                name=self.name,
                source=self.source,
                width=self.width,
                height=self.height,
                frame_rate=self.frame_rate,
                host=self.host,
                port=self.port
            )
        elif self.format == "H264":
            self.stream = H264Stream(
                name=self.name,
                source=self.source,
                width=self.width,
                height=self.heigh,
                frame_rate=self.frame_rate,
                port=self.port
            )
        else:
            print(f"Invalid format: {self.format}")

        self.stream.build_pipeline()

        print(self.stream.pipeline)

        self.device = Device(self.source)
        self.device.open()

        self.configure_params(config_dir)

    def close(self):
        self.stream.stop()
        self.device.close()

    def set_parameter(self, parameter, value):
        self.device.controls[parameter].value = value
        print("Successfully changed parameter: " + parameter + "\n")

    def configure_params(self, config_dir):
        config = f"{config_dir}/modes/{self.mode}.yaml"

        with open(config, "r") as file:
            config = yaml.safe_load(file)[self.profile]

            for param in config.keys():
                self.set_parameter(param, config[param])

