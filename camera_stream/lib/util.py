import yaml
from pathlib import Path

from data_classes import StreamConfig

def load_streams_config(config_dir):
    path = config_dir / "streams_config.yaml"

    try:
        with path.open("r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

    except FileNotFoundError as e:
        raise ConfigError(f"Configuration file not found: {path}") from e

    streams_config: dict[str, StreamConfig] = {}

    for camera_name, config in data.items():
        try:
            streams_config[camera_name] = StreamConfig(name=camera_name, **config)
        except TypeError as e:
            raise ValueError(f"Invalid fields in stream configuration for camera '{camera_name}'") from e

    return streams_config
