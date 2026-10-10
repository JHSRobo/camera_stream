from dataclasses import dataclass

@dataclass(frozen=True)
class StreamConfig:
    name: str
    source: str
    width: int
    height: int
    frame_rate: int
    format: str
    host: str
    port: str

@dataclass
class CameraSettings:
    brightness: int
    contrast: int
    saturation: int
    hue: int
    gamma: int
    sharpness: int
    backlight_compensation: int
    white_balance_automatic: bool
    white_balance_temperature: int
    auto_exposure: int
    exposure_time_absolute: int
    gain: int
    zoom_absolute: int
    power_line_frequency: int
