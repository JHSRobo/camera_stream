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
