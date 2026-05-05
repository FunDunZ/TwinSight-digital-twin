from __future__ import annotations

"""Configuration loading for the IMU bring-up project."""

import json
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# When installed as a ROS2 package, the config file lives in the package share
# directory. This is the standard ROS2 location for package data files.
DEFAULT_CONFIG_PATH = Path(get_package_share_directory('imu_pi')) / 'config' / 'imu_config.json'


@dataclass(frozen=True)
class ImuConfig:
    """Typed view of the JSON configuration file.

    bus:
        Linux I2C bus number. On most Raspberry Pi boards this is bus 1.
    address:
        The 7-bit I2C address of the IMU.
    profile:
        Name of the IMU register profile to use.
    poll_hz:
        Requested polling rate for the demo read loop.
    """

    bus: int
    address: int
    profile: str
    poll_hz: float


def _parse_address(raw_address: object) -> int:
    """Accept either decimal integers or strings like '0x68' from JSON."""

    if isinstance(raw_address, int):
        return raw_address
    if isinstance(raw_address, str):
        return int(raw_address, 0)
    raise TypeError(f"Unsupported I2C address type: {type(raw_address)!r}")


def load_config(path: Path = DEFAULT_CONFIG_PATH) -> ImuConfig:
    """Load configuration from disk and normalize field types."""

    with path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    return ImuConfig(
        bus=int(raw["bus"]),
        address=_parse_address(raw["address"]),
        profile=str(raw["profile"]).lower(),
        poll_hz=float(raw.get("poll_hz", 10.0)),
    )
