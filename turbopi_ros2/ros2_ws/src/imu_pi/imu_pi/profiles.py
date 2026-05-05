from __future__ import annotations

"""IMU register profiles."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ImuProfile:
    """Register-level description of one supported IMU."""

    name: str
    chip_id_register: int
    chip_id_value: int
    page_id_register: int
    operation_mode_register: int
    power_mode_register: int
    sys_trigger_register: int
    temperature_register: int
    calibration_register: int
    normal_power_mode_value: int
    config_mode_value: int
    raw_mode_value: int
    data_register: int
    data_length: int
    fused_mode_value: int
    quaternion_register: int
    quaternion_length: int
    quaternion_scale: float


PROFILES: dict[str, ImuProfile] = {
    "bno055": ImuProfile(
        name="bno055",
        chip_id_register=0x00,
        chip_id_value=0xA0,
        page_id_register=0x07,
        operation_mode_register=0x3D,
        power_mode_register=0x3E,
        sys_trigger_register=0x3F,
        temperature_register=0x34,
        calibration_register=0x35,
        normal_power_mode_value=0x00,
        config_mode_value=0x00,
        raw_mode_value=0x07,
        fused_mode_value=0x0C,
        data_register=0x08,
        data_length=18,
        quaternion_register=0x20,
        quaternion_length=8,
        quaternion_scale=16384.0,
    ),
}


def get_profile(name: str) -> ImuProfile:
    """Return a profile by name and raise a clear error if it is unknown."""

    try:
        return PROFILES[name]
    except KeyError as exc:
        available = ", ".join(sorted(PROFILES))
        raise ValueError(f"Unknown IMU profile '{name}'. Available: {available}") from exc
