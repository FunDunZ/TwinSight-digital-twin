from __future__ import annotations

"""Convert BNO055 raw register values into physical units."""

from dataclasses import dataclass
import math

from imu_pi.reader import RawSample


ACCEL_LSB_PER_M_S2 = 100.0
GYRO_LSB_PER_DPS = 16.0
MAG_LSB_PER_UT = 16.0
TESLA_PER_MICROTESLA = 1e-6


@dataclass(frozen=True)
class Vector3:
    """Simple 3-axis container used for physical-unit outputs."""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PhysicalSample:
    """Sensor sample converted into physical units."""

    linear_acceleration_m_s2: Vector3
    angular_velocity_rad_s: Vector3
    magnetic_field_t: Vector3
    temperature_c: float
    calibration: tuple[int, int, int, int]


def _vector_from_raw(raw_xyz: tuple[int, int, int], scale: float) -> Vector3:
    return Vector3(
        x=raw_xyz[0] / scale,
        y=raw_xyz[1] / scale,
        z=raw_xyz[2] / scale,
    )


def convert_raw_to_physical(sample: RawSample) -> PhysicalSample:
    """Convert one BNO055 raw sample into physical units."""

    linear_acceleration_m_s2 = _vector_from_raw(sample.accel, ACCEL_LSB_PER_M_S2)

    gyro_deg_s = _vector_from_raw(sample.gyro, GYRO_LSB_PER_DPS)
    angular_velocity_rad_s = Vector3(
        x=math.radians(gyro_deg_s.x),
        y=math.radians(gyro_deg_s.y),
        z=math.radians(gyro_deg_s.z),
    )

    mag_uT = _vector_from_raw(sample.mag, MAG_LSB_PER_UT)
    magnetic_field_t = Vector3(
        x=mag_uT.x * TESLA_PER_MICROTESLA,
        y=mag_uT.y * TESLA_PER_MICROTESLA,
        z=mag_uT.z * TESLA_PER_MICROTESLA,
    )

    return PhysicalSample(
        linear_acceleration_m_s2=linear_acceleration_m_s2,
        angular_velocity_rad_s=angular_velocity_rad_s,
        magnetic_field_t=magnetic_field_t,
        temperature_c=float(sample.temperature),
        calibration=sample.calibration,
    )
