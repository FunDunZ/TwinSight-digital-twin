from __future__ import annotations

"""High-level IMU reader that turns raw bytes into signed integer samples."""

from dataclasses import dataclass
import time

from imu_pi.config import ImuConfig
from imu_pi.i2c import read_block, write_byte
from imu_pi.profiles import ImuProfile, get_profile


@dataclass(frozen=True)
class RawSample:
    """One raw sample directly decoded from the sensor register frame."""

    accel: tuple[int, int, int]
    mag: tuple[int, int, int]
    temperature: int
    gyro: tuple[int, int, int]
    calibration: tuple[int, int, int, int]


@dataclass(frozen=True)
class QuaternionSample:
    """Fused BNO055 orientation quaternion."""

    w: float
    x: float
    y: float
    z: float


def _to_int16(high: int, low: int) -> int:
    """Convert two bytes from the IMU into one signed 16-bit integer."""

    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


class RawImuReader:
    """Read raw IMU frames using the selected configuration and register profile."""

    def __init__(self, config: ImuConfig, profile: ImuProfile) -> None:
        self._config = config
        self._profile = profile
        self._initialize_device()

    def read_quaternion(self) -> QuaternionSample:
        raw = read_block(
            self._config.bus,
            self._config.address,
            self._profile.quaternion_register,
            self._profile.quaternion_length,
        )

        w_raw = _to_int16(raw[1], raw[0])
        x_raw = _to_int16(raw[3], raw[2])
        y_raw = _to_int16(raw[5], raw[4])
        z_raw = _to_int16(raw[7], raw[6])

        scale = self._profile.quaternion_scale

        w = w_raw / scale
        x = x_raw / scale
        y = y_raw / scale
        z = z_raw / scale

        norm = (w**2 + x**2 + y**2 + z**2) ** 0.5

        calib = self._read_calibration_status()
        if calib[0] < 2:
            print("WARNING: IMU not calibrated:", calib)

        return QuaternionSample(
            w=w / norm,
            x=x / norm,
            y=y / norm,
            z=z / norm,
        )

    def _initialize_device(self) -> None:
        """Put the BNO055 into a known page, power state, and fused-orientation mode."""

        write_byte(self._config.bus, self._config.address, self._profile.page_id_register, 0x00)

        chip_id = read_block(self._config.bus, self._config.address, self._profile.chip_id_register, 1)[0]
        if chip_id != self._profile.chip_id_value:
            raise RuntimeError(
                f"Unexpected chip ID 0x{chip_id:02X}; expected 0x{self._profile.chip_id_value:02X}"
            )

        write_byte(self._config.bus, self._config.address, self._profile.operation_mode_register, self._profile.config_mode_value)
        time.sleep(0.02)

        write_byte(self._config.bus, self._config.address, self._profile.power_mode_register, self._profile.normal_power_mode_value)
        write_byte(self._config.bus, self._config.address, self._profile.sys_trigger_register, 0x00)
        write_byte(self._config.bus, self._config.address, self._profile.page_id_register, 0x00)

        write_byte(self._config.bus, self._config.address, self._profile.operation_mode_register, self._profile.fused_mode_value)
        time.sleep(0.02)

    def read_raw(self) -> RawSample:
        """Read one raw frame and split it into accel, mag, temp, and gyro values."""

        raw = read_block(
            self._config.bus,
            self._config.address,
            self._profile.data_register,
            self._profile.data_length,
        )

        accel = (
            _to_int16(raw[1], raw[0]),
            _to_int16(raw[3], raw[2]),
            _to_int16(raw[5], raw[4]),
        )
        mag = (
            _to_int16(raw[7], raw[6]),
            _to_int16(raw[9], raw[8]),
            _to_int16(raw[11], raw[10]),
        )
        gyro = (
            _to_int16(raw[13], raw[12]),
            _to_int16(raw[15], raw[14]),
            _to_int16(raw[17], raw[16]),
        )

        temperature = read_block(self._config.bus, self._config.address, self._profile.temperature_register, 1)[0]
        calibration = self._read_calibration_status()

        return RawSample(
            accel=accel,
            mag=mag,
            temperature=temperature,
            gyro=gyro,
            calibration=calibration,
        )

    def _read_calibration_status(self) -> tuple[int, int, int, int]:
        """Decode CALIB_STAT into (sys, gyro, accel, mag), each ranging from 0 to 3."""

        calib = read_block(self._config.bus, self._config.address, self._profile.calibration_register, 1)[0]
        system = (calib >> 6) & 0x03
        gyro = (calib >> 4) & 0x03
        accel = (calib >> 2) & 0x03
        mag = calib & 0x03
        return (system, gyro, accel, mag)


def build_reader(config: ImuConfig) -> RawImuReader:
    """Resolve the configured profile and create a ready-to-use reader."""

    profile = get_profile(config.profile)
    return RawImuReader(config=config, profile=profile)
