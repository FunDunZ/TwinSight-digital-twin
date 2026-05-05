#!/usr/bin/env python3
"""Shared helpers for tracking-focused BNO055 scripts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from imu_pi.config import load_config
from imu_pi.reader import build_reader
from imu_pi.units import convert_raw_to_physical


@dataclass(frozen=True)
class TrackingRuntime:
    """Bundle of objects commonly needed by the tracking scripts."""

    config: Any
    reader: Any
    period_s: float


@dataclass(frozen=True)
class TrackingRawSample:
    """Tracking-focused subset of raw sensor outputs."""

    accel: Any
    gyro: Any
    calibration: Any


@dataclass(frozen=True)
class TrackingPhysicalSample:
    """Tracking-focused subset of physical-unit sensor outputs."""

    linear_acceleration_m_s2: Any
    angular_velocity_rad_s: Any
    calibration: Any


def build_tracking_runtime() -> TrackingRuntime:
    """Create and return the config, reader, and poll period."""

    config = load_config()
    reader = build_reader(config)
    period_s = 1.0 / max(config.poll_hz, 0.1)
    return TrackingRuntime(config=config, reader=reader, period_s=period_s)


def select_tracking_raw_fields(raw_sample: Any) -> TrackingRawSample:
    """Reduce a full raw sample to the fields needed for tracking."""

    return TrackingRawSample(
        accel=raw_sample.accel,
        gyro=raw_sample.gyro,
        calibration=raw_sample.calibration,
    )


def select_tracking_physical_fields(raw_sample: Any) -> TrackingPhysicalSample:
    """Convert a raw sample and keep only the tracking-relevant outputs."""

    sample = convert_raw_to_physical(raw_sample)
    return TrackingPhysicalSample(
        linear_acceleration_m_s2=sample.linear_acceleration_m_s2,
        angular_velocity_rad_s=sample.angular_velocity_rad_s,
        calibration=sample.calibration,
    )
