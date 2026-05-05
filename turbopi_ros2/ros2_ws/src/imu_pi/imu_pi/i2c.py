from __future__ import annotations

"""Low-level I2C helpers."""

from smbus2 import SMBus


def read_block(bus_id: int, address: int, register: int, length: int) -> list[int]:
    """Read a consecutive block of bytes starting at a register address."""

    with SMBus(bus_id) as bus:
        return bus.read_i2c_block_data(address, register, length)


def write_byte(bus_id: int, address: int, register: int, value: int) -> None:
    """Write a single byte to one device register."""

    with SMBus(bus_id) as bus:
        bus.write_byte_data(address, register, value)


def scan_bus(bus_id: int) -> list[int]:
    """Probe common I2C addresses and return the devices that respond."""

    detected: list[int] = []
    with SMBus(bus_id) as bus:
        for address in range(0x03, 0x78):
            try:
                bus.write_quick(address)
            except OSError:
                continue
            detected.append(address)
    return detected
