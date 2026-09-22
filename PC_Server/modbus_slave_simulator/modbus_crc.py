"""
Modbus CRC16 computation.

Implements the standard Modbus CRC16 algorithm (polynomial 0xA001, reflected),
identical to usMBMCRC16() in the FreeModbus library used by the STM32 firmware.

Usage:
    crc = modbus_crc16(data)
    # Append to frame as little-endian: [crc & 0xFF, (crc >> 8) & 0xFF]
"""

from typing import Sequence


# Pre-computed CRC table for speed (same algorithm, lookup-table form)
# Polynomial: 0xA001 (reflected)
_CRC16_TABLE: list[int] = []


def _build_crc_table() -> list[int]:
    """Build the Modbus CRC16 lookup table."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        table.append(crc)
    return table


# Initialize the table once at module load time
_CRC16_TABLE = _build_crc_table()


def modbus_crc16(data: bytes | Sequence[int]) -> int:
    """
    Compute Modbus CRC16 over the given data bytes.

    Args:
        data: Bytes or sequence of integers (0-255).

    Returns:
        16-bit CRC value (uint16). Append to frame as little-endian:
            frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    """
    crc = 0xFFFF
    for byte in data:
        idx = (crc ^ byte) & 0xFF
        crc = (crc >> 8) ^ _CRC16_TABLE[idx]
    return crc


def modbus_crc16_bytes(data: bytes | Sequence[int]) -> bytes:
    """
    Compute Modbus CRC16 and return as 2 little-endian bytes.

    Args:
        data: Bytes or sequence of integers.

    Returns:
        Two bytes: [CRC_lo, CRC_hi]
    """
    crc = modbus_crc16(data)
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def verify_crc16(frame: bytes | Sequence[int]) -> bool:
    """
    Verify the CRC16 of a complete Modbus RTU frame.

    A Modbus RTU frame ends with 2 CRC bytes (little-endian).
    The CRC is computed over all bytes EXCEPT the last 2 CRC bytes.
    A valid frame will produce CRC = 0x0000 when CRC is computed over
    the entire frame (including the appended CRC).

    Args:
        frame: Complete Modbus RTU frame including CRC.

    Returns:
        True if CRC is valid, False otherwise.
    """
    if len(frame) < 4:
        return False
    # Method: compute CRC over all bytes INCLUDING the CRC bytes.
    # If the CRC is correct, the result is 0x0000.
    return modbus_crc16(frame) == 0


def append_crc16(data: bytearray | list[int]) -> bytearray:
    """
    Append Modbus CRC16 (2 bytes, little-endian) to a frame.

    Args:
        data: Frame data (modified in-place for bytearray).

    Returns:
        The data with CRC appended.
    """
    if isinstance(data, bytearray):
        crc = modbus_crc16(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)
        return data
    else:
        crc = modbus_crc16(data)
        result = bytearray(data)
        result.append(crc & 0xFF)
        result.append((crc >> 8) & 0xFF)
        return result
