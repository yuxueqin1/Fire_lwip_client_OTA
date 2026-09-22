"""
Standard Modbus RTU handler for function code 0x04 (Read Input Registers).

Handles all standard Modbus modules (0x31-0x36, 0x47-0x4A), supporting both
normal reads (≤125 registers) and extended reads (>125 registers) for
high-frequency modules like 0x48 (133 input registers).
"""

import struct
from typing import Optional

from modbus_crc import modbus_crc16, verify_crc16, modbus_crc16_bytes
from module_profiles import ModuleProfile
from data_simulator import DataSimulator


class StandardHandler:
    """
    Handles Modbus 0x04 (Read Input Registers) requests for one slave device.

    Supports both standard (≤125 registers) and extended (>125 registers)
    read modes.  Extended mode is needed for high-frequency modules like
    0x48 (133 input registers) which exceed the Modbus RTU byte_count limit.
    """

    # Modbus exception codes
    EXC_ILLEGAL_FUNCTION    = 0x01
    EXC_ILLEGAL_DATA_ADDR   = 0x02
    EXC_ILLEGAL_DATA_VALUE  = 0x03

    def __init__(self, address: int, profile: ModuleProfile,
                 data_gen: DataSimulator,
                 allow_extended_read: bool = False) -> None:
        """
        Args:
            address: Slave device address (1-247).
            profile: Module profile with register counts.
            data_gen: Data simulator instance for this device.
            allow_extended_read: If True, accept reads beyond the standard
                                 125-register Modbus limit.
        """
        self.address = address
        self.profile = profile
        self.data_gen = data_gen
        self.allow_extended_read = allow_extended_read

        # Statistics
        self.request_count: int = 0
        self.error_count: int = 0
        self.last_request: Optional[bytes] = None
        self.last_response: Optional[bytes] = None

    # ------------------------------------------------------------------
    # Frame detection
    # ------------------------------------------------------------------

    def can_handle(self, raw_bytes: bytes) -> bool:
        """
        Check if ``raw_bytes`` is a valid 0x04 request for this device.

        A standard Modbus RTU 0x04 request is exactly 8 bytes:
            [addr, 0x04, start_hi, start_lo, qty_hi, qty_lo, CRC_lo, CRC_hi]
        """
        if len(raw_bytes) != 8:
            return False
        if raw_bytes[0] != self.address:
            return False
        if raw_bytes[1] != 0x04:
            return False
        return True

    # ------------------------------------------------------------------
    # Response building
    # ------------------------------------------------------------------

    def build_response(self, raw_bytes: bytes) -> Optional[bytes]:
        """
        Parse the 0x04 request and build the response.

        Args:
            raw_bytes: The 8-byte Modbus RTU request frame.

        Returns:
            Response frame bytes (including CRC), or None if the request
            is invalid (CRC error, malformed, etc.).
        """
        self.last_request = raw_bytes
        self.request_count += 1

        # --- CRC validation ---
        if not verify_crc16(raw_bytes):
            self.error_count += 1
            return None  # ignore corrupted frames (real slaves do this)

        # --- Parse request ---
        # [addr, 0x04, start_hi, start_lo, qty_hi, qty_lo, CRC_lo, CRC_hi]
        start_addr = (raw_bytes[2] << 8) | raw_bytes[3]
        quantity = (raw_bytes[4] << 8) | raw_bytes[5]

        # --- Validate ---
        if start_addr != 0:
            # We always start from register 0; anything else is unusual
            # but not illegal — return exception 02
            resp = self._build_exception(0x04, self.EXC_ILLEGAL_DATA_ADDR)
            self.last_response = resp
            self.error_count += 1
            return resp

        if quantity == 0 or quantity > self.profile.input_regs:
            resp = self._build_exception(0x04, self.EXC_ILLEGAL_DATA_VALUE)
            self.last_response = resp
            self.error_count += 1
            return resp

        # Check for >125 register reads
        if quantity > 125 and not self.allow_extended_read:
            resp = self._build_exception(0x04, self.EXC_ILLEGAL_DATA_VALUE)
            self.last_response = resp
            self.error_count += 1
            return resp

        # --- Generate register values ---
        reg_values = self.data_gen.get_registers(start_addr, quantity)

        # --- Build response ---
        # PDU: [0x04, byte_count, reg0_hi, reg0_lo, ..., regN-1_hi, regN-1_lo]
        byte_count = quantity * 2
        # For extended reads, byte_count may exceed 255.
        # Use (byte_count & 0xFF) as the wire byte_count — the real slave
        # hardware uses this convention for >125 register reads.
        wire_byte_count = byte_count & 0xFF

        resp = bytearray([self.address, 0x04, wire_byte_count])
        for val in reg_values:
            resp.append((val >> 8) & 0xFF)   # hi byte
            resp.append(val & 0xFF)          # lo byte

        # Append CRC
        crc = modbus_crc16(resp)
        resp.append(crc & 0xFF)
        resp.append((crc >> 8) & 0xFF)

        self.last_response = bytes(resp)
        return bytes(resp)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_exception(self, func_code: int, exc_code: int) -> bytes:
        """Build a Modbus exception response."""
        # Exception frame: [addr, func | 0x80, exc_code, CRC_lo, CRC_hi]
        resp = bytearray([self.address, func_code | 0x80, exc_code])
        crc = modbus_crc16(resp)
        resp.append(crc & 0xFF)
        resp.append((crc >> 8) & 0xFF)
        return bytes(resp)
