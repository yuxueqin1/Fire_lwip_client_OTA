"""
DC 3-wire custom protocol handler for module types 0x51 and 0x52.

Protocol (mirrors firmware's port_dc3wire.c):
  Request (8 bytes):
    [addr, 0x03, 0x00, 0x00, curve_seq_hi, curve_seq_lo, CRC_lo, CRC_hi]

  Response (no curve — curve_flag=0x0000):
    [addr, 0x03, byte_count,
     realtime_data(20B big-endian),
     padding, 0x00, 0x00,
     CRC_lo, CRC_hi]

  Response (with curve — curve_flag=0x80FF, >= 186 bytes total):
    [addr, 0x03, byte_count,
     realtime_data(20B big-endian),
     padding,
     0x80, 0xFF,                    # curve_flag at offset 24-25
     curve_send_count(2B BE),       # offset 26-27
     channel(2B BE),                # offset 28-29
     curve_num(2B BE),             # offset 30-31
     start_time_sec(4B BE),        # offset 32-35
     point_count(2B BE),           # offset 36-37
     data_points(75×2B BE),        # offset 38+
     CRC_lo, CRC_hi]

  Time-sync (13 bytes):
    [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
     ts_byte3, ts_byte2, ts_byte1, ts_byte0, CRC_lo, CRC_hi]

The curve_seq from the master is used as an ACK mechanism. The slave
responds with curve_send_count = curve_seq + 1 when curve data is ready.
"""

import math
import random
import struct
import time as _time
from enum import Enum
from typing import Optional

from modbus_crc import modbus_crc16, verify_crc16


class DC3WireCurveMode(Enum):
    """Curve data mode for DC 3-wire modules."""
    REALTIME_ONLY = "realtime"   # Always respond with curve_flag=0x0000
    CURVE = "curve"              # Respond with 75-point curve when requested


class DC3WireHandler:
    """
    Handles the custom serial protocol for DC 3-wire slave devices.

    Supports two sub-module types: platform door (0x51) and station/field
    link (0x52). Both share the same protocol but differ in data_type
    encoding during the curve upload phase (handled by the master, not
    relevant to this handler).
    """

    def __init__(self, address: int, profile, data_gen,
                 curve_mode: DC3WireCurveMode = DC3WireCurveMode.REALTIME_ONLY,
                 realtime_bytes: int = 20) -> None:
        """
        Args:
            address: Slave device address (1-247).
            profile: ModuleProfile instance.
            data_gen: DataSimulator instance for register values.
            curve_mode: Whether to include 75-point curve data.
            realtime_bytes: Number of realtime data bytes in response.
        """
        self.address = address
        self.profile = profile
        self.data_gen = data_gen
        self._realtime_bytes = realtime_bytes

        # Curve mode
        self.curve_mode = curve_mode

        # Curve generation state
        self._last_curve_seq: int = 0         # Last seq received from master
        self._curve_counter: int = 0          # Internal curve counter
        self._cycle_active: bool = False      # Has a curve cycle been triggered
        self._curve_data: list[int] = []      # Current 75-point curve

        # Statistics
        self.request_count: int = 0
        self.error_count: int = 0
        self.last_request: Optional[bytes] = None
        self.last_response: Optional[bytes] = None

    # ------------------------------------------------------------------
    # Frame detection
    # ------------------------------------------------------------------

    def can_handle(self, raw_bytes: bytes) -> bool:
        """Check if ``raw_bytes`` is for this device.

        Matches:
            - 8-byte curve request:  [addr, 0x03, ...]
            - 13-byte time sync:     [addr, 0x10, ...]
        """
        if len(raw_bytes) < 4:
            return False
        if raw_bytes[0] != self.address:
            return False
        if raw_bytes[1] not in (0x03, 0x10):
            return False
        if raw_bytes[1] == 0x03 and len(raw_bytes) == 8:
            return True
        if raw_bytes[1] == 0x10 and len(raw_bytes) == 13:
            return True
        return False

    # ------------------------------------------------------------------
    # Response building
    # ------------------------------------------------------------------

    def build_response(self, raw_bytes: bytes) -> Optional[bytes]:
        """
        Parse the request and build the response.

        Returns:
            Response bytes with CRC, or None on CRC error.
        """
        self.last_request = raw_bytes
        self.request_count += 1

        if not verify_crc16(raw_bytes):
            self.error_count += 1
            return None

        func = raw_bytes[1]
        if func == 0x10:
            return self._handle_time_sync(raw_bytes)
        elif func == 0x03:
            return self._handle_curve_request(raw_bytes)
        else:
            self.error_count += 1
            return None

    # ------------------------------------------------------------------
    # Time sync (0x10)
    # ------------------------------------------------------------------

    def _handle_time_sync(self, raw_bytes: bytes) -> bytes:
        """Handle time-sync: [addr, 0x10, ... ts[4] BE, CRC]"""
        ts = ((raw_bytes[7] << 24) | (raw_bytes[8] << 16) |
              (raw_bytes[9] << 8) | raw_bytes[10])
        self.data_gen.set_system_time(ts)

        resp = bytearray(raw_bytes[:6])
        crc = modbus_crc16(resp)
        resp.append(crc & 0xFF)
        resp.append((crc >> 8) & 0xFF)

        self.last_response = bytes(resp)
        return bytes(resp)

    # ------------------------------------------------------------------
    # Curve request (0x03)
    # ------------------------------------------------------------------

    def _handle_curve_request(self, raw_bytes: bytes) -> bytes:
        """Handle 8-byte DC 3-wire request.

        Request: [addr, 0x03, 0x00, 0x00, curve_seq_hi, curve_seq_lo, CRC]
        """
        curve_seq = (raw_bytes[4] << 8) | raw_bytes[5]

        # Determine if we should include curve data
        if self.curve_mode == DC3WireCurveMode.CURVE:
            return self._build_curve_response(curve_seq)
        else:
            return self._build_realtime_response(curve_seq)

    # ------------------------------------------------------------------
    # Response builders
    # ------------------------------------------------------------------

    def _build_realtime_response(self, curve_seq: int) -> bytes:
        """Build a realtime-only response with curve_flag=0x0000."""
        rt = self.data_gen.get_raw_bytes(self._realtime_bytes)

        # Frame: [addr, 0x03, byte_count, realtime_data, padding, 0x00, 0x00, CRC]
        frame = bytearray()
        frame.append(self.address)
        frame.append(0x03)

        # Reserve byte_count position
        bc_idx = 2
        frame.append(0x00)

        # Realtime data
        frame.extend(rt)

        # Calculate total data bytes so far (after addr+func+byte_count)
        data_so_far = len(frame) - 3

        # Ensure curve_flag at offset 24-25 is accessible
        # Required minimum data area: at least 26 bytes so curve_flag check
        # in firmware works (real_byte_count >= 26 condition is on profile,
        # but offset 24-25 is a hardcoded position in the response).
        # We pad to at least 26 bytes of data.
        min_data = max(self._realtime_bytes, 26)
        while len(frame) - 3 < min_data:
            frame.append(0x00)

        # Set curve_flag = 0x0000 at offset 24-25 (from frame start)
        # This is at frame index 24 (addr(0)+func(1)+byte_count(2)+data_offset)
        flag_offset = 24  # absolute in frame
        while len(frame) <= flag_offset + 1:
            frame.append(0x00)
        frame[flag_offset] = 0x00
        frame[flag_offset + 1] = 0x00

        # Fill byte_count
        data_bytes = len(frame) - 3
        frame[bc_idx] = data_bytes & 0xFF

        # CRC
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        self.last_response = bytes(frame)
        return bytes(frame)

    def _build_curve_response(self, curve_seq: int) -> bytes:
        """Build a response with 75-point curve data (curve_flag=0x80FF).

        Generates fresh curve data when the master's curve_seq advances.
        """
        # Detect new curve cycle: when curve_seq changes or first request
        if curve_seq != self._last_curve_seq:
            self._last_curve_seq = curve_seq
            self._cycle_active = True
            self._curve_counter += 1
            self._generate_curve()

        rt = self.data_gen.get_raw_bytes(self._realtime_bytes)

        # Response format (>= 186 bytes total required by firmware):
        # addr(1) + func(1) + byte_count(1) + realtime + padding +
        # curve_flag(2) + curve_send_count(2) + channel(2) +
        # curve_num(2) + start_time(4) + point_count(2) + data(150) + CRC(2)
        frame = bytearray()
        frame.append(self.address)
        frame.append(0x03)

        bc_idx = 2
        frame.append(0x00)  # byte_count placeholder

        # Realtime data
        frame.extend(rt)

        # Padding to reach curve_flag offset (24-25)
        while len(frame) < 24:
            frame.append(0x00)

        # curve_flag = 0x80FF
        frame.append(0x80)
        frame.append(0xFF)

        # curve_send_count = curve_seq + 1
        send_count = (curve_seq + 1) & 0xFFFF
        frame.append((send_count >> 8) & 0xFF)
        frame.append(send_count & 0xFF)

        # channel (1..3, randomly chosen)
        channel = random.randint(1, 3)
        frame.append(0x00)
        frame.append(channel & 0xFF)

        # curve_num
        frame.append((self._curve_counter >> 8) & 0xFF)
        frame.append(self._curve_counter & 0xFF)

        # start_time_sec (4 bytes big-endian)
        now = int(_time.time())
        frame.append((now >> 24) & 0xFF)
        frame.append((now >> 16) & 0xFF)
        frame.append((now >> 8) & 0xFF)
        frame.append(now & 0xFF)

        # point_count = 75 (0x004B)
        pt_count = min(len(self._curve_data), 75)
        frame.append(0x00)
        frame.append(pt_count & 0xFF)

        # data_points (each 2 bytes big-endian)
        for i in range(75):
            val = self._curve_data[i] if i < len(self._curve_data) else 0
            frame.append((val >> 8) & 0xFF)
            frame.append(val & 0xFF)

        # Fill byte_count
        data_bytes = len(frame) - 3
        frame[bc_idx] = data_bytes & 0xFF

        # CRC
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        self.last_response = bytes(frame)
        return bytes(frame)

    # ------------------------------------------------------------------
    # Curve generation
    # ------------------------------------------------------------------

    def _generate_curve(self) -> None:
        """Generate a realistic 75-point curve data set.

        Simulates a DC signal with an initial transient that settles.
        """
        self._curve_data = []
        for i in range(75):
            t = i / 75.0  # Normalised 0..1

            # Base: decaying transient + noise
            if t < 0.1:
                # Start-up spike
                base = 3000.0 - t * 20000.0
            elif t < 0.3:
                # Decay to steady state
                base = 1000.0 + math.exp(-t * 3.0) * 2000.0
            else:
                # Steady with slight oscillation
                base = 800.0 + math.sin(t * 20.0) * 50.0

            # Add noise
            val = int(base + random.gauss(0, 20))
            val = max(0, min(65535, val))
            self._curve_data.append(val)

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def set_curve_mode(self, mode: DC3WireCurveMode) -> None:
        """Switch between realtime-only and curve modes."""
        self.curve_mode = mode
        if mode == DC3WireCurveMode.CURVE:
            self._cycle_active = False
            self._last_curve_seq = 0

    def toggle_curve_mode(self) -> DC3WireCurveMode:
        """Toggle curve mode and return the new mode."""
        new_mode = (DC3WireCurveMode.REALTIME_ONLY
                    if self.curve_mode == DC3WireCurveMode.CURVE
                    else DC3WireCurveMode.CURVE)
        self.set_curve_mode(new_mode)
        return new_mode

    def status_line(self) -> str:
        """Single-line status for CLI display."""
        mode_str = self.curve_mode.value
        return (f"  [{self.address:3d}] 0x{self.profile.module_type:02X} "
                f"DC3Wire [{mode_str}] "
                f"req={self.request_count} err={self.error_count}")
