"""
Axle Counter custom protocol handler for module type 0x50.

Protocol (mirrors firmware's port_axle_counter.c):
  Request (8 bytes):
    [addr, 0x03, confirm_hi, confirm_lo, pulse_hi, pulse_lo, CRC_lo, CRC_hi]

  Response (variable length):
    [addr, 0x03, byte_count,
     realtime_data(20B big-endian),
     reserved(2B),
     millis(2B),
     confirm_code(2B big-endian),
     unread_pulse_count(2B big-endian),
     optional_pulse_block,
     CRC_lo, CRC_hi]

  Time-sync (13 bytes):
    [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
     ts_byte3, ts_byte2, ts_byte1, ts_byte0, CRC_lo, CRC_hi]

The confirm_code is echoed back to the master. The req_pulse field requests
a block of pulse data from the slave.
"""

import random
import struct
import time as _time
from enum import Enum
from typing import Optional

from modbus_crc import modbus_crc16, verify_crc16


class AxlePulseMode(Enum):
    """Pulse generation mode for axle counter."""
    REALTIME_ONLY = "realtime"   # No pulse generation
    PULSE = "curve"              # Generate simulated pulse events


class AxleCounterHandler:
    """
    Handles the custom serial protocol for one axle counter slave device.

    Generates realistic realtime register values and simulates pulse data
    when requested.
    """

    def __init__(self, address: int, profile, data_gen,
                 pulse_mode: AxlePulseMode = AxlePulseMode.REALTIME_ONLY,
                 pulse_interval: float = 5.0,
                 max_unread_pulses: int = 50) -> None:
        """
        Args:
            address: Slave device address (1-247).
            profile: ModuleProfile instance.
            data_gen: DataSimulator instance for register values.
            pulse_mode: Whether to generate simulated pulse events.
            pulse_interval: Seconds between simulated pulse events.
            max_unread_pulses: Max simulated unread pulse count.
        """
        self.address = address
        self.profile = profile
        self.data_gen = data_gen

        # Pulse mode
        self.curve_mode = pulse_mode

        # Pulse simulation state
        self._pulse_interval = pulse_interval
        self._max_unread = max_unread_pulses
        self._last_pulse_time: float = 0.0
        self._pulse_events: list[bytes] = []

        # Confirm code tracking (internal auto-increment, per protocol)
        self._confirm_code: int = 0x0001   # 内部确认码 (首次=1, 范围 1~0xFFFE)
        self._last_sent_confirm: int = 0   # 上次应答中发出的确认码

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
            - 8-byte axle request:  [addr, 0x03, ...]
            - 13-byte time sync:    [addr, 0x10, ...]
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
        """Parse request and build response.

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
            return self._handle_axle_request(raw_bytes)
        else:
            self.error_count += 1
            return None

    # ------------------------------------------------------------------
    # Time sync (0x10)
    # ------------------------------------------------------------------

    def _handle_time_sync(self, raw_bytes: bytes) -> bytes:
        """Handle time-sync: [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
                               ts[4] BE, CRC_lo, CRC_hi]"""
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
    # Axle request (0x03)
    # ------------------------------------------------------------------

    def _handle_axle_request(self, raw_bytes: bytes) -> bytes:
        """Handle 8-byte axle counter request.

        Request: [addr, 0x03, confirm_hi, confirm_lo, pulse_hi, pulse_lo, CRC]

        Confirm-code protocol (per sensor spec):
          1. Sensor has an internal auto-incrementing confirm code (1..0xFFFE).
          2. Sensor returns the CURRENT internal code in every response.
          3. If master's request confirm code == last sent confirm code →
             match! Master received last response ok.
             → Clear already-sent pulses, increment internal code.
          4. If master's request confirm code != last sent confirm code →
             miss. Master did NOT receive last response.
             → Keep cached pulses for retransmission, do NOT increment.
          5. Master sends 0x0000 on first query (no prior knowledge).
        """
        req_confirm = (raw_bytes[2] << 8) | raw_bytes[3]
        req_pulse = (raw_bytes[4] << 8) | raw_bytes[5]

        # --- Update pulse simulation (only in PULSE mode) ---
        if self.curve_mode == AxlePulseMode.PULSE:
            self._simulate_pulses()

        # --- Confirm code matching ---
        if (req_confirm == self._last_sent_confirm
                and req_confirm != 0x0000
                and self._last_sent_confirm != 0x0000):
            # Match! Master received last response.
            # Clear pulses that were sent in the last response.
            pulses_to_clear = self._pulses_sent_last_response
            if pulses_to_clear > 0:
                # Remove cleared pulses from the front of the queue
                self._pulse_events = self._pulse_events[pulses_to_clear:]
            # Increment internal confirm code (wrap at 0xFFFE → 1)
            self._confirm_code += 1
            if self._confirm_code >= 0xFFFE:
                self._confirm_code = 1

        # --- Build response ---
        # Realtime data: 20 bytes big-endian (10 registers)
        rt_bytes = self.data_gen.get_raw_bytes(20)

        # Calculate unread count
        unread = max(0, len(self._pulse_events))

        # Determine pulse block: only include if request asked for pulses
        # AND there are unread pulse events
        has_pulse_block = (req_pulse > 0 and len(self._pulse_events) > 0)

        # Build frame (without CRC)
        frame = bytearray()
        frame.append(self.address)
        frame.append(0x03)

        # Reserve space for byte_count — fill later
        byte_count_idx = 2
        frame.append(0x00)  # placeholder

        # [3..22] Realtime data (20 bytes big-endian)
        frame.extend(rt_bytes)

        # [23..24] Reserved
        frame.extend([0x00, 0x00])

        # [25..26] Millis (current ms within second)
        now = _time.time()
        millis = int((now - int(now)) * 1000) & 0xFFFF
        frame.append((millis >> 8) & 0xFF)
        frame.append(millis & 0xFF)

        # [27..28] Confirm code — use current INTERNAL code (not echo!)
        confirm = self._confirm_code
        self._last_sent_confirm = confirm
        frame.append((confirm >> 8) & 0xFF)
        frame.append(confirm & 0xFF)

        # [29..30] Unread pulse count
        frame.append((unread >> 8) & 0xFF)
        frame.append(unread & 0xFF)

        # [31+] Optional pulse block
        self._pulses_sent_last_response = 0
        if has_pulse_block:
            # Limit pulses in this response (max 0x30 = 48 per spec)
            max_pulse_per_req = min(req_pulse, 0x30)
            to_send = self._pulse_events[:max_pulse_per_req]

            # Pulse count (2 bytes big-endian)
            frame.append((len(to_send) >> 8) & 0xFF)
            frame.append(len(to_send) & 0xFF)

            for pkt in to_send:
                frame.extend(pkt)

            self._pulses_sent_last_response = len(to_send)

        # Fill byte_count = total data bytes after byte_count field
        data_bytes = len(frame) - 3  # exclude addr(1)+func(1)+byte_count(1)
        frame[byte_count_idx] = data_bytes & 0xFF

        # Append CRC16
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        self.last_response = bytes(frame)
        return bytes(frame)

    # ------------------------------------------------------------------
    # Pulse simulation
    # ------------------------------------------------------------------

    def _simulate_pulses(self) -> None:
        """Generate a test pulse when queue is empty (PULSE mode only).

        Produces 1 pulse curve:
          - Channel 0
          - 8 data points
          - Amplitude 10V (raw = 100 = 0x64, since scale is ×0.1V)
          - Timestamp = current system time

        Once the master confirms (confirm-code match), the pulse is cleared
        from the queue.  On the next request a fresh pulse is generated,
        allowing the full confirm/clear cycle to be tested repeatedly.
        """
        if len(self._pulse_events) > 0:
            return  # still waiting for master to confirm previous pulse

        now = _time.monotonic()
        if now - self._last_pulse_time >= self._pulse_interval:
            self._last_pulse_time = now

            ts = int(_time.time())
            ms = 0
            channel = 0          # channel 0 (U1)
            num_points = 8       # 8 data points
            # 10V amplitude → raw = 10V / 0.1V = 100
            data_vals = [100] * num_points

            # Build pulse event:
            # sec(4B BE) + ms(2B BE) + channel(1B) + points(1B) + data(points B)
            pkt = bytearray()
            pkt.extend(struct.pack(">I", ts))       # sec BE
            pkt.append((ms >> 8) & 0xFF)
            pkt.append(ms & 0xFF)                    # ms BE
            pkt.append(channel & 0xFF)               # channel
            pkt.append(num_points & 0xFF)            # point count
            pkt.extend(data_vals)                    # data bytes

            self._pulse_events.append(bytes(pkt))

    # ------------------------------------------------------------------
    # Pulse mode switching
    # ------------------------------------------------------------------

    def set_curve_mode(self, mode: AxlePulseMode) -> None:
        """Switch pulse generation on/off."""
        self.curve_mode = mode

    def toggle_curve_mode(self) -> AxlePulseMode:
        """Toggle pulse mode; returns the new mode."""
        new_mode = (AxlePulseMode.PULSE
                    if self.curve_mode == AxlePulseMode.REALTIME_ONLY
                    else AxlePulseMode.REALTIME_ONLY)
        self.curve_mode = new_mode
        return new_mode

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def status_line(self) -> str:
        """Single-line status for CLI display."""
        unread = len(self._pulse_events)
        return (f"  [{self.address:3d}] 0x{self.profile.module_type:02X} "
                f"AxleCounter "
                f"req={self.request_count} err={self.error_count} "
                f"pulses={unread} confirm=0x{self._confirm_code:04X}")
