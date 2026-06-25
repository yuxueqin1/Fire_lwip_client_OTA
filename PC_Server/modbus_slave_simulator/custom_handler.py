"""
Custom serial protocol handler for curve modules.

Handles the non-Modbus protocol used by modules 0x37-0x39 and 0x4B-0x4D.
Two request types are recognised:

  1. Curve data request (8 bytes):
     [addr, 0x03, 0x00, 0x00, 0x00, frame_seq, CRC_lo, CRC_hi]

  2. Time-sync command (13 bytes):
     [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, ts[4] BE, CRC_lo, CRC_hi]

The response varies by frame flag:

  flag 0x00 — Normal realtime frame (no curve data)
  flag 0x55 — Intermediate curve frame
  flag 0xFF — End-of-curve frame (includes rotation direction word)

All byte offsets in the response are driven by the ModuleProfile table,
which mirrors the C firmware's uart_module_profiles[] exactly.
"""

import time as _time
from enum import Enum
from typing import Optional

from modbus_crc import modbus_crc16, verify_crc16, modbus_crc16_bytes
from module_profiles import ModuleProfile
from data_simulator import DataSimulator
from curve_generator import CurveGenerator


class CurveMode(Enum):
    """Curve transmission mode for a custom curve module."""
    REALTIME_ONLY = "realtime"   # Only send 0x00 normal frames (no curve)
    CURVE = "curve"              # Send 0x55 intermediate + 0xFF end frames


class CurveState(Enum):
    """Internal state of the curve transmission cycle."""
    IDLE = 0          # Waiting; next request will start a new cycle or be realtime
    TRANSMITTING = 1  # Sending 0x55 intermediate frames
    ENDING = 2        # About to send the 0xFF end frame


class CustomHandler:
    """
    Handles the custom serial protocol for one curve-capable slave device.

    In ``REALTIME_ONLY`` mode, every request gets a 0x00 normal frame response
    containing only realtime data.

    In ``CURVE`` mode, the handler runs a cycle:
        N-1 × 0x55 intermediate frames → 1 × 0xFF end frame → back to IDLE
    """

    def __init__(self, address: int, profile: ModuleProfile,
                 data_gen: DataSimulator,
                 curve_mode: CurveMode = CurveMode.REALTIME_ONLY,
                 curve_frames_per_cycle: int = 10,
                 curve_points_per_frame: int = 25) -> None:
        """
        Args:
            address: Slave device address (1-247).
            profile: Module profile (must have uses_custom_curve=1).
            data_gen: Data simulator instance.
            curve_mode: Whether to include curve data in responses.
            curve_frames_per_cycle: Number of 0x55 frames per curve cycle.
            curve_points_per_frame: Data points per branch per frame (≤ 25).
        """
        if not profile.uses_custom_curve:
            raise ValueError(
                f"CustomHandler requires uses_custom_curve=1, "
                f"got type 0x{profile.module_type:02X}"
            )
        if curve_points_per_frame > 25:
            raise ValueError("curve_points_per_frame must be ≤ 25")

        self.address = address
        self.profile = profile
        self.data_gen = data_gen
        self.curve_mode = curve_mode
        self.curve_frames_per_cycle = curve_frames_per_cycle
        self.points_per_frame = curve_points_per_frame

        # Total curve points = frames_per_cycle * points_per_frame
        self._total_curve_points = curve_frames_per_cycle * curve_points_per_frame

        # Curve generator — create fresh; reset() configures it
        self._curve_gen = CurveGenerator(
            branch_count=profile.curve_branch_count,
            points_per_frame=curve_points_per_frame,
            total_points=self._total_curve_points,
        )

        # State
        self._state: CurveState = CurveState.IDLE
        self._cycle_seq: int = 0        # sequence counter within current cycle

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
        Check if ``raw_bytes`` is for this device's custom protocol.

        Matches:
            - 8-byte curve request:  [addr, 0x03, 0x00, 0x00, 0x00, seq, CRC, CRC]
            - 13-byte time sync:     [addr, 0x10, 0x00, 0x00, 0x00, 0x02, ...]
        """
        if len(raw_bytes) < 4:
            return False
        if raw_bytes[0] != self.address:
            return False
        if raw_bytes[1] not in (0x03, 0x10):
            return False
        # Check frame length
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
        Parse the custom protocol request and build the response.

        Returns:
            Response bytes including CRC, or None if CRC is invalid.
        """
        self.last_request = raw_bytes
        self.request_count += 1

        # --- CRC validation ---
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
    # Time sync handler (0x10)
    # ------------------------------------------------------------------

    def _handle_time_sync(self, raw_bytes: bytes) -> bytes:
        """
        Handle the time-synchronisation command.

        Request: [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
                  ts_byte3, ts_byte2, ts_byte1, ts_byte0, CRC_lo, CRC_hi]

        The 4-byte timestamp is big-endian.  Reply with a standard Modbus
        0x10 echo: [addr, 0x10, 0x00, 0x00, 0x00, 0x02, CRC_lo, CRC_hi]
        """
        # Extract timestamp (big-endian)
        ts = ((raw_bytes[7] << 24) | (raw_bytes[8] << 16) |
              (raw_bytes[9] << 8)  | raw_bytes[10])
        self.data_gen.set_system_time(ts)

        # Build echo response
        resp = bytearray(raw_bytes[:6])  # [addr, 0x10, 0x00, 0x00, 0x00, 0x02]
        crc = modbus_crc16(resp)
        resp.append(crc & 0xFF)
        resp.append((crc >> 8) & 0xFF)

        self.last_response = bytes(resp)
        return bytes(resp)

    # ------------------------------------------------------------------
    # Curve request handler (0x03)
    # ------------------------------------------------------------------

    def _handle_curve_request(self, raw_bytes: bytes) -> bytes:
        """
        Handle the 8-byte curve data request.

        Determines the frame type (0x00, 0x55, 0xFF) based on curve_mode
        and internal state, then builds the appropriate response frame.
        """
        seq_from_master = raw_bytes[5]  # frame_sequence byte from master

        if self.curve_mode == CurveMode.REALTIME_ONLY:
            resp = self._build_normal_frame(seq_from_master)
        else:
            resp = self._build_curve_frame(seq_from_master)

        self.last_response = resp
        return resp

    # ------------------------------------------------------------------
    # Frame builders
    # ------------------------------------------------------------------

    def _build_normal_frame(self, seq: int) -> bytes:
        """
        Build a 0x00 normal realtime frame.

        Contains only realtime data (no curve data).  The frame is
        shorter — it ends just before the curve data area.
        """
        p = self.profile
        frame_end = p.curve_data_offset  # stops before curve data

        buf = bytearray(frame_end + 2)  # +2 for CRC
        buf[0] = self.address
        buf[1] = 0x03
        buf[2] = (frame_end - 3) & 0xFF  # byte_count

        # Realtime data (starts at offset 3)
        rt_bytes = self.data_gen.get_raw_bytes(p.realtime_bytes)
        buf[3 : 3 + len(rt_bytes)] = rt_bytes

        # Frame flag area
        buf[p.curve_flag_offset] = 0x00       # flag = normal
        buf[p.curve_flag_offset + 1] = 0x00   # flag+1

        # Timestamp (6 bytes at flag_offset+2 through flag_offset+7).
        # NOTE: curve_seq_offset == curve_flag_offset + 2 on most modules,
        # so timestamp[0] is intentionally shared with the frame-seq byte.
        ts_bytes = CurveGenerator.make_timestamp_bytes(
            self.data_gen.get_system_time()
        )
        ts_start = p.curve_flag_offset + 2
        for i, b in enumerate(ts_bytes):
            pos = ts_start + i
            if pos < frame_end:
                buf[pos] = b

        # Set seq byte AFTER timestamp — it overwrites timestamp[0] to
        # match the master's sequence number (same behaviour as real slave).
        buf[p.curve_seq_offset] = seq & 0xFF

        # CRC
        crc = modbus_crc16(buf[:frame_end])
        buf[frame_end] = crc & 0xFF
        buf[frame_end + 1] = (crc >> 8) & 0xFF

        return bytes(buf[:frame_end + 2])

    def _build_curve_frame(self, seq: int) -> bytes:
        """
        Build a 0x55 or 0xFF curve frame.

        Advances the curve cycle state machine.  The first request after IDLE
        starts a new cycle (0x55 frames), and the cycle ends with one 0xFF
        frame before returning to IDLE.
        """
        p = self.profile

        # --- State machine ---
        if self._state == CurveState.IDLE:
            # Start a new curve cycle
            self._state = CurveState.TRANSMITTING
            self._cycle_seq = 0
            self._curve_gen.reset()
            flag = 0x55
            is_first = True
        elif self._state == CurveState.TRANSMITTING:
            self._cycle_seq += 1
            is_first = False
            if self._cycle_seq >= self.curve_frames_per_cycle - 1:
                # Last intermediate frame → next will be 0xFF... but we need
                # one more request for that. However, we're building THIS frame,
                # and we haven't sent the 0xFF yet.  Let's check if the curve
                # generator is nearly done.
                pass
            # Check if we should transition to ENDING after this frame
            if self._curve_gen.frames_remaining <= 1:
                # After this frame, only the end frame remains → queue ENDING
                # But for THIS frame, still 0x55
                flag = 0x55
            else:
                flag = 0x55
            # After building this frame, if frames_remaining == 1 → next is ENDING
        elif self._state == CurveState.ENDING:
            flag = 0xFF
            is_first = False
            self._cycle_seq += 1
        else:
            flag = 0x00
            is_first = False

        # Generate curve data for this frame
        try:
            curve_data = self._curve_gen.next_frame_as_bytes()
        except StopIteration:
            # Cycle already complete — fall back to normal frame
            self._state = CurveState.IDLE
            return self._build_normal_frame(seq)

        # --- Build frame ---
        frame_end = p.curve_direction_offset + 2  # includes direction word
        buf = bytearray(frame_end + 2)  # +2 for CRC
        buf[0] = self.address
        buf[1] = 0x03
        buf[2] = (frame_end - 3) & 0xFF  # byte_count

        # Realtime data
        rt_bytes = self.data_gen.get_raw_bytes(p.realtime_bytes)
        buf[3 : 3 + len(rt_bytes)] = rt_bytes

        # Frame flag
        buf[p.curve_flag_offset] = flag
        buf[p.curve_flag_offset + 1] = 0x00 if is_first else 0xFF

        # Timestamp (6 bytes at flag_offset+2; first byte = seq, set later)
        ts_bytes = CurveGenerator.make_timestamp_bytes(
            self.data_gen.get_system_time()
        )
        ts_start = p.curve_flag_offset + 2
        for i, b in enumerate(ts_bytes):
            pos = ts_start + i
            if pos < p.curve_data_offset:  # don't overflow into curve data
                buf[pos] = b

        # Set seq byte AFTER timestamp to echo master's seq (overwrites timestamp[0])
        buf[p.curve_seq_offset] = seq & 0xFF

        # Curve length byte
        buf[p.curve_len_offset] = self.points_per_frame & 0xFF

        # Curve data (per-branch blocks, 50 bytes each)
        data_start = p.curve_data_offset
        data_end = data_start + len(curve_data)
        buf[data_start : data_end] = curve_data

        # Direction word (only for 0xFF end frame)
        if flag == 0xFF:
            direction = self._curve_gen.rotation_direction
            buf[p.curve_direction_offset] = (direction >> 8) & 0xFF
            buf[p.curve_direction_offset + 1] = direction & 0xFF
            # End of cycle → back to IDLE after this frame
            # (transition on NEXT request)
            self._state = CurveState.IDLE

        # --- Post-frame state update ---
        if flag == 0x55 and self._curve_gen.frames_remaining <= 1:
            self._state = CurveState.ENDING
        elif flag == 0x55:
            self._state = CurveState.TRANSMITTING

        # CRC
        crc = modbus_crc16(buf[:frame_end])
        buf[frame_end] = crc & 0xFF
        buf[frame_end + 1] = (crc >> 8) & 0xFF

        return bytes(buf[:frame_end + 2])

    # ------------------------------------------------------------------
    # Public helpers
    # ------------------------------------------------------------------

    def set_curve_mode(self, mode: CurveMode) -> None:
        """Switch between realtime-only and curve modes at runtime."""
        self.curve_mode = mode
        if mode == CurveMode.CURVE:
            # Reset state machine for a fresh cycle
            self._state = CurveState.IDLE
            self._cycle_seq = 0

    def toggle_curve_mode(self) -> CurveMode:
        """Toggle curve mode and return the new mode."""
        new_mode = (CurveMode.REALTIME_ONLY
                    if self.curve_mode == CurveMode.CURVE
                    else CurveMode.CURVE)
        self.set_curve_mode(new_mode)
        return new_mode

    @property
    def curve_state(self) -> CurveState:
        return self._state
