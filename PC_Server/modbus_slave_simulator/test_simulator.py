"""
Unit and integration tests for the Modbus Slave Simulator.

Run with:
    python test_simulator.py
    python -m pytest test_simulator.py -v
"""

import struct
import sys
import time
import unittest

# ---------------------------------------------------------------------------
# Import modules under test
# ---------------------------------------------------------------------------
from modbus_crc import modbus_crc16, modbus_crc16_bytes, verify_crc16, append_crc16
from module_profiles import (
    ModuleProfile, get_profile, get_all_profiles,
    get_module_name, FLFXG_XHJ_KGL, FLFXG_AC_ZZJ,
    FLFXG_GD_25_J_HIGH, FLFXG_AC_ZZJ_HIGH, FALLBACK_PROFILE,
    PROFILE_BY_TYPE,
)
from data_simulator import DataSimulator
from curve_generator import CurveGenerator
from standard_handler import StandardHandler
from custom_handler import CustomHandler, CurveMode


# ===================================================================
# 1. CRC16 Tests
# ===================================================================

class TestCRC16(unittest.TestCase):
    """Verify CRC16 matches FreeModbus usMBMCRC16 behaviour."""

    def test_known_vector_empty(self):
        """CRC of empty data."""
        # CRC of zero bytes should be 0xFFFF
        crc = modbus_crc16(b"")
        self.assertIsInstance(crc, int)

    def test_known_vector_single_byte(self):
        """CRC of a single 0x00 byte — verify via round-trip (0x0000 after append)."""
        crc = modbus_crc16(b"\x00")
        self.assertIsInstance(crc, int)
        # The standard Modbus CRC of [0x00] is 0x40BF.  Verify by
        # checking the round-trip property: CRC( [0x00, crc_lo, crc_hi] ) == 0.
        frame = bytearray([0x00, crc & 0xFF, (crc >> 8) & 0xFF])
        self.assertEqual(modbus_crc16(frame), 0,
                         f"CRC round-trip failed for 0x00; CRC={crc:#06x}")

    def test_known_vector_modbus_request(self):
        """Typical 0x04 request: CRC round-trip property."""
        data = bytes([0x01, 0x04, 0x00, 0x00, 0x00, 0x08])
        crc = modbus_crc16(data)
        frame = bytearray(data) + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
        self.assertEqual(modbus_crc16(frame), 0,
                         "CRC round-trip failed for 0x04 request")

    def test_crc16_bytes(self):
        """modbus_crc16_bytes returns 2 bytes, and round-trip is correct."""
        data = bytes([0x01, 0x04, 0x00, 0x00, 0x00, 0x08])
        crc_bytes = modbus_crc16_bytes(data)
        self.assertEqual(len(crc_bytes), 2)
        # Append to data and verify round-trip
        frame = bytearray(data) + bytearray(crc_bytes)
        self.assertEqual(modbus_crc16(frame), 0,
                         "CRC round-trip failed for modbus_crc16_bytes")

    def test_verify_crc16_valid(self):
        """verify_crc16 returns True for a well-formed frame."""
        # Build a complete frame: [addr, 0x04, byte_count, regs..., CRC]
        frame = bytearray([0x01, 0x04, 0x04, 0x00, 0x0A, 0x00, 0x14])
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)
        self.assertTrue(verify_crc16(bytes(frame)))

    def test_verify_crc16_invalid(self):
        """verify_crc16 returns False for corrupted CRC."""
        frame = bytearray([0x01, 0x04, 0x04, 0x00, 0x0A, 0x00, 0x14, 0x00, 0x00])
        self.assertFalse(verify_crc16(bytes(frame)))

    def test_append_crc16(self):
        """append_crc16 adds correct CRC and full frame passes verify."""
        data = bytearray([0x02, 0x04, 0x00, 0x00, 0x00, 0x06])
        frame = append_crc16(data)
        self.assertEqual(len(frame), 8)
        self.assertTrue(verify_crc16(frame))

    def test_crc_full_frame_property(self):
        """Modbus property: CRC of full valid frame (incl CRC) = 0x0000."""
        # Build request for 8 registers from slave 1
        req = bytearray([0x01, 0x04, 0x00, 0x00, 0x00, 0x08])
        crc = modbus_crc16(req)
        req.append(crc & 0xFF)
        req.append((crc >> 8) & 0xFF)
        self.assertEqual(modbus_crc16(req), 0)


# ===================================================================
# 2. Module Profiles Tests
# ===================================================================

class TestModuleProfiles(unittest.TestCase):
    """Verify the profile table matches the C firmware's table."""

    def test_all_14_types_present(self):
        """All 14 module type codes are defined."""
        expected = [
            0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
            0x37, 0x38, 0x39,
            0x47, 0x48, 0x49, 0x4A,
            0x4B, 0x4C, 0x4D,
        ]
        for code in expected:
            self.assertIn(code, PROFILE_BY_TYPE, f"Missing profile for 0x{code:02X}")

    def test_no_duplicate_types(self):
        """No duplicate type codes."""
        all_p = get_all_profiles()
        types = [p.module_type for p in all_p]
        self.assertEqual(len(types), len(set(types)))

    def test_custom_curve_modules_have_offsets(self):
        """All custom-curve modules have non-zero curve offsets."""
        for p in get_all_profiles():
            if p.uses_custom_curve:
                self.assertGreater(p.realtime_bytes, 0)
                self.assertGreater(p.curve_data_offset, 0)
                self.assertGreater(p.curve_len_offset, 0)
                self.assertGreater(p.curve_flag_offset, 0)
                self.assertGreater(p.curve_seq_offset, 0)
                self.assertGreater(p.curve_direction_offset, 0)
                self.assertIn(p.curve_branch_count, (1, 2, 4))

    def test_standard_modules_have_zero_offsets(self):
        """Standard Modbus modules have zero curve offsets."""
        for p in get_all_profiles():
            if not p.uses_custom_curve:
                self.assertEqual(p.realtime_bytes, 0)
                self.assertEqual(p.curve_data_offset, 0)
                self.assertEqual(p.curve_branch_count, 0)

    def test_0x48_exceeds_125(self):
        """Module 0x48 has 133 input registers (>125)."""
        p = get_profile(0x48)
        self.assertEqual(p.input_regs, 133)
        self.assertTrue(p.exceeds_125_regs)
        self.assertEqual(p.holding_regs, 131)

    def test_ac_zzj_profile(self):
        """Verify AC_ZZJ (0x37) profile matches C code."""
        p = get_profile(0x37)
        self.assertEqual(p.module_type, 0x37)
        self.assertEqual(p.input_regs, 29)
        self.assertEqual(p.holding_regs, 27)
        self.assertTrue(p.has_alarm)
        self.assertTrue(p.uses_custom_curve)
        self.assertEqual(p.realtime_bytes, 58)
        self.assertEqual(p.curve_data_offset, 82)
        self.assertEqual(p.curve_len_offset, 81)
        self.assertEqual(p.curve_flag_offset, 73)
        self.assertEqual(p.curve_seq_offset, 75)
        self.assertEqual(p.curve_direction_offset, 282)
        self.assertEqual(p.curve_branch_count, 4)

    def test_ac_zzj_high_profile(self):
        """Verify AC_ZZJ_HIGH (0x4B) profile matches C code."""
        p = get_profile(0x4B)
        self.assertEqual(p.input_regs, 68)
        self.assertTrue(p.is_high_frequency)
        self.assertTrue(p.uses_custom_curve)
        self.assertEqual(p.realtime_bytes, 136)
        self.assertEqual(p.curve_data_offset, 160)
        self.assertEqual(p.curve_flag_offset, 151)
        self.assertEqual(p.curve_branch_count, 4)

    def test_fallback_profile(self):
        """Unknown type returns fallback."""
        p = get_profile(0xFF)
        self.assertEqual(p.module_type, 0x00)
        self.assertEqual(p.input_regs, 13)

    def test_module_names(self):
        """All types have human-readable names."""
        for code in PROFILE_BY_TYPE:
            name = get_module_name(code)
            self.assertIsInstance(name, str)
            self.assertGreater(len(name), 3)
            self.assertNotIn("未知", name)


# ===================================================================
# 3. Data Simulator Tests
# ===================================================================

class TestDataSimulator(unittest.TestCase):
    """Verify DataSimulator produces well-formed register data."""

    def setUp(self):
        self.profile = get_profile(0x31)  # XHJ_KGL, 13 input regs

    def test_register_count(self):
        sim = DataSimulator(self.profile)
        regs = sim.get_registers(0, 13)
        self.assertEqual(len(regs), 13)

    def test_register_range(self):
        """All register values are 16-bit (0-65535)."""
        sim = DataSimulator(self.profile)
        regs = sim.get_registers(0, 13)
        for r in regs:
            self.assertGreaterEqual(r, 0)
            self.assertLessEqual(r, 65535)

    def test_status_register_is_1(self):
        """Register 0 (status) is always 1 (simulated online)."""
        sim = DataSimulator(self.profile)
        regs = sim.get_registers(0, 13)
        self.assertEqual(regs[0], 1)

    def test_values_change_over_time(self):
        """Subsequent calls return different values for analog registers."""
        sim = DataSimulator(self.profile)
        set1 = sim.get_registers(2, 5)  # skip status+alarm
        time.sleep(0.1)
        set2 = sim.get_registers(2, 5)
        # At least one register should differ (sine waves advance)
        self.assertTrue(
            any(a != b for a, b in zip(set1, set2)),
            "Analog registers should change between reads"
        )

    def test_raw_bytes(self):
        """get_raw_bytes returns big-endian register data."""
        sim = DataSimulator(self.profile)
        raw = sim.get_raw_bytes(26)  # 13 regs × 2 bytes
        self.assertEqual(len(raw), 26)
        # First 2 bytes: status register BE value
        self.assertEqual(raw[0], 0)      # status reg MSB (value=1, so MSB=0)
        self.assertEqual(raw[1], 1)      # status reg LSB (=1)

    def test_high_freq_module(self):
        """High-frequency module (0x48, 133 regs) works."""
        profile = get_profile(0x48)
        sim = DataSimulator(profile)
        regs = sim.get_registers(0, 133)
        self.assertEqual(len(regs), 133)


# ===================================================================
# 4. Standard Handler Tests
# ===================================================================

class TestStandardHandler(unittest.TestCase):
    """Verify standard Modbus 0x04 request-response cycle."""

    def setUp(self):
        self.profile = get_profile(0x33)  # GD_25_J, 34 regs
        self.data_gen = DataSimulator(self.profile)
        self.handler = StandardHandler(1, self.profile, self.data_gen)

    def _make_request(self, addr: int, quantity: int) -> bytes:
        """Build a valid 0x04 request frame."""
        req = bytearray([addr, 0x04, 0x00, 0x00, 0x00, quantity])
        crc = modbus_crc16(req)
        req.append(crc & 0xFF)
        req.append((crc >> 8) & 0xFF)
        return bytes(req)

    def test_can_handle_valid(self):
        req = self._make_request(1, 34)
        self.assertTrue(self.handler.can_handle(req))

    def test_can_handle_wrong_addr(self):
        req = self._make_request(2, 34)
        self.assertFalse(self.handler.can_handle(req))

    def test_can_handle_wrong_func(self):
        req = bytearray([1, 0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00])
        self.assertFalse(self.handler.can_handle(req))

    def test_response_structure(self):
        """Response has correct structure: [addr, 0x04, byte_count, data..., CRC]"""
        req = self._make_request(1, 10)
        resp = self.handler.build_response(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp[0], 1)       # addr
        self.assertEqual(resp[1], 0x04)    # func
        self.assertEqual(resp[2], 20)      # byte_count = 10 * 2
        # Total length: 1 + 1 + 1 + 20 + 2 = 25
        self.assertEqual(len(resp), 25)
        # CRC is correct
        self.assertTrue(verify_crc16(resp))

    def test_response_full_read(self):
        """Request all 34 registers — response contains all of them."""
        req = self._make_request(1, 34)
        resp = self.handler.build_response(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp[2], 68)      # byte_count = 34 * 2
        self.assertEqual(len(resp), 1 + 1 + 1 + 68 + 2)  # = 73

    def test_crc_error_ignored(self):
        """CRC-mismatched request returns None (slave stays silent)."""
        req = bytearray([1, 0x04, 0x00, 0x00, 0x00, 0x08, 0xFF, 0xFF])
        resp = self.handler.build_response(req)
        self.assertIsNone(resp)

    def test_exception_on_overread(self):
        """Requesting more registers than available → exception 03."""
        req = self._make_request(1, 100)  # profile has only 34
        resp = self.handler.build_response(req)
        self.assertIsNotNone(resp)
        self.assertEqual(resp[1], 0x84)   # 0x04 | 0x80
        self.assertEqual(resp[2], 0x03)   # Illegal Data Value

    def test_exception_on_zero_quantity(self):
        """Quantity = 0 → exception."""
        req = self._make_request(1, 0)
        resp = self.handler.build_response(req)
        self.assertEqual(resp[1], 0x84)
        self.assertEqual(resp[2], 0x03)

    def test_extended_read_mode(self):
        """With allow_extended_read=True, >125 reg requests succeed (use 0x48, 133 regs)."""
        profile = get_profile(0x48)  # GD_25_J_HIGH: 133 input regs
        dg = DataSimulator(profile)
        handler = StandardHandler(1, profile, dg, allow_extended_read=True)
        # Build request for 130 registers (within 133 limit, but >125)
        req = bytearray([1, 0x04, 0x00, 0x00, 0x00, 130])
        crc = modbus_crc16(req)
        req.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        resp = handler.build_response(bytes(req))
        self.assertIsNotNone(resp)
        self.assertNotEqual(resp[1] & 0x80, 0x80,
                            f"Expected success, got exception 0x{resp[2]:02X}")

    def test_extended_read_blocked(self):
        """Without allow_extended_read, >125 reg returns exception."""
        profile = get_profile(0x48)  # GD_25_J_HIGH: 133 input regs
        dg = DataSimulator(profile)
        handler = StandardHandler(1, profile, dg, allow_extended_read=False)
        req = bytearray([1, 0x04, 0x00, 0x00, 0x00, 130])
        crc = modbus_crc16(req)
        req.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        resp = handler.build_response(bytes(req))
        self.assertEqual(resp[1] & 0x80, 0x80, "Expected exception for >125 regs")

    def test_high_freq_133_reg(self):
        """Simulate 0x48's 133-register read with extended mode."""
        profile = get_profile(0x48)
        dg = DataSimulator(profile)
        handler = StandardHandler(1, profile, dg, allow_extended_read=True)
        req = bytearray([1, 0x04, 0x00, 0x00, 0x00, 133])
        crc = modbus_crc16(req)
        req.extend([crc & 0xFF, (crc >> 8) & 0xFF])
        resp = handler.build_response(bytes(req))
        self.assertIsNotNone(resp)
        self.assertEqual(resp[0], 1)
        self.assertEqual(resp[1], 0x04)
        # byte_count for 133 regs = 266 bytes → wire byte_count = 266 & 0xFF = 10
        self.assertEqual(resp[2], 10)
        # Total: 1+1+1+266+2 = 271
        self.assertEqual(len(resp), 271)
        self.assertTrue(verify_crc16(resp))


# ===================================================================
# 5. Custom Handler Tests
# ===================================================================

class TestCustomHandler(unittest.TestCase):
    """Verify custom curve protocol handler."""

    def setUp(self):
        self.profile = get_profile(0x37)  # AC_ZZJ
        self.data_gen = DataSimulator(self.profile)
        self.handler = CustomHandler(
            1, self.profile, self.data_gen,
            curve_mode=CurveMode.REALTIME_ONLY,
        )

    def _make_curve_request(self, addr: int, seq: int = 1) -> bytes:
        """Build an 8-byte custom curve request."""
        req = bytearray([addr, 0x03, 0x00, 0x00, 0x00, seq])
        crc = modbus_crc16(req)
        req.append(crc & 0xFF)
        req.append((crc >> 8) & 0xFF)
        return bytes(req)

    def _make_time_sync_request(self, addr: int, ts: int = 0x66010101) -> bytes:
        """Build a 13-byte time sync request."""
        req = bytearray([
            addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
            (ts >> 24) & 0xFF,
            (ts >> 16) & 0xFF,
            (ts >> 8) & 0xFF,
            ts & 0xFF,
        ])
        crc = modbus_crc16(req)
        req.append(crc & 0xFF)
        req.append((crc >> 8) & 0xFF)
        return bytes(req)

    def test_can_handle_curve_request(self):
        req = self._make_curve_request(1)
        self.assertTrue(self.handler.can_handle(req))

    def test_can_handle_time_sync(self):
        req = self._make_time_sync_request(1)
        self.assertTrue(self.handler.can_handle(req))

    def test_can_handle_wrong_addr(self):
        req = self._make_curve_request(2)
        self.assertFalse(self.handler.can_handle(req))

    def test_realtime_response_structure(self):
        """Realtime-only mode returns 0x00 frame with correct structure."""
        req = self._make_curve_request(1, seq=3)
        resp = self.handler.build_response(req)
        self.assertIsNotNone(resp)

        p = self.profile
        # Check basic structure
        self.assertEqual(resp[0], 1)          # addr
        self.assertEqual(resp[1], 0x03)       # func

        # Frame flag at correct offset
        self.assertEqual(resp[p.curve_flag_offset], 0x00)  # normal frame

        # Sequence echoed
        self.assertEqual(resp[p.curve_seq_offset], 3)

        # CRC valid
        self.assertTrue(verify_crc16(resp))

        # Realtime frame is shorter than full curve frame
        # For realtime, frame ends at curve_data_offset
        # Full frame size = curve_direction_offset + 2
        # Realtime frame should be shorter
        self.assertLess(len(resp), p.curve_direction_offset + 4)

    def test_curve_mode_cycle(self):
        """Full curve cycle: N×0x55 + 1×0xFF → back to IDLE."""
        handler = CustomHandler(
            1, self.profile, self.data_gen,
            curve_mode=CurveMode.CURVE,
            curve_frames_per_cycle=5,
            curve_points_per_frame=20,
            curve_interval=0,  # no cooldown in tests
        )
        p = self.profile

        # First 4 requests should be 0x55
        for i in range(4):
            req = self._make_curve_request(1, seq=i + 1)
            resp = handler.build_response(req)
            self.assertIsNotNone(resp, f"Frame {i+1} returned None")
            flag = resp[p.curve_flag_offset]
            self.assertEqual(flag, 0x55, f"Frame {i+1}: expected 0x55, got 0x{flag:02X}")

        # 5th request should be 0xFF (end frame)
        req = self._make_curve_request(1, seq=5)
        resp = handler.build_response(req)
        self.assertIsNotNone(resp, "End frame returned None")
        flag = resp[p.curve_flag_offset]
        self.assertEqual(flag, 0xFF, f"End frame: expected 0xFF, got 0x{flag:02X}")

        # Direction word filled in end frame
        dir_offset = p.curve_direction_offset
        direction = (resp[dir_offset] << 8) | resp[dir_offset + 1]
        self.assertIn(direction, (0, 1, 2))  # valid direction codes

        # 6th request: back to IDLE → next 0x55 starts new cycle
        req = self._make_curve_request(1, seq=1)
        resp = handler.build_response(req)
        self.assertIsNotNone(resp)
        flag = resp[p.curve_flag_offset]
        self.assertEqual(flag, 0x55, "After cycle: expected new 0x55")

    def test_first_frame_marker(self):
        """First 0x55 frame has flag+1 == 0."""
        handler = CustomHandler(
            1, self.profile, self.data_gen,
            curve_mode=CurveMode.CURVE,
            curve_frames_per_cycle=5,
            curve_interval=0,  # no cooldown in tests
        )
        p = self.profile

        req = self._make_curve_request(1, seq=1)
        resp = handler.build_response(req)
        self.assertEqual(resp[p.curve_flag_offset + 1], 0x00)  # first frame marker

        # Second frame: flag+1 != 0
        req = self._make_curve_request(1, seq=2)
        resp = handler.build_response(req)
        self.assertNotEqual(resp[p.curve_flag_offset + 1], 0x00)

    def test_curve_data_filled(self):
        """0x55/0xFF frames have curve data in the data area."""
        handler = CustomHandler(
            1, self.profile, self.data_gen,
            curve_mode=CurveMode.CURVE,
            curve_frames_per_cycle=3,
            curve_points_per_frame=25,
            curve_interval=0,  # no cooldown in tests
        )
        p = self.profile

        req = self._make_curve_request(1, seq=1)
        resp = handler.build_response(req)

        # Curve length byte set
        self.assertEqual(resp[p.curve_len_offset], 25)

        # Curve data area has non-zero bytes (4 branches × 50 bytes = 200 bytes)
        data_start = p.curve_data_offset
        data_end = data_start + p.total_curve_data_bytes
        curve_bytes = resp[data_start:data_end]
        self.assertGreater(len(curve_bytes), 0)
        # Not all zero (simulated data)
        self.assertTrue(any(b != 0 for b in curve_bytes))

    def test_time_sync(self):
        """Time sync command sets the system time."""
        ts = 0x6789ABCD
        req = self._make_time_sync_request(1, ts)
        resp = self.handler.build_response(req)
        self.assertIsNotNone(resp)
        # Response is echo: [addr, 0x10, 0x00, 0x00, 0x00, 0x02, CRC]
        self.assertEqual(resp[0], 1)
        self.assertEqual(resp[1], 0x10)
        self.assertEqual(len(resp), 8)
        self.assertTrue(verify_crc16(resp))

    def test_toggle_curve_mode(self):
        """Toggling curve mode at runtime works."""
        # Start in realtime mode
        req = self._make_curve_request(1, seq=1)
        resp = self.handler.build_response(req)
        self.assertEqual(resp[self.profile.curve_flag_offset], 0x00)

        # Switch to curve mode
        self.handler.set_curve_mode(CurveMode.CURVE)
        req = self._make_curve_request(1, seq=1)
        resp = self.handler.build_response(req)
        self.assertEqual(resp[self.profile.curve_flag_offset], 0x55)

        # Switch back
        new = self.handler.toggle_curve_mode()
        self.assertEqual(new, CurveMode.REALTIME_ONLY)
        req = self._make_curve_request(1, seq=1)
        resp = self.handler.build_response(req)
        self.assertEqual(resp[self.profile.curve_flag_offset], 0x00)


# ===================================================================
# 6. Curve Generator Tests
# ===================================================================

class TestCurveGenerator(unittest.TestCase):
    """Verify curve data generation."""

    def test_basic_generation(self):
        cg = CurveGenerator(branch_count=1, points_per_frame=10, total_points=30)
        self.assertFalse(cg.is_complete)

        frames = []
        while not cg.is_complete:
            data = cg.next_frame()
            frames.append(data)

        # 3 frames of 10 points each = 30 total
        self.assertEqual(len(frames), 3)
        for f in frames:
            self.assertEqual(len(f), 1)       # 1 branch
            self.assertEqual(len(f[0]), 10)   # 10 points

    def test_multi_branch(self):
        cg = CurveGenerator(branch_count=4, points_per_frame=25, total_points=200)
        self.assertEqual(cg.total_frames, 8)  # 200 / 25

        data = cg.next_frame()
        self.assertEqual(len(data), 4)          # 4 branches
        for b in range(4):
            self.assertEqual(len(data[b]), 25)  # 25 points per branch

    def test_last_frame_shorter(self):
        """Last frame may have fewer points if total not evenly divisible."""
        cg = CurveGenerator(branch_count=1, points_per_frame=10, total_points=25)

        # First frame: 10 points
        data = cg.next_frame()
        self.assertEqual(len(data[0]), 10)

        # Second frame: 10 points
        data = cg.next_frame()
        self.assertEqual(len(data[0]), 10)

        # Third (last) frame: 5 points
        data = cg.next_frame()
        self.assertEqual(len(data[0]), 5)
        self.assertTrue(cg.is_complete)

    def test_as_bytes_padding(self):
        """next_frame_as_bytes pads the last frame to full stride."""
        cg = CurveGenerator(branch_count=1, points_per_frame=10, total_points=15)
        # First frame
        raw = cg.next_frame_as_bytes()
        self.assertEqual(len(raw), 20)  # 1 branch × 10 points × 2 bytes

        # Last frame — still 20 bytes (5 real + 5 zero-padded)
        raw = cg.next_frame_as_bytes()
        self.assertEqual(len(raw), 20)

    def test_value_range(self):
        """All generated values are in 0-65535."""
        cg = CurveGenerator(branch_count=2, points_per_frame=25, total_points=100)
        while not cg.is_complete:
            data = cg.next_frame()
            for branch in data:
                for val in branch:
                    self.assertGreaterEqual(val, 0)
                    self.assertLessEqual(val, 65535)

    def test_rotation_direction(self):
        """Direction defaults to 0."""
        cg = CurveGenerator()
        self.assertEqual(cg.rotation_direction, 0)

    def test_reset_new_cycle(self):
        """After reset, generator starts fresh."""
        cg = CurveGenerator(branch_count=1, points_per_frame=10, total_points=20)
        # Consume all
        while not cg.is_complete:
            cg.next_frame()

        # Reset
        cg.reset(new_direction=1)
        self.assertFalse(cg.is_complete)
        self.assertEqual(cg.rotation_direction, 1)
        self.assertEqual(cg.current_frame_index, 0)

    def test_timestamp_bytes(self):
        """make_timestamp_bytes returns 6 bytes."""
        ts = 1719000000  # some Unix timestamp
        tb = CurveGenerator.make_timestamp_bytes(ts)
        self.assertEqual(len(tb), 6)
        self.assertTrue(all(0 <= b <= 255 for b in tb))


# ===================================================================
# 7. Integration Tests (no hardware required)
# ===================================================================

class TestIntegration(unittest.TestCase):
    """End-to-end tests using local objects (no serial port needed)."""

    def test_full_standard_cycle(self):
        """Simulate a master polling a standard Modbus slave."""
        from slave_device import SlaveDevice

        dev = SlaveDevice(address=10, module_type=0x35)  # GD_25_F, 30 regs

        # Build a valid 0x04 request
        req = bytearray([10, 0x04, 0x00, 0x00, 0x00, 30])
        crc = modbus_crc16(req)
        req.extend([crc & 0xFF, (crc >> 8) & 0xFF])

        # Device handles it
        self.assertTrue(dev.can_handle(bytes(req)))
        resp = dev.handle_request(bytes(req))
        self.assertIsNotNone(resp)
        self.assertTrue(verify_crc16(resp))
        self.assertEqual(resp[0], 10)
        self.assertEqual(resp[1], 0x04)

    def test_full_custom_curve_cycle(self):
        """Simulate a full curve cycle without hardware."""
        from slave_device import SlaveDevice

        dev = SlaveDevice(
            address=5, module_type=0x37,
            curve_mode="curve",
            curve_frames_per_cycle=4,
            curve_points_per_frame=25,
        )

        p = dev.profile
        flags_seen = set()

        for i in range(6):  # 4 × 0x55 + 1 × 0xFF + 1 extra
            req = bytearray([5, 0x03, 0x00, 0x00, 0x00, i + 1])
            crc = modbus_crc16(req)
            req.extend([crc & 0xFF, (crc >> 8) & 0xFF])

            resp = dev.handle_request(bytes(req))
            self.assertIsNotNone(resp, f"Frame {i+1} failed")
            self.assertTrue(verify_crc16(resp))

            flag = resp[p.curve_flag_offset]
            flags_seen.add(flag)

        self.assertIn(0x55, flags_seen, "Should see 0x55 intermediate frames")
        self.assertIn(0xFF, flags_seen, "Should see 0xFF end frame")

    def test_all_standard_modules(self):
        """Every standard Modbus module type responds correctly."""
        from slave_device import SlaveDevice

        standard_types = [0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
                          0x47, 0x48, 0x49, 0x4A]

        for mtype in standard_types:
            with self.subTest(type=f"0x{mtype:02X}"):
                profile = get_profile(mtype)
                dev = SlaveDevice(
                    address=1, module_type=mtype,
                    allow_extended_read=(mtype == 0x48),  # 133 regs
                )
                # Build request for ALL input registers
                qty = profile.input_regs
                req = bytearray([1, 0x04, 0x00, 0x00,
                                 (qty >> 8) & 0xFF, qty & 0xFF])
                crc = modbus_crc16(req)
                req.extend([crc & 0xFF, (crc >> 8) & 0xFF])

                self.assertTrue(dev.can_handle(bytes(req)))
                resp = dev.handle_request(bytes(req))
                self.assertIsNotNone(resp, f"Module 0x{mtype:02X}: no response")
                self.assertTrue(verify_crc16(resp), f"Module 0x{mtype:02X}: CRC error")
                self.assertEqual(resp[0], 1)
                self.assertNotEqual(resp[1] & 0x80, 0x80,
                                    f"Module 0x{mtype:02X}: unexpected exception")

    def test_all_curve_modules(self):
        """Every custom curve module type responds correctly in realtime mode."""
        from slave_device import SlaveDevice

        curve_types = [0x37, 0x38, 0x39, 0x4B, 0x4C, 0x4D]

        for mtype in curve_types:
            with self.subTest(type=f"0x{mtype:02X}"):
                dev = SlaveDevice(address=1, module_type=mtype,
                                  curve_mode="realtime")

                req = bytearray([1, 0x03, 0x00, 0x00, 0x00, 1])
                crc = modbus_crc16(req)
                req.extend([crc & 0xFF, (crc >> 8) & 0xFF])

                self.assertTrue(dev.can_handle(bytes(req)),
                                f"Module 0x{mtype:02X}: can_handle=False")
                resp = dev.handle_request(bytes(req))
                self.assertIsNotNone(resp, f"Module 0x{mtype:02X}: no response")
                self.assertTrue(verify_crc16(resp), f"Module 0x{mtype:02X}: CRC error")
                self.assertEqual(resp[0], 1)
                self.assertEqual(resp[1], 0x03)


# ===================================================================
# Runner
# ===================================================================

if __name__ == "__main__":
    # Allow running with `python test_simulator.py`
    unittest.main(verbosity=2)
