"""
Curve data generator for custom curve modules.

Simulates realistic action-current curves for railway switch machines
(转辙机动作电流曲线). Each curve simulates three phases:
  1. Start-up inrush  — High current with exponential decay
  2. Steady-state     — Moderate current with low-frequency oscillation
  3. Final transition — Small spike then settling

Each branch gets a phase offset so curves differ slightly between branches
(e.g., different cylinders in a switch machine).
"""

import math
import random
from typing import Sequence


class CurveGenerator:
    """
    Produces simulated action-current curve data for one switch machine.

    A full curve spans multiple "frames" (each frame = one response from the
    slave). Each frame contains ``points_per_frame`` data points per branch.
    The curve data is split across several intermediate frames (flag 0x55)
    followed by a final end frame (flag 0xFF).

    Usage::

        cg = CurveGenerator(branch_count=4, points_per_frame=25)
        while not cg.is_complete:
            frame_data = cg.next_frame()   # list[list[int]]
        direction = cg.rotation_direction  # 0 = normal

    After the cycle is complete, call :meth:`reset` to start a new cycle.
    """

    # Direction constants (matching the firmware convention)
    DIR_NORMAL     = 0   # Normal operation (定位→反位 or 反位→定位)
    DIR_OVERCURRENT = 1  # Overcurrent trip
    DIR_JAMMED     = 2   # Mechanical jam

    def __init__(self, branch_count: int = 4,
                 points_per_frame: int = 25,
                 total_points: int = 200) -> None:
        """
        Args:
            branch_count: Number of curve branches (1, 2, or 4).
            points_per_frame: Data points per branch per response frame.
                              Must be ≤ 25 to fit in the 50-byte branch stride.
            total_points: Total data points per branch in a complete curve.
        """
        if points_per_frame > 25:
            raise ValueError("points_per_frame must be ≤ 25 (50-byte branch stride)")
        self.branch_count = branch_count
        self.points_per_frame = points_per_frame
        self.total_points = total_points

        self._step: int = 0           # current frame index within cycle
        self._total_generated: int = 0  # cumulative points generated
        self._direction: int = self.DIR_NORMAL  # rotation direction for this cycle
        self._phase_per_branch: list[float] = [
            b * 0.6 for b in range(branch_count)
        ]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def is_complete(self) -> bool:
        """True when the current cycle has finished (all points emitted)."""
        return self._total_generated >= self.total_points

    @property
    def frames_remaining(self) -> int:
        """Number of frames remaining in the current cycle."""
        remaining_points = max(0, self.total_points - self._total_generated)
        return (remaining_points + self.points_per_frame - 1) // self.points_per_frame

    @property
    def total_frames(self) -> int:
        """Total number of frames in the current cycle (including this one)."""
        return (self.total_points + self.points_per_frame - 1) // self.points_per_frame

    @property
    def current_frame_index(self) -> int:
        """0-based index of the current frame within the cycle."""
        return self._step

    @property
    def rotation_direction(self) -> int:
        """Rotation direction word for the end frame."""
        return self._direction

    def next_frame(self) -> list[list[int]]:
        """
        Generate the next frame of curve data.

        Returns:
            A list of ``branch_count`` lists, each containing up to
            ``points_per_frame`` 16-bit integer values.

        Raises:
            StopIteration: If the cycle is already complete.
        """
        if self.is_complete:
            raise StopIteration("Curve cycle already complete")

        remaining = self.total_points - self._total_generated
        actual_points = min(self.points_per_frame, remaining)
        result: list[list[int]] = []

        for b in range(self.branch_count):
            branch_data: list[int] = []
            phase_offset = self._phase_per_branch[b]
            for p in range(actual_points):
                t = self._total_generated + p
                val = self._curve_value(t, phase_offset)
                branch_data.append(val & 0xFFFF)
            result.append(branch_data)

        self._step += 1
        self._total_generated += actual_points
        return result

    def next_frame_as_bytes(self) -> bytes:
        """
        Generate the next frame of curve data as raw bytes.

        Returns bytes in the wire format: per-branch blocks of
        ``points_per_frame * 2`` bytes each (big-endian uint16).

        For branches with fewer actual points (last frame), the remaining
        bytes are zero-padded to maintain the fixed stride.
        """
        frame_data = self.next_frame()
        buf = bytearray()
        stride = self.points_per_frame  # full stride per branch on the wire
        for b in range(self.branch_count):
            data = frame_data[b] if b < len(frame_data) else []
            for p in range(stride):
                if p < len(data):
                    val = data[p]
                else:
                    val = 0  # pad shorter frames with zeros
                buf.append((val >> 8) & 0xFF)
                buf.append(val & 0xFF)
        return bytes(buf)

    def reset(self, new_direction: int | None = None) -> None:
        """
        Reset for a new curve cycle.

        Args:
            new_direction: Rotation direction for the new cycle.
                           If None, toggles DIR_NORMAL / DIR_OVERCURRENT.
        """
        self._step = 0
        self._total_generated = 0
        if new_direction is not None:
            self._direction = new_direction
        else:
            # Toggle between normal and overcurrent for testing variety
            self._direction = (self.DIR_OVERCURRENT
                               if self._direction == self.DIR_NORMAL
                               else self.DIR_NORMAL)
        self._phase_per_branch = [
            b * 0.6 + random.uniform(-0.2, 0.2)
            for b in range(self.branch_count)
        ]

    # ------------------------------------------------------------------
    # Internal: curve value model
    # ------------------------------------------------------------------

    @staticmethod
    def _curve_value(t: int, phase_offset: float = 0.0) -> int:
        """Fixed curve data value — always 10 (user-configured)."""
        return 10

    # ------------------------------------------------------------------
    # Helpers for the CustomHandler
    # ------------------------------------------------------------------

    @staticmethod
    def make_timestamp_bytes(system_time: int) -> bytes:
        """
        Convert a Unix timestamp to the 5-byte packed BCD format used in curve
        frame headers.

        The firmware stores:
            [2] year/month  (BCD: 0xYYMM)  — actually stored in a compact form
            [3] day/hour    (BCD: 0xDDHH)
            [4] minute/second (BCD: 0xmmss)
            [5] second (repeated — for alignment/padding)

        For simplicity, we use a 6-byte representation matching what the
        C code expects at curve_flag_offset + 2:
            [0-1] reserved / zero (or extra timestamp)
            [2-5] 4 bytes of packed time, or simply 6 raw bytes

        Looking at the C code, the timestamp area at flag_offset+2 gets
        6 bytes from `flash_time` which is populated by the time-sync command.
        The exact format is whatever the module firmware writes.

        We replicate a reasonable BCD-like packing:
            byte 0: year - 2000 (e.g. 26 for 2026)
            byte 1: month (1-12)
            byte 2: day (1-31)
            byte 3: hour (0-23)
            byte 4: minute (0-59)
            byte 5: second (0-59)
        """
        import datetime
        dt = datetime.datetime.fromtimestamp(system_time)
        return bytes([
            dt.year - 2000,
            dt.month,
            dt.day,
            dt.hour,
            dt.minute,
            dt.second,
        ])
