"""
Register-level data simulator for Modbus slave devices.

Each DataSimulator instance maintains an independent state for one slave device,
producing realistic-looking register values over time.  Supports:

- Static registers (type code, version)
- Status registers (online bit, alarm bits with configurable probability)
- Analog registers (sine waves with configurable period/amplitude)
- Frequency registers (random walk around nominal)
- Phase / angle registers (0-360° mapped to 0-65535)
- Counter registers (monotonically incrementing)
"""

import math
import random
import time
from typing import Callable

from module_profiles import ModuleProfile


# ---------------------------------------------------------------------------
# Helper: individual register behaviours
# ---------------------------------------------------------------------------

class _RegisterGen:
    """Base class for a single register's value generator."""

    def get_value(self, t_now: float, tick: int) -> int:
        raise NotImplementedError


class _ConstReg(_RegisterGen):
    """Always returns the same value."""

    def __init__(self, value: int = 0) -> None:
        self._value = value

    def get_value(self, t_now: float, tick: int) -> int:
        return self._value


class _SineReg(_RegisterGen):
    """Sinusoidal register value (for analog signals)."""

    def __init__(self, amplitude: float = 1000.0, offset: float = 2000.0,
                 period_s: float = 10.0, phase: float = 0.0,
                 noise: float = 20.0) -> None:
        self._amplitude = amplitude
        self._offset = offset
        self._period_s = period_s
        self._phase = phase
        self._noise = noise

    def get_value(self, t_now: float, tick: int) -> int:
        val = (self._offset
               + self._amplitude * math.sin(2.0 * math.pi * t_now / self._period_s
                                             + self._phase))
        if self._noise > 0:
            val += random.gauss(0.0, self._noise)
        return max(0, min(65535, int(val)))


class _RandomWalkReg(_RegisterGen):
    """Random walk register (for frequency, temperature drift)."""

    def __init__(self, initial: float = 2000.0, step_size: float = 5.0,
                 min_val: float = 0.0, max_val: float = 65535.0) -> None:
        self._value = initial
        self._step = step_size
        self._min = min_val
        self._max = max_val
        self._last_tick = -1

    def get_value(self, t_now: float, tick: int) -> int:
        if tick != self._last_tick:
            self._value += random.uniform(-self._step, self._step)
            self._value = max(self._min, min(self._max, self._value))
            self._last_tick = tick
        return int(self._value)


class _CounterReg(_RegisterGen):
    """Monotonically incrementing counter."""

    def __init__(self, initial: int = 0, step: int = 1,
                 wrap_at: int = 65536) -> None:
        self._value = initial
        self._step = step
        self._wrap = wrap_at
        self._last_tick = -1

    def get_value(self, t_now: float, tick: int) -> int:
        if tick != self._last_tick:
            self._value = (self._value + self._step) % self._wrap
            self._last_tick = tick
        return self._value


class _AngleReg(_RegisterGen):
    """Angle register (0-360° mapped to 0-65535)."""

    def __init__(self, period_s: float = 20.0, phase: float = 0.0) -> None:
        self._period_s = period_s
        self._phase = phase

    def get_value(self, t_now: float, tick: int) -> int:
        angle = (t_now / self._period_s * 360.0 + self._phase) % 360.0
        return int(angle / 360.0 * 65535.0)


# ---------------------------------------------------------------------------
# DataSimulator
# ---------------------------------------------------------------------------


class DataSimulator:
    """
    Per-slave register data generator.

    Each call to :meth:`get_registers` advances an internal tick counter and
    returns the requested range of register values.  The values are
    deterministic enough to be repeatable but vary over time to look like real
    sensor data.
    """

    def __init__(self, profile: ModuleProfile,
                 alarm_probability: float = 0.02,
                 sine_period_s: float = 10.0,
                 random_walk_step: float = 5.0) -> None:
        """
        Args:
            profile: Module profile defining register count and type.
            alarm_probability: Probability (0.0-1.0) of alarm bits being set
                               on any given read (simulates intermittent faults).
            sine_period_s: Period in seconds for sine-wave registers.
            random_walk_step: Maximum step size for random-walk registers.
        """
        self._profile = profile
        self._alarm_prob = alarm_probability
        self._sine_period = sine_period_s
        self._rw_step = random_walk_step

        self._tick: int = 0
        self._t_start = time.monotonic()

        # Build generators for each register
        self._generators: list[_RegisterGen] = self._build_generators()

        # Manual overrides — user-editable register values
        self._manual_overrides: dict[int, int] = {}

        # System time (set by time-sync command for custom curve modules)
        self._system_time: int = int(time.time())

    # ------------------------------------------------------------------
    # Generator factory
    # ------------------------------------------------------------------

    def _build_generators(self) -> list[_RegisterGen]:
        """Create a generator per register, all initialised to 0.

        The user sets desired values manually via the register watch window.
        """
        n = self._profile.input_regs
        return [_ConstReg(0) for _ in range(n)]

    def _make_alarm_gen(self) -> _RegisterGen:
        """Generator for the alarm register (index 1)."""
        return _AlarmReg(self._alarm_prob)

    def _make_data_gen(self, idx: int, total: int) -> _RegisterGen:
        """Create a data generator for register ``idx``."""
        # Distribute register types based on position
        phase = idx * 0.7
        t = idx / max(total, 1)

        if t < 0.15:
            # Early registers: voltages — sine waves with moderate amplitude
            amp = 500.0 + idx * 80.0
            return _SineReg(amplitude=amp, offset=2200.0 + idx * 100.0,
                            period_s=self._sine_period, phase=phase, noise=15.0)
        elif t < 0.30:
            # Current registers — larger amplitude
            amp = 800.0 + idx * 50.0
            return _SineReg(amplitude=amp, offset=3000.0,
                            period_s=self._sine_period * 1.3, phase=phase, noise=30.0)
        elif t < 0.45:
            # Frequency registers — random walk around nominal
            nominal = 2500 if not self._profile.is_high_frequency else 5000
            return _RandomWalkReg(initial=nominal, step_size=self._rw_step,
                                  min_val=nominal - 500, max_val=nominal + 500)
        elif t < 0.60:
            # Phase / angle registers
            return _AngleReg(period_s=self._sine_period * 2.0, phase=phase)
        elif t < 0.80:
            # Power registers — larger sine waves
            return _SineReg(amplitude=1200.0, offset=4000.0 + idx * 200.0,
                            period_s=self._sine_period * 0.8, phase=phase, noise=40.0)
        else:
            # Counter / misc registers
            return _CounterReg(initial=idx * 10, step=1 + (idx % 5))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_system_time(self, unix_timestamp: int) -> None:
        """Update the simulated system time (from time-sync command)."""
        self._system_time = unix_timestamp

    def get_system_time(self) -> int:
        """Return the current simulated system time."""
        return self._system_time

    def get_registers(self, start: int, count: int) -> list[int]:
        """
        Return ``count`` register values starting from ``start``.

        Manual overrides take priority over auto-generated values.

        Args:
            start: First register index (0-based).
            count: Number of registers to return.

        Returns:
            List of 16-bit register values (0-65535).
        """
        tick = self._tick
        self._tick += 1
        t_now = time.monotonic() - self._t_start

        result = []
        for i in range(start, start + count):
            if i in self._manual_overrides:
                val = self._manual_overrides[i]
            elif i < len(self._generators):
                val = self._generators[i].get_value(t_now, tick)
            else:
                val = 0
            result.append(val & 0xFFFF)
        return result

    def peek_registers(self, start: int, count: int) -> list[int]:
        """
        Return ``count`` register values WITHOUT advancing the internal tick.

        Use this for UI display — does not consume register values that
        should go to the next request handler.

        Args:
            start: First register index (0-based).
            count: Number of registers to return.

        Returns:
            List of 16-bit register values (0-65535).
        """
        t_now = time.monotonic() - self._t_start

        result = []
        for i in range(start, start + count):
            if i in self._manual_overrides:
                val = self._manual_overrides[i]
            elif i < len(self._generators):
                val = self._generators[i].get_value(t_now, self._tick)
            else:
                val = 0
            result.append(val & 0xFFFF)
        return result

    def set_register_override(self, index: int, value: int) -> None:
        """Manually override a register value.

        Once set, the override persists until cleared, and takes priority
        over auto-generated values in both ``get_registers()`` and
        ``peek_registers()``.

        Args:
            index: Register index (0-based).
            value: New register value (0-65535).
        """
        self._manual_overrides[index] = value & 0xFFFF

    def clear_register_override(self, index: int) -> None:
        """Remove a manual override, restoring auto-generated values."""
        self._manual_overrides.pop(index, None)

    def clear_all_overrides(self) -> None:
        """Remove all manual overrides."""
        self._manual_overrides.clear()

    def has_override(self, index: int) -> bool:
        """Check if a register has a manual override."""
        return index in self._manual_overrides

    def get_raw_bytes(self, count: int) -> bytes:
        """
        Return simulated realtime data as raw bytes (big-endian register values).

        For custom curve modules, this produces the realtime section of the
        response frame.  ``count`` should equal ``profile.realtime_bytes``.

        Args:
            count: Number of bytes to produce.

        Returns:
            Big-endian byte representation of the simulated registers.
        """
        num_regs = (count + 1) // 2
        regs = self.get_registers(0, num_regs)
        buf = bytearray()
        for r in regs:
            buf.append((r >> 8) & 0xFF)
            buf.append(r & 0xFF)
        return bytes(buf[:count])

    def get_status_reg(self) -> int:
        """Return the status register value (register 0)."""
        return self._generators[0].get_value(0, self._tick) if self._generators else 1

    @property
    def tick(self) -> int:
        return self._tick


class _AlarmReg(_RegisterGen):
    """Alarm register generator with configurable probability."""

    def __init__(self, probability: float) -> None:
        self._prob = probability
        self._last_tick = -1
        self._value: int = 0

    def get_value(self, t_now: float, tick: int) -> int:
        if tick != self._last_tick:
            if random.random() < self._prob:
                # Set a random alarm bit
                bit = 1 << random.randint(0, 7)
                self._value |= bit
            else:
                self._value = 0
            self._last_tick = tick
        return self._value
