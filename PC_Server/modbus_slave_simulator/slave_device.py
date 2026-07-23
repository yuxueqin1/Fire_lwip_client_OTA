"""
SlaveDevice — a single simulated Modbus / custom-curve slave device.

Each instance binds to one slave address and owns:
  - A ModuleProfile (static config from C code)
  - A DataSimulator (generates realistic register values)
  - A ProtocolHandler (StandardHandler or CustomHandler)

The device can be hot-reconfigured at runtime (e.g. toggling curve mode).
"""

from typing import Optional

from module_profiles import (
    ModuleProfile, get_profile, get_module_name, FALLBACK_PROFILE,
    FLFXG_AXLE_COUNTER,
)
from data_simulator import DataSimulator
from standard_handler import StandardHandler
from custom_handler import CustomHandler, CurveMode
from axle_counter_handler import AxleCounterHandler, AxlePulseMode


class SlaveDevice:
    """
    One simulated slave device on the RS485 bus.

    Usage::

        dev = SlaveDevice(address=1, module_type=0x31)
        # ... when a request arrives ...
        if dev.can_handle(raw_bytes):
            response = dev.handle_request(raw_bytes)
            serial_port.write(response)
    """

    def __init__(self, address: int, module_type: int, *,
                 curve_mode: str = "realtime",
                 curve_frames_per_cycle: int = 10,
                 curve_points_per_frame: int = 25,
                 allow_extended_read: bool = False,
                 alarm_probability: float = 0.02,
                 sine_period_s: float = 10.0,
                 random_walk_step: float = 5.0,
                 pulse_interval: float = 5.0) -> None:
        """
        Args:
            address: Slave address (1-247).
            module_type: Module type code, e.g. 0x31, "0x37", or 0x48.
            curve_mode: "realtime" (no curve) or "curve".
            curve_frames_per_cycle: Number of 0x55 frames per curve cycle.
            curve_points_per_frame: Data points per branch per frame (≤ 25).
            allow_extended_read: Accept >125 register reads (for high-freq modules).
            alarm_probability: Per-read alarm bit probability.
            sine_period_s: Period in seconds for sine-wave generators.
            random_walk_step: Step size for random-walk generators.
            pulse_interval: Seconds between simulated pulse events (axle counter only).
        """
        if not (1 <= address <= 247):
            raise ValueError(f"Invalid slave address: {address}")

        # Accept both int and hex strings
        if isinstance(module_type, str):
            module_type = int(module_type, 16) if module_type.startswith("0x") else int(module_type)

        self.address = address
        self.module_type = module_type
        self._profile = get_profile(module_type)

        if self._profile is FALLBACK_PROFILE and module_type != 0x00:
            print(f"  [WARN] Unknown module type 0x{module_type:02X}, using fallback profile")

        # Simulation parameters
        self._alarm_prob = alarm_probability
        self._sine_period = sine_period_s
        self._rw_step = random_walk_step

        # Build data simulator and handler
        self._data_gen = DataSimulator(
            self._profile,
            alarm_probability=alarm_probability,
            sine_period_s=sine_period_s,
            random_walk_step=random_walk_step,
        )

        self._handler: StandardHandler | CustomHandler | AxleCounterHandler
        if module_type == FLFXG_AXLE_COUNTER:
            # 计轴传感器: 自定义 0x03/0x10 协议 (通讯确认码 + 脉冲)
            pulse = AxlePulseMode(curve_mode)
            self._handler = AxleCounterHandler(
                address=address,
                profile=self._profile,
                data_gen=self._data_gen,
                pulse_mode=pulse,
                pulse_interval=pulse_interval,
            )
        elif self._profile.is_custom_curve:
            mode = CurveMode(curve_mode)
            self._handler = CustomHandler(
                address=address,
                profile=self._profile,
                data_gen=self._data_gen,
                curve_mode=mode,
                curve_frames_per_cycle=curve_frames_per_cycle,
                curve_points_per_frame=curve_points_per_frame,
            )
        else:
            self._handler = StandardHandler(
                address=address,
                profile=self._profile,
                data_gen=self._data_gen,
                allow_extended_read=allow_extended_read,
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def profile(self) -> ModuleProfile:
        return self._profile

    @property
    def is_curve_capable(self) -> bool:
        """True if this device type supports curve/pulse data."""
        return self._profile.is_custom_curve or self.module_type == FLFXG_AXLE_COUNTER

    @property
    def curve_mode(self) -> Optional[str]:
        """Current curve/pulse mode string, or None for standard modules."""
        if isinstance(self._handler, CustomHandler):
            return self._handler.curve_mode.value
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.curve_mode.value
        return None

    def can_handle(self, raw_bytes: bytes) -> bool:
        """Check if ``raw_bytes`` is a request for this device."""
        return self._handler.can_handle(raw_bytes)

    def handle_request(self, raw_bytes: bytes) -> Optional[bytes]:
        """
        Process an incoming request and return the response.

        Returns:
            Response bytes (including CRC), or None if the request was
            invalid (CRC error, etc.).
        """
        return self._handler.build_response(raw_bytes)

    # ------------------------------------------------------------------
    # Runtime reconfiguration
    # ------------------------------------------------------------------

    def set_register_value(self, index: int, value: int) -> None:
        """Manually set a register value (overrides auto-generation).

        Args:
            index: Register index (0-based).
            value: New register value (0-65535).
        """
        self._data_gen.set_register_override(index, value)

    def clear_register_value(self, index: int) -> None:
        """Remove a manual override, restoring auto-generated values."""
        self._data_gen.clear_register_override(index)

    def has_register_override(self, index: int) -> bool:
        """Check if a register has a manual override."""
        return self._data_gen.has_override(index)

    def peek_registers(self, count: int) -> list[int]:
        """Return register values without advancing the internal tick."""
        return self._data_gen.peek_registers(0, count)

    def set_curve_mode(self, mode: str) -> None:
        """
        Switch curve/pulse mode at runtime.

        Args:
            mode: "realtime" or "curve"
        """
        if isinstance(self._handler, CustomHandler):
            self._handler.set_curve_mode(CurveMode(mode))
        elif isinstance(self._handler, AxleCounterHandler):
            self._handler.set_curve_mode(AxlePulseMode(mode))

    def toggle_curve_mode(self) -> str:
        """Toggle curve/pulse mode; returns the new mode string."""
        if isinstance(self._handler, CustomHandler):
            new = self._handler.toggle_curve_mode()
            return new.value
        if isinstance(self._handler, AxleCounterHandler):
            new = self._handler.toggle_curve_mode()
            return new.value
        return "N/A"

    def set_alarm_probability(self, prob: float) -> None:
        """Update alarm probability at runtime."""
        self._alarm_prob = prob
        # Recreate data gen — simplest way to change behaviour
        self._data_gen = DataSimulator(
            self._profile,
            alarm_probability=prob,
            sine_period_s=self._sine_period,
            random_walk_step=self._rw_step,
        )

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def request_count(self) -> int:
        return self._handler.request_count

    @property
    def error_count(self) -> int:
        return self._handler.error_count

    @property
    def last_request(self) -> Optional[bytes]:
        return self._handler.last_request

    @property
    def last_response(self) -> Optional[bytes]:
        return self._handler.last_response

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------

    def status_line(self) -> str:
        """Single-line status summary."""
        name = get_module_name(self.module_type)
        curve_info = ""
        if self.is_curve_capable:
            mode_label = "pulse" if self.module_type == FLFXG_AXLE_COUNTER else "curve"
            curve_info = f" [{self.curve_mode}]"
        flags = []
        if self._profile.is_high_frequency:
            flags.append("HF")
        if self._profile.exceeds_125_regs:
            flags.append(">125reg")
        if self.module_type == FLFXG_AXLE_COUNTER:
            flags.append("AXLE")
        flag_str = f" ({', '.join(flags)})" if flags else ""
        return (f"  [{self.address:3d}] 0x{self.module_type:02X} "
                f"{name}{flag_str}{curve_info} "
                f"req={self.request_count} err={self.error_count}")

    def __repr__(self) -> str:
        return (f"SlaveDevice(addr={self.address}, "
                f"type=0x{self.module_type:02X}, "
                f"profile={self._profile})")
