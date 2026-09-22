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
    FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION, AXLE_COUNTER_TYPES,
)
from data_simulator import DataSimulator
from standard_handler import StandardHandler
from custom_handler import CustomHandler, CurveMode
from axle_counter_handler import AxleCounterHandler, AxlePulseMode
from dc3wire_handler import DC3WireHandler, DC3WireCurveMode


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
                 pulse_amplitude_raw: int = 100,
                 pulse_width_ms: int = 250,
                 pulse_sample_ms: int = 4,
                 pulse_interval: float = 1.0,
                 channel_voltage_raw: int = 100,
                 channel_current_raw: int = 200,
                 num_channels: int = 3,
                 pulses_per_passage: int = 4) -> None:
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
            pulse_amplitude_raw: Axle pulse peak sample value in 0.1V
                (100 = 10V).
            pulse_width_ms: Nominal axle pulse window in ms (250, whole rise-
                fall curve; at 4 ms/sample = 63 samples).
            pulse_sample_ms: Time between axle pulse samples (4 ms).
            pulse_interval: Time (s) between consecutive pulses within one
                manually sent passage (1.0 = 1Hz axle rate); the passage's
                pulse timestamps are t0, t0+i*interval and the curve start
                time is t0 (first pulse).
            channel_voltage_raw: Axle channel voltage realtime value, 0.1V/LSB
                                 (100 = 10V).
            channel_current_raw: Axle channel current realtime value, 0.1mA/LSB
                                 (200 = 20mA).
            num_channels: Number of axle acquisition channels (3 → reg4-9).
            pulses_per_passage: Consecutive pulses sharing one channel before
                switching (one "axle passage" = one 0x09 curve group).
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

        self._handler: StandardHandler | CustomHandler | AxleCounterHandler | DC3WireHandler
        if module_type in AXLE_COUNTER_TYPES:
            # 计轴传感器 (0x50 低频 / 0x53 高频): 自定义 0x03/0x10 协议
            # (通讯确认码 + 脉冲); 高频只是实时值区重复 4 组。
            pulse = AxlePulseMode(curve_mode)
            self._handler = AxleCounterHandler(
                address=address,
                profile=self._profile,
                data_gen=self._data_gen,
                pulse_mode=pulse,
                pulse_amplitude_raw=pulse_amplitude_raw,
                pulse_width_ms=pulse_width_ms,
                pulse_sample_ms=pulse_sample_ms,
                pulse_interval=pulse_interval,
                channel_voltage_raw=channel_voltage_raw,
                channel_current_raw=channel_current_raw,
                num_channels=num_channels,
                pulses_per_passage=pulses_per_passage,
                # realtime_groups (实时值区组数) 由 handler 按 profile 自动推导
            )
        elif module_type in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
            # 直流3对线: 自定义 0x03/0x10 协议 (曲线序号ACK + 75点曲线)
            curve = DC3WireCurveMode(curve_mode)
            self._handler = DC3WireHandler(
                address=address,
                profile=self._profile,
                data_gen=self._data_gen,
                curve_mode=curve,
                realtime_bytes=self._profile.realtime_bytes,
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
        return (self._profile.is_custom_curve
                or self.module_type in AXLE_COUNTER_TYPES
                or self.module_type in (FLFXG_DC3WIRE_PLATFORM,
                                        FLFXG_DC3WIRE_STATION))

    @property
    def curve_mode(self) -> Optional[str]:
        """Current curve/pulse mode string, or None for standard modules."""
        if isinstance(self._handler, CustomHandler):
            return self._handler.curve_mode.value
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.curve_mode.value
        if isinstance(self._handler, DC3WireHandler):
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
        elif isinstance(self._handler, DC3WireHandler):
            self._handler.set_curve_mode(DC3WireCurveMode(mode))

    def toggle_curve_mode(self) -> str:
        """Toggle curve/pulse mode; returns the new mode string."""
        if isinstance(self._handler, CustomHandler):
            new = self._handler.toggle_curve_mode()
            return new.value
        if isinstance(self._handler, AxleCounterHandler):
            new = self._handler.toggle_curve_mode()
            return new.value
        if isinstance(self._handler, DC3WireHandler):
            new = self._handler.toggle_curve_mode()
            return new.value
        return "N/A"

    # ------------------------------------------------------------------
    # 计轴曲线手动发送 (按钮驱动, 不再 1Hz 自动生成)
    # ------------------------------------------------------------------

    def trigger_curve_send(self) -> bool:
        """Queue one full axle passage (curve) on the selected channel.

        Returns:
            True if a curve was queued; False if not an axle module, curve
            mode is off, or a previous batch is still unconfirmed.
        """
        if not isinstance(self._handler, AxleCounterHandler):
            return False
        return self._handler.trigger_curve_send()

    def cancel_curve_send(self) -> None:
        """Drop an in-flight curve batch without counting it (send timeout)."""
        if isinstance(self._handler, AxleCounterHandler):
            self._handler.cancel_curve_send()

    def set_curve_channel(self, channel: int) -> bool:
        """Select the channel for the next manually sent curve."""
        if not isinstance(self._handler, AxleCounterHandler):
            return False
        self._handler.set_curve_channel(channel)
        return True

    @property
    def current_channel(self) -> int:
        """Currently selected axle curve channel (0 if not an axle module)."""
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.current_channel
        return 0

    @property
    def curve_in_flight(self) -> bool:
        """True while a manually sent curve awaits master confirmation."""
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.curve_in_flight
        return False

    @property
    def curve_send_count(self) -> int:
        """Number of curves the master has confirmed receiving."""
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.curve_send_count
        return 0

    def pop_curve_confirmed(self) -> bool:
        """Return True (and clear) if the master just confirmed a curve."""
        if isinstance(self._handler, AxleCounterHandler):
            if self._handler.curve_confirmed_this_req:
                self._handler.reset_curve_confirmation()
                return True
        return False

    @property
    def unread_pulse_offset(self) -> Optional[int]:
        """剩余未读数寄存器在 0x03 应答帧中的字节偏移, 非计轴模块为 None.

        低频 0x50 = 28, 高频 0x53 = 64 (实时值 4 组把后续寄存器整体推后)。
        """
        if isinstance(self._handler, AxleCounterHandler):
            return self._handler.unread_offset
        return None

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
            mode_label = "pulse" if self.module_type in AXLE_COUNTER_TYPES else "curve"
            curve_info = f" [{self.curve_mode}]"
        flags = []
        if self._profile.is_high_frequency:
            flags.append("HF")
        if self._profile.exceeds_125_regs:
            flags.append(">125reg")
        if self.module_type in AXLE_COUNTER_TYPES:
            flags.append("AXLE")
        if self.module_type in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
            flags.append("DC3W")
        flag_str = f" ({', '.join(flags)})" if flags else ""
        return (f"  [{self.address:3d}] 0x{self.module_type:02X} "
                f"{name}{flag_str}{curve_info} "
                f"req={self.request_count} err={self.error_count}")

    def __repr__(self) -> str:
        return (f"SlaveDevice(addr={self.address}, "
                f"type=0x{self.module_type:02X}, "
                f"profile={self._profile})")
