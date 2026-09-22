"""
Modbus Slave Simulator — main entry point and Simulator class.

Launches a multi-device Modbus RTU slave simulator on a serial port.
Devices are configured via a JSON config file.  The simulator runs
in a background thread, dispatching incoming requests to the matching
SlaveDevice and writing responses back.

Usage:
    python -m modbus_slave_simulator              # with default config.json
    python -m modbus_slave_simulator -c my.json   # custom config
    python -m modbus_slave_simulator --list-ports  # list serial ports
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from typing import Optional

from serial_io import SerialPort
from slave_device import SlaveDevice
from module_profiles import (get_module_name,
                             FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION,
                             AXLE_COUNTER_TYPES)
from cli import SimulatorCLI


# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------


class Simulator:
    """
    Top-level simulator — owns the serial port and all slave devices.

    The ``run()`` method starts a background RX thread and optionally drops
    into an interactive CLI.  A foreground monitor thread prints periodic
    statistics.
    """

    def __init__(self, config_path: str = "config.json") -> None:
        """
        Args:
            config_path: Path to the JSON configuration file.
        """
        self._config_path = config_path
        self._config: dict = {}
        self._devices: dict[int, SlaveDevice] = {}
        self._serial: Optional[SerialPort] = None

        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._cli: Optional[SimulatorCLI] = None

        # Statistics
        self._start_time = 0.0
        self._total_rx = 0
        self._total_tx = 0

        # Load config
        self._load_config()

    # ------------------------------------------------------------------
    # Config loading
    # ------------------------------------------------------------------

    def _load_config(self) -> None:
        """Load and validate the JSON configuration."""
        # Find config file relative to this file if not absolute
        path = self._config_path
        if not os.path.isabs(path):
            # Try relative to CWD first, then relative to this file's directory
            if not os.path.exists(path):
                alt = os.path.join(os.path.dirname(__file__), path)
                if os.path.exists(alt):
                    path = alt

        if not os.path.exists(path):
            print(f"ERROR: Config file not found: {path}")
            print("  Use --example-config to generate a default config.json")
            sys.exit(1)

        with open(path, "r", encoding="utf-8") as f:
            self._config = json.load(f)

        print(f"[CONFIG] Loaded: {path}")
        self._validate_config()

    def _validate_config(self) -> None:
        """Basic config validation."""
        # Check serial section
        serial_cfg = self._config.get("serial", {})
        if not serial_cfg.get("port"):
            print("WARNING: No serial port specified.  Use --list-ports to see available ports.")
            print("  Defaulting to COM5 — edit config.json to change.")

        # Check devices
        devices = self._config.get("devices", [])
        if not devices:
            print("WARNING: No devices configured.  Use the default config or add entries.")

        seen_addrs: set[int] = set()
        for d in devices:
            addr = d.get("address", 0)
            if addr in seen_addrs:
                print(f"WARNING: Duplicate slave address {addr} in config!")
            seen_addrs.add(addr)

    # ------------------------------------------------------------------
    # Device creation
    # ------------------------------------------------------------------

    def _create_devices(self) -> None:
        """Instantiate all SlaveDevice instances from config."""
        sim_cfg = self._config.get("simulation", {})

        for dev_cfg in self._config.get("devices", []):
            addr = dev_cfg["address"]
            mtype = dev_cfg["module_type"]

            device = SlaveDevice(
                address=addr,
                module_type=mtype,
                curve_mode=dev_cfg.get("curve_mode", "realtime"),
                curve_frames_per_cycle=dev_cfg.get(
                    "curve_frames_per_cycle",
                    sim_cfg.get("default_curve_frames_per_cycle", 8),
                ),
                curve_points_per_frame=dev_cfg.get(
                    "curve_points_per_frame",
                    sim_cfg.get("default_curve_points_per_frame", 25),
                ),
                allow_extended_read=dev_cfg.get("allow_extended_read", False),
                alarm_probability=sim_cfg.get("alarm_probability", 0.02),
                sine_period_s=sim_cfg.get("sine_period_seconds", 10.0),
                random_walk_step=sim_cfg.get("random_walk_step", 5.0),
                # 计轴脉冲参数 (手动发送, 250ms 平滑随机脉冲 / 10V 峰值,
                # 一趟过车脉冲按 1Hz 依次递增)
                pulse_amplitude_raw=dev_cfg.get(
                    "pulse_amplitude_raw", sim_cfg.get("pulse_amplitude_raw", 100)),
                pulse_width_ms=dev_cfg.get(
                    "pulse_width_ms", sim_cfg.get("pulse_width_ms", 250)),
                pulse_sample_ms=dev_cfg.get(
                    "pulse_sample_ms", sim_cfg.get("pulse_sample_ms", 4)),
                pulse_interval=dev_cfg.get(
                    "pulse_interval", sim_cfg.get("pulse_interval", 1.0)),
                channel_voltage_raw=dev_cfg.get(
                    "channel_voltage_raw", sim_cfg.get("channel_voltage_raw", 100)),
                channel_current_raw=dev_cfg.get(
                    "channel_current_raw", sim_cfg.get("channel_current_raw", 200)),
                num_channels=dev_cfg.get(
                    "num_channels", sim_cfg.get("num_channels", 3)),
                pulses_per_passage=dev_cfg.get(
                    "pulses_per_passage",
                    sim_cfg.get("pulses_per_passage", 4)),
            )
            self._devices[addr] = device
            print(f"  [DEVICE] {device.status_line()}")

        print(f"  Total: {len(self._devices)} device(s) configured.")

    # ------------------------------------------------------------------
    # Run loop
    # ------------------------------------------------------------------

    def run(self, interactive: bool = True) -> None:
        """
        Start the simulator.

        Args:
            interactive: If True, drop into CLI after startup.
        """
        self._create_devices()

        # Open serial port
        serial_cfg = self._config.get("serial", {})
        self._serial = SerialPort(
            port=serial_cfg.get("port", "COM5"),
            baudrate=serial_cfg.get("baudrate", 19200),
            bytesize=serial_cfg.get("bytesize", 8),
            parity=serial_cfg.get("parity", "N"),
            stopbits=serial_cfg.get("stopbits", 1),
            timeout=serial_cfg.get("timeout", 0.005),
            rs485_delay_ms=serial_cfg.get("rs485_delay_ms", 1),
            verbose=True,
        )

        try:
            self._serial.open()
        except Exception as e:
            print(f"ERROR: Cannot open serial port: {e}")
            print("  Check that the port exists and is not in use by another application.")
            print("  Available ports:", SerialPort.list_ports())
            sys.exit(1)

        # Start threads
        self._running = True
        self._start_time = time.monotonic()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name="rx-thread"
        )
        self._rx_thread.start()

        print(f"\n{'='*60}")
        print(f"  Modbus Slave Simulator — RUNNING")
        print(f"  Port: {self._serial.port_name} @ {self._serial.baudrate}")
        print(f"  {len(self._devices)} device(s) waiting for requests...")
        print(f"{'='*60}\n")

        if interactive:
            self._run_cli()
        else:
            # Just keep running; handle Ctrl+C
            try:
                while self._running:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                self.stop()

    def stop(self) -> None:
        """Gracefully shut down the simulator."""
        print("\n[STOP] Shutting down...")
        self._running = False

        if self._serial:
            self._serial.close()

        # Print summary
        elapsed = time.monotonic() - self._start_time
        total_req = sum(d.request_count for d in self._devices.values())
        total_err = sum(d.error_count for d in self._devices.values())
        rx_frames = self._serial.rx_frame_count if self._serial else 0
        tx_frames = self._serial.tx_frame_count if self._serial else 0

        print(f"\n  --- Session Summary ---")
        print(f"  Duration:      {elapsed:.1f} s")
        print(f"  Total requests: {total_req}")
        print(f"  Total errors:   {total_err}")
        print(f"  RX frames:      {rx_frames}")
        print(f"  TX frames:      {tx_frames}")
        print(f"  Goodbye.\n")

    # ------------------------------------------------------------------
    # RX thread
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        """Background thread: read frames, dispatch, write responses."""
        while self._running:
            try:
                frame = self._serial.read_frame()  # type: ignore[union-attr]
            except Exception:
                time.sleep(0.001)
                continue

            if frame is None:
                time.sleep(0.001)
                continue

            if len(frame) < 4:
                continue

            # Dispatch: first byte is the slave address
            addr = frame[0]
            device = self._devices.get(addr)

            if device is not None and device.can_handle(frame):
                response = device.handle_request(frame)
                if response is not None:
                    # ── 先发送应答 (必须在100ms超时内完成, 打印日志不得阻塞此路径) ──
                    self._serial.write_frame(response)  # type: ignore[union-attr]

                    # ── 控制台数据交互流 (在发送之后打印, 不占用通信超时窗口) ──
                    module_name = get_module_name(device.module_type)
                    flag_info = ""
                    if device.module_type in AXLE_COUNTER_TYPES:
                        # 计轴模块 (0x50/0x53): 显示未读脉冲数
                        # 偏移随实时值组数变化 (0x50=28, 0x53=64)
                        off = device.unread_pulse_offset
                        if off is not None and len(response) > off + 1:
                            unread = (response[off] << 8) | response[off + 1]
                            flag_info = f" [未读脉冲={unread}]"
                    elif device.module_type in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
                        # 直流3对线: 显示 CurveFlag (resp offset 25-26)
                        if len(response) > 27:
                            cf = (response[25] << 8) | response[26]
                            flag_info = f" [曲线]" if cf == 0x80FF else f" [实时]"
                    elif device.is_curve_capable and len(response) > device.profile.curve_flag_offset:
                        flag_byte = response[device.profile.curve_flag_offset]
                        flag_names = {0x00: "实时帧", 0x55: "中间帧", 0xFF: "结束帧"}
                        flag_info = f" [{flag_names.get(flag_byte, f'0x{flag_byte:02X}')}]"

                    print(f"  [FLOW] {'─'*50}")
                    print(f"  [FLOW] 设备 #{device.request_count} | "
                          f"地址=0x{addr:02X} | 类型=0x{device.module_type:02X} "
                          f"{module_name}{flag_info}")
                    print(f"  [FLOW] 主机→从机: {frame.hex(' ')}")
                    print(f"  [FLOW] 从机→主机: {response.hex(' ')}")
                # else: CRC error or invalid request — real slave is silent
            # else: request for an unknown address — ignore (we're not that slave)

    # ------------------------------------------------------------------
    # CLI
    # ------------------------------------------------------------------

    def _run_cli(self) -> None:
        """Run the interactive command-line interface."""
        self._cli = SimulatorCLI(self)
        try:
            self._cli.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    # ------------------------------------------------------------------
    # Public accessors (used by CLI)
    # ------------------------------------------------------------------

    @property
    def devices(self) -> dict[int, SlaveDevice]:
        return self._devices

    @property
    def serial(self) -> Optional[SerialPort]:
        return self._serial

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def config(self) -> dict:
        return self._config


# ---------------------------------------------------------------------------
# CLI argument parsing
# ---------------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Modbus RTU Slave Simulator — simulate 14 module types",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m modbus_slave_simulator                        # default config.json
  python -m modbus_slave_simulator -c my_config.json       # custom config
  python -m modbus_slave_simulator --list-ports            # list serial ports
  python -m modbus_slave_simulator --example-config        # generate example config
        """,
    )
    parser.add_argument(
        "-c", "--config", default="config.json",
        help="Path to JSON configuration file (default: config.json)"
    )
    parser.add_argument(
        "--no-cli", action="store_true",
        help="Run without interactive CLI (Ctrl+C to stop)"
    )
    parser.add_argument(
        "--list-ports", action="store_true",
        help="List available serial ports and exit"
    )
    parser.add_argument(
        "--example-config", action="store_true",
        help="Print an example config.json and exit"
    )
    return parser.parse_args(argv)


def cmd_list_ports() -> None:
    """Print available serial ports."""
    print("Available serial ports:")
    ports = SerialPort.list_ports()
    if ports:
        for p in ports:
            print(f"  {p}")
    else:
        print("  (none found)")


def cmd_example_config() -> None:
    """Print an example config file."""
    example = {
        "serial": {
            "port": "COM5",
            "baudrate": 19200,
            "bytesize": 8,
            "parity": "N",
            "stopbits": 1,
            "timeout": 0.005,
            "rs485_delay_ms": 1,
        },
        "devices": [
            {"address": 1, "module_type": "0x31", "comment": "信号机开关量"},
            {"address": 2, "module_type": "0x48", "allow_extended_read": True,
             "comment": "高频轨道接收(133寄存器, 扩展读模式)"},
            {"address": 3, "module_type": "0x37", "curve_mode": "curve",
             "curve_frames_per_cycle": 8, "comment": "交流转辙机(带曲线)"},
            {"address": 4, "module_type": "0x50", "curve_mode": "curve",
             "pulse_width_ms": 250, "pulse_amplitude_raw": 100,
             "pulse_interval": 1.0, "pulses_per_passage": 4,
             "comment": "计轴采集传感器(低频, 手动发送曲线, 250ms平滑随机脉冲: 0V→10V→0V, 每过车4个同通道脉冲按1Hz递增, 三通道10V/20mA)"},
            {"address": 5, "module_type": "0x53", "curve_mode": "curve",
             "pulse_width_ms": 250, "pulse_amplitude_raw": 100,
             "pulse_interval": 1.0, "pulses_per_passage": 4,
             "comment": "高频计轴采集传感器(实时值4组reg4-27=reg4-9重复4次, 其余寄存器与0x50一致依次后移, 共32寄存器/64字节; 脉冲曲线部分与0x50相同)"},
        ],
        "simulation": {
            "alarm_probability": 0.02,
            "sine_period_seconds": 10.0,
            "random_walk_step": 5.0,
            "default_curve_points_per_frame": 25,
            "default_curve_frames_per_cycle": 8,
            "pulse_width_ms": 250,
            "pulse_sample_ms": 4,
            "pulse_amplitude_raw": 100,
            "pulse_interval": 1.0,
            "channel_voltage_raw": 100,
            "channel_current_raw": 200,
            "num_channels": 3,
            "pulses_per_passage": 4,
        },
    }
    print(json.dumps(example, indent=2, ensure_ascii=False))


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)

    if args.list_ports:
        cmd_list_ports()
        return

    if args.example_config:
        cmd_example_config()
        return

    sim = Simulator(config_path=args.config)
    sim.run(interactive=not args.no_cli)


if __name__ == "__main__":
    main()
