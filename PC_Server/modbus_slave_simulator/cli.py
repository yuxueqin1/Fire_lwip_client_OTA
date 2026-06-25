"""
Interactive command-line interface for the Modbus Slave Simulator.

Commands:
  list          — Show all configured devices and their status
  watch <addr>  — Live-print register values for one device
  curve <addr>  — Toggle curve mode on/off for a custom-curve device
  hex <addr>    — Show the last raw request/response for one device
  stats         — Global statistics (frame counts, uptime, etc.)
  ports         — List available serial ports
  help          — Show this help
  quit / exit   — Exit the simulator
"""

import time
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import Simulator
    from slave_device import SlaveDevice


class SimulatorCLI:
    """
    Interactive console for the slave simulator.

    Runs in the main thread while the RX thread processes serial data
    in the background.
    """

    def __init__(self, simulator: "Simulator") -> None:
        self._sim = simulator
        self._watching: int | None = None  # address being watched
        self._watch_running = False
        self._watch_thread: threading.Thread | None = None

    def run(self) -> None:
        """Main CLI loop."""
        print("Type 'help' for available commands.\n")
        while self._sim.is_running:
            try:
                raw = input("sim> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not raw:
                continue
            self._dispatch(raw)

    def _dispatch(self, line: str) -> None:
        """Parse and execute a command."""
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ("quit", "exit", "q"):
            self._sim.stop()
            return
        elif cmd == "help" or cmd == "?":
            self._cmd_help()
        elif cmd == "list" or cmd == "ls":
            self._cmd_list()
        elif cmd == "watch" or cmd == "w":
            addr = int(parts[1]) if len(parts) > 1 else None
            self._cmd_watch(addr)
        elif cmd == "unwatch" or cmd == "uw":
            self._cmd_unwatch()
        elif cmd == "curve" or cmd == "cv":
            addr = int(parts[1]) if len(parts) > 1 else None
            self._cmd_curve(addr)
        elif cmd == "hex" or cmd == "hx":
            addr = int(parts[1]) if len(parts) > 1 else None
            self._cmd_hex(addr)
        elif cmd == "stats" or cmd == "st":
            self._cmd_stats()
        elif cmd == "ports":
            self._cmd_ports()
        else:
            print(f"  Unknown command: {cmd}.  Type 'help' for available commands.")

    # ------------------------------------------------------------------
    # Command implementations
    # ------------------------------------------------------------------

    def _cmd_help(self) -> None:
        print("""
  Commands:
    list | ls                  List all configured devices
    watch | w <addr>           Live-print register values for a device
    unwatch | uw               Stop watching
    curve | cv <addr>          Toggle curve mode (custom-curve devices only)
    hex | hx <addr>            Show last raw request / response in hex
    stats | st                 Global statistics
    ports                      List available serial ports
    help | ?                   Show this help
    quit | exit | q            Exit the simulator
        """.strip() + "\n")

    def _cmd_list(self) -> None:
        devices = self._sim.devices
        if not devices:
            print("  No devices configured.")
            return
        print(f"\n  {'='*65}")
        print(f"  {'Addr':>5s}  {'Type':6s}  {'Name':28s}  {'Mode':10s}  {'Req':>6s}  {'Err':>5s}")
        print(f"  {'-'*65}")
        for addr in sorted(devices):
            dev = devices[addr]
            p = dev.profile
            name = p.__class__.__module__.split(".")[0]  # not great, use get_module_name instead
            # Actually import the name function
            from module_profiles import get_module_name
            label = get_module_name(p.module_type)
            mode = dev.curve_mode or "modbus"
            flags = ""
            if p.is_high_frequency:
                flags += " HF"
            if p.exceeds_125_regs:
                flags += " >125"
            print(f"  {addr:5d}  0x{p.module_type:02X}   {label:28s}  {mode:10s}  "
                  f"{dev.request_count:6d}  {dev.error_count:5d}{flags}")
        print(f"  {'='*65}\n")

    def _cmd_watch(self, addr: int | None) -> None:
        if addr is None:
            print("  Usage: watch <address>")
            return
        dev = self._sim.devices.get(addr)
        if dev is None:
            print(f"  No device at address {addr}.")
            return

        self._cmd_unwatch()  # stop any previous watch
        self._watching = addr
        self._watch_running = True
        self._watch_thread = threading.Thread(
            target=self._watch_loop, args=(dev,), daemon=True
        )
        self._watch_thread.start()
        print(f"  Watching device {addr} (register values update at 1 Hz).")
        print(f"  Type 'unwatch' to stop.\n")

    def _cmd_unwatch(self) -> None:
        if self._watching is not None:
            self._watch_running = False
            self._watching = None
            print(f"  Stopped watching.")

    def _watch_loop(self, dev: "SlaveDevice") -> None:
        """Background thread: periodically print register values."""
        while self._watch_running and self._sim.is_running:
            # Read a snapshot of registers
            regs = dev._data_gen.get_registers(0, min(dev.profile.input_regs, 20))
            ts = time.strftime("%H:%M:%S")
            vals = " ".join(f"{r:5d}" for r in regs)
            print(f"  [{ts}] addr={dev.address}: {vals}")
            time.sleep(1.0)

    def _cmd_curve(self, addr: int | None) -> None:
        if addr is None:
            print("  Usage: curve <address>")
            return
        dev = self._sim.devices.get(addr)
        if dev is None:
            print(f"  No device at address {addr}.")
            return
        if not dev.is_curve_capable:
            print(f"  Device {addr} (0x{dev.module_type:02X}) is not a curve-capable module.")
            return
        new_mode = dev.toggle_curve_mode()
        print(f"  Device {addr}: curve mode → {new_mode}")

    def _cmd_hex(self, addr: int | None) -> None:
        if addr is None:
            print("  Usage: hex <address>")
            return
        dev = self._sim.devices.get(addr)
        if dev is None:
            print(f"  No device at address {addr}.")
            return

        req = dev.last_request
        resp = dev.last_response

        print(f"\n  --- Device {addr} (0x{dev.module_type:02X}) ---")
        if req:
            print(f"  Last request  ({len(req)} bytes):  {req.hex(' ')}")
        else:
            print(f"  Last request:  (none yet)")
        if resp:
            print(f"  Last response ({len(resp)} bytes):  {resp.hex(' ')}")
            # For curve modules, try to decode the flag byte
            if dev.is_curve_capable and len(resp) > dev.profile.curve_flag_offset:
                flag = resp[dev.profile.curve_flag_offset]
                flag_names = {0x00: "NORMAL", 0x55: "INTERMEDIATE", 0xFF: "END"}
                flag_name = flag_names.get(flag, f"UNKNOWN(0x{flag:02X})")
                seq = resp[dev.profile.curve_seq_offset] if len(resp) > dev.profile.curve_seq_offset else 0
                print(f"  → Frame flag: 0x{flag:02X} ({flag_name}), seq={seq}")
        else:
            print(f"  Last response: (none yet)")
        print()

    def _cmd_stats(self) -> None:
        ser = self._sim.serial
        devices = self._sim.devices

        total_req = sum(d.request_count for d in devices.values())
        total_err = sum(d.error_count for d in devices.values())

        print(f"\n  --- Simulator Statistics ---")
        print(f"  Devices:        {len(devices)}")
        print(f"  Total requests: {total_req}")
        print(f"  Total errors:   {total_err}")
        if ser:
            s = ser.stats()
            print(f"  Serial port:    {s['port']} @ {s['baudrate']}")
            print(f"  RX frames:      {s['rx_frames']}")
            print(f"  TX frames:      {s['tx_frames']}")
            print(f"  RX bytes:       {s['rx_bytes']}")
            print(f"  Frame gap:      {s['inter_frame_gap_ms']:.1f} ms")
        print()

    @staticmethod
    def _cmd_ports() -> None:
        from serial_io import SerialPort
        ports = SerialPort.list_ports()
        if ports:
            print("  Available serial ports:")
            for p in ports:
                print(f"    {p}")
        else:
            print("  No serial ports found.")
        print()
