"""
RS485 serial port I/O wrapper.

Provides non-blocking frame-level read/write over a half-duplex RS485 link.
Frame boundaries are detected using the Modbus RTU 3.5-character silence rule.

Works with both hardware RS485 adapters (USB-to-RS485 dongles) and virtual
null-modem pairs (com0com / socat) for testing.
"""

import threading
import time
from datetime import datetime
from typing import Callable, Optional

try:
    import serial
    import serial.tools.list_ports
    HAS_PYSERIAL = True
except ImportError:
    HAS_PYSERIAL = False
    serial = None  # type: ignore


# ---------------------------------------------------------------------------
# Frame reader
# ---------------------------------------------------------------------------


class SerialPort:
    """
    Wraps a pyserial ``Serial`` instance for RS485 half-duplex communication.

    Detects complete Modbus RTU frames by waiting for a 3.5-character silent
    interval after the last received byte.

    Usage::

        port = SerialPort("COM5", 19200)
        port.open()
        while True:
            frame = port.read_frame()
            if frame:
                response = dispatch(frame)
                port.write_frame(response)
    """

    # Inter-frame timeout parameters
    # 3.5 characters at typical baud rates:
    #   9600:   3.5 * 10 bits / 9600  ≈ 3.65 ms
    #   19200:  3.5 * 10 bits / 19200 ≈ 1.82 ms
    #   38400:  3.5 * 10 bits / 38400 ≈ 0.91 ms
    #   57600:  3.5 * 10 bits / 57600 ≈ 0.61 ms
    #
    # We use a minimum of 2 ms to account for OS scheduling jitter on Windows.

    def __init__(self, port: str, baudrate: int = 19200,
                 bytesize: int = 8, parity: str = 'N',
                 stopbits: int = 1,
                 timeout: float = 0.005,
                 rs485_delay_ms: float = 1.0,
                 verbose: bool = True) -> None:
        """
        Args:
            port: Serial port name, e.g. "COM5" or "/dev/ttyUSB0".
            baudrate: Baud rate (9600, 19200, 38400, 57600).
            bytesize: Data bits (7 or 8).
            parity: 'N', 'E', or 'O'.
            stopbits: Stop bits (1 or 2).
            timeout: pyserial read timeout in seconds.
            rs485_delay_ms: Extra delay after TX before RX (RS485 turnaround).
            verbose: If True, print hex dumps of all sent/received data.
        """
        if not HAS_PYSERIAL:
            raise ImportError(
                "pyserial is required. Install with: pip install pyserial"
            )

        self._port_name = port
        self._baudrate = baudrate
        self._bytesize = bytesize
        self._parity = parity
        self._stopbits = stopbits
        self._timeout = timeout
        self._rs485_delay = rs485_delay_ms / 1000.0

        self._ser: Optional[serial.Serial] = None
        self._running = False
        self._lock = threading.Lock()
        self._verbose = verbose

        # Frame reception buffer
        self._rx_buf = bytearray()
        self._last_byte_time = 0.0
        self._inter_frame_gap = self._calc_inter_frame_gap(baudrate)

        # Statistics
        self.rx_frame_count: int = 0
        self.tx_frame_count: int = 0
        self.rx_byte_count: int = 0
        self.crc_error_count: int = 0

    @staticmethod
    def _calc_inter_frame_gap(baudrate: int) -> float:
        """Calculate the 3.5-character silence timeout for a given baud rate."""
        # Each character = start(1) + data(8) + parity(0/1) + stop(1/2) ≈ 10-11 bits
        bits_per_char = 11.0  # conservative estimate
        char_time = bits_per_char / baudrate
        gap = 3.5 * char_time
        # Minimum 2 ms to avoid false splits on Windows
        return max(gap, 0.002)

    @staticmethod
    def _timestamp() -> str:
        """Return current time string for log lines."""
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    @staticmethod
    def _hex_dump(data: bytes, max_len: int = 128) -> str:
        """Format bytes as space-separated hex, truncating if too long."""
        if len(data) <= max_len:
            return data.hex(' ')
        return data[:max_len].hex(' ') + f" ... (+{len(data) - max_len}B)"

    # ------------------------------------------------------------------
    # Open / Close
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def open(self) -> None:
        """Open the serial port."""
        if self.is_open:
            return

        # Map parity string to pyserial constant
        parity_map = {'N': serial.PARITY_NONE,
                      'E': serial.PARITY_EVEN,
                      'O': serial.PARITY_ODD}

        print(f"  [SERIAL] 正在打开串口 {self._port_name} ...")
        self._ser = serial.Serial(
            port=self._port_name,
            baudrate=self._baudrate,
            bytesize=self._bytesize,
            parity=parity_map.get(self._parity, serial.PARITY_NONE),
            stopbits=self._stopbits,
            timeout=self._timeout,
        )
        self._running = True
        self._rx_buf.clear()

        # 验证串口是否真正打开
        if self._ser and self._ser.is_open:
            print(f"  [SERIAL] ✓ 串口已连接: {self._port_name}")
            print(f"           波特率: {self._baudrate} bps")
            print(f"           数据位: {self._bytesize}  校验: {self._parity}  停止位: {self._stopbits}")
            print(f"           帧间隔: {self._inter_frame_gap*1000:.1f} ms (3.5字符静默)")
            print(f"           等待主站请求数据 ...")
        else:
            print(f"  [SERIAL] ✗ 串口打开失败: {self._port_name}")

    def close(self) -> None:
        """Close the serial port."""
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
            print(f"  [SERIAL] 串口已关闭: {self._port_name}")

    # ------------------------------------------------------------------
    # Frame-level read
    # ------------------------------------------------------------------

    def read_frame(self, block: bool = True) -> Optional[bytes]:
        """
        Read one complete Modbus RTU / custom protocol frame.

        Args:
            block: If True, blocks up to ``timeout`` seconds waiting for the
                   first byte.  If False, returns None immediately when no
                   data is available.

        Returns:
            Complete frame when the 3.5-char inter-frame gap is detected,
            or None if no complete frame is available.
        """
        if not self.is_open or not self._ser:
            return None

        try:
            with self._lock:
                # 阻塞等待首字节 (如果缓存为空且 block=True)
                if not self._rx_buf and block:
                    chunk = self._ser.read(1)  # 最多阻塞 timeout=5ms
                    if chunk:
                        self._rx_buf.extend(chunk)
                        self._last_byte_time = time.monotonic()
                        self.rx_byte_count += 1

                # 读取所有可用字节
                available = self._ser.in_waiting
                if available > 0:
                    chunk = self._ser.read(available)
                    if chunk:
                        self._rx_buf.extend(chunk)
                        self._last_byte_time = time.monotonic()
                        self.rx_byte_count += len(chunk)

                frame = self._detect_frame()

            # ── 在锁外打印: print 很慢, 不能阻塞响应路径 ──
            if frame is not None and self._verbose:
                print(f"  [SERIAL] [{self._timestamp()}] "
                      f"← 接收 ({len(frame)}B): {self._hex_dump(frame)}")
            return frame
        except (serial.SerialException, OSError) as e:
            print(f"  [SERIAL] Read error: {e}")
            return None

    def read_frame_blocking(self, timeout_s: float = 0.2) -> Optional[bytes]:
        """
        Read one frame, blocking until a complete frame is received or
        ``timeout_s`` elapses.

        Returns:
            Complete frame or None on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            frame = self.read_frame()
            if frame is not None:
                return frame
            time.sleep(0.001)
        return None

    def _detect_frame(self) -> Optional[bytes]:
        """If the inter-frame gap has elapsed, return the accumulated bytes.

        Caller is responsible for verbose logging (must be done outside the lock).
        """
        if not self._rx_buf:
            return None

        elapsed = time.monotonic() - self._last_byte_time
        if elapsed >= self._inter_frame_gap:
            frame = bytes(self._rx_buf)
            self._rx_buf.clear()
            if len(frame) >= 4:  # Minimum valid Modbus frame
                self.rx_frame_count += 1
                return frame
            # Discard too-short fragments (noise) — silently
            return None
        return None

    def flush_input(self) -> None:
        """Discard any buffered input."""
        with self._lock:
            self._rx_buf.clear()
            if self._ser and self._ser.is_open:
                self._ser.reset_input_buffer()

    # ------------------------------------------------------------------
    # Frame-level write
    # ------------------------------------------------------------------

    def write_frame(self, data: bytes) -> None:
        """
        Write a complete response frame to the serial port.

        Includes RS485 turnaround delay after write to allow the bus to
        switch from TX to RX (important for half-duplex adapters).
        """
        if not self.is_open or not self._ser:
            return

        try:
            with self._lock:
                self._ser.write(data)
                self._ser.flush()
                self.tx_frame_count += 1
                # RS485 turnaround delay (must be inside lock — 总线方向切换优先于 print)
                if self._rs485_delay > 0:
                    time.sleep(self._rs485_delay)
            # ── 在锁外打印: print 很慢(10-30ms), 不能阻塞串口通信 ──
            if self._verbose:
                print(f"  [SERIAL] [{self._timestamp()}] "
                      f"→ 发送 ({len(data)}B): {self._hex_dump(data)}")
        except (serial.SerialException, OSError) as e:
            print(f"  [SERIAL] Write error: {e}")

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def list_ports() -> list[str]:
        """List available serial ports."""
        if not HAS_PYSERIAL:
            return []
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    @property
    def port_name(self) -> str:
        return self._port_name

    @property
    def baudrate(self) -> int:
        return self._baudrate

    def stats(self) -> dict:
        """Return a dict of port statistics."""
        return {
            "port": self._port_name,
            "baudrate": self._baudrate,
            "rx_frames": self.rx_frame_count,
            "tx_frames": self.tx_frame_count,
            "rx_bytes": self.rx_byte_count,
            "inter_frame_gap_ms": self._inter_frame_gap * 1000,
        }
