"""
Axle Counter custom protocol handler for module types 0x50 (低频) / 0x53 (高频).

Protocol (mirrors firmware's ``uart_axle_counter_*`` in main.c / port_axle_counter.c,
per ``Doc/计轴采集盒协议-通信板与采集盒.md``):

  Request (8 bytes):
    [addr, 0x03, confirm_hi, confirm_lo, count_hi, count_lo, CRC_lo, CRC_hi]
      - ``confirm`` = 通信板原样带回的上一应答 reg12 (水位 ACK), 首次为 0x0000
      - ``count``   = 要读取的脉冲个数:
            0x0000 → 状态查询 (应答只回实时值 + reg13, 不带脉冲块)
            N>0    → 读取请求 (应答附带最多 N 个脉冲的脉冲块)

  Response (no byte_count field!):
    [addr, 0x03,
     reg0(2B), reg1(2B), reg2-3(4B), reg4..reg10(14B),
     reg11(2B), reg12(2B), reg13(2B),      <- 28 bytes of register data
     pulse_block(optional),                <- only on 读取请求 (count>0)
     CRC_lo, CRC_hi]

  Register map (all big-endian):
    reg0      保留 / 模块状态
    reg1      KGL 报警寄存器 (低6位非0 → 正常; 低6位全0 → 报警)
    reg2-3    本组曲线开始时间 Unix 秒 (4B)
    reg4-9    三个通道电压/电流实时值, 电压在前电流在后:
              V1,V2,V3,I1,I2,I3 (不是交错的 V,I,V,I)
    reg10     保留
    reg11     本组曲线开始时间 Unix 毫秒
    reg12     通讯确认码 (每条应答递增, 水位 ACK)
    reg13     剩余未读脉冲数 (交付后剩余)

  高频变体 0x53: 实时值区重复 4 次 (reg4-27 = 4 × reg4-9 的内容), 其余寄存器
  含义不变但依次后移 —— reg28 保留, reg29 曲线开始毫秒, reg30 通讯确认码,
  reg31 剩余未读数 (寄存器块 64 字节)。脉冲块格式与低频完全一致。
  布局由 ``realtime_groups`` 参数决定 (0x50→1, 0x53→4), 所有寄存器索引在
  ``__init__`` 里按组数推导, 不写死。

  Pulse block:
    [本包上报脉冲个数(2B BE)]
    per pulse: [秒(4B BE)][毫秒(2B BE)][通道号(1B)][采样点数n(1B)][采集值×n(1B, 0.1V)]

  Time-sync (13 bytes):
    [addr, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
     ts[4] BE, CRC_lo, CRC_hi]

  通道分组 (协议 §6): 板上传上位机的 0x09 脉冲帧按通道拆帧 — 一帧 = 一个
  通道的一组曲线, 帧间共享同一曲线起始时间戳(reg2-3 + reg11). 因此模拟器
  把同一趟"过车"的连续脉冲(pulses_per_passage 个)生成在同一通道上,
  通信板才能组成完整的单通道曲线帧.

两步读取 + 水位 ACK 状态机 (spec section 5):
  1. 采集盒缓存未确认的脉冲, 每条 0x03 应答 reg12 递增一个序号。
  2. 状态查询 (count=0): 只回实时值 + reg12 + reg13(剩余未读数), 不带脉冲块。
  3. 读取请求 (count=N): 从队首取最多 N 个未读脉冲进应答脉冲块,
     并在脉冲上记录本次应答的确认码; reg13 = 交付后剩余未读数。
  4. 请求帧回显码 C → 清除所有"已发出且序号 ≤ C"的脉冲 (通信板已收到)。

发送方式 (按钮驱动, 不再 1Hz 自动生成): GUI 点"发送曲线" →
  trigger_curve_send() 在所选通道入队一趟过车 (pulses_per_passage 个脉冲);
  一趟内每个脉冲的时间戳按 pulse_interval(默认 1s = 1Hz) 依次递增
  (t0, t0+1s, t0+2s, t0+3s), 曲线起始时间戳 reg2-3+reg11 = 首脉冲 t0,
  通信板按 §6 公式把各脉冲铺到 0x09 帧的不同点偏移上。
  该批全部脉冲被通信板水位 ACK 后计为一条曲线发送成功
  (curve_send_count+1, curve_confirmed_this_req=True)。
  同一时间只允许一个未确认批次, 否则 trigger_curve_send() 返回 False。
"""

import math
import random
import struct
import time as _time
from enum import Enum
from typing import Optional

from modbus_crc import modbus_crc16, verify_crc16
from module_profiles import AXLE_REALTIME_GROUPS


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

    # 单包最多携带的脉冲数 (帧长上限保护)
    MAX_PULSES_PER_REQ = 0x30   # 48

    def __init__(self, address: int, profile, data_gen,
                 pulse_mode: AxlePulseMode = AxlePulseMode.REALTIME_ONLY,
                 max_unread_pulses: int = 50,
                 pulse_amplitude_raw: int = 100,
                 pulse_width_ms: int = 250,
                 pulse_sample_ms: int = 4,
                 pulse_interval: float = 1.0,
                 channel_voltage_raw: int = 100,
                 channel_current_raw: int = 200,
                 num_channels: int = 3,
                 pulses_per_passage: int = 4,
                 realtime_groups: Optional[int] = None) -> None:
        """
        Args:
            address: Slave device address (1-247).
            profile: ModuleProfile instance.
            data_gen: DataSimulator instance for register values.
            pulse_mode: Whether the device may carry pulse curves (the actual
                sending is button-driven via ``trigger_curve_send``, not a
                continuous 1 Hz auto-stream).
            max_unread_pulses: Max queued (un-ACKed) pulse count.
            pulse_amplitude_raw: Pulse peak sample value in 0.1V (100 = 10V).
            pulse_width_ms: Nominal pulse window in ms (250 = whole rise-fall
                            curve; at 4 ms/sample = 63 samples).
            pulse_sample_ms: Time between samples in the pulse curve (4 ms
                             matches the protocol's time-offset granularity).
            pulse_interval: Time (s) between consecutive pulses within one
                manually sent passage (1.0 = 1Hz axle rate). The passage's
                pulses get timestamps t0, t0+i*interval; curve start time
                reg2-3+reg11 = t0 (first pulse).
            channel_voltage_raw: Per-channel voltage realtime value (0.1V/LSB,
                                 100 = 10V).
            channel_current_raw: Per-channel current realtime value (0.1mA/LSB,
                                 200 = 20mA).
            num_channels: Number of acquisition channels (3 → reg4-9).
            pulses_per_passage: Consecutive pulses sharing one channel before
                switching to the next channel (one "axle passage" = one 0x09
                curve group per the protocol §6 per-channel frame splitting).
            realtime_groups: How many times the 6-register realtime block
                (V1,V2,V3,I1,I2,I3) repeats from reg4 on.
                  1 → 低频 0x50: reg4-9 实时值, reg10 保留, reg11 毫秒,
                      reg12 确认码, reg13 剩余未读数        (28 字节)
                  4 → 高频 0x53: reg4-27 实时值, reg28 保留, reg29 毫秒,
                      reg30 确认码, reg31 剩余未读数        (64 字节)
        """
        self.address = address
        self.profile = profile
        self.data_gen = data_gen

        # Pulse mode
        self.curve_mode = pulse_mode

        # 待确认队列: (sent_seq, pkt).  sent_seq=None → 尚未交付给通信板.
        # 已发出但未被水位 ACK 确认的脉冲保留在队列中, 以便丢帧后重发.
        self._max_unread = max_unread_pulses
        self._pulse_events: list[tuple[Optional[int], bytes]] = []
        # 手动发送 (按钮驱动, 不再 1Hz 自动生成): 每次点"发送曲线"入队一趟
        # 完整过车 (pulses_per_passage 个脉冲), 全部落在当前所选通道; 通信板
        # 读取并经水位 ACK 确认后才算一条曲线发送成功。
        self._pulses_per_passage = max(1, pulses_per_passage)
        self._selected_channel: int = 0    # 下一次曲线发送的通道 (0..num_channels-1)
        self._passage_index: int = 0       # 由 set_curve_channel 对齐到所选通道
        self._pulses_in_passage: int = 0
        self._in_flight: bool = False      # 是否有未确认的曲线批次
        self._batch_remaining: int = 0     # 当前批次尚未被通信板 ACK 的脉冲数
        self._curve_send_count: int = 0    # 已被通信板确认接收的曲线数
        self._curve_confirmed_this_req: bool = False  # 本次请求是否刚确认一批

        # Pulse curve shape: ~250ms smooth random bump at 10V, 1Hz axle rate.
        # 波形在 _generate_pulse_samples() 里按脉冲生成 (随机边沿, 每次略不同).
        self._pulse_amplitude_raw = pulse_amplitude_raw
        self._pulse_width_ms = pulse_width_ms
        self._pulse_sample_ms = pulse_sample_ms
        self._pulse_num_points = max(1, (pulse_width_ms + pulse_sample_ms - 1)
                                     // pulse_sample_ms)   # ceil → 63 @4ms/250ms
        # 同一趟过车内相邻脉冲的时间间隔 (秒) — 时间戳按此递增, 1.0 = 1Hz
        self._pulse_interval = max(0.0, pulse_interval)

        # Realtime voltage/current: 低频 1 组 (reg4-9), 高频 4 组 (reg4-27).
        # 每组 = V1,V2,V3,I1,I2,I3 (num_channels×2 个寄存器); 高频是同一内容的
        # 4 次重复。其后的保留/曲线开始毫秒/通讯确认码/剩余未读数依次后移,
        # 含义不变。
        self._num_channels = num_channels
        # 实时值组数: 未显式指定时按模块类型查表推导 (0x50→1, 0x53→4)
        if realtime_groups is None:
            realtime_groups = AXLE_REALTIME_GROUPS.get(profile.module_type, 1)
        self._realtime_groups = max(1, realtime_groups)
        # reg4-9 = V1,V2,V3,I1,I2,I3 —— 3 个电压在前, 3 个电流在后 (不交错)
        npad = max(0, 3 - num_channels)
        self._vi_group: list[int] = (
            [channel_voltage_raw] * num_channels + [0] * npad    # V1..Vn
            + [channel_current_raw] * num_channels + [0] * npad  # I1..In
        )
        self._vi_defaults: list[int] = self._vi_group * self._realtime_groups

        # 寄存器索引 (随实时值组数变化)
        self._vi_reg_start = 4                                     # 实时值起始
        self._reserved2_reg = self._vi_reg_start + len(self._vi_defaults)
        self._curve_ms_reg = self._reserved2_reg + 1               # 曲线开始毫秒
        self._code_reg = self._curve_ms_reg + 1                    # 通讯确认码
        self._unread_reg = self._code_reg + 1                      # 剩余未读数
        self._reg_count = self._unread_reg + 1                     # 寄存器总数

        # Set as data_gen overrides so the register watch shows the values
        # (10V→100, 20mA→200) and GUI edits still take priority.
        for i, v in enumerate(self._vi_defaults):
            self.data_gen.set_register_override(self._vi_reg_start + i, v)

        # reg12 确认码: 每条应答递增 (水位 ACK), 0 → 1 → 2 ...
        self._current_code: int = 0

        # Curve start time base (reg2-3 + reg11); set on each new pulse batch
        self._curve_start_sec: int = 0
        self._curve_start_ms: int = 0

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

        Request: [addr, 0x03, confirm_hi, confirm_lo, count_hi, count_lo, CRC]
          - confirm = 通信板回显的最新 reg12 (水位 ACK)
          - count   = 0 → 状态查询;  >0 → 读取请求 (带脉冲块)

        State machine (spec section 5):
          1. 水位 ACK: 从队首清除"已发出且序号 ≤ echo_code"的脉冲.
             已发出而未确认的脉冲保留在队列中, 供丢帧后重发.
             若被清空的是当前手动发送批次, 则该曲线计为发送成功 (计数+1).
          2. 无自动入队 — 脉冲只在点"发送曲线"按钮时由 trigger_curve_send() 产生.
          3. 每条应答 reg12 递增; 读取请求从队首取最多 count 个脉冲,
             并记录本次应答的序号 (重发时也更新序号).
          4. reg13 = 队列剩余数 = 交付后仍未读走的脉冲数.
        """
        echo_code = (raw_bytes[2] << 8) | raw_bytes[3]
        req_count = (raw_bytes[4] << 8) | raw_bytes[5]

        self._curve_confirmed_this_req = False

        # --- 水位 ACK: 从队首清除已发出且序号 ≤ echo_code 的脉冲 ---
        if echo_code != 0:
            removed = 0
            while (removed < len(self._pulse_events)
                   and self._pulse_events[removed][0] is not None
                   and self._pulse_events[removed][0] <= echo_code):
                removed += 1
            if removed:
                self._pulse_events = self._pulse_events[removed:]
                # 手动批次: 该批全部脉冲被通信板 ACK 后, 曲线计为发送成功
                if self._in_flight:
                    self._batch_remaining = max(0, self._batch_remaining - removed)
                    if self._batch_remaining == 0:
                        self._in_flight = False
                        self._curve_send_count += 1
                        self._curve_confirmed_this_req = True

        # --- 每条应答 reg12 递增 (水位序号) ---
        self._current_code += 1
        if self._current_code >= 0xFFFE:
            self._current_code = 1
        code = self._current_code

        # --- 读取请求: 从队首取最多 req_count 个脉冲, 记录本次序号 ---
        # (队首可能包含先前已发出但未确认的脉冲 — 丢帧时按回显重发)
        take = 0
        if req_count > 0 and self._pulse_events:
            take = min(req_count, len(self._pulse_events),
                       self.MAX_PULSES_PER_REQ)
            for i in range(take):
                self._pulse_events[i] = (code, self._pulse_events[i][1])

        # reg13 = 交付后剩余未读数
        unread = len(self._pulse_events) - take

        # --- 构建帧: [addr, 0x03, 寄存器28B, 脉冲块(可选), CRC] ---
        frame = bytearray()
        frame.append(self.address)
        frame.append(0x03)
        frame.extend(self._build_register_block(unread))

        # 脉冲块: 仅读取请求 (count>0) 才附带
        if take > 0:
            frame.extend(struct.pack(">H", take))   # 本包上报脉冲个数
            for i in range(take):
                frame.extend(self._pulse_events[i][1])

        # Append CRC16 (low byte first)
        crc = modbus_crc16(frame)
        frame.append(crc & 0xFF)
        frame.append((crc >> 8) & 0xFF)

        self.last_response = bytes(frame)
        return bytes(frame)

    # ------------------------------------------------------------------
    # Register block (reg0..reg13, 28 bytes, big-endian)
    # ------------------------------------------------------------------

    def _read_reg(self, idx: int, default: int) -> int:
        """Read a register, honouring GUI/data_gen overrides."""
        if self.data_gen.has_override(idx):
            vals = self.data_gen.peek_registers(idx, 1)
            return vals[0] if vals else default
        return default

    def _build_register_block(self, unread: int) -> bytes:
        """Build the register block reg0..reg(reg_count-1), all big-endian.

        低频 0x50 (28 字节): reg4-9 实时值 | reg10 保留 | reg11 毫秒 |
                             reg12 确认码 | reg13 剩余未读数
        高频 0x53 (64 字节): reg4-27 实时值(4组) | reg28 保留 | reg29 毫秒 |
                             reg30 确认码 | reg31 剩余未读数
        """
        curve_sec = self._curve_start_sec
        curve_ms = self._curve_start_ms

        regs = [
            self._read_reg(0, 0x0000),                    # reg0  保留/状态
            self._read_reg(1, 0x003F),                    # reg1  KGL 正常 (低6位=0x3F)
            self._read_reg(2, (curve_sec >> 16) & 0xFFFF),  # reg2  曲线开始秒 高16位
            self._read_reg(3, curve_sec & 0xFFFF),          # reg3  曲线开始秒 低16位
        ]
        # 实时值区: 6 个寄存器/组 (始终占满, 保证帧内偏移固定)
        for i, v in enumerate(self._vi_defaults):
            regs.append(self._read_reg(self._vi_reg_start + i, v))
        regs.append(self._read_reg(self._reserved2_reg, 0x0000))   # 保留
        regs.append(self._read_reg(self._curve_ms_reg, curve_ms))  # 曲线开始毫秒
        regs.append(self._current_code & 0xFFFF)          # 通讯确认码 (递增)
        regs.append(unread & 0xFFFF)                      # 剩余未读数

        buf = bytearray()
        for r in regs:
            buf.append((r >> 8) & 0xFF)
            buf.append(r & 0xFF)
        return bytes(buf)

    # ------------------------------------------------------------------
    # Response frame layout (供 main.py / GUI / 测试定位寄存器)
    # ------------------------------------------------------------------

    @property
    def unread_offset(self) -> int:
        """剩余未读数寄存器在应答帧中的字节偏移 (从地址+功能码之后算起).

        应答帧: [addr, 0x03, 寄存器块, 脉冲块(可选), CRC] — 本偏移是相对
        帧首的绝对偏移: ``2 + 剩余未读数寄存器索引 × 2``。低频 0x50 = 28,
        高频 0x53 = 64。
        """
        return 2 + self._unread_reg * 2

    @property
    def register_block_bytes(self) -> int:
        """寄存器块字节数 (低频 28, 高频 64)。"""
        return self._reg_count * 2

    # ------------------------------------------------------------------
    # Pulse simulation
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Manual curve send (button-driven, replaces continuous 1Hz generation)
    # ------------------------------------------------------------------

    @property
    def current_channel(self) -> int:
        """Currently selected curve channel (0..num_channels-1)."""
        return self._selected_channel

    @property
    def curve_in_flight(self) -> bool:
        """True while a manually sent curve awaits master confirmation."""
        return self._in_flight

    @property
    def curve_send_count(self) -> int:
        """Number of curves the master has confirmed receiving."""
        return self._curve_send_count

    @property
    def curve_confirmed_this_req(self) -> bool:
        """True if the just-handled request confirmed a curve batch."""
        return self._curve_confirmed_this_req

    def reset_curve_confirmation(self) -> None:
        """Clear the per-request confirmation flag (called by the owner)."""
        self._curve_confirmed_this_req = False

    def set_curve_channel(self, channel: int) -> None:
        """Select the channel for the next manually sent curve (0..N-1)."""
        if self._num_channels <= 0:
            return
        channel = channel % self._num_channels
        self._selected_channel = channel
        self._passage_index = channel
        self._pulses_in_passage = 0

    def trigger_curve_send(self) -> bool:
        """Queue one full "passage" (pulses_per_passage pulses) on the
        selected channel for the master to read back.

        Only one batch may be in flight: returns False if a previous batch
        is still queued/unconfirmed, or if curve mode is off.

        Returns:
            True if a batch was queued, False otherwise.
        """
        if self.curve_mode != AxlePulseMode.PULSE:
            return False
        if self._pulse_events:
            return False                      # 上一批尚未被通信板确认

        self.set_curve_channel(self._selected_channel)   # 对齐到所选通道
        # 一趟过车的脉冲时间戳依次递增 pulse_interval (默认 1s = 1Hz):
        # t0, t0+1s, t0+2s, ... 曲线开始时间 = t0 (首脉冲, 偏移 0)。
        self._pulse_events = [
            (None, self._build_pulse_event(
                i == 0, ts_offset_s=i * self._pulse_interval))
            for i in range(self._pulses_per_passage)
        ]
        # _build_pulse_event 在过车组结束时自动轮换通道, 这里撤消, 保持用户所选
        self._passage_index = self._selected_channel
        self._pulses_in_passage = 0

        self._in_flight = True
        self._batch_remaining = len(self._pulse_events)
        return True

    def cancel_curve_send(self) -> None:
        """Drop an in-flight batch without counting it (GUI send timeout)."""
        self._pulse_events = []
        self._in_flight = False
        self._batch_remaining = 0

    def _generate_pulse_samples(self, n: int) -> list[int]:
        """Generate one smooth pulse curve: raised-cosine rise + fall, no plateau.

        Each call randomizes the rise/fall split so no two pulses look
        identical (real axle wheel pulses wobble). The peak lands exactly on
        ``pulse_amplitude_raw`` (10V) and both ends are 0V.

        Edge shaping uses a raised cosine ``1 - cos(πt)``: zero slope at both
        ends, so the curve joins 0V and peak smoothly — no trapezoid corners.

        Args:
            n: Number of samples (ceil(pulse_width_ms / pulse_sample_ms), 63).

        Returns:
            n integer samples in 0.1V units (0 = 0V, amp = 10V peak).
        """
        amp = self._pulse_amplitude_raw
        if n <= 1:
            return [amp]

        # 随机分配上升/下降沿: 上升沿约占窗口 1/3 ~ 2/3, 每次脉冲略不同
        rise_len = random.randint(max(2, n // 3), min(n - 2, (2 * n) // 3))
        fall_len = n - rise_len

        samples: list[int] = []
        for k in range(rise_len):
            t = k / max(1, rise_len - 1)
            samples.append(round(amp * 0.5 * (1.0 - math.cos(math.pi * t))))
        for k in range(fall_len):
            t = k / max(1, fall_len - 1)
            samples.append(round(amp * 0.5 * (1.0 + math.cos(math.pi * t))))
        return samples

    def _build_pulse_event(self, batch_start: bool,
                           ts_offset_s: float = 0.0) -> bytes:
        """Build one pulse event packet.

        Shape: 10V (raw 100) smooth random pulse, ~250ms window (63 samples
        @ 4ms) — raised-cosine rise 0V→10V then fall 10V→0V, no plateau,
        rise/fall split randomized per pulse (see ``_generate_pulse_samples``).
        Channel: consecutive pulses of one "passage" share a channel
        (pulses_per_passage), matching protocol §6 per-channel 0x09 grouping.

        Timestamp: each pulse gets its own absolute time ``time.time() +
        ts_offset_s``. Within one manually sent passage the pulses are spaced
        pulse_interval apart (offset = i * interval), so the master can place
        them at distinct 点偏移 in the 0x09 frame (§6). The first pulse of a
        batch (offset 0) also sets the curve start time reg2-3+reg11.

        Packet: [sec(4B BE)][ms(2B BE)][channel(1B)][n(1B)][data(n×1B, 0.1V)]
        """
        now = _time.time() + ts_offset_s
        ts_sec = int(now)
        ts_ms = int((now - ts_sec) * 1000)

        if batch_start:
            # Curve start = timestamp of the first pulse in this batch
            # (batch_start only applies to the offset-0 pulse, so this is t0)
            self._curve_start_sec = ts_sec
            self._curve_start_ms = ts_ms

        # 通道号 (0..num_channels-1): 同一过车组的连续脉冲在同一通道,
        # 组内满后切换到下一通道
        channel = self._passage_index % self._num_channels
        self._pulses_in_passage += 1
        if self._pulses_in_passage >= self._pulses_per_passage:
            self._pulses_in_passage = 0
            self._passage_index += 1

        n = self._pulse_num_points
        pkt = bytearray()
        pkt.extend(struct.pack(">I", ts_sec))
        pkt.extend(struct.pack(">H", ts_ms))
        pkt.append(channel & 0xFF)
        pkt.append(n & 0xFF)
        pkt.extend(self._generate_pulse_samples(n))
        return bytes(pkt)

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
        queue = len(self._pulse_events)
        return (f"  [{self.address:3d}] 0x{self.profile.module_type:02X} "
                f"AxleCounter "
                f"req={self.request_count} err={self.error_count} "
                f"queue={queue} ch={self._selected_channel} "
                f"ok={self._curve_send_count} "
                f"reg12=0x{self._current_code:04X}")
