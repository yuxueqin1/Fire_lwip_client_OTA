#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 OTA 固件升级服务器 GUI  —  STM32 Firmware Over-The-Air Update Tool
================================================================================

 功能概述:
   - 图形化选择固件文件 (.bin)，无需命令行输入
   - TCP 服务器等待通信板连接，自动推送固件升级
   - 实时显示客户端连接状态、传输进度
   - 彩色日志输出

 协议: 与 STM32F429 User/ota/ 下的 ota_client/ota_net/ota_flash 配套
帧格式: [0xAA][CMD][SEQ][LEN(2B BE)][OFFSET(4B BE)][DATA][CHECKSUM][0xBB]
       ⚠ 帧头里的 LEN/OFFSET 是大端, 但**载荷内部的 32 位整数一律小端**
         (对应设备侧 put_u32_le/get_u32_le)。两者不要搞混。

 A/B 双镜像 (乒乓升级):
   同一份源码编译两次, 链接地址不同, 所以固件文件也是两个:
       xxx_v1.0.0.2_A.bin  → 写入 APP1 (0x08008000, 480KB)
       xxx_v1.0.0.2_B.bin  → 写入 APP2 (0x08080000, 384KB)
   设备在 HELLO 里上报"当前运行槽位", 上位机总是把新固件推到另一个槽位,
   因此当前运行分区全程不被破坏 —— 传输失败/掉电/CRC 错都只是白跑一趟,
   重启用回老版本即可, 不会变砖。
   两个文件放在同一目录, 选任意一个, 本工具自动配对。单镜像上限 384KB。

 升级时序:
   HELLO ─► PUSH_UPDATE ─► CONFIRM_UPDATE ─[设备擦除 1~5s]─►
   ACK(DOWNLOADING) ─► DATA xN ─► END ─► ACK(SUCCESS) ─► 设备重启
   ⚠ 必须等 ACK(DOWNLOADING) 才发数据 —— 擦除期间设备完全不收包,
     提前猛灌只能靠 lwIP 的 8760 字节 TCP 窗口兜着, 链路很脆。

 使用方法:
     python ota_gui.py                    → 打开 GUI，手动选择固件
     python ota_gui.py -f firmware_A.bin  → 自动加载固件(自动配上同目录 _B)

 运行要求:
     Python 3.6+, 仅使用标准库 (tkinter, socket, threading 等)
"""

import argparse
import os
import queue
import re
import select
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import ttk, filedialog, messagebox
from zlib import crc32


# ============================================================================
# 常量定义 (与 User/ota/ota_config.h + ota_net.c 保持一致)
# ============================================================================

OTA_NET_FRAME_MAGIC = 0xAA
OTA_NET_FRAME_END   = 0xBB

# 单帧最大数据长度 == ota_config.h 的 OTA_MAX_PAYLOAD
OTA_MAX_PAYLOAD = 1024

# OTA 命令字
OTA_CMD_HELLO          = 0x00
OTA_CMD_HELLO_RESP     = 0x80
OTA_CMD_PUSH_UPDATE    = 0x10
OTA_CMD_CONFIRM_UPDATE = 0x11
OTA_CMD_REJECT_UPDATE  = 0x12
OTA_CMD_ERROR_UPDATE   = 0x13
OTA_CMD_START_DOWNLOAD = 0x03
OTA_CMD_DATA           = 0x04
OTA_CMD_END            = 0x05
OTA_CMD_ACK            = 0x06
OTA_CMD_NAK            = 0x07

CMD_NAMES = {
    0x00: "HELLO",       0x80: "HELLO_RESP",
    0x10: "PUSH_UPDATE", 0x11: "CONFIRM_UPDATE",
    0x12: "REJECT_UPDATE", 0x13: "ERROR_UPDATE",
    0x03: "START_DOWNLOAD", 0x04: "DATA",
    0x05: "END",         0x06: "ACK",         0x07: "NAK",
}

# ── 分区 / 槽位 (与 ota_config.h 严格一致) ──
SLOT_APP1 = 1
SLOT_APP2 = 2

APP1_ADDR = 0x08008000
APP1_SIZE = 480 * 1024
APP2_ADDR = 0x08080000
APP2_SIZE = 384 * 1024

# 单镜像上限 = min(480K, 384K)
APP_MAX_SIZE = APP2_SIZE

# 通信板里硬编码的服务器地址(ota_config.h 的 OTA_SERVER_IP_DEFAULT),
# 升级时把本机网卡配成这个地址即可连通, 不需要改固件。
OTA_SERVER_IP = "192.168.1.99"

SLOT_NAMES = {SLOT_APP1: "A", SLOT_APP2: "B"}

# 镜像校验用的 SRAM 范围 (IRAM1)
SRAM_START_ADDR = 0x20000000
SRAM_END_ADDR   = 0x20030000

# ── ACK 载荷里的状态码 (ota_state_t) ──
ACK_STATE_IDLE          = 0
ACK_STATE_REQUEST_RECV  = 2
ACK_STATE_DOWNLOADING   = 3     # 设备擦除完成, "可以发数据了"
ACK_STATE_VERIFYING     = 4
ACK_STATE_SUCCESS       = 5     # 数据已校验通过并落地

# ERROR_UPDATE 载荷里的错误码 → 可读名字 (ota_state_t >= 8)
OTA_ERROR_NAMES = {
    8:  "OTA_STATE_ERROR_SIZE (固件大小超限 / 非 4 字节倍数)",
    9:  "OTA_STATE_ERROR_INVALID_FRAME (无效帧)",
    10: "OTA_STATE_SEND_COMFIRM_FAILED (发送确认失败)",
    11: "OTA_STATE_START_DOWNLOAD_FAILED (擦除/启动下载失败)",
    12: "OTA_STATE_ERROR_IMAGE (镜像结构非法: 栈指针/复位向量不在目标分区)",
    13: "OTA_STATE_ERROR_CRC (CRC32 校验失败)",
    14: "OTA_STATE_ERROR_FLASH (Flash 擦写失败)",
    15: "OTA_STATE_ERROR_SLOT (槽位未知: 链接地址配置错误)",
}

# 一块 Flash 只能按 4 字节写入, 固件长度必须是 4 的倍数 (不足则补 0x00)
FW_ALIGN = 4

# 客户端状态
CLIENT_STATE_IDLE       = "空闲"
CLIENT_STATE_CONNECTED  = "已连接"
CLIENT_STATE_WAITING    = "等待确认"
CLIENT_STATE_ERASING    = "擦除中"
CLIENT_STATE_DOWNLOADING = "下载中"
CLIENT_STATE_COMPLETED  = "已完成"
CLIENT_STATE_ERROR      = "错误"
CLIENT_STATE_REJECTED   = "已拒绝"

# 默认值
DEFAULT_PORT       = 5002
DEFAULT_BIND_ADDR  = "0.0.0.0"
DEFAULT_VERSION    = 0x01000001
DEFAULT_CHUNK_SIZE = 1024   # 每块字节数 (== OTA_MAX_PAYLOAD)
DEFAULT_CHUNK_DELAY = 0.0   # 块间延迟(秒)。设备侧已改为"一次 recv 连发多帧",
                            # 不再需要人为限速; 靠 TCP 窗口自然背压即可。

# 单次循环最多入队多少帧(≈ 8KB, 与 lwIP TCP_WND=8760 同量级)。
# 队列有界 → 内存不会涨, 同时保证管道始终填满。
TX_BURST_FRAMES = 8

# 空闲多久没有任务收发就断开(秒)。设备擦除最长约 5s, 留足余量。
IDLE_TIMEOUT_S = 180.0

# 固件文件名里的版本号形式, 例如 YH-F429_v1.0.0.2_A.bin
VERSION_RE = re.compile(r'v?(\d{1,3})[._](\d{1,3})[._](\d{1,3})[._](\d{1,3})')


# ============================================================================
# OTA 帧编解码 (与 ota_net.c 保持一致)
# ============================================================================

class OTAFrame:
    """OTA 协议帧。帧格式:

       [MAGIC:0xAA] [CMD:1B] [SEQ:1B] [LEN:2B BE] [OFFSET:4B BE]
       [DATA: LEN bytes] [CHECKSUM:1B] [END:0xBB]
    """

    __slots__ = ('cmd', 'seq', 'data', 'offset', 'length')

    def __init__(self, cmd: int, seq: int = 0, data: bytes = b''):
        self.cmd = cmd
        self.seq = seq & 0xFF
        self.data = data
        self.offset = 0
        self.length = len(data)

    def to_bytes(self) -> bytes:
        """编码为网络传输字节串。"""
        buf = bytearray()
        buf.append(OTA_NET_FRAME_MAGIC)
        buf.append(self.cmd)
        buf.append(self.seq)
        buf.append((self.length >> 8) & 0xFF)
        buf.append(self.length & 0xFF)
        buf.append((self.offset >> 24) & 0xFF)
        buf.append((self.offset >> 16) & 0xFF)
        buf.append((self.offset >> 8) & 0xFF)
        buf.append(self.offset & 0xFF)
        buf.extend(self.data)
        # 简单校验和 (与STM32端一致)
        checksum = sum(buf) & 0xFF
        buf.append(checksum)
        buf.append(OTA_NET_FRAME_END)
        return bytes(buf)

    @staticmethod
    def from_bytes(data: bytes):
        """从一个**恰好只含一帧**的字节串解码。失败返回 None。

        ⚠ 不要拿它解析 recv() 的返回值 —— TCP 是字节流, 一次 recv 可能得到
          半帧或好几帧。接收侧请用 parse_stream_frames()。
          本函数只用于测试/离线分析单个完整帧。
        """
        if len(data) < 11:
            return None
        if data[0] != OTA_NET_FRAME_MAGIC or data[-1] != OTA_NET_FRAME_END:
            return None

        cmd = data[1]
        seq = data[2]
        length = (data[3] << 8) | data[4]
        offset = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8]

        # 长度合理性检查
        if length > 2048 or 9 + length + 2 > len(data):
            return None

        payload = data[9:9 + length]

        # 验证校验和 (cover bytes 0..9+length-1)
        calc_sum = sum(data[:9 + length]) & 0xFF
        if calc_sum != data[9 + length]:
            return None

        frame = OTAFrame(cmd, seq, payload)
        frame.offset = offset
        return frame

    def __repr__(self) -> str:
        cmd_name = CMD_NAMES.get(self.cmd, f"0x{self.cmd:02X}")
        return (f"OTAFrame(cmd={cmd_name}, seq={self.seq}, "
                f"len={self.length}, offset={self.offset})")


def decode_u32(data: bytes, offset: int = 0) -> int:
    """载荷里的 32 位整数一律小端 (对应设备侧 get_u32_le / put_u32_le)。

    ⚠ 老版本上位机用的是 '>I'(大端), 与设备侧完全对不上 —— 版本号/大小/
      CRC/ACK 状态码全部读错, 这是"看起来握手成功但校验必然失败"的根因。
    """
    return struct.unpack_from('<I', data, offset)[0]


def encode_u32(value: int) -> bytes:
    """小端打包, 与 decode_u32 配对。"""
    return struct.pack('<I', value & 0xFFFFFFFF)


def _parse_frame_at(buf: bytes, pos: int, avail: int):
    """尝试把 buf[pos:] 解析成一帧。

    返回 (frame, need):
        frame 非 None     → 成功, need = 该帧占用的字节数
        frame 为 None     → need == 0  该位置不是合法帧(帧头/长度/校验/帧尾不过)
                            need >  0  数据还不够, need = 整帧所需字节数
    """
    if avail < 11:
        # 连最小帧都不够: 只有帧头对才值得等
        if buf[pos] != OTA_NET_FRAME_MAGIC:
            return None, 0
        return None, 11

    if buf[pos] != OTA_NET_FRAME_MAGIC:
        return None, 0

    length = (buf[pos + 3] << 8) | buf[pos + 4]
    if length > OTA_MAX_PAYLOAD:
        return None, 0

    need = 9 + length + 2
    if avail < need:
        return None, need

    calc = sum(buf[pos:pos + 9 + length]) & 0xFF
    if calc != buf[pos + 9 + length] or buf[pos + need - 1] != OTA_NET_FRAME_END:
        return None, 0

    frame = OTAFrame(buf[pos + 1], buf[pos + 2], buf[pos + 9:pos + 9 + length])
    frame.offset = ((buf[pos + 5] << 24) | (buf[pos + 6] << 16) |
                    (buf[pos + 7] << 8) | buf[pos + 8])
    return frame, need


def parse_stream_frames(buf: bytes):
    """从 TCP 字节流里切出完整的帧。

    返回 (frames, leftover) —— frames 是 OTAFrame 列表, leftover 是尚未凑齐
    整帧的尾巴, 调用方必须保存下来与下次 recv 的数据拼接。

    TCP 是字节流, 一次 recv 可能带回 0.5 帧、1 帧或 5 帧。老实现把每次
    recv 的原始数据直接丢给 OTAFrame.from_bytes(), 于是:
      * 一帧跨两次 recv  → 被当成无效帧丢掉
      * 一次 recv 多帧   → 只解析第一帧, 后面全丢

    重同步有个坑: 遇到"数据不够一整帧"的候选位置时, 不能就此停住 ——
    字节流里的杂散 0xAA 会伪造出一个帧头, 它隐含的长度可能很大, 于是
    缓冲区里明明躺着的一帧真数据会被这个假候选挡住, 一直等到超时。
    所以这里记录最早的"未凑齐"候选 (stall), 继续往后找; 只有整条缓冲
    都找不出完整帧时, 才把 stall 之后的数据留作残留。
    正常情形(真·半帧在末尾)结果不变: 找不到别的完整帧, 残留就是从
    帧头开始的那半帧。
    """
    frames = []
    total = len(buf)
    pos = 0
    stall = None                            # 最早的"可能只是还没收全"的位置

    while pos < total:
        frame, need = _parse_frame_at(buf, pos, total - pos)

        if frame is not None:
            frames.append(frame)
            pos += need
            stall = None                    # 已跨过更早的候选, 作废
            continue

        if need == 0:
            # 该位置确定不是帧头 → 直接跳到下一个候选 0xAA (C 级查找, 快)
            nxt = buf.find(OTA_NET_FRAME_MAGIC, pos + 1)
            pos = nxt if nxt >= 0 else total
            continue

        # 数据不够一整帧: 记下候选, 继续往后找有没有完整的帧
        if stall is None:
            stall = pos
        nxt = buf.find(OTA_NET_FRAME_MAGIC, pos + 1)
        pos = nxt if nxt >= 0 else total

    return frames, (buf[stall:] if stall is not None else b'')


def pad_to_align(data: bytes, align: int = FW_ALIGN) -> bytes:
    """把固件补齐到 align 的整数倍。

    STM32 的 Flash 只能按 4 字节写入, 设备侧 ota_write_data_block() 会拒绝
    非 4 字节倍数的长度。补齐放在上位机做(用户要求), 设备侧因此不必特判;
    补的都是 0x00, 落到 Flash 里就是 0xFFFFFFFF 之外的"真实 0", 不影响执行
    (程序入口之前的填充区不会被取指)。
    """
    rem = len(data) % align
    if rem == 0:
        return data
    return data + b'\x00' * (align - rem)


# ============================================================================
# 会话阶段 (与 ota_client.c 的时序一一对应)
# ============================================================================

PHASE_WAIT_HELLO   = 0   # 等设备上报版本/槽位
PHASE_WAIT_CONFIRM = 1   # 已发 PUSH_UPDATE, 等 CONFIRM/REJECT/ERROR
PHASE_WAIT_READY   = 2   # 收到 CONFIRM, 设备正在擦除(数秒), 等 ACK(DOWNLOADING)
PHASE_PUSHING      = 3   # 正在推固件数据
PHASE_WAIT_FINAL   = 4   # 已发 END, 等 ACK(SUCCESS)
PHASE_DONE         = 5   # 会话结束


class OTAClientSession:
    """一块通信板的完整 OTA 会话。

    时序:
        [设备] HELLO ──────────────►
                    ◄────── PUSH_UPDATE   (版本 + 大小 + CRC32)
        [设备] CONFIRM_UPDATE ─────►
                    (设备擦除目标分区 1~5s, CPU 被 Flash 总线 stall)
                    ◄────── ACK(DOWNLOADING)   "擦好了, 可以发数据"
                    ◄────── DATA xN
                    ◄────── END
        [设备] ACK(SUCCESS) ───────►  (数据已校验并落地)
                    (设备写 BootInfo → 重启 → Bootloader 跳新分区)

    之所以必须等 ACK(DOWNLOADING): 擦除期间设备完全不收包, 若上位机提前
    猛灌数据, 只能靠 TCP 窗口兜住 —— 而 lwIP 的 TCP_WND 只有 8760 字节,
    窗口一满就丢包重传, 链路很脆。等一个明确的"就绪"信号要可靠得多。
    """

    def __init__(self, server, sock: socket.socket, addr: tuple):
        self.server = server
        self.sock = sock
        self.addr = addr
        self.client_id = f"{addr[0]}:{addr[1]}"

        self.rx = b''                 # TCP 流重组缓冲(半帧残留)
        self.txq: deque = deque()     # 待发字节(有界, 见 TX_BURST_FRAMES)
        self.phase = PHASE_WAIT_HELLO
        self.seq = 0
        self.last_activity = time.time()

        # 设备上报
        self.client_version = 0
        self.cur_slot = 0
        self.target_slot = 0

        # 本次推送
        self.push_slot = 0
        self.fw = None
        self.next_off = 0
        self.chunk_seq = 0
        self.end_queued = False
        self.last_progress = -1

    # ------------------------------------------------------------------
    # 会话主体
    # ------------------------------------------------------------------

    def run(self):
        self.server._register_client(self.client_id, self.sock, self.addr)
        try:
            self.sock.setblocking(False)
            while self.server.is_running and self.phase != PHASE_DONE:
                if not self._step():
                    break
        except Exception as e:
            self.server._post_event(
                "error", f"客户端 {self.client_id} 处理异常: {e}")
        finally:
            self.server._unregister_client(self.client_id)

    def _step(self) -> bool:
        """跑一轮 select + 收发。返回 False 表示本次会话结束。"""
        want_w = bool(self.txq)
        try:
            rl, wl, _ = select.select(
                [self.sock], [self.sock] if want_w else [], [], 0.2)
        except (OSError, ValueError):
            return False

        if rl and not self._on_readable():
            return False
        if wl and not self._flush_tx():
            return False

        # 数据推完/丢了一部分之后继续补队列
        if self.phase == PHASE_PUSHING:
            self._enqueue_next()

        if (time.time() - self.last_activity) > IDLE_TIMEOUT_S:
            self.server._post_event(
                "warning", f"客户端 {self.client_id} 空闲超时 "
                           f"({IDLE_TIMEOUT_S:.0f}s), 断开连接")
            return False
        return True

    # ------------------------------------------------------------------
    # 收
    # ------------------------------------------------------------------

    def _on_readable(self) -> bool:
        closed = False

        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            except OSError:
                closed = True
                break
            if not chunk:
                closed = True                      # 对端关闭
                break
            self.rx += chunk
            self.last_activity = time.time()
            if len(chunk) < 4096:
                break                              # 大致读空了

        if self.rx:
            frames, self.rx = parse_stream_frames(self.rx)
            for f in frames:
                self.server._post_event("frame_rx", {
                    "client_id": self.client_id,
                    "cmd": f.cmd,
                    "cmd_name": CMD_NAMES.get(f.cmd, f"0x{f.cmd:02X}"),
                    "seq": f.seq,
                    "len": f.length,
                })
                self.seq = f.seq
                self._dispatch(f)
                if self.phase == PHASE_DONE:
                    return False

        if closed:
            if self.phase != PHASE_DONE:
                self.server._post_event(
                    "warning", f"客户端 {self.client_id} 断开连接")
            return False
        return True

    # ------------------------------------------------------------------
    # 发
    # ------------------------------------------------------------------

    def _queue(self, frame: OTAFrame, desc: str, log: bool = True):
        """把一帧排进发送队列。DATA 帧不逐帧打日志(否则一次升级几百行)。"""
        self.txq.append(frame.to_bytes())
        if log:
            self.server._post_event("frame_tx", {
                "client_id": self.client_id,
                "cmd": frame.cmd,
                "cmd_name": CMD_NAMES.get(frame.cmd, f"0x{frame.cmd:02X}"),
                "seq": frame.seq,
                "len": frame.length,
                "desc": desc,
            })

    def _flush_tx(self) -> bool:
        """尽力把队列里的数据写进 socket。返回 False 表示连接出错。"""
        while self.txq:
            head = self.txq[0]
            try:
                n = self.sock.send(head)
            except BlockingIOError:
                return True                        # 发不动了, 等可写
            except OSError as e:
                self.server._post_event(
                    "error", f"客户端 {self.client_id}: 发送失败: {e}")
                return False

            if n <= 0:
                return False
            if n >= len(head):
                self.txq.popleft()
            else:
                self.txq[0] = head[n:]
            self.last_activity = time.time()
        return True

    def _enqueue_next(self):
        """持续补满发送队列(有界), 推完数据后紧接着排 END。"""
        if self.fw is None:
            return

        size = self.fw['size']
        chunk = min(self.server.chunk_size, OTA_MAX_PAYLOAD)
        chunk -= chunk % FW_ALIGN
        if chunk <= 0:
            chunk = FW_ALIGN

        while len(self.txq) < TX_BURST_FRAMES:
            if self.next_off < size:
                take = min(chunk, size - self.next_off)
                frame = OTAFrame(OTA_CMD_DATA, self.chunk_seq,
                                 self.fw['data'][self.next_off:
                                                 self.next_off + take])
                frame.offset = self.next_off
                self._queue(frame, "DATA", log=False)
                self.next_off += take
                self.chunk_seq = (self.chunk_seq + 1) & 0xFF
                self._report_progress()
            elif not self.end_queued:
                self._queue(OTAFrame(OTA_CMD_END, 0), "END")
                self.end_queued = True
                self.phase = PHASE_WAIT_FINAL
                self.server._post_event("info",
                    f"客户端 {self.client_id}: 固件已全部发出 "
                    f"({size:,} bytes), 等待设备校验...")
                break
            else:
                break

    def _report_progress(self):
        if self.fw is None or self.fw['size'] == 0:
            return
        pct = int(self.next_off * 100 / self.fw['size'])
        if pct == self.last_progress:
            return
        self.last_progress = pct
        self.server._update_client(self.client_id, progress=pct)
        self.server._post_event("transfer_progress", {
            "client_id": self.client_id,
            "offset": self.next_off,
            "total": self.fw['size'],
            "progress": pct,
        })

    # ------------------------------------------------------------------
    # 命令分发 (载荷里的整数一律小端!)
    # ------------------------------------------------------------------

    def _dispatch(self, f: OTAFrame):
        if f.cmd == OTA_CMD_HELLO:
            self._on_hello(f)
        elif f.cmd == OTA_CMD_CONFIRM_UPDATE:
            self._on_confirm(f)
        elif f.cmd == OTA_CMD_REJECT_UPDATE:
            self._on_reject(f)
        elif f.cmd == OTA_CMD_ERROR_UPDATE:
            self._on_error(f)
        elif f.cmd == OTA_CMD_ACK:
            self._on_ack(f)
        elif f.cmd in (OTA_CMD_START_DOWNLOAD, OTA_CMD_DATA, OTA_CMD_NAK):
            pass                                   # 设备侧不该发这些, 忽略
        else:
            self.server._post_event(
                "warning", f"客户端 {self.client_id} 未知命令 "
                           f"0x{f.cmd:02X}, 忽略")

    def _on_hello(self, f: OTAFrame):
        """设备心跳: 上报版本 + 当前槽位 + 目标槽位。"""
        if f.length >= 6:
            self.client_version = decode_u32(f.data, 0)
            self.cur_slot = f.data[4]
            self.target_slot = f.data[5]
        elif f.length >= 4:
            self.client_version = decode_u32(f.data, 0)

        slot_name = SLOT_NAMES.get(self.cur_slot, "?")
        ver_str = f"0x{self.client_version:08X}"

        self.server._update_client(self.client_id, version=self.client_version)
        self.server._post_event("client_version", {
            "client_id": self.client_id,
            "version": self.client_version,
            "slot": self.cur_slot,
            "slot_name": slot_name,
            "target_slot": self.target_slot,
        })

        # 已经在升级流程中, HELLO 只是心跳, 不要重复推
        if self.phase != PHASE_WAIT_HELLO:
            return

        if not self.server.firmware_images:
            self.server._post_event("warning",
                f"客户端 {self.client_id} 版本 {ver_str} (运行于 {slot_name} 区), "
                f"但服务器未加载固件, 不推送")
            return

        if self.client_version >= self.server.firmware_version:
            self.server._post_event("info",
                f"客户端 {self.client_id} 已是最新版本 {ver_str} "
                f"(服务器 0x{self.server.firmware_version:08X}), 不推送")
            self.server._update_client(self.client_id, state=CLIENT_STATE_IDLE)
            return

        # A/B 乒乓: 推给"设备当前没在跑"的那个槽位
        slot = self.target_slot
        if slot not in (SLOT_APP1, SLOT_APP2):
            slot = SLOT_APP2 if self.cur_slot == SLOT_APP1 else SLOT_APP1

        img = self.server.image_for_slot(slot)
        if img is None:
            self.server._post_event("error",
                f"客户端 {self.client_id} 运行在 {slot_name} 区, 需要推送到 "
                f"{SLOT_NAMES[slot]} 区, 但 {SLOT_NAMES[slot]} 镜像未加载 —— "
                f"请把 <固件名>_A.bin 和 _B.bin 一起放在同一目录后重新选择")
            return

        self.push_slot = slot
        self.seq = (f.seq + 1) & 0xFF
        payload = (encode_u32(self.server.firmware_version) +
                   encode_u32(img['size']) +
                   encode_u32(img['crc']))
        self._queue(OTAFrame(OTA_CMD_PUSH_UPDATE, self.seq, payload),
                    "PUSH_UPDATE")
        self.server._update_client(self.client_id, state=CLIENT_STATE_WAITING)

        self.server._post_event("info",
            f"客户端 {self.client_id} 版本 {ver_str} (运行于 {slot_name} 区), "
            f"将升级到 0x{self.server.firmware_version:08X} → "
            f"{SLOT_NAMES[slot]} 区 ({img['size']:,} bytes, "
            f"CRC32=0x{img['crc']:08X})")
        self.phase = PHASE_WAIT_CONFIRM

    def _on_confirm(self, f: OTAFrame):
        if self.phase != PHASE_WAIT_CONFIRM:
            return
        self.server._update_client(self.client_id, state=CLIENT_STATE_ERASING)
        self.server._post_event("info",
            f"客户端 {self.client_id} 已确认升级, 正在擦除 "
            f"{SLOT_NAMES.get(self.push_slot, '?')} 区 Flash "
            f"(最长约 5 秒, 期间设备不收包), 请稍候...")
        self.phase = PHASE_WAIT_READY

    def _on_reject(self, f: OTAFrame):
        self.server._update_client(self.client_id, state=CLIENT_STATE_REJECTED)
        self.server._post_event("warning",
            f"客户端 {self.client_id} 拒绝了升级请求"
            f"(设备侧判断版本未提升或大小非法)")
        self.phase = PHASE_DONE

    def _on_error(self, f: OTAFrame):
        code = decode_u32(f.data, 0) if f.length >= 4 else 0
        name = OTA_ERROR_NAMES.get(code, f"未知错误码 {code}")
        self.server._update_client(self.client_id, state=CLIENT_STATE_ERROR)
        self.server._post_event("error",
            f"客户端 {self.client_id} 升级失败: {name} (0x{code:08X})。"
            f"当前运行分区全程未被破坏, 设备会在 10s 后重连, 届时可直接重试。")
        self.phase = PHASE_DONE

    def _on_ack(self, f: OTAFrame):
        state = decode_u32(f.data, 0) if f.length >= 4 else -1

        if state == ACK_STATE_DOWNLOADING and self.phase == PHASE_WAIT_READY:
            self.fw = self.server.image_for_slot(self.push_slot)
            if self.fw is None:
                self.server._post_event("error",
                    f"客户端 {self.client_id}: 镜像在会话中丢失, 放弃")
                self.phase = PHASE_DONE
                return
            self.next_off = 0
            self.chunk_seq = 0
            self.end_queued = False
            self.last_progress = -1
            self.phase = PHASE_PUSHING
            self.server._update_client(self.client_id,
                                       state=CLIENT_STATE_DOWNLOADING,
                                       progress=0)
            self.server._post_event("info",
                f"客户端 {self.client_id} 擦除完毕, 开始推送固件...")
            return

        if state == ACK_STATE_SUCCESS:
            self.server._update_client(self.client_id,
                                       state=CLIENT_STATE_COMPLETED,
                                       progress=100)
            self.server._post_event("transfer_complete", {
                "client_id": self.client_id,
            })
            self.phase = PHASE_DONE
            return

        self.server._post_event("info",
            f"客户端 {self.client_id} 收到 ACK(state={state})")


# ============================================================================
# OTA 服务器后台线程
# ============================================================================

class OTAServerThread(threading.Thread):
    """后台 OTA 服务器线程: 监听连接，处理 OTA 协议交互。"""

    def __init__(self):
        super().__init__(daemon=True)
        self._running = False
        self._server_sock: socket.socket | None = None

        # 固件数据 —— A/B 双镜像, 按槽位索引
        #   结构: {SLOT_APP1: {data, size, crc, path, slot}, SLOT_APP2: {...}}
        self.firmware_images: dict[int, dict] = {}

        # 兼容字段: 反映"首选镜像"(一般为 A 镜像), 供 UI 显示用。
        # 真正发送用 firmware_images[目标槽位]。
        self.firmware_data: bytes = b''
        self.firmware_size: int = 0
        self.firmware_crc: int = 0
        self.firmware_path: str = ''
        self.firmware_version: int = DEFAULT_VERSION

        # 服务器参数
        self.bind_addr: str = DEFAULT_BIND_ADDR
        self.port: int = DEFAULT_PORT
        self.chunk_size: int = DEFAULT_CHUNK_SIZE
        self.chunk_delay: float = DEFAULT_CHUNK_DELAY

        # 客户端跟踪 (线程安全由事件队列保证)
        self._clients: dict[str, dict] = {}
        self._clients_lock = threading.Lock()

        # 事件队列 → GUI
        self.event_queue = queue.Queue()

    # ------------------------------------------------------------------
    # 固件加载
    # ------------------------------------------------------------------

    def load_firmware(self, filepath: str) -> bool:
        """加载固件文件(自动配对 A/B 双镜像)。返回 True 成功。

        A/B 乒乓升级要求同一份源码编译成两个链接地址不同的镜像:
            ..._A.bin  → 写入 APP1 (0x08008000)
            ..._B.bin  → 写入 APP2 (0x08080000)
        设备上报"当前槽位"后, 上位机推另一个槽位的镜像, 所以两个都要有。
        """
        if not os.path.exists(filepath):
            self._post_event("error", f"固件文件不存在: {filepath}")
            return False

        pair = self._pair_paths(filepath)
        images: dict[int, dict] = {}
        problems: list[str] = []
        versions: dict[int, int] = {}

        for slot, path in pair.items():
            if path is None:
                problems.append(f"{SLOT_NAMES[slot]} 镜像文件未找到")
                continue
            try:
                with open(path, 'rb') as f:
                    raw = f.read()
            except OSError as e:
                problems.append(f"无法读取 {os.path.basename(path)}: {e}")
                continue

            data = pad_to_align(raw)
            info = {
                'data': data,
                'size': len(data),
                'crc': crc32(data) & 0xFFFFFFFF,   # zlib.crc32 == 设备侧 crc32_calc
                'path': path,
                'slot': slot,
                'raw_size': len(raw),
            }

            err = self._validate_image(info)
            if err:
                problems.append(f"{SLOT_NAMES[slot]} 镜像 {os.path.basename(path)}: {err}")
                continue

            ver = self._parse_version_from_name(path)
            if ver is not None:
                versions[slot] = ver

            images[slot] = info

        if not images:
            self._post_event("error",
                "固件加载失败: " + ("; ".join(problems) if problems
                                  else "没有可用镜像"))
            return False

        # 两个镜像必须同版本 —— 否则升到 A 和升到 B 会得到不同固件
        if len(versions) == 2 and versions[SLOT_APP1] != versions[SLOT_APP2]:
            problems.append(
                f"A/B 版本号不一致 (A=0x{versions[SLOT_APP1]:08X}, "
                f"B=0x{versions[SLOT_APP2]:08X}), 请确认两个镜像是同一次编译产物")
            self._post_event("error", problems[-1])

        # 版本号优先级: 文件名解析 > 保持当前 UI 值
        if len(versions) == 2 and versions[SLOT_APP1] == versions[SLOT_APP2]:
            self.firmware_version = versions[SLOT_APP1]
        elif 1 in versions:
            self.firmware_version = versions[SLOT_APP1]

        self.firmware_images = images

        # 兼容字段取 A 槽(没有 A 就取 B), 只影响 UI 显示
        primary = images.get(SLOT_APP1) or images.get(SLOT_APP2)
        self.firmware_data = primary['data']
        self.firmware_size = primary['size']
        self.firmware_crc = primary['crc']
        self.firmware_path = primary['path']

        self._post_event("firmware_loaded", {
            "path": filepath,
            "filename": os.path.basename(filepath),
            "size": primary['size'],
            "raw_size": primary['raw_size'],
            "crc": primary['crc'],
            "version": self.firmware_version,
            "images": {s: {k: v for k, v in i.items() if k != 'data'}
                       for s, i in images.items()},
            "problems": problems,
        })
        return True

    # ------------------------------------------------------------------
    # 固件加载辅助
    # ------------------------------------------------------------------

    @staticmethod
    def _pair_paths(filepath: str) -> dict:
        """给定 A 或 B 镜像路径, 找出配对的另一个。返回 {槽位: 路径或 None}。"""
        directory = os.path.dirname(filepath)
        name = os.path.basename(filepath)

        m = re.match(r'^(.*)_([AB])\.bin$', name, re.IGNORECASE)
        if m:
            stem, letter = m.group(1), m.group(2).upper()
            other = 'B' if letter == 'A' else 'A'
            cand = {
                SLOT_APP1 if letter == 'A' else SLOT_APP2: filepath,
                SLOT_APP2 if letter == 'A' else SLOT_APP1: os.path.join(
                    directory, f"{stem}_{other}.bin"),
            }
            return {s: (p if os.path.exists(p) else None) for s, p in cand.items()}

        # 不含 _A/_B 后缀: 尝试 <名>_A.bin / <名>_B.bin
        stem = name[:-4] if name.lower().endswith('.bin') else name
        cand = {SLOT_APP1: os.path.join(directory, f"{stem}_A.bin"),
                SLOT_APP2: os.path.join(directory, f"{stem}_B.bin")}
        found = {s: (p if os.path.exists(p) else None) for s, p in cand.items()}

        # 两个都没找到 → 退化成"单镜像", 当 A 用(仅适用于设备当前在 B 区)
        if found[SLOT_APP1] is None and found[SLOT_APP2] is None:
            found[SLOT_APP1] = filepath
        return found

    @staticmethod
    def _parse_version_from_name(filepath: str):
        """从文件名解析版本号, 形如 ..._v1.0.0.2_A.bin → 0x01000002。

        取最后一个匹配(避免把日期 2025.09.22 之类当成版本号)。
        解析不出来返回 None, 由 UI 上的版本输入框决定。
        """
        matches = list(VERSION_RE.finditer(os.path.basename(filepath)))
        if not matches:
            return None
        try:
            major, minor, rev, build = (int(g) for g in matches[-1].groups())
        except ValueError:
            return None
        if max(major, minor, rev, build) > 0xFF:
            return None
        return (major << 24) | (minor << 16) | (rev << 8) | build

    @staticmethod
    def _validate_image(info: dict):
        """镜像预检, 返回 None 表示通过, 否则返回中文错误描述。

        这一层是"早失败": 除了省下白等一次的 248KB 传输, 更重要的是能在
        点下按钮之前就发现"拿错了文件"或"没按对应 IROM 重新编译"——
        位置无关性错误如果留到设备侧, 只会得到一个 ERROR_IMAGE(12),
        排查起来要连串口看日志。
        """
        slot = info['slot']
        size = info['size']
        data = info['data']

        base = APP1_ADDR if slot == SLOT_APP1 else APP2_ADDR
        limit = APP1_SIZE if slot == SLOT_APP1 else APP2_SIZE

        if size > limit:
            return (f"超过 {SLOT_NAMES[slot]} 区容量 "
                    f"({size:,} > {limit:,} bytes)")
        if size < 8:
            return "文件太小, 不像固件镜像"

        sp, rv = struct.unpack_from('<II', data, 0)

        if not (SRAM_START_ADDR <= sp <= SRAM_END_ADDR):
            return (f"初始栈指针 0x{sp:08X} 不在 SRAM "
                    f"(0x{SRAM_START_ADDR:08X}~0x{SRAM_END_ADDR:08X}) 内")

        if not (base <= rv < base + limit):
            return (f"复位向量 0x{rv:08X} 不在 {SLOT_NAMES[slot]} 区 "
                    f"(0x{base:08X}~0x{base + limit:08X}) 内 —— "
                    f"该文件是否用 {SLOT_NAMES[slot]} 镜像的链接地址编译?")

        return None

    def copy_firmware_from(self, other) -> None:
        """把另一个服务器实例的固件状态整套搬过来(重启服务器时用)。"""
        self.firmware_images = dict(other.firmware_images)
        self.firmware_data = other.firmware_data
        self.firmware_size = other.firmware_size
        self.firmware_crc = other.firmware_crc
        self.firmware_path = other.firmware_path
        self.firmware_version = other.firmware_version

    def image_for_slot(self, slot: int):
        """取指定槽位的镜像; 没有则返回 None。"""
        return self.firmware_images.get(slot)

    # ------------------------------------------------------------------
    # 运行 / 停止
    # ------------------------------------------------------------------

    def run(self):
        self._running = True
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.settimeout(1.0)

        try:
            self._server_sock.bind((self.bind_addr, self.port))
            self._server_sock.listen(5)
            self._post_event("server_started", {
                "addr": self.bind_addr,
                "port": self.port,
            })
        except OSError as e:
            self._post_event("error", f"无法绑定端口 {self.port}: {e}")
            self._running = False
            return

        while self._running:
            try:
                client_sock, addr = self._server_sock.accept()
                # 每个客户端一个独立线程
                t = threading.Thread(
                    target=self._handle_client,
                    args=(client_sock, addr),
                    daemon=True,
                )
                t.start()
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    continue
                break

        self._cleanup()

    def stop(self):
        """停止服务器。"""
        self._running = False
        self._cleanup()
        self._post_event("server_stopped", "OTA 服务器已停止")

    def _cleanup(self):
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
            self._server_sock = None

        # 关闭所有客户端连接
        with self._clients_lock:
            for cid, info in list(self._clients.items()):
                try:
                    info['sock'].close()
                except OSError:
                    pass
            self._clients.clear()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def client_count(self) -> int:
        with self._clients_lock:
            return len(self._clients)

    def get_clients(self) -> dict:
        with self._clients_lock:
            return dict(self._clients)

    # ------------------------------------------------------------------
    # 客户端处理
    # ------------------------------------------------------------------

    def _handle_client(self, sock: socket.socket, addr: tuple):
        """处理单个客户端连接的完整 OTA 交互流程。"""
        OTAClientSession(self, sock, addr).run()

    def _register_client(self, client_id: str, sock: socket.socket, addr: tuple):
        with self._clients_lock:
            self._clients[client_id] = {
                'sock': sock, 'addr': addr,
                'version': 0, 'state': CLIENT_STATE_CONNECTED,
                'progress': 0, 'connected_at': datetime.now(),
            }

        self._post_event("client_connected", {
            "client_id": client_id,
            "addr": f"{addr[0]}:{addr[1]}",
        })

    def _unregister_client(self, client_id: str):
        with self._clients_lock:
            info = self._clients.pop(client_id, None)

        if info is not None:
            try:
                info['sock'].close()
            except OSError:
                pass

        self._post_event("client_disconnected", {"client_id": client_id})

    def _update_client(self, client_id: str, **kwargs):
        """更新客户端状态。"""
        with self._clients_lock:
            if client_id in self._clients:
                self._clients[client_id].update(kwargs)

    def _post_event(self, etype: str, message):
        """向 GUI 事件队列投递消息。"""
        self.event_queue.put({
            "type": etype,
            "message": message,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })


# ============================================================================
# GUI 主窗口
# ============================================================================

class OTAGUI:
    """OTA 固件升级工具主窗口。"""

    def __init__(self, firmware_path: str = ''):
        self.server: OTAServerThread | None = None

        # 主窗口
        self.root = tk.Tk()
        self.root.title("OTA 固件升级服务器 v1.0")
        self.root.geometry("1050x680")
        self.root.minsize(900, 550)

        self._build_ui()

        # 如果命令行传入了固件路径, 自动加载
        if firmware_path:
            self._start_server(firmware_path)
        else:
            self._start_server()

    # ================================================================
    #  UI 构建
    # ================================================================

    def _build_ui(self):
        """构建全部界面。"""
        # ---- 顶部状态栏 ----
        self._build_top_bar()

        # ---- 主体: 左右分栏 ----
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_panel = ttk.Frame(paned, width=420)
        left_panel.pack_propagate(False)

        right_panel = ttk.Frame(paned)

        paned.add(left_panel, weight=0)
        paned.add(right_panel, weight=1)

        self._build_left_panel(left_panel)
        self._build_right_panel(right_panel)

        # ---- 底部状态栏 ----
        self._build_bottom_bar()

    def _build_top_bar(self):
        """顶部服务器状态栏。"""
        frame = ttk.Frame(self.root, relief=tk.RIDGE, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(5, 0))

        # 状态灯
        self.status_canvas = tk.Canvas(frame, width=20, height=20,
                                       highlightthickness=0)
        self.status_canvas.pack(side=tk.LEFT, padx=10)
        self._draw_status_dot("gray")

        self.status_label = ttk.Label(frame, text="服务器状态: 未启动",
                                      font=("Microsoft YaHei", 10, "bold"))
        self.status_label.pack(side=tk.LEFT, padx=5)

        self.port_label = ttk.Label(frame, text=f"端口: {DEFAULT_PORT}",
                                    font=("Microsoft YaHei", 9))
        self.port_label.pack(side=tk.LEFT, padx=15)

        self.client_count_label = ttk.Label(frame, text="已连接客户端: 0",
                                            font=("Microsoft YaHei", 9))
        self.client_count_label.pack(side=tk.RIGHT, padx=10)

        ttk.Separator(frame, orient=tk.VERTICAL).pack(
            side=tk.RIGHT, fill=tk.Y, padx=5)

    def _build_left_panel(self, parent):
        """左侧控制面板。"""
        # ── 固件文件区 ──
        fw_frame = ttk.LabelFrame(parent, text="固件文件", padding=8)
        fw_frame.pack(fill=tk.X, pady=(0, 10))

        # 文件路径显示 + 选择按钮
        file_row = ttk.Frame(fw_frame)
        file_row.pack(fill=tk.X, pady=(0, 8))

        self.file_path_var = tk.StringVar(value="(尚未选择固件文件)")
        self.file_entry = ttk.Entry(file_row, textvariable=self.file_path_var,
                                    state="readonly", font=("Consolas", 9))
        self.file_entry.pack(side=tk.LEFT, fill=tk.X, expand=True,
                             padx=(0, 5))

        self.select_btn = ttk.Button(file_row, text="打开升级文件...",
                                     command=self._select_firmware_file)
        self.select_btn.pack(side=tk.RIGHT)

        # 文件信息
        self.fw_info_frame = ttk.Frame(fw_frame)
        self.fw_info_frame.pack(fill=tk.X, pady=(0, 3))

        self.fw_size_label = ttk.Label(self.fw_info_frame, text="大小: —",
                                       font=("Consolas", 9))
        self.fw_size_label.pack(side=tk.LEFT, padx=(0, 15))

        self.fw_crc_label = ttk.Label(self.fw_info_frame, text="CRC32: —",
                                      font=("Consolas", 9))
        self.fw_crc_label.pack(side=tk.LEFT)

        # 版本号
        ver_row = ttk.Frame(fw_frame)
        ver_row.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(ver_row, text="固件版本:", font=("Microsoft YaHei", 9)).pack(
            side=tk.LEFT, padx=(0, 5))

        self.version_var = tk.StringVar(value=f"0x{DEFAULT_VERSION:08X}")
        self.version_entry = ttk.Entry(ver_row, textvariable=self.version_var,
                                       width=14, font=("Consolas", 10))
        self.version_entry.pack(side=tk.LEFT)
        ttk.Label(ver_row, text="(如 0x01000002)", foreground="gray",
                  font=("Microsoft YaHei", 8)).pack(side=tk.LEFT, padx=5)

        # ── 服务器设置 ──
        svr_frame = ttk.LabelFrame(parent, text="服务器设置", padding=8)
        svr_frame.pack(fill=tk.X, pady=(0, 10))

        # 端口
        port_row = ttk.Frame(svr_frame)
        port_row.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(port_row, text="监听端口:", font=("Microsoft YaHei", 9)).pack(
            side=tk.LEFT, padx=(0, 5))
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Spinbox(port_row, textvariable=self.port_var, from_=1024, to=65535,
                    width=8).pack(side=tk.LEFT)

        # 绑定地址
        addr_row = ttk.Frame(svr_frame)
        addr_row.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(addr_row, text="绑定地址:", font=("Microsoft YaHei", 9)).pack(
            side=tk.LEFT, padx=(0, 5))
        self.bind_var = tk.StringVar(value=DEFAULT_BIND_ADDR)
        ttk.Combobox(addr_row, textvariable=self.bind_var,
                     values=["0.0.0.0", OTA_SERVER_IP, "127.0.0.1"],
                     width=14).pack(side=tk.LEFT)
        ttk.Label(addr_row, text=f"(通信板固定连 {OTA_SERVER_IP}, "
                                 f"本机建该地址最省事; 0.0.0.0 监听所有网卡)",
                  foreground="gray",
                  font=("Microsoft YaHei", 8)).pack(side=tk.LEFT, padx=5)

        # 传输参数
        tx_row = ttk.Frame(svr_frame)
        tx_row.pack(fill=tk.X, pady=(5, 0))
        ttk.Label(tx_row, text="块大小:", font=("Microsoft YaHei", 9)).pack(
            side=tk.LEFT, padx=(0, 3))
        self.chunk_size_var = tk.IntVar(value=DEFAULT_CHUNK_SIZE)
        ttk.Spinbox(tx_row, textvariable=self.chunk_size_var,
                    from_=128, to=OTA_MAX_PAYLOAD, increment=128, width=6).pack(
            side=tk.LEFT, padx=(0, 10))
        ttk.Label(tx_row, text="间隔(ms):", font=("Microsoft YaHei", 9)).pack(
            side=tk.LEFT, padx=(0, 3))
        self.chunk_delay_var = tk.IntVar(value=int(DEFAULT_CHUNK_DELAY * 1000))
        ttk.Spinbox(tx_row, textvariable=self.chunk_delay_var,
                    from_=0, to=2000, increment=10, width=6).pack(
            side=tk.LEFT)

        # ── 控制按钮 ──
        ctrl_frame = ttk.LabelFrame(parent, text="服务器控制", padding=8)
        ctrl_frame.pack(fill=tk.X, pady=(0, 10))

        self.start_btn = ttk.Button(ctrl_frame, text="启动 OTA 服务器",
                                    command=self._toggle_server)
        self.start_btn.pack(fill=tk.X, pady=2)

        self.restart_btn = ttk.Button(ctrl_frame, text="重启服务器",
                                      command=self._restart_server,
                                      state=tk.DISABLED)
        self.restart_btn.pack(fill=tk.X, pady=2)

        # ── 快速操作提示 ──
        tip_frame = ttk.LabelFrame(parent, text="使用步骤", padding=8)
        tip_frame.pack(fill=tk.X)

        tips = (
            "① 把 A/B 两个镜像放同一目录, 选其中任意一个\n"
            "   (文件名形如 xxx_v1.0.0.2_A.bin / _B.bin)\n"
            "② 确认版本号大于通信板当前版本(必须递增)\n"
            "③ 本机网卡配成 192.168.1.99, 点击「启动服务器」\n"
            "④ 通信板连上后自动推送到它没在跑的那个区\n"
            "⑤ 等待进度条满 + 日志出现「固件传输完成」"
        )
        ttk.Label(tip_frame, text=tips, justify=tk.LEFT,
                  font=("Microsoft YaHei", 8), foreground="#569CD6").pack(
            anchor=tk.W)

        # ── 关于 ──
        about_frame = ttk.LabelFrame(parent, text="关于", padding=5)
        about_frame.pack(fill=tk.X, pady=(10, 0))
        about_text = ("OTA 固件升级工具 v1.0\n"
                      "协议: STM32F429 OTA (Server-Push)\n"
                      f"默认端口: {DEFAULT_PORT}\n"
                      "仅使用 Python 标准库")
        ttk.Label(about_frame, text=about_text, justify=tk.LEFT,
                  font=("Microsoft YaHei", 8), foreground="gray").pack()

    def _build_right_panel(self, parent):
        """右侧: 客户端列表 + 进度 + 日志。"""
        # ── 客户端列表 ──
        cli_frame = ttk.LabelFrame(parent, text="客户端列表", padding=3)
        cli_frame.pack(fill=tk.X, pady=(0, 5))

        columns = ("client", "version", "state", "progress")
        self.cli_tree = ttk.Treeview(cli_frame, columns=columns,
                                     show="headings", height=4)
        self.cli_tree.heading("client", text="客户端地址")
        self.cli_tree.heading("version", text="固件版本")
        self.cli_tree.heading("state", text="状态")
        self.cli_tree.heading("progress", text="进度")

        self.cli_tree.column("client", width=160, anchor=tk.CENTER)
        self.cli_tree.column("version", width=110, anchor=tk.CENTER)
        self.cli_tree.column("state", width=100, anchor=tk.CENTER)
        self.cli_tree.column("progress", width=80, anchor=tk.CENTER)

        cli_scroll = ttk.Scrollbar(cli_frame, orient=tk.VERTICAL,
                                   command=self.cli_tree.yview)
        self.cli_tree.configure(yscrollcommand=cli_scroll.set)

        self.cli_tree.pack(side=tk.LEFT, fill=tk.X, expand=True)
        cli_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # ── 传输进度 ──
        prog_frame = ttk.LabelFrame(parent, text="传输进度", padding=5)
        prog_frame.pack(fill=tk.X, pady=(0, 5))

        self.progress_bar = ttk.Progressbar(prog_frame, mode='determinate',
                                            length=100)
        self.progress_bar.pack(fill=tk.X, pady=(2, 5))

        self.progress_label = ttk.Label(prog_frame, text="等待客户端连接...",
                                        font=("Microsoft YaHei", 9),
                                        foreground="gray")
        self.progress_label.pack(anchor=tk.W)

        # ── 日志区 ──
        log_frame = ttk.LabelFrame(parent, text="通信日志", padding=3)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, wrap=tk.WORD, state=tk.DISABLED,
                                font=("Consolas", 9),
                                bg="#1E1E1E", fg="#D4D4D4",
                                insertbackground="white")
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL,
                                   command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)

        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 颜色标签 (与 board_server.py 风格统一)
        self.log_text.tag_configure("info",    foreground="#569CD6")
        self.log_text.tag_configure("success", foreground="#6A9955")
        self.log_text.tag_configure("warning", foreground="#CE9178")
        self.log_text.tag_configure("error",   foreground="#F44747")
        self.log_text.tag_configure("send",    foreground="#C586C0")
        self.log_text.tag_configure("recv",    foreground="#DCDCAA")
        self.log_text.tag_configure("time",    foreground="#808080")
        self.log_text.tag_configure("progress", foreground="#4EC9B0")

    def _build_bottom_bar(self):
        """底部状态栏。"""
        frame = ttk.Frame(self.root, relief=tk.SUNKEN, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.bottom_label = ttk.Label(frame, text="就绪 — 请先选择固件文件",
                                      font=("Microsoft YaHei", 8))
        self.bottom_label.pack(side=tk.LEFT, padx=5)

        self.msg_count_label = ttk.Label(frame, text="消息: 0",
                                         font=("Microsoft YaHei", 8))
        self.msg_count_label.pack(side=tk.RIGHT, padx=5)

    # ================================================================
    #  日志
    # ================================================================

    def _log(self, tag: str, message: str):
        now = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{now}] ", "time")
        self.log_text.insert(tk.END, message + "\n", tag)
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _clear_log(self):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ================================================================
    #  状态灯
    # ================================================================

    def _draw_status_dot(self, color: str):
        self.status_canvas.delete("all")
        self.status_canvas.create_oval(3, 3, 17, 17, fill=color, outline="")

    # ================================================================
    #  固件文件选择
    # ================================================================

    def _select_firmware_file(self):
        """打开文件对话框选择固件。"""
        filepath = filedialog.askopenfilename(
            title="选择固件文件 (.bin)",
            filetypes=[
                ("固件文件", "*.bin"),
                ("所有文件", "*.*"),
            ],
            initialdir=os.path.dirname(__file__),
        )
        if not filepath:
            return

        if self.server is None:
            self._start_server()

        if self.server.load_firmware(filepath):
            # 自动更新UI (触发于 _poll_events 中处理 firmware_loaded 事件)
            pass

    # ================================================================
    #  服务器控制
    # ================================================================

    def _start_server(self, firmware_path: str = ''):
        """创建 OTA 服务器线程 (不自动启动，等待用户点击按钮)。"""
        self.server = OTAServerThread()
        self.server.port = self.port_var.get()
        self.server.bind_addr = self.bind_var.get()

        if firmware_path:
            self.server.load_firmware(firmware_path)

        # 注意: 不调用 self.server.start() — 服务器线程只在用户点击按钮后启动

        self._draw_status_dot("gray")
        self.status_label.config(text="服务器状态: 未启动")
        self.bottom_label.config(text="服务器已就绪, 请点击「启动」按钮")
        self.start_btn.config(text="启动 OTA 服务器", state=tk.NORMAL)
        self.restart_btn.config(state=tk.DISABLED)

        # 开始轮询事件
        self._poll_events()

    def _toggle_server(self):
        """启动/停止 OTA 服务器。"""
        if self.server is None:
            return

        if self.server.is_running:
            # 正在运行 → 停止
            self.server.stop()
            self._log("warning", "OTA 服务器正在停止...")
            return

        # 未运行 → 启动
        # 检查是否已加载固件
        if not self.server.firmware_images:
            if not messagebox.askyesno(
                    "未加载固件",
                    "尚未选择固件文件。\n\n"
                    "没有固件时服务器只能接收客户端 HELLO，"
                    "但不会推送升级。\n\n"
                    "是否继续启动？"):
                return

        if not self._apply_ui_params(self.server):
            return

        # 启动后台线程
        self.server.start()
        self._log("info", "OTA 服务器正在启动, 等待通信板连接...")

    def _apply_ui_params(self, server) -> bool:
        """把界面上的参数套到服务器实例上。版本号非法时提示并返回 False。"""
        try:
            version = int(self.version_var.get().strip(), 16)
        except ValueError:
            messagebox.showerror("版本号非法",
                f"「固件版本」必须是十六进制整数, 例如 0x01000002。\n"
                f"当前填写: {self.version_var.get()}")
            return False

        server.port = self.port_var.get()
        server.bind_addr = self.bind_var.get()
        # 单帧上限固定 1024 (OTA_MAX_PAYLOAD), 界面填大了也不能超
        server.chunk_size = min(int(self.chunk_size_var.get()), OTA_MAX_PAYLOAD)
        server.chunk_delay = self.chunk_delay_var.get() / 1000.0
        server.firmware_version = version
        return True

    def _restart_server(self):
        """重启服务器(保留已加载的固件)。"""
        if self.server is None:
            return

        if not self._apply_ui_params(self.server):
            return

        self._log("warning", "服务器正在重启...")

        old_server = self.server
        old_server.stop()

        # 创建新服务器实例并整套搬过固件状态
        self.server = OTAServerThread()
        self.server.copy_firmware_from(old_server)
        self._apply_ui_params(self.server)

        # 自动启动
        self.server.start()
        self._log("success", "服务器已重启")

    # ================================================================
    #  事件轮询 (GUI 线程安全)
    # ================================================================

    def _poll_events(self):
        """定时从事件队列取出消息并更新 GUI。"""
        if self.server is None:
            self.root.after(100, self._poll_events)
            return

        msg_count = 0
        try:
            while True:
                event = self.server.event_queue.get_nowait()
                try:
                    self._process_event(event)
                except Exception as e:
                    self._log("error", f"事件处理异常: {e}")
                msg_count += 1
        except queue.Empty:
            pass

        if msg_count > 0:
            try:
                current = int(self.msg_count_label.cget("text").split(": ")[-1])
                self.msg_count_label.config(text=f"消息: {current + msg_count}")
            except Exception:
                pass

        # 定期刷新客户端列表
        self._refresh_client_list()

        self.root.after(100, self._poll_events)

    def _process_event(self, event: dict):
        """处理单个事件。"""
        etype = event["type"]
        msg = event["message"]

        if etype == "server_started":
            addr = msg["addr"] if isinstance(msg, dict) else "?"
            port = msg["port"] if isinstance(msg, dict) else "?"
            self._draw_status_dot("green")
            self.status_label.config(text="服务器状态: 运行中")
            self.port_label.config(text=f"端口: {port}")
            self.bottom_label.config(text=f"监听 {addr}:{port}, 等待通信板连接...")
            self.start_btn.config(text="停止 OTA 服务器", state=tk.NORMAL)
            self.restart_btn.config(state=tk.NORMAL)
            self._log("success",
                      f"━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self._log("success",
                      f"OTA 服务器已启动 — 监听 {addr}:{port}")
            if self.server and self.server.firmware_images:
                for slot, info in sorted(self.server.firmware_images.items()):
                    self._log("info",
                        f"待推送 [{SLOT_NAMES[slot]}区]: "
                        f"{os.path.basename(info['path'])} "
                        f"({info['size']} bytes, "
                        f"CRC32=0x{info['crc']:08X})")
                self._log("info",
                    f"目标版本: 0x{self.server.firmware_version:08X} "
                    f"(设备上报版本更低时, 自动推送到它当前没在跑的那个区)")

        elif etype == "server_stopped":
            self._draw_status_dot("gray")
            self.status_label.config(text="服务器状态: 已停止")
            self.port_label.config(text=f"端口: {self.port_var.get()}")
            self.bottom_label.config(text="服务器已停止")
            self.start_btn.config(text="启动 OTA 服务器", state=tk.NORMAL)
            self._log("warning", str(msg))

        elif etype == "firmware_loaded":
            images = msg.get("images", {})
            self.file_path_var.set(msg["path"])

            size_kb = msg["size"] / 1024
            if size_kb >= 1024:
                size_str = f"{size_kb/1024:.2f} MB"
            else:
                size_str = f"{size_kb:.1f} KB"
            self.fw_size_label.config(
                text=f"{SLOT_NAMES.get(next(iter(images), SLOT_APP1), '?')}区: "
                     f"{msg['size']:,} bytes ({size_str})")
            self.fw_crc_label.config(
                text=f"CRC32: 0x{msg['crc']:08X}")
            self.version_var.set(f"0x{msg['version']:08X}")

            for slot in (SLOT_APP1, SLOT_APP2):
                info = images.get(slot)
                if info is None:
                    self._log("warning",
                        f"  {SLOT_NAMES[slot]} 区镜像: 未加载 "
                        f"(设备若正运行在另一区, 仍可升级)")
                    continue
                pad_note = ("" if info['size'] == info['raw_size']
                            else f", 已补 0x00 至 4 字节对齐 "
                                 f"(原 {info['raw_size']:,})")
                self._log("success",
                    f"  {SLOT_NAMES[slot]} 区镜像: "
                    f"{os.path.basename(info['path'])} "
                    f"({info['size']:,} bytes, CRC32=0x{info['crc']:08X}"
                    f"{pad_note})")

            loaded = len(images)
            if loaded == 2:
                state = "A/B 双镜像就绪"
            elif loaded == 1:
                state = f"仅 {SLOT_NAMES[next(iter(images))]} 区镜像就绪"
            else:
                state = "无可用镜像"
            self.bottom_label.config(text=f"固件已加载 ({state}) — 请启动服务器")

            for p in msg.get("problems", []):
                self._log("warning", f"  提示: {p}")

        elif etype == "client_connected":
            self._log("info",
                      f"━━ 客户端已连接: {msg['client_id']}")
            self._update_client_row(msg['client_id'],
                                    state=CLIENT_STATE_CONNECTED,
                                    progress=0)

        elif etype == "client_version":
            cid = msg['client_id']
            slot_name = msg.get('slot_name', '?')
            ver = f"0x{msg['version']:08X} ({slot_name}区)"
            self._log("info",
                f"  客户端 {cid} 固件版本: 0x{msg['version']:08X}, "
                f"运行于 {slot_name} 区")
            self._update_client_row(cid, version=ver)

        elif etype == "client_disconnected":
            self._log("warning", f"  客户端断开: {msg['client_id']}")
            iid = self._client_iid(msg['client_id'])
            if self.cli_tree.exists(iid):
                self.cli_tree.delete(iid)

        elif etype == "frame_rx":
            self._log("recv",
                f"  ← [{msg['client_id']}] "
                f"{msg['cmd_name']} (seq={msg['seq']}, len={msg['len']})")

        elif etype == "frame_tx":
            self._log("send",
                f"  → [{msg['client_id']}] "
                f"{msg['cmd_name']} ({msg['desc']}) "
                f"(seq={msg['seq']}, len={msg['len']})")

        elif etype == "transfer_progress":
            pct = msg['progress']
            self.progress_bar['value'] = pct
            offset_kb = msg['offset'] / 1024
            total_kb = msg['total'] / 1024
            self.progress_label.config(
                text=f"传输中: {msg['offset']:,} / {msg['total']:,} bytes "
                     f"({offset_kb:.1f} / {total_kb:.1f} KB) — {pct}%")
            self._update_client_row(msg['client_id'], progress=pct)
            if pct % 10 == 0:
                self._log("progress",
                    f"  传输进度 [{msg['client_id']}]: {pct}% "
                    f"({msg['offset']:,} / {msg['total']:,} bytes)")

        elif etype == "transfer_complete":
            self.progress_bar['value'] = 100
            self.progress_label.config(
                text="升级完成! 通信板正在重启, 请等待设备以新版本重新连接...")
            self._update_client_row(msg['client_id'],
                                    state=CLIENT_STATE_COMPLETED, progress=100)
            self._log("success",
                f"━━ 固件传输完成 [{msg['client_id']}] — "
                f"通信板已校验通过并写入启动信息, 即将重启")

        elif etype == "info":
            self._log("info", str(msg))

        elif etype == "warning":
            self._log("warning", str(msg))

        elif etype == "error":
            self._log("error", str(msg))
            self.bottom_label.config(text=f"错误: {msg}")

    # ================================================================
    #  客户端列表维护
    # ================================================================

    def _client_iid(self, client_id: str) -> str:
        return client_id

    def _update_client_row(self, client_id: str, **kwargs):
        """更新客户端列表中的一行。"""
        iid = self._client_iid(client_id)
        if not self.cli_tree.exists(iid):
            self.cli_tree.insert("", tk.END, iid=iid,
                                 values=(client_id, "—", "已连接", "0%"))
        values = list(self.cli_tree.item(iid, "values"))
        if "version" in kwargs:
            values[1] = kwargs["version"]
        if "state" in kwargs:
            values[2] = kwargs["state"]
        if "progress" in kwargs:
            values[3] = f"{kwargs['progress']}%"
        self.cli_tree.item(iid, values=values)

    def _refresh_client_list(self):
        """定期从服务器同步客户端列表。"""
        if self.server is None or not self.server.is_running:
            return

        clients = self.server.get_clients()
        seen = set()

        for cid, info in clients.items():
            iid = self._client_iid(cid)
            seen.add(iid)
            if not self.cli_tree.exists(iid):
                ver_str = (f"0x{info.get('version', 0):08X}"
                           if info.get('version') else "—")
                self.cli_tree.insert("", tk.END, iid=iid,
                    values=(cid, ver_str,
                            info.get('state', CLIENT_STATE_CONNECTED),
                            f"{info.get('progress', 0)}%"))
            else:
                values = list(self.cli_tree.item(iid, "values"))
                if info.get('version'):
                    values[1] = f"0x{info['version']:08X}"
                values[2] = info.get('state', values[2])
                values[3] = f"{info.get('progress', 0)}%"
                self.cli_tree.item(iid, values=values)

        # 移除已断开的客户端行
        for iid in self.cli_tree.get_children():
            if iid not in seen:
                self.cli_tree.delete(iid)

        # 更新计数
        count = len(clients)
        self.client_count_label.config(text=f"已连接客户端: {count}")

    # ================================================================
    #  生命周期
    # ================================================================

    def run(self):
        """运行 GUI 主循环。"""
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        """关闭窗口时清理。"""
        if self.server:
            self.server.stop()
        self.root.destroy()


# ============================================================================
# 入口
# ============================================================================

def main():
    """程序入口。支持可选的 -f 命令行参数自动加载固件。"""
    parser = argparse.ArgumentParser(
        description="OTA 固件升级服务器 GUI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用方法:
  python ota_gui.py                    打开 GUI，手动选择固件文件
  python ota_gui.py -f firmware.bin    自动加载固件文件后打开 GUI
        """,
    )
    parser.add_argument(
        "-f", "--firmware",
        default="",
        help="固件文件路径 (.bin)，可选",
    )
    args = parser.parse_args()

    # Windows 高分屏适配
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = OTAGUI(firmware_path=args.firmware)
    app.run()


if __name__ == "__main__":
    main()
