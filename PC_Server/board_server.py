#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 通信板 上位机服务器程序  —  Board Communication Host Server
================================================================================

 功能概述:
   - TCP 服务器监听 0.0.0.0:5001，等待通信板连接
   - 收到申请配置帧后自动下发板卡配置 (BoardCfgTab)
   - 手动/自动发送校时命令 (Unix 时间戳)
   - 手动/自动发送心跳应答
   - 手动发送软复位命令
   - 解析并显示通信板日志 (启动/重连/看门狗复位/端口错误/曲线上传/校时)
   - 显示实时串口数据帧 (可折叠/限速，防止刷屏)
   - 显示网络连接/断开事件

 通信协议说明 (与 client.c / main.c 配套):
   板卡 → PC 帧格式 (11+N 字节):
     [0] 端口号    (0=控制通道, 1-6=串口)
     [1] 主类型    (0x07=申请配置, 0x50=日志)
     [2] 子类型
     [3] 标志/扩展
     [4] 分机号/扩展
     [5-6] 数据长度 (little-endian)
     [7-10] 链路掩码或数据区
     [11+] payload

   PC → 板卡 命令格式:
     配置下发:  [0x00, 0xA1, 0x2C, 0x01, config_data...]
     校时命令:  [0x00, 0xA3, 0x04, 0x00, unix_ts(4bytes LE)]
     软复位:    [0x00, 0xA2, 0x00, 0x00]
     心跳应答:  [0x00, 0xA5, 0x00, 0x00]

 运行要求:
     Python 3.6+, 仅使用标准库 (tkinter, socket, threading 等)
================================================================================
"""

import socket
import struct
import threading
import time
import queue
import json
import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime
from enum import IntEnum
from pathlib import Path

# ============================================================================
# 常量定义 (与嵌入式代码保持一致)
# ============================================================================

SERVER_PORT      = 5001
LISTEN_BACKLOG   = 1
RECV_BUF_SIZE    = 4096
UART_PORT_COUNT  = 6
BOARD_CONFIG_BYTES = 450       # 6 ports × 75 bytes
UART_MODULE_MAX_COUNT = 32
CONFIG_FILE      = "board_config.json"

# ---- 命令字 ----
NET_CMD_CONFIG        = 0x00
NET_CMD_CONFIG_MAGIC0 = 0xA1
NET_CMD_CONFIG_MAGIC1 = 0x2C
NET_CMD_CONFIG_MAGIC2 = 0x01
NET_CMD_TIME_MAGIC0   = 0xA3
NET_CMD_TIME_MAGIC1   = 0x04
NET_CMD_TIME_MAGIC2   = 0x00
NET_CMD_RESET_MAGIC0  = 0xA2
NET_CMD_RESET_MAGIC1  = 0x00
NET_CMD_RESET_MAGIC2  = 0x00
NET_CMD_HEARTBEAT_M0  = 0xA5
NET_CMD_HEARTBEAT_M1  = 0x00
NET_CMD_HEARTBEAT_M2  = 0x00

# ---- 帧识别 ----
FRAME_TYPE_CONFIG_REQ = 0x07   # 申请配置帧
FRAME_TYPE_LOG        = 0x50   # 日志帧

# ---- 日志 ID 映射 ----
LOG_MESSAGES = {
    1: "系统启动 (System Start)",
    2: "网络重连成功 (Network Reconnected)",
    3: "看门狗复位 (Watchdog Reset)",  # extra = WTD_RESET 复位码
    4: "端口数据错误 (Port Data Error)",
    5: "端口曲线上传完成 (Port Curve Upload Complete)",
    6: "校时完成 (Time Calibration Done)",
}

# ---- 看门狗复位码映射 ----
WTD_RESET_CODES = {
    0:  "正常 (Normal)",
    1:  "配置变更复位 (Config Changed)",
    2:  "上位机软复位 (PC Soft Reset)",
    21: "USART1 任务超时",
    22: "USART2 任务超时",
    23: "USART3 任务超时",
    24: "UART4 任务超时",
    25: "UART5 任务超时",
    26: "USART6 任务超时",
    27: "申请配置帧发送失败",
    28: "TCP 发送任务超时",
    29: "心跳超时 (上位机无应答)",
    30: "日志帧发送失败 (Log Frame Send Failed)",
}

# ---- 波特率编码 ----
BAUD_RATE_MAP = {0: "未配置", 1: 9600, 2: 19200, 3: 38400, 4: 57600}

# ---- 模块配置表 (镜像 C 代码 uart_module_profiles[]) ----
# 每个模块类型对应: input_regs(输入寄存器数), holding_regs(保持寄存器数),
#   has_alarm, uses_custom_curve, realtime_bytes
MODULE_PROFILES = {
    0x31: {"input_regs": 13,  "holding_regs": 0,   "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x32: {"input_regs": 13,  "holding_regs": 11,  "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x33: {"input_regs": 34,  "holding_regs": 32,  "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x34: {"input_regs": 10,  "holding_regs": 8,   "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x35: {"input_regs": 30,  "holding_regs": 28,  "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x36: {"input_regs": 8,   "holding_regs": 6,   "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x37: {"input_regs": 29,  "holding_regs": 27,  "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 58},
    0x38: {"input_regs": 8,   "holding_regs": 5,   "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 16},
    0x39: {"input_regs": 11,  "holding_regs": 5,   "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 22},
    0x47: {"input_regs": 50,  "holding_regs": 48,  "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x48: {"input_regs": 133, "holding_regs": 131, "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x49: {"input_regs": 117, "holding_regs": 115, "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x4A: {"input_regs": 62,  "holding_regs": 60,  "has_alarm": 1, "uses_custom_curve": 0, "realtime_bytes": 0},
    0x4B: {"input_regs": 68,  "holding_regs": 66,  "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 136},
    0x4C: {"input_regs": 23,  "holding_regs": 21,  "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 46},
    0x4D: {"input_regs": 26,  "holding_regs": 24,  "has_alarm": 1, "uses_custom_curve": 1, "realtime_bytes": 52},
}

def get_module_profile(module_type: int) -> dict:
    """返回模块配置, 未匹配时回退到 0x31 (信号机开关量)。"""
    return MODULE_PROFILES.get(module_type, MODULE_PROFILES[0x31])

# ---- 端口名称 (镜像 C 代码 uart_hw_config[]) ----
PORT_NAMES = ["USART1", "USART2", "USART3", "UART4", "UART5", "USART6"]

# ---- 端口类型 ----
PORT_TYPE_MAP = {0: "禁用", 1: "自定义曲线", 3: "Modbus 从机", 4: "Modbus 主站"}

# ---- 模块类型 ----
MODULE_TYPE_MAP = {
    0x00: "未配置",
    0x31: "信号机开关量",   0x32: "信号机",       0x33: "25Hz 轨道接收器",
    0x34: "DC/AC 转换器",  0x35: "25Hz 发送器",   0x36: "采集模块",
    0x37: "交流转辙机",    0x38: "直流四线制",    0x39: "直流六线制",
    0x47: "高频信号机",    0x48: "高频轨道接收器", 0x49: "高频发送器",
    0x4A: "高频采集模块",  0x4B: "高频交流转辙机", 0x4C: "高频直流四线制",
    0x4D: "高频直流六线制",
}


# ============================================================================
# 板卡配置数据结构 (BoardConfig_t, 每端口 75 字节, 共 6 端口 = 450 字节)
# ============================================================================

class BoardConfig:
    """管理 6 路端口的板卡配置，支持 JSON 持久化。"""

    # 每端口配置字段
    FIELD_NAMES = [
        "PortNum", "PortType", "BautRate", "DataBit", "StopBit", "Parity",
        "MainProType", "SubProType", "FenjiNum", "QueryPeriod", "SlaveModuelNum",
    ]

    def __init__(self):
        self.ports = []  # list of 6 dicts
        for i in range(UART_PORT_COUNT):
            # 端口2 默认配置: Modbus主站, 挂载1个信号机开关量从机
            if i == 1:
                port = {
                    "PortNum":       i + 1,
                    "PortType":      4,   # Modbus主站
                    "BautRate":      2,   # 19200
                    "DataBit":       8,
                    "StopBit":       1,
                    "Parity":        0,   # 无校验
                    "MainProType":   1,
                    "SubProType":    2,
                    "FenjiNum":      10,
                    "QueryPeriod":   100,
                    "SlaveModuelNum": 1,
                    "SlaveModuelAddress": [0] * UART_MODULE_MAX_COUNT,
                    "SlaveModuelType":    [0] * UART_MODULE_MAX_COUNT,
                }
                port["SlaveModuelAddress"][0] = 1
                port["SlaveModuelType"][0]    = 0x31  # 信号机开关量
            else:
                port = {
                    "PortNum":       i + 1,
                    "PortType":      0,   # 0=禁用
                    "BautRate":      2,   # 19200
                    "DataBit":       8,
                    "StopBit":       1,
                    "Parity":        0,
                    "MainProType":   0,
                    "SubProType":    0,
                    "FenjiNum":      i + 1,
                    "QueryPeriod":   100,
                    "SlaveModuelNum": 0,
                    "SlaveModuelAddress": [0] * UART_MODULE_MAX_COUNT,
                    "SlaveModuelType":    [0] * UART_MODULE_MAX_COUNT,
                }
            self.ports.append(port)

    def to_bytes(self) -> bytes:
        """序列化为 450 字节的 BoardCfgTab 二进制数据。"""
        data = bytearray()
        for port in self.ports:
            data.append(port["PortNum"])
            data.append(port["PortType"])
            data.append(port["BautRate"])
            data.append(port["DataBit"])
            data.append(port["StopBit"])
            data.append(port["Parity"])
            data.append(port["MainProType"])
            data.append(port["SubProType"])
            data.append(port["FenjiNum"])
            data.append(port["QueryPeriod"])
            data.append(port["SlaveModuelNum"])
            data.extend(port["SlaveModuelAddress"])
            data.extend(port["SlaveModuelType"])
        return bytes(data)

    def from_bytes(self, data: bytes):
        """从二进制数据反序列化。"""
        if len(data) < BOARD_CONFIG_BYTES:
            return
        offset = 0
        for i in range(UART_PORT_COUNT):
            self.ports[i]["PortNum"]       = data[offset]
            self.ports[i]["PortType"]      = data[offset + 1]
            self.ports[i]["BautRate"]      = data[offset + 2]
            self.ports[i]["DataBit"]       = data[offset + 3]
            self.ports[i]["StopBit"]       = data[offset + 4]
            self.ports[i]["Parity"]        = data[offset + 5]
            self.ports[i]["MainProType"]   = data[offset + 6]
            self.ports[i]["SubProType"]    = data[offset + 7]
            self.ports[i]["FenjiNum"]      = data[offset + 8]
            self.ports[i]["QueryPeriod"]   = data[offset + 9]
            self.ports[i]["SlaveModuelNum"] = data[offset + 10]
            self.ports[i]["SlaveModuelAddress"] = list(data[offset+11 : offset+43])
            self.ports[i]["SlaveModuelType"]    = list(data[offset+43 : offset+75])
            offset += 75

    def save(self, filepath: str):
        """保存为 JSON 文件。"""
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(self.ports, f, indent=2, ensure_ascii=False)

    def load(self, filepath: str):
        """从 JSON 文件加载。"""
        try:
            with open(filepath, "r", encoding="utf-8") as f:
                loaded = json.load(f)
                if isinstance(loaded, list) and len(loaded) == UART_PORT_COUNT:
                    for i, port_data in enumerate(loaded):
                        self.ports[i].update(port_data)
                    return True
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        return False


# ============================================================================
# 协议编解码
# ============================================================================

def make_config_download_cmd(config_bytes: bytes) -> bytes:
    """构造配置下发命令: [0x00, 0xA1, 0x2C, 0x01, config...]"""
    return bytes([NET_CMD_CONFIG, NET_CMD_CONFIG_MAGIC0,
                  NET_CMD_CONFIG_MAGIC1, NET_CMD_CONFIG_MAGIC2]) + config_bytes


def make_time_sync_cmd(timestamp: int = None) -> bytes:
    """构造校时命令: [0x00, 0xA3, 0x04, 0x00, timestamp(4 bytes LE)]"""
    if timestamp is None:
        timestamp = int(time.time())
    return struct.pack("<4BI",
        NET_CMD_CONFIG, NET_CMD_TIME_MAGIC0, NET_CMD_TIME_MAGIC1,
        NET_CMD_TIME_MAGIC2, timestamp)


def make_reset_cmd() -> bytes:
    """构造软复位命令: [0x00, 0xA2, 0x00, 0x00]"""
    return bytes([NET_CMD_CONFIG, NET_CMD_RESET_MAGIC0,
                  NET_CMD_RESET_MAGIC1, NET_CMD_RESET_MAGIC2])


def make_heartbeat_ack() -> bytes:
    """构造心跳应答: [0x00, 0xA5, 0x00, 0x00]"""
    return bytes([NET_CMD_CONFIG, NET_CMD_HEARTBEAT_M0,
                  NET_CMD_HEARTBEAT_M1, NET_CMD_HEARTBEAT_M2])


def parse_incoming_frame(data: bytes) -> dict:
    """
    解析板卡发来的 TCP 帧。

    返回 dict，包含:
      - type: 'config_request' | 'log' | 'serial_data' | 'unknown'
      - 各类具体字段
    """
    if len(data) < 7:
        return {"type": "too_short", "len": len(data)}

    port       = data[0]
    main_type  = data[1]
    sub_type   = data[2]
    byte3      = data[3]
    byte4      = data[4]
    data_len   = data[5] | (data[6] << 8)  # little-endian

    result = {
        "raw": data,
        "raw_len": len(data),
        "port": port,
        "main_type": main_type,
        "sub_type": sub_type,
        "byte3": byte3,
        "byte4": byte4,
        "data_len": data_len,
    }

    # ---- 申请配置帧 ----
    if port == 0 and main_type == FRAME_TYPE_CONFIG_REQ and len(data) >= 11:
        result["type"] = "config_request"
        return result

    # ---- 日志帧 ----
    if main_type == FRAME_TYPE_LOG and len(data) >= 9:
        log_id  = data[7]
        extra   = data[8]
        result["type"]     = "log"
        result["log_id"]   = log_id
        result["log_text"] = LOG_MESSAGES.get(log_id, f"未知日志(0x{log_id:02X})")
        result["extra"]    = extra
        return result

    # ---- 串口数据帧 ----
    if len(data) >= 11:
        frame_flag   = byte3
        fenji_num    = byte4
        link_mask    = data[7] | (data[8] << 8) | (data[9] << 16) | (data[10] << 24)
        payload      = data[11:] if len(data) > 11 else b""

        result["type"]        = "serial_data"
        result["frame_flag"]  = frame_flag
        result["fenji_num"]   = fenji_num
        result["link_mask"]   = link_mask
        result["payload_len"] = len(payload)
        result["payload"]     = payload
        return result

    result["type"] = "unknown"
    return result


# ============================================================================
# TCP 服务器 (后台线程)
# ============================================================================

class TcpBoardServer(threading.Thread):
    """TCP 服务器线程：监听连接、收发数据。"""

    def __init__(self, host: str = "0.0.0.0", port: int = SERVER_PORT):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.server_socket: socket.socket | None = None
        self.client_socket: socket.socket | None = None
        self.client_addr: tuple | None = None
        self.running = False
        self.connected = False
        self.listening = False  # 是否允许接受连接 (手动控制)

        # 线程安全的消息队列 → GUI
        self.event_queue = queue.Queue()

        # 发送锁
        self.send_lock = threading.Lock()

        # 粘包缓冲区 (处理半帧 / 非标准帧)
        self._recv_buf = bytearray()
        self._recv_buf_ts = 0.0   # 上次收到数据的时间
        self._last_buf_flush = 0.0
        self._RECV_BUF_TIMEOUT = 3.0  # 缓冲区超时 (秒), 超时后丢弃残留

    def run(self):
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.settimeout(1.0)

        try:
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(LISTEN_BACKLOG)
            self._post_event("server_started", f"服务器已启动，监听 {self.host}:{self.port}")
        except OSError as e:
            self._post_event("error", f"无法绑定端口 {self.port}: {e}")
            self.running = False
            return

        while self.running:
            # ---- 等待客户端连接 (仅在用户手动开启监听后才接受) ----
            if not self.connected:
                if not self.listening:
                    time.sleep(0.1)  # 未开启监听时休眠, 避免空转
                    continue
                try:
                    self.client_socket, self.client_addr = self.server_socket.accept()
                    self.client_socket.settimeout(0.5)
                    self.connected = True
                    self._post_event("connected",
                        f"通信板已连接: {self.client_addr[0]}:{self.client_addr[1]}")
                except socket.timeout:
                    continue
                except OSError:
                    if self.running:
                        continue
                    break

            # ---- 接收数据 ----
            if self.connected:
                try:
                    raw = self.client_socket.recv(RECV_BUF_SIZE)
                    if raw:
                        self._handle_raw_data(raw)
                    else:
                        # 对方关闭连接
                        self._disconnect("通信板断开连接 (对端关闭)")
                except socket.timeout:
                    continue
                except (ConnectionResetError, ConnectionAbortedError, OSError) as e:
                    self._disconnect(f"通信板连接异常断开: {e}")

        self._cleanup()

    def _handle_raw_data(self, raw: bytes):
        """处理收到的原始 TCP 数据。使用内部缓冲区处理粘包/半帧/非标准帧。"""
        self._recv_buf_ts = time.time()

        # 检查缓冲区是否超时 (长时间无新数据, 丢弃残留以防解析不同步)
        if self._recv_buf and (self._recv_buf_ts - getattr(self, '_last_buf_flush', 0) > self._RECV_BUF_TIMEOUT):
            self._post_event("warning", f"接收缓冲区超时, 丢弃 {len(self._recv_buf)} 字节残留数据")
            self._recv_buf.clear()

        self._recv_buf.extend(raw)
        self._last_buf_flush = self._recv_buf_ts

        offset = 0
        while offset + 7 <= len(self._recv_buf):
            remaining = len(self._recv_buf) - offset

            data_len_field = self._recv_buf[offset + 5] | (self._recv_buf[offset + 6] << 8)

            # 合理性检查: data_len 不应超过合理范围 (最大约 RECV_BUF_SIZE)
            if data_len_field > RECV_BUF_SIZE:
                # 非预期数据 (例如 debug testBuf), 跳过 1 字节后重试
                offset += 1
                continue

            total_frame_len = 7 + data_len_field

            if total_frame_len > remaining:
                # 帧不完整, 保留残余等待下次 recv
                break

            frame_data = bytes(self._recv_buf[offset : offset + total_frame_len])
            parsed = parse_incoming_frame(frame_data)
            self._dispatch_frame(parsed)
            offset += total_frame_len

        # 丢弃已处理的数据
        if offset > 0:
            del self._recv_buf[:offset]

        # 防止缓冲区无限增长 (超过 8KB 强制清理)
        if len(self._recv_buf) > 8192:
            self._post_event("warning", f"接收缓冲区溢出, 强制丢弃 {len(self._recv_buf)} 字节")
            self._recv_buf.clear()

    def _dispatch_frame(self, parsed: dict):
        """根据帧类型分发处理。"""
        frame_type = parsed.get("type", "unknown")

        if frame_type == "config_request":
            self._post_event("config_request",
                "通信板请求配置下发 (Config Request)")

        elif frame_type == "log":
            log_id   = parsed["log_id"]
            log_text = parsed["log_text"]
            extra    = parsed["extra"]
            port     = parsed["port"]

            msg = f"[日志] 端口{port} | {log_text}"
            if log_id == 3 and extra in WTD_RESET_CODES:
                msg += f" (复位码={extra}: {WTD_RESET_CODES[extra]})"
            elif log_id == 3:
                msg += f" (复位码={extra})"

            self._post_event("log", msg)

        elif frame_type == "serial_data":
            flag_str = "实时" if parsed["frame_flag"] == 0x00 else (
                "曲线数据" if parsed["frame_flag"] == 0x01 else f"0x{parsed['frame_flag']:02X}")

            # 发送完整帧数据 (限速移到 GUI 侧, 监视器需要全量数据)
            self._post_event("serial_data", {
                "summary": (
                    f"[数据] 端口{parsed['port']} | {flag_str} | "
                    f"分机={parsed['fenji_num']} | 负载={parsed['payload_len']}B | "
                    f"在线掩码=0x{parsed['link_mask']:08X}"
                ),
                "port":        parsed["port"],
                "main_type":   parsed["main_type"],
                "sub_type":    parsed["sub_type"],
                "frame_flag":  parsed["frame_flag"],
                "fenji_num":   parsed["fenji_num"],
                "data_len":    parsed["data_len"],
                "link_mask":   parsed["link_mask"],
                "payload_len": parsed["payload_len"],
                "raw":         parsed.get("raw", b""),
                "payload":     parsed.get("payload", b""),
            })

        elif frame_type == "unknown":
            self._post_event("unknown_frame",
                f"[未知帧] 端口={parsed['port']} 主类型=0x{parsed['main_type']:02X} "
                f"子类型=0x{parsed['sub_type']:02X} 长度={parsed['raw_len']}")

        elif frame_type == "too_short":
            pass  # 忽略碎片

    def _disconnect(self, reason: str):
        """断开当前 TCP 连接。"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except OSError:
                pass
        self.client_socket = None
        self.client_addr = None
        self.connected = False
        self._recv_buf.clear()
        self._post_event("disconnected", reason)

    def disconnect(self):
        """公开方法: 主动断开连接并停止监听, 下次需手动点击连接按钮。"""
        self.listening = False
        if self.connected:
            self._disconnect("上位机主动断开连接")
        else:
            self._post_event("listening_stopped", "监听已停止")

    def start_accepting(self):
        """公开方法: 开启监听, 允许通信板连接。"""
        if not self.listening:
            self.listening = True
            self._post_event("listening_started", "等待通信板连接...")

    def _post_event(self, event_type: str, message: str):
        """将事件放入 GUI 消息队列。"""
        self.event_queue.put({
            "type": event_type,
            "message": message,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })

    def _cleanup(self):
        """关闭所有 socket。"""
        if self.client_socket:
            try:
                self.client_socket.close()
            except OSError:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except OSError:
                pass
        self.client_socket = None
        self.server_socket = None
        self.connected = False

    def stop(self):
        """停止服务器。"""
        self.running = False
        self._cleanup()

    def send(self, data: bytes, description: str = "") -> bool:
        """发送数据到已连接的通信板。线程安全。"""
        if not self.connected or self.client_socket is None:
            self._post_event("error", f"发送失败 ({description}): 通信板未连接")
            return False

        with self.send_lock:
            try:
                self.client_socket.sendall(data)
                self._post_event("send", f"已发送: {description} ({len(data)} 字节)")
                return True
            except OSError as e:
                self._post_event("error", f"发送失败 ({description}): {e}")
                self._disconnect(f"发送失败断开: {e}")
                return False


# ============================================================================
# GUI 主窗口
# ============================================================================

class BoardServerGUI:
    """上位机主窗口。"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("通信板 上位机服务器 v2.0")
        self.root.geometry("1100x700")
        self.root.minsize(900, 500)

        # ---- 业务对象 ----
        self.config = BoardConfig()
        self.server: TcpBoardServer | None = None
        self.auto_heartbeat_enabled = tk.BooleanVar(value=True)
        self.auto_config_enabled   = tk.BooleanVar(value=True)
        self.show_data_frames      = tk.BooleanVar(value=False)
        self.serial_monitor = None          # 串口监视器窗口引用

        # GUI 侧数据帧限速
        self._last_data_log: dict[int, float] = {}

        # 心跳定时器
        self.heartbeat_job_id: str | None = None

        # 加载配置
        config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
        if not self.config.load(config_path):
            print(f"[INFO] 未找到配置文件，使用默认配置。")

        self._build_ui()

        # 启动 TCP 服务器
        self._start_server()

    # ================================================================
    #  UI 构建
    # ================================================================

    def _build_ui(self):
        """构建主界面。"""
        # ---- 顶部状态栏 ----
        self._build_status_bar()

        # ---- 主体: 左侧控制面板 + 右侧日志区 ----
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        left_panel = ttk.Frame(main_frame, width=350)
        left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))
        left_panel.pack_propagate(False)

        right_panel = ttk.Frame(main_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self._build_control_panel(left_panel)
        self._build_log_panel(right_panel)

        # ---- 底部状态栏 ----
        self._build_bottom_bar()

    def _build_status_bar(self):
        """顶部连接状态栏。"""
        frame = ttk.Frame(self.root, relief=tk.RIDGE, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(5, 0))

        self.status_indicator = tk.Canvas(frame, width=20, height=20,
                                           highlightthickness=0)
        self.status_indicator.pack(side=tk.LEFT, padx=10)
        self._draw_status_light("gray")

        self.status_label = ttk.Label(frame, text="服务器状态: 未启动",
                                       font=("Microsoft YaHei", 10, "bold"))
        self.status_label.pack(side=tk.LEFT, padx=5)

        self.client_info_label = ttk.Label(frame, text="",
                                            font=("Microsoft YaHei", 9))
        self.client_info_label.pack(side=tk.RIGHT, padx=10)

    def _build_control_panel(self, parent):
        """左侧控制面板。"""
        # ---- 配置下发区 ----
        cfg_frame = ttk.LabelFrame(parent, text="板卡配置", padding=5)
        cfg_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Checkbutton(cfg_frame, text="收到请求帧后自动回复配置",
                        variable=self.auto_config_enabled).pack(anchor=tk.W, pady=2)

        btn_row = ttk.Frame(cfg_frame)
        btn_row.pack(fill=tk.X, pady=5)
        ttk.Button(btn_row, text="下发配置", command=self._send_config).pack(
            side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="编辑配置...", command=self._open_config_editor).pack(
            side=tk.LEFT)

        # ---- 命令区 ----
        cmd_frame = ttk.LabelFrame(parent, text="控制命令", padding=5)
        cmd_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Button(cmd_frame, text="发送校时 (Time Sync)",
                   command=self._send_time_sync).pack(fill=tk.X, pady=2)

        ttk.Button(cmd_frame, text="发送心跳应答 (Heartbeat ACK)",
                   command=self._send_heartbeat).pack(fill=tk.X, pady=2)

        ttk.Checkbutton(cmd_frame, text="自动心跳 (每 5 秒)",
                        variable=self.auto_heartbeat_enabled,
                        command=self._toggle_auto_heartbeat).pack(anchor=tk.W, pady=2)

        ttk.Separator(cmd_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=5)

        ttk.Button(cmd_frame, text="发送软复位 (Soft Reset)",
                   command=self._send_reset).pack(fill=tk.X, pady=2)

        # ---- 显示选项 ----
        opt_frame = ttk.LabelFrame(parent, text="显示选项", padding=5)
        opt_frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Checkbutton(opt_frame, text="显示串口数据帧 (限速: 1条/秒/端口)",
                        variable=self.show_data_frames).pack(anchor=tk.W, pady=2)

        ttk.Button(opt_frame, text="打开串口监视器",
                   command=self._toggle_serial_monitor).pack(fill=tk.X, pady=2)

        ttk.Button(opt_frame, text="清空日志区",
                   command=self._clear_log).pack(fill=tk.X, pady=5)

        # ---- 服务器控制 ----
        svr_frame = ttk.LabelFrame(parent, text="服务器", padding=5)
        svr_frame.pack(fill=tk.X, pady=(0, 10))

        self.connect_btn = ttk.Button(svr_frame, text="连接",
                                       command=self._toggle_connection)
        self.connect_btn.pack(fill=tk.X, pady=2)

        self.restart_btn = ttk.Button(svr_frame, text="重启服务器",
                                       command=self._restart_server)
        self.restart_btn.pack(fill=tk.X, pady=2)

        # ---- 信息 ----
        info_frame = ttk.LabelFrame(parent, text="关于", padding=5)
        info_frame.pack(fill=tk.X)
        info_text = ("通信板 上位机服务器 v2.0\n"
                     f"默认端口: {SERVER_PORT}\n"
                     "协议: 6路串口通信板 (Modbus/Custom)\n"
                     "适配: STM32F429 + lwIP + FreeRTOS")
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT,
                  font=("Microsoft YaHei", 8), foreground="gray").pack()

    def _build_log_panel(self, parent):
        """右侧日志显示区。"""
        frame = ttk.LabelFrame(parent, text="通信日志", padding=3)
        frame.pack(fill=tk.BOTH, expand=True)

        # 日志颜色标签
        self.log_text = tk.Text(frame, wrap=tk.WORD, state=tk.DISABLED,
                                 font=("Consolas", 9),
                                 bg="#1E1E1E", fg="#D4D4D4",
                                 insertbackground="white")
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL,
                                   command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)

        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 配置颜色标签
        self.log_text.tag_configure("info",    foreground="#569CD6")  # 蓝
        self.log_text.tag_configure("success", foreground="#6A9955")  # 绿
        self.log_text.tag_configure("warning", foreground="#CE9178")  # 橙
        self.log_text.tag_configure("error",   foreground="#F44747")  # 红
        self.log_text.tag_configure("send",    foreground="#C586C0")  # 紫
        self.log_text.tag_configure("log",     foreground="#4EC9B0")  # 青
        self.log_text.tag_configure("time",    foreground="#808080")  # 灰
        self.log_text.tag_configure("data",    foreground="#DCDCAA")  # 黄

    def _build_bottom_bar(self):
        """底部状态栏。"""
        frame = ttk.Frame(self.root, relief=tk.SUNKEN, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.bottom_label = ttk.Label(frame, text="就绪",
                                       font=("Microsoft YaHei", 8))
        self.bottom_label.pack(side=tk.LEFT, padx=5)

        self.msg_count_label = ttk.Label(frame, text="消息: 0",
                                          font=("Microsoft YaHei", 8))
        self.msg_count_label.pack(side=tk.RIGHT, padx=5)

    # ================================================================
    #  日志输出辅助方法
    # ================================================================

    def _log(self, tag: str, message: str):
        """向日志区追加一条带时间戳和颜色标记的消息。"""
        now = datetime.now().strftime("%H:%M:%S")

        self.log_text.configure(state=tk.NORMAL)

        # 时间戳
        self.log_text.insert(tk.END, f"[{now}] ", "time")

        # 消息正文 (带颜色标签)
        self.log_text.insert(tk.END, message + "\n", tag)

        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _clear_log(self):
        """清空日志区。"""
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state=tk.DISABLED)

    # ================================================================
    #  服务器控制
    # ================================================================

    def _start_server(self):
        """启动 TCP 服务器线程。"""
        self.server = TcpBoardServer(port=SERVER_PORT)
        self.server.start()
        self._draw_status_light("gray")
        self.status_label.config(text="服务器状态: 未监听")
        self.bottom_label.config(text="请点击「连接」按钮开始监听...")
        self.connect_btn.config(text="连接", state=tk.DISABLED)
        # 开始轮询事件队列
        self._poll_events()

    def _restart_server(self):
        """重启服务器。"""
        if self.server:
            self.server.stop()
        self._draw_status_light("gray")
        self.status_label.config(text="服务器状态: 重启中...")
        self._log("warning", "服务器正在重启...")
        # 短暂延迟后重新启动
        self.root.after(500, self._start_server)

    # ================================================================
    #  事件轮询 (GUI 线程安全地消费后台线程的事件)
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
                self._process_event(event)
                msg_count += 1
        except queue.Empty:
            pass

        if msg_count > 0:
            # 更新消息计数
            current = int(self.msg_count_label.cget("text").split(": ")[-1])
            self.msg_count_label.config(text=f"消息: {current + msg_count}")

        self.root.after(100, self._poll_events)

    def _process_event(self, event: dict):
        """处理单个事件并更新 GUI。"""
        etype = event["type"]
        msg   = event["message"]

        if etype == "server_started":
            self._draw_status_light("gray")
            self.status_label.config(text="服务器状态: 未监听")
            self.bottom_label.config(text=f"端口 {SERVER_PORT} 已就绪, 请点击「连接」")
            self.connect_btn.config(text="连接", state=tk.NORMAL)
            self._log("info", msg)

        elif etype == "listening_started":
            self._draw_status_light("orange")
            self.status_label.config(text="服务器状态: 等待连接...")
            self.bottom_label.config(text="正在监听，等待通信板连接...")
            self.connect_btn.config(text="断开连接", state=tk.NORMAL)
            self._log("info", msg)

        elif etype == "listening_stopped":
            self._draw_status_light("gray")
            self.status_label.config(text="服务器状态: 未监听")
            self.bottom_label.config(text="监听已停止, 请点击「连接」开始")
            self.connect_btn.config(text="连接", state=tk.NORMAL)
            self._log("info", msg)

        elif etype == "connected":
            self._draw_status_light("green")
            self.status_label.config(text="服务器状态: 已连接")
            self.client_info_label.config(
                text=f"客户端: {self.server.client_addr[0]}:{self.server.client_addr[1]}")
            self.bottom_label.config(text="通信板已连接")
            self.connect_btn.config(text="断开连接", state=tk.NORMAL)
            self._log("success", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self._log("success", msg)
            # 自动心跳
            if self.auto_heartbeat_enabled.get():
                self._start_heartbeat_timer()

        elif etype == "disconnected":
            self._stop_heartbeat_timer()
            if self.server and self.server.listening:
                # 通信板自行断开 (非用户主动断开), 继续保持监听
                self._draw_status_light("orange")
                self.status_label.config(text="服务器状态: 等待连接...")
                self.client_info_label.config(text="")
                self.bottom_label.config(text="通信板已断开，继续监听等待重连...")
                self.connect_btn.config(text="断开连接", state=tk.NORMAL)
            else:
                # 用户主动断开, 已停止监听
                self._draw_status_light("gray")
                self.status_label.config(text="服务器状态: 未监听")
                self.client_info_label.config(text="")
                self.bottom_label.config(text="监听已停止, 请点击「连接」开始")
                self.connect_btn.config(text="连接", state=tk.NORMAL)
            self._log("warning", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self._log("warning", msg)

        elif etype == "config_request":
            self._log("info", msg)
            # 收到通信板请求帧后自动回复配置
            if self.auto_config_enabled.get():
                self._send_config()

        elif etype == "log":
            self._log("log", msg)

        elif etype == "serial_data":
            if isinstance(msg, dict):
                # 新格式: 完整帧数据 dict
                summary = msg.get("summary", "")
                port = msg.get("port", 0)
                now = time.time()
                # GUI 侧限速: 同一端口每秒最多显示一次摘要
                last = self._last_data_log.get(port, 0)
                if self.show_data_frames.get() and (now - last >= 1.0):
                    self._last_data_log[port] = now
                    self._log("data", summary)
                # 转发给串口监视器 (不限速, 全量)
                if self.serial_monitor is not None:
                    self.serial_monitor.handle_frame(msg)
            else:
                # 旧格式兼容: 纯字符串
                if self.show_data_frames.get():
                    self._log("data", msg)

        elif etype == "send":
            self._log("send", msg)

        elif etype == "unknown_frame":
            self._log("warning", msg)

        elif etype == "error":
            self._log("error", msg)
            self.bottom_label.config(text=f"错误: {msg}")

    # ================================================================
    #  命令发送
    # ================================================================

    def _toggle_connection(self):
        """切换连接/断开状态。"""
        if self.server is None:
            return
        if self.server.listening:
            # 当前正在监听或已连接 → 断开并停止监听
            self.server.disconnect()
            self._log("warning", "已断开连接，停止监听")
        else:
            # 当前未监听 → 开始监听
            self.server.start_accepting()
            self._log("info", "已开启监听，等待通信板连接...")

    def _send_config(self):
        """下发配置到通信板。"""
        if self.server is None or not self.server.connected:
            self._log("warning", "无法下发配置: 通信板未连接")
            return

        config_bytes = self.config.to_bytes()
        cmd = make_config_download_cmd(config_bytes)
        total = len(cmd)

        self._log("info", f"正在下发板卡配置 ({total} 字节, 含 {UART_PORT_COUNT} 路端口)...")
        for i, port in enumerate(self.config.ports):
            slave_count = port["SlaveModuelNum"]
            if slave_count > 0:
                slaves = ", ".join(
                    f"0x{port['SlaveModuelType'][j]:02X}"
                    for j in range(min(slave_count, 8))
                )
                self._log("info",
                    f"  端口{i+1}: {PORT_TYPE_MAP.get(port['PortType'], '?')} "
                    f"| 波特率={BAUD_RATE_MAP.get(port['BautRate'], '?')} "
                    f"| 从机数={slave_count} | 类型=[{slaves}]")

        self.server.send(cmd, f"配置下发 ({total}B)")

    def _send_time_sync(self):
        """发送校时命令。"""
        cmd = make_time_sync_cmd()
        now_ts = struct.unpack("<I", cmd[4:8])[0]
        now_str = datetime.fromtimestamp(now_ts).strftime("%Y-%m-%d %H:%M:%S")
        self.server.send(cmd, f"校时命令 → {now_str} (ts={now_ts})")

    def _send_heartbeat(self):
        """发送心跳应答。"""
        cmd = make_heartbeat_ack()
        self.server.send(cmd, "心跳应答")

    def _send_reset(self):
        """发送软复位命令 (需要二次确认)。"""
        if not messagebox.askyesno("确认操作",
            "确定要发送软复位命令?\n\n"
            "这将触发通信板的看门狗复位 (WTD_RESET=2)，\n"
            "复位后通信板将重新启动并重连。",
            icon="warning"):
            return
        cmd = make_reset_cmd()
        self._log("warning", "正在发送软复位命令...")
        self.server.send(cmd, "软复位命令 (WTD_RESET=2)")

    # ---- 串口监视器 ----

    def _toggle_serial_monitor(self):
        """打开或关闭串口数据监视器窗口。"""
        if self.serial_monitor is not None:
            self.serial_monitor.window.destroy()
            self.serial_monitor = None
            self._log("info", "串口监视器已关闭")
        else:
            self.serial_monitor = SerialDataMonitor(self.root, self.config)
            self._log("info", "串口监视器已打开")

    # ================================================================
    #  自动心跳
    # ================================================================

    def _toggle_auto_heartbeat(self):
        """开关自动心跳。"""
        if self.auto_heartbeat_enabled.get():
            self._start_heartbeat_timer()
        else:
            self._stop_heartbeat_timer()

    def _start_heartbeat_timer(self):
        """启动自动心跳定时器。"""
        if self.heartbeat_job_id is not None:
            return
        self._schedule_heartbeat()

    def _schedule_heartbeat(self):
        """单次心跳调度。"""
        if not self.auto_heartbeat_enabled.get():
            self.heartbeat_job_id = None
            return
        if self.server and self.server.connected:
            self._send_heartbeat()
        self.heartbeat_job_id = self.root.after(5000, self._schedule_heartbeat)

    def _stop_heartbeat_timer(self):
        """停止自动心跳定时器。"""
        if self.heartbeat_job_id is not None:
            self.root.after_cancel(self.heartbeat_job_id)
            self.heartbeat_job_id = None

    # ================================================================
    #  状态灯
    # ================================================================

    def _draw_status_light(self, color: str):
        """绘制圆形状态指示灯。"""
        self.status_indicator.delete("all")
        self.status_indicator.create_oval(3, 3, 17, 17, fill=color, outline="")

    # ================================================================
    #  配置编辑器 (弹出窗口)
    # ================================================================

    def _open_config_editor(self):
        """打开板卡配置编辑窗口。"""
        editor = ConfigEditorWindow(self.root, self.config)
        self.root.wait_window(editor.window)
        self._log("info", "板卡配置已更新")

    # ================================================================
    #  生命周期
    # ================================================================

    def run(self):
        """运行 GUI 主循环。"""
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        """关闭窗口时的清理。"""
        self._stop_heartbeat_timer()
        if self.serial_monitor is not None:
            self.serial_monitor.window.destroy()
            self.serial_monitor = None
        if self.server:
            self.server.stop()
        # 保存配置
        config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
        self.config.save(config_path)
        self.root.destroy()


# ============================================================================
# 配置编辑器窗口
# ============================================================================

class ConfigEditorWindow:
    """板卡配置编辑弹窗。使用 Notebook 分页显示 6 个端口。"""

    def __init__(self, parent, config: BoardConfig):
        self.config = config
        self.window = tk.Toplevel(parent)
        self.window.title("板卡配置编辑器")
        self.window.geometry("650x500")
        self.window.resizable(True, True)
        self.window.transient(parent)
        self.window.grab_set()

        self._build()

    def _build(self):
        """构建编辑器 UI。"""
        notebook = ttk.Notebook(self.window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 所有端口共享的变量字典 (在构建前初始化, 各端口页面追加条目)
        self.field_vars: dict[str, tk.Variable] = {}
        self.slave_vars: dict[str, tk.Variable] = {}

        self.port_frames = []
        for i in range(UART_PORT_COUNT):
            port = self.config.ports[i]
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=f"端口 {i+1}")
            self._build_port_page(frame, port, i)
            self.port_frames.append(frame)

        # ---- 底部按钮 ----
        btn_frame = ttk.Frame(self.window)
        btn_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(btn_frame, text="保存 (Save)", command=self._save).pack(
            side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="取消 (Cancel)", command=self.window.destroy).pack(
            side=tk.RIGHT, padx=5)
        ttk.Button(btn_frame, text="加载默认配置", command=self._load_defaults).pack(
            side=tk.LEFT, padx=5)

    def _build_port_page(self, parent, port: dict, port_idx: int):
        """构建单个端口的配置页面。"""
        row_frame = ttk.Frame(parent)
        row_frame.pack(fill=tk.X, padx=10, pady=10)

        # ---- 左列 ----
        left = ttk.Frame(row_frame)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 20))

        fields_left = [
            ("端口号", "PortNum", 1, 6),
            ("端口类型", "PortType", [("禁用", 0), ("自定义曲线", 1), ("Modbus从机", 3), ("Modbus主站", 4)]),
            ("波特率", "BautRate", [("9600", 1), ("19200", 2), ("38400", 3), ("57600", 4)]),
            ("数据位", "DataBit", [("5", 5), ("6", 6), ("7", 7), ("8", 8)]),
            ("停止位", "StopBit", [("1", 1), ("1.5", 15), ("2", 2)]),
            ("校验位", "Parity", [("无", 0), ("奇校验", 1), ("偶校验", 2)]),
        ]

        for label, key, *rest in fields_left:
            row = ttk.Frame(left)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label + ":", width=10, anchor=tk.E).pack(side=tk.LEFT, padx=(0, 5))

            if isinstance(rest[0], list):
                choices = rest[0]
                var = tk.StringVar()
                current_val = port[key]
                # 找当前值对应的显示文本
                found = False
                for text, val in choices:
                    if val == current_val:
                        var.set(text)
                        found = True
                        break
                if not found:
                    var.set(str(current_val))

                combo = ttk.Combobox(row, textvariable=var,
                                     values=[t for t, _ in choices],
                                     state="readonly", width=15)
                combo.pack(side=tk.LEFT)
                self.field_vars[f"{port_idx}_{key}"] = var
                # 保存 choices 以便 save 时使用
                self.field_vars[f"{port_idx}_{key}_choices"] = choices  # type: ignore
            else:
                min_v, max_v = rest
                var = tk.IntVar(value=port[key])
                spin = ttk.Spinbox(row, textvariable=var, from_=min_v, to=max_v,
                                   width=15)
                spin.pack(side=tk.LEFT)
                self.field_vars[f"{port_idx}_{key}"] = var

        # ---- 右列 ----
        right = ttk.Frame(row_frame)
        right.pack(side=tk.LEFT, fill=tk.Y)

        fields_right = [
            ("主协议类型", "MainProType", 0, 255),
            ("子协议类型", "SubProType", 0, 255),
            ("分机号", "FenjiNum", 0, 255),
            ("查询周期(ms)", "QueryPeriod", 10, 5000),
            ("从机数量", "SlaveModuelNum", 0, UART_MODULE_MAX_COUNT),
        ]

        for label, key, min_v, max_v in fields_right:
            row = ttk.Frame(right)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label + ":", width=14, anchor=tk.E).pack(
                side=tk.LEFT, padx=(0, 5))
            var = tk.IntVar(value=port[key])
            spin = ttk.Spinbox(row, textvariable=var, from_=min_v, to=max_v,
                               width=15)
            spin.pack(side=tk.LEFT)
            self.field_vars[f"{port_idx}_{key}"] = var

        # ---- 从机地址与类型 ----
        slave_frame = ttk.LabelFrame(parent, text="从机模块配置")
        slave_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        # 表头
        header = ttk.Frame(slave_frame)
        header.pack(fill=tk.X, padx=5, pady=(5, 0))
        ttk.Label(header, text="序号", width=6, anchor=tk.CENTER).pack(side=tk.LEFT)
        ttk.Label(header, text="地址", width=8, anchor=tk.CENTER).pack(side=tk.LEFT, padx=2)
        ttk.Label(header, text="模块类型", width=16, anchor=tk.W).pack(side=tk.LEFT, padx=2)
        ttk.Label(header, text="序号", width=6, anchor=tk.CENTER).pack(side=tk.LEFT)
        ttk.Label(header, text="地址", width=8, anchor=tk.CENTER).pack(side=tk.LEFT, padx=2)
        ttk.Label(header, text="模块类型", width=16, anchor=tk.W).pack(side=tk.LEFT, padx=2)

        # 可滚动的从机列表 (32 个, 分 2 列)
        canvas = tk.Canvas(slave_frame, height=200)
        scrollbar = ttk.Scrollbar(slave_frame, orient=tk.VERTICAL, command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        for i in range(UART_MODULE_MAX_COUNT):
            row = ttk.Frame(scroll_frame)
            row.pack(fill=tk.X)

            # 左半 (索引 0-15)
            ttk.Label(row, text=str(i+1), width=6, anchor=tk.CENTER).pack(side=tk.LEFT)
            addr_var = tk.IntVar(value=port["SlaveModuelAddress"][i])
            addr_spin = ttk.Spinbox(row, textvariable=addr_var, from_=0, to=255,
                                    width=6)
            addr_spin.pack(side=tk.LEFT, padx=2)
            self.slave_vars[f"{port_idx}_addr_{i}"] = addr_var

            type_var = tk.StringVar()
            type_val = port["SlaveModuelType"][i]
            type_str = f"0x{type_val:02X}" if type_val != 0 else "未配置"
            type_var.set(type_str)
            type_combo = ttk.Combobox(
                row, textvariable=type_var,
                values=["未配置"] + [f"0x{k:02X} - {v}" for k, v in MODULE_TYPE_MAP.items() if k != 0],
                width=20, state="readonly")
            type_combo.pack(side=tk.LEFT, padx=2)
            self.slave_vars[f"{port_idx}_type_{i}"] = type_var

        # 鼠标滚轮支持
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

    def _save(self):
        """保存配置到 BoardConfig 对象。"""
        for port_idx in range(UART_PORT_COUNT):
            port = self.config.ports[port_idx]

            # 基本字段
            for key in ["PortNum", "MainProType", "SubProType", "FenjiNum",
                        "QueryPeriod", "SlaveModuelNum"]:
                var = self.field_vars.get(f"{port_idx}_{key}")
                if var is not None:
                    port[key] = int(var.get())

            # 下拉选择字段
            for key in ["PortType", "BautRate", "DataBit", "StopBit", "Parity"]:
                var = self.field_vars.get(f"{port_idx}_{key}")
                choices = self.field_vars.get(f"{port_idx}_{key}_choices")
                if var is not None and choices is not None:
                    display_text = var.get()
                    for text, val in choices:
                        if text == display_text:
                            port[key] = val
                            break

            # 从机地址
            for i in range(UART_MODULE_MAX_COUNT):
                addr_var = self.slave_vars.get(f"{port_idx}_addr_{i}")
                if addr_var:
                    port["SlaveModuelAddress"][i] = int(addr_var.get())

                type_var = self.slave_vars.get(f"{port_idx}_type_{i}")
                if type_var:
                    text = type_var.get()
                    if text == "未配置" or text == "":
                        port["SlaveModuelType"][i] = 0
                    else:
                        # 解析 "0x37 - 交流转辙机"
                        try:
                            hex_str = text.split(" ")[0].replace("0x", "")
                            port["SlaveModuelType"][i] = int(hex_str, 16)
                        except (ValueError, IndexError):
                            port["SlaveModuelType"][i] = 0

        # 保存到文件
        config_path = os.path.join(os.path.dirname(__file__), CONFIG_FILE)
        self.config.save(config_path)

        self.window.destroy()

    def _load_defaults(self):
        """加载默认配置。"""
        for port_idx in range(UART_PORT_COUNT):
            port = self.config.ports[port_idx]

            # 端口2 特殊默认值: Modbus主站 + 信号机开关量从机
            if port_idx == 1:
                defaults = {
                    "PortNum":       2,
                    "PortType":      4,     # Modbus主站
                    "BautRate":      2,     # 19200
                    "DataBit":       8,
                    "StopBit":       1,
                    "Parity":        0,
                    "MainProType":   1,
                    "SubProType":    2,
                    "FenjiNum":      10,
                    "QueryPeriod":   100,
                    "SlaveModuelNum": 1,
                }
            else:
                defaults = {
                    "PortNum":       port_idx + 1,
                    "PortType":      0,
                    "BautRate":      2,
                    "DataBit":       8,
                    "StopBit":       1,
                    "Parity":        0,
                    "MainProType":   0,
                    "SubProType":    0,
                    "FenjiNum":      port_idx + 1,
                    "QueryPeriod":   100,
                    "SlaveModuelNum": 0,
                }

            for key, val in defaults.items():
                port[key] = val
                var = self.field_vars.get(f"{port_idx}_{key}")
                if var is None:
                    continue
                # 下拉框字段: var 是 StringVar (显示文本), 需要从 choices 查找对应文本
                choices = self.field_vars.get(f"{port_idx}_{key}_choices")
                if choices is not None:
                    display_text = None
                    for text, v in choices:
                        if v == val:
                            display_text = text
                            break
                    if display_text is not None:
                        var.set(display_text)
                else:
                    # Spinbox: IntVar, 直接设整数值
                    var.set(val)

            # 从机地址和类型全部清空
            for i in range(UART_MODULE_MAX_COUNT):
                port["SlaveModuelAddress"][i] = 0
                port["SlaveModuelType"][i] = 0
                addr_var = self.slave_vars.get(f"{port_idx}_addr_{i}")
                if addr_var:
                    addr_var.set(0)
                type_var = self.slave_vars.get(f"{port_idx}_type_{i}")
                if type_var:
                    type_var.set("未配置")

            # 端口2 从机默认: 地址=1, 类型=0x31 (信号机开关量)
            if port_idx == 1:
                port["SlaveModuelAddress"][0] = 1
                port["SlaveModuelType"][0]    = 0x31
                addr_var = self.slave_vars.get(f"{port_idx}_addr_0")
                if addr_var:
                    addr_var.set(1)
                type_var = self.slave_vars.get(f"{port_idx}_type_0")
                if type_var:
                    type_var.set("0x31 - 信号机开关量")

        messagebox.showinfo("默认配置", "已加载默认配置:\n"
                            "  端口2 (USART2): Modbus主站, 19200/8N1,\n"
                            "                  从机1台 (0x31 信号机开关量)\n"
                            "  其余端口: 未配置")


# ============================================================================
# 串口数据监视器窗口
# ============================================================================

class SerialDataMonitor:
    """串口数据监视器 — 独立窗口, 显示原始帧/帧头字段/从机寄存器数据。

    可在 6 个串口之间随时切换, 不限速显示所有帧。
    """

    def __init__(self, parent, config: BoardConfig):
        self.parent = parent
        self.config = config
        self.paused = False
        self.current_port = 1

        # 每端口缓存最后一帧
        self.port_cache: dict[int, dict | None] = {i + 1: None for i in range(UART_PORT_COUNT)}
        self.total_frames = 0
        self.port_frame_counts = {i + 1: 0 for i in range(UART_PORT_COUNT)}

        self.window = tk.Toplevel(parent)
        self.window.title("串口数据监视器 - Serial Data Monitor")
        self.window.geometry("1080x800")
        self.window.minsize(850, 550)
        self.window.transient(parent)
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        # 存储从机区域的 widget 引用, 用于高效更新
        self._slave_frames: list[dict] = []
        self._slave_built = False

        self._build_ui()
        self._update_port_label()

    # ================================================================
    #  UI 构建
    # ================================================================

    def _build_ui(self):
        """构建监视器界面。"""
        # ---- 顶部工具栏 ----
        self._build_toolbar()

        # ---- 主体: 上 (原始数据+帧头) + 下 (从机数据) ----
        top_frame = ttk.Frame(self.window)
        top_frame.pack(fill=tk.X, padx=5, pady=(5, 0))

        # 左: 原始 hex
        hex_frame = ttk.LabelFrame(top_frame, text="原始帧数据 (Raw Hex)", padding=3)
        hex_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 3))

        self.raw_hex_text = tk.Text(hex_frame, wrap=tk.NONE, state=tk.DISABLED,
                                     font=("Consolas", 9), width=56, height=7,
                                     bg="#1E1E1E", fg="#D4D4D4")
        hex_scroll_x = ttk.Scrollbar(hex_frame, orient=tk.HORIZONTAL,
                                      command=self.raw_hex_text.xview)
        hex_scroll_y = ttk.Scrollbar(hex_frame, orient=tk.VERTICAL,
                                      command=self.raw_hex_text.yview)
        self.raw_hex_text.configure(xscrollcommand=hex_scroll_x.set,
                                     yscrollcommand=hex_scroll_y.set)
        self.raw_hex_text.grid(row=0, column=0, sticky="nsew")
        hex_scroll_y.grid(row=0, column=1, sticky="ns")
        hex_scroll_x.grid(row=1, column=0, sticky="ew")
        hex_frame.grid_rowconfigure(0, weight=1)
        hex_frame.grid_columnconfigure(0, weight=1)

        # 右: 帧头字段
        hdr_frame = ttk.LabelFrame(top_frame, text="帧头字段 (Header)", padding=5)
        hdr_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(3, 0))

        self.header_labels = {}
        fields = [
            ("端口号",     "port"),
            ("主类型",     "main_type"),
            ("子类型",     "sub_type"),
            ("帧标志",     "frame_flag"),
            ("分机号",     "fenji_num"),
            ("数据长度",   "data_len"),
            ("链路掩码",   "link_mask"),
            ("负载长度",   "payload_len"),
        ]
        for idx, (label, key) in enumerate(fields):
            row, col = divmod(idx, 2)
            ttk.Label(hdr_frame, text=f"{label}:", font=("Microsoft YaHei", 9, "bold"),
                      anchor=tk.E).grid(row=row, column=col * 2, sticky=tk.E,
                                        padx=(5 if col == 0 else 15, 2), pady=1)
            val_label = ttk.Label(hdr_frame, text="—",
                                   font=("Consolas", 9))
            val_label.grid(row=row, column=col * 2 + 1, sticky=tk.W, padx=(2, 5), pady=1)
            self.header_labels[key] = val_label

        # ---- 从机数据区 ----
        slave_frame = ttk.LabelFrame(self.window, text="从机模块数据 (Slave Registers)", padding=3)
        slave_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.slave_canvas = tk.Canvas(slave_frame, highlightthickness=0)
        slave_scrollbar = ttk.Scrollbar(slave_frame, orient=tk.VERTICAL,
                                         command=self.slave_canvas.yview)
        self.slave_inner = ttk.Frame(self.slave_canvas)

        self.slave_inner.bind("<Configure>",
            lambda e: self.slave_canvas.configure(
                scrollregion=self.slave_canvas.bbox("all")))
        self.slave_canvas_window = self.slave_canvas.create_window(
            (0, 0), window=self.slave_inner, anchor=tk.NW)
        self.slave_canvas.configure(yscrollcommand=slave_scrollbar.set)

        self.slave_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        slave_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 鼠标滚轮
        def _on_mousewheel(event):
            self.slave_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        self.slave_canvas.bind("<Enter>", lambda e: self.slave_canvas.bind_all("<MouseWheel>", _on_mousewheel))
        self.slave_canvas.bind("<Leave>", lambda e: self.slave_canvas.unbind_all("<MouseWheel>"))

        # 从机区域占位
        self.slave_placeholder = ttk.Label(self.slave_inner,
            text="等待数据...\n(请确保通信板已连接并下发配置)",
            font=("Microsoft YaHei", 10), foreground="gray", anchor=tk.CENTER)
        self.slave_placeholder.pack(expand=True, pady=50)

        # ---- 底部状态栏 ----
        self._build_status_bar()

    def _build_toolbar(self):
        """顶部工具栏。"""
        toolbar = ttk.Frame(self.window, relief=tk.RIDGE, borderwidth=1)
        toolbar.pack(fill=tk.X, padx=5, pady=(5, 0))

        # 端口切换
        self.prev_btn = ttk.Button(toolbar, text="◀", width=3,
                                    command=lambda: self._switch_port(-1))
        self.prev_btn.pack(side=tk.LEFT, padx=(10, 2))

        self.port_label = ttk.Label(toolbar, text="端口 1 (USART1)",
                                     font=("Microsoft YaHei", 10, "bold"),
                                     width=20, anchor=tk.CENTER)
        self.port_label.pack(side=tk.LEFT, padx=2)

        self.next_btn = ttk.Button(toolbar, text="▶", width=3,
                                    command=lambda: self._switch_port(1))
        self.next_btn.pack(side=tk.LEFT, padx=(2, 15))

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # 帧计数
        self.frame_count_label = ttk.Label(toolbar, text="帧计数: 0",
                                            font=("Consolas", 9))
        self.frame_count_label.pack(side=tk.LEFT, padx=10)

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=5)

        # 暂停/继续
        self.pause_btn = ttk.Button(toolbar, text="暂停", width=6,
                                     command=self._toggle_pause)
        self.pause_btn.pack(side=tk.LEFT, padx=10)

        # 清除
        ttk.Button(toolbar, text="清除", width=6,
                   command=self._clear).pack(side=tk.LEFT, padx=5)

    def _build_status_bar(self):
        """底部状态栏。"""
        frame = ttk.Frame(self.window, relief=tk.SUNKEN, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(0, 5))

        self.status_label = ttk.Label(frame, text="就绪",
                                       font=("Microsoft YaHei", 8))
        self.status_label.pack(side=tk.LEFT, padx=5)

    # ================================================================
    #  帧处理入口 (GUI 线程调用)
    # ================================================================

    def handle_frame(self, frame_dict: dict):
        """接收来自主窗口的完整帧数据。在 GUI 线程中执行。"""
        if self.paused:
            return

        port = frame_dict["port"]
        if port < 1 or port > UART_PORT_COUNT:
            return

        # 存入缓存
        self.port_cache[port] = frame_dict
        self.total_frames += 1
        self.port_frame_counts[port] = self.port_frame_counts.get(port, 0) + 1

        # 仅当匹配当前端口时刷新显示
        if port == self.current_port:
            self._update_display(frame_dict)

        self._update_status_bar()

    # ================================================================
    #  端口切换
    # ================================================================

    def _switch_port(self, delta: int):
        """切换端口 (循环 1-6)。"""
        self.current_port = ((self.current_port - 1 + delta) % UART_PORT_COUNT) + 1
        self._update_port_label()

        cached = self.port_cache.get(self.current_port)
        if cached is not None:
            self._update_display(cached)
        else:
            self._clear_display()
        self._update_status_bar()

    def _update_port_label(self):
        """更新端口标签。"""
        name = PORT_NAMES[self.current_port - 1]
        port_cfg = self.config.ports[self.current_port - 1]
        slave_count = port_cfg["SlaveModuelNum"]
        type_name = PORT_TYPE_MAP.get(port_cfg["PortType"], "未知")
        self.port_label.config(text=f"端口 {self.current_port} ({name})  "
                                    f"[{type_name}, {slave_count}从机]")

    # ================================================================
    #  显示更新
    # ================================================================

    def _update_display(self, frame_dict: dict):
        """刷新全部显示区域。"""
        # 1. 原始 hex
        self._update_raw_hex(frame_dict["raw"])

        # 2. 帧头字段
        self._update_header_fields(frame_dict)

        # 3. 从机数据
        slaves = self._parse_payload_per_slave(
            frame_dict["port"], frame_dict["payload"], frame_dict["link_mask"])
        self._update_slave_grid(slaves)

    def _update_raw_hex(self, data: bytes):
        """更新原始 hex dump。"""
        lines = []
        for i in range(0, len(data), 16):
            chunk = data[i:i + 16]
            hex_str = " ".join(f"{b:02X}" for b in chunk)
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            lines.append(f"{i:04X}  {hex_str:<48s}  {ascii_str}")

        self.raw_hex_text.configure(state=tk.NORMAL)
        self.raw_hex_text.delete("1.0", tk.END)
        self.raw_hex_text.insert("1.0", f"({len(data)} bytes total)\n" + "\n".join(lines))
        self.raw_hex_text.configure(state=tk.DISABLED)

    def _update_header_fields(self, frame_dict: dict):
        """更新帧头字段标签。"""
        link_mask = frame_dict["link_mask"]
        online_bits = [i for i in range(32) if link_mask & (1 << i)]

        values = {
            "port":        str(frame_dict["port"]),
            "main_type":   f"0x{frame_dict['main_type']:02X}",
            "sub_type":    f"0x{frame_dict['sub_type']:02X}",
            "frame_flag":  f"0x{frame_dict['frame_flag']:02X}"
                           + (" (实时)" if frame_dict["frame_flag"] == 0x00
                              else " (曲线)" if frame_dict["frame_flag"] == 0x01
                              else ""),
            "fenji_num":   str(frame_dict["fenji_num"]),
            "data_len":    f"{frame_dict['data_len']} (0x{frame_dict['data_len']:04X})",
            "link_mask":   f"0x{link_mask:08X} 在线: {online_bits if online_bits else '无'}",
            "payload_len": f"{frame_dict['payload_len']} bytes",
        }
        for key, label in self.header_labels.items():
            label.config(text=values.get(key, "—"))

    # ================================================================
    #  Payload 解析
    # ================================================================

    def _parse_payload_per_slave(self, port: int, payload: bytes,
                                  link_mask: int) -> list:
        """按配置拆分 payload 为每个从机的寄存器值列表。

        返回 list[dict], 每个 dict 含:
          idx, address, module_type, type_name, online, registers, reg_count
        """
        port_cfg = self.config.ports[port - 1]
        num_slaves = port_cfg["SlaveModuelNum"]
        offset = 0
        slaves_data = []

        for idx in range(min(num_slaves, UART_MODULE_MAX_COUNT)):
            mtype = port_cfg["SlaveModuelType"][idx]
            if mtype == 0:
                continue
            profile = get_module_profile(mtype)
            reg_count = profile["input_regs"]
            reg_bytes = reg_count * 2
            online = bool(link_mask & (1 << idx))

            if online and offset + reg_bytes <= len(payload):
                raw = payload[offset:offset + reg_bytes]
                registers = []
                for r in range(reg_count):
                    val = raw[r * 2] | (raw[r * 2 + 1] << 8)
                    registers.append(val)
                offset += reg_bytes
            else:
                registers = [0] * reg_count

            slaves_data.append({
                "idx":         idx,
                "address":     port_cfg["SlaveModuelAddress"][idx],
                "module_type": mtype,
                "type_name":   MODULE_TYPE_MAP.get(mtype, f"0x{mtype:02X}"),
                "online":      online,
                "registers":   registers,
                "reg_count":   reg_count,
            })

        return slaves_data

    # ================================================================
    #  从机数据网格更新
    # ================================================================

    def _update_slave_grid(self, slaves_data: list):
        """重建或更新从机数据区域。"""
        if not slaves_data:
            self.slave_placeholder.pack(expand=True, pady=50)
            self._slave_built = False
            return

        # 隐藏占位符
        self.slave_placeholder.pack_forget()

        if not self._slave_built:
            # 首次: 创建全部 widget
            self._build_slave_grid(slaves_data)
            self._slave_built = True
        else:
            # 后续: 仅更新 widget 文本
            self._refresh_slave_grid(slaves_data)

    def _build_slave_grid(self, slaves_data: list):
        """首次创建从机网格 widget。"""
        self._slave_frames.clear()

        for slave in slaves_data:
            # 从机整体容器
            sf = ttk.LabelFrame(self.slave_inner,
                text=f"从机 {slave['idx'] + 1}  |  "
                     f"地址: 0x{slave['address']:02X}  |  "
                     f"类型: {slave['type_name']}",
                padding=3)
            sf.pack(fill=tk.X, padx=5, pady=3)

            # 状态行
            status_row = ttk.Frame(sf)
            status_row.pack(fill=tk.X, padx=5, pady=(2, 5))

            status_canvas = tk.Canvas(status_row, width=12, height=12,
                                       highlightthickness=0)
            status_canvas.pack(side=tk.LEFT, padx=(0, 5))
            self._draw_dot(status_canvas, slave["online"])

            status_text = "在线" if slave["online"] else "离线"
            status_color = "#6A9955" if slave["online"] else "#F44747"
            status_label = ttk.Label(status_row, text=status_text,
                                      font=("Microsoft YaHei", 9, "bold"),
                                      foreground=status_color)
            status_label.pack(side=tk.LEFT)

            reg_label = ttk.Label(status_row,
                text=f"  |  寄存器数: {slave['reg_count']} (input_regs)",
                font=("Microsoft YaHei", 8), foreground="gray")
            reg_label.pack(side=tk.LEFT)

            # 寄存器表头
            reg_frame = ttk.Frame(sf)
            reg_frame.pack(fill=tk.X, padx=5)
            ttk.Label(reg_frame, text="寄存器#", width=8, anchor=tk.CENTER,
                      font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=1)
            ttk.Label(reg_frame, text="十六进制", width=10, anchor=tk.CENTER,
                      font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=1)
            ttk.Label(reg_frame, text="十进制", width=8, anchor=tk.CENTER,
                      font=("Consolas", 8, "bold")).pack(side=tk.LEFT, padx=1)

            # 寄存器值 (每行最多显示16个寄存器, 多了折叠)
            reg_grid = ttk.Frame(sf)
            reg_grid.pack(fill=tk.X, padx=5, pady=(2, 5))

            reg_labels = []
            max_show = min(slave["reg_count"], 16)  # 最多展示前16个
            for r in range(max_show):
                row = r // 4
                col = (r % 4) * 3
                rl = ttk.Label(reg_grid, text=f"R{r:02d}", width=6, anchor=tk.E,
                               font=("Consolas", 8), foreground="gray")
                rl.grid(row=row, column=col, sticky=tk.E, padx=1, pady=1)
                hl = ttk.Label(reg_grid, text="0x0000", width=8, anchor=tk.CENTER,
                               font=("Consolas", 8))
                hl.grid(row=row, column=col + 1, sticky=tk.W, padx=1, pady=1)
                dl = ttk.Label(reg_grid, text="0", width=8, anchor=tk.E,
                               font=("Consolas", 8))
                dl.grid(row=row, column=col + 2, sticky=tk.E, padx=1, pady=1)
                reg_labels.append((rl, hl, dl))

            if slave["reg_count"] > 16:
                more_label = ttk.Label(reg_grid,
                    text=f"... 还有 {slave['reg_count'] - 16} 个寄存器 (未展开)",
                    font=("Microsoft YaHei", 7), foreground="gray")
                more_label.grid(row=4, column=0, columnspan=12, sticky=tk.W, pady=(3, 0))

            self._slave_frames.append({
                "status_canvas":  status_canvas,
                "status_label":   status_label,
                "reg_labels":     reg_labels,
                "slave":          slave,
            })

    def _refresh_slave_grid(self, slaves_data: list):
        """仅更新已存在的从机 widget 文本。"""
        # 如果从机数量变化, 重建
        if len(slaves_data) != len(self._slave_frames):
            self._rebuild_slave_grid(slaves_data)
            return

        for i, (sf, slave) in enumerate(zip(self._slave_frames, slaves_data)):
            # 检查从机类型/在线状态是否变化
            old_slave = sf["slave"]
            if (old_slave["module_type"] != slave["module_type"] or
                old_slave["online"] != slave["online"] or
                old_slave["reg_count"] != slave["reg_count"]):
                self._rebuild_slave_grid(slaves_data)
                return

            # 更新状态灯
            self._draw_dot(sf["status_canvas"], slave["online"])
            color = "#6A9955" if slave["online"] else "#F44747"
            sf["status_label"].config(
                text="在线" if slave["online"] else "离线",
                foreground=color)

            # 更新寄存器值
            for r, (rl, hl, dl) in enumerate(sf["reg_labels"]):
                if r < len(slave["registers"]):
                    val = slave["registers"][r]
                    hl.config(text=f"0x{val:04X}")
                    dl.config(text=str(val))

            sf["slave"] = slave

    def _rebuild_slave_grid(self, slaves_data: list):
        """销毁并重建从机区域。"""
        for sf in self._slave_frames:
            for widget in sf.get("widgets", []):
                widget.destroy()
        self._slave_frames.clear()
        # 销毁 inner 中所有子 widget
        for child in self.slave_inner.winfo_children():
            child.destroy()
        self._slave_built = False
        self._build_slave_grid(slaves_data)
        self._slave_built = True

    def _draw_dot(self, canvas: tk.Canvas, online: bool):
        """在 canvas 上画状态圆点。"""
        canvas.delete("all")
        color = "#6A9955" if online else "#F44747"
        canvas.create_oval(2, 2, 10, 10, fill=color, outline="")

    def _clear_display(self):
        """清空显示。"""
        # 清空 hex
        self.raw_hex_text.configure(state=tk.NORMAL)
        self.raw_hex_text.delete("1.0", tk.END)
        self.raw_hex_text.configure(state=tk.DISABLED)

        # 清空帧头
        for label in self.header_labels.values():
            label.config(text="—")

        # 清空从机
        for child in self.slave_inner.winfo_children():
            child.destroy()
        self._slave_frames.clear()
        self._slave_built = False
        self.slave_placeholder.pack(expand=True, pady=50)

    def _update_status_bar(self):
        """更新底部状态栏。"""
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        port_stats = "  |  ".join(
            f"P{p}: {self.port_frame_counts.get(p, 0)}" for p in range(1, UART_PORT_COUNT + 1))
        self.status_label.config(
            text=f"总帧: {self.total_frames}  |  最后更新: {now}  |  {port_stats}")

    # ================================================================
    #  按钮动作
    # ================================================================

    def _toggle_pause(self):
        """暂停/继续切换。"""
        self.paused = not self.paused
        self.pause_btn.config(text="继续" if self.paused else "暂停")

    def _clear(self):
        """清除当前端口缓存和显示。"""
        self.port_cache[self.current_port] = None
        self.port_frame_counts[self.current_port] = 0
        self._clear_display()
        self._update_status_bar()

    def _on_close(self):
        """关闭窗口时清理 MainWindow 中的引用。"""
        if hasattr(self.parent, 'serial_monitor'):
            self.parent.serial_monitor = None
        self.window.destroy()


# ============================================================================
# 入口
# ============================================================================

def main():
    """程序入口。"""
    # Windows 高分屏适配
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = BoardServerGUI()
    app.run()


if __name__ == "__main__":
    main()
