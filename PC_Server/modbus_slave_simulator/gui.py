#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 Modbus 从机模拟器 GUI  —  PC-Side Slave Simulator (tkinter)
================================================================================

 功能概述:
   - RS485 串口从机模拟器, 覆盖全部 14 种 Modbus 模块类型
   - 图形化串口配置 (端口/波特率/数据位/校验位)
   - 设备列表实时显示请求/错误计数
   - 运行时切换曲线模式
   - 寄存器实时监视弹窗
   - 通信日志 (请求/响应 hex dump + 彩色标签)

 架构:
   主线程 = tkinter GUI (事件循环)
   后台线程 = 串口接收 (读帧 → 分发 → 发送响应)
   通过 queue.Queue 实现线程间安全通信

 运行要求:
   Python 3.6+, tkinter (标准库), pyserial
================================================================================
"""

import json
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox
from typing import Optional

from serial_io import SerialPort
from slave_device import SlaveDevice
from module_profiles import (get_module_name, get_profile, FALLBACK_PROFILE,
                             FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION,
                             AXLE_COUNTER_TYPES)

# ============================================================================
# 常量
# ============================================================================

CONFIG_FILE = "config.json"

MODULE_TYPE_NAMES = {
    0x00: "未配置",
    0x31: "信号机开关量",  0x32: "信号机",        0x33: "25Hz 轨道接收器",
    0x34: "DC/AC 转换器", 0x35: "25Hz 轨道发送器", 0x36: "采集模块",
    0x37: "交流转辙机",    0x38: "直流四线制道岔",  0x39: "直流六线制道岔",
    0x47: "高频信号机",    0x48: "高频25Hz轨道接收", 0x49: "高频25Hz轨道发送",
    0x4A: "高频采集模块",  0x4B: "高频交流转辙机",   0x4C: "高频直流四线制",
    0x4D: "高频直流六线制",
    0x50: "计轴采集传感器",
    0x51: "站台门采集 (DC3W)",
    0x52: "站场联采集 (DC3W)",
    0x53: "高频计轴采集传感器",
}

BAUD_RATE_OPTIONS = ["9600", "19200", "38400", "57600"]
DATA_BIT_OPTIONS = ["7", "8"]
STOP_BIT_OPTIONS = ["1", "1.5", "2"]
PARITY_OPTIONS = ["无 (N)", "奇校验 (O)", "偶校验 (E)"]


# ============================================================================
# 后台模拟器线程
# ============================================================================

class SimulatorThread(threading.Thread):
    """后台线程: 管理串口和从机设备, 循环读帧 → 分发 → 响应。"""

    def __init__(self):
        super().__init__(daemon=True)
        self._running = False
        self._serial: Optional[SerialPort] = None
        self._devices: dict[int, SlaveDevice] = {}
        self._config: dict = {}
        self._lock = threading.Lock()

        # 事件队列 → GUI
        self.event_queue = queue.Queue()

        # 统计
        self.start_time = 0.0
        self._last_rx_time = 0.0       # 上次收到数据的时间
        self._waiting_notified = False  # 是否已打印过等待提示

    # ------------------------------------------------------------------
    # 配置
    # ------------------------------------------------------------------

    def load_config(self, config_path: str) -> bool:
        """加载 JSON 配置文件。返回 True 成功。"""
        # 查找配置文件
        path = config_path
        if not os.path.isabs(path):
            alt = os.path.join(os.path.dirname(__file__), path)
            if os.path.exists(alt):
                path = alt
            elif not os.path.exists(path):
                alt2 = os.path.join(os.path.dirname(os.path.dirname(__file__)), path)
                if os.path.exists(alt2):
                    path = alt2

        if not os.path.exists(path):
            self._post_event("error", f"配置文件不存在: {path}")
            return False

        try:
            with open(path, "r", encoding="utf-8") as f:
                self._config = json.load(f)
            self._post_event("info", f"已加载配置: {path}")
            return True
        except json.JSONDecodeError as e:
            self._post_event("error", f"配置 JSON 解析错误: {e}")
            return False

    def get_config(self) -> dict:
        return self._config

    # ------------------------------------------------------------------
    # 设备创建
    # ------------------------------------------------------------------

    def create_devices(self) -> int:
        """根据配置创建设备。返回设备数量。"""
        with self._lock:
            self._devices.clear()

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
                )
                self._devices[addr] = device
                self._post_event("device_added", {
                    "address": addr,
                    "module_type": device.module_type,
                    "name": get_module_name(device.module_type),
                    "is_curve_capable": device.is_curve_capable,
                    "curve_mode": device.curve_mode or "N/A",
                    "input_regs": device.profile.input_regs,
                    "is_high_frequency": device.profile.is_high_frequency,
                })

            count = len(self._devices)
            self._post_event("info", f"已创建 {count} 台从机设备")
            return count

    # ------------------------------------------------------------------
    # 串口
    # ------------------------------------------------------------------

    def open_serial(self, port: str, baudrate: int, bytesize: int,
                    parity: str, stopbits: float) -> bool:
        """打开串口。返回 True 成功。"""
        if self._serial and self._serial.is_open:
            self.close_serial()

        # 映射 stopbits
        stopbits_int = 1
        if stopbits == 1.5:
            stopbits_int = 2  # pyserial: 2 = 1.5 stop bits
        elif stopbits >= 2:
            stopbits_int = 2

        serial_cfg = self._config.get("serial", {})
        self._serial = SerialPort(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits_int,
            timeout=serial_cfg.get("timeout", 0.005),
            rs485_delay_ms=serial_cfg.get("rs485_delay_ms", 1),
            verbose=True,
        )

        try:
            self._serial.open()
            self._post_event("serial_opened", {
                "port": port,
                "baudrate": baudrate,
                "bytesize": bytesize,
                "parity": parity,
                "stopbits": stopbits_int,
            })
            return True
        except Exception as e:
            self._post_event("error", f"无法打开串口 {port}: {e}")
            self._serial = None
            return False

    def close_serial(self) -> None:
        """关闭串口。"""
        if self._serial:
            self._serial.close()
            self._serial = None
            self._post_event("serial_closed", "串口已关闭")

    def is_serial_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    # ------------------------------------------------------------------
    # 运行循环
    # ------------------------------------------------------------------

    def run(self):
        """启动后台线程。"""
        self._running = True
        self.start_time = time.monotonic()
        self._last_rx_time = self.start_time
        self._post_event("simulator_started", "模拟器已启动")
        print(f"\n{'='*55}")
        print(f"  Modbus 从机模拟器 — 后台线程已启动")
        print(f"  共加载 {len(self._devices)} 台从机设备")
        print(f"{'='*55}")

        while self._running:
            try:
                self._process_one_cycle()
            except Exception as e:
                self._post_event("error", f"运行异常: {e}")
                time.sleep(0.01)

    def _process_one_cycle(self):
        """单次处理周期。"""
        if not self._serial or not self._serial.is_open:
            time.sleep(0.05)
            return

        frame = self._serial.read_frame()
        if frame is None or len(frame) < 4:
            # 串口已打开但无数据: 首次等待3秒后给出提示
            if (not self._waiting_notified
                    and time.monotonic() - self.start_time > 3.0):
                print(f"  [SERIAL] ⏳ 串口已就绪，等待主站请求数据 ... "
                      f"(已等待 {time.monotonic() - self.start_time:.0f}s)")
                self._waiting_notified = True
            time.sleep(0.001)
            return

        self._last_rx_time = time.monotonic()
        self._waiting_notified = True  # 收到数据后不再显示等待提示

        addr = frame[0]
        with self._lock:
            device = self._devices.get(addr)

        if device is not None and device.can_handle(frame):
            response = device.handle_request(frame)
            if response is not None:
                # ── 先发送应答 (必须在100ms超时内完成, 打印日志/事件不得阻塞此路径) ──
                self._serial.write_frame(response)

                # ── 控制台数据交互流 (在发送之后打印, 不占用通信超时窗口) ──
                module_name = get_module_name(device.module_type)
                flag_info = ""
                is_curve = device.is_curve_capable
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
                elif is_curve and len(response) > device.profile.curve_flag_offset:
                    flag_byte = response[device.profile.curve_flag_offset]
                    flag_names = {0x00: "实时帧", 0x55: "中间帧", 0xFF: "结束帧"}
                    flag_info = f" [{flag_names.get(flag_byte, f'0x{flag_byte:02X}')}]"

                print(f"  [FLOW] {'─'*50}")
                print(f"  [FLOW] 设备 #{device.request_count} | "
                      f"地址=0x{addr:02X} | 类型=0x{device.module_type:02X} "
                      f"{module_name}{flag_info}")
                print(f"  [FLOW] 主机→从机: {frame.hex(' ')}")
                print(f"  [FLOW] 从机→主机: {response.hex(' ')}")

                # 发送事件给 GUI
                flags = {}
                if device.module_type in AXLE_COUNTER_TYPES:
                    flags["axle_counter"] = True
                elif device.module_type in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
                    flags["dc3wire"] = True
                elif is_curve and len(response) > device.profile.curve_flag_offset:
                    flag_byte = response[device.profile.curve_flag_offset]
                    flags["frame_flag"] = flag_byte
                    flags["frame_seq"] = (
                        response[device.profile.curve_seq_offset]
                        if len(response) > device.profile.curve_seq_offset else 0
                    )

                self._post_event("traffic", {
                    "address": addr,
                    "module_type": device.module_type,
                    "module_name": get_module_name(device.module_type),
                    "request": frame,
                    "response": response,
                    "is_curve": is_curve,
                    "flag_info": flag_info,
                    "flags": flags,
                    "req_count": device.request_count,
                    "err_count": device.error_count,
                })

                # 计轴曲线: 通信板水位 ACK 确认后, 通知 GUI 更新成功计数
                if device.module_type in AXLE_COUNTER_TYPES and device.pop_curve_confirmed():
                    self._post_event("curve_confirmed", {
                        "address": addr,
                        "send_count": device.curve_send_count,
                    })
        else:
            # 收到数据但无匹配设备
            if device is None:
                print(f"  [FLOW] ⚠ 收到未知设备请求: 地址=0x{addr:02X} "
                      f"({len(frame)}B): {frame.hex(' ')}")
            else:
                print(f"  [FLOW] ⚠ 设备 0x{addr:02X} 无法处理请求 "
                      f"({len(frame)}B): {frame.hex(' ')}")

    def stop(self):
        """停止模拟器。"""
        self._running = False
        self.close_serial()
        self._post_event("simulator_stopped", "模拟器已停止")

    # ------------------------------------------------------------------
    # 设备操作 (线程安全)
    # ------------------------------------------------------------------

    def toggle_curve_mode(self, addr: int) -> Optional[str]:
        """切换曲线模式。返回新模式或 None。"""
        with self._lock:
            dev = self._devices.get(addr)
            if dev is None or not dev.is_curve_capable:
                return None
            new_mode = dev.toggle_curve_mode()
            return new_mode

    # ------------------------------------------------------------------
    # 计轴曲线手动发送 (按钮驱动)
    # ------------------------------------------------------------------

    def trigger_curve_send(self, addr: int) -> bool:
        """让计轴设备入队一趟曲线 (当前所选通道)。

        Returns:
            True 成功入队; False 设备不存在/非计轴/上一批未确认/实时模式。
        """
        with self._lock:
            dev = self._devices.get(addr)
            if dev is None:
                return False
            return dev.trigger_curve_send()

    def cancel_curve_send(self, addr: int) -> None:
        """取消计轴设备未确认的曲线批次 (发送超时用)。"""
        with self._lock:
            dev = self._devices.get(addr)
            if dev is not None:
                dev.cancel_curve_send()

    def set_curve_channel(self, addr: int, channel: int) -> bool:
        """切换计轴设备下次曲线发送的通道。"""
        with self._lock:
            dev = self._devices.get(addr)
            if dev is None:
                return False
            return dev.set_curve_channel(channel)

    def get_curve_channel(self, addr: int) -> Optional[int]:
        """读取计轴设备当前所选通道。"""
        with self._lock:
            dev = self._devices.get(addr)
            if dev is None:
                return None
            return dev.current_channel

    def get_curve_state(self, addr: int) -> Optional[dict]:
        """读取计轴曲线状态: {in_flight, send_count, channel}。"""
        with self._lock:
            dev = self._devices.get(addr)
            if dev is None:
                return None
            return {
                "in_flight": dev.curve_in_flight,
                "send_count": dev.curve_send_count,
                "channel": dev.current_channel,
            }

    def modify_device(self, addr: int, module_type, *,
                      curve_mode: str = "realtime",
                      allow_extended_read: bool = False,
                      curve_frames_per_cycle: int = 8,
                      curve_points_per_frame: int = 25) -> Optional[str]:
        """运行时修改某地址的设备类型和参数。返回新模块名称或 None。"""
        if isinstance(module_type, str):
            module_type = int(module_type, 16) if module_type.startswith("0x") else int(module_type)

        sim_cfg = self._config.get("simulation", {})
        new_dev = SlaveDevice(
            address=addr,
            module_type=module_type,
            curve_mode=curve_mode,
            curve_frames_per_cycle=curve_frames_per_cycle,
            curve_points_per_frame=curve_points_per_frame,
            allow_extended_read=allow_extended_read,
            alarm_probability=sim_cfg.get("alarm_probability", 0.02),
            sine_period_s=sim_cfg.get("sine_period_seconds", 10.0),
            random_walk_step=sim_cfg.get("random_walk_step", 5.0),
        )

        name = get_module_name(module_type)
        with self._lock:
            self._devices[addr] = new_dev

        # 同步更新内存中的 config dict, 方便后续保存
        for dev_cfg in self._config.get("devices", []):
            if dev_cfg.get("address") == addr:
                dev_cfg["module_type"] = f"0x{module_type:02X}"
                dev_cfg["curve_mode"] = curve_mode
                dev_cfg["allow_extended_read"] = allow_extended_read
                dev_cfg["curve_frames_per_cycle"] = curve_frames_per_cycle
                dev_cfg["curve_points_per_frame"] = curve_points_per_frame
                break

        self._post_event("info",
                         f"设备 0x{addr:02X} 已更新为 0x{module_type:02X} {name}")
        return name

    def add_device(self, addr: int, module_type, *,
                   curve_mode: str = "realtime",
                   allow_extended_read: bool = False,
                   curve_frames_per_cycle: int = 8,
                   curve_points_per_frame: int = 25,
                   silent: bool = False) -> Optional[str]:
        """运行时添加一台新设备。返回新模块名称或 None (地址冲突时)。

        Args:
            silent: True 时不发事件通知 (用于批量操作, 如满配置)。
        """
        if isinstance(module_type, str):
            module_type = int(module_type, 16) if module_type.startswith("0x") else int(module_type)

        with self._lock:
            if addr in self._devices:
                if not silent:
                    self._post_event("warning",
                                     f"地址 0x{addr:02X} 已被占用, 请先删除或编辑该设备")
                return None

            sim_cfg = self._config.get("simulation", {})
            device = SlaveDevice(
                address=addr,
                module_type=module_type,
                curve_mode=curve_mode,
                curve_frames_per_cycle=curve_frames_per_cycle,
                curve_points_per_frame=curve_points_per_frame,
                allow_extended_read=allow_extended_read,
                alarm_probability=sim_cfg.get("alarm_probability", 0.02),
                sine_period_s=sim_cfg.get("sine_period_seconds", 10.0),
                random_walk_step=sim_cfg.get("random_walk_step", 5.0),
            )
            self._devices[addr] = device

        name = get_module_name(module_type)
        if not silent:
            self._post_event("device_added", {
                "address": addr,
                "module_type": device.module_type,
                "name": name,
                "is_curve_capable": device.is_curve_capable,
                "curve_mode": device.curve_mode or "N/A",
                "input_regs": device.profile.input_regs,
                "is_high_frequency": device.profile.is_high_frequency,
            })
            self._post_event("info",
                             f"已添加设备 0x{addr:02X} — 0x{module_type:02X} {name}")
        return name

    def remove_device(self, addr: int) -> bool:
        """运行时删除一台设备。返回 True 成功。"""
        with self._lock:
            if addr not in self._devices:
                return False
            del self._devices[addr]
        self._post_event("device_removed", addr)
        self._post_event("info", f"已删除设备 0x{addr:02X}")
        return True

    def remove_all_devices(self, silent: bool = False) -> int:
        """删除所有设备。返回删除数量。"""
        with self._lock:
            count = len(self._devices)
            self._devices.clear()
        if not silent:
            self._post_event("devices_cleared", count)
            self._post_event("info", f"已清除全部 {count} 台设备")
        return count

    def get_device(self, addr: int) -> Optional[SlaveDevice]:
        with self._lock:
            return self._devices.get(addr)

    def get_devices(self) -> dict[int, SlaveDevice]:
        with self._lock:
            return dict(self._devices)

    def get_device_registers(self, addr: int, count: int = 20) -> list[int]:
        """获取设备当前寄存器值快照 (不消耗 tick，不影响通信数据)。"""
        dev = self.get_device(addr)
        if dev is None:
            return []
        n = min(count, dev.profile.input_regs)
        return dev.peek_registers(n)

    def set_device_register(self, addr: int, reg: int, value: int) -> bool:
        """手动设置设备寄存器的值。返回 True 成功。"""
        dev = self.get_device(addr)
        if dev is None:
            return False
        dev.set_register_value(reg, value)
        return True

    def clear_device_register(self, addr: int, reg: int) -> bool:
        """清除手动覆写，恢复自动生成的寄存器值。"""
        dev = self.get_device(addr)
        if dev is None:
            return False
        dev.clear_register_value(reg)
        return True

    # ------------------------------------------------------------------
    # 事件
    # ------------------------------------------------------------------

    def _post_event(self, etype: str, message):
        self.event_queue.put({
            "type": etype,
            "message": message,
            "timestamp": datetime.now().strftime("%H:%M:%S"),
        })


# ============================================================================
# GUI 主窗口
# ============================================================================

class SlaveSimulatorGUI:
    """从机模拟器主窗口。"""

    def __init__(self, config_path: str = CONFIG_FILE):
        self._config_path = config_path
        self.sim: Optional[SimulatorThread] = None

        # 寄存器监视器引用
        self.register_watch: Optional[RegisterWatchWindow] = None

        # 通信日志限速 (每地址每秒最多 1 条摘要)
        self._last_traffic_log: dict[int, float] = {}

        # 计轴曲线手动发送状态 (发送按钮 / 通道 / 成功计数)
        self._curve_send_addr: Optional[int] = None   # 正在等待通信板确认的设备
        self._curve_send_start: float = 0.0
        self._curve_send_timeout: float = 20.0

        # ----- 主窗口 -----
        self.root = tk.Tk()
        self.root.title("Modbus 从机模拟器 v1.0")
        self.root.geometry("1050x700")
        self.root.minsize(850, 500)

        self._build_ui()
        self._start_simulator()

    # ================================================================
    #  UI 构建
    # ================================================================

    def _build_ui(self):
        """构建界面。"""
        # 设置 Treeview 样式: 增大行高 + 中文字体, 避免文字被裁切堆叠
        style = ttk.Style()
        style.configure("DeviceList.Treeview",
                        rowheight=30,
                        font=("Microsoft YaHei", 9))
        # 表头字体: 用基础 Treeview.Heading 而非 DeviceList.Treeview.Heading,
        # 避免部分 Windows 主题抛出 "Can't find filter element" 警告
        try:
            style.configure("Treeview.Heading",
                            font=("Microsoft YaHei", 9, "bold"))
        except Exception:
            pass  # 部分主题不支持, 忽略即可

        # ---- 顶部状态栏 ----
        self._build_top_bar()

        # ---- 主体: 使用 PanedWindow 支持拖动调整左右比例 ----
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧面板 (可拖动调整宽度)
        left_panel = ttk.Frame(paned, width=420)
        left_panel.pack_propagate(False)  # 保持内容不撑开 PanedWindow

        # 右侧日志区
        right_panel = ttk.Frame(paned)

        paned.add(left_panel, weight=0)   # weight=0: 拖动时优先保持左侧宽度
        paned.add(right_panel, weight=1)  # weight=1: 右侧随窗口拉伸

        self._build_left_panel(left_panel)
        self._build_log_panel(right_panel)

        # ---- 底部状态栏 ----
        self._build_bottom_bar()

    def _build_top_bar(self):
        """顶部串口状态栏。"""
        frame = ttk.Frame(self.root, relief=tk.RIDGE, borderwidth=1)
        frame.pack(fill=tk.X, padx=5, pady=(5, 0))

        # 状态灯
        self.status_canvas = tk.Canvas(frame, width=20, height=20,
                                       highlightthickness=0)
        self.status_canvas.pack(side=tk.LEFT, padx=10)
        self._draw_status_dot("gray")

        self.status_label = ttk.Label(frame, text="串口状态: 未打开",
                                      font=("Microsoft YaHei", 10, "bold"))
        self.status_label.pack(side=tk.LEFT, padx=5)

        self.port_info_label = ttk.Label(frame, text="",
                                         font=("Microsoft YaHei", 9))
        self.port_info_label.pack(side=tk.RIGHT, padx=10)

    def _build_left_panel(self, parent):
        """左侧控制面板。"""
        # ---- 串口设置 ----
        ser_frame = ttk.LabelFrame(parent, text="串口设置", padding=5)
        ser_frame.pack(fill=tk.X, pady=(0, 10))

        # 第1行: 端口 + 波特率 (用 grid 确保对齐不重叠)
        row1 = ttk.Frame(ser_frame)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="端口:", anchor=tk.W).grid(row=0, column=0, sticky=tk.W, padx=(0, 3))
        self.port_var = tk.StringVar(value="COM5")
        self.port_combo = ttk.Combobox(row1, textvariable=self.port_var, width=14)
        self.port_combo.grid(row=0, column=1, padx=(0, 12))
        ttk.Button(row1, text="刷新", width=5,
                   command=self._refresh_ports).grid(row=0, column=2)

        ttk.Label(row1, text="波特率:", anchor=tk.W).grid(row=0, column=3, sticky=tk.W, padx=(0, 3))
        self.baud_var = tk.StringVar(value="19200")
        ttk.Combobox(row1, textvariable=self.baud_var, values=BAUD_RATE_OPTIONS,
                     state="readonly", width=10).grid(row=0, column=4)

        # 第2行: 数据位 + 停止位 + 校验
        row2 = ttk.Frame(ser_frame)
        row2.pack(fill=tk.X, pady=2)
        ttk.Label(row2, text="数据位:", anchor=tk.W).grid(row=0, column=0, sticky=tk.W, padx=(0, 3))
        self.data_bit_var = tk.StringVar(value="8")
        ttk.Combobox(row2, textvariable=self.data_bit_var, values=DATA_BIT_OPTIONS,
                     state="readonly", width=6).grid(row=0, column=1, padx=(0, 12))

        ttk.Label(row2, text="停止位:", anchor=tk.W).grid(row=0, column=2, sticky=tk.W, padx=(0, 3))
        self.stop_bit_var = tk.StringVar(value="1")
        ttk.Combobox(row2, textvariable=self.stop_bit_var, values=STOP_BIT_OPTIONS,
                     state="readonly", width=6).grid(row=0, column=3, padx=(0, 12))

        ttk.Label(row2, text="校验:", anchor=tk.W).grid(row=0, column=4, sticky=tk.W, padx=(0, 3))
        self.parity_var = tk.StringVar(value="无 (N)")
        ttk.Combobox(row2, textvariable=self.parity_var, values=PARITY_OPTIONS,
                     state="readonly", width=10).grid(row=0, column=5)

        # 打开/关闭按钮
        btn_row = ttk.Frame(ser_frame)
        btn_row.pack(fill=tk.X, pady=(8, 2))
        self.open_btn = ttk.Button(btn_row, text="打开串口",
                                   command=self._toggle_serial)
        self.open_btn.pack(side=tk.LEFT, padx=(0, 5))
        ttk.Button(btn_row, text="刷新端口列表",
                   command=self._refresh_ports).pack(side=tk.LEFT)

        # ---- 设备列表 ----
        dev_frame = ttk.LabelFrame(parent, text="从机设备列表", padding=3)
        dev_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # Treeview (使用自定义 style 增大行高, 避免中文被裁切)
        columns = ("addr", "type", "name", "mode", "req", "err")
        self.dev_tree = ttk.Treeview(dev_frame, columns=columns,
                                     show="headings", height=14,
                                     style="DeviceList.Treeview")
        self.dev_tree.heading("addr", text="地址")
        self.dev_tree.heading("type", text="类型")
        self.dev_tree.heading("name", text="模块名称")
        self.dev_tree.heading("mode", text="模式")
        self.dev_tree.heading("req", text="请求")
        self.dev_tree.heading("err", text="错误")

        self.dev_tree.column("addr", width=40, anchor=tk.CENTER)
        self.dev_tree.column("type", width=50, anchor=tk.CENTER)
        self.dev_tree.column("name", width=150, anchor=tk.W)
        self.dev_tree.column("mode", width=75, anchor=tk.CENTER)
        self.dev_tree.column("req", width=50, anchor=tk.CENTER)
        self.dev_tree.column("err", width=45, anchor=tk.CENTER)

        dev_scroll = ttk.Scrollbar(dev_frame, orient=tk.VERTICAL,
                                   command=self.dev_tree.yview)
        self.dev_tree.configure(yscrollcommand=dev_scroll.set)

        self.dev_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dev_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 绑定选择事件
        self.dev_tree.bind("<<TreeviewSelect>>", self._on_device_select)

        # ---- 设备操作 ----
        act_frame = ttk.LabelFrame(parent, text="设备操作", padding=5)
        act_frame.pack(fill=tk.X, pady=(0, 10))

        self.sel_dev_label = ttk.Label(act_frame, text="请从列表中选中一台设备",
                                       font=("Microsoft YaHei", 9))
        self.sel_dev_label.pack(anchor=tk.W, pady=(0, 5))

        self.sel_dev_info = ttk.Label(act_frame, text="",
                                      font=("Microsoft YaHei", 8), foreground="gray")
        self.sel_dev_info.pack(anchor=tk.W, pady=(0, 5))

        # 按钮分行放置 (每行2个, 避免全部挤在一行导致窗口过宽)
        btn_row1 = ttk.Frame(act_frame)
        btn_row1.pack(fill=tk.X, pady=(0, 3))
        ttk.Button(btn_row1, text="添加设备",
                   command=self._add_device).pack(side=tk.LEFT, padx=(0, 5), fill=tk.X, expand=True)

        self.edit_btn = ttk.Button(btn_row1, text="编辑设备",
                                   command=self._edit_device, state=tk.DISABLED)
        self.edit_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

        btn_row2 = ttk.Frame(act_frame)
        btn_row2.pack(fill=tk.X, pady=(0, 3))
        self.del_btn = ttk.Button(btn_row2, text="删除设备",
                                  command=self._delete_device, state=tk.DISABLED)
        self.del_btn.pack(side=tk.LEFT, padx=(0, 5), fill=tk.X, expand=True)

        self.curve_btn = ttk.Button(btn_row2, text="切换曲线模式",
                                    command=self._toggle_curve, state=tk.DISABLED)
        self.curve_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

        btn_row3 = ttk.Frame(act_frame)
        btn_row3.pack(fill=tk.X)
        self.watch_btn = ttk.Button(btn_row3, text="打开寄存器监视",
                                    command=self._open_register_watch,
                                    state=tk.DISABLED)
        self.watch_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # ---- 计轴曲线: 发送按钮 + 通道切换 + 发送成功计数 ----
        axle_row = ttk.Frame(act_frame)
        axle_row.pack(fill=tk.X, pady=(6, 3))
        self.send_curve_btn = ttk.Button(axle_row, text="发送曲线",
                                         command=self._send_curve,
                                         state=tk.DISABLED)
        self.send_curve_btn.pack(side=tk.LEFT, padx=(0, 5), fill=tk.X,
                                 expand=True)
        self.channel_btn = ttk.Button(axle_row, text="通道: 0",
                                      command=self._cycle_channel,
                                      state=tk.DISABLED)
        self.channel_btn.pack(side=tk.LEFT)
        self.curve_send_count_label = ttk.Label(axle_row, text="发送成功: 0",
                                                font=("Microsoft YaHei", 9))
        self.curve_send_count_label.pack(side=tk.LEFT, padx=(6, 0))

        self._selected_addr: Optional[int] = None

        # ---- 显示选项 ----
        opt_frame = ttk.LabelFrame(parent, text="显示选项", padding=5)
        opt_frame.pack(fill=tk.X)

        self.show_hex_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opt_frame, text="显示原始帧 hex dump",
                        variable=self.show_hex_var).pack(anchor=tk.W)

        self.auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(opt_frame, text="自动滚动日志",
                        variable=self.auto_scroll_var).pack(anchor=tk.W)

        ttk.Button(opt_frame, text="清空日志",
                   command=self._clear_log).pack(fill=tk.X, pady=(5, 0))

        # ---- 配置 ----
        cfg_frame = ttk.LabelFrame(parent, text="配置", padding=5)
        cfg_frame.pack(fill=tk.X, pady=(10, 0))

        self.cfg_label = ttk.Label(cfg_frame, text=f"配置文件: {self._config_path}",
                                   font=("Microsoft YaHei", 8), foreground="gray")
        self.cfg_label.pack(anchor=tk.W)

        cfg_btn_row = ttk.Frame(cfg_frame)
        cfg_btn_row.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(cfg_btn_row, text="重新加载配置",
                   command=self._reload_config).pack(side=tk.LEFT, fill=tk.X,
                   expand=True, padx=(0, 3))
        ttk.Button(cfg_btn_row, text="满配置",
                   command=self._fill_max_config).pack(side=tk.RIGHT,
                   padx=(3, 0))
        ttk.Button(cfg_btn_row, text="清空设备",
                   command=self._clear_all_devices).pack(side=tk.RIGHT)

    def _build_log_panel(self, parent):
        """右侧日志区。"""
        frame = ttk.LabelFrame(parent, text="通信日志", padding=3)
        frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(frame, wrap=tk.WORD, state=tk.DISABLED,
                                font=("Consolas", 9),
                                bg="#1E1E1E", fg="#D4D4D4",
                                insertbackground="white")
        log_scroll = ttk.Scrollbar(frame, orient=tk.VERTICAL,
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
        self.log_text.tag_configure("time",    foreground="#808080")
        self.log_text.tag_configure("data",    foreground="#DCDCAA")
        self.log_text.tag_configure("traffic", foreground="#4EC9B0")

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
    #  日志辅助
    # ================================================================

    def _log(self, tag: str, message: str):
        """追加带时间戳和颜色的日志。"""
        now = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{now}] ", "time")
        self.log_text.insert(tk.END, message + "\n", tag)
        if self.auto_scroll_var.get():
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
    #  启动 / 停止
    # ================================================================

    def _start_simulator(self):
        """创建并启动模拟器线程。"""
        self.sim = SimulatorThread()

        if not self.sim.load_config(self._config_path):
            self._log("warning", "未找到配置文件, 使用空白配置 (请手动添加从机设备)")
            # 空白配置: 不预创建任何从机设备, 由用户逐个添加
            self.sim._config = {
                "serial": {"port": "COM5", "baudrate": 19200, "bytesize": 8,
                          "parity": "N", "stopbits": 1, "timeout": 0.005,
                          "rs485_delay_ms": 1},
                "devices": [],
                "simulation": {},
            }

        self.sim.create_devices()
        self._populate_device_list()
        if len(self.sim.get_devices()) == 0:
            self._log("info", "当前无设备 — 请使用「添加设备」按钮手动添加从机")
        self._curve_send_timeout = float(
            self.sim.get_config().get("simulation", {})
            .get("curve_send_timeout_s", 20.0))
        self.sim.start()

        self._draw_status_dot("gray")
        self.status_label.config(text="串口状态: 未打开")
        self.bottom_label.config(text="请配置串口并点击「打开串口」")
        self.open_btn.config(text="打开串口")

        # 刷新端口列表
        self._refresh_ports()

        # 开始事件轮询
        self._poll_events()

    def _reload_config(self):
        """重新加载配置文件。"""
        was_open = self.sim and self.sim.is_serial_open()
        if was_open:
            self.sim.close_serial()

        if self.sim:
            self.sim.stop()

        # 重新创建
        self.sim = SimulatorThread()
        if self.sim.load_config(self._config_path):
            self.sim.create_devices()
            self.sim.start()
            self._populate_device_list()

            if was_open:
                # 恢复串口
                self._toggle_serial()

            self._log("success", "配置已重新加载")
        else:
            self._log("error", "配置重新加载失败")

        # 更新设备列表
        for item in self.dev_tree.get_children():
            self.dev_tree.delete(item)
        self._populate_device_list()

        # 重置曲线发送状态
        self._curve_send_addr = None
        self._refresh_axle_curve_ui()

    # ================================================================
    #  事件轮询
    # ================================================================

    def _poll_events(self):
        """定时从事件队列取出消息并更新 GUI。"""
        if self.sim is None:
            self.root.after(100, self._poll_events)
            return

        msg_count = 0
        try:
            while True:
                event = self.sim.event_queue.get_nowait()
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

        # 计轴曲线发送超时: 通信板长时间未确认 → 取消该批, 按钮恢复
        if (self._curve_send_addr is not None
                and self.sim is not None
                and time.time() - self._curve_send_start > self._curve_send_timeout):
            addr = self._curve_send_addr
            self.sim.cancel_curve_send(addr)
            self._curve_send_addr = None
            self._log("warning",
                      f"设备 {addr} 曲线发送超时 ({self._curve_send_timeout:.0f}s), "
                      f"通信板未确认, 已取消")
            # 恢复当前选中设备应有的按钮状态 (可能已切换到别的设备)
            self._refresh_axle_curve_ui()

        self.root.after(100, self._poll_events)

    def _process_event(self, event: dict):
        """处理单个事件。"""
        etype = event["type"]
        msg = event["message"]

        if etype == "simulator_started":
            self._log("info", "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            self._log("info", "从机模拟器已启动 (后台线程)")

        elif etype == "device_added":
            self._log("info",
                      f"  设备 0x{msg['address']:02X}  "
                      f"0x{msg['module_type']:02X} {msg['name']}")

        elif etype == "serial_opened":
            self._draw_status_dot("green")
            self.status_label.config(text="串口状态: 已打开")
            self.port_info_label.config(
                text=f"{msg['port']} @ {msg['baudrate']} "
                     f"{msg['bytesize']}{msg['parity']}{msg['stopbits']}")
            self.open_btn.config(text="关闭串口")
            self.bottom_label.config(
                text=f"串口 {msg['port']} 已打开, 等待主站请求...")
            self._log("success", f"串口已打开: {msg['port']} @ {msg['baudrate']}")

        elif etype == "serial_closed":
            self._draw_status_dot("gray")
            self.status_label.config(text="串口状态: 未打开")
            self.port_info_label.config(text="")
            self.open_btn.config(text="打开串口")
            self.bottom_label.config(text="串口已关闭")
            self._log("warning", str(msg))

        elif etype == "traffic":
            # 通信数据
            show_hex = self.show_hex_var.get()

            # 限速: 同一地址每秒最多输出一条完整日志
            addr = msg["address"]
            now = time.time()
            last = self._last_traffic_log.get(addr, 0)

            if now - last >= 0.3:
                self._last_traffic_log[addr] = now

                req = msg["request"]
                resp = msg["response"]
                flag_str = f" [{msg['flag_info']}]" if msg['flag_info'] else ""

                self._log("traffic",
                          f"━━ 设备 {addr} (0x{msg['module_type']:02X} "
                          f"{msg['module_name']}){flag_str} "
                          f"#{msg['req_count']}")

                if show_hex:
                    self._log("data",
                              f"  ← 请求 ({len(req)}B): {req.hex(' ')}")
                    self._log("data",
                              f"  → 响应 ({len(resp)}B): {resp.hex(' ')}")
                else:
                    self._log("data",
                              f"  ← {len(req)}B → {len(resp)}B")

            # 更新设备列表中的计数
            self._update_device_row(addr, msg["req_count"], msg["err_count"])

            # 转发给寄存器监视器
            if self.register_watch is not None:
                try:
                    if self.register_watch.window.winfo_exists():
                        self.register_watch.handle_traffic(msg)
                except tk.TclError:
                    self.register_watch = None

        elif etype == "curve_confirmed":
            # 计轴曲线已被通信板确认接收 → 计数+1, 按钮恢复
            addr = msg["address"]
            count = msg["send_count"]
            self._log("success",
                      f"设备 {addr} 曲线已被通信板确认接收 (累计 {count} 条)")
            if self._selected_addr == addr:
                self.curve_send_count_label.config(text=f"发送成功: {count}")
                if self._curve_send_addr == addr:
                    self._curve_send_addr = None
                self.send_curve_btn.config(text="发送曲线", state=tk.NORMAL)

        elif etype == "simulator_stopped":
            self._log("warning", str(msg))

        elif etype == "device_removed":
            # 从设备列表中移除
            iid = str(msg)
            if self.dev_tree.exists(iid):
                self.dev_tree.delete(iid)
            # 如果寄存器监视器正在监视该设备, 关闭它
            if (self.register_watch is not None
                    and self.register_watch.address == msg):
                try:
                    self.register_watch.window.destroy()
                except tk.TclError:
                    pass
                self.register_watch = None
            # 如果被删除的是当前选中的设备, 清除选中状态
            if self._selected_addr == msg:
                self._selected_addr = None
                self.curve_btn.config(state=tk.DISABLED)
                self.watch_btn.config(state=tk.DISABLED)
                self.edit_btn.config(state=tk.DISABLED)
                self.del_btn.config(state=tk.DISABLED)
                self.sel_dev_label.config(text="请从列表中选中一台设备")
                self.sel_dev_info.config(text="")
            if self._curve_send_addr == msg:
                self._curve_send_addr = None
            self._refresh_axle_curve_ui()

        elif etype == "devices_cleared":
            # 清空设备列表
            for item in self.dev_tree.get_children():
                self.dev_tree.delete(item)
            self._selected_addr = None
            self.curve_btn.config(state=tk.DISABLED)
            self.watch_btn.config(state=tk.DISABLED)
            self.edit_btn.config(state=tk.DISABLED)
            self.del_btn.config(state=tk.DISABLED)
            self.sel_dev_label.config(text="请从列表中选中一台设备")
            self.sel_dev_info.config(text="")
            self._curve_send_addr = None
            self._refresh_axle_curve_ui()
            self._log("warning", f"已清除全部 {msg} 台设备")

        elif etype == "warning":
            self._log("warning", str(msg))

        elif etype == "info":
            self._log("info", str(msg))

        elif etype == "error":
            self._log("error", str(msg))
            self.bottom_label.config(text=f"错误: {msg}")

    # ================================================================
    #  串口按钮
    # ================================================================

    def _toggle_serial(self):
        """打开/关闭串口。"""
        if self.sim is None:
            return

        if self.sim.is_serial_open():
            self.sim.close_serial()
        else:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            data = int(self.data_bit_var.get())

            # 校验位解析
            parity_str = self.parity_var.get()
            if "N" in parity_str or "无" in parity_str:
                parity = "N"
            elif "O" in parity_str or "奇" in parity_str:
                parity = "O"
            else:
                parity = "E"

            # 停止位
            stop_str = self.stop_bit_var.get()
            stopbits: float = 1.0
            try:
                stopbits = float(stop_str)
            except ValueError:
                stopbits = 1.0

            self.sim.open_serial(port, baud, data, parity, stopbits)

    def _refresh_ports(self):
        """刷新可用串口列表。"""
        try:
            # 重定向 stderr 以抑制 pyserial 的 "Can't find filter element" 等噪音
            import io
            old_stderr = sys.stderr
            sys.stderr = io.StringIO()
            try:
                ports = SerialPort.list_ports()
            finally:
                sys.stderr = old_stderr
        except Exception as e:
            ports = []
            self._log("warning", f"串口扫描异常: {e}")
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self._log("info", f"可用串口: {ports if ports else '无'}")

    # ================================================================
    #  设备列表
    # ================================================================

    def _populate_device_list(self):
        """填充设备列表 Treeview。"""
        for item in self.dev_tree.get_children():
            self.dev_tree.delete(item)

        if self.sim is None:
            return

        devices = self.sim.get_devices()
        for addr in sorted(devices):
            dev = devices[addr]
            name = get_module_name(dev.module_type)
            mode = dev.curve_mode or "Modbus"
            flags = ""
            if dev.profile.is_high_frequency:
                flags = " HF"
            if dev.profile.exceeds_125_regs:
                flags += " >125"
            self.dev_tree.insert("", tk.END,
                                 iid=str(addr),
                                 values=(addr,
                                        f"0x{dev.module_type:02X}",
                                        name,
                                        mode + flags,
                                        dev.request_count,
                                        dev.error_count))

    def _update_device_row(self, addr: int, req_count: int, err_count: int):
        """更新单个设备行的请求/错误计数。"""
        iid = str(addr)
        if self.dev_tree.exists(iid):
            values = list(self.dev_tree.item(iid, "values"))
            if len(values) >= 6:
                values[4] = req_count
                values[5] = err_count
                self.dev_tree.item(iid, values=values)

    def _on_device_select(self, event):
        """设备列表选中事件。"""
        selection = self.dev_tree.selection()
        if not selection:
            self._selected_addr = None
            self.curve_btn.config(state=tk.DISABLED)
            self.watch_btn.config(state=tk.DISABLED)
            self.edit_btn.config(state=tk.DISABLED)
            self.del_btn.config(state=tk.DISABLED)
            self.sel_dev_label.config(text="请从列表中选中一台设备")
            self.sel_dev_info.config(text="")
            self._refresh_axle_curve_ui()
            return

        addr = int(selection[0])
        self._selected_addr = addr
        dev = self.sim.get_device(addr) if self.sim else None

        if dev is None:
            self._refresh_axle_curve_ui()
            return

        name = get_module_name(dev.module_type)
        self.sel_dev_label.config(
            text=f"选中: 地址 {addr} — 0x{dev.module_type:02X} {name}")

        info_parts = [
            f"输入寄存器: {dev.profile.input_regs}",
            f"保持寄存器: {dev.profile.holding_regs}",
        ]
        if dev.is_curve_capable:
            mode_label = ("脉冲模式" if dev.module_type in AXLE_COUNTER_TYPES
                          else "曲线模式")
            info_parts.append(f"{mode_label}: {dev.curve_mode}")
            if (dev.module_type not in AXLE_COUNTER_TYPES
                    and dev.module_type not in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION)):
                info_parts.append(f"分支数: {dev.profile.curve_branch_count}")
        if dev.profile.is_high_frequency:
            info_parts.append("高频模块")
        if dev.module_type in AXLE_COUNTER_TYPES:
            info_parts.append("计轴模块")
        if dev.module_type in (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
            info_parts.append("直流3对线")
        self.sel_dev_info.config(text="  |  ".join(info_parts))

        # 曲线/脉冲开关按钮
        if dev.is_curve_capable:
            btn_label = ("切换脉冲模式" if dev.module_type in AXLE_COUNTER_TYPES
                         else "切换曲线模式")
            self.curve_btn.config(state=tk.NORMAL,
                                  text=f"{btn_label} (当前: {dev.curve_mode})")
        else:
            self.curve_btn.config(state=tk.DISABLED, text="非曲线模块")

        self.watch_btn.config(state=tk.NORMAL)
        self.edit_btn.config(state=tk.NORMAL)
        self.del_btn.config(state=tk.NORMAL)
        self._refresh_axle_curve_ui()

    # ================================================================
    #  按钮操作
    # ================================================================

    def _toggle_curve(self):
        """切换曲线模式。"""
        if self._selected_addr is None or self.sim is None:
            return
        new_mode = self.sim.toggle_curve_mode(self._selected_addr)
        if new_mode is not None:
            self._log("success",
                      f"设备 {self._selected_addr} 曲线模式 → {new_mode}")
            self._on_device_select(None)  # 刷新按钮文字
            # 更新列表中的模式列
            iid = str(self._selected_addr)
            if self.dev_tree.exists(iid):
                values = list(self.dev_tree.item(iid, "values"))
                values[3] = new_mode
                self.dev_tree.item(iid, values=values)

    # ---------------------------------------------------------------
    #  计轴曲线: 发送 / 通道切换 / 状态刷新
    # ---------------------------------------------------------------

    def _refresh_axle_curve_ui(self):
        """刷新计轴曲线控制区 (发送按钮 / 通道 / 成功计数)。"""
        if self._selected_addr is None or self.sim is None:
            self.send_curve_btn.config(text="发送曲线", state=tk.DISABLED)
            self.channel_btn.config(text="通道: -", state=tk.DISABLED)
            self.curve_send_count_label.config(text="发送成功: -")
            return
        dev = self.sim.get_device(self._selected_addr)
        if dev is None or dev.module_type not in AXLE_COUNTER_TYPES:
            self.send_curve_btn.config(text="发送曲线", state=tk.DISABLED)
            self.channel_btn.config(text="通道: -", state=tk.DISABLED)
            self.curve_send_count_label.config(text="发送成功: -")
            return
        in_flight = dev.curve_in_flight
        self.send_curve_btn.config(
            text="正在发送中..." if in_flight else "发送曲线",
            state=tk.DISABLED if in_flight else tk.NORMAL)
        self.channel_btn.config(text=f"通道: {dev.current_channel}",
                                state=tk.NORMAL)
        self.curve_send_count_label.config(
            text=f"发送成功: {dev.curve_send_count}")
        if in_flight:
            self._curve_send_addr = self._selected_addr
            self._curve_send_start = time.time()

    def _send_curve(self):
        """发送一条计轴曲线 (按钮驱动)。"""
        if self._selected_addr is None or self.sim is None:
            return
        addr = self._selected_addr
        dev = self.sim.get_device(addr)
        if dev is None or dev.module_type not in AXLE_COUNTER_TYPES:
            return
        # 若还在实时模式, 自动切到曲线/脉冲模式
        if dev.curve_mode != "curve":
            new_mode = self.sim.toggle_curve_mode(addr)
            if new_mode is not None:
                self._log("info", f"设备 {addr} 已自动切换脉冲模式 → {new_mode}")
                self._on_device_select(None)
                iid = str(addr)
                if self.dev_tree.exists(iid):
                    values = list(self.dev_tree.item(iid, "values"))
                    values[3] = new_mode
                    self.dev_tree.item(iid, values=values)
        if not self.sim.trigger_curve_send(addr):
            self._log("warning",
                      f"设备 {addr} 上一批曲线尚未被通信板确认, 请等待")
            return
        self._curve_send_addr = addr
        self._curve_send_start = time.time()
        self.send_curve_btn.config(text="正在发送中...", state=tk.DISABLED)
        self._log("info",
                  f"设备 {addr} 曲线已入队 (通道 {dev.current_channel}), "
                  f"等待通信板读取确认...")

    def _cycle_channel(self):
        """循环切换计轴曲线通道 0→1→2→0。"""
        if self._selected_addr is None or self.sim is None:
            return
        addr = self._selected_addr
        state = self.sim.get_curve_state(addr)
        if state is None:
            return
        if not self.sim.set_curve_channel(addr, state["channel"] + 1):
            return
        new_ch = self.sim.get_curve_channel(addr)
        self.channel_btn.config(text=f"通道: {new_ch}")
        self._log("info", f"设备 {addr} 曲线通道 → {new_ch}")

    def _edit_device(self):
        """打开设备编辑对话框, 修改模块类型和参数。"""
        if self._selected_addr is None or self.sim is None:
            return
        dev = self.sim.get_device(self._selected_addr)
        if dev is None:
            return

        editor = DeviceEditDialog(self.root, self._selected_addr, dev)
        self.root.wait_window(editor.window)

        if editor.result is None:
            return  # 用户取消

        # 应用修改
        new_type = editor.result["module_type"]
        new_mode = editor.result["curve_mode"]
        new_ext = editor.result["allow_extended_read"]
        new_fpc = editor.result["curve_frames_per_cycle"]
        new_ppf = editor.result["curve_points_per_frame"]

        name = self.sim.modify_device(
            self._selected_addr, new_type,
            curve_mode=new_mode,
            allow_extended_read=new_ext,
            curve_frames_per_cycle=new_fpc,
            curve_points_per_frame=new_ppf,
        )
        if name:
            self._log("success",
                      f"设备 {self._selected_addr} → 0x{new_type:02X} {name}")
            # 刷新设备列表和选中状态
            self._populate_device_list()
            self._on_device_select(None)
            # 自动重新选中
            iid = str(self._selected_addr)
            if self.dev_tree.exists(iid):
                self.dev_tree.selection_set(iid)

    def _open_register_watch(self):
        """打开寄存器监视弹窗。"""
        if self._selected_addr is None:
            return
        if self.register_watch is not None:
            try:
                if self.register_watch.window.winfo_exists():
                    self.register_watch.window.lift()
                    return
            except tk.TclError:
                pass
        dev = self.sim.get_device(self._selected_addr) if self.sim else None
        if dev is None:
            return
        self.register_watch = RegisterWatchWindow(
            self.root, self._selected_addr, dev, self.sim, self)
        self._log("info",
                  f"已打开设备 {self._selected_addr} 的寄存器监视器")

    def _add_device(self):
        """打开添加设备对话框。"""
        dialog = AddDeviceDialog(self.root, self.sim)
        self.root.wait_window(dialog.window)

        if dialog.result is None:
            return

        addr = dialog.result["address"]
        mtype = dialog.result["module_type"]
        self.sim.add_device(
            addr, mtype,
            curve_mode=dialog.result["curve_mode"],
            allow_extended_read=dialog.result["allow_extended_read"],
            curve_frames_per_cycle=dialog.result["curve_frames_per_cycle"],
            curve_points_per_frame=dialog.result["curve_points_per_frame"],
        )
        # add_device() 内部已发事件通知, 此处不再重复日志
        self._populate_device_list()
        # 选中新设备
        iid = str(addr)
        if self.dev_tree.exists(iid):
            self.dev_tree.selection_set(iid)
            self.dev_tree.see(iid)
            self._on_device_select(None)

    def _delete_device(self):
        """删除选中的设备。"""
        if self._selected_addr is None or self.sim is None:
            return
        addr = self._selected_addr
        if not messagebox.askyesno("确认删除",
                                   f"确定要删除设备 0x{addr:02X} 吗？"):
            return
        if self.sim.remove_device(addr):
            self._log("warning", f"已删除设备 0x{addr:02X}")

    def _clear_all_devices(self):
        """清空全部设备。"""
        if self.sim is None:
            return
        devices = self.sim.get_devices()
        if not devices:
            self._log("info", "当前无设备, 无需清空")
            return
        if not messagebox.askyesno("确认清空",
                                   f"确定要删除全部 {len(devices)} 台设备吗？"):
            return
        self.sim.remove_all_devices()

    def _fill_max_config(self):
        """满配置: 13 台 0x48 高频25Hz轨道接收模块 (每台 133 个输入寄存器)。

        全部 13 台分配到地址 1-13, 总共 13×133 = 1729 个输入寄存器。
        用于测试串口最大数据承受量。静默操作, 不在日志中逐条打印。
        """
        if self.sim is None:
            return

        devices = self.sim.get_devices()
        if devices:
            if not messagebox.askyesno(
                    "确认满配置",
                    f"当前已有 {len(devices)} 台设备。\n\n"
                    "满配置将清空现有设备并创建 13 台 0x48 设备\n"
                    "(高频25Hz轨道接收, 每台 133 个输入寄存器)。\n\n"
                    "是否继续？"):
                return

        MAX_ADDR = 0x48  # 高频25Hz轨道接收 (133 regs)

        # 静默清空+静默添加, 不触发逐条事件
        self.sim.remove_all_devices(silent=True)

        total_regs = 0
        for addr in range(1, 14):
            total_regs += 133
            self.sim.add_device(
                addr, MAX_ADDR,
                curve_mode="realtime",
                allow_extended_read=True,
                silent=True,
            )

        # 直接刷新设备列表 + 一条汇总日志
        self._populate_device_list()
        self._log("success",
                  f"满配置已就绪: 13 台 0x48 设备, "
                  f"共 {total_regs} 个输入寄存器 (每台 133)")

    # ================================================================
    #  生命周期
    # ================================================================

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()

    def _on_close(self):
        if self.register_watch is not None:
            try:
                self.register_watch.window.destroy()
            except tk.TclError:
                pass
            self.register_watch = None
        if self.sim:
            self.sim.stop()
        self.root.destroy()


# ============================================================================
# 添加设备弹窗
# ============================================================================

class AddDeviceDialog:
    """添加新设备弹窗 — 选择地址和模块类型。"""

    def __init__(self, parent, sim: SimulatorThread):
        self.sim = sim
        self.result: Optional[dict] = None

        self.window = tk.Toplevel(parent)
        self.window.title("添加从机设备")
        self.window.geometry("640x640")
        self.window.resizable(True, True)
        self.window.minsize(580, 580)
        self.window.transient(parent)
        self.window.grab_set()

        self._build()
        # 居中
        self.window.update_idletasks()
        pw, ph = parent.winfo_width(), parent.winfo_height()
        ww, wh = 640, 640
        x = parent.winfo_rootx() + (pw - ww) // 2
        y = parent.winfo_rooty() + (ph - wh) // 2
        self.window.geometry(f"+{x}+{y}")

    def _build(self):
        """构建界面。"""
        main = ttk.Frame(self.window, padding=20)
        main.pack(fill=tk.BOTH, expand=True)

        # ---- 地址 ----
        ttk.Label(main, text="从机地址 (1-247):",
                  font=("Microsoft YaHei", 11, "bold")).pack(anchor=tk.W, pady=(0, 5))

        addr_frame = ttk.Frame(main)
        addr_frame.pack(fill=tk.X, pady=(0, 15))

        # 自动推荐下一个可用地址
        existing = self.sim.get_devices()
        suggested = 1
        while suggested in existing:
            suggested += 1
        if suggested > 247:
            suggested = 1  # 回退

        self.addr_var = tk.IntVar(value=suggested)
        ttk.Spinbox(addr_frame, textvariable=self.addr_var, from_=1, to=247,
                    width=8, font=("Microsoft YaHei", 11)).pack(side=tk.LEFT)
        ttk.Label(addr_frame, text=f"  (建议: {suggested}, 已占用: "
                  f"{sorted(existing.keys()) if existing else '无'})",
                  font=("Microsoft YaHei", 8), foreground="gray").pack(
            side=tk.LEFT, padx=5)

        # ---- 模块类型 ----
        ttk.Label(main, text="模块类型:",
                  font=("Microsoft YaHei", 11, "bold")).pack(anchor=tk.W, pady=(0, 8))

        type_frame = ttk.Frame(main)
        type_frame.pack(fill=tk.X, pady=(0, 10))

        type_names = [f"0x{k:02X} — {v}" for k, v in MODULE_TYPE_NAMES.items() if k != 0]
        self.type_var = tk.StringVar(value=type_names[0])

        self.type_combo = ttk.Combobox(type_frame, textvariable=self.type_var,
                                       values=type_names, state="readonly", width=42,
                                       font=("Microsoft YaHei", 10))
        self.type_combo.pack(fill=tk.X, ipady=2)
        self.type_combo.bind("<<ComboboxSelected>>", self._on_type_changed)

        # ---- 设备信息 ----
        self.info_var = tk.StringVar()
        self._update_info_text(type_names[0])
        ttk.Label(main, textvariable=self.info_var,
                  font=("Microsoft YaHei", 9), foreground="gray").pack(
            anchor=tk.W, pady=(0, 12))

        # ---- 曲线模式 ----
        self.curve_frame = ttk.LabelFrame(main, text="自定义曲线设置", padding=12)
        self.curve_frame.pack(fill=tk.X, pady=(0, 12))

        self.curve_var = tk.StringVar(value="realtime")
        ttk.Radiobutton(self.curve_frame, text="仅实时数据 (realtime)",
                        variable=self.curve_var, value="realtime").pack(anchor=tk.W, pady=3)
        ttk.Radiobutton(self.curve_frame, text="包含曲线数据 (curve)",
                        variable=self.curve_var, value="curve").pack(anchor=tk.W, pady=3)

        # 曲线帧参数
        param_row = ttk.Frame(self.curve_frame)
        param_row.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(param_row, text="每周期帧数:").pack(side=tk.LEFT)
        self.fpc_var = tk.IntVar(value=8)
        ttk.Spinbox(param_row, textvariable=self.fpc_var, from_=2, to=50,
                    width=5).pack(side=tk.LEFT, padx=(5, 20))
        ttk.Label(param_row, text="每帧点数:").pack(side=tk.LEFT)
        self.ppf_var = tk.IntVar(value=25)
        ttk.Spinbox(param_row, textvariable=self.ppf_var, from_=5, to=25,
                    width=5).pack(side=tk.LEFT, padx=5)

        # ---- 扩展读 ----
        self.ext_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(main, text="允许扩展读 (>125 寄存器)",
                        variable=self.ext_var).pack(anchor=tk.W, pady=(0, 12))

        # 初始化控件状态
        self._on_type_changed()

        # ---- 按钮 ----
        btn_frame = ttk.Frame(main)
        btn_frame.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(btn_frame, text="添加 (Add)", command=self._save).pack(
            side=tk.RIGHT, padx=(5, 0), ipadx=8, ipady=2)
        ttk.Button(btn_frame, text="取消 (Cancel)",
                   command=self.window.destroy).pack(side=tk.RIGHT, ipadx=8, ipady=2)

    def _on_type_changed(self, event=None):
        """模块类型切换时, 更新信息显示和控件状态。"""
        type_str = self.type_var.get()
        self._update_info_text(type_str)

        try:
            mtype = int(type_str.split(" ")[0].replace("0x", ""), 16)
        except (ValueError, IndexError):
            return

        profile = get_profile(mtype)

        _DC3W_TYPES = (FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION)
        # 自定义曲线模块 / 计轴 / 直流3对线 都需要使能模式选择 (realtime / curve)
        if profile.is_custom_curve or mtype in AXLE_COUNTER_TYPES or mtype in _DC3W_TYPES:
            for child in self.curve_frame.winfo_children():
                child.configure(state=tk.NORMAL)
        else:
            for child in self.curve_frame.winfo_children():
                try:
                    child.configure(state=tk.DISABLED)
                except tk.TclError:
                    pass

        self.ext_var.set(profile.exceeds_125_regs)

    def _update_info_text(self, type_str: str):
        """更新设备信息文本。"""
        try:
            mtype = int(type_str.split(" ")[0].replace("0x", ""), 16)
        except (ValueError, IndexError):
            return

        profile = get_profile(mtype)
        is_curve = "是" if profile.is_custom_curve else "否"
        is_hf = "是" if profile.is_high_frequency else "否"
        is_axle = "是" if mtype in AXLE_COUNTER_TYPES else "否"
        self.info_var.set(
            f"输入寄存器: {profile.input_regs}  |  "
            f"保持寄存器: {profile.holding_regs}  |  "
            f"自定义曲线: {is_curve}  |  "
            f"计轴模块: {is_axle}  |  "
            f"高频模块: {is_hf}"
            + (f"  |  曲线分支数: {profile.curve_branch_count}"
               if profile.is_custom_curve else "")
        )

    def _save(self):
        """保存并关闭。"""
        addr = self.addr_var.get()
        if not (1 <= addr <= 247):
            messagebox.showwarning("输入错误",
                                   "地址必须在 1-247 之间", parent=self.window)
            return

        # 检查地址冲突
        existing = self.sim.get_devices()
        if addr in existing:
            if not messagebox.askyesno(
                    "地址冲突",
                    f"地址 {addr} 已被设备 0x{existing[addr].module_type:02X} 占用。\n\n"
                    "确定将覆盖该设备吗？",
                    parent=self.window):
                return

        type_str = self.type_var.get()
        try:
            mtype = int(type_str.split(" ")[0].replace("0x", ""), 16)
        except (ValueError, IndexError):
            messagebox.showwarning("输入错误", "无法解析模块类型", parent=self.window)
            return

        self.result = {
            "address": addr,
            "module_type": mtype,
            "curve_mode": self.curve_var.get(),
            "allow_extended_read": self.ext_var.get(),
            "curve_frames_per_cycle": self.fpc_var.get(),
            "curve_points_per_frame": self.ppf_var.get(),
        }
        self.window.destroy()


# ============================================================================
# 设备编辑弹窗
# ============================================================================

class DeviceEditDialog:
    """设备参数编辑弹窗 — 修改模块类型、曲线模式、扩展读等。"""

    def __init__(self, parent, address: int, device: SlaveDevice):
        self.address = address
        self.device = device
        self.result: Optional[dict] = None

        self.window = tk.Toplevel(parent)
        self.window.title(f"编辑设备 — 地址 {address}")
        self.window.geometry("640x540")
        self.window.resizable(True, True)
        self.window.minsize(560, 480)
        self.window.transient(parent)
        self.window.grab_set()

        self._build()
        # 居中
        self.window.update_idletasks()
        pw, ph = parent.winfo_width(), parent.winfo_height()
        ww, wh = 640, 540
        x = parent.winfo_rootx() + (pw - ww) // 2
        y = parent.winfo_rooty() + (ph - wh) // 2
        self.window.geometry(f"+{x}+{y}")

    def _build(self):
        """构建编辑界面。"""
        main = ttk.Frame(self.window, padding=20)
        main.pack(fill=tk.BOTH, expand=True)

        # ---- 模块类型 ----
        ttk.Label(main, text="模块类型:",
                  font=("Microsoft YaHei", 11, "bold")).pack(anchor=tk.W, pady=(0, 8))

        type_frame = ttk.Frame(main)
        type_frame.pack(fill=tk.X, pady=(0, 10))

        current_type = f"0x{self.device.module_type:02X}"
        type_names = [f"0x{k:02X} — {v}" for k, v in MODULE_TYPE_NAMES.items() if k != 0]
        self.type_var = tk.StringVar()
        # 设置当前值
        for item in type_names:
            if item.startswith(current_type):
                self.type_var.set(item)
                break
        if not self.type_var.get():
            self.type_var.set(type_names[0])

        self.type_combo = ttk.Combobox(type_frame, textvariable=self.type_var,
                                       values=type_names, state="readonly", width=42,
                                       font=("Microsoft YaHei", 10))
        self.type_combo.pack(fill=tk.X, ipady=2)
        self.type_combo.bind("<<ComboboxSelected>>", self._on_type_changed)

        # ---- 设备信息 ----
        self.info_var = tk.StringVar()
        new_profile = get_profile(self.device.module_type)
        self._update_info(new_profile)
        ttk.Label(main, textvariable=self.info_var,
                  font=("Microsoft YaHei", 9), foreground="gray").pack(
            anchor=tk.W, pady=(0, 12))

        # ---- 曲线模式 ----
        self.curve_frame = ttk.LabelFrame(main, text="自定义曲线设置", padding=12)
        self.curve_frame.pack(fill=tk.X, pady=(0, 12))

        self.curve_var = tk.StringVar(
            value=self.device.curve_mode if self.device.is_curve_capable else "realtime")
        ttk.Radiobutton(self.curve_frame, text="仅实时数据 (realtime)",
                        variable=self.curve_var, value="realtime").pack(anchor=tk.W, pady=3)
        ttk.Radiobutton(self.curve_frame, text="包含曲线数据 (curve)",
                        variable=self.curve_var, value="curve").pack(anchor=tk.W, pady=3)

        # 曲线帧参数
        param_row = ttk.Frame(self.curve_frame)
        param_row.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(param_row, text="每周期帧数:").pack(side=tk.LEFT)
        self.fpc_var = tk.IntVar(value=8)
        ttk.Spinbox(param_row, textvariable=self.fpc_var, from_=2, to=50,
                    width=5).pack(side=tk.LEFT, padx=(5, 20))
        ttk.Label(param_row, text="每帧点数:").pack(side=tk.LEFT)
        self.ppf_var = tk.IntVar(value=25)
        ttk.Spinbox(param_row, textvariable=self.ppf_var, from_=5, to=25,
                    width=5).pack(side=tk.LEFT, padx=5)

        # ---- 扩展读 ----
        self.ext_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(main, text="允许扩展读 (>125 寄存器)",
                        variable=self.ext_var).pack(anchor=tk.W, pady=(0, 12))

        # 刷新控件状态
        self._on_type_changed()

        # ---- 按钮 ----
        btn_frame = ttk.Frame(main)
        btn_frame.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(btn_frame, text="保存 (Save)", command=self._save).pack(
            side=tk.RIGHT, padx=(5, 0), ipadx=8, ipady=2)
        ttk.Button(btn_frame, text="取消 (Cancel)",
                   command=self.window.destroy).pack(side=tk.RIGHT, ipadx=8, ipady=2)

    def _on_type_changed(self, event=None):
        """模块类型切换时, 更新信息显示和控件状态。"""
        type_str = self.type_var.get()
        try:
            mtype = int(type_str.split(" ")[0].replace("0x", ""), 16)
        except (ValueError, IndexError):
            return

        profile = get_profile(mtype)
        self._update_info(profile)

        # 曲线模块 / 计轴 / 直流3对线 才显示曲线/脉冲设置
        if profile.is_custom_curve or mtype in AXLE_COUNTER_TYPES or mtype in (
                FLFXG_DC3WIRE_PLATFORM, FLFXG_DC3WIRE_STATION):
            for child in self.curve_frame.winfo_children():
                child.configure(state=tk.NORMAL)
            if self.curve_var.get() not in ("realtime", "curve"):
                self.curve_var.set("realtime")
        else:
            for child in self.curve_frame.winfo_children():
                try:
                    child.configure(state=tk.DISABLED)
                except tk.TclError:
                    pass

        # 高频模块才显示扩展读
        self.ext_var.set(profile.exceeds_125_regs)

    def _update_info(self, profile):
        """更新设备信息文本。"""
        is_curve = "是" if profile.is_custom_curve else "否"
        is_hf = "是" if profile.is_high_frequency else "否"
        self.info_var.set(
            f"输入寄存器: {profile.input_regs}  |  "
            f"保持寄存器: {profile.holding_regs}  |  "
            f"自定义曲线: {is_curve}  |  "
            f"高频模块: {is_hf}"
            + (f"  |  曲线分支数: {profile.curve_branch_count}"
               if profile.is_custom_curve else "")
        )

    def _save(self):
        """保存并关闭。"""
        type_str = self.type_var.get()
        try:
            mtype = int(type_str.split(" ")[0].replace("0x", ""), 16)
        except (ValueError, IndexError):
            messagebox.showwarning("输入错误", "无法解析模块类型", parent=self.window)
            return

        self.result = {
            "module_type": mtype,
            "curve_mode": self.curve_var.get(),
            "allow_extended_read": self.ext_var.get(),
            "curve_frames_per_cycle": self.fpc_var.get(),
            "curve_points_per_frame": self.ppf_var.get(),
        }
        self.window.destroy()


# ============================================================================
# 寄存器监视弹窗
# ============================================================================

class RegisterWatchWindow:
    """寄存器实时监视器 — 独立弹窗, 实时刷新寄存器值。

    显示:
      - 前 N 个输入寄存器 (hex + 十进制)
      - 上次请求/响应原始帧
      - 刷新率切换 (手动/自动 1s)
    """

    def __init__(self, parent, address: int, device: SlaveDevice,
                 sim: SimulatorThread, main_gui: SlaveSimulatorGUI):
        self.address = address
        self.device = device
        self.sim = sim
        self.main_gui = main_gui
        self.paused = False
        self.auto_refresh = True
        self._refresh_job: Optional[str] = None

        self.window = tk.Toplevel(parent)
        self.window.title(
            f"设备 {address} (0x{device.module_type:02X} "
            f"{get_module_name(device.module_type)}) — 寄存器监视")
        self.window.geometry("600x420")
        self.window.minsize(320, 280)
        self.window.transient(parent)
        self.window.protocol("WM_DELETE_WINDOW", self._on_close)

        self._last_req: Optional[bytes] = None
        self._last_resp: Optional[bytes] = None
        self._reg_entries: list = []

        self._build_ui()
        self._do_refresh()
        self._start_auto_refresh()

    def _build_ui(self):
        """构建监视器界面。"""
        # ---- 工具栏 ----
        toolbar = ttk.Frame(self.window, relief=tk.RIDGE, borderwidth=1)
        toolbar.pack(fill=tk.X, padx=5, pady=(5, 0))

        title = ttk.Label(toolbar,
                          text=f"设备 {self.address}  |  "
                               f"0x{self.device.module_type:02X} "
                               f"{get_module_name(self.device.module_type)}  |  "
                               f"寄存器数: {self.device.profile.input_regs}",
                          font=("Microsoft YaHei", 9, "bold"))
        title.pack(side=tk.LEFT, padx=10)

        self.pause_btn = ttk.Button(toolbar, text="暂停", width=6,
                                    command=self._toggle_pause)
        self.pause_btn.pack(side=tk.RIGHT, padx=10)

        ttk.Button(toolbar, text="手动刷新", width=8,
                   command=self._do_refresh).pack(side=tk.RIGHT, padx=5)

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(
            side=tk.RIGHT, fill=tk.Y, padx=5)

        ttk.Button(toolbar, text="清除覆写", width=8,
                   command=self._clear_all_overrides).pack(
            side=tk.RIGHT, padx=5)

        # ---- 主体 ----
        main_frame = ttk.Frame(self.window)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 上方: 寄存器网格
        reg_frame = ttk.LabelFrame(main_frame, text="输入寄存器值", padding=5)
        reg_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 5))

        self.reg_canvas = tk.Canvas(reg_frame, highlightthickness=0)
        reg_scroll = ttk.Scrollbar(reg_frame, orient=tk.VERTICAL,
                                   command=self.reg_canvas.yview)
        self.reg_inner = ttk.Frame(self.reg_canvas)
        self.reg_inner.bind("<Configure>",
            lambda e: self.reg_canvas.configure(
                scrollregion=self.reg_canvas.bbox("all")))
        self.reg_canvas.create_window((0, 0), window=self.reg_inner,
                                      anchor=tk.NW)
        self.reg_canvas.configure(yscrollcommand=reg_scroll.set)

        self.reg_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        reg_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 鼠标滚轮
        def _mousewheel(event):
            self.reg_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        self.reg_canvas.bind("<Enter>",
            lambda e: self.reg_canvas.bind_all("<MouseWheel>", _mousewheel))
        self.reg_canvas.bind("<Leave>",
            lambda e: self.reg_canvas.unbind_all("<MouseWheel>"))

        # 构建寄存器标签
        self._build_reg_grid()

        # 下方: 原始帧
        raw_frame = ttk.LabelFrame(main_frame, text="最近一次通信", padding=5)
        raw_frame.pack(fill=tk.X)

        req_frame = ttk.Frame(raw_frame)
        req_frame.pack(fill=tk.X, pady=1)
        ttk.Label(req_frame, text="请求:", width=6, anchor=tk.E,
                  font=("Consolas", 9, "bold")).pack(side=tk.LEFT)
        self.req_hex_label = ttk.Label(req_frame, text="(等待数据...)",
                                       font=("Consolas", 9), foreground="gray")
        self.req_hex_label.pack(side=tk.LEFT, padx=(5, 0))

        resp_frame = ttk.Frame(raw_frame)
        resp_frame.pack(fill=tk.X, pady=1)
        ttk.Label(resp_frame, text="响应:", width=6, anchor=tk.E,
                  font=("Consolas", 9, "bold")).pack(side=tk.LEFT)
        self.resp_hex_label = ttk.Label(resp_frame, text="(等待数据...)",
                                        font=("Consolas", 9), foreground="gray")
        self.resp_hex_label.pack(side=tk.LEFT, padx=(5, 0))

    def _build_reg_grid(self, groups_per_row: int | None = None):
        """创建可编辑的寄存器网格 (只显示十进制)。

        布局随窗口宽度自适应: 窄窗口 → 每行 2-3 组, 宽窗口 → 6-8 组。
        每组: 序号标签 + 十进制输入框。

        直接点击输入框即可编辑，按 Enter 或移开焦点即生效。
        手动修改过的寄存器用浅蓝色背景标记。
        """
        # 清除旧 widgets
        for w in self.reg_inner.winfo_children():
            w.destroy()
        self._reg_entries.clear()

        max_show = min(self.device.profile.input_regs, 64)

        # 自适应列数: 每组约 95px (标签 35 + 输入框 55 + 间距 5)
        if groups_per_row is None:
            self.reg_inner.update_idletasks()
            width = self.reg_canvas.winfo_width()
            groups_per_row = max(2, min(8, (width - 5) // 100)) if width > 10 else 4

        cols_per_group = 2  # R## label + dec entry

        for r in range(max_show):
            row = r // groups_per_row
            col_base = (r % groups_per_row) * cols_per_group

            # 寄存器序号
            idx_label = ttk.Label(self.reg_inner,
                                  text=f"R{r:02d}", width=5, anchor=tk.E,
                                  font=("Consolas", 9), foreground="gray")
            idx_label.grid(row=row, column=col_base, sticky=tk.E, padx=1, pady=1)

            # 十进制输入框
            dec_var = tk.StringVar(value="0")
            dec_entry = tk.Entry(self.reg_inner, textvariable=dec_var,
                                 width=6, font=("Consolas", 9),
                                 justify=tk.RIGHT,
                                 relief=tk.FLAT, borderwidth=1,
                                 bg="SystemWindow",
                                 insertbackground="black")
            dec_entry.grid(row=row, column=col_base + 1, sticky=tk.E, padx=(1, 6), pady=1)
            dec_entry.bind("<Return>", lambda e, ri=r: self._commit_edit(ri))
            dec_entry.bind("<FocusOut>", lambda e, ri=r: self._commit_edit(ri))
            # 右键清除覆写
            dec_entry.bind("<Button-3>",
                           lambda e, ri=r: self._clear_override_popup(ri, e))

            self._reg_entries.append({
                "reg": r,
                "idx_label": idx_label,
                "dec_var": dec_var,
                "dec_entry": dec_entry,
            })

        # 绑定窗口大小变化 → 重排布局
        self.reg_canvas.bind("<Configure>", self._on_reg_canvas_resize, add="+")

    def _on_reg_canvas_resize(self, event=None):
        """窗口宽度变化时重新计算列数并重排寄存器网格。"""
        width = self.reg_canvas.winfo_width()
        if width < 50:
            return
        # 每组约 95px: 标签 35 + 输入框 55 + 间距 5
        groups_per_row = max(2, min(8, (width - 5) // 100))
        # 只有当列数实际变化时才重建 (避免频繁重建造成闪烁)
        current_rows = (len(self._reg_entries) + groups_per_row - 1) // groups_per_row if self._reg_entries else 0
        # 用 grid_slaves 推测当前列数: 如果变化超过阈值则重建
        if not hasattr(self, '_last_groups_per_row'):
            self._last_groups_per_row = 0
        if groups_per_row != self._last_groups_per_row:
            self._last_groups_per_row = groups_per_row
            self._build_reg_grid(groups_per_row)

    def _commit_edit(self, reg_index: int):
        """将用户编辑的十进制寄存器值提交到模拟器。"""
        entry_info = self._reg_entries[reg_index]
        try:
            raw = entry_info["dec_var"].get().strip()
            value = int(raw)
        except (ValueError, AttributeError):
            self._sync_one_entry(reg_index)
            return

        value = max(0, min(65535, value))
        self.sim.set_device_register(self.address, reg_index, value)

        # 同步显示
        entry_info["dec_var"].set(str(value))

        # 标记为已覆写
        self._mark_overridden(reg_index)

    def _clear_override_popup(self, reg_index: int, event):
        """右键菜单: 清除手动覆写。"""
        dev = self.sim.get_device(self.address)
        if dev is None or not dev.has_register_override(reg_index):
            return

        menu = tk.Menu(self.window, tearoff=0)
        menu.add_command(
            label=f"清除 R{reg_index:02d} 的手动覆写",
            command=lambda: self._clear_override(reg_index))
        menu.post(event.x_root, event.y_root)

    def _clear_override(self, reg_index: int):
        """清除单个寄存器的手动覆写。"""
        self.sim.clear_device_register(self.address, reg_index)
        self._unmark_overridden(reg_index)
        self._sync_one_entry(reg_index)

    def _clear_all_overrides(self):
        """清除当前设备所有手动覆写。"""
        dev = self.sim.get_device(self.address)
        if dev is None:
            return
        for info in self._reg_entries:
            ri = info["reg"]
            if dev.has_register_override(ri):
                self.sim.clear_device_register(self.address, ri)
                self._unmark_overridden(ri)
        self._do_refresh()

    def _mark_overridden(self, reg_index: int):
        """用浅蓝色背景标记手动覆写的寄存器。"""
        info = self._reg_entries[reg_index]
        color = "#C8E6FF"
        info["dec_entry"].configure(bg=color, readonlybackground=color)

    def _unmark_overridden(self, reg_index: int):
        """恢复默认背景色。"""
        info = self._reg_entries[reg_index]
        default = "SystemWindow"
        info["dec_entry"].configure(bg=default, readonlybackground=default)

    def _sync_one_entry(self, reg_index: int):
        """同步单个寄存器的显示值 (从模拟器读取)。"""
        values = self.sim.get_device_registers(self.address, reg_index + 1)
        if reg_index < len(values):
            v = values[reg_index]
            info = self._reg_entries[reg_index]
            info["dec_var"].set(str(v))

    # ================================================================
    #  刷新
    # ================================================================

    def _do_refresh(self):
        """执行一次寄存器刷新 (不覆盖正在编辑的输入框)。"""
        if self.paused:
            return

        values = self.sim.get_device_registers(
            self.address, len(self._reg_entries))

        dev = self.sim.get_device(self.address)

        for i, info in enumerate(self._reg_entries):
            if i < len(values):
                v = values[i]
                # 只更新当前焦点不在的输入框, 避免打断用户编辑
                focused = self.window.focus_get()
                if info["dec_entry"] != focused:
                    info["dec_var"].set(str(v))
                # 更新覆写标记
                if dev and dev.has_register_override(i):
                    self._mark_overridden(i)
                else:
                    self._unmark_overridden(i)

        # 更新原始帧
        dev = self.sim.get_device(self.address)
        if dev:
            req = dev.last_request
            resp = dev.last_response
            if req:
                self.req_hex_label.config(
                    text=f"({len(req)}B)  {req.hex(' ')}",
                    foreground="#D4D4D4" if self._use_dark_theme() else "black")
            if resp:
                self.resp_hex_label.config(
                    text=f"({len(resp)}B)  {resp.hex(' ')}",
                    foreground="#D4D4D4" if self._use_dark_theme() else "black")

    def _start_auto_refresh(self):
        """启动自动刷新定时器。"""
        if self._refresh_job is not None:
            return
        self._schedule_refresh()

    def _schedule_refresh(self):
        """单次刷新调度。"""
        if not self.auto_refresh or self.paused:
            self._refresh_job = None
            return
        self._do_refresh()
        self._refresh_job = self.window.after(500, self._schedule_refresh)

    def _toggle_pause(self):
        """暂停/继续。"""
        self.paused = not self.paused
        self.pause_btn.config(text="继续" if self.paused else "暂停")
        if not self.paused:
            self._start_auto_refresh()

    # ================================================================
    #  外部触发更新
    # ================================================================

    def handle_traffic(self, msg: dict):
        """当有新的通信数据时触发 (来自主窗口事件)。"""
        if msg["address"] != self.address:
            return
        self._last_req = msg["request"]
        self._last_resp = msg["response"]
        # 不在此处刷新寄存器, 由自动刷新定时器处理

    @staticmethod
    def _use_dark_theme() -> bool:
        """简单检测是否深色主题 (始终返回 True, 因为我们用深色日志)。"""
        return True

    def _on_close(self):
        """关闭窗口时清理引用。"""
        self.auto_refresh = False
        if self._refresh_job:
            self.window.after_cancel(self._refresh_job)
            self._refresh_job = None
        if self.main_gui is not None:
            self.main_gui.register_watch = None
        self.window.destroy()


# ============================================================================
# 入口
# ============================================================================

def main(config_path: str = CONFIG_FILE):
    """GUI 程序入口。"""
    # Windows 高分屏适配
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = SlaveSimulatorGUI(config_path)
    app.run()


if __name__ == "__main__":
    main()
