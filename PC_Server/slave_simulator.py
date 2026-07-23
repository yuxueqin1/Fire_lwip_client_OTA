#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
  Modbus 从机模拟器  —  PC-Side Slave Simulator Launcher
================================================================================

 功能概述:
   - 通过 RS485 串口模拟全部 14 种 Modbus 模块类型
   - 标准 Modbus (0x31-0x36, 0x47-0x4A) 用功能码 0x04
   - 自定义曲线协议 (0x37-0x39, 0x4B-0x4D) 用 8 字节请求帧
   - tkinter GUI 界面操作 (串口配置/设备列表/寄存器监视/通信日志)
   - 配置通过 config.json 管理

 使用方式:
   python slave_simulator.py                 # 使用默认配置启动 GUI
   python slave_simulator.py --cli           # 使用终端交互模式

 与 board_server.py 配合:
   本程序模拟下位机(从机), board_server.py 作为上位机(服务器),
   两者通过物理 RS485 总线 + 通信板中转 构成完整测试链路.

================================================================================
"""

import os
import sys

# ── Windows 高精度定时器 (提升串口响应速度) ──
if sys.platform == 'win32':
    try:
        import ctypes
        _winmm = ctypes.WinDLL('winmm')
        _winmm.timeBeginPeriod(1)
    except Exception:
        pass

# 将 modbus_slave_simulator 包目录加入 sys.path
_pkg_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "modbus_slave_simulator")
sys.path.insert(0, _pkg_dir)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Modbus 从机模拟器 (GUI 模式)")
    parser.add_argument("--cli", action="store_true",
                        help="使用终端交互模式 (默认 GUI)")
    parser.add_argument("-c", "--config", default="config.json",
                        help="配置文件路径")
    args = parser.parse_args()

    if args.cli:
        from main import main
        main()
    else:
        from gui import main
        main(args.config)
