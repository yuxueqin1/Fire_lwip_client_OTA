#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通信板 UDP 调试信息接收工具

背景:
    通信板的调试串口 (USART6) 与 RS485 第5路共用外设, 现场常常不方便开;
    因此板子在 User/task_rs485_7.c 里把 [RS485-7] 的运行信息以 UDP 报文
    发到固定目标 (DBG_NET_IP : DBG_NET_PORT, 默认 192.168.1.100:39999)。
    注意: 6000/6001 在本机被 NI 服务 (nidmsrv/lktsrv) 占用, 所以选了 39999。

用法:
    python udp_debug_listen.py                 # 监听 0.0.0.0:39999
    python udp_debug_listen.py -p 39999        # 指定端口
    python udp_debug_listen.py --log run.txt   # 同时存文件

说明:
    - 只依赖标准库, Python 3.6+ 即可。
    - 板子是盲发 (UDP 不等应答), 只要本工具开着并绑对端口就能收到。
    - Windows 首次运行若弹防火墙询问, 选"允许"。要改板子侧的目标 IP/端口,
      见 User/task_rs485_7.c 顶部的 DBG_NET_IP / DBG_NET_PORT。
"""

import argparse
import socket
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Receive the comm board's UDP debug output.")
    parser.add_argument("-b", "--bind", default="0.0.0.0",
                        help="local address to bind (default: 0.0.0.0)")
    parser.add_argument("-p", "--port", type=int, default=39999,
                        help="UDP port to listen on (default: 39999)")
    parser.add_argument("--log", default=None,
                        help="also append received lines to this file")
    args = parser.parse_args()

    log_fp = None
    if args.log:
        try:
            log_fp = open(args.log, "a", encoding="utf-8")
        except OSError as e:
            print("cannot open log file %s: %s" % (args.log, e))
            return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 允许端口复用, 方便工具重启时不至于被 TIME_WAIT 之类挡住 (UDP 一般用不上,
    # 但加上无害)。
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        print("bind %s:%d failed: %s" % (args.bind, args.port, e))
        print("(port in use? firewall? or wrong --port)")
        return 1

    print("listening UDP %s:%d ... (Ctrl+C to quit)" % (args.bind, args.port))
    if log_fp:
        print("also logging to %s" % args.log)
    sys.stdout.flush()

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            text = data.decode("utf-8", errors="replace").rstrip("\r\n")
            line = "%s  %-15s  %s" % (
                time.strftime("%H:%M:%S"), addr[0], text)
            print(line)
            sys.stdout.flush()
            if log_fp:
                # 日志里带上日期, 便于事后翻查
                log_fp.write("%s  %-15s  %s\n" % (
                    time.strftime("%Y-%m-%d %H:%M:%S"), addr[0], text))
                log_fp.flush()
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        sock.close()
        if log_fp:
            log_fp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
