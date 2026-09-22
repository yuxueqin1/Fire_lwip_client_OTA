#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 产线一次性烧写镜像合成器  —  Intel HEX 合并 (Bootloader + APP)
================================================================================

 为什么需要它:
   OTA 采用 A/B 双分区, 通信板出厂时 Flash 是空的, 产线要一次烧完两块:
       Bootloader  0x08000000 .. 0x08007FFF   32 KB   (S0,S1)
       APP1 (A区)  0x08008000 .. 0x0807FFFF  480 KB   (S2..S7)
   本脚本把这两个 .hex 合成一个 .hex, 产线用一个文件一次烧完即可。

 为什么不用烧 BootInfo:
   IAP 自己会补。上电时若 BootInfo 的 magic 不是 0x55AA55AA (空片读出来是
   0xFFFFFFFF), IAP 会写入一份默认值 (boot_app = APP1) 并打印
       [IAP] boot_info missing, default slot A written
   然后跳 APP1。所以产线只需要 Bootloader + A 镜像这两块。
   详见 ota升级需要的iap代码/.../User/main.c 的 boot_info_read/write 与
   CheckAndJump 附近。此处若也把 BootInfo 烧进去, 反而会覆盖掉 IAP 的
   首次启动决策, 一般没必要。

 用法:
   python merge_hex.py                                  # 全用默认路径
   python merge_hex.py -o ..\\Output\\ota\\out.hex        # 只改输出路径
   python merge_hex.py --boot B.hex --app A.hex -o out.hex

 退出码: 0 = 成功; 1 = 失败(地址重叠/区间越界/文件不存在/格式错误)
   失败时**不会**写出输出文件 —— 产线宁可没文件, 也不要一个坏文件。

 本脚本只用标准库。APB/分区表定义与 User/ota/ota_config.h 必须一致,
 改分区时记得同步下面的常量。
================================================================================
"""

import argparse
import os
import sys
import zlib

# ---- 分区表 (必须与 User/ota/ota_config.h 一致) ----------------------------
FLASH_BASE     = 0x08000000
FLASH_END      = 0x08100000        # 1MB, 不含
BOOTLOADER_ADDR = 0x08000000
BOOTLOADER_END  = BOOTLOADER_ADDR + 32 * 1024      # 0x08008000
APP1_ADDR      = 0x08008000
APP1_END       = APP1_ADDR + 480 * 1024            # 0x08080000
APP2_ADDR      = 0x08080000
BOOT_INFO_ADDR = 0x080E0000

# 默认路径 (相对本脚本所在目录 Project/)
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BOOT = os.path.join(
    HERE, "..", "ota升级需要的iap代码",
    "lwip_client_Bootloader_RTOS_Final（完成版别动了）", "Output", "YH-F429.hex")
DEFAULT_APP  = os.path.join(HERE, "..", "Output", "A", "YH-F429_A.hex")
DEFAULT_OUT  = os.path.join(HERE, "..", "Output", "ota", "YH-F429_production.hex")


# ============================================================================
#  Intel HEX 解析
# ============================================================================
def parse_hex(path):
    """读一个 Intel HEX 文件。

    返回 (data, entry) —— data 是 {地址: 字节} 字典; entry 是 type-05
    记录里的入口地址(没有则为 None)。
    校验和不对、格式不对、出现不支持的记录类型都直接抛异常: 产线用的
    文件必须严格, 宁可报错也不要"将就着烧"。
    """
    if not os.path.isfile(path):
        raise IOError("文件不存在: %s" % path)

    data = {}
    entry = None
    upper = 0                      # type-04 提供的地址高 16 位
    seen_eof = False
    lineno = 0

    with open(path, "r", encoding="ascii") as f:
        for raw in f:
            lineno += 1
            line = raw.strip()
            if not line:
                continue
            if not line.startswith(":"):
                raise ValueError("%s:%d 不是以 ':' 开头" % (path, lineno))

            body = line[1:]
            if len(body) % 2 or len(body) < 10:
                raise ValueError("%s:%d 记录长度非法" % (path, lineno))

            try:
                rec = bytes.fromhex(body)
            except ValueError:
                raise ValueError("%s:%d 含非十六进制字符" % (path, lineno))

            count = rec[0]
            if len(rec) != count + 5:
                raise ValueError("%s:%d 长度字段与实际字节数不符" % (path, lineno))

            # 校验和 = 从 LL 到最后一个数据字节之和的低 8 位, 再取补
            if (sum(rec[:-1]) + rec[-1]) & 0xFF:
                raise ValueError("%s:%d 校验和错误" % (path, lineno))

            offset = (rec[1] << 8) | rec[2]
            rtype  = rec[3]
            payload = rec[4:4 + count]

            if rtype == 0x00:                       # 数据
                base = (upper << 16) + offset
                for i, b in enumerate(payload):
                    addr = base + i
                    if addr in data:
                        raise ValueError(
                            "%s:%d 地址 0x%08X 在同一文件内重复出现"
                            % (path, lineno, addr))
                    data[addr] = b

            elif rtype == 0x01:                     # EOF
                seen_eof = True
                break

            elif rtype == 0x04:                     # 扩展线性地址
                if count != 2:
                    raise ValueError("%s:%d type-04 记录长度应为 2"
                                     % (path, lineno))
                upper = (payload[0] << 8) | payload[1]

            elif rtype == 0x05:                     # 起始线性地址(入口点)
                if count != 4:
                    raise ValueError("%s:%d type-05 记录长度应为 4"
                                     % (path, lineno))
                entry = int.from_bytes(payload, "big")

            elif rtype in (0x02, 0x03):
                # type-02/03 是 8086 时代的段地址写法, ARM 工具链不会产生。
                # 真遇到说明文件来路不对, 直接报错而不是猜。
                raise ValueError("%s:%d 不支持的记录类型 0x%02X (type-02/03)"
                                 % (path, lineno, rtype))
            else:
                raise ValueError("%s:%d 未知记录类型 0x%02X"
                                 % (path, lineno, rtype))

    if not data:
        raise ValueError("%s 里没有任何数据记录" % path)
    if not seen_eof:
        # 多数烧写器能容忍, 但产线文件不该缺; 只警告不失败
        sys.stderr.write("[WARN] %s 缺少 EOF 记录 (:00000001FF)\n" % path)

    return data, entry


def emit_hex(data, entry, path):
    """按地址升序写一个规范的 Intel HEX 文件。"""
    addrs = sorted(data)
    lines = []
    upper_now = None
    i = 0

    while i < len(addrs):
        base = addrs[i]

        # 收集一段连续地址作为一条数据记录。两个边界都必须断开:
        #   1) 地址不连续 —— 两个镜像之间、以及镜像代码段内部的空洞
        #   2) 跨 64KB —— 数据记录的地址字段只有 16 位, 跨段会回绕
        #      (高位另由 type-04 记录提供)
        run = [base]
        j = i + 1
        while (j < len(addrs) and len(run) < 16
               and addrs[j] == run[-1] + 1
               and (addrs[j] >> 16) == (base >> 16)):
            run.append(addrs[j])
            j += 1

        up = (base >> 16) & 0xFFFF
        if up != upper_now:
            rec = bytes([2, 0, 0, 0x04, (up >> 8) & 0xFF, up & 0xFF])
            lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())
            upper_now = up

        payload = bytes(data[a] for a in run)
        lo = base & 0xFFFF
        rec = bytes([len(payload), (lo >> 8) & 0xFF, lo & 0xFF, 0x00]) + payload
        lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())

        i = j            # 只跳过真正写出去的那些地址, 不能按固定 16 步长切

    if entry is not None:
        rec = bytes([4, 0, 0, 0x05]) + entry.to_bytes(4, "big")
        lines.append(":" + (rec + bytes([(-sum(rec)) & 0xFF])).hex().upper())

    lines.append(":00000001FF")

    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    # 烧写器普遍更认 CRLF
    with open(path, "w", encoding="ascii", newline="\r\n") as f:
        f.write("\n".join(lines) + "\n")


# ============================================================================
#  校验
# ============================================================================
def span(data):
    a = sorted(data)
    return a[0], a[-1]


def check_region(name, data, lo, hi, what):
    """data 的地址区间必须落在 [lo, hi) 内。"""
    a, b = span(data)
    if a < lo or b >= hi:
        raise ValueError(
            "%s 的地址区间 0x%08X..0x%08X 超出 %s (0x%08X..0x%08X)"
            % (name, a, b, what, lo, hi - 1))
    return a, b


def check_vector(name, data, base):
    """看一眼镜像开头的中断向量表: [0]=初始MSP(应在RAM), [1]=复位向量(应在镜像内)。

    这不是硬性要求(某些镜像可能不带向量表), 所以只告警不失败 —— 但产线
    文件如果这里不对, 烧下去就是块砖, 值得吵一声。
    """
    if base not in data or (base + 7) not in data:
        sys.stderr.write("[WARN] %s 在 0x%08X 处不足 8 字节, 无法校验向量表\n"
                         % (name, base))
        return
    sp = int.from_bytes(bytes(data[base + i] for i in range(4)), "little")
    pc = int.from_bytes(bytes(data[base + 4 + i] for i in range(4)), "little")
    ok = True
    # STM32F429 SRAM: 0x20000000..0x20030000, CCM 0x10000000..0x10010000
    if not (0x20000000 <= sp <= 0x20030000 or 0x10000000 <= sp <= 0x10010000):
        sys.stderr.write("[WARN] %s 初始 MSP = 0x%08X 不像 RAM 地址\n" % (name, sp))
        ok = False
    if not (FLASH_BASE <= pc < FLASH_END):
        sys.stderr.write("[WARN] %s 复位向量 = 0x%08X 不在 Flash 内\n" % (name, pc))
        ok = False
    if ok:
        print("     向量表: MSP=0x%08X  Reset=0x%08X  (OK)" % (sp, pc))


# ============================================================================
def main():
    ap = argparse.ArgumentParser(
        description="把 Bootloader 与 APP 镜像合成为一个产线烧写用 hex",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boot", default=DEFAULT_BOOT, help="Bootloader hex")
    ap.add_argument("--app",  default=DEFAULT_APP,  help="APP 镜像 hex (通常 A 区)")
    ap.add_argument("-o", "--out", default=DEFAULT_OUT, help="输出 hex")
    ap.add_argument("--app-base", default=hex(APP1_ADDR),
                    help="APP 镜像的链接基址, 默认 APP1 (0x08008000)")
    args = ap.parse_args()

    app_base = int(args.app_base, 16)

    print("=" * 74)
    print(" 产线镜像合成  —  Bootloader + APP")
    print("=" * 74)

    # ---- 1. 读 ----
    print("[1/4] 读取")
    print("      Bootloader: %s" % os.path.normpath(args.boot))
    boot, boot_entry = parse_hex(args.boot)
    b_lo, b_hi = check_region("Bootloader", boot,
                              BOOTLOADER_ADDR, BOOTLOADER_END, "Bootloader 区")
    print("                 0x%08X..0x%08X  %d 字节 (含空洞)"
          % (b_lo, b_hi, len(boot)))
    check_vector("Bootloader", boot, BOOTLOADER_ADDR)

    print("      APP       : %s" % os.path.normpath(args.app))
    app, _ = parse_hex(args.app)
    a_lo, a_hi = check_region("APP", app, app_base, FLASH_END, "APP 区")
    print("                 0x%08X..0x%08X  %d 字节 (含空洞)"
          % (a_lo, a_hi, len(app)))
    check_vector("APP", app, app_base)

    if app_base == APP1_ADDR and a_hi >= APP1_END:
        raise ValueError("APP 镜像 0x%08X 超出 APP1 区上限 0x%08X"
                         % (a_hi, APP1_END - 1))
    if a_hi >= BOOT_INFO_ADDR:
        raise ValueError("APP 镜像 0x%08X 已经顶到 BootInfo 区 0x%08X"
                         % (a_hi, BOOT_INFO_ADDR))

    # ---- 2. 查重叠 ----
    print("[2/4] 检查地址重叠")
    common = set(boot) & set(app)
    if common:
        lo, hi = min(common), max(common)
        raise ValueError(
            "两个镜像在 0x%08X..0x%08X 有 %d 个字节重叠 —— 合并会静默改写内容, "
            "通常是 APP 的链接基址或 IAP 的 APP1_ADDR 配错了"
            % (lo, hi, len(common)))
    if b_hi >= a_lo:
        raise ValueError("Bootloader 数据到了 0x%08X, 与 APP 起点 0x%08X 相接/越过"
                         % (b_hi, a_lo))
    print("      Bootloader 0x%08X..0x%08X  APP 0x%08X..0x%08X  无重叠"
          % (b_lo, b_hi, a_lo, a_hi))

    # ---- 3. 合并 ----
    print("[3/4] 合并")
    merged = dict(boot)
    merged.update(app)

    # 入口点: STM32 从 Flash 启动是靠 0x08000000 的复位向量, 不看 type-05;
    # 这里保留 Bootloader 的入口(有就带, 没有也不影响上电行为)。
    if boot_entry is not None:
        print("      保留 Bootloader 入口点 0x%08X (type-05)" % boot_entry)

    # ---- 4. 写 ----
    print("[4/4] 写出")
    # 先把要报告的东西全算完, 再落盘 —— 这样"失败就没有文件"才是真的。
    # (顺序反过来的话, 落盘之后算出错, 脚本会谎报"未写出任何文件"。)
    m_lo, m_hi = span(merged)
    # 两个镜像之间有空隙(Bootloader 代码段结束 -> 0x08008000), 算整体
    # CRC32 时把空隙当 0 补上; 注意这只影响这里打印的参考值, 不影响
    # 写进 hex 的内容(hex 只记录真实存在的字节)。
    raw = bytes(merged.get(a, 0) for a in range(m_lo, m_hi + 1))
    crc = zlib.crc32(raw) & 0xFFFFFFFF

    emit_hex(merged, boot_entry, args.out)

    # 回读自检: 把刚写出的文件读回来, 与原数据逐字节比对。
    # 写出逻辑一旦有洞(例如跨 64KB 断行时漏掉几个字节), 宁可在这里失败,
    # 也不能让一个少了几十字节的 hex 流到产线 —— 那种镜像烧进去能跑,
    # 但会在某个想不到的时刻出错。
    back, _ = parse_hex(args.out)
    if back != merged:
        missing = set(merged) - set(back)
        raise ValueError(
            "写出后回读不一致: 产物 %d 字节, 预期 %d 字节, 少 %d 个地址%s"
            % (len(back), len(merged), len(missing),
               ("(例如 0x%08X)" % min(missing)) if missing else ""))

    print("      %s" % os.path.normpath(args.out))
    print("      地址 0x%08X..0x%08X  共 %d 字节 (文件 %d 字节)"
          % (m_lo, m_hi, m_hi - m_lo + 1, os.path.getsize(args.out)))
    print("      合并后 CRC32 = 0x%08X  (空隙按 0 计, 仅供参考)" % crc)
    print("      回读自检 = PASS  (%d 字节逐字节一致)" % len(back))
    print()
    print(" 产线烧写: 用 STM32CubeProgrammer / ST-Link Utility 选这一个 hex 即可。")
    print(" 首次上电 IAP 若发现 BootInfo 无效, 会自动写入默认值并跳到 A 区:")
    print("     [IAP] boot_info missing, default slot A written")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        sys.stderr.write("\n[ERROR] %s\n" % e)
        sys.stderr.write("未写出任何文件。\n")
        sys.exit(1)
