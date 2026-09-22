# -*- coding: utf-8 -*-
"""逐行转写 User/ota/ota_net.c 的 parse_frame()/ota_net_next_frame(),
用同一组边界用例验证设备侧接收缓冲的重组与重同步逻辑。

(本机没有 C 编译器, 所以用转写版做算法级验证; 编译由 Keil 工程负责。)
临时文件, 跑完可删。
"""
import struct
import sys

MAGIC = 0xAA
END = 0xBB
MAX_PAYLOAD = 1024
HEADER = 9
TAIL = 2
FRAME_MAX = HEADER + MAX_PAYLOAD + TAIL
RX_BUF_SIZE = FRAME_MAX * 3


class DeviceNet:
    """与 ota_net.c 的静态状态 g_rx_buf / g_rx_len 对应。"""

    def __init__(self):
        self.buf = bytearray(RX_BUF_SIZE)
        self.g_rx_len = 0

    # ---- ota_net_rx_reset() ----
    def rx_reset(self):
        self.g_rx_len = 0

    # ---- ota_net_recv() 的数据部分 ----
    def feed(self, data):
        assert self.g_rx_len + len(data) <= RX_BUF_SIZE, "缓冲溢出"
        self.buf[self.g_rx_len:self.g_rx_len + len(data)] = data
        self.g_rx_len += len(data)

    # ---- parse_frame() ----
    def parse_frame(self, start):
        """返回 (rc, frame)。rc>0=整帧长度, 0=未凑齐, -1=非法。"""
        avail = self.g_rx_len - start
        p = self.buf

        if avail < HEADER:
            return (0 if p[start] == MAGIC else -1), None

        if p[start] != MAGIC:
            return -1, None

        length = (p[start + 3] << 8) | p[start + 4]
        if length > MAX_PAYLOAD:
            return -1, None

        if avail < HEADER + length + TAIL:
            return 0, None

        ck = 0
        for i in range(start, start + HEADER + length):
            ck = (ck + p[i]) & 0xFF
        if ck != p[start + HEADER + length]:
            return -1, None
        if p[start + HEADER + length + 1] != END:
            return -1, None

        frame = (p[start + 1], p[start + 2],
                 bytes(p[start + HEADER:start + HEADER + length]),
                 int.from_bytes(p[start + 5:start + 9], 'big'))
        return HEADER + length + TAIL, frame

    # ---- ota_net_next_frame() ----
    def next_frame(self):
        """返回 (1, frame) 或 (0, None)。"""
        stall = 0xFFFFFFFF
        pos = 0

        while pos < self.g_rx_len:
            rc, frame = self.parse_frame(pos)

            if rc > 0:
                consumed = pos + rc
                if consumed < self.g_rx_len:
                    keep = self.g_rx_len - consumed
                    self.buf[0:keep] = self.buf[consumed:consumed + keep]
                self.g_rx_len -= consumed
                return 1, frame

            if rc == 0:
                if stall == 0xFFFFFFFF:
                    stall = pos

            i = pos + 1
            while i < self.g_rx_len:
                if self.buf[i] == MAGIC:
                    break
                i += 1
            pos = i

        if stall != 0xFFFFFFFF:
            if stall > 0:
                keep = self.g_rx_len - stall
                self.buf[0:keep] = self.buf[stall:stall + keep]
                self.g_rx_len = keep
        else:
            self.g_rx_len = 0

        return 0, None

    def drain(self):
        out = []
        while True:
            ok, f = self.next_frame()
            if not ok:
                return out
            out.append(f)


def mframe(cmd, seq, payload=b'', offset=0):
    head = bytearray([MAGIC, cmd, seq,
                      (len(payload) >> 8) & 0xFF, len(payload) & 0xFF,
                      (offset >> 24) & 0xFF, (offset >> 16) & 0xFF,
                      (offset >> 8) & 0xFF, offset & 0xFF])
    body = bytes(head) + payload
    return body + bytes([sum(body) & 0xFF, END])


FAIL = []


def ck(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


print("[A] 基本重组")
d = DeviceNet()
d.feed(mframe(0x04, 1, b'\x11' * 8, 0) + mframe(0x05, 2))
fr = d.drain()
ck(len(fr) == 2, "一次喂两帧 → 两帧都取出")
ck(fr[0][0] == 0x04 and fr[0][2] == b'\x11' * 8, "第 1 帧内容正确")
ck(fr[0][3] == 0, "第 1 帧 offset 大端解读正确")
ck(fr[1][0] == 0x05 and fr[1][2] == b'', "第 2 帧内容正确")
ck(d.g_rx_len == 0, "取完后缓冲清空")

print("[B] 半帧分两次到达")
d = DeviceNet()
raw = mframe(0x04, 7, b'\xAB' * 300, 1024)
d.feed(raw[:100])
ck(d.next_frame() == (0, None), "只到 100 字节 → 取不出帧")
ck(d.g_rx_len == 100, "半帧残余 100 字节被保留")
d.feed(raw[100:])
ok, fr = d.next_frame()
ck(ok == 1 and fr[2] == b'\xAB' * 300 and fr[3] == 1024, "补齐后解析出完整帧")
ck(d.g_rx_len == 0, "取完后缓冲清空")

print("[C] 缓冲区前部的垃圾字节")
d = DeviceNet()
d.feed(b'\x00\xFF\x01' + mframe(0x06, 9, b'\x00' * 4))
fr = d.drain()
ck(len(fr) == 1 and fr[0][0] == 0x06, "跳过 3 个垃圾字节后取到帧")

print("[D] 关键用例: 杂散 0xAA 伪造出大长度帧头")
d = DeviceNet()
# 假帧头 0xAA, 后面紧跟着一个隐含长度很大的 LEN(0x0300=768)
fake = b'\xAA\x99\x99\x03\x00\x00\x00\x00\x00'
d.feed(fake + mframe(0x06, 3, b'\x22' * 8))
ok, fr = d.next_frame()
ck(ok == 1 and fr[0] == 0x06,
   "假帧头挡在前面时, 后面的真帧仍能立刻取出(老实现会一直卡住)")
ck(d.g_rx_len == 0, "假帧头与垃圾一起被丢弃")

print("[E] 尾部短垃圾(<9 字节)")
d = DeviceNet()
d.feed(mframe(0x05, 1) + b'\x01\x02\x03\x04')
fr = d.drain()
ck(len(fr) == 1, "取出唯一的帧")
ck(d.g_rx_len == 0, "不足 9 字节且首字节非 0xAA 的尾部垃圾被丢弃")

print("[F] 尾部 4 字节且首字节恰为 0xAA (可能只是还没收全)")
d = DeviceNet()
d.feed(b'\xAA\x04\x00\x00')
ck(d.next_frame() == (0, None), "取不出帧")
ck(d.g_rx_len == 4, "这 4 字节必须保留(万一是下一帧的开头)")

print("[G] 校验和损坏")
d = DeviceNet()
bad = bytearray(mframe(0x04, 1, b'\x55' * 8))
bad[9] ^= 0xFF
d.feed(bytes(bad) + mframe(0x05, 2))
fr = d.drain()
ck(len(fr) == 1 and fr[0][0] == 0x05, "坏帧被丢弃, 紧跟的好帧仍能取出")

print("[H] 长度字段非法(>1024)")
d = DeviceNet()
d.feed(b'\xAA\x04\x01\xFF\xFF' + b'\x00' * 10 + mframe(0x06, 5))
fr = d.drain()
ck(len(fr) == 1 and fr[0][0] == 0x06, "LEN 越界的假帧被跳过")

print("[I] 载荷里含 0xAA 的半帧, 残余必须仍从真帧头开始")
d = DeviceNet()
raw = mframe(0x04, 1, b'\xAA' * 40 + b'\x00' * 200, 0)
d.feed(raw[:60])                       # 只发前半截(payload 里有 0xAA)
ck(d.next_frame() == (0, None), "半帧取不出")
ck(d.g_rx_len == 60 and d.buf[0] == MAGIC,
   "残余仍从真正的帧头开始(没有被载荷里的 0xAA 带偏)")
d.feed(raw[60:])
ok, fr = d.next_frame()
ck(ok == 1 and len(fr[2]) == 240, "补齐后正确解析出 240 字节载荷")

print("[J] 满载荷 1024 字节帧 (连续 3 帧 = 缓冲上限)")
d = DeviceNet()
big = mframe(0x04, 1, b'\x33' * MAX_PAYLOAD, 0)
ck(len(big) == FRAME_MAX, "整帧长度 == OTA_FRAME_MAX_SIZE")
d.feed(big * 3)
fr = d.drain()
ck(len(fr) == 3, "缓冲满时 3 个最大帧全部取出")
ck(all(len(f[2]) == MAX_PAYLOAD for f in fr), "载荷长度均为 1024")

print("[K] 空缓冲反复调用")
d = DeviceNet()
ck(d.next_frame() == (0, None), "空缓冲返回 0")
ck(d.g_rx_len == 0, "不越界, 保持空")

print()
print("==== %d FAIL ====" % len(FAIL))
for m in FAIL:
    print("  !! " + m)
sys.exit(1 if FAIL else 0)
