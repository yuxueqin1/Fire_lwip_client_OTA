# -*- coding: utf-8 -*-
"""离线自测: 上位机 OTA 协议层 + 会话状态机 (用一个假设备模拟 STM32 侧)。

临时文件, 跑完可删:  python _ota_selftest.py
"""
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import ota_gui as G

FAIL = []


def ck(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond:
        FAIL.append(msg)


# ---------- 1. 流式帧重组 ----------
print("[1] parse_stream_frames")
f1 = G.OTAFrame(0x00, 1, b'\x02\x00\x00\x01\x01\x02\x00\x00')
f2 = G.OTAFrame(0x11, 2, b'')
b1, b2 = f1.to_bytes(), f2.to_bytes()
fr, left = G.parse_stream_frames(b1 + b2)
ck(len(fr) == 2 and left == b'', "一次喂两帧 → 两帧都解析出来")
fr, left = G.parse_stream_frames(b1 + b2[:5])
ck(len(fr) == 1 and left == b2[:5], "第二帧只到一半 → 保留为尾部残留")
fr, left = G.parse_stream_frames(left + b2[5:])
ck(len(fr) == 1 and left == b'', "补齐后半帧 → 解析出第二帧")
fr, left = G.parse_stream_frames(b'\x00\xFF\xAA' + b1)
ck(len(fr) == 1, "前面有垃圾字节 → 重同步成功")
bad = bytearray(b1)
bad[9] ^= 0xFF
fr, left = G.parse_stream_frames(bytes(bad) + b1)
ck(len(fr) == 1, "校验和错了 → 丢弃 1 字节后重同步")
fr, _ = G.parse_stream_frames(b'\xAA' + b'\x04' * 3 + b'\x00' * 7)
ck(len(fr) == 0, "半帧不完整 → 不产出")
ck(b1[3:5] == struct.pack('>H', 8), "帧头 LEN 大端")
ck(struct.unpack_from('<I', f1.data, 0)[0] == 0x01000002, "载荷版本小端 → 0x01000002")

# ---------- 2. 4 字节补齐 ----------
print("[2] pad_to_align")
ck(G.pad_to_align(b'ABCD') == b'ABCD', "已是 4 的倍数 → 不变")
ck(G.pad_to_align(b'ABC') == b'ABC\x00', "3 字节 → 补 1 个 0x00")
ck(len(G.pad_to_align(b'A' * 1023)) == 1024, "1023 → 1024")
ck(len(G.pad_to_align(b'A' * 1025)) == 1028, "1025 → 1028")

# ---------- 3. 文件名配对 / 版本解析 ----------
print("[3] _pair_paths / _parse_version_from_name")
d = tempfile.mkdtemp()
for n in ('fw_v1.0.0.2_A.bin', 'fw_v1.0.0.2_B.bin'):
    open(os.path.join(d, n), 'wb').write(b'\x00' * 64)
p = G.OTAServerThread._pair_paths(os.path.join(d, 'fw_v1.0.0.2_A.bin'))
ck(G.SLOT_APP1 in p and p.get(G.SLOT_APP2) is not None, "选 _A.bin → 自动配上 _B.bin")
p = G.OTAServerThread._pair_paths(os.path.join(d, 'fw_v1.0.0.2_B.bin'))
ck(p.get(G.SLOT_APP1) is not None, "选 _B.bin → 自动配上 _A.bin")
open(os.path.join(d, 'plain_A.bin'), 'wb').write(b'\x00' * 64)
open(os.path.join(d, 'plain_B.bin'), 'wb').write(b'\x00' * 64)
p = G.OTAServerThread._pair_paths(os.path.join(d, 'plain_A.bin'))
ck(p.get(G.SLOT_APP2) is not None, "无 _A/_B 名字 → 按 <名>_A/_B 找")
ck(G.OTAServerThread._parse_version_from_name('fw_v1.0.0.2_A.bin') == 0x01000002,
   "解析 v1.0.0.2 → 0x01000002")
ck(G.OTAServerThread._parse_version_from_name(
        'YH-F429_2025.09.22_v2.3.4.5_B.bin') == 0x02030405,
   "名字里有日期 → 取最后一个版本号")
ck(G.OTAServerThread._parse_version_from_name('nover.bin') is None, "无版本号 → None")

# ---------- 4. 镜像预检 ----------
print("[4] _validate_image")


def mkimg(slot, base, size=1024, sp=0x20012345, rv=None, fill=0):
    rv = base + 0x100 if rv is None else rv
    data = struct.pack('<II', sp, rv) + bytes([fill]) * (size - 8)
    return {'slot': slot, 'data': data, 'size': size, 'crc': 0, 'raw_size': size}


ck(G.OTAServerThread._validate_image(mkimg(G.SLOT_APP1, 0x08008000)) is None,
   "A 镜像复位向量落在 APP1 → 通过")
ck(G.OTAServerThread._validate_image(mkimg(G.SLOT_APP2, 0x08080000)) is None,
   "B 镜像复位向量落在 APP2 → 通过")
e = G.OTAServerThread._validate_image(mkimg(G.SLOT_APP2, 0x08080000, rv=0x08008100))
ck(e is not None and '复位向量' in e, "A 镜像塞进 B 槽 → 被拦下 (复位向量越界)")
e = G.OTAServerThread._validate_image(mkimg(G.SLOT_APP1, 0x08008000, sp=0x10000000))
ck(e is not None and '栈指针' in e, "栈指针不在 SRAM → 被拦下")
e = G.OTAServerThread._validate_image(mkimg(G.SLOT_APP2, 0x08080000, size=400 * 1024))
ck(e is not None and '容量' in e, "超过 384KB → 被拦下")


# ---------- 5. 假设备 + 全流程 ----------
print("[5] 端到端会话 (假设备, 故意乱切帧)")


class FakeDevice(threading.Thread):
    def __init__(self, sock, fw_size, fw_crc, version_old, cur_slot):
        super().__init__(daemon=True)
        self.s = sock
        self.fw_size = fw_size
        self.fw_crc = fw_crc
        self.ver = version_old
        self.cur_slot = cur_slot
        self.got = b''
        self.seq = 0
        self.got_fw = b''
        self.saw_push = self.saw_data = self.saw_end = False
        self.acked_ready = self.acked_success = False
        self.final_crc_ok = None
        self.push_payload = None
        self.err = None

    def _recv_more(self):
        c = self.s.recv(4096)
        if not c:
            return False
        self.got += c
        return True

    def frames(self):
        while True:
            while len(self.got) < 11:
                if not self._recv_more():
                    return
            if self.got[0] != 0xAA:
                self.got = self.got[1:]
                continue
            ln = (self.got[3] << 8) | self.got[4]
            need = 9 + ln + 2
            while len(self.got) < need:
                if not self._recv_more():
                    return
            body = self.got[:need]
            self.got = self.got[need:]
            yield body[1], body[9:9 + ln], struct.unpack('>I', body[5:9])[0]

    def send(self, cmd, payload=b'', offset=0):
        f = G.OTAFrame(cmd, self.seq, payload)
        f.offset = offset
        self.seq = (self.seq + 1) & 0xFF
        raw = f.to_bytes()
        # 故意每 7 字节切一段, 制造"半帧/粘帧"
        for i in range(0, len(raw), 7):
            self.s.sendall(raw[i:i + 7])
            time.sleep(0.001)

    def run(self):
        try:
            self.send(0x00,
                      struct.pack('<I', self.ver) + bytes([self.cur_slot]) +
                      bytes([G.SLOT_APP2 if self.cur_slot == G.SLOT_APP1 else G.SLOT_APP1]) +
                      b'\x00\x00')
            expect_off = 0
            for cmd, payload, off in self.frames():
                if cmd == 0x10:
                    self.saw_push = True
                    v = struct.unpack('<I', payload[0:4])[0]
                    sz = struct.unpack('<I', payload[4:8])[0]
                    cr = struct.unpack('<I', payload[8:12])[0]
                    self.push_payload = (v, sz, cr)
                    self.send(0x11)
                    time.sleep(0.3)                 # 假装在擦除
                    self.send(0x06, struct.pack('<I', G.ACK_STATE_DOWNLOADING))
                    self.acked_ready = True
                elif cmd == 0x04:
                    self.saw_data = True
                    assert off == expect_off, "offset 不连续: %d != %d" % (off, expect_off)
                    self.got_fw += payload
                    expect_off += len(payload)
                elif cmd == 0x05:
                    self.saw_end = True
                    self.final_crc_ok = (zlib.crc32(self.got_fw) & 0xFFFFFFFF) == self.fw_crc
                    self.send(0x06, struct.pack('<I', G.ACK_STATE_SUCCESS))
                    self.acked_success = True
                    return
        except Exception as exc:                    # noqa: BLE001
            self.err = exc


def listen():
    ls = socket.socket()
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind(('127.0.0.1', 0))
    ls.listen(1)
    return ls, ls.getsockname()[1]


fw = bytearray(struct.pack('<II', 0x20012345, 0x08008101) + b'\xA5' * 4096)
fw = G.pad_to_align(bytes(fw))
fw_crc = zlib.crc32(fw) & 0xFFFFFFFF

srv = G.OTAServerThread()
srv.firmware_images = {G.SLOT_APP1: {'data': fw, 'size': len(fw), 'crc': fw_crc,
                                     'path': 'test_A.bin', 'slot': G.SLOT_APP1,
                                     'raw_size': len(fw)}}
srv.firmware_version = 0x01000002
lsock, port = listen()
srv._running = True

csock = socket.create_connection(('127.0.0.1', port))
asock, addr = lsock.accept()
dev = FakeDevice(csock, len(fw), fw_crc, 0x01000001, G.SLOT_APP2)
dev.start()
srv._handle_client(asock, addr)
dev.join(5)

ck(dev.err is None, "假设备侧无异常 (%s)" % (dev.err,))
ck(dev.saw_push, "设备收到 PUSH_UPDATE")
ck(dev.push_payload == (0x01000002, len(fw), fw_crc),
   "PUSH 载荷小端解读正确 (版本/大小/CRC)")
ck(dev.saw_data, "设备收到 DATA")
ck(dev.saw_end, "设备收到 END")
ck(dev.final_crc_ok is True, "设备重组出的固件 CRC32 == 上位机上报的 CRC32")
ck(dev.acked_ready, "擦除完成后发了 ACK(DOWNLOADING)")
ck(dev.acked_success, "校验通过后发了 ACK(SUCCESS)")
evs = []
while not srv.event_queue.empty():
    evs.append(srv.event_queue.get_nowait())
ck(any(e['type'] == 'transfer_complete' for e in evs), "上位机判定升级完成")
ck(any(e['type'] == 'client_version' and e['message'].get('slot_name') == 'B'
       for e in evs), "HELLO 里的槽位被解析 (B 区)")
ck(any(e['type'] == 'frame_tx' and e['message'].get('cmd') == 0x05 for e in evs),
   "END 帧已记录")

# ---------- 6. 设备报错 → 会话结束 ----------
print("[6] 设备报错 → 会话结束, 不卡死")
srv2 = G.OTAServerThread()
srv2.firmware_images = srv.firmware_images
srv2.firmware_version = 0x01000002
srv2._running = True
lsock2, p2 = listen()
sock_c = socket.create_connection(('127.0.0.1', p2))
sock_s, a2 = lsock2.accept()


def bad_dev():
    sock_c.sendall(G.OTAFrame(0x00, 0,
                              struct.pack('<I', 1) + b'\x02\x01\x00\x00').to_bytes())
    sock_c.recv(4096)
    sock_c.sendall(G.OTAFrame(0x13, 0, struct.pack('<I', 13)).to_bytes())


threading.Thread(target=bad_dev, daemon=True).start()
t0 = time.time()
srv2._handle_client(sock_s, a2)
ck(time.time() - t0 < 5, "收到 ERROR 后会话立即结束(不占线程)")
evs2 = []
while not srv2.event_queue.empty():
    evs2.append(srv2.event_queue.get_nowait())
ck(any(e['type'] == 'error' and 'CRC' in str(e['message']) for e in evs2),
   "错误码翻译成可读文本 (CRC32 校验失败)")

# ---------- 7. 设备侧拒绝时 ISR 不泄漏 ----------
print("[7] 版本已最新 → 不推送")
srv3 = G.OTAServerThread()
srv3.firmware_images = srv.firmware_images
srv3.firmware_version = 0x01000001
srv3._running = True
lsock3, p3 = listen()
sock_c3 = socket.create_connection(('127.0.0.1', p3))
sock_s3, a3 = lsock3.accept()


def same_ver_dev():
    sock_c3.sendall(G.OTAFrame(0x00, 0,
                               struct.pack('<I', 0x01000001) + b'\x01\x02\x00\x00').to_bytes())
    time.sleep(0.6)
    sock_c3.close()


threading.Thread(target=same_ver_dev, daemon=True).start()
srv3._handle_client(sock_s3, a3)
evs3 = []
while not srv3.event_queue.empty():
    evs3.append(srv3.event_queue.get_nowait())
ck(not any(e['type'] == 'frame_tx' and e['message'].get('cmd') == 0x10 for e in evs3),
   "版本相同时不发 PUSH_UPDATE")
ck(srv3.client_count == 0, "会话结束后客户端已注销(无泄漏)")

print()
print("==== %d FAIL ====" % len(FAIL))
for m in FAIL:
    print("  !! " + m)
sys.exit(1 if FAIL else 0)
