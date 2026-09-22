"""Offline round-trip verification of the axle counter handler against
spec section 5 (two-step read + watermark ACK)."""
import random
import struct
import time

from axle_counter_handler import AxleCounterHandler, AxlePulseMode
from module_profiles import (get_profile, FLFXG_AXLE_COUNTER,
                             FLFXG_AXLE_COUNTER_HIGH)
from data_simulator import DataSimulator
from modbus_crc import modbus_crc16


def mk_req(addr, cc, count):
    raw = struct.pack(">BBHH", addr, 0x03, cc, count)
    crc = modbus_crc16(raw)
    return raw + struct.pack("<H", crc)


def reg(resp, i):
    return (resp[2 + i * 2] << 8) | resp[3 + i * 2]


def pulse_block(resp, reg_bytes=28):
    """Return list of pulse pkts from a response (empty if none).

    寄存器块长度随实时值组数变化 —— 低频 0x50 = 28 字节, 高频 0x53 = 64 字节。
    脉冲块紧跟寄存器块之后: [本包脉冲个数(2B)] + per pulse
    [sec(4)][ms(2)][channel(1)][n(1)][data(n×1B)] → 每条 8+n 字节。
    """
    count_off = 2 + reg_bytes           # 脉冲个数紧跟在寄存器块之后
    # 状态查询应答止于"寄存器块 + CRC", 没有脉冲个数域 → 视为无脉冲块
    if len(resp) <= count_off + 2:
        return []
    n = (resp[count_off] << 8) | resp[count_off + 1]
    out, off = [], count_off + 2
    for _ in range(n):
        pn = resp[off + 7]              # 采样点数 n
        out.append(resp[off:off + 8 + pn])
        off += 8 + pn
    return out


def make_handler(n=3):
    profile = get_profile(FLFXG_AXLE_COUNTER)
    h = AxleCounterHandler(address=4, profile=profile,
                           data_gen=DataSimulator(profile),
                           pulse_mode=AxlePulseMode.PULSE,
                           channel_voltage_raw=100, channel_current_raw=200)
    # 手工注入 n 个脉冲 (不走按钮发送, 便于测试状态机)
    h._pulse_events = [(None, h._build_pulse_event(i == 0)) for i in range(n)]
    return h


def check(name, cond, detail=""):
    if not cond:
        raise AssertionError(f"FAIL {name}: {detail}")
    print(f"  ok  {name}" + (f"  ({detail})" if detail else ""))


def test_three_step_cycle():
    print("== spec section 5: A/B/C 完整读取循环 ==")
    h = make_handler()

    # Cycle A: [addr,03,cc=0,count=0] 状态查询
    ra = mk_req(4, 0, 0)
    a = h.build_response(ra)
    check("A 长度==32 (无脉冲块)", len(a) == 32, f"len={len(a)}")
    check("A reg12==1", reg(a, 12) == 1, f"reg12={reg(a,12)}")
    check("A reg13==3", reg(a, 13) == 3, f"reg13={reg(a,13)}")
    check("A reg1 KGL==0x3F", reg(a, 1) == 0x3F, hex(reg(a, 1)))
    vi = [reg(a, i) for i in range(4, 10)]
    check("A reg4-9 == V1,V2,V3,I1,I2,I3 (10V/20mA)",
          vi == [100, 100, 100, 200, 200, 200], str(vi))
    check("A 无脉冲块", pulse_block(a) == [])

    # Cycle B: [addr,03,cc=1,count=3] 读取请求
    b = h.build_response(mk_req(4, 1, 3))
    check("B reg12==2", reg(b, 12) == 2, f"reg12={reg(b,12)}")
    check("B reg13==0", reg(b, 13) == 0, f"reg13={reg(b,13)}")
    pb = pulse_block(b)
    check("B 3 个脉冲", len(pb) == 3, f"n={len(pb)}")
    for i, p in enumerate(pb):
        n = p[7]
        data = list(p[8:8 + n])
        check(f"B 脉冲{i+1} 63点平滑脉冲(0→10V→0)", n == 63 and data[0] == 0
              and data[-1] == 0 and max(data) == 100, f"n={n}")
    check("B 脉冲已发出未确认 (队列保留, seq=2)",
          len(h._pulse_events) == 3 and all(s == 2 for s, _ in h._pulse_events))

    # Cycle C: [addr,03,cc=2,count=0] 回显确认
    c = h.build_response(mk_req(4, 2, 0))
    check("C reg12==3", reg(c, 12) == 3, f"reg12={reg(c,12)}")
    check("C reg13==0", reg(c, 13) == 0, f"reg13={reg(c,13)}")
    check("C 无脉冲块", pulse_block(c) == [])
    check("C 水位 ACK 清空队列", len(h._pulse_events) == 0,
          f"queue={len(h._pulse_events)}")


def test_smooth_random_pulse():
    print("== 平滑随机脉冲: 10V/250ms, 升余弦升降沿, 每次略不同 ==")
    profile = get_profile(FLFXG_AXLE_COUNTER)
    h = AxleCounterHandler(address=4, profile=profile,
                           data_gen=DataSimulator(profile),
                           pulse_mode=AxlePulseMode.PULSE)
    h._last_pulse_time = time.monotonic()
    random.seed(0x50)   # 固定随机源, 测试可复现
    rises = []
    for i in range(5):
        pkt = h._build_pulse_event(i == 0)
        n = pkt[7]
        data = list(pkt[8:8 + n])
        check("250ms/4ms = 63 采样点", n == 63, f"n={n}")
        check("起点 0V", data[0] == 0, f"data[0]={data[0]}")
        check("终点 0V", data[-1] == 0, f"data[-1]={data[-1]}")
        check("峰值 10V(100)", max(data) == 100, f"max={max(data)}")
        # 完整形态: 先单调上升后单调下降 (无折线/无毛刺)
        peak = max(range(n), key=lambda k: data[k])
        left = [data[k] <= data[k + 1] for k in range(peak)]
        right = [data[k] >= data[k + 1] for k in range(peak, n - 1)]
        check("先单调上升后单调下降", all(left) and all(right), str(data))
        # 升余弦边沿: 起始/结尾平缓, 第5点仍在低位 (远未到峰值)
        check("起点附近平缓上升", 0 < data[5] < 40, f"data[5]={data[5]}")
        # 无平顶保持段: 到达峰值附近后立即回落 (峰值点数很少)
        n_peak = sum(1 for v in data if v == 100)
        check("无平顶(峰值点极少)", n_peak <= 4, f"峰值点数={n_peak}")
        # 统计首个到达峰值(10V)的索引 → 反映随机上升沿长度
        rises.append(next(k for k, v in enumerate(data) if v == 100))
    check("上升沿长度随机(5个脉冲不全相同)", len(set(rises)) > 1, f"rises={rises}")


def test_manual_send_confirm():
    print("== 手动发送曲线 + 通信板水位确认 ==")
    h = make_handler(0)   # 空队列

    # 1) 实时模式下不允许发送
    h.curve_mode = AxlePulseMode.REALTIME_ONLY
    check("实时模式禁止发送", h.trigger_curve_send() is False)
    h.curve_mode = AxlePulseMode.PULSE

    # 2) 发送 → 入队 pulses_per_passage 个脉冲, 全部同通道
    check("发送入队成功", h.trigger_curve_send() is True)
    check("队列 4 脉冲", len(h._pulse_events) == 4, f"n={len(h._pulse_events)}")
    check("未确认前 in_flight", h.curve_in_flight is True)
    check("成功计数为 0", h.curve_send_count == 0, f"ok={h.curve_send_count}")
    chans = {h._pulse_events[i][1][6] for i in range(4)}
    check("4 脉冲同通道 0 (默认)", chans == {0}, str(sorted(chans)))

    # 时间戳: 一趟过车脉冲按 pulse_interval(默认1s=1Hz) 依次递增,
    # 曲线开始时间(reg2-3+reg11) = 首脉冲时间戳
    def pulse_ts(pkt):
        sec, ms = struct.unpack(">IH", pkt[0:6])
        return sec * 1000 + ms

    secs = [pulse_ts(h._pulse_events[i][1]) for i in range(4)]
    check("4 脉冲时间戳 1Hz 递增 (t0,+1s,+2s,+3s)",
          secs == [secs[0] + 1000 * i for i in range(4)], str(secs))
    check("曲线开始时间 = 首脉冲时间戳",
          h._curve_start_sec * 1000 + h._curve_start_ms == secs[0],
          f"start={h._curve_start_sec*1000 + h._curve_start_ms} p1={secs[0]}")

    # 3) 状态查询: reg13=4, code=1, 仍 in_flight
    a = h.build_response(mk_req(4, 0, 0))
    check("状态 reg12==1 reg13==4", reg(a, 12) == 1 and reg(a, 13) == 4,
          f"reg12={reg(a,12)} reg13={reg(a,13)}")
    check("状态后仍未确认", h.curve_in_flight is True)

    # 4) 读取全部: 交付 4 个, code=2, reg13=0, 仍 in_flight
    b = h.build_response(mk_req(4, 1, 4))
    check("读取 reg12==2 reg13==0 4脉冲", reg(b, 12) == 2 and reg(b, 13) == 0
          and len(pulse_block(b)) == 4,
          f"reg12={reg(b,12)} reg13={reg(b,13)}")
    check("读取后仍未确认", h.curve_in_flight is True)

    # 5) 回显 C2 → 全部确认 → 计数+1
    c = h.build_response(mk_req(4, 2, 0))
    check("确认后 in_flight False", h.curve_in_flight is False)
    check("确认后计数 1", h.curve_send_count == 1, f"ok={h.curve_send_count}")
    check("确认事件标志", h.curve_confirmed_this_req is True)

    # 6) 下一请求会重置确认标志; 清空后可再次发送
    d = h.build_response(mk_req(4, 3, 0))
    check("下一请求重置确认标志", h.curve_confirmed_this_req is False)
    check("清空后可再次发送", h.trigger_curve_send() is True)


def test_manual_no_double_send():
    print("== 同一时间仅允许一个未确认批次 ==")
    h = make_handler(0)
    check("首次发送", h.trigger_curve_send() is True)
    check("未确认时拒绝再次发送", h.trigger_curve_send() is False)
    h.cancel_curve_send()
    check("取消后可再发送", h.trigger_curve_send() is True)


def test_manual_channel_select():
    print("== 通道切换 (0,1,2, 所选通道持续生效) ==")
    h = make_handler(0)
    check("默认通道 0", h.current_channel == 0, f"ch={h.current_channel}")

    h.set_curve_channel(1)
    check("切到通道 1", h.current_channel == 1, f"ch={h.current_channel}")
    h.trigger_curve_send()
    chans = {h._pulse_events[i][1][6] for i in range(len(h._pulse_events))}
    check("发送在通道 1", chans == {1}, str(sorted(chans)))
    check("发送后通道仍 1", h.current_channel == 1, f"ch={h.current_channel}")

    # 确认清空后再切通道 2
    h.build_response(mk_req(4, 0, 0))
    h.build_response(mk_req(4, 1, 4))
    h.build_response(mk_req(4, 2, 0))
    h.set_curve_channel(2)
    check("切到通道 2", h.current_channel == 2, f"ch={h.current_channel}")
    h.trigger_curve_send()
    chans = {h._pulse_events[i][1][6] for i in range(len(h._pulse_events))}
    check("发送在通道 2", chans == {2}, str(sorted(chans)))

    # 通道号循环回绕: 3 → 0
    h.set_curve_channel(3)
    check("切到通道 0 (3 回绕)", h.current_channel == 0, f"ch={h.current_channel}")


def test_passage_channel_grouping():
    print("== 过车通道分组 (§6 按通道拆帧) ==")
    profile = get_profile(FLFXG_AXLE_COUNTER)
    h = AxleCounterHandler(address=4, profile=profile,
                           data_gen=DataSimulator(profile),
                           pulse_mode=AxlePulseMode.PULSE,
                           num_channels=3, pulses_per_passage=4)
    h._last_pulse_time = time.monotonic()
    # 手动生成 10 个脉冲, 通道应依次为 0,0,0,0, 1,1,1,1, 2,2
    h._pulse_events = [(None, h._build_pulse_event(i == 0)) for i in range(10)]
    channels = [h._pulse_events[i][1][6] for i in range(10)]
    expected = [0, 0, 0, 0, 1, 1, 1, 1, 2, 2]
    check("过车分组通道", channels == expected, str(channels))


def test_high_freq_layout():
    print("== 高频计轴 0x53: 实时值 4 组 (reg4-27), 其余与 0x50 一致 ==")
    profile = get_profile(FLFXG_AXLE_COUNTER_HIGH)
    check("profile 32 寄存器", profile.input_regs == 32, f"in={profile.input_regs}")
    check("realtime_bytes 64", profile.realtime_bytes == 64,
          f"rt={profile.realtime_bytes}")

    h = AxleCounterHandler(address=5, profile=profile,
                           data_gen=DataSimulator(profile),
                           pulse_mode=AxlePulseMode.PULSE,
                           channel_voltage_raw=100, channel_current_raw=200)
    check("剩余未读数偏移 64", h.unread_offset == 64, f"off={h.unread_offset}")
    check("寄存器块 64 字节", h.register_block_bytes == 64,
          f"blk={h.register_block_bytes}")

    h._pulse_events = [(None, h._build_pulse_event(i == 0)) for i in range(3)]

    # 状态查询: 应答 = addr+func(2) + 寄存器块(64) + CRC(2) = 68 字节, 无脉冲块
    a = h.build_response(mk_req(5, 0, 0))
    check("状态应答 68 字节", len(a) == 68, f"len={len(a)}")
    check("reg1 KGL==0x3F", reg(a, 1) == 0x3F, hex(reg(a, 1)))
    # reg4-27 = 低频 reg4-9 的内容 (V1,V2,V3,I1,I2,I3) 重复 4 次
    grp = [100, 100, 100, 200, 200, 200]
    vi = [reg(a, i) for i in range(4, 28)]
    check("reg4-27 = 4×[V1,V2,V3,I1,I2,I3]", vi == grp * 4, str(vi))
    check("reg28 保留==0", reg(a, 28) == 0, f"reg28={reg(a, 28)}")
    check("reg29 曲线开始毫秒<1000", reg(a, 29) < 1000, f"ms={reg(a, 29)}")
    check("reg30 确认码==1", reg(a, 30) == 1, f"reg30={reg(a, 30)}")
    check("reg31 剩余未读数==3", reg(a, 31) == 3, f"reg31={reg(a, 31)}")
    check("状态查询无脉冲块", pulse_block(a, 64) == [])
    curve_sec = (reg(a, 2) << 16) | reg(a, 3)
    check("reg2-3 曲线开始秒 非0", curve_sec > 0, str(curve_sec))

    # 读取请求: 脉冲块紧跟 64 字节寄存器块, 格式与低频完全一致
    b = h.build_response(mk_req(5, 1, 3))
    check("读取 reg30==2 reg31==0", reg(b, 30) == 2 and reg(b, 31) == 0,
          f"reg30={reg(b, 30)} reg31={reg(b, 31)}")
    pb = pulse_block(b, 64)
    check("3 个脉冲", len(pb) == 3, f"n={len(pb)}")
    for i, p in enumerate(pb):
        n = p[7]
        data = list(p[8:8 + n])
        check(f"脉冲{i+1} 63点平滑脉冲(0→10V→0)",
              n == 63 and data[0] == 0 and data[-1] == 0 and max(data) == 100,
              f"n={n}")

    # 水位 ACK: 回显 C2 清空该批
    c = h.build_response(mk_req(5, 2, 0))
    check("回显 C2 清空队列", len(h._pulse_events) == 0,
          f"queue={len(h._pulse_events)}")
    check("确认后 reg31==0", reg(c, 31) == 0, f"reg31={reg(c, 31)}")


def test_high_freq_manual_send():
    print("== 高频计轴 0x53: 手动发送/通道/1Hz 时间戳 与 0x50 行为一致 ==")
    profile = get_profile(FLFXG_AXLE_COUNTER_HIGH)
    h = AxleCounterHandler(address=5, profile=profile,
                           data_gen=DataSimulator(profile),
                           pulse_mode=AxlePulseMode.PULSE)
    check("默认通道 0", h.current_channel == 0, f"ch={h.current_channel}")
    check("发送入队成功", h.trigger_curve_send() is True)

    pb = pulse_block(h.build_response(mk_req(5, 0, 4)), 64)
    check("交付 4 脉冲", len(pb) == 4, f"n={len(pb)}")
    chans = {p[6] for p in pb}
    check("4 脉冲同通道 0", chans == {0}, str(sorted(chans)))

    secs = [struct.unpack(">IH", p[0:6])[0] * 1000
            + struct.unpack(">IH", p[0:6])[1] for p in pb]
    check("时间戳 1Hz 递增", secs == [secs[0] + 1000 * i for i in range(4)],
          str(secs))
    check("曲线开始时间 = 首脉冲",
          h._curve_start_sec * 1000 + h._curve_start_ms == secs[0],
          f"start={h._curve_start_sec*1000 + h._curve_start_ms} p1={secs[0]}")

    h.build_response(mk_req(5, 2, 0))       # 通信板回显 C2 → 确认整批
    check("确认后计数 1", h.curve_send_count == 1, f"ok={h.curve_send_count}")
    check("确认后 in_flight False", h.curve_in_flight is False)


def test_frame_loss_retransmit():
    print("== 丢帧后重发 (已发出未确认的脉冲不能丢) ==")
    h = make_handler(n=1)

    # 状态查询: reg13=1, code=1
    s = h.build_response(mk_req(4, 0, 0))
    check("状态 reg12==1 reg13==1", reg(s, 12) == 1 and reg(s, 13) == 1,
          f"reg12={reg(s,12)} reg13={reg(s,13)}")

    # 读取: 交付 1 个, reg13=0, code=2 — 该帧丢失 (通信板未回显 C2)
    r1 = h.build_response(mk_req(4, 1, 1))
    check("读取 reg12==2 reg13==0 1脉冲", reg(r1, 12) == 2 and reg(r1, 13) == 0
          and len(pulse_block(r1)) == 1)
    check("未确认脉冲保留在队列", len(h._pulse_events) == 1,
          f"queue={len(h._pulse_events)}")

    # 通信板仍持 cc=1, 重新读取
    r2 = h.build_response(mk_req(4, 1, 1))
    check("重发 reg12==3", reg(r2, 12) == 3, f"reg12={reg(r2,12)}")
    check("重发仍带 1 个脉冲", len(pulse_block(r2)) == 1)
    check("重发更新序号为 3", all(s == 3 for s, _ in h._pulse_events))

    # 通信板收到后回显 C3 → 清除
    c = h.build_response(mk_req(4, 3, 0))
    check("回显 C3 后队列清空", len(h._pulse_events) == 0,
          f"queue={len(h._pulse_events)}")


def test_partial_read_watermark():
    print("== 分批读取 + 水位推进 ==")
    h = make_handler()
    h.build_response(mk_req(4, 0, 0))   # code=1, reg13=3

    # 读 1 个 (cc=1): 交付 p1(code=2), reg13=2, 队列3
    r1 = h.build_response(mk_req(4, 1, 1))
    check("分批读1 reg12==2 reg13==2", reg(r1, 12) == 2 and reg(r1, 13) == 2,
          f"reg12={reg(r1,12)} reg13={reg(r1,13)}")
    check("队列3(p1已发未确认)", len(h._pulse_events) == 3)

    # 读 1 个, 回显 C2 → 确认 p1; 交付 p2(code=3), reg13=1, 队列2
    r2 = h.build_response(mk_req(4, 2, 1))
    check("分批读2 reg12==3 reg13==1", reg(r2, 12) == 3 and reg(r2, 13) == 1,
          f"reg12={reg(r2,12)} reg13={reg(r2,13)}")
    check("队列2(p2已发未确认 + p3未发)", len(h._pulse_events) == 2)

    # 读 1 个, 回显 C3 → 确认 p2; 交付 p3(code=4), reg13=0, 队列1
    r3 = h.build_response(mk_req(4, 3, 1))
    check("分批读3 reg12==4 reg13==0", reg(r3, 12) == 4 and reg(r3, 13) == 0,
          f"reg12={reg(r3,12)} reg13={reg(r3,13)}")
    check("队列1(p3已发未确认)", len(h._pulse_events) == 1)

    # 状态查询回显 C4 → 全部确认清空
    s = h.build_response(mk_req(4, 4, 0))
    check("回显 C4 清空", len(h._pulse_events) == 0 and reg(s, 13) == 0)


if __name__ == "__main__":
    test_three_step_cycle()
    test_smooth_random_pulse()
    test_manual_send_confirm()
    test_manual_no_double_send()
    test_manual_channel_select()
    test_high_freq_layout()
    test_high_freq_manual_send()
    test_frame_loss_retransmit()
    test_partial_read_watermark()
    test_passage_channel_grouping()
    print("\nALL PASS")
