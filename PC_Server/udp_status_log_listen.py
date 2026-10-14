#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通信板 状态日志 接收工具 (二进制帧, v2 变长)

背景:
    User/task_rs485_7.c 里的 dbg_log_udp_send() 在【每次写完 IO 模块 DO 之后】
    (主机模式) 或【限流 500ms】(从机模式) 把一个状态帧发到
    192.168.1.88:50000。本工具就是那个端口的接收端。
    与 udp_debug_listen.py 不是一回事: 后者收的是 39999 端口上的文本调试信息。

帧格式 (v3, 变长, 没有长度字段, 靠 UDP 报文长度定界):
    固定头 15 字节:
      [0..4]  00 A6 6A 88 99        帧头同步字
      [5]     s_local_board_alarm   本板报警 (维度1离线 或 维度2 KGL)
      [6]     s_remote_board_alarm  从机通信板报警 (第7路 FC03 读回)
      [7]     s_io_cut_switch       DI0 切断开关
      [8]     s_io_break_switch     DI1 断接开关
      [9]     s_buzzer              DO1 蜂鸣器 (真值表输出)
      [10]    s_alarm_light         DO2 报警灯 (真值表输出)
      [11]    s_work_light          DO0 工作灯 (上位机 TCP 连接状态)
      [12]    s_io_comm_fail_cnt    IO模块 FC02 连续失败计数
      [13]    s_slave_read_fail_cnt 第7路 FC03 读从机板 连续失败计数
      [14]    port_cnt              后面跟的端口块个数 (0..6)
    然后紧跟 port_cnt 个端口块, 每块 13 字节:
      +0      port_idx        端口号 0..5 (第几路 RS485)
      +1..+4  link_mask       LE u32, 在线模块位图
      +5..+8  expected_mask   LE u32, 按 SlaveModuelNum 应有位图
      +9..+12 kgl_alarm_mask  LE u32, bit=1 → 该模块 KGL 报警
    最后是帧尾诊断量 (定长 4 字节 + port_cnt 字节):
      +0      s_slave_last_err  最后一次 FC03 读从机板的 eMBErrorCode (0=成功)
      +1..+2  uptime            上电至今秒数 LE u16 (HAL_GetTick()/1000 低 16 位)
      +3      WTD_RESET         看门狗复位码 (0 = 此刻没判要复位)
      +4..    每端口"本轮实际发起请求的模块数", 与端口块一一对应

    总长 = 15 + 13*port_cnt + 4 + port_cnt。只有【配了采集盒的端口】才在
    端口块和每端口字节里。诊断量一律加在帧尾, 所以 [5..13] 和 [14]/端口块
    的偏移永远不变。也兼容 v39/v40 (帧尾定长 3 字节, 没有 WTD_RESET)、
    v2 (帧尾只有 1 字节错误码)、没有帧尾的 v2、以及最老的 14 字节定长帧
    (不打印端口行) —— 四种长度互不相等, 不会误判。

    第二类帧 (v42 临时): OTA 任务存活诊断帧, 定长 25 字节, 帧头是
      00 A6 6A 88 9A (末字节 9A, 与状态帧的 99 区分), 与状态帧共用同一个
      socket / 目的地址, 跟着状态帧后面一帧一帧地发。布局见
      User/task_rs485_7.c 的 dbg_ota_diag_send(); 本工具把它渲染成一行
      "[OTA诊断] ...", 用来判板子为什么以复位码 31(OTA任务卡死)反复复位:
        report/loop 两帧之间不涨 + stage=1(循环顶) → 任务被饿死(排不上CPU);
        report/loop 不涨 + stage=2..10             → 卡在那个调用里出不来。
      正常(任务健康)时它每 500ms 也发, 所以【别把这一行当报警】:
      只有 stage 变化 / 计数器冻结 / 倒计时见底 / WTD_RESET 非 0 才会打。

    主机模式: 头部 14 字节全部有效。
    从机模式: 只有 [5] 和 [11] 有效, 其余恒 0; 端口块照常有意义
              (从机板自己也扫自己那 6 路采集盒)。
    两块板发的是同一种帧, 靠【源 IP】区分是哪块。

怎么用它定位报警:
    先看主行的 L/R/brk 是哪一位在报, 再看端口块把 L=1 拆到具体模块:
      expected & ~link  ->  这几位【离线】(维度1)
      kgl_alarm_mask    ->  这几位【KGL 开关量报警】(维度2)
      两者都为 0 的端口是干净的
    端口行尾部带 "<--" 的就是报警源, 本工具会直接标出来。

    ⚠ 端口行里的"问=N"是【本轮实际发起请求的模块数】(帧尾每端口那一个字节),
      它把"link 全 0"分成了两种完全不同的病:
        问=0  ->  这一轮压根没发请求: 该端口所有模块都处于"连续失败≥3 次
                  后的跳过态", 每 10 轮(约 10s)才探一次 → 【粘住】, 不是
                  新故障; 盒子可能早就好了, 但掩码要等下一次探测才会恢复
        问>0  ->  真的问了这么多台却一台都没答 → 【真故障】
        问==该端口模块数 -> 本轮全量轮询, 没有模块被跳过(正常状态)
      本工具会直接把"link 全 0 且 问=0"标成"跳过态(粘住)"。

    另外几条:
      [6]=1 且 [13]=0  ->  FC03 读成功、从机板确实返回了 1:
                            是【从机板真的报警】, 去查从机板自己那台
      [6]=1 且 [13]>0  ->  FC03 读失败, R 是 fail-safe 置的, 再看帧尾错误码:
                            尾=8 ETIMEDOUT -> 从机根本没应答. 同总线上别的
                              从站(如 IO 模块)是好的就说明总线没问题,
                              要查 addr1 那侧: 从机栈没跑/地址不是1/收发器
                            尾=3 EPORTERR  -> 应答来了但帧坏了(CRC):
                              方向控制时序/总线冲突/干扰/波特率不一致
                            ⚠ 注意 [13] 是"连续"失败次数, 每次成功都清零,
                              所以 [13]=38 意味着到这一帧为止已经连续失败
                              38 轮 (第7路一轮 ~500ms, 即 ~19s 以上)
      [5]=1 且端口块里 link 全 0 -> link_mask 每轮开头清 0、轮末才回写,
                            所以"全 0"=【上一整轮一个模块都没应答】, 不是
                            某台盒子离线。至于是"粘住"还是"真故障", 看问=N
      [5]=1 但端口块各端口都干净 -> 与采集盒无关, 回头查 brk/cut 那几位
      [12]>0           ->  IO 模块通信故障, 同一行的 cut/brk 不可信

    uptime (帧尾 +1..+2, 上电至今秒数) 是最直接的"板子有没有在复位"判据:
      同一块板的【两帧之间这个值变小】= 板子重启过 (18.2 小时=65536s 回绕一次,
      回绕表现为"从 ~65535 掉到 0 附近", 与复位的"掉到很小的值"要区分看)。
      为什么重要: 端口任务要等配置从网口下发完才开始跑, 而 uart_setup_header()
      在任务启动时把 link_mask 清 0 —— 所以【每次复位都会重演一遍"所有端口
      全掉线"的报警】, 看着像总线故障, 其实只是刚上电。本工具检测到变小时
      会直接打一行 <-- 板子复位过。

用法:
    python udp_status_log_listen.py                 # 监听 0.0.0.0:50000
    python udp_status_log_listen.py -a              # 每一帧都打印
    python udp_status_log_listen.py --log run.txt   # 同时存文件

说明:
    - 只依赖标准库, Python 3.6+。
    - 板子从 v1.1.0.36 起【只在报警时才发】(见 task_rs485_7.c 的
      DBG_LOG_ALARM_ONLY), 常态静默。所以收到一帧 = 这一刻在报警,
      帧与帧之间的空档 = 已恢复, 时间戳差就是报警间隔。
    - 默认【只在内容变化时打印】, 用来把一次报警里连续几帧相同内容折叠成
      一行, 看"谁在报警 + 间隔"最清楚; 想量报警持续了多久就加 -a/--all,
      连续几帧 × 轮询周期(约 0.5s) 就是持续时长。
    - 端口行里的 bit 位序与 link_mask 一致 (bit0 = 第 1 个模块)，
      与上位机"在线"显示用的是同一个位。上位机若按 1 起编号, 记得减 1。
    - 长度不对或帧头不对的包会单独提示, 便于发现收发两端版本不一致。
"""

import argparse
import socket
import sys
import time

HEADER = b"\x00\xa6\x6a\x88\x99"
STATES = 9                     # [5..13]
HEAD_LEN = 15                  # 帧头5 + 状态量9 + port_cnt 1
BLOCK_LEN = 13                 # 每个端口块
TAIL_FIXED_LEN = 4             # v41 帧尾定长部分: 错误码1 + 上电秒数2 + 复位码1
TAIL_FIXED_LEN_V39 = 3         # v39/v40 帧尾定长部分: 错误码1 + 上电秒数2 (无复位码)
TAIL_V2_LEN = 1                # v2 帧尾: 只有错误码
FRAME_V1_LEN = 14              # 最老的定长帧, 没有端口块
MAX_PORTS = 6

# ---- 第二类帧: OTA 任务存活诊断帧 (v42 临时, 见 task_rs485_7.c 的
#      dbg_ota_diag_send)。magic 末字节 9A(状态帧是 99), 定长 25 字节,
#      与状态帧共用 socket / 目的地址, 收端按 magic 分流。 ----
OTA_MAGIC = b"\x00\xa6\x6a\x88\x9a"
OTA_FRAME_LEN = 25

# ota_client_stage() 的取值 (表见 ota_client.c / ota_client.h)
STAGE_NAME = {
    0:  "还没进主循环(刚创建/堆不足没起来)",
    1:  "循环顶(不在任何阻塞调用里)",
    2:  "等重连间隔",
    3:  "connect 中",
    4:  "发 HELLO",
    5:  "会话-上报存活",
    6:  "会话-取帧/分发(Flash 擦写在这里)",
    7:  "会话-收网络",
    8:  "会话-周期 HELLO",
    9:  "断开连接",
    10: "升级完成等重启",
}
SESS_NAME = {0: "IDLE", 1: "DOWNLOADING", 2: "DONE"}

# eMBErrorCode -> 人话 (完整表见 mbmaster/common/mbtypes.h)
ERR_NAME = {
    0:  "OK",
    1:  "ENOREG(非法寄存器地址)",
    2:  "EINVAL(非法参数)",
    3:  "EPORTERR(应答帧坏/CRC错)",
    4:  "ENORES(资源不足)",
    5:  "EIO(IO错)",
    6:  "EILLSTATE(协议栈状态异常)",
    7:  "EAGAIN(需重试)",
    8:  "ETIMEDOUT(从机没应答)",
}

# WTD_RESET -> 人话 (表见 Doc/通信主机嵌入式软件-详细设计说明书-草稿.md 与
# User/main.c 的 App_TaskStart; 21-26 的码-20 就是卡住的那路端口号)
RESET_NAME = {
    0:  "无",
    1:  "配置变更",
    2:  "上位机软复位命令",
    21: "串口任务卡死(端口1)",
    22: "串口任务卡死(端口2)",
    23: "串口任务卡死(端口3)",
    24: "串口任务卡死(端口4)",
    25: "串口任务卡死(端口5)",
    26: "串口任务卡死(端口6)",
    27: "心跳超时(上位机长时间无应答)",
    28: "TCP发送任务卡死",
    29: "申请配置帧发送失败",
    30: "日志帧发送失败",
    31: "OTA任务卡死",
    # ---- 下面这些码 v40 起不会再出现, 留给 v39 及更早的固件/旧抓包 ----
    19: "端口数据帧发送失败(Custom, v39及以前)",
    20: "端口数据帧发送失败(端口1, v39及以前)",
    32: "端口数据帧发送失败(端口2, v39及以前)",
    38: "端口数据帧发送失败(端口3, v39及以前)",
    44: "端口数据帧发送失败(端口4, v39及以前)",
    50: "端口数据帧发送失败(端口5, v39及以前)",
}


def err_name(e):
    return ERR_NAME.get(e, "EX/未知(%d)" % e)


def reset_name(c):
    if c in RESET_NAME:
        return RESET_NAME[c]
    if c >= 3 and c <= 9 and (c % 2) == 1:
        return "曲线上传失败(分支%d, v39及以前)" % ((c - 3) // 2)
    return "未知(%d)" % c


def le32(data, off):
    """小端 u32"""
    return (data[off] | (data[off + 1] << 8) |
            (data[off + 2] << 16) | (data[off + 3] << 24))


def decode(data):
    """合法返回 (states, ports, err, uptime, wdt, polled), 否则返回 None

    states: 9 个状态量组成的 list
    ports:  [(port_idx, link_mask, expected_mask, kgl_alarm_mask), ...]
    err:    帧尾的 s_slave_last_err, 老版本帧没有则为 None
    uptime: 上电至今秒数 (v39 起才有, 否则 None)
    wdt:    帧尾的 WTD_RESET 看门狗复位码 (v41 起才有, 否则 None)
    polled: [每端口"本轮实际发起请求的模块数"] (v39 起才有, 否则 None),
            下标与 ports 一一对应
    """
    if len(data) < FRAME_V1_LEN or data[:5] != HEADER:
        return None
    states = [data[i] for i in range(5, 5 + STATES)]
    if len(data) == FRAME_V1_LEN:
        return states, [], None, None, None, None   # 最老的定长帧
    if len(data) < HEAD_LEN:
        return None
    n = data[14]
    if n > MAX_PORTS:
        return None

    # 帧尾是变长的: v41/v42 定长4(+复位码) + 每端口1; v39/v40 定长3 + 每端口1;
    # v2 定长1; 更老的没有帧尾。四种总长互不相等(见 task_rs485_7.c 的帧布局
    # 注释), 不会误判 —— 所以新旧固件的抓包可以一起丢进来。
    base = HEAD_LEN + n * BLOCK_LEN
    uptime = None
    wdt = None
    polled = None
    if len(data) == base + TAIL_FIXED_LEN + n:              # v41/v42
        err    = data[base]
        uptime = data[base + 1] | (data[base + 2] << 8)
        wdt    = data[base + 3]
        polled = [data[base + TAIL_FIXED_LEN + k] for k in range(n)]
    elif len(data) == base + TAIL_FIXED_LEN_V39 + n:        # v39
        err    = data[base]
        uptime = data[base + 1] | (data[base + 2] << 8)
        polled = [data[base + TAIL_FIXED_LEN_V39 + k] for k in range(n)]
    elif len(data) == base + TAIL_V2_LEN:
        err = data[base]
    elif len(data) == base:
        err = None                              # 无帧尾的 v2 帧
    else:
        return None

    ports = []
    for k in range(n):
        o = HEAD_LEN + k * BLOCK_LEN
        ports.append((data[o], le32(data, o + 1),
                      le32(data, o + 5), le32(data, o + 9)))
    return states, ports, err, uptime, wdt, polled


def decode_ota(data):
    """OTA 诊断帧 -> dict, 否则 None

    字段见 task_rs485_7.c 的 dbg_ota_diag_send() 帧布局注释。
    left_s: 距"判 OTA 任务卡死"还剩几秒; 0 = 已到点(马上就要复位),
            0xFFFF = 任务还没 arm(看门狗不判超时)。
    """
    if len(data) != OTA_FRAME_LEN or data[:5] != OTA_MAGIC:
        return None
    return {
        "uptime": data[5] | (data[6] << 8),
        "stage":  data[7],
        "sock":   data[8],
        "sess":   data[9],
        "left":   data[10] | (data[11] << 8),
        "report": le32(data, 12),
        "loop":   le32(data, 16),
        "test1":  le32(data, 20),
        "wdt":    data[24],
    }


def _stage_name(s):
    return STAGE_NAME.get(s, "未知(%d)" % s)


def ota_summary(f):
    """诊断帧的一行人话"""
    if f["left"] == 0xFFFF:
        left = "未接管(不判超时)"
    elif f["left"] == 0:
        left = "已到点!要复位了"
    else:
        left = "还剩%us" % f["left"]
    return ("[OTA诊断] 上电=%us stage=%u(%s) sock=%u sess=%s %s "
            "report=%u loop=%u c1=%u WTD_RESET=%u(%s)" % (
                f["uptime"], f["stage"], _stage_name(f["stage"]), f["sock"],
                SESS_NAME.get(f["sess"], "?"), left,
                f["report"], f["loop"], f["test1"],
                f["wdt"], reset_name(f["wdt"])))


def ota_verdict(f, frozen, c1_frozen):
    """把这一帧翻译成结论; 没有异常就返回 None

    frozen: 与上一帧相比 report+loop 都没涨 —— 任务没在跑
    """
    if f["wdt"]:
        return ("!! 板子即将复位: 复位码=%u(%s) !!"
                % (f["wdt"], reset_name(f["wdt"])))

    if frozen:
        if f["stage"] == 1:
            note = ("!! OTA 任务被饿死: 停在循环顶、计数器不涨 —— "
                    "它是全系统最低优先级(prio 4), CPU 被 prio>=5 占满了")
            if c1_frozen:
                note += "; 且 prio1 任务也不动 -> 整机都在忙, 不是单个任务的事"
            return note
        if f["stage"] == 0:
            return ("!! OTA 任务根本没跑起来 (stage=0): 查 xTaskCreate 是否"
                    "因堆不足失败(它拿 4096B, configTOTAL_HEAP_SIZE 只有 28KB)")
        if f["left"] != 0xFFFF and f["left"] <= 3:
            return ("!! OTA 任务卡在 stage=%u(%s) 出不来了 "
                    "(倒计时只剩 %us) -> 查这个调用里的超时/阻塞"
                    % (f["stage"], _stage_name(f["stage"]), f["left"]))
        # 还剩不少时间: 擦除/校验期间是按扇区上报的, 帧间隔 500ms
        # 本来就可能看不到它涨, 不算异常
        if f["stage"] in (5, 6, 7):
            return None
        return ("?? OTA 任务卡在 stage=%u(%s) 里 (还剩 %us), 继续观察: "
                "再连续几帧不涨就是真卡死"
                % (f["stage"], _stage_name(f["stage"]), f["left"]))

    if f["left"] != 0xFFFF and f["left"] <= 3:
        return ("!! OTA 看门狗还剩 %us 就判卡死 (但任务还在上报): "
                "上报的不是 kick? 窗口对不上" % f["left"])
    return None


def _hx(v, ref):
    """按参考值大小选十六进制宽度, 免得 3 个模块也打成 00000003"""
    if ref > 0xFFFF:
        w = 8
    elif ref > 0xFF:
        w = 4
    else:
        w = 2
    return "%0*X" % (w, v)


def _bits(mask):
    """位图 -> 'bit0,bit3' 形式"""
    return ",".join("bit%d" % b for b in range(32) if (mask >> b) & 1)


def port_lines(ports, polled=None, uptime=None):
    """把端口块渲染成缩进行, 报警的端口标 <--

    uptime: 用来区分"刚上电还没跑完首轮"和"粘住" —— 两者都长成
            link 全 0 且 问=0, 靠上电秒数分。
    """
    out = []
    for k, (idx, link, exp, kgl) in enumerate(ports):
        ref = exp | link
        miss = exp & ~link           # 应有但没有 -> 离线
        bad = []
        if miss:
            bad.append("离线:%s" % _bits(miss))
        if kgl:
            bad.append("KGL:%s" % _bits(kgl))
        line = "    port%-3u link=%s exp=%s  %s" % (
            idx, _hx(link, ref), _hx(exp, ref),
            "  ".join(bad) if bad else "ok")
        if polled is not None and k < len(polled):
            p = polled[k]
            line += "  问=%u" % p
            if miss and p == 0:
                # 问=0 有两种成因, 靠上电秒数分:
                #   刚上电  -> 端口任务要等配置下发才开始轮询 (v42 及更早)
                #   跑了一阵 -> 该端口所有模块都进了跳过态(粘住)
                if uptime is not None and uptime < 15:
                    line += "(刚上电/首轮还没跑完, v42及更早才有的瞬态)"
                else:
                    line += "(跳过态/粘住)"
        if bad:
            line += "   <--"
        out.append(line)
    return out


def hint(v, ports, err, polled=None, uptime=None):
    """按帧内容给一句人话结论"""
    local, remote, cut, brk, buz, light, work, io_fail, s7_fail = v
    notes = []

    # ---- 刚上电的报警是最容易被当成故障的一种, 先声明 ----
    # v42 及更早: 端口任务启动时会把 link_mask 清 0, 要等配置下发 + 走完一整轮
    # 才重新发布, 所以【每次复位】都会重演一遍"所有端口离线", 看着像总线故障。
    # v43 起 link_mask 初值改成"全在线"(乐观初值), 上电这个假报警没有了;
    # v44 起离线判决又改成"静默 ≥2s"(不再是连续 5 轮), 起步还有 2s 宽限;
    # v45 把发布时机收回"每轮末尾一次" —— 都与上电瞬态无关。
    # 所以本提示在新固件上基本不会再触发, 一旦触发要认真当故障看。
    if uptime is not None and uptime < 15 and (local or any(
            (exp & ~link) for _, link, exp, _ in ports)):
        notes.append("注意: 上电才 %us 就报离线。v42 及更早这是【刚上电】的正常瞬态"
                     "(端口任务等配置下发才开始跑, 首轮跑完前 link_mask 一直是 0); "
                     "v43 及以后不该再出现, 出现了就当真故障查这一路" % uptime)

    if remote:
        if s7_fail == 0:
            notes.append("R=1: FC03读成功返回的 -> 从机板真报警")
        elif err == 8:
            notes.append("R=1: FC03超时(从机没应答) -> addr1 那侧: "
                         "从机栈没跑/地址不是1/收发器或方向控制")
        elif err == 3:
            notes.append("R=1: FC03收到坏帧(CRC错) -> 方向控制时序/总线冲突/"
                         "干扰/两端波特率不一致")
        else:
            notes.append("R=1: FC03失败(连续%u次, err=%s) -> 第7路链路"
                         % (s7_fail, err_name(err) if err is not None else "?"))
    if local:
        if any((exp & ~link) or kgl for _, link, exp, kgl in ports):
            notes.append("L=1: 见下面标 <-- 的端口")
        elif not ports:
            notes.append("L=1 但本帧没有端口块 (port_cnt=0) -> 该板一个采集盒"
                         "端口都没配, L 却为 1, 先确认两端版本/配置是否一致")
        elif not any(link for _, link, _, _ in ports):
            # 全 0: 再问一句"这一轮到底发过请求没有", 这才是粘住/真故障之分
            if polled is not None and all(p == 0 for p in polled) and polled:
                notes.append("L=1 且所有端口 link 全 0、本轮又一台都没问 -> "
                             "全部端口都处于跳过态(粘住), 不是新故障: "
                             "掩码要等下一次探测(该端口约 10 轮)才会恢复")
            else:
                notes.append("L=1 且所有端口 link 全 0 -> 上一整轮一个模块"
                             "都没应答 (link_mask 每轮开头清 0、轮末才回写), "
                             "不是某台盒子离线")
        else:
            notes.append("L=1 但各端口都干净 -> 不是采集盒, 查 brk/cut")
    if io_fail:
        notes.append("IO模块通信故障 -> 本行 cut/brk 不可信")
    return "  ".join(notes)


def summary(v, err, uptime=None, wdt=None):
    line = ("L=%u R=%u cut=%u brk=%u buz=%u light=%u "
            "work=%u io_fail=%u s7_fail=%u" % tuple(v))
    if err is not None:
        line += " s7_err=%s" % err_name(err)
    if uptime is not None:
        line += " 上电=%us(%.1fh)" % (uptime, uptime / 3600.0)
    if wdt is not None:
        line += " WTD_RESET=%u(%s)" % (wdt, reset_name(wdt))
    return line


def main():
    ap = argparse.ArgumentParser(
        description="Receive the comm board's status log (UDP).")
    ap.add_argument("-b", "--bind", default="0.0.0.0",
                    help="local address to bind (default: 0.0.0.0)")
    ap.add_argument("-p", "--port", type=int, default=50000,
                    help="UDP port to listen on (default: 50000)")
    ap.add_argument("-a", "--all", action="store_true",
                    help="print every frame (default: only when content changes)")
    ap.add_argument("--log", default=None,
                    help="also append received lines to this file")
    args = ap.parse_args()

    # 控制台默认是 GBK(cp936): 万一打印内容里混进一个 GBK 没有的字符(如 ⚠),
    # print 会抛 UnicodeEncodeError 把整个监听程序打断 —— 而它本来是要连续跑
    # 几小时盯板子的。改成编不出来的字符用 '?' 代替, 绝不因此中断。
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

    log_fp = None
    if args.log:
        try:
            log_fp = open(args.log, "a", encoding="utf-8")
        except OSError as e:
            print("cannot open log file %s: %s" % (args.log, e))
            return 1

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        print("bind %s:%d failed: %s" % (args.bind, args.port, e))
        print("(port in use? firewall? or wrong --port)")
        return 1

    print("listening UDP %s:%d ... (Ctrl+C to quit)" % (args.bind, args.port))
    if not args.all:
        print("only printing on change; use -a to print every frame")
    if log_fp:
        print("also logging to %s" % args.log)
    sys.stdout.flush()

    last = {}
    last_up = {}          # 每块板上一帧的"上电秒数", 用来判复位
    last_wdt = {}         # 每块板上一帧的 WTD_RESET, 用来给复位点名
    last_ota = {}         # 每块板上一帧的诊断帧 key, 用来变化抑制
    last_ota_cnt = {}     # 每块板上一帧的诊断帧计数器 (report, loop, test1)
    try:
        while True:
            data, addr = sock.recvfrom(2048)
            v = decode(data)
            stamp = time.strftime("%H:%M:%S")
            ip = addr[0]

            # ---- 先看是不是 OTA 诊断帧 (magic 末字节 9A), 是就单独处理 ----
            fo = decode_ota(data)
            if fo is not None:
                prev_cnt = last_ota_cnt.get(ip)
                frozen = c1_frozen = False
                if prev_cnt is not None:
                    p_rep, p_loop, p_c1 = prev_cnt
                    frozen = (fo["report"] == p_rep and fo["loop"] == p_loop)
                    c1_frozen = (fo["test1"] == p_c1)
                last_ota_cnt[ip] = (fo["report"], fo["loop"], fo["test1"])

                key = (fo["stage"], fo["sock"], fo["sess"], fo["wdt"],
                       frozen, c1_frozen)
                note = ota_verdict(fo, frozen, c1_frozen)
                # 计数器不动本身就是信息, 所以要能穿透变化抑制打出来
                if (not args.all and last_ota.get(ip) == key
                        and not frozen and not fo["wdt"]):
                    continue
                last_ota[ip] = key

                head = "%s  %-15s  %s" % (stamp, ip, ota_summary(fo))
                if note:
                    head += "   <-- " + note
                print(head)
                sys.stdout.flush()
                if log_fp:
                    log_fp.write("%s  %s\n"
                                 % (time.strftime("%Y-%m-%d %H:%M:%S"), head))
                    log_fp.flush()
                continue

            if v is None:
                print("%s  %-15s     <bad frame: %d bytes, %s>" % (
                    stamp, ip, len(data), data[:5].hex(" ")))
                sys.stdout.flush()
                continue

            states, ports, err, uptime, wdt, polled = v
            # ---- 复位检测: 同一块板的上电秒数变小 = 板子重启过 ----
            # ⚠ 端口任务要等配置从网口下发完才开始跑, 而任务启动时会把
            #   link_mask 清 0 —— 所以【每次复位都会重演一遍"所有端口全掉线"
            #   的报警】, 看着像总线故障, 其实只是刚上电。这一行就是用来把
            #   两者分开的。
            reset_note = None
            if uptime is not None:
                up_prev = last_up.get(ip)
                if up_prev is not None and uptime < up_prev:
                    # 18.2h(65536s)回绕也表现为变小, 用"旧值很大、新值很小"区分
                    if up_prev > 60000 and uptime < 6000:
                        reset_note = ("上电秒数回绕 %us -> %us (板子已连续跑了"
                                      "约 18h, 不是复位)" % (up_prev, uptime))
                    else:
                        reset_note = ("板子复位过! 上电秒数 %us -> %us "
                                      "(本帧的端口全掉线很可能就是它造成的)"
                                      % (up_prev, uptime))
                        # 复位码是普通 RAM, 复位后就被清零 —— 值只可能出现在
                        # 【复位前那一帧】里, 所以拿上一帧存的码来点名。
                        w_prev = last_wdt.get(ip)
                        if w_prev:
                            reset_note += ("  ← 复位前最后一帧的复位码 = %u(%s)"
                                           % (w_prev, reset_name(w_prev)))
                        elif w_prev == 0:
                            reset_note += ("  ← 复位前最后一帧复位码是 0, "
                                           "说明复位不是软件喂狗引起的"
                                           "(掉电/看门狗芯片自己超时/手动复位)")
                last_up[ip] = uptime
            if wdt is not None:
                last_wdt[ip] = wdt

            # ⚠ 上电秒数不进变化抑制的 key: 它每帧都在变, 放进去就等于关掉抑制
            key = (ip, tuple(states), tuple(ports), err, wdt,
                   tuple(polled) if polled is not None else None)
            if (not args.all and last.get(ip) == key
                    and reset_note is None and not wdt):
                continue
            last[ip] = key

            head = "%s  %-15s  %s" % (stamp, ip,
                                      summary(states, err, uptime, wdt))
            note = hint(states, ports, err, polled, uptime)
            if wdt:
                # 这一帧之后板子就会自己复位(SP706S 兜底, 约 1.6s), 所以要喊出来
                note = ("!! 板子即将复位: 复位码=%u(%s) !!"
                        % (wdt, reset_name(wdt)) +
                        ("  " + note if note else ""))
            if reset_note:
                note = (reset_note + "  " + note) if note else reset_note
            if note:
                head += "   <-- " + note

            lines = [head] + port_lines(ports, polled, uptime)
            for ln in lines:
                print(ln)
            sys.stdout.flush()

            if log_fp:
                full = time.strftime("%Y-%m-%d %H:%M:%S")
                for i, ln in enumerate(lines):
                    log_fp.write(("%s  %s\n" % (full, ln)) if i == 0
                                 else (ln + "\n"))
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
