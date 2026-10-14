/**
  ******************************************************************************
  * @file    task_rs485_7.c
  * @author  fire
  * @version V2.0
  * @date    2026-08-03
  * @brief   第7路RS485 串口任务 (USART3, 物理端口索引6)
  *          根据拨码开关配置运行于 Modbus 主机或从机模式
  *
  *  主机模式业务逻辑:
  *    1. 计算本板报警状态 (基于6路端口的 ALARM 变量)
  *    2. FC 03 读取从机通信板报警状态 (Modbus地址1), 读失败即报警
  *    3. FC 02 读取IO联网模块 DI0(切断开关)/DI1(断接开关) (Modbus地址4)
  *       超过 15s 读不到 DI 就丢弃过期的 cut/break, 清掉由 DI 导出的报警
  *       (time_ok 保护, 参考文档 5.4; 工作灯不受影响)
  *    4. 根据报警真值表计算 蜂鸣器/报警灯/工作灯 状态
  *    5. FC 05 写入IO联网模块 DO0(工作灯)/DO1(蜂鸣器)/DO2(报警灯)
  *    6. 写完 DO 后把全部状态以变长帧 UDP 上报上位机 (15+13*N+1 字节)
  *       (192.168.1.88:50000, 开关 g_rs485_7_log_udp_enable, 默认开;
  *        默认只在报警时才真的发, 常态静默 —— 见 DBG_LOG_ALARM_ONLY)
  *
  *  从机模式业务逻辑:
  *    1. 计算本板报警状态
  *    2. FC 03/04 回调中暴露报警状态给主机查询
  *    3. 同样以变长帧 UDP 上报本板报警 (限流 500ms, 同样只在
  *       报警时才发), 上位机按源 IP 区分是哪块板发来的
  *
  *  报警真值表 (来源: Doc/和串联io联网模块的通信概述参考.md):
  *    蜂鸣器(DO1)  = (alarm || break) XOR cut
  *    报警灯(DO2)  = alarm || break || cut
  *    工作灯(DO0)  = 通信板和上位机之间的 TCP 连接正常
  *    注: 参考文档中这一位是 time_ok —— "15s 内收到过对端状态更新".
  *        本板的对端是上位机服务器(TCP). 现场要求: 只要 TCP 连接未断开
  *        工作灯就点亮, 断链立即熄灭, 不叠加心跳新鲜度判决.
  *        早期实现曾误用"与 IO 联网模块之间的通信状态", 已修正.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "task_rs485_7.h"
#include "./dip_switch/bsp_dip_switch.h"
#include "./led/bsp_led.h"
#include "bsp_rs485.h"
#include "client.h"     /* client_is_connected(): 与上位机的 TCP 连接状态 */
#include "ota_client.h" /* 诊断: OTA 任务此刻停在哪一步 (v42 临时) */
#include "ota_wdt.h"    /* 诊断: OTA 任务看门狗的累计上报次数 / 剩余时间 */
#include "FreeRTOS.h"
#include "task.h"
#include "mbm.h"    /* Modbus Master API */
#include "mbs.h"    /* Modbus Slave API */
#include "lwip/sockets.h"   /* 网络调试输出: UDP */
#include "lwip/inet.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 诊断 (v42 临时): prio 1 的 Test1_Task 累计圈数, 定义在 User/main.c。
 * 本任务读它来判断"CPU 还有没有空闲" —— 见 dbg_ota_diag_send()。 */
extern volatile uint32_t g_test1_cnt;

/* ============================================================================
 *  网络调试输出 (没有调试串口时, 用 UDP 把运行信息发到上位机)
 *  ---------------------------------------------------------------------------
 *  背景: 调试串口 USART6 与 RS485 第5路共用外设, 现场常常不方便开;
 *        这里把 [RS485-7] 的调试信息同时以 UDP 报文发到上位机, 用网口观察.
 *
 *  使用: 上位机(调试接收端)在本机 UDP 端口 DBG_NET_PORT 上监听即可,
 *        例如 Python:
 *            import socket
 *            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
 *            s.bind(("0.0.0.0", DBG_NET_PORT))
 *            while True: print(s.recvfrom(512)[0].decode(errors="replace"))
 *
 *  说明: 只有 RS485-7 这一个任务调用, 所以静态 socket + 静态缓冲即可,
 *        不需要加锁. socket 惰性创建, 失败(或发送失败)后下次调用重建.
 * ==========================================================================*/
#define DBG_NET_ENABLE   0
#define DBG_NET_IP       "192.168.1.100"   /* 上位机(调试接收端) IP */
#define DBG_NET_PORT     39999U            /* UDP 端口, 上位机在此端口收
                                             * (6000 被 NI 服务 nidmsrv/lktsrv 占用, 改 39999) */

#if DBG_NET_ENABLE
static int                s_dbg_sock = -1;
static struct sockaddr_in s_dbg_addr;
static char               s_dbg_buf[192];

static void dbg_net_open(void)
{
    if (s_dbg_sock >= 0) {
        return;
    }
    s_dbg_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_dbg_sock < 0) {
        return;   /* 创建失败, 下次调用再试 */
    }
    memset(&s_dbg_addr, 0, sizeof(s_dbg_addr));
    s_dbg_addr.sin_family      = AF_INET;
    s_dbg_addr.sin_port        = htons(DBG_NET_PORT);
    s_dbg_addr.sin_addr.s_addr = inet_addr(DBG_NET_IP);
}

/* printf 风格的网络调试输出: 先格式化到静态缓冲, 再同时走串口和 UDP */
static void dbg_net_printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(s_dbg_buf, sizeof(s_dbg_buf), fmt, ap);
    va_end(ap);
    if (n <= 0) {
        return;
    }
    if ((size_t)n >= sizeof(s_dbg_buf)) {
        n = (int)(sizeof(s_dbg_buf) - 1U);   /* 被截断: 只发缓冲里的部分 */
    }

    /* 串口仍然照发 (调试串口打开时可见; 关闭时是空实现, 无副作用) */
    printf("%s", s_dbg_buf);

    /* 网络发送 */
    dbg_net_open();
    if (s_dbg_sock >= 0) {
        if (sendto(s_dbg_sock, s_dbg_buf, (size_t)n, 0,
                   (struct sockaddr *)&s_dbg_addr, sizeof(s_dbg_addr)) < 0) {
            /* 发送失败: 关掉, 下次调用重建 */
            (void)closesocket(s_dbg_sock);
            s_dbg_sock = -1;
        }
    }
}
#else
#define dbg_net_printf(...)   printf(__VA_ARGS__)
#endif

/* ============================================================================
 *  私有定义
 *  ==========================================================================*/
#define RS485_7_PORT_INDEX      6U     /* USART3 在 mbportserial 中的索引 */
#define RS485_7_SLAVE_ADDR      1U     /* 本板从机模式下的 Modbus 地址    */
#define RS485_7_TASK_PRIO       5      /* 与6路UART同级, 不可调低(否则被饿死) */
#define RS485_7_TASK_STACK      512    /* 栈大小 (words); 网络调试(vsnprintf+UDP)后加大 */
#define RS485_7_POLL_DELAY_MS   500    /* 主机轮询间隔 (ms)               */
#define RS485_7_INIT_RETRY_MAX        3U     /* 初始化失败快速重试次数     */
#define RS485_7_INIT_RETRY_DELAY_MS   200U   /* 快速重试间隔 (ms)          */
#define RS485_7_INIT_RECOVER_DELAY_MS 5000U  /* 之后慢速自恢复间隔 (ms)    */

/* ---- 从机通信板: 读失败即报警 (fail-safe) ----
 * 从机通信板是常驻设备, 现场不存在"总线上没有从机板"的情况, 因此不做在位探测:
 * 每轮 FC 03 读失败(超时/CRC错)直接置报警, 不区分"没接"和"坏了".
 * 例外: 开始轮询后的 RS485_7_SLAVE_GRACE_MS 内为启动宽限, 读失败不报警
 * (两块板同时上电, 从机板的 Modbus 从机栈可能还没进 eMBSPoll). */
#define RS485_7_SLAVE_GRACE_MS       3000U  /* 启动宽限时长 (ms), 从开始轮询算起 */

/* ---- IO联网模块 DI 读取超时 (time_ok 保护, 参考文档 5.4) ----
 * 超过这么久没有一次成功的 FC02 读 DI, 就认定"拿不到 DI 状态": 把过期的
 * cut/break 按 0 参与真值表, 即清掉由 DI 导出的报警, 不让最后一次读到的
 * break=1 一直响/亮下去.
 *
 * 文档 5.4 原文: "如果超过 15 秒没有收到进程A发来的状态报警数据包, 则认为
 * 通信超时, 此时: 工作灯熄灭(DO0=OFF) + 报警标志被强制清零(超时不报警,
 * 避免误报)". 本板只采用其中"清报警"一半 —— 工作灯按现场要求只看
 * client_is_connected(), 不跟随这条超时 (见 apply_alarm_truth_table)。
 *
 * ⚠ 现场要求把这个超时从文档的 15s 收到 5s: 15s 太长, 模块掉线后那组过期的
 *   cut/break 还要继续驱动蜂鸣器/报警灯十几秒. 收到 5s 后, IO 模块一断,
 *   ≈5s (实测 5~7s, 每次失败的 FC02 最多阻塞 2s) 内由 DI 导出的报警就被清掉。
 *   抖动余量: 单次 FC02 超时(2000ms)后下一轮成功读会立刻刷新时基, 偶发一次
 *   超时不会清报警; 连续两次读不到 (≈4~5s) 就属于真断了, 清掉是对的。
 *
 * 计时用 HAL_GetTick() 差值(无符号回绕安全), 与轮询周期长短无关. */
#define RS485_7_IO_DI_TIMEOUT_MS     5000U

/* ---- 工作灯(DO0): 与上位机的网络通信判据 ----
 * 工作灯反映"通信板 ←→ 上位机"的链路状态, 不是与 IO 联网模块的通信状态.
 * 唯一判据 = client_is_connected(), 即 TCP 连接本身是否已建立:
 *   连接建立 → 亮; 收到 FIN/RST 或链路超时导致断开 → 立即灭.
 * 这里不再叠加"心跳是否新鲜"的判决(见 apply_alarm_truth_table 说明). */

/* ============================================================================
 *  文件级静态变量
 *  ==========================================================================*/

/* —— 本板报警（主/从机通用）—— */
static uint8_t  s_local_board_alarm = 0;

/* —— 主机模式专用 —— */
static uint8_t  s_remote_board_alarm = 0;   /* 从机通信板报警状态  */
static uint32_t s_slave_grace_start  = 0;   /* 启动宽限计时起点 (HAL_GetTick, 进主机循环前打点) */
static uint8_t  s_io_cut_switch     = 0;    /* DI0: 切断开关       */
static uint8_t  s_io_break_switch   = 0;    /* DI1: 断接开关       */
static uint8_t  s_io_comm_fail_cnt  = 0;    /* IO模块FC02失败累加; 仅供调试打印,
                                             * 已不参与报警判决 (工作灯改判上位机网络) */
static uint32_t s_io_last_ok_tick   = 0;    /* 上次 FC02 成功读 DI 的时间戳
                                             * (HAL_GetTick), time_ok 超时的时基 */
static uint8_t  s_slave_read_fail_cnt = 0;  /* 第7路 FC03 读从机板失败累加; 仅供调试.
                                             * 用来区分 R=1 到底是"读失败"(此值>0)
                                             * 还是"读成功但从机板返回了1"(此值=0). */
static uint8_t  s_slave_last_err    = 0;    /* 最后一次 FC03 读从机板返回的 eMBErrorCode.
                                             * 0 = 成功; 8(MB_ETIMEDOUT) = 从机没应答;
                                             * 3(MB_EPORTERR) = 应答来了但帧坏(CRC/干扰).
                                             * 取值表见 mbmaster/common/mbtypes.h 的
                                             * eMBErrorCode. 每次读都刷新, 成功也清 0. */
static uint32_t s_slave_log_tick    = 0;    /* 从机模式状态日志限流计时 (HAL_GetTick) */
static uint8_t  s_work_light        = 0;    /* DO0: 工作灯         */
static uint8_t  s_buzzer            = 0;    /* DO1: 蜂鸣器         */
static uint8_t  s_alarm_light       = 0;    /* DO2: 报警灯         */

/* ============================================================================
 *  全局变量定义 (供外部模块读取串口7的IO状态)
 *  ==========================================================================*/
uint8_t g_rs485_7_local_alarm  = 0;
uint8_t g_rs485_7_remote_alarm = 0;
uint8_t g_rs485_7_io_cut       = 0;
uint8_t g_rs485_7_io_break     = 0;
uint8_t g_rs485_7_work_light   = 0;
uint8_t g_rs485_7_buzzer       = 0;
uint8_t g_rs485_7_alarm_light  = 0;

/* 状态日志上报总开关 (运行期可改): 0=关闭(不建socket/不发包), 1=开启 */
volatile uint8_t g_rs485_7_log_udp_enable = 1U;

/* 状态日志上报编译期总开关:
 *   1 = 打开 (调试用) —— 状态帧 + OTA 诊断帧 + 复位前抢发帧三样都发
 *   0 = 发布版关闭 —— 对应代码一行没删, 改回 1 重编即可整条恢复
 * 比运行期的 g_rs485_7_log_udp_enable 更彻底: 被调函数在入口就返回,
 * 连"要不要发"都不判断, UDP socket 也不会创建。
 * 关的只是这条上报通道本身; 报警判决/IO 读写/端口轮询一律不受影响。 */
#define DBG_LOG_ENABLE   0

/* ============================================================================
 *  状态日志上报 (UDP 定长二进制帧 → 上位机 192.168.1.88:50000)
 *  ---------------------------------------------------------------------------
 *  与上面的文本调试通道 (→ 192.168.1.100:39999) 完全独立:
 *    - 目的 IP / 端口都不同, 互不影响;
 *    - 那条受编译期开关 DBG_NET_ENABLE 控制, 这条受全局变量
 *      g_rs485_7_log_udp_enable 运行期控制, 默认开启.
 *
 *  ⚠ 现在这条还多了一层编译期总开关 DBG_LOG_ENABLE (定义在上面):
 *     两个开关是"与"的关系 —— DBG_LOG_ENABLE=0 时函数在入口就返回,
 *     连 g_rs485_7_log_udp_enable 都不看。发布版 DBG_LOG_ENABLE=0。
 *
 *  用途: 主板每写完一次 IO 模块 DO, 立刻把串口7的全部状态推给上位机,
 *        让上位机当普通日志收下来, 便于事后定位"到底是谁在报警".
 *        默认【只在报警时才真的发】(见 DBG_LOG_ALARM_ONLY), 常态静默 ——
 *        排查"蜂鸣器/报警灯偶尔响一下亮一下"这类瞬态时, 收到一帧就
 *        等于这一刻在报警, 帧间空档即已恢复, 时间戳直接给出持续时长.
 *
 *  帧格式 (v2 变长, 没有长度字段, 靠 UDP 报文长度定界):
 *    [0] 0x00  [1] 0xA6  [2] 0x6A  [3] 0x88  [4] 0x99   <- 帧头(同步字)
 *    [5]  s_local_board_alarm    本板报警 (维度1离线 或 维度2 KGL)
 *    [6]  s_remote_board_alarm   从机通信板报警 (第7路 FC03 读回)
 *    [7]  s_io_cut_switch        DI0 切断开关  (FC02 超时 15s 后强制 0)
 *    [8]  s_io_break_switch      DI1 断接开关  (FC02 超时 15s 后强制 0)
 *    [9]  s_buzzer               DO1 蜂鸣器 (真值表输出)
 *    [10] s_alarm_light          DO2 报警灯 (真值表输出)
 *    [11] s_work_light           DO0 工作灯 (上位机 TCP 连接状态)
 *    [12] s_io_comm_fail_cnt     IO模块 FC02 连续失败计数
 *    [13] s_slave_read_fail_cnt  第7路 FC03 读从机板 连续失败计数
 *    [14] port_cnt               后面跟的端口块个数 (0..6)
 *    [15..] 端口块 × port_cnt, 每块 13 字节 (布局见 DBG_LOG_FRAME_HEAD_LEN
 *           上面的注释): 只有配了采集盒的端口才在里面, 用来把"本板报警"
 *           拆到具体端口的具体模块上 —— expected 少了哪位就是哪位离线,
 *           kgl_alarm_mask 哪位是 1 就是哪位 KGL 报警。
 *    [尾]   4 字节定长 + port_cnt 字节变长 (布局见 DBG_LOG_TAIL_FIXED_LEN):
 *             +0   s_slave_last_err  最后一次 FC03 读从机板的 eMBErrorCode
 *             +1   ~ 上电至今秒数 (LE u16, 低 16 位)
 *             +3   WTD_RESET  看门狗复位码 (0 = 此刻没判要复位)
 *             +4.. 每端口"本轮实际发起请求的模块数", 与端口块一一对应
 *
 *  [尾+0] 这个错误码是给 R=1 分病用的 —— [13]>0 说明读失败了, 但"为什么失败"
 *  只有这个码说得清:
 *    8 (MB_ETIMEDOUT) ->  从机【根本没有应答】. 总线是好的(同总线的 IO 模块
 *                         [12]=0 就证明这一点), 问题在 addr 1 那一侧:
 *                         从机的 Modbus 从机栈没在跑 / 从机地址不是 1 /
 *                         从机收发器或方向控制不对.
 *    3 (MB_EPORTERR)  ->  【应答来了但帧坏了】(CRC 错/帧错). 方向控制时序、
 *                         总线冲突、干扰、两端波特率不一致都是这个码.
 *
 *  [尾+1] 为什么带上电秒数: 板子一旦复位, 所有端口的 link_mask 都会先回到 0
 *  (uart_setup_header() 在端口任务启动时把它清 0, 而端口任务要等配置从网口
 *   下发完才开始跑), 于是"整路全掉线"的报警会从头重演一遍。光看端口块分不出
 *   "板子刚重启"和"总线真故障" —— 但连续两帧之间这个秒数【变小】(回绕除外)
 *   就等于"板子复位过", 是最直接的判据 (UDP 直发, 不依赖上位机 TCP 链路)。
 *
 *  [尾+3] 为什么把看门狗复位码也带出来: WTD_RESET 是普通 RAM 变量, 复位后
 *  被初始化清零 —— 复位【之后】读不到, 只能复位【之前】送出去。而 TCP 那条
 *  路不可靠(要 tcp_connected, 还和控制槽共用覆盖式槽位), 调试串口又常接到
 *  RS485 第5路不方便。UDP 直发不依赖 TCP 链路, 所以:
 *    - 这个字节在每一帧里都有 (常态 0), 看门狗一置位, 下一帧就带出来了;
 *    - 另外 RS485-7 任务在循环里一旦发现 WTD_RESET != 0, 会【无视"只在报警
 *      时才发"的门槛】立刻补发一帧 (见 rs485_7_log_send_now), 抢在 SP706S
 *      复位那约 1.6s 窗口内把码送出去。
 *  复位码表: 1 配置变更, 2 上位机软复位, 21-26 串口任务卡死(码-20=端口号),
 *  27 心跳超时, 28 TCP发送任务卡死, 29 申请配置发送失败, 30 日志发送失败,
 *  31 OTA任务卡死。⚠ v40 起数据帧(实时/计轴/直流3线/曲线)不再带复位码,
 *  所以 19 / 20+port*6 / 3+n*2 这些码在新固件里不会再出现。
 *
 *  [尾+4..] 为什么带"实际发起请求的模块数": link_mask 全 0 有两种完全不同的
 *  成因, 光看掩码分不出:
 *    该字节 == 0  ->  这一轮压根没发请求 (该端口所有模块都在"连续失败≥3 次
 *                     后的跳过态"), 每 10 轮才探一次 → 是【粘住】, 不是新故障
 *    该字节 >  0  ->  真的问了这么多台, 却一台都没答 → 是【真故障】
 *                  (等于该端口 SlaveModuelNum 时表示本轮全量轮询)
 *
 *  [12][13] 这两个计数是给"报警灯常亮"定位用的:
 *    [13]==0 而 [6]==1  ->  FC03 读成功, 是【从机板真的返回了 1】,
 *                            去查从机板自己的采集盒 (维度1/维度2)
 *    [13]>0  而 [6]==1  ->  读失败, R 是 fail-safe 置的, 查第7路链路
 *    [12]>0              ->  IO 模块 DI 值不可信 (cut/brk 可能是故障值)
 *
 *  [5]==1 (本板报警) 光看字节本身只能知道"有采集盒报警", 分不出是离线
 *  还是 KGL、更不知道是哪个。往下看端口块:
 *    expected & ~link     非 0  ->  这几位模块【离线】(维度1)
 *    kgl_alarm_mask       非 0  ->  这几位模块【KGL 开关量报警】(维度2)
 *    两者都为 0 的端口就是干净的 —— 全是 0 说明 [5]=1 与采集盒无关,
 *    回头去看 brk/cut 那几位。
 *
 *  主/从两种模式都会发这条日志, 上位机按【源 IP】区分是哪块板:
 *    主机模式: 头 14 字节 (状态量) 全部有效, 发送节奏 = 主机轮询周期 (~500ms)
 *    从机模式: 只有 [5](本板报警) 和 [11](TCP连接) 有效, 其余全 0
 *              (从机不轮询IO模块也不驱动DO), 按 DBG_LOG_SLAVE_PERIOD_MS 限流;
 *              端口块照常有意义 —— 从机板自己也扫自己那6路采集盒
 *  两种模式都受 DBG_LOG_ALARM_ONLY 约束, 只有报警时才真的发出。
 *
 *  为什么用 UDP 不用 TCP: 本板在 TCP 里是客户端, 新建连接要带一整套
 *  connect/重连/保活状态机, 对端没监听时还会阻塞这个 500ms 周期的任务.
 *  而这条日志只是周期性快照, 丢一帧 500ms 后就有新的, 不需要可靠性.
 *  socket 上设了 SO_SNDTIMEO, 即使发送缓冲满也不会永久阻塞本任务.
 * ==========================================================================*/
#define DBG_LOG_IP            "192.168.1.88"    /* 上位机 IP (收状态日志) */
#define DBG_LOG_PORT          50000U           /* UDP 端口, 上位机在此收 */
#define DBG_LOG_SNDTIMEO_MS   100U             /* 单次 sendto 时间上限 (ms) */

/* ---- 帧布局 (v2, 变长) ----------------------------------------------------
 *   [0..4]   00 A6 6A 88 99   帧头同步字
 *   [5..13]  9 个状态量       (与 v1 完全一致, 见下面的帧格式注释)
 *   [14]     port_cnt         本帧后面跟了几个端口块 (0..DBG_LOG_PORT_MAX)
 *   [15..]   端口块 × port_cnt, 每块 DBG_LOG_PORT_BLOCK_LEN 字节:
 *              +0      port_idx        端口号 0..5 (第几路 RS485)
 *              +1..+4  link_mask       LE u32, 在线模块位图
 *              +5..+8  expected_mask   LE u32, 按 SlaveModuelNum 应有位图
 *              +9..+12 kgl_alarm_mask  LE u32, bit=1 → 该模块 KGL 报警
 *
 *   帧尾 (定长 4 字节 + 每端口 1 字节, 紧跟最后一个端口块之后):
 *     [len]     s_slave_last_err    最后 1 次 FC03 读从机板的 eMBErrorCode
 *     [len+1]   上电至今秒数 低字节  LE u16 (HAL_GetTick()/1000, 低 16 位)
 *     [len+2]   上电至今秒数 高字节
 *     [len+3]   WTD_RESET           看门狗复位码 (0 = 此刻没判要复位)
 *     [len+4..] 每端口"本轮实际发起请求的模块数" × port_cnt,
 *               顺序与端口块一一对应 (同一个 port_idx)
 *
 *   总长 = 15 + 13*port_cnt + 4 + port_cnt, 上限 15 + 13*6 + 4 + 6 = 103 字节。
 *
 *   只有【配了采集盒的端口】才会出现在端口块里, 所以 port_cnt 通常小于 6;
 *   一块都没有时 port_cnt=0, 帧长退化成 19 字节 (15 + 4 + 0)。
 *
 *   ⚠ 诊断量一律加在【帧尾】而不是插进帧头: 这样 [5..13] 状态量和
 *     [14] port_cnt / 端口块的位置永远不变, 已经按偏移量读这两段的工具
 *     不用跟着改; 已知端口块个数就能直接算出尾字节在哪。往后还要加
 *     诊断量 (比如 IO 模块 FC02 的错误码、原始 DI 字节) 也往尾巴上接。
 *
 *   为什么要带 expected_mask: 光看 link_mask 不知道"少了哪一位"——
 *   比如 link=00000003 可能是"配了2个模块都在线", 也可能是"配了4个掉了2个",
 *   带上应有位图就不用再回查上位机里那块板配了几个从机。
 *
 *   怎么用它区分两种报警维度:
 *     link_mask & ~expected_mask  -> 位图里有不该有的模块, 配置/数据错乱
 *     expected_mask & ~link_mask  -> 该位模块【离线】(维度1)
 *     kgl_alarm_mask              -> 该位模块有【KGL 开关量报警】(维度2)
 */
#define DBG_LOG_FRAME_HEAD_LEN  15U             /* 帧头5 + 状态量9 + port_cnt 1 */
#define DBG_LOG_PORT_MAX        UART_PORT_COUNT /* 采集盒端口数, 与轮询循环一致 */
#define DBG_LOG_PORT_BLOCK_LEN  13U             /* 每端口块字节数 */
#define DBG_LOG_TAIL_FIXED_LEN  4U              /* 帧尾定长部分: 错误码1 + 上电秒数2
                                                 * + 看门狗复位码1 */
#define DBG_LOG_TAIL_LEN        (DBG_LOG_TAIL_FIXED_LEN + DBG_LOG_PORT_MAX)
                                                /* 帧尾总长上限: 定长3 + 每端口1 */
#define DBG_LOG_FRAME_MAX       (DBG_LOG_FRAME_HEAD_LEN + \
                                 DBG_LOG_PORT_BLOCK_LEN * DBG_LOG_PORT_MAX + \
                                 DBG_LOG_TAIL_LEN)

#define DBG_LOG_SLAVE_PERIOD_MS 500U           /* 从机模式发送限流周期 (ms);
                                                * 从机循环是 10/50ms 一轮,
                                                * 不限流会发得比主机还密 */
/* 只在报警时上报 (1=常态静默, 只在报警条件成立时发; 0=每轮都发).
 *
 * 排查"蜂鸣器/报警灯偶尔响一下亮一下、1~2 秒自己恢复"这类瞬态时用 1:
 *   常态帧 2 帧/秒, 一块板一天就是十几万行, 而且真正有信息量的那 1~3 帧
 *   会被淹在里面; 常态静默之后, 收到一帧就等于"这一刻在报警",
 *   帧与帧之间的空档就是"已恢复", 时间戳直接给出报警持续时长和间隔。
 *
 * ⚠ 判据用的是【报警灯(DO2) 里除 cut 之外的那部分】, 不是 s_alarm_light:
 *   s_alarm_light = 报警 || 断接 || cut, 其中的 cut 是操作员按下的动作、
 *   不是故障, 按着不放会一直刷屏, 排查故障时它只是噪声。
 *   (蜂鸣器判断同理: buzzer = 报警 || 断接 ^ cut)
 *   要连切断开关一起看, 把下面改成 0 恢复成每轮都发即可。 */
#define DBG_LOG_ALARM_ONLY    1U

static int                s_log_sock = -1;
static struct sockaddr_in s_log_addr;
static uint8_t            s_log_frame[DBG_LOG_FRAME_MAX];

/* 端口报警快照 —— compute_local_board_alarm() 每轮扫描时顺手填,
 * dbg_log_udp_send() 取出来塞进帧尾巴。只为了把"到底哪个端口的哪个模块
 * 在报警"带出去, 不参与任何判决。
 * 写入方和读取方都是同一个 RS485-7 任务, 不需要加锁。 */
typedef struct {
    uint8_t  port_idx;       /* 端口号 0..5 */
    uint32_t link_mask;      /* 在线模块位图 */
    uint32_t expected_mask;  /* 应有模块位图 */
    uint32_t kgl_alarm_mask; /* bit=1 → 该模块 KGL 报警 */
    uint8_t  polled_cnt;     /* 上一轮该端口实际发起请求的模块数 (0=整路跳过态) */
} dbg_log_port_info_t;

static dbg_log_port_info_t s_log_port[DBG_LOG_PORT_MAX];
static uint8_t             s_log_port_cnt = 0;

static void dbg_log_udp_open(void)
{
    struct timeval tv;

    if (s_log_sock >= 0) {
        return;
    }
    s_log_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (s_log_sock < 0) {
        return;   /* 创建失败, 下次调用再试 */
    }

    /* 发送限时: 对端不可达时 lwIP 可能返回错误, 但不允许它把本任务挂住.
     * 生效前提: lwipopts.h 里 LWIP_SO_SNDTIMEO=1 (已确认). */
    tv.tv_sec  = DBG_LOG_SNDTIMEO_MS / 1000U;
    tv.tv_usec = (DBG_LOG_SNDTIMEO_MS % 1000U) * 1000U;
    (void)setsockopt(s_log_sock, SOL_SOCKET, SO_SNDTIMEO,
                     (const void *)&tv, sizeof(tv));

    memset(&s_log_addr, 0, sizeof(s_log_addr));
    s_log_addr.sin_family      = AF_INET;
    s_log_addr.sin_port        = htons(DBG_LOG_PORT);
    s_log_addr.sin_addr.s_addr = inet_addr(DBG_LOG_IP);
}

/**
 * @brief  把串口7当前全部状态打成变长帧, 发给上位机
 * @note   帧布局见 DBG_LOG_IP 上面的注释
 *         (v2: 15 字节固定头 + N 个端口块 + 帧尾诊断量).
 *         主机模式: 每次写完 IO 模块 DO 之后调用 (见主机任务循环步骤6.5).
 *         从机模式: 在从机循环里按 DBG_LOG_SLAVE_PERIOD_MS 限流调用.
 *         开关关闭时直接返回: 既不建 socket 也不发包, 无任何副作用.
 *         DBG_LOG_ALARM_ONLY 打开时只在报警条件下才真的发 (见该宏说明),
 *         常态静默 —— 排查瞬态报警用, 此时调用点每轮调进来也不算错。
 *         发送失败就关掉 socket, 下次调用重建 (与文本调试通道同样处理).
 *         主/从两种模式共用同一份静态 socket 和缓冲 (同一任务上下文,
 *         一个任务只会走其中一个分支, 不需要加锁).
 * @param  force  1 = 无视 DBG_LOG_ALARM_ONLY 门槛, 无条件发一帧 (看门狗判定
 *                要复位时抢发用, 见 rs485_7_log_send_now);
 *                0 = 按门槛走 (两个常规调用点).
 *                ⚠ g_rs485_7_log_udp_enable 这一层开关对 force 同样有效 ——
 *                  有人明确关掉了上报就不再发包, 复位码也一并放弃。
 */
static void dbg_log_udp_send_ex(uint8_t force)
{
#if !DBG_LOG_ENABLE
    /* 发布版: 整条上报通道编译期关闭 (见 DBG_LOG_ENABLE)。帧构造与发送的代码
     * 原样保留在下面, 只是这一版不编进去。
     * 下面这一串 (void) 不是逻辑, 只是"引用一下" —— 这些静态量和静态函数只被
     * 本上报通道使用, 一旦连引用都没有, 编译器会报 never-referenced 警告。 */
    (void)force;
    (void)s_log_frame;
    (void)s_log_port;
    (void)s_log_port_cnt;
    (void)s_slave_last_err;
    (void)dbg_log_udp_open;
    return;
#else
    uint16_t len;
    uint8_t  n;
    uint8_t  k;

    if (g_rs485_7_log_udp_enable == 0U) {
        return;
    }

    /* 报警门槛 —— 判据见 DBG_LOG_ALARM_ONLY 处: 取报警灯里除 cut(操作员
     * 动作)之外的那一部分。从机模式下 R/break 恒 0, 等价于只看本板报警。 */
    if ((force == 0U) && (DBG_LOG_ALARM_ONLY != 0U)) {
        uint8_t alarming = (uint8_t)((s_local_board_alarm  != 0U) ||
                                     (s_remote_board_alarm != 0U) ||
                                     (s_io_break_switch    != 0U));
        if (alarming == 0U) {
            return;
        }
    }

    /* 帧头: 固定同步字, 收端靠它对齐帧 */
    s_log_frame[0] = 0x00U;
    s_log_frame[1] = 0xA6U;
    s_log_frame[2] = 0x6AU;
    s_log_frame[3] = 0x88U;
    s_log_frame[4] = 0x99U;

    /* 负载: 每个状态量占 1 字节 */
    s_log_frame[5]  = s_local_board_alarm;
    s_log_frame[6]  = s_remote_board_alarm;
    s_log_frame[7]  = s_io_cut_switch;
    s_log_frame[8]  = s_io_break_switch;
    s_log_frame[9]  = s_buzzer;
    s_log_frame[10] = s_alarm_light;
    s_log_frame[11] = s_work_light;
    s_log_frame[12] = s_io_comm_fail_cnt;
    s_log_frame[13] = s_slave_read_fail_cnt;

    /* ---- 端口块: 每个配了采集盒的端口 13 字节 ----
     * 数据来自 compute_local_board_alarm() 刚填好的快照 s_log_port[],
     * 与本次判决用的是同一批 link_mask / ALARM_TEMP, 不会错位一拍。 */
    n = s_log_port_cnt;
    if (n > (uint8_t)DBG_LOG_PORT_MAX) {
        n = (uint8_t)DBG_LOG_PORT_MAX;   /* 防御: 理论上 cnt 不会超过端口数 */
    }
    s_log_frame[14] = n;

    for (k = 0U; k < n; k++) {
        uint8_t *p = &s_log_frame[DBG_LOG_FRAME_HEAD_LEN +
                                  (uint16_t)k * DBG_LOG_PORT_BLOCK_LEN];
        uint32_t mask;

        p[0] = s_log_port[k].port_idx;

        mask = s_log_port[k].link_mask;      /* 小端: 低字节在前, 与
                                              * SerialDataBuf[i][7..10] 的
                                              * 内存顺序一致, 便于和现有
                                              * 抓包结果直接对照 */
        p[1] = (uint8_t)( mask        & 0xFFU);
        p[2] = (uint8_t)((mask >>  8) & 0xFFU);
        p[3] = (uint8_t)((mask >> 16) & 0xFFU);
        p[4] = (uint8_t)((mask >> 24) & 0xFFU);

        mask = s_log_port[k].expected_mask;
        p[5] = (uint8_t)( mask        & 0xFFU);
        p[6] = (uint8_t)((mask >>  8) & 0xFFU);
        p[7] = (uint8_t)((mask >> 16) & 0xFFU);
        p[8] = (uint8_t)((mask >> 24) & 0xFFU);

        mask = s_log_port[k].kgl_alarm_mask;
        p[9]  = (uint8_t)( mask        & 0xFFU);
        p[10] = (uint8_t)((mask >>  8) & 0xFFU);
        p[11] = (uint8_t)((mask >> 16) & 0xFFU);
        p[12] = (uint8_t)((mask >> 24) & 0xFFU);
    }

    len = (uint16_t)(DBG_LOG_FRAME_HEAD_LEN +
                     (uint16_t)n * DBG_LOG_PORT_BLOCK_LEN);

    /* ---- 帧尾诊断量 ----
     * 定长 3 字节在前 (偏移不随 n 变), 之后是 n 个"每端口实际发起请求的
     * 模块数"。顺序与上面的端口块一一对应 (同一个 port_idx), 只有配了
     * 采集盒的端口才有对应字节 —— 与端口块的取舍完全一致。 */
    s_log_frame[len] = s_slave_last_err;      /* [len] FC03 错误码 */
    len++;

    {
        uint32_t up_s = HAL_GetTick() / 1000U;   /* 上电至今秒数, 取低 16 位
                                                  * (11.1h 回绕; 帧间【变小】
                                                  *  即板子复位过) */
        s_log_frame[len] = (uint8_t)( up_s        & 0xFFU);
        len++;
        s_log_frame[len] = (uint8_t)((up_s >>  8) & 0xFFU);
        len++;
    }

    /* 看门狗复位码: 复位后这个变量被初始化清零, 只能复位前送出去, 所以
     * 每帧都带 (常态 0); 看门狗一置位, 下一帧就能看到。帧是 UDP 直发,
     * 不需要 TCP 链路, 正是"板子要没了"这种时候唯一还发得出去的通道。 */
    s_log_frame[len] = WTD_RESET;
    len++;

    for (k = 0U; k < n; k++) {
        s_log_frame[len] = s_log_port[k].polled_cnt;
        len++;
    }

    dbg_log_udp_open();
    if (s_log_sock < 0) {
        return;
    }

    if (sendto(s_log_sock, s_log_frame, (size_t)len, 0,
               (struct sockaddr *)&s_log_addr, sizeof(s_log_addr)) < 0) {
        (void)closesocket(s_log_sock);
        s_log_sock = -1;
    }
#endif /* DBG_LOG_ENABLE */
}

/* ============================================================================
 *  dbg_log_udp_send  —  常规上报 (按 DBG_LOG_ALARM_ONLY 门槛)
 * ==========================================================================*/
static void dbg_log_udp_send(void)
{
    dbg_log_udp_send_ex(0U);
}

/* ============================================================================
 *  rs485_7_log_send_now  —  无条件立刻上报一帧 (看门狗复位前抢发)
 *  ---------------------------------------------------------------------------
 *  背景: 现场排查"板子每几十秒自己复位一次"时, 复位码是唯一能指认凶手的东西,
 *        但 WTD_RESET 复位后就没了, TCP 日志通道又要求 tcp_connected —— 复位
 *        往往就是链路出问题引起的, 那条路经常发不出去。
 *
 *  用法: 主/从机两个循环里各有一处 "WTD_RESET != 0 → 调本函数"。看门狗
 *        (App_TaskStart) 置位到 SP706S 复位约 1.6s, 而本循环周期 500ms
 *        (主) / 10~50ms (从), 所以总能在窗口内把带复位码的帧发出去。
 *        门槛绕过, 但 g_rs485_7_log_udp_enable 仍然有效。
 *
 *  ⚠ 故意做成 static: 上面的静态 socket / 帧缓冲是【本任务私有】的, 从别的
 *    任务调进来会和本任务自己的常规上报打架(缓冲被覆写、socket 被对方关掉)。
 *    要"一复位就发帧"只需要在本任务的循环里加一处判断, 不要跨任务调用。
 * ==========================================================================*/
static void rs485_7_log_send_now(void)
{
    dbg_log_udp_send_ex(1U);
}

/* ============================================================================
 *  dbg_ota_diag_send  —  OTA 任务存活诊断帧 (v42 临时, 只为查复位码 31)
 *  ---------------------------------------------------------------------------
 *  由来: v41 的帧尾已经能报出"复位码 31 = OTA 任务卡死", 现场一抓就抓到了 ——
 *        板子每隔约 36s 自己复位一次, 每次都是 31, 而同一帧里 6 路端口全部
 *        正常在线。也就是说: 系统其余部分都好, 只有 OTA 任务 30s 没上报存活。
 *
 *        但"30s 没上报"有两种完全不同的成因, 光看复位码分不开:
 *          (a) 卡在某个调用里出不来 (connect / send / recv / Flash 擦写);
 *          (b) 被饿死 —— 6 路 UART 和本 RS485-7 的 Modbus 轮询里 HAL_Delay()
 *              是纯自旋(不让出 CPU), 这几个 prio 5 任务几乎常驻就绪; 只要它们
 *              把 CPU 占满, 当时还在 prio 4 的 OTA 任务就一次都排不上, 现场
 *              表现和"卡住"一模一样。
 *
 *  结论 (2026-10-14): 实测就是 (b)。7 连帧里 report/loop/c1 三个计数器同时冻住、
 *        stage 恒 1、sock_open 恒 0 ⇒ 整个 OTA 任务拿不到 CPU, 不是卡在某个调用。
 *        处置: OTA 优先级 4 → 5 (出 v46), 与端口任务同台 1ms 轮转, 不再复位。
 *        ⚠ 根因(上面的自旋)未清, c1 是否恢复增长是后续复测的重点。
 *        本帧把 OTA 任务的"生死"搬出来, 与状态帧共用同一个 socket / 目的地址。
 *
 *  判读 (关键):
 *      report_cnt / loop_cnt 不涨 + stage=1(循环顶)  → 饿死, CPU 被 prio>=5 占满
 *      report_cnt / loop_cnt 不涨 + stage=2..10      → 卡在那个调用里
 *      report_cnt / loop_cnt 不涨 + stage=1 且 c1 也不涨 → 整机都被占满
 *      report_cnt 在涨但 left_s 却往下走          → 上报的不是 kick(窗口不同)
 *      c1 (Test1_Task, prio 1) 是"CPU 还有没有空闲"的旁证。
 *
 *  帧 (25 字节定长, magic 末字节 9A 与状态帧的 99 区分, 收端按 magic 分流):
 *      [0..4]   magic 00 A6 6A 88 9A
 *      [5..6]   上电秒数 LE u16
 *      [7]      stage        OTA 停在哪一步 (取值表见 ota_client.h)
 *      [8]      sock_open    1 = 当前有 TCP 连接
 *      [9]      sess         0=IDLE 1=DOWNLOADING 2=DONE
 *      [10..11] left_s       距"判卡死"还剩几秒 (0 = 已经到点; 0xFFFF = 还没 arm)
 *      [12..15] report_cnt   LE u32  OTA 看门狗累计上报次数 (只增)
 *      [16..19] loop_cnt     LE u32  OTA 主循环累计圈数 (只增)
 *      [20..23] test1_cnt    LE u32  prio1 任务累计圈数 (CPU 余量旁证)
 *      [24]     WTD_RESET
 *
 *  只是读几个 volatile + 一次 sendto, 本任务里不做任何等待, 送不出去也没有副作用。
 * ==========================================================================*/
static void dbg_ota_diag_send(void)
{
#if !DBG_LOG_ENABLE
    /* 发布版: 与状态帧同受 DBG_LOG_ENABLE 控制 (见该宏)。
     * 这一帧是 v42 专为查"复位码 31"加的, 发布版关掉, 代码原样保留。 */
    return;
#else
    static uint8_t f[25];
    uint32_t v;
    uint16_t up_s;
    int32_t  left_ms;

    if (g_rs485_7_log_udp_enable == 0U) {
        return;
    }

    f[0] = 0x00U;
    f[1] = 0xA6U;
    f[2] = 0x6AU;
    f[3] = 0x88U;
    f[4] = 0x9AU;

    up_s  = (uint16_t)(HAL_GetTick() / 1000U);
    f[5]  = (uint8_t)( up_s       & 0xFFU);
    f[6]  = (uint8_t)((up_s >> 8) & 0xFFU);

    f[7]  = ota_client_stage();
    f[8]  = ota_client_sock_open();
    f[9]  = ota_client_sess();

    if (ota_task_wdt_armed() == 0U) {
        up_s = 0xFFFFU;                       /* 还没接管 */
    } else {
        left_ms = ota_task_wdt_left_ms();
        if (left_ms <= 0) {
            up_s = 0U;                        /* 已经到点, 马上要复位了 */
        } else if (left_ms >= 0xFFFE000) {    /* 不可能, 防御 */
            up_s = 0xFFFEU;
        } else {
            up_s = (uint16_t)((uint32_t)left_ms / 1000U);
        }
    }
    f[10] = (uint8_t)( up_s       & 0xFFU);
    f[11] = (uint8_t)((up_s >> 8) & 0xFFU);

    v = ota_task_wdt_report_cnt();
    f[12] = (uint8_t)( v        & 0xFFU);
    f[13] = (uint8_t)((v >>  8) & 0xFFU);
    f[14] = (uint8_t)((v >> 16) & 0xFFU);
    f[15] = (uint8_t)((v >> 24) & 0xFFU);

    v = ota_client_loop_cnt();
    f[16] = (uint8_t)( v        & 0xFFU);
    f[17] = (uint8_t)((v >>  8) & 0xFFU);
    f[18] = (uint8_t)((v >> 16) & 0xFFU);
    f[19] = (uint8_t)((v >> 24) & 0xFFU);

    v = g_test1_cnt;
    f[20] = (uint8_t)( v        & 0xFFU);
    f[21] = (uint8_t)((v >>  8) & 0xFFU);
    f[22] = (uint8_t)((v >> 16) & 0xFFU);
    f[23] = (uint8_t)((v >> 24) & 0xFFU);

    f[24] = WTD_RESET;

    dbg_log_udp_open();
    if (s_log_sock < 0) {
        return;
    }

    if (sendto(s_log_sock, f, sizeof(f), 0,
               (struct sockaddr *)&s_log_addr, sizeof(s_log_addr)) < 0) {
        (void)closesocket(s_log_sock);
        s_log_sock = -1;
    }
#endif /* DBG_LOG_ENABLE */
}

/* ============================================================================
 *  全局句柄 (用于主/从模式)
 *  ==========================================================================*/
static xMBMHandle xMBMaster7 = NULL;   /* Modbus 主机句柄 */
static xMBSHandle xMBSlave7  = NULL;   /* Modbus 从机句柄 */

/* ============================================================================
 *  板级报警计算
 *  ==========================================================================*/

/**
 * @brief  计算本板报警状态 — 基于从机模块在线状态 + KGL开关量报警
 * @note
 *   报警判定需要同时检查两个维度 (任一触发即报警):
 *     维度1: 模块在线状态 — link_mask 检查
 *           模块离线(Modbus无应答) → link_mask 对应位=0 → 报警
 *     维度2: KGL开关量报警 — ALARM_TEMP 检查
 *           模块在线但 KGL 位(bit0-5)非0 → ALARM_TEMP 对应位=0 → 报警
 *
 *   不能只复用 ALARM 全局变量的原因:
 *     ALARM 只基于 ALARM_TEMP(KGL位), 而 uart_modbus_mark_offline()
 *     不更新 ALARM_TEMP, 导致模块离线时 ALARM 不变化, 无法检测离线.
 *
 *   link_mask 存于 SerialDataBuf[port][7-10], 各端口每轮采集后更新.
 *   初始值=0, 首轮采集前会短暂误报(安全侧, fail-safe).
 *
 * @return 0=全部正常, 1=至少一个模块不在线或KGL报警
 */
static uint8_t compute_local_board_alarm(void)
{
    uint8_t i;
    uint8_t alarm = 0U;
    uint8_t cnt   = 0U;

    for (i = 0; i < UART_PORT_COUNT; i++) {
        uint8_t  module_count;
        uint32_t link_mask;
        uint32_t expected_mask;
        uint32_t kgl_alarm_mask;

        /* 跳过未启用或从机模式的端口 */
        if (BoardCfgTab[i].PortType == UART_PORT_DISABLED ||
            BoardCfgTab[i].PortType == UART_PORT_SLAVE) {
            continue;
        }

        module_count = BoardCfgTab[i].SlaveModuelNum;
        if (module_count == 0U) {
            continue;
        }
        if (module_count > 32U) {
            module_count = 32U;
        }

        link_mask     = *( (uint32_t *)&SerialDataBuf[i][7] );
        /* module_count 上面已钳到 32, 而 32 位机上 1UL<<32 是未定义行为, 单独处理 */
        expected_mask = (module_count >= 32U) ? 0xFFFFFFFFUL
                                              : (( 1UL << module_count ) - 1UL);

        /* 注: 道岔采集盒(交流/直流转辙机, S700K)同样参与在线掩码检查.
         * 曾因 9600bps 下偶发单帧 CRC 错被误判离线而整体屏蔽, 现恢复:
         * 道岔盒离线时应与其它模块一样触发本板报警. */

        /* ---- 维度2 的原始位图 (维度1 就是 link_mask 本身) ---- */
        /* ALARM_TEMP bit=1 表示该模块无KGL报警, bit=0 表示有KGL报警.
         * 只检查在线模块(在 link_mask 中的)的KGL状态:
         *   ~ALARM_TEMP[i] & link_mask != 0  →  有在线模块触发KGL报警  */
        kgl_alarm_mask = ~ALARM_TEMP[i] & link_mask;

        /* ---- 快照: 配了采集盒的端口一律记录, 不管它有没有报警 ----
         * 原来这里一发现报警就 return 1, 后面的端口不再扫描, 于是日志里
         * 也无法把"是哪个端口哪个模块"带出去。改成只置标志、跑完整轮:
         * 判决结果 (任一端口报警 → 1) 与原来完全等价, 只是不再提前退出。 */
        if (cnt < (uint8_t)DBG_LOG_PORT_MAX) {
            s_log_port[cnt].port_idx       = i;
            s_log_port[cnt].link_mask      = link_mask;
            s_log_port[cnt].expected_mask  = expected_mask;
            s_log_port[cnt].kgl_alarm_mask = kgl_alarm_mask;
            /* 与 link_mask 在同一时刻读走: 该端口上一轮到底问了几台模块.
             * 由 uart_module_should_poll() 维护 (见 bsp_rs485.c 的定义处). */
            s_log_port[cnt].polled_cnt     = uart_polled_cnt[i];
            cnt++;
        }

        /* ---- 维度1: 模块在线/离线检查 (link_mask != expected_mask) ---- */
        if (link_mask != expected_mask) {
            alarm = 1U;
        }

        /* ---- 维度2: KGL 开关量报警检查 ---- */
        if (kgl_alarm_mask != 0UL) {
            alarm = 1U;
        }
    }

    s_log_port_cnt = cnt;

    return alarm;  /* 1=有端口离线或有KGL报警, 0=全部正常 */
}

/* ============================================================================
 *  报警真值表判决
 *  ==========================================================================*/

/**
 * @brief  根据报警真值表计算 DO0/DO1/DO2 输出状态
 * @note   真值表逻辑:
 *          蜂鸣器(DO1)  = (alarm || break) XOR cut
 *          报警灯(DO2)  = alarm || break || cut
 *          工作灯(DO0)  = 与上位机的 TCP 连接正常 (client_is_connected())
 *
 *         cut/break 来自 master_read_io_module_di(), FC02 超时后那里已经把
 *         它们强制成 0 (time_ok 保护), 所以这里用的不会是过期的 DI 值.
 */
static void apply_alarm_truth_table(void)
{
    uint8_t combined_alarm;

    combined_alarm = s_local_board_alarm || s_remote_board_alarm;

    /* DO1: 蜂鸣器 = (alarm || break) XOR cut */
    s_buzzer = (combined_alarm || s_io_break_switch) ^ s_io_cut_switch;

    /* DO2: 报警灯 = alarm || break || cut */
    s_alarm_light = combined_alarm || s_io_break_switch || s_io_cut_switch;

    /*
     * DO0: 工作灯 = 与上位机(服务器)之间的 TCP 连接状态, 只此一条判据.
     *   注意: 这里曾经判的是"与 IO 联网模块的通信状态"(s_io_comm_fail_cnt),
     *   那是错的 —— 工作灯对外表达的是本板到上位机的链路是否正常.
     *   现场要求直接读 TCP 连接本身:
     *     连接已建立 → 亮; 断开(FIN/RST/链路超时) → 立即灭.
     *   不再叠加心跳新鲜度(heartbeat)判决 —— 那会让"上位机进程卡死但
     *   socket 未断"这种边缘情况把灯判灭, 与现场"断链才灭"的预期不符;
     *   该边缘情况由 main.c 的心跳复位机制和 client.c 的重连逻辑兜底.
     */
    s_work_light = client_is_connected() ? 1U : 0U;
}

/* ============================================================================
 *  全局变量同步
 *  ==========================================================================*/

/**
 * @brief  将静态变量同步到全局变量 (供外部模块读取)
 */
static void sync_global_variables(void)
{
    g_rs485_7_local_alarm  = s_local_board_alarm;
    g_rs485_7_remote_alarm = s_remote_board_alarm;
    g_rs485_7_io_cut       = s_io_cut_switch;
    g_rs485_7_io_break     = s_io_break_switch;
    g_rs485_7_work_light   = s_work_light;
    g_rs485_7_buzzer       = s_buzzer;
    g_rs485_7_alarm_light  = s_alarm_light;
}

/* ============================================================================
 *  从机模式: 寄存器回调
 *  ==========================================================================*/

/**
 * @brief  从机保持寄存器回调 (FC 03/06/10)
 * @note   地址0: 本板报警状态 (0=全部在线, 1=有报警)
 *         地址1: ALARM 原始值 (各端口报警位掩码摘要)
 */
static eMBException slave7_holding_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                       USHORT usNRegs, eMBSRegisterMode eRegMode)
{
    USHORT i;

    /* 每次访问时刷新本板报警状态 */
    s_local_board_alarm = compute_local_board_alarm();

    for (i = 0; i < usNRegs; i++) {
        USHORT reg_addr = usAddress + i;

        if (eRegMode == MBS_REGISTER_READ) {
            USHORT val = 0;

            switch (reg_addr) {
            case SLAVE_REG_ALARM_STATUS:
                val = (USHORT)s_local_board_alarm;
                break;
            case SLAVE_REG_ALARM_RAW:
                val = (USHORT)ALARM;
                break;
            default:
                val = 0xFFFFU;  /* 未定义寄存器: 返回0xFFFF */
                break;
            }

            *pubRegBuffer++ = (UCHAR)(val >> 8);
            *pubRegBuffer++ = (UCHAR)(val & 0xFF);
        } else {
            /* 写入模式: 暂不处理 (预留扩展) */
            pubRegBuffer += 2;
        }
    }

    /* 同步全局变量 */
    sync_global_variables();

    return MB_PDU_EX_NONE;
}

/**
 * @brief  从机输入寄存器回调 (FC 04)
 * @note   地址0: 本板报警状态 (只读)
 */
static eMBException slave7_input_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                     USHORT usNRegs)
{
    USHORT i;
    (void)usAddress;

    /* 每次访问时刷新本板报警状态 */
    s_local_board_alarm = compute_local_board_alarm();

    for (i = 0; i < usNRegs; i++) {
        USHORT val = (USHORT)s_local_board_alarm;
        *pubRegBuffer++ = (UCHAR)(val >> 8);
        *pubRegBuffer++ = (UCHAR)(val & 0xFF);
    }

    /* 同步全局变量 */
    sync_global_variables();

    return MB_PDU_EX_NONE;
}

/* ============================================================================
 *  主机模式: IO模块通信辅助函数
 *  ==========================================================================*/

/**
 * @brief  读取从机通信板报警状态 (FC 03, 地址1)
 * @return 1=通信成功(已刷新 s_remote_board_alarm), 0=失败(超时/CRC错)
 * @note   只负责通信与取值, 是否报警由调用方 poll_slave_comm_board()
 *         按读取成功与否决定
 */
static uint8_t master_read_slave_board(void)
{
    USHORT regs[1] = {0};
    eMBErrorCode eStatus;

    eStatus = eMBMReadHoldingRegisters(xMBMaster7, (UCHAR)RS485_7_SLAVE_COMM_ADDR,
                                       (USHORT)SLAVE_REG_ALARM_STATUS,
                                       (UBYTE)1, regs);

    /* 错误码原样留给状态日志帧尾, 用来给 R=1 分病 (超时 vs 帧坏).
     * 成功也刷成 0, 这样"尾字节==0"就等价于"上一次读是成功的". */
    s_slave_last_err = (uint8_t)eStatus;

    if (eStatus != MB_ENOERR) {
        return 0U;
    }

    s_remote_board_alarm = (uint8_t)(regs[0] & 0x0001U);
    return 1U;
}

/**
 * @brief  读取从机通信板报警状态 (仅主机模式调用)
 * @note   fail-safe: 每轮轮询, FC 03 读失败(超时/CRC错)直接置报警.
 *         从机通信板是常驻设备, 不做在位探测, 读不到就按故障报.
 *         读成功时 s_remote_board_alarm 由 master_read_slave_board()
 *         按从机返回的寄存器值刷新, 因此从机恢复正常后报警会自动解除.
 *
 *         例外: 开始轮询后的 RS485_7_SLAVE_GRACE_MS 内为启动宽限期, 此时读失败
 *         不报警 (两块板同时上电时从机板的 Modbus 从机栈可能还没进 eMBSPoll,
 *         读超时属于启动竞态而非故障). 宽限期内若读成功, 仍按从机寄存器返回的
 *         报警状态刷新, 即从机自己报的警不会被宽限期掩盖.
 *
 *         计时用 HAL_GetTick() 差值(无符号回绕安全), 与轮询周期长短无关.
 *         不要改回'按轮数': IO 模块不通时一轮要 5s, 3 轮会拖到 15s.
 */
static void poll_slave_comm_board(void)
{
    uint8_t ok = master_read_slave_board();

    /* 失败计数: 宽限期内也照记 —— 它是诊断量, 不参与报警判决.
     * 饱和在 255, 避免回绕后看起来像"刚刚恢复". */
    if (ok != 0U) {
        s_slave_read_fail_cnt = 0U;
    } else if (s_slave_read_fail_cnt < 255U) {
        s_slave_read_fail_cnt++;
    }

    if ((HAL_GetTick() - s_slave_grace_start) < RS485_7_SLAVE_GRACE_MS) {
        return;   /* 启动宽限: 只压'读失败即报警', 计数照记 */
    }

    if (ok == 0U) {
        s_remote_board_alarm = 1U;
    }
}

/**
 * @brief  读取IO联网模块DI状态 (FC 02, 地址4, DI0-DI1)
 * @note   成功时更新 cut/break 并清零超时计数; 失败时累加超时计数.
 *
 *         ---- time_ok 超时保护 (参考文档 5.4) ----
 *         超过 RS485_7_IO_DI_TIMEOUT_MS 没有一次成功的 FC02, 就把 cut/break
 *         按 0 参与真值表, 也就是清掉由 DI 导出的报警; 而不是拿着最后一次
 *         读到的过期值继续驱动蜂鸣器/报警灯.
 *
 *         为什么必须清: IO 模块掉线时 FC05 写 DO 大概率也发不出去, 输出会
 *         冻在模块里; 如果这时还保留过期的 break=1, 报警就永远解除不了.
 *         台面实测过这个现象: 断掉 IO 模块后 io_fail 涨到 4, 而 cut/break
 *         仍是上一次成功读到的值, 板子继续照那组值驱动输出.
 *
 *         只清 DI 这一条链路: L(本板采集盒) / R(从机板) / 工作灯 DO0 都不
 *         受影响, 工作灯按现场要求仍然只看 client_is_connected().
 *         恢复通信后, 下一次成功的 FC02 会自动把 cut/break 填回真值.
 */
static void master_read_io_module_di(void)
{
    UBYTE di_buf[1] = {0};
    eMBErrorCode eStatus;

    /* IO模块响应较慢(~90ms), 设置2000ms超时确保有足够余量 */
    (void)eMBMSetSlaveTimeout(xMBMaster7, 2000U);

    eStatus = eMBMReadDiscreteInputs(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                                     (USHORT)IO_MODULE_DI_START,
                                     (USHORT)IO_MODULE_DI_COUNT, di_buf);

    /* 恢复默认超时 */
    (void)eMBMSetSlaveTimeout(xMBMaster7, 1000U);

    if (eStatus == MB_ENOERR) {
        /* DI数据按位打包: LSB of byte[0] = DI0 */
        s_io_cut_switch   = (di_buf[0] >> 0) & 0x01U;   /* DI0: 切断开关 */
        s_io_break_switch = (di_buf[0] >> 1) & 0x01U;   /* DI1: 断接开关 */
        s_io_comm_fail_cnt = 0;                          /* 清零超时计数 */
        s_io_last_ok_tick  = HAL_GetTick();              /* 刷新新鲜度时基 */
    } else {
        /* 打印具体错误码帮助排查
         * eStatus: 0=OK, 5=EIO(CRC错), 8=ETIMEDOUT(超时-设备未回复) */
        dbg_net_printf("[RS485-7] FC02 fail: eStatus=%d (0=OK, 5=EIO, 8=TIMEOUT)\n", (int)eStatus);

        /* 饱和在 255: 它是诊断量, 回绕到 0 会被误读成"刚刚恢复".
         * 用 uint8 是为了直接塞进状态日志帧 [12]. */
        if (s_io_comm_fail_cnt < 255U) {
            s_io_comm_fail_cnt++;
        }

        /* time_ok 超时: 这么长时间没读到过 DI, 丢弃过期的 cut/break.
         * 按 0 参与真值表 = 清掉由 DI 导出的报警 (break 那份).
         * 用 HAL_GetTick() 差值, 与轮询周期长短无关. */
        if ((HAL_GetTick() - s_io_last_ok_tick) >= RS485_7_IO_DI_TIMEOUT_MS) {
            s_io_cut_switch   = 0U;
            s_io_break_switch = 0U;
        }
    }
}

/**
 * @brief  写入IO联网模块DO状态 (FC 05, 地址4, DO0-DO2)
 * @note   逐线圈写入, 失败不重试 (下轮循环自动重写)
 */
static void master_write_io_module_do(void)
{
    /* DO0: 工作灯 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_WORK_LIGHT,
                              (BOOL)(s_work_light ? TRUE : FALSE));

    /* DO1: 蜂鸣器 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_BUZZER,
                              (BOOL)(s_buzzer ? TRUE : FALSE));

    /* DO2: 报警灯 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_ALARM_LIGHT,
                              (BOOL)(s_alarm_light ? TRUE : FALSE));
}

/* ============================================================================
 *  第7路RS485 任务主函数
 *  ==========================================================================*/
static void rs485_7_task_handler(void *pvParameters)
{
    uint32_t baudrate;
    uint8_t  mode;

    (void)pvParameters;

    /* 读取拨码配置 */
    baudrate = g_dip_rs485_7_baudrate;
    mode     = g_dip_rs485_7_mode;   /* 3=Slave, 4=Master */

    dbg_net_printf("[RS485-7] Baud=%lu, Mode=%s\n",
           (unsigned long)baudrate,
           (mode == 4U) ? "Master" : "Slave");

    /* ---- 初始化: 带有限重试 + 周期自恢复 ---- */
    {
        eMBErrorCode eStatus;
        uint32_t     retry = 0U;

        for (;;) {
            eStatus = (mode == 4U)
                ? eMBMSerialInit(&xMBMaster7, MB_RTU, RS485_7_PORT_INDEX,
                                 baudrate, MB_PAR_NONE)
                : eMBSSerialInit(&xMBSlave7, MB_RTU, RS485_7_SLAVE_ADDR,
                                 RS485_7_PORT_INDEX, baudrate, MB_PAR_NONE);

            if (eStatus == MB_ENOERR) {
                break;   /* 初始化成功, 进入业务循环 */
            }

            /* 打印具体错误码便于定位: 2=EINVAL 3=EPORTERR 4=ENORES 7=EAGAIN */
            dbg_net_printf("[RS485-7] %s init FAILED, err=%d, retry=%lu\n",
                   (mode == 4U) ? "Master" : "Slave",
                   (int)eStatus, (unsigned long)retry);

            retry++;
            /* 前几次快速重试(扛瞬态故障), 之后进入慢速自恢复 */
            vTaskDelay(pdMS_TO_TICKS(retry <= RS485_7_INIT_RETRY_MAX
                                     ? RS485_7_INIT_RETRY_DELAY_MS
                                     : RS485_7_INIT_RECOVER_DELAY_MS));
        }
    }

    /* ---- 根据模式进入业务 ---- */
    if (mode == 4U) {
        /* =============================================================
         *  Modbus 主机模式
         * ============================================================= */
        dbg_net_printf("[RS485-7] Master mode started\n");
        dbg_net_printf("[RS485-7]   -> Slave Comm Board addr=%u\n", RS485_7_SLAVE_COMM_ADDR);
        dbg_net_printf("[RS485-7]   -> IO Module addr=%u\n", RS485_7_IO_MODULE_ADDR);

        /* 初始化后延迟, 确保RS485总线稳定 */
        vTaskDelay(pdMS_TO_TICKS(200));

        /* 启动宽限计时起点: 从'开始轮询'算起(前面的串口初始化 + 200ms
         * 稳定延迟已消耗掉一部分上电时间, 从这里算能保证宽限期是完整的). */
        s_slave_grace_start = HAL_GetTick();

        /* DI 新鲜度时基: 同一时刻打点, 保证上电后 15s 内不会因为"还没读到
         * 过 DI"就被判成超时 (此时 cut/break 本来也是初值 0, 判不判都一样). */
        s_io_last_ok_tick   = HAL_GetTick();

        /* ---- 主机任务循环 ---- */
        while (1) {
            /* 步骤1: 更新本板报警 */
            s_local_board_alarm = compute_local_board_alarm();

            /* 步骤2: 读取从机通信板报警状态 (FC 03, 地址1), 读失败即报警 */
            poll_slave_comm_board();

            /* 步骤3: 读取IO模块DI (FC 02, 地址4) */
            master_read_io_module_di();

            /* 步骤4+5: 报警真值表判决 */
            apply_alarm_truth_table();

            /* 步骤6: 写入IO模块DO (FC 05, 地址4) */
            master_write_io_module_do();

            /* 步骤6.5: 写完 DO 立刻把串口7全部状态推给上位机
             * (UDP → 192.168.1.88:50000, 变长帧).
             * 放在 DO 写入之后, 保证帧里的 buz/light/work 就是刚刚落到
             * IO 模块上的那一组值. 开关: g_rs485_7_log_udp_enable.
             * 默认只在报警时才真的发 (DBG_LOG_ALARM_ONLY), 这里照常每轮
             * 调用, 门槛判断在 dbg_log_udp_send() 内部。 */
            dbg_log_udp_send();
            dbg_ota_diag_send();   /* 诊断帧: OTA 任务此刻的生死 (v42 临时) */

            /* 步骤6.6: 看门狗已判定要复位 → 无视报警门槛立刻补发一帧, 抢在
             * SP706S 复位那约 1.6s 窗口内把复位码送出去 (为什么走 UDP 而不是
             * TCP 日志通道, 见 rs485_7_log_send_now 的说明). */
            if (WTD_RESET != 0U) {
                rs485_7_log_send_now();
            }

            /* 步骤7: 同步全局变量 */
            sync_global_variables();

            /* LED闪烁指示 */
            // LED_485_Set(0, TOGGLE);

            /* 调试输出 (每轮一次, 经 UDP 发到上位机 DBG_NET_IP:DBG_NET_PORT)
             * 每轮发是为了不漏掉只持续一个轮次的蜂鸣器抖动 —— 原来每 10 轮
             * 发一次, 正好会漏掉这种瞬态.
             * [排查] 蜂鸣器/报警灯同步切换: 看这一行就能定位是谁在跳 ——
             *   cut/brk 在跳       → IO模块 DI0/DI1 输入抖动
             *   L 在跳             → 本板报警在跳 (link_mask 或 KGL 位)
             *   三者都不跳但仍切换 → IO模块通信间歇失败, 是模块自身
             *                        通信中断保持/置位在驱动输出 */
            dbg_net_printf("[RS485-7] L=%u R=%u cut=%u brk=%u -> buz=%u light=%u work=%u fail=%u\r\n",
                           s_local_board_alarm, s_remote_board_alarm,
                           s_io_cut_switch, s_io_break_switch,
                           s_buzzer, s_alarm_light, s_work_light,
                           s_io_comm_fail_cnt);

            /* 步骤8: 任务延迟 */
            vTaskDelay(pdMS_TO_TICKS(RS485_7_POLL_DELAY_MS));
        }

    } else {
        /* =============================================================
         *  Modbus 从机模式 (默认)
         * ============================================================= */

        /* 注册寄存器回调 */
        eMBSRegisterHoldingCB(xMBSlave7, slave7_holding_cb);
        eMBSRegisterInputCB(xMBSlave7, slave7_input_cb);

        dbg_net_printf("[RS485-7] Slave mode started, addr=%u\n", RS485_7_SLAVE_ADDR);

        /* ---- 从机任务循环 ---- */
        while (1) {
            eMBErrorCode eStatus;

            /* 周期性更新本板报警 (即使没有主机查询也保持最新) */
            s_local_board_alarm = compute_local_board_alarm();

            /* 从机不驱动 IO 模块, 但工作灯这一位对外表达的是"本板跟上位机
             * 的 TCP 连接状态", 与主机模式语义一致, 所以这里也刷新它,
             * 免得状态日志帧 [11] 恒为 0 被误读成"从机板跟上位机断链". */
            s_work_light = client_is_connected() ? 1U : 0U;

            sync_global_variables();

            /* 状态日志: 从机循环是 10/50ms 一轮, 不限制会比主机发得还密,
             * 所以按 DBG_LOG_SLAVE_PERIOD_MS 限流.
             * (DBG_LOG_ALARM_ONLY 打开时这一层限流其实用不上了 —— 门槛
             *  在 dbg_log_udp_send() 里, 不报警就直接返回. 留着是为了
             *  把它关掉时仍然有节奏上限.)
             * 帧里只有 [5] 本板报警 和 [11] TCP连接 有效 —— 把从机板的
             * 报警状态也送上来了, 主机板日志里的 [6](R)=1 到底是"读失败"
             * 还是"从机板真报了警", 对着两块板的 [5] 一比就清楚了.
             * [13] 在从机模式下无意义(不轮询第7路), 恒 0. */
            if ((HAL_GetTick() - s_slave_log_tick) >= DBG_LOG_SLAVE_PERIOD_MS) {
                s_slave_log_tick = HAL_GetTick();
                dbg_log_udp_send();
                dbg_ota_diag_send();   /* 诊断帧: OTA 任务此刻的生死 (v42 临时) */
            }

            /* 看门狗判定要复位 → 无视门槛立刻补发一帧带走复位码 (同主机模式,
             * 见 rs485_7_log_send_now). 从机循环周期 10~50ms, 1.6s 窗口内发得
             * 出去很多次; 复位码不变, 多发的几帧内容一样, 不算噪声。 */
            if (WTD_RESET != 0U) {
                rs485_7_log_send_now();
            }

            /* 轮询 Modbus 请求 */
            eStatus = eMBSPoll(xMBSlave7);

            if (eStatus == MB_ENOERR) {
                /* 成功处理了一帧 → 闪烁 LED */
                // LED_485_Set(0, TOGGLE);
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                /* 无请求或出错 → 短暂延迟后继续轮询 */
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
}

/* ============================================================================
 *  rs485_7_task_create  —  创建第7路 RS485 任务
 *  ==========================================================================*/
void rs485_7_task_create(void)
{
    TaskHandle_t xHandle = NULL;
    BaseType_t xReturn;

    xReturn = xTaskCreate(rs485_7_task_handler,
                          "RS485_7",
                          RS485_7_TASK_STACK,
                          NULL,
                          RS485_7_TASK_PRIO,
                          &xHandle);

    if (xReturn == pdPASS) {
        printf("Create RS485_7_Task success...\r\n");
    } else {
        printf("Create RS485_7_Task FAILED!\r\n");
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
