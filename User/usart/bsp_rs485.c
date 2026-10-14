/**
 *********************************************************************
 * @file    bsp_rs485.c
 * @brief   6 路 RS485 串口 — 全局变量、模块描述表、曲线帧处理
 *
 * 本文件是串口采集系统的核心模块, 包含:
 *   1. 全局变量定义 (配置表、数据缓冲、报警、日志标志等)
 *   2. 模块描述表 (9 种模块类型, 用一张表驱动差异化处理)
 *   3. SDRAM 辅助函数 (实时值 SSZ / 曲线 QX 的地址计算)
 *   4. 工具函数 (大端转小端、CRC16、波特率查询)
 *   5. 自定义串口底层驱动 (6 路 UART HAL 句柄 + RS485 DE 引脚)
 *   6. 曲线帧处理 (普通帧/中间帧/结束帧)
 *
 * 6 路串口引脚映射:
 *   端口     RX      TX      EN(DE)
 *   USART1  PA10    PA9     PA11
 *   USART2  PD6     PD5     PD4
 *   USART3  PB11    PB10    PH6
 *   UART4   PC11    PC10    PA15
 *   UART5   PD2     PC12    PA15
 *   USART6  PC7     PC6     PC8
 *********************************************************************
 */

#include "bsp_rs485.h"
#include "./sdram/bsp_sdram.h"
#include "bsp_eeprom.h"
#include "mbmcrc.h"
#include "client.h"

/*======================================================================*/
/*  一、全局变量定义                                                      */
/*======================================================================*/

/* ---- 板卡配置表 (上位机下发, 包含 6 路端口的参数) ---- */
BoardConfig_t BoardCfgTab[UART_PORT_COUNT];

/* ---- 串口数据缓冲区 (每端口 2KB, 帧头 11 字节 + 数据体) ---- */
uint8_t       SerialDataBuf[UART_PORT_COUNT][UART_SERIAL_BUF_SIZE];

/* ---- EEPROM 旧配置暂存 (与上位机新配置比较, 判断是否需要重写 EEPROM) ---- */
uint8_t       MramCfgDataBuf[BOARD_CONFIG_BYTES];

/* ---- 端口协议标志: 0=Modbus RTU, 1=自定义串口 ---- */
uint8_t       UART_Custom[UART_PORT_COUNT] = {0};

/* ---- 端口轮询延迟 (ms), 根据从机数量自动计算 ---- */
uint32_t      UART_delay[UART_PORT_COUNT] = {0};

/* ---- 报警温度/状态 (每端口一个 32 位掩码, 初始化为全 1 = 无报警) ---- */
uint32_t      ALARM_TEMP[UART_PORT_COUNT] = {0};

/* ---- 配置下载标志: 0=未下载, 1=已下载 (串口任务等待此标志) ---- */
uint8_t       download_flag = 0;

/* ---- 各端口看门狗计数器 (串口任务清零, App_TaskStart 累加并检查超时) ---- */
uint16_t      WDTUARTFLAG[UART_PORT_COUNT] = {0};

/* ---- 串口任务句柄 (用于外部管理任务状态) ---- */
TaskHandle_t  Serial_Task_Handles[UART_PORT_COUNT] = {NULL};

/* ---- 曲线模块信息 (帧序号/帧标记/曲线编号/点数, 每个端口 32 个模块) ---- */
CurveModuleInfo_t AC_module_infor[UART_PORT_COUNT];

/* ---- 曲线微步数 (每端口每模块的已缓存曲线点数) ---- */
uint16_t          Weibo_number[UART_PORT_COUNT][32];

/* ---- 校时标志: 1=需要对曲线模块发送校时命令 ---- */
uint8_t           changetime[UART_PORT_COUNT] = {0};

/* ---- 已校时模块计数 (达到 SlaveModuelNum 后清零 changetime) ---- */
uint8_t           TaskAdjustNum[UART_PORT_COUNT] = {0};

/* ---- 系统时间 (上位机校时命令下发, Unix 时间戳格式) ---- */
uint32_t          Systemtime = 0;

/* ---- 诊断计数器: 发送窗口内到达、被 TC 中断冲掉的残字节次数 ----
 * 由 mbport/mbportserial.c 的 TC 分支自增, 任务侧只读.
 * 非 0 说明"发送期间总线还有别人在发字节" (上一台迟到应答 / 回声 / 噪声),
 * 即旧实现会把残字节当应答首字节、导致整帧前移一字节而 CRC 失败的场景. */
volatile uint16_t uart_custom_flush_cnt[UART_PORT_COUNT] = {0};

/* ---- 诊断计数器: 本轮【实际发起过请求】的模块数 (每端口一个) ----
 * 由 uart_module_should_poll() 维护 (它在四个端口处理器的轮询循环里,
 * 每模块每轮恰好被调用一次), 串口7的 UDP 状态日志每轮读走.
 *
 * 为什么要它: 已发布的 link_mask 为 0 有两种完全不同的成因 ——
 *   本轮请求都发出去了、但没有一台应答   → 真故障 (总线/盒子/干扰)
 *   本轮压根没发请求 (全部模块处于"连续失败 ≥3 次后的跳过态") → 粘住,
 *                                        连总线都没碰, 每 10 轮才探一次
 * 光看 link_mask 分不出这两种, 必须知道"这一轮到底问了几台":
 *   polled_cnt == 0 且 link_mask == 0  →  整路处于跳过态 (不是新故障)
 *   polled_cnt > 0  且 link_mask == 0  →  问了 N 台一台都没答 (真故障)
 * 等于"该端口 SlaveModuelNum"时说明本轮全量轮询, 没有模块被跳过. */
volatile uint8_t  uart_polled_cnt[UART_PORT_COUNT] = {0};

/* ---- 诊断计数器: 接收侧丢字节定位 (2026-09-28, 由 mbportserial.c 的 ISR 自增) ----
 * 现场已证实: 盒子回的 143B 帧在线上完整且 CRC 自洽, 主站只拿到 142B 且切帧后
 * 再等 300ms 也无字节到达 → 末字节丢在主站侧. 这三个计数器把"丢在哪一环"分开:
 *   dc_rx_store_cnt: ISR 存进 ring 的字节数 (与任务侧 copied 对比)
 *   dc_rx_ore_cnt  : ORE 置位次数 → 硬件溢出丢字节 (ISR 被阻塞 > 1 字节时间)
 *   dc_rx_steal_cnt: 错误标志块读 DR 时 RXNE 仍置位 → 该字节原本会被吃掉 (已改为存回)
 * 任务侧在每帧开始/结束取差值打印, 定位后连同 [DC-SHORT] 一起删除. */
volatile uint16_t dc_rx_store_cnt[UART_PORT_COUNT] = {0};
volatile uint16_t dc_rx_ore_cnt[UART_PORT_COUNT]   = {0};
volatile uint16_t dc_rx_steal_cnt[UART_PORT_COUNT] = {0};

/* ---- 总报警标志 (6 路端口各占 1 bit) ---- */
uint8_t           ALARM = 0;

/*======================================================================*/
/*  TCP / 看门狗 / 日志 全局变量                                          */
/*======================================================================*/

/* TCP 发送任务看门狗计数器: App_TaskStart 累加, TCP 发送任务清零 */
uint16_t          WDTTCPSENDFLAG = 0;

/* 心跳计数器: 收到上位机心跳应答时清零, 超过阈值 → 触发复位 */
uint16_t          heartbeat      = 0;

/* 连接持续时间计数器 */
uint16_t          connect_time   = 0;

/* 看门狗复位原因码:
 *   0  = 正常 (喂狗)
 *   1  = 配置变更复位
 *   2  = 上位机软复位命令
 *   21-26 = 对应串口任务超时 (21=USART1 ... 26=USART6)
 *   27 = 心跳超时
 *   28 = TCP 发送任务超时
 *   29 = 申请配置帧发送失败
 */
uint8_t           WTD_RESET      = 0;

/* 系统启动日志标志: 启动时置 1, checkSendLog 发送后清零 */
volatile uint8_t  pStart         = 0;

/* 网络重连日志标志: TCP 重连成功时置 1, checkSendLog 发送后清零 */
volatile uint8_t  netReset       = 0;

/* 看门狗复位日志标志: 即将复位时置 1, 发送后置 2 (防止重复发送) */
uint8_t           logWTD_RESET   = 0;

/* 校时日志标志: 收到校时命令时置 1, checkSendLog 发送后清零 */
uint8_t           jiaoshi        = 0;

/* 端口数据错误标志 (每端口): 1/2=待发送, 0=已发送并清除, 0xFF=不再发送 */
uint8_t           portError[UART_PORT_COUNT] = {0};

/* 端口曲线上传完成标志 (每端口): 非零=有曲线数据需要上传 */
uint8_t           portDLQX[UART_PORT_COUNT] = {0};

/* 计轴模块通讯确认码状态 (每端口每模块跟踪上次收到的确认码) */
AxleCounterState_t axle_counter_state[UART_PORT_COUNT];
DC3WireState_t     dc3wire_state[UART_PORT_COUNT];

/* ---- UART RX 环形缓冲区 (中断驱动, 防止高优先级任务抢占导致 ORE 丢字节) ---- */
UART_RxRing_t       uart_rx_ring[UART_PORT_COUNT];

/* ---- UART TX 状态 (中断驱动发送: TXE 逐字节, TC 完成切接收; 免疫任务抢占) ---- */
UART_CustomTxState_t uart_custom_tx[UART_PORT_COUNT];

/* ---- 计轴模块调试打印开关: 上位机写入 1=开启, 0=关闭 (默认关闭) ---- */
uint8_t             g_debug_axle_enable = 0;
uint8_t             g_debug_dc3w_enable = 0;
uint8_t             g_debug_dc3w_port   = 3;   /* 0=全部端口, 非0=仅显示指定端口 */

/* ---- UART IRQ 编号 (与 uart_init_custom_raw 的端口顺序一致) ---- */
static const IRQn_Type uart_irq_numbers[UART_PORT_COUNT] = {
    UART7_IRQn,     /* port 0: UART7  */
    USART2_IRQn,    /* port 1: USART2 */
    UART5_IRQn,     /* port 2: UART5  */
    UART4_IRQn,     /* port 3: UART4  */
    USART1_IRQn,    /* port 4: USART1 */
    USART6_IRQn     /* port 5: USART6 */
};


/*======================================================================*/
/*  二、串口硬件配置表                                                    */
/*======================================================================*/

/* ---- 6 路 UART 的 HAL 句柄 (自定义串口模式使用) ---- */
UART_HandleTypeDef  uart_raw_handles[UART_PORT_COUNT];

/* ---- 6 路 RS485 DE 引脚配置 (发送前拉高, 发送后拉低) ---- */
UART_RawConfig_t    uart_raw_config[UART_PORT_COUNT];

/* ---- 端口名称和 UART 外设编号 ---- */
const UART_HW_Config_t uart_hw_config[UART_PORT_COUNT] = {
    {"USART1", 1},   /* RX:PA10, TX:PA9,   EN:PA11 */
    {"USART2", 2},   /* RX:PD6,  TX:PD5,   EN:PD4  */
    {"USART3", 3},   /* RX:PB11, TX:PB10,  EN:PH6  */
    {"UART4",  4},   /* RX:PC11, TX:PC10,  EN:PA15 */
    {"UART5",  5},   /* RX:PD2,  TX:PC12,  EN:PA15 */
    {"USART6", 6}    /* RX:PC7,  TX:PC6,   EN:PC8  */
};


/*======================================================================*/
/*  三、模块描述表                                                        */
/*                                                                       */
/*  设计思想: 每种模块类型的差异化参数集中在一张表里,                       */
/*  处理函数通过查表获取参数, 避免为每种类型写重复的 switch-case.           */
/*                                                                       */
/*  字段说明:                                                             */
/*    module_type:         模块类型编号 (0x31-0x39)                        */
/*    input_regs:          Modbus 输入寄存器个数 (要读取的寄存器数)         */
/*    holding_regs:        保持寄存器个数 (要拷贝到 Holding 区的数据寄存器)  */
/*    has_alarm:           1=检查状态寄存器的报警位 (bit0-5)                */
/*    uses_custom_curve:   1=使用自定义串口协议 (非 Modbus)                  */
/*    realtime_bytes:      实时数据字节数 (= input_regs * 2)                */
/*    curve_data_offset:   响应帧中曲线数据起始偏移                          */
/*    curve_len_offset:    每个分支的曲线长度字节偏移                        */
/*    curve_flag_offset:   帧标志字节偏移 (0=普通帧/0x55=中间帧/0xFF=结束帧) */
/*    curve_seq_offset:    帧序号字节偏移                                   */
/*    curve_direction_offset: 转动方向字偏移 (结束帧时拷贝到曲线数据末尾)     */
/*    curve_branch_count:  曲线分支数 (1/2/4)                               */
/*======================================================================*/
static const UART_ModuleProfile_t uart_module_profiles[] = {
    /*  type,         in, hold, alarm, custom, rtbytes, dat_off, len_off, flag_off, seq_off, dir_off, branches */
    /* ---- 普通 Modbus 模块 (uses_custom_curve=0, 无曲线相关偏移) ---- */
    {FLFXG_XHJ_KGL,  13, 11,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_XHJ,      13, 11,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_GD_25_J,  34, 32,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_DC_AC,    10, 8,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_GD_25_F,  30, 28,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_CL,       8,  6,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* ---- 自定义曲线模块 (uses_custom_curve=1) ---- */
    /* AC_ZZJ:  交流转辙机, 29 个寄存器 (58 字节实时值), 4 分支曲线      */
    {FLFXG_AC_ZZJ,   29, 27,   0,     1,      58,      71,      69,      61,       63,      271,     4},
    /* DC_ZZJ4: 直流四线制, 8 个寄存器 (16 字节实时值), 1 分支曲线        */
    {FLFXG_DC_ZZJ4,  8,  6,    0,     1,      16,      29,      27,      19,       21,      79,      1},
    /* DC_ZZJ6: 直流六线制, 11 个寄存器 (22 字节实时值), 2 分支曲线       */
    {FLFXG_DC_ZZJ6,  11, 9,    0,     1,      22,      35,      33,      25,       27,      135,     2},
    /* S700K_C: 四线制交流道岔, 13 个寄存器 (26 字节实时值), 4 分支曲线  */
    {FLFXG_S700K_C,  13, 11,   0,     1,      26,      38,      36,      28,       30,       238,     4},
    /* ---- 高频 Modbus 模块 (uses_custom_curve=0) ---- */
    /* NEW_XHJ:      高频信号机, 50 个输入寄存器                          */
    {FLFXG_NEW_XHJ,       50, 48,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* GD_25_J_HIGH: 高频25Hz轨道接收, 133 个输入寄存器                    */
    {FLFXG_GD_25_J_HIGH,  133,131,  1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* GD_25_F_HIGH: 高频25Hz轨道发送, 117 个输入寄存器                       */
    {FLFXG_GD_25_F_HIGH,  117,115,  1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* CL_HIGH:      高频采集, 62 个输入寄存器                            */
    {FLFXG_CL_HIGH,       62, 60,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* ---- 高频自定义曲线模块 (uses_custom_curve=1) ---- */
    /* AC_ZZJ_HIGH:   高频交流转辙机, 68 个寄存器 (136 字节), 4 分支      */
    {FLFXG_AC_ZZJ_HIGH,   68, 66,   0,     1,      136,     149,     147,     139,      141,      349,     4},
    /* DC_ZZJ4_HIGH:  高频直流四线制, 23 个寄存器 (46 字节), 1 分支       */
    {FLFXG_DC_ZZJ4_HIGH,  23, 21,   0,     1,      46,      59,      57,      49,       51,       109,     1},
    /* DC_ZZJ6_HIGH:  高频直流六线制, 26 个寄存器 (52 字节), 2 分支       */
    {FLFXG_DC_ZZJ6_HIGH,  26, 24,   0,     1,      52,      65,      63,      55,       57,       165,     2},
    /* S700K_C_HIGH: 高频四线制交流道岔, 44 个寄存器 (88 字节实时值), 4 分支 */
    {FLFXG_S700K_C_HIGH,  44, 42,   0,     1,      88,     100,      98,      90,       92,       300,     4},
    /* ---- 防雷分线柜模块 ---- */
    /* Axle Counter: 计轴采集传感器, 10 寄存器(20B), 带报警(KGL开关量), 非标准Modbus */
    {FLFXG_AXLE_COUNTER,       10, 0,    1,     0,      20,      0,       0,       0,        0,       0,       0},
    /* DC3Wire: 直流3对线, 10寄存器(20B), 带报警(KGL开关量位), 非标准Modbus */
    {FLFXG_DC3WIRE_ZTM,   10, 0,    1,     0,      20,      0,       0,       0,        0,       0,       0},
    {FLFXG_DC3WIRE_ZCL,    10, 0,    1,     0,      20,      0,       0,       0,        0,       0,       0},
    /* ---- 防雷分线柜模块（高频） ---- */
    {FLFXG_AXLE_COUNTER_HIGH,   28, 0,    1,     0,      56,      0,       0,       0,        0,       0,       0},
    /* ---- 哨兵条目 (未匹配到类型时的回退) ---- */
    {0x00,           13, 11,   1,     0,      0,       0,       0,       0,        0,       0,       0}
};


/*======================================================================*/
/*  四、SDRAM 辅助函数                                                    */
/*                                                                       */
/*  SDRAM 布局 (IS42S16160J-6TLI, 32MB, 基址 0xC0000000):                */
/*                                                                       */
/*  实时值区域 (SSZ):                                                     */
/*    基址: SDRAM_BASE_ADDR (0xC0000000)                                  */
/*    每端口 0x1000 字节 (4KB)                                            */
/*    Port0: 0xC0000000, Port1: 0xC0001000, ... Port5: 0xC0005000        */
/*                                                                       */
/*  曲线缓存区域 (QX):                                                    */
/*    基址: SDRAM_BASE_ADDR + 0x5000 (0xC0005000)                         */
/*    每端口 0x1E000 字节, 每模块 0x1000 字节 (4KB)                        */
/*    每分支 0x400 字节 (1KB)                                             */
/*    Port0 曲线: 0xC0005000, Port1 曲线: 0xC0023000, ...                */
/*======================================================================*/

/**
 * @brief  获取指定端口的实时值 SDRAM 指针
 * @param  port_idx  端口索引 (0-5)
 * @return SDRAM 地址指针 (uint16_t 对齐)
 */
void *uart_ssz_ptr(uint8_t port_idx)
{
    return (void *)(SDRAM_SSZ_BASE + (uint32_t)port_idx * SDRAM_SSZ_SIZE_PER_PORT);
}

/**
 * @brief  获取指定端口+模块的曲线缓存 SDRAM 指针
 * @param  port_idx    端口索引 (0-5)
 * @param  module_idx  模块索引 (0-31)
 * @return SDRAM 地址指针 (uint16_t 对齐)
 */
void *uart_qx_ptr(uint8_t port_idx, uint8_t module_idx)
{
    return (void *)(SDRAM_QX_BASE
                    + (uint32_t)port_idx * 0x1E000U
                    + (uint32_t)module_idx * SDRAM_QX_SIZE_PER_MODULE);
}


/*======================================================================*/
/*  五、工具函数                                                          */
/*======================================================================*/

/**
 * @brief  16 位大端转小端 (原地转换)
 * @note   Modbus 协议使用大端字节序, STM32 使用小端字节序.
 *         从模块读取的寄存器数据需要调用此函数转换后才能正确解读.
 * @param  buf    待转换的 uint16_t 数组
 * @param  count  元素个数
 */
void uart_swap16(uint16_t *buf, uint16_t count)
{
    while (count > 0U) {
        *buf = (uint16_t)((*buf >> 8) | (*buf << 8));
        buf++;
        count--;
    }
}

/**
 * @brief  计算 CRC16 (Modbus 标准)
 * @note   调用 FreeModbus 主站库的 usMBMCRC16 函数.
 * @param  buf  数据缓冲区
 * @param  len  数据长度 (字节)
 * @return 16 位 CRC 值
 */
uint16_t uart_crc16(const uint8_t *buf, uint16_t len)
{
    return usMBMCRC16((const UBYTE *)buf, len);
}

/**
 * @brief  波特率编码 → 实际值转换
 * @param  baud_code  1=9600, 2=19200, 3=38400, 4=57600
 * @return 实际波特率值, 默认 19200
 */
uint32_t uart_get_baud_rate(uint8_t baud_code)
{
    switch (baud_code) {
        case 1:  return 9600;
        case 2:  return 19200;
        case 3:  return 38400;
        case 4:  return 57600;
        default: return 19200;
    }
}

/**
 * @brief  第7路RS485 波特率转换 — 拨码开关 BA5-BA6
 * @param  dip_code  0=9600, 1=19200, 2=57600, 3=115200
 * @return 实际波特率值, 默认 19200
 */
uint32_t uart_get_baud_rate_port7(uint8_t dip_code)
{
    switch (dip_code) {
        case 0:  return 9600;
        case 1:  return 19200;
        case 2:  return 57600;
        case 3:  return 115200;
        default: return 19200;
    }
}


/*======================================================================*/
/*  六、串口初始化                                                        */
/*======================================================================*/

/**
 * @brief  初始化指定端口 (Modbus RTU 或自定义串口)
 * @note   根据 BoardCfgTab[port_idx].PortType 选择初始化方式:
 *          - UART_PORT_MODBUS_MASTER (4): 调用 eMBMSerialInit (FreeModbus)
 *          - UART_PORT_CUSTOM (1):       调用 uart_init_custom_raw (HAL 底层)
 * @param  port_idx    端口索引 (0-5)
 * @param  baud_rate   波特率 (9600/19200/38400/57600)
 * @param  pxMBMaster  Modbus 主站句柄指针 (Modbus 模式使用)
 * @return MB_ENOERR=成功, 其他=失败
 */
eMBErrorCode uart_init_port(uint8_t port_idx, uint32_t baud_rate, xMBMHandle *pxMBMaster)
{
    if ((port_idx >= UART_PORT_COUNT) || (pxMBMaster == NULL)) {
        return MB_EINVAL;
    }

    if (BoardCfgTab[port_idx].PortType == UART_PORT_MODBUS_MASTER) {
        /* Modbus RTU 主站模式: 使用 FreeModbus 主站库初始化 */
        UART_Custom[port_idx] = 0;
        if (eMBMSerialInit(pxMBMaster, MB_RTU, port_idx, baud_rate, MB_PAR_NONE) != MB_ENOERR) {
            UART_delay[port_idx] = UART_DEFAULT_DELAY_MS;
            return MB_EPORTERR;
        }
    } else if ((BoardCfgTab[port_idx].PortType == UART_PORT_CUSTOM) ||
               (BoardCfgTab[port_idx].PortType == UART_PORT_AXLE_COUNTER) ||
               (BoardCfgTab[port_idx].PortType == UART_PORT_DC3WIRE)) {
        /* 自定义串口模式: 使用 HAL 底层直接控制
         *   - UART_PORT_CUSTOM (1):      曲线模块的非 Modbus 协议
         *   - UART_PORT_AXLE_COUNTER (7): 计轴传感器非标 485 协议
         *   - UART_PORT_DC3WIRE (8):      直流3对线非标 485 协议
         */
        UART_Custom[port_idx] = 1;
        if (uart_init_custom_raw(port_idx, baud_rate) != MB_ENOERR) {
            UART_delay[port_idx] = UART_DEFAULT_DELAY_MS;
            return MB_EPORTERR;
        }
    } else {
        UART_delay[port_idx] = UART_DEFAULT_DELAY_MS;
        return MB_EINVAL;
    }

    /* 计算轮询周期: 基础 900ms, 每个从机减去 25ms */
    UART_delay[port_idx] = 900U - (uint32_t)BoardCfgTab[port_idx].SlaveModuelNum * 25U;
    return MB_ENOERR;
}

/*======================================================================*/
/*  uart_init_custom_raw()  —  自定义串口底层初始化                        */
/*                                                                       */
/*  功能: 为自定义协议模块 (0x37/0x38/0x39 曲线模块) 初始化 UART 硬件.      */
/*  与 Modbus 模式不同, 自定义模式需要直接控制 HAL UART 句柄,               */
/*  以便在 uart_custom_exchange 中手动管理收发和 RS485 DE 方向切换.         */
/*                                                                       */
/*  初始化内容:                                                           */
/*    1. 使能 UART 外设时钟                                               */
/*    2. 配置 TX/RX/DE 引脚                                               */
/*    3. 配置 UART 参数 (波特率/数据位/停止位/校验)                         */
/*    4. 将 HAL 句柄存入 uart_raw_handles[port_idx]                       */
/*    5. 将 DE 引脚信息存入 uart_raw_config[port_idx]                      */
/*                                                                       */
/*  6 路串口引脚映射:                                                     */
/*    端口     UART外设    TX      RX      EN(DE)                          */
/*    USART1  USART1     PA9     PA10    PA11                             */
/*    USART2  USART2     PD5     PD6     PD4                              */
/*    USART3  USART3     PB10    PB11    PH6                              */
/*    UART4   UART4      PC10    PC11    PA15                             */
/*    UART5   UART5      PC12    PD2     PA15                             */
/*    USART6  USART6     PC6     PC7     PC8                              */
/*======================================================================*/
eMBErrorCode uart_init_custom_raw(uint8_t port_idx, uint32_t baud_rate)
{
    UART_HandleTypeDef *huart;
    UART_RawConfig_t   *cfg;
    GPIO_InitTypeDef    gpio_init = {0};
    GPIO_TypeDef       *gpio_port_tx, *gpio_port_rx, *gpio_port_de;
    uint16_t            gpio_pin_tx, gpio_pin_rx, gpio_pin_de;
    uint8_t             af_tx, af_rx;
    USART_TypeDef      *uart_instance;

    if (port_idx >= UART_PORT_COUNT) {
        return MB_EINVAL;
    }

    huart = &uart_raw_handles[port_idx];
    cfg   = &uart_raw_config[port_idx];

    /*
     * 根据端口索引选择对应的 UART 外设、GPIO 和 AF.
     * 注意: USART1/USART6 在 APB2 上, USART2/3 和 UART4/5 在 APB1 上.
     */
    switch (port_idx) {
    case 0:  /* UART7: TX=PF7(AF8), RX=PF6(AF8), DE=PF8 */
        uart_instance   = UART7;
        gpio_port_tx    = GPIOF;  gpio_pin_tx = GPIO_PIN_7;   af_tx = GPIO_AF8_UART7;
        gpio_port_rx    = GPIOF;  gpio_pin_rx = GPIO_PIN_6;  af_rx = GPIO_AF8_UART7;
        gpio_port_de    = GPIOF;  gpio_pin_de = GPIO_PIN_8;
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_UART7_CLK_ENABLE();
        break;
    case 1:  /* USART2: TX=PD5(AF7), RX=PD6(AF7), DE=PD4 */
        uart_instance   = USART2;
        gpio_port_tx    = GPIOD;  gpio_pin_tx = GPIO_PIN_5;   af_tx = GPIO_AF7_USART2;
        gpio_port_rx    = GPIOD;  gpio_pin_rx = GPIO_PIN_6;   af_rx = GPIO_AF7_USART2;
        gpio_port_de    = GPIOD;  gpio_pin_de = GPIO_PIN_4;
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        break;
    case 2:  /* UART5: TX=PC12(AF8), RX=PD2(AF8), DE=PD3 */
        uart_instance   = UART5;
        gpio_port_tx    = GPIOC;  gpio_pin_tx = GPIO_PIN_12;  af_tx = GPIO_AF8_UART5;
        gpio_port_rx    = GPIOD;  gpio_pin_rx = GPIO_PIN_2;   af_rx = GPIO_AF8_UART5;
        gpio_port_de    = GPIOD;  gpio_pin_de = GPIO_PIN_3;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_UART5_CLK_ENABLE();
        break;
    case 3:  /* UART4: TX=PC10(AF8), RX=PC11(AF8), DE=PA15 */
        uart_instance   = UART4;
        gpio_port_tx    = GPIOC;  gpio_pin_tx = GPIO_PIN_10;  af_tx = GPIO_AF8_UART4;
        gpio_port_rx    = GPIOC;  gpio_pin_rx = GPIO_PIN_11;  af_rx = GPIO_AF8_UART4;
        gpio_port_de    = GPIOA;  gpio_pin_de = GPIO_PIN_15;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();
        break;
    case 4:  /* USART1: TX=PA9(AF7), RX=PA10(AF7), DE=PA11 */
        uart_instance   = USART1;
        gpio_port_tx    = GPIOA;  gpio_pin_tx = GPIO_PIN_9;   af_tx = GPIO_AF7_USART1;
        gpio_port_rx    = GPIOA;  gpio_pin_rx = GPIO_PIN_10;  af_rx = GPIO_AF7_USART1;
        gpio_port_de    = GPIOA;  gpio_pin_de = GPIO_PIN_11;
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        break;
    case 5:  /* USART6: TX=PC6(AF8), RX=PC7(AF8), DE=PC8 */
        uart_instance   = USART6;
        gpio_port_tx    = GPIOC;  gpio_pin_tx = GPIO_PIN_6;   af_tx = GPIO_AF8_USART6;
        gpio_port_rx    = GPIOC;  gpio_pin_rx = GPIO_PIN_7;   af_rx = GPIO_AF8_USART6;
        gpio_port_de    = GPIOC;  gpio_pin_de = GPIO_PIN_8;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART6_CLK_ENABLE();
        break;
    default:
        return MB_EINVAL;
    }

    /* ---- 配置 TX 引脚: 推挽复用输出 ---- */
    gpio_init.Pin       = gpio_pin_tx;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = af_tx;
    HAL_GPIO_Init(gpio_port_tx, &gpio_init);

    /* ---- 配置 RX 引脚: 浮空输入 ---- */
    gpio_init.Pin       = gpio_pin_rx;
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = af_rx;
    HAL_GPIO_Init(gpio_port_rx, &gpio_init);

    /* ---- 配置 DE 引脚: 推挽输出 (初始低电平 = 接收模式) ---- */
    gpio_init.Pin       = gpio_pin_de;
    gpio_init.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(gpio_port_de, &gpio_init);
    HAL_GPIO_WritePin(gpio_port_de, gpio_pin_de, GPIO_PIN_RESET);

    /* ---- 保存 DE 引脚信息 (供 uart_custom_exchange 使用) ---- */
    cfg->de_port = gpio_port_de;
    cfg->de_pin  = gpio_pin_de;

    /* ---- 配置 UART 外设 ---- */
    huart->Instance          = uart_instance;
    huart->Init.BaudRate     = baud_rate;
    huart->Init.WordLength   = UART_WORDLENGTH_8B;
    huart->Init.StopBits     = UART_STOPBITS_1;
    huart->Init.Parity       = UART_PARITY_NONE;
    huart->Init.Mode         = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        return MB_EPORTERR;
    }

    /*
     * 启用 NVIC 中断通道 (RXNEIE 在 uart_custom_exchange 中按需开关,
     * 仅在收发期间短暂开启, 避免总线噪声引发持续中断).
     *
     * 优先级 2: 与 FreeModbus 串口 ISR 一致, 高于 TIM6(3), 低于 SysTick(0).
     */
    HAL_NVIC_SetPriority(uart_irq_numbers[port_idx], 2, 0);
    HAL_NVIC_EnableIRQ(uart_irq_numbers[port_idx]);
    /* 注意: RXNEIE 不在此处开启, 由 uart_custom_exchange 管理 */

    /*
     * 将 UART 实例指针注入 mbport 层的 arxSerialPorts[],
     * 使 prvUARTIRQHandler 可以通过 pxSerial->huart.Instance 访问
     * 真实的 UART 寄存器 (SR/DR).  若此处缺失, Instance 为 NULL,
     * ISR 解引用空指针 → 读取 Flash 0x00 的垃圾 → 中断风暴 → 看门狗复位.
     */
    mbport_set_custom_uart_instance(huart->Instance);

    return MB_ENOERR;
}


/*======================================================================*/
/*  七、缓冲区管理                                                        */
/*======================================================================*/

/**
 * @brief  初始化端口数据帧头
 * @note   设置 SerialDataBuf 的前 11 字节 (帧头):
 *         [0]=端口号, [1]=主类型, [2]=子类型, [3]=帧标志(0x00),
 *         [4]=分机号, [5-6]=长度(先清零), [7-10]=链路状态掩码(清零)
 *         同时初始化报警温度为全 1 (无报警状态).
 * @param  port_idx  端口索引
 */
void uart_setup_header(int port_idx)
{
    if ((port_idx < 0) || (port_idx >= UART_PORT_COUNT)) return;

    SerialDataBuf[port_idx][0] = BoardCfgTab[port_idx].PortNum;
    SerialDataBuf[port_idx][1] = BoardCfgTab[port_idx].MainProType;
    SerialDataBuf[port_idx][2] = BoardCfgTab[port_idx].SubProType;
    SerialDataBuf[port_idx][3] = 0x00;   /* 帧标志: 由采集任务设置 */
    SerialDataBuf[port_idx][4] = BoardCfgTab[port_idx].FenjiNum;
    SerialDataBuf[port_idx][5] = 0x00;   /* 数据长度低字节 */
    SerialDataBuf[port_idx][6] = 0x00;   /* 数据长度高字节 */
    *((uint32_t *)&SerialDataBuf[port_idx][7]) = 0;  /* 链路状态掩码 */
    ALARM_TEMP[port_idx] = 0xFFFFFFFFUL;  /* 初始值: 全 1 = 无报警 */
}

/**
 * @brief  等待端口配置完成
 * @note   阻塞等待直到:
 *         1. download_flag == 1 (上位机已下发配置)
 *         2. PortType != 0 (未禁用) 且 != 3 (从机模式)
 *         3. SlaveModuelNum != 0 (该端口已配置从机)
 *         等待期间持续清零 WDT 计数器, 防止看门狗误复位.
 * @param  port_idx  端口索引
 */
void uart_wait_until_configured(uint8_t port_idx)
{
    while ((port_idx >= UART_PORT_COUNT) ||
           (BoardCfgTab[port_idx].PortType == UART_PORT_DISABLED) ||
           (BoardCfgTab[port_idx].PortType == UART_PORT_SLAVE) ||
           (BoardCfgTab[port_idx].SlaveModuelNum == 0) ||
           (download_flag == 0)) {
        if (port_idx < UART_PORT_COUNT) {
            WDTUARTFLAG[port_idx] = 0;  /* 喂狗: 告知看门狗本任务存活 */
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        
    }
}

/**
 * @brief  计算端口在 Holding 寄存器区间的基地址
 * @note   每端口预留 UART_HOLDING_REG_STRIDE (500) 个寄存器空间.
 *         Port0: 0-499, Port1: 500-999, ... Port5: 2500-2999.
 * @param  port_idx  端口索引
 * @return Holding 寄存器基地址偏移
 */
uint16_t uart_holding_base(uint8_t port_idx)
{
    return (uint16_t)(port_idx * UART_HOLDING_REG_STRIDE);
}


/*======================================================================*/
/*  八、模块描述表查询                                                    */
/*======================================================================*/

/**
 * @brief  根据模块类型查找模块描述表
 * @param  module_type  模块类型编号 (0x31-0x39)
 * @return 指向描述表条目的指针, 未匹配时返回哨兵条目
 */
const UART_ModuleProfile_t *uart_get_module_profile(uint8_t module_type)
{
    uint32_t i;
    for (i = 0; i < (sizeof(uart_module_profiles) / sizeof(uart_module_profiles[0])); i++) {
        if (uart_module_profiles[i].module_type == module_type) {
            return &uart_module_profiles[i];
        }
    }
    return &uart_module_profiles[0]; /* 回退: 使用信号机开关量 (XHJ_KGL) 的配置 */
}

/**
 * @brief  判断模块是否为自定义曲线类型
 * @param  module_type  模块类型编号
 * @return 1=曲线模块, 0=普通 Modbus 模块
 */
uint8_t uart_profile_is_custom_curve(uint8_t module_type)
{
    return uart_get_module_profile(module_type)->uses_custom_curve;
}


/*======================================================================*/
/*  九、自定义串口数据交换 (核心函数)                                      */
/*                                                                       */
/*  uart_custom_exchange() 是自定义曲线模块通信的底层函数.                 */
/*  它完成一次完整的"发送请求 → 等待应答 → 接收数据"过程.                  */
/*                                                                       */
/*  通信流程 (RS485 半双工):                                               */
/*    1. 清除 UART 错误标志 (ORE/FE/NE)                                   */
/*    2. 拉高 DE 引脚 (切换到发送模式)                                     */
/*    3. HAL_UART_Transmit 发送数据                                       */
/*    4. 等待 TC (发送完成) 标志                                          */
/*    5. 拉低 DE 引脚 (切换到接收模式)                                     */
/*    6. 逐字节接收, 首字节超时 first_byte_timeout_ms                     */
/*    7. 后续字节超时 inter_byte_timeout_ms (字节间隔超时 = 接收完成)      */
/*                                                                       */
/*  使用 uart_raw_handles[port_idx] 和 uart_raw_config[port_idx],         */
/*  这两个数组由 uart_init_custom_raw() 初始化.                            */
/*======================================================================*/

/**
 * @brief  自定义串口数据交换 (发送 + 接收)
 * @param  port_idx                 端口索引 (0-5)
 * @param  tx_buf                   发送数据缓冲区
 * @param  tx_len                   发送数据长度
 * @param  rx_buf                   接收数据缓冲区
 * @param  rx_max_len               接收缓冲区最大容量
 * @param  first_byte_timeout_ms    首字节超时 (ms)
 * @param  inter_byte_timeout_ms    字节间隔超时 (ms), 超时即认为接收完成
 * @return 实际接收到的字节数, 0 表示失败
 */
uint16_t uart_custom_exchange(uint8_t port_idx, const uint8_t *tx_buf, uint16_t tx_len,
                              uint8_t *rx_buf, uint16_t rx_max_len,
                              uint32_t first_byte_timeout_ms, uint32_t inter_byte_timeout_ms)
{
    UART_HandleTypeDef *huart;
    const UART_RawConfig_t *cfg;
    uint16_t rx_len = 0;

    /* ---- 参数校验 ----
     * 允许纯发送调用 (rx_buf==NULL 且 rx_max_len==0, 如校时命令无需应答).
     * 修复: 此前 rx_buf==NULL/rx_max_len==0 直接返回, 导致校时命令从未真正发出. */
    if ((port_idx >= UART_PORT_COUNT) || (tx_buf == NULL) || (tx_len == 0U)) {
        return 0U;
    }

    /*
     * 获取本端口的 HAL UART 句柄和 DE 引脚配置.
     * 这两个数组由 uart_init_custom_raw() 初始化 ——
     * 在 uart_wait_until_configured → uart_init_port → uart_init_custom_raw 调用链中完成.
     */
    huart = &uart_raw_handles[port_idx];
    cfg   = &uart_raw_config[port_idx];

    /* 安全检查: 如果 UART 外设未初始化 (Instance 仍为 NULL), 直接返回失败 */
    if (huart->Instance == NULL) {
        return 0U;
    }

    /* ---- 等待总线静默 + 排空所有残留字节, 连带清除所有错误标志 ----
     * [为什么必须等静默] 从机被放弃 (首字节超时) 后, 它的应答仍会在几十~几百 ms
     *   后到达. 若此时本机已向下一台从机发出请求, 两条帧在半双工总线上重叠,
     *   双方的 CRC 全部失败 → 表现为"随机几台采集盒同时掉线, 下一秒又恢复",
     *   掉线地址不固定. 只排空 DR 是不够的: 排空只清掉"已经到了的"字节, 挡不住
     *   "还没到的"迟到应答. 必须先确认总线真的安静了再发下一帧.
     * [怎么判] 边排空边记录"最后一次收到字节的时刻", 只有连续静默
     *   UART_BUS_QUIET_MS 才认为总线空闲; 静默期间收到的迟到字节全部丢弃.
     *   每次读 DR 会同时清除 RXNE/ORE/FE/NE (HAL 的 CLEAR_FEFLAG/CLEAR_NEFLAG
     *   宏只读 SR 不读 DR, 清不掉 FE/NE, 故直接读 DR).
     * [为什么不污染 ring] 进入本函数时 RXNEIE 已被上一次交换关闭且 rx_active=0,
     *   字节只落 DR 不进 ring buffer, 故此处轮询 DR 是安全的.
     * [代价] 总线本来就空闲时, 每次交换多花 ~UART_BUS_QUIET_MS (5ms);
     *   6 台 ≈ 30ms/轮, 相对 1s 轮询预算可忽略 (字节间隔超时 100ms→10ms 已省下
     *   540ms). 也不能把窗口压到 1~2ms: 任务侧是 1ms tick 粒度采样, 判定窗口
     *   必须明显大于调度抖动, 否则会把"采样没赶上"误判成"总线已静默".
     * [兜底] 总线被拉死或持续噪声时, 累计等待超过 UART_BUS_QUIET_MAX_MS 强制
     *   放行: 宁可带着碰撞风险试一次, 也不能在此死等把整个端口拖停.
     * [残余风险] 若某台从机的应答晚到超过 UART_BUS_QUIET_MAX_MS (极端慢), 仍会
     *   与本次发送碰撞. 该情形只能靠首字节超时给足余量 (见调用处) 来避免. */
    {
        uint32_t quiet_start = HAL_GetTick();
        uint32_t last_rx     = quiet_start;

        do {
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
                (void)huart->Instance->DR;    /* 丢弃迟到字节, 顺带清错误标志 */
                last_rx = HAL_GetTick();
            }
            if ((HAL_GetTick() - last_rx) >= UART_BUS_QUIET_MS) {
                break;                        /* 已连续静默 → 总线空闲 */
            }
            vTaskDelay(pdMS_TO_TICKS(1));     /* 让出 CPU, 不忙等饿死同级任务 */
        } while ((HAL_GetTick() - quiet_start) < UART_BUS_QUIET_MAX_MS);
    }
    /* 排空后再走一遍标准清理 (处理极端情况: DR 空但 SR 还有残余标志) */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* ---- 步骤 1-2: 拉高 DE, 切换到发送模式 ---- */
    HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_SET);

    /* ---- 步骤 3-5: 中断驱动发送 (TXE/TC 中断, 抗任务抢占) ----
     * [为什么改] 原 HAL_UART_Transmit 是阻塞轮询发送: 请求帧 (8 字节 @9600
     *   ≈ 9.2ms) 发送期间, 本任务 (prio 5) 若被更高优先级任务 (tcpip_thread=7,
     *   App_Task_TcpSend=6, App_TaskStart=8) 或同级时间片 (1ms) 抢占停顿超过
     *   从机 T3.5 (9600≈4ms), 帧被拆成两段. 从机按 Modbus RTU 帧间超时把两段
     *   判为无效帧整体丢弃 → 该地址轮询无响应 → 上传数据全 FFFF
     *   (道岔采集盒随机丢请求帧).
     * [怎么改] 字节改由 TXE/TC 中断 (prio 2) 按波特率逐字节移出, 与任务调度
     *   完全解耦, 不存在被抢占拆帧的可能.
     * [关键点1] 先关 TCIE 再开 TXEIE: 防止上一次发送残留的 TC 标志在后续开
     *   TCIE 时立即误触发, 提前拉低 DE 导致帧未发完就切入接收.
     * [关键点2] TC 中断负责拉低 DE 并立即开启 RXNE 接收窗口: 消除 DE→RX 切换
     *   间隙. 即使任务在 vTaskDelay 中尚未醒来, 从机快速应答的首字节也会被 ISR
     *   直接收入 ring buffer, 不会丢首字节.
     * [时序] DE高 → 开TXE → (ISR 发完 8 字节) → 末字节移出触发 TC → ISR 拉低
     *   DE + 开 RXNE → 任务轮询到 tx_done → 进入接收等待. */
    uart_custom_tx[port_idx].tx_buf    = tx_buf;
    uart_custom_tx[port_idx].tx_len    = tx_len;
    uart_custom_tx[port_idx].tx_idx    = 0;
    uart_custom_tx[port_idx].tx_active = 1;
    uart_custom_tx[port_idx].tx_done   = 0;
    uart_custom_tx[port_idx].de_port   = cfg->de_port;
    uart_custom_tx[port_idx].de_pin    = cfg->de_pin;

    __HAL_UART_DISABLE_IT(huart, UART_IT_TC);    /* 防残留 TCIE 误触发 */
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);  /* 清残留 TC 标志      */
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);    /* DR 空 → 立即进入首次 TXE 中断 */

    /* 等待发送完成: TC 中断拉低 DE, 开启接收窗口并置位 tx_done.
     * vTaskDelay(1) 让出 CPU: 发送由 ISR 驱动, 任务阻塞等待不影响帧连续性;
     * 期间更高优先级任务可正常运行 (这正是中断驱动发送的意义). */
    {
        uint32_t tx_start = HAL_GetTick();
        while (uart_custom_tx[port_idx].tx_done == 0U) {
            if ((HAL_GetTick() - tx_start) > first_byte_timeout_ms) {
                /* 发送超时: 中止并回退到接收模式.
                 * 8 字节请求在 first_byte_timeout_ms(80~100ms) 内必然发完, 此分支
                 * 仅在 UART 硬件异常/总线被拉死时触发. 关掉 TXE/TC 中断、DE 拉低,
                 * 返回 0 由上层按"该从机离线"处理, 下一轮轮询重试. */
                __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
                __HAL_UART_DISABLE_IT(huart, UART_IT_TC);
                uart_custom_tx[port_idx].tx_active = 0;
                uart_custom_tx[port_idx].tx_done   = 0;
                HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_RESET);
                return 0U;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    uart_custom_tx[port_idx].tx_active = 0;

    /* ---- 纯发送调用 (rx_buf==NULL, 如校时命令不期待应答): 直接返回 ----
     * 修复: 原逻辑即使 rx_buf==NULL 也会进入下方"等待首字节"循环, 白等
     *       first_byte_timeout_ms(100ms) 才超时返回, 校时期间每模块浪费100ms.
     *       此处关闭接收窗口后立即返回, 残余应答字节由下一次交换前的
     *       排空循环 (上方 DR 循环) 清掉. */
    // if (rx_buf == NULL) {
    //     __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    //     uart_rx_ring[port_idx].rx_active = 0;
    //     return 0U;
    // }

    /* ---- 步骤 6-7: 中断驱动接收 (ring buffer 流式排空 + vTaskDelay) ---- */
    /*
     * ring buffer 已由 TC 中断复位并开启 RXNEIE (见 mbportserial.c 自定义分支),
     * 此处只等待数据, 不再重复初始化 ring —— 避免任务在 vTaskDelay 醒来后重新
     * 初始化 ring, 与 ISR 竞争把已经收入的应答字节清掉.
     * 首字节由 first_byte_timeout_ms 兜底; 帧结束按 inter_byte_timeout_ms 字节
     * 间隔超时判定 (与 Modbus T3.5 同理). 任务全程 vTaskDelay 让出 CPU, 不会
     * 与同优先级任务互相抢占导致 ORE 丢字节.
     *
     * [为什么改成流式排空] 原实现是"整帧收完再从 ring 拷到 rx_buf": 接收期间
     *   任务只等 ring->count 停止增长, 一个字节都不搬走, 因此 ring 必须能装下
     *   "完整一帧", 单帧长度被死死限制在 UART_RX_RING_SIZE (512B). 计轴脉冲
     *   帧(应答 = 2B头 + 寄存器块 + 脉冲块 + 2B CRC, 高频计轴 0x53 光寄存器块
     *   就 64B, 再按 reg13 一次性回多个脉冲)轻易超过 512B, ISR 在 ring 满后
     *   静默丢字节 (见 mbportserial.c 的 count 上限判断) → 帧尾 CRC 丢失 →
     *   整帧 CRC 校验失败 → 上层按"模块离线"处理 → 应答里的通讯确认码根本没
     *   读到, 水位 ACK 永远不推进 → 采集盒不清脉冲缓存 → 下一轮回同样的超长
     *   帧 → 永久死锁.
     * [怎么改] 每轮循环把 ring 里已到的字节立刻搬进 rx_buf 并推进 tail, 腾出
     *   ring 空间; ring 退化为抖动缓冲, 只用于吸收任务调度延迟, 单帧长度不再
     *   受 UART_RX_RING_SIZE 限制 (上限变为调用方给的 rx_max_len).
     * [帧结束判定] 不能再用"ring 非空"判断有没有新字节 —— 本循环会把字节搬走.
     *   改用"累计到达字节数" = copied + ring 当前占用, 搬空 ring 后保持恒定,
     *   只有 ISR 收入新字节时才增长; 帧尾仍按字节间隔超时判定 (见下方 ③).
     */
    {
        UART_RxRing_t *ring = &uart_rx_ring[port_idx];
        uint32_t       start_tick;
        uint32_t       last_byte_tick;
        uint16_t       arrived;      /* 上一轮采样到的"累计到达字节数" */
        uint16_t       copied = 0U;  /* 已经搬进 rx_buf 的字节数        */
        /* 本帧 ISR 侧计数基线 (帧长异常打印差值, 见下方 [DC-SHORT]) */
        uint16_t       dc_store0 = dc_rx_store_cnt[port_idx];
        uint16_t       dc_ore0   = dc_rx_ore_cnt[port_idx];
        uint16_t       dc_steal0 = dc_rx_steal_cnt[port_idx];

        /* 等待首字节 (first_byte_timeout_ms 超时) */
        start_tick = HAL_GetTick();
        while (UART_RX_RING_USED(ring) == 0U) {
            if ((HAL_GetTick() - start_tick) > first_byte_timeout_ms) {
                __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
                ring->rx_active = 0;
                return 0U;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        last_byte_tick = HAL_GetTick();
        arrived        = UART_RX_RING_USED(ring);

        for (;;) {
            uint16_t used = UART_RX_RING_USED(ring);

            /* ① 流式排空: 把 ring 中已到的字节搬进 rx_buf 并推进 tail.
             *    ring 是严格 SPSC (ISR 只写 buffer/head, 本循环只写 tail),
             *    两个上下文没有共享变量的读改写, 所以**这里不需要任何临界区**.
             *    原实现要读改写 count, 靠 __disable_irq() 互斥; 但 count 的"读"
             *    被编译器提到了 CPSID 之前, 临界区只包住"写" —— ISR 在那几条指令
             *    间隙里存进字节, 它那次 count++ 就被回写的老值覆盖, count 少 1,
             *    帧尾字节永远取不出来 (现场 [DC-TAIL] h=143 t=142 cnt=0 即此).
             *    used 是本轮开始时的快照, 排空过程中 ISR 再存进来的字节由下一轮取走. */
            while ((used != 0U) && (copied < rx_max_len)) {
                rx_buf[copied++] = ring->buffer[ring->tail];
                ring->tail       = (uint16_t)((ring->tail + 1U) & UART_RX_RING_MASK);
                last_byte_tick   = HAL_GetTick();
                used--;
            }

            /* ② rx_buf 已满: 不能再收, 结束本次交换 (调用方按截断帧处理) */
            if (copied >= rx_max_len) {
                break;
            }

            /* ③ 帧结束判定: 字节间隔超时 (inter_byte_timeout_ms)
             *    [为什么不用应答头里的字节数提前收帧] 这些采集盒的帧型里, 只有
             *    0x8A 交流道岔帧的第 3 字节是字节数; 计轴(0x50/0x53)、直流三对线
             *    (0x51/0x52) 的应答是 [addr][03] + 数据, 第 3 字节就是**数据**
             *    (main.c 里标注 "无 byte_count"). 拿数据当长度算 → 收满就会提前
             *    收帧把长帧截断, 虽然要 CRC 巧合才真截断, 但按帧率折算一年也有
             *    若干次, 不能接受. 故一律以字节间隔超时为准.
             *    [想再快] 请把调用处的 inter_byte_timeout_ms 从 100ms 调到 ~20ms
             *    (19200 下字节间隔仅 0.52ms, 20ms 有 38 倍余量), 不要去信长度字节. */
            {
                uint16_t now_arrived = (uint16_t)(copied + UART_RX_RING_USED(ring));

                if (now_arrived != arrived) {
                    /* 有新字节到达, 重置超时计时器 */
                    arrived        = now_arrived;
                    last_byte_tick = HAL_GetTick();
                } else if ((HAL_GetTick() - last_byte_tick) > inter_byte_timeout_ms) {
                    /* 字节间隔超时 → 帧接收完成 */
                    break;
                }
            }

            /* ④ 暂无新字节/未到超时 → 让出 CPU, 下一轮再排空 */
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /* ---- 帧长异常哨兵: 交流道岔 0x8A 应答应为 143B, 少了就是又丢字节了 ----
         * 背景 (2026-09-28 现场): 调试助手与主站挂同一条 485 总线, 抓到的应答在线
         *   上是完整 143B 且 CRC 逐帧自洽, 主站却只收到 142B —— 丢在主站侧. 根因是
         *   ring 的 count 读改写有竞态 (见 bsp_rs485.h 里 UART_RxRing_t 的说明),
         *   已改为 SPSC 无 count 结构.
         * **修好后这里应该永不打印.** 一旦又打出来, 这行给出全部现场证据:
         *   len      : 本次实际收帧长度 (期望 143 = 3 + 0x8A + 2)
         *   store/ore/steal: ISR 入 ring 次数 / 硬件 ORE 次数 / 错误标志块抢存次数
         *   used/h/t : 收帧瞬间 ring 占用与读写指针 (SPSC 下正常恒有 h-t == used)
         *   sr/cr1/de: 收帧瞬间 UART 状态与 DE 引脚电平 (de=1 说明没切回接收)
         * 下面的兜底让现场数据先能用, 定位后连兜底一起删. */
        if ((copied >= 4U) && (copied < rx_max_len)
            && (rx_buf[1] == 0x03U) && (rx_buf[2] == 0x8AU)) {
            const uint16_t dc_need = (uint16_t)(3U + 0x8AU + 2U);   /* 143 */

            if (copied != dc_need) {
                static uint8_t s_tail_rep[UART_PORT_COUNT] = { 0 };

                if (s_tail_rep[port_idx] == 0U) {
                    s_tail_rep[port_idx] = 1U;
                    printf("[DC-SHORT] port=%u len=%u/%u store=%u ore=%u steal=%u"
                           " ring(used=%u h=%u t=%u) sr=%04X cr1=%04X de=%u\r\n",
                           (unsigned)(port_idx + 1U), (unsigned)copied,
                           (unsigned)dc_need,
                           (unsigned)(uint16_t)(dc_rx_store_cnt[port_idx] - dc_store0),
                           (unsigned)(uint16_t)(dc_rx_ore_cnt[port_idx] - dc_ore0),
                           (unsigned)(uint16_t)(dc_rx_steal_cnt[port_idx] - dc_steal0),
                           (unsigned)UART_RX_RING_USED(ring),
                           (unsigned)ring->head, (unsigned)ring->tail,
                           (unsigned)huart->Instance->SR,
                           (unsigned)huart->Instance->CR1,
                           (unsigned)HAL_GPIO_ReadPin(cfg->de_port, cfg->de_pin));
                }

                /* 兜底: 只缺末字节且 CRC 低字节吻合 → 补出高字节, 上层原有 16 位
                 * CRC 校验照常通过 (此路兜底后校验强度只剩 8 位, 故必须尽快定位) */
                if ((copied == (uint16_t)(dc_need - 1U)) && (copied < rx_max_len)) {
                    uint16_t crc = uart_crc16(rx_buf, (uint16_t)(copied - 1U));

                    if ((uint8_t)(crc & 0x00FFU) == rx_buf[copied - 1U]) {
                        static uint8_t s_tail_fix_rep[UART_PORT_COUNT] = { 0 };

                        rx_buf[copied] = (uint8_t)((crc >> 8) & 0x00FFU);   /* 补出 CRCH */
                        copied++;

                        if (s_tail_fix_rep[port_idx] == 0U) {
                            s_tail_fix_rep[port_idx] = 1U;
                            printf("[DC-FIX] port=%u cnt=%u CRC-low fallback applied\r\n",
                                   (unsigned)(port_idx + 1U), (unsigned)rx_buf[2]);
                        }
                    }
                }
            }
        }

        /* ---- 帧尾兜底: UART 里若还压着没被 ISR 收走的字节, 直接补进 rx_buf ----
         * 正常情况 RXNE 中断会在收到字节的当下存进 ring, 这里还能读到 RXNE, 说明
         * 那个字节被漏收了 (ISR 被屏蔽/被错误标志块吃掉) —— 不补就是"末字节消失"
         * 的直接原因。读 SR→读 DR 同时清掉 RXNE/ORE/FE/NE. */
        if ((copied < rx_max_len) && ((huart->Instance->SR & USART_SR_RXNE) != 0U)) {
            rx_buf[copied++] = (uint8_t)(huart->Instance->DR & 0xFFU);
        }

        rx_len = copied;
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        ring->rx_active = 0;   /* 通知 ISR: 接收完毕 */
    }

    return rx_len;
}


/*======================================================================*/
/*  十、曲线模块校时                                                      */
/*======================================================================*/

/**
 * @brief  向指定曲线模块发送校时命令
 * @note   使用功能码 0x10 (写多个寄存器), 将 4 字节 Unix 时间戳写入模块.
 *         命令帧: [addr, 0x10, 0x0000, 0x0002, 0x04, time(4B), CRC16]
 *         共 13 字节.
 * @param  port_idx   端口索引
 * @param  com_port   串口号 (与 port_idx 相同, 保留参数)
 * @param  slave_addr 从机地址
 * @param  sys_time   系统时间 (Unix 时间戳)
 * @return 始终返回 1 (兼容原程序)
 */
uint8_t uart_custom_send_adjust_time(uint8_t port_idx, uint8_t com_port,
                                     uint8_t slave_addr, uint32_t sys_time)
{
    uint8_t  cmd[13];
    uint16_t crc;

    /* 构建校时命令: 功能码 0x10 (Write Multiple Registers) */
    cmd[0]  = slave_addr;                         /* 从机地址            */
    cmd[1]  = 0x10;                               /* 功能码: 写多寄存器   */
    cmd[2]  = 0x00;                               /* 起始地址高字节       */
    cmd[3]  = 0x00;                               /* 起始地址低字节       */
    cmd[4]  = 0x00;                               /* 寄存器数量高字节     */
    cmd[5]  = 0x02;                               /* 寄存器数量低字节 (2) */
    cmd[6]  = 0x04;                               /* 数据字节数 (4)      */
    cmd[7]  = (uint8_t)((sys_time >> 24) & 0xFFU); /* 时间戳 byte3 (最高) */
    cmd[8]  = (uint8_t)((sys_time >> 16) & 0xFFU); /* 时间戳 byte2        */
    cmd[9]  = (uint8_t)((sys_time >> 8)  & 0xFFU); /* 时间戳 byte1        */
    cmd[10] = (uint8_t)(sys_time & 0xFFU);         /* 时间戳 byte0 (最低) */
    crc     = uart_crc16(cmd, 11);
    cmd[11] = (uint8_t)(crc & 0xFFU);              /* CRC 低字节          */
    cmd[12] = (uint8_t)((crc >> 8) & 0xFFU);       /* CRC 高字节          */

    /* 发送校时命令, 并把盒子的应答读掉.
     * 盒子手册 4.1: 0x10 命令有返回帧 —— [addr,10,00,00,00,02,CRCL,CRCH] 共 8 字节.
     * 原来传 rx_buf=NULL: 发完立即返回, 盒子几~几十 ms 后到达的应答正好撞进紧接着
     * 发出的本模块 0x03 请求 (静默窗口只有几 ms, 挡不住"还没到"的应答) → 请求被
     * 撞坏 → 从机不回帧 → 该地址当轮离线.
     * 现场表现: 一按上位机"发送校时", 就随机掉几台 (13 台 × 6 端口 = 78 次撞车机会). */
    {
        uint8_t ack[16];   /* 收下并丢弃 (帧尾用 30ms 字节间隔判定) */
        (void)uart_custom_exchange(port_idx, cmd, 13, ack, sizeof(ack), 200, 30);
    }
    WDTUARTFLAG[port_idx] = 0;  /* 喂狗 */
    return 1;
}


/*======================================================================*/
/*  十一、曲线帧处理                                                      */
/*                                                                       */
/*  曲线模块 (0x37/0x38/0x39) 的帧类型由响应数据中的标志字节决定:          */
/*    0x00 — 普通实时帧: 只有实时数据, 无曲线数据                         */
/*    0x55 — 中间曲线帧: 包含一段曲线数据, 缓存到 SDRAM                   */
/*    0xFF — 结束曲线帧: 包含最后一段曲线数据, 缓存后统一打包上传          */
/*                                                                       */
/*  曲线存储格式 (SDRAM QX 区域, 每分支 0x400 字节):                      */
/*    [0-5]:   Flash_time 头部 (地址 + 分支 + 年月日时分秒)               */
/*    [6+...]: 曲线数据点 (每点 2 字节, 大端序)                            */
/*======================================================================*/

/**
 * @brief  处理普通帧 (flag == 0x00)
 * @note   如果之前存在未完成的曲线 (Frame_mark == 0xFF 且 Curve_number == 1),
 *         说明上位机可能已复位, 需要清理残留状态.
 *         无论如何, 最终将 Frame_mark/Frame_sequence/Weibo_number 全部归零.
 * @param  port_idx    端口索引
 * @param  module_idx  模块索引
 */
void uart_handle_curve_normal_frame(uint8_t port_idx, uint8_t module_idx)
{
    /*
     * 检查是否有未完成的曲线 (上次传输在结束帧之前中断).
     * 如果有, 清理曲线编号并保存到 EEPROM (与上位机保持同步).
     */
    if ((AC_module_infor[port_idx].Frame_mark[module_idx] == 0xFFU) &&
        (AC_module_infor[port_idx].Curve_number[module_idx] == 1U) &&
        (AC_module_infor[port_idx].Points_number[module_idx] < CURVE_MAX_POINTS)) {
        /* 标记曲线传输完成, 曲线编号归零 */
        AC_module_infor[port_idx].Frame_mark[module_idx] = 0;
        AC_module_infor[port_idx].Frame_sequence[module_idx] = 0;
        AC_module_infor[port_idx].Curve_number[module_idx] = 0;
        /* 保存到 EEPROM (掉电保护) */
        EEPROM_Write(0, (uint32_t)(5U + (uint32_t)port_idx * 2U),
                     (void *)&AC_module_infor[port_idx].Curve_number[0],
                     MODE_8_BIT, CURVE_EEPROM_SIZE);
    }
    /* 全部归零, 开始新一轮采集 */
    AC_module_infor[port_idx].Frame_mark[module_idx] = 0;
    AC_module_infor[port_idx].Frame_sequence[module_idx] = 0;
    Weibo_number[port_idx][module_idx] = 0;
}

/**
 * @brief  处理中间曲线帧 (flag == 0x55)
 * @note   将本帧中的曲线数据拷贝到 SDRAM 对应分支区域.
 *         如果是首帧 (is_first), 先写入 Flash_time 时间戳头部.
 *         每个分支拷贝 weibo_temp 字节数据.
 * @param  port_idx    端口索引
 * @param  module_idx  模块索引
 * @param  profile     模块描述表 (含各偏移量)
 * @param  ser_buf     完整 SerialDataBuf (含帧头 + 响应数据)
 * @param  is_first    是否为首帧 (1=是, 0=否)
 * @param  flash_time  时间戳头部 [addr, branch, 年/月, 日/时, 分, 秒]
 */
void uart_handle_curve_continue_frame(uint8_t port_idx, uint8_t module_idx,
                                      const UART_ModuleProfile_t *profile,
                                      uint8_t *ser_buf, uint8_t is_first,
                                      uint8_t flash_time[6])
{
    uint16_t weibo_temp;
    uint16_t *qx = uart_qx_ptr(port_idx, module_idx);
    uint8_t  n;
    const uint8_t *resp;  /* 响应数据起始 = ser_buf + 11 (跳过 TCP 帧头) */

    resp = ser_buf + 11U;

    /* 更新帧序号 (16 位大端序) 和帧标记 */
    AC_module_infor[port_idx].Frame_sequence[module_idx] =
        ((uint16_t)resp[profile->curve_seq_offset] << 8)
      | (uint16_t)resp[profile->curve_seq_offset + 1U];
    AC_module_infor[port_idx].Frame_mark[module_idx] = 0x55U;

    /* 首帧: 清零微步计数, 写入时间戳头部 */
    if (is_first) {
        Weibo_number[port_idx][module_idx] = 0;
        AC_module_infor[port_idx].Curve_number[module_idx] = 1;

        for (n = 0; n < profile->curve_branch_count; n++) {
            /*
             * flash_time[1] (分支号):
             *   AC_ZZJ (0x37):  = n + 1 (分支 1/2/3/4)
             *   DC 类型 (0x38/0x39): = 5 (固定值)
             * flash_time[0] (地址):
             *   DC_ZZJ6 (0x39): = 基地址 + n (每个分支对应一个从机地址)
             *   其他: 保持不变
             */
            if ((profile->module_type == FLFXG_AC_ZZJ) ||
                (profile->module_type == FLFXG_AC_ZZJ_HIGH) ||
                (profile->module_type == FLFXG_S700K_C) ||
                (profile->module_type == FLFXG_S700K_C_HIGH)) {
                flash_time[1] = n + 1;
            } else {
                flash_time[1] = 5;
            }
            if ((profile->module_type == FLFXG_DC_ZZJ6) ||
                (profile->module_type == FLFXG_DC_ZZJ6_HIGH)) {
                flash_time[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx] + n;
            }
            /* 写入 6 字节时间戳头部到 SDRAM 对应分支区域 */
            memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)),
                   flash_time, 6);
        }
    }

    /* 计算本帧曲线数据字节数: 长度 (16 位大端序, 点数) * 2 (每点 2 字节) */
    weibo_temp = (uint16_t)(((uint16_t)resp[profile->curve_len_offset] << 8)
                           | (uint16_t)resp[profile->curve_len_offset + 1U]) * 2U;

    /* 将各分支曲线数据写入 SDRAM (追加到已有数据之后) */
    for (n = 0; n < profile->curve_branch_count; n++) {
        memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)
                           + 3U + Weibo_number[port_idx][module_idx] / 2U),
               &resp[profile->curve_data_offset + n * 50U],
               weibo_temp);
    }
    Weibo_number[port_idx][module_idx] += weibo_temp;
    AC_module_infor[port_idx].Points_number[module_idx] =
        Weibo_number[port_idx][module_idx];
}

/**
 * @brief  处理结束曲线帧 (flag == 0xFF)
 * @note   1. 缓存最后一段曲线数据到 SDRAM (同 continue 帧逻辑)
 *         2. 拷贝转动方向字 (direction word) 到最后一个分支的数据末尾
 *         3. 将各分支的完整曲线数据打包成帧, 通过 TCP 上传
 *         4. 曲线帧格式: 帧头(11) + 曲线数据(N+6) + 方向字(2) = N+19 字节
 * @param  port_idx    端口索引
 * @param  module_idx  模块索引
 * @param  profile     模块描述表
 * @param  ser_buf     完整 SerialDataBuf
 * @param  weibo_len   当前 Weibo_number (本帧之前的累计微步数)
 */
void uart_handle_curve_end_frame(uint8_t port_idx, uint8_t module_idx,
                                 const UART_ModuleProfile_t *profile,
                                 uint8_t *ser_buf, uint16_t weibo_len)
{
    uint16_t weibo_temp;
    uint16_t *qx = uart_qx_ptr(port_idx, module_idx);
    uint8_t  n;
    const uint8_t *resp;  /* 响应数据起始 = ser_buf + 11 (跳过 TCP 帧头) */

    resp = ser_buf + 11U;

    /* 更新帧序号 (16 位大端序) 和帧标记 */
    AC_module_infor[port_idx].Frame_sequence[module_idx] =
        ((uint16_t)resp[profile->curve_seq_offset] << 8)
      | (uint16_t)resp[profile->curve_seq_offset + 1U];
    AC_module_infor[port_idx].Frame_mark[module_idx] = 0xFFU;

    /* 首帧检测: 传感器每个曲线周期的第一帧序号总是 1 */
    {
        uint16_t seq_word = ((uint16_t)resp[profile->curve_seq_offset] << 8)
                          | (uint16_t)resp[profile->curve_seq_offset + 1U];
        if (seq_word == 1U) {
            uint8_t ft[6];
            Weibo_number[port_idx][module_idx] = 0;
            AC_module_infor[port_idx].Curve_number[module_idx] = 1;

            /* 构建时间戳头部 (从响应帧中的时间字段读取, 位于 FrameFlag+4) */
            ft[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
            ft[2] = resp[profile->curve_flag_offset + 4U];  /* 年/月 */
            ft[3] = resp[profile->curve_flag_offset + 5U];  /* 日/时 */
            ft[4] = resp[profile->curve_flag_offset + 6U];  /* 分/秒 */
            ft[5] = resp[profile->curve_flag_offset + 7U];

            /* 为每个分支写入时间戳头部 */
            for (n = 0; n < profile->curve_branch_count; n++) {
                if ((profile->module_type == FLFXG_AC_ZZJ) ||
                    (profile->module_type == FLFXG_AC_ZZJ_HIGH) ||
                    (profile->module_type == FLFXG_S700K_C) ||
                    (profile->module_type == FLFXG_S700K_C_HIGH)) {
                    ft[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
                    ft[1] = n + 1;
                } else {
                    ft[1] = 5;
                }
                if ((profile->module_type == FLFXG_DC_ZZJ6) ||
                    (profile->module_type == FLFXG_DC_ZZJ6_HIGH)) {
                    ft[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx] + n;
                }
                memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)),
                       ft, 6);
            }
        }

    }  /* 关闭 seq_word 作用域 */

    /* ---- 缓存最后一段曲线数据 ---- */
    /* 曲线长度: 16 位大端序, 单位是点数, 每点 2 字节 */
    weibo_temp = (uint16_t)(((uint16_t)resp[profile->curve_len_offset] << 8)
                           | (uint16_t)resp[profile->curve_len_offset + 1U]) * 2U;
    for (n = 0; n < profile->curve_branch_count; n++) {
        memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)
                           + 3U + Weibo_number[port_idx][module_idx] / 2U),
               &resp[profile->curve_data_offset + n * 50U],
               weibo_temp);
    }
    Weibo_number[port_idx][module_idx] += weibo_temp;

    /*
     * 拷贝转动方向字:
     * 从响应帧的 direction_offset 处读取 2 字节,
     * 追加到最后一个分支的曲线数据末尾.
     */
    memcpy((uint8_t *)(qx + (profile->curve_branch_count - 1U) * (SDRAM_QX_BRANCH_SIZE / 2U)
                       + 3U + Weibo_number[port_idx][module_idx] / 2U),
           &resp[profile->curve_direction_offset], 2);

    AC_module_infor[port_idx].Points_number[module_idx] =
        Weibo_number[port_idx][module_idx];

    /* ---- 打包并上传各分支曲线 ---- */
    {
        /* 保存最终的微步数值 (本地变量, 避免 Weibo_number 后续被修改) */
        uint16_t final_weibo = Weibo_number[port_idx][module_idx];

        for (n = 0; n < profile->curve_branch_count; n++) {
            uint16_t frame_len;
            uint16_t *src_qx;

            /* 设置帧长度: 曲线数据(N+6) + 方向字(2) + 帧头(11) 中已有的偏移 = N+12 */
            *((uint16_t *)&ser_buf[5]) = final_weibo + 12U;
            ser_buf[3] = 0x00;  /* 帧标志: 0x00 = 曲线数据上传帧 */

            /* 从 SDRAM 拷贝该分支的完整曲线数据到帧缓冲区 */
            src_qx = (uint16_t *)((uint8_t *)qx + n * SDRAM_QX_BRANCH_SIZE);
            memcpy(&ser_buf[11], src_qx, final_weibo + 6U);

            /* 在曲线数据之后追加转动方向字 */
            memcpy(&ser_buf[17 + final_weibo],
                   (uint8_t *)(qx + (profile->curve_branch_count - 1U)
                               * (SDRAM_QX_BRANCH_SIZE / 2U) + 3U
                               + final_weibo / 2U),
                   2);

            /* 大端转小端 (曲线数据在 SDRAM 中以大端序存储) */
            uart_swap16((uint16_t *)&ser_buf[11], (final_weibo + 8U) / 2U);

            /* 计算总帧长并推入 TCP 发送队列 */
            frame_len = final_weibo + 19U;
            /* reset_flag = 0: 曲线帧是数据帧, 发送失败只该断链重连, 不该整机
             * 复位 (原来是 3 + n*2 的分支号, 一次发送失败就复位, 见 client.c
             * 里 client_add_to_tcp_send_buf() 的 reset_flag 取值约定). */
            client_add_curve_to_tcp_send_buf(&ser_buf[0], frame_len, 0U);
        }

        /* 标记曲线上传完成, 触发日志上报 (由 checkSendLog 处理) */
        portDLQX[port_idx] = 1U;
    }
}
