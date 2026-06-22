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
    {FLFXG_XHJ_KGL,  13, 0,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_XHJ,      13, 11,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_GD_25_J,  34, 32,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_DC_AC,    10, 8,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_GD_25_F,  30, 28,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    {FLFXG_CL,       8,  6,    1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* ---- 自定义曲线模块 (uses_custom_curve=1) ---- */
    /* AC_ZZJ:  交流转辙机, 29 个寄存器 (58 字节实时值), 4 分支曲线      */
    {FLFXG_AC_ZZJ,   29, 27,   1,     1,      58,      82,      81,      73,       75,      282,     4},
    /* DC_ZZJ4: 直流四线制, 8 个寄存器 (16 字节实时值), 1 分支曲线        */
    {FLFXG_DC_ZZJ4,  8,  5,    1,     1,      16,      40,      39,      31,       33,      90,      1},
    /* DC_ZZJ6: 直流六线制, 11 个寄存器 (22 字节实时值), 2 分支曲线       */
    {FLFXG_DC_ZZJ6,  11, 5,    1,     1,      22,      46,      45,      37,       39,      146,     2},
    /* ---- 高频 Modbus 模块 (uses_custom_curve=0) ---- */
    /* NEW_XHJ:      高频信号机, 50 个输入寄存器                          */
    {FLFXG_NEW_XHJ,       50, 48,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* GD_25_J_HIGH: 高频25Hz轨道接收, 133 个输入寄存器                    */
    {FLFXG_GD_25_J_HIGH,  133,131,  1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* GD_25_F_HIGH: 高频25Hz发送, 117 个输入寄存器                       */
    {FLFXG_GD_25_F_HIGH,  117,115,  1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* CL_HIGH:      高频采集, 62 个输入寄存器                            */
    {FLFXG_CL_HIGH,       62, 60,   1,     0,      0,       0,       0,       0,        0,       0,       0},
    /* ---- 高频自定义曲线模块 (uses_custom_curve=1) ---- */
    /* AC_ZZJ_HIGH:   高频交流转辙机, 68 个寄存器 (136 字节), 4 分支      */
    {FLFXG_AC_ZZJ_HIGH,   68, 66,   1,     1,      136,     160,     159,     151,      153,      360,     4},
    /* DC_ZZJ4_HIGH:  高频直流四线制, 23 个寄存器 (46 字节), 1 分支       */
    {FLFXG_DC_ZZJ4_HIGH,  23, 21,   1,     1,      46,      70,      69,      61,       63,       120,     1},
    /* DC_ZZJ6_HIGH:  高频直流六线制, 26 个寄存器 (52 字节), 2 分支       */
    {FLFXG_DC_ZZJ6_HIGH,  26, 24,   1,     1,      52,      76,      75,      67,       69,       176,     2},
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
    } else if (BoardCfgTab[port_idx].PortType == UART_PORT_CUSTOM) {
        /* 自定义串口模式: 使用 HAL 底层直接控制 (用于曲线模块的非 Modbus 协议) */
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
    case 0:  /* USART1: TX=PA9(AF7), RX=PA10(AF7), DE=PA11 */
        uart_instance   = USART1;
        gpio_port_tx    = GPIOA;  gpio_pin_tx = GPIO_PIN_9;   af_tx = GPIO_AF7_USART1;
        gpio_port_rx    = GPIOA;  gpio_pin_rx = GPIO_PIN_10;  af_rx = GPIO_AF7_USART1;
        gpio_port_de    = GPIOA;  gpio_pin_de = GPIO_PIN_11;
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
        break;
    case 1:  /* USART2: TX=PD5(AF7), RX=PD6(AF7), DE=PD4 */
        uart_instance   = USART2;
        gpio_port_tx    = GPIOD;  gpio_pin_tx = GPIO_PIN_5;   af_tx = GPIO_AF7_USART2;
        gpio_port_rx    = GPIOD;  gpio_pin_rx = GPIO_PIN_6;   af_rx = GPIO_AF7_USART2;
        gpio_port_de    = GPIOD;  gpio_pin_de = GPIO_PIN_4;
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        break;
    case 2:  /* USART3: TX=PB10(AF7), RX=PB11(AF7), DE=PH6 */
        uart_instance   = USART3;
        gpio_port_tx    = GPIOB;  gpio_pin_tx = GPIO_PIN_10;  af_tx = GPIO_AF7_USART3;
        gpio_port_rx    = GPIOB;  gpio_pin_rx = GPIO_PIN_11;  af_rx = GPIO_AF7_USART3;
        gpio_port_de    = GPIOH;  gpio_pin_de = GPIO_PIN_6;
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
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
    case 4:  /* UART5: TX=PC12(AF8), RX=PD2(AF8), DE=PA15 */
        uart_instance   = UART5;
        gpio_port_tx    = GPIOC;  gpio_pin_tx = GPIO_PIN_12;  af_tx = GPIO_AF8_UART5;
        gpio_port_rx    = GPIOD;  gpio_pin_rx = GPIO_PIN_2;   af_rx = GPIO_AF8_UART5;
        gpio_port_de    = GPIOA;  gpio_pin_de = GPIO_PIN_15;
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_UART5_CLK_ENABLE();
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
 *         等待期间持续清零 WDT 计数器, 防止看门狗误复位.
 * @param  port_idx  端口索引
 */
void uart_wait_until_configured(uint8_t port_idx)
{
    while ((port_idx >= UART_PORT_COUNT) ||
           (BoardCfgTab[port_idx].PortType == UART_PORT_DISABLED) ||
           (BoardCfgTab[port_idx].PortType == UART_PORT_SLAVE) ||
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
    uint8_t  byte = 0;

    /* ---- 参数校验 ---- */
    if ((port_idx >= UART_PORT_COUNT) || (tx_buf == NULL) || (rx_buf == NULL) ||
        (tx_len == 0U) || (rx_max_len == 0U)) {
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

    /* ---- 清除可能残留的错误标志 (上溢/帧错/噪声) ---- */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);

    /* ---- 步骤 1-2: 拉高 DE, 切换到发送模式 ---- */
    HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_SET);

    /* ---- 步骤 3: 发送数据 ---- */
    if (HAL_UART_Transmit(huart, (uint8_t *)tx_buf, tx_len, first_byte_timeout_ms) != HAL_OK) {
        HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_RESET);
        return 0U;
    }

    /* ---- 步骤 4: 等待发送完成 (TC = Transmission Complete) ---- */
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET) { }

    /* ---- 步骤 5: 拉低 DE, 切换到接收模式 ---- */
    HAL_GPIO_WritePin(cfg->de_port, cfg->de_pin, GPIO_PIN_RESET);

    /* ---- 步骤 6: 接收首字节 (阻塞等待, 超时即失败) ---- */
    if (HAL_UART_Receive(huart, &byte, 1U, first_byte_timeout_ms) != HAL_OK) {
        return 0U;
    }
    rx_buf[rx_len++] = byte;

    /* ---- 步骤 7: 逐字节接收后续数据 (字节间隔超时 = 帧结束) ---- */
    while (rx_len < rx_max_len) {
        if (HAL_UART_Receive(huart, &byte, 1U, inter_byte_timeout_ms) != HAL_OK) {
            break;  /* 超时 → 一帧接收完成 */
        }
        rx_buf[rx_len++] = byte;
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

    /* 发送校时命令 (不需要接收应答, rx_buf=NULL) */
    (void)uart_custom_exchange(port_idx, cmd, 13, NULL, 0, 100, 0);
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

    /* 更新帧序号和帧标记 */
    AC_module_infor[port_idx].Frame_sequence[module_idx] =
        ser_buf[profile->curve_seq_offset];
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
                (profile->module_type == FLFXG_AC_ZZJ_HIGH)) {
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

    /* 计算本帧曲线数据字节数: 长度字节 * 2 (每点 2 字节) */
    weibo_temp = (uint16_t)(ser_buf[profile->curve_len_offset] * 2U);

    /* 将各分支曲线数据写入 SDRAM (追加到已有数据之后) */
    for (n = 0; n < profile->curve_branch_count; n++) {
        memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)
                           + 3U + Weibo_number[port_idx][module_idx] / 2U),
               &ser_buf[profile->curve_data_offset + n * 50U],
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

    /* 更新帧序号和帧标记 */
    AC_module_infor[port_idx].Frame_sequence[module_idx] =
        ser_buf[profile->curve_seq_offset];
    AC_module_infor[port_idx].Frame_mark[module_idx] = 0xFFU;

    /* 首帧检测: flag+1 字节为 0 且序列号为 1 */
    if ((ser_buf[profile->curve_flag_offset + 1U] == 0U) &&
        (ser_buf[profile->curve_seq_offset] == 1U)) {
        uint8_t ft[6];
        Weibo_number[port_idx][module_idx] = 0;
        AC_module_infor[port_idx].Curve_number[module_idx] = 1;

        /* 构建时间戳头部 (从响应帧中的时间字段读取) */
        ft[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
        ft[2] = ser_buf[profile->curve_flag_offset + 3U];  /* 年/月 */
        ft[3] = ser_buf[profile->curve_flag_offset + 4U];  /* 日/时 */
        ft[4] = ser_buf[profile->curve_flag_offset + 5U];  /* 分/秒 */
        ft[5] = ser_buf[profile->curve_flag_offset + 6U];

        /* 为每个分支写入时间戳头部 */
        for (n = 0; n < profile->curve_branch_count; n++) {
            if ((profile->module_type == FLFXG_AC_ZZJ) ||
                (profile->module_type == FLFXG_AC_ZZJ_HIGH)) {
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

    /* ---- 缓存最后一段曲线数据 ---- */
    weibo_temp = (uint16_t)(ser_buf[profile->curve_len_offset] * 2U);
    for (n = 0; n < profile->curve_branch_count; n++) {
        memcpy((uint8_t *)(qx + n * (SDRAM_QX_BRANCH_SIZE / 2U)
                           + 3U + Weibo_number[port_idx][module_idx] / 2U),
               &ser_buf[profile->curve_data_offset + n * 50U],
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
           &ser_buf[profile->curve_direction_offset], 2);

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
            /*
             * reset_flag = 3 + n*2:
             *   曲线上传失败时, 看门狗复位码根据分支号变化.
             *   AC_ZZJ 有 4 分支 → 复位码 3/5/7/9
             *   DC_ZZJ4 有 1 分支 → 复位码 3
             *   DC_ZZJ6 有 2 分支 → 复位码 3/5
             */
            client_add_to_tcp_send_buf(&ser_buf[0], frame_len,
                                       3U + (uint8_t)(n * 2U), port_idx);
        }

        /* 标记曲线上传完成, 触发日志上报 (由 checkSendLog 处理) */
        portDLQX[port_idx] = 1U;
    }
}
