#ifndef __RS485_H
#define	__RS485_H

#include "main.h"
#include "stm32f4xx.h"
#include <stdio.h>


/* Board configuration structure */
typedef struct {
    uint8_t PortNum;
    uint8_t PortType;
    uint8_t BautRate;
    uint8_t DataBit;
    uint8_t StopBit;
    uint8_t Parity;
    uint8_t MainProType;
    uint8_t SubProType;
    uint8_t FenjiNum;
    uint8_t QueryPeriod;
    uint8_t SlaveModuelNum;
    uint8_t SlaveModuelAddress[32];
    uint8_t SlaveModuelType[32];
} BoardConfig_t;

#define UART_PORT_COUNT             6
#define UART_SERIAL_BUF_SIZE        (2 * 1024)
#define UART_PORT_DISABLED          0
#define UART_PORT_CUSTOM            1
#define UART_PORT_SLAVE             3
#define UART_PORT_MODBUS_MASTER     4
#define UART_DEFAULT_DELAY_MS       440
#define UART_MODULE_MAX_COUNT       32
#define UART_MODULE_DEFAULT_REGS    13
#define UART_HOLDING_REG_STRIDE     500
#define BOARD_CONFIG_BYTES          (sizeof(BoardCfgTab))

#define CONFIG_PORT                 0x00
#define FLFXG_XHJ_KGL               0x31
#define FLFXG_XHJ                   0x32
#define FLFXG_GD_25_J               0x33
#define FLFXG_DC_AC                 0x34
#define FLFXG_GD_25_F               0x35
#define FLFXG_CL                    0x36
#define FLFXG_AC_ZZJ                0x37
#define FLFXG_DC_ZZJ4               0x38
#define FLFXG_DC_ZZJ6               0x39

/* 高频模块类型 (新增) */
#define FLFXG_NEW_XHJ               0x47    /* 高频信号机           */
#define FLFXG_GD_25_J_HIGH          0x48    /* 高频25Hz轨道接收      */
#define FLFXG_GD_25_F_HIGH          0x49    /* 高频25Hz发送          */
#define FLFXG_CL_HIGH               0x4A    /* 高频采集              */
#define FLFXG_AC_ZZJ_HIGH           0x4B    /* 高频交流转辙机         */
#define FLFXG_DC_ZZJ4_HIGH          0x4C    /* 高频直流转辙机4路      */
#define FLFXG_DC_ZZJ6_HIGH          0x4D    /* 高频直流转辙机6路      */

/* SDRAM curve buffer layout */
/* Realtime data (SSZ): port_idx * 0x1000 bytes from SDRAM_BASE_ADDR */
/* Curve data (QX):    per port, per module 0x1000 bytes blocks */

#define SDRAM_BASE_ADDR             SDRAM_BANK_ADDR
#define SDRAM_SSZ_BASE              SDRAM_BASE_ADDR
#define SDRAM_SSZ_SIZE_PER_PORT     0x1000U
#define SDRAM_QX_BASE               (SDRAM_BASE_ADDR + 0x5000U)
#define SDRAM_QX_SIZE_PER_MODULE    0x1000U
#define SDRAM_QX_BRANCH_SIZE        0x400U

#define CURVE_MAX_POINTS            2001
#define CURVE_EEPROM_SIZE           100

/* ---- Curve module info (matches original AC_module_mrammanage) ---- */
typedef struct {
    uint8_t  Curve_number[32];
    uint8_t  Frame_mark[32];
    uint8_t  Frame_sequence[32];
    uint16_t Points_number[32];
} CurveModuleInfo_t;

/* ---- UART 硬件配置 ---- */
typedef struct {
    const char *name;          /* 任务名称, 如 "USART1"               */
    uint8_t     uart_num;      /* UART 外设编号 1-6                   */
} UART_HW_Config_t;

/* ---- 自定义串口的 DE 引脚配置 (RS485 方向控制) ---- */
typedef struct {
    GPIO_TypeDef *de_port;     /* DE 引脚所在的 GPIO 端口, 如 GPIOA   */
    uint16_t      de_pin;      /* DE 引脚的 Pin 号, 如 GPIO_PIN_11    */
} UART_RawConfig_t;

/* ---- Module profile (drive different module types with one table) ---- */
typedef struct {
    uint8_t  module_type;
    uint8_t  input_regs;          /* Modbus input registers to read          */
    uint8_t  holding_regs;        /* Holding registers to copy (data-only)   */
    uint8_t  has_alarm;           /* 1 = check alarm bit in status register  */
    uint8_t  uses_custom_curve;   /* 1 = custom serial + curve handling      */
    uint16_t realtime_bytes;      /* Bytes of real-time data per module      */
    uint16_t curve_data_offset;   /* Offset in response where curve data starts */
    uint16_t curve_len_offset;    /* Offset of per-branch curve length byte  */
    uint16_t curve_flag_offset;   /* Offset of frame flag (0/0x55/0xFF)      */
    uint16_t curve_seq_offset;    /* Offset of frame sequence byte           */
    uint16_t curve_direction_offset; /* Offset of rotation-direction word    */
    uint8_t  curve_branch_count;  /* Number of curve branches (1/2/4)        */
} UART_ModuleProfile_t;

/* ---- Per-port runtime state (one per UART task) ---- */
typedef struct {
    uint8_t        port_idx;
    xMBMHandle     xMBMaster;
    uint8_t        module_error[UART_MODULE_MAX_COUNT];
    uint8_t        port_error;
    uint16_t       holding_base;
    uint32_t       link_mask;
    uint16_t       data_len;
    uint16_t      *ssz_buf;       /* SDRAM realtime data pointer for this port */
    uint16_t      *qx_buf;        /* SDRAM curve data base pointer             */
} UART_PortRuntime_t;

/* ---- Global variables for 6 UART ports ---- */
extern BoardConfig_t    BoardCfgTab[UART_PORT_COUNT];
extern uint8_t          SerialDataBuf[UART_PORT_COUNT][UART_SERIAL_BUF_SIZE];
extern uint8_t          MramCfgDataBuf[BOARD_CONFIG_BYTES];
extern uint8_t          UART_Custom[UART_PORT_COUNT];
extern uint32_t         UART_delay[UART_PORT_COUNT];
extern uint32_t         ALARM_TEMP[UART_PORT_COUNT];
extern uint8_t          download_flag;
extern uint16_t         WDTUARTFLAG[UART_PORT_COUNT];
extern TaskHandle_t     Serial_Task_Handles[UART_PORT_COUNT];
extern const UART_HW_Config_t uart_hw_config[UART_PORT_COUNT];

/* ---- 自定义串口底层句柄 (6路 UART 的 HAL 句柄和 DE 引脚配置) ---- */
extern UART_HandleTypeDef  uart_raw_handles[UART_PORT_COUNT];
extern UART_RawConfig_t    uart_raw_config[UART_PORT_COUNT];

/* ---- 曲线模块信息 (每个端口一份) ---- */
extern CurveModuleInfo_t     AC_module_infor[UART_PORT_COUNT];
extern uint16_t              Weibo_number[UART_PORT_COUNT][32];
extern uint8_t               changetime[UART_PORT_COUNT];
extern uint8_t               TaskAdjustNum[UART_PORT_COUNT];
extern uint32_t              Systemtime;
extern uint8_t               ALARM;

/* ---- TCP / Watchdog / Log globals ---- */
extern uint16_t              WDTTCPSENDFLAG;                      /* TCP send task watchdog counter    */
extern uint16_t              heartbeat;                           /* Heartbeat timeout counter         */
extern uint16_t              connect_time;                        /* Connection uptime counter         */
extern uint8_t               WTD_RESET;                           /* Watchdog reset reason code        */
extern volatile uint8_t      pStart;                              /* Start-up log flag                 */
extern volatile uint8_t      netReset;                            /* Network reconnect log flag        */
extern uint8_t               logWTD_RESET;                        /* WDT reset log flag                */
extern uint8_t               jiaoshi;                             /* Time-calibration log flag         */
extern uint8_t               portError[UART_PORT_COUNT];          /* Per-port data error log flags     */
extern uint8_t               portDLQX[UART_PORT_COUNT];           /* Per-port curve upload log flags   */

/* ---- 函数声明 ---- */
uint32_t uart_get_baud_rate(uint8_t baud_code);
eMBErrorCode uart_init_port(uint8_t port_idx, uint32_t baud_rate, xMBMHandle *pxMBMaster);
/* 自定义串口底层初始化 (非 Modbus 模式) */
eMBErrorCode uart_init_custom_raw(uint8_t port_idx, uint32_t baud_rate);
void     uart_setup_header(int port_idx);
void     uart_wait_until_configured(uint8_t port_idx);
uint16_t uart_holding_base(uint8_t port_idx);
const UART_ModuleProfile_t *uart_get_module_profile(uint8_t module_type);
uint8_t  uart_profile_is_custom_curve(uint8_t module_type);
uint16_t uart_custom_exchange(uint8_t port_idx, const uint8_t *tx_buf, uint16_t tx_len,
                              uint8_t *rx_buf, uint16_t rx_max_len,
                              uint32_t first_byte_timeout_ms, uint32_t inter_byte_timeout_ms);

/* SDRAM curve data helpers */
void    *uart_ssz_ptr(uint8_t port_idx);
void    *uart_qx_ptr(uint8_t port_idx, uint8_t module_idx);

/* Utility: byte-swap 16-bit array in place (Modbus big-endian -> LE) */
void     uart_swap16(uint16_t *buf, uint16_t count);
/* Utility: compute CRC16 over arbitrary buffer (uses Modbus library) */
uint16_t uart_crc16(const uint8_t *buf, uint16_t len);

/* Curve frame handling */
uint8_t  uart_custom_send_adjust_time(uint8_t port_idx, uint8_t com_port,
                                      uint8_t slave_addr, uint32_t sys_time);
void     uart_handle_curve_end_frame(uint8_t port_idx, uint8_t module_idx,
                                     const UART_ModuleProfile_t *profile,
                                     uint8_t *ser_buf, uint16_t weibo_len);
void     uart_handle_curve_continue_frame(uint8_t port_idx, uint8_t module_idx,
                                          const UART_ModuleProfile_t *profile,
                                          uint8_t *ser_buf, uint8_t is_first,
                                          uint8_t flash_time[6]);
void     uart_handle_curve_normal_frame(uint8_t port_idx, uint8_t module_idx);

/* ---- Log system ---- */
void     SendLogToServer(uint8_t port, uint8_t log_id);
void     checkSendLog(void);

#endif /* __RS485_H */
