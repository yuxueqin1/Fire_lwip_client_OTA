#ifndef __DIP_SWITCH_H
#define __DIP_SWITCH_H

#include "main.h"
#include "stm32f4xx.h"

/* ============================================================================
 *  8-bit DIP Switch — 硬件配置拨码开关
 *  ============================================================================
 *  位分配:
 *    BA0-BA4 (PE5,PE4,PE3,PE2,PI7) → IP_ADDR3 末字节 (0-31)
 *    BA5-BA6 (PI6,PI5)             → 第7路RS485 波特率
 *    BA7     (PI4)                  → 第7路RS485 Modbus 模式 (0=从机, 1=主机)
 *  ============================================================================
 */

/* ---- 引脚定义 ---- */

/* BA0: IP_ADDR3 bit0 (LSB) */
#define DIP_BA0_PIN         GPIO_PIN_5
#define DIP_BA0_PORT        GPIOE

/* BA1: IP_ADDR3 bit1 */
#define DIP_BA1_PIN         GPIO_PIN_4
#define DIP_BA1_PORT        GPIOE

/* BA2: IP_ADDR3 bit2 */
#define DIP_BA2_PIN         GPIO_PIN_3
#define DIP_BA2_PORT        GPIOE

/* BA3: IP_ADDR3 bit3 */
#define DIP_BA3_PIN         GPIO_PIN_2
#define DIP_BA3_PORT        GPIOE

/* BA4: IP_ADDR3 bit4 (MSB) */
#define DIP_BA4_PIN         GPIO_PIN_7
#define DIP_BA4_PORT        GPIOI

/* BA5: 第7路485 波特率 bit0 */
#define DIP_BA5_PIN         GPIO_PIN_6
#define DIP_BA5_PORT        GPIOI

/* BA6: 第7路485 波特率 bit1 */
#define DIP_BA6_PIN         GPIO_PIN_5
#define DIP_BA6_PORT        GPIOI

/* BA7: 第7路485 Modbus 模式 (0=从机, 1=主机) */
#define DIP_BA7_PIN         GPIO_PIN_4
#define DIP_BA7_PORT        GPIOI

/* ---- 波特率编码 (BA5-BA6) ---- */
#define DIP_PORT7_BAUD_9600      0
#define DIP_PORT7_BAUD_19200     1
#define DIP_PORT7_BAUD_57600     2
#define DIP_PORT7_BAUD_115200    3

/* ---- Modbus 模式编码 (BA7) ---- */
#define DIP_PORT7_MODE_SLAVE     0
#define DIP_PORT7_MODE_MASTER    1

/* ---- 全局变量 (由 DIP_SWITCH_Read 填充) ---- */
extern uint8_t  g_dip_ip_last_octet;       /* IP末字节, BA0-BA4, 范围 0-31 */
extern uint32_t g_dip_rs485_7_baudrate;     /* 第7路485波特率实际值 */
extern uint8_t  g_dip_rs485_7_mode;         /* 第7路485模式: 3=Slave, 4=Master */

/* ---- 函数声明 ---- */
void DIP_SWITCH_Config(void);    /* 初始化8个GPIO为上拉输入 */
void DIP_SWITCH_Read(void);      /* 读取拨码并解析到全局变量 */

#endif /* __DIP_SWITCH_H */
