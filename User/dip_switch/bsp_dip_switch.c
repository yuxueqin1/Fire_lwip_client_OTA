/**
  ******************************************************************************
  * @file    bsp_dip_switch.c
  * @author  fire
  * @version V1.0
  * @date    2026-07-15
  * @brief   8位硬件拨码开关驱动 — 上电读取一次, 缓存到全局变量
  ******************************************************************************
  *  位分配:
  *     BA0-BA4 (PE5,PE4,PE3,PE2,PI7) → g_dip_ip_last_octet (0-31)
  *     BA5-BA6 (PI6,PI5)             → g_dip_rs485_7_baudrate
  *     BA7     (PI4)                  → g_dip_rs485_7_mode
  *
  *  电路: 开关闭合 → GPIO_PIN_RESET=0=ON, 开关断开 → GPIO_PIN_SET=1=OFF
  *  ON=0, OFF=1, 与原始电平一致（无需取反）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_dip_switch.h"

/* ============================================================================
 *  全局变量定义
 *  ==========================================================================*/
uint8_t  g_dip_ip_last_octet   = 0;       /* 默认 0 */
uint32_t g_dip_rs485_7_baudrate = 19200;  /* 默认 19200 */
uint8_t  g_dip_rs485_7_mode     = 3;      /* 默认从机 (UART_PORT_SLAVE) */

/* ============================================================================
 *  静态辅助: 读取单个引脚 (ON=0, OFF=1)
 *  ==========================================================================*/
static uint8_t dip_read_bit(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin);  /* GPIO_PIN_RESET(0)=ON, GPIO_PIN_SET(1)=OFF */
}

/* ============================================================================
 *  DIP_SWITCH_Config  —  初始化 GPIO (上拉输入)
 *  ==========================================================================*/
void DIP_SWITCH_Config(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /*
     * GPIOE 和 GPIOI 时钟已由 GPIO_CLK_Init() 在 BSP_Init 早期使能,
     * 此处再次使能确保时钟就绪 (幂等安全).
     */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* ---- PE2-PE5 (BA0-BA3): 上拉输入 ---- */
    gpio_init.Pin   = DIP_BA0_PIN | DIP_BA1_PIN | DIP_BA2_PIN | DIP_BA3_PIN;
    gpio_init.Mode  = GPIO_MODE_INPUT;
    gpio_init.Pull  = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIP_BA0_PORT, &gpio_init);

    /* ---- PI4-PI7 (BA4-BA7): 上拉输入 ---- */
    gpio_init.Pin   = DIP_BA4_PIN | DIP_BA5_PIN | DIP_BA6_PIN | DIP_BA7_PIN;
    gpio_init.Mode  = GPIO_MODE_INPUT;
    gpio_init.Pull  = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DIP_BA4_PORT, &gpio_init);

    /* 等待引脚电平稳定 */
    HAL_Delay(1);

    /* 上电读取一次 */
    DIP_SWITCH_Read();
}

/* ============================================================================
 *  DIP_SWITCH_Read  —  读取 8 位拨码并解析到全局变量
 *  ==========================================================================*/
void DIP_SWITCH_Read(void)
{
    uint8_t ba0, ba1, ba2, ba3, ba4, ba5, ba6, ba7;
    uint8_t baud_code;

    /* 读取 BA0-BA4 → IP_ADDR3 (ON=0, OFF=1) */
    ba0 = dip_read_bit(DIP_BA0_PORT, DIP_BA0_PIN);   /* PE5, bit0 LSB */
    ba1 = dip_read_bit(DIP_BA1_PORT, DIP_BA1_PIN);   /* PE4, bit1 */
    ba2 = dip_read_bit(DIP_BA2_PORT, DIP_BA2_PIN);   /* PE3, bit2 */
    ba3 = dip_read_bit(DIP_BA3_PORT, DIP_BA3_PIN);   /* PE2, bit3 */
    ba4 = dip_read_bit(DIP_BA4_PORT, DIP_BA4_PIN);   /* PI7, bit4 MSB */

    g_dip_ip_last_octet = (uint8_t)(ba0 | (ba1 << 1) | (ba2 << 2)
                                     | (ba3 << 3) | (ba4 << 4));

    /* 读取 BA5-BA6 → 波特率 */
    ba5 = dip_read_bit(DIP_BA5_PORT, DIP_BA5_PIN);   /* PI6, bit0 */
    ba6 = dip_read_bit(DIP_BA6_PORT, DIP_BA6_PIN);   /* PI5, bit1 */
    baud_code = (uint8_t)(ba5 | (ba6 << 1));

    switch (baud_code) {
        case DIP_PORT7_BAUD_9600:   g_dip_rs485_7_baudrate = 9600;   break;
        case DIP_PORT7_BAUD_19200:  g_dip_rs485_7_baudrate = 19200;  break;
        case DIP_PORT7_BAUD_57600:  g_dip_rs485_7_baudrate = 57600;  break;
        case DIP_PORT7_BAUD_115200: g_dip_rs485_7_baudrate = 115200; break;
        default:                    g_dip_rs485_7_baudrate = 19200;  break;
    }

    /* 读取 BA7 → Modbus 模式 */
    ba7 = dip_read_bit(DIP_BA7_PORT, DIP_BA7_PIN);   /* PI4 */
    g_dip_rs485_7_mode = ba7 ? 4U : 3U;   /* 1→UART_PORT_MODBUS_MASTER(4), 0→UART_PORT_SLAVE(3) */

    /* 打印调试信息 */
    printf("\n========================================\n");
    printf("  DIP Switch Configuration\n");
    printf("========================================\n");
    printf("  BA0-BA4 (IP):   %u%u%u%u%ub = %u\n",
           ba4, ba3, ba2, ba1, ba0, g_dip_ip_last_octet);
    printf("  BA5-BA6 (Baud): %u%ub = %lu\n",
           ba6, ba5, (unsigned long)g_dip_rs485_7_baudrate);
    printf("  BA7     (Mode): %ub = %s\n",
           ba7, (g_dip_rs485_7_mode == 4U) ? "Master" : "Slave");
    printf("========================================\n\n");
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
