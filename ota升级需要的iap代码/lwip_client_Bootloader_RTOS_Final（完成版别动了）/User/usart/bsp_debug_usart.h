#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

//串口波特率
#define DEBUG_USART_BAUDRATE                    115200

/* 引导程序专用的短发送超时(ms)。
 * fputc 是逐字节阻塞发送, 超时就是"串口坏了要卡多久"。
 * 引导程序宁可不打印也不能卡住启动, 所以取 10ms。 */
#define DEBUG_USART_TX_TIMEOUT_MS               10

//引脚定义
/*******************************************************/
#define DEBUG_USART                             USART6
#define DEBUG_USART_CLK_ENABLE()                __USART6_CLK_ENABLE();

#define RCC_PERIPHCLK_UARTx                     RCC_PERIPHCLK_USART6
#define RCC_UARTxCLKSOURCE_SYSCLK               RCC_USART6CLKSOURCE_SYSCLK

#define DEBUG_USART_RX_GPIO_PORT                GPIOC
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_7
#define DEBUG_USART_RX_AF                       GPIO_AF8_USART6

#define DEBUG_USART_TX_GPIO_PORT                GPIOC
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_6
#define DEBUG_USART_TX_AF                       GPIO_AF8_USART6

/* RS485 方向控制脚 (与 APP 侧 bsp_debug_usart.h 一致) */
#define DEBUG_USART_DE_GPIO_PORT                GPIOC
#define DEBUG_USART_DE_GPIO_CLK_ENABLE()        __GPIOC_CLK_ENABLE()
#define DEBUG_USART_DE_PIN                      GPIO_PIN_8

#define DEBUG_USART_IRQHandler                  USART6_IRQHandler
#define DEBUG_USART_IRQ                 		    USART6_IRQn
/************************************************************/

void Usart_SendString(uint8_t *str);
void DEBUG_USART_Config(void);
//int fputc(int ch, FILE *f);
extern UART_HandleTypeDef UartHandle;
#endif /* __USART6_H */
