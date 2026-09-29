#ifndef __DEBUG_USART_H

#define	__DEBUG_USART_H



#include "stm32f4xx.h"

#include <stdio.h>



/* ==================================================================
 *  调试串口总开关 —— 发布版必须为 0
 *
 *  ⚠ 本板 DEBUG_USART 是 USART6, 而 USART6 同时被 RS485 第5路占用
 *    (bsp_rs485.c case 5: PC6=TX / PC7=RX / PC8=DE, AF8)。两者是同一个
 *    外设, 而且 USART6_IRQHandler 已经归第5路
 *    (mbport/mbportserial.c:593 -> prvUARTIRQHandler(&arxSerialPorts[5]))。
 *
 *  所以调试串口一旦初始化, 就会和第5路抢同一个 USART:
 *    - printf 的内容直接打到 RS485 总线上, 破坏第5路的报文
 *    - 本文件的 HAL_UART_MspInit 会覆盖第5路刚配好的 PC8(DE) 与 NVIC 优先级
 *
 *  置 0 (发布版): 不初始化外设、不产生中断; fputc/fgetc 是空实现 ——
 *                printf 全工程静默, 且不会死等 (HAL_UART_Transmit 见到
 *                gState != READY 会立刻返回 HAL_BUSY)。
 *                USART6 由 RS485 第5路独占。
 *  置 1 (调试版): 恢复原有调试输出。此时不要同时启用 RS485 第5路。
 * ================================================================== */
#define DEBUG_UART_ENABLE                       1


//���ڲ�����

#define DEBUG_USART_BAUDRATE                    115200



//���Ŷ���

/*******************************************************/

#define DEBUG_USART                             USART6

#define DEBUG_USART_CLK_ENABLE()                __HAL_RCC_USART6_CLK_ENABLE();



// USART6 on APB2, PC6=TX, PC7=RX



#define DEBUG_USART_RX_GPIO_PORT                GPIOC

#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

#define DEBUG_USART_RX_PIN                      GPIO_PIN_7

#define DEBUG_USART_RX_AF                       GPIO_AF8_USART6



#define DEBUG_USART_TX_GPIO_PORT                GPIOC

#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

#define DEBUG_USART_TX_PIN                      GPIO_PIN_6

#define DEBUG_USART_TX_AF                       GPIO_AF8_USART6



#define DEBUG_USART_DE_GPIO_PORT                GPIOC

#define DEBUG_USART_DE_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOC_CLK_ENABLE()

#define DEBUG_USART_DE_PIN                      GPIO_PIN_8



#define DEBUG_USART_IRQHandler                  USART6_IRQHandler

#define DEBUG_USART_IRQ                 		USART6_IRQn

/************************************************************/



void Usart_SendString(uint8_t *str);

void DEBUG_USART_Config(void);

//int fputc(int ch, FILE *f);

extern UART_HandleTypeDef UartHandle;

#endif /* __USART1_H */

