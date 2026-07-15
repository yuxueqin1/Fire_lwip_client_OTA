#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

//���ڲ�����
#define DEBUG_USART_BAUDRATE                    115200

//���Ŷ���
/*******************************************************/
#define DEBUG_USART                             UART7
#define DEBUG_USART_CLK_ENABLE()                __HAL_RCC_UART7_CLK_ENABLE();

//注意: STM32F429 上 UART7 时钟固定为 PCLK1, 不支持时钟源选择 (无 RCC_PERIPHCLK_UART7 宏)

#define DEBUG_USART_RX_GPIO_PORT                GPIOF
#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define DEBUG_USART_RX_PIN                      GPIO_PIN_6
#define DEBUG_USART_RX_AF                       GPIO_AF8_UART7

#define DEBUG_USART_TX_GPIO_PORT                GPIOF
#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define DEBUG_USART_TX_PIN                      GPIO_PIN_7
#define DEBUG_USART_TX_AF                       GPIO_AF8_UART7

#define DEBUG_USART_DE_GPIO_PORT                GPIOF
#define DEBUG_USART_DE_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()
#define DEBUG_USART_DE_PIN                      GPIO_PIN_8

#define DEBUG_USART_IRQHandler                  UART7_IRQHandler
#define DEBUG_USART_IRQ                 		UART7_IRQn
/************************************************************/

void Usart_SendString(uint8_t *str);
void DEBUG_USART_Config(void);
//int fputc(int ch, FILE *f);
extern UART_HandleTypeDef UartHandle;
#endif /* __USART1_H */
