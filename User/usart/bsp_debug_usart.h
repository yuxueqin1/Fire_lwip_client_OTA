#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>

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
