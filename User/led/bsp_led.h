#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

/*==========================================================================
 * 7 路单色 LED 引脚定义
 * 485-1-LED : PD7    485-2-LED : PG9
 * 485-3-LED : PG10   485-4-LED : PG12
 * 485-5-LED : PB3    485-6-LED : PB4
 * 485-run-LED : PB8
 *==========================================================================*/

/* 485-1-LED : PD7 */
#define LED_485_1_PIN               GPIO_PIN_7
#define LED_485_1_PORT              GPIOD
#define LED_485_1_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

/* 485-2-LED : PG9 */
#define LED_485_2_PIN               GPIO_PIN_9
#define LED_485_2_PORT              GPIOG
#define LED_485_2_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()

/* 485-3-LED : PG10 */
#define LED_485_3_PIN               GPIO_PIN_10
#define LED_485_3_PORT              GPIOG
#define LED_485_3_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()

/* 485-4-LED : PG12 */
#define LED_485_4_PIN               GPIO_PIN_12
#define LED_485_4_PORT              GPIOG
#define LED_485_4_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE()

/* 485-5-LED : PB3 (SWD 调试下 JTAG-SWO 释放为 GPIO) */
#define LED_485_5_PIN               GPIO_PIN_3
#define LED_485_5_PORT              GPIOB
#define LED_485_5_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* 485-6-LED : PB4 (SWD 调试下 JTAG-JNTRST 释放为 GPIO) */
#define LED_485_6_PIN               GPIO_PIN_4
#define LED_485_6_PORT              GPIOB
#define LED_485_6_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()

/* 通信板运行指示灯 : PB8 */
#define LED_RUN_PIN                 GPIO_PIN_8
#define LED_RUN_PORT                GPIOB
#define LED_RUN_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE()


/*==========================================================================
 * LED 电平逻辑 (低电平点亮, 灌电流驱动)
 *   ON  = GPIO_PIN_RESET  (输出低电平 → LED 亮)
 *   OFF = GPIO_PIN_SET    (输出高电平 → LED 灭)
 *==========================================================================*/
#define ON     GPIO_PIN_RESET
#define OFF    GPIO_PIN_SET
#define TOGGLE 2U             /* LED_485_Set() 翻转 */

/*==========================================================================
 * 直接操作寄存器的 GPIO 原子操作
 *==========================================================================*/
#define	digitalHi(p,i)       {p->BSRR=i;}              /* 输出高电平 */
#define digitalLo(p,i)       {p->BSRR=(uint32_t)i << 16;} /* 输出低电平 */
#define digitalToggle(p,i)   {p->ODR ^=i;}             /* 翻转输出 */

/*==========================================================================
 * 各 LED 的控制宏
 *==========================================================================*/
/* 485-1-LED */
#define LED_485_1_TOGGLE    digitalToggle(LED_485_1_PORT, LED_485_1_PIN)
#define LED_485_1_OFF       digitalHi(LED_485_1_PORT, LED_485_1_PIN)
#define LED_485_1_ON        digitalLo(LED_485_1_PORT, LED_485_1_PIN)

/* 485-2-LED */
#define LED_485_2_TOGGLE    digitalToggle(LED_485_2_PORT, LED_485_2_PIN)
#define LED_485_2_OFF       digitalHi(LED_485_2_PORT, LED_485_2_PIN)
#define LED_485_2_ON        digitalLo(LED_485_2_PORT, LED_485_2_PIN)

/* 485-3-LED */
#define LED_485_3_TOGGLE    digitalToggle(LED_485_3_PORT, LED_485_3_PIN)
#define LED_485_3_OFF       digitalHi(LED_485_3_PORT, LED_485_3_PIN)
#define LED_485_3_ON        digitalLo(LED_485_3_PORT, LED_485_3_PIN)

/* 485-4-LED */
#define LED_485_4_TOGGLE    digitalToggle(LED_485_4_PORT, LED_485_4_PIN)
#define LED_485_4_OFF       digitalHi(LED_485_4_PORT, LED_485_4_PIN)
#define LED_485_4_ON        digitalLo(LED_485_4_PORT, LED_485_4_PIN)

/* 485-5-LED */
#define LED_485_5_TOGGLE    digitalToggle(LED_485_5_PORT, LED_485_5_PIN)
#define LED_485_5_OFF       digitalHi(LED_485_5_PORT, LED_485_5_PIN)
#define LED_485_5_ON        digitalLo(LED_485_5_PORT, LED_485_5_PIN)

/* 485-6-LED */
#define LED_485_6_TOGGLE    digitalToggle(LED_485_6_PORT, LED_485_6_PIN)
#define LED_485_6_OFF       digitalHi(LED_485_6_PORT, LED_485_6_PIN)
#define LED_485_6_ON        digitalLo(LED_485_6_PORT, LED_485_6_PIN)

/* 运行灯 */
#define LED_RUN_TOGGLE      digitalToggle(LED_RUN_PORT, LED_RUN_PIN)
#define LED_RUN_OFF         digitalHi(LED_RUN_PORT, LED_RUN_PIN)
#define LED_RUN_ON          digitalLo(LED_RUN_PORT, LED_RUN_PIN)

/*==========================================================================
 * 批量控制宏
 *==========================================================================*/
/* 全部 485 通道灯同时亮/灭 */
#define LED_485_ALL_ON      \
    do {                    \
        LED_485_1_ON;       \
        LED_485_2_ON;       \
        LED_485_3_ON;       \
        LED_485_4_ON;       \
        LED_485_5_ON;       \
        LED_485_6_ON;       \
    } while(0)

#define LED_485_ALL_OFF     \
    do {                    \
        LED_485_1_OFF;      \
        LED_485_2_OFF;      \
        LED_485_3_OFF;      \
        LED_485_4_OFF;      \
        LED_485_5_OFF;      \
        LED_485_6_OFF;      \
    } while(0)

/*==========================================================================
 * 函数声明
 *==========================================================================*/
void LED_GPIO_Config(void);
void LED_485_Set(uint8_t ch, uint8_t state);

#endif /* __LED_H */
