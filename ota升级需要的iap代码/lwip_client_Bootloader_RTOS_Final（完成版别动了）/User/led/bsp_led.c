/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V2.0
  * @date    2026-07-10
  * @brief   7 路单色 LED 驱动 (6 路 485 通道灯 + 1 路运行灯)
  ******************************************************************************
  */

#include "./led/bsp_led.h"

/**
  * @brief  初始化全部 7 路 LED 的 GPIO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;

    /* 使能各 LED 所在 GPIO 端口的时钟 */
    LED_485_1_CLK_ENABLE();
    LED_485_2_CLK_ENABLE();
    LED_485_3_CLK_ENABLE();
    LED_485_4_CLK_ENABLE();
    LED_485_5_CLK_ENABLE();
    LED_485_6_CLK_ENABLE();
    LED_RUN_CLK_ENABLE();

    /* 通用配置: 推挽输出 + 上拉 + 高速 */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

    /* 485-1-LED : PD7 */
    GPIO_InitStruct.Pin = LED_485_1_PIN;
    HAL_GPIO_Init(LED_485_1_PORT, &GPIO_InitStruct);

    /* 485-2-LED : PG9 */
    GPIO_InitStruct.Pin = LED_485_2_PIN;
    HAL_GPIO_Init(LED_485_2_PORT, &GPIO_InitStruct);

    /* 485-3-LED : PG10 */
    GPIO_InitStruct.Pin = LED_485_3_PIN;
    HAL_GPIO_Init(LED_485_3_PORT, &GPIO_InitStruct);

    /* 485-4-LED : PG12 */
    GPIO_InitStruct.Pin = LED_485_4_PIN;
    HAL_GPIO_Init(LED_485_4_PORT, &GPIO_InitStruct);

    /* 485-5-LED : PB3 */
    GPIO_InitStruct.Pin = LED_485_5_PIN;
    HAL_GPIO_Init(LED_485_5_PORT, &GPIO_InitStruct);

    /* 485-6-LED : PB4 */
    GPIO_InitStruct.Pin = LED_485_6_PIN;
    HAL_GPIO_Init(LED_485_6_PORT, &GPIO_InitStruct);

    /* 运行灯 : PB8 */
    GPIO_InitStruct.Pin = LED_RUN_PIN;
    HAL_GPIO_Init(LED_RUN_PORT, &GPIO_InitStruct);

    /* 初始化后全部熄灭 */
    LED_485_ALL_OFF;
    LED_RUN_OFF;
}

/**
  * @brief  按通道号控制 485 LED
  * @param  ch    : 通道号 (0~5, 对应 485-1 ~ 485-6)
  * @param  state : ON (亮), OFF (灭), 或其它值 (翻转)
  * @retval 无
  */
void LED_485_Set(uint8_t ch, uint8_t state)
{
    switch (ch) {
    case 0: {
        if (state == ON)        LED_485_1_ON
        else if (state == OFF)  LED_485_1_OFF
        else                    LED_485_1_TOGGLE;
        break;
    }
    case 1: {
        if (state == ON)        LED_485_2_ON
        else if (state == OFF)  LED_485_2_OFF
        else                    LED_485_2_TOGGLE;
        break;
    }
    case 2: {
        if (state == ON)        LED_485_3_ON
        else if (state == OFF)  LED_485_3_OFF
        else                    LED_485_3_TOGGLE;
        break;
    }
    case 3: {
        if (state == ON)        LED_485_4_ON
        else if (state == OFF)  LED_485_4_OFF
        else                    LED_485_4_TOGGLE;
        break;
    }
    case 4: {
        if (state == ON)        LED_485_5_ON
        else if (state == OFF)  LED_485_5_OFF
        else                    LED_485_5_TOGGLE;
        break;
    }
    case 5: {
        if (state == ON)        LED_485_6_ON
        else if (state == OFF)  LED_485_6_OFF
        else                    LED_485_6_TOGGLE;
        break;
    }
    default:
        break;
    }
}

/*********************************************END OF FILE**********************/
