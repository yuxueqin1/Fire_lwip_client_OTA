/**
 *********************************************************************
 * @file    bsp_eeprom.h
 * @brief   AT24C64D EEPROM I2C 驱动 — 头文件
 * @note    引脚: I2C_SCL(PB6)  I2C_SDA(PB7), WP(PB5) MCU控制拉低使能写
 *          AT24C64D: 64Kbit = 8192Byte, 32-byte/页
 *          MODE_8_BIT:  地址以 256 字节为单位 (addr << 8 = 实际字节地址)
 *          MODE_16_BIT: 地址直接为 16 位字节地址
 *********************************************************************
 */

#ifndef __BSP_EEPROM_H
#define __BSP_EEPROM_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*======================================================================*/
/*  AT24C64D 参数                                                        */
/*======================================================================*/
#define EEPROM_I2C_ADDR         0x50U   /* 7-bit 地址 (A2=A1=A0=0)       */
#define EEPROM_PAGE_SIZE        32U     /* 32-byte/页                     */
#define EEPROM_TOTAL_SIZE       8192U   /* 64Kbit = 8KB                   */
#define EEPROM_MAX_RETRY        3U      /* 失败重试次数                   */

/* ---- 地址模式 (兼容原程序 API) ---- */
#define MODE_8_BIT              1U      /* addr 以 256 字节块为单位       */
#define MODE_16_BIT             2U      /* addr 为直接字节地址             */

/*======================================================================*/
/*  I2C 引脚定义 (I2C1)                                                  */
/*  SCL — PB6  (I2C1 复用)                                               */
/*  SDA — PB7  (I2C1 复用)                                               */
/*  WP  — PB5  (推挽输出, 由 MCU 控制拉低使能写)                           */
/*======================================================================*/
#define EEPROM_I2C              I2C1
#define EEPROM_I2C_CLK_ENABLE() __HAL_RCC_I2C1_CLK_ENABLE()

#define EEPROM_WP_PORT          GPIOB
#define EEPROM_WP_PIN           GPIO_PIN_5
#define EEPROM_WP_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define EEPROM_SCL_PORT         GPIOB
#define EEPROM_SCL_PIN          GPIO_PIN_6
#define EEPROM_SCL_AF           GPIO_AF4_I2C1
#define EEPROM_SCL_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

#define EEPROM_SDA_PORT         GPIOB
#define EEPROM_SDA_PIN          GPIO_PIN_7
#define EEPROM_SDA_AF           GPIO_AF4_I2C1
#define EEPROM_SDA_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

/*======================================================================*/
/*  API 函数声明                                                         */
/*======================================================================*/

/**
 * @brief  初始化 EEPROM 的 I2C 接口
 * @note   WP(PB5) 由 MCU 推挽输出 LOW 使能写入; 必须在 BSP_Init() 阶段调用
 */
void EEPROM_Init(void);

/**
 * @brief  从 EEPROM 读取数据
 * @param  i2c_idx   I2C 总线索引 (保留, 兼容原 API, 固定传 0)
 * @param  addr      逻辑地址 (MODE_8_BIT 时以 256 字节为单位)
 * @param  buf       目标缓冲区指针
 * @param  mode      地址模式: MODE_8_BIT 或 MODE_16_BIT
 * @param  size      读取字节数
 * @retval 0=成功, 非0=失败
 */
uint8_t EEPROM_Read(uint8_t i2c_idx, uint32_t addr, void *buf,
                    uint8_t mode, uint16_t size);

/**
 * @brief  向 EEPROM 写入数据
 * @param  i2c_idx   I2C 总线索引 (保留, 兼容原 API, 固定传 0)
 * @param  addr      逻辑地址 (MODE_8_BIT 时以 256 字节为单位)
 * @param  buf       源数据缓冲区指针
 * @param  mode      地址模式: MODE_8_BIT 或 MODE_16_BIT
 * @param  size      写入字节数
 * @note   自动处理页边界 (AT24C64D 每页 32 字节)
 * @retval 0=成功, 非0=失败
 */
uint8_t EEPROM_Write(uint8_t i2c_idx, uint32_t addr, const void *buf,
                     uint8_t mode, uint16_t size);

#endif /* __BSP_EEPROM_H */
