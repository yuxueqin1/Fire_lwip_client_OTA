/**
 *********************************************************************
 * @file    bsp_eeprom.c
 * @brief   AT24C64D EEPROM I2C 驱动 — 实现
 * @note    引脚: I2C_SCL(PB6)  I2C_SDA(PB7), WP(PB5) MCU控制拉低使能写
 *          AT24C64D: 64Kbit = 8192Byte, 32-byte/页
 *
 * 地址模式说明:
 *   MODE_8_BIT:  addr 以 256 字节为单位块, 实际字节地址 = addr << 8
 *                用于兼容原程序 API (EEPROM 地址 0/5/7/9/11/...)
 *   MODE_16_BIT: addr 即为直接 16 位字节地址
 *
 * AT24C64D 存储布局 (总容量 8192 字节):
 *   字节   0..8191: 全部可用空间
 *   MODE_8_BIT 下块 0..31 有效 (block<<8 = 0..7936)
 *   块 0   = 字节 0..449   (配置数据, 450 字节)
 *   块 5/7/9/11/13/15 = 曲线数据, 各 100 字节
 *********************************************************************
 */

#include "bsp_eeprom.h"

/* ---- 内部 I2C 句柄 ---- */
static I2C_HandleTypeDef ee_i2c_handle;

/*======================================================================*/
/*  EEPROM_Init()                                                        */
/*  初始化 I2C1 外设                                                     */
/*  - PB5 (WP):  推挽输出 LOW, 使能 EEPROM 写入                            */
/*  - PB6 (SCL): 开漏, AF4 (I2C1)                                        */
/*  - PB7 (SDA): 开漏, AF4 (I2C1)                                        */
/*  - I2C 时钟:  标准模式 100kHz                                          */
/*======================================================================*/
void EEPROM_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* ---- 使能时钟 ---- */
    EEPROM_I2C_CLK_ENABLE();
    EEPROM_WP_CLK_ENABLE();
    EEPROM_SCL_CLK_ENABLE();
    EEPROM_SDA_CLK_ENABLE();

    /* ---- WP (PB5): 推挽输出 LOW, 使能 EEPROM 写入 ---- */
    gpio_init.Pin       = EEPROM_WP_PIN;
    gpio_init.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EEPROM_WP_PORT, &gpio_init);
    HAL_GPIO_WritePin(EEPROM_WP_PORT, EEPROM_WP_PIN, GPIO_PIN_RESET);

    /* ---- SCL (PB6): 开漏, 复用 I2C1, 无内部上拉 (外部上拉电阻) ---- */
    gpio_init.Pin       = EEPROM_SCL_PIN;
    gpio_init.Mode      = GPIO_MODE_AF_OD;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FAST;
    gpio_init.Alternate = EEPROM_SCL_AF;
    HAL_GPIO_Init(EEPROM_SCL_PORT, &gpio_init);

    /* ---- SDA (PB7): 开漏, 复用 I2C1, 无内部上拉 (外部上拉电阻) ---- */
    gpio_init.Pin       = EEPROM_SDA_PIN;
    gpio_init.Mode      = GPIO_MODE_AF_OD;
    gpio_init.Pull      = GPIO_NOPULL;
    gpio_init.Speed     = GPIO_SPEED_FAST;
    gpio_init.Alternate = EEPROM_SDA_AF;
    HAL_GPIO_Init(EEPROM_SDA_PORT, &gpio_init);

    /* ---- I2C1 外设复位 ---- */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    /* ---- I2C1 外设配置 (标准模式 100kHz) ---- */
    ee_i2c_handle.Instance             = EEPROM_I2C;
    ee_i2c_handle.Init.ClockSpeed      = 100000U;
    ee_i2c_handle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    ee_i2c_handle.Init.OwnAddress1     = 0U;
    ee_i2c_handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    ee_i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    ee_i2c_handle.Init.OwnAddress2     = 0U;
    ee_i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    ee_i2c_handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&ee_i2c_handle);

    /* ---- 使能模拟噪声滤波器 ---- */
    HAL_I2CEx_AnalogFilter_Config(&ee_i2c_handle, I2C_ANALOGFILTER_ENABLE);
}

/*======================================================================*/
/*  eeprom_addr_to_byte()                                                */
/*  将逻辑地址转换为实际 EEPROM 字节地址                                   */
/*  MODE_8_BIT  → 实际地址 = addr * 256 (块号 -> 字节偏移)                */
/*  MODE_16_BIT → 实际地址 = addr (直接 16 位地址)                        */
/*======================================================================*/
static uint16_t eeprom_addr_to_byte(uint32_t addr, uint8_t mode)
{
    if (mode == MODE_8_BIT) {
        /* addr 以 256 字节块为单位: 块号 * 256 = 字节偏移 */
        return (uint16_t)(addr << 8);
    }
    /* MODE_16_BIT: 直接使用 16 位地址 */
    return (uint16_t)(addr & 0xFFFFU);
}

/*======================================================================*/
/*  eeprom_wait_ready()                                                  */
/*  等待 EEPROM 内部写入完成 (轮询 ACK)                                    */
/*  返回值: HAL_OK = 就绪, 其他 = 超时                                    */
/*======================================================================*/
static HAL_StatusTypeDef eeprom_wait_ready(void)
{
    HAL_StatusTypeDef status;
    uint32_t timeout = 10000U;  /* 最多等待 ~100ms */

    do {
        /*
         * 发送 EEPROM 地址 (不跟数据) 来检测 ACK.
         * 如果 EEPROM 正在内部写入, 它不会应答.
         */
        status = HAL_I2C_IsDeviceReady(&ee_i2c_handle,
                                        (uint16_t)(EEPROM_I2C_ADDR << 1),
                                        1, 5U);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        timeout--;
    } while (timeout > 0U);

    return HAL_TIMEOUT;
}

/*======================================================================*/
/*  EEPROM_Read()                                                        */
/*  从 EEPROM 读取数据                                                    */
/*  - 先发送 2 字节内存地址 (AT24C64D 始终使用 16 位地址)                   */
/*  - 再读取数据                                                          */
/*======================================================================*/
uint8_t EEPROM_Read(uint8_t i2c_idx, uint32_t addr, void *buf,
                    uint8_t mode, uint16_t size)
{
    uint16_t ee_addr;
    uint8_t  retry;
    HAL_StatusTypeDef hal_ret;

    (void)i2c_idx;  /* 保留参数, 始终使用 I2C1 */

    if ((buf == NULL) || (size == 0U)) {
        return 1U;
    }

    ee_addr = eeprom_addr_to_byte(addr, mode);
    if ((uint32_t)ee_addr + size > EEPROM_TOTAL_SIZE) {
        return 2U;  /* 地址越界 */
    }

    for (retry = 0U; retry < EEPROM_MAX_RETRY; retry++) {
        /* 等待 EEPROM 就绪 */
        if (eeprom_wait_ready() != HAL_OK) {
            continue;
        }

        /*
         * HAL_I2C_Mem_Read:
         * - 先发设备地址 + 内存地址
         * - 再发设备地址(读) + 接收数据
         * AT24C64D 始终使用 I2C_MEMADD_SIZE_16BIT (2 字节地址)
         */
        hal_ret = HAL_I2C_Mem_Read(&ee_i2c_handle,
                                    (uint16_t)(EEPROM_I2C_ADDR << 1),
                                    (uint16_t)ee_addr,
                                    I2C_MEMADD_SIZE_16BIT,
                                    (uint8_t *)buf,
                                    (uint16_t)size,
                                    1000U);
        if (hal_ret == HAL_OK) {
            return 0U;
        }
    }

    return 3U;  /* 读取失败 */
}

/*======================================================================*/
/*  EEPROM_Write()                                                       */
/*  向 EEPROM 写入数据 (带页边界处理)                                     */
/*  AT24C64D 每页 32 字节, 跨页写需要分多次                               */
/*======================================================================*/
uint8_t EEPROM_Write(uint8_t i2c_idx, uint32_t addr, const void *buf,
                     uint8_t mode, uint16_t size)
{
    uint16_t ee_addr;
    uint16_t remaining;
    uint16_t offset;
    uint16_t chunk;
    uint8_t  retry;
    const uint8_t *src;
    HAL_StatusTypeDef hal_ret;

    (void)i2c_idx;  /* 保留参数, 始终使用 I2C1 */

    if ((buf == NULL) || (size == 0U)) {
        return 1U;
    }

    ee_addr = eeprom_addr_to_byte(addr, mode);
    if ((uint32_t)ee_addr + size > EEPROM_TOTAL_SIZE) {
        return 2U;  /* 地址越界 */
    }

    remaining = size;
    offset    = 0U;
    src       = (const uint8_t *)buf;

    while (remaining > 0U) {
        /*
         * 计算本次可写字节数:
         * - 不能超过当前页剩余空间 (EEPROM_PAGE_SIZE - (ee_addr % PAGE_SIZE))
         * - 不能超过剩余待写字节数
         */
        uint16_t page_remain = EEPROM_PAGE_SIZE - (ee_addr % EEPROM_PAGE_SIZE);
        chunk = (remaining < page_remain) ? remaining : page_remain;

        for (retry = 0U; retry < EEPROM_MAX_RETRY; retry++) {
            /* 等待 EEPROM 就绪 (上一页写入完成) */
            if (eeprom_wait_ready() != HAL_OK) {
                continue;
            }

            hal_ret = HAL_I2C_Mem_Write(&ee_i2c_handle,
                                         (uint16_t)(EEPROM_I2C_ADDR << 1),
                                         (uint16_t)ee_addr,
                                         I2C_MEMADD_SIZE_16BIT,
                                         (uint8_t *)(src + offset),
                                         (uint16_t)chunk,
                                         1000U);
            if (hal_ret == HAL_OK) {
                break;  /* 本块写入成功, 跳出重试循环 */
            }
        }

        if (hal_ret != HAL_OK) {
            return 3U;  /* 写入失败 */
        }

        ee_addr   += chunk;
        offset    += chunk;
        remaining -= chunk;
    }

    return 0U;
}
