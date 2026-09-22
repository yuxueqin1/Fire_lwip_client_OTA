/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2017-xx-xx
  * @brief   GPIO���--ʹ�ù̼������LED��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32 F429 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx.h"
#include "./usart/bsp_debug_usart.h"
#include "./led/bsp_led.h"  
#include "./sdram/bsp_sdram.h"
#include "bsp_eeprom.h"
#include "./dip_switch/bsp_dip_switch.h"

#include <string.h>
#include <cm_backtrace.h>

#define HARDWARE_VERSION               "V1.0.0"
#define SOFTWARE_VERSION               "V0.1.0"

/* ---- SP706S 外部看门狗 WDI 引脚 (PF10) ---- */
#define WDI_PORT                        GPIOF
#define WDI_PIN                         GPIO_PIN_10
#define WDI_CLK_ENABLE()                __HAL_RCC_GPIOF_CLK_ENABLE()

static void SystemClock_Config(void);
static void GPIO_CLK_Init(void);
static void WDI_GPIO_Init(void);
static void Startup_LED_Running(void);

/*
 * ===========================================================================
 *  SDRAM 曲线数据区诊断测试 (临时 — 测试通过后即可删除)
 * ===========================================================================
 * 测试 SDRAM 中 QX 曲线数据区的读写完整性。
 * IS42S16160J-6TLI: 32MB (4M×16×4banks), 13 row bits, 9 col bits
 *
 * 测试 5 个不同地址区域, 每个区域用 3 种 bit 模式遍历:
 *   [1] 0xAAAA (偶数位=1, 奇数位=0) → 检测相邻 bit 短路
 *   [2] 0x5555 (偶数位=0, 奇数位=1) → 检测相邻 bit 短路 (互补)
 *   [3] 地址低 16 位作为数据           → 检测地址线粘连/断路
 *
 * 测试范围 (覆盖 32MB 地址空间的关键位):
 *   区域 A: Bank0, Row 0      (0xC0000000) — 低地址基准
 *   区域 B: Bank0, Row 20     (0xC0005000) — QX 曲线数据区基址
 *   区域 C: Bank0, Row 4096   (0xC0800000) — 验证 A12 行地址线
 *   区域 D: Bank1, Row 0      (0xC0808000) — 验证 Bank 选择 (BA0)
 *   区域 E: Bank2, Row 0      (0xC1000000) — 验证 Bank 选择 (BA1)
 *
 * 每个区域测试 256 字节 (128 个 uint16_t)。
 * 测试前先备份原始内容, 测试后恢复。
 *
 * LED 指示:
 *   LED_485_1-5 → 对应区域 A-E, 通过=灭, 失败=常亮
 *   LED_485_6   → 整体结果: 快闪 3 次=全部通过, 慢闪=有失败
 * ===========================================================================
 */
#define SDRAM_TEST_REGION_SIZE   256U
#define SDRAM_TEST_ADDR_COUNT    5U

static uint8_t SDRAM_CurveArea_Test(void)
{
    const uint32_t test_addrs[SDRAM_TEST_ADDR_COUNT] = {
        0xC0000000U,  /* A: Bank0 Row0      — 低地址基准 */
        0xC0005000U,  /* B: Bank0 Row20     — QX 基址 */
        0xC0800000U,  /* C: Bank0 Row4096   — 验证 A12 行地址线 */
        0xC0808000U,  /* D: Bank1 Row0      — 验证 BA0 bank 选择 */
        0xC1000000U,  /* E: Bank2 Row0      — 验证 BA1 bank 选择 */
    };
    const char *region_names[SDRAM_TEST_ADDR_COUNT] = {
        "Bank0 Row0",
        "Bank0 Row20 (QX)",
        "Bank0 Row4096 (A12)",
        "Bank1 Row0 (BA0)",
        "Bank2 Row0 (BA1)",
    };

    uint16_t backup[SDRAM_TEST_REGION_SIZE / 2U];
    uint16_t test_buf[SDRAM_TEST_REGION_SIZE / 2U];
    uint8_t  region;
    uint8_t  all_pass = 1U;
    uint32_t addr;
    uint32_t i;

    printf("\n");
    printf("========================================\n");
    printf("  SDRAM 32MB (IS42S16160J) Diagnostic\n");
    printf("========================================\n");

    for (region = 0U; region < SDRAM_TEST_ADDR_COUNT; region++) {
        addr = test_addrs[region];
        printf("\n[Zone %c] %s (0x%08lX)\n",
               'A' + region, region_names[region], (unsigned long)addr);

        /* 备份原数据 */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            backup[i] = *(__IO uint16_t *)(addr + i * 2U);
        }

        /* test 1: 0xAAAA */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            *(__IO uint16_t *)(addr + i * 2U) = 0xAAAAU;
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            test_buf[i] = *(__IO uint16_t *)(addr + i * 2U);
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            if (test_buf[i] != 0xAAAAU) {
                printf("  [FAIL] 0xAAAA @ +0x%04lX: exp 0xAAAA, got 0x%04X\n",
                       (unsigned long)(i * 2U), test_buf[i]);
                all_pass = 0U;
                goto restore;
            }
        }
        printf("  [PASS] pattern 0xAAAA — %u bytes OK\n",
               (unsigned int)SDRAM_TEST_REGION_SIZE);

        /* test 2: 0x5555 */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            *(__IO uint16_t *)(addr + i * 2U) = 0x5555U;
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            test_buf[i] = *(__IO uint16_t *)(addr + i * 2U);
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            if (test_buf[i] != 0x5555U) {
                printf("  [FAIL] 0x5555 @ +0x%04lX: exp 0x5555, got 0x%04X\n",
                       (unsigned long)(i * 2U), test_buf[i]);
                all_pass = 0U;
                goto restore;
            }
        }
        printf("  [PASS] pattern 0x5555 — %u bytes OK\n",
               (unsigned int)SDRAM_TEST_REGION_SIZE);

        /* test 3: address-as-data */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            *(__IO uint16_t *)(addr + i * 2U) = (uint16_t)(i & 0xFFFFU);
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            test_buf[i] = *(__IO uint16_t *)(addr + i * 2U);
        }
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            if (test_buf[i] != (uint16_t)(i & 0xFFFFU)) {
                printf("  [FAIL] addr @ +0x%04lX: exp 0x%04lX, got 0x%04X\n",
                       (unsigned long)(i * 2U), (unsigned long)(i & 0xFFFFU), test_buf[i]);
                all_pass = 0U;
                goto restore;
            }
        }
        printf("  [PASS] address pattern — %u bytes OK\n",
               (unsigned int)SDRAM_TEST_REGION_SIZE);

restore:
        /* 恢复原数据 */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            *(__IO uint16_t *)(addr + i * 2U) = backup[i];
        }
    }

    printf("\n========================================\n");
    if (all_pass) {
        printf("  SDRAM 32MB Test: ALL PASSED\n");
    } else {
        printf("  SDRAM 32MB Test: FAILURES DETECTED!\n");
    }
    printf("========================================\n\n");

    /*
     * LED 信号 (不依赖串口即可观察结果):
     *   LED_485_1-5: 区域 A-E 测试时短暂亮, 通过后熄灭
     *   通过   → LED_485_6 快闪 3 次, 然后全部熄灭继续启动
     *   失败   → LED_485_6 持续慢闪 (0.3s 周期), 板子不继续启动
     */
    if (all_pass) {
        uint8_t blink;
        for (blink = 0U; blink < 3U; blink++) {
            LED_485_6_ON;
            HAL_Delay(500U);
            LED_485_6_OFF;
            HAL_Delay(500U);
        }
    } else {
        while (1U) {
            LED_485_6_ON;
            HAL_Delay(150U);
            LED_485_6_OFF;
            HAL_Delay(150U);
        }
    }

    return all_pass;
}

/*
 * ===========================================================================
 *  EEPROM AT24C64D 诊断测试 (临时 — 测试通过后即可删除)
 * ===========================================================================
 * 测试 EEPROM (64Kbit = 8192 字节) 的读写完整性。
 * AT24C64D: 32-byte/页, 共 256 页, I2C 地址 0x50
 *
 * 测试 3 个不同地址区域, 每个区域用 3 种 bit 模式遍历:
 *   [1] 0xAA (偶数位=1, 奇数位=0) → 检测相邻 bit 短路
 *   [2] 0x55 (偶数位=0, 奇数位=1) → 检测相邻 bit 短路 (互补)
 *   [3] 地址低 8 位作为数据           → 检测地址线粘连/断路
 *
 * 测试范围:
 *   区域 A: 字节 0x0000-0x003F   (MODE_16_BIT) — 低地址, 跨越 page 0→1 边界
 *   区域 B: 字节 0x1FC0-0x1FFF   (MODE_8_BIT,  block 31) — 高地址边界
 *   区域 C: 字节 0x1000-0x103F   (MODE_16_BIT) — 中间地址, 4KB 偏移
 *
 * 每个区域测试 64 字节。
 * 测试前先备份原始内容, 测试后恢复。
 *
 * LED 指示:
 *   LED_485_1-3 → 对应区域 A-C, 通过=灭, 失败=常亮
 *   LED_485_5   → 整体结果: 快闪 3 次=全部通过, 慢闪=有失败
 * ===========================================================================
 */
#define EEPROM_TEST_REGION_SIZE   64U
#define EEPROM_TEST_ADDR_COUNT    3U

static uint8_t EEPROM_Test(void)
{
    /*
     * A: 0x0000 (MODE_16_BIT) — 低地址, 跨越 page 0→1 边界 (page_size=32)
     * B: 0x1FC0 (MODE_16_BIT) — 高地址边界, 和 MODE_8_BIT block 31 一致
     * C: 0x1000 (MODE_16_BIT) — 中间地址, 测试 A12 地址线
     */
    const uint16_t test_addrs[EEPROM_TEST_ADDR_COUNT] = {
        0x0000U,   /* A: 低地址, 跨 page 边界         */
        0x1FC0U,   /* B: 高地址 (block 31 offset 192) */
        0x1000U,   /* C: 中间地址 4KB offset          */
    };
    /*
     * 区域 B 也通过 MODE_8_BIT 测试: block=31, offset=0xC0 → addr=31, start=0xC0
     * 等效字节地址 = 31*256 + 0xC0 = 0x1FC0, 与 test_addrs[1] 一致.
     * 此处统一用 MODE_16_BIT 测试, 另用一个专门的小测试项验证 MODE_8_BIT API.
     */
    const char *region_names[EEPROM_TEST_ADDR_COUNT] = {
        "Low 0x0000 (MODE_16_BIT)",
        "High 0x1FC0 (MODE_16_BIT)",
        "Mid 0x1000 (MODE_16_BIT)",
    };

    uint8_t  backup[EEPROM_TEST_REGION_SIZE];
    uint8_t  test_buf[EEPROM_TEST_REGION_SIZE];
    uint8_t  region;
    uint8_t  all_pass = 1U;
    uint16_t addr;
    uint16_t i;

    printf("\n");
    printf("========================================\n");
    printf("  EEPROM AT24C64D 8KB Diagnostic\n");
    printf("========================================\n");

    for (region = 0U; region < EEPROM_TEST_ADDR_COUNT; region++) {
        addr = test_addrs[region];
        printf("\n[Zone %c] %s (0x%04X)\n",
               'A' + region, region_names[region], addr);

        /* 备份原数据 */
        if (EEPROM_Read(0U, (uint32_t)addr, backup, MODE_16_BIT,
                         EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] backup read failed!\n");
            // LED_485_1_ON;
            all_pass = 0U;
            continue;
        }

        /* ---- test 1: 0xAA ---- */
        memset(test_buf, 0xAA, EEPROM_TEST_REGION_SIZE);
        if (EEPROM_Write(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                          EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] write 0xAA failed!\n");
            // LED_485_2_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        memset(test_buf, 0x00, EEPROM_TEST_REGION_SIZE);
        if (EEPROM_Read(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                         EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] readback 0xAA failed!\n");
            // LED_485_3_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        for (i = 0U; i < EEPROM_TEST_REGION_SIZE; i++) {
            if (test_buf[i] != 0xAAU) {
                printf("  [FAIL] 0xAA @ +0x%04X: exp 0xAA, got 0x%02X\n",
                       (unsigned int)(addr + i), test_buf[i]);
                // LED_485_4_ON;
                all_pass = 0U;
                goto ee_restore;
            }
        }
        printf("  [PASS] pattern 0xAA — %u bytes OK\n",
               (unsigned int)EEPROM_TEST_REGION_SIZE);

        /* ---- test 2: 0x55 ---- */
        memset(test_buf, 0x55, EEPROM_TEST_REGION_SIZE);
        if (EEPROM_Write(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                          EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] write 0x55 failed!\n");
            // LED_485_1_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        memset(test_buf, 0x00, EEPROM_TEST_REGION_SIZE);
        if (EEPROM_Read(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                         EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] readback 0x55 failed!\n");
            // LED_485_2_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        for (i = 0U; i < EEPROM_TEST_REGION_SIZE; i++) {
            if (test_buf[i] != 0x55U) {
                printf("  [FAIL] 0x55 @ +0x%04X: exp 0x55, got 0x%02X\n",
                       (unsigned int)(addr + i), test_buf[i]);
                // LED_485_3_ON;
                all_pass = 0U;
                goto ee_restore;
            }
        }
        printf("  [PASS] pattern 0x55 — %u bytes OK\n",
               (unsigned int)EEPROM_TEST_REGION_SIZE);

        /* ---- test 3: address-as-data (低 8 位) ---- */
        for (i = 0U; i < EEPROM_TEST_REGION_SIZE; i++) {
            test_buf[i] = (uint8_t)(i & 0xFFU);
        }
        if (EEPROM_Write(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                          EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] write addr-pattern failed!\n");
            // LED_485_4_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        memset(test_buf, 0x00, EEPROM_TEST_REGION_SIZE);
        if (EEPROM_Read(0U, (uint32_t)addr, test_buf, MODE_16_BIT,
                         EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [FAIL] readback addr-pattern failed!\n");
            // LED_485_1_ON;
            all_pass = 0U;
            goto ee_restore;
        }
        for (i = 0U; i < EEPROM_TEST_REGION_SIZE; i++) {
            if (test_buf[i] != (uint8_t)(i & 0xFFU)) {
                printf("  [FAIL] addr @ +0x%04X: exp 0x%02X, got 0x%02X\n",
                       (unsigned int)(addr + i),
                       (unsigned int)(i & 0xFFU), test_buf[i]);
                // LED_485_2_ON;
                all_pass = 0U;
                goto ee_restore;
            }
        }
        printf("  [PASS] address pattern — %u bytes OK\n",
               (unsigned int)EEPROM_TEST_REGION_SIZE);

    ee_restore:
        /* 恢复原数据 */
        if (EEPROM_Write(0U, (uint32_t)addr, backup, MODE_16_BIT,
                          EEPROM_TEST_REGION_SIZE) != 0U) {
            printf("  [WARN] restore failed for zone %c!\n", 'A' + region);
        } else {
            printf("  [INFO] zone %c original data restored.\n", 'A' + region);
        }
    }

    /*
     * ---- 专项: MODE_8_BIT API 测试 ----
     * MODE_8_BIT 下 block 号 << 8 = 字节地址.
     * block=31 → 字节 0x1F00-0x1FFF, 测试前 32 字节.
     * MODE_16_BIT 交叉验证用同样的字节地址 0x1F00.
     */
    {
        const uint32_t mode8_block  = 31U;
        const uint16_t mode8_size   = 32U;
        const uint16_t mode8_offset = 0x1F00U;  /* = 31*256, 与 MODE_8_BIT block 31 对应的字节地址 */
        uint8_t mode8_backup[32];
        uint8_t mode8_buf[32];

        printf("\n[Zone D] MODE_8_BIT API test (block 31)\n");

        /* 备份 */
        if (EEPROM_Read(0U, mode8_block, mode8_backup, MODE_8_BIT, mode8_size) != 0U) {
            printf("  [FAIL] MODE_8_BIT backup read failed!\n");
            // LED_485_3_ON;
            all_pass = 0U;
            goto ee_skip_mode8;
        }

        /* 写入 pattern 0xA5 */
        memset(mode8_buf, 0xA5, mode8_size);
        if (EEPROM_Write(0U, mode8_block, mode8_buf, MODE_8_BIT, mode8_size) != 0U) {
            printf("  [FAIL] MODE_8_BIT write 0xA5 failed!\n");
            // LED_485_4_ON;
            all_pass = 0U;
            goto ee_restore_mode8;
        }

        /* 用 MODE_16_BIT 交叉验证 */
        memset(mode8_buf, 0x00, mode8_size);
        if (EEPROM_Read(0U, mode8_offset, mode8_buf, MODE_16_BIT, mode8_size) != 0U) {
            printf("  [FAIL] MODE_16_BIT cross-read failed!\n");
            LED_485_1_ON;
            all_pass = 0U;
            goto ee_restore_mode8;
        }
        {
            uint8_t ok = 1U;
            for (i = 0U; i < mode8_size; i++) {
                if (mode8_buf[i] != 0xA5U) {
                    printf("  [FAIL] MODE_8_BIT x-check @ +0x%04X: exp 0xA5, got 0x%02X\n",
                           (unsigned int)(mode8_offset + i), mode8_buf[i]);
                    LED_485_2_ON;
                    all_pass = 0U;
                    ok = 0U;
                    break;
                }
            }
            if (ok) {
                printf("  [PASS] MODE_8_BIT 0xA5 — cross-verified %u bytes OK\n",
                       (unsigned int)mode8_size);
            }
        }

    ee_restore_mode8:
        if (EEPROM_Write(0U, mode8_block, mode8_backup, MODE_8_BIT, mode8_size) != 0U) {
            printf("  [WARN] MODE_8_BIT restore failed!\n");
        }
    ee_skip_mode8:
        ;
    }

    printf("\n========================================\n");
    if (all_pass) {
        printf("  EEPROM AT24C64D 8KB Test: ALL PASSED\n");
    } else {
        printf("  EEPROM AT24C64D 8KB Test: FAILURES DETECTED!\n");
    }
    printf("========================================\n\n");

    /*
     * LED 信号 (不依赖串口即可观察结果):
     *   通过   → LED_485_5 快闪 3 次
     *   失败   → LED_485_5 持续慢闪 (0.3s 周期), 板子不继续启动
     */
    if (all_pass) {
        uint8_t blink;
        for (blink = 0U; blink < 3U; blink++) {
            LED_485_5_ON;
            HAL_Delay(500U);
            LED_485_5_OFF;
            HAL_Delay(500U);
        }
    } else {
        while (1U) {
            LED_485_5_ON;
            HAL_Delay(150U);
            LED_485_5_OFF;
            HAL_Delay(150U);
        }
    }

    return all_pass;
}

void BSP_Init(void)
{
  HAL_Init();    
  GPIO_CLK_Init();
  /* ����ϵͳʱ��Ϊ180 MHz */ 
  SystemClock_Config();
	/* ��ʼ��RGB�ʵ� */ 
  LED_GPIO_Config(); 
  Startup_LED_Running();
  SDRAM_Init();
  /* ---- 临时: SDRAM 曲线数据区诊断测试 (通过后可删除) ---- */
//   SDRAM_CurveArea_Test();
  EEPROM_Init();    /* AT24C64D I2C EEPROM init (PB5=WP, PB6=SCL, PB7=SDA) */
  /* ---- 临时: EEPROM AT24C64D 诊断测试 (通过后可删除) ---- */
//   EEPROM_Test();
  /* 调试串口: USART6, PC6=TX / PC7=RX / PC8=DE, 115200 8-N-1, 用于 printf 输出。
   * ⚠ 它与 RS485 第5路共用同一个 USART6, 发布版必须关闭 —— 详见
   *   bsp_debug_usart.h 里 DEBUG_UART_ENABLE 的说明。 */
#if DEBUG_UART_ENABLE
  DEBUG_USART_Config();
#endif
  DIP_SWITCH_Config();
  WDI_GPIO_Init();
  WatchDog_Clear();
  
  /* CmBacktrace initialize */
  // cm_backtrace_init("YH-F429", HARDWARE_VERSION, SOFTWARE_VERSION);
  
  // printf("----------- Lwip demo ------------\n");
	
}

/**
  * @brief  ϵͳʱ������ 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  ��
  * @retval ��
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* ʹ��HSE������HSEΪPLL��ʱ��Դ������PLL�ĸ��ַ�Ƶ����M N P Q 
	 * PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
	 */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
   while(1) {}
  }

  /* ���� OverDrive ģʽ */
  HAL_PWREx_EnableOverDrive();
 
  /* ѡ��PLLCLK��ΪSYSCLK�������� HCLK, PCLK1 and PCLK2 ��ʱ�ӷ�Ƶ���� 
	 * SYSCLK = PLLCLK     = 180M
	 * HCLK   = SYSCLK / 1 = 180M
	 * PCLK2  = SYSCLK / 2 = 90M
	 * PCLK1  = SYSCLK / 4 = 45M
	 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    while(1) {}
  }
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/configTICK_RATE_HZ);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_CLK_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
}


/**
  * @brief  启动流水灯 — 7颗LED依次点亮, 3个循环
  * @note   每次上电/复位后执行, 用于目视确认板子是否重启.
  *         流水方向: LED_485_1 → 2 → 3 → 4 → 5 → 6 → RUN, 每步80ms.
  *         3个循环约1.7秒.
  */
static void Startup_LED_Running(void)
{
    uint8_t cycle;
    for (cycle = 0; cycle < 2; cycle++) {
        LED_485_1_ON;  HAL_Delay(80);  LED_485_1_OFF;
        LED_485_2_ON;  HAL_Delay(80);  LED_485_2_OFF;
        LED_485_3_ON;  HAL_Delay(80);  LED_485_3_OFF;
        LED_485_4_ON;  HAL_Delay(80);  LED_485_4_OFF;
        LED_485_5_ON;  HAL_Delay(80);  LED_485_5_OFF;
        LED_485_6_ON;  HAL_Delay(80);  LED_485_6_OFF;
        LED_RUN_ON;    HAL_Delay(80);  LED_RUN_OFF;
    }
}


/**
  * @brief  SP706S 外部看门狗 WDI 引脚初始化 (PF10)
  * @note   推挽输出, 初始低电平.
  *         上电后首次 WatchDog_Clear() 会产生上升沿, 启动看门狗.
  */
static void WDI_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* GPIOF 时钟已在 GPIO_CLK_Init() 中使能, 此处再次使能保证安全 (幂等) */
    WDI_CLK_ENABLE();

    gpio_init.Pin   = WDI_PIN;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(WDI_PORT, &gpio_init);

    /* 初始输出低电平, 之后每次 WatchDog_Clear() 翻转一次 */
    HAL_GPIO_WritePin(WDI_PORT, WDI_PIN, GPIO_PIN_RESET);
}


void WatchDog_Clear(void)
{
  /*
   * SP706S: WDI 引脚需要一个跳变沿 (上升沿或下降沿) 来清除内部看门狗定时器.
   *         典型超时周期 1.6s, 超时后 SP706S 拉低 RST 复位 MCU.
   *         此处通过翻转 PF10 产生跳变沿来喂狗.
   *         PF10 在 WDI_GPIO_Init() 中初始化为推挽输出, 初始低电平.
   */
  HAL_GPIO_TogglePin(WDI_PORT, WDI_PIN);
}

void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* Infinite loop */
    cm_backtrace_assert(cmb_get_sp());
    printf("assert failed at %s:%d \n", file, line);
    while (1) {
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
