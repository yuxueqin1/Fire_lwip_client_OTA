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

#include <cm_backtrace.h>

#define HARDWARE_VERSION               "V1.0.0"
#define SOFTWARE_VERSION               "V0.1.0"

static void SystemClock_Config(void);
static void GPIO_CLK_Init(void);

/*
 * ===========================================================================
 *  SDRAM 曲线数据区诊断测试 (临时 — 测试通过后即可删除)
 * ===========================================================================
 * 测试 SDRAM 中 QX 曲线数据区的读写完整性。
 * 测试 3 个不同地址区域, 每个区域用 3 种 bit 模式遍历:
 *   [1] 0xAAAA (偶数位=1, 奇数位=0) → 检测相邻 bit 短路
 *   [2] 0x5555 (偶数位=0, 奇数位=1) → 检测相邻 bit 短路 (互补)
 *   [3] 地址低 16 位作为数据           → 检测地址线粘连/断路
 *
 * 测试范围:
 *   区域 A: SSZ 实时数据区 (0xD0000000)
 *   区域 B: QX  曲线数据区基址 (0xD0005000)
 *   区域 C: Port2 曲线数据区 (0xD0023000) ← 正在测试的端口
 *
 * 每个区域测试 256 字节 (128 个 uint16_t)。
 * 测试前先备份原始内容, 测试后恢复。
 * ===========================================================================
 */
#define SDRAM_TEST_REGION_SIZE   256U
#define SDRAM_TEST_ADDR_COUNT    3U

static uint8_t SDRAM_CurveArea_Test(void)
{
    const uint32_t test_addrs[SDRAM_TEST_ADDR_COUNT] = {
        0xD0000000U,
        0xD0005000U,
        0xD0023000U,
    };
    const char *region_names[SDRAM_TEST_ADDR_COUNT] = {
        "SSZ",
        "QX base",
        "Port2 QX",
    };

    uint16_t backup[SDRAM_TEST_REGION_SIZE / 2U];
    uint16_t test_buf[SDRAM_TEST_REGION_SIZE / 2U];
    uint8_t  region;
    uint8_t  all_pass = 1U;
    uint32_t addr;
    uint32_t i;

    printf("\n");
    printf("========================================\n");
    printf("  SDRAM Curve Area Diagnostic Test\n");
    printf("========================================\n");

    for (region = 0U; region < SDRAM_TEST_ADDR_COUNT; region++) {
        addr = test_addrs[region];
        printf("\n[Zone %c] %s (0x%08lX)\n",
               'A' + region, region_names[region], (unsigned long)addr);

        /* backup */
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
                break;
            }
        }
        if (i == (SDRAM_TEST_REGION_SIZE / 2U)) {
            printf("  [PASS] pattern 0xAAAA — %u bytes OK\n",
                   (unsigned int)SDRAM_TEST_REGION_SIZE);
        }

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
                break;
            }
        }
        if (i == (SDRAM_TEST_REGION_SIZE / 2U)) {
            printf("  [PASS] pattern 0x5555 — %u bytes OK\n",
                   (unsigned int)SDRAM_TEST_REGION_SIZE);
        }

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
                break;
            }
        }
        if (i == (SDRAM_TEST_REGION_SIZE / 2U)) {
            printf("  [PASS] address pattern — %u bytes OK\n",
                   (unsigned int)SDRAM_TEST_REGION_SIZE);
        }

        /* restore */
        for (i = 0U; i < (SDRAM_TEST_REGION_SIZE / 2U); i++) {
            *(__IO uint16_t *)(addr + i * 2U) = backup[i];
        }
    }

    printf("\n========================================\n");
    if (all_pass) {
        printf("  SDRAM Curve Area Test: ALL PASSED\n");
    } else {
        printf("  SDRAM Curve Area Test: FAILURES DETECTED!\n");
    }
    printf("========================================\n\n");

    /*
     * LED 信号 (不依赖串口即可观察结果):
     *   通过 → LED 绿灯快闪 3 次, 然后全部熄灭继续启动
     *   失败 → LED 红灯持续慢闪 (0.3s 周期), 板子不继续启动
     */
    if (all_pass) {
        uint8_t blink;
        for (blink = 0U; blink < 3U; blink++) {
            LED_GREEN;
            HAL_Delay(200U);
            LED_RGBOFF;
            HAL_Delay(200U);
        }
    } else {
        while (1U) {
            LED_RED;
            HAL_Delay(150U);
            LED_RGBOFF;
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
  SDRAM_Init();
  /* ---- 临时: SDRAM 曲线数据区诊断测试 (通过后可删除) ---- */
  SDRAM_CurveArea_Test();
  EEPROM_Init();    /* AT24C64D I2C EEPROM init (PB5=WP, PB6=SCL, PB7=SDA) */
  /*��ʼ��USART ����ģʽΪ 115200 8-N-1���жϽ���*/
  // DEBUG_USART_Config();
  
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
}


void WatchDog_Clear(void)
{
  // todo:待写
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
