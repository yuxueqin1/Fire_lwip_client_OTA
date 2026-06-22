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

void BSP_Init(void)
{
  HAL_Init();    
  GPIO_CLK_Init();
  /* ����ϵͳʱ��Ϊ180 MHz */ 
  SystemClock_Config();
	/* ��ʼ��RGB�ʵ� */ 
  LED_GPIO_Config(); 
  // SDRAM_Init();
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
