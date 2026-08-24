/**
  ******************************************************************************
  * @file    bsp_eth.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   DP83848K DP83848K Ethernet PHY driver
  ******************************************************************************
  */
#include "./eth/bsp_eth.h"
#include "main.h"


/* Global Ethernet handle */
ETH_HandleTypeDef heth;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */


void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /**DP83848K GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PG11    ------> ETH_TX_EN
    PG13    ------> ETH_TXD0
    PG14    ------> ETH_TXD1
    */
    GPIO_InitStruct.Pin = ETH_MDC_Pin|ETH_RXD0_Pin|ETH_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ETH_REF_CLK_Pin|ETH_MDIO_Pin|ETH_CRS_DV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ETH_TX_EN_Pin|ETH_TXD0_Pin|ETH_TXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(ETH_TX_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ETH_MspInit 1 */
  /* Enable the Ethernet global Interrupt */
  HAL_NVIC_SetPriority(ETH_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(ETH_IRQn);

  /* Enable ETHERNET clock  */
  __HAL_RCC_ETH_CLK_ENABLE();
  /* USER CODE END ETH_MspInit 1 */
  }
}

/**
  * @brief  DP83848K hardware reset (PH2)
  * @note   PH2 connects to DP83848K pin 23 (RESET_N), external pull-up to 3.3V for power-on self-reset
  */
static void Eth_Reset(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(10);
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ETH_CLK_DISABLE();

    /**DP83848K GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PG11    ------> ETH_TX_EN
    PG13    ------> ETH_TXD0
    PG14    ------> ETH_TXD1
    */
    HAL_GPIO_DeInit(GPIOC, ETH_MDC_Pin|ETH_RXD0_Pin|ETH_RXD1_Pin);

    HAL_GPIO_DeInit(GPIOA, ETH_REF_CLK_Pin|ETH_MDIO_Pin|ETH_CRS_DV_Pin);

    HAL_GPIO_DeInit(ETH_TX_EN_GPIO_Port, ETH_TX_EN_Pin|ETH_TXD0_Pin|ETH_TXD1_Pin);

  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }
}

/*======================================================================*/
/*  bsp_get_unique_mac()                                                */
/*  从 STM32 每芯片唯一的 96-bit UID 派生本地管理单播 MAC.               */
/*  格式: 02-00-XX-XX-XX-XX  (0x02 = 本地管理位 + 单播位)               */
/*  同一个柜子的两块板是两颗不同芯片 → UID 不同 → MAC 必然不同.          */
/*======================================================================*/
static void bsp_get_unique_mac(uint8_t mac[6])
{
    volatile uint32_t *uid = (volatile uint32_t *)UID_BASE;

    mac[0] = 0x02U;                                  /* 本地管理 + 单播  */
    mac[1] = 0x00U;                                  /* 段位, 与上位机约定 */
    mac[2] = (uint8_t)((uid[2]        ) & 0xFFU);    /* UID[95:64] 低字节 */
    mac[3] = (uint8_t)((uid[2] >> 8U  ) & 0xFFU);
    mac[4] = (uint8_t)((uid[1]        ) & 0xFFU);    /* UID[63:32] 低字节 */
    mac[5] = (uint8_t)((uid[1] >> 8U  ) & 0xFFU);
}

HAL_StatusTypeDef Bsp_Eth_Init(void)
{
  HAL_StatusTypeDef ret;

  static uint8_t MACAddr[6];

  HAL_ETH_DeInit(&heth);

  Eth_Reset();

  ETH->DMABMR |= ETH_DMABMR_SR;

  /* Init ETH: 每块板唯一的 MAC (由芯片 UID 派生) */
  bsp_get_unique_mac(MACAddr);
  PRINT_DEBUG("eth MAC = %02X:%02X:%02X:%02X:%02X:%02X\n",
              MACAddr[0], MACAddr[1], MACAddr[2],
              MACAddr[3], MACAddr[4], MACAddr[5]);
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;  /* DP83848K strap AN_EN=1, let PHY negotiate with link partner */
  heth.Init.PhyAddress = DP83848_PHY_ADDRESS;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  ret = HAL_ETH_Init(&heth);

  if (ret == HAL_OK)
    PRINT_DEBUG("eth hardware init sucess...\n");
  else
    PRINT_DEBUG("eth hardware init faild...\n");

  /* Initialize Tx Descriptors list: Chain Mode */
  HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

  /* Initialize Rx Descriptors list: Chain Mode  */
  HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);
    /* Enable MAC and DMA transmission and reception */
  return ret;
}




void ETH_IRQHandler(void)
{
  uint32_t ulReturn;
  /* Enter critical section (nestable) */
  ulReturn = taskENTER_CRITICAL_FROM_ISR();

  HAL_ETH_IRQHandler(&heth);

  /* Exit critical section */
  taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  heth: ETH handle
  * @retval None
  */
extern xSemaphoreHandle s_xSemaphore;
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth)
{
  ;
}

void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth)
{
    /* 以太网错误回调: 已移除调试打印, 暂不处理 */
}
