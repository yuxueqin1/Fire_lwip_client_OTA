/**
  ******************************************************************************
  * @file    bsp_eth.c
  * @author  fire
  * @version V1.0
  * @date    2019-xx-xx
  * @brief   DP83848K 以太网 PHY 驱动
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
  * @brief  DP83848K 硬件复位 (PH2)
  * @note   PH2 连接 DP83848K pin 23 (RESET), 同时该引脚外部上拉到 3.3V 自复位
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
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(5);
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

HAL_StatusTypeDef Bsp_Eth_Init(void)
{
  HAL_StatusTypeDef ret;

  uint8_t MACAddr[6] ;

  HAL_ETH_DeInit(&heth);

  Eth_Reset();

  ETH->DMABMR |= ETH_DMABMR_SR;

  /* Init ETH */
  MACAddr[0] = 0x02;
  MACAddr[1] = 0x00;
  MACAddr[2] = 0x00;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE; /* PHY hw auto-negotiates; don't read PHY via MDIO */
  heth.Init.PhyAddress = DP83848_PHY_ADDRESS;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;

  /* configure ethernet peripheral (GPIOs, clocks, MAC, DMA) */
  ret = HAL_ETH_Init(&heth);

  /*
   * Force MAC to 100M full-duplex regardless of what HAL_ETH_Init may
   * have derived from (possibly incorrect) PHY MDIO reads.  DP83848K
   * auto-negotiates in hardware independently — the MAC just needs to
   * match the expected result.
   */
  heth.Init.Speed      = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;

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
  /* 进入临界区, 临界区可嵌套 */
  ulReturn = taskENTER_CRITICAL_FROM_ISR();

  HAL_ETH_IRQHandler(&heth);

  /* 退出临界区 */
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
    PRINT_ERR("eth err\n");
}
