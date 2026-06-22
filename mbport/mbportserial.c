/*
 * MODBUS Library: STM32F4 porting layer for RS485 serial.
 * Target MCU: STM32F429IGT6
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "stm32f4xx_hal.h"
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbportlayer.h"
#include "mbtypes.h"
#include "mbframe.h"
#include "mbutils.h"

#define MBP_SERIAL_PORTS            ( 7 )
#define IDX_INVALID                 ( 255 )


typedef struct
{
    UCHAR                   ubIdx;
    UART_HandleTypeDef      huart;
    GPIO_TypeDef           *pxRxPort;
    uint16_t                usRxPin;
    uint8_t                 ucRxAF;
    GPIO_TypeDef           *pxTxPort;
    uint16_t                usTxPin;
    uint8_t                 ucTxAF;
    GPIO_TypeDef           *pxDePort;
    uint16_t                usDePin;
    IRQn_Type               eIRQn;
    xMBHandle               xMBHdl;
    pbMBPSerialTransmitterEmptyCB pbTXFN;
    pvMBPSerialReceiverCB   pvRXFN;
    BOOL                    bTxEnabled;
    BOOL                    bRxEnabled;
} xSerialPortInternal;

static const struct
{
    USART_TypeDef          *Instance;
    GPIO_TypeDef           *RxPort;
    uint16_t                RxPin;
    uint8_t                 RxAF;
    GPIO_TypeDef           *TxPort;
    uint16_t                TxPin;
    uint8_t                 TxAF;
    GPIO_TypeDef           *DePort;
    uint16_t                DePin;
    IRQn_Type               IRQn;
} arxSerialPortConfig[MBP_SERIAL_PORTS] =
{
    { USART1, GPIOA, GPIO_PIN_10, GPIO_AF7_USART1, GPIOA, GPIO_PIN_9, GPIO_AF7_USART1, GPIOA, GPIO_PIN_11, USART1_IRQn },
    { USART2, GPIOD, GPIO_PIN_6,  GPIO_AF7_USART2, GPIOD, GPIO_PIN_5, GPIO_AF7_USART2, GPIOD, GPIO_PIN_11,   USART2_IRQn },//现在在其他板子上测试，所以这个引脚有区别，DIR:PD4->PD11
    { USART3, GPIOB, GPIO_PIN_11, GPIO_AF7_USART3, GPIOB, GPIO_PIN_10, GPIO_AF7_USART3, GPIOH, GPIO_PIN_6,  USART3_IRQn },
    { UART4, GPIOC, GPIO_PIN_11, GPIO_AF8_UART4, GPIOC, GPIO_PIN_10, GPIO_AF8_UART4, GPIOA, GPIO_PIN_15, UART4_IRQn  },
    { UART5, GPIOD, GPIO_PIN_2,  GPIO_AF8_UART5, GPIOC, GPIO_PIN_12, GPIO_AF8_UART5, GPIOA, GPIO_PIN_15, UART5_IRQn  },
    { USART6,GPIOC, GPIO_PIN_7,  GPIO_AF8_USART6,GPIOC, GPIO_PIN_6,  GPIO_AF8_USART6,GPIOC, GPIO_PIN_8,  USART6_IRQn },
    { UART7, GPIOF, GPIO_PIN_6,  GPIO_AF8_UART7, GPIOF, GPIO_PIN_7, GPIO_AF8_UART7, GPIOF, GPIO_PIN_8,  UART7_IRQn  }
};

static xSerialPortInternal arxSerialPorts[MBP_SERIAL_PORTS];
static BOOL bIsInitialized = FALSE;


static void prvConfigureUARTPins( const xSerialPortInternal *pxSerial );
static void prvInitDirectionPin( xSerialPortInternal *pxSerial );
static void prvInitPortClocks( void );
static xSerialPortInternal *prvGetSerialPort( UCHAR ucPort );
static void prvSetDE( xSerialPortInternal *pxSerial, BOOL bEnable );
static void prvUARTTxISR( xSerialPortInternal *pxSerial );
static void prvUARTRxISR( xSerialPortInternal *pxSerial );

static void prvEnableGPIOClock( GPIO_TypeDef *port )
{
    if( port == GPIOA )
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if( port == GPIOB )
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if( port == GPIOC )
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if( port == GPIOD )
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if( port == GPIOF )
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if( port == GPIOH )
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
}

static void prvInitPortClocks( void )
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_UART7_CLK_ENABLE();
}

static void prvConfigureUARTPins( const xSerialPortInternal *pxSerial )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    prvEnableGPIOClock( pxSerial->pxTxPort );
    prvEnableGPIOClock( pxSerial->pxRxPort );

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Pin = pxSerial->usTxPin;
    GPIO_InitStruct.Alternate = pxSerial->ucTxAF;
    HAL_GPIO_Init( pxSerial->pxTxPort, &GPIO_InitStruct );

    GPIO_InitStruct.Pin = pxSerial->usRxPin;
    GPIO_InitStruct.Alternate = pxSerial->ucRxAF;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //test: 测试增加的代码
    HAL_GPIO_Init( pxSerial->pxRxPort, &GPIO_InitStruct );
}

static void prvInitDirectionPin( xSerialPortInternal *pxSerial )
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if( pxSerial == NULL || pxSerial->pxDePort == NULL )
    {
        return;
    }

    prvEnableGPIOClock( pxSerial->pxDePort );

    GPIO_InitStruct.Pin = pxSerial->usDePin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init( pxSerial->pxDePort, &GPIO_InitStruct );
    HAL_GPIO_WritePin( pxSerial->pxDePort, pxSerial->usDePin, GPIO_PIN_RESET );
}

static xSerialPortInternal *prvGetSerialPort( UCHAR ucPort )
{
    if( ucPort >= MBP_SERIAL_PORTS )
    {
        return NULL;
    }
    if( arxSerialPorts[ucPort].ubIdx != ucPort )
    {
        return NULL;
    }
    return &arxSerialPorts[ucPort];
}

static void prvSetDE( xSerialPortInternal *pxSerial, BOOL bEnable )
{
    if( pxSerial == NULL || pxSerial->pxDePort == NULL )
    {
        return;
    }
    HAL_GPIO_WritePin( pxSerial->pxDePort, pxSerial->usDePin, bEnable ? GPIO_PIN_SET : GPIO_PIN_RESET ); //Receiver Output Enable Active Low
}

static void prvUARTTxISR( xSerialPortInternal *pxSerial )
{
    BOOL bTxMore = FALSE;
    UBYTE ubValue = 0;

    if( pxSerial == NULL || pxSerial->pbTXFN == NULL )
    {
        return;
    }

#if MBS_SERIAL_API_VERSION == 2
    USHORT usBufferWritten = 0;
    bTxMore = ( ( pbMBPSerialTransmitterEmptyAPIV2CB )pxSerial->pbTXFN )( pxSerial->xMBHdl,
                                                                     &ubValue,
                                                                     1,
                                                                     &usBufferWritten );
    if( usBufferWritten > 0 )
    {
        pxSerial->huart.Instance->DR = ubValue;
    }
    if( !bTxMore || usBufferWritten == 0 )
#else
    bTxMore = ( ( pbMBPSerialTransmitterEmptyAPIV1CB )pxSerial->pbTXFN )( pxSerial->xMBHdl, &ubValue );
    if( bTxMore )
    {
        pxSerial->huart.Instance->DR = ubValue;
    }
    if( !bTxMore )
#endif
    {
        __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TXE );
        __HAL_UART_ENABLE_IT( &pxSerial->huart, UART_IT_TC );
        pxSerial->pbTXFN = NULL;
    }
}

static void prvUARTRxISR( xSerialPortInternal *pxSerial )
{
    UBYTE ubValue;

    if( pxSerial == NULL || pxSerial->pvRXFN == NULL )
    {
        return;
    }

    ubValue = ( UBYTE )( pxSerial->huart.Instance->DR & 0xFFU );

#if MBS_SERIAL_API_VERSION == 2
    ( ( pvMBPSerialReceiverAPIV2CB )pxSerial->pvRXFN )( pxSerial->xMBHdl, &ubValue, 1 );
#else
    ( ( pvMBPSerialReceiverAPIV1CB )pxSerial->pvRXFN )( pxSerial->xMBHdl, ubValue );
#endif
}

static void prvUARTTcISR( xSerialPortInternal *pxSerial )
{
    if( pxSerial == NULL )
    {
        return;
    }

    __HAL_UART_DISABLE_IT( &pxSerial->huart, UART_IT_TC );
    __HAL_UART_CLEAR_FLAG( &pxSerial->huart, UART_FLAG_TC );
    prvSetDE( pxSerial, FALSE );// RX
}

// static void prvUARTIRQHandler( xSerialPortInternal *pxSerial )
// {
//     volatile ULONG tmp;
//     // ULONG sr = pxSerial->huart.Instance->SR;

//     if( pxSerial == NULL )
//     {
//         return;
//     }

//     // printf("SR=%08lx\r\n", sr);

//     /* RXNE */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_RXNE ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_RXNE ) != RESET ) )
//     {
//         prvUARTRxISR( pxSerial );
//     }

//     /* TXE */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_TXE ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_TXE ) != RESET ) )
//     {
//         prvUARTTxISR( pxSerial );
//     }

//     /* TC */
//     if( ( __HAL_UART_GET_FLAG( &pxSerial->huart, UART_FLAG_TC ) != RESET ) &&
//         ( __HAL_UART_GET_IT_SOURCE( &pxSerial->huart, UART_IT_TC ) != RESET ) )
//     {
//         prvUARTTcISR( pxSerial );
//     }

//     /* ERROR FLAGS */
//     if( __HAL_USART_GET_FLAG( &pxSerial->huart, USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE) != RESET )
//     {
//         //  __HAL_UART_CLEAR_FLAG(&pxSerial->huart, (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE));

//         // __HAL_UART_CLEAR_OREFLAG( &pxSerial->huart );
//         // __HAL_UART_CLEAR_NEFLAG( &pxSerial->huart );
//         // __HAL_UART_CLEAR_FEFLAG( &pxSerial->huart );
//         tmp = pxSerial->huart.Instance->SR;
//         tmp = pxSerial->huart.Instance->DR;
//         (void)tmp;
//     }
// }

static void prvUARTIRQHandler( xSerialPortInternal *pxSerial )
{
    ULONG sr;
    ULONG cr1;
    volatile ULONG tmp;

    if( pxSerial == NULL )
    {
        return;
    }

    sr  = pxSerial->huart.Instance->SR;
    cr1 = pxSerial->huart.Instance->CR1;

    /* ERROR */
    if( sr & (USART_SR_ORE |
              USART_SR_NE  |
              USART_SR_FE  |
              USART_SR_PE) )
    {
        tmp = pxSerial->huart.Instance->SR;
        tmp = pxSerial->huart.Instance->DR;

        (void)tmp;

        return;
    }

    /* RXNE */
    if( (sr & USART_SR_RXNE) &&
        (cr1 & USART_CR1_RXNEIE) )
    {
        prvUARTRxISR( pxSerial );
    }

    /* TXE */
    if( (sr & USART_SR_TXE) &&
        (cr1 & USART_CR1_TXEIE) )
    {
        prvUARTTxISR( pxSerial );
    }

    /* TC */
    if( (sr & USART_SR_TC) &&
        (cr1 & USART_CR1_TCIE) )
    {
        prvUARTTcISR( pxSerial );
    }
}

void USART1_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[0] );
}

void USART2_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[1] );
}

void USART3_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[2] );
}

void UART4_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[3] );
}

void UART5_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[4] );
}

void USART6_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[5] );
}

void UART7_IRQHandler( void )
{
    prvUARTIRQHandler( &arxSerialPorts[6] );
}

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPSerialInit( xMBPSerialHandle * pxSerialHdl, UCHAR ucPort, ULONG ulBaudRate,
                   UCHAR ucDataBits, eMBSerialParity eParity, UCHAR ucStopBits, xMBHandle xMBHdl )
{
    eMBErrorCode eStatus = MB_EPORTERR;
    xSerialPortInternal *pxSerialPort;
    ULONG wordLength;
    ULONG stopBits;
    ULONG parity;

    if( ( pxSerialHdl == NULL ) || ( xMBHdl == MB_HDL_INVALID ) || ( ucPort >= MBP_SERIAL_PORTS ) ||
        ( ulBaudRate == 0 ) || ( ( ucStopBits != 1 ) && ( ucStopBits != 2 ) ) )
    {
        return MB_EINVAL;
    }

    MBP_ENTER_CRITICAL_SECTION();
    if( !bIsInitialized )
    {
        for( UCHAR ubIdx = 0; ubIdx < MBP_SERIAL_PORTS; ubIdx++ )
        {
            arxSerialPorts[ubIdx].ubIdx = IDX_INVALID;
            arxSerialPorts[ubIdx].pbTXFN = NULL;
            arxSerialPorts[ubIdx].pvRXFN = NULL;
            arxSerialPorts[ubIdx].bTxEnabled = FALSE;
            arxSerialPorts[ubIdx].bRxEnabled = FALSE;
        }

        prvInitPortClocks();
        bIsInitialized = TRUE;
    }

    pxSerialPort = prvGetSerialPort( ucPort );
    if( pxSerialPort != NULL )
    {
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EAGAIN;
    }

    pxSerialPort = &arxSerialPorts[ucPort];
    pxSerialPort->ubIdx = ucPort;
    pxSerialPort->xMBHdl = xMBHdl;
    pxSerialPort->pbTXFN = NULL;
    pxSerialPort->pvRXFN = NULL;
    pxSerialPort->bTxEnabled = FALSE;
    pxSerialPort->bRxEnabled = FALSE;

    pxSerialPort->pxRxPort = arxSerialPortConfig[ucPort].RxPort;
    pxSerialPort->usRxPin = arxSerialPortConfig[ucPort].RxPin;
    pxSerialPort->ucRxAF = arxSerialPortConfig[ucPort].RxAF;
    pxSerialPort->pxTxPort = arxSerialPortConfig[ucPort].TxPort;
    pxSerialPort->usTxPin = arxSerialPortConfig[ucPort].TxPin;
    pxSerialPort->ucTxAF = arxSerialPortConfig[ucPort].TxAF;
    pxSerialPort->pxDePort = arxSerialPortConfig[ucPort].DePort;
    pxSerialPort->usDePin = arxSerialPortConfig[ucPort].DePin;
    pxSerialPort->eIRQn = arxSerialPortConfig[ucPort].IRQn;
    pxSerialPort->huart.Instance = arxSerialPortConfig[ucPort].Instance;

    if( eParity == MB_PAR_NONE )
    {
        wordLength = UART_WORDLENGTH_8B;
        parity = UART_PARITY_NONE;
    }
    else if( eParity == MB_PAR_EVEN )
    {
        wordLength = UART_WORDLENGTH_9B;
        parity = UART_PARITY_EVEN;
    }
    else if( eParity == MB_PAR_ODD )
    {
        wordLength = UART_WORDLENGTH_9B;
        parity = UART_PARITY_ODD;
    }
    else
    {
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EINVAL;
    }

    stopBits = ( ucStopBits == 2 ) ? UART_STOPBITS_2 : UART_STOPBITS_1;

    pxSerialPort->huart.Init.BaudRate = ulBaudRate;
    pxSerialPort->huart.Init.WordLength = ( ULONG )wordLength;
    pxSerialPort->huart.Init.StopBits = stopBits;
    pxSerialPort->huart.Init.Parity = parity;
    pxSerialPort->huart.Init.Mode = UART_MODE_TX_RX;
    pxSerialPort->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    pxSerialPort->huart.Init.OverSampling = UART_OVERSAMPLING_16;
    // pxSerialPort->huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // pxSerialPort->huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    prvConfigureUARTPins( pxSerialPort );
    prvInitDirectionPin( pxSerialPort );

    if( HAL_UART_Init( &pxSerialPort->huart ) != HAL_OK )
    {
        pxSerialPort->ubIdx = IDX_INVALID;
        MBP_EXIT_CRITICAL_SECTION();
        return MB_EPORTERR;
    }

    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
    __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );

    HAL_NVIC_SetPriority( pxSerialPort->eIRQn, 2, 0 );
    HAL_NVIC_EnableIRQ( pxSerialPort->eIRQn );

    *pxSerialHdl = pxSerialPort;
    eStatus = MB_ENOERR;
    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}


eMBErrorCode
eMBPSerialClose( xMBPSerialHandle xSerialHdl )
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    if( pxSerialPort != NULL )
    {
        if( pxSerialPort->ubIdx != IDX_INVALID )
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );
            HAL_UART_DeInit( &pxSerialPort->huart );
            pxSerialPort->ubIdx = IDX_INVALID;
            pxSerialPort->xMBHdl = MB_HDL_INVALID;
            pxSerialPort->pbTXFN = NULL;
            pxSerialPort->pvRXFN = NULL;
            pxSerialPort->bTxEnabled = FALSE;
            pxSerialPort->bRxEnabled = FALSE;
            eStatus = MB_ENOERR;
        }
    }
    return eStatus;
}


eMBErrorCode
eMBPSerialTxEnable( xMBPSerialHandle xSerialHdl,
                        pbMBPSerialTransmitterEmptyCB pbMBPTransmitterEmptyFN ) //mark:eMBPSerialTxEnable
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( pxSerialPort != NULL )
    {
        pxSerialPort->pbTXFN = pbMBPTransmitterEmptyFN;
        if( pbMBPTransmitterEmptyFN != NULL )
        {
            prvSetDE( pxSerialPort, TRUE ); //TX
            __HAL_UART_ENABLE_IT( &pxSerialPort->huart, UART_IT_TXE ); //上电时 TXE 默认为 1（空）,一开 TXE 中断，CPU 立刻进入一次 prvUARTTxISR
            pxSerialPort->bTxEnabled = TRUE;
            
        }
        else
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TXE );
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_TC );
            prvSetDE( pxSerialPort, FALSE ); //RX
            pxSerialPort->bTxEnabled = FALSE;
        }
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION()
    return eStatus;
}


eMBErrorCode
eMBPSerialRxEnable( xMBPSerialHandle xSerialHdl, pvMBPSerialReceiverCB pvMBPReceiveFN ) //mark:eMBPSerialRxEnable
{
    eMBErrorCode eStatus = MB_EINVAL;
    xSerialPortInternal *pxSerialPort = xSerialHdl;

    if( pxSerialPort != NULL )
    {
        pxSerialPort->pvRXFN = ( pvMBPSerialReceiverAPIV1CB ) pvMBPReceiveFN;
        if( pvMBPReceiveFN != NULL )
        {
            // prvSetDE( pxSerialPort, FALSE ); //RX, 在TC中断里开比较好，等最后一个字节发送完成后再开RX，避免发送过程中被打断
            __HAL_UART_CLEAR_PEFLAG(&pxSerialPort->huart);
            __HAL_UART_ENABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            pxSerialPort->bRxEnabled = TRUE;
            
        }
        else
        {
            __HAL_UART_DISABLE_IT( &pxSerialPort->huart, UART_IT_RXNE );
            pxSerialPort->bRxEnabled = FALSE;
        }
        eStatus = MB_ENOERR;
    }
    return eStatus;
}
