/*
 * MODBUS Library: STM32F4 timer port.
 * Target MCU: STM32F429IGT6
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "stm32f4xx_hal.h"
#include "mbport.h"
#include "./led/bsp_led.h" 


/* ----------------------- Modbus includes ----------------------------------*/
#include "mbtypes.h"
#include "mbportlayer.h"
#include "mbframe.h"
#include "mbutils.h"

#define MAX_TIMER_HDLS                  ( 13 )
#define IDX_INVALID                     ( 255 )
#define TIMER_TIMEOUT_INVALID           ( 65535U )

#define RESET_HDL( x ) do { \
    ( x )->ubIdx = IDX_INVALID; \
    ( x )->usNTimeOutMS = 0; \
    ( x )->usNTimeLeft = TIMER_TIMEOUT_INVALID; \
    ( x )->xMBMHdl = MB_HDL_INVALID; \
    ( x )->pbMBPTimerExpiredFN = NULL; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UBYTE           ubIdx;
    USHORT          usNTimeOutMS;
    USHORT          usNTimeLeft;
    xMBHandle       xMBMHdl;
    pbMBPTimerExpiredCB pbMBPTimerExpiredFN;
} xTimerInternalHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC xTimerInternalHandle arxTimerHdls[MAX_TIMER_HDLS];
STATIC BOOL bIsInitialized = FALSE;
STATIC TIM_HandleTypeDef htim6;

/* ----------------------- Static functions ---------------------------------*/
static void prvInitializeTimer( void );

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPTimerInit( xMBPTimerHandle * xTimerHdl, USHORT usTimeOut1ms,
               pbMBPTimerExpiredCB pbMBPTimerExpiredFN, xMBHandle xHdl )
{
    eMBErrorCode    eStatus = MB_EPORTERR;
    UBYTE           ubIdx;

    MBP_ENTER_CRITICAL_SECTION();
    if( ( NULL != xTimerHdl ) && ( NULL != pbMBPTimerExpiredFN ) && ( MB_HDL_INVALID != xHdl ) )
    {
        if( !bIsInitialized )
        {
            for( ubIdx = 0; ubIdx < ( UBYTE )( sizeof( arxTimerHdls ) / sizeof( arxTimerHdls[0] ) ); ubIdx++ )
            {
                RESET_HDL( &arxTimerHdls[ubIdx] );
            }
            prvInitializeTimer();
            bIsInitialized = TRUE;
        }

        for( ubIdx = 0; ubIdx < ( UBYTE )( sizeof( arxTimerHdls ) / sizeof( arxTimerHdls[0] ) ); ubIdx++ )
        {
            if( IDX_INVALID == arxTimerHdls[ubIdx].ubIdx )
            {
                break;
            }
        }
        if( MAX_TIMER_HDLS != ubIdx )
        {
            arxTimerHdls[ubIdx].ubIdx = ubIdx;
            arxTimerHdls[ubIdx].usNTimeOutMS = usTimeOut1ms;
            arxTimerHdls[ubIdx].usNTimeLeft = TIMER_TIMEOUT_INVALID;
            arxTimerHdls[ubIdx].xMBMHdl = xHdl;
            arxTimerHdls[ubIdx].pbMBPTimerExpiredFN = pbMBPTimerExpiredFN;

            *xTimerHdl = &arxTimerHdls[ubIdx];
            eStatus = MB_ENOERR;
        }
        else
        {
            eStatus = MB_ENORES;
        }
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}

void
vMBPTimerClose( xMBPTimerHandle xTimerHdl )
{
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    // if( ( NULL != pxTimerIntHdl ) && ( pxTimerIntHdl->ubIdx < MAX_TIMER_HDLS ) &&
    //     ( pxTimerIntHdl->ubIdx != IDX_INVALID ) )
    // {
    //     RESET_HDL( pxTimerIntHdl );
    // }

    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) )
    {
        RESET_HDL( pxTimerIntHdl );
    }
}


eMBErrorCode
eMBPTimerSetTimeout( xMBPTimerHandle xTimerHdl, USHORT usTimeOut1ms )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION();
    // if( ( NULL != pxTimerIntHdl ) &&
    //     ( pxTimerIntHdl->ubIdx < MAX_TIMER_HDLS ) &&
    //     ( IDX_INVALID != pxTimerIntHdl->ubIdx ) &&
    //     ( usTimeOut1ms > 0 ) && ( usTimeOut1ms != TIMER_TIMEOUT_INVALID ) )

    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) &&
    ( usTimeOut1ms > 0 ) && ( usTimeOut1ms != TIMER_TIMEOUT_INVALID ) )
    {
        pxTimerIntHdl->usNTimeOutMS = usTimeOut1ms;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}


eMBErrorCode
eMBPTimerStart( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION();
    // if( ( NULL != pxTimerIntHdl ) &&
    //     ( pxTimerIntHdl->ubIdx < MAX_TIMER_HDLS ) &&
    //     ( IDX_INVALID != pxTimerIntHdl->ubIdx ) )
    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) )
    {
        pxTimerIntHdl->usNTimeLeft = pxTimerIntHdl->usNTimeOutMS;
        eStatus = MB_ENOERR;
    }

    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}


eMBErrorCode
eMBPTimerStop( xMBPTimerHandle xTimerHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xTimerInternalHandle *pxTimerIntHdl = xTimerHdl;

    MBP_ENTER_CRITICAL_SECTION();
    // if( ( NULL != pxTimerIntHdl ) &&
    //     ( pxTimerIntHdl->ubIdx < MAX_TIMER_HDLS ) &&
    //     ( IDX_INVALID != pxTimerIntHdl->ubIdx ) )
    if( MB_IS_VALID_HDL( pxTimerIntHdl, arxTimerHdls ) )
    {
        pxTimerIntHdl->usNTimeLeft = TIMER_TIMEOUT_INVALID;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION();
    return eStatus;
}

void
TIM6_DAC_IRQHandler( void )
{
    UBYTE ubIdx;

    if( ( __HAL_TIM_GET_FLAG( &htim6, TIM_FLAG_UPDATE ) != RESET ) &&
        ( __HAL_TIM_GET_IT_SOURCE( &htim6, TIM_IT_UPDATE ) != RESET ) )
    {
        __HAL_TIM_CLEAR_IT( &htim6, TIM_IT_UPDATE );

        for(ubIdx = 0; ubIdx < ( UBYTE )( sizeof( arxTimerHdls ) / sizeof( arxTimerHdls[0] ) ); ubIdx++ )
        {
            if( ( IDX_INVALID != arxTimerHdls[ubIdx].ubIdx ) &&
                ( TIMER_TIMEOUT_INVALID != arxTimerHdls[ubIdx].usNTimeLeft ) )
            {
                arxTimerHdls[ubIdx].usNTimeLeft--;
                if( 0 == arxTimerHdls[ubIdx].usNTimeLeft )
                {
                    arxTimerHdls[ubIdx].usNTimeLeft = TIMER_TIMEOUT_INVALID;
                    ( void )arxTimerHdls[ubIdx].pbMBPTimerExpiredFN( arxTimerHdls[ubIdx].xMBMHdl );
                }
            }
        }
    }
}


void
prvInitializeTimer( void )
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    ULONG pclk1 = HAL_RCC_GetPCLK1Freq();
    ULONG timerClock = pclk1;

    if( ( RCC->CFGR & RCC_CFGR_PPRE1 ) != RCC_CFGR_PPRE1_DIV1 ) // APB1 prescaler is not 1, timer clock is multiplied by 2
    {
        timerClock *= 2U;   //APB1 预分频 = 4 分频，不是 1 分频，定时器时钟需要乘以 2
    }

    __HAL_RCC_TIM6_CLK_ENABLE();

    // 计算定时器预分频值，使得定时器计数频率为 1 kHz（1 ms 的计数周期）
    // 溢出时间 = (Prescaler + 1) × (Period + 1) /timerClock(90mhz)
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = ( ULONG )(timerClock / 1000000) - 1U; 
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000U - 1U; 
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.RepetitionCounter = 0U;

    if( HAL_TIM_Base_Init( &htim6 ) != HAL_OK )
    {
        return;
    }

    // 确保不处于单脉冲模式（One Pulse Mode），并且自动重载预装载使能（可选）
    // TIM6->CR1 &= ~TIM_CR1_OPM;        // 清除 OPM 位，使用连续模式
    // TIM6->CR1 |= TIM_CR1_ARPE;        // 使能自动重载预装载（可选，不影响周期）

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization( &htim6, &sMasterConfig );

    HAL_NVIC_SetPriority( TIM6_DAC_IRQn, 3, 0 );
    HAL_NVIC_EnableIRQ( TIM6_DAC_IRQn );

    HAL_TIM_Base_Start_IT( &htim6 );
}
