/*
 * MODBUS Library: Cortex M3 Port
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbportevent.c,v 1.1 2009/01/03 09:13:01 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbtypes.h"
#include "mbportlayer.h"
#include "mbframe.h"
#include "mbutils.h"

/* ----------------------- Defines ------------------------------------------*/
/*
 * 事件句柄池容量 (主站/从站共用同一个池).
 * 每个 Modbus 实例恰好占 1 个句柄:
 *   主站: eMBPEventCreate (mbm.c pxMBMGetNewHdl)
 *   从站: eMBPEventCreate (mbs.c pxMBSGetNewHdl)
 * 本板最多同时存在 7 个实例 (6 路端口 + 第 7 路 RS485, 第7路开机即初始化),
 * 故池容量必须 >= 7. 原值 4 是库的默认值, 与本板实例数不匹配:
 * 第 5 个及以后的实例 eMBPEventCreate 返回 MB_EINVAL →
 * pxMBMGetNewHdl 返回 NULL → eMBMSerialInit 把 *pxHdl 置 NULL (mbm.c) →
 * 该端口 uart_process_modbus_master() 在入口直接返回, 永远只上传
 * 帧头 + 长度 4 + 全 0 链路掩码 (无模块数据), 且无法自愈 ——
 * uart_check_port_error_and_arm() 虽检测到链路全 0, 但重试时事件池仍满,
 * 重新初始化同样失败, 且 port_error 累加到 4 后彻底不再重试.
 * 注意: 池耗尽发生在"应答超时定时器分配成功之后", 所以只把
 * MAX_TIMER_HDLS 改大不能解决本问题 (定时器不是瓶颈).
 */
#define MAX_EVENT_HDLS          ( 8 )
#define IDX_INVALID             ( 255 )
#define EV_NONE                 ( 0 )

#define HDL_RESET( x ) do { \
    ( x )->ubIdx = IDX_INVALID; \
    ( x )->xType = EV_NONE; \
} while( 0 );

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
    UBYTE           ubIdx;
    xMBPEventType   xType;
} xEventInternalHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xEventInternalHandle arxEventHdls[MAX_EVENT_HDLS];

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBPEventCreate( xMBPEventHandle * pxEventHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    UBYTE           i;

    if( NULL != pxEventHdl )
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        if( !bIsInitialized )
        {
            for( i = 0; i < MAX_EVENT_HDLS; i++ )
            {
                HDL_RESET( &arxEventHdls[i] );
            }
            bIsInitialized = TRUE;
        }
        for( i = 0; i < MAX_EVENT_HDLS; i++ )
        {
            if( IDX_INVALID == arxEventHdls[i].ubIdx )
            {
                arxEventHdls[i].ubIdx = i;
                arxEventHdls[i].xType = EV_NONE;
                *pxEventHdl = &arxEventHdls[i];
                eStatus = MB_ENOERR;
                break;
            }
        }
        MBP_EXIT_CRITICAL_SECTION(  );
    }
    return eStatus;
}

eMBErrorCode
eMBPEventPost( const xMBPEventHandle xEventHdl, xMBPEventType xEvent )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xEventInternalHandle *pxEventHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventHdl, arxEventHdls ) )
    {
        pxEventHdl->xType = xEvent;
        eStatus = MB_ENOERR;
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

BOOL
bMBPEventGet( const xMBPEventHandle xEventHdl, xMBPEventType * pxEvent )
{
    BOOL            bEventInQueue = FALSE;
    xEventInternalHandle *pxEventHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventHdl, arxEventHdls ) )
    {
        if( EV_NONE != pxEventHdl->xType )
        {
            bEventInQueue = TRUE;
            *pxEvent = pxEventHdl->xType;
            pxEventHdl->xType = EV_NONE;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return bEventInQueue;
}

void
vMBPEventDelete( xMBPEventHandle xEventHdl )
{
    xEventInternalHandle *pxEventIntHdl = xEventHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxEventIntHdl, arxEventHdls ) )
    {
        HDL_RESET( pxEventIntHdl );
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}
