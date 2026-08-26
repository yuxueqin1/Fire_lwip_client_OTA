/* 
 * MODBUS Library: A portable MODBUS master for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007-2010 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbmfuncinput.c,v 1.16 2011/01/02 16:15:36 embedded-solutions.cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#if defined( __18CXX )
#include <stdio.h>
#endif

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"
#include "stm32f4xx_hal.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mbm.h"
#include "mbframe.h"
#include "mbutils.h"
#include "mbportlayer.h"
#include "mbmiframe.h"
#include "mbmi.h"

/*lint --e{788} ~ suppress messages: enum constant not used within defaulted switch */

/* ----------------------- Defines ------------------------------------------*/
#define MBM_FUNC_RD_INPUT_REG                   ( 0x04 )
#define MBM_PDU_FUNC_RD_REGCNT_MAX              ( 0x00FA )  /* 250: 放宽以支持 0x48 等大寄存器模块 (>125 regs) */
#define MBM_PDU_FUNC_RD_RESP_SIZE_BYTECNT_OFF   ( MB_PDU_DATA_OFF )
#define MBM_PDU_FUNC_RD_RESP_REGS_OFF           ( MBM_PDU_FUNC_RD_RESP_SIZE_BYTECNT_OFF + 1 )

#define MBM_PDU_FUNC_RD_RESP_SIZE( nRegs )      ( USHORT )( 1 + 1 + 2 * ( nRegs ) )     /*!< Expected size of response. */

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation (Polling functions)----------*/
#if MBM_FUNC_RD_INPUT_REGS_ENABLED == 1
void
vMBMReadInputRegistersPolled( xMBHandle xHdl, UCHAR ucSlaveAddress, USHORT usRegStartAddress, UBYTE ubNRegs,
                              USHORT arusBufferOut[], eMBMQueryState * peState, eMBErrorCode * peStatus )
{
    UBYTE           ubNRegCnt;
    xMBMInternalHandle *pxIntHdl = ( xMBMInternalHandle * ) xHdl;
    UBYTE          *pubCurPtr;

#if MBM_ENABLE_FULL_API_CHECKS == 1
    if( bMBMIsHdlValid( pxIntHdl ) && ( peState != NULL ) && ( peStatus != NULL ) )
#else
    if( TRUE )
#endif
    {
        switch ( *peState )
        {
            /* In this state we prepare the frame and send it. */
        case MBM_STATE_NONE:
            if( ( ( ( ULONG ) usRegStartAddress + ( ULONG ) ubNRegs ) < ( ULONG ) 0x10000 ) &&
                ( ubNRegs > 0 ) && ( ubNRegs <= MBM_PDU_FUNC_RD_REGCNT_MAX ) )
            {
#if defined( MBP_ENABLE_DEBUG_FACILITY ) && ( MBP_ENABLE_DEBUG_FACILITY == 1 )
                if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_CORE ) )
                {
                    vMBPPortLog( MB_LOG_DEBUG, MB_LOG_CORE,
                                 "[IDX=" MBP_FORMAT_USHORT "] Read input registers request (start=" MBP_FORMAT_USHORT
                                 ", length=" MBP_FORMAT_USHORT ").\n", ( USHORT ) pxIntHdl->ubIdx, usRegStartAddress,
                                 ( USHORT ) ubNRegs );
                }
#endif
                pxIntHdl->usFrameMBPDULength = 0;
                pxIntHdl->pubFrameMBPDUBuffer[pxIntHdl->usFrameMBPDULength] = MBM_FUNC_RD_INPUT_REG;
				pxIntHdl->usFrameMBPDULength++;
                pxIntHdl->pubFrameMBPDUBuffer[pxIntHdl->usFrameMBPDULength] = ( UBYTE ) ( usRegStartAddress >> 8U );
				pxIntHdl->usFrameMBPDULength++;
                pxIntHdl->pubFrameMBPDUBuffer[pxIntHdl->usFrameMBPDULength] =
                    ( UBYTE ) ( usRegStartAddress & 0x00FFU );
				pxIntHdl->usFrameMBPDULength++;
                pxIntHdl->pubFrameMBPDUBuffer[pxIntHdl->usFrameMBPDULength] = 0;
				pxIntHdl->usFrameMBPDULength++;
                pxIntHdl->pubFrameMBPDUBuffer[pxIntHdl->usFrameMBPDULength] = ubNRegs;
				pxIntHdl->usFrameMBPDULength++;

                /* Frame is assembled. Now send it. */
                *peState = MBM_STATE_SEND;
            }
            else
            {
                *peStatus = MB_EINVAL;
                *peState = MBM_STATE_DONE;
            }
            break;

            /* These states are handled by the common state machine. */
        case MBM_STATE_SEND:
        case MBM_STATE_WAITING:
            vMBMMasterTransactionPolled( pxIntHdl, ucSlaveAddress, peState, peStatus ); //标记
            break;

            /* We need to disassemble the response here. */
        case MBM_STATE_DISASSEMBLE:
            *peState = MBM_STATE_DONE;
            if( ( MBM_PDU_FUNC_RD_RESP_SIZE( ubNRegs ) == pxIntHdl->usFrameMBPDULength ) &&
                ( MBM_FUNC_RD_INPUT_REG == pxIntHdl->pubFrameMBPDUBuffer[MB_PDU_FUNC_OFF] ) &&
                ( ( ( ( 2 * ubNRegs ) & 0xFF ) == pxIntHdl->pubFrameMBPDUBuffer[MBM_PDU_FUNC_RD_RESP_SIZE_BYTECNT_OFF] ) ||
                  ( ( ( 2 * ubNRegs ) > 255U ) &&                       /* 读 >127 寄存器: 字节计数字段溢出, 模块回填 0xFF */
                    ( 0xFFU == pxIntHdl->pubFrameMBPDUBuffer[MBM_PDU_FUNC_RD_RESP_SIZE_BYTECNT_OFF] ) ) ) )
            {

                /* A pointer pointing to the first register value in the stream. */
                pubCurPtr = &( pxIntHdl->pubFrameMBPDUBuffer[MBM_PDU_FUNC_RD_RESP_REGS_OFF] );
                for( ubNRegCnt = 0; ubNRegCnt < ubNRegs; ubNRegCnt++ )
                {
                    arusBufferOut[ubNRegCnt] = ( USHORT ) ( ( USHORT ) * pubCurPtr++ << 8U );
                    arusBufferOut[ubNRegCnt] |= ( USHORT ) * pubCurPtr++;
                }
                *peStatus = MB_ENOERR;
            }
            /* Check for exception frame. */
            else if( ( MB_PDU_EX_RESP_SIZE == pxIntHdl->usFrameMBPDULength ) &&
                     MB_PDU_FUNC_ISEXCEPTION_FOR( pxIntHdl->pubFrameMBPDUBuffer[MB_PDU_FUNC_OFF],
                                                  MBM_FUNC_RD_INPUT_REG ) )
            {
                *peStatus = eMBExceptionToErrorcode( pxIntHdl->pubFrameMBPDUBuffer[MB_PDU_EX_CODE_OFF] );
            }
            else
            {
                *peStatus = MB_EIO;
            }
            break;

        case MBM_STATE_ERROR:
            /* No cleanup required. */
            *peState = MBM_STATE_DONE;
            break;

        default:
            *peState = MBM_STATE_DONE;
            *peStatus = MB_EILLSTATE;
        }
#if defined( MBP_ENABLE_DEBUG_FACILITY ) && ( MBP_ENABLE_DEBUG_FACILITY == 1 )
        if( MBM_STATE_DONE == *peState )
        {
            if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_CORE ) )
            {
                vMBPPortLog( MB_LOG_DEBUG, MB_LOG_CORE,
                             "[IDX=" MBP_FORMAT_USHORT "] Read input registers finished. Status: " MBP_FORMAT_USHORT
                             "\n", ( USHORT ) pxIntHdl->ubIdx, ( USHORT ) * peStatus );
            }
        }
#endif
    }
#if MBM_ENABLE_FULL_API_CHECKS == 1
    else
    {
        if( NULL != peState )
        {
            *peState = MBM_STATE_DONE;
        }
        if( NULL != peStatus )
        {
            *peStatus = MB_EINVAL;
        }
    }
#endif
}
#endif

/* ----------------------- Start implementation (Blocking functions) --------*/
#if MBM_FUNC_RD_INPUT_REGS_ENABLED == 1
eMBErrorCode
eMBMReadInputRegisters( xMBHandle xHdl, UCHAR ucSlaveAddress,
                        USHORT usRegStartAddress, UBYTE ubNRegs, USHORT arusBufferOut[] )
{
    eMBErrorCode    eStatus;
    eMBMQueryState  eState;

    /*
     * ============================================================
     * Modbus RTU 协议限制:
     *   应答帧的 "字节计数" 字段仅有 1 个字节 (最大 0xFF = 255),
     *   每个寄存器占 2 字节. 因此当请求超过 127 个寄存器时,
     *   应答的字节计数字段会溢出 — 见 vMBMReadInputRegistersPolled
     *   DISASSEMBLE 态对该情况的放宽处理 (读 >127 寄存器时接受
     *   模块回填的 0xFF).
     *
     *   当前项目中, 超过此限制的仅有 "高频 25Hz 轨道接收模块"
     *   (FLFXG_GD_25_J_HIGH, 0x48), 固定需要 133 个输入寄存器.
     *
     * 注意: 对此类大寄存器模块绝不能再做 "拆分多次读取".
     *   实测该模块会无视请求的寄存器数量, 永远返回其全部
     *   133 个寄存器 (266 字节) 的固定应答. 若拆成两次事务,
     *   第二次读取返回的仍是整段数据 (从寄存器 0 起), 会覆盖
     *   arusBufferOut[100..], 数据全错. 因此必须一次性请求
     *   全部寄存器, 靠 DISASSEMBLE 态的放宽字节计数校验来
     *   接受其 266 字节应答.
     * ============================================================
     */

    /* 单次 Modbus 事务: 一次性请求全部 ubNRegs 个寄存器. */
    eState = MBM_STATE_NONE;
    do
    {
        vMBMReadInputRegistersPolled( xHdl, ucSlaveAddress, usRegStartAddress,
                                      ubNRegs, arusBufferOut, &eState, &eStatus );
        HAL_Delay( 1 );
    }
    while( eState != MBM_STATE_DONE );
    return eStatus;
}
#endif
