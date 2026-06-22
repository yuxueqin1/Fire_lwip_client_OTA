/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsfunccoils.c,v 1.6 2009/10/21 20:12:00 embedded-so.embedded-solutions.1 Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbs.h"
#include "mbsiframe.h"
#include "mbsi.h"
#include "mbsfunctions.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_PDU_FUNC_READ_ADDR_OFF           ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_COILCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_COILCNT_MAX        ( 0x07D0 )

#define MB_PDU_FUNC_WRITE_ADDR_OFF          ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_WRITE_VALUE_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE              ( 4 )

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF      ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF   ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF    ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN      ( 5 )
#define MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX   ( 0x07B0 )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

#if MBS_FUNC_READ_COILS_ENABLED == 1
eMBException
eMBSFuncReadCoils( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usCoilsCnt;
    UBYTE           ubNBytes;
    UBYTE          *pubFrameCur;

    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    if( *pusMBPDULen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        /* Additional casts a for PIC MCC18 compiler to fix a bug when -Oi is not used. 
         * This is required because it does not enforce ANSI c integer promotion
         * rules.
         */
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF + 1] );

        usCoilsCnt = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_COILCNT_OFF] << 8 );
        usCoilsCnt |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_COILCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usCoilsCnt >= 1 ) && ( usCoilsCnt < MB_PDU_FUNC_READ_COILCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pubFrameCur = &pubMBPDU[MB_PDU_FUNC_OFF];
            *pusMBPDULen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pubFrameCur++ = MBS_FUNC_READ_COILS;
            *pusMBPDULen += ( USHORT ) 1;

            /* Test if the quantity of coils is a multiple of 8. If not last
             * byte is only partially field with unused coils set to zero. 
             */
            if( ( usCoilsCnt & 0x0007 ) != 0 )
            {
                ubNBytes = ( UBYTE ) ( usCoilsCnt / 8 + 1 );
            }
            else
            {
                ubNBytes = ( UBYTE ) ( usCoilsCnt / 8 );
            }
            *pubFrameCur++ = ubNBytes;
            *pusMBPDULen += ( USHORT ) 1;

            /* Get the acutal register values from the callback. */
            if( NULL != pxMBSRegisterCB->peMBSCoilsCB )
            {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSCoilsCB( pxMBSRegisterCB->pvCtx, pubFrameCur, usRegAddress, usCoilsCnt,
                                                   MBS_REGISTER_READ );
#else
                eStatus = pxMBSRegisterCB->peMBSCoilsCB( pubFrameCur, usRegAddress, usCoilsCnt, MBS_REGISTER_READ );
#endif
                *pusMBPDULen += ( USHORT ) ubNBytes;;
            }
            else
            {
                eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
            }
        }
        else
        {
            eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid read coils request because the length
         * is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MBS_FUNC_WRITE_SINGLE_COIL_ENABLED == 1
eMBException
eMBSFuncWriteSingleCoil( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    UBYTE           ubBuf[2];
    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    if( *pusMBPDULen == ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );

        if( ( pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF + 1] == 0x00 ) &&
            ( ( pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF ) || ( pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0x00 ) ) )
        {
            ubBuf[1] = 0;
            if( pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF] == 0xFF )
            {
                ubBuf[0] = 1;
            }
            else
            {
                ubBuf[0] = 0;
            }

            if( NULL != pxMBSRegisterCB->peMBSCoilsCB )
            {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSCoilsCB( pxMBSRegisterCB->pvCtx, ubBuf, usRegAddress, 1, MBS_REGISTER_WRITE );
#else
                eStatus = pxMBSRegisterCB->peMBSCoilsCB( ubBuf, usRegAddress, 1, MBS_REGISTER_WRITE );
#endif
            }
            else
            {
                eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
            }
        }
        else
        {
            eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MBS_FUNC_WRITE_MULTIPLE_COILS_ENABLED != 0
eMBException
eMBSFuncWriteMultipleCoils( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usCoilCnt;
    UBYTE           ubByteCount;
    UBYTE           ubByteCountVerify;
    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    if( *pubMBPDU > ( MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );

        usCoilCnt = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF] << 8 );
        usCoilCnt |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_MUL_COILCNT_OFF + 1] );

        ubByteCount = pubMBPDU[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        /* Compute the number of expected bytes in the request. */
        if( ( usCoilCnt & 0x0007 ) != 0 )
        {
            ubByteCountVerify = ( UBYTE ) ( usCoilCnt / 8 + 1 );
        }
        else
        {
            ubByteCountVerify = ( UBYTE ) ( usCoilCnt / 8 );
        }

        if( ( usCoilCnt >= 1 ) &&
            ( usCoilCnt <= MB_PDU_FUNC_WRITE_MUL_COILCNT_MAX ) && ( ubByteCountVerify == ubByteCount ) )
        {
            if( NULL != pxMBSRegisterCB->peMBSCoilsCB )
            {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSCoilsCB( pxMBSRegisterCB->pvCtx, &pubMBPDU[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                                   usRegAddress, usCoilCnt, MBS_REGISTER_WRITE );
#else
                eStatus =
                    pxMBSRegisterCB->peMBSCoilsCB( &pubMBPDU[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF], usRegAddress, usCoilCnt,
                                                   MBS_REGISTER_WRITE );
#endif
                /* The response contains the function code, the starting address
                 * and the quantity of registers. We reuse the old values in the 
                 * buffer because they are still valid. 
                 */
                *pusMBPDULen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
            else
            {
                eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
            }
        }
        else
        {
            eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid write coil register request because the length
         * is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif
