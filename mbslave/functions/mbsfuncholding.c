/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsfuncholding.c,v 1.11 2009/10/21 20:12:01 embedded-so.embedded-solutions.1 Exp $
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
#define MB_PDU_FUNC_READ_HOLDING_SIZE           ( 4 )
#define MB_PDU_FUNC_READ_ADDR_OFF               ( MB_PDU_DATA_OFF )
#define MB_PDU_FUNC_READ_REGCNT_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_HOLDING_REGCNT_MAX     (0x007D )

#define MB_PDU_FUNC_WRITE_ADDR_OFF              ( MB_PDU_DATA_OFF + 0)
#define MB_PDU_FUNC_WRITE_VALUE_OFF             ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_SIZE                  ( 4 )

#define MB_PDU_FUNC_WRITE_MUL_ADDR_OFF          ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_WRITE_MUL_VALUES_OFF        ( MB_PDU_DATA_OFF + 5 )
#define MB_PDU_FUNC_WRITE_MUL_SIZE_MIN          ( 5 )
#define MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX        ( 0x0078 )

#define MB_PDU_FUNC_READWRITE_READ_ADDR_OFF     ( MB_PDU_DATA_OFF + 0 )
#define MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF   ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF    ( MB_PDU_DATA_OFF + 4 )
#define MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF  ( MB_PDU_DATA_OFF + 6 )
#define MB_PDU_FUNC_READWRITE_BYTECNT_OFF       ( MB_PDU_DATA_OFF + 8 )
#define MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF  ( MB_PDU_DATA_OFF + 9 )
#define MB_PDU_FUNC_READWRITE_SIZE_MIN          ( 9 )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
#if MBS_FUNC_READ_HOLDING_REGISTERS_ENABLED == 1
eMBException
eMBSFuncReadHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UBYTE          *pubFrameCur;
    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    /* Length check */
    if( ( MB_PDU_FUNC_READ_HOLDING_SIZE + MB_PDU_SIZE_MIN ) == *pusMBPDULen )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF + 1] );
        usRegCount = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_REGCNT_OFF] << 8 );
        usRegCount |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_REGCNT_OFF + 1] );
        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usRegCount >= 1 ) && ( usRegCount < MB_PDU_FUNC_READ_HOLDING_REGCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pubFrameCur = &pubMBPDU[MB_PDU_FUNC_OFF];
            *pusMBPDULen = MB_PDU_FUNC_OFF;
            /* First byte contains the function code. */
            *pubFrameCur++ = MBS_FUNCCODE_READ_HOLDING_REGISTERS;
            *pusMBPDULen += ( USHORT ) 1;

            /* Second byte in the response contain the number of bytes. */
            *pubFrameCur++ = ( UBYTE ) ( usRegCount * 2 );
            *pusMBPDULen += ( USHORT ) 1;

            /* Get the acutal register values from the callback. */
            if( NULL != pxMBSRegisterCB->peMBSRegHoldingCB )
            {
                *pusMBPDULen += ( USHORT ) ( usRegCount * 2 );
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSRegHoldingCB( pxMBSRegisterCB->pvCtx, pubFrameCur, usRegAddress, usRegCount,
                                                        MBS_REGISTER_READ );

#else
                eStatus =
                    pxMBSRegisterCB->peMBSRegHoldingCB( pubFrameCur, usRegAddress, usRegCount, MBS_REGISTER_READ );
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
    return eStatus;
}

#endif

#if MBS_FUNC_WRITE_SINGLE_REGISTER_ENABLED == 1
eMBException
eMBSFuncWriteSingleRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    if( ( MB_PDU_FUNC_WRITE_SIZE + MB_PDU_SIZE_MIN ) == *pusMBPDULen )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_ADDR_OFF + 1] );

        /* Get the acutal register values from the callback. */
        if( NULL != pxMBSRegisterCB->peMBSRegHoldingCB )
        {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
            eStatus =
                pxMBSRegisterCB->peMBSRegHoldingCB( pxMBSRegisterCB->pvCtx, &pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF],
                                                    usRegAddress, 1, MBS_REGISTER_WRITE );
#else
            eStatus =
                pxMBSRegisterCB->peMBSRegHoldingCB( &pubMBPDU[MB_PDU_FUNC_WRITE_VALUE_OFF], usRegAddress, 1,
                                                    MBS_REGISTER_WRITE );
#endif
        }
        else
        {
            eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MBS_FUNC_WRITE_MULTIPLE_REGISTERS_ENABLED == 1
eMBException
eMBSFuncWriteMultipleHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UBYTE           ubRegByteCount;

    eMBException    eStatus = MB_PDU_EX_NONE;

    if( *pusMBPDULen >= ( MB_PDU_FUNC_WRITE_MUL_SIZE_MIN + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_MUL_ADDR_OFF + 1] );

        usRegCount = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF] << 8 );
        usRegCount |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_WRITE_MUL_REGCNT_OFF + 1] );

        ubRegByteCount = pubMBPDU[MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF];

        if( ( NULL != pxMBSRegisterCB->peMBSRegHoldingCB ) &&
            ( usRegCount >= 1 ) &&
            ( usRegCount <= MB_PDU_FUNC_WRITE_MUL_REGCNT_MAX ) && ( ubRegByteCount == ( UBYTE ) ( 2 * usRegCount ) ) )
        {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
            eStatus =
                pxMBSRegisterCB->peMBSRegHoldingCB( pxMBSRegisterCB->pvCtx, &pubMBPDU[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                                    usRegAddress, usRegCount, MBS_REGISTER_WRITE );
#else
            eStatus = pxMBSRegisterCB->peMBSRegHoldingCB( &pubMBPDU[MB_PDU_FUNC_WRITE_MUL_VALUES_OFF],
                                                          usRegAddress, usRegCount, MBS_REGISTER_WRITE );
#endif
            if( MB_PDU_EX_NONE == eStatus )
            {
                /* The response contains the function code, the starting
                 * address and the quantity of registers. We reuse the
                 * old values in the buffer because they are still valid.
                 */
                *pusMBPDULen = MB_PDU_FUNC_WRITE_MUL_BYTECNT_OFF;
            }
        }
        else
        {
            eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
        }
    }
    else
    {
        /* Can't be a valid request because the length is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}
#endif

#if MBS_FUNC_READWRITE_MULTIPLE_REGISTERS_ENABLED  == 1
eMBException
eMBSFuncReadWriteMultipleHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen,
                                          const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegReadAddress;
    USHORT          usRegReadCount;
    USHORT          usRegWriteAddress;
    USHORT          usRegWriteCount;
    UBYTE           ubRegWriteByteCount;
    UBYTE          *pubFrameCur;

    eMBException    eStatus = MB_PDU_EX_NONE;

    if( *pusMBPDULen >= ( MB_PDU_FUNC_READWRITE_SIZE_MIN + MB_PDU_SIZE_MIN ) )
    {
        usRegReadAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF] << 8U );
        usRegReadAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READWRITE_READ_ADDR_OFF + 1] );

        usRegReadCount = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF] << 8U );
        usRegReadCount |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READWRITE_READ_REGCNT_OFF + 1] );

        usRegWriteAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF] << 8U );
        usRegWriteAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_ADDR_OFF + 1] );

        usRegWriteCount = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF] << 8U );
        usRegWriteCount |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_REGCNT_OFF + 1] );

        ubRegWriteByteCount = pubMBPDU[MB_PDU_FUNC_READWRITE_BYTECNT_OFF];

        if( ( usRegReadCount >= 1 ) && ( usRegReadCount <= 0x7D ) &&
            ( usRegWriteCount >= 1 ) && ( usRegWriteCount <= 0x79 ) &&
            ( ( 2 * usRegWriteCount ) == ubRegWriteByteCount ) )
        {
            if( NULL != pxMBSRegisterCB->peMBSRegHoldingCB )
            {
                /* Make callback to update the register values. */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSRegHoldingCB( pxMBSRegisterCB->pvCtx,
                                                        &pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF],
                                                        usRegWriteAddress, usRegWriteCount, MBS_REGISTER_WRITE );
#else
                eStatus = pxMBSRegisterCB->peMBSRegHoldingCB( &pubMBPDU[MB_PDU_FUNC_READWRITE_WRITE_VALUES_OFF],
                                                              usRegWriteAddress, usRegWriteCount, MBS_REGISTER_WRITE );
#endif
                if( MB_PDU_EX_NONE == eStatus )
                {
                    /* Set the current PDU data pointer to the beginning. */
                    pubFrameCur = &pubMBPDU[MB_PDU_FUNC_OFF];
                    *pusMBPDULen = MB_PDU_FUNC_OFF;

                    /* First byte contains the function code. */
                    *pubFrameCur++ = MBS_FUNCCODE_READWRITE_MULTIPLE_REGISTERS;
                    *pusMBPDULen += ( USHORT ) 1;

                    /* Second byte in the response contain the number of bytes. */
                    *pubFrameCur++ = ( UBYTE ) ( usRegReadCount * 2 );
                    *pusMBPDULen += ( USHORT ) 1;

                    /* Make the read callback. */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                    eStatus =
                        pxMBSRegisterCB->peMBSRegHoldingCB( pxMBSRegisterCB->pvCtx, pubFrameCur, usRegReadAddress,
                                                            usRegReadCount, MBS_REGISTER_READ );
#else
                    eStatus =
                        pxMBSRegisterCB->peMBSRegHoldingCB( pubFrameCur, usRegReadAddress, usRegReadCount,
                                                            MBS_REGISTER_READ );
#endif
                    if( MB_PDU_EX_NONE == eStatus )
                    {
                        *pusMBPDULen += ( USHORT ) ( 2 * usRegReadCount );
                    }
                }
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
    return eStatus;
}
#endif
