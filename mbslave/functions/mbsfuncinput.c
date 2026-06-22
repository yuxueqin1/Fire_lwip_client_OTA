/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsfuncinput.c,v 1.6 2010/11/13 14:29:15 embedded-so.embedded-solutions.1 Exp $
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
#define MB_PDU_FUNC_READ_REGCNT_OFF         ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_REGCNT_MAX         ( 0x007D )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
#if MBS_FUNC_READ_INPUT_REGISTERS_ENABLED == 1
eMBException
eMBSFuncReadInputRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usRegCount;
    UBYTE          *pubFrameCur;

    eMBException    eStatus = MB_PDU_EX_NONE;

    if( *pusMBPDULen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF + 1] );

        usRegCount = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_REGCNT_OFF] << 8 );
        usRegCount |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_REGCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usRegCount >= 1 ) && ( usRegCount < MB_PDU_FUNC_READ_REGCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pubFrameCur = &pubMBPDU[MB_PDU_FUNC_OFF];
            *pusMBPDULen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pubFrameCur++ = MBS_FUNCCODE_READ_INPUT_REGISTERS;
            *pusMBPDULen += ( USHORT ) 1;

            /* Second byte in the response contain the number of bytes. */
            *pubFrameCur++ = ( UBYTE ) ( usRegCount * 2 );
            *pusMBPDULen += ( USHORT ) 1;

            /* Get the acutal register values from the callback. */
            if( NULL != pxMBSRegisterCB->peMBSRegInputCB )
            {
                *pusMBPDULen += ( USHORT ) ( usRegCount * 2 );
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSRegInputCB( pxMBSRegisterCB->pvCtx, pubFrameCur, usRegAddress, usRegCount );
#else
#if defined( HI_TECH_C ) && defined( __PICC18__ )
               eStatus = pxMBSRegisterCB->peMBSRegInputCB( pubFrameCur, usRegAddress, usRegCount, NULL );
#else
               eStatus = pxMBSRegisterCB->peMBSRegInputCB( pubFrameCur, usRegAddress, usRegCount );
#endif
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
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
