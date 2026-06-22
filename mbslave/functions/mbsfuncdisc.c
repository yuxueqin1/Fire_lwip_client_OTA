/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsfuncdisc.c,v 1.8 2010/11/14 13:17:45 embedded-so.embedded-solutions.1 Exp $
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
#define MB_PDU_FUNC_READ_DISCCNT_OFF        ( MB_PDU_DATA_OFF + 2 )
#define MB_PDU_FUNC_READ_SIZE               ( 4 )
#define MB_PDU_FUNC_READ_DISCCNT_MAX        ( 0x07D0 )

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/

#if MBS_FUNC_READ_DISCRETE_ENABLED == 1
eMBException
eMBSFuncReadDiscreteInputs( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB )
    MB_CDECL_SUFFIX
{
    USHORT          usRegAddress;
    USHORT          usDiscreteCnt;
    UBYTE           ubNBytes;
    UBYTE          *pubFrameCur;

    eMBException    eStatus = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;

    if( *pusMBPDULen == ( MB_PDU_FUNC_READ_SIZE + MB_PDU_SIZE_MIN ) )
    {
        usRegAddress = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF] << 8 );
        usRegAddress |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_ADDR_OFF + 1] );

        usDiscreteCnt = ( USHORT ) ( ( USHORT ) pubMBPDU[MB_PDU_FUNC_READ_DISCCNT_OFF] << 8 );
        usDiscreteCnt |= ( USHORT ) ( pubMBPDU[MB_PDU_FUNC_READ_DISCCNT_OFF + 1] );

        /* Check if the number of registers to read is valid. If not
         * return Modbus illegal data value exception. 
         */
        if( ( usDiscreteCnt >= 1 ) && ( usDiscreteCnt < MB_PDU_FUNC_READ_DISCCNT_MAX ) )
        {
            /* Set the current PDU data pointer to the beginning. */
            pubFrameCur = &pubMBPDU[MB_PDU_FUNC_OFF];
            *pusMBPDULen = MB_PDU_FUNC_OFF;

            /* First byte contains the function code. */
            *pubFrameCur++ = MBS_FUNC_READ_DISCRETE_INPUTS;
            *pusMBPDULen += ( USHORT ) 1;

            /* Test if the quantity of coils is a multiple of 8. If not last
             * byte is only partially field with unused coils set to zero. 
             */
            if( ( usDiscreteCnt & 0x0007 ) != 0 )
            {
                ubNBytes = ( UBYTE ) ( usDiscreteCnt / 8 + 1 );
            }
            else
            {
                ubNBytes = ( UBYTE ) ( usDiscreteCnt / 8 );
            }
            *pubFrameCur++ = ubNBytes;
            *pusMBPDULen += ( USHORT ) 1;

            /* Get the acutal register values from the callback. */
            if( NULL != pxMBSRegisterCB->peMBSDiscInputCB )
            {
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
                eStatus =
                    pxMBSRegisterCB->peMBSDiscInputCB( pxMBSRegisterCB->pvCtx, pubFrameCur, usRegAddress,
                                                       usDiscreteCnt );
#else
#if defined( HI_TECH_C ) && defined( __PICC18__ )
               	eStatus = pxMBSRegisterCB->peMBSDiscInputCB( pubFrameCur, usRegAddress, usDiscreteCnt, NULL );
#else
               	eStatus = pxMBSRegisterCB->peMBSDiscInputCB( pubFrameCur, usRegAddress, usDiscreteCnt );
#endif
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
        /* Can't be a valid read discrete input request because the length
         * is incorrect. */
        eStatus = MB_PDU_EX_ILLEGAL_DATA_VALUE;
    }
    return eStatus;
}

#endif
