/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbutils.c,v 1.2 2008/03/06 22:01:48 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "mbport.h"
#include "stdlib.h"
#include "string.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbtypes.h"
#include "mbframe.h"
#include "mbutils.h"

/* ----------------------- Platform includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/

/* ----------------------- Defines ------------------------------------------*/
#define BITS_UCHAR      8U

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
void
xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,
                UCHAR ucValue )
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;
    USHORT          usValue = ucValue;

    assert( ucNBits <= 8 );
    assert( ( size_t )BITS_UCHAR == sizeof( UCHAR ) * 8 );

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );

    /* How many bits precede our bits to set. */
    usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );

    /* Move bit field into position over bits to set */
    usValue <<= usNPreBits;

    /* Prepare a mask for setting the new bits. */
    usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );
    usMask <<= usBitOffset - usByteOffset * BITS_UCHAR;

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;

    /* Zero out bit field bits and then or value bits into them. */
    usWordBuf = ( USHORT )( ( usWordBuf & ( ~usMask ) ) | usValue );

    /* move bits back into storage */
    ucByteBuf[usByteOffset] = ( UCHAR )( usWordBuf & 0xFF );
    ucByteBuf[usByteOffset + 1] = ( UCHAR )( usWordBuf >> BITS_UCHAR );
}

UCHAR
xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits )
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );

    /* How many bits precede our bits to set. */
    usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );

    /* Prepare a mask for setting the new bits. */
    usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;

    /* throw away unneeded bits. */
    usWordBuf >>= usNPreBits;

    /* mask away bits above the requested bitfield. */
    usWordBuf &= usMask;

    return ( UCHAR ) usWordBuf;
}

eMBException
eMBErrorcodeToException( eMBErrorCode eCode )
{
    eMBException    eException;

    switch ( eCode )
    {
    case MB_EX_ILLEGAL_FUNCTION:
        eException = MB_PDU_EX_ILLEGAL_FUNCTION;
        break;
    case MB_EX_ILLEGAL_DATA_ADDRESS:
        eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
        break;
    case MB_EX_ILLEGAL_DATA_VALUE:
        eException = MB_PDU_EX_ILLEGAL_DATA_VALUE;
        break;
    case MB_EX_SLAVE_DEVICE_FAILURE:
        eException = MB_PDU_EX_SLAVE_DEVICE_FAILURE;
        break;
    case MB_EX_ACKNOWLEDGE:
        eException = MB_PDU_EX_ACKNOWLEDGE;
        break;
    case MB_EX_SLAVE_BUSY:
        eException = MB_PDU_EX_SLAVE_BUSY;
        break;
    case MB_EX_MEMORY_PARITY_ERROR:
        eException = MB_PDU_EX_MEMORY_PARITY_ERROR;
        break;
    case MB_EX_GATEWAY_PATH_UNAVAILABLE:
        eException = MB_PDU_EX_GATEWAY_PATH_UNAVAILABLE;
        break;
    case MB_EX_GATEWAY_TARGET_FAILED:
        eException = MB_PDU_EX_GATEWAY_TARGET_FAILED;
        break;
    default:
        eException = MB_PDU_EX_SLAVE_DEVICE_FAILURE;
        break;
    }
    return eException;
}

eMBErrorCode
eMBExceptionToErrorcode( UBYTE eMBPDUException )
{
    eMBErrorCode    eStatus = MB_EIO;

    switch ( eMBPDUException )
    {
    case MB_PDU_EX_ILLEGAL_FUNCTION:
        eStatus = MB_EX_ILLEGAL_FUNCTION;
        break;
    case MB_PDU_EX_ILLEGAL_DATA_ADDRESS:
        eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
        break;
    case MB_PDU_EX_ILLEGAL_DATA_VALUE:
        eStatus = MB_EX_ILLEGAL_DATA_VALUE;
        break;
    case MB_PDU_EX_SLAVE_DEVICE_FAILURE:
        eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
        break;
    case MB_PDU_EX_ACKNOWLEDGE:
        eStatus = MB_EX_ACKNOWLEDGE;
        break;
    case MB_PDU_EX_SLAVE_BUSY:
        eStatus = MB_EX_SLAVE_BUSY;
        break;
    case MB_PDU_EX_MEMORY_PARITY_ERROR:
        eStatus = MB_EX_MEMORY_PARITY_ERROR;
        break;
    case MB_PDU_EX_GATEWAY_PATH_UNAVAILABLE:
        eStatus = MB_EX_GATEWAY_PATH_UNAVAILABLE;
        break;
    case MB_PDU_EX_GATEWAY_TARGET_FAILED:
        eStatus = MB_EX_GATEWAY_TARGET_FAILED;
        break;
    default:
        break;
    }
    return eStatus;
}
