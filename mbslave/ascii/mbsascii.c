/*
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2009 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsascii.c,v 1.18 2010/11/13 14:28:19 embedded-so.embedded-solutions.1 Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include <string.h>

/* ----------------------- Platform includes --------------------------------*/
#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbs.h"
#include "mbportlayer.h"
#include "mbutils.h"
#include "mbsiframe.h"
#include "mbsi.h"
#include "mbsascii.h"

#if MBS_ASCII_ENABLED == 1

/* ----------------------- Defines ------------------------------------------*/
#define MBS_SER_PDU_SIZE_MIN        ( 4 )       /*!< Minimum size of a MODBUS ASCII frame. */
#define MBS_SER_PDU_SIZE_MAX        ( 255 )     /*!< Maximum size of a MODBUS ASCII frame. */
#define MBS_SER_PDU_SIZE_LRC        ( 1 )       /*!< Size of LRC checksum. */
#define MBS_SER_PDU_ADDR_OFF        ( 0 )       /*!< Offset of slave address in ASCII frame. */
#define MBS_SER_PDU_PDU_OFF         ( 1 )       /*!< Offset of Modbus-PDU in ASCII frame. */

#define MBS_ASCII_DEFAULT_CR        ( 0x0D )    /*!< Default CR character for MODBUS ASCII. */
#define MBS_ASCII_DEFAULT_LF        ( 0x0A )    /*!< Default LF character for MODBUS ASCII. */
#define MBS_ASCII_START             ( 0x3A )    /*!< Start character ':' for MODBUS ASCII. */
#define IDX_INVALID                 ( 255 )

#define HDL_RESET_RX( x ) do { \
    ( x )->eRcvState = STATE_RX_IDLE; \
    ( x )->usRcvBufferPos = 0; \
    ( x )->eBytePos = BYTE_HIGH_NIBBLE; \
} while( 0 );

#define HDL_RESET_TX( x ) do { \
    ( x )->eSndState = STATE_TX_IDLE; \
    ( x )->usSndBufferCnt = 0; \
    ( x )->eBytePos = BYTE_HIGH_NIBBLE; \
    ( x )->pubSndBufferCur = NULL; \
} while( 0 );

#define HDL_RESET_BASIC( x ) do { \
    ( x )->ubIdx = IDX_INVALID; \
    HDL_RESET_RX( x ); \
    HDL_RESET_TX( x ); \
    ( x )->xTmrHdl = MBP_TIMERHDL_INVALID; \
    ( x )->xSerHdl = MBP_SERIALHDL_INVALID; \
    MBP_MEMSET( ( void * )&( ( x )->ubASCIIFrameBuffer[ 0 ] ), 0, MBS_SER_PDU_SIZE_MAX ); \
} while( 0 );

#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
#define HDL_RESET_WAITAFTERSEND( x ) do { \
    ( x )->xWaitTmrHdl = MBP_TIMERHDL_INVALID; \
} while( 0 );
#else
#define HDL_RESET_WAITAFTERSEND( x )
#endif

#if MBS_ASCII_BACKOF_TIME_MS > 0
#define HDL_RESET_BACKOF_TIME( x ) do { \
    ( x )->xBackOffTmrHdl = MBP_TIMERHDL_INVALID; \
} while( 0 );
#else
#define HDL_RESET_BACKOF_TIME( x )
#endif

#define HDL_RESET( x ) do { \
    HDL_RESET_BASIC( x ); \
    HDL_RESET_BACKOF_TIME( x ); \
    HDL_RESET_WAITAFTERSEND( x ); \
} while( 0 );


/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_WAIT_EOF,          /*!< Wait for End of Frame. */
    STATE_RX_ERROR              /*!< Error during receive. */
} eMBSASCIIRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_START,             /*!< Starting transmission (':' sent). */
    STATE_TX_DATA,              /*!< Sending of data (Address, Data, LRC). */
    STATE_TX_END                /*!< End of transmission. */
} eMBSASCIISndState;

typedef enum
{
    BYTE_HIGH_NIBBLE,           /*!< Character for high nibble of byte. */
    BYTE_LOW_NIBBLE             /*!< Character for low nibble of byte. */
} eMBSASCIIBytePos;

typedef struct
{
    UBYTE           ubIdx;
    volatile UBYTE  ubASCIIFrameBuffer[MBS_SER_PDU_SIZE_MAX];
    eMBSASCIIBytePos eBytePos;
    volatile eMBSASCIIRcvState eRcvState;
    volatile USHORT usRcvBufferPos;

    volatile eMBSASCIISndState eSndState;
    volatile USHORT usSndBufferCnt;
    UBYTE          *pubSndBufferCur;

    xMBPTimerHandle xTmrHdl;
#if MBS_ASCII_BACKOF_TIME_MS > 0
    xMBPTimerHandle xBackOffTmrHdl;
#endif
#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
    xMBPTimerHandle xWaitTmrHdl;
#endif
    xMBPSerialHandle xSerHdl;
} xMBSASCIIFrameHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xMBSASCIIFrameHandle xMBSASCIIFrameHdl[MBS_SERIAL_ASCII_MAX_INSTANCES];

/* ----------------------- Static functions ---------------------------------*/
#if MBS_TEST_INSTANCES == 0
STATIC INLINE
#endif
UBYTE           ubMBSSerialASCIICHAR2BIN( UBYTE ubCharacter );

#if MBS_TEST_INSTANCES == 0
STATIC INLINE
#endif
UBYTE           ubMBSSerialASCIIBIN2CHAR( UBYTE ubByte );
STATIC UBYTE    ubMBSSerialASCIILRC( const UBYTE * pucFrame, USHORT usLen );

/**INDENT-OFF* */
#if MBS_ASCII_BACKOF_TIME_MS > 0
STATIC  BOOL    bMBSSerialASCIIBackOffTimerCB( xMBHandle xHdl ) MB_CDECL_SUFFIX;
#endif

#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
STATIC BOOL     bMBSSerialWaitCB( xMBHandle xHdl );
#endif

#if MBS_TEST_INSTANCES == 0
STATIC
#endif
BOOL                bMBSSerialASCIITimerCB( xMBHandle xHdl ) MB_CDECL_SUFFIX;
STATIC eMBErrorCode eMBSSerialASCIIFrameSend( xMBHandle xHdl, USHORT usMBPDULength );
STATIC eMBErrorCode eMBSSerialASCIIFrameReceive( xMBHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength )MB_CDECL_SUFFIX;
STATIC eMBErrorCode eMBSSerialASCIIFrameClose( xMBHandle xHdl );
STATIC eMBErrorCode eMBSSerialASCIIFrameCloseInternal( xMBSASCIIFrameHandle * pxASCIIHdl );

#if MBS_SERIAL_API_VERSION == 2
STATIC BOOL     bMBSSerialTransmitterEmptyAPIV2CB( xMBHandle xHdl, UBYTE * pubBufferOut, USHORT usBufferMax,
                                                        USHORT * pusBufferWritten ) MB_CDECL_SUFFIX;
STATIC void     vMBPSerialReceiverAPIV2CB( xMBHandle xHdl, const UBYTE * pubBufferIn, USHORT usBufferLen ) MB_CDECL_SUFFIX;
#elif MBS_SERIAL_API_VERSION == 1

STATIC BOOL     bMBSSerialTransmitterEmptyAPIV1CB( xMBHandle xHdl, UBYTE * pubValue ) MB_CDECL_SUFFIX;
STATIC void     vMBPSerialReceiverAPIV1CB( xMBHandle xHdl, UBYTE ubValue ) MB_CDECL_SUFFIX;
#else
#error "unsupported serial API version set in configuration."
#endif
/**INDENT-ON* */

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/

/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBSSerialASCIIInit( xMBSInternalHandle * pxIntHdl, UBYTE ubPort, ULONG ulBaudRate, eMBSerialParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR, eStatus2;
    xMBSASCIIFrameHandle *pxFrameHdl = NULL;
    UBYTE           ubIdx;
    USHORT          usTimeoutMS;
    UCHAR           ucStopBits;

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( ( NULL != pxIntHdl ) && ( ulBaudRate > 0 ) )
#else
    if( TRUE )
#endif
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        if( !bIsInitialized )
        {
            for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBSASCIIFrameHdl ); ubIdx++ )
            {
                HDL_RESET( &xMBSASCIIFrameHdl[ubIdx] );
            }
            bIsInitialized = TRUE;
        }

        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBSASCIIFrameHdl ); ubIdx++ )
        {
            if( IDX_INVALID == xMBSASCIIFrameHdl[ubIdx].ubIdx )
            {
                pxFrameHdl = &xMBSASCIIFrameHdl[ubIdx];
                pxFrameHdl->ubIdx = ubIdx;
                break;
            }
        }

        if( NULL != pxFrameHdl )
        {
            usTimeoutMS = ( USHORT ) ( MBS_ASCII_TIMEOUT_SEC * 1000U );

            /* No parity requires two stop bits. */
            ucStopBits = MB_PAR_NONE == eParity ? ( UCHAR ) 2 : ( UCHAR ) 1;

            if( MB_ENOERR !=
                ( eStatus2 =
                  eMBPSerialInit( &( pxFrameHdl->xSerHdl ), ( UCHAR ) ubPort, ulBaudRate, 7, eParity, ucStopBits,
                                  pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
            else if( MB_ENOERR != ( eStatus2 = eMBPTimerInit( &( pxFrameHdl->xTmrHdl ), usTimeoutMS,
                                                              bMBSSerialASCIITimerCB, pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
#if MBS_ASCII_BACKOF_TIME_MS > 0
            else if( MB_ENOERR != ( eStatus2 = eMBPTimerInit( &( pxFrameHdl->xBackOffTmrHdl ),
                                                              MBS_ASCII_BACKOF_TIME_MS,
                                                              bMBSSerialASCIIBackOffTimerCB, pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
#endif
#if MBS_ASCII_WAITAFTERSEND_ENABLED == 1
            else if( MB_ENOERR != ( eStatus2 = eMBPTimerInit( &( pxFrameHdl->xWaitTmrHdl ),
            		MBS_SERIAL_ASCII_DYNAMIC_WAITAFTERSEND_TIMEOUT_MS( ulBaudRate ), bMBSSerialWaitCB, pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
#endif
            else
            {
                /* Attach the frame handle to the protocl stack. */
                pxIntHdl->pubFrameMBPDUBuffer = ( UBYTE * ) & pxFrameHdl->ubASCIIFrameBuffer[MBS_SER_PDU_PDU_OFF];
                pxIntHdl->xFrameHdl = pxFrameHdl;
                pxIntHdl->pFrameSendFN = eMBSSerialASCIIFrameSend;
                pxIntHdl->pFrameRecvFN = eMBSSerialASCIIFrameReceive;
                pxIntHdl->pFrameCloseFN = eMBSSerialASCIIFrameClose;
                eStatus = MB_ENOERR;
#if MBS_SERIAL_API_VERSION == 2
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#elif MBS_SERIAL_API_VERSION == 1
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#else
#error "Define either MBS_SERIAL_API_VERSION=1 or 2!"
#endif
            }

            if( MB_ENOERR != eStatus )
            {
                ( void )eMBSSerialASCIIFrameCloseInternal( pxFrameHdl );
            }
        }
        else
        {
            eStatus = MB_ENORES;
        }
        MBP_EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          eMBErrorCode
eMBSSerialASCIIFrameSend( xMBHandle xHdl, USHORT usMBPDULength )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIHdl;
    UBYTE           ubLRC;

#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
    xMBAnalyzerFrame xAnalyzerFrame;
    xMBPTimeStamp   xTimeStamp;
#endif

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        pxASCIIHdl = pxIntHdl->xFrameHdl;

        if( MB_IS_VALID_HDL( pxASCIIHdl, xMBSASCIIFrameHdl ) )
        {
            if( 0 == usMBPDULength )
            {
#if MBS_SERIAL_API_VERSION == 2
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxASCIIHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#elif MBS_SERIAL_API_VERSION == 1
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxASCIIHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#else
#error "Define either MBS_SERIAL_API_VERSION=1 or 2!"
#endif
                else
                {
                    eStatus = MB_ENOERR;
                }
            }
            else if( usMBPDULength <= ( MBS_SER_PDU_SIZE_MAX - ( 1 /* Slave Address */  + 1 /* LRC */  ) ) )
            {
                MBP_ASSERT( STATE_TX_IDLE == pxASCIIHdl->eSndState );

                /* Added the MODBUS ASCII header (= slave address) */
                pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF] = pxIntHdl->ubSlaveAddress;
                pxASCIIHdl->usSndBufferCnt = 1;

                /* MODBUS PDU is already embedded in the frame. */
                pxASCIIHdl->usSndBufferCnt += usMBPDULength;

                ubLRC =
                    ubMBSSerialASCIILRC( ( UBYTE * ) & ( pxASCIIHdl->ubASCIIFrameBuffer[0] ),
                                         pxASCIIHdl->usSndBufferCnt );
                pxASCIIHdl->ubASCIIFrameBuffer[pxASCIIHdl->usSndBufferCnt++] = ubLRC;

                /* Enable transmitter */
                pxASCIIHdl->eSndState = STATE_TX_START;
                pxASCIIHdl->pubSndBufferCur = ( UBYTE * ) & pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF];
#if MBS_SERIAL_API_VERSION == 2
                if( MB_ENOERR !=
                    eMBPSerialTxEnable( pxASCIIHdl->xSerHdl,
                                        ( pbMBPSerialTransmitterEmptyCB ) bMBSSerialTransmitterEmptyAPIV2CB ) )
                {
                    eStatus = MB_EPORTERR;
                    HDL_RESET_TX( pxASCIIHdl );
                }
#elif MBS_SERIAL_API_VERSION == 1
                if( MB_ENOERR !=
                    eMBPSerialTxEnable( pxASCIIHdl->xSerHdl,
                                        ( pbMBPSerialTransmitterEmptyCB ) bMBSSerialTransmitterEmptyAPIV1CB ) )
                {
                    eStatus = MB_EPORTERR;
                    HDL_RESET_TX( pxASCIIHdl );
                }
#else
#error "Define either MBS_SERIAL_API_VERSION=1 or 2!"
#endif
                else
                {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                    pxIntHdl->xFrameStat.ulNPacketsSent += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                    /* Pass frame to protocol analyzer. */
                    xAnalyzerFrame.eFrameDir = MB_FRAME_SEND;
                    xAnalyzerFrame.eFrameType = MB_FRAME_ASCII;
                    xAnalyzerFrame.x.xASCIIHeader.ubSlaveAddress = pxIntHdl->ubSlaveAddress;
                    xAnalyzerFrame.x.xASCIIHeader.ubLRC = ubLRC;
                    xAnalyzerFrame.ubDataPayload =
                        ( const UBYTE * )&pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF];
                    xAnalyzerFrame.usDataPayloadLength = pxASCIIHdl->usSndBufferCnt;
                    if( NULL != pxIntHdl->pvMBAnalyzerCallbackFN )
                    {
                        vMBPGetTimeStamp( &xTimeStamp );
                        pxIntHdl->pvMBAnalyzerCallbackFN( pxIntHdl, pxIntHdl->pvCtx, &xTimeStamp, &xAnalyzerFrame );
                    }
#endif
                    eStatus = MB_ENOERR;
                }
            }
        }
    }
    return eStatus;
}

STATIC          eMBErrorCode
eMBSSerialASCIIFrameReceive( xMBHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength )
    MB_CDECL_SUFFIX
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIHdl;

#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
    xMBAnalyzerFrame xAnalyzerFrame;
    xMBPTimeStamp   xTimeStamp;
#endif

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        pxASCIIHdl = pxIntHdl->xFrameHdl;
        if( MB_IS_VALID_HDL( pxASCIIHdl, xMBSASCIIFrameHdl ) )
        {
            if( ( pxASCIIHdl->usRcvBufferPos >= MBS_SER_PDU_SIZE_MIN ) &&
                ( ubMBSSerialASCIILRC( ( UBYTE * ) & ( pxASCIIHdl->ubASCIIFrameBuffer[0] ), pxASCIIHdl->usRcvBufferPos )
                  == 0 ) )
            {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                pxIntHdl->xFrameStat.ulNPacketsReceived += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                xAnalyzerFrame.eFrameType = MB_FRAME_ASCII;
                xAnalyzerFrame.x.xASCIIHeader.ubSlaveAddress = pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF];
                xAnalyzerFrame.x.xASCIIHeader.ubLRC = pxASCIIHdl->ubASCIIFrameBuffer[pxASCIIHdl->usRcvBufferPos - 1];
#endif
                *pubSlaveAddress = pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF];
                *pusMBPDULength =
                    ( USHORT ) ( pxASCIIHdl->usRcvBufferPos - ( MBS_SER_PDU_PDU_OFF + MBS_SER_PDU_SIZE_LRC ) );
                eStatus = MB_ENOERR;
            }
            else
            {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                pxIntHdl->xFrameStat.ulNChecksumErrors += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                xAnalyzerFrame.eFrameType = MB_FRAME_DAMAGED;
#endif
                eStatus = MB_EIO;
            }
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
            xAnalyzerFrame.eFrameDir = MB_FRAME_RECEIVE;
            xAnalyzerFrame.ubDataPayload = ( const UBYTE * )&pxASCIIHdl->ubASCIIFrameBuffer[MBS_SER_PDU_ADDR_OFF];
            xAnalyzerFrame.usDataPayloadLength = pxASCIIHdl->usRcvBufferPos;
            if( NULL != pxIntHdl->pvMBAnalyzerCallbackFN )
            {
                vMBPGetTimeStamp( &xTimeStamp );
                pxIntHdl->pvMBAnalyzerCallbackFN( pxIntHdl, pxIntHdl->pvCtx, &xTimeStamp, &xAnalyzerFrame );
            }
#endif
            HDL_RESET_RX( pxASCIIHdl );
        }
    }

    return eStatus;
}

#if MBS_SERIAL_API_VERSION == 2
#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          BOOL
bMBSSerialTransmitterEmptyAPIV2CB( xMBHandle xHdl, UBYTE * pubBufferOut, USHORT usBufferMax, USHORT * pusBufferWritten )
    MB_CDECL_SUFFIX
{
    BOOL            bMoreTxData = FALSE;
    BOOL            bEnableRx = FALSE;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxASCIIFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( NULL != pxASCIIFrameHdl );
    MBP_ASSERT( pxASCIIFrameHdl->eRcvState == STATE_RX_IDLE );
    MBP_ASSERT( usBufferMax > 0 );

    *pusBufferWritten = 0;
    do
    {
        switch ( pxASCIIFrameHdl->eSndState )
        {
        case STATE_TX_START:
            *pubBufferOut++ = MBS_ASCII_START;
            *pusBufferWritten = ( USHORT ) ( *pusBufferWritten + 1 );
            pxASCIIFrameHdl->eSndState = STATE_TX_DATA;
            pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
            bMoreTxData = TRUE;
            break;

        case STATE_TX_DATA:
            if( pxASCIIFrameHdl->usSndBufferCnt > 0 )
            {
                MBP_ASSERT( NULL != pxASCIIFrameHdl->pubSndBufferCur );
                switch ( pxASCIIFrameHdl->eBytePos )
                {
                case BYTE_HIGH_NIBBLE:
                    *pubBufferOut++ =
                        ubMBSSerialASCIIBIN2CHAR( ( UBYTE ) ( *( pxASCIIFrameHdl->pubSndBufferCur ) >> 4 ) );
                    *pusBufferWritten = ( USHORT ) ( *pusBufferWritten + 1 );
                    pxASCIIFrameHdl->eBytePos = BYTE_LOW_NIBBLE;
                    break;

                case BYTE_LOW_NIBBLE:
                    *pubBufferOut++ =
                        ubMBSSerialASCIIBIN2CHAR( ( UBYTE ) ( *( pxASCIIFrameHdl->pubSndBufferCur ) & 0x0F ) );
                    *pusBufferWritten = ( USHORT ) ( *pusBufferWritten + 1 );
                    pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
                    pxASCIIFrameHdl->pubSndBufferCur++;
                    pxASCIIFrameHdl->usSndBufferCnt--;
                    break;
                }
                bMoreTxData = TRUE;
            }
            else
            {
                *pubBufferOut++ = MBS_ASCII_DEFAULT_CR;
                *pusBufferWritten = ( USHORT ) ( *pusBufferWritten + 1 );
                bMoreTxData = TRUE;
                pxASCIIFrameHdl->eSndState = STATE_TX_END;
            }
            break;

        case STATE_TX_END:
            *pubBufferOut++ = MBS_ASCII_DEFAULT_LF;
            *pusBufferWritten = ( USHORT ) ( *pusBufferWritten + 1 );
            bMoreTxData = TRUE;
            pxASCIIFrameHdl->eSndState = STATE_TX_IDLE;
            break;

        case STATE_TX_IDLE:
            bEnableRx = TRUE;
            break;
        }
    }
    while( ( pxASCIIFrameHdl->eSndState != STATE_TX_IDLE ) && ( *pusBufferWritten < usBufferMax ) );

    if( !bMoreTxData )
    {
        HDL_RESET_TX( pxASCIIFrameHdl );
    }
    else
    {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
        pxIntHdl->xFrameStat.ulNBytesSent += *pusBufferWritten;
#endif
    }
    if( bEnableRx )
    {
#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
        if( MB_ENOERR != eMBPTimerStart( pxASCIIFrameHdl->xWaitTmrHdl ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#else
        if( MB_ENOERR !=
            eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#endif
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return bMoreTxData;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC void
vMBPSerialReceiverAPIV2CB( xMBHandle xHdl, const UBYTE * pubBufferIn, USHORT usBufferLen )
    MB_CDECL_SUFFIX
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl;
    eMBErrorCode    eStatus;
    USHORT          usBufferLeft = usBufferLen;
    UBYTE           ubValue;
    UBYTE           ubBinValue;
    BOOL            bEnableTimer = TRUE;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxASCIIFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( NULL != pxASCIIFrameHdl );
    MBP_ASSERT( pxASCIIFrameHdl->eSndState == STATE_TX_IDLE );

    while( usBufferLeft > 0 )
    {
        ubValue = *pubBufferIn++;
        usBufferLeft--;
        switch ( pxASCIIFrameHdl->eRcvState )
        {
        case STATE_RX_RCV:
            if( MBS_ASCII_START == ubValue )
            {
                /* Empty receive buffer. */
                pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
                pxASCIIFrameHdl->usRcvBufferPos = 0;
            }
            else if( MBS_ASCII_DEFAULT_CR == ubValue )
            {
                pxASCIIFrameHdl->eRcvState = STATE_RX_WAIT_EOF;
            }
            else
            {
                ubBinValue = ubMBSSerialASCIICHAR2BIN( ubValue );
                switch ( pxASCIIFrameHdl->eBytePos )
                {
                case BYTE_HIGH_NIBBLE:
                    /* High nibble comes first. We need to check for an overflow. */
                    if( pxASCIIFrameHdl->usRcvBufferPos < MBS_SER_PDU_SIZE_MAX )
                    {
                        pxASCIIFrameHdl->ubASCIIFrameBuffer[pxASCIIFrameHdl->usRcvBufferPos] =
                            ( UBYTE ) ( ubBinValue << 4 );
                        pxASCIIFrameHdl->eBytePos = BYTE_LOW_NIBBLE;
                    }
                    else
                    {
                        pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
                    }
                    break;

                case BYTE_LOW_NIBBLE:
                    pxASCIIFrameHdl->ubASCIIFrameBuffer[pxASCIIFrameHdl->usRcvBufferPos++] |= ubBinValue;
                    pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
                    break;
                }
            }
            break;

        case STATE_RX_WAIT_EOF:
            if( MBS_ASCII_DEFAULT_LF == ubValue )
            {
                bEnableTimer = FALSE;
                eStatus = eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, NULL );
                MBP_ASSERT( MB_ENOERR == eStatus );
                eStatus = eMBPTimerStop( pxASCIIFrameHdl->xTmrHdl );
                MBP_ASSERT( MB_ENOERR == eStatus );

#if MBS_ASCII_BACKOF_TIME_MS > 0
                eStatus = eMBPTimerStart( pxASCIIFrameHdl->xBackOffTmrHdl );
                MBP_ASSERT( MB_ENOERR == eStatus );
#else
                eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_RECEIVED );
                MBP_ASSERT( MB_ENOERR == eStatus );
#endif
            }
            else if( MBS_ASCII_START == ubValue )
            {
                HDL_RESET_RX( pxASCIIFrameHdl );
                pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
            }
            else
            {
                pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
            }
            break;


        case STATE_RX_IDLE:
            if( MBS_ASCII_START == ubValue )
            {
                pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
            }
            break;

        case STATE_RX_ERROR:
        default:
            if( MBS_ASCII_START == ubValue )
            {
                HDL_RESET_RX( pxASCIIFrameHdl );
                pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
            }
            break;
        }
    }
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
    pxIntHdl->xFrameStat.ulNBytesReceived += usBufferLen;
#endif
    if( bEnableTimer )
    {
        if( MB_ENOERR != eMBPTimerStart( pxASCIIFrameHdl->xTmrHdl ) )
        {
            /* We can only abort here because or timers failed. */
            pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
            eStatus = eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, NULL );
            MBP_ASSERT( MB_ENOERR == eStatus );
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}

#elif MBS_SERIAL_API_VERSION == 1
#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          BOOL
bMBSSerialTransmitterEmptyAPIV1CB( xMBHandle xHdl, UBYTE * pubValue )
    MB_CDECL_SUFFIX
{
    BOOL            bMoreTxData = FALSE;
    BOOL            bEnableRx = FALSE;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxASCIIFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( NULL != pxASCIIFrameHdl );
    MBP_ASSERT( pxASCIIFrameHdl->eRcvState == STATE_RX_IDLE );

    switch ( pxASCIIFrameHdl->eSndState )
    {
    case STATE_TX_START:
        *pubValue = MBS_ASCII_START;
        pxASCIIFrameHdl->eSndState = STATE_TX_DATA;
        pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
        bMoreTxData = TRUE;
        break;

    case STATE_TX_DATA:
        if( pxASCIIFrameHdl->usSndBufferCnt > 0 )
        {
            MBP_ASSERT( NULL != pxASCIIFrameHdl->pubSndBufferCur );
            switch ( pxASCIIFrameHdl->eBytePos )
            {
            case BYTE_HIGH_NIBBLE:
                *pubValue = ubMBSSerialASCIIBIN2CHAR( ( UBYTE ) ( *( pxASCIIFrameHdl->pubSndBufferCur ) >> 4 ) );
                pxASCIIFrameHdl->eBytePos = BYTE_LOW_NIBBLE;
                break;

            case BYTE_LOW_NIBBLE:
                *pubValue = ubMBSSerialASCIIBIN2CHAR( ( UBYTE ) ( *( pxASCIIFrameHdl->pubSndBufferCur ) & 0x0F ) );
                pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
                pxASCIIFrameHdl->pubSndBufferCur++;
                pxASCIIFrameHdl->usSndBufferCnt--;
                break;
            }
            bMoreTxData = TRUE;
        }
        else
        {
            *pubValue = MBS_ASCII_DEFAULT_CR;
            bMoreTxData = TRUE;
            pxASCIIFrameHdl->eSndState = STATE_TX_END;
        }
        break;

    case STATE_TX_END:
        *pubValue = MBS_ASCII_DEFAULT_LF;
        bMoreTxData = TRUE;
        pxASCIIFrameHdl->eSndState = STATE_TX_IDLE;
        break;

    case STATE_TX_IDLE:
        bEnableRx = TRUE;
        break;
    }
    if( !bMoreTxData )
    {
        HDL_RESET_TX( pxASCIIFrameHdl );
    }
    else
    {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
        pxIntHdl->xFrameStat.ulNBytesSent += 1;
#endif
    }
    if( bEnableRx )
    {
#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
        if( MB_ENOERR != eMBPTimerStart( pxASCIIFrameHdl->xWaitTmrHdl ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#else
        if( MB_ENOERR !=
            eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#endif
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return bMoreTxData;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC void
vMBPSerialReceiverAPIV1CB( xMBHandle xHdl, UBYTE ubValue )
    MB_CDECL_SUFFIX
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl;
    eMBErrorCode    eStatus;
    UBYTE           ubBinValue;
    BOOL            bEnableTimer = TRUE;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxASCIIFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( NULL != pxASCIIFrameHdl );
    MBP_ASSERT( pxASCIIFrameHdl->eSndState == STATE_TX_IDLE );

    switch ( pxASCIIFrameHdl->eRcvState )
    {
    case STATE_RX_RCV:
        if( MBS_ASCII_START == ubValue )
        {
            /* Empty receive buffer. */
            pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
            pxASCIIFrameHdl->usRcvBufferPos = 0;
        }
        else if( MBS_ASCII_DEFAULT_CR == ubValue )
        {
            pxASCIIFrameHdl->eRcvState = STATE_RX_WAIT_EOF;
        }
        else
        {
            ubBinValue = ubMBSSerialASCIICHAR2BIN( ubValue );
            switch ( pxASCIIFrameHdl->eBytePos )
            {
            case BYTE_HIGH_NIBBLE:
                /* High nibble comes first. We need to check for an overflow. */
                if( pxASCIIFrameHdl->usRcvBufferPos < MBS_SER_PDU_SIZE_MAX )
                {
                    pxASCIIFrameHdl->ubASCIIFrameBuffer[pxASCIIFrameHdl->usRcvBufferPos] =
                        ( UBYTE ) ( ubBinValue << 4 );
                    pxASCIIFrameHdl->eBytePos = BYTE_LOW_NIBBLE;
                }
                else
                {
                    pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
                }
                break;

            case BYTE_LOW_NIBBLE:
                pxASCIIFrameHdl->ubASCIIFrameBuffer[pxASCIIFrameHdl->usRcvBufferPos++] |= ubBinValue;
                pxASCIIFrameHdl->eBytePos = BYTE_HIGH_NIBBLE;
                break;
            }
        }
        break;

    case STATE_RX_WAIT_EOF:
        if( MBS_ASCII_DEFAULT_LF == ubValue )
        {
            bEnableTimer = FALSE;
            eStatus = eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, NULL );
            MBP_ASSERT( MB_ENOERR == eStatus );
            eStatus = eMBPTimerStop( pxASCIIFrameHdl->xTmrHdl );
            MBP_ASSERT( MB_ENOERR == eStatus );

#if MBS_ASCII_BACKOF_TIME_MS > 0
            eStatus = eMBPTimerStart( pxASCIIFrameHdl->xBackOffTmrHdl );
            MBP_ASSERT( MB_ENOERR == eStatus );
#else
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_RECEIVED );
            MBP_ASSERT( MB_ENOERR == eStatus );
#endif
        }
        else if( MBS_ASCII_START == ubValue )
        {
            HDL_RESET_RX( pxASCIIFrameHdl );
            pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
        }
        else
        {
            pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
        }
        break;


    case STATE_RX_IDLE:
        if( MBS_ASCII_START == ubValue )
        {
            pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
        }
        break;

    case STATE_RX_ERROR:
    default:
        if( MBS_ASCII_START == ubValue )
        {
            HDL_RESET_RX( pxASCIIFrameHdl );
            pxASCIIFrameHdl->eRcvState = STATE_RX_RCV;
        }
        break;
    }
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
    pxIntHdl->xFrameStat.ulNBytesReceived += 1;
#endif
    if( bEnableTimer )
    {
        if( MB_ENOERR != eMBPTimerStart( pxASCIIFrameHdl->xTmrHdl ) )
        {
            /* We can only abort here because or timers failed. */
            pxASCIIFrameHdl->eRcvState = STATE_RX_ERROR;
            eStatus = eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, NULL );
            MBP_ASSERT( MB_ENOERR == eStatus );
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}
#endif

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    BOOL
bMBSSerialASCIITimerCB( xMBHandle xHdl )
    MB_CDECL_SUFFIX
{
    BOOL            bNeedCtxSwitch = FALSE;

    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl = pxIntHdl->xFrameHdl;

    MBP_ASSERT( NULL != pxASCIIFrameHdl );
    MBP_ASSERT( pxASCIIFrameHdl->eSndState == STATE_TX_IDLE );
    HDL_RESET_RX( pxASCIIFrameHdl );

    return bNeedCtxSwitch;
}

#if MBS_ASCII_BACKOF_TIME_MS > 0
#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    BOOL
bMBSSerialASCIIBackOffTimerCB( xMBHandle xHdl )
    MB_CDECL_SUFFIX
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    eMBErrorCode    eStatus;

    eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_RECEIVED );
    MBP_ASSERT( MB_ENOERR == eStatus );
    return TRUE;
}
#endif

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          eMBErrorCode
eMBSSerialASCIIFrameClose( xMBHandle xHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;

    if( bMBSIsHdlValid( pxIntHdl ) )
    {
        eStatus = eMBSSerialASCIIFrameCloseInternal( ( xMBSASCIIFrameHandle * ) pxIntHdl->xFrameHdl );
    }
    return eStatus;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          eMBErrorCode
eMBSSerialASCIIFrameCloseInternal( xMBSASCIIFrameHandle * pxASCIIHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxASCIIHdl, xMBSASCIIFrameHdl ) )
    {

        if( MBP_SERIALHDL_INVALID != pxASCIIHdl->xSerHdl )
        {
            if( MB_ENOERR != eMBPSerialClose( pxASCIIHdl->xSerHdl ) )
            {
                eStatus = MB_EPORTERR;
            }
            else
            {
                if( MBP_TIMERHDL_INVALID != pxASCIIHdl->xTmrHdl )
                {
                    vMBPTimerClose( pxASCIIHdl->xTmrHdl );
                }
#if MBS_ASCII_BACKOF_TIME_MS > 0
                if( MBP_TIMERHDL_INVALID != pxASCIIHdl->xBackOffTmrHdl )
                {

                    vMBPTimerClose( pxASCIIHdl->xBackOffTmrHdl );
                }
#endif
                HDL_RESET( pxASCIIHdl );
                eStatus = MB_ENOERR;
            }
        }
        else
        {
            /* Make sure that no timers are created. */
            MBP_ASSERT( MBP_TIMERHDL_INVALID == pxASCIIHdl->xTmrHdl );
#if MBS_ASCII_BACKOF_TIME_MS > 0
            MBP_ASSERT( MBP_TIMERHDL_INVALID == pxASCIIHdl->xBackOffTmrHdl );
#endif
            HDL_RESET( pxASCIIHdl );
            eStatus = MB_ENOERR;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    UBYTE
ubMBSSerialASCIICHAR2BIN( UBYTE ubCharacter )
{
    if( ( ubCharacter >= 0x30 /* ASCII '0' */  ) && ( ubCharacter <= 0x39 /* ASCII '9' */  ) )
    {
        return ( UBYTE ) ( ubCharacter - 0x30 /* ASCII '0' */  );
    }
    else if( ( ubCharacter >= 0x41 /* ASCII 'A' */  ) && ( ubCharacter <= 0x46 /* ASCII 'F' */  ) )
    {
        return ( UBYTE ) ( ( ubCharacter - 0x41 ) /* ASCII 'A' */  + 0x0A );
    }
    else
    {
        return 0xFF;
    }
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    UBYTE
ubMBSSerialASCIIBIN2CHAR( UBYTE ubByte )
{
    if( ubByte <= 0x09 )
    {
        return ( UBYTE ) ( 0x30 + ubByte );
    }
    else if( ( ubByte >= 0x0A ) && ( ubByte <= 0x0F ) )
    {
        return ( UBYTE ) ( ( ubByte - 0x0A ) + 0x41 /* ASCII 'A' */  );
    }
    MBP_ASSERT( 0 );
    /*lint -e(527) */ return 0xFF;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    UBYTE
ubMBSSerialASCIILRC( const UBYTE * pubFrame, USHORT usLen )
{
    UBYTE           ubLRC = 0;  /* LRC char initialized */

    while( usLen-- > 0 )
    {
        ubLRC += *pubFrame++;   /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ubLRC = ( UBYTE ) ( -( ( UBYTE ) ubLRC ) );
    return ubLRC;
}

#if MBS_ASCII_WAITAFTERSEND_ENABLED > 0
#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    BOOL
bMBSSerialWaitCB( xMBHandle xHdl )
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSASCIIFrameHandle *pxASCIIFrameHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxASCIIFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxASCIIFrameHdl->eSndState == STATE_TX_IDLE );
#if MBS_SERIAL_API_VERSION == 2
    if( MB_ENOERR !=
        eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
#elif MBS_SERIAL_API_VERSION == 1
    if( MB_ENOERR !=
        eMBPSerialRxEnable( pxASCIIFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
#endif
    {
        ( void )eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return FALSE;
}
#endif

#endif
