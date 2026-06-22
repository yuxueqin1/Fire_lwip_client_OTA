/*
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007-2010 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsrtu.c,v 1.24 2010/12/08 17:49:04 embedded-so.embedded-solutions.1 Exp $
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
#include "mbsrtu.h"
#include "mbscrc.h"

#if MBS_RTU_ENABLED == 1

/* ----------------------- Defines ------------------------------------------*/
#define MBS_SER_PDU_SIZE_MIN            ( 4 )   /*!< Minimum size of a MODBUS RTU frame. */
#if !defined( MBS_SER_PDU_SIZE_MAX )
#define MBS_SER_PDU_SIZE_MAX            ( 256 ) /*!< Maximum size of a MODBUS RTU frame. */
#endif
#define MBS_SER_PDU_SIZE_CRC            ( 2 )   /*!< Size of CRC field in PDU. */
#define MBS_SER_PDU_ADDR_OFF            ( 0 )   /*!< Offset of slave address in RTU frame. */
#define MBS_SER_PDU_PDU_OFF             ( 1 )   /*!< Offset of Modbus-PDU in RTU frame. */

#define IDX_INVALID                     ( 255 )

#define HDL_RESET_RX( x ) do { \
    ( x )->eRcvState = STATE_RX_IDLE; \
    ( x )->usRcvBufferPos = 0; \
} while( 0 )

#define HDL_RESET_TX( x ) do { \
    ( x )->eSndState = STATE_TX_IDLE; \
    ( x )->usSndBufferCnt = 0; \
    ( x )->pubSndBufferCur = NULL; \
} while( 0 )

#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
#define HDL_RESET_WAIT( x ) do { \
    ( x )->xTmrWaitHdl = MBP_TIMERHDL_INVALID; \
} while( 0 )
#else
#define HDL_RESET_WAIT( x )
#endif

#define HDL_RESET( x ) do { \
    ( x )->ubIdx = IDX_INVALID; \
    HDL_RESET_RX( x ); \
    HDL_RESET_TX( x ); \
    ( x )->xTmrHdl = MBP_TIMERHDL_INVALID; \
    ( x )->xSerHdl = MBP_SERIALHDL_INVALID; \
    HDL_RESET_WAIT( x ); \
    MBP_MEMSET( ( void * )&( ( x )->ubRTUFrameBuffer[ 0 ] ), 0, MBS_SER_PDU_SIZE_MAX ); \
} while( 0 )

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< Receiver error condition. */
} eMBSRTURcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is sending data. */
} eMBSRTUSndState;

typedef struct
{
    UBYTE           ubIdx;
    volatile UBYTE  ubRTUFrameBuffer[MBS_SER_PDU_SIZE_MAX];

    volatile eMBSRTURcvState eRcvState;
    volatile USHORT usRcvBufferPos;

    volatile eMBSRTUSndState eSndState;
    volatile USHORT usSndBufferCnt;
    UBYTE          *pubSndBufferCur;

    xMBPTimerHandle xTmrHdl;
    xMBPSerialHandle xSerHdl;
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
    xMBPTimerHandle xTmrWaitHdl;
#endif
} xMBSRTUFrameHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xMBSRTUFrameHandle xMBSRTUFrameHdl[MBS_SERIAL_RTU_MAX_INSTANCES];

/* ----------------------- Static functions ---------------------------------*/
#if MBS_TEST_INSTANCES == 0
STATIC
#endif
    BOOL bMBSSerialRTUT35CB( xMBHandle xHdl );

#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
STATIC BOOL     bMBSSerialWaitCB( xMBHandle xHdl );
#endif

STATIC eMBErrorCode eMBSSerialRTUFrameSend( xMBHandle xHdl, USHORT usMBPDULength );
STATIC          eMBErrorCode
eMBSSerialRTUFrameReceive( xMBHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength )
    MB_CDECL_SUFFIX;
     STATIC eMBErrorCode eMBSSerialRTUFrameClose( xMBHandle xHdl );
     STATIC eMBErrorCode eMBSSerialRTUFrameCloseInternal( xMBSRTUFrameHandle * pxRTUHdl );

#if MBS_SERIAL_API_VERSION == 2
     STATIC BOOL     bMBSSerialTransmitterEmptyAPIV2CB( xMBHandle xHdl, UBYTE * pubBufferOut, USHORT usBufferMax,
                                                        USHORT * pusBufferWritten ) MB_CDECL_SUFFIX;
     STATIC void     vMBPSerialReceiverAPIV2CB( xMBHandle xHdl, const UBYTE * pubBufferIn,
                                                USHORT usBufferLen ) MB_CDECL_SUFFIX;
#elif MBS_SERIAL_API_VERSION == 1
     STATIC BOOL     bMBSSerialTransmitterEmptyAPIV1CB( xMBHandle xHdl, UBYTE * pubValue ) MB_CDECL_SUFFIX;
     STATIC void     vMBPSerialReceiverAPIV1CB( xMBHandle xHdl, UBYTE ubValue );
#else
#error "unsupported serial API version set in configuration."
#endif

/* ----------------------- Start implementation -----------------------------*/

#if defined( MBP_HITECH_LINKERWORKAROUND_ENABLED ) && ( MBP_HITECH_LINKERWORKAROUND_ENABLED == 1 )
void 
vMBSSerialLinkerWorkaround( void )
{
    volatile BOOL   bLinkerWorkaround = FALSE;
    if( bLinkerWorkaround )
    {
        /* Never executed but otherwise linker strips functions */
        ( void )bMBSSerialRTUT35CB( NULL );
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
        ( void )bMBSSerialWaitCB( NULL );
#endif
#if MBS_SERIAL_API_VERSION == 2
#elif MBS_SERIAL_API_VERSION == 1
        ( void )bMBSSerialTransmitterEmptyAPIV1CB( NULL, NULL );
        ( void )vMBPSerialReceiverAPIV1CB( NULL, 0 );
#endif
    }
}
#endif

eMBErrorCode
eMBSSerialRTUInit( xMBSInternalHandle * pxIntHdl, UBYTE ubPort, ULONG ulBaudRate, eMBSerialParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR, eStatus2;
    xMBSRTUFrameHandle *pxFrameHdl = NULL;
    UBYTE           ubIdx;
    USHORT          usTimeoutMS;
    UCHAR           ucStopBits;

#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
    USHORT          usTimeoutMSWaitAfterSend;
#endif

#if defined( MBP_HITECH_LINKERWORKAROUND_ENABLED ) && ( MBP_HITECH_LINKERWORKAROUND_ENABLED == 1 )
    vMBSSerialLinkerWorkaround(  );
#endif

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( ( NULL != pxIntHdl ) && ( ulBaudRate > 0 ) )
#else
    if( TRUE )
#endif
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        if( !bIsInitialized )
        {
            for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBSRTUFrameHdl ); ubIdx++ )
            {
                HDL_RESET( &xMBSRTUFrameHdl[ubIdx] );
            }
            bIsInitialized = TRUE;
        }

        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( xMBSRTUFrameHdl ); ubIdx++ )
        {
            if( IDX_INVALID == xMBSRTUFrameHdl[ubIdx].ubIdx )
            {
                pxFrameHdl = &xMBSRTUFrameHdl[ubIdx];
                pxFrameHdl->ubIdx = ubIdx;
                break;
            }
        }

        if( NULL != pxFrameHdl )
        {
#if MBS_SERIAL_API_VERSION == 2
            usTimeoutMS = MBS_SERIAL_APIV2_RTU_DYNAMIC_TIMEOUT_MS( ulBaudRate );
#else
            /* If baudrate > 19200 then we should use the fixed timer value 1750us.
             * We can't match this exactly so we use 2000us. Otherwise use 3.5 times
             * the character timeout. */
            if( ulBaudRate > 19200 )
            {
                usTimeoutMS = 2;
            }
            else
            {
                /* The number of ticks required for a character is given by
                 * xTicksCh = TIMER_TICKS_PER_SECOND * 11 / BAUDRATE
                 * The total timeout is given by xTicksCh * 3.5 = xTicksCh * 7/2.
                 */
                usTimeoutMS = ( USHORT ) ( ( 1000UL * 11UL * 7UL ) / ( 2 * ulBaudRate ) );
            }
#endif
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
            usTimeoutMSWaitAfterSend = MBS_SERIAL_RTU_DYNAMIC_WAITAFTERSEND_TIMEOUT_MS( ulBaudRate );
#endif

            /* No parity requires two stop bits. */
            ucStopBits = MB_PAR_NONE == eParity ? ( UCHAR ) 2 : ( UCHAR ) 1;

            if( MB_ENOERR !=
                ( eStatus2 =
                  eMBPSerialInit( &( pxFrameHdl->xSerHdl ), ( UCHAR ) ubPort, ulBaudRate, 8, eParity, ucStopBits,
                                  pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
            else if( MB_ENOERR != ( eStatus2 = eMBPTimerInit( &( pxFrameHdl->xTmrHdl ), usTimeoutMS,
                                                              bMBSSerialRTUT35CB, pxIntHdl ) ) )
            {
                eStatus = eStatus2;
            }
            else
            {
                /* Attach the frame handle to the protocol stack. */
                pxIntHdl->pubFrameMBPDUBuffer = ( UBYTE * ) & pxFrameHdl->ubRTUFrameBuffer[MBS_SER_PDU_PDU_OFF];
                pxIntHdl->xFrameHdl = pxFrameHdl;
                pxIntHdl->pFrameSendFN = eMBSSerialRTUFrameSend;
                pxIntHdl->pFrameRecvFN = eMBSSerialRTUFrameReceive;
                pxIntHdl->pFrameCloseFN = eMBSSerialRTUFrameClose;
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
#endif
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
                else if( MB_ENOERR != eMBPTimerInit( &( pxFrameHdl->xTmrWaitHdl ), usTimeoutMSWaitAfterSend,
                                                     bMBSSerialWaitCB, pxIntHdl ) )
                {
                    eStatus = MB_EPORTERR;
                }
#endif
            }

            if( MB_ENOERR != eStatus )
            {
                ( void )eMBSSerialRTUFrameCloseInternal( pxFrameHdl );
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
eMBSSerialRTUFrameSend( xMBHandle xHdl, USHORT usMBPDULength )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    USHORT          usCRC16;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUHdl;

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
        pxRTUHdl = pxIntHdl->xFrameHdl;

        if( MB_IS_VALID_HDL( pxRTUHdl, xMBSRTUFrameHdl ) )
        {
            if( 0 == usMBPDULength )
            {
#if MBS_SERIAL_API_VERSION == 2
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxRTUHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#elif MBS_SERIAL_API_VERSION == 1
                if( MB_ENOERR !=
                    eMBPSerialRxEnable( pxRTUHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
                {
                    eStatus = MB_EPORTERR;
                }
#endif
                else
                {
                    eStatus = MB_ENOERR;
                }
            }
            else if( usMBPDULength <= ( MBS_SER_PDU_SIZE_MAX - ( 1 /* Slave Address */  + 2 /* CRC16 */  ) ) )
            {
                MBP_ASSERT( STATE_TX_IDLE == pxRTUHdl->eSndState );

                /* Added the MODBUS RTU header (= slave address) */
                pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF] = pxIntHdl->ubSlaveAddress;
                pxRTUHdl->usSndBufferCnt = 1;

                /* MODBUS PDU is already embedded in the frame. */
                pxRTUHdl->usSndBufferCnt += usMBPDULength;

                usCRC16 = usMBSCRC16( ( const UBYTE * )&pxRTUHdl->ubRTUFrameBuffer[0], pxRTUHdl->usSndBufferCnt );
                pxRTUHdl->ubRTUFrameBuffer[pxRTUHdl->usSndBufferCnt] = ( UBYTE ) ( usCRC16 & 0xFF );
                pxRTUHdl->usSndBufferCnt++;
                pxRTUHdl->ubRTUFrameBuffer[pxRTUHdl->usSndBufferCnt] = ( UBYTE ) ( usCRC16 >> 8 );
                pxRTUHdl->usSndBufferCnt++;

                /* Enable transmitter */
                pxRTUHdl->eSndState = STATE_TX_XMIT;
                pxRTUHdl->pubSndBufferCur = ( UBYTE * ) & pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF];
#if MBS_SERIAL_API_VERSION == 2
                if( MB_ENOERR !=
                    eMBPSerialTxEnable( pxRTUHdl->xSerHdl,
                                        ( pbMBPSerialTransmitterEmptyCB ) bMBSSerialTransmitterEmptyAPIV2CB ) )
                {
                    eStatus = MB_EPORTERR;
                    HDL_RESET_TX( pxRTUHdl );
                }
#elif MBS_SERIAL_API_VERSION == 1
                if( MB_ENOERR !=
                    eMBPSerialTxEnable( pxRTUHdl->xSerHdl,
                                        ( pbMBPSerialTransmitterEmptyCB ) bMBSSerialTransmitterEmptyAPIV1CB ) )
                {
                    eStatus = MB_EPORTERR;
                    HDL_RESET_TX( pxRTUHdl );
                }
#endif
                else
                {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                    pxIntHdl->xFrameStat.ulNPacketsSent += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                    /* Pass frame to protocol analyzer. */
                    xAnalyzerFrame.eFrameDir = MB_FRAME_SEND;
                    xAnalyzerFrame.eFrameType = MB_FRAME_RTU;
                    xAnalyzerFrame.x.xRTUHeader.ubSlaveAddress = pxIntHdl->ubSlaveAddress;
                    xAnalyzerFrame.x.xRTUHeader.usCRC16 =
                        ( USHORT ) ( ( usCRC16 & 0xFF ) << 8U ) + ( USHORT ) ( usCRC16 >> 8U );
                    xAnalyzerFrame.ubDataPayload = ( const UBYTE * )&pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF];
                    xAnalyzerFrame.usDataPayloadLength = pxRTUHdl->usSndBufferCnt;
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
eMBSSerialRTUFrameReceive( xMBHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength )
    MB_CDECL_SUFFIX
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUHdl;

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
        pxRTUHdl = pxIntHdl->xFrameHdl;
        if( MB_IS_VALID_HDL( pxRTUHdl, xMBSRTUFrameHdl ) )
        {
            if( ( pxRTUHdl->usRcvBufferPos >= MBS_SER_PDU_SIZE_MIN ) &&
                ( usMBSCRC16( ( const UBYTE * )&( pxRTUHdl->ubRTUFrameBuffer[0] ), pxRTUHdl->usRcvBufferPos ) == 0 ) )
            {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                pxIntHdl->xFrameStat.ulNPacketsReceived += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                xAnalyzerFrame.eFrameType = MB_FRAME_RTU;
                xAnalyzerFrame.x.xRTUHeader.ubSlaveAddress = pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF];;
                xAnalyzerFrame.x.xRTUHeader.usCRC16 = pxRTUHdl->ubRTUFrameBuffer[pxRTUHdl->usRcvBufferPos - 1];
                xAnalyzerFrame.x.xRTUHeader.usCRC16 |=
                    ( USHORT ) ( pxRTUHdl->ubRTUFrameBuffer[pxRTUHdl->usRcvBufferPos - 2] << 8U );
#endif
                *pubSlaveAddress = pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF];
                *pusMBPDULength =
                    ( USHORT ) ( pxRTUHdl->usRcvBufferPos - ( MBS_SER_PDU_PDU_OFF + MBS_SER_PDU_SIZE_CRC ) );

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
            xAnalyzerFrame.ubDataPayload = ( const UBYTE * )&pxRTUHdl->ubRTUFrameBuffer[MBS_SER_PDU_ADDR_OFF];
            xAnalyzerFrame.usDataPayloadLength = pxRTUHdl->usRcvBufferPos;
            if( NULL != pxIntHdl->pvMBAnalyzerCallbackFN )
            {
                vMBPGetTimeStamp( &xTimeStamp );
                pxIntHdl->pvMBAnalyzerCallbackFN( pxIntHdl, pxIntHdl->pvCtx, &xTimeStamp, &xAnalyzerFrame );
            }
#endif
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
    USHORT          usBytesToSend;
    BOOL            bMoreTxData = FALSE;
    BOOL            bEnableRx = FALSE;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eRcvState == STATE_RX_IDLE );
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_XMIT );

    switch ( pxRTUFrameHdl->eSndState )
    {
    case STATE_TX_XMIT:
        if( pxRTUFrameHdl->usSndBufferCnt > 0 )
        {
            usBytesToSend = pxRTUFrameHdl->usSndBufferCnt < usBufferMax ? pxRTUFrameHdl->usSndBufferCnt : usBufferMax;
            memcpy( pubBufferOut, pxRTUFrameHdl->pubSndBufferCur, ( size_t ) usBytesToSend );
            pxRTUFrameHdl->pubSndBufferCur += usBytesToSend;
            pxRTUFrameHdl->usSndBufferCnt -= usBytesToSend;
            *pusBufferWritten = usBytesToSend;
            bMoreTxData = TRUE;
        }
        else
        {
            bEnableRx = TRUE;
        }
        break;

    default:
        break;
    }
    if( !bMoreTxData )
    {
        HDL_RESET_TX( pxRTUFrameHdl );
    }
    else
    {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
        pxIntHdl->xFrameStat.ulNBytesSent += *pusBufferWritten;
#endif
    }
    if( bEnableRx )
    {
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
        if( MB_ENOERR != eMBPTimerStart( pxRTUFrameHdl->xTmrWaitHdl ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#else
        if( MB_ENOERR !=
            eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
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
    xMBSRTUFrameHandle *pxRTUFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_IDLE );
    switch ( pxRTUFrameHdl->eRcvState )
    {
    case STATE_RX_IDLE:
        if( usBufferLen < MBS_SER_PDU_SIZE_MAX )
        {
            memcpy( ( void * )&( pxRTUFrameHdl->ubRTUFrameBuffer[0] ), pubBufferIn, ( size_t ) usBufferLen );
            pxRTUFrameHdl->usRcvBufferPos = usBufferLen;
            pxRTUFrameHdl->eRcvState = STATE_RX_RCV;
        }
        break;

    case STATE_RX_RCV:
        if( ( pxRTUFrameHdl->usRcvBufferPos + usBufferLen ) < MBS_SER_PDU_SIZE_MAX )
        {
            memcpy( ( void * )&( pxRTUFrameHdl->ubRTUFrameBuffer[pxRTUFrameHdl->usRcvBufferPos] ), pubBufferIn,
                    ( size_t ) usBufferLen );
            pxRTUFrameHdl->usRcvBufferPos += usBufferLen;
        }
        else
        {
            pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
        }
        break;

    default:
    case STATE_RX_ERROR:
        pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
    }
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
    pxIntHdl->xFrameStat.ulNBytesReceived += usBufferLen;
#endif

    if( MB_ENOERR != eMBPTimerStart( pxRTUFrameHdl->xTmrHdl ) )
    {
        pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
        /* We can only abort here because or timers failed. */
        eStatus = eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, NULL );
        MBP_ASSERT( MB_ENOERR == eStatus );
        eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
        MBP_ASSERT( MB_ENOERR == eStatus );
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
    xMBSRTUFrameHandle *pxRTUFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eRcvState == STATE_RX_IDLE );
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_XMIT );

    switch ( pxRTUFrameHdl->eSndState )
    {
    case STATE_TX_XMIT:
        if( pxRTUFrameHdl->usSndBufferCnt > 0 )
        {
            *pubValue = *pxRTUFrameHdl->pubSndBufferCur++;
            pxRTUFrameHdl->usSndBufferCnt -= ( USHORT ) 1;
            bMoreTxData = TRUE;
        }
        else
        {
            bEnableRx = TRUE;
        }
        break;

    default:
        break;
    }
    if( !bMoreTxData )
    {
        HDL_RESET_TX( pxRTUFrameHdl );
    }
    else
    {
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
        pxIntHdl->xFrameStat.ulNBytesSent += 1;
#endif
    }
    if( bEnableRx )
    {
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
        if( MB_ENOERR != eMBPTimerStart( pxRTUFrameHdl->xTmrWaitHdl ) )
        {
            eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
#else
        if( MB_ENOERR !=
            eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
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
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUFrameHdl;
    eMBErrorCode    eStatus;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_IDLE );
    switch ( pxRTUFrameHdl->eRcvState )
    {
    case STATE_RX_IDLE:
        pxRTUFrameHdl->ubRTUFrameBuffer[0] = ubValue;
        pxRTUFrameHdl->usRcvBufferPos = 1;
        pxRTUFrameHdl->eRcvState = STATE_RX_RCV;
        break;

    case STATE_RX_RCV:
        if( pxRTUFrameHdl->usRcvBufferPos < MBS_SER_PDU_SIZE_MAX )
        {
            pxRTUFrameHdl->ubRTUFrameBuffer[pxRTUFrameHdl->usRcvBufferPos] = ubValue;
            pxRTUFrameHdl->usRcvBufferPos++;
        }
        else
        {
            pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
        }
        break;

    default:
    case STATE_RX_ERROR:
        pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
    }
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
    pxIntHdl->xFrameStat.ulNBytesReceived += 1;
#endif
    if( MB_ENOERR != eMBPTimerStart( pxRTUFrameHdl->xTmrHdl ) )
    {
        /* We can only abort here because or timers failed. */
        pxRTUFrameHdl->eRcvState = STATE_RX_ERROR;
        eStatus = eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, NULL );
        MBP_ASSERT( MB_ENOERR == eStatus );
        eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
        MBP_ASSERT( MB_ENOERR == eStatus );
    }
    MBP_EXIT_CRITICAL_SECTION(  );
}
#endif

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          eMBErrorCode
eMBSSerialRTUFrameClose( xMBHandle xHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        eStatus = eMBSSerialRTUFrameCloseInternal( ( xMBSRTUFrameHandle * ) pxIntHdl->xFrameHdl );
    }
    return eStatus;
}

#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          eMBErrorCode
eMBSSerialRTUFrameCloseInternal( xMBSRTUFrameHandle * pxRTUHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;

    MBP_ENTER_CRITICAL_SECTION(  );
    if( MB_IS_VALID_HDL( pxRTUHdl, xMBSRTUFrameHdl ) )
    {

        if( MBP_SERIALHDL_INVALID != pxRTUHdl->xSerHdl )
        {
            if( MB_ENOERR != eMBPSerialClose( pxRTUHdl->xSerHdl ) )
            {
                eStatus = MB_EPORTERR;
            }
            else
            {
                if( MBP_TIMERHDL_INVALID != pxRTUHdl->xTmrHdl )
                {
                    vMBPTimerClose( pxRTUHdl->xTmrHdl );
                }
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
                if( MBP_TIMERHDL_INVALID != pxRTUHdl->xTmrWaitHdl )
                {
                    vMBPTimerClose( pxRTUHdl->xTmrWaitHdl );
                }
#endif
                HDL_RESET( pxRTUHdl );
                eStatus = MB_ENOERR;
            }
        }
        else
        {
            /* Make sure that no timers are created. */
            MBP_ASSERT( MBP_TIMERHDL_INVALID == pxRTUHdl->xTmrHdl );
#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
            MBP_ASSERT( MBP_TIMERHDL_INVALID == pxRTUHdl->xTmrWaitHdl );
#endif
            HDL_RESET( pxRTUHdl );
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
    BOOL
bMBSSerialRTUT35CB( xMBHandle xHdl )
{
    BOOL            bNeedCtxSwitch = TRUE;
    eMBErrorCode    eStatus;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUFrameHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_IDLE );


    switch ( pxRTUFrameHdl->eRcvState )
    {
    case STATE_RX_RCV:
        /* Disable receiver since we want to process this frame first. */
        eStatus = eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, NULL );
        MBP_ASSERT( MB_ENOERR == eStatus );
        /* Put receiver back to idle - Information about new frame is in the
         * MBS_EV_RECEIVED event.
         */
        pxRTUFrameHdl->eRcvState = STATE_RX_IDLE;
        /* A frame has been received. Handle this one. */
        eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_RECEIVED );
        MBP_ASSERT( MB_ENOERR == eStatus );
        break;

    default:
        /* An error occurred during frame reception. Ignore this
         * frame.
         */
        pxRTUFrameHdl->eRcvState = STATE_RX_IDLE;
        break;
    }

    MBP_EXIT_CRITICAL_SECTION(  );
    return bNeedCtxSwitch;
}

#if MBS_RTU_WAITAFTERSEND_ENABLED == 1
#if defined( HI_TECH_C ) && defined( __PICC18__ )
#pragma interrupt_level 1
#endif
STATIC          BOOL
bMBSSerialWaitCB( xMBHandle xHdl )
{
    eMBErrorCode    eStatus;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSRTUFrameHandle *pxRTUFrameHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
    pxRTUFrameHdl = pxIntHdl->xFrameHdl;
    MBP_ASSERT( pxRTUFrameHdl->eSndState == STATE_TX_IDLE );
#if MBS_SERIAL_API_VERSION == 2
    if( MB_ENOERR != eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV2CB ) )
#elif MBS_SERIAL_API_VERSION == 1
    if( MB_ENOERR != eMBPSerialRxEnable( pxRTUFrameHdl->xSerHdl, ( pvMBPSerialReceiverCB ) vMBPSerialReceiverAPIV1CB ) )
#endif
    {
        eStatus = eMBPEventPost( pxIntHdl->xFrameEventHdl, ( xMBPEventType ) MBS_EV_ERROR );
        MBP_ASSERT( MB_ENOERR == eStatus );
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return FALSE;
}
#endif

#endif
