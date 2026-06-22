/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbstcp.c,v 1.17 2010/01/29 23:44:14 embedded-so.embedded-solutions.1 Exp $
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
#include <stdio.h>
#if MBS_TCP_ENABLED == 1

/* ----------------------- MBAP Header --------------------------------------*/
/*
 *
 * <------------------------ MODBUS TCP/IP ADU(1) ------------------------->
 *              <----------- MODBUS PDU (1') ---------------->
 *  +-----------+---------------+------------------------------------------+
 *  | TID | PID | Length | UID  |Code | Data                               |
 *  +-----------+---------------+------------------------------------------+
 *  |     |     |        |      |                                           
 * (2)   (3)   (4)      (5)    (6)                                          
 *
 * (2)  ... MBS_TCP_TID_OFF     = 0 (Transaction Identifier - 2 Byte) 
 * (3)  ... MBS_TCP_PID_OFF     = 2 (Protocol Identifier - 2 Byte)
 * (4)  ... MBS_TCP_LEN_OFF     = 4 (Number of bytes - 2 Byte)
 * (5)  ... MBS_TCP_UID_OFF     = 6 (Unit Identifier - 1 Byte)
 * (6)  ... MBS_TCP_PDU_OFF     = 7 (MODBUS PDU)
 *
 * (1)  ... MODBUS TCP/IP ADU (application data unit)
 * (1') ... MODBUS PDU (protocol data unit)
 */

#define MBS_TCP_TID_OFF             ( 0 )
#define MBS_TCP_PID_OFF             ( 2 )
#define MBS_TCP_LEN_OFF             ( 4 )
#define MBS_TCP_UID_OFF             ( 6 )
#define MBS_TCP_MBAP_HEADER_SIZE    ( 7 )
#define MBS_TCP_MB_PDU_OFF          ( 7 )

#define MBS_TCP_PROTOCOL_ID         ( 0 )       /* 0 = Modbus Protocol */

/* ----------------------- Defines ------------------------------------------*/
#define MBS_TCP_PDU_SIZE_MIN        ( 7 )
#define MBS_TCP_PDU_SIZE_MAX        ( 260 )

#define MBS_TCP_BUFFER_SIZE         ( MBS_TCP_PDU_SIZE_MAX )

#define IDX_INVALID                 ( 255 )

/* ----------------------- Type definitions ---------------------------------*/
typedef struct
{
/* *INDENT-OFF* */
    BOOL            bInUse;
    BOOL            bHasData;
    xMBPTCPClientHandle xMBTCPClientHdl;
    UBYTE           arubBuffer[MBS_TCP_BUFFER_SIZE];
    USHORT          usBufferPos;
/* *INDENT-ON* */
} xMBSTCPClientHandle;

typedef struct
{
/* *INDENT-OFF* */
    UBYTE           ubIdx;
    UBYTE           ubActiveClient;
    xMBPTCPHandle   xMBTCPServerHdl;
    xMBPEventHandle xMBTCPEventHdl;
    xMBSTCPClientHandle xClientHdl[MBS_TCP_MAX_CLIENTS];
/* *INDENT-ON* */
} xMBSTCPFrameHandle;

/* ----------------------- Static variables ---------------------------------*/
STATIC BOOL     bIsInitialized = FALSE;
STATIC xMBSTCPFrameHandle arxMBSTCPFrameHdl[MBS_TCP_MAX_INSTANCES];

/* ----------------------- Static functions ---------------------------------*/
STATIC void     vMBSTCPResetClientHdl( const xMBSTCPFrameHandle * pxFrameHdl, xMBSTCPClientHandle * pxTCPClientHdl,
                                       BOOL bClose );
STATIC void     vMBSTCPResetHdl( xMBSTCPFrameHandle * pxFrameHdl, BOOL bClose );
STATIC eMBErrorCode eMBSTCPFrameSend( xMBSHandle xHdl, USHORT usMBPDULength );
STATIC          eMBErrorCode
eMBSTCPFrameReceive( xMBSHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength ) MB_CDECL_SUFFIX;
STATIC eMBErrorCode eMBSTCPFrameClose( xMBSHandle xHdl );
STATIC eMBErrorCode eMBPTCPClientConnectedCB( xMBHandle xMBHdl, xMBPTCPClientHandle xTCPClientHdl );
STATIC eMBErrorCode eMBPTCPClientDisconnectedCB( xMBHandle xHdl, xMBPTCPClientHandle xTCPClientHdl );
STATIC eMBErrorCode eMBPTCPClientNewDataCB( xMBHandle xHdl, xMBPTCPClientHandle xClientHdl );
#if ( defined( MBS_ENABLE_DEBUG_FACILITY ) && ( MBS_ENABLE_DEBUG_FACILITY == 1 ) )
void            vMBSLogTCPFrame( eMBPortLogLevel eLevel, xMBSTCPFrameHandle * pxIntHdl, char *szMsg,
                                 const UBYTE * pubPayload, USHORT usLength );
#endif
/* ----------------------- Start implementation -----------------------------*/

eMBErrorCode
eMBSTCPIntInit( xMBSInternalHandle * pxIntHdl, CHAR * pcBindAddress, USHORT usTCPPort )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSTCPFrameHandle *pxFrameHdl = NULL;
    UBYTE           ubIdx;

#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        MBP_ENTER_CRITICAL_SECTION(  );
        if( !bIsInitialized )
        {
            for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( arxMBSTCPFrameHdl ); ubIdx++ )
            {
                /* We need to initialize the index because otherwise the valid
                 * handle check in vMBSTCPResetHdl will fail.
                 */
                arxMBSTCPFrameHdl[ubIdx].ubIdx = ubIdx;
                vMBSTCPResetHdl( &arxMBSTCPFrameHdl[ubIdx], FALSE );
            }
            bIsInitialized = TRUE;
        }

        for( ubIdx = 0; ubIdx < ( UBYTE ) MB_UTILS_NARRSIZE( arxMBSTCPFrameHdl ); ubIdx++ )
        {
            if( IDX_INVALID == arxMBSTCPFrameHdl[ubIdx].ubIdx )
            {
                pxFrameHdl = &arxMBSTCPFrameHdl[ubIdx];
                pxFrameHdl->ubIdx = ubIdx;
                break;
            }
        }
        MBP_EXIT_CRITICAL_SECTION(  );

        if( NULL != pxFrameHdl )
        {
            eStatus = MB_EPORTERR;
            if( MB_ENOERR != eMBPEventCreate( &( pxFrameHdl->xMBTCPEventHdl ) ) )
            {
                eStatus = MB_EPORTERR;
            }
            else if( MB_ENOERR ==
                     eMBPTCPServerInit( &( pxFrameHdl->xMBTCPServerHdl ), pcBindAddress, usTCPPort, pxIntHdl,
                                        eMBPTCPClientNewDataCB, eMBPTCPClientDisconnectedCB,
                                        eMBPTCPClientConnectedCB ) )
            {
                pxIntHdl->pubFrameMBPDUBuffer = NULL;
                pxIntHdl->pFrameSendFN = eMBSTCPFrameSend;
                pxIntHdl->pFrameRecvFN = eMBSTCPFrameReceive;
                pxIntHdl->pFrameCloseFN = eMBSTCPFrameClose;
                pxIntHdl->xFrameHdl = pxFrameHdl;
                eStatus = MB_ENOERR;
            }
            else
            {
                MBP_ASSERT( ubIdx < MBS_TCP_MAX_INSTANCES );
                vMBSTCPResetHdl( &arxMBSTCPFrameHdl[ubIdx], TRUE );
            }
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
    return eStatus;
}

STATIC          eMBErrorCode
eMBSTCPFrameSend( xMBSHandle xHdl, USHORT usMBPDULength )
{
    eMBErrorCode    eStatus = MB_EINVAL, eStatus2;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSTCPClientHandle *pxTCPClientHdl;
    xMBSTCPFrameHandle *pxFrameHdl;
    USHORT          usMBAPLengthField;
    USHORT          usLen;
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
        if( MB_IS_VALID_HDL( ( xMBSTCPFrameHandle * ) pxIntHdl->xFrameHdl, arxMBSTCPFrameHdl ) )
        {
            pxFrameHdl = pxIntHdl->xFrameHdl;
            MBP_ENTER_CRITICAL_SECTION(  );
            if( IDX_INVALID != pxFrameHdl->ubActiveClient )
            {
                pxTCPClientHdl = &( pxFrameHdl->xClientHdl[pxFrameHdl->ubActiveClient] );
                if( 0 == usMBPDULength )
                {
                    pxIntHdl->pubFrameMBPDUBuffer = NULL;
                    pxTCPClientHdl->usBufferPos = 0;
                    pxFrameHdl->ubActiveClient = IDX_INVALID;
                    eStatus = MB_ENOERR;
                }
                else
                {
                    MBP_ASSERT( pxTCPClientHdl->bInUse );
                    if( ( usMBPDULength >= MB_PDU_SIZE_MIN ) && ( usMBPDULength <= MB_PDU_SIZE_MAX ) )
                    {
                        usMBAPLengthField = ( USHORT ) ( usMBPDULength + 1 );
                        pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF] = ( UBYTE ) ( usMBAPLengthField >> 8U );
                        pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF + 1] = ( UBYTE ) ( usMBAPLengthField & 0xFF );
                        usLen = usMBPDULength + MBS_TCP_MBAP_HEADER_SIZE;
#if defined( MBS_ENABLE_DEBUG_FACILITY ) && ( MBS_ENABLE_DEBUG_FACILITY == 1 )
                        if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_TCP ) )
                        {
                            vMBSLogTCPFrame( MB_LOG_DEBUG, pxFrameHdl, "Sending frame: ",
                                             ( const UBYTE * )&pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF], usLen );
                        }
#endif
                        eStatus2 = eMBPTCPConWrite( pxFrameHdl->xMBTCPServerHdl, pxTCPClientHdl->xMBTCPClientHdl,
                                                    pxTCPClientHdl->arubBuffer, usLen );
                        switch ( eStatus2 )
                        {
                        case MB_ENOERR:
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                            /* Account for sent frame and bytes. */
                            pxIntHdl->xFrameStat.ulNBytesSent += usLen;
                            pxIntHdl->xFrameStat.ulNPacketsSent += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                            /* Pass frame to protocol analyzer. */
                            xAnalyzerFrame.eFrameDir = MB_FRAME_SEND;
                            xAnalyzerFrame.eFrameType = MB_FRAME_TCP;
                            /* Its better to take these values directly out of the request 
                             * in case this part changes later because otherwise bugs could
                             * be introduced easily.
                             */
                            xAnalyzerFrame.x.xTCPHeader.usMBAPTransactionId =
                                ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF] << 8U );
                            xAnalyzerFrame.x.xTCPHeader.usMBAPTransactionId |=
                                ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF + 1] );
                            xAnalyzerFrame.x.xTCPHeader.usMBAPProtocolId = MBS_TCP_PROTOCOL_ID;
                            xAnalyzerFrame.x.xTCPHeader.usMBAPLength =
                                ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF] << 8U );;
                            xAnalyzerFrame.x.xTCPHeader.usMBAPLength |=
                                ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF + 1] );;
                            xAnalyzerFrame.x.xTCPHeader.ubUnitIdentifier = pxTCPClientHdl->arubBuffer[MBS_TCP_UID_OFF];
                            xAnalyzerFrame.ubDataPayload =
                                ( const UBYTE * )&pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF];
                            xAnalyzerFrame.usDataPayloadLength = usLen;
                            if( NULL != pxIntHdl->pvMBAnalyzerCallbackFN )
                            {
                                vMBPGetTimeStamp( &xTimeStamp );
                                pxIntHdl->pvMBAnalyzerCallbackFN( pxIntHdl, pxIntHdl->pvCtx, &xTimeStamp,
                                                                  &xAnalyzerFrame );
                            }
#endif
                            eStatus = MB_ENOERR;
                            break;
                        case MB_EIO:
                            eStatus = MB_EIO;
                            break;
                        default:
                            eStatus = MB_EPORTERR;
                            break;
                        }
                    }
                    /* Reset the internal state. */
                    pxIntHdl->pubFrameMBPDUBuffer = NULL;
                    pxTCPClientHdl->usBufferPos = 0;

                    pxFrameHdl->ubActiveClient = IDX_INVALID;
                }
            }
            else
            {
                eStatus = MB_EIO;

            }
            MBP_EXIT_CRITICAL_SECTION(  );
        }
    }
    return eStatus;
}

STATIC          eMBErrorCode
eMBSTCPFrameReceive( xMBSHandle xHdl, UBYTE * pubSlaveAddress, USHORT * pusMBPDULength )
    MB_CDECL_SUFFIX
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSTCPClientHandle *pxTCPClientHdl;
    xMBSTCPFrameHandle *pxFrameHdl;

    USHORT          usMBAPTransactionIDField;
    USHORT          usMBAPLengthField;
    UBYTE           ubMBAPUnitIDield;
    USHORT          usMBAPProtocolIDField;

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
        if( MB_IS_VALID_HDL( ( xMBSTCPFrameHandle * ) pxIntHdl->xFrameHdl, arxMBSTCPFrameHdl ) )
        {
            eStatus = MB_EIO;
            pxFrameHdl = pxIntHdl->xFrameHdl;
            /* check if there is an client active. */
            MBP_ENTER_CRITICAL_SECTION(  );
            if( IDX_INVALID != pxFrameHdl->ubActiveClient )
            {
                pxTCPClientHdl = &( pxFrameHdl->xClientHdl[pxFrameHdl->ubActiveClient] );
                if( pxTCPClientHdl->bInUse )
                {
                    MBP_EXIT_CRITICAL_SECTION(  );
                    usMBAPTransactionIDField =
                        ( USHORT ) ( ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF] << 8 );
                    usMBAPTransactionIDField |= ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF + 1];
                    ( void )usMBAPTransactionIDField;
                    usMBAPProtocolIDField = ( USHORT ) ( ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_PID_OFF] << 8 );
                    usMBAPProtocolIDField |= ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_PID_OFF + 1];
                    usMBAPLengthField = ( USHORT ) ( ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF] << 8 );
                    usMBAPLengthField |= ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF + 1];
                    ubMBAPUnitIDield = pxTCPClientHdl->arubBuffer[MBS_TCP_UID_OFF];
                    if( ( MBS_TCP_PROTOCOL_ID == usMBAPProtocolIDField ) &&
                        ( usMBAPLengthField > ( 1 + MB_PDU_SIZE_MIN ) ) &&
                        ( usMBAPLengthField < ( 1 + MB_PDU_SIZE_MAX ) ) &&
                        ( ( USHORT ) ( usMBAPLengthField + MBS_TCP_UID_OFF ) == pxTCPClientHdl->usBufferPos ) )
                    {
                        /* Frame is valid. */
#if defined( MBS_ENABLE_DEBUG_FACILITY ) && ( MBS_ENABLE_DEBUG_FACILITY == 1 )
                        if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_TCP ) )
                        {
                            vMBSLogTCPFrame( MB_LOG_DEBUG, pxFrameHdl,
                                             "Received frame: ",
                                             ( const UBYTE * )&pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF],
                                             pxTCPClientHdl->usBufferPos );
                        }
#endif
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                        pxIntHdl->xFrameStat.ulNPacketsReceived += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                        xAnalyzerFrame.eFrameType = MB_FRAME_TCP;
                        xAnalyzerFrame.x.xTCPHeader.usMBAPTransactionId = usMBAPTransactionIDField;
                        xAnalyzerFrame.x.xTCPHeader.usMBAPProtocolId = usMBAPProtocolIDField;
                        xAnalyzerFrame.x.xTCPHeader.usMBAPLength =
                            ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF] << 8U );
                        xAnalyzerFrame.x.xTCPHeader.usMBAPLength |=
                            ( USHORT ) ( pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF + 1] );
                        xAnalyzerFrame.x.xTCPHeader.ubUnitIdentifier = pxTCPClientHdl->arubBuffer[MBS_TCP_UID_OFF];
#endif
                        if( MB_SER_BROADCAST_ADDR == ubMBAPUnitIDield )
                        {
                            *pubSlaveAddress = MBS_ANY_ADDR;
                        }
                        else
                        {
                            *pubSlaveAddress = ubMBAPUnitIDield;
                        }
                        *pusMBPDULength = ( USHORT ) ( usMBAPLengthField - 1 );
                        pxIntHdl->pubFrameMBPDUBuffer = &pxTCPClientHdl->arubBuffer[MBS_TCP_MB_PDU_OFF];
                        eStatus = MB_ENOERR;
                    }
                    else
                    {
#if defined( MBS_ENABLE_DEBUG_FACILITY ) && ( MBS_ENABLE_DEBUG_FACILITY == 1 )
                        if( bMBPPortLogIsEnabled( MB_LOG_DEBUG, MB_LOG_TCP ) )
                        {
                            vMBSLogTCPFrame( MB_LOG_DEBUG, pxFrameHdl,
                                             "Received frame with incorrect length or wrong slave address: ",
                                             ( const UBYTE * )&pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF],
                                             pxTCPClientHdl->usBufferPos );
                        }
#endif
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                        /* Account for checksum failure. */
                        pxIntHdl->xFrameStat.ulNChecksumErrors += 1;
#endif
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                        xAnalyzerFrame.eFrameType = MB_FRAME_DAMAGED;
#endif
                    }
#if MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1
                    xAnalyzerFrame.eFrameDir = MB_FRAME_RECEIVE;
                    xAnalyzerFrame.ubDataPayload = ( const UBYTE * )&pxTCPClientHdl->arubBuffer[MBS_TCP_TID_OFF];
                    xAnalyzerFrame.usDataPayloadLength = pxTCPClientHdl->usBufferPos;
                    if( NULL != pxIntHdl->pvMBAnalyzerCallbackFN )
                    {
                        vMBPGetTimeStamp( &xTimeStamp );
                        pxIntHdl->pvMBAnalyzerCallbackFN( pxIntHdl, pxIntHdl->pvCtx, &xTimeStamp, &xAnalyzerFrame );
                    }
#endif
                }
                else
                {
                    /* No longer lock the stack. */
                    pxFrameHdl->ubActiveClient = IDX_INVALID;
                    MBP_EXIT_CRITICAL_SECTION(  );
                }
            }
            else
            {
                MBP_EXIT_CRITICAL_SECTION(  );
            }

        }
    }
    return eStatus;
}

STATIC          eMBErrorCode
eMBSTCPFrameClose( xMBSHandle xHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSTCPFrameHandle *pxFrameHdl;

    MBP_ENTER_CRITICAL_SECTION(  );
#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        pxFrameHdl = pxIntHdl->xFrameHdl;
        if( MB_IS_VALID_HDL( pxFrameHdl, arxMBSTCPFrameHdl ) )
        {
            vMBSTCPResetHdl( pxFrameHdl, TRUE );
            eStatus = MB_ENOERR;
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

STATIC          eMBErrorCode
eMBPTCPClientConnectedCB( xMBHandle xMBHdl, xMBPTCPClientHandle xTCPClientHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL;
    xMBSInternalHandle *pxIntHdl = xMBHdl;
    xMBSTCPFrameHandle *pxFrameHdl;
    UBYTE           ubIdx;

    MBP_ENTER_CRITICAL_SECTION(  );
#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        if( MB_IS_VALID_HDL( ( xMBSTCPFrameHandle * ) pxIntHdl->xFrameHdl, arxMBSTCPFrameHdl ) )
        {
            pxFrameHdl = pxIntHdl->xFrameHdl;
            eStatus = MB_ENORES;
            for( ubIdx = 0; ubIdx < MBS_TCP_MAX_CLIENTS; ubIdx++ )
            {
                if( !pxFrameHdl->xClientHdl[ubIdx].bInUse )
                {
                    pxFrameHdl->xClientHdl[ubIdx].bInUse = TRUE;
                    pxFrameHdl->xClientHdl[ubIdx].xMBTCPClientHdl = xTCPClientHdl;
                    eStatus = MB_ENOERR;
                    break;
                }
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

STATIC          eMBErrorCode
eMBPTCPClientDisconnectedCB( xMBHandle xHdl, xMBPTCPClientHandle xTCPClientHdl )
{
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSTCPFrameHandle *pxFrameHdl;
    xMBSTCPClientHandle *pxTCPClientHdl;
    UBYTE           ubIdx;
    eMBErrorCode    eStatus = MB_EINVAL;

    MBP_ENTER_CRITICAL_SECTION(  );
#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        if( MB_IS_VALID_HDL( ( xMBSTCPFrameHandle * ) pxIntHdl->xFrameHdl, arxMBSTCPFrameHdl ) )
        {
            pxFrameHdl = pxIntHdl->xFrameHdl;
            for( ubIdx = 0; ubIdx < MBS_TCP_MAX_CLIENTS; ubIdx++ )
            {
                if( xTCPClientHdl == pxFrameHdl->xClientHdl[ubIdx].xMBTCPClientHdl )
                {
                    /* If the MODBUS stack is currently processing a client request but 
                     * the client has dropped the connection we should mark this 
                     * request as invalid.
                     */
                    if( pxFrameHdl->ubActiveClient == ubIdx )
                    {
                        pxFrameHdl->ubActiveClient = IDX_INVALID;
                    }
                    pxTCPClientHdl = &( pxFrameHdl->xClientHdl[ubIdx] );
                    vMBSTCPResetClientHdl( pxFrameHdl, pxTCPClientHdl, TRUE );
                    eStatus = MB_ENOERR;
                    break;
                }
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

STATIC          eMBErrorCode
eMBPTCPClientNewDataCB( xMBHandle xHdl, xMBPTCPClientHandle xClientHdl )
{
    eMBErrorCode    eStatus = MB_EINVAL, eStatus2;
    xMBSInternalHandle *pxIntHdl = xHdl;
    xMBSTCPFrameHandle *pxFrameHdl;
    xMBSTCPClientHandle *pxTCPClientHdl;
    UBYTE           ubIdx;
    USHORT          usBytesRead;
    USHORT          usMaxBytesToRead;
    USHORT          usMBAPLengthField;

    MBP_ENTER_CRITICAL_SECTION(  );
#if MBS_ENABLE_FULL_API_CHECKS == 1
    if( bMBSIsHdlValid( pxIntHdl ) )
#else
    if( TRUE )
#endif
    {
        if( MB_IS_VALID_HDL( ( xMBSTCPFrameHandle * ) pxIntHdl->xFrameHdl, arxMBSTCPFrameHdl ) )
        {
            pxFrameHdl = pxIntHdl->xFrameHdl;
            if( IDX_INVALID == pxFrameHdl->ubActiveClient )
            {
                for( ubIdx = 0; ubIdx < MBS_TCP_MAX_CLIENTS; ubIdx++ )
                {
                    if( xClientHdl == pxFrameHdl->xClientHdl[ubIdx].xMBTCPClientHdl )
                    {
                        /* We only return an error to the porting layer if the lowlevel
                         * functions fails. It is not considered an error if the client
                         * sends wrong data or has dropped its connection.
                         */
                        eStatus = MB_ENOERR;

                        pxTCPClientHdl = &( pxFrameHdl->xClientHdl[ubIdx] );
                        /* First read the MBAP header which is required to find 
                         * the length of the complete packet.
                         */
                        if( pxTCPClientHdl->usBufferPos < MBS_TCP_MBAP_HEADER_SIZE )
                        {
                            usMaxBytesToRead = ( USHORT ) ( MBS_TCP_MBAP_HEADER_SIZE - pxTCPClientHdl->usBufferPos );
                            eStatus2 = eMBPTCPConRead( pxFrameHdl->xMBTCPServerHdl, pxTCPClientHdl->xMBTCPClientHdl,
                                                       &( pxTCPClientHdl->arubBuffer[pxTCPClientHdl->usBufferPos] ),
                                                       &usBytesRead, usMaxBytesToRead );
                            switch ( eStatus2 )
                            {
                            case MB_ENOERR:
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                                pxIntHdl->xFrameStat.ulNBytesReceived += usBytesRead;
#endif
                                pxTCPClientHdl->usBufferPos += usBytesRead;
                                break;
                            case MB_EIO:
                                vMBSTCPResetClientHdl( pxFrameHdl, pxTCPClientHdl, TRUE );
                                break;

                            default:
                                vMBSTCPResetClientHdl( pxFrameHdl, pxTCPClientHdl, TRUE );
                                eStatus = MB_EPORTERR;
                                break;
                            }
                        }
                        /* Header is complete. Read the rest of the data. Note
                         * that verification is performed in the receive 
                         * function. 
                         */
                        else
                        {
                            usMBAPLengthField =
                                ( USHORT ) ( ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF] << 8 );
                            usMBAPLengthField |= ( USHORT ) pxTCPClientHdl->arubBuffer[MBS_TCP_LEN_OFF + 1];
                            usMaxBytesToRead =
                                ( USHORT ) ( ( usMBAPLengthField + MBS_TCP_UID_OFF ) - pxTCPClientHdl->usBufferPos );
                            if( ( pxTCPClientHdl->usBufferPos + usMaxBytesToRead ) < MBS_TCP_BUFFER_SIZE )
                            {
                                eStatus2 = eMBPTCPConRead( pxFrameHdl->xMBTCPServerHdl, pxTCPClientHdl->xMBTCPClientHdl,
                                                           &( pxTCPClientHdl->arubBuffer[pxTCPClientHdl->usBufferPos] ),
                                                           &usBytesRead, usMaxBytesToRead );
                                switch ( eStatus2 )
                                {
                                case MB_ENOERR:
                                    pxTCPClientHdl->usBufferPos += usBytesRead;
                                    MBP_ASSERT( pxTCPClientHdl->usBufferPos <=
                                                ( USHORT ) ( usMBAPLengthField + MBS_TCP_UID_OFF ) );
                                    /* Check if MODBUS frame is complete. */
                                    if( ( USHORT ) ( usMBAPLengthField + MBS_TCP_UID_OFF ) ==
                                        pxTCPClientHdl->usBufferPos )
                                    {
                                        if( MB_ENOERR ==
                                            eMBPEventPost( pxIntHdl->xFrameEventHdl,
                                                           ( xMBPEventType ) MBS_EV_RECEIVED ) )
                                        {
                                            pxFrameHdl->ubActiveClient = ubIdx;
                                        }
                                        else
                                        {
                                            eStatus = MB_EPORTERR;
                                        }
                                    }
#if MBS_ENABLE_STATISTICS_INTERFACE == 1
                                    pxIntHdl->xFrameStat.ulNBytesReceived += usBytesRead;
#endif
                                    break;
                                case MB_EIO:
                                    vMBSTCPResetClientHdl( pxFrameHdl, pxTCPClientHdl, TRUE );
                                    break;
                                default:
                                case MB_EPORTERR:
                                    vMBSTCPResetClientHdl( pxFrameHdl, pxTCPClientHdl, TRUE );
                                    eStatus = MB_EPORTERR;
                                }
                            }
                            else
                            {
                                pxTCPClientHdl->usBufferPos = 0;
                            }
                        }
                    }
                }
            }
            else
            {
                eStatus = MB_ENOERR;
            }
        }
    }
    MBP_EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

STATIC void
vMBSTCPResetClientHdl( const xMBSTCPFrameHandle * pxFrameHdl, xMBSTCPClientHandle * pxTCPClientHdl, BOOL bClose )
{
    if( ( NULL != pxFrameHdl ) && ( NULL != pxTCPClientHdl ) )
    {
        if( bClose && pxTCPClientHdl->bInUse && ( MBP_TCPHDL_CLIENT_INVALID != pxTCPClientHdl->xMBTCPClientHdl ) )
        {
            ( void )eMBPTCPConClose( pxFrameHdl->xMBTCPServerHdl, pxTCPClientHdl->xMBTCPClientHdl );
        }
        pxTCPClientHdl->bInUse = FALSE;
        pxTCPClientHdl->bHasData = FALSE;
        pxTCPClientHdl->usBufferPos = 0;
        pxTCPClientHdl->xMBTCPClientHdl = MBP_TCPHDL_CLIENT_INVALID;
    }
}

STATIC void
vMBSTCPResetHdl( xMBSTCPFrameHandle * pxFrameHdl, BOOL bClose )
{
    UBYTE           ubIdx;
    eMBErrorCode    eStatus;

    if( MB_IS_VALID_HDL( pxFrameHdl, arxMBSTCPFrameHdl ) )
    {
        for( ubIdx = 0; ubIdx < MBS_TCP_MAX_CLIENTS; ubIdx++ )
        {
            vMBSTCPResetClientHdl( pxFrameHdl, &( pxFrameHdl->xClientHdl[ubIdx] ), bClose );
        }
        if( bClose && ( MBP_TCPHDL_INVALID != pxFrameHdl->xMBTCPServerHdl ) )
        {
            eStatus = eMBTCPServerClose( pxFrameHdl->xMBTCPServerHdl );
            MBP_ASSERT( MB_ENOERR == eStatus );
        }
        pxFrameHdl->xMBTCPServerHdl = MBP_TCPHDL_INVALID;
        if( bClose && ( MBP_EVENTHDL_INVALID != pxFrameHdl->xMBTCPEventHdl ) )
        {
            vMBPEventDelete( pxFrameHdl->xMBTCPEventHdl );
        }
        pxFrameHdl->xMBTCPEventHdl = MBP_EVENTHDL_INVALID;
        pxFrameHdl->ubIdx = IDX_INVALID;
        pxFrameHdl->ubActiveClient = IDX_INVALID;
    }
}

#if ( defined( MBS_ENABLE_DEBUG_FACILITY ) && ( MBS_ENABLE_DEBUG_FACILITY == 1 ) )
void
vMBSLogTCPFrame( eMBPortLogLevel eLevel, xMBSTCPFrameHandle * pxIntHdl, char *szMsg, const UBYTE * pubPayload,
                 USHORT usLength )
{
    USHORT          usIdx;
    CHAR            arubBuffer[MBS_TCP_BUFFER_SIZE * 2U + 1];

    MBP_ASSERT( usLength < MBS_TCP_BUFFER_SIZE );
    if( usLength > 0 )
    {
        for( usIdx = 0; usIdx < usLength; usIdx++ )
        {
            sprintf( &arubBuffer[usIdx * 2], "%02X", pubPayload[usIdx] );
        }
    }
    else
    {
        strcpy( arubBuffer, "empty" );
    }
    vMBPPortLog( eLevel, MB_LOG_TCP, "[IDX=%hu] %s%s\n", ( USHORT ) pxIntHdl->ubIdx, szMsg, arubBuffer );
}
#endif

#endif
