/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbsiframe.h,v 1.4 2008/09/18 16:20:30 embedded-solutions Exp $
 */

#ifndef _MBSI_FRAME_H
#define _MBSI_FRAME_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/

/*!
 * \if INTERNAL_DOCS
 * \addtogroup mbs_int
 * @{
 * \endif
 */

/*! \brief An invalid frame handle.
 * \internal
 */
#define MBS_FRAME_HANDLE_INVALID    ( NULL )

/*! \brief Dummy address used by the stack to answer all MODBUS requests. 
 * \internal
 */
#define MBS_ANY_ADDR                ( 255 )

#ifndef MB_CDECL_SUFFIX
#define MB_CDECL_SUFFIX
#endif		
/* ----------------------- Type definitions ---------------------------------*/

/*! \brief A handle for a frame handler instance.
 * \internal
 */
typedef void   *xMBSFrameHandle;

/*! \brief This function is called by the protocol stack when a new frame 
 *   should be sent.
 * \internal
 *
 * Before this function is called the MODBUS stack has prepared the buffer 
 * pxIntHdl->pubFrameMBPDUBuffer. The size of the MODBUS PDU is usMBPDULength. This
 * function is then responsible for adding the header and the trailer depending
 * on the transmission mode used. For example in MODBUS RTU mode it would add the
 * slave address to the beginning and the CRC16 checksum to the end.i
 *
 * \return eMBErrorCode::MB_ENOERR if the frame was sent. If the frame can not be
 *  sent right now but it is not a permanent error the function should return
 *  eMBErrorCode::MB_EIO. If no more frames can be sent the function should return
 *  eMBErrorCode::MB_EPORTERR indicating that there was a serious error in the 
 *  porting layer.
 */
typedef         eMBErrorCode( *peMBSFrameSend ) ( xMBSHandle xHdl, USHORT usMBPDULength );

typedef         eMBErrorCode( *peMBSFrameReceive ) ( xMBSHandle xHdl, /*@out@ */ UBYTE * pubSlaveAddress, /*@out@ */
                                                     USHORT * pusMBPDULength ) MB_CDECL_SUFFIX;

typedef         eMBErrorCode( *peMBSFrameManagement ) ( xMBSHandle xHdl );

typedef         eMBErrorCode( *peMBSFrameClose ) ( xMBSHandle xHdl );


/*! 
 *\if INTERNAL_DOCS
 * @} 
 * \endif
 */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
