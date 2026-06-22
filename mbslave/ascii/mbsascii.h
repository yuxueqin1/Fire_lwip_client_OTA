/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbsascii.h,v 1.3 2007/11/02 16:33:50 cwalter Exp $
 */

#ifndef _MBSASCII_H
#define _MBSASCII_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/*!
 * \if INTERNAL_DOCS
 * \addtogroup mbs_ascii_int
 * @{
 * \endif
 */

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/

/*! \brief Configure a MODBUS slave ASCII instance.
 * \internal
 *
 * \param pxIntHdl An internal handle.
 * \param ubPort The port. This value is passed through to the porting layer.
 * \param ulBaudRate Baudrate.
 * \param eParity Parity.
 *
 * \return eMBErrorCode::MB_ENOERR if a new instance has been created. In 
 *   this case the members pxFrameHdl, pFrameSendFN, pFrameRecvFN,
 *   pFrameCloseFN and pFrameManagementFN in the handle are updated to point 
 *   to this ASCII instance. In case of an invalid handle or baudrate it returns
 *   eMBErrorCode::MB_EINVAL. In case of a porting layer error it returns
 *   eMBErrorCode::MB_EPORTERR.
 */
eMBErrorCode
eMBSSerialASCIIInit(  /*@shared@*/ xMBSInternalHandle * pxIntHdl, UBYTE ubPort, ULONG ulBaudRate, eMBSerialParity eParity );

/*!
 * \if INTERNAL_DOCS
 * @}
 * \endif
 */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
