/* 
 * MODBUS Library: A portable MODBUS master for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbmascii.h,v 1.8 2008/03/06 10:41:01 cwalter Exp $
 */

#ifndef _MBM_ASCII_H
#define _MBM_ASCII_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#if MBM_ASCII_ENABLED == 1

/*! 
 * \if INTERNAL_DOCS
 * \addtogroup mbm_ascii_int
 * @{
 * \endif
 */

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/

/*! \brief Configure a MODBUS master ASCII instances.
 * \internal
 *
 * \param pxIntHdl An internal handle.
 * \param ucPort The port. This value is passed through to the porting layer.
 * \param ulBaudRate Baudrate.
 * \param eParity Parity.
 *
 * \return MB_ENOERR if a new instances has been created. In this case the
 *   memers pxFrameHdl, pFrameSendFN, pFrameRecvFN and pFrameCloseFN in the
 *   handle are updated to point to this RTU instances. If pxIntHdl equals
 *   \c NULL or the baudrate is zero it returns MB_EINVAL. Otherwise the 
 *   error code depends on the porting layer.
 */
eMBErrorCode
eMBMSerialASCIIInit(  /*@shared@ */ xMBMInternalHandle * pxIntHdl, UCHAR ucPort, ULONG ulBaudRate, eMBSerialParity eParity );

/*!
 * \if INTERNAL_DOCS
 * @}
 * \endif
 */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif

#endif
