/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbstcp.h,v 1.3 2007/11/02 16:42:11 cwalter Exp $
 */

#ifndef _MBSTCP_H
#define _MBSTCP_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#if MBS_TCP_ENABLED == 1

/*! 
 * \if INTERNAL_DOCS
 * \addtogroup mbs_tcp_int
 * @{
 * \endif
 */

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/
eMBErrorCode eMBSTCPIntInit( xMBSInternalHandle *pxIntHdl, CHAR *pcBindAddress, USHORT usTCPPort );

/*!
 * \if INTERNAL_DOCS
 * @}
 * \endif
 */

#endif

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
