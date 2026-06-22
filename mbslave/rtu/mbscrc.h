/* 
 * MODBUS Library: A portable MODBUS master for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbscrc.h,v 1.3 2009/01/03 17:37:41 cwalter Exp $
 */


#ifndef _MBS_CRC_H
#define _MBS_CRC_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/*! 
 * \if INTERNAL_DOCS
 * \addtogroup mbs_rtu_int
 * @{
 * \endif
 */

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/

/*! \brief Calculates the CRC16 checksum of a frame of length usLen and
 *   returns it.
 * \internal
 *
 * \param pucFrame A pointer to a frame of length \c usLen.
 * \param usLen Size of the frame to check.
 * \return the CRC16 checksum for this frame.
 */
USHORT usMBSCRC16( const UBYTE * pucFrame, USHORT usLen );

/*! 
 * \if INTERNAL_DOCS
 * @} 
 * \endif
 */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
