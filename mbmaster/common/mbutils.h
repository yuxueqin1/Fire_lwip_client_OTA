/* 
 * MODBUS Library: Common utilities shared by master and slave stack.
 *
 * Copyright (c) 2007 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbutils.h,v 1.12 2010/11/11 22:05:31 embedded-so.embedded-solutions.1 Exp $
 */

#ifndef _MB_UTILS_H
#define _MB_UTILS_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/

/*! 
 * \if INTERNAL_DOCS
 * \addtogroup mb_cmn_int
 * @{
 * \endif
 */

/*! \brief Rounds an integer number to the next multiple of 2.
 * \internal
 *
 * \param num The number to round.
 */
#define MB_CEIL_2( num )       			( ( ( ( num ) + 1 ) / 2 ) * 2 )

/* \brief Perform integer division where the result is rounded
 *  toward +infinity.
 * \internal
 *
 * \param dividend Divided
 * \param divisor  Divisor
 */
#define MB_INTDIV_CEIL( dividend, divisor )		\
	( ( ( dividend ) + ( divisor ) - 1 ) / ( divisor ) )

/* \brief Perform integer division where the result is rounded
 *   toward the nearest integer number.
 * \internal
 *
 * \param dividend Divided
 * \param divisor  Divisor
 */
#define MB_INTDIV_ROUND( dividend, divisor )	\
	( ( ( dividend ) + ( divisor ) / 2 ) / ( divisor ) )

/* \brief Perform integer division where the result is rounded
 *   toward -infinity.
 * \internal
 *
 * \param dividend Divided
 * \param divisor  Divisor
 */
#define MB_INTDIV_FLOOR( dividend, divisor )	\
	( ( dividend ) / ( divisor ) )

/*! \brief Calculate the number of elements in an array which size is known
 *   at compile time.
 * \internal
 * \param x The array. 
 */
#define MB_UTILS_NARRSIZE( x ) ( sizeof( x ) / sizeof( ( x )[ 0 ] ) )

/*! \brief Checks if a handle is valid.
 * \internal
 *
 * This method checks if a handle is valid. It uses the ubIdx index to
 * check if the pointer points to a valid handle.
 *
 * \param pxHdl A pointer to a handle.
 * \param arxHdl An array of handles.
 * \return A boolean value.
 */
#if ( ( defined( MBM_ENABLE_FULL_API_CHECKS ) && ( MBM_ENABLE_FULL_API_CHECKS == 1 ) ) || \
      ( defined( MBS_ENABLE_FULL_API_CHECKS ) && ( MBS_ENABLE_FULL_API_CHECKS == 1 ) ) )
#define MB_IS_VALID_HDL( pxHdl, arxHdl ) \
    ( ( ( pxHdl ) != NULL ) && \
      ( ( ( ( pxHdl )->ubIdx ) ) < MB_UTILS_NARRSIZE( arxHdl ) ) && \
      ( ( pxHdl ) == &arxHdl[ ( pxHdl )->ubIdx ] ) )
#else
#define MB_IS_VALID_HDL( pxHdl, arxHdl ) \
    ( ( ( pxHdl ) != NULL ) && ( ( pxHdl ) == &arxHdl[ ( pxHdl )->ubIdx ] ) )
#endif    

/*! 
 * \if INTERNAL_DOCS
 * @} 
 * \endif
 */

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Function prototypes ------------------------------*/

/*! \brief Function to set bits in a byte buffer.
 *
 * This function allows the efficient use of an array to implement bitfields.
 * The array used for storing the bits must always be a multiple of two
 * bytes. Up to eight bits can be set or cleared in one operation.
 *
 * \param ucByteBuf A buffer where the bit values are stored. Must be a
 *   multiple of 2 bytes. No length checking is performed and if
 *   usBitOffset / 8 is greater than the size of the buffer memory contents
 *   is overwritten.
 * \param usBitOffset The starting address of the bits to set. The first
 *   bit has the offset 0.
 * \param ucNBits Number of bits to modify. The value must always be smaller
 *   than 8.
 * \param ucValues Thew new values for the bits. The value for the first bit
 *   starting at <code>usBitOffset</code> is the LSB of the value
 *   <code>ucValues</code>
 *
 * \code
 * ucBits[2] = {0, 0};
 *
 * // Set bit 4 to 1 (read: set 1 bit starting at bit offset 4 to value 1)
 * xMBUtilSetBits( ucBits, 4, 1, 1 );
 *
 * // Set bit 7 to 1 and bit 8 to 0.
 * xMBUtilSetBits( ucBits, 7, 2, 0x01 );
 *
 * // Set bits 8 - 11 to 0x05 and bits 12 - 15 to 0x0A;
 * xMBUtilSetBits( ucBits, 8, 8, 0x5A);
 * \endcode
 */
void            xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset,
                                UCHAR ucNBits, UCHAR ucValues );

/*! \brief Function to read bits in a byte buffer.
 *
 * This function is used to extract up bit values from an array. Up to eight
 * bit values can be extracted in one step.
 *
 * \param ucByteBuf A buffer where the bit values are stored.
 * \param usBitOffset The starting address of the bits to set. The first
 *   bit has the offset 0.
 * \param ucNBits Number of bits to modify. The value must always be smaller
 *   than 8.
 *
 * \code
 * UCHAR ucBits[2] = {0, 0};
 * UCHAR ucResult;
 *
 * // Extract the bits 3 - 10.
 * ucResult = xMBUtilGetBits( ucBits, 3, 8 );
 * \endcode
 */
UCHAR           xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset,
                                UCHAR ucNBits );

/*! \brief Map a MODBUS exception code to an application error code.
 * \ingroup mb_cmn
 *
 * \param ubMBException a MODBUS exception code from a MODBUS frame.
 * \return The MODBUS exception converted to an application error.
 */
eMBErrorCode 	eMBExceptionToErrorcode( UBYTE ubMBException );

/*! \brief Map an application exception to a MODBUS exception.
 * \ingroup mb_cmn
 *
 * \param eCode Any of the eMBErrorCode values. Non exceptional
 *   values are mapped to eMBException::MB_PDU_EX_SLAVE_DEVICE_FAILURE.
 * \return The exception converted to a MODBUS PDU exception.
 */
eMBException    eMBErrorcodeToException( eMBErrorCode eCode );

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
