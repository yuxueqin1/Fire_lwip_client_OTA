/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsfunctions.h,v 1.10 2009/01/03 19:02:05 cwalter Exp $
 */

#ifndef _MBS_FUNCTIONS_H
#define _MBS_FUNCTIONS_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/*! 
 * \if INTERNAL_DOCS
 * \addtogroup mbs_functions
 * @{
 * \endif
 */

/* ----------------------- Defines ------------------------------------------*/
#ifndef MB_CDECL_SUFFIX
#define MB_CDECL_SUFFIX
#endif	

/*! \brief Empty/Illegal function code
 * \internal
 */
#define MBS_FUNCCODE_NONE                           ( 0 )

/*! \brief <em>Read Input Registers</em> function code \c 0x04.
 * \internal
 */
#define MBS_FUNCCODE_READ_INPUT_REGISTERS           ( 0x04 )

/*! \brief <em>Read Holding Registers</em> function code \c 0x03.
 * \internal
 */
#define MBS_FUNCCODE_READ_HOLDING_REGISTERS         ( 0x03 )

/*! \brief <em>Write Single Register</em> function code \c 0x06.
 * \internal
 */
#define MBS_FUNCCODE_WRITE_SINGLE_REGISTER          ( 0x06 )

/*! \brief <em>Write Multiple Registers</em> function code \c 0x10.
 * \internal
 */
#define MBS_FUNCCODE_WRITE_MULTIPLE_REGISTERS       ( 0x10 )

/*! \brief <em>Read/Write Multiple Registers</em> function code \c 0x17.
 * \internal
 */
#define MBS_FUNCCODE_READWRITE_MULTIPLE_REGISTERS   ( 0x17 )

/*! \brief <em>Read Discrete Inputs</em> function code \c 0x02. 
 * \internal
 */
#define MBS_FUNC_READ_DISCRETE_INPUTS               ( 0x02 )

/*! \brief <em>Read Coils</em> function code \c 0x01. 
 * \internal
 */
#define MBS_FUNC_READ_COILS                         ( 0x01 )

/*! \brief <em>Write Single Coil</em> function code \c 0x05. 
 * \internal
 */
#define MBS_FUNC_WRITE_SINGLE_COIL                  ( 0x05 )

/*! \brief <em>Write Multiple Coils</em> function code \c 0x0F. 
 * \internal
 */
#define MBS_FUNC_WRITE_MULTIPLE_COILS               ( 0x0F )

/* ----------------------- Type definitions ---------------------------------*/

/*! \brief Callback function for standard function codes.
 * \internal
 *
 * Default interface for MODBUS function codes. 
 *
 * \param pubMBPDU The MODBUS request sent by the MODBUS master which is
 *  \c pusMBPDULength bytes long. The response should be written into the
 *  same buffer and the length of the response should be written to
 *  \c pubMBPDULength. It must not exceed the size of 243 bytes.
 * \param pubMBPDULen Length of the MODBUS PDU.
 * \param pxMBSRegisterCB Data type holding register callbacks.
 *
 * \return If the function returns eMBException::MB_PDU_EX_NONE a response
 *  is sent to the master with a length of \c pusMBPDULength and the
 *  contents of \c pubMBPDU. Otherwise an exception frame is generated.
 */
typedef         eMBException( *peMBSStandardFunctionCB ) ( UBYTE * pubMBPDU, USHORT * pubMBPDULen,
                                                           const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;

/* ----------------------- Function prototypes ------------------------------*/

#if ( MBS_FUNC_READ_INPUT_REGISTERS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Read Input Register</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException    eMBSFuncReadInputRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen,
                                           const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_READ_HOLDING_REGISTERS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Read Holding Register</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException eMBSFuncReadHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_WRITE_SINGLE_REGISTER_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Write Single Register</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncWriteSingleRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_WRITE_MULTIPLE_REGISTERS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Write Multiple Register</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncWriteMultipleHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_READWRITE_MULTIPLE_REGISTERS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Read/Write Multiple Register</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncReadWriteMultipleHoldingRegister( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_READ_DISCRETE_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Read Discrete Inputs</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncReadDiscreteInputs( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_READ_COILS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Read Discrete Inputs</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncReadCoils( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_WRITE_SINGLE_COIL_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Write Single Coil</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncWriteSingleCoil( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

#if ( MBS_FUNC_WRITE_MULTIPLE_COILS_ENABLED == 1 ) || defined( DOXYGEN )
/*! \brief Implementation of <em>Write Multiple Coils</em> function.
 *
 * \param pubMBPDU The MODBUS protocol PDU which contains the MODBUS request.
 * \param pusMBPDULen The length of the request in bytes.
 * \param pxMBSRegisterCB Handlers registered for Input/Holding/Discrete and Input
 *   registers.
 * \return eMBException::MB_PDU_EX_NONE if a response has been generated. Otherwise
 *   one of the exception codes.
 */
eMBException
eMBSFuncWriteMultipleCoils( UBYTE * pubMBPDU, USHORT * pusMBPDULen, const xMBSRegisterCB * pxMBSRegisterCB ) MB_CDECL_SUFFIX;
#endif

/*!
 a \if INTERNAL_DOCS  
 * @} 
 * \endif
 */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
