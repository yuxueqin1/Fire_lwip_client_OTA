/* 
 * MODBUS Slave Library: A portable MODBUS slave for MODBUS ASCII/RTU/TCP.
 * Copyright (c) 2007-2010 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbs.h,v 1.17 2010/11/13 14:30:39 embedded-so.embedded-solutions.1 Exp $
 */

#ifndef _MBS_H
#define _MBS_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif
/*! \addtogroup mbs
 * @{
 */
/* ----------------------- Modbus includes ----------------------------------*/
#include "mbsconfig.h"
#include "mbsiconfig.h"
#include "mbtypes.h"
#include "mbframe.h"
/* ----------------------- Defines ------------------------------------------*/
#ifndef MB_CDECL_SUFFIX
#define MB_CDECL_SUFFIX
#endif
/* ----------------------- Type definitions ---------------------------------*/
/*! \brief A handle to a MODBUS slave instance. */
typedef void   *xMBSHandle;

/*! \brief If a register is written or read by the stack.*/
typedef enum
{
    /*! \brief The application values should be updated from the values passed
     * to the callback function.
     */
    MBS_REGISTER_WRITE,

    /*! \brief The application should store the current register values into 
     * the buffer passed to the callback function.
     */
    MBS_REGISTER_READ
} eMBSRegisterMode;

/*! \brief Callback function if a <em>coil register</em> is read or
 *    written by the protocol stack.
 *
 * The callback may read or write up to 2000 coils where the first coil
 * is address by the parameter \c usAddress. If the coils are read by
 * the protocol stack the first coil should be written to the buffer 
 * \c pubRegBuffer where the first 8 coils should be written to 
 * pubRegBuffer[0], the second 8 coils to pubRegBuffer[1], ... If the
 * total amount is not a multiple of 8 the missing coils should be
 * set to zero.<br>
 * If the coils are written by the protocol stack, which is indicated
 * by the argument \c eRegMode set to eMBSRegisterMode::MBS_REGISTER_WRITE
 * , then the callback should update its coils with the values supplied
 * in \c pubRegBuffer. The enconding is the same as above. 
 *
 * \param pubRegBuffer If the values are read by the stack the callback
 *   should update the buffer. Otherwise the callback can read the new
 *   status of the coils from this buffer.
 * \param usAddress Address of first coil.
 * \param usNRegs Number of coils.
 * \param eRegMode If set to eMBSRegisterMode::MBS_REGISTER_READ the 
 *   coils are read by the protocol stack. In case of 
 *   eMBSRegisterMode::MBS_REGISTER_WRITE the protocol stack needs to
 *   know the current value of the coil register.
 * \return If the callback returns eMBException::MB_PDU_EX_NONE a response is
 *   sent back to the master. Otherwise an appropriate exception frame is
 *   generated.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
typedef         eMBException( *peMBSCoilCB ) ( void *pvCtx, UBYTE * pubRegBuffer, USHORT usAddress,
                                               USHORT usNRegs, eMBSRegisterMode eRegMode )MB_CDECL_SUFFIX;
#else
typedef         eMBException( *peMBSCoilCB ) ( UBYTE * pubRegBuffer, USHORT usAddress,
                                               USHORT usNRegs, eMBSRegisterMode eRegMode ) MB_CDECL_SUFFIX;
#endif

/*! \brief Callback function if an <em>discrete register</em> is read by
 *    the protocol stack.
 *
 * The callback may request up to 2000 discrete inputs where the first 
 * input to be returned is \c usAddress. The status of the discrete inputs
 * should be written to the buffer \c pubRegBuffer where the first
 * 8 registers should be written to pubRegBuffer[0], the second 8 registers
 * to pubRegBuffer[1]. If the total amount is not a multiple of 8 missing
 * values should be set to zero.
 *
 * \param pubRegBuffer Buffer where the status of the registers should
 *   be written to.
 * \param usAddress First discrete input to be returned.
 * \param usNRegs Number of registers requested.
 * \return If the callback returns eMBException::MB_PDU_EX_NONE a response is
 *   sent back to the master. Otherwise an appropriate exception frame is
 *   generated.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
typedef         eMBException( *peMBSDiscreteInputCB ) ( void *pvCtx, UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs )MB_CDECL_SUFFIX;
#else
#if defined( HI_TECH_C ) && defined( __PICC18__ )
typedef         eMBException( *peMBSDiscreteInputCB ) ( UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs, void *pvDummy ) MB_CDECL_SUFFIX;
#else
typedef         eMBException( *peMBSDiscreteInputCB ) ( UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs ) MB_CDECL_SUFFIX;
#endif
#endif

/*! \brief Callback function if an <em>input register</em> is read by the 
 *   protocol stack.
 *
 * If the MODBUS slave stack needs to now the values of input registers this
 * callback function must store the current value of the registers starting
 * at\c usAddress to <tt>usAddress + usNRegs</tt> (not including the last one)
 * into the buffer \c pubRegBuffer. The 16 bit registers values must be stored
 * in big endian format. 
 *
 * \param pubRegBuffer A pointer to an internal buffer. Exactly
 *   <tt>2 * usNRegs</tt> bytes must be written to this buffer.
 * \param usAddress The address of the first register which should be returned.
 *   Registers start at zero.
 * \param usNRegs The number of registers to read.
 *
 * \return If the callback returns eMBException::MB_PDU_EX_NONE a response is
 *   sent back to the master. Otherwise an appropriate exception frame is
 *   generated.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
typedef         eMBException( *peMBSRegisterInputCB ) ( void *pvCtx, UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs )MB_CDECL_SUFFIX;

#else
#if defined( HI_TECH_C ) && defined( __PICC18__ )
typedef         eMBException( *peMBSRegisterInputCB ) ( UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs, void *pvDummy ) MB_CDECL_SUFFIX;
#else
typedef         eMBException( *peMBSRegisterInputCB ) ( UBYTE * pubRegBuffer, USHORT usAddress,
                                                        USHORT usNRegs ) MB_CDECL_SUFFIX;
#endif
#endif

/*! \brief Callback function if a <em>holding register</em> is read or 
 *   written by the protocol stack.
 *
 * If the MODBUS slave stack needs to now the value of holding registers it
 * executes this callback with \c eRegisterMode = MBS_REGISTER_READ. The
 * callback function should then store the current values of the registers 
 * starting at \c usAddress up to <tt>usAddress + usNRegs</tt> (not 
 * including the last one) into the buffer \c pubRegBuffer. The 16 bit
 * registers values must be stored in big endian format.<br>
 * If the values should be updated the function is called with \c eRegisterMode
 * set to MBS_REGISTER_WRITE. The register values are passed in the buffer \c
 * pubRegBuffer and it is the applications responsibility to use these values.
 * The first register is stored in <tt>pubRegBuffer[0] - pubRegBuffer[1]</tt>
 * where the high byte comes first (big endian).
 *
 * \param pubRegBuffer A pointer to an internal buffer. If registers are read
 *   then exactly <tt>2 * usNRegs</tt> bytes must be written to this buffer.
 *   If registers are written the application can read up to 
 *   <tt>2 * usNRegs</tt> from this buffer.
 * \param usAddress The address of the first register which should be returned.
 *   Registers start at zero.
 * \param usNRegs The number of registers to read.
 * \param eRegMode If the registers are read or written.
 * 
 * \return If the callback returns eMBException::MB_PDU_EX_NONE a response is
 *   sent back to the master. Otherwise an appropriate exception frame is
 *   generated.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
typedef         eMBException( *peMBSRegisterHoldingCB ) ( void *pvCtx, UBYTE * pubRegBuffer, USHORT usAddress,
                                                          USHORT usNRegs, eMBSRegisterMode eRegMode )MB_CDECL_SUFFIX;

#else
typedef         eMBException( *peMBSRegisterHoldingCB ) ( UBYTE * pubRegBuffer, USHORT usAddress, USHORT usNRegs,
                                                          eMBSRegisterMode eRegMode ) MB_CDECL_SUFFIX;
#endif

/*! \brief Callback function for custom function codes.
 * \ingroup mbs_functions
 *
 * If a custom MODBUS function code should be implemented a callback function
 * can be registered. The function is called by the MODBUS stack whenever such 
 * a function is encountered. 
 *
 * \param pubMBPDU The MODBUS request sent by the MODBUS master which is
 *  \c pusMBPDULength bytes long. The response should be written into the
 *  same buffer and the length of the response should be written to
 *  \c pubMBPDULength. It must not exceed the size of 243 bytes.
 * \param pubMBPDULen Length of the MODBUS PDU.
 *
 * \return If the function returns eMBException::MB_PDU_EX_NONE a response
 *  is sent to the master with a length of \c pusMBPDULength and the
 *  contents of \c pubMBPDU. Otherwise an exception frame is generated.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
typedef         eMBException( *peMBSCustomFunctionCB ) ( void *pvCtx, UBYTE * pubMBPDU,
                                                         USHORT * pusMBPDULength )MB_CDECL_SUFFIX;
#else
typedef         eMBException( *peMBSCustomFunctionCB ) ( UBYTE * pubMBPDU, USHORT * pusMBPDULength ) MB_CDECL_SUFFIX;
#endif

#if defined( DOXYGEN) || ( MBS_ENABLE_GATEWAY_MODE == 1  )
/*! \brief Callback function for gateway mode. 
 *
 * The argument pubMBPDU contains the complete MODBUS protocol data unit.
 *
 * \param ubSlaveAddress The slave address for which the request was intended.
 * \param pubMBPDU The MODBUS request sent by the MODBUS master which is
 *  \c pusMBPDULength bytes long. The response should be written into the
 *  same buffer and the length of the response should be written to
 *  \c pubMBPDULength. It must not exceed the size of 243 bytes.
 * \param pubMBPDULen Length of the MODBUS PDU.
 */
typedef         eMBException( *peMBSGatewayCB ) ( UBYTE ubSlaveAddress, UBYTE * pubMBPDU,
                                                  USHORT * pusMBPDULength ) MB_CDECL_SUFFIX;
#endif
/* ----------------------- Function prototypes ------------------------------*/

#if defined( DOXYGEN ) || ( MBS_TRACK_SLAVEADDRESS == 1 )
/*! \brief Retrieve the current slave address for this request.
 *
 * This function can only be called during a register callback. It then
 * returns the slave address for this request in pubAddress.
 *
 * \param xHdl A valid MODBUS slave handle. 
 * \param pubAddress On return slave address is stored in this variable.
 * \return eMBErrorCode::MB_EINVAL if called not within a callback. Otherwise
 *   eMBErrorCode::MB_ENOERR;
 */
eMBErrorCode    eMBSGetRequestSlaveAddress( xMBSHandle xHdl, UBYTE * pubAddress );
#endif

#if defined( DOXYGEN ) || ( MBS_ENABLE_STATISTICS_INTERFACE == 1 )
/*! \brief Retrieve the current slave statistics.
 *
 * This function populates the argument pxMBSCurrentStat with the
 * current internal counters.
 *
 * \param xHdl A valid MODBUS slave handle. 
 * \param pxMBSCurrentStat A pointer to an (potentially unitialized)
 *  xMBStat datastructure. When the return value is 
 *  eMBErrorCode::MB_ENOERR this data structure holds a copy
 *  of the internal counters.
 * \return eMBErrorCode::MB_ENOERR if successful. In case of an
 *  invalid argument the function returns eMBErrorCode::MB_EINVAL.
 */
eMBErrorCode    eMBSGetStatistics( xMBSHandle xHdl, xMBStat * pxMBSCurrentStat );

/*! \brief Clears the current statistic counters
 *
 * \param xHdl A valid MODBUS slave handle.
 * \return eMBErrorCode::MB_ENOERR if successful. In case of an
 *  invalid argument the function returns eMBErrorCode::MB_EINVAL.
 */
eMBErrorCode    eMBSResetStatistics( xMBSHandle xHdl );
#endif

#if defined( DOXYGEN ) || ( MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1 )
/*! \brief Register an protocol analyzer.
 *
 * If a protocol analyzer has been registered a callback is made
 * whenever a frame has been sent or received.
 *
 * \param xHdl A valid MODBUS slave handle.
 * \param pvMBAnalyzerCallbackFN A valid pointer to a callback
 *  handler or \c NULL if the analyzer should be removed.
 * \param pvCtxArg A user defined context. Can be \c NULL.
 * \return eMBErrorCode::MB_ENOERR if the analyzer has been added
 *  or removed. eMBErrorCode::MB_EINVAL in case of an invalid MODBUS
 *  handle.
 */
eMBErrorCode    eMBSRegisterProtAnalyzer( xMBSHandle xHdl, void *pvCtxArg,
                                          pvMBAnalyzerCallbackCB pvMBAnalyzerCallbackFN );
#endif

/*! \brief Register a new custom function for MODBUS function code 
 *   \c ubFuncIdx 
 *
 * \note Custom registered functions have precedence over internal function
 * handlers. Therefore it is possible to implement your own function for
 * handling input, holding, discrete and coil registers.
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param ubFuncIdx MODBUS function code for which the handle should be
 *   registered.
 * \param peFuncCB A callback function which is called by the stack whenever
 *   such a function is processed. Use \c NULL to remove a previous callback.
 *
 * \return eMBErrorCode::MB_ENOERR if the function code has been registered
 *   or removed. eMBErrorCode::MB_EINVAL if the handle is invalid or the
 *   function code is not within the allowed range of 1 - 127.
 */
eMBErrorCode    eMBSRegisterFunctionCB( xMBSHandle xHdl, UBYTE ubFuncIdx, peMBSCustomFunctionCB peFuncCB );

/*! \brief Register a function callback for coils.
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param peMBSCoilsCB A pointer to a function. This function is called
 *   whenever a coil is read or written. Use \c NULL to remove the 
 *   previous callback.
 *
 * \return eMBErrorCode::MB_ENOERR if the input callback has been set or removed.
 *   eMBErrorCode::MB_EINVAL if the handle is not valid.
 */
eMBErrorCode    eMBSRegisterCoilCB( xMBSHandle xHdl, peMBSCoilCB peMBSCoilsCB );

/*! \brief Register a function callback for discrete inputs. 
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param peDiscInputCB A pointer to a function. This function is called
 *   whenever a discrete register is read. Use \c NULL to remove the 
 *   previous callback.
 *
 * \return eMBErrorCode::MB_ENOERR if the input callback has been set or removed.
 *   eMBErrorCode::MB_EINVAL if the handle is not valid.
 */
eMBErrorCode    eMBSRegisterDiscreteCB( xMBSHandle xHdl, peMBSDiscreteInputCB peDiscInputCB );

/*! \brief Register a function callback for input registers.
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param peRegInputCB A pointer to a function. This function is called
 *   whenever the value of an input register is required. Use \c NULL to remove
 *   a previous callback.
 *
 * \return eMBErrorCode::MB_ENOERR if the input callback has been set or removed.
 *   eMBErrorCode::MB_EINVAL if the handle is not valid.
 */
eMBErrorCode    eMBSRegisterInputCB( xMBSHandle xHdl, peMBSRegisterInputCB peRegInputCB );

/*! \brief Register a function callback for holding registers.
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param peRegHoldingCB A pointer to a function. This function is called
 *   whenever the value of a holding register is written or read. Use \c NULL to remove
 *   a previous callback.
 *
 * \return eMBErrorCode::MB_ENOERR if the input callback has been set or removed.
 *   eMBErrorCode::MB_EINVAL if the handle is not valid.
 */
eMBErrorCode    eMBSRegisterHoldingCB( xMBSHandle xHdl, peMBSRegisterHoldingCB peRegHoldingCB );

#if defined( DOXYGEN) || ( MBS_ENABLE_GATEWAY_MODE == 1  )
/*! \brief Register function callback for gateway mode.
 *
 * \param xHdl A handle to a MODBUS slave instance.
 * \param peGatewayCB A pointer to a function. This function is called
 *   when the gateway mode is enabled and a new frame is received which
 *   has a different slave address than this stack instance. In TCP mode
 *   these are all slave addresses different from 0 and 255. In RTU mode
 *   these are all slave addresses different from the slave address set
 *   by eMBSSerialInit.
 *
 * \return eMBErrorCode::MB_ENOERR if the input callback has been set or removed.
 *   eMBErrorCode::MB_EINVAL if the handle is not valid.
 */
eMBErrorCode    eMBSRegisterGatewayCB( xMBSHandle xHdl, peMBSGatewayCB peGatewayCB );
#endif

/*! \brief Close the stack. 
 *
 * Shutdown the slave stack. This function waits until all pending MODBUS
 * requests have been answered and then operation is stop. All resources
 * are returned to the porting layer.
 *
 * \param xHdl A handle for a MODBUS slave instances. 
 * \return eMBErrorCode::MB_ENOERR if the stack has been shut down. 
 */
eMBErrorCode    eMBSClose( xMBSHandle xHdl );

/*! \brief The main polling loop of the MODBUS stack.
 *
 * This function must be called periodically. The timer interval required
 * is given by the application but a good starting point is somewhere around
 * 50ms. Internally the function checks if any events have happened and if
 * yes processes them.
 *
 * \note Most customers do not need to implement the handling of
 *   eMBErrorCode::MB_EILLSTATE. But for example if a PPP connection which
 *   might break is used a possible fallback would shut down the stack,
 *   reopen the connection and then start the stack again.
 *   
 * \param xHdl The handle to poll.
 *
 * \return If the handle is not valid the function returns 
 *   eMBErrorCode::MB_EINVAL. If an internal error occurred in the porting
 *   layer the function returns eMBErrorCode::MB_EILLSTATE and this 
 *   protocol instance should be closed. Otherwise the function returns 
 *   eMBErrorCode::MB_ENOERR.
 */
eMBErrorCode    eMBSPoll( xMBSHandle xHdl );

/* ----------------------- Function prototypes ( Serial ) -------------------*/

#if MBS_ASCII_ENABLED == 1 || MBS_RTU_ENABLED == 1
/*! 
 * \brief Create a new instances for a serial MODBUS slave instance using
 *   either ASCII or RTU transmission mode. 
 *
 * Note that after the stack has been created function pointers for input,
 * holding, coil and discrete registers have to be provided. They can be set
 * by calling any of the functions:
 *
 *  - eMBSRegisterHoldingCB
 *  - eMBSRegisterInputCB
 *  - eMBSRegisterDiscreteCB
 *  - eMBSRegisterCoilCB
 *
 * \note
 * In RTU mode 11 bits are used for each data byte. The coding system is
 * 8bit binary.
 *  - 1 start bit.
 *  - 8 data bits with LSB sent first.
 *  - 1 bit for parity (Even, Odd)
 *  - 1 or 2 stop bits (Two stopbits if no parity is used).
 *
 * In ASCII mode 10 bits are used. The coding system uses the hexadecimal
 * ASCII characters 0-8 and A-F. One hexadecimal characters contains 4-bits
 * of data.
 *  - 1 start bit
 *  - 7 data bits with LSB sent first.
 *  - 1 bit for parity (Even, Odd)
 *  - 1 or 2 stop bits (Two stopbits if no parity is used).
 *
 * \param pxHdl A pointer to a MODBUS handle. If the function returns 
 *   eMBErrorCode::MB_ENOERR the handle is updated to hold a new and valid 
 *   slave handle. This handle should never be modified by the user.
 * \param eMode The serial transmission mode to use. Either MB_RTU or MB_ASCII.
 * \param ubSlaveAddress The slave address. Only frames sent to this address
 *   or to the broadcast address are handled. Valid slave addresses are in 
 *   the range 1 - 247.
 * \param ubPort The serial port to use. The meaning of this value depends on
 *   the porting layer.
 * \param ulBaudRate The baudrate. For example 38400.
 * \param eParity The parity to use. 
 *
 * \return eMBErrorCode::MB_ENOERR if a new SLAVE has been created. Otherwise
 *   one of the following error codes is returned.
 *    - eMBErrorCode::MB_EINVAL If any of the arguments are not valid.
 *    - eMBErrorCode::MB_EPORTERR If the porting layer returned an error.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
eMBErrorCode    eMBSSerialInit( xMBSHandle * pxHdl, eMBSerialMode eMode, UBYTE ubSlaveAddress,
                                UBYTE ubPort, ULONG ulBaudRate, eMBSerialParity eParity, void *pvCtx );
#else
eMBErrorCode    eMBSSerialInit( xMBSHandle * pxHdl, eMBSerialMode eMode, UBYTE ubSlaveAddress,
                                UBYTE ubPort, ULONG ulBaudRate, eMBSerialParity eParity );
#endif
#endif

/* ----------------------- Function prototypes ( TCP ) ----------------------*/

#if MBS_TCP_ENABLED == 1
/*! \brief Create a new MODBUS TCP slave instance.
 *
 * This function tries to create a new MODBUS TCP slave listening on the 
 * address \c pcBindAddress and on port \c usTCPPort.
 *
 * \param pxHdl A pointer to a handle. On return this handle is updated
 *   to point to a valid internal handle. The handle should never be 
 *   modified by the user.
 * \param pcBindAddress Address to bind to. For example "0.0.0.0".
 * \param usTCPPort The TCP port to bind to. Default port should be 502.
 * \return eMBErrorCode::MB_ENOERR if a new SLAVE has been created. Otherwise
 *   one of the following error codes is returned.
 *    - eMBErrorCode::MB_EINVAL If any of the arguments are not valid.
 *    - eMBErrorCode::MB_EPORTERR If the porting layer returned an error.
 */
#if MBS_CALLBACK_ENABLE_CONTEXT == 1
eMBErrorCode    eMBSTCPInit( xMBSHandle * pxHdl, CHAR * pcBindAddress, USHORT usTCPPort, void *pvCtx );
#else
eMBErrorCode    eMBSTCPInit( xMBSHandle * pxHdl, CHAR * pcBindAddress, USHORT usTCPPort );
#endif
#endif

#if defined( DOXYGEN ) || ( MBS_ENABLE_GATEWAY_MODE == 1 )
/*! \brief Enable gateway mode where the MODBUS TCP slave answers on
 *   all addresses.
 *
 * Addresses 0 and 255 are MODBUS requests targeted to this slave. Other
 * addresses from 1 - 247 should be forwarded to the appropriate clients.
 */
eMBErrorCode    eMBSTCPSetGatewayMode( xMBSHandle xHdl, BOOL bEnable );
#endif

/*! @} */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
