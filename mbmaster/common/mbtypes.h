/* 
 * MODBUS Library: Common datatypes shared by master and slave stack.
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbtypes.h,v 1.7 2009/12/20 23:09:08 embedded-so.embedded-solutions.1 Exp $
 */

#ifndef _MB_TYPES_H
#define _MB_TYPES_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/*! \addtogroup mb_cmn
 * @{
 */
 
/* ----------------------- Defines ------------------------------------------*/

/*! \brief An invalid MASTER handle. */
#define MB_HDL_INVALID                         ( NULL )

/*! \brief Tests if an error code of the type eMBErrorCode is an exception. 
 *
 * \param eStatus The error code returned by any of the API functions.
 */
#define MB_ERROR_IS_EXCEPTION( eStatus )       \
    ( ( ( eStatus ) >= MB_EX_ILLEGAL_FUNCTION ) && \
      ( ( eStatus ) <= MB_EX_GATEWAY_TARGET_FAILED ) ? TRUE : FALSE )

/* ----------------------- Type definitions ---------------------------------*/
/*! \brief A handle used to hold either MASTER or SLAVE stack handles. */
typedef void   *xMBHandle;

/*! 
 * \brief Error code reported by the protocol stack.
 */
typedef enum
{
    MB_ENOERR = 0,              /*!< No error. */
    MB_ENOREG = 1,              /*!< Illegal register address. */
    MB_EINVAL = 2,              /*!< Illegal argument. */
    MB_EPORTERR = 3,            /*!< Porting layer error. */
    MB_ENORES = 4,              /*!< Insufficient resources. */
    MB_EIO = 5,                 /*!< I/O error. */
    MB_EILLSTATE = 6,           /*!< Protocol stack in illegal state. */
    MB_EAGAIN = 7,              /*!< Retry I/O operation. */
    MB_ETIMEDOUT = 8,           /*!< Timeout error occurred. */
    MB_EX_ILLEGAL_FUNCTION = 10,        /*!< Illegal function exception. */
    MB_EX_ILLEGAL_DATA_ADDRESS = 11,    /*!< Illegal data address. */
    MB_EX_ILLEGAL_DATA_VALUE = 12,      /*!< Illegal data value. */
    MB_EX_SLAVE_DEVICE_FAILURE = 13,    /*!< Slave device failure. */
    MB_EX_ACKNOWLEDGE = 14,     /*!< Slave acknowledge. */
    MB_EX_SLAVE_BUSY = 15,      /*!< Slave device busy. */
    MB_EX_MEMORY_PARITY_ERROR = 16,     /*!< Memory parity error. */
    MB_EX_GATEWAY_PATH_UNAVAILABLE = 17,        /*!< Gateway path unavailable. */
    MB_EX_GATEWAY_TARGET_FAILED = 18    /*!< Gateway target device failed to respond. */
} eMBErrorCode;

/*!
 * \brief Modes supported by the serial protocol stack.
 */
typedef enum
{
    MB_RTU,                     /*!< RTU transmission mode. */
    MB_ASCII                    /*!< ASCII transmission mode. */
} eMBSerialMode;

/*! 
 * \brief Parity for serial transmission mode.
 */
typedef enum
{
    MB_PAR_ODD,                 /*!< ODD parity. */
    MB_PAR_EVEN,                /*!< Even parity. */
    MB_PAR_NONE                 /*!< No parity. */
} eMBSerialParity;

#if defined( DOXYGEN ) || \
( defined( MBM_ENABLE_STATISTICS_INTERFACE ) && ( MBM_ENABLE_STATISTICS_INTERFACE == 1 ) ) || \
( defined( MBS_ENABLE_STATISTICS_INTERFACE ) && ( MBS_ENABLE_STATISTICS_INTERFACE == 1 ) )
/*! \brief This data structure is used by the statistic interface. 
 */
typedef struct
{
    ULONG           ulNPacketsSent;     /*!< Number of sent packets */
    ULONG           ulNPacketsReceived; /*!< Number of received packets */
    ULONG           ulNTimeouts;        /*!< Number of timeout errors */
    ULONG           ulNChecksumErrors;  /*!< Number of checksum errors */
    ULONG           ulNBytesSent;       /*!< Number of bytes sent */
    ULONG           ulNBytesReceived;   /*!< Number of bytes received */
} xMBStat;
#endif



#if defined( DOXYGEN ) || \
( defined( MBM_ENABLE_PROT_ANALYZER_INTERFACE ) && ( MBM_ENABLE_PROT_ANALYZER_INTERFACE == 1 ) ) || \
( defined( MBS_ENABLE_PROT_ANALYZER_INTERFACE ) && ( MBS_ENABLE_PROT_ANALYZER_INTERFACE == 1 ) )

/*! \brief This data structure is used by the protocol analyzer. 
 */
typedef struct
{
    /*! \brief Direction of the frame.
     */
    enum
    {
        MB_FRAME_SEND,          /*!< Frame has been sent. */
        MB_FRAME_RECEIVE,       /*!< Frame has been received. */
    } eFrameDir;

    /*! \brief Depending on the instance of the MODBUS stack different 
     * protocols can be analyzed. The implementation selects the
     * approriate protocol and initializes the member eFrameType
     * to this type. 
     */
    enum
    {
        MB_FRAME_RTU,           /*!< Frame is a valid MODBUS RTU frame. */
        MB_FRAME_ASCII,         /*!< Frame is a valid MODBUS ASCII frame. */
        MB_FRAME_TCP,           /*!< Frame is a valid MODBUS TCP frame. */
        MB_FRAME_DAMAGED        /*!< Only raw data because of invalid frame. */
    } eFrameType;

    /*! \brief Shared structure for protocol headers. You are
     * only allowed to access the element specified by eFrameType.
     */
    union
    {
        struct
        {
            UBYTE           ubSlaveAddress;
            USHORT          usCRC16;
        } xRTUHeader;
        struct
        {
            UBYTE           ubSlaveAddress;
            UBYTE           ubLRC;
        } xASCIIHeader;
        struct
        {
            USHORT          usMBAPTransactionId;
            USHORT          usMBAPProtocolId;
            USHORT          usMBAPLength;
            UBYTE           ubUnitIdentifier;
        } xTCPHeader;
    } x;
    /*! \brief In case of MB_FRAME_RTU, MB_FRAME_ASCII or MB_FRAME_TCP 
     *   contains the MODBUS PDU. In case of MB_FRAME_DAMAGED contains
     *   the raw frame including the header.
     */
    const UBYTE    *ubDataPayload;
    /*! \brief Number of bytes in ubDataPayload which can be accessed.
     */
    USHORT          usDataPayloadLength;
} xMBAnalyzerFrame;

#if !defined( DOXYGEN ) && ( !defined( MBP_HAS_TIMESTAMP ) || ( MBP_HAS_TIMESTAMP == 0 ) )
#error "statistics interface needs timestamp."
#else
/*! \brief A callback function used by the protocol analyzer.
 *
 * Note that for ASCII the request is already converted into its binary 
 * equivalent and the control characters are stripped. The reason for this
 * is that otherwise the stack would require internally twice the amount of
 * memory if requests would be stored in raw. Therefore they are automatically
 * decoded and encoded during transmit and receive.
 *
 * \param xHdl The handle which made the callback. Do NOT call any
 *  MODBUS functions from within.
 * \param pxCtx The context supplied to the init function.
 * \param pxTime Timestamp when the frame has been captured.
 * \param  peFrame The captured frame.
 */
typedef void    ( *pvMBAnalyzerCallbackCB ) ( xMBHandle xHdl, void *pxCtx,
                                               const xMBPTimeStamp * pxTime,
                                               const xMBAnalyzerFrame * peFrame );
#endif

#endif

/* ----------------------- Function prototypes ------------------------------*/

/*! @} */

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
