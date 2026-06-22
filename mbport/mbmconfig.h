/*
 * MODBUS Library: CortexM3 port
 * Copyright (c) 2007 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * $Id: mbmconfig.h,v 1.2 2009/01/02 14:07:29 cwalter Exp $
 */

#ifndef _MBM_CONFIG_H
#define _MBM_CONFIG_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/
#define MBM_ASCII_ENABLED                       ( 0)
#define MBM_RTU_ENABLED                         ( 1 )
#define MBM_TCP_ENABLED                         ( 0 )
#define MBM_DEFAULT_RESPONSE_TIMEOUT            ( 1000)
#define MBM_SERIAL_RTU_MAX_INSTANCES            ( 7 )
#define MBM_SERIAL_ASCII_MAX_INSTANCES          ( 2 )
#define MBM_TCP_MAX_INSTANCES                   ( 0 )
#define MBM_RTU_WAITAFTERSEND_ENABLED						( 0 ) // Mark：MBM_RTU_WAITAFTERSEND_ENABLED 延时使能，RTU发送完成后等待一段时间再使能接收，默认值为0，即不等待直接使能接收。设置为1时，等待3.5个字符时间，设置为2时，等待动态计算的时间（根据波特率计算）。如果使用了MBM_RTU_WAITAFTERSEND_ENABLED，则必须定义MBM_SERIAL_RTU_DYNAMIC_WAITAFTERSEND_TIMEOUT_MS来指定等待时间。
#define MBM_ASCII_WAITAFTERSEND_ENABLED					( 2 )

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
