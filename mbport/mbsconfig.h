/* 
 * MODBUS Library: CortexM3 port
 * Copyright (c) 2008 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbsconfig.h,v 1.1 2009/01/03 09:13:01 cwalter Exp $
 */

#ifndef _MBS_CONFIG_H
#define _MBS_CONFIG_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

/* ----------------------- Defines ------------------------------------------*/
#define MBS_ASCII_ENABLED                       ( 0 )
#define MBS_RTU_ENABLED                         ( 1 )
#define MBS_TCP_ENABLED                         ( 0 )
#define MBS_SERIAL_RTU_MAX_INSTANCES            ( 7 )
#define MBS_SERIAL_ASCII_MAX_INSTANCES          ( 2 )
#define MBS_TCP_MAX_INSTANCES                   ( 0 )
#define MBS_SERIAL_API_VERSION                  ( 1 )
#define MBS_RTU_WAITAFTERSEND_ENABLED           ( 1 )
#define MBS_ASCII_WAITAFTERSEND_ENABLED         ( 2 )

#ifdef __cplusplus
PR_END_EXTERN_C
#endif

#endif
