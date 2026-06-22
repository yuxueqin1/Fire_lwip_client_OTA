/*
 * MODBUS Library: Cortex M3 Port
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbport.h,v 1.1 2009/01/03 09:13:01 cwalter Exp $
 */

#ifndef _MB_PORT_H
#define _MB_PORT_H

#include <assert.h>

#ifdef _cplusplus
extern          "C"
{
#endif

/* ----------------------- Defines ------------------------------------------*/

#define INLINE
#define STATIC                         static

#define PR_BEGIN_EXTERN_C              extern "C" {
#define	PR_END_EXTERN_C                }

#define MBP_ASSERT( x )                ( ( x ) ? ( void ) 0 : vMBPAssert(  ) )

#define MBP_ENTER_CRITICAL_SECTION( )  		   
#define MBP_EXIT_CRITICAL_SECTION( )   		   

#ifndef TRUE
#define TRUE                           ( BOOL )1
#endif

#ifndef FALSE
#define FALSE                          ( BOOL )0
#endif

#define MBP_EVENTHDL_INVALID            NULL
#define MBP_TIMERHDL_INVALID            NULL
#define MBP_SERIALHDL_INVALID           NULL
#define MBP_TCPHDL_INVALID              NULL

//#define MBP_TCPHDL_INVALID                  NULL
//#define MBP_TCPHDL_CLIENT_INVALID           NULL

//#define MBP_TASK_PRIORITY                   ( 4 )  

/* ----------------------- Type definitions ---------------------------------*/
typedef		void			*xMBPEventHandle;
typedef		void			*xMBPTimerHandle;
typedef		void			*xMBPSerialHandle;
typedef		void			*xMBPTCPHandle;
typedef		void			*xMBPTCPClientHandle;

typedef		unsigned  char		BOOL;
typedef		unsigned  char		UBYTE;
typedef			signed  char		BYTE;
typedef		unsigned  char		UCHAR;
typedef			signed  char		CHAR;
typedef		unsigned  short		USHORT;
typedef		 	signed  short		SHORT;
typedef			unsigned  int		ULONG;
typedef				signed  int		LONG;


/* ----------------------- Function prototypes ------------------------------*/
void                vMBPAssert( void );
void                vMBPEnterCritical( void );
void                vMBPExitCritical( void );

#ifdef _cplusplus
}
#endif

#endif
