/*
 * MODBUS Library: Cortex M3 port
 * Copyright (c) Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * $Id: mbportother.c,v 1.1 2009/01/03 09:13:01 cwalter Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
/* ----------------------- Platform includes --------------------------------*/

#include "mbport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mbtypes.h"
#include "mbframe.h"
#include "mbutils.h"

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Type definitions ---------------------------------*/

/* ----------------------- Static variables ---------------------------------*/


/* ----------------------- Static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
void
vMBPAssert( void )
{
    volatile BOOL   bBreakOut = FALSE;

    vMBPEnterCritical(  );
    while( !bBreakOut );
}

void
vMBPEnterCritical( void )
{
	taskENTER_CRITICAL(); // 用FREERTOS的临界区函数来保护临界区
}

void
vMBPExitCritical( void )
{
	taskEXIT_CRITICAL(); // 退出临界区
}

