#ifndef __TASK_RS485_7_H
#define __TASK_RS485_7_H

#include "main.h"

/**
 * @brief  创建第7路RS485任务 (USART3, 物理端口索引6)
 * @note   根据拨码开关 BA7 决定 Modbus 主/从模式
 *         BA5-BA6 决定波特率
 *         通信正常时 LED_485_1 闪烁
 */
void rs485_7_task_create(void);

#endif /* __TASK_RS485_7_H */
