#ifndef __TASK_RS485_7_H
#define __TASK_RS485_7_H

#include "main.h"

/**
 * @brief  创建第7路RS485任务 (USART3, 物理端口索引6)
 * @note   根据拨码开关 BA7 决定 Modbus 主/从机模式
 *         BA5-BA6 决定波特率
 *
 *  主机模式:
 *    - 读取本板6路端口所有从机模块在线状态 → 生成本板报警变量
 *    - FC 03 读取从机通信板报警状态 (地址1)
 *    - FC 02 读取IO联网模块 DI0(切断开关)/DI1(断接开关) (地址4)
 *    - 根据报警真值表计算 蜂鸣器/报警灯/工作灯 状态
 *    - FC 05 写入IO联网模块 DO0(工作灯)/DO1(蜂鸣器)/DO2(报警灯)
 *
 *  从机模式:
 *    - FC 03/04 暴露本板报警状态给主机查询 (地址1)
 */

/* ---- Modbus 从机地址 ---- */
#define RS485_7_SLAVE_COMM_ADDR   1U    /* 从机通信板地址       */
#define RS485_7_IO_MODULE_ADDR    4U    /* IO联网模块地址       */

/* ---- IO联网模块 Modbus 寄存器地址 (MW-IOD0808B) ---- */
#define IO_MODULE_DI_START        0U    /* DI起始地址           */
#define IO_MODULE_DI_COUNT        2U    /* DI数量 (DI0=cut, DI1=break) */
#define IO_MODULE_DO_START        0U    /* DO/线圈起始地址      */
#define IO_MODULE_DO_WORK_LIGHT   0U    /* DO0: 工作灯(通信状态灯) */
#define IO_MODULE_DO_BUZZER       1U    /* DO1: 蜂鸣器          */
#define IO_MODULE_DO_ALARM_LIGHT  2U    /* DO2: 报警灯          */

/* ---- 从机通信板寄存器映射 ---- */
#define SLAVE_REG_ALARM_STATUS    0U    /* 保持寄存器0: 报警状态 */
#define SLAVE_REG_ALARM_RAW       1U    /* 保持寄存器1: ALARM原始值 */

/* ---- 通信超时阈值 (任务周期1秒 × 15 = 15秒) ---- */
#define IO_COMM_TIMEOUT_CNT       15U

/* ---- 全局状态变量 (供其他模块读取串口7的IO状态) ---- */
extern uint8_t g_rs485_7_local_alarm;    /* 本板报警状态 (主/从机均有效) */
extern uint8_t g_rs485_7_remote_alarm;   /* 远程板报警状态 (仅主机有效) */
extern uint8_t g_rs485_7_io_cut;         /* 切断开关状态 (仅主机有效)   */
extern uint8_t g_rs485_7_io_break;       /* 断接开关状态 (仅主机有效)   */
extern uint8_t g_rs485_7_work_light;     /* 工作灯状态 (仅主机有效)     */
extern uint8_t g_rs485_7_buzzer;         /* 蜂鸣器状态 (仅主机有效)     */
extern uint8_t g_rs485_7_alarm_light;    /* 报警灯状态 (仅主机有效)     */
extern uint8_t g_rs485_7_slave_present;  /* 1=已确认从机通信板在线 (现场诊断用) */

/* ---- 函数声明 ---- */
void rs485_7_task_create(void);

#endif /* __TASK_RS485_7_H */
