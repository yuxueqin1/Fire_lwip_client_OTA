/**
  ******************************************************************************
  * @file    task_rs485_7.c
  * @author  fire
  * @version V1.0
  * @date    2026-07-15
  * @brief   第7路RS485 串口任务 (USART3, 物理端口索引6)
  *          根据拨码开关配置运行于 Modbus 主机或从机模式
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "task_rs485_7.h"
#include "./dip_switch/bsp_dip_switch.h"
#include "./led/bsp_led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mbm.h"    /* Modbus Master API */
#include "mbs.h"    /* Modbus Slave API */

/* ============================================================================
 *  私有定义
 *  ==========================================================================*/
#define RS485_7_PORT_INDEX      6U     /* USART3 在 mbportserial 中的索引 */
#define RS485_7_SLAVE_ADDR      1U     /* 默认 Modbus 从机地址 */
#define RS485_7_TASK_PRIO       3      /* 与现有6路UART同级 */
#define RS485_7_TASK_STACK      384    /* 栈大小 (words) */

/* ============================================================================
 *  全局句柄 (用于主/从模式)
 *  ==========================================================================*/
static xMBMHandle xMBMaster7 = NULL;   /* Modbus 主机句柄 */
static xMBSHandle xMBSlave7  = NULL;   /* Modbus 从机句柄 */

/* ============================================================================
 *  从机模式: 寄存器回调 (静态函数, 通过 eMBSRegister*CB 注册)
 *  ==========================================================================*/

/**
 * @brief  从机保持寄存器回调 (FC 03/06/10)
 * @note   供 PC 主机读写测试, 返回简单数据
 */
static eMBException slave7_holding_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                       USHORT usNRegs, eMBSRegisterMode eRegMode)
{
    USHORT i;
    for (i = 0; i < usNRegs; i++) {
        if (eRegMode == MBS_REGISTER_READ) {
            /* 返回一个可识别的模式: 地址+1 */
            USHORT val = (USHORT)(usAddress + i + 1U);
            *pubRegBuffer++ = (UCHAR)(val >> 8);
            *pubRegBuffer++ = (UCHAR)(val & 0xFF);
        } else {
            /* 写入模式: 跳过写入字节 (只读缓冲区, 丢弃写入) */
            pubRegBuffer += 2;
        }
    }
    return MB_PDU_EX_NONE;
}

/**
 * @brief  从机输入寄存器回调 (FC 04)
 * @note   返回固定值 0x4857 (标识 485-7 端口)
 */
static eMBException slave7_input_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                     USHORT usNRegs)
{
    USHORT i;
    (void)usAddress;
    for (i = 0; i < usNRegs; i++) {
        USHORT val = 0x4857U;  /* 固定值: 485-7 标识 */
        *pubRegBuffer++ = (UCHAR)(val >> 8);
        *pubRegBuffer++ = (UCHAR)(val & 0xFF);
    }
    return MB_PDU_EX_NONE;
}

/* ============================================================================
 *  第7路RS485 任务主函数
 *  ==========================================================================*/
static void rs485_7_task_handler(void *pvParameters)
{
    uint32_t baudrate;
    uint8_t  mode;
    uint8_t  led_state = 0;

    (void)pvParameters;

    /* 读取拨码配置 */
    baudrate = g_dip_rs485_7_baudrate;
    mode     = g_dip_rs485_7_mode;   /* 3=Slave, 4=Master */

    printf("[RS485-7] Baud=%lu, Mode=%s\n",
           (unsigned long)baudrate,
           (mode == 4U) ? "Master" : "Slave");

    /* ---- 根据模式初始化 ---- */
    if (mode == 4U) {
        /* =============================================================
         *  Modbus 主机模式
         * ============================================================= */
        if (eMBMSerialInit(&xMBMaster7, MB_RTU, RS485_7_PORT_INDEX,
                           baudrate, MB_PAR_NONE) != MB_ENOERR) {
            printf("[RS485-7] Master init FAILED!\n");
            while (1) { vTaskDelay(1000); }
        }
        printf("[RS485-7] Master mode started, polling slave addr=%u\n",
               RS485_7_SLAVE_ADDR);

        /* 主机任务循环: 轮询从机并闪烁 LED */
        while (1) {
            USHORT regs[2] = {0};
            eMBErrorCode eStatus;

            eStatus = eMBMReadInputRegisters(xMBMaster7, (UCHAR)RS485_7_SLAVE_ADDR,
                                              (USHORT)0, (UBYTE)2, regs);
            if (eStatus == MB_ENOERR) {
                /* 通信成功: 闪烁 LED */
                LED_485_Set(0, TOGGLE);
                vTaskDelay(pdMS_TO_TICKS(200));
            } else {
                /* 通信失败: LED 灭, 等待重试 */
                LED_485_1_OFF;
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

    } else {
        /* =============================================================
         *  Modbus 从机模式 (默认)
         * ============================================================= */
        if (eMBSSerialInit(&xMBSlave7, MB_RTU, RS485_7_SLAVE_ADDR,
                           RS485_7_PORT_INDEX, baudrate,
                           MB_PAR_NONE) != MB_ENOERR) {
            printf("[RS485-7] Slave init FAILED!\n");
            while (1) { vTaskDelay(1000); }
        }

        /* 注册寄存器回调 */
        eMBSRegisterHoldingCB(xMBSlave7, slave7_holding_cb);
        eMBSRegisterInputCB(xMBSlave7, slave7_input_cb);

        printf("[RS485-7] Slave mode started, addr=%u\n",
               RS485_7_SLAVE_ADDR);

        /* 从机任务循环: 轮询 Modbus 请求 */
        while (1) {
            eMBErrorCode eStatus;

            eStatus = eMBSPoll(xMBSlave7);

            if (eStatus == MB_ENOERR) {
                /* 处理了一帧请求 → 闪烁 LED */
                LED_485_Set(0, TOGGLE);
                vTaskDelay(pdMS_TO_TICKS(500));
            } else {
                /* 通信失败: LED 灭, 等待重试 */
                LED_485_1_OFF;
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}

/* ============================================================================
 *  rs485_7_task_create  —  创建第7路 RS485 任务
 *  ==========================================================================*/
void rs485_7_task_create(void)
{
    TaskHandle_t xHandle = NULL;
    BaseType_t xReturn;

    xReturn = xTaskCreate(rs485_7_task_handler,
                          "RS485_7",
                          RS485_7_TASK_STACK,
                          NULL,
                          RS485_7_TASK_PRIO,
                          &xHandle);

    if (xReturn == pdPASS) {
        printf("Create RS485_7_Task success...\r\n");
    } else {
        printf("Create RS485_7_Task FAILED!\r\n");
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
