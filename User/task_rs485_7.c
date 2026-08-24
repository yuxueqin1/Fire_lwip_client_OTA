/**
  ******************************************************************************
  * @file    task_rs485_7.c
  * @author  fire
  * @version V2.0
  * @date    2026-08-03
  * @brief   第7路RS485 串口任务 (USART3, 物理端口索引6)
  *          根据拨码开关配置运行于 Modbus 主机或从机模式
  *
  *  主机模式业务逻辑:
  *    1. 计算本板报警状态 (基于6路端口的 ALARM 变量)
  *    2. FC 03 读取从机通信板报警状态 (Modbus地址1)
  *    3. FC 02 读取IO联网模块 DI0(切断开关)/DI1(断接开关) (Modbus地址4)
  *    4. 根据报警真值表计算 蜂鸣器/报警灯/工作灯 状态
  *    5. FC 05 写入IO联网模块 DO0(工作灯)/DO1(蜂鸣器)/DO2(报警灯)
  *
  *  从机模式业务逻辑:
  *    1. 计算本板报警状态
  *    2. FC 03/04 回调中暴露报警状态给主机查询
  *
  *  报警真值表 (来源: Doc/和串联io联网模块的通信概述参考.md):
  *    蜂鸣器(DO1)  = (alarm || break) XOR cut
  *    报警灯(DO2)  = alarm || break || cut
  *    工作灯(DO0)  = 通信未超时(15s内有成功通信)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "task_rs485_7.h"
#include "./dip_switch/bsp_dip_switch.h"
#include "./led/bsp_led.h"
#include "bsp_rs485.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mbm.h"    /* Modbus Master API */
#include "mbs.h"    /* Modbus Slave API */

/* ============================================================================
 *  私有定义
 *  ==========================================================================*/
#define RS485_7_PORT_INDEX      6U     /* USART3 在 mbportserial 中的索引 */
#define RS485_7_SLAVE_ADDR      1U     /* 本板从机模式下的 Modbus 地址    */
#define RS485_7_TASK_PRIO       4      /* 与现有6路UART同级               */
#define RS485_7_TASK_STACK      384    /* 栈大小 (words)                  */
#define RS485_7_POLL_DELAY_MS   500    /* 主机轮询间隔 (ms)               */
#define RS485_7_INIT_RETRY_MAX        3U     /* 初始化失败快速重试次数     */
#define RS485_7_INIT_RETRY_DELAY_MS   200U   /* 快速重试间隔 (ms)          */
#define RS485_7_INIT_RECOVER_DELAY_MS 5000U  /* 之后慢速自恢复间隔 (ms)    */

/* ============================================================================
 *  文件级静态变量
 *  ==========================================================================*/

/* —— 本板报警（主/从机通用）—— */
static uint8_t  s_local_board_alarm = 0;

/* —— 主机模式专用 —— */
static uint8_t  s_remote_board_alarm = 0;   /* 从机通信板报警状态  */
static uint8_t  s_io_cut_switch     = 0;    /* DI0: 切断开关       */
static uint8_t  s_io_break_switch   = 0;    /* DI1: 断接开关       */
static uint8_t  s_io_comm_fail_cnt  = 0;    /* IO模块通信失败累加  */
static uint8_t  s_work_light        = 0;    /* DO0: 工作灯         */
static uint8_t  s_buzzer            = 0;    /* DO1: 蜂鸣器         */
static uint8_t  s_alarm_light       = 0;    /* DO2: 报警灯         */

/* ============================================================================
 *  全局变量定义 (供外部模块读取串口7的IO状态)
 *  ==========================================================================*/
uint8_t g_rs485_7_local_alarm  = 0;
uint8_t g_rs485_7_remote_alarm = 0;
uint8_t g_rs485_7_io_cut       = 0;
uint8_t g_rs485_7_io_break     = 0;
uint8_t g_rs485_7_work_light   = 0;
uint8_t g_rs485_7_buzzer       = 0;
uint8_t g_rs485_7_alarm_light  = 0;

/* ============================================================================
 *  全局句柄 (用于主/从模式)
 *  ==========================================================================*/
static xMBMHandle xMBMaster7 = NULL;   /* Modbus 主机句柄 */
static xMBSHandle xMBSlave7  = NULL;   /* Modbus 从机句柄 */

/* ============================================================================
 *  板级报警计算
 *  ==========================================================================*/

/**
 * @brief  计算本板报警状态 — 基于从机模块在线状态 + KGL开关量报警
 * @note
 *   报警判定需要同时检查两个维度 (任一触发即报警):
 *     维度1: 模块在线状态 — link_mask 检查
 *           模块离线(Modbus无应答) → link_mask 对应位=0 → 报警
 *     维度2: KGL开关量报警 — ALARM_TEMP 检查
 *           模块在线但 KGL 位(bit0-5)非0 → ALARM_TEMP 对应位=0 → 报警
 *
 *   不能只复用 ALARM 全局变量的原因:
 *     ALARM 只基于 ALARM_TEMP(KGL位), 而 uart_modbus_mark_offline()
 *     不更新 ALARM_TEMP, 导致模块离线时 ALARM 不变化, 无法检测离线.
 *
 *   link_mask 存于 SerialDataBuf[port][7-10], 各端口每轮采集后更新.
 *   初始值=0, 首轮采集前会短暂误报(安全侧, fail-safe).
 *
 * @return 0=全部正常, 1=至少一个模块不在线或KGL报警
 */
static uint8_t compute_local_board_alarm(void)
{
    uint8_t i;

    for (i = 0; i < UART_PORT_COUNT; i++) {
        uint8_t  module_count;
        uint32_t link_mask;
        uint32_t expected_mask;
        uint32_t kgl_alarm_mask;

        /* 跳过未启用或从机模式的端口 */
        if (BoardCfgTab[i].PortType == UART_PORT_DISABLED ||
            BoardCfgTab[i].PortType == UART_PORT_SLAVE) {
            continue;
        }

        module_count = BoardCfgTab[i].SlaveModuelNum;
        if (module_count == 0U) {
            continue;
        }
        if (module_count > 32U) {
            module_count = 32U;
        }

        link_mask     = *( (uint32_t *)&SerialDataBuf[i][7] );
        expected_mask = ( 1UL << module_count ) - 1UL;

        /* ---- 维度1: 模块在线/离线检查 ---- */
        if (link_mask != expected_mask) {
            return 1U;  /* 有模块不在线 → 报警 */
        }

        /* ---- 维度2: KGL 开关量报警检查 ----
         * ALARM_TEMP bit=1 表示该模块无KGL报警, bit=0 表示有KGL报警.
         * 只检查在线模块(在 link_mask 中的)的KGL状态:
         *   ~ALARM_TEMP[i] & link_mask != 0  →  有在线模块触发KGL报警  */
        kgl_alarm_mask = ~ALARM_TEMP[i] & link_mask;
        if (kgl_alarm_mask != 0UL) {
            return 1U;  /* 在线模块有KGL开关量报警 */
        }
    }

    return 0U;  /* 全部正常: 所有模块在线且无KGL报警 */
}

/* ============================================================================
 *  报警真值表判决
 *  ==========================================================================*/

/**
 * @brief  根据报警真值表计算 DO0/DO1/DO2 输出状态
 * @note   真值表逻辑:
 *          蜂鸣器(DO1)  = (alarm || break) XOR cut
 *          报警灯(DO2)  = alarm || break || cut
 *          工作灯(DO0)  = 通信未超时
 */
static void apply_alarm_truth_table(void)
{
    uint8_t combined_alarm;

    combined_alarm = s_local_board_alarm || s_remote_board_alarm;

    /* DO1: 蜂鸣器 = (alarm || break) XOR cut */
    s_buzzer = (combined_alarm || s_io_break_switch) ^ s_io_cut_switch;

    /* DO2: 报警灯 = alarm || break || cut */
    s_alarm_light = combined_alarm || s_io_break_switch || s_io_cut_switch;

    /* DO0: 工作灯 = 通信未超时 */
    s_work_light = (s_io_comm_fail_cnt < IO_COMM_TIMEOUT_CNT) ? 1U : 0U;
}

/* ============================================================================
 *  全局变量同步
 *  ==========================================================================*/

/**
 * @brief  将静态变量同步到全局变量 (供外部模块读取)
 */
static void sync_global_variables(void)
{
    g_rs485_7_local_alarm  = s_local_board_alarm;
    g_rs485_7_remote_alarm = s_remote_board_alarm;
    g_rs485_7_io_cut       = s_io_cut_switch;
    g_rs485_7_io_break     = s_io_break_switch;
    g_rs485_7_work_light   = s_work_light;
    g_rs485_7_buzzer       = s_buzzer;
    g_rs485_7_alarm_light  = s_alarm_light;
}

/* ============================================================================
 *  从机模式: 寄存器回调
 *  ==========================================================================*/

/**
 * @brief  从机保持寄存器回调 (FC 03/06/10)
 * @note   地址0: 本板报警状态 (0=全部在线, 1=有报警)
 *         地址1: ALARM 原始值 (各端口报警位掩码摘要)
 */
static eMBException slave7_holding_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                       USHORT usNRegs, eMBSRegisterMode eRegMode)
{
    USHORT i;

    /* 每次访问时刷新本板报警状态 */
    s_local_board_alarm = compute_local_board_alarm();

    for (i = 0; i < usNRegs; i++) {
        USHORT reg_addr = usAddress + i;

        if (eRegMode == MBS_REGISTER_READ) {
            USHORT val = 0;

            switch (reg_addr) {
            case SLAVE_REG_ALARM_STATUS:
                val = (USHORT)s_local_board_alarm;
                break;
            case SLAVE_REG_ALARM_RAW:
                val = (USHORT)ALARM;
                break;
            default:
                val = 0xFFFFU;  /* 未定义寄存器: 返回0xFFFF */
                break;
            }

            *pubRegBuffer++ = (UCHAR)(val >> 8);
            *pubRegBuffer++ = (UCHAR)(val & 0xFF);
        } else {
            /* 写入模式: 暂不处理 (预留扩展) */
            pubRegBuffer += 2;
        }
    }

    /* 同步全局变量 */
    sync_global_variables();

    return MB_PDU_EX_NONE;
}

/**
 * @brief  从机输入寄存器回调 (FC 04)
 * @note   地址0: 本板报警状态 (只读)
 */
static eMBException slave7_input_cb(UBYTE *pubRegBuffer, USHORT usAddress,
                                     USHORT usNRegs)
{
    USHORT i;
    (void)usAddress;

    /* 每次访问时刷新本板报警状态 */
    s_local_board_alarm = compute_local_board_alarm();

    for (i = 0; i < usNRegs; i++) {
        USHORT val = (USHORT)s_local_board_alarm;
        *pubRegBuffer++ = (UCHAR)(val >> 8);
        *pubRegBuffer++ = (UCHAR)(val & 0xFF);
    }

    /* 同步全局变量 */
    sync_global_variables();

    return MB_PDU_EX_NONE;
}

/* ============================================================================
 *  主机模式: IO模块通信辅助函数
 *  ==========================================================================*/

/**
 * @brief  读取从机通信板报警状态 (FC 03, 地址1)
 * @note   成功时读取寄存器值作为报警状态
 *         通信失败(超时/CRC错等)时直接置 alarm=1:
 *           从机板失联本身就是故障, 应触发报警
 */
static void master_read_slave_board(void)
{
    USHORT regs[2] = {0};
    eMBErrorCode eStatus;

    eStatus = eMBMReadHoldingRegisters(xMBMaster7, (UCHAR)RS485_7_SLAVE_COMM_ADDR,
                                       (USHORT)SLAVE_REG_ALARM_STATUS,
                                       (UBYTE)1, regs);
    if (eStatus == MB_ENOERR) {
        s_remote_board_alarm = (uint8_t)(regs[0] & 0x0001U);
    } else {
        /* 从机板通信失败 → 直接视为报警 */
        s_remote_board_alarm = 1U;
    }
}

/**
 * @brief  读取IO联网模块DI状态 (FC 02, 地址4, DI0-DI1)
 * @note   成功时更新 cut/break 并清零超时计数, 失败时累加超时计数
 */
static void master_read_io_module_di(void)
{
    UBYTE di_buf[1] = {0};
    eMBErrorCode eStatus;

    /* IO模块响应较慢(~90ms), 设置2000ms超时确保有足够余量 */
    (void)eMBMSetSlaveTimeout(xMBMaster7, 2000U);

    eStatus = eMBMReadDiscreteInputs(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                                     (USHORT)IO_MODULE_DI_START,
                                     (USHORT)IO_MODULE_DI_COUNT, di_buf);

    /* 恢复默认超时 */
    (void)eMBMSetSlaveTimeout(xMBMaster7, 1000U);

    if (eStatus == MB_ENOERR) {
        /* DI数据按位打包: LSB of byte[0] = DI0 */
        s_io_cut_switch   = (di_buf[0] >> 0) & 0x01U;   /* DI0: 切断开关 */
        s_io_break_switch = (di_buf[0] >> 1) & 0x01U;   /* DI1: 断接开关 */
        s_io_comm_fail_cnt = 0;                          /* 清零超时计数 */
    } else {
        /* 打印具体错误码帮助排查
         * eStatus: 0=OK, 5=EIO(CRC错), 8=ETIMEDOUT(超时-设备未回复) */
        // printf("[RS485-7] FC02 fail: eStatus=%d (0=OK, 5=EIO, 8=TIMEOUT)\n", (int)eStatus);
        s_io_comm_fail_cnt++;  /* 累加超时计数 */
    }
}

/**
 * @brief  写入IO联网模块DO状态 (FC 05, 地址4, DO0-DO2)
 * @note   逐线圈写入, 失败不重试 (下轮循环自动重写)
 */
static void master_write_io_module_do(void)
{
    /* DO0: 工作灯 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_WORK_LIGHT,
                              (BOOL)(s_work_light ? TRUE : FALSE));

    /* DO1: 蜂鸣器 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_BUZZER,
                              (BOOL)(s_buzzer ? TRUE : FALSE));

    /* DO2: 报警灯 */
    (void)eMBMWriteSingleCoil(xMBMaster7, (UCHAR)RS485_7_IO_MODULE_ADDR,
                              (USHORT)IO_MODULE_DO_ALARM_LIGHT,
                              (BOOL)(s_alarm_light ? TRUE : FALSE));
}

/* ============================================================================
 *  第7路RS485 任务主函数
 *  ==========================================================================*/
static void rs485_7_task_handler(void *pvParameters)
{
    uint32_t baudrate;
    uint8_t  mode;

    (void)pvParameters;

    /* 读取拨码配置 */
    baudrate = g_dip_rs485_7_baudrate;
    mode     = g_dip_rs485_7_mode;   /* 3=Slave, 4=Master */

    printf("[RS485-7] Baud=%lu, Mode=%s\n",
           (unsigned long)baudrate,
           (mode == 4U) ? "Master" : "Slave");

    /* ---- 初始化: 带有限重试 + 周期自恢复 ---- */
    {
        eMBErrorCode eStatus;
        uint32_t     retry = 0U;

        for (;;) {
            eStatus = (mode == 4U)
                ? eMBMSerialInit(&xMBMaster7, MB_RTU, RS485_7_PORT_INDEX,
                                 baudrate, MB_PAR_NONE)
                : eMBSSerialInit(&xMBSlave7, MB_RTU, RS485_7_SLAVE_ADDR,
                                 RS485_7_PORT_INDEX, baudrate, MB_PAR_NONE);

            if (eStatus == MB_ENOERR) {
                break;   /* 初始化成功, 进入业务循环 */
            }

            /* 打印具体错误码便于定位: 2=EINVAL 3=EPORTERR 4=ENORES 7=EAGAIN */
            printf("[RS485-7] %s init FAILED, err=%d, retry=%lu\n",
                   (mode == 4U) ? "Master" : "Slave",
                   (int)eStatus, (unsigned long)retry);

            retry++;
            /* 前几次快速重试(扛瞬态故障), 之后进入慢速自恢复 */
            vTaskDelay(pdMS_TO_TICKS(retry <= RS485_7_INIT_RETRY_MAX
                                     ? RS485_7_INIT_RETRY_DELAY_MS
                                     : RS485_7_INIT_RECOVER_DELAY_MS));
        }
    }

    /* ---- 根据模式进入业务 ---- */
    if (mode == 4U) {
        /* =============================================================
         *  Modbus 主机模式
         * ============================================================= */
        printf("[RS485-7] Master mode started\n");
        printf("[RS485-7]   -> Slave Comm Board addr=%u\n", RS485_7_SLAVE_COMM_ADDR);
        printf("[RS485-7]   -> IO Module addr=%u\n", RS485_7_IO_MODULE_ADDR);

        /* 初始化后延迟, 确保RS485总线稳定 */
        vTaskDelay(pdMS_TO_TICKS(200));

        /* ---- 主机任务循环 ---- */
        while (1) {
            /* 步骤1: 更新本板报警 */
            s_local_board_alarm = compute_local_board_alarm();

            /* 步骤2: 读取从机通信板报警 (FC 03, 地址1) */
            master_read_slave_board();

            /* 步骤3: 读取IO模块DI (FC 02, 地址4) */
            master_read_io_module_di();

            /* 步骤4+5: 报警真值表判决 */
            apply_alarm_truth_table();

            /* 步骤6: 写入IO模块DO (FC 05, 地址4) */
            master_write_io_module_do();

            /* 步骤7: 同步全局变量 */
            sync_global_variables();

            /* LED闪烁指示 */
            // LED_485_Set(0, TOGGLE);

            /* 调试打印 (每10轮打印一次, 避免刷屏) */
            // {
            //     static uint8_t dbg_cnt = 0;
            //     if (++dbg_cnt >= 10U) {
            //         dbg_cnt = 0;
            //         printf("[RS485-7] L=%u R=%u cut=%u brk=%u -> buz=%u light=%u work=%u fail=%u\n",
            //                s_local_board_alarm, s_remote_board_alarm,
            //                s_io_cut_switch, s_io_break_switch,
            //                s_buzzer, s_alarm_light, s_work_light,
            //                s_io_comm_fail_cnt);
            //     }
            // }

            /* 步骤8: 任务延迟 */
            vTaskDelay(pdMS_TO_TICKS(RS485_7_POLL_DELAY_MS));
        }

    } else {
        /* =============================================================
         *  Modbus 从机模式 (默认)
         * ============================================================= */

        /* 注册寄存器回调 */
        eMBSRegisterHoldingCB(xMBSlave7, slave7_holding_cb);
        eMBSRegisterInputCB(xMBSlave7, slave7_input_cb);

        printf("[RS485-7] Slave mode started, addr=%u\n", RS485_7_SLAVE_ADDR);

        /* ---- 从机任务循环 ---- */
        while (1) {
            eMBErrorCode eStatus;

            /* 周期性更新本板报警 (即使没有主机查询也保持最新) */
            s_local_board_alarm = compute_local_board_alarm();
            sync_global_variables();

            /* 轮询 Modbus 请求 */
            eStatus = eMBSPoll(xMBSlave7);

            if (eStatus == MB_ENOERR) {
                /* 成功处理了一帧 → 闪烁 LED */
                // LED_485_Set(0, TOGGLE);
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
                /* 无请求或出错 → 短暂延迟后继续轮询 */
                vTaskDelay(pdMS_TO_TICKS(50));
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
