/**
 *********************************************************************
 * @file    main.c
 * @author  yxq
 * @version V2.0
 * @date    2019-xx-xx  (重构于 2026-06)
 * @brief   FreeRTOS V9.0.0 + STM32 LwIP  —  6-port UART communication board
 *
 * 架构概览:
 *   - AppTaskCreate:    创建所有任务 (UART ×6, TCP Send, WatchDog, Test)
 *   - uart_task_handler:  通用 6 路串口采集 (Modbus Master / Custom Curve)
 *   - App_Task_TcpSend:   独立 TCP 发送任务 (收发分离)
 *   - App_TaskStart:      看门狗管理 + LED 心跳 + 心跳超时检测
 *   - TCP 接收:           client_thread (在 client.c 中), 处理上位机命令
 *   - 日志系统:           SendLogToServer / checkSendLog
 *
 * 优先级设计原则 (数值越大优先级越高):
 *   App_TaskStart 拥有最高优先级(8), 确保看门狗周期性喂狗,
 *     即使 HAL_ETH_Init 忙等待 5s 也不会导致 SP706S 超时复位.
 *   以太网接收路径必须高于所有可能繁忙的应用任务:
 *     tcpip_thread(7) > ETHIN(6) > 应用任务(5/6).
 *     若 ETHIN 低于串口/TcpSend, DMA 接收环来不及清空 → ACK 被丢弃
 *     (RBUS) → TCP 发送窗口永不释放 → 发送任务长期阻塞 (已踩坑, 勿改回).
 *   串口采集优先级最低, 可被网络通信随时抢占.
 *
 * 任务列表:
 *   | 任务名            | 优先级 | 栈(字) | 功能                          |
 *   |-------------------|--------|--------|-------------------------------|
 *   | App_TaskStart     | 8      | 256    | 看门狗喂狗 (最高优先级防超时)  |
 *   | AppTaskCreate     | 7      | 256    | 初始化 + 创建子任务 (用完即删) |
 *   | tcpip_thread      | 7      | 384    | lwIP 内核 (TCPIP_Init 自动创建)|
 *   | ETHIN             | 6      | 256    | 以太网 RX 环读取 (ethernetif.c)|
 *   | td_tcp_client     | 6      | 256    | TCP 连接管理 + 接收 (client.c) |
 *   | App_Task_TcpSend  | 6      | 256    | TCP 发送队列轮询               |
 *   | USART1..USART6    | 5      | 384    | 6 路串口数据采集               |
 *   | RS485_7           | 4      | 384    | 第7路RS485 Modbus主/从机      |
 *   | Test1_Task        | 1      | 128    | LED1 心跳                     |
 *   |
 *   | 注: 栈大小单位 = 字(4B). lwIP 线程栈常数按字节定义
 *   |     (TCPIP_THREAD_STACKSIZE=1536B, td_tcp_client=1024B,
 *   |      NETIF_IN_TASK_STACK_SIZE=1024B),
 *   |     由 sys_thread_new 内部除以 sizeof(StackType_t) 换算为字后创建.
 *********************************************************************
 */

/*
*************************************************************************
*                            INCLUDES
*************************************************************************
*/
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "client.h"
#include "ota_client.h"
#include "bsp_rs485.h"
#include "bsp_eeprom.h"
#include "./dip_switch/bsp_dip_switch.h"
#include "./task_rs485_7.h"
#include <stdint.h>
#include <string.h>

/*======================================================================*/
/*  Task handles                                                        */
/*======================================================================*/
static TaskHandle_t AppTaskCreate_Handle      = NULL;
static TaskHandle_t Test1_Task_Handle         = NULL;
static TaskHandle_t App_Task_TcpSend_Handle   = NULL;
static TaskHandle_t App_TaskStart_Handle      = NULL;

/*======================================================================*/
/*  Forward declarations                                                */
/*======================================================================*/
static void AppTaskCreate(void *pvParameters);
static void Test1_Task(void *pvParameters);

/* ---- TCP & Watchdog tasks ---- */
static void App_Task_TcpSend(void *pvParameters);
static void App_TaskStart(void *pvParameters);

/* ---- UART task handler (generic, used for all 6 ports) ---- */
static void uart_task_handler(void *pvParameters);

/* ---- Port-type processing functions ---- */
static void uart_process_modbus_master(UART_PortRuntime_t *runtime);
static void uart_process_custom_port(UART_PortRuntime_t *runtime);

/* ---- Module-level helpers for Modbus modules ---- */
static void uart_modbus_mark_online(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                    const UART_ModuleProfile_t *profile,
                                    CPU_INT16U *data);
static void uart_modbus_mark_offline(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                     const UART_ModuleProfile_t *profile,
                                     CPU_INT16U **data);

/* ---- Module-level helpers for custom curve modules ---- */
static uint16_t uart_custom_module_exchange(uint8_t port_idx, uint8_t module_idx);
static void uart_custom_module_online(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                      const UART_ModuleProfile_t *profile,
                                      uint16_t rx_len, uint16_t *dc_offset);
static void uart_custom_module_offline(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                       const UART_ModuleProfile_t *profile,
                                       uint16_t *dc_offset);

/* ---- Frame flush: send buffered data to TCP ---- */
static void uart_flush_realtime_frame(uint8_t port_idx, uint16_t len, uint8_t reset_flag);

/* ---- Watchdog / port-error polling ---- */
static void uart_check_port_error_and_arm(UART_PortRuntime_t *runtime);

/* ---- Complete curve-frame dispatch (called when CRC is valid) ---- */
static void uart_handle_curve_frame_dispatch(uint8_t port_idx, uint8_t module_idx,
                                             const UART_ModuleProfile_t *profile,
                                             uint8_t *ser_buf);

/* ---- Axle counter protocol (0x07) ---- */
static void     uart_process_axle_counter(UART_PortRuntime_t *runtime);
static uint16_t uart_axle_counter_module_exchange(uint8_t port_idx, uint8_t module_idx);
static void     uart_axle_counter_online(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                         uint16_t rx_len, uint16_t *dc_offset);
static void     uart_axle_counter_offline(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                          uint16_t *dc_offset);
static void     uart_axle_counter_upload_pulse(uint8_t port_idx, uint8_t module_idx,
                                               const uint8_t *pulse_data,
                                               uint16_t pulse_len, uint32_t link_mask);

/* ---- DC 3-wire protocol (0x08) ---- */
static void     uart_process_dc3wire(UART_PortRuntime_t *runtime);
static uint16_t uart_dc3wire_module_exchange(uint8_t port_idx, uint8_t module_idx);
static void     uart_dc3wire_online(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                    uint16_t rx_len, uint16_t *dc_offset);
static void     uart_dc3wire_offline(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                     uint16_t *dc_offset);
static void     uart_dc3wire_upload_curve(uint8_t port_idx, uint8_t module_idx,
                                          const uint8_t *resp, uint32_t link_mask);

/* ---- Modbus slave callbacks ---- */
static eMBException eMBSRegHoldingCB(UBYTE *pubRegBuffer, USHORT usAddress,
                                      USHORT usNRegs, eMBSRegisterMode eRegMode);
static eMBException eMBSRegInputCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs);

/* ---- Modbus registers ---- */
static USHORT MBRegHoldingBuf[MB_REG_HOLDING_NREGS] = {0};
static USHORT MBRegInputBuf[MB_REG_INPUT_NREGS]    = {0};

extern void TCPIP_Init(void);


/*======================================================================*/
/*  main()                                                              */
/*======================================================================*/
int main(void)
{
    BaseType_t xReturn = pdPASS;

    BSP_Init();

    xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,
                          (const char    *)"AppTaskCreate",
                          (uint16_t       )256,
                          (void          *)NULL,
                          (UBaseType_t    )7,
                          (TaskHandle_t  *)&AppTaskCreate_Handle);

    if (pdPASS == xReturn) {
        vTaskStartScheduler();
    }

    return -1;
}


/*======================================================================*/
/*  AppTaskCreate  —  初始化子系统, 创建全部任务                           */
/*======================================================================*/
static void AppTaskCreate(void *pvParameters)
{
    BaseType_t xReturn = pdPASS;
    int i;

    (void)pvParameters;

    /*
     * ================================================================
     * 步骤 1: 先创建所有工作线程 (低优先级)
     * ================================================================
     * 这些任务创建后会立即阻塞在各自的等待点上:
     *   - UART 任务: uart_wait_until_configured → vTaskDelay(100ms)
     *   - TcpSend:   client_send_pending (tcp_conn==NULL时立即返回) → vTaskDelay(10ms)
     *   - App_TaskStart: checkSendLog → vTaskDelay(500ms)
     *
     * 由于 AppTaskCreate 本身优先级为 7, 高于所有工作线程和网络线程,
     * 不会被任何新创建的子任务抢占, 初始化过程可以一气呵成.
     * ================================================================
     */

    /* ---- Test / heartbeat tasks (prio=1) ---- */
    xReturn = xTaskCreate((TaskFunction_t)Test1_Task,
                          (const char    *)"Test1_Task",
                          (uint16_t       )128,
                          (void          *)NULL,
                          (UBaseType_t    )1,
                          (TaskHandle_t  *)&Test1_Task_Handle);
    if (pdPASS == xReturn)
        printf("Create Test1_Task success...\r\n");
    /* ---- 6 UART port tasks (prio=5) ---- */
    for (i = 0; i < UART_PORT_COUNT; i++) {
        xReturn = xTaskCreate((TaskFunction_t)uart_task_handler,
                              (const char    *)uart_hw_config[i].name,
                              (uint16_t       )384,
                              (void          *)(intptr_t)i,
                              (UBaseType_t    )5,
                              (TaskHandle_t  *)&Serial_Task_Handles[i]);
        if (pdPASS == xReturn) {
            printf("Create %s_Task success...\r\n", uart_hw_config[i].name);
        }
    }

    /* ---- App_Task_TcpSend (prio=5) ---- */
    xReturn = xTaskCreate((TaskFunction_t)App_Task_TcpSend,
                          (const char    *)"App_Task_TcpSend",
                          (uint16_t       )256,
                          (void          *)NULL,
                          (UBaseType_t    )6,  
                          (TaskHandle_t  *)&App_Task_TcpSend_Handle);
    if (pdPASS == xReturn)
        printf("Create App_Task_TcpSend success...\r\n");

    /* ---- App_TaskStart (prio=8) ---- */
    xReturn = xTaskCreate((TaskFunction_t)App_TaskStart,
                          (const char    *)"App_TaskStart",
                          (uint16_t       )256,
                          (void          *)NULL,
                          (UBaseType_t    )8,  
                          (TaskHandle_t  *)&App_TaskStart_Handle);
    if (pdPASS == xReturn)
        printf("Create App_TaskStart success...\r\n");

    /*
     * ================================================================
     * 步骤 2: 所有工作线程已就绪，现在初始化网络 (高优先级)
     * ================================================================
     * UART / TcpSend / App_TaskStart 等任务已全部创建完成并阻塞在
     * 各自的等待点上. 现在启动网络, tcpip_thread (prio=7) 和
     * client_thread (prio=6) 可以安全地建立 TCP 连接.
     * 网络线程优先级高于工作线程, 确保 TCP/IP 协议栈不会饥饿.
     */

    /* ---- 初始化网络协议栈 (创建 tcpip_thread, prio=7) ---- */
    TCPIP_Init();

    /* ---- 初始化 TCP 客户端 (创建 client_thread, prio=6) ---- */
    client_init();

    /* ---- 打印拨码开关配置 ---- */
    printf("\n[DIP] Board IP: 192.168.1.%u\n", g_dip_ip_last_octet);
    printf("[DIP] Port7 baud: %lu, Mode: %s\n",
           (unsigned long)g_dip_rs485_7_baudrate,
           (g_dip_rs485_7_mode == 4U) ? "Master" : "Slave");

    /* ---- 创建第7路 RS485 任务 (根据拨码初始化为主/从机) ---- */
    rs485_7_task_create();


    /* ---- 初始化 OTA 客户端 ---- */
    // ota_client_init();

    /*
     * ================================================================
     * 步骤 3: 初始化完成，删除本任务
     * ================================================================
     * 注意: vTaskDelete(NULL) 必须放在最后，且不能在临界区内调用，
     * 否则 taskYIELD 不生效，调度器状态混乱。
     */
    vTaskDelete(NULL);  /* 删除自己，此函数不会返回 */
}


/*======================================================================*/
/*  Test1_Task  —  LED1 心跳 (1Hz)                                       */
/*======================================================================*/
static void Test1_Task(void *parameter)
{
    (void)parameter;
    while (1) {
        
        LED_RUN_TOGGLE;
        vTaskDelay(1000);
    }
}


/*======================================================================*/
/*  App_Task_TcpSend  —  TCP 发送任务 (独立的发送线程)                    */
/*                                                                       */
/*  与原程序 App_Task_TcpSend 功能一致:                                   */
/*  - 周期遍历 TCP 发送队列 (6路端口数据 + 1路控制通道 = 7个槽位)          */
/*  - 调用 client_send_pending() 执行实际的 lwIP netconn_write            */
/*  - 每次循环清零 WDTTCPSENDFLAG (看门狗计数器)                          */
/*  - 发送失败时 client_send_pending 内部会断开 TCP 连接 + 设置 WTD_RESET  */
/*                                                                       */
/*  收发分离的好处:                                                       */
/*  - 发送不阻塞接收 (上位机命令能及时处理)                                */
/*  - 接收不阻塞发送 (采集数据能及时上传)                                  */
/*======================================================================*/
static void App_Task_TcpSend(void *pvParameters)
{
    (void)pvParameters;

    while (1) {
        /*
         * 清零 TCP 发送任务看门狗计数器.
         * App_TaskStart 会周期性累加此计数器, 如果超过阈值则触发复位.
         * 此处清零表示"发送任务仍在正常运行".
         */
        WDTTCPSENDFLAG = 0;
        

        /*
         * 调用 client.c 的发送函数:
         * - 遍历 7 个发送槽位 (port 0-5 数据 + 控制通道)
         * - 对每个 ready=1 的槽位, 通过 netconn_write 发送
         * - 发送成功后清零槽位 (ready=0)
         * - 发送失败重试 TCP_SEND_RETRY_COUNT 次后断开连接
         */
        client_send_pending();

        /*
         * 延迟 10ms.
         * 相当于原程序的 osDelay(10), 保证发送及时性的同时不过度占用 CPU.
         */
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/*======================================================================*/
/*  App_TaskStart  —  看门狗管理 + LED 心跳 + 日志检查                    */
/*                                                                       */
/*  与原程序 App_TaskStart 功能一致. 每 10ms 执行一次:                     */
/*                                                                       */
/*  1. LED 闪烁:                                                         */
/*     - 配置已下载 (download_flag=1): 每 500ms 翻转 LED                  */
/*     - 配置未下载:                    每 100ms 翻转 LED                  */
/*                                                                       */
/*  2. 报警灯控制:                                                        */
/*     - ALARM != 0 → 点亮报警灯 (GPIO_PIN_11)                            */
/*                                                                       */
/*  3. 看门狗计数器:                                                      */
/*     - WDTUART0-5FLAG:     各串口任务清零, 此处累加. >500 → 复位         */
/*     - WDTTCPSENDFLAG:     TCP 发送任务清零, 此处累加. >500 → 复位       */
/*     - heartbeat:          收到心跳应答时清零, 此处累加. 超限 → 复位     */
/*                                                                       */
/*  4. 喂狗 / 复位:                                                       */
/*     - WTD_RESET == 0: 喂狗 (WatchDog_Clear)                            */
/*     - WTD_RESET != 0: 不喂狗 → 看门狗超时 → 系统复位                   */
/*                        同时设置 logWTD_RESET 标志 (触发复位日志上报)     */
/*                                                                       */
/*  5. 日志检查: checkSendLog()                                           */
/*======================================================================*/
static void App_TaskStart(void *pvParameters)
{
    uint16_t number = 3000U; /* 心跳超时阈值 (×10ms)      */
    uint8_t  i;

    (void)pvParameters;

    pStart = 1U;  /* 标记系统启动, 触发启动日志上报 */

    while (1) {
        heartbeat++;

        /* ---- 累加各端口 WDT 计数器 ---- */
        for (i = 0; i < UART_PORT_COUNT; i++) {
            WDTUARTFLAG[i]++;
            if (WDTUARTFLAG[i] > 500U) {
                /* 端口 i 的串口任务超时未喂狗 → 触发复位 */
                WTD_RESET = (uint8_t)(21U + i);  /* 复位码 21-26 */
            }
        }

        /* ---- 心跳超时检测 ---- */
        if (heartbeat > number) {
            WTD_RESET = 27U;  /* 心跳超时: 上位机长时间无应答 */
        }

        /* ---- 累加 TCP 发送 WDT 计数器 ---- */
        WDTTCPSENDFLAG++;
        if (WDTTCPSENDFLAG > 500U) {
            WTD_RESET = 28U;  /* TCP 发送任务超时 */
        }


        /* ---- 看门狗处理 ---- */
        if (WTD_RESET == 0U) {
            /* 所有模块正常 → 喂狗 */
            WatchDog_Clear();
        } else {
            /*
             * 有模块异常 → 不喂狗, 等待看门狗复位.
             * 设置 logWTD_RESET 标志, 复位前通过 checkSendLog 上报日志.
             * LED_5 闪烁 = 看门狗复位指示.
             */
            LED_485_5_TOGGLE;
            if (logWTD_RESET == 0U) {
                logWTD_RESET = 1U;
            }
        }

        /* ---- 检查并发送日志 ---- */
        // checkSendLog();  //test：关闭日志

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


/*======================================================================*/
/*  uart_task_handler  —  通用 UART 任务 (驱动全部 6 路端口)               */
/*                                                                       */
/*  初始化流程:                                                           */
/*  1. 等待配置下发 (download_flag + PortType 检查)                       */
/*  2. 初始化串口 (Modbus RTU 或 Custom RAW)                              */
/*  3. 设置数据帧头                                                       */
/*                                                                       */
/*  主循环:                                                               */
/*  1. 清零 WDT 计数器                                                    */
/*  2. 根据 PortType 分发:                                                */
/*     - UART_PORT_MODBUS_MASTER → uart_process_modbus_master()           */
/*     - UART_PORT_CUSTOM        → uart_process_custom_port()             */
/*  3. 报警 GPIO 控制                                                     */
/*  4. 端口错误灯控制                                                     */
/*  5. 校时跟踪                                                          */
/*  6. 端口错误检测与恢复 (uart_check_port_error_and_arm)                  */
/*  7. 数据帧刷新到 TCP 发送队列                                          */
/*  8. 延迟 (端口轮询周期)                                                */
/*======================================================================*/
static void uart_task_handler(void *pvParameters)
{
    uint8_t            port_idx = (uint8_t)(uintptr_t)pvParameters;
    uint32_t           baud_rate;
    UART_PortRuntime_t runtime;

    if (port_idx >= UART_PORT_COUNT) {
        vTaskDelete(NULL);
    }

    memset(&runtime, 0, sizeof(runtime));
    runtime.port_idx     = port_idx;
    runtime.holding_base = uart_holding_base(port_idx);
    runtime.ssz_buf      = (uint16_t *)uart_ssz_ptr(port_idx);
    runtime.qx_buf       = (uint16_t *)uart_qx_ptr(port_idx, 0);

    /*
     * 等待配置下发完成.
     * BoardCfgTab[port_idx].PortType == 0 (未配置) 或 3 (从机模式) 时不启动.
     * SlaveModuelNum == 0 (该端口无从机) 时也不启动.
     * download_flag == 0 (配置未下载) 时也不启动.
     */
    uart_wait_until_configured(port_idx);
    

    /* ---- 初始化串口硬件 ---- */
    baud_rate = uart_get_baud_rate(BoardCfgTab[port_idx].BautRate);
    (void)uart_init_port(port_idx, baud_rate, &runtime.xMBMaster);
    uart_setup_header((int)port_idx);

    while (1) {
        uint32_t round_start = HAL_GetTick();   /* 记录本轮开始, 用于补齐轮询周期 */
        uint32_t poll_start;                     /* 打点: 轮询起始 */
        uint32_t poll_ms;                        /* 打点: 轮询耗时 */
        uint32_t round_ms;                       /* 打点: 整轮耗时 */

        LED_485_Set(port_idx, TOGGLE);  /* 485 通道通信闪烁指示 */
        /* ---- 清零本端口 WDT 计数器 (告知 App_TaskStart 本任务存活) ---- */
        WDTUARTFLAG[port_idx] = 0;
        runtime.data_len = 0;

        /* 无从机 → 跳过采集, 仅延迟 */
        if (BoardCfgTab[port_idx].SlaveModuelNum == 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        /* ---- 根据端口类型分派处理 ---- */
        poll_start = HAL_GetTick();   /* 打点: 轮询开始 */
        switch (BoardCfgTab[port_idx].PortType) {

        case UART_PORT_MODBUS_MASTER:
            /* Modbus RTU 主站模式: 信号机/轨道/发送/开关量 */
            uart_process_modbus_master(&runtime);
            break;

        case UART_PORT_CUSTOM:
            /* 自定义串口模式: 交流转辙机/直流四线制/直流六线制曲线模块 */
            uart_process_custom_port(&runtime);
            break;

        case UART_PORT_AXLE_COUNTER:
            /* 计轴传感器端口: 非标485协议 (通讯确认码 + 脉冲数据) */
            uart_process_axle_counter(&runtime);
            break;

        case UART_PORT_DC3WIRE:
            /* 直流3对线端口: 非标485协议 (曲线序号ACK + 75点曲线数据) */
            uart_process_dc3wire(&runtime);
            break;

        default:
            break;
        }
        poll_ms = HAL_GetTick() - poll_start;   /* 打点: 轮询结束 */

        // /*==================================================================*/
        // /*  报警 GPIO 控制                                                    */
        // /*  ALARM_TEMP != 0xFFFFFFFF → 有报警 → GPIO 置高                    */
        // /*  ALARM |= (1 << port_idx) 或 &= ~(1 << port_idx)                  */
        // /*==================================================================*/
        // if (ALARM_TEMP[port_idx] != 0xFFFFFFFFUL) {
        //     // GPIO_PinWrite(2, (uint8_t)(2U - port_idx % 3U), 1);
        //     ALARM |= (uint8_t)(1U << port_idx);
        // } else {
        //     // GPIO_PinWrite(2, (uint8_t)(2U - port_idx % 3U), 0);
        //     ALARM &= (uint8_t)(~(1U << port_idx));
        // }

        // /*==================================================================*/
        // /*  端口错误灯控制                                                    */
        // /*  链路掩码 != 全在线 → 点亮错误灯                                    */
        // /*==================================================================*/
        // if ((*((uint32_t *)&SerialDataBuf[port_idx][7]))
        //         != ((1UL << BoardCfgTab[port_idx].SlaveModuelNum) - 1UL)) {
        //     // GPIO_PinWrite(2, (uint8_t)(7U - port_idx % 3U), 1);
        // } else {
        //     // GPIO_PinWrite(2, (uint8_t)(7U - port_idx % 3U), 0);
        // }

        /*==================================================================*/
        /*  校时跟踪                                                          */
        /*  当本端口所有模块都已发送校时命令后, 清除 changetime 标志           */
        /*==================================================================*/
        if (TaskAdjustNum[port_idx] >= BoardCfgTab[port_idx].SlaveModuelNum
            && changetime[port_idx] != 0) {
            changetime[port_idx] = 0;
        }

        /*==================================================================*/
        /*  端口错误检测与恢复                                                */
        /*  当端口上全部从机都离线时, 尝试重新初始化串口                       */
        /*==================================================================*/
        uart_check_port_error_and_arm(&runtime);

        /*==================================================================*/
        /*  数据帧刷新到 TCP 发送队列                                         */
        /*  - Custom 端口: 从 SDRAM 拷贝实时数据, 设置帧头, 入队               */
        /*  - Modbus 端口: 数据已在 SerialDataBuf 中, 直接入队                */
        /*==================================================================*/
        switch (BoardCfgTab[port_idx].PortType) {
        case UART_PORT_CUSTOM:
            /* Custom port: 长度 = data_len + 4 (状态位), 帧标志 = 0x01 (实时帧) */
            *((uint16_t *)&SerialDataBuf[port_idx][5]) = runtime.data_len + 4U;
            SerialDataBuf[port_idx][3] = 0x01;
            memcpy(&SerialDataBuf[port_idx][11], runtime.ssz_buf, runtime.data_len);
            client_add_to_tcp_send_buf(&SerialDataBuf[port_idx][0],
                                       (uint16_t)(runtime.data_len + 11U),
                                       19U, port_idx);
            break;

        case UART_PORT_MODBUS_MASTER:
            /* Modbus port: 使用 uart_flush_realtime_frame 统一设置帧标志 */
            uart_flush_realtime_frame(port_idx, runtime.data_len,
                                      20U + port_idx * 6U);
            break;

        case UART_PORT_AXLE_COUNTER:
            /* 计轴端口实时帧: 帧标志 = 0x01 (区别于脉冲帧 0x09) */
            *((uint16_t *)&SerialDataBuf[port_idx][5]) = runtime.data_len + 4U;
            SerialDataBuf[port_idx][3] = 0x01;
            memcpy(&SerialDataBuf[port_idx][11], runtime.ssz_buf, runtime.data_len);
            {
                if (g_debug_axle_enable) {
                    uint8_t _m;
                    printf("[AXLE-DBG] port=%d mask=0x%08lX len=%u",
                           port_idx + 1, runtime.link_mask, runtime.data_len);
                    for (_m = 0; _m < BoardCfgTab[port_idx].SlaveModuelNum; _m++) {
                        uint16_t _off = (uint16_t)(_m * AXLE_REALTIME_REGS);
                        printf(" | m%d[addr=%u reg0=%04X reg1=%04X reg2=%04X]",
                               _m,
                               BoardCfgTab[port_idx].SlaveModuelAddress[_m],
                               runtime.ssz_buf[_off],
                               runtime.ssz_buf[_off + 1],
                               runtime.ssz_buf[_off + 2]);
                    }
                    printf("\r\n");
                }
            }
            client_add_to_tcp_send_buf(&SerialDataBuf[port_idx][0],
                                       (uint16_t)(runtime.data_len + 11U),
                                       20U + port_idx * 6U, port_idx);
            break;

        case UART_PORT_DC3WIRE:
            /* 直流3对线端口实时帧: 帧标志 = 0x01 (区别于曲线帧 0x0B/0x0A) */
            *((uint16_t *)&SerialDataBuf[port_idx][5]) = runtime.data_len + 4U;
            SerialDataBuf[port_idx][3] = 0x01;
            memcpy(&SerialDataBuf[port_idx][11], runtime.ssz_buf, runtime.data_len);
            {
                if (g_debug_dc3w_enable &&
                    (g_debug_dc3w_port == 0U || g_debug_dc3w_port == (uint8_t)(port_idx + 1U))) {
                    uint8_t _m;
                    printf("[DC3W-DBG] port=%d mask=0x%08lX len=%u",
                           port_idx + 1, runtime.link_mask, runtime.data_len);
                    for (_m = 0; _m < BoardCfgTab[port_idx].SlaveModuelNum; _m++) {
                        uint16_t _off = (uint16_t)(_m * DC3WIRE_REALTIME_REGS);
                        printf(" | m%d[addr=%u reg0=%04X reg1=%04X reg2=%04X]",
                               _m,
                               BoardCfgTab[port_idx].SlaveModuelAddress[_m],
                               runtime.ssz_buf[_off],
                               runtime.ssz_buf[_off + 1],
                               runtime.ssz_buf[_off + 2]);
                    }
                    printf("\r\n");
                }
            }
            client_add_to_tcp_send_buf(&SerialDataBuf[port_idx][0],
                                       (uint16_t)(runtime.data_len + 11U),
                                       20U + port_idx * 6U, port_idx);
            break;

        default:
            break;
        }

        /* 延迟到目标轮询周期: 补齐剩余时间 (替代固定 UART_delay 尾巴) */
        {
            uint32_t elapsed = HAL_GetTick() - round_start;
            uint32_t remain  = (elapsed < UART_POLL_PERIOD_MS)
                               ? (UART_POLL_PERIOD_MS - elapsed) : 0U;
            vTaskDelay(pdMS_TO_TICKS(remain));
        }

        /* 打点: 整轮耗时异常 (>1.5s) 时打印, 用于定位阻塞点 */
        round_ms = HAL_GetTick() - round_start;
        if (round_ms > 1500UL) {
            printf("[ROUND] port=%u poll=%lums round=%lums\r\n",
                   (unsigned)port_idx, (unsigned long)poll_ms,
                   (unsigned long)round_ms);
        }
    }
}


/*======================================================================*/
/*  uart_process_modbus_master                                           */
/*  遍历从机模块, 读取 Modbus 输入寄存器, 更新全局状态.                    */
/*  每个模块的行为由模块描述表 (uart_module_profiles[]) 驱动.              */
/*======================================================================*/
static void uart_process_modbus_master(UART_PortRuntime_t *runtime)
{
    uint8_t                   m;
    uint8_t                   module_count;
    CPU_INT16U               *data;
    const UART_ModuleProfile_t *profile;

    if ((runtime == NULL) || (runtime->xMBMaster == NULL)) {
        return;
    }

    module_count = BoardCfgTab[runtime->port_idx].SlaveModuelNum;
    if (module_count > UART_MODULE_MAX_COUNT) {
        module_count = UART_MODULE_MAX_COUNT;
    }

    data = (CPU_INT16U *)&SerialDataBuf[runtime->port_idx][11];
    runtime->link_mask = 0;

    for (m = 0; m < module_count; m++) {
        profile = uart_get_module_profile(
            BoardCfgTab[runtime->port_idx].SlaveModuelType[m]);

        if (MB_ENOERR == eMBMReadInputRegisters(
                runtime->xMBMaster,
                BoardCfgTab[runtime->port_idx].SlaveModuelAddress[m],
                0,
                profile->input_regs,
                data)) {
            uart_modbus_mark_online(runtime, m, profile, data);
            data += profile->input_regs;
        } else {
            uart_modbus_mark_offline(runtime, m, profile, &data);
        }

        runtime->data_len += (uint16_t)(profile->input_regs * 2U);
        WDTUARTFLAG[runtime->port_idx] = 0;
        vTaskDelay(pdMS_TO_TICKS(5));  
    }

    *((uint32_t *)&SerialDataBuf[runtime->port_idx][7]) = runtime->link_mask;

    /* 校时跟踪: 每轮所有模块都发送了校时命令, TaskAdjustNum 累加 */
    if (changetime[runtime->port_idx] != 0) {
        TaskAdjustNum[runtime->port_idx] += module_count;
    }
}


/*======================================================================*/
/*  uart_modbus_mark_online                                              */
/*  模块 Modbus 读取成功时调用: 清除错误计数, 设置在线位, 更新报警,        */
/*  拷贝 holding 寄存器.                                                  */
/*======================================================================*/
static void uart_modbus_mark_online(UART_PortRuntime_t *runtime,
                                    uint8_t module_idx,
                                    const UART_ModuleProfile_t *profile,
                                    CPU_INT16U *data)
{
    uint16_t holding_pos;
    uint16_t copy_regs;

    runtime->module_error[module_idx] = 0;
    runtime->link_mask |= (1UL << module_idx);

    MBRegHoldingBuf[runtime->holding_base] |= (1U << module_idx);

    /* 报警: 状态寄存器的 bit0-5, data[1] & 0x3F == 0 表示无报警 */
    if ((profile->has_alarm != 0U) && ((*(data + 1) & 0x3fU) == 0U)) {
        ALARM_TEMP[runtime->port_idx] |= (1UL << module_idx);
        MBRegHoldingBuf[runtime->holding_base + 1U] |= (1U << module_idx);
    } else {
        ALARM_TEMP[runtime->port_idx] &= ~(1UL << module_idx);
        MBRegHoldingBuf[runtime->holding_base + 1U] &= ~(1U << module_idx);
    }

    /* 拷贝数据寄存器 (跳过状态+报警, 即 data+2) 到 holding 区域 */
    holding_pos = (uint16_t)(runtime->holding_base + 2U
                             + module_idx * UART_MODULE_DEFAULT_REGS);
    copy_regs = profile->holding_regs;
    if ((holding_pos + copy_regs) > MB_REG_HOLDING_NREGS) {
        copy_regs = (uint16_t)(MB_REG_HOLDING_NREGS - holding_pos);
    }
    if ((copy_regs > 0U) && (profile->input_regs > 2U)) {
        memcpy(&MBRegHoldingBuf[holding_pos], data + 2,
               copy_regs * sizeof(USHORT));
    }
}


/*======================================================================*/
/*  uart_modbus_mark_offline                                             */
/*  模块 Modbus 读取失败时调用: 连续 3 次失败后标记离线, 填充 0xFFFF,       */
/*  清除 holding 寄存器.                                                  */
/*======================================================================*/
static void uart_modbus_mark_offline(UART_PortRuntime_t *runtime,
                                     uint8_t module_idx,
                                     const UART_ModuleProfile_t *profile,
                                     CPU_INT16U **data)
{
    uint8_t  n;
    uint16_t holding_pos;
    uint16_t clear_regs;

    if (runtime->module_error[module_idx] > 2U) {
        runtime->module_error[module_idx] = 0;
        runtime->link_mask &= ~(1UL << module_idx);
        MBRegHoldingBuf[runtime->holding_base] &= ~(1U << module_idx);
    } else {
        runtime->module_error[module_idx]++;
    }

    MBRegHoldingBuf[runtime->holding_base + 1U] &= ~(1U << module_idx);

    /* 填充输入数据为 0xFFFF (离线标志) */
    for (n = 0; n < profile->input_regs; n++) {
        *(*data)++ = 0xFFFFU;
    }

    /* 清除本模块 holding 寄存器 */
    holding_pos = (uint16_t)(runtime->holding_base + 2U
                             + module_idx * UART_MODULE_DEFAULT_REGS);
    clear_regs = profile->holding_regs;
    if ((holding_pos + clear_regs) > MB_REG_HOLDING_NREGS) {
        clear_regs = (uint16_t)(MB_REG_HOLDING_NREGS - holding_pos);
    }
    if (clear_regs > 0U) {
        memset(&MBRegHoldingBuf[holding_pos], 0, clear_regs * sizeof(USHORT));
    }
}


/*======================================================================*/
/*  uart_process_custom_port                                             */
/*  处理自定义串口模块 (0x37/0x38/0x39 — 曲线模块).                       */
/*  每个模块: 校时(如需要) → 发送请求 → 等待应答 → CRC校验 → 数据解析      */
/*======================================================================*/
static void uart_process_custom_port(UART_PortRuntime_t *runtime)
{
    uint8_t                   m;
    uint8_t                   module_count;
    const UART_ModuleProfile_t *profile;
    uint16_t                   rx_len;
    uint16_t                   dc_offset = 0;

    if (runtime == NULL) {
        return;
    }

    module_count = BoardCfgTab[runtime->port_idx].SlaveModuelNum;
    if (module_count > UART_MODULE_MAX_COUNT) {
        module_count = UART_MODULE_MAX_COUNT;
    }

    runtime->link_mask = 0;
    runtime->data_len = 0;

    for (m = 0; m < module_count; m++) {
        profile = uart_get_module_profile(
            BoardCfgTab[runtime->port_idx].SlaveModuelType[m]);

        /* ---- 校时: 每个曲线模块只需发送一次校时命令 ---- */
        if (changetime[runtime->port_idx] == 1U) {
            (void)uart_custom_send_adjust_time(
                runtime->port_idx,
                (uint8_t)runtime->port_idx,
                BoardCfgTab[runtime->port_idx].SlaveModuelAddress[m],
                Systemtime);
            TaskAdjustNum[runtime->port_idx]++;
        }

        /* ---- 执行自定义串口交换 (响应 → SerialDataBuf[port][11]) ---- */
        rx_len = uart_custom_module_exchange(runtime->port_idx, m);

        if (rx_len > 0U) {
            /* CRC OK — 标记在线, 解析数据 */
            uart_custom_module_online(runtime, m, profile, rx_len, &dc_offset);
        } else {
            /* CRC fail 或超时 — 标记离线 */
            uart_custom_module_offline(runtime, m, profile, &dc_offset);
        }

        WDTUARTFLAG[runtime->port_idx] = 0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *((uint32_t *)&SerialDataBuf[runtime->port_idx][7]) = runtime->link_mask;
}


/*======================================================================*/
/*  uart_custom_module_exchange                                          */
/*  构建请求帧 [addr, 0x03, 0,0,0, seq, CRC16], 通过 uart_custom_exchange */
/*  发送并接收应答. 响应存储在 SerialDataBuf[port_idx][11] 开始处.         */
/*  返回 rx_len (成功) 或 0 (失败).                                       */
/*======================================================================*/
static uint16_t uart_custom_module_exchange(uint8_t port_idx, uint8_t module_idx)
{
    uint8_t  tx_buf[8];
    uint16_t tx_crc;
    uint16_t rx_len;
    uint16_t rx_crc_calc;
    uint16_t rx_crc_recv;
    uint8_t *resp;   /* 指向 SerialDataBuf 中的响应起始位置 */

    resp = (uint8_t *)&SerialDataBuf[port_idx][11];

    /* 构建请求: [addr, 0x03, 0,0, Frame_sequence(2B), CRC16] */
    {
        uint16_t frame_seq = AC_module_infor[port_idx].Frame_sequence[module_idx];
        tx_buf[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
        tx_buf[1] = 0x03;
        tx_buf[2] = 0x00;
        tx_buf[3] = 0x00;
        tx_buf[4] = (uint8_t)((frame_seq >> 8) & 0xFFU);
        tx_buf[5] = (uint8_t)(frame_seq & 0xFFU);
    }
    tx_crc    = uart_crc16(tx_buf, 6);
    tx_buf[6] = (uint8_t)(tx_crc & 0xFFU);
    tx_buf[7] = (uint8_t)((tx_crc >> 8) & 0xFFU);

    /* 发送 + 接收, 直接写入 SerialDataBuf */
    rx_len = uart_custom_exchange(port_idx, tx_buf, 8,
                                  resp,
                                  (uint16_t)(UART_SERIAL_BUF_SIZE - 11U),
                                  80, 10);
    if (rx_len < 4U) {
        return 0;
    }

    /* CRC16 校验 (对整个响应帧, 排除最后 2 字节 CRC) */
    rx_crc_calc = uart_crc16(resp, rx_len - 2U);
    rx_crc_recv = (uint16_t)((uint16_t)resp[rx_len - 1U] << 8)
                | (uint16_t)resp[rx_len - 2U];

    if (rx_crc_calc != rx_crc_recv) {
        return 0;
    }

    return rx_len;
}


/*======================================================================*/
/*  uart_custom_module_online                                            */
/*  CRC 校验通过. 提取实时值到 SDRAM, 拷贝 holding 寄存器,                */
/*  分发曲线帧处理.                                                       */
/*  响应数据已在 SerialDataBuf[port_idx][11] 处.                          */
/*======================================================================*/
static void uart_custom_module_online(UART_PortRuntime_t *runtime,
                                      uint8_t module_idx,
                                      const UART_ModuleProfile_t *profile,
                                      uint16_t rx_len,
                                      uint16_t *dc_offset)
{
    uint16_t reg_count;
    uint16_t holding_pos;
    uint8_t *resp;    /* 响应起始位置 (SerialDataBuf 中)  */
    uint8_t *ser_buf; /* 完整 SerialDataBuf (用于帧分发)  */

    resp    = (uint8_t *)&SerialDataBuf[runtime->port_idx][11];
    ser_buf = (uint8_t *)&SerialDataBuf[runtime->port_idx][0];

    runtime->module_error[module_idx] = 0;
    runtime->link_mask |= (1UL << module_idx);
    MBRegHoldingBuf[runtime->holding_base] |= (1U << module_idx);

    /*
     * 交换字节序 (大端 → 小端) 并拷贝实时寄存器到 SDRAM.
     * 数据从 resp[3] 开始 (跳过 addr + func + len = 3 字节).
     */
    reg_count = profile->realtime_bytes / 2U;
    memcpy(&runtime->ssz_buf[*dc_offset], &resp[3], profile->realtime_bytes);
    uart_swap16(&runtime->ssz_buf[*dc_offset], reg_count);

    /*
     * 拷贝 holding 寄存器 (从 resp[7] 开始, 跳过 addr+func+len=3 + status+alarm=4).
     */
    holding_pos = (uint16_t)(runtime->holding_base + 2U
                             + module_idx * UART_MODULE_DEFAULT_REGS);
    if (profile->holding_regs > 0U) {
        uint16_t copy = profile->holding_regs;
        if ((holding_pos + copy) > MB_REG_HOLDING_NREGS) {
            copy = (uint16_t)(MB_REG_HOLDING_NREGS - holding_pos);
        }
        memcpy(&MBRegHoldingBuf[holding_pos], &resp[7], copy * 2U);
    }

    *dc_offset += reg_count;
    runtime->data_len += profile->realtime_bytes;

    /*
     * 在曲线帧分发之前更新 SerialDataBuf 中的链路掩码.
     * uart_handle_curve_frame_dispatch 在收到 0xFF 结束帧时会立即通过
     * client_add_to_tcp_send_buf 上传完整曲线数据, 此时 ser_buf[7-10]
     * 必须反映当前在线状态, 否则上位机会看到 link_mask=0x00000000.
     */
    *((uint32_t *)&ser_buf[7]) = runtime->link_mask;

    /* 曲线帧分发 (使用完整 SerialDataBuf 中的绝对偏移) */
    uart_handle_curve_frame_dispatch(runtime->port_idx, module_idx,
                                     profile, ser_buf);

    (void)rx_len;
}


/*======================================================================*/
/*  uart_custom_module_offline                                           */
/*  CRC/超时失败. SDRAM 填充 0xFFFF, 清除 holding 寄存器.                 */
/*======================================================================*/
static void uart_custom_module_offline(UART_PortRuntime_t *runtime,
                                       uint8_t module_idx,
                                       const UART_ModuleProfile_t *profile,
                                       uint16_t *dc_offset)
{
    uint16_t reg_count;
    uint16_t holding_pos;
    uint16_t clear_regs;
    uint16_t i;

    if (runtime->module_error[module_idx] > 3U) {
        runtime->link_mask &= ~(1UL << module_idx);
        MBRegHoldingBuf[runtime->holding_base] &= ~(1U << module_idx);
    } else {
        runtime->module_error[module_idx]++;
    }

    /* SDRAM 区域填充 0xFFFF (离线标志) */
    reg_count = profile->realtime_bytes / 2U;
    for (i = 0; i < reg_count; i++) {
        runtime->ssz_buf[*dc_offset + i] = 0xFFFFU;
    }

    /* 清除 holding 寄存器 */
    holding_pos = (uint16_t)(runtime->holding_base + 2U
                             + module_idx * UART_MODULE_DEFAULT_REGS);
    clear_regs = profile->holding_regs;
    if ((holding_pos + clear_regs) > MB_REG_HOLDING_NREGS) {
        clear_regs = (uint16_t)(MB_REG_HOLDING_NREGS - holding_pos);
    }
    memset(&MBRegHoldingBuf[holding_pos], 0, clear_regs * sizeof(USHORT));

    *dc_offset += reg_count;
    runtime->data_len += profile->realtime_bytes;
}


/*======================================================================*/
/*  uart_handle_curve_frame_dispatch                                     */
/*  从响应数据中读取帧标志字节, 调用对应的曲线帧处理器.                     */
/*  - 0x0000: 普通实时帧 (无曲线数据)                                       */
/*  - 0x8055: 中间曲线帧 (缓存在 SDRAM)                                     */
/*  - 0x80FF: 结束曲线帧 (缓存 + 上传全部曲线)                               */
/*======================================================================*/
static void uart_handle_curve_frame_dispatch(uint8_t port_idx, uint8_t module_idx,
                                             const UART_ModuleProfile_t *profile,
                                             uint8_t *ser_buf)
{
    uint16_t flag_word;
    uint16_t seq_word;
    uint8_t  flash_time[6];
    uint8_t  is_first;
    const uint8_t *resp;  /* 响应数据起始 = ser_buf + 11 (跳过 TCP 帧头) */

    resp = ser_buf + 11U;

    /* 读取 16 位 FrameFlag (大端序: 0x0000=普通帧, 0x8055=中间帧, 0x80FF=结束帧) */
    flag_word = ((uint16_t)resp[profile->curve_flag_offset] << 8)
              | (uint16_t)resp[profile->curve_flag_offset + 1U];

    /* 读取 16 位曲线帧序号 (大端序) */
    seq_word = ((uint16_t)resp[profile->curve_seq_offset] << 8)
             | (uint16_t)resp[profile->curve_seq_offset + 1U];

    /* 首帧检测: 传感器每个曲线周期的第一帧序号总是 1 */
    is_first = (seq_word == 1U) ? 1U : 0U;

    /*
     * 构建 flash_time 头部 (用于曲线存储).
     * flash_time[0]: 从机地址 (DC_ZZJ6 按分支递增)
     * flash_time[1]: 分支号 (AC_ZZJ) 或 5 (DC 类型)
     * flash_time[2-5]: 年月日时分秒
     * 时间戳位于 FrameFlag(2B) + curve_seq(2B) 之后, 即 F+4 ~ F+7
     */
    flash_time[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
    flash_time[2] = resp[profile->curve_flag_offset + 4U];  /* 年/月 */
    flash_time[3] = resp[profile->curve_flag_offset + 5U];  /* 日/时 */
    flash_time[4] = resp[profile->curve_flag_offset + 6U];  /* 分/秒 */
    flash_time[5] = resp[profile->curve_flag_offset + 7U];

    switch (flag_word) {

    case 0x0000U:  /* 普通帧 — 只有实时数据, 清理残留曲线状态 */
        uart_handle_curve_normal_frame(port_idx, module_idx);
        break;

    case 0x8055U:  /* 中间曲线帧 — 缓存数据到 SDRAM */
        uart_handle_curve_continue_frame(port_idx, module_idx, profile,
                                         ser_buf, is_first, flash_time);
        /*
         * 保存曲线编号到 EEPROM (掉电恢复).
         * EEPROM 地址: block = 5 + port_idx * 2 → 5/7/9/11/13/15
         */
        EEPROM_Write(0, (uint32_t)(5U + (uint32_t)port_idx * 2U),
                     (void *)&AC_module_infor[port_idx].Curve_number[0],
                     MODE_8_BIT, CURVE_EEPROM_SIZE);
        break;

    case 0x80FFU:  /* 结束曲线帧 — 缓存最后一段 + 上传全部曲线 */
        uart_handle_curve_end_frame(port_idx, module_idx, profile,
                                    ser_buf,
                                    Weibo_number[port_idx][module_idx]);
        /*
         * 保存曲线编号到 EEPROM (掉电恢复).
         * 结束帧上传完成后也立即保存, 防止上传过程中掉电导致曲线编号丢失.
         */
        EEPROM_Write(0, (uint32_t)(5U + (uint32_t)port_idx * 2U),
                     (void *)&AC_module_infor[port_idx].Curve_number[0],
                     MODE_8_BIT, CURVE_EEPROM_SIZE);
        break;

    default:
        break;
    }
}


/*======================================================================*/
/*  uart_flush_realtime_frame                                            */
/*  设置长度、帧标志, 推入 TCP 发送队列.                                   */
/*======================================================================*/
static void uart_flush_realtime_frame(uint8_t port_idx, uint16_t len,
                                      uint8_t reset_flag)
{
    *((uint16_t *)&SerialDataBuf[port_idx][5]) = (uint16_t)(len + 4U);
    SerialDataBuf[port_idx][3] =
        (BoardCfgTab[port_idx].PortType == UART_PORT_CUSTOM) ? 0x01 : 0x00;
    client_add_to_tcp_send_buf(&SerialDataBuf[port_idx][0],
                               (uint16_t)(len + 11U), reset_flag, port_idx);
}


/*======================================================================*/
/*  uart_check_port_error_and_arm                                        */
/*  当端口上全部从机都离线时, 尝试重新初始化串口 (最多 4 次连续尝试).        */
/*  超过 4 次后触发看门狗复位.                                            */
/*======================================================================*/
static void uart_check_port_error_and_arm(UART_PortRuntime_t *runtime)
{
    uint32_t baud_rate;

    if (*((uint32_t *)&SerialDataBuf[runtime->port_idx][7]) == 0UL) {
        /*
         * 链路掩码全为 0 → 所有从机离线.
         * 累计 port_error, 前 3 次尝试重新初始化串口.
         */
        runtime->port_error++;
        if (runtime->port_error < 4U) {
            if ((BoardCfgTab[runtime->port_idx].PortType == UART_PORT_MODBUS_MASTER) ||
                (BoardCfgTab[runtime->port_idx].PortType == UART_PORT_CUSTOM) ||
                (BoardCfgTab[runtime->port_idx].PortType == UART_PORT_AXLE_COUNTER) ||
                (BoardCfgTab[runtime->port_idx].PortType == UART_PORT_DC3WIRE)) {

                /*
                 * Before re-initializing a Modbus port, close the old handle
                 * to release the serial port slot in the FreeModbus port layer.
                 * Otherwise eMBPSerialInit() finds the old port still registered
                 * (via prvGetSerialPort), returns MB_EAGAIN, causing the new
                 * eMBMSerialInit() to fail and set xMBMaster to NULL.
                 * With xMBMaster == NULL, uart_process_modbus_master() returns
                 * immediately and never sends any Modbus requests again —
                 * a permanent failure that only a board restart can recover from.
                 */
                if (BoardCfgTab[runtime->port_idx].PortType == UART_PORT_MODBUS_MASTER) {
                    if (runtime->xMBMaster != NULL) {
                        eMBMClose(runtime->xMBMaster);
                        runtime->xMBMaster = NULL;
                    }
                }

                /*
                 * Modbus → Custom 切换: 若用户通过上位机将端口从 Modbus
                 * 切换为自定义曲线模式, Modbus 初始化时开启的 UART 中断
                 * (TXEIE/RXNEIE/TCIE) 仍在触发 prvUARTIRQHandler, 会与
                 * uart_custom_exchange 的 HAL_UART_Transmit 阻塞轮询
                 * 发生数据竞争. 必须显式关闭残留的 Modbus 句柄.
                 */
                if (BoardCfgTab[runtime->port_idx].PortType == UART_PORT_CUSTOM) {
                    if (runtime->xMBMaster != NULL) {
                        eMBMClose(runtime->xMBMaster);
                        runtime->xMBMaster = NULL;
                    }
                }

                baud_rate = uart_get_baud_rate(
                    BoardCfgTab[runtime->port_idx].BautRate);
                (void)uart_init_port(runtime->port_idx, baud_rate,
                                     &runtime->xMBMaster);
            }
            /* 标记端口数据错误, 触发日志上报 */
            portError[runtime->port_idx] = 1U;
        }
        /*
         * port_error 达到 4 时不重试, 等待看门狗复位.
         * 此时 portError 仍为 1, checkSendLog 会发送错误日志.
         */
    } else {
        /* 至少有一个从机在线 → 清零错误计数 */
        runtime->port_error = 0;
    }
}




/*======================================================================*/
/*  uart_axle_counter_module_exchange                                    */
/*  构建 0x03 查询帧 [addr,03,comm_confirm(2B),00,00,CRC]                */
/*  发送并接收应答 (写入 SerialDataBuf[port_idx][11])                     */
/*======================================================================*/
static uint16_t uart_axle_counter_module_exchange(uint8_t port_idx, uint8_t module_idx)
{
    uint8_t  tx_buf[8];
    uint16_t tx_crc;
    uint16_t rx_len;
    uint16_t rx_crc_calc, rx_crc_recv;
    uint8_t *resp;
    uint16_t comm_confirm;

    resp = (uint8_t *)&SerialDataBuf[port_idx][11];

    /* 获取上次的通讯确认码 (首次 = 0x0000) */
    comm_confirm = axle_counter_state[port_idx].comm_confirm[module_idx];

    /* 构建请求: [addr, 03, comm_confirm(2B), 00, 00, CRC] */
    tx_buf[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
    tx_buf[1] = 0x03;
    tx_buf[2] = (uint8_t)((comm_confirm >> 8) & 0xFFU);
    tx_buf[3] = (uint8_t)(comm_confirm & 0xFFU);
    tx_buf[4] = 0x00;    /* 脉冲个数 = 查询模式 (不清楚是否有脉冲) */
    tx_buf[5] = 0x00;
    tx_crc    = uart_crc16(tx_buf, 6);
    tx_buf[6] = (uint8_t)(tx_crc & 0xFFU);
    tx_buf[7] = (uint8_t)((tx_crc >> 8) & 0xFFU);

    /* 发送 + 接收 */
    rx_len = uart_custom_exchange(port_idx, tx_buf, 8, resp,
                                  (uint16_t)(UART_SERIAL_BUF_SIZE - 11U),
                                  100, 50); //test：间隔时间改长了，因为计轴模块丢包严重
    if (rx_len < 5U) {
        return 0;
    }

    /* CRC16 校验 */
    rx_crc_calc = uart_crc16(resp, rx_len - 2U);
    rx_crc_recv = (uint16_t)((uint16_t)resp[rx_len - 1U] << 8)
                | (uint16_t)resp[rx_len - 2U];
    if (rx_crc_calc != rx_crc_recv) {
        return 0;
    }

    return rx_len;
}


/*======================================================================*/
/*  uart_axle_counter_upload_pulse                                       */
/*  将传感器原始脉冲数据打包为 TCP 脉冲帧 (帧标志 0x09) 上传              */
/*                                                                       */
/*  TCP 脉冲数据包格式 (协议文档 Table 1.4):                              */
/*    设备ID(1B) + 通道号(1B) + 时间戳秒(4B) + 时间戳毫秒(2B)            */
/*    + 采集标志(1B) + 当前脉冲数量(2B)                                   */
/*    + Σ(脉冲N_时间偏移(2B) + 脉冲N_采样点数(1B) + 采集值[n](1B×n))    */
/*                                                                       */
/*  时间偏移 = ((pulse_ts - curve_ts)×1000 + (pulse_tms - curve_ms)) / 4 */
/*======================================================================*/
static void uart_axle_counter_upload_pulse(uint8_t port_idx, uint8_t module_idx,
                                           const uint8_t *pulse_data,
                                           uint16_t pulse_len, uint32_t link_mask)
{
    uint8_t  *pkt;
    uint16_t  pkt_offset;
    uint16_t  read_offset;
    uint16_t  pkt_pulse_count;
    uint32_t  curve_unix;
    uint16_t  curve_ms;
    uint16_t  p, i;

    if ((pulse_data == NULL) || (pulse_len < 10U)) {
        return;
    }

    /* 读取"本包上报脉冲个数"(前2字节) */
    pkt_pulse_count = ((uint16_t)pulse_data[0] << 8) | (uint16_t)pulse_data[1];
    read_offset = 2;

    /*
     * 从实时数据中提取曲线起始时间戳.
     * 实时数据已由 uart_axle_counter_online 写入 SerialDataBuf[port][11..].
     * 偏移: resp[2+4]=Uinx(4B), resp[2+22]=Uinx_ms(2B). (无byte_count)
     */
    {
        uint8_t *base = (uint8_t *)&SerialDataBuf[port_idx][11];
        curve_unix = ((uint32_t)base[6]  << 24) | ((uint32_t)base[7]  << 16)
                   | ((uint32_t)base[8]  << 8)  |  (uint32_t)base[9];
        curve_ms   = ((uint16_t)base[24] << 8)  |  (uint16_t)base[25];
    }

    /* 脉冲数据包写入 SerialDataBuf 中 (复用缓冲区) */
    pkt = (uint8_t *)&SerialDataBuf[port_idx][11];
    pkt_offset = 0;

    for (p = 0; p < pkt_pulse_count && read_offset < pulse_len; p++) {
        uint32_t pulse_ts;
        uint16_t pulse_tms;
        uint8_t  ch_no;
        uint8_t  pulse_size;
        int32_t  delta_s, delta_ms, total_ms;
        uint32_t time_offset;

        /* ---- 解析传感器脉冲结构 ---- */
        pulse_ts   = ((uint32_t)pulse_data[read_offset]     << 24)
                   | ((uint32_t)pulse_data[read_offset + 1] << 16)
                   | ((uint32_t)pulse_data[read_offset + 2] << 8)
                   |  (uint32_t)pulse_data[read_offset + 3];
        pulse_tms  = ((uint16_t)pulse_data[read_offset + 4] << 8)
                   |  (uint16_t)pulse_data[read_offset + 5];
        ch_no      = pulse_data[read_offset + 6];
        pulse_size = pulse_data[read_offset + 7];
        read_offset += 8;

        if ((uint16_t)(read_offset + pulse_size) > pulse_len) {
            break;
        }

        /* 时间偏移 = ((pulse_ts - curve_unix)×1000 + (pulse_tms - curve_ms)) / 4 */
        delta_s  = (int32_t)(pulse_ts  - curve_unix);
        delta_ms = (int32_t)(pulse_tms - curve_ms);
        total_ms = delta_s * 1000 + delta_ms;
        time_offset = (uint32_t)(total_ms / 4);

        /* ---- 写入 TCP 脉冲数据包条目 ---- */
        pkt[pkt_offset++] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx]; /* 设备ID */
        pkt[pkt_offset++] = ch_no;                          /* 通道号 */
        pkt[pkt_offset++] = (uint8_t)(pulse_ts);            /* 时间戳(秒) 低字节在前 */
        pkt[pkt_offset++] = (uint8_t)(pulse_ts >> 8);
        pkt[pkt_offset++] = (uint8_t)(pulse_ts >> 16);
        pkt[pkt_offset++] = (uint8_t)(pulse_ts >> 24);
        pkt[pkt_offset++] = (uint8_t)(pulse_tms);           /* 时间戳(毫秒) 低字节在前 */
        pkt[pkt_offset++] = (uint8_t)(pulse_tms >> 8);
        pkt[pkt_offset++] = 0x01;                           /* 采集标志 = 1 (完成) */
        pkt[pkt_offset++] = (uint8_t)(pulse_size & 0xFFU);  /* 当前脉冲数量(低) */
        pkt[pkt_offset++] = (uint8_t)((pulse_size >> 8) & 0xFFU); /* 当前脉冲数量(高) */
        pkt[pkt_offset++] = (uint8_t)(time_offset & 0xFFU); /* 时间偏移(低) */
        pkt[pkt_offset++] = (uint8_t)((time_offset >> 8) & 0xFFU); /* 时间偏移(高) */
        pkt[pkt_offset++] = pulse_size;                     /* 采样点数 */
        for (i = 0; i < pulse_size; i++) {
            pkt[pkt_offset++] = pulse_data[read_offset + i]; /* 采集值(×0.1V) */
        }
        read_offset += pulse_size;
    }

    /* ---- 构建完整 TCP 帧: [端口,主协议,子协议,0x09,分机号,len(2B),link_mask(4B),脉冲包] ---- */
    if (pkt_offset > 0U) {
        uint8_t *frame = (uint8_t *)&SerialDataBuf[port_idx][0];
        uint16_t f_len;

        frame[0] = (uint8_t)(port_idx + 1U);
        frame[1] = BoardCfgTab[port_idx].MainProType;
        frame[2] = BoardCfgTab[port_idx].SubProType;
        frame[3] = 0x09;                                     /* 帧标志: 脉冲曲线帧 */
        frame[4] = BoardCfgTab[port_idx].FenjiNum;
        *((uint16_t *)&frame[5]) = (uint16_t)(pkt_offset + 4U); /* 数据长度 */
        *((uint32_t *)&frame[7]) = link_mask;                /* 模块状态 */

        f_len = (uint16_t)(pkt_offset + 11U);
        client_add_curve_to_tcp_send_buf(frame, f_len,
                                         20U + port_idx * 6U);
    }
}


/*======================================================================*/
/*  uart_axle_counter_online                                            */
/*  计轴模块在线处理:                                                    */
/*  1. 清除错误计数, 设置在线位                                          */
/*  2. 提取实时数据 (28 字节) 到 SDRAM                                  */
/*  3. 检查 KGL 报警位, 更新 ALARM_TEMP                                 */
/*  4. 保存通讯确认码 (寄存器 12)                                        */
/*  5. 检查未读脉冲数 → 触发脉冲上传                                     */
/*======================================================================*/
static void uart_axle_counter_online(UART_PortRuntime_t *runtime,
                                     uint8_t module_idx, uint16_t rx_len,
                                     uint16_t *dc_offset)
{
    uint8_t *resp = (uint8_t *)&SerialDataBuf[runtime->port_idx][11];
    uint16_t comm_confirm;
    uint16_t unread_pulses;
    uint16_t kgl_val;

    runtime->module_error[module_idx] = 0;
    runtime->link_mask |= (1UL << module_idx);

    /* ---- 提取实时数据 28 字节 (resp[2] = 跳过 addr+func, 无byte_count) ---- */
    if (rx_len >= (uint16_t)(2U + AXLE_REALTIME_BYTES + 2U)) {
        memcpy(&runtime->ssz_buf[*dc_offset], &resp[2], AXLE_REALTIME_BYTES);
        uart_swap16(&runtime->ssz_buf[*dc_offset], AXLE_REALTIME_REGS);
        runtime->data_len += AXLE_REALTIME_BYTES;
    }

    /* ---- 报警检查: KGL 寄存器 (寄存器1, resp offset 2+2=4) ---- */
    kgl_val = runtime->ssz_buf[*dc_offset + 1];  /* 已做 swap16, 第二个16位字 = 寄存器1 */
    if ((kgl_val & 0x3fU) == 0U) {
        ALARM_TEMP[runtime->port_idx] |= (1UL << module_idx);
    } else {
        ALARM_TEMP[runtime->port_idx] &= ~(1UL << module_idx);
    }

    /* ---- 读取通讯确认码 (寄存器12, resp offset 2+24=26) ---- */
    comm_confirm = ((uint16_t)resp[26] << 8) | (uint16_t)resp[27];

    /* ---- 检查未读脉冲数 (寄存器13, resp offset 2+26=28) ---- */
    unread_pulses = ((uint16_t)resp[28] << 8) | (uint16_t)resp[29];

    if (unread_pulses > 0U) {
        /*
         * 有脉冲: 保存本次的通讯确认码, 下次请求帧带上此值作为 ACK,
         * 告诉采集器"已收到脉冲数据, 可以清除缓存".
         */
        axle_counter_state[runtime->port_idx].comm_confirm[module_idx] = comm_confirm;

        /*
         * 脉冲数据位于 resp[30] 之后.
         * resp[0..1]=头(2B), resp[2..29]=实时数据(28B),
         * resp[30..]=脉冲块(pulse_count+events), resp[last-2..last-1]=CRC.
         * 脉冲数据长度 = rx_len - 2(头) - 28(实时) - 2(CRC) = rx_len - 32.
         */
        uart_axle_counter_upload_pulse(runtime->port_idx, module_idx,
                                       &resp[30],
                                       (uint16_t)(rx_len - 32U),
                                       runtime->link_mask);
    } else {
        /*
         * 无脉冲: 通讯确认码清零, 下次请求帧填 0x0000 (普通查询),
         * 表示"未收到脉冲"或"不需要采集器删除缓存".
         */
        axle_counter_state[runtime->port_idx].comm_confirm[module_idx] = 0x0000;
    }

    *dc_offset += AXLE_REALTIME_REGS;
}


/*======================================================================*/
/*  uart_axle_counter_offline                                           */
/*  计轴模块离线处理: SDRAM 填充 0xFFFF, 累计错误计数                    */
/*======================================================================*/
static void uart_axle_counter_offline(UART_PortRuntime_t *runtime,
                                      uint8_t module_idx, uint16_t *dc_offset)
{
    uint16_t i;

    if (runtime->module_error[module_idx] > 3U) {
        runtime->link_mask &= ~(1UL << module_idx);
    } else {
        runtime->module_error[module_idx]++;
    }

    /* 填充 0xFFFF (离线标志) */
    for (i = 0; i < AXLE_REALTIME_REGS; i++) {
        runtime->ssz_buf[*dc_offset + i] = 0xFFFFU;
    }

    *dc_offset += AXLE_REALTIME_REGS;
    runtime->data_len += AXLE_REALTIME_BYTES;
}


/*======================================================================*/
/*  uart_process_axle_counter                                           */
/*  计轴传感器端口主处理函数 (PortType = 0x07)                           */
/*                                                                       */
/*  遍历从机模块 → 校时(如需) → 0x03查询 → 解析应答                      */
/*  → 实时数据 SDRAM → 脉冲检测 → 更新链路掩码                          */
/*======================================================================*/
static void uart_process_axle_counter(UART_PortRuntime_t *runtime)
{
    uint8_t  m;
    uint8_t  module_count;
    uint16_t rx_len;
    uint16_t dc_offset = 0;

    if (runtime == NULL) {
        return;
    }

    module_count = BoardCfgTab[runtime->port_idx].SlaveModuelNum;
    if (module_count > UART_MODULE_MAX_COUNT) {
        module_count = UART_MODULE_MAX_COUNT;
    }

    runtime->link_mask = 0;
    runtime->data_len  = 0;

    for (m = 0; m < module_count; m++) {
        /* ---- 校时 (每个模块只需发送一次) ---- */
        if (changetime[runtime->port_idx] == 1U) {
            (void)uart_custom_send_adjust_time(
                runtime->port_idx,
                (uint8_t)runtime->port_idx,
                BoardCfgTab[runtime->port_idx].SlaveModuelAddress[m],
                Systemtime);
            TaskAdjustNum[runtime->port_idx]++;
        }

        /* ---- 发送 0x03 查询 + 接收应答 ---- */
        rx_len = uart_axle_counter_module_exchange(runtime->port_idx, m);

        if (rx_len > 0U) {
            /* CRC OK — 在线, 提取实时数据 + 脉冲检测 */
            uart_axle_counter_online(runtime, m, rx_len, &dc_offset);
        } else {
            /* CRC 失败或超时 — 离线 */
            uart_axle_counter_offline(runtime, m, &dc_offset);
        }

        WDTUARTFLAG[runtime->port_idx] = 0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *((uint32_t *)&SerialDataBuf[runtime->port_idx][7]) = runtime->link_mask;
}


/*======================================================================*/
/*  直流3对线模块 (PortType = 0x08) — 协议处理                            */
/*======================================================================*/
/*  uart_dc3wire_module_exchange                                         */
/*  构建 0x03 查询帧 [addr,03,00,00,curve_seq(2B),CRC]                   */
/*  curve_seq = 上次收到的 AllCurveFrame (首次 = 0)                       */
/*======================================================================*/
static uint16_t uart_dc3wire_module_exchange(uint8_t port_idx, uint8_t module_idx)
{
    uint8_t  tx_buf[8];
    uint16_t tx_crc;
    uint16_t rx_len;
    uint16_t rx_crc_calc, rx_crc_recv;
    uint8_t *resp;
    uint16_t curve_seq;

    resp = (uint8_t *)&SerialDataBuf[port_idx][11];

    /* 获取上次的 AllCurveFrame (首次 = 0) */
    curve_seq = dc3wire_state[port_idx].curve_seq[module_idx];

    /* 构建请求: [addr, 03, 00, 00, curve_seq(2B), CRC] */
    tx_buf[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
    tx_buf[1] = 0x03;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;
    tx_buf[4] = (uint8_t)((curve_seq >> 8) & 0xFFU);
    tx_buf[5] = (uint8_t)(curve_seq & 0xFFU);
    tx_crc    = uart_crc16(tx_buf, 6);
    tx_buf[6] = (uint8_t)(tx_crc & 0xFFU);
    tx_buf[7] = (uint8_t)((tx_crc >> 8) & 0xFFU);

    /* 发送 + 接收 */
    rx_len = uart_custom_exchange(port_idx, tx_buf, 8, resp,
                                  (uint16_t)(UART_SERIAL_BUF_SIZE - 11U),
                                  80, 20);
    if (rx_len < 5U) {
        return 0;
    }

    /* CRC16 校验 */
    rx_crc_calc = uart_crc16(resp, rx_len - 2U);
    rx_crc_recv = (uint16_t)((uint16_t)resp[rx_len - 1U] << 8)
                | (uint16_t)resp[rx_len - 2U];
    if (rx_crc_calc != rx_crc_recv) {
        return 0;
    }

    return rx_len;
}


/*======================================================================*/
/*  uart_dc3wire_upload_curve                                            */
/*  将传感器曲线帧(0x80FF)打包为 TCP 曲线帧上传                          */
/*  帧标志: 站台门(ZTM)=0x0B, 站场联(ZCL)=0x0A                          */
/*                                                                       */
/*  TCP 曲线数据包 (协议文档 Table 1.5):                                  */
/*    设备ID(1B) + 通道号(1B) + 时间戳秒(4B) + 曲线点数(2B)              */
/*    + 电压采样1(2B ×0.01V) + ... + 电压采样N(2B ×0.01V)              */
/*======================================================================*/
static void uart_dc3wire_upload_curve(uint8_t port_idx, uint8_t module_idx,
                                      const uint8_t *resp, uint32_t link_mask)
{
    uint8_t  device_id;
    uint8_t  channel;       /* 曲线通道号 (寄存器13, resp[28..29] 低字节) */
    uint16_t curve_len;     /* 曲线点数 (寄存器17, resp[36..37])        */
    uint16_t point_count;   /* 75                                                                        */
    uint32_t curve_time;    /* 曲线起始时间戳 (regs 15-16)                   */
    uint8_t *pkt;
    uint16_t pkt_offset;
    uint16_t i;

    if (resp == NULL) return;

    device_id = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];

    /* 寄存器13: CurveCHx — 曲线通道号 (2B BE)
     * 应答帧格式: [addr][03] + 24B数据 + ...  (无 byte_count 字节)
     * reg N 偏移 = 2 + N*2  →  reg13 = resp[28..29]
     * 传感器用 1/2/3 表示三通道 (存于低字节 resp[29]);
     * 上位机要求 0/1/2, 上传前需减 1.                          */
    channel = resp[2 + 27];  /* reg13 低字节 = 传感器通道号 1/2/3 */
    if (channel > 0U) {
        channel -= 1U;       /* 上位机要求 0/1/2 */
    }

    /* 寄存器15-16: CHx_CurveTime — 曲线起始时间戳(秒) */
    curve_time = ((uint32_t)resp[2 + 30] << 24)
               | ((uint32_t)resp[2 + 31] << 16)
               | ((uint32_t)resp[2 + 32] << 8)
               |  (uint32_t)resp[2 + 33];

    /* 寄存器17: CHx_CurveLen — 有效数据点数 (固定75) */
    curve_len  = ((uint16_t)resp[2 + 34] << 8) | (uint16_t)resp[2 + 35];
    if (curve_len > 75U) curve_len = 75U;

    /* 曲线数据从寄存器18开始 (resp offset 2+36) — 75×2=150 字节 */
    point_count = curve_len;

    /* 构建 TCP 曲线帧 */
    pkt = (uint8_t *)&SerialDataBuf[port_idx][11];
    pkt_offset = 0;

    pkt[pkt_offset++] = device_id;
    pkt[pkt_offset++] = channel;                       /* 通道号 0/1/2 */
    pkt[pkt_offset++] = (uint8_t)(curve_time);          /* 时间戳(秒) 低字节在前 */
    pkt[pkt_offset++] = (uint8_t)(curve_time >> 8);
    pkt[pkt_offset++] = (uint8_t)(curve_time >> 16);
    pkt[pkt_offset++] = (uint8_t)(curve_time >> 24);
    pkt[pkt_offset++] = (uint8_t)(point_count & 0xFFU);        /* 曲线点数低 */
    pkt[pkt_offset++] = (uint8_t)((point_count >> 8) & 0xFFU); /* 曲线点数高 */

    /* 拷贝电压采样点 (int16 ×0.01V, 低字节在前) */
    for (i = 0; i < point_count && pkt_offset < (UART_SERIAL_BUF_SIZE - 30U); i++) {
        uint16_t offset = (uint16_t)(2U + 36U + i * 2U);
        pkt[pkt_offset++] = resp[offset + 1]; /* 低字节 */
        pkt[pkt_offset++] = resp[offset];     /* 高字节 */
    }

    /* 构建完整 TCP 帧: [端口,主协议,子协议,帧标志(0x0B站台门/0x0A站场联),分机号,len(2B),link_mask(4B),曲线包] */
    if (pkt_offset > 8U) {
        uint8_t *frame = (uint8_t *)&SerialDataBuf[port_idx][0];
        uint8_t  module_type;
        uint16_t f_len;

        /* 帧标志: 站台门(ZTM,0x51)曲线帧=0x0B, 站场联(ZCL,0x52)曲线帧=0x0A */
        module_type = BoardCfgTab[port_idx].SlaveModuelType[module_idx];
        frame[0] = (uint8_t)(port_idx + 1U);
        frame[1] = BoardCfgTab[port_idx].MainProType;
        frame[2] = BoardCfgTab[port_idx].SubProType;
        frame[3] = (module_type == FLFXG_DC3WIRE_ZTM) ? 0x0BU : 0x0AU;
        frame[4] = BoardCfgTab[port_idx].FenjiNum;
        *((uint16_t *)&frame[5]) = (uint16_t)(pkt_offset + 4U);
        *((uint32_t *)&frame[7]) = link_mask;

        f_len = (uint16_t)(pkt_offset + 11U);
        client_add_curve_to_tcp_send_buf(frame, f_len,
                                         20U + port_idx * 6U);
    }
}


/*======================================================================*/
/*  uart_dc3wire_online                                                  */
/*  直流3对线模块在线处理:                                                */
/*  1. 提取实时数据 24 字节 (跳过 addr+func, 无 byte_count)                 */
/*  2. 检查 KGL 报警位                                                    */
/*  3. 检查 CurveFlag = 0x80FF → 触发曲线上传                             */
/*  4. 保存 AllCurveFrame (用于下次查询的 curve_seq)                       */
/*======================================================================*/
static void uart_dc3wire_online(UART_PortRuntime_t *runtime,
                                uint8_t module_idx, uint16_t rx_len,
                                uint16_t *dc_offset)
{
    uint8_t *resp = (uint8_t *)&SerialDataBuf[runtime->port_idx][11];
    uint16_t curve_flag;
    uint16_t all_curve_frame;
    uint16_t kgl_val;

    runtime->module_error[module_idx] = 0;
    runtime->link_mask |= (1UL << module_idx);

    /* 提取实时数据 24 字节 (resp[2] = 跳过 addr+func, 无 byte_count) */
    if (rx_len >= (uint16_t)(2U + DC3WIRE_REALTIME_BYTES + 2U)) {
        memcpy(&runtime->ssz_buf[*dc_offset], &resp[2], DC3WIRE_REALTIME_BYTES);
        uart_swap16(&runtime->ssz_buf[*dc_offset], DC3WIRE_REALTIME_REGS);
        runtime->data_len += DC3WIRE_REALTIME_BYTES;
    }

    /* 报警检查: KGL (寄存器1, ssz_buf offset 1) */
    kgl_val = runtime->ssz_buf[*dc_offset + 1];
    if ((kgl_val & 0x3fU) == 0U) {
        ALARM_TEMP[runtime->port_idx] |= (1UL << module_idx);
    } else {
        ALARM_TEMP[runtime->port_idx] &= ~(1UL << module_idx);
    }

    /* 读取 CurveFlag (寄存器11, 偏移 2+22=24, 无 byte_count) */
    curve_flag = ((uint16_t)resp[24] << 8) | (uint16_t)resp[25];

    /* CurveFlag = 0x80FF → 有曲线数据, 需要ACK机制 */
    if (curve_flag == 0x80FFU) {
        /* 读取 AllCurveFrame (寄存器12, 偏移 2+24=26, 无 byte_count)
         * 注意: 只有曲线帧长度足够时 resp[26..27] 才是 AllCurveFrame;
         * 无曲线时 resp[26..27] 是 CRC, 读到的是随机值, 不能使用. */
        all_curve_frame = ((uint16_t)resp[26] << 8) | (uint16_t)resp[27];
        if (all_curve_frame > 0U) {
            dc3wire_state[runtime->port_idx].curve_seq[module_idx] = all_curve_frame;
        }

        uart_dc3wire_upload_curve(runtime->port_idx, module_idx,
                                  resp, runtime->link_mask);
    } else {
        /* 无曲线: 清除 curve_seq, 下次请求恢复 00 00 */
        dc3wire_state[runtime->port_idx].curve_seq[module_idx] = 0x0000;
    }

    *dc_offset += DC3WIRE_REALTIME_REGS;
}


/*======================================================================*/
/*  uart_dc3wire_offline                                                  */
/*  离线处理: SDRAM 填充 0xFFFF, 累计错误计数                             */
/*======================================================================*/
static void uart_dc3wire_offline(UART_PortRuntime_t *runtime,
                                  uint8_t module_idx, uint16_t *dc_offset)
{
    uint16_t i;

    if (runtime->module_error[module_idx] > 3U) {
        runtime->link_mask &= ~(1UL << module_idx);
    } else {
        runtime->module_error[module_idx]++;
    }

    for (i = 0; i < DC3WIRE_REALTIME_REGS; i++) {
        runtime->ssz_buf[*dc_offset + i] = 0xFFFFU;
    }
    *dc_offset += DC3WIRE_REALTIME_REGS;
    runtime->data_len += DC3WIRE_REALTIME_BYTES;
}


/*======================================================================*/
/*  uart_process_dc3wire                                                  */
/*  直流3对线端口主处理函数 (PortType = 0x08)                              */
/*                                                                       */
/*  遍历从机 → 校时 → 0x03查询 → 解析应答 → 实时数据/曲线                 */
/*======================================================================*/
static void uart_process_dc3wire(UART_PortRuntime_t *runtime)
{
    uint8_t  m;
    uint8_t  module_count;
    uint16_t rx_len;
    uint16_t dc_offset = 0;

    if (runtime == NULL) return;

    module_count = BoardCfgTab[runtime->port_idx].SlaveModuelNum;
    if (module_count > UART_MODULE_MAX_COUNT) {
        module_count = UART_MODULE_MAX_COUNT;
    }

    runtime->link_mask = 0;
    runtime->data_len  = 0;

    for (m = 0; m < module_count; m++) {
        /* 校时 */
        if (changetime[runtime->port_idx] == 1U) {
            (void)uart_custom_send_adjust_time(
                runtime->port_idx, (uint8_t)runtime->port_idx,
                BoardCfgTab[runtime->port_idx].SlaveModuelAddress[m],
                Systemtime);
            TaskAdjustNum[runtime->port_idx]++;
        }

        /* 0x03 查询 */
        rx_len = uart_dc3wire_module_exchange(runtime->port_idx, m);

        if (rx_len > 0U) {
            uart_dc3wire_online(runtime, m, rx_len, &dc_offset);
        } else {
            uart_dc3wire_offline(runtime, m, &dc_offset);
        }

        WDTUARTFLAG[runtime->port_idx] = 0;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *((uint32_t *)&SerialDataBuf[runtime->port_idx][7]) = runtime->link_mask;
}


/*======================================================================*/
/*  SendLogToServer  —  向服务器发送日志信息                              */
/*                                                                       */
/*  帧格式 (9 字节):                                                      */
/*  [端口号, 0x50, 0x01, 0xFF, 0xFF, 0x02, 0x00, 日志ID, 附加信息]        */
/*                                                                       */
/*  日志 ID 定义:                                                         */
/*   1 = 系统启动                                                         */
/*   2 = 网络重连成功                                                     */
/*   3 = 看门狗复位 (附带 WTD_RESET 复位码)                                */
/*   4 = 端口数据错误                                                     */
/*   5 = 端口曲线上传完成                                                  */
/*   6 = 校时                                                             */
/*                                                                       */
/*  通过 port_flag = UART_PORT_COUNT (控制通道) 发送.                      */
/*======================================================================*/
void SendLogToServer(uint8_t port, uint8_t log_id)
{
    uint8_t s_buf[32];
    uint8_t m = 0U;

    memset(s_buf, 0, sizeof(s_buf));

    /*
     * 构建日志帧:
     * [0]: 端口号
     * [1]: 主类型 = 0x50 (日志帧专用)
     * [2]: 子类型 = 0x01
     * [3]: 分机号 = 0xFF
     * [4]: 分机号 = 0xFF
     * [5]: 数据长度低字节 = 0x02
     * [6]: 数据长度高字节 = 0x00 → 总数据长度 = 2 字节
     * [7]: 日志 ID
     * [8]: 附加信息 (复位码 或 0)
     */
    s_buf[m++] = port;      /* 端口号 */
    s_buf[m++] = 0x50U;     /* 主类型: 日志帧 */
    s_buf[m++] = 0x01U;     /* 子类型 */
    s_buf[m++] = 0xFFU;     /* 分机号 */
    s_buf[m++] = 0xFFU;     /* 分机号 */
    s_buf[m++] = 0x02U;     /* 数据长度低字节 */
    s_buf[m++] = 0x00U;     /* 数据长度高字节 */
    s_buf[m++] = log_id;    /* 日志 ID */

    if (log_id == 3U) {
        /* 看门狗复位日志: 附加 WTD_RESET 复位码 */
        s_buf[m++] = WTD_RESET;
    } else {
        s_buf[m++] = 0U;
    }

    /*
     * 通过控制通道发送 (port_flag = UART_PORT_COUNT = 6).
     * reset_flag = 30: 如果发送失败触发看门狗, 复位码 = 30.
     */
    client_add_to_tcp_send_buf(s_buf, (uint16_t)m, 30U, UART_PORT_COUNT);
}


/*======================================================================*/
/*  checkSendLog  —  检查日志标志并触发发送                               */
/*                                                                       */
/*  由 App_TaskStart 周期性调用 (每 10ms).                                 */
/*  检查各类日志标志, 如果被置位则调用 SendLogToServer, 然后清除/修改标志.  */
/*                                                                       */
/*  日志标志说明:                                                         */
/*  - pStart:        系统启动时置 1 (App_TaskStart 中设置)                 */
/*  - netReset:      TCP 重连成功时置 1 (client_thread 中设置)            */
/*  - logWTD_RESET:  看门狗即将触发复位时置 1 (App_TaskStart 中设置)       */
/*  - portError[]:   端口全部从机离线时置 1, 发送后清零或置 0xFF           */
/*  - portDLQX[]:    曲线上传完成时置非零值 (bsp_rs485.c 中设置)           */
/*  - jiaoshi:       校时发生时置 1 (client_handle_rx 中设置)              */
/*======================================================================*/
void checkSendLog(void)
{
    uint8_t i;

    /* ---- 系统启动日志 ---- */
    if (pStart == 1U) {
        SendLogToServer(0, 1U);
        pStart = 0U;
    }

    /* ---- 网络重连日志 ---- */
    if (netReset == 1U) {
        SendLogToServer(0, 2U);
        netReset = 0U;
    }

    /* ---- 看门狗复位日志 ---- */
    if (logWTD_RESET == 1U) {
        SendLogToServer(0, 3U);
        logWTD_RESET = 2U;  /* 标记为已发送, 避免重复发送 */
    }

    /* ---- 端口数据错误日志 (6 路端口) ---- */
    for (i = 0; i < UART_PORT_COUNT; i++) {
        if (portError[i] == 1U || portError[i] == 2U) {
            SendLogToServer((uint8_t)(i + 1U), 4U);
            if (portError[i] == 1U) {
                portError[i] = 0U;     /* 已发送, 清除标志 */
            } else {
                portError[i] = 0xFFU;  /* 已发送, 标记为"不再发送" */
            }
        }
    }

    /* ---- 端口曲线上传完成日志 (6 路端口) ---- */
    for (i = 0; i < UART_PORT_COUNT; i++) {
        if (portDLQX[i] != 0U) {
            SendLogToServer((uint8_t)(i + 1U), 5U);
            portDLQX[i] = 0U;
        }
    }

    /* ---- 校时日志 ---- */
    if (jiaoshi != 0U) {
        SendLogToServer(0, 6U);
        jiaoshi = 0U;
    }
}


/*************************************************************************************
 *                            USER FUNCTIONS
 *                            Modbus Slave callbacks
 *  与上位机通过 Modbus TCP 交互时使用 (保持寄存器 / 输入寄存器 读写)
 *************************************************************************************/

/*======================================================================*/
/*  eMBSRegHoldingCB                                                     */
/*  Modbus 保持寄存器回调: 上位机可读写 MBRegHoldingBuf                     */
/*======================================================================*/
static eMBException
eMBSRegHoldingCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBSRegisterMode eRegMode)
{
    eMBException eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
    USHORT       usRegIndex;

    if ((usAddress >= MB_REG_HOLDING_START) &&
        (usAddress + usNRegs <= MB_REG_HOLDING_START + MB_REG_HOLDING_NREGS)) {
        usRegIndex = (USHORT)(usAddress - MB_REG_HOLDING_START);

        switch (eRegMode) {
        case MBS_REGISTER_READ:
            while (usNRegs > 0) {
                *pubRegBuffer++ = (UCHAR)(MBRegHoldingBuf[usRegIndex] >> 8);
                *pubRegBuffer++ = (UCHAR)(MBRegHoldingBuf[usRegIndex] & 0xFF);
                usRegIndex++;
                usNRegs--;
            }
            break;

        case MBS_REGISTER_WRITE:
            while (usNRegs > 0) {
                MBRegHoldingBuf[usRegIndex] = *pubRegBuffer++ << 8;
                MBRegHoldingBuf[usRegIndex] |= *pubRegBuffer++;
                usRegIndex++;
                usNRegs--;
            }
            break;

        default:
            break;
        }
        eException = MB_PDU_EX_NONE;
    }
    return eException;
}


/*======================================================================*/
/*  eMBSRegInputCB                                                       */
/*  Modbus 输入寄存器回调: 上位机可读取 MBRegInputBuf (只读)               */
/*======================================================================*/
static eMBException
eMBSRegInputCB(UBYTE *pubRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBException eException = MB_PDU_EX_ILLEGAL_DATA_ADDRESS;
    USHORT       usRegIndex;

    if ((usAddress >= MB_REG_INPUT_START) &&
        (usAddress + usNRegs <= MB_REG_INPUT_START + MB_REG_INPUT_NREGS)) {
        usRegIndex = (USHORT)(usAddress - MB_REG_INPUT_START);

        while (usNRegs > 0) {
            *pubRegBuffer++ = (UCHAR)(MBRegInputBuf[usRegIndex] >> 8);
            *pubRegBuffer++ = (UCHAR)(MBRegInputBuf[usRegIndex] & 0xFF);
            usRegIndex++;
            usNRegs--;
        }
        eException = MB_PDU_EX_NONE;
    }
    return eException;
}


/******************************** END OF FILE ***************************/
