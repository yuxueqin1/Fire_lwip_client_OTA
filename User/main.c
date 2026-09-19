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

/* ---- Offline skip helper: 连续3次失败后跳过等待, 每10个周期探测一次 ---- */
static uint8_t uart_module_should_poll(UART_PortRuntime_t *runtime, uint8_t module_idx);

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
                                         const UART_ModuleProfile_t *profile,
                                         uint16_t rx_len, uint16_t *dc_offset);
static void     uart_axle_counter_offline(UART_PortRuntime_t *runtime, uint8_t module_idx,
                                          const UART_ModuleProfile_t *profile,
                                          uint16_t *dc_offset);
static void     uart_axle_counter_upload_pulse(uint8_t port_idx, uint8_t module_idx,
                                               const uint8_t *pulse_data,
                                               uint16_t pulse_len, uint8_t end_flag,
                                               uint32_t link_mask);

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
    /*
     * 初始化失败时 eMBMSerialInit 会把 xMBMaster 置 NULL, 此后
     * uart_process_modbus_master() 会在入口直接返回, 该端口永远不上传模块数据
     * (只有帧头 + 长度 4 + 全 0 链路掩码), 且 uart_check_port_error_and_arm()
     * 因 xMBMaster 为 NULL 也不会走 close 分支重试, 无法自愈. 必须打出来.
     */
    if (uart_init_port(port_idx, baud_rate, &runtime.xMBMaster) != MB_ENOERR) {
        printf("[UART] port=%u init FAILED, xMBMaster=NULL\r\n", port_idx + 1U);
    }
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
                    uint16_t _off = 0;
                    printf("[AXLE-DBG] port=%d mask=0x%08lX len=%u",
                           port_idx + 1, runtime.link_mask, runtime.data_len);
                    for (_m = 0; _m < BoardCfgTab[port_idx].SlaveModuelNum; _m++) {
                        printf(" | m%d[addr=%u reg0=%04X reg1=%04X reg2=%04X]",
                               _m,
                               BoardCfgTab[port_idx].SlaveModuelAddress[_m],
                               runtime.ssz_buf[_off],
                               runtime.ssz_buf[_off + 1],
                               runtime.ssz_buf[_off + 2]);
                        /* 按模块实时寄存器个数累计偏移 (普通计轴=20B/10reg, 高频计轴=56B/28reg) */
                        _off += (uint16_t)(uart_get_module_profile(
                                    BoardCfgTab[port_idx].SlaveModuelType[_m])->realtime_bytes / 2U);
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
/*  uart_module_should_poll                                              */
/*  从机离线跳过机制 (四种端口类型共用):                                   */
/*  - 正常状态 (连续失败 <3 次): 每轮都发送请求轮询.                      */
/*  - 连续 3 次轮询无响应 (module_error >= 3) 后: 进入跳过状态, 不再       */
/*    发送请求/等待应答超时, 仅由上层保持 SDRAM 0xFFFF 离线数据填充.       */
/*  - 跳过状态每累计 10 个轮询周期 (每轮补齐到 UART_POLL_PERIOD_MS=1s,    */
/*    即 ≈10s) 探测一次: 若恢复在线 (online 处理清零 module_error) 则       */
/*    恢复正常轮询; 仍无响应则保持跳过.                                   */
/*  - skip_probe_count 由本函数维护; 探测成功后 module_error 被清零,       */
/*    本函数在 me<3 分支将计数复位为 0, 状态自然回到正常轮询.             */
/*  返回 1 = 本轮正常轮询/探测, 0 = 本轮跳过.                             */
/*======================================================================*/
static uint8_t uart_module_should_poll(UART_PortRuntime_t *runtime, uint8_t module_idx)
{
    uint8_t *cnt = &runtime->skip_probe_count[module_idx];

    if (runtime->module_error[module_idx] < 3U) {
        /* 正常状态: 清零探测计数, 每轮都轮询 */
        *cnt = 0U;
        return 1U;
    }

    /* 跳过状态 (连续失败≥3次): 首周期直接跳过, 之后每10个周期探测一次 */
    if (*cnt == 0U) {
        *cnt = 1U;
        return 0U;
    }
    if (*cnt >= 10U) {
        *cnt = 0U;
        return 1U;   /* 本轮探测 */
    }
    (*cnt)++;
    return 0U;
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

        /* 离线跳过: 连续3次失败后不再发请求/等应答超时, 每10个周期探测一次 */
        if (!uart_module_should_poll(runtime, m)) {
            uart_modbus_mark_offline(runtime, m, profile, &data);
            runtime->data_len += (uint16_t)(profile->input_regs * 2U);
            WDTUARTFLAG[runtime->port_idx] = 0;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

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

    if (runtime->module_error[module_idx] > 3U) {
        /* 保持计数 ≥3 (不再清零), 与自定义/计轴/直流3对线处理器一致:
         * 离线跳过机制 (uart_module_should_poll) 依赖 module_error>=3
         * 维持跳过状态, 若在此清零会导致跳过状态在下一轮丢失、退回每轮
         * 等待应答超时 (200ms), 无法满足 1s 轮询预算. */
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

        /* 离线跳过: 连续3次失败后不再发请求/等超时, 每10个周期探测一次 */
        if (!uart_module_should_poll(runtime, m)) {
            uart_custom_module_offline(runtime, m, profile, &dc_offset);
            WDTUARTFLAG[runtime->port_idx] = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
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
                                  40, 10);  /* 首字节超时 80→40ms: 离线从机更快暴露, 1s内轮询完13台 */
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
     * 计算寄存器数据在应答中的起始偏移 data_offset.
     *  多数模块应答含 1 字节字节计数 len: [地址][0x03][len][寄存器...] → 从 resp[3] 开始;
     *  S700K_C (0x3A) / S700K_C_HIGH (0x4E) 应答无字节计数字段:
     *  [地址][0x03][44寄存器...][曲线帧标识][曲线帧序号][时间高字][时间低字][有效长度]...
     *  → 从 resp[2] 开始.
     *  由 curve_flag_offset = data_offset + realtime_bytes 的关系可统一推导:
     *    带 len 的模块 data_offset = 3;  S700K 模块 data_offset = 2.
     */
    uint16_t data_offset = (profile->uses_custom_curve != 0U)
                         ? (uint16_t)(profile->curve_flag_offset - profile->realtime_bytes)
                         : 3U;

    /* 交换字节序 (大端 → 小端) 并拷贝实时寄存器到 SDRAM. */
    reg_count = profile->realtime_bytes / 2U;
    memcpy(&runtime->ssz_buf[*dc_offset], &resp[data_offset], profile->realtime_bytes);
    uart_swap16(&runtime->ssz_buf[*dc_offset], reg_count);

    /*
     * 拷贝 holding 寄存器 (跳过前 2 个寄存器, 从 data_offset + 4 字节开始).
     */
    holding_pos = (uint16_t)(runtime->holding_base + 2U
                             + module_idx * UART_MODULE_DEFAULT_REGS);
    if (profile->holding_regs > 0U) {
        uint16_t copy = profile->holding_regs;
        if ((holding_pos + copy) > MB_REG_HOLDING_NREGS) {
            copy = (uint16_t)(MB_REG_HOLDING_NREGS - holding_pos);
        }
        memcpy(&MBRegHoldingBuf[holding_pos], &resp[data_offset + 4U], copy * 2U);
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
/*  构建 0x03 查询帧 [addr,03,comm_confirm(2B),pulse_count(2B),CRC]     */
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
    uint16_t pulse_count;

    resp = (uint8_t *)&SerialDataBuf[port_idx][11];

    /* 获取上次的通讯确认码 (首次 = 0x0000) 和要读取的脉冲个数 (首次 = 0) */
    comm_confirm = axle_counter_state[port_idx].comm_confirm[module_idx];
    pulse_count  = axle_counter_state[port_idx].pending_pulse_count[module_idx];

    /* 构建请求: [addr, 03, comm_confirm(2B), pulse_count(2B), CRC] */
    tx_buf[0] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx];
    tx_buf[1] = 0x03;
    tx_buf[2] = (uint8_t)((comm_confirm >> 8) & 0xFFU);
    tx_buf[3] = (uint8_t)(comm_confirm & 0xFFU);
    tx_buf[4] = (uint8_t)((pulse_count >> 8) & 0xFFU);   /* 要读取的脉冲个数 (reg13剩余未读数, 首次/无脉冲=0) */
    tx_buf[5] = (uint8_t)(pulse_count & 0xFFU);
    tx_crc    = uart_crc16(tx_buf, 6);
    tx_buf[6] = (uint8_t)(tx_crc & 0xFFU);
    tx_buf[7] = (uint8_t)((tx_crc >> 8) & 0xFFU);

    /* 发送 + 接收 */
    rx_len = uart_custom_exchange(port_idx, tx_buf, 8, resp,
                                  (uint16_t)(UART_SERIAL_BUF_SIZE - 11U),
                                  50, 50); //首字节超时 100→50ms; 字节间隔保持50ms(丢包问题在字节间隔, 不动)
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
/*  将采集盒本次下发的脉冲块 (一包) 打包为 TCP 脉冲帧 (帧标志 0x09) 上传  */
/*                                                                       */
/*  一包内可能混多个通道: **按通道分组**, 每个通道单独发一个 0x09 帧,     */
/*  同一通道的全部脉冲合并进同一帧 (N = 该通道脉冲数).                   */
/*                                                                       */
/*  每个脉冲自带秒/毫秒时间戳, 帧头时间戳取**该通道最早脉冲的时间**,      */
/*  同通道其余脉冲的点偏移相对它计算:                                     */
/*    点偏移 = ((脉冲秒 - 本通道最早秒)×1000 + (脉冲毫秒 - 本通道最早毫秒)) / 4 */
/*  基准取的是本通道最早脉冲, 点偏移恒 ≥ 0.                              */
/*                                                                       */
/*  TCP 曲线数据包格式 (协议文档 Table 1.4):                              */
/*    设备ID(1B) + 通道号(1B) + 时间戳秒(4B) + 时间戳毫秒(2B)            */
/*    + 结束标志(1B) + 当前包脉冲数(2B)=N                                 */
/*    + Σ(脉冲: 点偏移(2B) + 包含点数(1B) + 采集值(k×1B))                */
/*                                                                       */
/*  结束标志 end_flag 由调用方传入, **每个通道的帧都带上**. 当前唯一调用方  */
/*  (uart_axle_counter_online) 恒传 1 (曲线采集结束), 上位机据此展示曲线;   */
/*  置 0 的曲线上位机不显示 (原 reg13 判据在活传感器下几乎恒为 0, 见调用处).*/
/*                                                                       */
/*  缓冲复用 (两区分离, 不能省):                                          */
/*    SerialDataBuf[port][11..] 前部放压缩记录区 base[0..rec_len),        */
/*    帧区从 base[rec_len] 开始, 两区不重叠.                             */
/*                                                                       */
/*    [为什么必须先把源数据挪到记录区] 脉冲源数据 pulse_data 与本函数的  */
/*    组帧区是同一块内存 (pulse_data = base + pulse_off), 而每个通道都要  */
/*    从脉冲块**开头**重新扫描一遍. 若各通道的帧就地组在 base[0..],      */
/*    前一个通道的输出会覆盖后一个通道尚未扫描的源数据 (输出累计超过      */
/*    pulse_off - 9 字节即发生: 高频约 5 个脉冲、低频约 2 个脉冲就超标),  */
/*    导致后面通道的采集值/点偏移被污染. 故先做一次前移压缩, 让帧区与     */
/*    记录区彻底分离.                                                     */
/*                                                                       */
/*  容量限制: 记录区 + 单个通道帧 ≤ 4085 字节. 普通一包 (几十个脉冲) 远   */
/*  低于此值; 仅当整包接近 4KB 时才可能装不下, 此时按防溢出判断截断该帧   */
/*  (帧内脉冲数 N 随之减少), 不会越界.                                   */
/*  任务栈仅 384 字, 故不使用大局部数组, 全部就地于 4KB 串口缓冲中完成.   */
/*======================================================================*/
static void uart_axle_counter_upload_pulse(uint8_t port_idx, uint8_t module_idx,
                                           const uint8_t *pulse_data,
                                           uint16_t pulse_len, uint8_t end_flag,
                                           uint32_t link_mask)
{
    uint8_t  *base;                          /* SerialDataBuf[port][11..] 记录区+帧区共用 */
    uint8_t   ch_list[AXLE_MAX_CHANNELS];    /* 去重后的通道号 (首次出现顺序) */
    uint32_t  ch_base_ts[AXLE_MAX_CHANNELS]; /* 各通道最早脉冲的秒 */
    uint16_t  ch_base_ms[AXLE_MAX_CHANNELS]; /* 各通道最早脉冲的毫秒 */
    uint8_t   ch_num = 0;                    /* 本包出现过的通道数 */
    uint16_t  pkt_pulse_count;
    uint16_t  rec_len = 0;                   /* 压缩记录区长度 */
    uint16_t  read_offset;
    uint16_t  p, i, c;

    if ((pulse_data == NULL) || (pulse_len < 10U)) {
        return;
    }

    base = (uint8_t *)&SerialDataBuf[port_idx][11];

    /* 读取"本包上报脉冲个数"(前2字节) */
    pkt_pulse_count = ((uint16_t)pulse_data[0] << 8) | (uint16_t)pulse_data[1];

    /*
     * ---- 第一遍: 源脉冲块逐字节压缩拷贝到记录区 base[0..rec_len),
     *      同时收集通道号、求各通道最早脉冲时间 (按通道拆帧用) ----
     * 记录 = [秒(4B)+毫秒(2B)+通道(1B)+点数(1B)+采集值(n)] 原样, 共 8+n 字节,
     * 去掉脉冲块开头的"本包脉冲个数"(2B).
     * 写指针 rec_len 恒比源读指针 read_offset 低位 (read_offset 从 2 起步,
     * rec_len 从 0 起步, 且两者累加相同的 plen), 故前向逐字节拷贝即使与自身
     * 源数据重叠也安全.
     * 记录区长度上限: read_offset + plen <= pulse_len <= 4085 ⇒ rec_len <= 4083,
     * 不会越过 SerialDataBuf[port][11] 之后的 4085 字节.
     */
    read_offset = 2;
    for (p = 0; p < pkt_pulse_count && read_offset < pulse_len; p++) {
        uint32_t ts;
        uint16_t tms;
        uint8_t  ch, size;
        uint16_t plen;

        if ((uint16_t)(read_offset + 8U) > pulse_len) {
            break;
        }
        ts   = ((uint32_t)pulse_data[read_offset]     << 24)
             | ((uint32_t)pulse_data[read_offset + 1] << 16)
             | ((uint32_t)pulse_data[read_offset + 2] << 8)
             |  (uint32_t)pulse_data[read_offset + 3];
        tms  = ((uint16_t)pulse_data[read_offset + 4] << 8)
             |  (uint16_t)pulse_data[read_offset + 5];
        ch   = pulse_data[read_offset + 6];
        size = pulse_data[read_offset + 7];
        plen = (uint16_t)(8U + size);
        if ((uint16_t)(read_offset + plen) > pulse_len) {
            break;
        }

        /* 逐字节前向拷贝到记录区 (写指针恒低于读指针) */
        for (i = 0; i < plen; i++) {
            base[rec_len + i] = pulse_data[read_offset + i];
        }
        rec_len     += plen;
        read_offset += plen;

        /* 收集通道号 (去重) */
        for (c = 0; c < ch_num; c++) {
            if (ch_list[c] == ch) {
                break;
            }
        }
        if (c >= ch_num) {
            if (ch_num >= AXLE_MAX_CHANNELS) {
                continue;              /* 通道数超上限: 该脉冲不参与统计 */
            }
            ch_list[c]  = ch;
            ch_base_ts[c] = ts;
            ch_base_ms[c] = tms;
            ch_num++;
        } else if ((ts < ch_base_ts[c]) ||
                   ((ts == ch_base_ts[c]) && (tms < ch_base_ms[c]))) {
            ch_base_ts[c] = ts;
            ch_base_ms[c] = tms;
        }
    }

    if (ch_num == 0U) {
        return;
    }

    /* ---- 第二遍: 每个通道单独一个 0x09 帧 (帧区在记录区之后, 两区不重叠) ---- */
    for (c = 0; c < ch_num; c++) {
        uint8_t  *frame = &base[rec_len]; /* 帧区起点: 记录区之后 */
        uint16_t  off;                    /* 帧内写偏移 (含11B TCP帧头) */
        uint16_t  n = 0;                  /* 本帧实际写入的脉冲数 N */
        uint32_t  curve_ts = ch_base_ts[c]; /* 本通道最早脉冲的秒 */
        uint16_t  curve_ms = ch_base_ms[c]; /* 本通道最早脉冲的毫秒 */

        /* 防溢出: 帧区起点 + TCP帧头(11B) + 曲线包头(11B) 必须落在缓冲内 */
        if ((uint16_t)(rec_len + 22U) > (uint16_t)(UART_SERIAL_BUF_SIZE - 11U)) {
            continue;
        }

        /*
         * ---- TCP 帧头: [端口,主协议,子协议,0x09,分机号,len(2B,LE),link_mask(4B,LE)] ----
         * 逐字节写: 帧区起点 base[rec_len] 的奇偶不定, 避免非对齐的16/32位访问.
         * frame[5..6] = 数据长度, frame[7..10] = 模块状态, 本帧脉冲数收完一并回填.
         */
        frame[0] = (uint8_t)(port_idx + 1U);
        frame[1] = BoardCfgTab[port_idx].MainProType;
        frame[2] = BoardCfgTab[port_idx].SubProType;
        frame[3] = 0x09;                                   /* 帧标志: 脉冲曲线帧 */
        frame[4] = BoardCfgTab[port_idx].FenjiNum;

        /* ---- 曲线数据包头部 (协议表1.4, 本帧只写一次) ---- */
        off = 11;
        frame[off++] = BoardCfgTab[port_idx].SlaveModuelAddress[module_idx]; /* 设备ID */
        frame[off++] = ch_list[c];                         /* 通道号: 本帧只含该通道 */
        frame[off++] = (uint8_t)(curve_ts);                /* 时间戳(秒) = 本通道最早脉冲, 低字节在前 */
        frame[off++] = (uint8_t)(curve_ts >> 8);
        frame[off++] = (uint8_t)(curve_ts >> 16);
        frame[off++] = (uint8_t)(curve_ts >> 24);
        frame[off++] = (uint8_t)(curve_ms);                /* 时间戳(毫秒) 低字节在前 */
        frame[off++] = (uint8_t)(curve_ms >> 8);
        frame[off++] = (end_flag != 0U) ? 0x01U : 0x00U;   /* 结束标志: 每个通道帧都带上 */
        off += 2;                                          /* 当前包脉冲数 N, 稍后回填 */

        /* ---- 扫描记录区, 只取本通道的脉冲, 计算点偏移 ---- */
        read_offset = 0;
        while (read_offset < rec_len) {
            uint32_t ts;
            uint16_t tms;
            uint8_t  size;
            int32_t  delta_s, delta_ms, total_ms;
            uint32_t time_offset;

            if ((uint16_t)(read_offset + 8U) > rec_len) {
                break;
            }
            size = base[read_offset + 7];
            if ((uint16_t)(read_offset + 8U + size) > rec_len) {
                break;
            }
            if (base[read_offset + 6] != ch_list[c]) {
                read_offset += (uint16_t)(8U + size);      /* 非本通道, 跳过 */
                continue;
            }

            /* 防溢出: 记录区 + 帧区 + 本脉冲体 不超过缓冲尾 (rx_len <= 4085) */
            if ((uint16_t)(rec_len + off + 3U + size) >
                (uint16_t)(UART_SERIAL_BUF_SIZE - 11U)) {
                break;
            }

            ts  = ((uint32_t)base[read_offset]     << 24)
                | ((uint32_t)base[read_offset + 1] << 16)
                | ((uint32_t)base[read_offset + 2] << 8)
                |  (uint32_t)base[read_offset + 3];
            tms = ((uint16_t)base[read_offset + 4] << 8)
                |  (uint16_t)base[read_offset + 5];

            /* 点偏移 = ((脉冲秒 - 本通道最早秒)×1000 + (脉冲毫秒 - 本通道最早毫秒)) / 4 */
            delta_s  = (int32_t)(ts - curve_ts);
            delta_ms = (int32_t)(tms - curve_ms);
            total_ms = delta_s * 1000 + delta_ms;
            time_offset = (uint32_t)(total_ms / 4);

            /* ---- 脉冲体: [点偏移(2B,LE)][包含点数(1B)][采集值×n] ---- */
            frame[off++] = (uint8_t)(time_offset & 0xFFU);         /* 点偏移(低) */
            frame[off++] = (uint8_t)((time_offset >> 8) & 0xFFU);  /* 点偏移(高) */
            frame[off++] = size;                                   /* 包含点数 */
            for (i = 0; i < size; i++) {
                frame[off++] = base[read_offset + 8U + i];         /* 采集值(×0.1V) */
            }
            n++;

            read_offset += (uint16_t)(8U + size);
        }

        if (n == 0U) {
            continue;                       /* 本包无该通道脉冲, 不发帧 */
        }

        /* 回填当前包脉冲数 N (曲线包内偏移9-10 = 帧内 20-21, 小端) */
        frame[20] = (uint8_t)(n & 0xFFU);
        frame[21] = (uint8_t)((n >> 8) & 0xFFU);

        /* 数据长度 = 曲线数据包长度 + 4 (模块状态), 小端 */
        {
            uint16_t dlen = (uint16_t)(off - 7U);
            frame[5] = (uint8_t)(dlen & 0xFFU);
            frame[6] = (uint8_t)(dlen >> 8);
        }
        /* 模块状态 link_mask (4B, 小端) */
        frame[7]  = (uint8_t)(link_mask & 0xFFU);
        frame[8]  = (uint8_t)((link_mask >> 8) & 0xFFU);
        frame[9]  = (uint8_t)((link_mask >> 16) & 0xFFU);
        frame[10] = (uint8_t)((link_mask >> 24) & 0xFFU);

        client_add_curve_to_tcp_send_buf(frame, off, 20U + port_idx * 6U);
    }
}


/*======================================================================*/
/*  uart_axle_counter_online                                            */
/*  计轴模块在线处理:                                                    */
/*  1. 清除错误计数, 设置在线位                                          */
/*  2. 提取实时数据 (profile->realtime_bytes: 普通20B/高频56B) 到 SDRAM  */
/*  3. 检查 KGL 报警位, 更新 ALARM_TEMP                                 */
/*  4. 保存通讯确认码 (一直累加, 回显做水位ACK)                          */
/*  5. 剩余未读数 → 记录下次请求要读的脉冲个数                          */
/*  6. 本次请求带了脉冲个数时, 应答里含脉冲块 → 解析上传                 */
/*======================================================================*/
static void uart_axle_counter_online(UART_PortRuntime_t *runtime,
                                     uint8_t module_idx,
                                     const UART_ModuleProfile_t *profile,
                                     uint16_t rx_len, uint16_t *dc_offset)
{
    uint8_t *resp = (uint8_t *)&SerialDataBuf[runtime->port_idx][11];
    uint16_t comm_confirm;
    uint16_t unread_pulses;
    uint16_t kgl_val;
    uint16_t requested;
    uint16_t rt_regs;   /* 本模块实时寄存器个数 (普通计轴=10, 高频计轴=28) */
    uint16_t rt_bytes;  /* 本模块实时数据字节数 (普通计轴=20, 高频计轴=56) */
    uint16_t cc_off;    /* 通讯确认码在应答中的偏移 (reg rt+2) */
    uint16_t un_off;    /* 剩余未读脉冲数偏移             (reg rt+3) */
    uint16_t pulse_off; /* 脉冲块起始偏移                  (reg rt+4) */

    rt_regs  = profile->realtime_bytes / 2U;
    rt_bytes = profile->realtime_bytes;

    runtime->module_error[module_idx] = 0;
    runtime->link_mask |= (1UL << module_idx);

    /* ---- 提取实时数据 rt_bytes 字节 (resp[2] = 跳过 addr+func, 无byte_count) ---- */
    if (rx_len >= (uint16_t)(2U + rt_bytes + 2U)) {
        memcpy(&runtime->ssz_buf[*dc_offset], &resp[2], rt_bytes);
        uart_swap16(&runtime->ssz_buf[*dc_offset], rt_regs);
        runtime->data_len += rt_bytes;
    }

    /* ---- 报警检查: KGL 寄存器 (寄存器1, resp offset 2+2=4) ---- */
    kgl_val = runtime->ssz_buf[*dc_offset + 1];  /* 已做 swap16, 第二个16位字 = 寄存器1 */
    if ((kgl_val & 0x3fU) == 0U) {
        ALARM_TEMP[runtime->port_idx] |= (1UL << module_idx);
    } else {
        ALARM_TEMP[runtime->port_idx] &= ~(1UL << module_idx);
    }

    /* ---- 本次请求帧第5-6字节带的脉冲个数 (决定本条应答是否含脉冲块) ---- */
    requested = axle_counter_state[runtime->port_idx].pending_pulse_count[module_idx];

    /*
     * 控制寄存器 (通讯确认码/剩余未读) 与脉冲块紧跟实时寄存器块之后, 与普通
     * 计轴(0x50)相同的布局: 实时块后还有 2 个保留寄存器, 然后才是控制寄存器.
     *   普通(0x50): 实时 reg0-9 → reg10/11保留 → reg12确认码 → reg13未读 → 脉冲块
     *                确认码=resp[26], 未读=resp[28], 脉冲块=resp[30]
     *   高频(0x53): 实时 reg0-27 → reg28/29保留 → reg30确认码 → reg31未读 → 脉冲块
     *                确认码=resp[62], 未读=resp[64], 脉冲块=resp[66]
     * 通用公式 (reg N 位于 resp[2+2N]): 确认码=reg(rt+2), 未读=reg(rt+3), 脉冲块=reg(rt+4).
     */
    cc_off    = (uint16_t)(2U + (uint16_t)(rt_regs + 2U) * 2U);
    un_off    = (uint16_t)(2U + (uint16_t)(rt_regs + 3U) * 2U);
    pulse_off = (uint16_t)(2U + (uint16_t)(rt_regs + 4U) * 2U);

    /* ---- 读取通讯确认码 ---- */
    comm_confirm = ((uint16_t)resp[cc_off] << 8) | (uint16_t)resp[cc_off + 1U];

    /* ---- 检查未读脉冲数 ---- */
    unread_pulses = ((uint16_t)resp[un_off] << 8) | (uint16_t)resp[un_off + 1U];

    /*
     * 通讯确认码一直累加: 总是保存最新值, 下次请求帧第3-4字节原样带回 (水位ACK).
     * 采集器靠回显值判断通信板已收到到哪一帧, 从而清除已确认的脉冲缓存.
     */
    axle_counter_state[runtime->port_idx].comm_confirm[module_idx] = comm_confirm;

    /*
     * 只有本次请求带了脉冲个数 (>0) 时, 本条应答里才有脉冲块.
     * 脉冲块紧随寄存器块之后:
     *   resp[0..1] = 头(2B), resp[2..] = 寄存器块 ((rt_regs+4)*2 字节),
     *   resp[pulse_off..] = 脉冲块, resp[last-2..last-1] = CRC.
     * 脉冲数据长度 = rx_len - 2(头) - 寄存器块 - 2(CRC)
     *              = rx_len - 2 - (rt_bytes+8) - 2 = rx_len - rt_bytes - 12.
     * rx_len 必须 ≥ rt_bytes+12 才有脉冲块 (截断帧直接跳过, 防下溢).
     */
    if (requested > 0U) {
        /*
         * 每收到一包脉冲块就立即上传: 按通道分组, 每个通道一个 0x09 帧,
         * 同一通道的脉冲合并进一帧; 帧头时间戳 = 该通道最早脉冲的时间.
         *
         * [结束标志] 固定 = 1 (曲线采集结束), 每个通道帧都带上.
         *
         *   原因: 计轴是连续采集的活传感器, 读取期间仍有新脉冲持续入队, 采集盒
         *   回的 reg13(剩余未读) 基本不会等于本包实发数, 原判据
         *   (reg13==0 或 reg13==本包实发数) 于是几乎恒判为"未结束"(0); 而上位机
         *   只展示结束标志=1 的曲线 → 首包及后续包全部不显示.
         *   上位机按"每包即一条完整曲线"处理, 故对齐低频那版做法统一置 1.
         *   (协议文档第6节的原判据作废)
         */
        if (rx_len >= (uint16_t)(rt_bytes + 12U)) {
            uint16_t pulse_len = (uint16_t)(rx_len - (uint16_t)(rt_bytes + 12U));

            uart_axle_counter_upload_pulse(runtime->port_idx, module_idx,
                                           &resp[pulse_off], pulse_len,
                                           0x01U, runtime->link_mask);
        }
    }

    /*
     * 剩余未读脉冲数 (reg rt+3) 作为下次请求帧第5-6字节继续读取,
     * 为 0 时下次就是普通查询 (第5-6字节 = 0).
     */
    axle_counter_state[runtime->port_idx].pending_pulse_count[module_idx] = unread_pulses;

    *dc_offset += rt_regs;
}


/*======================================================================*/
/*  uart_axle_counter_offline                                           */
/*  计轴模块离线处理: SDRAM 填充 0xFFFF, 累计错误计数                    */
/*======================================================================*/
static void uart_axle_counter_offline(UART_PortRuntime_t *runtime,
                                      uint8_t module_idx,
                                      const UART_ModuleProfile_t *profile,
                                      uint16_t *dc_offset)
{
    uint16_t i;
    uint16_t rt_regs = profile->realtime_bytes / 2U;

    if (runtime->module_error[module_idx] > 3U) {
        runtime->link_mask &= ~(1UL << module_idx);
    } else {
        runtime->module_error[module_idx]++;
    }

    /* 填充 0xFFFF (离线标志), 按本模块实时寄存器个数 (普通10reg/高频28reg) */
    for (i = 0; i < rt_regs; i++) {
        runtime->ssz_buf[*dc_offset + i] = 0xFFFFU;
    }

    *dc_offset += rt_regs;
    runtime->data_len += profile->realtime_bytes;
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
    const UART_ModuleProfile_t *profile;

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
        /* 模块类型决定实时寄存器个数: 普通计轴(0x50)=20B, 高频计轴(0x53)=56B */
        profile = uart_get_module_profile(
            BoardCfgTab[runtime->port_idx].SlaveModuelType[m]);

        /* ---- 校时 (每个模块只需发送一次) ---- */
        if (changetime[runtime->port_idx] == 1U) {
            (void)uart_custom_send_adjust_time(
                runtime->port_idx,
                (uint8_t)runtime->port_idx,
                BoardCfgTab[runtime->port_idx].SlaveModuelAddress[m],
                Systemtime);
            TaskAdjustNum[runtime->port_idx]++;
        }

        /* 离线跳过: 连续3次失败后不再发请求/等超时, 每10个周期探测一次 */
        if (!uart_module_should_poll(runtime, m)) {
            uart_axle_counter_offline(runtime, m, profile, &dc_offset);
            WDTUARTFLAG[runtime->port_idx] = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* ---- 发送 0x03 查询 + 接收应答 ---- */
        rx_len = uart_axle_counter_module_exchange(runtime->port_idx, m);

        if (rx_len > 0U) {
            /* CRC OK — 在线, 提取实时数据 + 脉冲检测 */
            uart_axle_counter_online(runtime, m, profile, rx_len, &dc_offset);
        } else {
            /* CRC 失败或超时 — 离线 */
            uart_axle_counter_offline(runtime, m, profile, &dc_offset);
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
                                  40, 20);  /* 首字节超时 80→40ms */
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

        /* 离线跳过: 连续3次失败后不再发请求/等超时, 每10个周期探测一次 */
        if (!uart_module_should_poll(runtime, m)) {
            uart_dc3wire_offline(runtime, m, &dc_offset);
            WDTUARTFLAG[runtime->port_idx] = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
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
