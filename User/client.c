/**
 *********************************************************************
 * @file    client.c
 * @brief   lwIP TCP 客户端 — 连接/接收/发送队列
 *
 * 架构说明:
 *   - client_thread:    负责 TCP 连接管理和数据接收 (处理上位机下发的
 *                        配置/校时/复位/心跳等命令)
 *   - client_send_pending: 由 App_Task_TcpSend 任务周期性调用,
 *                          将发送队列中的数据通过 netconn_write 发出
 *   - client_add_to_tcp_send_buf: 各串口任务调用, 将数据入队
 *
 * 配置下发流程:
 *   1. 上位机下发 0xA1 0x2C 0x01 配置帧
 *   2. 与 EEPROM 中旧配置比较
 *   3. 相同 → 直接设置 download_flag = 1
 *   4. 不同 → 保存曲线信息到 EEPROM, 保存新配置到 EEPROM, 触发看门狗复位
 *
 * 收发分离:
 *   原程序有独立的 App_Task_TcpSend 任务和 App_TaskClient 任务.
 *   本实现中, client_thread 处理接收, client_send_pending 由
 *   外部的 App_Task_TcpSend 任务调用, 实现收发分离.
 *********************************************************************
 */

#include "client.h"
#include "bsp_rs485.h"
#include "bsp_eeprom.h"
#include "debug.h"
#include "./sdram/bsp_sdram.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/ip4_addr.h"
#include "lwip/err.h"

#include <string.h>
#include <stdint.h>

/*======================================================================*/
/*  调试追踪开关 —— 定位"频繁向上位机申请配置 / 反复重连"                   */
/*                                                                      */
/*  置 1: 打印 [TCP] 追踪                                              */
/*        - 每次连接建立的序号 (重连次数一目了然)                         */
/*        - tcp_connected 1→0 的发生位置 + err + reset_flag/WTD_RESET    */
/*        - netconn_recv 的异常返回码 (ERR_CLOSED 会暴露"对端已断开但      */
/*          本端不知情"的空转)                                           */
/*        - 申请配置帧的入队时机 (连接建立 / 重试定时器) 与配置生效        */
/*  置 0: 所有追踪打印消失, 不影响任何逻辑. (发布版 = 0)                  */
/*  ⚠ 需 DEBUG_UART_ENABLE=1 (bsp_debug_usart.h), 否则 printf 静默.       */
/*======================================================================*/
#define TCP_TRACE_ENABLE               0

#if TCP_TRACE_ENABLE
#define TCP_TRACE(fmt, args...)        do{printf("\r\n[TCP] " fmt "\r\n", ##args);}while(0)
#else
#define TCP_TRACE(fmt, args...)
#endif

/*======================================================================*/
/*  服务器地址 & 端口 (从 lwIP 配置继承)                                   */
/*======================================================================*/
#define TCP_SERVER_IP1                 DEST_IP_ADDR0
#define TCP_SERVER_IP2                 DEST_IP_ADDR1
#define TCP_SERVER_IP3                 DEST_IP_ADDR2
#define TCP_SERVER_IP4                 DEST_IP_ADDR3
#define TCP_SERVER_PORT                DEST_PORT

/*======================================================================*/
/*  队列与缓冲区常量                                                      */
/*  队列数 = UART_PORT_COUNT (6路串口) + 1 (控制通道/日志)                */
/*======================================================================*/
#define TCP_SEND_QUEUE_COUNT           (UART_PORT_COUNT + 1U)
#define TCP_SEND_BUF_SIZE              4096U   /* 4KB: 匹配 UART_SERIAL_BUF_SIZE */
#define TCP_RECV_BUF_SIZE              512U
#define TCP_SEND_RETRY_COUNT           10U     /* 单帧发送重试次数 */
/*
 * 连续接收超时 (ERR_TIMEOUT) 多少次后断开重连.
 * 每轮 ≈ 100ms(netconn_recv 超时) + 10ms(vTaskDelay) = 110ms,
 * 1200 次 ≈ 130s 的"上位机全程无数据"才断链.
 *
 * 为什么不是 200 (≈22s): 实测上位机的报文间隔在 16~57s 之间波动,
 * 22s 的阈值正落在这个区间里, 导致链路时不时被抽签式断开 ——
 * 每次重连都会无条件重发一次申请配置帧, 面板工作灯也会闪灭一下.
 * 130s 留了 2 倍余量, 对端真失联时 2 分钟内仍能自愈.
 *
 * ⚠ 计数器类型必须够宽 (见 client_thread 里的 recv_err_cnt: uint16_t).
 */
#define TCP_RECV_ERR_MAX               1200U     /* 连续接收超时阈值 → 重连 (≈130s) */
#define CONFIG_REQ_RETRY_INTERVAL      100U      /* 配置请求重试间隔 (循环次数) */

/*======================================================================*/
/*  曲线 FIFO 常量 (放在 SDRAM, 不丢包)                                   */
/*  - 实时数据: 覆盖式 (每端口1槽, 只保留最新帧)                           */
/*  - 曲线数据: FIFO 追加式 (不覆盖, 不丢包)                              */
/*======================================================================*/
#define CURVE_FIFO_DEPTH              16U    /* 曲线 FIFO 深度 */
/* QX 缓存区结束于 0xC0005000 + 6×0x1E000 = 0xC00B9000, FIFO 基址取其后 */
#define CURVE_FIFO_BASE_ADDR          (SDRAM_BASE_ADDR + 0xC0000U)

/*======================================================================*/
/*  网络命令字 (与上位机协议)                                              */
/*======================================================================*/
#define NET_CMD_CONFIG                 0x00
#define NET_CMD_CONFIG_MAGIC0          0xA1
#define NET_CMD_CONFIG_MAGIC1          0xC2
#define NET_CMD_CONFIG_MAGIC2          0x01
#define NET_CMD_TIME_MAGIC0            0xA3
#define NET_CMD_TIME_MAGIC1            0x04
#define NET_CMD_TIME_MAGIC2            0x00
#define NET_CMD_RESET_MAGIC0           0xA2
#define NET_CMD_RESET_MAGIC1           0x00
#define NET_CMD_RESET_MAGIC2           0x00
#define NET_CMD_HEARTBEAT_MAGIC0       0xA5
#define NET_CMD_HEARTBEAT_MAGIC1       0x00
#define NET_CMD_HEARTBEAT_MAGIC2       0x00

/* ---- EEPROM 地址布局 (MODE_8_BIT, 即块号) ---- */
/* AT24C02: 2Kbit = 256Byte                               */
#define EEPROM_BLOCK_CONFIG            0U    /* BoardCfgTab: 块0, BOARD_CONFIG_BYTES 字节 */
#define EEPROM_BLOCK_CURVE_PORT0       5U    /* port0 曲线: 块5, CURVE_EEPROM_SIZE 字节    */
#define EEPROM_BLOCK_CURVE_PORT1       7U    /* port1 曲线: 块7, CURVE_EEPROM_SIZE 字节    */
#define EEPROM_BLOCK_CURVE_PORT2       9U    /* port2 曲线: 块9, CURVE_EEPROM_SIZE 字节    */
#define EEPROM_BLOCK_CURVE_PORT3       11U   /* port3 曲线: 块11, CURVE_EEPROM_SIZE 字节   */
#define EEPROM_BLOCK_CURVE_PORT4       13U   /* port4 曲线: 块13, CURVE_EEPROM_SIZE 字节   */
#define EEPROM_BLOCK_CURVE_PORT5       15U   /* port5 曲线: 块15, CURVE_EEPROM_SIZE 字节   */

/* ---- Curve EEPROM 块号查找表 (6 端口) ---- */
static const uint8_t ee_curve_block[UART_PORT_COUNT] = {
    EEPROM_BLOCK_CURVE_PORT0,
    EEPROM_BLOCK_CURVE_PORT1,
    EEPROM_BLOCK_CURVE_PORT2,
    EEPROM_BLOCK_CURVE_PORT3,
    EEPROM_BLOCK_CURVE_PORT4,
    EEPROM_BLOCK_CURVE_PORT5
};

/*======================================================================*/
/*  申请配置帧 (TCP 连接成功后立即发送)                                    */
/*  帧格式: [0x00, 0x07, 0x01, 0xFF, 0xFF, 0x04, 0x00, 0xFF,0xFF,0xFF,0xFF] */
/*======================================================================*/
static const uint8_t request_config_frame[11] = {
    0x00, 0x07, 0x01, 0xFF, 0xFF, 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0xFF
};

/*======================================================================*/
/*  TCP 发送槽位结构体 (与原程序 TcpWaitingSendBuf 一致)                   */
/*======================================================================*/
typedef struct {
    volatile uint8_t ready;          /* 1 = 有待发送数据, 0 = 空闲          */
    uint8_t          port_flag;      /* 槽位编号 (0-5=端口, 6=控制/日志)   */
    uint8_t          reset_flag;     /* 看门狗复位码 (发送失败时触发)       */
    uint16_t         len;            /* 数据长度                            */
    uint8_t          send_buf[TCP_SEND_BUF_SIZE];
} TcpSendSlot_t;

/*======================================================================*/
/*  曲线 FIFO 槽位 (放在 SDRAM)                                           */
/*  - 曲线数据不可丢: FIFO 追加式, 而非实时数据的覆盖式.                   */
/*======================================================================*/
typedef struct {
    uint16_t len;                       /* 数据长度 (2B, 对齐) */
    uint8_t  reset_flag;                /* 看门狗复位码 */
    uint8_t  send_buf[TCP_SEND_BUF_SIZE];
} CurveFifoSlot_t;                      /* 4100 B, 4 字节对齐 */

typedef struct {
    volatile uint32_t head;             /* 发送任务读取位置 */
    volatile uint32_t tail;             /* 串口任务写入位置 */
    volatile uint32_t count;            /* 待发送帧数 (区分空/满) */
    CurveFifoSlot_t   slot[CURVE_FIFO_DEPTH];
} CurveFifo_t;

/*======================================================================*/
/*  静态全局变量                                                          */
/*======================================================================*/
static TcpSendSlot_t     tcp_waiting_send_buf[TCP_SEND_QUEUE_COUNT];
static SemaphoreHandle_t tcp_queue_lock;
static struct netconn   *tcp_conn = NULL;
static volatile uint8_t  tcp_connected = 0U;
static uint8_t           tcp_recv_buf[TCP_RECV_BUF_SIZE];
static CurveFifo_t      *curve_fifo = NULL;   /* 指向 SDRAM 曲线 FIFO (client_init 初始化) */

/*======================================================================*/
/*  前向声明                                                              */
/*======================================================================*/
static void client_thread(void *thread_param);
static void client_handle_rx(const uint8_t *buf, uint16_t len);
static uint8_t client_board_config_valid(void);
#if 0  /* 通信板就绪后改为 #if 1 */
static void client_save_curve_to_eeprom(void);
#endif

/*======================================================================*/
/*  client_is_connected()                                                */
/*  返回 TCP 连接状态, 供外部模块查询                                      */
/*======================================================================*/
uint8_t client_is_connected(void)
{
    return tcp_connected;
}

/*======================================================================*/
/*  client_disconnect()                                                  */
/*  主动断开 TCP 连接                                                      */
/*======================================================================*/
void client_disconnect(void)
{
    TCP_TRACE("tcp_connected 1->0 @ client_disconnect()");
    tcp_connected = 0U;
}

/*======================================================================*/
/*  client_add_to_tcp_send_buf()                                         */
/*  将数据放入 TCP 发送队列 (覆盖式, 只保留最新帧)                          */
/*  - buf:        数据指针                                                */
/*  - len:        数据长度                                                */
/*  - reset_flag: 发送失败时的看门狗复位码                                  */
/*  - port_flag:  槽位编号 (0-5=端口数据, TCP_SEND_QUEUE_COUNT-1=控制通道) */
/*======================================================================*/
void client_add_to_tcp_send_buf(const void *buf, uint16_t len,
                                 uint8_t reset_flag, uint8_t port_flag)
{
    TcpSendSlot_t *slot;

    if ((buf == NULL) || (len == 0U)) {
        return;
    }

    if (port_flag >= TCP_SEND_QUEUE_COUNT) {
        port_flag = (uint8_t)(TCP_SEND_QUEUE_COUNT - 1U);
    }
    if (len > TCP_SEND_BUF_SIZE) {
        len = TCP_SEND_BUF_SIZE;
    }

    slot = &tcp_waiting_send_buf[port_flag];

    /* ---- 覆盖式入队: 槽位满则直接覆盖旧数据 (实时数据只保留最新帧, 不阻塞) ---- */
    if (tcp_queue_lock != NULL) {
        xSemaphoreTake(tcp_queue_lock, portMAX_DELAY);
    }

    slot->port_flag  = port_flag;
    slot->reset_flag = reset_flag;
    memcpy(slot->send_buf, buf, len);
    slot->len   = len;
    slot->ready = 1U;

    if (tcp_queue_lock != NULL) {
        xSemaphoreGive(tcp_queue_lock);
    }
}

/*======================================================================*/
/*  client_add_curve_to_tcp_send_buf()                                   */
/*  将曲线数据追加进 SDRAM 曲线 FIFO (不覆盖, 不丢包).                     */
/*  - buf:        数据指针                                                */
/*  - len:        数据长度                                                */
/*  - reset_flag: 发送失败时的看门狗复位码                                  */
/*  FIFO 满时阻塞等待 (背压), 保证曲线帧一定被发出.                         */
/*======================================================================*/
void client_add_curve_to_tcp_send_buf(const void *buf, uint16_t len,
                                      uint8_t reset_flag)
{
    CurveFifoSlot_t *slot;

    if ((curve_fifo == NULL) || (buf == NULL) || (len == 0U)) {
        return;
    }
    if (len > TCP_SEND_BUF_SIZE) {
        len = TCP_SEND_BUF_SIZE;
    }

    /* ---- 背压: FIFO 满则等待发送任务取走 (曲线数据不丢, 宁可短暂阻塞) ---- */
    for (;;) {
        if (tcp_queue_lock != NULL) {
            xSemaphoreTake(tcp_queue_lock, portMAX_DELAY);
        }
        if (curve_fifo->count < CURVE_FIFO_DEPTH) {
            break;
        }
        if (tcp_queue_lock != NULL) {
            xSemaphoreGive(tcp_queue_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    slot = &curve_fifo->slot[curve_fifo->tail];
    slot->reset_flag = reset_flag;
    memcpy(slot->send_buf, buf, len);
    slot->len = len;

    curve_fifo->tail = (curve_fifo->tail + 1U) % CURVE_FIFO_DEPTH;
    curve_fifo->count++;

    if (tcp_queue_lock != NULL) {
        xSemaphoreGive(tcp_queue_lock);
    }
}

/*======================================================================*/
/*  client_send_frame()                                                  */
/*  发送一帧数据 (带重试), 返回 err_t.                                     */
/*======================================================================*/
static err_t client_send_frame(const uint8_t *buf, uint16_t len)
{
    uint8_t retry = TCP_SEND_RETRY_COUNT;
    err_t   err   = ERR_OK;

    do {
        err = netconn_write(tcp_conn, buf, len, NETCONN_COPY);
        if (err == ERR_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (retry-- > 0U);

    return err;
}

/*======================================================================*/
/*  client_send_pending()                                                */
/*  遍历发送队列, 将待发送数据通过 netconn_write 发出                       */
/*  由 App_Task_TcpSend 任务周期性调用 (与原程序 App_Task_TcpSend 一致)     */
/*                                                                       */
/*  发送顺序:                                                             */
/*    1) 实时/控制槽 (覆盖式, 每端口只保留最新帧)                          */
/*    2) 曲线 FIFO   (追加式, 不丢包)                                     */
/*                                                                       */
/*  重试机制: 每帧最多重试 TCP_SEND_RETRY_COUNT 次, 全部失败则断开连接      */
/*  看门狗: 发送失败时通过 reset_flag 触发 WTD_RESET                       */
/*======================================================================*/
void client_send_pending(void)
{
    uint8_t i;
    TcpSendSlot_t *slot;
    err_t    err;
    static uint8_t send_copy[TCP_SEND_BUF_SIZE];   /* 发送本地缓冲 (静态, 避免占用任务栈) */
    uint16_t copy_len;
    uint8_t  copy_reset_flag;

    if (tcp_conn == NULL || tcp_connected == 0U) {
        return;
    }

    /* ---- 1) 实时/控制槽: 拷出并立即释放槽位 (覆盖式, 不拖住串口轮询) ---- */
    for (i = 0; i < TCP_SEND_QUEUE_COUNT; i++) {
        slot = &tcp_waiting_send_buf[i];

        if (slot->ready == 0U) {
            continue;
        }

        if (tcp_queue_lock != NULL) {
            xSemaphoreTake(tcp_queue_lock, portMAX_DELAY);
        }
        copy_len        = slot->len;
        copy_reset_flag = slot->reset_flag;
        memcpy(send_copy, slot->send_buf, copy_len);
        slot->len       = 0;
        slot->ready     = 0U;
        if (tcp_queue_lock != NULL) {
            xSemaphoreGive(tcp_queue_lock);
        }

        err = client_send_frame(send_copy, copy_len);
        if (err != ERR_OK) {
            if (copy_reset_flag != 0U) {
                WTD_RESET = copy_reset_flag;
            }
            TCP_TRACE("tcp_connected 1->0 @ send_pending/realtime slot%u"
                      " len=%u err=%d reset_flag=%u WTD_RESET=%u",
                      (unsigned)i, (unsigned)copy_len, (int)err,
                      (unsigned)copy_reset_flag, (unsigned)WTD_RESET);
            tcp_connected = 0U;
            return;
        }

    }

    /* ---- 2) 曲线 FIFO: 逐帧取出发送 (追加式, 不丢包) ---- */
    for (;;) {
        CurveFifoSlot_t *cslot;

        if (tcp_queue_lock != NULL) {
            xSemaphoreTake(tcp_queue_lock, portMAX_DELAY);
        }
        if (curve_fifo == NULL || curve_fifo->count == 0U) {
            if (tcp_queue_lock != NULL) {
                xSemaphoreGive(tcp_queue_lock);
            }
            break;
        }

        cslot = &curve_fifo->slot[curve_fifo->head];
        copy_len        = cslot->len;
        copy_reset_flag = cslot->reset_flag;
        memcpy(send_copy, cslot->send_buf, copy_len);
        curve_fifo->head  = (curve_fifo->head + 1U) % CURVE_FIFO_DEPTH;
        curve_fifo->count--;

        if (tcp_queue_lock != NULL) {
            xSemaphoreGive(tcp_queue_lock);
        }

        err = client_send_frame(send_copy, copy_len);
        if (err != ERR_OK) {
            if (copy_reset_flag != 0U) {
                WTD_RESET = copy_reset_flag;
            }
            TCP_TRACE("tcp_connected 1->0 @ send_pending/curve fifo"
                      " len=%u err=%d reset_flag=%u WTD_RESET=%u",
                      (unsigned)copy_len, (int)err,
                      (unsigned)copy_reset_flag, (unsigned)WTD_RESET);
            tcp_connected = 0U;
            return;
        }
    }
}

/*======================================================================*/
/*  client_board_config_valid()                                          */
/*  校验下发的配置是否合法 (每路从机数不超过最大值)                          */
/*======================================================================*/
static uint8_t client_board_config_valid(void)
{
    uint8_t i;

    for (i = 0; i < UART_PORT_COUNT; i++) {
        if (BoardCfgTab[i].SlaveModuelNum > UART_MODULE_MAX_COUNT) {
            return 0U;
        }
    }
    return 1U;
}

/*======================================================================*/
/*  client_save_curve_to_eeprom()                                        */
/*  将所有 6 路端口的曲线信息保存到 EEPROM                                  */
/*  在原程序配置变更时调用, 防止曲线数据丢失                                */
/*  通信板就绪后: 将下方 #if 0 改为 #if 1 恢复此函数                        */
/*======================================================================*/

static void client_save_curve_to_eeprom(void)
{
    uint8_t i;

    for (i = 0; i < UART_PORT_COUNT; i++) {
        (void)EEPROM_Write(0, (uint32_t)ee_curve_block[i],
                           (void *)&AC_module_infor[i].Curve_number[0],
                           MODE_8_BIT, CURVE_EEPROM_SIZE);
    }
}


/*======================================================================*/
/*  client_handle_rx()                                                   */
/*  处理上位机下发的命令帧                                                 */
/*                                                                       */
/*  命令类型:                                                             */
/*   [0xA1,0x2C,0x01] — 配置下发 (含 EEPROM 持久化)                       */
/*   [0xA3,0x04,0x00] — 校时命令                                         */
/*   [0xA2,0x00,0x00] — 软复位 / 看门狗复位                               */
/*   [0xA5,0x00,0x00] — 心跳应答                                          */
/*======================================================================*/
static void client_handle_rx(const uint8_t *buf, uint16_t len)
{
    uint16_t copy_len;
    uint8_t  i;

    if ((buf == NULL) || (len < 4U)) {
        return;
    }

    /* ---- 仅处理 CONFIG_PORT (0x00) 命令 ---- */
    if (buf[0] != NET_CMD_CONFIG) {
        return;
    }

    /*==================================================================*/
    /*  配置下发: [0x00, 0xA1, 0xC2, 0x01, ...config_data...]           */
    /*==================================================================*/
    // if ((buf[1] == NET_CMD_CONFIG_MAGIC0) &&
    //     (buf[2] == NET_CMD_CONFIG_MAGIC1) &&
    //     (buf[3] == NET_CMD_CONFIG_MAGIC2)) 
        if ((buf[1] == NET_CMD_CONFIG_MAGIC0) ) 
        {

        copy_len = (uint16_t)(len - 4U);
        if (copy_len > BOARD_CONFIG_BYTES) {
            copy_len = BOARD_CONFIG_BYTES;
        }


        /* ---- 将接收到的配置写入 BoardCfgTab ---- */
        memset(BoardCfgTab, 0, BOARD_CONFIG_BYTES);
        memcpy(BoardCfgTab, &buf[4], copy_len);

        /* ---- 校验配置合法性 ---- */
        if (client_board_config_valid() == 0U) {
            return;  /* 配置非法, 丢弃 */
        }
#if 0   /* ====== 调试阶段: 绕过 EEPROM (AT24C02 容量 256<450) ====== */
        /*
         * 原有 EEPROM 配置持久化流程.
         * 通信板 (AT24C64D) 就绪后, 将上方 #if 0 改为 #if 1 即可恢复.
         */
        uint8_t ee_ret;

        /*
         * ---- EEPROM 配置持久化流程 ----
         * 1. 读取 EEPROM 中的旧配置
         * 2. 与新的 BoardCfgTab 比较
         * 3. 相同 → 说明之前已保存过, 直接设置 download_flag
         * 4. 不同 → 先保存曲线信息, 再保存新配置, 触发看门狗复位
         */
        ee_ret = EEPROM_Read(0, EEPROM_BLOCK_CONFIG,
                             (void *)MramCfgDataBuf,
                             MODE_8_BIT, BOARD_CONFIG_BYTES);
        if (ee_ret != 0U) {
            /*
             * EEPROM 读取失败 (首次启动 EEPROM 为空属正常情况,
             * 此时 MramCfgDataBuf 保持 BSS 零值, memcmp 不匹配后
             * 将进入写入分支).
             * 如果反复出现此日志, 请检查 I2C 硬件连接.
             */
        }

        if (memcmp(MramCfgDataBuf, BoardCfgTab, BOARD_CONFIG_BYTES) == 0) {
            /*
             * 配置与 EEPROM 中存储的一致 — 之前已保存过.
             * 注意: 原程序在此处有一大段注释掉的代码,
             * 其逻辑是从 EEPROM 恢复曲线编号 (防止因复位丢失).
             * 当前实现跳过此恢复逻辑 (与原程序最终行为一致).
             */
            download_flag = 1U;
        } else {
            /*
             * 配置变更 — 先保存所有端口的曲线信息 (防丢失),
             * 再保存新配置到 EEPROM, 最后触发看门狗复位.
             */
            client_save_curve_to_eeprom();
            ee_ret = EEPROM_Write(0, EEPROM_BLOCK_CONFIG,
                                  (void *)&BoardCfgTab[0].PortNum,
                                  MODE_8_BIT, BOARD_CONFIG_BYTES);
            if (ee_ret != 0U) {
                /* EEPROM 写入失败! 配置未能持久化, 复位后仍为旧配置.
                 * 可能原因: I2C 通信故障 / EEPROM 芯片损坏 / WP 被拉高.
                 */
            }
            WTD_RESET = 1U;   /* 看门狗复位码: 1 = 配置变更复位 */
        }
#endif /* 调试阶段: 绕过 EEPROM */

        /*
         * ---- 调试阶段: 配置校验通过后直接生效 (不经过 EEPROM) ----
         * 通信板就绪后, 删除下面两行, 并将上方 #if 0 改为 #if 1.
         */
        if (download_flag == 0U) {
            TCP_TRACE("config accepted (len=%u) -> download_flag=1,"
                      " stop requesting config",
                      (unsigned)len);
        }
        download_flag = 1U;


    }
    /*==================================================================*/
    /*  校时命令: [0x00, 0xA3, 0x04, 0x00, timestamp(4 bytes)]         */
    /*  设置系统时间并标记全部端口需要对曲线模块校时                          */
    /*==================================================================*/
    else if ((buf[1] == NET_CMD_TIME_MAGIC0) &&
             (buf[2] == NET_CMD_TIME_MAGIC1) &&
             (buf[3] == NET_CMD_TIME_MAGIC2)) {
        if (len >= 8U) {
            Systemtime = *((CPU_INT32U *)&buf[4]);

            /* 设置所有 6 路端口的校时标志 */
            for (i = 0; i < UART_PORT_COUNT; i++) {
                changetime[i]   = 1U;
                TaskAdjustNum[i] = 0U;
            }
            jiaoshi = 1U;  /* 触发校时日志上报 */
        }
    }
    /*==================================================================*/
    /*  软复位命令: [0x00, 0xA2, 0x00, 0x00]                             */
    /*==================================================================*/
    else if ((buf[1] == NET_CMD_RESET_MAGIC0) &&
             (buf[2] == NET_CMD_RESET_MAGIC1) &&
             (buf[3] == NET_CMD_RESET_MAGIC2)) {
        WTD_RESET = 2U;   /* 看门狗复位码: 2 = 上位机软复位命令 */
    }
    /*==================================================================*/
    /*  心跳应答: [0x00, 0xA5, 0x00, 0x00]                                */
    /*  收到后清零心跳计数器, 防止因心跳超时而复位                           */
    /*==================================================================*/
    else if ((buf[1] == NET_CMD_HEARTBEAT_MAGIC0) &&
             (buf[2] == NET_CMD_HEARTBEAT_MAGIC1) &&
             (buf[3] == NET_CMD_HEARTBEAT_MAGIC2)) {
        heartbeat = 0U;
        
    }
}

/*======================================================================*/
/*  client_thread()                                                      */
/*  TCP 客户端主线程:                                                     */
/*  1. 循环连接服务器                                                     */
/*  2. 连接成功后发送申请配置帧                                            */
/*  3. 循环接收数据 → client_handle_rx 处理                               */
/*  4. 连续接收错误超过阈值 → 断开重连                                     */
/*  5. 发送动作由外部 App_Task_TcpSend 任务完成 (收发分离)                  */
/*======================================================================*/
static void client_thread(void *thread_param)
{
    ip4_addr_t ipaddr;
    err_t      err;
    struct netbuf *rx_buf;
    void      *data;
    u16_t      data_len;
    uint16_t   recv_err_cnt      = 0U;   /* 必须 >= TCP_RECV_ERR_MAX 的位宽, 否则回绕 */
    uint16_t   config_req_timer  = 0U;
    static uint32_t connect_cnt  = 0U;       /* 调试追踪: 连接建立次数 (= 重连次数+1) */

    (void)thread_param;
    IP4_ADDR(&ipaddr, TCP_SERVER_IP1, TCP_SERVER_IP2,
             TCP_SERVER_IP3, TCP_SERVER_IP4);

    while (1) {
        /* ---- 创建 TCP 连接 ---- */
        tcp_conn = netconn_new(NETCONN_TCP);
        if (tcp_conn == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

#if LWIP_SO_RCVTIMEO
        /* 设置接收超时: 50ms, 避免永久阻塞 recv */
        netconn_set_recvtimeout(tcp_conn, 100); //test:50ms改成100ms
#endif

        /* ---- 连接服务器 ---- */
        err = netconn_connect(tcp_conn, &ipaddr, TCP_SERVER_PORT);
        if (err != ERR_OK) {
            netconn_delete(tcp_conn);
            tcp_conn = NULL;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* ---- 连接成功 ---- */
        tcp_connected = 1U;
        recv_err_cnt  = 0U;
        netReset      = 1U;    /* 触发 "网络重连成功" 日志 */
        connect_cnt++;
        TCP_TRACE("connected #%lu (0 = first connect since power-on)",
                  (unsigned long)connect_cnt - 1UL);

        /*
         * 立即发送申请配置帧 (通过控制通道槽位).
         * reset_flag = 29: 如果发送失败且超过重试次数,
         * 看门狗复位码 = 29 (申请配置失败).
         */
        client_add_to_tcp_send_buf(request_config_frame,
                                    sizeof(request_config_frame),
                                    29U,
                                    (uint8_t)(TCP_SEND_QUEUE_COUNT - 1U));
        TCP_TRACE("request_config queued (triggered by connect)");

        /* ---- 接收循环 ---- */
        config_req_timer = 0U;

        while (tcp_connected != 0U) {
            /* 注意: 发送由 App_Task_TcpSend 任务独立处理,
             * 不在此处调用 client_send_pending (收发分离) */

            err = netconn_recv(tcp_conn, &rx_buf);

            if (err == ERR_OK) {
                recv_err_cnt = 0U;

                /* 处理 netbuf 链表 (可能分片) */
                do {
                    netbuf_data(rx_buf, &data, &data_len);
                    if (data_len > TCP_RECV_BUF_SIZE) {
                        data_len = TCP_RECV_BUF_SIZE;
                    }
                    memcpy(tcp_recv_buf, data, data_len);
                    client_handle_rx(tcp_recv_buf, data_len);
                } while (netbuf_next(rx_buf) >= 0);
                netbuf_delete(rx_buf);
            } else if(err == ERR_TIMEOUT){
                /*
                 * 连续超过 TCP_RECV_ERR_MAX 次 → 断开重连.
                 */

                recv_err_cnt++;
                /* 调试追踪: 超时计数爬到阈值即断开, 每 200 次(≈22s)打印一次进度.
                 * 收到任何数据都会把计数清零(上方 ERR_OK 分支), 所以这里只统计
                 * "连续无数据"的时长. */
                if ((recv_err_cnt % 200U) == 0U) {
                    TCP_TRACE("recv timeout count %u/%u (download_flag=%u)",
                              (unsigned)recv_err_cnt, (unsigned)TCP_RECV_ERR_MAX,
                              (unsigned)download_flag);
                }
                if (recv_err_cnt >= TCP_RECV_ERR_MAX) {
                    TCP_TRACE("tcp_connected 1->0 @ recv ERR_TIMEOUT x%u"
                              " (threshold %u)",
                              (unsigned)recv_err_cnt, (unsigned)TCP_RECV_ERR_MAX);
                    tcp_connected = 0U;
                }
            } else {
                /* 其他接收错误 → 立即断开重连 */
                TCP_TRACE("tcp_connected 1->0 @ recv err=%d (not ERR_TIMEOUT)"
                          " connected=%u",
                          (int)err, (unsigned)tcp_connected);
                tcp_connected = 0U;
            }

            /*
             * 配置请求重试机制:
             * 如果上位机一直发心跳但未回复配置帧,
             * 每隔 CONFIG_REQ_RETRY_INTERVAL 次循环重发一次请求,
             * 直到 download_flag == 1 (配置已收到) 为止.
             */
            if (download_flag == 0U) {
                config_req_timer++;
                if (config_req_timer >= CONFIG_REQ_RETRY_INTERVAL) {
                    config_req_timer = 0U;
                    client_add_to_tcp_send_buf(request_config_frame,
                                                sizeof(request_config_frame),
                                                29U,
                                                (uint8_t)(TCP_SEND_QUEUE_COUNT - 1U));
                    TCP_TRACE("request_config queued (retry timer, every %u loops)",
                              (unsigned)CONFIG_REQ_RETRY_INTERVAL);
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        /* ---- 断开连接, 准备重连 ---- */
        TCP_TRACE("recv loop exit -> close and reconnect"
                  " (last err=%d, timeout cnt=%u)",
                  (int)err, (unsigned)recv_err_cnt);
        netconn_close(tcp_conn);
        netconn_delete(tcp_conn);
        tcp_conn = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*======================================================================*/
/*  client_init()                                                        */
/*  初始化 TCP 客户端: 创建互斥锁 + 启动 client_thread                     */
/*======================================================================*/
void client_init(void)
{
    tcp_queue_lock = xSemaphoreCreateMutex();

    /* ---- 初始化 SDRAM 曲线 FIFO (追加式, 不丢包) ---- */
    curve_fifo = (CurveFifo_t *)CURVE_FIFO_BASE_ADDR;
    memset(curve_fifo, 0, sizeof(CurveFifo_t));

    sys_thread_new("td_tcp_client", client_thread, NULL, 1024, 6);  
}
