#include "ota_client.h"
#include "ota_manager.h"
#include "ota_net.h"
#include "ota_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sys.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ==================================================================
 *  OTA 客户端任务
 *
 *  与上位机 (PC_Server/ota_gui.py) 的完整时序:
 *
 *    [连接建立]
 *      设备 → PC   HELLO        载荷 8B: 版本(4,LE) + 当前槽位(1) + 目标槽位(1) + 保留(2)
 *      PC   → 设备 PUSH_UPDATE  载荷 12B: 版本(4,LE) + 大小(4,LE) + CRC32(4,LE)
 *      设备 → PC   CONFIRM_UPDATE          ← 版本更新且大小合法
 *                 (或 REJECT_UPDATE / ERROR_UPDATE)
 *      设备擦除目标分区(1~5s)  ← 期间 CPU 被 Flash 总线 stall, 由 TIM7+DMA 硬件喂狗
 *      设备 → PC   ACK          载荷 4B: OTA_STATE_DOWNLOADING  ← "擦好了, 可以发数据"
 *      PC   → 设备 DATA xN      每帧 offset 递增
 *      PC   → 设备 END
 *      设备校验(长度 + CRC32 + 镜像结构)
 *      设备 → PC   ACK          载荷 4B: OTA_STATE_SUCCESS
 *      (或 ERROR_UPDATE 载荷 4B: 错误码)
 *      设备写 BootInfo → 重启 → Bootloader 跳到新分区
 *
 *  失败自恢复(P7):
 *    任何环节出错都不卡死 —— 回 ERROR_UPDATE, 断开连接, 回到 IDLE,
 *    10s 后重连并重新 HELLO(仍报老版本), PC 会在下一轮重新推送,
 *    因为当前运行分区全程未被破坏, 重试永远是从干净状态开始。
 * ================================================================== */

#define OTA_SERVER_IP           OTA_SERVER_IP_DEFAULT
#define OTA_SERVER_PORT         OTA_SERVER_PORT_DEFAULT

/* 收到 END 校验通过后, 等多久再重启(留给上位机收 ACK / 打印日志) */
#define OTA_REBOOT_DELAY_MS     3000U

typedef enum {
    SESS_IDLE = 0,
    SESS_DOWNLOADING,
    SESS_DONE
} ota_session_t;

static int32_t       g_sock  = -1;
static uint8_t       g_seq   = 0U;
static ota_session_t g_sess  = SESS_IDLE;
static uint8_t       g_reboot = 0U;
static uint8_t       g_need_reconnect = 0U;


/* ==================================================================
 *  发送辅助 (小端, 与上位机 struct.unpack('<I') 对应)
 * ================================================================== */
static int32_t client_send(uint8_t cmd, uint32_t offset,
                           const uint8_t *data, uint16_t len)
{
    int32_t rc;

    if (g_sock < 0) {
        return -1;
    }

    rc = ota_net_send(g_sock, cmd, g_seq++, offset, data, len);
    if (rc < 0) {
        printf("[OTA_CLIENT] send cmd=0x%02X failed\r\n", (unsigned)cmd);
    }
    return rc;
}

static void put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static uint32_t get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]        | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* HELLO: 上报版本 + 当前槽位 + 目标槽位, 让上位机自动挑对镜像 */
static int32_t client_send_hello(void)
{
    uint8_t payload[8];

    put_u32_le(&payload[0], ota_get_current_version());
    payload[4] = ota_get_current_slot();
    payload[5] = ota_get_target_slot();
    payload[6] = 0U;
    payload[7] = 0U;

    printf("[OTA_CLIENT] HELLO: ver=" FW_VER_FMT ", cur_slot=%c, target_slot=%c\r\n",
           FW_VER_ARG(ota_get_current_version()),
           OTA_SLOT_CHAR(payload[4]), OTA_SLOT_CHAR(payload[5]));

    return client_send(OTA_CMD_HELLO, 0U, payload, 8U);
}

/* ACK 载荷 = 一个状态码(小端) */
static int32_t client_send_ack(uint32_t state)
{
    uint8_t payload[4];

    put_u32_le(payload, state);
    printf("[OTA_CLIENT] ACK: state=%lu\r\n", (unsigned long)state);

    return client_send(OTA_CMD_ACK, 0U, payload, 4U);
}

/* 把管理器记录的错误码上报给上位机。
 *
 * 报完一律要求重连: 出错后两端的会话状态都已经不确定,
 * 重新建链 + HELLO 是最干净的恢复方式, 上位机也据此从头再来一次。 */
static void client_report_error(ota_state_t fallback)
{
    ota_state_t err = ota_take_error();
    uint8_t payload[4];

    if (err == OTA_STATE_IDLE) {
        err = fallback;
    }

    put_u32_le(payload, (uint32_t)err);
    printf("[OTA_CLIENT] ERROR_UPDATE: err=%u\r\n", (unsigned)err);

    (void)client_send(OTA_CMD_ERROR_UPDATE, 0U, payload, 4U);

    g_sess = SESS_IDLE;
    g_need_reconnect = 1U;
}


/* ==================================================================
 *  命令处理
 * ================================================================== */

/* PUSH_UPDATE: 版本 + 大小 + CRC32 */
static void client_handle_push(const ota_frame_t *f)
{
    uint32_t new_version;
    uint32_t new_size;
    uint32_t new_crc;
    int32_t  rc;

    if (f->len < 12U) {
        printf("[OTA_CLIENT] PUSH_UPDATE payload len invalid (%u)\r\n", (unsigned)f->len);
        (void)ota_handle_push_update_request(0U, 0U, 0U);   /* 记错误码 */
        client_report_error(OTA_STATE_ERROR_INVALID_FRAME);
        return;
    }

    new_version = get_u32_le(f->data + 0);
    new_size    = get_u32_le(f->data + 4);
    new_crc     = get_u32_le(f->data + 8);

    rc = ota_handle_push_update_request(new_version, new_size, new_crc);

    if (rc > 0) {
        /* 同意升级: 回 CONFIRM, 然后擦除目标分区 */
        if (client_send(OTA_CMD_CONFIRM_UPDATE, 0U, NULL, 0U) < 0) {
            client_report_error(OTA_STATE_SEND_COMFIRM_FAILED);
            return;
        }

        if (ota_start_download() < 0) {
            client_report_error(OTA_STATE_START_DOWNLOAD_FAILED);
            return;
        }

        /* 擦除已完成, 明确告诉上位机"可以发数据了" ——
         * 避免上位机在设备擦除(CPU stall 数秒)期间硬塞,
         * 那样只能靠 TCP 窗口兜着, 很脆。 */
        if (client_send_ack(OTA_STATE_DOWNLOADING) < 0) {
            client_report_error(OTA_STATE_SEND_COMFIRM_FAILED);
            return;
        }

        g_sess = SESS_DOWNLOADING;
    } else if (rc == 0) {
        printf("[OTA_CLIENT] no upgrade needed, reply REJECT\r\n");
        (void)client_send(OTA_CMD_REJECT_UPDATE, 0U, NULL, 0U);
    } else {
        client_report_error(OTA_STATE_ERROR_SIZE);
    }
}

/* DATA: 一个固件数据块 */
static void client_handle_data(const ota_frame_t *f)
{
    if (g_sess != SESS_DOWNLOADING) {
        printf("[OTA_CLIENT] DATA in non-download state, dropped (offset=%lu)\r\n",
               (unsigned long)f->offset);
        return;
    }

    if (ota_write_data_block(f->data, f->len, f->offset) < 0) {
        client_report_error(OTA_STATE_ERROR_FLASH);
        g_sess = SESS_IDLE;
    }
}

/* END: 传输结束 */
static void client_handle_end(void)
{
    if (g_sess != SESS_DOWNLOADING) {
        printf("[OTA_CLIENT] END in non-download state, ignored\r\n");
        return;
    }

    if (ota_finish_download() < 0) {
        client_report_error(OTA_STATE_ERROR_CRC);
        g_sess = SESS_IDLE;
        return;
    }

    /* 校验通过, 先回 ACK 让上位机确认"数据已经安全落地" */
    if (client_send_ack(OTA_STATE_SUCCESS) < 0) {
        client_report_error(OTA_STATE_SEND_COMFIRM_FAILED);
        g_sess = SESS_IDLE;
        return;
    }

    if (ota_prepare_update() < 0) {
        client_report_error(OTA_STATE_ERROR_FLASH);
        g_sess = SESS_IDLE;
        return;
    }

    printf("[OTA_CLIENT] ready, reboot in %lums\r\n",
           (unsigned long)OTA_REBOOT_DELAY_MS);

    g_sess   = SESS_DONE;
    g_reboot = 1U;
}

static void client_dispatch(const ota_frame_t *f)
{
    switch (f->cmd) {
    case OTA_CMD_PUSH_UPDATE:
        client_handle_push(f);
        break;

    case OTA_CMD_START_DOWNLOAD:
        /* 擦除在回 CONFIRM 时已经做完了, 这里只是服务器的一次性通知 */
        printf("[OTA_CLIENT] START_DOWNLOAD\r\n");
        break;

    case OTA_CMD_DATA:
        client_handle_data(f);
        break;

    case OTA_CMD_END:
        client_handle_end();
        break;

    case OTA_CMD_ACK:
    case OTA_CMD_NAK:
    case OTA_CMD_HELLO_RESP:
        printf("[OTA_CLIENT] recv cmd=0x%02X\r\n", (unsigned)f->cmd);
        break;

    default:
        printf("[OTA_CLIENT] unknown cmd 0x%02X, ignored\r\n", (unsigned)f->cmd);
        break;
    }
}


/* ==================================================================
 *  一次连接内的收发循环
 *  返回 1 = 需要重启, 0 = 连接结束需要重连
 * ================================================================== */
static uint8_t client_session(void)
{
    ota_frame_t f;
    uint32_t    last_hello = xTaskGetTickCount() * portTICK_PERIOD_MS;
    int32_t     rc;
    uint32_t    i;

    for (;;) {
        /* ---- 1. 先把缓冲里已成帧的全部处理掉(一次 recv 可能带回好几帧) ---- */
        for (i = 0U; i < OTA_MAX_FRAMES_PER_LOOP; i++) {
            if (ota_net_next_frame(&f) != 1) {
                break;
            }
            client_dispatch(&f);

            if (g_reboot != 0U) {
                return 1U;
            }
            if (g_need_reconnect != 0U) {
                g_need_reconnect = 0U;
                return 0U;
            }
        }

        if (i >= OTA_MAX_FRAMES_PER_LOOP) {
            /* 到上限还有剩余, 让出一次 CPU 再继续, 别把低优先级任务饿死 */
            vTaskDelay(1);
            continue;
        }

        /* ---- 2. 缓冲空了: 读网络(内部最多等 200ms) ---- */
        rc = ota_net_recv(g_sock);
        if (rc < 0) {
            printf("[OTA_CLIENT] disconnected\r\n");
            return 0U;
        }

        /* ---- 3. 心跳(下载期间不发, 免得和数据帧抢带宽) ---- */
        if (g_sess != SESS_DOWNLOADING) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if ((now - last_hello) >= OTA_HELLO_INTERVAL) {
                last_hello = now;

                if (client_send_hello() < 0) {
                    printf("[OTA_CLIENT] HELLO failed, reconnect\r\n");
                    return 0U;
                }
            }
        }
    }
}


/* ==================================================================
 *  任务主体
 * ================================================================== */
static void ota_client_task(void *param)
{
    uint32_t last_connect = 0U;

    (void)param;

    printf("[OTA_CLIENT] task started\r\n");

    ota_manager_init();

    for (;;) {
        if (g_sock < 0) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if ((now - last_connect) < OTA_RECONNECT_INTERVAL) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            last_connect = now;

            ota_net_rx_reset();

            g_sock = ota_net_connect(OTA_SERVER_IP, OTA_SERVER_PORT);
            if (g_sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            /* 新连接一律从干净状态开始(P7: 上次失败留下的半成品不带到这次) */
            g_sess = SESS_IDLE;
            ota_abort(OTA_STATE_IDLE);

            if (client_send_hello() < 0) {
                ota_net_disconnect(g_sock);
                g_sock = -1;
                continue;
            }

            continue;   /* 立刻进入收发循环, 不再等一轮 */
        }

        if (client_session() != 0U) {
            /* 升级已就绪: 断开连接, 给上位机一点时间, 然后重启 */
            printf("[OTA_CLIENT] disconnecting, preparing reboot...\r\n");
            ota_net_disconnect(g_sock);
            g_sock = -1;

            vTaskDelay(pdMS_TO_TICKS(OTA_REBOOT_DELAY_MS));
            NVIC_SystemReset();
        }

        ota_net_disconnect(g_sock);
        g_sock = -1;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ota_client_init(void)
{
    sys_thread_new("ota_client", ota_client_task, NULL, 4096, 4);
}
