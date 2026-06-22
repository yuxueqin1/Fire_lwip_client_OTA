#include "ota_manager.h"
#include "ota_net.h"
#include "ota_config.h"
#include "client.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sys.h"
#include <stdio.h>
#include <string.h>
#include "ota_client.h"

/* OTA客户端配置 */
#define OTA_SERVER_IP       "192.168.0.181"     // 需要修改为您的服务器IP
#define OTA_SERVER_PORT     5002                 // OTA服务器端口
#define OTA_RECONNECT_INTERVAL  10000            // 断线重连间隔(ms)
#define OTA_HELLO_INTERVAL  60000                // HELLO心跳间隔(ms)

typedef struct {
    int sock;
    uint8_t seq;
    uint32_t state;
    uint32_t last_hello_time;
} ota_client_t;

static ota_client_t g_ota_client = {
    .sock = -1,
    .seq = 0,
    .state = 0,
    .last_hello_time = 0
};

ota_state_t g_ota_reply_state;

/* 发送HELLO命令，报告当前版本 */
static int32_t ota_client_send_hello(int sock)
{
    ota_net_frame_t frame;
    uint32_t curr_version = ota_get_current_version();
    
    frame.cmd = OTA_CMD_HELLO;
    frame.seq = g_ota_client.seq++;
    frame.len = 4;
    frame.offset = 0;
    
    *(uint32_t *)frame.data = curr_version;
    
    printf("[OTA_CLIENT] Sending HELLO: version 0x%08lX\n", curr_version);
    return ota_net_send_frame(sock, &frame);
}

/* 处理服务器的升级推送请求 */
static int32_t ota_client_handle_push_update(const ota_net_frame_t *frame)
{
    if (frame->len < 12) {
        printf("[OTA_CLIENT] Error: Invalid push update frame\n");
        g_ota_reply_state = OTA_STATE_ERROR_INVALID_FRAME;
        return -1;
    }
    
    uint32_t new_version = *(uint32_t *)(frame->data + 0);
    uint32_t new_size = *(uint32_t *)(frame->data + 4);
    uint32_t new_crc = *(uint32_t *)(frame->data + 8);
    
    // 调用管理器处理版本对比
    int32_t result = ota_handle_push_update_request(new_version, new_size, new_crc);
    
    return result;
}

/* 发送升级确认命令 */
static int32_t ota_client_send_confirm(int sock)
{
    ota_net_frame_t frame;
    
    frame.cmd = OTA_CMD_CONFIRM_UPDATE;
    frame.seq = g_ota_client.seq++;
    frame.len = 0;
    frame.offset = 0;
    
    printf("[OTA_CLIENT] Sending CONFIRM_UPDATE\n");
    return ota_net_send_frame(sock, &frame);
}

/* 发送升级拒绝命令 */
static int32_t ota_client_send_reject(int sock)
{
    ota_net_frame_t frame;
    
    frame.cmd = OTA_CMD_REJECT_UPDATE;
    frame.seq = g_ota_client.seq++;
    frame.len = 0;
    frame.offset = 0;
    
    printf("[OTA_CLIENT] Sending REJECT_UPDATE\n");
    return ota_net_send_frame(sock, &frame);
}


/* 报告错误信息命令 */
static int32_t ota_client_send_error(int sock)
{
    ota_net_frame_t frame;
    
    frame.cmd = OTA_CMD_ERROR_UPDATE;
    frame.seq = g_ota_client.seq++;
    frame.len = 4;
    frame.offset = 0;

    *(uint32_t *)frame.data = g_ota_reply_state; // 发送当前状态作为错误码
    
    printf("[OTA_CLIENT] Sending ERROR_UPDATE\n");
    return ota_net_send_frame(sock, &frame);
}

/* OTA客户端主任务 - 连接到服务器并等待升级请求 */
static void ota_client_task(void *param)
{
    int sock = -1;
    static ota_net_frame_t recv_frame;
    ota_manager_t *mgr = ota_get_manager();
    uint32_t last_hello_time = 0;
    uint32_t last_connect_time = 0;
    
    printf("[OTA_CLIENT] Task started\n");
    
    ota_manager_init();
    
    while (1) {
        TickType_t tick_now = xTaskGetTickCount();
        uint32_t ms_now = tick_now * portTICK_PERIOD_MS;
        
        // 如果连接已断开，尝试重新连接
        if (sock < 0) {
            if ((ms_now - last_connect_time) >= OTA_RECONNECT_INTERVAL) {
                last_connect_time = ms_now;
                
                printf("\n[OTA_CLIENT] Connecting to OTA server at %s:%d\n", OTA_SERVER_IP, OTA_SERVER_PORT);
                
                sock = ota_net_connect(OTA_SERVER_IP, OTA_SERVER_PORT, OTA_SOCKET_TIMEOUT);
                if (sock < 0) {
                    printf("[OTA_CLIENT] Failed to connect\n");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
                
                printf("[OTA_CLIENT] Connected to server\n");
                last_hello_time = ms_now;
                
                // 发送初始HELLO
                if (ota_client_send_hello(sock) < 0) {
                    printf("[OTA_CLIENT] Failed to send HELLO\n");
                    ota_net_disconnect(sock);
                    sock = -1;
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    continue;
                }
            }
        } else {
            // 连接已建立，定期发送HELLO心跳
            if ((ms_now - last_hello_time) >= OTA_HELLO_INTERVAL) {
                last_hello_time = ms_now;
                
                if (ota_client_send_hello(sock) < 0) {
                    printf("[OTA_CLIENT] HELLO failed, reconnecting\n");
                    ota_net_disconnect(sock);
                    sock = -1;
                    continue;
                }
            }
            
            // 等待服务器消息（非阻塞接收）
            if (ota_net_recv_frame(sock, &recv_frame, 2000) > 0) {
                printf("[OTA_CLIENT] Received command: 0x%02X\n", recv_frame.cmd);
                
                switch (recv_frame.cmd) {
                    case OTA_CMD_PUSH_UPDATE: {
                        // 服务器推送升级请求
                        int32_t need_update = ota_client_handle_push_update(&recv_frame);
                        
                        if (need_update > 0) {
                            // 版本更新，同意升级
                            if (ota_client_send_confirm(sock) < 0) {
                                printf("[OTA_CLIENT] Failed to send confirm\n");
                                g_ota_reply_state = OTA_STATE_SEND_COMFIRM_FAILED;
                                ota_client_send_error(sock);
                                ota_net_disconnect(sock);
                                sock = -1;
                                break;
                            }
                            
                            // 准备接收固件
                            if (ota_start_download(OTA_SERVER_IP, OTA_SERVER_PORT) < 0) {
                                printf("[OTA_CLIENT] Failed to prepare download\n");
                                g_ota_reply_state = OTA_STATE_START_DOWNLOAD_FAILED;
                                ota_client_send_error(sock);
                                ota_net_disconnect(sock);
                                sock = -1;
                                break;
                            }
                            
                            mgr->state = OTA_STATE_DOWNLOADING;
                        } else if (need_update == 0) {
                            // 版本相同或更新，拒绝
                            printf("[OTA_CLIENT] Version check failed, rejecting update\n");
                            ota_client_send_reject(sock);
                        } else {
                            // 错误
                            ota_client_send_error(sock);
                            ota_client_send_reject(sock);
                            ota_net_disconnect(sock);
                            sock = -1;
                        }
                        break;
                    }
                    
                    case OTA_CMD_START_DOWNLOAD: {
                        // 服务器通知开始下载
                        printf("[OTA_CLIENT] Server starting firmware download\n");
                        mgr->state = OTA_STATE_DOWNLOADING;
                        break;
                    }
                    
                    case OTA_CMD_DATA: {
                        // 接收固件数据
                        if (mgr->state == OTA_STATE_DOWNLOADING) {
                            if (ota_write_data_block(recv_frame.data, recv_frame.len, recv_frame.offset) < 0) {
                                printf("[OTA_CLIENT] Failed to write data block\n");
                                mgr->state = OTA_STATE_FAILED;
                                ota_net_disconnect(sock);
                                sock = -1;
                                break;
                            }
                            
                            uint32_t progress = ota_get_progress();
                            if (progress % 10 == 0) {
                                printf("[OTA_CLIENT] Download progress: %lu%%\n", progress);
                            }
                        }
                        break;
                    }
                    
                    case OTA_CMD_END: {
                        // 固件传输完成
                        printf("[OTA_CLIENT] Firmware transmission completed\n");
                        
                        if (mgr->state == OTA_STATE_DOWNLOADING) {
                            if (ota_finish_download() < 0) {
                                printf("[OTA_CLIENT] Download verification failed\n");
                                mgr->state = OTA_STATE_FAILED;
                                ota_net_disconnect(sock);
                                sock = -1;
                                break;
                            }
                            
                            printf("[OTA_CLIENT] Verification passed\n");
                            
                            // 准备更新
                            if (ota_prepare_update() < 0) {
                                printf("[OTA_CLIENT] Failed to prepare update\n");
                                mgr->state = OTA_STATE_FAILED;
                                ota_net_disconnect(sock);
                                sock = -1;
                                break;
                            }
                            
                            printf("[OTA_CLIENT] Update will be applied on next reboot\n");
                            printf("[OTA_CLIENT] Rebooting in 5 seconds...\n");
                            
                            ota_net_disconnect(sock);
                            sock = -1;
                            
                            vTaskDelay(pdMS_TO_TICKS(5000));
                            
                            // 触发系统重启
                            NVIC_SystemReset();
                        }
                        break;
                    }
                    
                    default:
                        printf("[OTA_CLIENT] Unknown command: 0x%02X\n", recv_frame.cmd);
                        break;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* 初始化OTA客户端 */
void ota_client_init(void)
{
    sys_thread_new("ota_client", ota_client_task, NULL, 4096, 4);
}
