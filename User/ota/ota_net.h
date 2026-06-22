#ifndef __OTA_NET_H
#define __OTA_NET_H

#include <stdint.h>

/* 网络连接状态 */
typedef enum {
    OTA_NET_DISCONNECTED = 0,
    OTA_NET_CONNECTING,
    OTA_NET_CONNECTED,
    OTA_NET_ERROR
} ota_net_state_t;

/* OTA网络协议帧结构 */
typedef struct {
    uint8_t cmd;                       // 命令
    uint8_t seq;                       // 序列号
    uint16_t len;                      // 数据长度
    uint32_t offset;                   // 偏移量(用于断点续传)
    uint8_t data[2048];                // 数据区
} ota_net_frame_t;

/* 发送OTA数据帧 */
int32_t ota_net_send_frame(int sock, const ota_net_frame_t *frame);

/* 接收OTA数据帧 */
int32_t ota_net_recv_frame(int sock, ota_net_frame_t *frame, uint32_t timeout_ms);

/* 连接到OTA服务器 */
int32_t ota_net_connect(const char *server_ip, uint16_t server_port, uint32_t timeout_ms);

/* 断开连接 */
void ota_net_disconnect(int sock);

#endif /* __OTA_NET_H */
