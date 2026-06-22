#include "ota_net.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#define OTA_NET_FRAME_MAGIC     0xAA
#define OTA_NET_FRAME_END       0xBB

int32_t ota_net_connect(const char *server_ip, uint16_t server_port, uint32_t timeout_ms)
{
    int sock;
    struct sockaddr_in server_addr;
    struct timeval tv;
    
    printf("[OTA_NET] Connecting to %s:%d\n", server_ip, server_port);
    
    // 创建TCP socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("[OTA_NET] Error: Failed to create socket\n");
        return -1;
    }
    
    // 设置socket超时
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const void *)&tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const void *)&tv, sizeof(tv));
    
    // 连接服务器
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    server_addr.sin_addr.s_addr = inet_addr(server_ip);
    
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        printf("[OTA_NET] Error: Connection failed\n");
        closesocket(sock);
        return -1;
    }
    
    printf("[OTA_NET] Connected successfully\n");
    return sock;
}

void ota_net_disconnect(int sock)
{
    if (sock >= 0) {
        closesocket(sock);
    }
}

int32_t ota_net_send_frame(int sock, const ota_net_frame_t *frame)
{
    uint8_t buffer[2048];
    uint32_t pos = 0;
    
    if (!frame || sock < 0) {
        return -1;
    }
    
    // 构建帧格式: [帧头][CMD][SEQ][LEN_H][LEN_L][OFFSET][数据][校验][帧尾]
    buffer[pos++] = OTA_NET_FRAME_MAGIC;
    buffer[pos++] = frame->cmd;
    buffer[pos++] = frame->seq;
    buffer[pos++] = (frame->len >> 8) & 0xFF;
    buffer[pos++] = frame->len & 0xFF;
    buffer[pos++] = (frame->offset >> 24) & 0xFF;
    buffer[pos++] = (frame->offset >> 16) & 0xFF;
    buffer[pos++] = (frame->offset >> 8) & 0xFF;
    buffer[pos++] = frame->offset & 0xFF;
    
    // 拷贝数据
    memcpy(buffer + pos, frame->data, frame->len);
    pos += frame->len;
    
    // 计算校验和
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < pos; i++) {
        checksum += buffer[i];
    }
    buffer[pos++] = checksum;
    buffer[pos++] = OTA_NET_FRAME_END;
    
    // 发送
    int32_t sent = send(sock, (const void *)buffer, pos, 0);
    if (sent < 0) {
        printf("[OTA_NET] Error: Send failed\n");
        return -1;
    }
    
    return sent;
}

int32_t ota_net_recv_frame(int sock, ota_net_frame_t *frame, uint32_t timeout_ms)
{
    uint8_t buffer[2048];
    int32_t len;
//    uint32_t pos = 0;
    
    if (!frame || sock < 0) {
        return -1;
    }
    
    // 接收数据
    len = recv(sock, (void *)buffer, sizeof(buffer), 0);
    if (len <= 0) {
        printf("[OTA_NET] Error: Recv failed or timeout\n");
        return -1;
    }
    
    // 解析帧格式
    if (buffer[0] != OTA_NET_FRAME_MAGIC) {
        printf("[OTA_NET] Error: Invalid frame magic\n");
        return -1;
    }
    
    frame->cmd = buffer[1];
    frame->seq = buffer[2];
    frame->len = (buffer[3] << 8) | buffer[4];
    frame->offset = (buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8];
    
    // 验证帧长度
    if (frame->len > 1024) {
        printf("[OTA_NET] Error: Frame too large\n");
        return -1;
    }
    
    // 拷贝数据
    memcpy(frame->data, buffer + 9, frame->len);
    
    // 验证校验和
    uint8_t checksum = 0;
    for (uint32_t i = 0; i < (9 + frame->len); i++) {
        checksum += buffer[i];
    }
    
    if (checksum != buffer[9 + frame->len]) {
        printf("[OTA_NET] Error: Checksum mismatch\n");
        return -1;
    }
    
    if (buffer[10 + frame->len] != OTA_NET_FRAME_END) {
        printf("[OTA_NET] Error: Invalid frame end\n");
        return -1;
    }
    
    return len;
}
