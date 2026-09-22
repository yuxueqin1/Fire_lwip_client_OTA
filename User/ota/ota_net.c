#include "ota_net.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>


/* 单次 recv 最多等待多久(ms)。够短以便及时发 HELLO / 检查断线,
 * 又够长以免把任务变成空转轮询。 */
#define OTA_NET_POLL_MS         200U

/* 接收缓冲: 至少 2 整帧, 这样 TCP 粘连的两帧不会被截断。
 * (老实现每次 recv 到一个 2048 字节的临时数组里只解析第一帧,
 *  粘连的第二帧直接被丢掉 —— 这是升级数据丢失的主要根因之一) */
static uint8_t  g_rx_buf[OTA_RX_BUF_SIZE];
static uint32_t g_rx_len = 0U;

/* 解析出的帧载荷暂存 —— 让 out->data 有一个**不会被搬动**的落点。
 *
 * 为什么必须拷贝: next_frame() 在 return 之前会把缓冲尾部前移(压缩),
 * 而载荷区恰恰落在被覆盖的前缀 [0, consumed) 里面, 于是调用方拿到手
 * 的已经是"被下一帧改写过的"数据了。
 *
 * 帧长 1024 时一次覆盖 1035 字节, 只要缓冲里积压超过一帧(TCP 粘连,
 * 10MB/s 下几乎必然发生)就会中招; 积压多少决定覆盖多少, 所以损坏
 * 形态每次都不一样 —— 表现为**收到的字节总数完全正确, 但 CRC 对不上**,
 * 这是 OTA 里最难查的一类症状(长度校验全过, 只有内容不对)。
 */
static uint8_t  g_frame_data[OTA_MAX_PAYLOAD];


/* ==================================================================
 *  连接
 * ================================================================== */
int32_t ota_net_connect(const char *server_ip, uint16_t server_port)
{
    int32_t sock;
    struct sockaddr_in server_addr;
    struct timeval tv;

    if (server_ip == NULL || server_port == 0U) {
        return -1;
    }

    printf("[OTA_NET] connecting %s:%u ...\r\n", server_ip, (unsigned)server_port);

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("[OTA_NET] socket create failed\r\n");
        return -1;
    }

    tv.tv_sec  = OTA_SOCKET_TIMEOUT / 1000U;
    tv.tv_usec = (OTA_SOCKET_TIMEOUT % 1000U) * 1000U;
    (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const void *)&tv,
                     sizeof(tv));
    (void)setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (const void *)&tv,
                     sizeof(tv));

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(server_port);
    server_addr.sin_addr.s_addr = inet_addr(server_ip);

    if (connect(sock, (struct sockaddr *)&server_addr,
                sizeof(server_addr)) < 0) {
        printf("[OTA_NET] connect failed\r\n");
        (void)closesocket(sock);
        return -1;
    }

    ota_net_rx_reset();

    printf("[OTA_NET] connected\r\n");
    return sock;
}

void ota_net_disconnect(int32_t sock)
{
    if (sock >= 0) {
        (void)closesocket(sock);
    }
}


/* ==================================================================
 *  发送
 * ================================================================== */
int32_t ota_net_send(int32_t sock, uint8_t cmd, uint8_t seq,
                     uint32_t offset, const uint8_t *data, uint16_t len)
{
    /* 缓冲大小按最大帧来定 —— 老实现固定 2048, 数据 1024+9+2 时
     * 实际是 "11 字节越界写"(踩栈), 已修正。 */
    uint8_t  buffer[OTA_FRAME_MAX_SIZE];
    uint32_t pos = 0U;
    uint32_t total = 0U;
    uint8_t  checksum = 0U;
    uint32_t i;

    if (sock < 0) {
        return -1;
    }
    if (len > OTA_MAX_PAYLOAD) {
        printf("[OTA_NET] send len %u over max %u\r\n",
               (unsigned)len, (unsigned)OTA_MAX_PAYLOAD);
        return -1;
    }
    if (data == NULL && len != 0U) {
        return -1;
    }

    buffer[pos++] = OTA_FRAME_MAGIC;
    buffer[pos++] = cmd;
    buffer[pos++] = seq;
    buffer[pos++] = (uint8_t)((len >> 8) & 0xFFU);
    buffer[pos++] = (uint8_t)(len & 0xFFU);
    buffer[pos++] = (uint8_t)((offset >> 24) & 0xFFU);
    buffer[pos++] = (uint8_t)((offset >> 16) & 0xFFU);
    buffer[pos++] = (uint8_t)((offset >> 8) & 0xFFU);
    buffer[pos++] = (uint8_t)(offset & 0xFFU);

    if (len != 0U) {
        memcpy(&buffer[pos], data, len);
        pos += len;
    }

    for (i = 0U; i < pos; i++) {
        checksum = (uint8_t)(checksum + buffer[i]);
    }
    buffer[pos++] = checksum;
    buffer[pos++] = OTA_FRAME_END;

    /* 完整发送: TCP 发送是流式的, 可能只发出去一部分 */
    while (total < pos) {
        int32_t n = send(sock, (const void *)(buffer + total),
                         (int)(pos - total), 0);

        if (n <= 0) {
            printf("[OTA_NET] send failed (%lu/%lu sent)\r\n",
                   (unsigned long)total, (unsigned long)pos);
            return -1;
        }
        total += (uint32_t)n;
    }

    return (int32_t)total;
}


/* ==================================================================
 *  接收: 流式缓冲 + 逐帧取用
 * ================================================================== */
void ota_net_rx_reset(void)
{
    g_rx_len = 0U;
}

int32_t ota_net_recv(int32_t sock)
{
    fd_set rfds;
    struct timeval tv;
    int32_t n;

    if (sock < 0) {
        return -1;
    }

    /* 剩余空间不足一整帧就先不读, 避免把半帧挤掉。
     * 调用方应当先把已成帧的数据取走再回来。 */
    if ((OTA_RX_BUF_SIZE - g_rx_len) < OTA_FRAME_MAX_SIZE) {
        return 0;
    }

    /* 先用 select 探一下可读性, 顺便限定本次最多等 OTA_NET_POLL_MS。
     * 这样 recv 返回 -1 就确实是真的出错, 不必依赖 errno
     * (lwIP + ARMCC 组合下 errno 的可用性不确定)。 */
    FD_ZERO(&rfds);
    FD_SET(sock, &rfds);
    tv.tv_sec  = OTA_NET_POLL_MS / 1000U;
    tv.tv_usec = (OTA_NET_POLL_MS % 1000U) * 1000U;

    if (select(sock + 1, &rfds, NULL, NULL, &tv) <= 0) {
        return 0;                       /* 超时 / 本任务应让出 CPU */
    }

    n = recv(sock, (void *)&g_rx_buf[g_rx_len],
             (int)(OTA_RX_BUF_SIZE - g_rx_len), 0);

    if (n < 0) {
        printf("[OTA_NET] recv error\r\n");
        return -1;
    }
    if (n == 0) {                       /* 对端关闭 */
        printf("[OTA_NET] peer closed connection\r\n");
        return -1;
    }

    g_rx_len += (uint32_t)n;
    return n;
}

/* 在 [start, g_rx_len) 中找完整的一帧。
 *   返回 >0 : 整帧长度(帧从 start 开始)
 *   返回  0 : 数据还不够凑齐一整帧(该位置像个帧头, 需要更多数据)
 *   返回 -1 : 该位置不是合法帧, 调用方后移继续找 */
static int32_t parse_frame(ota_frame_t *out, uint32_t start)
{
    uint32_t avail = g_rx_len - start;
    const uint8_t *p = &g_rx_buf[start];
    uint16_t len;
    uint8_t  checksum = 0U;
    uint32_t i;

    if (avail < OTA_FRAME_HEADER_SIZE) {
        /* 连帧头都不够。只有首字节像帧头时才值得继续等, 否则这就是
         * 必须丢掉的尾部垃圾 —— 若在这里返回 0, 后面 next_frame 会把
         * 它记成"未凑齐的候选"保留下来, 白白挡路。 */
        return (p[0] == OTA_FRAME_MAGIC) ? 0 : -1;
    }
    if (p[0] != OTA_FRAME_MAGIC) {
        return -1;                      /* 帧头不对, 重新同步 */
    }

    len = (uint16_t)(((uint16_t)p[3] << 8) | (uint16_t)p[4]);
    if (len > OTA_MAX_PAYLOAD) {
        return -1;                      /* 长度非法, 重新同步 */
    }
    if (avail < (uint32_t)(OTA_FRAME_HEADER_SIZE + len + OTA_FRAME_TAIL_SIZE)) {
        return 0;                       /* 整帧还没到齐 */
    }

    for (i = 0U; i < (uint32_t)(OTA_FRAME_HEADER_SIZE + len); i++) {
        checksum = (uint8_t)(checksum + p[i]);
    }
    if (checksum != p[OTA_FRAME_HEADER_SIZE + len]) {
        return -1;                      /* 校验错, 重新同步 */
    }
    if (p[OTA_FRAME_HEADER_SIZE + len + 1U] != OTA_FRAME_END) {
        return -1;                      /* 帧尾不对, 重新同步 */
    }

    out->cmd    = p[1];
    out->seq    = p[2];
    out->len    = len;
    out->offset = ((uint32_t)p[5] << 24) | ((uint32_t)p[6] << 16) |
                  ((uint32_t)p[7] << 8)  |  (uint32_t)p[8];

    /* 必须拷出来再交出去: 本函数返回后 next_frame() 会 memmove 压缩缓冲,
     * 而载荷区就在被覆盖的前缀里。直接指 &p[9] 等于把还没读的数据
     * 交给下一帧去踩(详见 g_frame_data 的说明)。 */
    memcpy(g_frame_data, &p[OTA_FRAME_HEADER_SIZE], len);
    out->data   = g_frame_data;

    return (int32_t)(OTA_FRAME_HEADER_SIZE + len + OTA_FRAME_TAIL_SIZE);
}

int32_t ota_net_next_frame(ota_frame_t *out)
{
    /* 下一个候选帧头的位置(从 from 起找 0xAA) */
    uint32_t pos;
    uint32_t stall = 0xFFFFFFFFU;       /* 最早的"还没凑齐"的候选位置 */
    uint32_t i;

    if (out == NULL) {
        return -1;
    }

    pos = 0U;

    while (pos < g_rx_len) {
        int32_t rc = parse_frame(out, pos);

        if (rc > 0) {
            /* 取走这一帧, 把后面的数据前移 */
            uint32_t consumed = pos + (uint32_t)rc;

            if (consumed < g_rx_len) {
                memmove(g_rx_buf, &g_rx_buf[consumed], g_rx_len - consumed);
            }
            g_rx_len -= consumed;
            return 1;
        }

        if (rc == 0) {
            /* 这里像个帧头但还没凑齐。记录候选后**必须继续往后找** ——
             * 杂散 0xAA 伪造出的帧头隐含长度可能很大, 就此停住会把
             * 后面躺着的一帧真数据一直挡住, 等到上位机超时为止。 */
            if (stall == 0xFFFFFFFFU) {
                stall = pos;
            }
        }

        /* rc <= 0: 无论"不是帧头"还是"没凑齐", 都跳到下一个候选 0xAA */
        for (i = pos + 1U; i < g_rx_len; i++) {
            if (g_rx_buf[i] == OTA_FRAME_MAGIC) {
                break;
            }
        }
        pos = i;
    }

    if (stall != 0xFFFFFFFFU) {
        /* 保留从最早候选开始的残余(那半帧), 我们还得等它的后半截 */
        if (stall > 0U) {
            uint32_t keep = g_rx_len - stall;

            memmove(g_rx_buf, &g_rx_buf[stall], keep);
            g_rx_len = keep;
        }
    } else {
        g_rx_len = 0U;                  /* 整条缓冲都是垃圾(或已全部取走) */
    }

    return 0;
}
