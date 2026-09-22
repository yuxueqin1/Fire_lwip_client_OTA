#ifndef __OTA_NET_H
#define __OTA_NET_H

#include <stdint.h>
#include "ota_config.h"

/* ==================================================================
 *  OTA 帧协议 (上位机 PC_Server/ota_gui.py 必须与此一致)
 *
 *    +1B---+1B---+2B----+4B------+ LEN B +1B-------+1B---+
 *    | 0xAA | CMD  | SEQ   | LEN(BE) | OFFSET(BE) | DATA  | CKSUM | 0xBB |
 *    +------+------+-------+---------+------------+-------+------+
 *      [0]    [1]    [2:3]   [4:7]      [8..]              [..]   [..]
 *
 *    CHECKSUM = 前面 9+LEN 个字节的算术和 & 0xFF
 *
 *  TCP 是字节流, 不保证一次 recv() 正好一帧 —— 所以接收侧
 *  必须先攒够一整帧再解析(见 ota_net_recv / ota_net_next_frame)。
 * ================================================================== */

/* 接收侧解析出的一帧。
 * ⚠ data 指向 ota_net.c 内部一块**专门的载荷暂存区**(不是接收缓冲),
 *   生命周期到下一次 ota_net_next_frame() 调用为止。
 *   即: 取到一帧后可以放心用到处理完, 但需要跨帧缓存内容时必须自己拷贝。
 *
 *   (曾经这里是直接指向接收缓冲的, 而 next_frame() 在返回前会 memmove
 *    压缩缓冲, 载荷区正好被覆盖 —— 于是调用方读到的是下一帧的数据,
 *    症状是"字节数全对、CRC 全错"。已用独立暂存区修掉。) */
typedef struct {
    uint8_t        cmd;
    uint8_t        seq;
    uint16_t       len;
    uint32_t       offset;
    const uint8_t *data;
} ota_frame_t;


/* ---- 连接管理 ---- */

/* 连接 OTA 服务器。成功返回 socket 句柄(>=0), 失败返回 -1。
 * 收发超时由内部统一设置为 OTA_SOCKET_TIMEOUT。 */
int32_t ota_net_connect(const char *server_ip, uint16_t server_port);

/* 关闭连接 */
void ota_net_disconnect(int32_t sock);


/* ---- 发送 ---- */

/* 组装并完整发送一帧。
 * data 可以为 NULL(当 len == 0)。len 必须 <= OTA_MAX_PAYLOAD。
 * 返回实际发送字节数, 失败返回 -1。 */
int32_t ota_net_send(int32_t sock, uint8_t cmd, uint8_t seq,
                     uint32_t offset, const uint8_t *data, uint16_t len);


/* ---- 接收 ---- */

/* 清空接收缓冲(重连/开始新会话时调用, 丢掉上次的半帧残留) */
void ota_net_rx_reset(void);

/* 执行一次 recv() 并把数据追加到内部接收缓冲。
 *   返回 >0 : 本次新增字节数
 *   返回  0 : 本次没读到(超时/缓冲已满, 需先取走已成帧的数据)
 *   返回 -1 : socket 错误或对端断开
 * 注意: 缓冲剩余空间不足一整帧时直接返回 0, 不会截断数据。 */
int32_t ota_net_recv(int32_t sock);

/* 从接收缓冲里取出一整帧。
 *   返回  1 : 取到一帧(out 已填好)
 *   返回  0 : 数据还不够一帧, 需要继续 ota_net_recv()
 *   返回 -1 : 缓冲已满却始终找不到合法帧(不会发生, 防御性返回)
 * 自动重同步: 帧头不对/长度非法/校验错/帧尾不对时丢弃 1 字节继续找,
 *             保证一次线路误码不会让整个会话卡死。 */
int32_t ota_net_next_frame(ota_frame_t *out);

#endif /* __OTA_NET_H */
