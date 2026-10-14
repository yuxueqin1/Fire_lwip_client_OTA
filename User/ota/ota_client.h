#ifndef __OTA_CLIENT_H
#define __OTA_CLIENT_H

#include <stdint.h>


void ota_client_init(void);


/* ==================================================================
 *  诊断接口 (v42 临时加, 只为抓"复位码 31"的成因, 只读)
 *
 *  stage 的取值表见 ota_client.c 里的 OTA_STAGE_xxx:
 *      1 = 循环顶(不在任何阻塞调用里)  ⇒ 排不上 CPU / 任务死了
 *      2 = 等重连间隔   3 = connect 中   4 = 发 HELLO
 *      5 = 会话-上报存活   6 = 会话-取帧/分发(Flash 擦写在这里)
 *      7 = 会话-收网络   8 = 会话-周期 HELLO
 *      9 = 断开连接     10 = 升级完成等重启     0 = 还没进主循环
 * ================================================================== */
uint8_t  ota_client_stage(void);      /* 此刻停在哪一步 */
uint32_t ota_client_loop_cnt(void);   /* 主循环累计圈数, 只增不减 */
uint8_t  ota_client_sock_open(void);  /* 1 = 当前有 TCP 连接 */
uint8_t  ota_client_sess(void);       /* 0=IDLE 1=DOWNLOADING 2=DONE */

#endif /* __OTA_CLIENT_H */
