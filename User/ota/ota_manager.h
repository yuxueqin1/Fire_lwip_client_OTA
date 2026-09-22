#ifndef __OTA_MANAGER_H
#define __OTA_MANAGER_H

#include "ota_config.h"

/* ==================================================================
 *  OTA 管理器
 *
 *  职责: 只做"业务决策 + Flash 落地", 不碰网络。
 *        网络收发在 ota_net.c / 协议流程在 ota_client.c。
 *
 *  分区与槽位:
 *    当前运行分区由 SCB->VTOR(或编译期 OTA_SLOT_SEL)判定,
 *    下载永远写入"另一个"分区 —— 当前运行分区绝不触碰。
 *
 *  错误处理原则(P7 自恢复):
 *    任何失败都不把 state 卡死在错误态, 而是
 *      state    → OTA_STATE_IDLE   (立刻可以重试)
 *      last_error → 错误码          (由 ota_take_error() 取走并清零)
 *    这样"上报错误"和"保持可重试"两件事不再互相冲突。
 * ================================================================== */

typedef struct {
    ota_state_t state;              /* 当前状态             */
    ota_state_t last_error;         /* 最近一次错误码, 0 = 无 */
    uint32_t    error_count;        /* 累计失败次数          */

    uint32_t    downloaded_size;    /* 已接收字节数          */
    uint32_t    total_size;         /* 服务器声明的固件大小   */
    uint32_t    download_crc;       /* 边收边算的 CRC32 中间值 */
    uint32_t    remote_crc;         /* 服务器声明的 CRC32     */
    uint32_t    remote_version;     /* 服务器声明的版本       */

    uint8_t     current_slot;       /* 正在运行的分区 OTA_SLOT_APPx */
    uint8_t     target_slot;        /* 本次要写入的分区            */
} ota_manager_t;


/* ---- 初始化与查询 ---- */
void          ota_manager_init(void);
ota_manager_t* ota_get_manager(void);

/* 当前运行槽位(Ota_SLOT_APP1 / APP2) */
uint8_t       ota_get_current_slot(void);

/* 目标槽位(与当前槽位相反) */
uint8_t       ota_get_target_slot(void);

/* 当前运行固件的版本号 */
uint32_t      ota_get_current_version(void);

ota_state_t   ota_get_state(void);
uint32_t      ota_get_progress(void);       /* 0-100 */

/* 取走并清除"最近一次错误码"; 无错误时返回 0 */
ota_state_t   ota_take_error(void);


/* ---- 流程 ---- */

/* 处理服务器推送的升级请求。
 *   返回  1 = 同意升级(状态→REQUEST_RECEIVED)
 *         0 = 拒绝, 版本不更新或更高
 *        -1 = 出错(大小超限等), 错误码由 ota_take_error() 取 */
int32_t ota_handle_push_update_request(uint32_t new_version,
                                       uint32_t new_size,
                                       uint32_t new_crc);

/* 进入下载态并擦除目标分区。会自动启动硬件看门狗喂狗。
 * 成功返回 0, 失败返回 -1(错误码见 ota_take_error)。 */
int32_t ota_start_download(void);

/* 写入一个数据块。offset 相对固件起始, 长度必须 4 字节对齐。 */
int32_t ota_write_data_block(const uint8_t *data, uint32_t len, uint32_t offset);

/* 传输结束: 校验长度 + CRC32 + 镜像结构(栈指针/复位向量)。
 * 成功返回 0 且状态→SUCCESS。 */
int32_t ota_finish_download(void);

/* 校验通过后写 BootInfo, 指明下次启动到目标分区。
 * 成功返回 0 且状态→UPDATING, 调用方随后应重启。 */
int32_t ota_prepare_update(void);

/* 中止当前流程(断线/超时/出错), 回到 IDLE 并记录错误码, 允许重试。
 * 传 0 表示"仅中止, 不记错误"。 */
void ota_abort(ota_state_t err);

#endif /* __OTA_MANAGER_H */
