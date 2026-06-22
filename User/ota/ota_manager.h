#ifndef __OTA_MANAGER_H
#define __OTA_MANAGER_H

#include "ota_config.h"

/* OTA管理结构体 */
typedef struct {
    ota_state_t state;
    uint32_t downloaded_size;
    uint32_t total_size;
    uint32_t download_crc;
    uint32_t remote_crc;
    uint32_t remote_version;
} ota_manager_t;

#define BOOT_INFO_ADDR      0x080E0000
#define BOOT_INFO_MAGIC     0x55AA55AA

typedef enum
{
    BOOT_APP1 = 1,
    BOOT_APP2 = 2

} boot_app_t;

typedef struct
{
    uint32_t magic;

    uint32_t boot_app;

    uint32_t crc;

    uint32_t reserved;

} boot_info_t;


/* 初始化OTA管理器 */
void ota_manager_init(void);

/* 获取OTA管理器实例 */
ota_manager_t* ota_get_manager(void);

/* 处理服务器的升级推送请求（返回值：1=同意升级，0=拒绝，-1=错误） */
int32_t ota_handle_push_update_request(uint32_t new_version, uint32_t new_size, uint32_t new_crc);

/* 启动OTA下载过程 */
int32_t ota_start_download(const char *server_ip, uint16_t server_port);

/* 处理OTA数据块 */
int32_t ota_write_data_block(const uint8_t *data, uint32_t len, uint32_t offset);

/* 完成下载并验证 */
int32_t ota_finish_download(void);

/* 准备进行固件更新 */
int32_t ota_prepare_update(void);

/* 获取当前运行的固件版本 */
uint32_t ota_get_current_version(void);

/* 获取OTA状态 */
ota_state_t ota_get_state(void);

/* 获取下载进度 */
uint32_t ota_get_progress(void);

#endif /* __OTA_MANAGER_H */
