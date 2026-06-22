#ifndef __OTA_CONFIG_H
#define __OTA_CONFIG_H

#include <stdint.h>

/* ==================== Flash分区配置 (1MB总容量: STM32F429IGT6) ==================== */
#define FLASH_SIZE                  (1024 * 1024)      // 1MB
#define SECTOR_SIZE                 (64 * 1024)        // 64KB

/* Bootloader区 */
#define BOOTLOADER_ADDR             0x08000000
#define BOOTLOADER_SIZE             (32 * 1024)        // 32KB (Sector 0)

/* App1区(当前运行固件) */
#define APP1_ADDR                   0x08008000
#define APP1_SIZE                   (512 * 1024)       // 512KB (Sector 1-7)

/* App2区(OTA下载区) */
#define APP2_ADDR                   0x08088000
#define APP2_SIZE                   (480 * 1024)       // 480KB (Sector 8-11)

/* 固件标志信息 */
typedef struct {
    uint32_t magic;                 // 魔数: 0x12345678
    uint32_t version;               // 版本号
    uint32_t size;                  // 固件大小(单位: 字节)
    uint32_t crc32;                 // CRC32校验值
    uint32_t timestamp;             // 时间戳
    uint8_t reserved[12];           // 预留字段
} ota_app_header_t;

#define OTA_HEADER_SIZE             sizeof(ota_app_header_t)
#define OTA_MAGIC                   0x12345678

/* OTA协议相关 */
#define OTA_SOCKET_TIMEOUT          30000              // 30秒
#define OTA_BUFFER_SIZE             2048               // 单次接收缓冲
#define OTA_MAX_CHUNK_SIZE          4096               // 最大数据块
#define OTA_FRAME_SIZE              (OTA_BUFFER_SIZE + 64)  // 帧大小

/* OTA状态 */
typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_WAITING_REQUEST,     // 等待服务器升级请求
    OTA_STATE_REQUEST_RECEIVED,    // 收到升级请求
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED,
    OTA_STATE_UPDATING,
    OTA_STATE_ERROR_SIZE,          // 固件大小超限错误
    OTA_STATE_ERROR_INVALID_FRAME, // 无效帧错误
    OTA_STATE_SEND_COMFIRM_FAILED,  // 发送确认失败
    OTA_STATE_START_DOWNLOAD_FAILED // 启动下载失败
} ota_state_t;

/* OTA命令 - 新流程：服务器推送升级请求 */
typedef enum {
    // 握手和版本管理
    OTA_CMD_HELLO = 0x00,              // 客户端问候，报告当前版本
    OTA_CMD_HELLO_RESP = 0x80,        // 服务器响应
    
    // 服务器主动推送升级
    OTA_CMD_PUSH_UPDATE = 0x10,       // 服务器推送升级请求 + 新固件信息
    OTA_CMD_CONFIRM_UPDATE = 0x11,    // 客户端确认升级
    OTA_CMD_REJECT_UPDATE = 0x12,     // 客户端拒绝升级
    OTA_CMD_ERROR_UPDATE = 0x13,       // 客户端报告升级错误
    
    // 数据传输
    OTA_CMD_START_DOWNLOAD = 0x03,    // 服务器开始推送固件
    OTA_CMD_DATA = 0x04,              // 固件数据块
    OTA_CMD_END = 0x05,               // 传输结束
    
    // 通用应答
    OTA_CMD_ACK = 0x06,               // 确认
    OTA_CMD_NAK = 0x07                // 否认
} ota_cmd_t;

/* ==================== 固件版本信息 ==================== */
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            0
#define FW_VERSION_REVISION         0
#define FW_VERSION_BUILD            1

#define FW_VERSION    ( (FW_VERSION_MAJOR << 24) | \
                               (FW_VERSION_MINOR << 16) | \
                               (FW_VERSION_REVISION << 8) | \
                               FW_VERSION_BUILD )


                              
#endif /* __OTA_CONFIG_H */
