#ifndef __OTA_CONFIG_H
#define __OTA_CONFIG_H

#include <stdint.h>

/* ==================================================================
 *  Flash 分区配置  (STM32F429IGT6, 1MB)
 *
 *  硬件扇区表(固定, 不可改):
 *    S0  0x08000000  16K     S1  0x08004000  16K
 *    S2  0x08008000  16K     S3  0x0800C000  16K
 *    S4  0x08010000  64K     S5  0x08020000 128K
 *    S6  0x08040000 128K     S7  0x08060000 128K
 *    S8  0x08080000 128K     S9  0x080A0000 128K
 *    S10 0x080C0000 128K     S11 0x080E0000 128K
 *
 *  分区划分(与扇区严格对齐, 见 2026-09-22 分区表修正):
 *    +------------------+ 0x08000000
 *    |  Bootloader      |   32 KB   S0, S1              永不更新
 *    +------------------+ 0x08008000
 *    |  APP1  (A区)     |  480 KB   S2,S3,S4,S5,S6,S7
 *    +------------------+ 0x08080000
 *    |  APP2  (B区)     |  384 KB   S8,S9,S10
 *    +------------------+ 0x080E0000
 *    |  BootInfo        |  128 KB   S11 (仅用前 32 字节)
 *    +------------------+ 0x08100000
 *
 *  A/B 双镜像乒乓升级:
 *    每次把新固件写入"另一个"分区, 当前运行分区永不被破坏,
 *    因此天然具备失败回滚能力(掉电/传输失败/CRC 错 → 重启用回老版本)。
 *
 *  ⇒ 同一份源码需编译成两个镜像:
 *      A镜像  IROM1 = 0x08008000  (写入 APP1)
 *      B镜像  IROM1 = 0x08080000  (写入 APP2)
 *    两个镜像除链接地址与 VECT_TAB_OFFSET 外完全相同。
 *    单镜像上限 = min(480K, 384K) = 384 KB ← 以 B区 为准
 *
 *  ⚠ 分区地址变更时必须同步修改:
 *     1. IAP 工程 User/main.c 中的 APP1_ADDR / APP2_ADDR / BOOT_INFO_ADDR
 *     2. Keil 两个 Target 的 IROM1 / VECT_TAB_OFFSET
 * ================================================================== */

#define FLASH_BASE_ADDR             0x08000000U
#define FLASH_END_ADDR              0x08100000U
#define FLASH_TOTAL_SIZE            (1024U * 1024U)

/* Bootloader 区 (S0+S1) */
#define BOOTLOADER_ADDR             0x08000000U
#define BOOTLOADER_SIZE             (32U * 1024U)

/* APP1 区 / A区 (S2..S7) */
#define APP1_ADDR                   0x08008000U
#define APP1_SIZE                   (480U * 1024U)

/* APP2 区 / B区 (S8..S10) */
#define APP2_ADDR                   0x08080000U
#define APP2_SIZE                   (384U * 1024U)

/* BootInfo 区 (S11) */
#define BOOT_INFO_ADDR              0x080E0000U
#define BOOT_INFO_SIZE              (128U * 1024U)

/* 单镜像上限(取两个分区较小者) */
#define APP_MAX_SIZE                (APP2_SIZE)

/* 槽位编号 (与 boot_info_t.boot_app 取值一致) */
#define OTA_SLOT_APP1               1U
#define OTA_SLOT_APP2               2U


/* ==================================================================
 *  BootInfo —— Bootloader 与 APP 之间唯一的约定结构
 *
 *  存放于 BOOT_INFO_ADDR, 由 APP 在 OTA 完成后写入, 由 Bootloader 读取。
 *
 *  ⚠ 本结构必须与 IAP 工程中的同名结构逐字节一致:
 *     ota升级需要的iap代码/lwip_client_Bootloader_RTOS_Final（完成版别动了）/
 *         User/main.c  →  boot_info_t
 * ================================================================== */
#define BOOT_INFO_MAGIC             0x55AA55AAU

typedef struct {
    uint32_t magic;         /* 必须等于 BOOT_INFO_MAGIC, 否则整块视为无效 */
    uint32_t boot_app;      /* 下次启动的分区: OTA_SLOT_APP1 / OTA_SLOT_APP2 */
    uint32_t crc;           /* boot_app 分区内固件镜像的 CRC32 (size==0 时忽略) */
    uint32_t size;          /* boot_app 分区内固件镜像大小(字节); 0=未知, 仅做结构校验 */
    uint32_t version;       /* boot_app 分区内固件版本 (IAP 诊断用) */
    uint32_t reserved[3];   /* 预留(试用启动 trial-boot 等), 写 0 */
} boot_info_t;


/* ==================================================================
 *  OTA 协议
 * ================================================================== */
#define OTA_SERVER_IP_DEFAULT       "192.168.1.99"  /* 升级时把上位机网卡配成该地址 */
#define OTA_SERVER_PORT_DEFAULT     5002

#define OTA_SOCKET_TIMEOUT          30000U      /* socket 收发超时 (ms) */
#define OTA_RECONNECT_INTERVAL      10000U      /* 断线重连间隔 (ms) */
#define OTA_HELLO_INTERVAL          60000U      /* HELLO 心跳间隔 (ms) */

#define OTA_MAX_PAYLOAD             1024U       /* 单帧最大数据长度, 与上位机一致 */
#define OTA_FRAME_HEADER_SIZE       9U          /* 帧头: MAGIC+CMD+SEQ+LEN+OFFSET */
#define OTA_FRAME_TAIL_SIZE         2U          /* 帧尾: CHECKSUM+END */
#define OTA_FRAME_MAX_SIZE          (OTA_FRAME_HEADER_SIZE + OTA_MAX_PAYLOAD + OTA_FRAME_TAIL_SIZE)

/* TCP 流重组缓冲: 至少容纳 2 整帧, 避免半帧残留导致丢帧 */
#define OTA_RX_BUF_SIZE             (OTA_FRAME_MAX_SIZE * 3U)

/* 每次任务循环最多连续处理的帧数, 防止长时间不给其它任务让出 CPU */
#define OTA_MAX_FRAMES_PER_LOOP     32U

/* 烧写 Flash 的分块大小(字节, 必须 4 的倍数) */
#define OTA_FLASH_WRITE_CHUNK       1024U

#define OTA_FRAME_MAGIC             0xAAU
#define OTA_FRAME_END               0xBBU


/* OTA 状态 / 错误码 (ERROR_UPDATE 帧上报的就是本枚举值) */
typedef enum {
    OTA_STATE_IDLE = 0,
    OTA_STATE_WAITING_REQUEST,      /* 等待服务器升级请求 */
    OTA_STATE_REQUEST_RECEIVED,     /* 收到升级请求, 待确认 */
    OTA_STATE_DOWNLOADING,
    OTA_STATE_VERIFYING,
    OTA_STATE_SUCCESS,
    OTA_STATE_FAILED,
    OTA_STATE_UPDATING,
    OTA_STATE_ERROR_SIZE,           /* 固件大小超限 */
    OTA_STATE_ERROR_INVALID_FRAME,  /* 无效帧 */
    OTA_STATE_SEND_COMFIRM_FAILED,  /* 发送确认失败 */
    OTA_STATE_START_DOWNLOAD_FAILED,/* 启动下载(擦除)失败 */
    OTA_STATE_ERROR_IMAGE,          /* 镜像结构非法(栈指针/复位向量不在目标分区) */
    OTA_STATE_ERROR_CRC,            /* CRC32 校验失败 */
    OTA_STATE_ERROR_FLASH,          /* Flash 擦写失败 */
    OTA_STATE_ERROR_SLOT,           /* 槽位未知(链接地址配置错误) */
    OTA_STATE_ERROR_MAX
} ota_state_t;


/* OTA 命令字 */
typedef enum {
    /* 握手与版本管理 */
    OTA_CMD_HELLO = 0x00,              /* 客户端问候: 上报版本 + 当前槽位 */
    OTA_CMD_HELLO_RESP = 0x80,         /* 服务器响应(保留) */

    /* 服务器主动推送升级 */
    OTA_CMD_PUSH_UPDATE = 0x10,        /* 推送升级请求: 版本 + 大小 + CRC32 */
    OTA_CMD_CONFIRM_UPDATE = 0x11,     /* 客户端确认升级 */
    OTA_CMD_REJECT_UPDATE = 0x12,      /* 客户端拒绝升级 */
    OTA_CMD_ERROR_UPDATE = 0x13,       /* 客户端报告升级错误(载荷=错误码) */

    /* 数据传输 */
    OTA_CMD_START_DOWNLOAD = 0x03,     /* 服务器开始推送固件 */
    OTA_CMD_DATA = 0x04,               /* 固件数据块 */
    OTA_CMD_END = 0x05,                /* 传输结束 */

    /* 通用应答 */
    OTA_CMD_ACK = 0x06,                /* 确认 */
    OTA_CMD_NAK = 0x07                 /* 否认 */
} ota_cmd_t;


/* ==================================================================
 *  固件版本号
 *    高8位 主版本 | 次8位 次版本 | 再8位 修订 | 低8位 构建号
 *    服务器按数值大小比较: 只有 服务器版本 > 客户端版本 才推送升级
 *  ⚠ 每次发版只需改 FW_VERSION_BUILD
 * ================================================================== */
#define FW_VERSION_MAJOR            1
#define FW_VERSION_MINOR            1
#define FW_VERSION_REVISION         0
#define FW_VERSION_BUILD            0

#define FW_VERSION    ( (FW_VERSION_MAJOR    << 24) | \
                        (FW_VERSION_MINOR    << 16) | \
                        (FW_VERSION_REVISION <<  8) | \
                         FW_VERSION_BUILD )

/* 版本号的可读打印格式: 0x01000001 -> "v1.0.0.1"
 * 用法:  printf("ver=" FW_VER_FMT "\r\n", FW_VER_ARG(ver));
 * 之所以做成 2 个宏, 是因为 printf 的可变参数没法在运行时拆字段。 */
#define FW_VER_FMT      "v%u.%u.%u.%u"
#define FW_VER_ARG(v)   (unsigned)(((uint32_t)(v) >> 24) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >> 16) & 0xFFU), \
                        (unsigned)(((uint32_t)(v) >>  8) & 0xFFU), \
                        (unsigned)( (uint32_t)(v)        & 0xFFU)

/* 槽位字母: 1 -> 'A', 2 -> 'B'
 * 日志里一律写 A/B 而不是 1/2 —— 槽位号 1/2 混在版本号数字里很容易看错。 */
#define OTA_SLOT_CHAR(s)   (((s) == OTA_SLOT_APP2) ? 'B' : 'A')

#endif /* __OTA_CONFIG_H */
