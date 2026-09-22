#ifndef __OTA_FLASH_H
#define __OTA_FLASH_H

#include <stdint.h>
#include "ota_config.h"

/* Flash 操作返回值 */
typedef enum {
    OTA_FLASH_OK = 0,
    OTA_FLASH_ERR_ERASE,        /* 擦除失败 */
    OTA_FLASH_ERR_WRITE,        /* 编程失败 */
    OTA_FLASH_ERR_ADDR,         /* 地址/长度非法 */
    OTA_FLASH_ERR_SIZE,         /* 长度非法(非 4 字节对齐等) */
    OTA_FLASH_ERR_PROTECT       /* 试图擦写当前正在运行的分区 */
} ota_flash_result_t;

/* 每完成一个扇区(或每扫过一块)后的回调, 用于喂硬件看门狗。可为 NULL。
 *
 * ⚠ 重要: STM32F4 擦写 Flash 期间总线被占用, 从 Flash 取指的 CPU 会被 stall,
 *   连中断都进不来。所以"擦完一个扇区后回调一次"只能覆盖扇区之间,
 *   单个 128K 扇区擦除期间(~1.1s, 最坏数秒)仍无法喂狗。
 *   真正的兜底是 ota_wdt_hw_start() 的 TIM6+DMA 硬件自动喂狗。 */
typedef void (*ota_flash_feed_cb_t)(void);


/* ------------------------------------------------------------------ */
/* 扇区信息                                                            */
/* ------------------------------------------------------------------ */

/* 返回 addr 所在扇区的大小(字节); addr 不在有效范围时返回 0 */
uint32_t ota_flash_sector_size(uint32_t addr);

/* 返回把 [addr, addr+size) 完整覆盖所需擦除的字节数(向上对齐到扇区边界) */
uint32_t ota_flash_erase_span(uint32_t addr, uint32_t size);


/* ------------------------------------------------------------------ */
/* 基础操作                                                            */
/* ------------------------------------------------------------------ */

/* 初始化(把 Flash 置于锁定状态) */
void ota_flash_init(void);

/* 保护当前正在运行的分区: 该分区范围内的擦除请求一律返回 OTA_FLASH_ERR_PROTECT。
 * 传 0 取消保护。ota_manager_init() 会设置为当前运行分区。 */
void ota_flash_protect(uint32_t partition_addr);

/* 擦除 [addr, addr+size), 要求整段落在 APP1 或 APP2 分区内, 且不与受保护分区重叠。
 * 按真实扇区表逐个扇区擦除, 每擦完一个扇区调用一次 feed_cb(可为 NULL)。 */
ota_flash_result_t ota_flash_erase_range(uint32_t addr, uint32_t size,
                                         ota_flash_feed_cb_t feed_cb);

/* 读 Flash (内存映射, 直接 memcpy) */
ota_flash_result_t ota_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);

/* 写 Flash (按字编程, len 必须为 4 的倍数; 目标区域必须已擦除) */
ota_flash_result_t ota_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);

/* 计算 Flash 区域内数据的 CRC32 (标准 Ethernet CRC32, 与上位机 zlib.crc32 一致)。
 * 每扫过 64KB 调用一次 feed_cb(可为 NULL)。 */
uint32_t ota_flash_crc32(uint32_t addr, uint32_t size, ota_flash_feed_cb_t feed_cb);


/* ------------------------------------------------------------------ */
/* BootInfo                                                            */
/* ------------------------------------------------------------------ */

/* 写入 BootInfo(会擦除整个 Sector 11)。info 会先拷贝到栈上再写, 允许传入
 * 指向 BOOT_INFO_ADDR 的只读指针(即读-改-写场景)。 */
ota_flash_result_t boot_info_write(const boot_info_t *info);

/* 读取 BootInfo。始终返回非 NULL(直接返回 Flash 映射地址, 只读)。 */
const boot_info_t *boot_info_read(void);


/* ------------------------------------------------------------------ */
/* 镜像校验                                                            */
/* ------------------------------------------------------------------ */

/* 校验 slot 分区内的固件镜像是否可以启动:
 *   1. 首字(初始栈指针)  必须落在 SRAM 范围内
 *   2. 复位向量          必须落在该分区地址范围内  ← 挡住"上下位机发错镜像"
 *   3. size != 0 时按 size 计算 CRC32 并与 expect_crc 比对
 * 返回 0 = 通过; 负值 = 失败(见 ota_image_check_t)。 */
int32_t ota_flash_check_image(uint32_t slot, uint32_t size, uint32_t expect_crc,
                              ota_flash_feed_cb_t feed_cb);

typedef enum {
    OTA_IMAGE_OK            =  0,
    OTA_IMAGE_ERR_SLOT      = -1,   /* 槽位非法 */
    OTA_IMAGE_ERR_STACK     = -2,   /* 栈指针不在 SRAM 内 */
    OTA_IMAGE_ERR_RESET_VEC = -3,   /* 复位向量不在本分区内 */
    OTA_IMAGE_ERR_CRC       = -4,   /* CRC32 不匹配 */
    OTA_IMAGE_ERR_SIZE      = -5    /* 大小非法 */
} ota_image_check_t;

#endif /* __OTA_FLASH_H */
