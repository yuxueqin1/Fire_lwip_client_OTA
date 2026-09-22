#include "ota_flash.h"
#include "ota_crc32.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ==================================================================
 *  SRAM 范围 —— 用于校验镜像向量表中的初始栈指针是否合法。
 *  ⚠ 必须与 IAP 工程 User/main.c 的 JumpToApplication() 中判断一致,
 *    否则 APP 与 IAP 对"镜像是否合法"会得出不同结论。
 * ================================================================== */
#define SRAM_START_ADDR             0x20000000U
#define SRAM_END_ADDR               0x20030000U     /* IRAM1: 192 KB */

/* CRC32 扫描时每多少字节喂一次狗 */
#define CRC_FEED_INTERVAL           (64U * 1024U)


/* ==================================================================
 *  硬件扇区表 (STM32F429IGT6, 1MB)
 *  下标 i 即 FLASH_SECTOR_i, 供 HAL_FLASHEx_Erase 使用。
 * ================================================================== */
typedef struct {
    uint32_t addr;
    uint32_t size;
} ota_sector_t;

static const ota_sector_t g_sectors[] = {
    { 0x08000000U,  16U * 1024U },   /* S0  */
    { 0x08004000U,  16U * 1024U },   /* S1  */
    { 0x08008000U,  16U * 1024U },   /* S2  */
    { 0x0800C000U,  16U * 1024U },   /* S3  */
    { 0x08010000U,  64U * 1024U },   /* S4  */
    { 0x08020000U, 128U * 1024U },   /* S5  */
    { 0x08040000U, 128U * 1024U },   /* S6  */
    { 0x08060000U, 128U * 1024U },   /* S7  */
    { 0x08080000U, 128U * 1024U },   /* S8  */
    { 0x080A0000U, 128U * 1024U },   /* S9  */
    { 0x080C0000U, 128U * 1024U },   /* S10 */
    { 0x080E0000U, 128U * 1024U },   /* S11 */
};

#define OTA_SECTOR_COUNT    (sizeof(g_sectors) / sizeof(g_sectors[0]))

/* 当前受保护(正在运行)的分区起始地址, 0 = 不保护 */
static uint32_t g_protected_partition = 0U;


/* ==================================================================
 *  内部工具
 * ================================================================== */

/* 返回 addr 所在扇区下标, 越界返回 -1 */
static int32_t sector_index_of(uint32_t addr)
{
    uint32_t i;

    for (i = 0U; i < OTA_SECTOR_COUNT; i++) {
        if (addr >= g_sectors[i].addr &&
            addr <  (g_sectors[i].addr + g_sectors[i].size)) {
            return (int32_t)i;
        }
    }
    return -1;
}

/* 判断 [addr, addr+size) 是否完整落在 APP1 或 APP2 分区内 */
static int32_t range_in_app_partition(uint32_t addr, uint32_t size)
{
    uint32_t end;

    if (size == 0U) {
        return 0;                                       /* 空区间视为合法 */
    }
    if (addr + size < addr) {                           /* 溢出 */
        return -1;
    }
    end = addr + size;

    if (addr >= APP1_ADDR && end <= (APP1_ADDR + APP1_SIZE)) {
        return 0;
    }
    if (addr >= APP2_ADDR && end <= (APP2_ADDR + APP2_SIZE)) {
        return 0;
    }
    return -1;
}

/* 判断 [addr, addr+size) 是否与受保护分区重叠 */
static int32_t range_hits_protected(uint32_t addr, uint32_t size)
{
    uint32_t prot_end;

    if (g_protected_partition == 0U || size == 0U) {
        return 0;
    }
    prot_end = g_protected_partition + APP1_SIZE;      /* 两个分区等大取大者即可 */

    if (addr < prot_end && (addr + size) > g_protected_partition) {
        return -1;
    }
    return 0;
}

static void flash_clear_flags(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

uint32_t ota_flash_sector_size(uint32_t addr)
{
    int32_t idx = sector_index_of(addr);
    return (idx < 0) ? 0U : g_sectors[idx].size;
}

uint32_t ota_flash_erase_span(uint32_t addr, uint32_t size)
{
    uint32_t end;
    uint32_t span;

    if (size == 0U) {
        return 0U;
    }
    end  = addr + size;
    span = 0U;

    while (addr < end) {
        int32_t idx = sector_index_of(addr);
        if (idx < 0) {
            break;
        }
        span += g_sectors[idx].size;
        addr  = g_sectors[idx].addr + g_sectors[idx].size;   /* 跳到下一扇区边界 */
    }
    return span;
}

void ota_flash_init(void)
{
    /* 复位后 Flash 默认即为锁定状态, 这里显式锁定一次以保证状态确定。
     * (旧实现在这里 Unlock 且从不 Lock, 使 Flash 长期处于解锁状态, 已修正) */
    flash_clear_flags();
    HAL_FLASH_Lock();
}

void ota_flash_protect(uint32_t partition_addr)
{
    g_protected_partition = partition_addr;
}


/* ==================================================================
 *  擦除
 * ================================================================== */
ota_flash_result_t ota_flash_erase_range(uint32_t addr, uint32_t size,
                                         ota_flash_feed_cb_t feed_cb)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    uint32_t end;
    uint32_t guard = 0U;

    if (size == 0U) {
        return OTA_FLASH_OK;
    }
    if (range_in_app_partition(addr, size) != 0) {
        printf("[FLASH] ERASE addr out of app partition: 0x%08lX +%lu\r\n",
               (unsigned long)addr, (unsigned long)size);
        return OTA_FLASH_ERR_ADDR;
    }
    if (range_hits_protected(addr, size) != 0) {
        printf("[FLASH] ERASE refused: hits running partition 0x%08lX\r\n",
               (unsigned long)g_protected_partition);
        return OTA_FLASH_ERR_PROTECT;
    }

    end = addr + size;

    HAL_FLASH_Unlock();
    flash_clear_flags();

    /* 按真实扇区表逐个擦除。旧实现按固定 64KB 步进判断扇区下标,
     * 在 128KB 扇区上会漏擦 S3、并多擦到 boot_info 所在的 S11, 已修正。 */
    while (addr < end) {
        int32_t idx = sector_index_of(addr);
        if (idx < 0) {
            HAL_FLASH_Lock();
            return OTA_FLASH_ERR_ADDR;
        }

        erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
        erase.Sector       = (uint32_t)idx;
        erase.NbSectors    = 1U;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        printf("[FLASH] erase S%ld @0x%08lX (%luKB)\r\n",
               (long)idx, (unsigned long)g_sectors[idx].addr,
               (unsigned long)(g_sectors[idx].size / 1024U));

        if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[FLASH] ERASE failed, sector=%ld err=0x%08lX\r\n",
                   (long)idx, (unsigned long)sector_error);
            return OTA_FLASH_ERR_ERASE;
        }

        /* 每擦完一个扇区立刻喂一次狗(覆盖扇区之间的间隙) */
        if (feed_cb != NULL) {
            feed_cb();
        }

        addr = g_sectors[idx].addr + g_sectors[idx].size;

        if (++guard > OTA_SECTOR_COUNT) {
            break;                                       /* 防御性: 不应发生 */
        }
    }

    HAL_FLASH_Lock();
    return OTA_FLASH_OK;
}


/* ==================================================================
 *  读 / 写
 * ================================================================== */
ota_flash_result_t ota_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len == 0U) {
        return OTA_FLASH_ERR_ADDR;
    }
    if (addr < FLASH_BASE_ADDR || (addr + len) > FLASH_END_ADDR) {
        return OTA_FLASH_ERR_ADDR;
    }

    memcpy(buf, (const void *)addr, len);
    return OTA_FLASH_OK;
}

ota_flash_result_t ota_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t i;

    if (buf == NULL || len == 0U) {
        return OTA_FLASH_ERR_ADDR;
    }
    /* STM32F4 内部 Flash 以 32bit 字为最小编程单位, 长度必须 4 字节对齐 */
    if ((len & 0x3U) != 0U) {
        return OTA_FLASH_ERR_SIZE;
    }
    if (range_in_app_partition(addr, len) != 0) {
        return OTA_FLASH_ERR_ADDR;
    }
    if (range_hits_protected(addr, len) != 0) {
        printf("[FLASH] WRITE refused: hits running partition\r\n");
        return OTA_FLASH_ERR_PROTECT;
    }

    HAL_FLASH_Unlock();
    flash_clear_flags();

    for (i = 0U; i < len; i += 4U) {
        uint32_t word = *(const uint32_t *)(buf + i);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[FLASH] WRITE failed @0x%08lX\r\n", (unsigned long)(addr + i));
            return OTA_FLASH_ERR_WRITE;
        }
    }

    HAL_FLASH_Lock();
    return OTA_FLASH_OK;
}


/* ==================================================================
 *  CRC32
 * ================================================================== */
uint32_t ota_flash_crc32(uint32_t addr, uint32_t size, ota_flash_feed_cb_t feed_cb)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t done = 0U;

    if (addr < FLASH_BASE_ADDR || (addr + size) > FLASH_END_ADDR) {
        return 0U;
    }

    while (done < size) {
        uint32_t chunk = size - done;
        if (chunk > CRC_FEED_INTERVAL) {
            chunk = CRC_FEED_INTERVAL;
        }

        crc32_update(&crc, (const uint8_t *)(addr + done), chunk);
        done += chunk;

        if (feed_cb != NULL) {
            feed_cb();
        }
    }

    return crc32_finalize(crc);
}


/* ==================================================================
 *  BootInfo
 * ================================================================== */
ota_flash_result_t boot_info_write(const boot_info_t *info)
{
    boot_info_t local;
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    const uint32_t *p;
    uint32_t words;
    uint32_t i;

    if (info == NULL) {
        return OTA_FLASH_ERR_ADDR;
    }

    /* 先拷贝到栈上: 允许调用方传入指向 BOOT_INFO_ADDR 本身的指针
     * (读-改-写场景), 否则擦除后源数据就没了。 */
    memcpy(&local, info, sizeof(local));
    local.magic = BOOT_INFO_MAGIC;

    HAL_FLASH_Unlock();
    flash_clear_flags();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = FLASH_SECTOR_11;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        printf("[FLASH] BootInfo erase failed err=0x%08lX\r\n",
               (unsigned long)sector_error);
        return OTA_FLASH_ERR_ERASE;
    }

    p     = (const uint32_t *)&local;
    words = (uint32_t)(sizeof(boot_info_t) / 4U);

    for (i = 0U; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              BOOT_INFO_ADDR + (i * 4U), p[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            printf("[FLASH] BootInfo write failed @word %lu\r\n",
                   (unsigned long)i);
            return OTA_FLASH_ERR_WRITE;
        }
    }

    HAL_FLASH_Lock();

    /* 回读校验 */
    if (memcmp((const void *)BOOT_INFO_ADDR, &local, sizeof(local)) != 0) {
        printf("[FLASH] BootInfo verify failed\r\n");
        return OTA_FLASH_ERR_WRITE;
    }

    return OTA_FLASH_OK;
}

const boot_info_t *boot_info_read(void)
{
    return (const boot_info_t *)BOOT_INFO_ADDR;
}


/* ==================================================================
 *  镜像校验
 * ================================================================== */
int32_t ota_flash_check_image(uint32_t slot, uint32_t size, uint32_t expect_crc,
                              ota_flash_feed_cb_t feed_cb)
{
    uint32_t app_addr;
    uint32_t app_size;
    uint32_t stack;
    uint32_t reset_vec;
    uint32_t crc;

    if (slot == OTA_SLOT_APP1) {
        app_addr = APP1_ADDR;
        app_size = APP1_SIZE;
    } else if (slot == OTA_SLOT_APP2) {
        app_addr = APP2_ADDR;
        app_size = APP2_SIZE;
    } else {
        return OTA_IMAGE_ERR_SLOT;
    }

    if (size > app_size) {
        return OTA_IMAGE_ERR_SIZE;
    }

    /* 1. 首字 = 初始栈指针 */
    stack = *(const volatile uint32_t *)app_addr;
    if (stack < SRAM_START_ADDR || stack > SRAM_END_ADDR) {
        printf("[FLASH] image check: bad SP 0x%08lX (slot %lu)\r\n",
               (unsigned long)stack, (unsigned long)slot);
        return OTA_IMAGE_ERR_STACK;
    }

    /* 2. 第二字 = 复位向量, 必须落在本分区内。
     *    这一条能挡住"上位机把 A 镜像推到了 B 区"这类错误 ——
     *    A 镜像的复位向量指向 0x08008xxx, 不在 [APP2_ADDR, +APP2_SIZE) 内。 */
    reset_vec = *(const volatile uint32_t *)(app_addr + 4U);
    if (reset_vec < app_addr || reset_vec >= (app_addr + app_size)) {
        printf("[FLASH] image check: reset vec 0x%08lX not in slot %lu "
               "[0x%08lX,0x%08lX)\r\n",
               (unsigned long)reset_vec, (unsigned long)slot,
               (unsigned long)app_addr, (unsigned long)(app_addr + app_size));
        return OTA_IMAGE_ERR_RESET_VEC;
    }

    /* 3. CRC32 (size==0 表示未记录, 跳过 —— 例如产线 JLink 直接烧录的镜像) */
    if (size != 0U) {
        crc = ota_flash_crc32(app_addr, size, feed_cb);
        if (crc != expect_crc) {
            printf("[FLASH] image check: CRC 0x%08lX != expect 0x%08lX\r\n",
                   (unsigned long)crc, (unsigned long)expect_crc);
            return OTA_IMAGE_ERR_CRC;
        }
    }

    return OTA_IMAGE_OK;
}
