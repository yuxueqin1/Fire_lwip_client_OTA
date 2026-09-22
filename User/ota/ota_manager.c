#include "ota_manager.h"
#include "ota_flash.h"
#include "ota_crc32.h"
#include "ota_wdt.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>


/* ==================================================================
 *  编译期槽位选择
 *
 *  Keil 两个 Target 里分别加 Define:
 *      A 镜像 Target:  OTA_SLOT_SEL=1
 *      B 镜像 Target:  OTA_SLOT_SEL=2
 *  未定义时(例如只用一个 Target 调试)退化为运行时从 SCB->VTOR 推断。
 * ================================================================== */
#if defined(OTA_SLOT_SEL) && (OTA_SLOT_SEL == 2)
  #define OTA_SLOT_COMPILED     OTA_SLOT_APP2
#elif defined(OTA_SLOT_SEL) && (OTA_SLOT_SEL == 1)
  #define OTA_SLOT_COMPILED     OTA_SLOT_APP1
#else
  #define OTA_SLOT_COMPILED     0U
#endif


static ota_manager_t g_ota_manager;

/* 本次 OTA 的落地参数 */
static uint32_t g_current_addr   = 0U;   /* 正在运行的分区基址 */
static uint32_t g_download_addr  = 0U;   /* 本次要写的分区基址 */
static uint32_t g_download_size  = 0U;   /* 本次要写的分区大小 */

static uint32_t g_last_log_pct   = 0U;   /* 进度打印去重 */


/* ==================================================================
 *  槽位判定
 * ================================================================== */
static uint8_t slot_of_addr(uint32_t addr)
{
    if (addr >= APP1_ADDR && addr < (APP1_ADDR + APP1_SIZE)) {
        return OTA_SLOT_APP1;
    }
    if (addr >= APP2_ADDR && addr < (APP2_ADDR + APP2_SIZE)) {
        return OTA_SLOT_APP2;
    }
    return 0U;
}

static uint32_t addr_of_slot(uint8_t slot)
{
    return (slot == OTA_SLOT_APP2) ? APP2_ADDR : APP1_ADDR;
}

static uint32_t size_of_slot(uint8_t slot)
{
    return (slot == OTA_SLOT_APP2) ? APP2_SIZE : APP1_SIZE;
}

/* 判定当前运行分区。
 *
 * 优先级:
 *   1) SCB->VTOR  —— 正在生效的向量表基址, 就是"此刻在跑哪个分区"的直接证据
 *   2) OTA_SLOT_SEL 编译期宏 —— VTOR 没配好(如调试器直接烧录)时的兜底
 *   3) BootInfo.boot_app
 *   4) 默认 APP1 (并打印告警)
 */
static uint8_t detect_current_slot(void)
{
    uint32_t vec       = SCB->VTOR;
    uint8_t  by_vtor   = slot_of_addr(vec);
    const boot_info_t *bi;
    uint8_t  slot;
    const char *src;

    if (by_vtor != 0U) {
        slot = by_vtor;
        src  = "VTOR";
    } else if (OTA_SLOT_COMPILED != 0U) {
        slot = OTA_SLOT_COMPILED;
        src  = "OTA_SLOT_SEL(compile-time)";
        printf("[OTA] warn: VTOR=0x%08lX not a valid slot base, use compile-time slot\r\n",
               (unsigned long)vec);
    } else {
        bi = boot_info_read();
        if (bi->magic == BOOT_INFO_MAGIC &&
            (bi->boot_app == OTA_SLOT_APP1 || bi->boot_app == OTA_SLOT_APP2)) {
            slot = (uint8_t)bi->boot_app;
            src  = "BootInfo";
            printf("[OTA] warn: VTOR=0x%08lX invalid, use BootInfo.boot_app=%c\r\n",
                   (unsigned long)vec, OTA_SLOT_CHAR(slot));
        } else {
            slot = OTA_SLOT_APP1;
            src  = "fallback";
            printf("[OTA] warn: cannot determine running slot, assume APP1 "
                   "(VTOR=0x%08lX)\r\n", (unsigned long)vec);
        }
    }

    /* 交叉校验: 编译期指定与 VTOR 不一致时告警(通常意味着 IROM1/VECT_TAB_OFFSET 配错) */
    if (OTA_SLOT_COMPILED != 0U && by_vtor != 0U && OTA_SLOT_COMPILED != by_vtor) {
        printf("[OTA] WARN: OTA_SLOT_SEL=%c and VTOR(0x%08lX) imply slot %c, mismatch!\r\n",
               OTA_SLOT_CHAR(OTA_SLOT_COMPILED), (unsigned long)vec,
               OTA_SLOT_CHAR(by_vtor));
    }

    printf("[OTA] running slot = %c (source: %s), VTOR=0x%08lX\r\n",
           OTA_SLOT_CHAR(slot), src, (unsigned long)vec);
    return slot;
}


/* ==================================================================
 *  初始化
 * ================================================================== */
void ota_manager_init(void)
{
    const boot_info_t *bi;

    ota_flash_init();
    ota_wdt_init();

    memset(&g_ota_manager, 0, sizeof(g_ota_manager));
    g_ota_manager.download_crc = 0xFFFFFFFFU;
    g_ota_manager.state        = OTA_STATE_IDLE;

    g_ota_manager.current_slot = detect_current_slot();
    g_ota_manager.target_slot  = (g_ota_manager.current_slot == OTA_SLOT_APP1)
                                 ? OTA_SLOT_APP2 : OTA_SLOT_APP1;

    g_current_addr  = addr_of_slot(g_ota_manager.current_slot);
    g_download_addr = addr_of_slot(g_ota_manager.target_slot);
    g_download_size = size_of_slot(g_ota_manager.target_slot);

    /* 保护正在运行的分区: 任何越界擦写都会被 ota_flash 拦下 */
    ota_flash_protect(g_current_addr);

    bi = boot_info_read();
    printf("[OTA] current ver " FW_VER_FMT ", target slot %c (0x%08lX, %luKB)\r\n",
           FW_VER_ARG(FW_VERSION), OTA_SLOT_CHAR(g_ota_manager.target_slot),
           (unsigned long)g_download_addr, (unsigned long)(g_download_size / 1024U));
    if (bi->magic == BOOT_INFO_MAGIC) {
        printf("[OTA] boot_info: boot_app=%c ver=" FW_VER_FMT
               " size=%lu crc=0x%08lX\r\n",
               OTA_SLOT_CHAR(bi->boot_app), FW_VER_ARG(bi->version),
               (unsigned long)bi->size, (unsigned long)bi->crc);
    } else {
        printf("[OTA] boot_info invalid (magic=0x%08lX), first boot / not initialized?\r\n",
               (unsigned long)bi->magic);
    }
}


ota_manager_t* ota_get_manager(void)
{
    return &g_ota_manager;
}

uint8_t ota_get_current_slot(void)
{
    return g_ota_manager.current_slot;
}

uint8_t ota_get_target_slot(void)
{
    return g_ota_manager.target_slot;
}

uint32_t ota_get_current_version(void)
{
    /* 版本号在编译期确定(ota_config.h), 就是"当前这份镜像"的版本 */
    return (uint32_t)FW_VERSION;
}

ota_state_t ota_get_state(void)
{
    return g_ota_manager.state;
}

uint32_t ota_get_progress(void)
{
    if (g_ota_manager.total_size == 0U) {
        return 0U;
    }
    return (g_ota_manager.downloaded_size * 100U) / g_ota_manager.total_size;
}

ota_state_t ota_take_error(void)
{
    ota_state_t err = g_ota_manager.last_error;
    g_ota_manager.last_error = OTA_STATE_IDLE;      /* 0 = 无错误 */
    return err;
}

void ota_abort(ota_state_t err)
{
    ota_wdt_hw_stop();

    if (err != OTA_STATE_IDLE) {
        g_ota_manager.last_error = err;
        g_ota_manager.error_count++;
        printf("[OTA] ABORT: err=%u (total %lu), back to IDLE, retryable\r\n",
               (unsigned)err, (unsigned long)g_ota_manager.error_count);
    } else {
        printf("[OTA] aborted, back to IDLE\r\n");
    }

    g_ota_manager.state           = OTA_STATE_IDLE;
    g_ota_manager.downloaded_size = 0U;
    g_ota_manager.download_crc    = 0xFFFFFFFFU;
    g_last_log_pct                = 0U;
}


/* ==================================================================
 *  流程: 推送请求
 * ================================================================== */
int32_t ota_handle_push_update_request(uint32_t new_version,
                                       uint32_t new_size,
                                       uint32_t new_crc)
{
    uint32_t curr_version = ota_get_current_version();

    printf("[OTA] PUSH_UPDATE: ver=" FW_VER_FMT " size=%lu crc=0x%08lX "
           "(current " FW_VER_FMT ", current slot %c)\r\n",
           FW_VER_ARG(new_version), (unsigned long)new_size,
           (unsigned long)new_crc, FW_VER_ARG(curr_version),
           OTA_SLOT_CHAR(g_ota_manager.current_slot));

    /* 正在传输/校验中又收到推送 —— 忽略, 不打断手上的流程 */
    if (g_ota_manager.state != OTA_STATE_IDLE) {
        printf("[OTA] busy(state=%u), push ignored\r\n",
               (unsigned)g_ota_manager.state);
        return 0;
    }

    /* 大小校验 */
    if (new_size == 0U) {
        printf("[OTA] Error: size is 0\r\n");
        g_ota_manager.last_error = OTA_STATE_ERROR_SIZE;
        g_ota_manager.error_count++;
        return -1;
    }
    if (new_size > g_download_size) {
        printf("[OTA] Error: image too large %lu > slot %lu (0x%08lX)\r\n",
               (unsigned long)new_size, (unsigned long)g_download_size,
               (unsigned long)g_download_addr);
        g_ota_manager.last_error = OTA_STATE_ERROR_SIZE;
        g_ota_manager.error_count++;
        return -1;
    }
    if ((new_size & 0x3U) != 0U) {
        /* Flash 按字编程, 上位机必须先补齐到 4 字节倍数 */
        printf("[OTA] Error: size %lu not multiple of 4 (PC must pad)\r\n",
               (unsigned long)new_size);
        g_ota_manager.last_error = OTA_STATE_ERROR_SIZE;
        g_ota_manager.error_count++;
        return -1;
    }

    /* 版本校验: 只有更"新"才升级 */
    if (new_version <= curr_version) {
        printf("[OTA] version not newer, reject upgrade\r\n");
        return 0;
    }

    g_ota_manager.remote_version = new_version;
    g_ota_manager.remote_crc     = new_crc;
    g_ota_manager.total_size     = new_size;
    g_ota_manager.downloaded_size = 0U;
    g_ota_manager.download_crc    = 0xFFFFFFFFU;
    g_last_log_pct                = 0U;
    g_ota_manager.state           = OTA_STATE_REQUEST_RECEIVED;

    printf("[OTA] accept upgrade: write slot %c @0x%08lX\r\n",
           OTA_SLOT_CHAR(g_ota_manager.target_slot),
           (unsigned long)g_download_addr);
    return 1;
}


/* ==================================================================
 *  流程: 下载
 * ================================================================== */
int32_t ota_start_download(void)
{
    uint32_t erase_span;

    if (g_ota_manager.state != OTA_STATE_REQUEST_RECEIVED) {
        printf("[OTA] Error: no push request, state=%u\r\n",
               (unsigned)g_ota_manager.state);
        return -1;
    }
    if (g_ota_manager.total_size == 0U ||
        g_ota_manager.total_size > g_download_size) {
        printf("[OTA] Error: total_size invalid (%lu)\r\n",
               (unsigned long)g_ota_manager.total_size);
        ota_abort(OTA_STATE_ERROR_SIZE);
        return -1;
    }

    g_ota_manager.state           = OTA_STATE_DOWNLOADING;
    g_ota_manager.downloaded_size = 0U;
    g_ota_manager.download_crc    = 0xFFFFFFFFU;
    g_last_log_pct                = 0U;

    /* 只擦除本次固件实际覆盖到的扇区(而不是整个分区):
     * 384KB 分区至少要 3 个 128KB 扇区, 全擦要 ~3.3s;
     * 88KB 的固件只覆盖 1 个扇区, 只擦 ~1.1s。 */
    erase_span = ota_flash_erase_span(g_download_addr, g_ota_manager.total_size);
    if (erase_span == 0U) {
        printf("[OTA] Error: erase range calc failed\r\n");
        ota_abort(OTA_STATE_ERROR_FLASH);
        return -1;
    }

    printf("[OTA] erase 0x%08lX +%luKB (only needed sectors)...\r\n",
           (unsigned long)g_download_addr, (unsigned long)(erase_span / 1024U));

    /* 擦除期间 CPU 会被 Flash 总线 stall, 软件喂狗靠不住 ——
     * 用 TIM7+DMA 硬件喂狗, 只包住擦除这段时间。 */
    ota_wdt_hw_start();

    {
        ota_flash_result_t fr =
            ota_flash_erase_range(g_download_addr, erase_span, ota_wdt_feed);

        ota_wdt_hw_stop();

        if (fr != OTA_FLASH_OK) {
            printf("[OTA] Error: erase failed\r\n");
            ota_abort(OTA_STATE_ERROR_FLASH);
            return -1;
        }
    }

    printf("[OTA] erase done, waiting %lu bytes\r\n",
           (unsigned long)g_ota_manager.total_size);
    return 0;
}

int32_t ota_write_data_block(const uint8_t *data, uint32_t len, uint32_t offset)
{
    uint32_t write_addr;

    if (g_ota_manager.state != OTA_STATE_DOWNLOADING) {
        printf("[OTA] Error: not downloading (state=%u), drop block\r\n",
               (unsigned)g_ota_manager.state);
        return -1;
    }
    if (data == NULL || len == 0U) {
        return -1;
    }
    if ((len & 0x3U) != 0U || (offset & 0x3U) != 0U) {
        /* 帧长通常 1024(4 的倍数), 只有最后一帧可能不齐 —— 上位机已补齐 */
        printf("[OTA] Error: len=%lu offset=%lu not 4-byte aligned\r\n",
               (unsigned long)len, (unsigned long)offset);
        ota_abort(OTA_STATE_ERROR_SIZE);
        return -1;
    }
    if (offset + len > g_download_size) {
        printf("[OTA] Error: write out of range offset=%lu len=%lu (slot %lu)\r\n",
               (unsigned long)offset, (unsigned long)len,
               (unsigned long)g_download_size);
        ota_abort(OTA_STATE_ERROR_SIZE);
        return -1;
    }
    if (offset + len > g_ota_manager.total_size) {
        printf("[OTA] Error: beyond declared size offset=%lu len=%lu total=%lu\r\n",
               (unsigned long)offset, (unsigned long)len,
               (unsigned long)g_ota_manager.total_size);
        ota_abort(OTA_STATE_ERROR_SIZE);
        return -1;
    }

    write_addr = g_download_addr + offset;

    if (ota_flash_write(write_addr, data, len) != OTA_FLASH_OK) {
        printf("[OTA] Error: flash write failed @0x%08lX\r\n",
               (unsigned long)write_addr);
        ota_abort(OTA_STATE_ERROR_FLASH);
        return -1;
    }

    crc32_update(&g_ota_manager.download_crc, data, len);

    /* 用 offset+len 而不是累加: 即使上位机重发同一块, 进度也不会超 100% */
    if (offset + len > g_ota_manager.downloaded_size) {
        g_ota_manager.downloaded_size = offset + len;
    }

    {
        uint32_t pct = ota_get_progress();
        if (pct >= g_last_log_pct + 10U || pct == 100U) {
            g_last_log_pct = pct;
            printf("[OTA] progress %lu%% (%lu/%lu)\r\n",
                   (unsigned long)pct,
                   (unsigned long)g_ota_manager.downloaded_size,
                   (unsigned long)g_ota_manager.total_size);
        }
    }

    return 0;
}

int32_t ota_finish_download(void)
{
    int32_t rc;

    if (g_ota_manager.state != OTA_STATE_DOWNLOADING) {
        printf("[OTA] Error: not downloading, cannot finish\r\n");
        return -1;
    }

    g_ota_manager.state = OTA_STATE_VERIFYING;

    printf("[OTA] transfer end: received %lu / declared %lu bytes\r\n",
           (unsigned long)g_ota_manager.downloaded_size,
           (unsigned long)g_ota_manager.total_size);

    if (g_ota_manager.downloaded_size != g_ota_manager.total_size) {
        printf("[OTA] Error: length mismatch\r\n");
        ota_abort(OTA_STATE_ERROR_SIZE);
        return -1;
    }

    g_ota_manager.download_crc = crc32_finalize(g_ota_manager.download_crc);
    printf("[OTA] CRC32: got 0x%08lX, expected 0x%08lX\r\n",
           (unsigned long)g_ota_manager.download_crc,
           (unsigned long)g_ota_manager.remote_crc);

    if (g_ota_manager.download_crc != g_ota_manager.remote_crc) {
        printf("[OTA] Error: CRC32 mismatch\r\n");
        ota_abort(OTA_STATE_ERROR_CRC);
        return -1;
    }

    /* 再对"已经落进 Flash 的内容"做一次结构与 CRC 复核:
     * 1) 首字(栈指针)必须在 SRAM 内
     * 2) 复位向量必须落在本分区内 ← 挡住上位机发错镜像(A镜像发到B区)
     * 3) 按 size 重新算 CRC
     * CRC 扫描要读几十~几百 KB Flash, 同样会占住 Flash 总线, 一并包住。 */
    ota_wdt_hw_start();

    rc = ota_flash_check_image(g_ota_manager.target_slot,
                              g_ota_manager.downloaded_size,
                              g_ota_manager.remote_crc,
                              ota_wdt_feed);

    ota_wdt_hw_stop();

    if (rc != OTA_IMAGE_OK) {
        printf("[OTA] Error: image validate failed (rc=%ld)\r\n", (long)rc);
        ota_abort((rc == OTA_IMAGE_ERR_CRC) ? OTA_STATE_ERROR_CRC
                                            : OTA_STATE_ERROR_IMAGE);
        return -1;
    }

    printf("[OTA] verify OK\r\n");
    g_ota_manager.state = OTA_STATE_SUCCESS;
    return 0;
}


/* ==================================================================
 *  流程: 生效
 * ================================================================== */
int32_t ota_prepare_update(void)
{
    boot_info_t info;

    if (g_ota_manager.state != OTA_STATE_SUCCESS) {
        printf("[OTA] Error: not verified yet, cannot update (state=%u)\r\n",
               (unsigned)g_ota_manager.state);
        return -1;
    }

    g_ota_manager.state = OTA_STATE_UPDATING;

    memset(&info, 0, sizeof(info));
    info.magic   = BOOT_INFO_MAGIC;
    info.boot_app = g_ota_manager.target_slot;
    info.crc     = g_ota_manager.download_crc;
    info.size    = g_ota_manager.downloaded_size;
    info.version = g_ota_manager.remote_version;
    /* info.reserved 已 memset 为 0 */

    if (boot_info_write(&info) != OTA_FLASH_OK) {
        printf("[OTA] Error: BootInfo write failed\r\n");
        ota_abort(OTA_STATE_ERROR_FLASH);
        return -1;
    }

    printf("[OTA] BootInfo updated: next boot -> %c slot, ver=" FW_VER_FMT ", "
           "%lu bytes\r\n",
           OTA_SLOT_CHAR(g_ota_manager.target_slot),
           FW_VER_ARG(info.version), (unsigned long)info.size);

    return 0;
}
